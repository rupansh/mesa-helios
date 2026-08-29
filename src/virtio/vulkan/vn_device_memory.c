/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#include "vn_device_memory.h"

void vn_renderer_helios_diag_log(const char *fmt, ...);

#include "venus-protocol/vn_protocol_driver_device_memory.h"
#include "venus-protocol/vn_protocol_driver_transport.h"
#include "vk_debug_utils.h"

#include "vn_android.h"
#include "vn_buffer.h"
#include "vn_cs.h"
#include "vn_device.h"
#ifdef _WIN32
#include "vn_helios_direct_dispatch.h"
#include "vn_helios_record_submit.h"
#endif
#include "vn_image.h"
#include "vn_physical_device.h"
#include "vn_renderer.h"
#include "vn_renderer_util.h"

#include <stdio.h>

static bool
vn_helios_env_enabled(const char *name)
{
   const char *value = os_get_option(name);
   return value && strcmp(value, "0") != 0 && strcasecmp(value, "false") != 0;
}

/* True when the memory type the app allocated from advertises
 * HOST_VISIBLE|HOST_COHERENT|HOST_CACHED. Such a type PROMISES the app cached
 * CPU access, and because it is also COHERENT no explicit cache maintenance is
 * owed for it — helios_bo_needs_cache_ops() exempts exactly this combination.
 */
static bool
vn_device_memory_type_is_coherent_cached(const struct vn_device *dev,
                                         const struct vn_device_memory *mem)
{
   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   const VkMemoryPropertyFlags want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                      VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                                      VK_MEMORY_PROPERTY_HOST_CACHED_BIT;

   return (mem_type->propertyFlags & want) == want;
}

static bool
vn_device_memory_is_coherent_cached(struct vn_device *dev,
                                    struct vn_device_memory *mem)
{
   if (!mem->base_bo || !mem->base_bo->prefer_cached_map)
      return false;

   return vn_device_memory_type_is_coherent_cached(dev, mem);
}

static void
vn_device_memory_register_coherent_cached_mapping(struct vn_device *dev,
                                                  struct vn_device_memory *mem)
{
   if (!vn_device_memory_is_coherent_cached(dev, mem) || !mem->base_bo ||
       !mem->base_bo->mmap_ptr || mem->map_end <= mem->map_start)
      return;

   simple_mtx_lock(&dev->mutex);
   if (!mem->coherent_cached_mapped) {
      list_addtail(&mem->coherent_cached_link,
                   &dev->coherent_cached_memory);
      mem->coherent_cached_mapped = true;
   }
   simple_mtx_unlock(&dev->mutex);
}

static void
vn_device_memory_unregister_coherent_cached_mapping(struct vn_device *dev,
                                                    struct vn_device_memory *mem)
{
   if (!mem->coherent_cached_mapped)
      return;

   simple_mtx_lock(&dev->mutex);
   if (mem->coherent_cached_mapped) {
      list_delinit(&mem->coherent_cached_link);
      mem->coherent_cached_mapped = false;
   }
   simple_mtx_unlock(&dev->mutex);
}

static void
vn_device_memory_cache_op_coherent_cached_mappings(struct vn_device *dev,
                                                   bool invalidate)
{
   simple_mtx_lock(&dev->mutex);
   list_for_each_entry(struct vn_device_memory, mem,
                       &dev->coherent_cached_memory,
                       coherent_cached_link) {
      if (!mem->base_bo || !mem->base_bo->mmap_ptr ||
          mem->map_end <= mem->map_start)
         continue;

      const VkDeviceSize size = mem->map_end - mem->map_start;
      if (invalidate) {
         vn_renderer_bo_invalidate(dev->renderer, mem->base_bo,
                                   mem->map_start, size);
      } else if (mem->wsi_buffer_blit_dst) {
         /* The CPU only reads software WSI blit buffers after the GPU writes
          * them.  Flushing them before submit is unnecessary and can dominate
          * present time when the buffer is host-cached.
          */
         continue;
      } else {
         vn_renderer_bo_flush(dev->renderer, mem->base_bo, mem->map_start,
                              size);
      }
   }
   simple_mtx_unlock(&dev->mutex);
}

void
vn_device_memory_flush_coherent_cached_mappings(struct vn_device *dev)
{
   vn_device_memory_cache_op_coherent_cached_mappings(dev, false);
}

void
vn_device_memory_invalidate_coherent_cached_mappings(struct vn_device *dev)
{
   vn_device_memory_cache_op_coherent_cached_mappings(dev, true);
}

void
vn_device_memory_cleanup_coherent_cached_mappings(struct vn_device *dev)
{
   simple_mtx_lock(&dev->mutex);
   list_for_each_entry_safe(struct vn_device_memory, mem,
                            &dev->coherent_cached_memory,
                            coherent_cached_link) {
      list_delinit(&mem->coherent_cached_link);
      mem->coherent_cached_mapped = false;
   }
   simple_mtx_unlock(&dev->mutex);
}

/* device memory commands */

static inline VkResult
vn_device_memory_alloc_simple(struct vn_device *dev,
                              struct vn_device_memory *mem,
                              const VkMemoryAllocateInfo *alloc_info)
{
   /* ALWAYS synchronous on this transport (matches upstream's
    * VN_PERF=no_async_mem_alloc semantics; mirrors import_resource_id and
    * alloc_export, which were already made sync). Upstream's async path
    * assumes host vkAllocateMemory cannot fail; on this stack it can
    * (external-handle rules, udmabuf limits, host driver changes). An
    * unconfirmed alloc lets the guest emit vkBindImageMemory2/vkFreeMemory
    * against a host object that does not exist, and vkr treats
    * phantom-object commands as fatal decoder state — the whole venus
    * context of the process dies ("failed to look up object N of type 8").
    * The async branch is deleted, not gated, so it cannot regress.
    */
   VkDevice dev_handle = vn_device_to_handle(dev);
   VkDeviceMemory mem_handle = vn_device_memory_to_handle(mem);
   return vn_call_vkAllocateMemory(dev->primary_ring, dev_handle, alloc_info,
                                   NULL, &mem_handle);
}

static inline void
vn_device_memory_free_simple(struct vn_device *dev,
                             struct vn_device_memory *mem)
{
   VkDevice dev_handle = vn_device_to_handle(dev);
   VkDeviceMemory mem_handle = vn_device_memory_to_handle(mem);
   vn_async_vkFreeMemory(dev->primary_ring, dev_handle, mem_handle, NULL);
}

static bool
vn_device_memory_needs_wait_alloc(struct vn_device *dev,
                                  struct vn_device_memory *mem)
{
#if DETECT_OS_WINDOWS
   /* A8 HVC1 allocation calls are already host-terminal and publish no ring
    * sequence.  Clear stale generic bookkeeping without constructing a
    * vkWaitRingSeqnoMESA command. */
   (void)dev;
   mem->bo_ring_seqno_valid = false;
   return false;
#else
   if (!mem->bo_ring_seqno_valid)
      return false;

   /* fine to false it here since renderer submission failure is fatal */
   mem->bo_ring_seqno_valid = false;

   /* no need to wait for ring if
    * - mem alloc is done upon bo map or export
    * - mem import is done upon bo destroy
    */
   return !vn_ring_get_seqno_status(dev->primary_ring, mem->bo_ring_seqno);
#endif
}

static inline VkResult
vn_device_memory_bo_init(struct vn_device *dev, struct vn_device_memory *mem)
{
   struct vn_renderer_submit_batch *batch = NULL;
#if !DETECT_OS_WINDOWS
   struct vn_renderer_submit_batch local_batch;
   uint32_t local_data[8];
   if (vn_device_memory_needs_wait_alloc(dev, mem)) {
      struct vn_cs_encoder local_enc =
         VN_CS_ENCODER_INITIALIZER_LOCAL(local_data, sizeof(local_data));
      vn_encode_vkWaitRingSeqnoMESA(&local_enc, 0,
                                    vn_ring_get_id(dev->primary_ring),
                                    mem->bo_ring_seqno);
      local_batch = (struct vn_renderer_submit_batch){
         .cs_data = local_data,
         .cs_size = vn_cs_encoder_get_len(&local_enc),
      };
      batch = &local_batch;
   }
#endif

   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   VkExternalMemoryHandleTypeFlags renderer_export_handle_types =
      mem_vk->export_handle_types;
#ifdef _WIN32
   renderer_export_handle_types = mem->renderer_export_handle_types;
#endif
   return vn_renderer_bo_create_from_device_memory(
      dev->renderer, batch, mem_vk->size, mem->base.id,
      mem_type->propertyFlags, renderer_export_handle_types, &mem->base_bo);
}

static inline void
vn_device_memory_bo_fini(struct vn_device *dev, struct vn_device_memory *mem)
{
   if (!mem->base_bo)
      return;

#if !DETECT_OS_WINDOWS
   if (vn_device_memory_needs_wait_alloc(dev, mem)) {
      uint32_t local_data[8];
      struct vn_cs_encoder local_enc =
         VN_CS_ENCODER_INITIALIZER_LOCAL(local_data, sizeof(local_data));
      vn_encode_vkWaitRingSeqnoMESA(&local_enc, 0,
                                    vn_ring_get_id(dev->primary_ring),
                                    mem->bo_ring_seqno);
      vn_renderer_submit_simple(dev->renderer, local_data,
                                vn_cs_encoder_get_len(&local_enc));
   }
#else
   mem->bo_ring_seqno_valid = false;
#endif

   vn_renderer_bo_release_resource(dev->renderer, mem->base_bo);
   vn_renderer_bo_unref(dev->renderer, mem->base_bo);
   mem->base_bo = NULL;
}

#if !defined(_WIN32)
static VkResult
vn_device_memory_import_resource_id(struct vn_device *dev,
                                    struct vn_device_memory *mem,
                                    const VkMemoryAllocateInfo *alloc_info,
                                    uint32_t resource_id)
{
   if (!resource_id)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];

   VkResult result = vn_renderer_bo_create_from_resource_id(
      dev->renderer, mem_vk->size, resource_id,
      mem_type->propertyFlags, &mem->base_bo);
   if (result != VK_SUCCESS)
      return result;

   const VkImportMemoryResourceInfoMESA import_info = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA,
      .pNext = alloc_info->pNext,
      .resourceId = resource_id,
   };
   const VkMemoryAllocateInfo memory_allocate_info = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = &import_info,
      .allocationSize = alloc_info->allocationSize,
      .memoryTypeIndex = alloc_info->memoryTypeIndex,
   };

   vn_ring_roundtrip(dev->primary_ring);

   /* Synchronous on purpose: a host-side import failure must surface here as
    * a clean VkResult. The async path returns VK_SUCCESS optimistically; the
    * caller then binds an image to a memory object the host never created,
    * which poisons the ring ("failed to look up object of type 8" ->
    * vkBindImageMemory2 CS error -> fatal decoder state) and kills the whole
    * venus context (observed live with DWM opening Helios KMD allocations).
    */
   VkDevice dev_handle = vn_device_to_handle(dev);
   VkDeviceMemory mem_handle = vn_device_memory_to_handle(mem);
   result = vn_call_vkAllocateMemory(dev->primary_ring, dev_handle,
                                     &memory_allocate_info, NULL, &mem_handle);
   if (result != VK_SUCCESS) {
      vn_renderer_bo_unref(dev->renderer, mem->base_bo);
      mem->base_bo = NULL;
      return result;
   }

   return VK_SUCCESS;
}

VkResult
vn_device_memory_import_dma_buf(struct vn_device *dev,
                                struct vn_device_memory *mem,
                                const VkMemoryAllocateInfo *alloc_info,
                                int fd)
{
   const VkMemoryType *mem_type =
      &dev->physical_device->memory_properties
          .memoryTypes[alloc_info->memoryTypeIndex];

   struct vn_renderer_bo *bo;
   VkResult result = vn_renderer_bo_create_from_dma_buf(
      dev->renderer, alloc_info->allocationSize, fd, mem_type->propertyFlags,
      &bo);
   if (result != VK_SUCCESS)
      return result;

   vn_ring_roundtrip(dev->primary_ring);

   const VkImportMemoryResourceInfoMESA import_memory_resource_info = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA,
      .pNext = alloc_info->pNext,
      .resourceId = bo->res_id,
   };
   const VkMemoryAllocateInfo memory_allocate_info = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = &import_memory_resource_info,
      .allocationSize = alloc_info->allocationSize,
      .memoryTypeIndex = alloc_info->memoryTypeIndex,
   };
   result = vn_device_memory_alloc_simple(dev, mem, &memory_allocate_info);
   if (result != VK_SUCCESS) {
      vn_renderer_bo_unref(dev->renderer, bo);
      return result;
   }

   /* need to close import fd on success to avoid fd leak */
   close(fd);
   mem->base_bo = bo;

   return VK_SUCCESS;
}

static VkResult
vn_device_memory_alloc_guest_vram(struct vn_device *dev,
                                  struct vn_device_memory *mem,
                                  const VkMemoryAllocateInfo *alloc_info)
{
   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   VkMemoryPropertyFlags flags = mem_type->propertyFlags;
   VkExternalMemoryHandleTypeFlags renderer_export_handle_types =
      mem_vk->export_handle_types;
#ifdef _WIN32
   renderer_export_handle_types = mem->renderer_export_handle_types;
#endif

   /* For external allocation handles, it's possible scenario when requested
    * non-mappable memory. To make sure that virtio-gpu driver will send to
    * the host the address of allocated blob using RESOURCE_MAP_BLOB command
    * a flag VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT must be set.
    */
   if (renderer_export_handle_types)
      flags |= VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

   VkResult result = vn_renderer_bo_create_from_device_memory(
      dev->renderer, NULL, mem_vk->size, mem->base.id, flags,
      renderer_export_handle_types, &mem->base_bo);
   if (result != VK_SUCCESS) {
      return result;
   }

   const VkImportMemoryResourceInfoMESA import_memory_resource_info = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA,
      .pNext = alloc_info->pNext,
      .resourceId = mem->base_bo->res_id,
   };

   const VkMemoryAllocateInfo memory_allocate_info = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = &import_memory_resource_info,
      .allocationSize = alloc_info->allocationSize,
      .memoryTypeIndex = alloc_info->memoryTypeIndex,
   };

   vn_ring_roundtrip(dev->primary_ring);

   result = vn_device_memory_alloc_simple(dev, mem, &memory_allocate_info);
   if (result != VK_SUCCESS) {
      vn_renderer_bo_unref(dev->renderer, mem->base_bo);
      return result;
   }

   return VK_SUCCESS;
}

static VkResult
vn_device_memory_alloc_export(struct vn_device *dev,
                              struct vn_device_memory *mem,
                              const VkMemoryAllocateInfo *alloc_info)
{
   /* Synchronous on purpose (mirrors vn_device_memory_import_resource_id): a
    * host-side failure of an EXPORT allocation must surface here as a clean
    * VkResult. The async path returns VK_SUCCESS optimistically; when the
    * host allocation actually failed (observed with DWM shared-surface
    * export allocs on the NVIDIA render server), the subsequent blob create
    * silently EPERMs (vkr_context_create_resource_from_device_memory cannot
    * find the object) and the cleanup below then sends vkFreeMemory for an
    * object the host never created — "failed to look up object N of type 8"
    * → CS error → fatal decoder state → the whole venus context dies and
    * takes DWM down with it (0xc0000409 crash-loop, ~4 min cadence).
    */
   VkDevice dev_handle = vn_device_to_handle(dev);
   VkDeviceMemory mem_handle = vn_device_memory_to_handle(mem);
   VkResult result = vn_call_vkAllocateMemory(dev->primary_ring, dev_handle,
                                              alloc_info, NULL, &mem_handle);
   if (result != VK_SUCCESS)
      return result;

   result = vn_device_memory_bo_init(dev, mem);
   if (result != VK_SUCCESS) {
      vn_device_memory_free_simple(dev, mem);
      return result;
   }

   result =
      vn_ring_submit_roundtrip(dev->primary_ring, &mem->bo_roundtrip_seqno);
   if (result != VK_SUCCESS) {
      vn_renderer_bo_unref(dev->renderer, mem->base_bo);
      vn_device_memory_free_simple(dev, mem);
      return result;
   }

   mem->bo_roundtrip_seqno_valid = true;

   return VK_SUCCESS;
}
#endif

struct vn_device_memory_alloc_info {
   VkMemoryAllocateInfo alloc;
   VkExportMemoryAllocateInfo export;
   VkMemoryAllocateFlagsInfo flags;
   VkMemoryDedicatedAllocateInfo dedicated;
   VkMemoryOpaqueCaptureAddressAllocateInfo capture;
};

static const VkMemoryAllocateInfo *
vn_device_memory_fix_alloc_info(
   const VkMemoryAllocateInfo *alloc_info,
   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type,
   bool has_guest_vram,
   bool force_capture_replay,
   struct vn_device_memory_alloc_info *local_info)
{
   bool has_flags = false;
   local_info->alloc = *alloc_info;
   VkBaseOutStructure *cur = (void *)&local_info->alloc;

   vk_foreach_struct_const(src, alloc_info->pNext) {
      void *next = NULL;
      switch (src->sType) {
      case VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO:
         /* guest vram turns export alloc into import, so drop export info */
         if (has_guest_vram)
            break;
         memcpy(&local_info->export, src, sizeof(local_info->export));
         local_info->export.handleTypes = renderer_handle_type;
         next = &local_info->export;
         break;
      case VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO:
         memcpy(&local_info->flags, src, sizeof(local_info->flags));
         if (force_capture_replay)
            local_info->flags.flags |=
               VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT;
         has_flags = true;
         next = &local_info->flags;
         break;
      case VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO:
         memcpy(&local_info->dedicated, src, sizeof(local_info->dedicated));
         next = &local_info->dedicated;
         break;
      case VK_STRUCTURE_TYPE_MEMORY_OPAQUE_CAPTURE_ADDRESS_ALLOCATE_INFO:
         memcpy(&local_info->capture, src, sizeof(local_info->capture));
         next = &local_info->capture;
         break;
      default:
         break;
      }

      if (next) {
         cur->pNext = next;
         cur = next;
      }
   }

   if (force_capture_replay && !has_flags) {
      local_info->flags = (VkMemoryAllocateFlagsInfo){
         .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO,
         .flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT |
                  VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT,
      };
      cur->pNext = (void *)&local_info->flags;
      cur = (void *)&local_info->flags;
   }

   cur->pNext = NULL;

   return &local_info->alloc;
}

#ifdef _WIN32
static VkResult
vn_device_memory_defer_outer_allocate(
   struct vn_device *dev,
   struct vn_device_memory *mem,
   const VkMemoryAllocateInfo *alloc_info)
{
   if (!mem->helios_outer_registered || mem->base.vk.export_handle_types)
      return VK_ERROR_VALIDATION_FAILED_EXT;

   const VkMemoryAllocateFlagsInfo *flags_info =
      vk_find_struct_const(alloc_info->pNext, MEMORY_ALLOCATE_FLAGS_INFO);
   const bool force_capture_replay =
      dev->base.vk.enabled_extensions.EXT_buffer_device_address &&
      dev->base.vk.enabled_features.bufferDeviceAddress &&
      dev->base.vk.enabled_features.bufferDeviceAddressCaptureReplay &&
      flags_info &&
      (flags_info->flags & VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT);
   struct vn_device_memory_alloc_info clean_info;
   const VkMemoryAllocateInfo *clean = vn_device_memory_fix_alloc_info(
      alloc_info, 0, false, force_capture_replay, &clean_info);
   clean_info.alloc.memoryTypeIndex =
      vn_physical_device_renderer_memory_type_index(
         dev->physical_device, mem->base.vk.memory_type_index);

   const VkImportMemoryResourceInfoMESA import = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA,
      .pNext = clean->pNext,
      .resourceId = 0,
   };
   const VkMemoryAllocateInfo local = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = &import,
      .allocationSize = mem->helios_outer.outer_allocation_bytes,
      .memoryTypeIndex = clean_info.alloc.memoryTypeIndex,
   };
   VkDeviceMemory memory = vn_device_memory_to_handle(mem);
   const size_t generated_bytes = vn_sizeof_vkAllocateMemory(
      vn_device_to_handle(dev), &local, NULL, &memory);
   if (!generated_bytes ||
       generated_bytes > HELIOS_HOB1_MAX_BYTES ||
       sizeof(clean_info) > HELIOS_HOB1_MAX_BYTES - generated_bytes) {
      vn_renderer_helios_diag_log("HAM1 defer arm=size generated=%zu",
                                  generated_bytes);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   /* The normalized chain can contain at most the members held by clean_info.
    * Use that whole fixed storage as bounded encoder headroom, then seal only
    * the measured byte count below.  This keeps an encoder/sizeof disagreement
    * from becoming an unchecked write without allocating HOB1's 15 MiB limit
    * for every ordinary resource. */
   const size_t payload_capacity = generated_bytes + sizeof(clean_info);
   uint8_t *payload = malloc(payload_capacity);
   if (!payload)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   /* Encode the exact generated command components directly.  The generated
    * vkAllocateMemory wrapper emitted 136 bytes for its own 120-byte size on
    * the Windows record-only dedicated-buffer chain, corrupting the process
    * heap before the first D3D resource could be admitted.  These are the same
    * schema components in the same order; the measured terminal length, not
    * the faulty generated size, defines the sealed deferred record. */
   struct vn_cs_encoder encoder =
      VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_capacity);
   const VkCommandTypeEXT command_type = VK_COMMAND_TYPE_vkAllocateMemory_EXT;
   const VkFlags command_flags = 0;
   const VkDevice device = vn_device_to_handle(dev);
   vn_encode_VkCommandTypeEXT(&encoder, &command_type);
   vn_encode_VkFlags(&encoder, &command_flags);
   vn_encode_VkDevice(&encoder, &device);
   vn_encode_simple_pointer(&encoder, &local);
   vn_encode_VkStructureType(&encoder, &local.sType);
   vn_encode_VkMemoryAllocateInfo_pnext(&encoder, local.pNext);
   vn_encode_VkMemoryAllocateInfo_self(&encoder, &local);
   vn_encode_simple_pointer(&encoder, NULL);
   vn_encode_simple_pointer(&encoder, &memory);
   vn_encode_VkDeviceMemory(&encoder, &memory);
   const size_t payload_bytes = vn_cs_encoder_get_len(&encoder);
   if (vn_cs_encoder_get_fatal(&encoder) ||
       !payload_bytes || payload_bytes > payload_capacity ||
       payload_bytes > HELIOS_HOB1_MAX_BYTES) {
      vn_renderer_helios_diag_log(
         "HAM1 defer arm=encode fatal=%d bytes=%zu generated=%zu cap=%zu",
         vn_cs_encoder_get_fatal(&encoder) ? 1 : 0, payload_bytes,
         generated_bytes, payload_capacity);
      free(payload);
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   const size_t chain_bytes =
      vn_sizeof_VkMemoryAllocateInfo_pnext(clean->pNext);
   if (chain_bytes > UINT32_MAX - 40u) {
      free(payload);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   const uint32_t operand_offset = 40u + (uint32_t)chain_bytes;
   uint32_t placeholder = UINT32_MAX;
   if (payload_bytes >= sizeof(placeholder) &&
       operand_offset <= payload_bytes - sizeof(placeholder))
      memcpy(&placeholder, payload + operand_offset, sizeof(placeholder));
   if (payload_bytes < sizeof(uint32_t) ||
       operand_offset > payload_bytes - sizeof(uint32_t) ||
       placeholder != 0) {
      vn_renderer_helios_diag_log(
         "HAM1 defer arm=operand off=%u bytes=%zu placeholder=0x%08x",
         operand_offset, payload_bytes, placeholder);
      free(payload);
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   struct vn_helios_deferred_record *record =
      vn_device_memory_helios_record_create(
         mem, payload, payload_bytes, operand_offset, true, false);
   if (!record)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   VkResult result =
      vn_device_memory_helios_record_install(dev, mem, record);
   if (result != VK_SUCCESS)
      vn_device_memory_helios_record_destroy(record);
   return result;
}
#endif

/* Storage for host-visible memory in record-only mode.
 *
 * The deferred outer route cannot supply it. The UMD's association carries a
 * process-heap `CpuBacking`, so what the application maps and what the host
 * renders into are two unrelated buffers; and the shared-backing-store contract
 * that would join them requires an allocation "created as shared", which
 * dxgkrnl enforces and no UMD can request through `D3DDDICB_ALLOCATE`
 * (`tools/k2a_unshared_backing_probe.c`, arms B vs D).
 *
 * So own the storage here: a role-1 HVM1 allocation whose guest pages the KMD
 * exports as a `VIRTIO_GPU_BLOB_MEM_GUEST` resource and the host imports, with
 * `vkAllocateMemory` executed through the session carrying that resource in its
 * import operand. Both mechanisms already run in this mode -- they are what the
 * D3D12-resource import path uses -- and both halves of the pair are then one
 * memory by construction.
 */
static VkResult
vn_device_memory_alloc_helios_shared(struct vn_device *dev,
                                     struct vn_device_memory *mem,
                                     const VkMemoryAllocateInfo *alloc_info)
{
   /* Same capture-replay decision the deferred route makes; a buffer device
    * address must not depend on which storage route its memory took. */
   const VkMemoryAllocateFlagsInfo *flags_info =
      vk_find_struct_const(alloc_info->pNext, MEMORY_ALLOCATE_FLAGS_INFO);
   const bool force_capture_replay =
      dev->base.vk.enabled_extensions.EXT_buffer_device_address &&
      dev->base.vk.enabled_features.bufferDeviceAddress &&
      dev->base.vk.enabled_features.bufferDeviceAddressCaptureReplay &&
      flags_info &&
      (flags_info->flags & VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT);
   struct vn_device_memory_alloc_info local_info;
   const VkMemoryAllocateInfo *clean = vn_device_memory_fix_alloc_info(
      alloc_info, 0, false, force_capture_replay, &local_info);
   /* NOT the ordinary translation: this memory is imported by the host from
    * guest pages, and only some renderer types accept that import. The host
    * never maps it -- the guest holds the CPU view of the very same pages.
    * HELIOS_IMPORTED_MEMORY_TYPE=translated is the A/B arm; it is the type the
    * host refused with OUT_OF_DEVICE_MEMORY, kept reachable so the two can be
    * compared without a rebuild. */
   const char *type_arm = os_get_option("HELIOS_IMPORTED_MEMORY_TYPE");
   local_info.alloc.memoryTypeIndex =
      (type_arm && !strcmp(type_arm, "translated"))
         ? vn_physical_device_renderer_memory_type_index(
              dev->physical_device, mem->base.vk.memory_type_index)
         : dev->physical_device->helios_renderer_imported_memory_type_index;
   vn_renderer_helios_diag_log("HHV1 allocate size=%llu renderer_type=%u",
                               (unsigned long long)mem->base.vk.size,
                               local_info.alloc.memoryTypeIndex);

   VkResult result = vn_device_memory_bo_init(dev, mem);
   if (result != VK_SUCCESS) {
      vn_renderer_helios_diag_log("HHV1 bo_init failed result=%d size=%llu",
                                  (int)result,
                                  (unsigned long long)mem->base.vk.size);
      return result;
   }

   /* HELIOS_HOST_VISIBLE_SHARED=bo-only stops here, so the two halves of this
    * route can be told apart: creating the HVM1 allocation from inside a
    * record-only device, versus executing vkAllocateMemory on the session. A
    * later control-lane render is refused whenever the route is attempted at
    * all, and only one of these two can be responsible. */
   const char *arm = os_get_option("HELIOS_HOST_VISIBLE_SHARED");
   if (arm && !strcmp(arm, "bo-only")) {
      vn_device_memory_bo_fini(dev, mem);
      return VK_ERROR_FEATURE_NOT_PRESENT;
   }

   VkDeviceMemory memory = vn_device_memory_to_handle(mem);
   result = vn_renderer_helios_allocate_memory(
      dev->renderer, vn_device_to_handle(dev), clean, &memory, mem->base_bo);
   if (result != VK_SUCCESS) {
      vn_renderer_helios_diag_log("HHV1 session allocate failed result=%d",
                                  (int)result);
      vn_device_memory_bo_fini(dev, mem);
      return result;
   }
   /* The host object exists now rather than when a batch consumes a deferred
    * record, so teardown must free it. */
   mem->helios_host_materialized = true;
   return VK_SUCCESS;
}

/* Host-visible memory this ICD could not back with guest pages, and therefore
 * had to leave on the deferred route where the CPU view is not the host's
 * memory. The KMD caps a CPU-visible HVM1 allocation at the host's stock
 * udmabuf `list_limit`; the bound is read from that refusal rather than
 * mirrored here, so it cannot drift. Any nonzero value is unfixed black-frame
 * surface, not a benign fallback. */
static uint32_t vn_helios_unshared_host_visible_count;

static bool
vn_device_memory_is_host_visible(const struct vn_device *dev,
                                 const struct vn_device_memory *mem)
{
   return (dev->physical_device->memory_properties
              .memoryTypes[mem->base.vk.memory_type_index]
              .propertyFlags &
           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0;
}

static VkResult
vn_device_memory_alloc(struct vn_device *dev,
                       struct vn_device_memory *mem,
                       const VkMemoryAllocateInfo *alloc_info)
{
#ifdef _WIN32
   if (vn_helios_submit_instance_mode(dev->instance) ==
       VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      /* ⛔ OFF by default, and the default is the configuration that was
       * measured. Every part of this route now works except one:
       * vn_renderer_helios_allocate_memory executes on helios->bootstrap, and
       * a session execute is not safe once the device's primary ring is live.
       * The next ring command -- the uncached direct vkCreateBuffer that
       * vn_buffer.c legitimately issues in this mode -- is then refused with
       * STATUS_INVALID_PARAMETER, the session is poisoned, and every D3D11
       * device on the box is removed. Isolated by control:
       * HELIOS_HOST_VISIBLE_SHARED=bo-only builds the same HVM1 allocation and
       * skips only the session execute, and the probe runs clean.
       *
       * Turning it on today is strictly worse than the defect it fixes -- a
       * black desktop becomes no desktop -- so it stays off until the allocate
       * is issued on the device's own ring with its import operand patched
       * there. HELIOS_HOST_VISIBLE_SHARED=1 is the arm.
       *
       * Page-granular only: a sub-page allocation rounded up to a page is
       * refused before any KMD counter fires, and DXVK asks for 64-byte
       * host-visible allocations. */
      const bool page_granular =
         mem->base.vk.size >= 4096 && (mem->base.vk.size & 4095) == 0;
      if (vn_helios_env_enabled("HELIOS_HOST_VISIBLE_SHARED") &&
          page_granular && vn_device_memory_is_host_visible(dev, mem) &&
          !mem->base.vk.export_handle_types) {
         const VkResult shared =
            vn_device_memory_alloc_helios_shared(dev, mem, alloc_info);
         if (shared == VK_SUCCESS)
            return shared;
         /* Every failure falls back rather than only the KMD's size refusal:
          * the deferred route is the status quo and always admits, so a hard
          * failure here would be a regression rather than a loud one. The
          * counter is what stays loud -- any nonzero value is host-visible
          * memory whose CPU view is still not the host's. */
         const uint32_t n = ++vn_helios_unshared_host_visible_count;
         if (n <= 16 || (n & 63) == 0)
            vn_renderer_helios_diag_log(
               "HHV1 host-visible NOT guest-backed n=%u bytes=%llu result=%d",
               n, (unsigned long long)mem->base.vk.size, (int)shared);
      }
      return vn_device_memory_defer_outer_allocate(dev, mem, alloc_info);
   }

   /* A3 owns every ordinary allocation as one HVM1 object before the host
    * import.  Export handle types are A6 and are not advertised or emulated. */
   if (mem->base.vk.export_handle_types)
      return VK_ERROR_FEATURE_NOT_PRESENT;
   struct vn_device_memory_alloc_info local_info;
   alloc_info = vn_device_memory_fix_alloc_info(
      alloc_info, 0, false, false, &local_info);
   local_info.alloc.memoryTypeIndex =
      vn_physical_device_renderer_memory_type_index(
         dev->physical_device, mem->base.vk.memory_type_index);
   VkResult result = vn_device_memory_bo_init(dev, mem);
   if (result != VK_SUCCESS)
      return result;
   VkDeviceMemory memory = vn_device_memory_to_handle(mem);
   result = vn_renderer_helios_allocate_memory(
      dev->renderer, vn_device_to_handle(dev), alloc_info, &memory,
      mem->base_bo);
   if (result != VK_SUCCESS)
      vn_device_memory_bo_fini(dev, mem);
   return result;
#else
   struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];

   const bool has_guest_vram = dev->renderer->info.has_guest_vram;
   const bool host_visible =
      mem_type->propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
   const bool export_alloc = mem_vk->export_handle_types;

   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type =
      vn_renderer_handle_type_for_guest(dev->physical_device,
                                        mem_vk->export_handle_types);
   struct vn_device_memory_alloc_info local_info;
   /* Preserve the caller's explicit DMA_BUF export handle exactly, matching
    * the image-side policy. */
   const bool preserve_export =
      vn_preserve_explicit_dmabuf_handle_types(
         dev->physical_device, mem_vk->export_handle_types);
#ifdef _WIN32
   mem->renderer_export_handle_types =
      mem_vk->export_handle_types
         ? (preserve_export ? mem_vk->export_handle_types
                            : vn_renderer_handle_type_for_guest(
                                 dev->physical_device,
                                 mem_vk->export_handle_types))
         : 0;
#endif
   if (!preserve_export &&
       mem_vk->export_handle_types &&
       mem_vk->export_handle_types != renderer_handle_type) {
      alloc_info = vn_device_memory_fix_alloc_info(
         alloc_info, renderer_handle_type, has_guest_vram, false,
         &local_info);
   }

   if (has_guest_vram && (host_visible || export_alloc)) {
      return vn_device_memory_alloc_guest_vram(dev, mem, alloc_info);
   } else if (export_alloc) {
      return vn_device_memory_alloc_export(dev, mem, alloc_info);
   } else {
      return vn_device_memory_alloc_simple(dev, mem, alloc_info);
   }
#endif
}

#ifdef _WIN32
static VkResult
vn_device_memory_import_win32(
   struct vn_device *dev,
   struct vn_device_memory *mem,
   const VkMemoryAllocateInfo *alloc_info,
   const VkImportMemoryWin32HandleInfoKHR *import_info)
{
   const VkMemoryDedicatedAllocateInfo *dedicated =
      vk_find_struct_const(alloc_info->pNext,
                           MEMORY_DEDICATED_ALLOCATE_INFO);
   struct vn_image *dedicated_image =
      dedicated && dedicated->image != VK_NULL_HANDLE
         ? vn_image_from_handle(dedicated->image)
         : NULL;
   if (import_info->handleType !=
          VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT ||
       !import_info->handle || import_info->name ||
       alloc_info->memoryTypeIndex != VN_HELIOS_MEMORY_TYPE_DEVICE_LOCAL ||
       !dedicated_image ||
       dedicated_image->base.vk.base.device != &dev->base.vk ||
       dedicated_image->base.vk.external_handle_types !=
          VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT ||
       dedicated->buffer != VK_NULL_HANDLE ||
       !(dedicated_image->requirements[0].memory.memoryRequirements
            .memoryTypeBits &
         (UINT32_C(1) << VN_HELIOS_MEMORY_TYPE_DEVICE_LOCAL)) ||
       mem->base.vk.export_handle_types)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   uint64_t payload_size = 0;
   HeliosWddmAllocationDescV2 payload_desc;
   struct vn_renderer_bo *bo = NULL;
   struct vn_renderer_helios_external_memory *external = NULL;
   VkResult result = vn_renderer_helios_external_memory_open(
      dev->renderer, import_info, alloc_info->allocationSize, &payload_size,
      &payload_desc, &bo, &external);
   if (result != VK_SUCCESS)
      return result;
   if (payload_desc.allocation_kind != HELIOS_HWA2_KIND_IMAGE) {
      vn_renderer_bo_unref(dev->renderer, bo);
      vn_renderer_helios_external_memory_destroy(dev->renderer, external);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   struct vn_device_memory_alloc_info clean_info;
   VkMemoryAllocateInfo local = *vn_device_memory_fix_alloc_info(
      alloc_info, 0, false, false, &clean_info);
   local.allocationSize = payload_size;
   local.memoryTypeIndex = vn_physical_device_renderer_memory_type_index(
      dev->physical_device, VN_HELIOS_MEMORY_TYPE_DEVICE_LOCAL);
   VkDeviceMemory memory = vn_device_memory_to_handle(mem);
   result = vn_renderer_helios_allocate_memory(
      dev->renderer, vn_device_to_handle(dev), &local, &memory, bo);
   if (result != VK_SUCCESS) {
      vn_renderer_bo_unref(dev->renderer, bo);
      vn_renderer_helios_external_memory_destroy(dev->renderer, external);
      return result;
   }

   /* HWA2 is validation evidence only.  Identity remains the retained local
    * allocation object plus its immutable generation in `bo`. */
   (void)payload_desc;
   /* allocationSize is ignored for D3D12_RESOURCE imports; retain the opened
    * allocation's checked HWA2 extent as the VkDeviceMemory object's truth. */
   mem->base.vk.size = payload_size;
   mem->base_bo = bo;
   mem->helios_external_memory = external;
   return VK_SUCCESS;
}

#endif

static void
vn_device_memory_emit_report(struct vn_device *dev,
                             struct vn_device_memory *mem,
                             bool is_alloc,
                             VkResult result)
{
   struct vk_device *dev_vk = &dev->base.vk;

   if (likely(!dev_vk->memory_reports))
      return;

   const struct vk_device_memory *mem_vk = &mem->base.vk;
   VkDeviceMemoryReportEventTypeEXT type;
   if (result != VK_SUCCESS) {
      type = VK_DEVICE_MEMORY_REPORT_EVENT_TYPE_ALLOCATION_FAILED_EXT;
   } else if (is_alloc) {
      type = mem_vk->import_handle_type
                ? VK_DEVICE_MEMORY_REPORT_EVENT_TYPE_IMPORT_EXT
                : VK_DEVICE_MEMORY_REPORT_EVENT_TYPE_ALLOCATE_EXT;
   } else {
      type = mem_vk->import_handle_type
                ? VK_DEVICE_MEMORY_REPORT_EVENT_TYPE_UNIMPORT_EXT
                : VK_DEVICE_MEMORY_REPORT_EVENT_TYPE_FREE_EXT;
   }
   const uint64_t mem_obj_id =
#ifdef _WIN32
      mem->base.id;
#else
      (mem_vk->import_handle_type | mem_vk->export_handle_types) &&
            mem->base_bo
         ? mem->base_bo->res_id
         : mem->base.id;
#endif
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   vk_emit_device_memory_report(dev_vk, type, mem_obj_id, mem_vk->size,
                                VK_OBJECT_TYPE_DEVICE_MEMORY, (uintptr_t)mem,
                                mem_type->heapIndex);
}

#ifdef _WIN32
#define VN_HELIOS_MAX_OUTER_ALLOCATIONS HELIOS_HOB1_MAX_USE_RECORDS

static VkResult
vn_device_memory_reserve_outer_association(
   struct vn_device *dev,
   struct vn_device_memory *mem,
   const VkMemoryAllocateInfo *alloc_info)
{
   const HeliosResourceAssociationV1 *association = NULL;
   uint32_t association_count = 0;

   for (const VkBaseInStructure *node = alloc_info->pNext; node;
        node = node->pNext) {
      if ((uint32_t)node->sType ==
          HELIOS_RESOURCE_ASSOCIATION_STRUCTURE_TYPE) {
         association = (const HeliosResourceAssociationV1 *)node;
         association_count++;
      }
   }

   list_inithead(&mem->helios_outer_link);
   list_inithead(&mem->helios_deferred_records);
   memset(&mem->helios_outer, 0, sizeof(mem->helios_outer));
   mem->helios_deferred_record_count = 0;
   mem->helios_outer_registered = false;
   mem->helios_host_materialized = false;
   mem->helios_free_pending = false;
   memset(&mem->helios_free_allocator, 0,
          sizeof(mem->helios_free_allocator));
   if (!association_count) {
      if (vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
         vn_helios_record_note_deferred_use(dev->instance);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      return VK_SUCCESS;
   }

   const uint64_t expected_generation =
      vn_renderer_helios_session_generation(dev->renderer);
   const VkMemoryType *memory_type =
      &dev->physical_device->memory_properties.memoryTypes
         [mem->base.vk.memory_type_index];
   const bool host_visible =
      memory_type->propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
   const bool has_cpu_mapping =
      association->association_flags &
      HELIOS_RESOURCE_ASSOCIATION_FLAG_CPU_MAPPING;
   const uintptr_t cpu_mapping = (uintptr_t)association->cpu_mapping;
   if (association_count != 1 ||
       vn_helios_submit_instance_mode(dev->instance) !=
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY ||
       association->s_type != HELIOS_RESOURCE_ASSOCIATION_STRUCTURE_TYPE ||
       association->struct_bytes != HELIOS_RESOURCE_ASSOCIATION_BYTES ||
       association->abi_version != HELIOS_RESOURCE_ASSOCIATION_ABI_VERSION ||
       association->reserved != 0 ||
       association->package_generation != HELIOS_PACKAGE_GENERATION ||
       !expected_generation ||
       association->device_generation != expected_generation ||
       !association->outer_allocation_token ||
       !association->outer_allocation_bytes ||
       alloc_info->allocationSize > association->outer_allocation_bytes ||
       (association->association_flags &
        ~HELIOS_RESOURCE_ASSOCIATION_FLAG_MASK) ||
       association->reserved1 ||
       (!!association->cpu_mapping != has_cpu_mapping) ||
       (host_visible && !has_cpu_mapping) ||
       (has_cpu_mapping &&
        ((cpu_mapping & 4095u) ||
         association->outer_allocation_bytes > UINTPTR_MAX - cpu_mapping)))
      return VK_ERROR_VALIDATION_FAILED_EXT;

   /* Keep only the immutable record.  Its optional CPU pointer is a data view,
    * never identity; the caller's pNext pointer is stack-owned and is never
    * retained as identity or as a later traversal edge. */
   mem->helios_outer = *association;
   mem->helios_outer.p_next = NULL;

   simple_mtx_lock(&dev->mutex);
   if (dev->helios_outer_allocation_count >=
       VN_HELIOS_MAX_OUTER_ALLOCATIONS) {
      simple_mtx_unlock(&dev->mutex);
      memset(&mem->helios_outer, 0, sizeof(mem->helios_outer));
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   list_for_each_entry(struct vn_device_memory, live,
                       &dev->helios_outer_allocations,
                       helios_outer_link) {
      if (live->helios_outer.outer_allocation_token ==
          association->outer_allocation_token) {
         simple_mtx_unlock(&dev->mutex);
         memset(&mem->helios_outer, 0, sizeof(mem->helios_outer));
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
   }
   list_addtail(&mem->helios_outer_link, &dev->helios_outer_allocations);
   dev->helios_outer_allocation_count++;
   mem->helios_outer_registered = true;
   simple_mtx_unlock(&dev->mutex);
   return VK_SUCCESS;
}

struct vn_helios_deferred_record *
vn_device_memory_helios_record_create(struct vn_device_memory *mem,
                                      void *payload,
                                      size_t payload_bytes,
                                      uint32_t resource_operand_offset,
                                      bool allocation_record,
                                      bool final_free_record)
{
   if (!mem || !payload || !payload_bytes || payload_bytes > UINT32_MAX ||
       (resource_operand_offset != VN_HELIOS_NO_RESOURCE_OPERAND &&
        (payload_bytes < sizeof(uint32_t) ||
         (resource_operand_offset & (HELIOS_HOB1_OPERAND_ALIGN - 1u)) ||
         resource_operand_offset > payload_bytes - sizeof(uint32_t)))) {
      free(payload);
      return NULL;
   }
   struct vn_helios_deferred_record *record = calloc(1, sizeof(*record));
   if (!record) {
      free(payload);
      return NULL;
   }
   list_inithead(&record->link);
   record->owner = mem;
   record->payload = payload;
   record->payload_bytes = (uint32_t)payload_bytes;
   record->resource_operand_offset = resource_operand_offset;
   record->allocation_record = allocation_record;
   record->final_free_record = final_free_record;
   return record;
}

VkResult
vn_device_memory_helios_records_install(
   struct vn_device *dev,
   struct vn_helios_deferred_record *const *records,
   uint32_t record_count)
{
   if (!dev || (record_count && !records) ||
       record_count > HELIOS_HOB1_MAX_USE_RECORDS)
      return VK_ERROR_VALIDATION_FAILED_EXT;

   VkResult result = VK_SUCCESS;
   simple_mtx_lock(&dev->mutex);
   for (uint32_t i = 0; i < record_count; i++) {
      struct vn_helios_deferred_record *record = records[i];
      struct vn_device_memory *mem = record ? record->owner : NULL;
      uint32_t same_owner_before = 0;
      for (uint32_t j = 0; j < i; j++)
         same_owner_before += records[j]->owner == mem;
      if (!record || !mem || !list_is_empty(&record->link) ||
          !mem->helios_outer_registered ||
          mem->base.vk.base.device != &dev->base.vk ||
          (mem->helios_free_pending != record->final_free_record) ||
          same_owner_before >= HELIOS_HOB1_MAX_USE_RECORDS -
                                  mem->helios_deferred_record_count ||
          (record->allocation_record &&
           (mem->helios_host_materialized ||
            mem->helios_deferred_record_count != 0 ||
            same_owner_before != 0))) {
         result = VK_ERROR_VALIDATION_FAILED_EXT;
         break;
      }
   }
   if (result == VK_SUCCESS) {
      for (uint32_t i = 0; i < record_count; i++) {
         struct vn_helios_deferred_record *record = records[i];
         struct vn_device_memory *mem = record->owner;
         list_addtail(&record->link, &mem->helios_deferred_records);
         mem->helios_deferred_record_count++;
      }
   }
   simple_mtx_unlock(&dev->mutex);
   return result;
}

VkResult
vn_device_memory_helios_record_install(
   struct vn_device *dev,
   struct vn_device_memory *mem,
   struct vn_helios_deferred_record *record)
{
   if (!record || record->owner != mem)
      return VK_ERROR_VALIDATION_FAILED_EXT;
   return vn_device_memory_helios_records_install(dev, &record, 1);
}

void
vn_device_memory_helios_record_destroy(
   struct vn_helios_deferred_record *record)
{
   if (!record)
      return;
   assert(list_is_empty(&record->link));
   free(record->payload);
   free(record);
}

struct vn_device_memory *
vn_device_memory_helios_binding_memory(
   struct vn_device *dev,
   const struct vn_helios_memory_binding *binding)
{
   if (!dev || !binding || !binding->valid)
      return NULL;
   struct vn_device_memory *match = NULL;
   simple_mtx_lock(&dev->mutex);
   list_for_each_entry(struct vn_device_memory, mem,
                       &dev->helios_outer_allocations,
                       helios_outer_link) {
      if (mem->helios_outer_registered &&
          mem->helios_outer.device_generation ==
             binding->device_generation &&
          mem->helios_outer.outer_allocation_token ==
             binding->outer_allocation_token &&
          mem->helios_outer.outer_allocation_bytes ==
             binding->outer_allocation_bytes) {
         if (match) {
            match = NULL;
            break;
         }
         match = mem;
      }
   }
   simple_mtx_unlock(&dev->mutex);
   return match;
}

static bool
vn_device_memory_release_outer_association(struct vn_device *dev,
                                           struct vn_device_memory *mem)
{
   if (!mem->helios_outer_registered)
      return true;
   struct list_head retired;
   list_inithead(&retired);
   simple_mtx_lock(&dev->mutex);
   if (mem->helios_outer_registered) {
      list_for_each_entry(struct vn_helios_deferred_record, record,
                          &mem->helios_deferred_records, link) {
         if (record->reserved_context_generation ||
             record->reserved_batch_id) {
            simple_mtx_unlock(&dev->mutex);
            return false;
         }
      }
      list_splicetail(&mem->helios_deferred_records, &retired);
      list_inithead(&mem->helios_deferred_records);
      mem->helios_deferred_record_count = 0;
      list_delinit(&mem->helios_outer_link);
      assert(dev->helios_outer_allocation_count > 0);
      dev->helios_outer_allocation_count--;
      mem->helios_outer_registered = false;
   }
   simple_mtx_unlock(&dev->mutex);
   list_for_each_entry_safe(struct vn_helios_deferred_record, record,
                            &retired, link) {
      list_delinit(&record->link);
      vn_device_memory_helios_record_destroy(record);
   }
   memset(&mem->helios_outer, 0, sizeof(mem->helios_outer));
   mem->helios_host_materialized = false;
   return true;
}

bool
vn_device_memory_helios_finalize_pending_free(
   struct vn_device *dev,
   struct vn_device_memory *mem)
{
   if (!dev || !mem || !mem->helios_free_pending ||
       mem->base.vk.base.device != &dev->base.vk)
      return false;
   if (!vn_device_memory_release_outer_association(dev, mem))
      return false;

   mem->helios_free_pending = false;
   vn_device_memory_bo_fini(dev, mem);
   if (mem->helios_external_memory) {
      vn_renderer_helios_external_memory_destroy(
         dev->renderer, mem->helios_external_memory);
      mem->helios_external_memory = NULL;
   }
   const VkAllocationCallbacks allocator = mem->helios_free_allocator;
   vk_device_memory_destroy(&dev->base.vk, &allocator, &mem->base.vk);
   return true;
}

VkResult
vn_device_memory_helios_bind(struct vn_device *dev,
                             struct vn_device_memory *mem,
                             VkDeviceSize offset,
                             VkDeviceSize length,
                             struct vn_helios_memory_binding *out)
{
   if (!dev || !mem || !out || !length ||
       mem->base.vk.base.device != &dev->base.vk ||
       !mem->helios_outer_registered)
      return VK_ERROR_VALIDATION_FAILED_EXT;

   const uint64_t end = (uint64_t)offset + (uint64_t)length;
   if (end < (uint64_t)offset || end > mem->base.vk.size ||
       end > mem->helios_outer.outer_allocation_bytes)
      return VK_ERROR_VALIDATION_FAILED_EXT;

   struct vn_helios_memory_binding binding = {
      .device_generation = mem->helios_outer.device_generation,
      .outer_allocation_token =
         mem->helios_outer.outer_allocation_token,
      .outer_allocation_bytes =
         mem->helios_outer.outer_allocation_bytes,
      .byte_offset = offset,
      .byte_length = length,
      .valid = true,
   };
   if (!vn_device_memory_helios_binding_live(dev, &binding))
      return VK_ERROR_VALIDATION_FAILED_EXT;
   *out = binding;
   return VK_SUCCESS;
}

bool
vn_device_memory_helios_binding_live(
   struct vn_device *dev,
   const struct vn_helios_memory_binding *binding)
{
   if (!dev || !binding || !binding->valid ||
       !binding->device_generation || !binding->outer_allocation_token ||
       !binding->byte_length)
      return false;
   const uint64_t end = binding->byte_offset + binding->byte_length;
   if (end < binding->byte_offset ||
       end > binding->outer_allocation_bytes)
      return false;

   bool found = false;
   simple_mtx_lock(&dev->mutex);
   list_for_each_entry(struct vn_device_memory, live,
                       &dev->helios_outer_allocations,
                       helios_outer_link) {
      if (live->helios_outer.device_generation ==
             binding->device_generation &&
          live->helios_outer.outer_allocation_token ==
             binding->outer_allocation_token &&
          live->helios_outer.outer_allocation_bytes ==
             binding->outer_allocation_bytes) {
         found = true;
         break;
      }
   }
   simple_mtx_unlock(&dev->mutex);
   return found;
}
#endif

VKAPI_ATTR VkResult VKAPI_CALL
vn_AllocateMemory(VkDevice device,
                  const VkMemoryAllocateInfo *pAllocateInfo,
                  const VkAllocationCallbacks *pAllocator,
                  VkDeviceMemory *pMemory)
{
   struct vn_device *dev = vn_device_from_handle(device);

   struct vn_device_memory *mem = vk_device_memory_create(
      &dev->base.vk, pAllocateInfo, pAllocator, sizeof(*mem));
   if (!mem)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

   list_inithead(&mem->coherent_cached_link);
   vn_object_set_id(mem, vn_get_next_obj_id(), VK_OBJECT_TYPE_DEVICE_MEMORY);

#ifdef _WIN32
   VkResult association_result =
      vn_device_memory_reserve_outer_association(dev, mem, pAllocateInfo);
   if (association_result != VK_SUCCESS) {
      vn_renderer_helios_diag_log(
         "HAM1 vkAllocateMemory ASSOC-REFUSED result=%d size=%llu type=%u "
         "mode=%d",
         association_result,
         (unsigned long long)pAllocateInfo->allocationSize,
         pAllocateInfo->memoryTypeIndex,
         (int)vn_helios_submit_instance_mode(dev->instance));
      vk_device_memory_destroy(&dev->base.vk, pAllocator, &mem->base.vk);
      return vn_error(dev->instance, association_result);
   }

   const bool preserve_export =
      vn_preserve_explicit_dmabuf_handle_types(
         dev->physical_device, mem->base.vk.export_handle_types);
   mem->renderer_export_handle_types =
      mem->base.vk.export_handle_types
         ? (preserve_export
               ? mem->base.vk.export_handle_types
               : vn_renderer_handle_type_for_guest(
                    dev->physical_device,
                    mem->base.vk.export_handle_types))
         : 0;
#endif

#if !defined(_WIN32)
   const VkImportMemoryFdInfoKHR *import_fd_info =
      vk_find_struct_const(pAllocateInfo->pNext, IMPORT_MEMORY_FD_INFO_KHR);
   const VkImportMemoryResourceInfoMESA *import_resource_info =
      (const VkImportMemoryResourceInfoMESA *)__vk_find_struct(
         (void *)pAllocateInfo->pNext,
         VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA);
#endif
#ifdef _WIN32
   const VkImportMemoryWin32HandleInfoKHR *import_win32_info =
      vk_find_struct_const(pAllocateInfo->pNext,
                           IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR);
#endif

   VkResult result;
   if (mem->base.vk.ahardware_buffer) {
      result = vn_android_device_import_ahb(dev, mem, pAllocateInfo);
#ifdef _WIN32
   } else if (import_win32_info) {
      result = vn_device_memory_import_win32(dev, mem, pAllocateInfo,
                                             import_win32_info);
#endif
   }
#if !defined(_WIN32)
   else if (import_resource_info) {
      struct vn_device_memory_alloc_info local_info;
      const bool preserve_resource_export =
         vn_preserve_explicit_dmabuf_handle_types(
            dev->physical_device, mem->base.vk.export_handle_types);
      const VkExternalMemoryHandleTypeFlagBits resource_export_type =
         preserve_resource_export && mem->base.vk.export_handle_types
            ? (VkExternalMemoryHandleTypeFlagBits)
                 mem->base.vk.export_handle_types
            : vn_renderer_handle_type_for_guest(
                 dev->physical_device,
                 mem->base.vk.export_handle_types);
      const VkMemoryAllocateInfo *renderer_alloc_info =
         vn_device_memory_fix_alloc_info(pAllocateInfo,
                                         resource_export_type, false, false,
                                         &local_info);
      result = vn_device_memory_import_resource_id(
         dev, mem, renderer_alloc_info, import_resource_info->resourceId);
   } else if (import_fd_info) {
      result = vn_device_memory_import_dma_buf(dev, mem, pAllocateInfo,
                                               import_fd_info->fd);
   }
#endif
   else {
      result = vn_device_memory_alloc(dev, mem, pAllocateInfo);
      if (result == VK_SUCCESS)
         vn_wsi_memory_info_init(mem, pAllocateInfo);
   }

   vn_device_memory_emit_report(dev, mem, /* is_alloc */ true, result);

   if (result != VK_SUCCESS) {
#ifdef _WIN32
      vn_renderer_helios_diag_log(
         "HAM1 vkAllocateMemory FAILED result=%d size=%llu type=%u assoc=%d "
         "mode=%d",
         result, (unsigned long long)pAllocateInfo->allocationSize,
         pAllocateInfo->memoryTypeIndex, mem->helios_outer_registered ? 1 : 0,
         (int)vn_helios_submit_instance_mode(dev->instance));
      (void)vn_device_memory_release_outer_association(dev, mem);
#endif
      vk_device_memory_destroy(&dev->base.vk, pAllocator, &mem->base.vk);
      return vn_error(dev->instance, result);
   }

   /* ⛔ The VidMm accounting mirror that stood here is DELETED. Every ordinary,
    * non-imported vkAllocateMemory used to create a second, content-free WDDM
    * "tracking" allocation purely so Windows budget and Task Manager would show
    * a charge for renderer-owned venus memory. That mechanism has no successor
    * (K4-CONTRACT §6) and re-adding it is forbidden by §10.3. The named
    * consequence: venus device memory is invisible to the Windows video-memory
    * budget until the KMD owns the allocation itself (HVM1, mesa unit A3 plus
    * the KMD lane). The bytes were always renderer-owned; only the mirror is
    * gone. */

   *pMemory = vn_device_memory_to_handle(mem);

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_FreeMemory(VkDevice device,
              VkDeviceMemory memory,
              const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   if (!mem)
      return;

   if (vn_helios_env_enabled("HELIOS_RELEASE_TRACE"))
      fprintf(stderr, "Helios vn_FreeMemory mem=%p base_bo=%p\n", mem,
              mem->base_bo);

   vn_device_memory_emit_report(dev, mem, /* is_alloc */ false, VK_SUCCESS);
   vn_device_memory_unregister_coherent_cached_mapping(dev, mem);

#ifdef _WIN32
   const bool record_only =
      vn_helios_submit_instance_mode(dev->instance) ==
      VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
   const bool host_materialized = mem->helios_host_materialized;

   if (record_only && mem->base_bo) {
      /* Storage this ICD owns. Its host object was created synchronously
       * through the session, so it is retired the same way rather than through
       * the deferred-record machinery, which describes outer-allocation
       * lifetimes this memory does not have. */
      if (!vn_device_memory_release_outer_association(dev, mem)) {
         p_atomic_set(&dev->helios_lost, 1);
         (void)vn_error(dev->instance, VK_ERROR_DEVICE_LOST);
         return;
      }
      const VkResult free_result = vn_renderer_helios_free_memory(
         dev->renderer, device, memory, mem->base_bo);
      if (free_result != VK_SUCCESS)
         (void)vn_error(dev->instance, free_result);
      vn_device_memory_bo_fini(dev, mem);
      vk_device_memory_destroy(&dev->base.vk, pAllocator, &mem->base.vk);
      return;
   }

   if (record_only) {
      /* An allocation that never appeared in an accepted batch has no host
       * namespace object to destroy and no queue milestone to join.  Discard
       * its unconsumed immutable records and retire local ownership directly,
       * including when DXVK opened a teardown scope before issuing Destroy and
       * Free.  Emitting allocate/bind/destroy/free into that empty scope would
       * create an allocation-only A7 stream with no real final queue operation
       * (measured as Nr2OuterRej reason Schema on the bounded create/destroy
       * probe).  The scope consequently remains empty and closes ABANDONED. */
      if (!host_materialized) {
         const VkAllocationCallbacks *free_alloc =
            pAllocator ? pAllocator : &dev->base.vk.alloc;
         mem->helios_free_allocator = *free_alloc;
         mem->helios_free_pending = true;
         if (!vn_device_memory_helios_finalize_pending_free(dev, mem)) {
            vn_helios_record_note_deferred_use(dev->instance);
            p_atomic_set(&dev->helios_lost, 1);
            (void)vn_error(dev->instance, VK_ERROR_DEVICE_LOST);
         }
         return;
      }

      bool joined = true;
      /* A materialized allocation can still be referenced by every exact
       * outer context that consumed it.  Join those HQC1 values before adding
       * the terminal FreeMemory record to the calling thread's teardown
       * scope.  Ring zero and a lower Vulkan wait are not substitutes. */
      struct vn_helios_outer_progress
         progress_points[HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION];
      uint32_t progress_count = 0;
      simple_mtx_lock(&dev->mutex);
      progress_count = mem->helios_outer_progress_count;
      memcpy(progress_points, mem->helios_outer_progress,
             sizeof(progress_points));
      simple_mtx_unlock(&dev->mutex);
      if (host_materialized && !progress_count)
         joined = false;
      for (uint32_t i = 0; joined && i < progress_count; i++) {
         HeliosSyncProgressResultV1 progress;
         memset(&progress, 0, sizeof(progress));
         const HeliosTranslatorStatusCode status =
            vn_helios_direct_join_context(
               dev->instance, progress_points[i].context_generation,
               progress_points[i].progress_value, &progress);
         if (status != HELIOS_TRANSLATOR_STATUS_OK ||
             (progress.flags & HELIOS_TRANSLATOR_PROGRESS_FLAG_DEVICE_LOST)) {
            joined = false;
         }
      }

      const VkAllocationCallbacks *free_alloc =
         pAllocator ? pAllocator : &dev->base.vk.alloc;
      mem->helios_free_allocator = *free_alloc;
      mem->helios_free_pending = true;

      VkResult teardown_result = joined ? VK_SUCCESS
                                        : VK_ERROR_DEVICE_LOST;
      const size_t payload_bytes =
         vn_sizeof_vkFreeMemory(device, memory, NULL);
      uint8_t *payload = NULL;
      struct vn_helios_deferred_record *record = NULL;
      if (teardown_result == VK_SUCCESS) {
         if (!payload_bytes || payload_bytes > HELIOS_HOB1_MAX_BYTES ||
             !(payload = malloc(payload_bytes))) {
            teardown_result = VK_ERROR_OUT_OF_HOST_MEMORY;
         }
      }
      if (teardown_result == VK_SUCCESS) {
         struct vn_cs_encoder encoder =
            VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_bytes);
         vn_encode_vkFreeMemory(&encoder, 0, device, memory, NULL);
         if (vn_cs_encoder_get_fatal(&encoder) ||
             vn_cs_encoder_get_len(&encoder) != payload_bytes) {
            free(payload);
            payload = NULL;
            teardown_result = VK_ERROR_INITIALIZATION_FAILED;
         }
      }
      if (teardown_result == VK_SUCCESS) {
         record = vn_device_memory_helios_record_create(
            mem, payload, payload_bytes, VN_HELIOS_NO_RESOURCE_OPERAND,
            false, true);
         payload = NULL;
         if (!record) {
            teardown_result = VK_ERROR_OUT_OF_HOST_MEMORY;
         } else {
            teardown_result =
               vn_device_memory_helios_record_install(dev, mem, record);
            if (teardown_result != VK_SUCCESS) {
               vn_device_memory_helios_record_destroy(record);
               record = NULL;
            }
         }
      }
      if (teardown_result == VK_SUCCESS)
         teardown_result = vn_helios_record_memory_teardown(dev, mem);
      if (teardown_result == VK_SUCCESS)
         return;

      free(payload);
      vn_helios_record_note_deferred_use(dev->instance);
      p_atomic_set(&dev->helios_lost, 1);
      (void)vn_error(dev->instance, teardown_result);
      /* No host command was accepted.  Remove the association before local
       * storage can be reused; the lost session owns any unexecuted host
       * objects until its normal namespace teardown. */
      if (!vn_device_memory_helios_finalize_pending_free(dev, mem))
         (void)vn_error(dev->instance, VK_ERROR_DEVICE_LOST);
      return;
   }

   if (!vn_device_memory_release_outer_association(dev, mem)) {
      p_atomic_set(&dev->helios_lost, 1);
      (void)vn_error(dev->instance, VK_ERROR_DEVICE_LOST);
      return;
   }
   if (mem->base_bo) {
      VkResult free_result = vn_renderer_helios_free_memory(
         dev->renderer, device, memory, mem->base_bo);
      if (free_result != VK_SUCCESS)
         (void)vn_error(dev->instance, free_result);
   }
   vn_device_memory_bo_fini(dev, mem);
   if (mem->helios_external_memory) {
      vn_renderer_helios_external_memory_destroy(
         dev->renderer, mem->helios_external_memory);
      mem->helios_external_memory = NULL;
   }
#else
   /* ensure renderer side import still sees the resource */
   vn_device_memory_bo_fini(dev, mem);

   if (mem->bo_roundtrip_seqno_valid)
      vn_ring_wait_roundtrip(dev->primary_ring, mem->bo_roundtrip_seqno);

   vn_device_memory_free_simple(dev, mem);
#endif
   vk_device_memory_destroy(&dev->base.vk, pAllocator, &mem->base.vk);
}

#ifdef _WIN32
VKAPI_ATTR VkResult VKAPI_CALL
vn_GetMemoryWin32HandleKHR(
   VkDevice device,
   const VkMemoryGetWin32HandleInfoKHR *pGetWin32HandleInfo,
   HANDLE *pHandle)
{
   VN_TRACE_FUNC();
   struct vn_device *dev = vn_device_from_handle(device);
   (void)pGetWin32HandleInfo;
   *pHandle = NULL;
   /* A6 owns export capability.  A3 implements exact D3D12_RESOURCE import
    * only and never aliases it to OPAQUE_WIN32. */
   return vn_error(dev->instance, VK_ERROR_INVALID_EXTERNAL_HANDLE);
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_GetMemoryWin32HandlePropertiesKHR(
   VkDevice device,
   VkExternalMemoryHandleTypeFlagBits handleType,
   HANDLE handle,
   VkMemoryWin32HandlePropertiesKHR *pMemoryWin32HandleProperties)
{
   VN_TRACE_FUNC();
   struct vn_device *dev = vn_device_from_handle(device);
   pMemoryWin32HandleProperties->memoryTypeBits = 0;

   /* Vulkan forbids this query for opaque Win32 handle types.  C57 is an
    * import allocation class rather than an HVM1 memory role, but Vulkan still
    * requires one compatible allocation index.  A6 maps it only to guest role
    * 4; HWA2's memory_class is never interpreted as an index. */
   if (handleType != VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT)
      return vn_error(dev->instance, VK_ERROR_INVALID_EXTERNAL_HANDLE);

   const VkImportMemoryWin32HandleInfoKHR probe_info = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR,
      .handleType = handleType,
      .handle = handle,
   };
   uint64_t payload_size = 0;
   HeliosWddmAllocationDescV2 payload_desc;
   struct vn_renderer_bo *bo = NULL;
   struct vn_renderer_helios_external_memory *external = NULL;
   VkResult result = vn_renderer_helios_external_memory_open(
      dev->renderer, &probe_info, 0, &payload_size, &payload_desc, &bo,
      &external);
   if (result != VK_SUCCESS)
      return vn_error(dev->instance, VK_ERROR_INVALID_EXTERNAL_HANDLE);

   const bool exact_image =
      payload_desc.allocation_kind == HELIOS_HWA2_KIND_IMAGE;
   vn_renderer_bo_unref(dev->renderer, bo);
   vn_renderer_helios_external_memory_destroy(dev->renderer, external);
   if (!exact_image)
      return vn_error(dev->instance, VK_ERROR_INVALID_EXTERNAL_HANDLE);

   pMemoryWin32HandleProperties->memoryTypeBits =
      UINT32_C(1) << VN_HELIOS_MEMORY_TYPE_DEVICE_LOCAL;
   return VK_SUCCESS;
}
#endif

VKAPI_ATTR uint64_t VKAPI_CALL
vn_GetDeviceMemoryOpaqueCaptureAddress(
   VkDevice device, const VkDeviceMemoryOpaqueCaptureAddressInfo *pInfo)
{
   struct vn_device *dev = vn_device_from_handle(device);
   return vn_call_vkGetDeviceMemoryOpaqueCaptureAddress(dev->primary_ring,
                                                        device, pInfo);
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_MapMemory2(VkDevice device,
              const VkMemoryMapInfo *pMemoryMapInfo,
              void **ppData)
{
   VN_TRACE_FUNC();
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_device_memory *mem =
      vn_device_memory_from_handle(pMemoryMapInfo->memory);
#ifdef _WIN32
   if (vn_helios_submit_instance_mode(dev->instance) ==
       VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      if (!ppData || !mem ||
          mem->base.vk.base.device != &dev->base.vk ||
          !mem->helios_outer_registered || mem->helios_free_pending ||
          pMemoryMapInfo->flags || pMemoryMapInfo->pNext ||
          mem->map_end > mem->map_start) {
         return vn_error(dev->instance,
                         VK_ERROR_VALIDATION_FAILED_EXT);
      }
      const struct vk_device_memory *outer_mem_vk = &mem->base.vk;
      if (mem->base_bo) {
         /* Storage this ICD owns (vn_device_memory_alloc_helios_shared): the
          * mapping is the allocation's own pages, which the host imported, so
          * it is the same memory on both sides. */
         VkDeviceSize bo_end = 0;
         if (pMemoryMapInfo->size == VK_WHOLE_SIZE) {
            bo_end = outer_mem_vk->size;
         } else if (!pMemoryMapInfo->size ||
                    __builtin_add_overflow(pMemoryMapInfo->offset,
                                           pMemoryMapInfo->size, &bo_end)) {
            return vn_error(dev->instance, VK_ERROR_VALIDATION_FAILED_EXT);
         }
         if (bo_end <= pMemoryMapInfo->offset || bo_end > outer_mem_vk->size)
            return vn_error(dev->instance, VK_ERROR_VALIDATION_FAILED_EXT);
         void *bo_ptr = vn_renderer_bo_map(dev->renderer, mem->base_bo, NULL);
         if (!bo_ptr)
            return vn_error(dev->instance, VK_ERROR_MEMORY_MAP_FAILED);
         mem->map_start = pMemoryMapInfo->offset;
         mem->map_end = bo_end;
         *ppData = (char *)bo_ptr + pMemoryMapInfo->offset;
         return VK_SUCCESS;
      }
      const VkMemoryType *memory_type =
         &dev->physical_device->memory_properties.memoryTypes
            [outer_mem_vk->memory_type_index];
      if (!(memory_type->propertyFlags &
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) ||
          !(memory_type->propertyFlags &
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) ||
          !(mem->helios_outer.association_flags &
            HELIOS_RESOURCE_ASSOCIATION_FLAG_CPU_MAPPING) ||
          !mem->helios_outer.cpu_mapping) {
         return vn_error(dev->instance, VK_ERROR_MEMORY_MAP_FAILED);
      }

      const VkDeviceSize outer_offset = pMemoryMapInfo->offset;
      VkDeviceSize outer_end = 0;
      if (pMemoryMapInfo->size == VK_WHOLE_SIZE) {
         outer_end = outer_mem_vk->size;
      } else if (!pMemoryMapInfo->size ||
                 __builtin_add_overflow(outer_offset,
                                        pMemoryMapInfo->size,
                                        &outer_end)) {
         return vn_error(dev->instance,
                         VK_ERROR_VALIDATION_FAILED_EXT);
      }
      const uintptr_t mapping =
         (uintptr_t)mem->helios_outer.cpu_mapping;
      if (outer_end <= outer_offset || outer_end > outer_mem_vk->size ||
          outer_end > mem->helios_outer.outer_allocation_bytes ||
          outer_offset > UINTPTR_MAX - mapping) {
         return vn_error(dev->instance,
                         VK_ERROR_VALIDATION_FAILED_EXT);
      }

      mem->map_start = outer_offset;
      mem->map_end = outer_end;
      *ppData = (void *)(mapping + (uintptr_t)outer_offset);
      return VK_SUCCESS;
   }
#endif
   const VkDeviceSize offset = pMemoryMapInfo->offset;
   const VkDeviceSize size = pMemoryMapInfo->size;
   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const bool need_bo = !mem->base_bo;
   void *placed_addr = NULL;
   void *ptr = NULL;
   VkResult result;

   /* We don't want to blindly create a bo for each HOST_VISIBLE memory as
    * that has a cost. By deferring bo creation until now, we can avoid the
    * cost unless a bo is really needed. However, that means
    * vn_renderer_bo_map will block until the renderer creates the resource
    * and injects the pages into the guest.
    *
    * XXX We also assume that a vn_renderer_bo can be created as long as the
    * renderer VkDeviceMemory has a mappable memory type.  That is plain
    * wrong.  It is impossible to fix though until some new extension is
    * created and supported by the driver, and that the renderer switches to
    * the extension.
    */
   if (need_bo) {
      result = vn_device_memory_bo_init(dev, mem);
      if (result != VK_SUCCESS)
         return vn_error(dev->instance, result);
   }

   /* A memory type that advertises HOST_CACHED must actually be mapped cached.
    * An app picks that type precisely to get fast CPU reads — DXVK does so for
    * every D3D11_USAGE_STAGING resource (d3d11_texture.cpp:1015) — and a WC
    * mapping silently breaks the promise, because an ordinary memcpy out of WC
    * memory is uncached and crawls.
    *
    * Measured, not assumed. In an RDP session the desktop-capture consumer
    * (RDPIDD, hosted in WUDFHost) runs CopyResource -> Map(READ) -> memcpy over
    * a 1920x1080 BGRA frame. tools/d3d11_rdp_capture_probe.cpp on that exact
    * resource desc, before this change:
    *
    *   memcpy from the WC mapping     25.209 ms     313.8 MB/s
    *   MOVNTDQA from the same pages    0.839 ms    9428.1 MB/s   <- WC signature
    *   memcpy, ordinary cached heap    0.202 ms   39081.8 MB/s
    *
    * ~25 ms of pure CPU per captured frame: it saturated a core and capped RDP
    * capture near 40 fps, which is what made the desktop lag whenever anything
    * on it was actually changing.
    *
    * The host already reports CACHED for these types; it was this override that
    * forced WC, since effective_map_cache() honours the ICD's request over the
    * host's. Cache maintenance stays free: the type is COHERENT, so guest WB
    * over host WB is hardware-coherent under KVM and helios_bo_needs_cache_ops()
    * returns false for exactly this flag combination.
    */
   mem->base_bo->prefer_cached_map =
      mem->wsi_buffer_blit_dst ||
      vn_device_memory_type_is_coherent_cached(dev, mem);

   if (pMemoryMapInfo->flags & VK_MEMORY_MAP_PLACED_BIT_EXT) {
      const VkMemoryMapPlacedInfoEXT *placed_info = vk_find_struct_const(
         pMemoryMapInfo->pNext, MEMORY_MAP_PLACED_INFO_EXT);
      assert(placed_info != NULL);
      placed_addr = placed_info->pPlacedAddress;
   }

   ptr = vn_renderer_bo_map(dev->renderer, mem->base_bo, placed_addr);
   if (!ptr) {
      /* vn_renderer_bo_map implies a roundtrip on success, but not here. */
      if (need_bo) {
         result = vn_ring_submit_roundtrip(dev->primary_ring,
                                           &mem->bo_roundtrip_seqno);
         if (result != VK_SUCCESS)
            return vn_error(dev->instance, result);

         mem->bo_roundtrip_seqno_valid = true;
      }

      return vn_error(dev->instance, VK_ERROR_MEMORY_MAP_FAILED);
   }

   mem->map_start = offset;
   mem->map_end = size == VK_WHOLE_SIZE ? mem_vk->size : offset + size;

   *ppData = ptr + offset;
   vn_device_memory_register_coherent_cached_mapping(dev, mem);

   return VK_SUCCESS;
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_UnmapMemory2(VkDevice device, const VkMemoryUnmapInfo *pMemoryUnmapInfo)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_device_memory *mem =
      vn_device_memory_from_handle(pMemoryUnmapInfo->memory);

#ifdef _WIN32
   if (vn_helios_submit_instance_mode(dev->instance) ==
       VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      if (!mem || mem->base.vk.base.device != &dev->base.vk ||
          (!mem->helios_outer_registered && !mem->base_bo) ||
          mem->helios_free_pending ||
          pMemoryUnmapInfo->flags || pMemoryUnmapInfo->pNext ||
          mem->map_end <= mem->map_start) {
         return vn_error(dev->instance,
                         VK_ERROR_VALIDATION_FAILED_EXT);
      }
      mem->map_start = 0;
      mem->map_end = 0;
      return VK_SUCCESS;
   }
#endif

   if (mem) {
      if (mem->coherent_cached_mapped && mem->base_bo &&
          mem->base_bo->mmap_ptr && mem->map_end > mem->map_start &&
          !mem->wsi_buffer_blit_dst) {
         vn_renderer_bo_flush(dev->renderer, mem->base_bo, mem->map_start,
                              mem->map_end - mem->map_start);
      }
      vn_device_memory_unregister_coherent_cached_mapping(dev, mem);
      mem->map_start = 0;
      mem->map_end = 0;
   }

   return VK_SUCCESS;
}

#ifdef _WIN32
static VkResult
vn_device_memory_validate_outer_mapped_ranges(
   struct vn_device *dev,
   uint32_t memory_range_count,
   const VkMappedMemoryRange *memory_ranges)
{
   if (memory_range_count && !memory_ranges)
      return VK_ERROR_VALIDATION_FAILED_EXT;

   for (uint32_t i = 0; i < memory_range_count; i++) {
      const VkMappedMemoryRange *range = &memory_ranges[i];
      struct vn_device_memory *mem =
         vn_device_memory_from_handle(range->memory);
      if (!mem || mem->base.vk.base.device != &dev->base.vk ||
          !mem->helios_outer_registered || mem->helios_free_pending ||
          range->pNext || mem->map_end <= mem->map_start ||
          !(mem->helios_outer.association_flags &
            HELIOS_RESOURCE_ASSOCIATION_FLAG_CPU_MAPPING) ||
          !mem->helios_outer.cpu_mapping) {
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      const struct vk_device_memory *mem_vk = &mem->base.vk;
      const VkMemoryType *memory_type =
         &dev->physical_device->memory_properties.memoryTypes
            [mem_vk->memory_type_index];
      if ((memory_type->propertyFlags &
           (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) !=
          (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
           VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)) {
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }

      VkDeviceSize end = 0;
      if (range->size == VK_WHOLE_SIZE) {
         end = mem_vk->size;
      } else if (!range->size ||
                 __builtin_add_overflow(range->offset, range->size,
                                        &end)) {
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      if (range->offset < mem->map_start || end <= range->offset ||
          end > mem->map_end || end > mem_vk->size ||
          end > mem->helios_outer.outer_allocation_bytes) {
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
   }
   return VK_SUCCESS;
}
#endif

VKAPI_ATTR VkResult VKAPI_CALL
vn_FlushMappedMemoryRanges(VkDevice device,
                           uint32_t memoryRangeCount,
                           const VkMappedMemoryRange *pMemoryRanges)
{
   struct vn_device *dev = vn_device_from_handle(device);

#ifdef _WIN32
   if (vn_helios_submit_instance_mode(dev->instance) ==
       VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      const VkResult result =
         vn_device_memory_validate_outer_mapped_ranges(
            dev, memoryRangeCount, pMemoryRanges);
      return result == VK_SUCCESS
                ? VK_SUCCESS
                : vn_error(dev->instance, result);
   }
#endif

   for (uint32_t i = 0; i < memoryRangeCount; i++) {
      const VkMappedMemoryRange *range = &pMemoryRanges[i];
      struct vn_device_memory *mem =
         vn_device_memory_from_handle(range->memory);

      const VkDeviceSize size = range->size == VK_WHOLE_SIZE
                                   ? mem->map_end - range->offset
                                   : range->size;
      vn_renderer_bo_flush(dev->renderer, mem->base_bo, range->offset, size);
   }

   return VK_SUCCESS;
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_InvalidateMappedMemoryRanges(VkDevice device,
                                uint32_t memoryRangeCount,
                                const VkMappedMemoryRange *pMemoryRanges)
{
   struct vn_device *dev = vn_device_from_handle(device);

#ifdef _WIN32
   if (vn_helios_submit_instance_mode(dev->instance) ==
       VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      const VkResult result =
         vn_device_memory_validate_outer_mapped_ranges(
            dev, memoryRangeCount, pMemoryRanges);
      return result == VK_SUCCESS
                ? VK_SUCCESS
                : vn_error(dev->instance, result);
   }
#endif

   for (uint32_t i = 0; i < memoryRangeCount; i++) {
      const VkMappedMemoryRange *range = &pMemoryRanges[i];
      struct vn_device_memory *mem =
         vn_device_memory_from_handle(range->memory);

      const VkDeviceSize size = range->size == VK_WHOLE_SIZE
                                   ? mem->map_end - range->offset
                                   : range->size;
      vn_renderer_bo_invalidate(dev->renderer, mem->base_bo, range->offset,
                                size);
   }

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_GetDeviceMemoryCommitment(VkDevice device,
                             VkDeviceMemory memory,
                             VkDeviceSize *pCommittedMemoryInBytes)
{
   struct vn_device *dev = vn_device_from_handle(device);
   vn_call_vkGetDeviceMemoryCommitment(dev->primary_ring, device, memory,
                                       pCommittedMemoryInBytes);
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_GetMemoryFdKHR(VkDevice device,
                  const VkMemoryGetFdInfoKHR *pGetFdInfo,
                  int *pFd)
{
   VN_TRACE_FUNC();
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_device_memory *mem =
      vn_device_memory_from_handle(pGetFdInfo->memory);

   /* At the moment, we support only the below handle types. */
   assert(pGetFdInfo->handleType &
          (VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT |
           VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT));
   assert(mem->base_bo);

   /* The Windows renderer has no fd export carrier.  Keep the unadvertised
    * entrypoint fail-closed instead of dereferencing a NULL operation. */
   if (!dev->renderer->bo_ops.export_dma_buf)
      return vn_error(dev->instance, VK_ERROR_FEATURE_NOT_PRESENT);

   *pFd = vn_renderer_bo_export_dma_buf(dev->renderer, mem->base_bo);
   if (*pFd < 0)
      return vn_error(dev->instance, VK_ERROR_TOO_MANY_OBJECTS);

   return VK_SUCCESS;
}

VkResult
vn_get_memory_dma_buf_properties(struct vn_device *dev,
                                 int fd,
                                 uint32_t *out_mem_type_bits)
{
#ifdef _WIN32
   (void)dev;
   (void)fd;
   *out_mem_type_bits = 0;
   return VK_ERROR_FEATURE_NOT_PRESENT;
#else
   VkDevice device = vn_device_to_handle(dev);

   struct vn_renderer_bo *bo;
   VkResult result = vn_renderer_bo_create_from_dma_buf(
      dev->renderer, 0 /* size */, fd, 0 /* flags */, &bo);
   if (result != VK_SUCCESS) {
      vn_log(dev->instance, "bo_create_from_dma_buf failed");
      return result;
   }

   vn_ring_roundtrip(dev->primary_ring);

   VkMemoryResourcePropertiesMESA props = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_RESOURCE_PROPERTIES_MESA,
   };
   result = vn_call_vkGetMemoryResourcePropertiesMESA(
      dev->primary_ring, device, bo->res_id, &props);
   vn_renderer_bo_unref(dev->renderer, bo);
   if (result != VK_SUCCESS) {
      vn_log(dev->instance, "vkGetMemoryResourcePropertiesMESA failed");
      return result;
   }

   *out_mem_type_bits = props.memoryTypeBits;

   return VK_SUCCESS;
#endif
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_GetMemoryFdPropertiesKHR(VkDevice device,
                            VkExternalMemoryHandleTypeFlagBits handleType,
                            int fd,
                            VkMemoryFdPropertiesKHR *pMemoryFdProperties)
{
   VN_TRACE_FUNC();
   struct vn_device *dev = vn_device_from_handle(device);
   uint32_t mem_type_bits = 0;
   VkResult result = VK_SUCCESS;

   if (handleType != VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT)
      return vn_error(dev->instance, VK_ERROR_INVALID_EXTERNAL_HANDLE);

   result = vn_get_memory_dma_buf_properties(dev, fd, &mem_type_bits);
   if (result != VK_SUCCESS)
      return vn_error(dev->instance, result);

   pMemoryFdProperties->memoryTypeBits = mem_type_bits;

   return VK_SUCCESS;
}
