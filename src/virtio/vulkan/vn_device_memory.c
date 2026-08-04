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

#include "venus-protocol/vn_protocol_driver_device_memory.h"
#include "venus-protocol/vn_protocol_driver_transport.h"
#include "vk_debug_utils.h"

#include "vn_android.h"
#include "vn_buffer.h"
#include "vn_device.h"
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

static bool
vn_device_memory_is_coherent_cached(struct vn_device *dev,
                                    struct vn_device_memory *mem)
{
   if (!mem->base_bo || !mem->base_bo->prefer_cached_map)
      return false;

   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   const VkMemoryPropertyFlags flags = mem_type->propertyFlags;

   return (flags & (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                    VK_MEMORY_PROPERTY_HOST_CACHED_BIT)) ==
          (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
           VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
           VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
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
   if (!mem->bo_ring_seqno_valid)
      return false;

   /* fine to false it here since renderer submission failure is fatal */
   mem->bo_ring_seqno_valid = false;

   /* no need to wait for ring if
    * - mem alloc is done upon bo map or export
    * - mem import is done upon bo destroy
    */
   return !vn_ring_get_seqno_status(dev->primary_ring, mem->bo_ring_seqno);
}

static inline VkResult
vn_device_memory_bo_init(struct vn_device *dev, struct vn_device_memory *mem)
{
   struct vn_renderer_submit_batch *batch = NULL;
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

   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   return vn_renderer_bo_create_from_device_memory(
      dev->renderer, batch, mem_vk->size, mem->base.id,
      mem_type->propertyFlags, mem_vk->export_handle_types, &mem->base_bo);
}

static inline void
vn_device_memory_bo_fini(struct vn_device *dev, struct vn_device_memory *mem)
{
   if (!mem->base_bo)
      return;

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

   vn_renderer_bo_release_resource(dev->renderer, mem->base_bo);
   vn_renderer_bo_unref(dev->renderer, mem->base_bo);
   mem->base_bo = NULL;
}

static VkResult
vn_device_memory_import_resource_id(struct vn_device *dev,
                                    struct vn_device_memory *mem,
                                    const VkMemoryAllocateInfo *alloc_info,
                                    const VkImportMemoryResourceInfoMESA *import_info)
{
   if (!import_info->resourceId)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   const struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];

   VkResult result = vn_renderer_bo_create_from_resource_id(
      dev->renderer, mem_vk->size, import_info->resourceId,
      mem_type->propertyFlags, &mem->base_bo);
   if (result != VK_SUCCESS)
      return result;

   const VkMemoryAllocateInfo memory_allocate_info = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = alloc_info->pNext,
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

   /* For external allocation handles, it's possible scenario when requested
    * non-mappable memory. To make sure that virtio-gpu driver will send to
    * the host the address of allocated blob using RESOURCE_MAP_BLOB command
    * a flag VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT must be set.
    */
   if (mem_vk->export_handle_types)
      flags |= VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

   VkResult result = vn_renderer_bo_create_from_device_memory(
      dev->renderer, NULL, mem_vk->size, mem->base.id, flags,
      mem_vk->export_handle_types, &mem->base_bo);
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
   struct vn_device_memory_alloc_info *local_info)
{
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

   cur->pNext = NULL;

   return &local_info->alloc;
}

static VkResult
vn_device_memory_alloc(struct vn_device *dev,
                       struct vn_device_memory *mem,
                       const VkMemoryAllocateInfo *alloc_info)
{
   struct vk_device_memory *mem_vk = &mem->base.vk;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];

   const bool has_guest_vram = dev->renderer->info.has_guest_vram;
   const bool host_visible =
      mem_type->propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
   const bool export_alloc = mem_vk->export_handle_types;

   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type =
      dev->physical_device->external_memory.renderer_handle_type;
   struct vn_device_memory_alloc_info local_info;
   /* Preserve the caller's explicit DMA_BUF export handle exactly, matching
    * the image-side policy. */
   const bool preserve_export =
      vn_preserve_explicit_dmabuf_handle_types(
         dev->physical_device, mem_vk->export_handle_types);
   if (!preserve_export &&
       mem_vk->export_handle_types &&
       mem_vk->export_handle_types != renderer_handle_type) {
      alloc_info = vn_device_memory_fix_alloc_info(
         alloc_info, renderer_handle_type, has_guest_vram, &local_info);

      /* ensure correct blob flags */
      mem_vk->export_handle_types = renderer_handle_type;
   }

   if (has_guest_vram && (host_visible || export_alloc)) {
      return vn_device_memory_alloc_guest_vram(dev, mem, alloc_info);
   } else if (export_alloc) {
      return vn_device_memory_alloc_export(dev, mem, alloc_info);
   } else {
      return vn_device_memory_alloc_simple(dev, mem, alloc_info);
   }
}

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
      (mem_vk->import_handle_type | mem_vk->export_handle_types)
         ? mem->base_bo->res_id
         : mem->base.id;
   const VkMemoryType *mem_type = &dev->physical_device->memory_properties
                                      .memoryTypes[mem_vk->memory_type_index];
   vk_emit_device_memory_report(dev_vk, type, mem_obj_id, mem_vk->size,
                                VK_OBJECT_TYPE_DEVICE_MEMORY, (uintptr_t)mem,
                                mem_type->heapIndex);
}

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

   const VkImportMemoryFdInfoKHR *import_fd_info =
      vk_find_struct_const(pAllocateInfo->pNext, IMPORT_MEMORY_FD_INFO_KHR);
   const VkImportMemoryResourceInfoMESA *import_resource_info =
      (const VkImportMemoryResourceInfoMESA *)__vk_find_struct(
         (void *)pAllocateInfo->pNext,
         VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA);

   VkResult result;
   if (mem->base.vk.ahardware_buffer) {
      result = vn_android_device_import_ahb(dev, mem, pAllocateInfo);
   } else if (import_resource_info) {
      result = vn_device_memory_import_resource_id(dev, mem, pAllocateInfo,
                                                   import_resource_info);
   } else if (import_fd_info) {
      result = vn_device_memory_import_dma_buf(dev, mem, pAllocateInfo,
                                               import_fd_info->fd);
   } else {
      result = vn_device_memory_alloc(dev, mem, pAllocateInfo);
      if (result == VK_SUCCESS)
         vn_wsi_memory_info_init(mem, pAllocateInfo);
   }

   vn_device_memory_emit_report(dev, mem, /* is_alloc */ true, result);

   if (result != VK_SUCCESS) {
      vk_device_memory_destroy(&dev->base.vk, pAllocator, &mem->base.vk);
      return vn_error(dev->instance, result);
   }

#ifdef _WIN32
   /* Imported allocations already have an owner and must not be charged as a
    * second physical allocation.  Ordinary and exportable allocations are
    * owned by this VkDeviceMemory and receive one matching VidMm lifetime. */
   if (!mem->base.vk.import_handle_type && !import_resource_info &&
       !import_fd_info && !mem->base.vk.ahardware_buffer) {
      const VkMemoryType *memory_type =
         &dev->physical_device->memory_properties
             .memoryTypes[mem->base.vk.memory_type_index];
      const VkMemoryHeap *memory_heap =
         &dev->physical_device->memory_properties
             .memoryHeaps[memory_type->heapIndex];
      const bool device_local =
         memory_heap->flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT;
      (void)vn_renderer_helios_vidmm_alloc(
         dev->renderer, mem->base.vk.size, device_local,
         &mem->helios_vidmm_resource, &mem->helios_vidmm_allocation,
         &mem->helios_vidmm_global_share, &mem->helios_vidmm_cookie);
   }
#endif

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

   /* ensure renderer side import still sees the resource */
   vn_device_memory_bo_fini(dev, mem);

   if (mem->bo_roundtrip_seqno_valid)
      vn_ring_wait_roundtrip(dev->primary_ring, mem->bo_roundtrip_seqno);

   vn_device_memory_free_simple(dev, mem);
#ifdef _WIN32
   if (mem->helios_vidmm_allocation)
      vn_renderer_helios_vidmm_free(dev->renderer,
                                    mem->helios_vidmm_resource,
                                    mem->helios_vidmm_allocation);
#endif
   vk_device_memory_destroy(&dev->base.vk, pAllocator, &mem->base.vk);
}

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

   mem->base_bo->prefer_cached_map = mem->wsi_buffer_blit_dst;

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

VKAPI_ATTR VkResult VKAPI_CALL
vn_FlushMappedMemoryRanges(VkDevice device,
                           uint32_t memoryRangeCount,
                           const VkMappedMemoryRange *pMemoryRanges)
{
   struct vn_device *dev = vn_device_from_handle(device);

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

   /* Helios: the renderer may NULL out bo_ops.export_dma_buf (no guest-side fd
    * export — the host exports the dmabuf from the res_id for SET_SCANOUT_BLOB).
    * Guard so a stray vkGetMemoryFdKHR fails cleanly instead of dereferencing a
    * NULL fn-ptr (vn_renderer_helios.c:3405). DXVK on Windows shares via Win32
    * NT handles, not fd export, so this should not fire in the scan-out path. */
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
