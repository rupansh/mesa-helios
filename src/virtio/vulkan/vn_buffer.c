/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#include "vn_buffer.h"

#include "venus-protocol/vn_protocol_driver_buffer.h"
#include "venus-protocol/vn_protocol_driver_buffer_view.h"

#include "vn_device.h"
#include "vn_device_memory.h"
#if DETECT_OS_WINDOWS
#include "vn_cs.h"
#include "vn_helios_hwa2.h"
#include "vn_helios_record_submit.h"
#endif
#include "vn_physical_device.h"
#include "util/stack_array.h"

/* buffer commands */

static VkBufferUsageFlags2
vn_buffer_effective_usage(const VkBufferCreateInfo *create_info)
{
   const VkBufferUsageFlags2CreateInfo *usage2 =
      vk_find_struct_const(create_info->pNext,
                           BUFFER_USAGE_FLAGS_2_CREATE_INFO);
   return usage2 ? usage2->usage : create_info->usage;
}

static uint64_t
vn_buffer_get_cache_index(const VkBufferCreateInfo *create_info,
                          struct vn_buffer_reqs_cache *cache)
{
   /* No need to cache for size exceeding the limit. */
   if (create_info->size > cache->max_buffer_size)
      return 0;

   /* Only 7 bits are taken for VkBufferCreateFlagBits as of spec 1.4.339. We
    * preserve 12 bits for the create flags.
    */
   if (create_info->flags & 0xFFFFF000)
      return 0;

   /* VK_SHARING_MODE_EXCLUSIVE or VK_SHARING_MODE_CONCURRENT across all */
   const bool is_exclusive =
      create_info->sharingMode == VK_SHARING_MODE_EXCLUSIVE;
   const bool is_concurrent =
      create_info->sharingMode == VK_SHARING_MODE_CONCURRENT &&
      create_info->queueFamilyIndexCount == cache->queue_family_count;
   if (!is_exclusive && !is_concurrent)
      return 0;

   /* Per spec:
    *
    * VkBufferCreateInfo:
    * If the pNext chain includes a VkBufferUsageFlags2CreateInfo structure,
    * VkBufferUsageFlags2CreateInfo::usage from that structure is used instead
    * of usage from this structure.
    *
    * VUID-VkBufferCreateInfo-None-09500
    * If the pNext chain does not include a VkBufferUsageFlags2CreateInfo
    * structure, usage must not be 0
    *
    * VUID-VkBufferUsageFlags2CreateInfo-usage-requiredbitmask
    * usage must not be 0
    */
   uint64_t usage = (uint64_t)create_info->usage;
   vk_foreach_struct_const(pnext, create_info->pNext) {
      switch (pnext->sType) {
      case VK_STRUCTURE_TYPE_BUFFER_USAGE_FLAGS_2_CREATE_INFO: {
         const VkBufferUsageFlags2CreateInfo *usage2 = (void *)pnext;
         usage = (uint64_t)usage2->usage;
         break;
      }
      default:
         /* Other pNext structs are not cacheable. */
         return 0;
      }
   }

   /* Only 34 bits are taken for VkBufferUsageFlagBits2 as of spec 1.4.339. We
    * preserve 51 bits for the usage flags.
    */
   if (usage & 0xFFF8000000000000ULL)
      return 0;

   /* Combine sharing mode, flags and usage bits to form a unique index:
    *
    * | 63: concurrent | 51 ~ 62: create flags | 0 ~ 50: usage |
    */
   return (uint64_t)is_concurrent << 63 | (uint64_t)create_info->flags << 51 |
          usage;
}

static inline uint64_t
vn_buffer_get_max_buffer_size(struct vn_physical_device *physical_dev)
{
   /* Without maintenance4, hardcode the min of supported drivers:
    * - anv:  1ull << 30
    * - radv: UINT32_MAX - 4
    * - tu:   UINT32_MAX + 1
    * - lvp:  UINT32_MAX
    * - mali: UINT32_MAX
    */
   static const uint64_t safe_max_buffer_size = 1ULL << 30;
   return physical_dev->base.vk.supported_features.maintenance4
             ? physical_dev->base.vk.properties.maxBufferSize
             : safe_max_buffer_size;
}

void
vn_buffer_reqs_cache_init(struct vn_device *dev)
{
   assert(dev->physical_device->queue_family_count);

   dev->buffer_reqs_cache.max_buffer_size =
      vn_buffer_get_max_buffer_size(dev->physical_device);
   dev->buffer_reqs_cache.queue_family_count =
      dev->physical_device->queue_family_count;

   simple_mtx_init(&dev->buffer_reqs_cache.mutex, mtx_plain);
   util_sparse_array_init(&dev->buffer_reqs_cache.entries,
                          sizeof(struct vn_buffer_reqs_cache_entry), 64);
}

static void
vn_buffer_reqs_cache_debug_dump(struct vn_buffer_reqs_cache *cache)
{
   vn_log(NULL, "dumping buffer cache statistics");
   vn_log(NULL, "  cache hit: %d", cache->debug.cache_hit_count);
   vn_log(NULL, "  cache miss: %d", cache->debug.cache_miss_count);
   vn_log(NULL, "  cache skip: %d", cache->debug.cache_skip_count);
}

void
vn_buffer_reqs_cache_fini(struct vn_device *dev)
{
   util_sparse_array_finish(&dev->buffer_reqs_cache.entries);
   simple_mtx_destroy(&dev->buffer_reqs_cache.mutex);

   if (VN_DEBUG(CACHE))
      vn_buffer_reqs_cache_debug_dump(&dev->buffer_reqs_cache);
}

static inline VkDeviceSize
vn_buffer_get_aligned_memory_requirement_size(VkDeviceSize size,
                                              const VkMemoryRequirements *req)
{
   /* TODO remove comment after mandating VK_KHR_maintenance4
    *
    * This is based on below implementation defined behavior:
    *    req.size <= align64(info.size, req.alignment)
    */
   return align64(size, req->alignment);
}

static struct vn_buffer_reqs_cache_entry *
vn_buffer_get_cached_memory_requirements(
   struct vn_buffer_reqs_cache *cache,
   const VkBufferCreateInfo *create_info,
   struct vn_buffer_memory_requirements *out)
{
   if (VN_PERF(NO_ASYNC_BUFFER_CREATE))
      return NULL;

   /* 12.7. Resource Memory Association
    *
    * The memoryTypeBits member is identical for all VkBuffer objects created
    * with the same value for the flags and usage members in the
    * VkBufferCreateInfo structure and the handleTypes member of the
    * VkExternalMemoryBufferCreateInfo structure passed to vkCreateBuffer.
    */
   const uint64_t idx = vn_buffer_get_cache_index(create_info, cache);
   if (idx) {
      struct vn_buffer_reqs_cache_entry *entry =
         util_sparse_array_get(&cache->entries, idx);

      if (entry->valid) {
         *out = entry->requirements;

         out->memory.memoryRequirements.size =
            vn_buffer_get_aligned_memory_requirement_size(
               create_info->size, &out->memory.memoryRequirements);

         p_atomic_inc(&cache->debug.cache_hit_count);
      } else {
         p_atomic_inc(&cache->debug.cache_miss_count);
      }

      return entry;
   }

   p_atomic_inc(&cache->debug.cache_skip_count);

   return NULL;
}

static void
vn_buffer_reqs_cache_entry_init(struct vn_buffer_reqs_cache *cache,
                                struct vn_buffer_reqs_cache_entry *entry,
                                VkMemoryRequirements2 *req)
{
   simple_mtx_lock(&cache->mutex);

   /* Entry might have already been initialized by another thread
    * before the lock
    */
   if (entry->valid)
      goto unlock;

   entry->requirements.memory = *req;

   const VkMemoryDedicatedRequirements *dedicated_req =
      vk_find_struct_const(req->pNext, MEMORY_DEDICATED_REQUIREMENTS);
   if (dedicated_req)
      entry->requirements.dedicated = *dedicated_req;

   entry->valid = true;

unlock:
   simple_mtx_unlock(&cache->mutex);

   /* ensure invariance of the memory requirement size */
   req->memoryRequirements.size =
      vn_buffer_get_aligned_memory_requirement_size(
         req->memoryRequirements.size,
         &entry->requirements.memory.memoryRequirements);
}

static void
vn_copy_cached_memory_requirements(
   const struct vn_buffer_memory_requirements *cached,
   VkMemoryRequirements2 *out_mem_req)
{
   union {
      VkBaseOutStructure *pnext;
      VkMemoryRequirements2 *two;
      VkMemoryDedicatedRequirements *dedicated;
   } u = { .two = out_mem_req };

   while (u.pnext) {
      switch (u.pnext->sType) {
      case VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2:
         u.two->memoryRequirements = cached->memory.memoryRequirements;
         break;
      case VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS:
         u.dedicated->prefersDedicatedAllocation =
            cached->dedicated.prefersDedicatedAllocation;
         u.dedicated->requiresDedicatedAllocation =
            cached->dedicated.requiresDedicatedAllocation;
         break;
      default:
         break;
      }
      u.pnext = u.pnext->pNext;
   }
}

static VkResult
vn_buffer_init(struct vn_device *dev,
               const VkBufferCreateInfo *create_info,
               struct vn_buffer *buf)
{
   VkDevice dev_handle = vn_device_to_handle(dev);
   VkBuffer buf_handle = vn_buffer_to_handle(buf);
   struct vn_buffer_reqs_cache *cache = &dev->buffer_reqs_cache;
   VkResult result;

   /* If cacheable and mem requirements found in cache, make async call */
   struct vn_buffer_reqs_cache_entry *entry =
      vn_buffer_get_cached_memory_requirements(cache, create_info,
                                               &buf->requirements);

   /* Check size instead of entry->valid to be lock free */
   if (buf->requirements.memory.memoryRequirements.size) {
      vn_async_vkCreateBuffer(dev->primary_ring, dev_handle, create_info,
                              NULL, &buf_handle);
      return VK_SUCCESS;
   }

   /* The record-only Windows transport gives every replyless control command
    * a real C51 terminal before this call returns.  Do not request the
    * redundant output-handle reply for vkCreateBuffer there: the guest already
    * assigned buf_handle, and the K11 private reply path otherwise observes no
    * matching opcode for this first uncached create.  The immediately
    * following synchronous requirements query names that exact handle, so it
    * both proves host object creation and supplies the uncached result.  Normal
    * loader instances retain the stock synchronous create path.
    */
#if DETECT_OS_WINDOWS
   const bool record_only =
      vn_helios_submit_instance_mode(dev->instance) ==
      VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
   if (record_only) {
      vn_async_vkCreateBuffer(dev->primary_ring, dev_handle, create_info,
                              NULL, &buf_handle);
   } else
#endif
   {
      result = vn_call_vkCreateBuffer(dev->primary_ring, dev_handle,
                                      create_info, NULL, &buf_handle);
      if (result != VK_SUCCESS)
         return result;
   }

   buf->requirements.memory.sType = VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2;
   buf->requirements.memory.pNext = &buf->requirements.dedicated;
   buf->requirements.dedicated.sType =
      VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS;
   buf->requirements.dedicated.pNext = NULL;

   vn_call_vkGetBufferMemoryRequirements2(
      dev->primary_ring, dev_handle,
      &(VkBufferMemoryRequirementsInfo2){
         .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_REQUIREMENTS_INFO_2,
         .buffer = buf_handle,
      },
      &buf->requirements.memory);
#if DETECT_OS_WINDOWS
   if (record_only &&
       (!buf->requirements.memory.memoryRequirements.size ||
        !buf->requirements.memory.memoryRequirements.alignment ||
        !buf->requirements.memory.memoryRequirements.memoryTypeBits))
      return VK_ERROR_DEVICE_LOST;
#endif
   vn_physical_device_sanitize_memory_requirements(
      dev->physical_device,
      &buf->requirements.memory.memoryRequirements);

   /* If cacheable, store mem requirements from the synchronous call */
   if (entry) {
      vn_buffer_reqs_cache_entry_init(cache, entry,
                                      &buf->requirements.memory);
   }

   return VK_SUCCESS;
}

VkResult
vn_buffer_create(struct vn_device *dev,
                 const VkBufferCreateInfo *create_info,
                 const VkAllocationCallbacks *alloc,
                 struct vn_buffer **out_buf)
{
   struct vn_buffer *buf = NULL;
   VkResult result;

   buf = vk_zalloc(alloc, sizeof(*buf), VN_DEFAULT_ALIGN,
                   VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
   if (!buf)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   vn_object_base_init(&buf->base, VK_OBJECT_TYPE_BUFFER, &dev->base);
#if DETECT_OS_WINDOWS
   list_inithead(&buf->helios_address_link);
#endif

   result = vn_buffer_init(dev, create_info, buf);
   if (result != VK_SUCCESS) {
      vn_object_base_fini(&buf->base);
      vk_free(alloc, buf);
      return result;
   }

   *out_buf = buf;

   return VK_SUCCESS;
}

#if DETECT_OS_WINDOWS
/* The installed renderer does not implement the maintenance4 opcode used by
 * vkGetDeviceBufferMemoryRequirements, while its ordinary create/query path
 * is complete.  Keep the fallback spec-valid and A7-complete by placing the
 * private buffer's whole lifetime in one generated control transaction:
 *
 *   CreateBuffer -> GetBufferMemoryRequirements2 -> DestroyBuffer
 *
 * KMD admission accepts this sole allocation-class command only after the
 * generated schema proves all three commands name the same nonzero device and
 * buffer, with no intervening bind or trailing command.  The buffer is thus
 * provably unbound and dead before this function returns; no child object is
 * left for vkDestroyDevice and no outer allocation identity is fabricated.
 */
static VkResult
vn_buffer_query_unbound_requirements_record_only(
   struct vn_device *dev,
   const VkBufferCreateInfo *create_info,
   struct vn_buffer_memory_requirements *out_reqs)
{
   const VkAllocationCallbacks *alloc = &dev->base.vk.alloc;
   struct vn_buffer *probe =
      vk_zalloc(alloc, sizeof(*probe), VN_DEFAULT_ALIGN,
                VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
   if (!probe)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   vn_object_base_init(&probe->base, VK_OBJECT_TYPE_BUFFER, &dev->base);
   VkDevice device = vn_device_to_handle(dev);
   VkBuffer buffer = vn_buffer_to_handle(probe);
   VkBufferMemoryRequirementsInfo2 info = {
      .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_REQUIREMENTS_INFO_2,
      .buffer = buffer,
   };
   struct vn_buffer_memory_requirements reqs = {
      .memory = {
         .sType = VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2,
      },
      .dedicated = {
         .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS,
      },
   };
   reqs.memory.pNext = &reqs.dedicated;

   const size_t create_bytes =
      vn_sizeof_vkCreateBuffer(device, create_info, NULL, &buffer);
   const size_t query_bytes =
      vn_sizeof_vkGetBufferMemoryRequirements2(device, &info,
                                               &reqs.memory);
   const size_t destroy_bytes =
      vn_sizeof_vkDestroyBuffer(device, buffer, NULL);
   const size_t reply_bytes =
      vn_sizeof_vkGetBufferMemoryRequirements2_reply(device, &info,
                                                     &reqs.memory);
   VkResult result = VK_ERROR_OUT_OF_HOST_MEMORY;
   if (!create_bytes || !query_bytes || !destroy_bytes || !reply_bytes ||
       create_bytes > SIZE_MAX - query_bytes ||
       create_bytes + query_bytes > SIZE_MAX - destroy_bytes)
      goto out_probe;

   const size_t command_bytes =
      create_bytes + query_bytes + destroy_bytes;
   uint8_t *command = malloc(command_bytes);
   if (!command)
      goto out_probe;

   struct vn_ring_submit_command submit;
   struct vn_cs_encoder *encoder = vn_ring_submit_command_init(
      dev->primary_ring, &submit, command, command_bytes, reply_bytes);
   vn_encode_vkCreateBuffer(encoder, 0, device, create_info, NULL, &buffer);
   vn_encode_vkGetBufferMemoryRequirements2(
      encoder, VK_COMMAND_GENERATE_REPLY_BIT_EXT, device, &info,
      &reqs.memory);
   vn_encode_vkDestroyBuffer(encoder, 0, device, buffer, NULL);
   if (vn_cs_encoder_get_fatal(encoder) ||
       vn_cs_encoder_get_len(encoder) != command_bytes) {
      result = VK_ERROR_INITIALIZATION_FAILED;
      goto out_command;
   }

   vn_ring_submit_command(dev->primary_ring, &submit);
   struct vn_cs_decoder *decoder =
      vn_ring_get_command_reply(dev->primary_ring, &submit);
   if (!decoder) {
      result = VK_ERROR_DEVICE_LOST;
      goto out_command;
   }
   vn_decode_vkGetBufferMemoryRequirements2_reply(
      decoder, device, &info, &reqs.memory);
   const bool complete_reply = decoder->cur == decoder->end;
   vn_ring_free_command_reply(dev->primary_ring, &submit);
   if (!complete_reply ||
       !reqs.memory.memoryRequirements.size ||
       !reqs.memory.memoryRequirements.alignment ||
       !reqs.memory.memoryRequirements.memoryTypeBits) {
      result = VK_ERROR_DEVICE_LOST;
      goto out_command;
   }

   vn_physical_device_sanitize_memory_requirements(
      dev->physical_device, &reqs.memory.memoryRequirements);
   if (!reqs.memory.memoryRequirements.memoryTypeBits) {
      result = VK_ERROR_FEATURE_NOT_PRESENT;
      goto out_command;
   }
   *out_reqs = reqs;
   out_reqs->memory.pNext = &out_reqs->dedicated;
   result = VK_SUCCESS;

out_command:
   free(command);
out_probe:
   vn_object_base_fini(&probe->base);
   vk_free(alloc, probe);
   return result;
}
#endif

struct vn_buffer_create_info {
   VkBufferCreateInfo create;
   VkExternalMemoryBufferCreateInfo external;
   VkBufferOpaqueCaptureAddressCreateInfo capture;
   VkBufferDeviceAddressCreateInfoEXT address;
   VkBufferUsageFlags2CreateInfo usage2;
};

static const VkBufferCreateInfo *
vn_buffer_fix_create_info(
   const VkBufferCreateInfo *create_info,
   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type,
   bool force_capture_replay,
   struct vn_buffer_create_info *local_info)
{
   bool has_external = false;
   local_info->create = *create_info;
   if (force_capture_replay)
      local_info->create.flags |=
         VK_BUFFER_CREATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT;
   VkBaseOutStructure *cur = (void *)&local_info->create;

   vk_foreach_struct_const(src, create_info->pNext) {
      void *next = NULL;
      switch (src->sType) {
      case VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO:
         memcpy(&local_info->external, src, sizeof(local_info->external));
         local_info->external.handleTypes = renderer_handle_type;
         has_external = true;
         next = &local_info->external;
         break;
      case VK_STRUCTURE_TYPE_BUFFER_OPAQUE_CAPTURE_ADDRESS_CREATE_INFO:
         memcpy(&local_info->capture, src, sizeof(local_info->capture));
         next = &local_info->capture;
         break;
      case VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_CREATE_INFO_EXT:
         memcpy(&local_info->address, src, sizeof(local_info->address));
         next = &local_info->address;
         break;
      case VK_STRUCTURE_TYPE_BUFFER_USAGE_FLAGS_2_CREATE_INFO:
         memcpy(&local_info->usage2, src, sizeof(local_info->usage2));
         next = &local_info->usage2;
         break;
      default:
         break;
      }

      if (next) {
         cur->pNext = next;
         cur = next;
      }
   }

   /* Helios: vkr force-exports every HOST_VISIBLE allocation (with the
    * renderer handle type), so a buffer that may bind such memory must carry
    * matching handleTypes or the bind violates
    * VUID-VkBindBufferMemoryInfo-memory-02726 (UB; observed faulting the
    * GPU on the NVIDIA proprietary driver while ANV tolerates it). Inject
    * the renderer handle type when the app provided no external info. */
   if (renderer_handle_type && !has_external) {
      local_info->external = (VkExternalMemoryBufferCreateInfo){
         .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO,
         .handleTypes = renderer_handle_type,
      };
      cur->pNext = (void *)&local_info->external;
      cur = (void *)&local_info->external;
   }

   cur->pNext = NULL;

   return &local_info->create;
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_CreateBuffer(VkDevice device,
                const VkBufferCreateInfo *pCreateInfo,
                const VkAllocationCallbacks *pAllocator,
                VkBuffer *pBuffer)
{
   struct vn_device *dev = vn_device_from_handle(device);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;
   struct vn_buffer_create_info local_info;
   const VkExternalMemoryBufferCreateInfo *external_info =
      vk_find_struct_const(pCreateInfo->pNext,
                           EXTERNAL_MEMORY_BUFFER_CREATE_INFO);
   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type =
      vn_renderer_handle_type_for_guest(
         dev->physical_device,
         external_info ? external_info->handleTypes : 0);
   const bool helios_capture_replay =
      vn_helios_submit_instance_mode(dev->instance) ==
         VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY &&
      dev->base.vk.enabled_extensions.EXT_buffer_device_address &&
      dev->base.vk.enabled_features.bufferDeviceAddress &&
      /* The EXT struct's captureReplay maps to the RENAMED unified bit
       * (vk_physical_device_features_gen.py RENAMED_FEATURES); the core
       * bufferDeviceAddressCaptureReplay stays false by design in
       * record-only DXVK, so checking it zeroed EVERY device address
       * (2026-08-24: Xid-31 VA-0 GPU faults on every content submit). */
      dev->base.vk.enabled_features.bufferDeviceAddressCaptureReplayEXT &&
      (vn_buffer_effective_usage(pCreateInfo) &
       VK_BUFFER_USAGE_2_SHADER_DEVICE_ADDRESS_BIT);
   if (helios_capture_replay ||
       (renderer_handle_type &&
        (!external_info || !external_info->handleTypes ||
         external_info->handleTypes != renderer_handle_type))) {
      pCreateInfo = vn_buffer_fix_create_info(
         pCreateInfo, renderer_handle_type, helios_capture_replay,
         &local_info);
   }

   struct vn_buffer *buf;
   VkResult result = vn_buffer_create(dev, pCreateInfo, alloc, &buf);
   if (result != VK_SUCCESS)
      return vn_error(dev->instance, result);

#if DETECT_OS_WINDOWS
   buf->helios_capture_replay = helios_capture_replay;
#endif

   *pBuffer = vn_buffer_to_handle(buf);

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_DestroyBuffer(VkDevice device,
                 VkBuffer buffer,
                 const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_buffer *buf = vn_buffer_from_handle(buffer);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   if (!buf)
      return;

#if DETECT_OS_WINDOWS
   if (buf->helios_address_registered) {
      simple_mtx_lock(&dev->mutex);
      if (buf->helios_address_registered) {
         list_delinit(&buf->helios_address_link);
         assert(dev->helios_address_buffer_count > 0);
         dev->helios_address_buffer_count--;
         buf->helios_address_registered = false;
      }
      simple_mtx_unlock(&dev->mutex);
   }
#endif

#if DETECT_OS_WINDOWS
   const bool record_only =
      vn_helios_submit_instance_mode(dev->instance) ==
      VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
   if (record_only && buf->helios_binding.valid) {
      struct vn_device_memory *mem =
         vn_device_memory_helios_binding_memory(
            dev, &buf->helios_binding);
      const size_t payload_bytes =
         vn_sizeof_vkDestroyBuffer(device, buffer, NULL);
      uint8_t *payload = payload_bytes ? malloc(payload_bytes) : NULL;
      VkResult defer_result = VK_SUCCESS;
      if (!mem || !payload || payload_bytes > HELIOS_HOB1_MAX_BYTES) {
         free(payload);
         defer_result = mem ? VK_ERROR_OUT_OF_HOST_MEMORY
                            : VK_ERROR_VALIDATION_FAILED_EXT;
      } else {
         struct vn_cs_encoder encoder =
            VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_bytes);
         vn_encode_vkDestroyBuffer(&encoder, 0, device, buffer, NULL);
         if (vn_cs_encoder_get_fatal(&encoder) ||
             vn_cs_encoder_get_len(&encoder) != payload_bytes) {
            free(payload);
            defer_result = VK_ERROR_INITIALIZATION_FAILED;
         } else {
            struct vn_helios_deferred_record *record =
               vn_device_memory_helios_record_create(
                  mem, payload, payload_bytes,
                  VN_HELIOS_NO_RESOURCE_OPERAND, false, false);
            if (!record) {
               defer_result = VK_ERROR_OUT_OF_HOST_MEMORY;
            } else {
               defer_result = vn_device_memory_helios_record_install(
                  dev, mem, record);
               if (defer_result != VK_SUCCESS)
                  vn_device_memory_helios_record_destroy(record);
            }
         }
      }
      if (defer_result != VK_SUCCESS) {
         vn_helios_record_note_deferred_use(dev->instance);
         p_atomic_set(&dev->helios_lost, 1);
         (void)vn_error(dev->instance, defer_result);
      }
   } else
#endif
   {
      vn_async_vkDestroyBuffer(dev->primary_ring, device, buffer, NULL);
   }

   vn_object_base_fini(&buf->base);
   vk_free(alloc, buf);
}

VKAPI_ATTR VkDeviceAddress VKAPI_CALL
vn_GetBufferDeviceAddress(VkDevice device,
                          const VkBufferDeviceAddressInfo *pInfo)
{
   struct vn_device *dev = vn_device_from_handle(device);
#if DETECT_OS_WINDOWS
   struct vn_buffer *buf = vn_buffer_from_handle(pInfo->buffer);
   if (vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY &&
       (!buf || buf->base.vk.device != &dev->base.vk ||
        !buf->helios_capture_replay))
      return 0;
#endif
   VkDeviceAddress address =
      vn_call_vkGetBufferDeviceAddress(dev->primary_ring, device, pInfo);
#if DETECT_OS_WINDOWS
   if (address && buf && buf->base.vk.device == &dev->base.vk) {
      simple_mtx_lock(&dev->mutex);
      if (buf->helios_device_address &&
          buf->helios_device_address != address) {
         address = 0;
      } else {
         buf->helios_device_address = address;
         if (!buf->helios_address_registered &&
             dev->helios_address_buffer_count <
                HELIOS_HOB1_MAX_USE_RECORDS) {
            list_addtail(&buf->helios_address_link,
                         &dev->helios_address_buffers);
            dev->helios_address_buffer_count++;
            buf->helios_address_registered = true;
         } else if (!buf->helios_address_registered) {
            address = 0;
         }
      }
      simple_mtx_unlock(&dev->mutex);
   }
#endif
   return address;
}

VKAPI_ATTR uint64_t VKAPI_CALL
vn_GetBufferOpaqueCaptureAddress(VkDevice device,
                                 const VkBufferDeviceAddressInfo *pInfo)
{
   struct vn_device *dev = vn_device_from_handle(device);

   return vn_call_vkGetBufferOpaqueCaptureAddress(dev->primary_ring, device,
                                                  pInfo);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetBufferMemoryRequirements2(VkDevice device,
                                const VkBufferMemoryRequirementsInfo2 *pInfo,
                                VkMemoryRequirements2 *pMemoryRequirements)
{
   const struct vn_buffer *buf = vn_buffer_from_handle(pInfo->buffer);

   vn_copy_cached_memory_requirements(&buf->requirements,
                                      pMemoryRequirements);
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_BindBufferMemory2(VkDevice device,
                     uint32_t bindInfoCount,
                     const VkBindBufferMemoryInfo *pBindInfos)
{
   struct vn_device *dev = vn_device_from_handle(device);
   STACK_ARRAY(VkBindBufferMemoryInfo, local_infos, bindInfoCount);
   if (!local_infos)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

#if DETECT_OS_WINDOWS
   STACK_ARRAY(struct vn_helios_memory_binding, helios_bindings,
               bindInfoCount);
   STACK_ARRAY(struct vn_helios_deferred_record *, helios_records,
               bindInfoCount);
   if (!helios_bindings || !helios_records) {
      STACK_ARRAY_FINISH(helios_records);
      STACK_ARRAY_FINISH(helios_bindings);
      STACK_ARRAY_FINISH(local_infos);
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);
   }
   memset(helios_records, 0, bindInfoCount * sizeof(*helios_records));
   const bool record_only =
      vn_helios_submit_instance_mode(dev->instance) ==
      VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
   if (record_only) {
      for (uint32_t i = 0; i < bindInfoCount; i++) {
         struct vn_buffer *buf =
            vn_buffer_from_handle(pBindInfos[i].buffer);
         struct vn_device_memory *mem =
            vn_device_memory_from_handle(pBindInfos[i].memory);
         if (!buf || buf->base.vk.device != &dev->base.vk ||
             buf->helios_binding.valid ||
             vn_device_memory_helios_bind(
                dev, mem, pBindInfos[i].memoryOffset,
                buf->requirements.memory.memoryRequirements.size,
                &helios_bindings[i]) != VK_SUCCESS) {
            STACK_ARRAY_FINISH(helios_records);
            STACK_ARRAY_FINISH(helios_bindings);
            STACK_ARRAY_FINISH(local_infos);
            return vn_error(dev->instance,
                            VK_ERROR_VALIDATION_FAILED_EXT);
         }
      }
   }
#endif

   typed_memcpy(local_infos, pBindInfos, bindInfoCount);
   for (uint32_t i = 0; i < bindInfoCount; i++)
      local_infos[i].pNext = NULL;

#if DETECT_OS_WINDOWS
   if (record_only) {
      VkResult defer_result = VK_SUCCESS;
      for (uint32_t i = 0; i < bindInfoCount; i++) {
         struct vn_device_memory *mem =
            vn_device_memory_from_handle(local_infos[i].memory);
         const size_t payload_bytes =
            vn_sizeof_vkBindBufferMemory2(device, 1, &local_infos[i]);
         uint8_t *payload = payload_bytes ? malloc(payload_bytes) : NULL;
         if (!payload || payload_bytes > HELIOS_HOB1_MAX_BYTES) {
            free(payload);
            defer_result = VK_ERROR_OUT_OF_HOST_MEMORY;
            break;
         }
         struct vn_cs_encoder encoder =
            VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_bytes);
         vn_encode_vkBindBufferMemory2(&encoder, 0, device, 1,
                                       &local_infos[i]);
         if (vn_cs_encoder_get_fatal(&encoder) ||
             vn_cs_encoder_get_len(&encoder) != payload_bytes) {
            free(payload);
            defer_result = VK_ERROR_INITIALIZATION_FAILED;
            break;
         }
         helios_records[i] = vn_device_memory_helios_record_create(
            mem, payload, payload_bytes, VN_HELIOS_NO_RESOURCE_OPERAND,
            false, false);
         if (!helios_records[i]) {
            defer_result = VK_ERROR_OUT_OF_HOST_MEMORY;
            break;
         }
      }
      if (defer_result == VK_SUCCESS)
         defer_result = vn_device_memory_helios_records_install(
            dev, helios_records, bindInfoCount);
      if (defer_result != VK_SUCCESS) {
         for (uint32_t i = 0; i < bindInfoCount; i++)
            vn_device_memory_helios_record_destroy(helios_records[i]);
         STACK_ARRAY_FINISH(helios_records);
         STACK_ARRAY_FINISH(helios_bindings);
         STACK_ARRAY_FINISH(local_infos);
         return vn_error(dev->instance, defer_result);
      }
      for (uint32_t i = 0; i < bindInfoCount; i++) {
         struct vn_buffer *buf =
            vn_buffer_from_handle(pBindInfos[i].buffer);
         buf->helios_binding = helios_bindings[i];
      }
   } else {
      vn_async_vkBindBufferMemory2(dev->primary_ring, device, bindInfoCount,
                                   local_infos);
   }
   STACK_ARRAY_FINISH(helios_records);
   STACK_ARRAY_FINISH(helios_bindings);
#else
   vn_async_vkBindBufferMemory2(dev->primary_ring, device, bindInfoCount,
                                local_infos);
#endif

   STACK_ARRAY_FINISH(local_infos);

   for (uint32_t i = 0; i < bindInfoCount; i++) {
      const VkBindMemoryStatus *bind_status =
         vk_find_struct((void *)pBindInfos[i].pNext, BIND_MEMORY_STATUS);
      if (bind_status)
         *bind_status->pResult = VK_SUCCESS;
   }

   return VK_SUCCESS;
}

/* buffer view commands */

VKAPI_ATTR VkResult VKAPI_CALL
vn_CreateBufferView(VkDevice device,
                    const VkBufferViewCreateInfo *pCreateInfo,
                    const VkAllocationCallbacks *pAllocator,
                    VkBufferView *pView)
{
   struct vn_device *dev = vn_device_from_handle(device);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   struct vn_buffer_view *view =
      vk_zalloc(alloc, sizeof(*view), VN_DEFAULT_ALIGN,
                VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
   if (!view)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

   vn_object_base_init(&view->base, VK_OBJECT_TYPE_BUFFER_VIEW, &dev->base);
#if DETECT_OS_WINDOWS
   view->helios_buffer = vn_buffer_from_handle(pCreateInfo->buffer);
#endif

   VkBufferView view_handle = vn_buffer_view_to_handle(view);

#if DETECT_OS_WINDOWS
   /* A7 object-materialization lane: see vn_CreateImageView. */
   if (vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY &&
       view->helios_buffer && view->helios_buffer->helios_binding.valid) {
      VkResult defer_result = VK_ERROR_OUT_OF_HOST_MEMORY;
      const size_t generated =
         vn_sizeof_vkCreateBufferView(device, pCreateInfo, NULL,
                                      &view_handle);
      if (generated && generated <= HELIOS_HOB1_MAX_BYTES - 256) {
         const size_t capacity = generated + 256;
         uint8_t *buf = malloc(capacity);
         if (buf) {
            struct vn_cs_encoder enc =
               VN_CS_ENCODER_INITIALIZER_LOCAL(buf, capacity);
            vn_encode_vkCreateBufferView(&enc, 0, device, pCreateInfo, NULL,
                                         &view_handle);
            const size_t len = vn_cs_encoder_get_len(&enc);
            if (!vn_cs_encoder_get_fatal(&enc) && len && len <= capacity) {
               defer_result = vn_helios_record_defer_object_command(
                  dev, buf, len, &view->helios_buffer->helios_binding, 1,
                  NULL, 0);
               if (defer_result == VK_SUCCESS)
                  buf = NULL;
            } else {
               defer_result = VK_ERROR_INITIALIZATION_FAILED;
            }
            free(buf);
         }
      }
      if (defer_result != VK_SUCCESS) {
         vn_object_base_fini(&view->base);
         vk_free(alloc, view);
         return vn_error(dev->instance, defer_result);
      }
      *pView = view_handle;
      return VK_SUCCESS;
   }
#endif

   vn_async_vkCreateBufferView(dev->primary_ring, device, pCreateInfo, NULL,
                               &view_handle);

   *pView = view_handle;

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_DestroyBufferView(VkDevice device,
                     VkBufferView bufferView,
                     const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_buffer_view *view = vn_buffer_view_from_handle(bufferView);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   if (!view)
      return;

#if DETECT_OS_WINDOWS
   /* Trail the create on the object-materialization lane; see
    * vn_DestroyImageView. */
   bool destroyed = false;
   if (vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY &&
       view->helios_buffer && view->helios_buffer->helios_binding.valid) {
      const size_t generated =
         vn_sizeof_vkDestroyBufferView(device, bufferView, NULL);
      if (generated && generated <= HELIOS_HOB1_MAX_BYTES - 256) {
         const size_t capacity = generated + 256;
         uint8_t *buf = malloc(capacity);
         if (buf) {
            struct vn_cs_encoder enc =
               VN_CS_ENCODER_INITIALIZER_LOCAL(buf, capacity);
            vn_encode_vkDestroyBufferView(&enc, 0, device, bufferView, NULL);
            const size_t len = vn_cs_encoder_get_len(&enc);
            if (!vn_cs_encoder_get_fatal(&enc) && len && len <= capacity &&
                vn_helios_record_defer_object_command(
                   dev, buf, len, &view->helios_buffer->helios_binding,
                   1, NULL, 0) == VK_SUCCESS) {
               destroyed = true;
            } else {
               free(buf);
            }
         }
      }
      if (!destroyed) {
         vn_renderer_helios_diag_log(
            "HOC1 buffer-view destroy defer failed view=%p", (void *)view);
         destroyed = true;
      }
   }
   if (!destroyed)
      vn_async_vkDestroyBufferView(dev->primary_ring, device, bufferView,
                                   NULL);
#else
   vn_async_vkDestroyBufferView(dev->primary_ring, device, bufferView, NULL);
#endif

   vn_object_base_fini(&view->base);
   vk_free(alloc, view);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetDeviceBufferMemoryRequirements(
   VkDevice device,
   const VkDeviceBufferMemoryRequirements *pInfo,
   VkMemoryRequirements2 *pMemoryRequirements)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_buffer_reqs_cache *cache = &dev->buffer_reqs_cache;
   struct vn_buffer_memory_requirements reqs = { 0 };

   /* Helios: vn_CreateBuffer injects the renderer external handle type into
    * every buffer create (vn_buffer_fix_create_info — required because vkr
    * force-exports HOST_VISIBLE memory, VUID-02726). The requirements
    * reported HERE must describe the same fixed-up create info, or callers
    * compute memory-type masks the real (external) buffer cannot satisfy:
    * DXVK sized its global per-chunk buffers off this query's wide mask,
    * placed buffer chunks in memory types the external buffer cannot bind,
    * and every such chunk's global-buffer creation failed — leaving
    * buffer-less allocations whose CPU writes no VkBuffer ever aliases
    * (dwm's all-zero composition: dynamic vertex/constant data lost). */
   const VkExternalMemoryBufferCreateInfo *external_info =
      vk_find_struct_const(pInfo->pCreateInfo->pNext,
                           EXTERNAL_MEMORY_BUFFER_CREATE_INFO);
   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type =
      vn_renderer_handle_type_for_guest(
         dev->physical_device,
         external_info ? external_info->handleTypes : 0);
   const bool record_only =
      vn_helios_submit_instance_mode(dev->instance) ==
      VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
   const bool helios_capture_replay =
      record_only &&
      dev->base.vk.enabled_extensions.EXT_buffer_device_address &&
      dev->base.vk.enabled_features.bufferDeviceAddress &&
      /* The EXT struct's captureReplay maps to the RENAMED unified bit
       * (vk_physical_device_features_gen.py RENAMED_FEATURES); the core
       * bufferDeviceAddressCaptureReplay stays false by design in
       * record-only DXVK, so checking it zeroed EVERY device address
       * (2026-08-24: Xid-31 VA-0 GPU faults on every content submit). */
      dev->base.vk.enabled_features.bufferDeviceAddressCaptureReplayEXT &&
      (vn_buffer_effective_usage(pInfo->pCreateInfo) &
       VK_BUFFER_USAGE_2_SHADER_DEVICE_ADDRESS_BIT);
   struct vn_buffer_create_info local_info;
   VkDeviceBufferMemoryRequirements fixed_info;
   if (helios_capture_replay ||
       (renderer_handle_type &&
        (!external_info || !external_info->handleTypes ||
         external_info->handleTypes != renderer_handle_type))) {
      fixed_info = *pInfo;
      fixed_info.pCreateInfo = vn_buffer_fix_create_info(
         pInfo->pCreateInfo, renderer_handle_type, helios_capture_replay,
         &local_info);
      pInfo = &fixed_info;
   }

   /* If cacheable and mem requirements found in cache, skip host call */
   struct vn_buffer_reqs_cache_entry *entry =
      vn_buffer_get_cached_memory_requirements(cache, pInfo->pCreateInfo,
                                               &reqs);

   /* Check size instead of entry->valid to be lock free */
   if (reqs.memory.memoryRequirements.size) {
      vn_copy_cached_memory_requirements(&reqs, pMemoryRequirements);
      return;
   }

#if DETECT_OS_WINDOWS
   if (record_only) {
      /* The installed virglrenderer 1.3 host completes the device bootstrap
       * through vkGetDeviceQueue2, but leaves the immediately following
       * generated vkGetDeviceBufferMemoryRequirements reply target untouched.
       * Do not let that one non-replying generated command kill the directly
       * owned K11 session.  Vulkan defines this query to return the memory
       * requirements a buffer created from the same VkBufferCreateInfo would
       * have, so use the exact self-contained unbound query transaction above.
       * Its create info is already the same fixed-up external/capture-replay
       * chain used by vn_CreateBuffer; no requirement is inferred.
       */
      const VkResult result =
         vn_buffer_query_unbound_requirements_record_only(
            dev, pInfo->pCreateInfo, &reqs);
      if (result == VK_SUCCESS) {
         if (entry)
            vn_buffer_reqs_cache_entry_init(cache, entry, &reqs.memory);
         vn_copy_cached_memory_requirements(&reqs, pMemoryRequirements);
         return;
      }

      /* This entry point has no VkResult channel.  Leave an unmistakably
       * unusable requirement instead of allowing the caller to consume a
       * partially initialized host reply. */
      pMemoryRequirements->memoryRequirements = (VkMemoryRequirements){ 0 };
      (void)vn_error(dev->instance, result);
      return;
   }
#endif

   /* Make the host call if not found in cache or not cacheable */
   vn_call_vkGetDeviceBufferMemoryRequirements(dev->primary_ring, device,
                                               pInfo, pMemoryRequirements);
   vn_physical_device_sanitize_memory_requirements(
      dev->physical_device, &pMemoryRequirements->memoryRequirements);

   /* If cacheable, store mem requirements from the host call */
   if (entry)
      vn_buffer_reqs_cache_entry_init(cache, entry, pMemoryRequirements);
}
