/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#include "vn_image.h"

#include "venus-protocol/vn_protocol_driver_image.h"
#include "venus-protocol/vn_protocol_driver_image_view.h"
#include "venus-protocol/vn_protocol_driver_sampler.h"
#include "venus-protocol/vn_protocol_driver_sampler_ycbcr_conversion.h"
#include "vk_format.h"

#include "vn_android.h"
#if DETECT_OS_WINDOWS
#include "vn_cs.h"
#endif
#include "vn_device.h"
#include "vn_device_memory.h"
#if DETECT_OS_WINDOWS
#include "vn_helios_hwa2.h"
#include "vn_helios_record_submit.h"
#endif
#include "vn_physical_device.h"
#include "vn_wsi.h"
#include "util/stack_array.h"

#define IMAGE_REQS_CACHE_MAX_ENTRIES 500

/* image commands */

static inline uint32_t
vn_image_get_plane_count(const VkImageCreateInfo *create_info)
{
   if (!(create_info->flags & VK_IMAGE_CREATE_DISJOINT_BIT))
      return 1;

   /* TODO VkDrmFormatModifierPropertiesEXT::drmFormatModifierPlaneCount */
   assert(create_info->tiling != VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT);
   return vk_format_get_plane_count(create_info->format);
}

static inline uint32_t
vn_image_get_plane(const VkImageAspectFlagBits plane_aspect)
{
   switch (plane_aspect) {
   case VK_IMAGE_ASPECT_PLANE_1_BIT:
      return 1;
   case VK_IMAGE_ASPECT_PLANE_2_BIT:
      return 2;
   default:
      return 0;
   }
}

static void
vn_image_fill_reqs(const struct vn_image_memory_requirements *req,
                   VkMemoryRequirements2 *out_reqs)
{
   union {
      VkBaseOutStructure *pnext;
      VkMemoryRequirements2 *two;
      VkMemoryDedicatedRequirements *dedicated;
   } u = { .two = out_reqs };

   while (u.pnext) {
      switch (u.pnext->sType) {
      case VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2:
         u.two->memoryRequirements = req->memory.memoryRequirements;
         break;
      case VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS:
         u.dedicated->prefersDedicatedAllocation =
            req->dedicated.prefersDedicatedAllocation;
         u.dedicated->requiresDedicatedAllocation =
            req->dedicated.requiresDedicatedAllocation;
         break;
      default:
         break;
      }
      u.pnext = u.pnext->pNext;
   }
}

static void
vn_image_cache_debug_dump(struct vn_image_reqs_cache *cache)
{
   vn_log(NULL, "dumping image reqs cache statistics");
   vn_log(NULL, "  hit %u\n", cache->debug.cache_hit_count);
   vn_log(NULL, "  miss %u\n", cache->debug.cache_miss_count);
   vn_log(NULL, "  skip %u\n", cache->debug.cache_skip_count);
}

static bool
vn_image_get_image_reqs_key(struct vn_device *dev,
                            const VkImageCreateInfo *create_info,
                            uint8_t *key)
{
   blake3_hasher blake3_ctx;

   if (!dev->image_reqs_cache.ht)
      return false;

   _mesa_blake3_init(&blake3_ctx);

   /* Hash relevant fields in the pNext chain */
   vk_foreach_struct_const(src, create_info->pNext) {
      switch (src->sType) {
      case VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO: {
         struct VkExternalMemoryImageCreateInfo *ext_mem =
            (struct VkExternalMemoryImageCreateInfo *)src;
         _mesa_blake3_update(&blake3_ctx, &ext_mem->handleTypes,
                           sizeof(VkExternalMemoryHandleTypeFlags));
         break;
      }
      case VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO: {
         struct VkImageFormatListCreateInfo *format_list =
            (struct VkImageFormatListCreateInfo *)src;
         _mesa_blake3_update(&blake3_ctx, format_list->pViewFormats,
                           sizeof(VkFormat) * format_list->viewFormatCount);
         break;
      }
      case VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT: {
         struct VkImageDrmFormatModifierListCreateInfoEXT *format_mod_list =
            (struct VkImageDrmFormatModifierListCreateInfoEXT *)src;
         _mesa_blake3_update(
            &blake3_ctx, format_mod_list->pDrmFormatModifiers,
            sizeof(uint64_t) * format_mod_list->drmFormatModifierCount);
         break;
      }
      case VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_EXPLICIT_CREATE_INFO_EXT: {
         struct VkImageDrmFormatModifierExplicitCreateInfoEXT
            *format_mod_explicit =
               (struct VkImageDrmFormatModifierExplicitCreateInfoEXT *)src;
         _mesa_blake3_update(&blake3_ctx, &format_mod_explicit->drmFormatModifier,
                           sizeof(uint64_t));
         _mesa_blake3_update(
            &blake3_ctx, format_mod_explicit->pPlaneLayouts,
            sizeof(VkSubresourceLayout) *
               format_mod_explicit->drmFormatModifierPlaneCount);
         break;
      }
      case VK_STRUCTURE_TYPE_IMAGE_STENCIL_USAGE_CREATE_INFO: {
         struct VkImageStencilUsageCreateInfo *stencil_usage =
            (struct VkImageStencilUsageCreateInfo *)src;
         _mesa_blake3_update(&blake3_ctx, &stencil_usage->stencilUsage,
                           sizeof(VkImageUsageFlags));
         break;
      }
      case VK_STRUCTURE_TYPE_OPAQUE_CAPTURE_DATA_CREATE_INFO_EXT:
      default:
         /* Skip cache for unsupported pNext */
         dev->image_reqs_cache.debug.cache_skip_count++;
         return false;
      }
   }

   /* Hash contingous block of VkImageCreateInfo starting with
    * VkImageCreateInfo->flags and ending with VkImageCreateInfo->sharingMode
    *
    * There's no padding in involved in this hash block so no concern for C
    * enum sizes or alignment.
    */
   static const size_t create_image_hash_block_size =
      offsetof(VkImageCreateInfo, queueFamilyIndexCount) -
      offsetof(VkImageCreateInfo, flags);

   _mesa_blake3_update(&blake3_ctx, &create_info->flags,
                     create_image_hash_block_size);

   /* Follow pointer and hash pQueueFamilyIndices separately.
    * pQueueFamilyIndices is ignored if sharingMode is not
    * VK_SHARING_MODE_CONCURRENT
    */
   if (create_info->sharingMode == VK_SHARING_MODE_CONCURRENT) {
      _mesa_blake3_update(
         &blake3_ctx, create_info->pQueueFamilyIndices,
         sizeof(uint32_t) * create_info->queueFamilyIndexCount);
   }

   _mesa_blake3_update(&blake3_ctx, &create_info->initialLayout,
                     sizeof(create_info->initialLayout));
   _mesa_blake3_final(&blake3_ctx, key);

   return true;
}

void
vn_image_reqs_cache_init(struct vn_device *dev)
{
   struct vn_image_reqs_cache *cache = &dev->image_reqs_cache;

   if (VN_PERF(NO_ASYNC_IMAGE_CREATE))
      return;

   cache->ht = _mesa_hash_table_create(NULL, vn_cache_key_hash_function,
                                       vn_cache_key_equal_function);
   if (!cache->ht)
      return;

   simple_mtx_init(&cache->mutex, mtx_plain);
   list_inithead(&dev->image_reqs_cache.lru);
}

void
vn_image_reqs_cache_fini(struct vn_device *dev)
{
   const VkAllocationCallbacks *alloc = &dev->base.vk.alloc;
   struct vn_image_reqs_cache *cache = &dev->image_reqs_cache;

   if (!cache->ht)
      return;

   hash_table_foreach(cache->ht, hash_entry) {
      struct vn_image_reqs_cache_entry *cache_entry = hash_entry->data;
      list_del(&cache_entry->head);
      vk_free(alloc, cache_entry);
   }
   assert(list_is_empty(&dev->image_reqs_cache.lru));

   _mesa_hash_table_destroy(cache->ht, NULL);

   simple_mtx_destroy(&cache->mutex);

   if (VN_DEBUG(CACHE))
      vn_image_cache_debug_dump(cache);
}

static bool
vn_image_init_reqs_from_cache(struct vn_device *dev,
                              struct vn_image *img,
                              uint8_t *key)
{
   struct vn_image_reqs_cache *cache = &dev->image_reqs_cache;

   assert(cache->ht);

   simple_mtx_lock(&cache->mutex);
   struct hash_entry *hash_entry = _mesa_hash_table_search(cache->ht, key);
   if (hash_entry) {
      struct vn_image_reqs_cache_entry *cache_entry = hash_entry->data;
      for (uint32_t i = 0; i < cache_entry->plane_count; i++)
         img->requirements[i] = cache_entry->requirements[i];
      list_move_to(&cache_entry->head, &dev->image_reqs_cache.lru);
      p_atomic_inc(&cache->debug.cache_hit_count);
   } else {
      p_atomic_inc(&cache->debug.cache_miss_count);
   }
   simple_mtx_unlock(&cache->mutex);

   return !!hash_entry;
}

static struct vn_image_memory_requirements *
vn_image_get_reqs_from_cache(struct vn_device *dev,
                             uint8_t *key,
                             uint32_t plane)
{
   struct vn_image_memory_requirements *requirements = NULL;
   struct vn_image_reqs_cache *cache = &dev->image_reqs_cache;

   assert(cache->ht);

   simple_mtx_lock(&cache->mutex);
   struct hash_entry *hash_entry = _mesa_hash_table_search(cache->ht, key);
   if (hash_entry) {
      struct vn_image_reqs_cache_entry *cache_entry = hash_entry->data;
      requirements = &cache_entry->requirements[plane];
      list_move_to(&cache_entry->head, &dev->image_reqs_cache.lru);
      p_atomic_inc(&cache->debug.cache_hit_count);
   } else {
      p_atomic_inc(&cache->debug.cache_miss_count);
   }
   simple_mtx_unlock(&cache->mutex);

   return requirements;
}

static void
vn_image_store_reqs_in_cache(struct vn_device *dev,
                             uint8_t *key,
                             uint32_t plane_count,
                             struct vn_image_memory_requirements *requirements)
{
   const VkAllocationCallbacks *alloc = &dev->base.vk.alloc;
   struct vn_image_reqs_cache *cache = &dev->image_reqs_cache;
   struct vn_image_reqs_cache_entry *cache_entry;

   assert(cache->ht);

   simple_mtx_lock(&cache->mutex);

   /* Check if entry was added before lock */
   if (_mesa_hash_table_search(cache->ht, key)) {
      simple_mtx_unlock(&cache->mutex);
      return;
   }

   if (_mesa_hash_table_num_entries(cache->ht) ==
       IMAGE_REQS_CACHE_MAX_ENTRIES) {
      /* Evict/use the last entry in the lru list for this new entry */
      cache_entry =
         list_last_entry(&cache->lru, struct vn_image_reqs_cache_entry, head);

      _mesa_hash_table_remove_key(cache->ht, cache_entry->key);
      list_del(&cache_entry->head);
   } else {
      cache_entry = vk_zalloc(alloc, sizeof(*cache_entry), VN_DEFAULT_ALIGN,
                              VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
      if (!cache_entry) {
         simple_mtx_unlock(&cache->mutex);
         return;
      }
   }

   for (uint32_t i = 0; i < plane_count; i++)
      cache_entry->requirements[i] = requirements[i];

   memcpy(cache_entry->key, key, BLAKE3_KEY_LEN);
   cache_entry->plane_count = plane_count;

   _mesa_hash_table_insert(dev->image_reqs_cache.ht, cache_entry->key,
                           cache_entry);
   list_add(&cache_entry->head, &cache->lru);

   simple_mtx_unlock(&cache->mutex);
}

static void
vn_image_init_memory_requirement_structs(struct vn_image *img,
                                         uint32_t plane_count)
{
   assert(plane_count <= ARRAY_SIZE(img->requirements));

   for (uint32_t i = 0; i < plane_count; i++) {
      img->requirements[i].memory.sType =
         VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2;
      img->requirements[i].memory.pNext = &img->requirements[i].dedicated;
      img->requirements[i].dedicated.sType =
         VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS;
      img->requirements[i].dedicated.pNext = NULL;
   }
}

static void
vn_image_init_memory_requirements(struct vn_image *img,
                                  struct vn_device *dev,
                                  uint32_t plane_count)
{
   vn_image_init_memory_requirement_structs(img, plane_count);

   VkDevice dev_handle = vn_device_to_handle(dev);
   VkImage img_handle = vn_image_to_handle(img);
   if (plane_count == 1) {
      vn_call_vkGetImageMemoryRequirements2(
         dev->primary_ring, dev_handle,
         &(VkImageMemoryRequirementsInfo2){
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_REQUIREMENTS_INFO_2,
            .image = img_handle,
         },
         &img->requirements[0].memory);
      vn_physical_device_sanitize_memory_requirements(
         dev->physical_device,
         &img->requirements[0].memory.memoryRequirements);

      /* AHB backed image requires dedicated allocation */
      if (img->deferred) {
         img->requirements[0].dedicated.prefersDedicatedAllocation = VK_TRUE;
         img->requirements[0].dedicated.requiresDedicatedAllocation = VK_TRUE;
      }
   } else {
      for (uint32_t i = 0; i < plane_count; i++) {
         vn_call_vkGetImageMemoryRequirements2(
            dev->primary_ring, dev_handle,
            &(VkImageMemoryRequirementsInfo2){
               .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_REQUIREMENTS_INFO_2,
               .pNext =
                  &(VkImagePlaneMemoryRequirementsInfo){
                     .sType =
                        VK_STRUCTURE_TYPE_IMAGE_PLANE_MEMORY_REQUIREMENTS_INFO,
                     .planeAspect = VK_IMAGE_ASPECT_PLANE_0_BIT << i,
                  },
               .image = img_handle,
            },
            &img->requirements[i].memory);
         vn_physical_device_sanitize_memory_requirements(
            dev->physical_device,
            &img->requirements[i].memory.memoryRequirements);
      }
   }
}

#if DETECT_OS_WINDOWS
/* Keep creation and the first requirements query in one finite host-control
 * transaction.  The image deliberately remains live: the caller adopts this
 * exact object after creating its associated WDDM allocation, and the existing
 * bind/destroy path then moves its allocation-backed work to the outer batch.
 * The generated KMD schema accepts this pair only when both commands name the
 * same nonzero device and image.  A successful, complete requirements reply
 * therefore proves that the guest-assigned image handle exists without a
 * second host-worker boundary between create and query. */
static VkResult
vn_image_create_query_record_only(struct vn_device *dev,
                                  const VkImageCreateInfo *create_info,
                                  struct vn_image *img)
{
   VkDevice device = vn_device_to_handle(dev);
   VkImage image = vn_image_to_handle(img);
   VkImageMemoryRequirementsInfo2 info = {
      .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_REQUIREMENTS_INFO_2,
      .image = image,
   };
   vn_image_init_memory_requirement_structs(img, 1);

   const size_t create_bytes =
      vn_sizeof_vkCreateImage(device, create_info, NULL, &image);
   const size_t query_bytes = vn_sizeof_vkGetImageMemoryRequirements2(
      device, &info, &img->requirements[0].memory);
   const size_t reply_bytes = vn_sizeof_vkGetImageMemoryRequirements2_reply(
      device, &info, &img->requirements[0].memory);
   if (!create_bytes || !query_bytes || !reply_bytes ||
       create_bytes > SIZE_MAX - query_bytes)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   const size_t command_bytes = create_bytes + query_bytes;
   uint8_t *command = malloc(command_bytes);
   if (!command)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   struct vn_ring_submit_command submit;
   struct vn_cs_encoder *encoder = vn_ring_submit_command_init(
      dev->primary_ring, &submit, command, command_bytes, reply_bytes);
   vn_encode_vkCreateImage(encoder, 0, device, create_info, NULL, &image);
   vn_encode_vkGetImageMemoryRequirements2(
      encoder, VK_COMMAND_GENERATE_REPLY_BIT_EXT, device, &info,
      &img->requirements[0].memory);
   if (vn_cs_encoder_get_fatal(encoder) ||
       vn_cs_encoder_get_len(encoder) != command_bytes) {
      free(command);
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   vn_ring_submit_command(dev->primary_ring, &submit);
   struct vn_cs_decoder *decoder =
      vn_ring_get_command_reply(dev->primary_ring, &submit);
   if (!decoder) {
      free(command);
      return VK_ERROR_DEVICE_LOST;
   }
   vn_decode_vkGetImageMemoryRequirements2_reply(
      decoder, device, &info, &img->requirements[0].memory);
   const bool complete_reply = decoder->cur == decoder->end;
   vn_ring_free_command_reply(dev->primary_ring, &submit);
   free(command);
   /* Temporary runtime diagnostic: keep the wire path unchanged but expose
    * which validated reply field was unusable through vkCreateImage's result.
    * Removed after the bounded DWM capture. */
   if (!complete_reply)
      return VK_ERROR_UNKNOWN;
   if (!img->requirements[0].memory.memoryRequirements.size)
      return VK_ERROR_FORMAT_NOT_SUPPORTED;
   if (!img->requirements[0].memory.memoryRequirements.alignment)
      return VK_ERROR_INITIALIZATION_FAILED;
   if (!img->requirements[0].memory.memoryRequirements.memoryTypeBits)
      return VK_ERROR_FEATURE_NOT_PRESENT;

   vn_physical_device_sanitize_memory_requirements(
      dev->physical_device,
      &img->requirements[0].memory.memoryRequirements);
   return img->requirements[0].memory.memoryRequirements.memoryTypeBits
             ? VK_SUCCESS
             : VK_ERROR_FEATURE_NOT_PRESENT;
}
#endif

static VkResult
vn_image_init(struct vn_device *dev,
              const VkImageCreateInfo *create_info,
              struct vn_image *img)
{
   VkDevice device = vn_device_to_handle(dev);
   VkImage image = vn_image_to_handle(img);
   VkResult result = VK_SUCCESS;

   /* Check if mem reqs in cache. If found, make async call */
   uint8_t key[BLAKE3_KEY_LEN] = { 0 };
   const bool cacheable = vn_image_get_image_reqs_key(dev, create_info, key);

   if (cacheable && vn_image_init_reqs_from_cache(dev, img, key)) {
      vn_async_vkCreateImage(dev->primary_ring, device, create_info, NULL,
                             &image);
      return VK_SUCCESS;
   }

   const uint32_t plane_count = vn_image_get_plane_count(create_info);
#if DETECT_OS_WINDOWS
   if (plane_count == 1 &&
       vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      result = vn_image_create_query_record_only(dev, create_info, img);
      if (result != VK_SUCCESS)
         return result;
   } else
#endif
   {
      result = vn_call_vkCreateImage(dev->primary_ring, device, create_info,
                                     NULL, &image);
      if (result != VK_SUCCESS)
         return result;
      vn_image_init_memory_requirements(img, dev, plane_count);
   }

   if (cacheable)
      vn_image_store_reqs_in_cache(dev, key, plane_count, img->requirements);

   return VK_SUCCESS;
}

static VkResult
vn_image_create_with_renderer_info(
   struct vn_device *dev,
   const VkImageCreateInfo *create_info,
   const VkImageCreateInfo *renderer_create_info,
   const VkAllocationCallbacks *alloc,
   struct vn_image **out_img)
{
   struct vn_image *img =
      vk_image_create(&dev->base.vk, create_info, alloc, sizeof(*img));
   if (!img)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   vn_object_set_id(img, vn_get_next_obj_id(), VK_OBJECT_TYPE_IMAGE);

   VkResult result = vn_image_init(dev, renderer_create_info, img);
   if (result != VK_SUCCESS) {
      vk_image_destroy(&dev->base.vk, alloc, &img->base.vk);
      return result;
   }

   *out_img = img;

   return VK_SUCCESS;
}

VkResult
vn_image_create(struct vn_device *dev,
                const VkImageCreateInfo *create_info,
                const VkAllocationCallbacks *alloc,
                struct vn_image **out_img)
{
   return vn_image_create_with_renderer_info(
      dev, create_info, create_info, alloc, out_img);
}

VkResult
vn_image_init_deferred(struct vn_device *dev,
                       const VkImageCreateInfo *create_info,
                       struct vn_image *img)
{
   VkResult result = vn_image_init(dev, create_info, img);
   img->deferred_initialized = result == VK_SUCCESS;
   return result;
}

static VkResult
vn_image_create_deferred(struct vn_device *dev,
                         const VkImageCreateInfo *create_info,
                         const VkAllocationCallbacks *alloc,
                         struct vn_image **out_img)
{
   struct vn_image *img =
      vk_image_create(&dev->base.vk, create_info, alloc, sizeof(*img));
   if (!img)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   vn_object_set_id(img, vn_get_next_obj_id(), VK_OBJECT_TYPE_IMAGE);

   VkResult result = vk_android_init_deferred_image(
      &dev->base.vk, &img->base.vk, create_info, alloc);
   if (result != VK_SUCCESS) {
      vk_image_destroy(&dev->base.vk, alloc, &img->base.vk);
      return result;
   }

   img->deferred = true;
   *out_img = img;

   return VK_SUCCESS;
}

struct vn_image_create_info {
   VkImageCreateInfo create;
   VkExternalMemoryImageCreateInfo external;
   VkImageFormatListCreateInfo format_list;
   VkImageStencilUsageCreateInfo stencil;
   VkImageDrmFormatModifierListCreateInfoEXT modifier_list;
   VkImageDrmFormatModifierExplicitCreateInfoEXT modifier_explicit;
};

static const VkImageCreateInfo *
vn_image_fix_create_info(
   const VkImageCreateInfo *create_info,
   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type,
   struct vn_image_create_info *local_info)
{
   bool has_external = false;
   local_info->create = *create_info;
   VkBaseOutStructure *cur = (void *)&local_info->create;

   vk_foreach_struct_const(src, create_info->pNext) {
      void *next = NULL;
      switch (src->sType) {
      case VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO:
         memcpy(&local_info->external, src, sizeof(local_info->external));
         local_info->external.handleTypes = renderer_handle_type;
         has_external = true;
         next = &local_info->external;
         break;
      case VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO:
         memcpy(&local_info->format_list, src,
                sizeof(local_info->format_list));
         next = &local_info->format_list;
         break;
      case VK_STRUCTURE_TYPE_IMAGE_STENCIL_USAGE_CREATE_INFO:
         memcpy(&local_info->stencil, src, sizeof(local_info->stencil));
         next = &local_info->stencil;
         break;
      case VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT:
         memcpy(&local_info->modifier_list, src,
                sizeof(local_info->modifier_list));
         next = &local_info->modifier_list;
         break;
      case VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_EXPLICIT_CREATE_INFO_EXT:
         memcpy(&local_info->modifier_explicit, src,
                sizeof(local_info->modifier_explicit));
         next = &local_info->modifier_explicit;
         break;
      default:
         break;
      }

      if (next) {
         cur->pNext = next;
         cur = next;
      }
   }

   /* Helios: append external info when the app provided none (see
    * vn_buffer_fix_create_info — vkr force-exports HOST_VISIBLE memory, so
    * an image that may bind such memory must carry matching handleTypes or
    * the bind violates VUID-VkBindImageMemoryInfo-memory-02728). */
   if (!has_external) {
      local_info->external = (VkExternalMemoryImageCreateInfo){
         .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO,
         .handleTypes = (VkExternalMemoryHandleTypeFlags)renderer_handle_type,
      };
      cur->pNext = (void *)&local_info->external;
      cur = (void *)&local_info->external;
   }

   cur->pNext = NULL;

   return &local_info->create;
}

/* Whether this create info must be rewritten to carry the renderer's external
 * handle type, for BOTH vn_CreateImage and the memory-requirements query.
 *
 * ⛔ ONE function on purpose: the two call sites must agree exactly, and when
 * they were hand-mirrored the disagreement showed up live as undersized
 * dedicated allocations and disallowed memory types on dwm's composition
 * images (VUID 02964/01615/01617).
 *
 * The no-external-info arm used to require LINEAR. That left every OPTIMAL
 * image DXVK creates for a Helios outer-associated allocation with
 * handleTypes = 0 while the host bound imported DMA_BUF memory to it --
 * VUID-VkBindImageMemoryInfo-memory-02989, 26 times per boot on 2026-08-28,
 * and the shape that makes an image whose bytes the dma-buf cannot see.
 * PREINITIALIZED still cannot carry external info (VUID-VkImageCreateInfo-
 * pNext-01443). `VN_DEBUG=no_ext_optimal` restores the LINEAR-only rule.
 */
static bool
vn_image_needs_external_fixup(
   const struct vn_physical_device *physical_dev,
   const VkImageCreateInfo *create_info,
   const VkExternalMemoryImageCreateInfo *external_info,
   VkExternalMemoryHandleTypeFlagBits renderer_handle_type)
{
   if (!renderer_handle_type)
      return false;
   if (external_info &&
       vn_preserve_explicit_dmabuf_handle_types(physical_dev,
                                                external_info->handleTypes))
      return false;
   if (external_info)
      return external_info->handleTypes != renderer_handle_type;
   if (create_info->initialLayout != VK_IMAGE_LAYOUT_UNDEFINED)
      return false;
   return !VN_DEBUG(NO_EXT_OPTIMAL) ||
          create_info->tiling == VK_IMAGE_TILING_LINEAR;
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_CreateImage(VkDevice device,
               const VkImageCreateInfo *pCreateInfo,
               const VkAllocationCallbacks *pAllocator,
               VkImage *pImage)
{
   struct vn_device *dev = vn_device_from_handle(device);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;
   struct vn_image *img;
   VkResult result;

   const struct wsi_image_create_info *wsi_info = NULL;
   const VkNativeBufferANDROID *anb_info = NULL;
   const VkImageSwapchainCreateInfoKHR *swapchain_info = NULL;
   const VkExternalMemoryImageCreateInfo *external_info = NULL;
   bool ahb_info = false;

   vk_foreach_struct_const(pnext, pCreateInfo->pNext) {
      switch ((uint32_t)pnext->sType) {
      case VK_STRUCTURE_TYPE_WSI_IMAGE_CREATE_INFO_MESA:
         wsi_info = (void *)pnext;
         break;
      case VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID:
         anb_info = (void *)pnext;
         break;
      case VK_STRUCTURE_TYPE_IMAGE_SWAPCHAIN_CREATE_INFO_KHR:
         swapchain_info = (void *)pnext;
         if (!swapchain_info->swapchain)
            swapchain_info = NULL;
         break;
      case VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO:
         external_info = (void *)pnext;
         if (!external_info->handleTypes)
            external_info = NULL;
         else if (
            external_info->handleTypes ==
            VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID)
            ahb_info = true;
         break;
      default:
         break;
      }
   }

#if DETECT_OS_WINDOWS
   /* 2026-08-31: a cross-process open zeroes the creator's pixels, and only the
    * two sides' MEMORY parameters were ever compared (they match). Two images
    * aliasing one host allocation with different create parameters would not,
    * so print the image's too — bounded, first 64 per process. */
   {
      static uint32_t him1_logged;
      if (him1_logged < 64u) {
         him1_logged++;
         vn_renderer_helios_diag_log(
            "HIM1 pid=%lu %ux%ux%u fmt=%u mips=%u layers=%u samples=%u "
            "tiling=%u usage=0x%x flags=0x%x sharing=%u layout=%u ext=0x%x",
            (unsigned long)GetCurrentProcessId(), pCreateInfo->extent.width,
            pCreateInfo->extent.height, pCreateInfo->extent.depth,
            (unsigned)pCreateInfo->format, pCreateInfo->mipLevels,
            pCreateInfo->arrayLayers, (unsigned)pCreateInfo->samples,
            (unsigned)pCreateInfo->tiling, (unsigned)pCreateInfo->usage,
            (unsigned)pCreateInfo->flags, (unsigned)pCreateInfo->sharingMode,
            (unsigned)pCreateInfo->initialLayout,
            external_info ? (unsigned)external_info->handleTypes : 0u);
      }
   }
#endif

   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type =
      vn_renderer_handle_type_for_guest(
         dev->physical_device,
         external_info ? external_info->handleTypes : 0);

   /* No need to fix external handle type for:
    * - common wsi image: dma_buf is hard-coded in wsi_configure_native_image
    * - common wsi image alias: it aligns with wsi_info on external handle
    * - Android wsi image: VK_ANDROID_native_buffer involves no external info
    * - AHB external image: deferred creation reconstructs external info
    *
    * Must fix the external handle type for:
    * - non-AHB external image requesting handle types different from renderer
    *
    * Native Win32 opaque memory deliberately selects its own renderer-side
    * handle type here, independently of the DMA_BUF scanout contract.
    */
   if (wsi_info) {
      result = vn_wsi_create_image(dev, pCreateInfo, wsi_info, alloc, &img);
   } else if (anb_info) {
      result = vn_android_image_from_anb(dev, pCreateInfo, alloc, &img);
   } else if (ahb_info) {
      result = vn_image_create_deferred(dev, pCreateInfo, alloc, &img);
   } else if (swapchain_info) {
#ifdef VK_USE_PLATFORM_ANDROID_KHR
      result = vn_image_create_deferred(dev, pCreateInfo, alloc, &img);
#elif defined(VN_USE_WSI_PLATFORM)
      result = wsi_common_create_swapchain_image(
         &dev->physical_device->wsi_device, pCreateInfo,
         (VkImage *)&img);
#else
      result = VK_ERROR_FEATURE_NOT_PRESENT;
#endif
   } else {
      struct vn_image_create_info local_info;
#if DETECT_OS_WINDOWS
      const VkImageCreateInfo *app_create_info = pCreateInfo;
#endif
      const bool fix_external = vn_image_needs_external_fixup(
         dev->physical_device, pCreateInfo, external_info,
         renderer_handle_type);
      if (fix_external) {
         pCreateInfo = vn_image_fix_create_info(
            pCreateInfo, renderer_handle_type, &local_info);
      }

#if DETECT_OS_WINDOWS
      /* The renderer consumes its own external handle type, while the direct
       * guest object must retain the exact type the caller requested.  In
       * particular, the later C57 dedicated import and A7 presentable-image
       * validation must see D3D12_RESOURCE rather than a renderer fd type. */
      result = vn_image_create_with_renderer_info(
         dev, app_create_info, pCreateInfo, alloc, &img);
#else
      result = vn_image_create(dev, pCreateInfo, alloc, &img);
#endif
   }

   if (result != VK_SUCCESS)
      return vn_error(dev->instance, result);

   *pImage = vn_image_to_handle(img);
   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_DestroyImage(VkDevice device,
                VkImage image,
                const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_image *img = vn_image_from_handle(image);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   if (!img)
      return;

   /* must not ask renderer to destroy uninitialized deferred image */
   if (!img->deferred || img->deferred_initialized) {
#if DETECT_OS_WINDOWS
      const bool record_only =
         vn_helios_submit_instance_mode(dev->instance) ==
         VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
      const struct vn_helios_memory_binding *binding = NULL;
      uint32_t binding_count = 0;
      if (record_only) {
         for (uint32_t i = 0; i < ARRAY_SIZE(img->helios_bindings); i++) {
            if (img->helios_bindings[i].valid) {
               binding = &img->helios_bindings[i];
               binding_count++;
            }
         }
      }
      if (record_only && binding_count == 1) {
         struct vn_device_memory *mem =
            vn_device_memory_helios_binding_memory(dev, binding);
         const size_t payload_bytes =
            vn_sizeof_vkDestroyImage(device, image, NULL);
         uint8_t *payload = payload_bytes ? malloc(payload_bytes) : NULL;
         VkResult defer_result = VK_SUCCESS;
         if (!mem || !payload || payload_bytes > HELIOS_HOB1_MAX_BYTES) {
            free(payload);
            defer_result = mem ? VK_ERROR_OUT_OF_HOST_MEMORY
                               : VK_ERROR_VALIDATION_FAILED_EXT;
         } else {
            struct vn_cs_encoder encoder =
               VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_bytes);
            vn_encode_vkDestroyImage(&encoder, 0, device, image, NULL);
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
      } else if (record_only && binding_count > 1) {
         /* One generated destroy cannot be repeated once per plane.  The
          * record-only profile therefore refuses a disjoint multi-allocation
          * image instead of omitting one allocation from its closure. */
         vn_helios_record_note_deferred_use(dev->instance);
         p_atomic_set(&dev->helios_lost, 1);
         (void)vn_error(dev->instance,
                        VK_ERROR_VALIDATION_FAILED_EXT);
      } else
#endif
      {
         vn_async_vkDestroyImage(dev->primary_ring, device, image, NULL);
      }
   }

   vk_image_destroy(&dev->base.vk, alloc, &img->base.vk);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetImageMemoryRequirements2(VkDevice device,
                               const VkImageMemoryRequirementsInfo2 *pInfo,
                               VkMemoryRequirements2 *pMemoryRequirements)
{
   const struct vn_image *img = vn_image_from_handle(pInfo->image);

   uint32_t plane = 0;
   const VkImagePlaneMemoryRequirementsInfo *plane_info =
      vk_find_struct_const(pInfo->pNext,
                           IMAGE_PLANE_MEMORY_REQUIREMENTS_INFO);
   if (plane_info)
      plane = vn_image_get_plane(plane_info->planeAspect);

   vn_image_fill_reqs(&img->requirements[plane], pMemoryRequirements);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetImageSparseMemoryRequirements2(
   VkDevice device,
   const VkImageSparseMemoryRequirementsInfo2 *pInfo,
   uint32_t *pSparseMemoryRequirementCount,
   VkSparseImageMemoryRequirements2 *pSparseMemoryRequirements)
{
   struct vn_device *dev = vn_device_from_handle(device);

   /* see vn_GetPhysicalDeviceSparseImageFormatProperties2 */
   if (dev->physical_device->sparse_binding_disabled) {
      *pSparseMemoryRequirementCount = 0;
      return;
   }

   /* TODO local or per-device cache */
   vn_call_vkGetImageSparseMemoryRequirements2(
      dev->primary_ring, device, pInfo, pSparseMemoryRequirementCount,
      pSparseMemoryRequirements);
}

static VkResult
vn_image_bind_wsi_memory(struct vn_device *dev,
                         uint32_t count,
                         const VkBindImageMemoryInfo *infos)
{
   STACK_ARRAY(VkBindImageMemoryInfo, local_infos, count);
   typed_memcpy(local_infos, infos, count);

   for (uint32_t i = 0; i < count; i++) {
      VkBindImageMemoryInfo *info = &local_infos[i];

      if (info->memory == VK_NULL_HANDLE) {
#ifdef VK_USE_PLATFORM_ANDROID_KHR
         info->memory = vn_android_get_wsi_memory(dev, info);
         if (info->memory == VK_NULL_HANDLE) {
            STACK_ARRAY_FINISH(local_infos);
            return VK_ERROR_OUT_OF_HOST_MEMORY;
         }
#elif defined(VN_USE_WSI_PLATFORM)
         const VkBindImageMemorySwapchainInfoKHR *swapchain_info =
            vk_find_struct_const(info->pNext,
                                 BIND_IMAGE_MEMORY_SWAPCHAIN_INFO_KHR);
         assert(swapchain_info);

         info->memory = wsi_common_get_memory(swapchain_info->swapchain,
                                              swapchain_info->imageIndex);
#else
         STACK_ARRAY_FINISH(local_infos);
         return VK_ERROR_FEATURE_NOT_PRESENT;
#endif
         info->memoryOffset = 0;
      }
      assert(info->memory != VK_NULL_HANDLE);
   }

   vn_async_vkBindImageMemory2(dev->primary_ring, vn_device_to_handle(dev),
                               count, local_infos);

   STACK_ARRAY_FINISH(local_infos);

   return VK_SUCCESS;
}

VKAPI_ATTR VkResult VKAPI_CALL
vn_BindImageMemory2(VkDevice device,
                    uint32_t bindInfoCount,
                    const VkBindImageMemoryInfo *pBindInfos)
{
   struct vn_device *dev = vn_device_from_handle(device);

   for (uint32_t i = 0; i < bindInfoCount; i++) {
      if (pBindInfos[i].memory == VK_NULL_HANDLE)
         return vn_image_bind_wsi_memory(dev, bindInfoCount, pBindInfos);
   }

   STACK_ARRAY(VkBindImageMemoryInfo, local_infos, bindInfoCount);
   if (!local_infos)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

#if DETECT_OS_WINDOWS
   STACK_ARRAY(struct vn_helios_memory_binding, helios_bindings,
               bindInfoCount);
   STACK_ARRAY(uint32_t, helios_planes, bindInfoCount);
   STACK_ARRAY(struct vn_helios_deferred_record *, helios_records,
               bindInfoCount);
   if (!helios_bindings || !helios_planes || !helios_records) {
      STACK_ARRAY_FINISH(helios_records);
      STACK_ARRAY_FINISH(helios_planes);
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
         struct vn_image *img = vn_image_from_handle(pBindInfos[i].image);
         struct vn_device_memory *mem =
            vn_device_memory_from_handle(pBindInfos[i].memory);
         const VkBindImagePlaneMemoryInfo *plane_info =
            vk_find_struct_const(pBindInfos[i].pNext,
                                 BIND_IMAGE_PLANE_MEMORY_INFO);
         const uint32_t plane =
            plane_info ? vn_image_get_plane(plane_info->planeAspect) : 0;
         if (!img || img->base.vk.base.device != &dev->base.vk ||
             plane >= ARRAY_SIZE(img->helios_bindings) ||
             img->helios_bindings[plane].valid ||
             vn_device_memory_helios_bind(
                dev, mem, pBindInfos[i].memoryOffset,
                img->requirements[plane].memory.memoryRequirements.size,
                &helios_bindings[i]) != VK_SUCCESS) {
            STACK_ARRAY_FINISH(helios_planes);
            STACK_ARRAY_FINISH(helios_records);
            STACK_ARRAY_FINISH(helios_bindings);
            STACK_ARRAY_FINISH(local_infos);
            return vn_error(dev->instance,
                            VK_ERROR_VALIDATION_FAILED_EXT);
         }
         helios_planes[i] = plane;
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
            vn_sizeof_vkBindImageMemory2(device, 1, &local_infos[i]);
         uint8_t *payload = payload_bytes ? malloc(payload_bytes) : NULL;
         if (!payload || payload_bytes > HELIOS_HOB1_MAX_BYTES) {
            free(payload);
            defer_result = VK_ERROR_OUT_OF_HOST_MEMORY;
            break;
         }
         struct vn_cs_encoder encoder =
            VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_bytes);
         vn_encode_vkBindImageMemory2(&encoder, 0, device, 1,
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
         STACK_ARRAY_FINISH(helios_planes);
         STACK_ARRAY_FINISH(helios_bindings);
         STACK_ARRAY_FINISH(local_infos);
         return vn_error(dev->instance, defer_result);
      }
      for (uint32_t i = 0; i < bindInfoCount; i++) {
         struct vn_image *img = vn_image_from_handle(pBindInfos[i].image);
         img->helios_bindings[helios_planes[i]] = helios_bindings[i];
      }
   } else {
      vn_async_vkBindImageMemory2(dev->primary_ring, device, bindInfoCount,
                                  local_infos);
   }
   STACK_ARRAY_FINISH(helios_records);
   STACK_ARRAY_FINISH(helios_planes);
   STACK_ARRAY_FINISH(helios_bindings);
#else
   vn_async_vkBindImageMemory2(dev->primary_ring, device, bindInfoCount,
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

VKAPI_ATTR VkResult VKAPI_CALL
vn_GetImageDrmFormatModifierPropertiesEXT(
   VkDevice device,
   VkImage image,
   VkImageDrmFormatModifierPropertiesEXT *pProperties)
{
   struct vn_device *dev = vn_device_from_handle(device);

   /* TODO local cache */
   return vn_call_vkGetImageDrmFormatModifierPropertiesEXT(
      dev->primary_ring, device, image, pProperties);
}

static VkImageAspectFlags
vn_image_get_aspect(struct vn_image *img, VkImageAspectFlags aspect)
{
   if (!img->deferred)
      return aspect;

   switch (aspect) {
   case VK_IMAGE_ASPECT_COLOR_BIT:
   case VK_IMAGE_ASPECT_DEPTH_BIT:
   case VK_IMAGE_ASPECT_STENCIL_BIT:
   case VK_IMAGE_ASPECT_PLANE_0_BIT:
      return VK_IMAGE_ASPECT_MEMORY_PLANE_0_BIT_EXT;
   case VK_IMAGE_ASPECT_PLANE_1_BIT:
      return VK_IMAGE_ASPECT_MEMORY_PLANE_1_BIT_EXT;
   case VK_IMAGE_ASPECT_PLANE_2_BIT:
      return VK_IMAGE_ASPECT_MEMORY_PLANE_2_BIT_EXT;
   default:
      break;
   }
   UNREACHABLE("unexpected aspect");
}

VKAPI_ATTR void VKAPI_CALL
vn_GetImageSubresourceLayout(VkDevice device,
                             VkImage image,
                             const VkImageSubresource *pSubresource,
                             VkSubresourceLayout *pLayout)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_image *img = vn_image_from_handle(image);

   /* override aspect mask for ahb images with tiling modifier */
   VkImageSubresource local_subresource;
   const VkImageAspectFlags aspect =
      vn_image_get_aspect(img, pSubresource->aspectMask);
   if (aspect != pSubresource->aspectMask) {
      local_subresource = *pSubresource;
      local_subresource.aspectMask = aspect;
      pSubresource = &local_subresource;
   }

   /* TODO local cache */
   vn_call_vkGetImageSubresourceLayout(dev->primary_ring, device, image,
                                       pSubresource, pLayout);
}

/* image view commands */

VKAPI_ATTR VkResult VKAPI_CALL
vn_CreateImageView(VkDevice device,
                   const VkImageViewCreateInfo *pCreateInfo,
                   const VkAllocationCallbacks *pAllocator,
                   VkImageView *pView)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_image *img = vn_image_from_handle(pCreateInfo->image);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   VkImageViewCreateInfo local_info;
   if (pCreateInfo->format == VK_FORMAT_UNDEFINED) {
      local_info = *pCreateInfo;
      local_info.format = img->base.vk.format;
      pCreateInfo = &local_info;

      assert(pCreateInfo->format != VK_FORMAT_UNDEFINED);
   }

   struct vn_image_view *view =
      vk_zalloc(alloc, sizeof(*view), VN_DEFAULT_ALIGN,
                VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
   if (!view)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

   vn_object_base_init(&view->base, VK_OBJECT_TYPE_IMAGE_VIEW, &dev->base);
   view->image = img;

   VkImageView view_handle = vn_image_view_to_handle(view);

#if DETECT_OS_WINDOWS
   /* A7: a view dereferences the image's outer allocation on the host, and
    * HVC1 is unordered against the batch that materializes the deferred
    * allocate/bind — a ring-borne create reaches the host on an unbound
    * image (measured: dwm's views baked dead addresses, black desktop,
    * 2026-08-24).  Ride the object-materialization lane instead. */
   if (vn_helios_submit_instance_mode(dev->instance) ==
       VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      struct vn_helios_memory_binding deps[ARRAY_SIZE(img->helios_bindings)];
      uint32_t dep_count = 0;
      for (uint32_t i = 0; i < ARRAY_SIZE(img->helios_bindings); i++) {
         if (img->helios_bindings[i].valid)
            deps[dep_count++] = img->helios_bindings[i];
      }
      if (dep_count) {
         VkResult defer_result = VK_ERROR_OUT_OF_HOST_MEMORY;
         const size_t generated =
            vn_sizeof_vkCreateImageView(device, pCreateInfo, NULL,
                                        &view_handle);
         if (generated && generated <= HELIOS_HOB1_MAX_BYTES - 256) {
            const size_t capacity = generated + 256;
            uint8_t *buf = malloc(capacity);
            if (buf) {
               struct vn_cs_encoder enc =
                  VN_CS_ENCODER_INITIALIZER_LOCAL(buf, capacity);
               vn_encode_vkCreateImageView(&enc, 0, device, pCreateInfo,
                                           NULL, &view_handle);
               const size_t len = vn_cs_encoder_get_len(&enc);
               if (!vn_cs_encoder_get_fatal(&enc) && len &&
                   len <= capacity) {
                  defer_result = vn_helios_record_defer_object_command(
                     dev, buf, len, deps, dep_count);
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
   }
#endif

   vn_async_vkCreateImageView(dev->primary_ring, device, pCreateInfo, NULL,
                              &view_handle);

   *pView = view_handle;

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_DestroyImageView(VkDevice device,
                    VkImageView imageView,
                    const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_image_view *view = vn_image_view_from_handle(imageView);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   if (!view)
      return;

#if DETECT_OS_WINDOWS
   /* The destroy must trail its create on the object-materialization lane;
    * a ring destroy would overtake a still-pending create.  Same deps as the
    * create so both skip or drop together. */
   bool destroyed = false;
   if (vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY &&
       view->image) {
      const struct vn_image *img = view->image;
      struct vn_helios_memory_binding deps[ARRAY_SIZE(img->helios_bindings)];
      uint32_t dep_count = 0;
      for (uint32_t i = 0; i < ARRAY_SIZE(img->helios_bindings); i++) {
         if (img->helios_bindings[i].valid)
            deps[dep_count++] = img->helios_bindings[i];
      }
      if (dep_count) {
         const size_t generated =
            vn_sizeof_vkDestroyImageView(device, imageView, NULL);
         if (generated && generated <= HELIOS_HOB1_MAX_BYTES - 256) {
            const size_t capacity = generated + 256;
            uint8_t *buf = malloc(capacity);
            if (buf) {
               struct vn_cs_encoder enc =
                  VN_CS_ENCODER_INITIALIZER_LOCAL(buf, capacity);
               vn_encode_vkDestroyImageView(&enc, 0, device, imageView,
                                            NULL);
               const size_t len = vn_cs_encoder_get_len(&enc);
               if (!vn_cs_encoder_get_fatal(&enc) && len &&
                   len <= capacity &&
                   vn_helios_record_defer_object_command(
                      dev, buf, len, deps, dep_count) == VK_SUCCESS) {
                  destroyed = true;
               } else {
                  free(buf);
               }
            }
         }
         if (!destroyed) {
            /* Never fall back to the ring: it could overtake the pending
             * create.  Leak the host twin loudly; context teardown
             * reclaims it. */
            vn_renderer_helios_diag_log(
               "HOC1 view destroy defer failed view=%p", (void *)view);
            destroyed = true;
         }
      }
   }
   if (!destroyed)
      vn_async_vkDestroyImageView(dev->primary_ring, device, imageView,
                                  NULL);
#else
   vn_async_vkDestroyImageView(dev->primary_ring, device, imageView, NULL);
#endif

   vn_object_base_fini(&view->base);
   vk_free(alloc, view);
}

/* sampler commands */

VKAPI_ATTR VkResult VKAPI_CALL
vn_CreateSampler(VkDevice device,
                 const VkSamplerCreateInfo *pCreateInfo,
                 const VkAllocationCallbacks *pAllocator,
                 VkSampler *pSampler)
{
   struct vn_device *dev = vn_device_from_handle(device);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   struct vn_sampler *sampler =
      vk_zalloc(alloc, sizeof(*sampler), VN_DEFAULT_ALIGN,
                VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
   if (!sampler)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

   vn_object_base_init(&sampler->base, VK_OBJECT_TYPE_SAMPLER, &dev->base);

   VkSampler sampler_handle = vn_sampler_to_handle(sampler);
   vn_async_vkCreateSampler(dev->primary_ring, device, pCreateInfo, NULL,
                            &sampler_handle);

   *pSampler = sampler_handle;

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_DestroySampler(VkDevice device,
                  VkSampler _sampler,
                  const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_sampler *sampler = vn_sampler_from_handle(_sampler);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   if (!sampler)
      return;

   vn_async_vkDestroySampler(dev->primary_ring, device, _sampler, NULL);

   vn_object_base_fini(&sampler->base);
   vk_free(alloc, sampler);
}

/* sampler YCbCr conversion commands */

VKAPI_ATTR VkResult VKAPI_CALL
vn_CreateSamplerYcbcrConversion(
   VkDevice device,
   const VkSamplerYcbcrConversionCreateInfo *pCreateInfo,
   const VkAllocationCallbacks *pAllocator,
   VkSamplerYcbcrConversion *pYcbcrConversion)
{
   struct vn_device *dev = vn_device_from_handle(device);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;
   const VkExternalFormatANDROID *ext_info =
      vk_find_struct_const(pCreateInfo->pNext, EXTERNAL_FORMAT_ANDROID);

   VkSamplerYcbcrConversionCreateInfo local_info;
   if (ext_info && ext_info->externalFormat) {
      assert(pCreateInfo->format == VK_FORMAT_UNDEFINED);

      local_info = *pCreateInfo;
      local_info.format = ext_info->externalFormat;
      local_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
      local_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
      local_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
      local_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
      pCreateInfo = &local_info;

      assert(pCreateInfo->format != VK_FORMAT_UNDEFINED);
   }

   struct vn_sampler_ycbcr_conversion *conv =
      vk_zalloc(alloc, sizeof(*conv), VN_DEFAULT_ALIGN,
                VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);
   if (!conv)
      return vn_error(dev->instance, VK_ERROR_OUT_OF_HOST_MEMORY);

   vn_object_base_init(&conv->base, VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION,
                       &dev->base);

   VkSamplerYcbcrConversion conv_handle =
      vn_sampler_ycbcr_conversion_to_handle(conv);
   vn_async_vkCreateSamplerYcbcrConversion(dev->primary_ring, device,
                                           pCreateInfo, NULL, &conv_handle);

   *pYcbcrConversion = conv_handle;

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
vn_DestroySamplerYcbcrConversion(VkDevice device,
                                 VkSamplerYcbcrConversion ycbcrConversion,
                                 const VkAllocationCallbacks *pAllocator)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_sampler_ycbcr_conversion *conv =
      vn_sampler_ycbcr_conversion_from_handle(ycbcrConversion);
   const VkAllocationCallbacks *alloc =
      pAllocator ? pAllocator : &dev->base.vk.alloc;

   if (!conv)
      return;

   vn_async_vkDestroySamplerYcbcrConversion(dev->primary_ring, device,
                                            ycbcrConversion, NULL);

   vn_object_base_fini(&conv->base);
   vk_free(alloc, conv);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetDeviceImageMemoryRequirements(
   VkDevice device,
   const VkDeviceImageMemoryRequirements *pInfo,
   VkMemoryRequirements2 *pMemoryRequirements)
{
   struct vn_device *dev = vn_device_from_handle(device);

   /* The requirements reported HERE must describe the create info
    * vn_CreateImage will actually use, or callers size and place memory the
    * real (external) image cannot bind. Shared predicate — see
    * vn_image_needs_external_fixup. */
   const VkExternalMemoryImageCreateInfo *external_info = vk_find_struct_const(
      pInfo->pCreateInfo->pNext, EXTERNAL_MEMORY_IMAGE_CREATE_INFO);
   const VkExternalMemoryHandleTypeFlagBits renderer_handle_type =
      vn_renderer_handle_type_for_guest(
         dev->physical_device,
         external_info ? external_info->handleTypes : 0);
   const bool fix_external = vn_image_needs_external_fixup(
      dev->physical_device, pInfo->pCreateInfo, external_info,
      renderer_handle_type);
   struct vn_image_create_info local_info;
   VkDeviceImageMemoryRequirements fixed_info;
   if (fix_external) {
      fixed_info = *pInfo;
      fixed_info.pCreateInfo = vn_image_fix_create_info(
         pInfo->pCreateInfo, renderer_handle_type, &local_info);
      pInfo = &fixed_info;
   }

   uint8_t key[BLAKE3_KEY_LEN] = { 0 };
   const bool cacheable =
      vn_image_get_image_reqs_key(dev, pInfo->pCreateInfo, key);

   if (cacheable) {
      uint32_t plane = 0;
      if (pInfo->pCreateInfo->flags & VK_IMAGE_CREATE_DISJOINT_BIT)
         plane = vn_image_get_plane(pInfo->planeAspect);

      const struct vn_image_memory_requirements *cached_reqs =
         vn_image_get_reqs_from_cache(dev, key, plane);
      if (cached_reqs) {
         vn_image_fill_reqs(cached_reqs, pMemoryRequirements);
         return;
      }

      const uint32_t plane_count =
         vn_image_get_plane_count(pInfo->pCreateInfo);
      STACK_ARRAY(VkDeviceImageMemoryRequirements, req_info, plane_count);
      STACK_ARRAY(struct vn_image_memory_requirements, reqs, plane_count);

      /* Retrieve reqs for all planes so the cache entry is complete */
      for (uint32_t i = 0; i < plane_count; i++) {
         req_info[i].sType =
            VK_STRUCTURE_TYPE_DEVICE_IMAGE_MEMORY_REQUIREMENTS;
         req_info[i].pNext = NULL;
         req_info[i].pCreateInfo = pInfo->pCreateInfo;
         req_info[i].planeAspect = VK_IMAGE_ASPECT_PLANE_0_BIT << i;

         reqs[i].memory.sType = VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2;
         reqs[i].memory.pNext = &reqs[i].dedicated;
         reqs[i].dedicated.sType =
            VK_STRUCTURE_TYPE_MEMORY_DEDICATED_REQUIREMENTS;
         reqs[i].dedicated.pNext = NULL;

         vn_call_vkGetDeviceImageMemoryRequirements(
            dev->primary_ring, device, &req_info[i], &reqs[i].memory);
         vn_physical_device_sanitize_memory_requirements(
            dev->physical_device,
            &reqs[i].memory.memoryRequirements);
      }
      vn_image_fill_reqs(&reqs[plane], pMemoryRequirements);
      vn_image_store_reqs_in_cache(dev, key, plane_count, reqs);

      STACK_ARRAY_FINISH(req_info);
      STACK_ARRAY_FINISH(reqs);
   } else {
      vn_call_vkGetDeviceImageMemoryRequirements(dev->primary_ring, device,
                                                 pInfo, pMemoryRequirements);
      vn_physical_device_sanitize_memory_requirements(
         dev->physical_device, &pMemoryRequirements->memoryRequirements);
   }
}

VKAPI_ATTR void VKAPI_CALL
vn_GetDeviceImageSparseMemoryRequirements(
   VkDevice device,
   const VkDeviceImageMemoryRequirements *pInfo,
   uint32_t *pSparseMemoryRequirementCount,
   VkSparseImageMemoryRequirements2 *pSparseMemoryRequirements)
{
   struct vn_device *dev = vn_device_from_handle(device);

   /* see vn_GetPhysicalDeviceSparseImageFormatProperties2 */
   if (dev->physical_device->sparse_binding_disabled) {
      *pSparseMemoryRequirementCount = 0;
      return;
   }

   /* TODO per-device cache */
   vn_call_vkGetDeviceImageSparseMemoryRequirements(
      dev->primary_ring, device, pInfo, pSparseMemoryRequirementCount,
      pSparseMemoryRequirements);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetDeviceImageSubresourceLayout(VkDevice device,
                                   const VkDeviceImageSubresourceInfo *pInfo,
                                   VkSubresourceLayout2 *pLayout)
{
   struct vn_device *dev = vn_device_from_handle(device);

   /* TODO per-device cache */
   vn_call_vkGetDeviceImageSubresourceLayout(dev->primary_ring, device, pInfo,
                                             pLayout);
}

VKAPI_ATTR void VKAPI_CALL
vn_GetImageSubresourceLayout2(VkDevice device,
                              VkImage image,
                              const VkImageSubresource2 *pSubresource,
                              VkSubresourceLayout2 *pLayout)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_image *img = vn_image_from_handle(image);

   /* override aspect mask for ahb images with tiling modifier */
   VkImageSubresource2 local_subresource;
   const VkImageAspectFlags aspect =
      vn_image_get_aspect(img, pSubresource->imageSubresource.aspectMask);
   if (aspect != pSubresource->imageSubresource.aspectMask) {
      local_subresource = *pSubresource;
      local_subresource.imageSubresource.aspectMask = aspect;
      pSubresource = &local_subresource;
   }

   /* TODO local cache */
   vn_call_vkGetImageSubresourceLayout2(dev->primary_ring, device, image,
                                        pSubresource, pLayout);
}

#if DETECT_OS_WINDOWS

/* The private presentable-image tag call. Contract and rationale:
 * src/vulkan/helios_private_wsi.h. It is deliberately NOT a registry
 * entrypoint — it never appears in vk.xml, so it cannot travel through the
 * generated dispatch table and is returned by vn_GetDeviceProcAddr by name.
 *
 * This is a gate, not a rubber stamp. CLAUDE.md rule 2 forbids a call that
 * accepts whatever it is handed and reports success; each refusal below is a
 * property the reference requires of a presentable image, checked against what
 * the ICD already knows about the image.
 */
VKAPI_ATTR VkResult VKAPI_CALL
vn_SetHeliosPresentableImageHELIOS(VkDevice device,
                                   VkImage image,
                                   uint64_t swapchainId,
                                   uint32_t imageIndex)
{
   struct vn_device *dev = vn_device_from_handle(device);
   struct vn_image *img = vn_image_from_handle(image);

   if (!img || img->base.vk.base.device != &dev->base.vk || !swapchainId ||
       vn_helios_submit_instance_mode(dev->instance) ==
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return vn_error(dev->instance, VK_ERROR_INITIALIZATION_FAILED);

   /* §10.3: a presentable image is a D3D12 committed resource imported with
    * D3D12_RESOURCE_BIT. An image created without that external handle type is
    * not one, whatever the caller believes — this is the check that makes the
    * tag mean something, and it costs nothing because vk_image already records
    * the create-time handle types. */
   if (!(img->base.vk.external_handle_types &
         VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT)) {
      vn_log(dev->instance,
             "presentable-image tag refused: image was not created with "
             "D3D12_RESOURCE_BIT (external_handle_types=0x%x)",
             img->base.vk.external_handle_types);
      return vn_error(dev->instance, VK_ERROR_INITIALIZATION_FAILED);
   }

   const bool candidate =
      imageIndex == HELIOS_PRESENTABLE_IMAGE_ALIAS_CANDIDATE;

   /* Alias creation records only its exact swapchain generation. Bind then
    * resolves that candidate to one concrete slot. No other retag or
    * candidate-to-generation transition is accepted. */
   if (candidate) {
      if (img->helios_presentable.tagged ||
          (img->helios_presentable.alias_candidate &&
           img->helios_presentable.swapchain_id != swapchainId))
         return vn_error(dev->instance, VK_ERROR_INITIALIZATION_FAILED);
      img->helios_presentable.alias_candidate = true;
      img->helios_presentable.swapchain_id = swapchainId;
      img->helios_presentable.image_index =
         HELIOS_PRESENTABLE_IMAGE_ALIAS_CANDIDATE;
      return VK_SUCCESS;
   }

   if (img->helios_presentable.alias_candidate) {
      if (img->helios_presentable.swapchain_id != swapchainId)
         return vn_error(dev->instance, VK_ERROR_INITIALIZATION_FAILED);
      img->helios_presentable.alias_candidate = false;
      img->helios_presentable.tagged = true;
      img->helios_presentable.image_index = imageIndex;
      return VK_SUCCESS;
   }

   /* One canonical image is one slot. A second tag naming a different slot
    * would make every later ownership transfer ambiguous. Re-tagging the same
    * slot is idempotent. */
   if (img->helios_presentable.tagged &&
       (img->helios_presentable.swapchain_id != swapchainId ||
        img->helios_presentable.image_index != imageIndex)) {
      vn_log(dev->instance,
             "presentable-image tag refused: image already tagged as "
             "swapchain %" PRIu64 " slot %u, re-tag as swapchain %" PRIu64
             " slot %u",
             img->helios_presentable.swapchain_id,
             img->helios_presentable.image_index, swapchainId, imageIndex);
      return vn_error(dev->instance, VK_ERROR_INITIALIZATION_FAILED);
   }

   img->helios_presentable.tagged = true;
   img->helios_presentable.alias_candidate = false;
   img->helios_presentable.swapchain_id = swapchainId;
   img->helios_presentable.image_index = imageIndex;
   return VK_SUCCESS;
}

#endif /* DETECT_OS_WINDOWS */
