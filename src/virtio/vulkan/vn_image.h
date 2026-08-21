/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#ifndef VN_IMAGE_H
#define VN_IMAGE_H

#include "vn_common.h"
#if DETECT_OS_WINDOWS
#include "vn_device_memory.h"
#endif

#if DETECT_OS_WINDOWS
#include "vulkan/helios_private_wsi.h"
#endif

/* changing this to VK_IMAGE_LAYOUT_PRESENT_SRC_KHR disables ownership
 * transfers and can be useful for debugging
 */
#define VN_PRESENT_SRC_INTERNAL_LAYOUT VK_IMAGE_LAYOUT_GENERAL

struct vn_image_memory_requirements {
   VkMemoryRequirements2 memory;
   VkMemoryDedicatedRequirements dedicated;
};

struct vn_image_reqs_cache_entry {
   struct vn_image_memory_requirements requirements[4];
   uint8_t plane_count;
   uint8_t key[BLAKE3_KEY_LEN];
   struct list_head head;
};

struct vn_image_reqs_cache {
   struct hash_table *ht;
   struct list_head lru;
   simple_mtx_t mutex;

   struct {
      uint32_t cache_hit_count;
      uint32_t cache_miss_count;
      uint32_t cache_skip_count;
   } debug;
};

struct vn_image {
   struct vn_image_base base;

   struct vn_image_memory_requirements requirements[4];

   /* track whether the image init has been deferred and whether the deferred
    * init has succeeded (renderer side image created)
    */
   bool deferred;
   bool deferred_initialized;

   struct {
      bool is_prime_blit_src;

      /* memory backing the prime blit dst buffer */
      struct vn_device_memory *blit_mem;
   } wsi;

#if DETECT_OS_WINDOWS
   struct vn_helios_memory_binding helios_bindings[4];

   /* The presentable-image tag set by VK_LAYER_HELIOS_present through
    * vkSetHeliosPresentableImageHELIOS (src/vulkan/helios_private_wsi.h).
    * `tagged` is what makes VK_IMAGE_LAYOUT_PRESENT_SRC_KHR meaningful for an
    * image that is not a VkSwapchainKHR image, and the id/index are what let a
    * misuse be reported against a specific slot rather than "some image". */
   struct {
      bool tagged;
      bool alias_candidate;
      uint64_t swapchain_id;
      uint32_t image_index;
   } helios_presentable;
#endif
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_image,
                               base.vk.base,
                               VkImage,
                               VK_OBJECT_TYPE_IMAGE)

#if DETECT_OS_WINDOWS
/* Implemented in vn_image.c; published only through vn_GetDeviceProcAddr. */
VKAPI_ATTR VkResult VKAPI_CALL
vn_SetHeliosPresentableImageHELIOS(VkDevice device,
                                   VkImage image,
                                   uint64_t swapchainId,
                                   uint32_t imageIndex);
#endif

struct vn_image_view {
   struct vn_object_base base;

   const struct vn_image *image;
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_image_view,
                               base.vk,
                               VkImageView,
                               VK_OBJECT_TYPE_IMAGE_VIEW)

struct vn_sampler {
   struct vn_object_base base;
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_sampler,
                               base.vk,
                               VkSampler,
                               VK_OBJECT_TYPE_SAMPLER)

struct vn_sampler_ycbcr_conversion {
   struct vn_object_base base;
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_sampler_ycbcr_conversion,
                               base.vk,
                               VkSamplerYcbcrConversion,
                               VK_OBJECT_TYPE_SAMPLER_YCBCR_CONVERSION)

VkResult
vn_image_create(struct vn_device *dev,
                const VkImageCreateInfo *create_info,
                const VkAllocationCallbacks *alloc,
                struct vn_image **out_img);

VkResult
vn_image_init_deferred(struct vn_device *dev,
                       const VkImageCreateInfo *create_info,
                       struct vn_image *img);

void
vn_image_reqs_cache_init(struct vn_device *dev);

void
vn_image_reqs_cache_fini(struct vn_device *dev);

#endif /* VN_IMAGE_H */
