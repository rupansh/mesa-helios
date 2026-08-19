/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#ifndef VN_PHYSICAL_DEVICE_H
#define VN_PHYSICAL_DEVICE_H

#include "vn_common.h"

#include "util/sparse_array.h"

#include "vn_descriptor.h"
#include "vn_wsi.h"

struct vn_format_properties_entry {
   atomic_bool valid;
   VkFormatProperties props;
   VkFormatProperties3 props3;
   VkBool32 srpq;
};

struct vn_image_format_properties {
   VkImageFormatProperties2 format;
   VkResult cached_result;

   VkExternalImageFormatProperties ext_image;
   VkHostImageCopyDevicePerformanceQuery host_copy;
   VkImageCompressionPropertiesEXT compression;
   VkSamplerYcbcrConversionImageFormatProperties ycbcr_conversion;
   VkFilterCubicImageViewImageFormatPropertiesEXT filter_cubic;
};

struct vn_image_format_cache_entry {
   struct vn_image_format_properties properties;
   uint8_t key[BLAKE3_KEY_LEN];
   struct list_head head;
};

struct vn_image_format_properties_cache {
   struct hash_table *ht;
   struct list_head lru;
   simple_mtx_t mutex;

   struct {
      uint32_t cache_hit_count;
      uint32_t cache_miss_count;
      uint32_t cache_skip_count;
   } debug;
};

struct vn_layered_api_properties {
   VkPhysicalDeviceLayeredApiPropertiesKHR api;
   VkPhysicalDeviceLayeredApiVulkanPropertiesKHR vk;
   VkPhysicalDeviceDriverProperties driver;
   VkPhysicalDeviceIDProperties id;
};

struct vn_physical_device {
   struct vn_physical_device_base base;

   struct vn_instance *instance;

   /* Between the driver and the app, properties.properties.apiVersion is what
    * we advertise and is capped by VN_MAX_API_VERSION and others.
    *
    * Between the driver and the renderer, renderer_version is the device
    * version we can use internally.
    */
   uint32_t renderer_version;

   /* For maintenance7 layered api properties. */
   struct vn_layered_api_properties layered_properties;

   /* Between the driver and the app, base.base.supported_extensions is what
    * we advertise.
    *
    * Between the driver and the renderer, renderer_extensions is what we can
    * use internally (after enabling).
    */
   struct vk_device_extension_table renderer_extensions;
   uint32_t *extension_spec_versions;

   /* passthrough ray tracing support */
   bool ray_tracing;

   /* Venus feedback encounters cacheline overflush issue on Intel JSL, and
    * has to workaround by further aligning up the feedback buffer alignment.
    */
   uint32_t wa_min_fb_align;

   VkDriverId renderer_driver_id;
   uint32_t renderer_driver_version;

   /* Static storage so that host copy properties query can be done once. */
   VkImageLayout copy_src_layouts[64];
   VkImageLayout copy_dst_layouts[64];

   VkQueueFamilyProperties2 *queue_family_properties;
   VkQueueFamilyGlobalPriorityProperties *global_priority_properties;
   uint32_t queue_family_count;
   bool sparse_binding_disabled;
   /* Track the queue family index to emulate a second queue. -1 means no
    * emulation is needed. To be noted that the emulation is a workaround for
    * Android 14+ UI framework and it does not handle wait-before-signal.
    */
   int emulate_second_queue;

   VkPhysicalDeviceMemoryProperties memory_properties;
#if DETECT_OS_WINDOWS
   /* A6 exposes two guest HLM1 roles, but the renderer still consumes its own
    * VkMemoryType indices.  Keep the exact per-physical-device translation;
    * never reinterpret a guest index as a renderer index. */
   uint32_t helios_renderer_memory_type_indices[2];
#endif

   struct {
      VkExternalMemoryHandleTypeFlagBits renderer_handle_type;
      /* Native Win32 opaque sharing is an object-identity contract, not a
       * scanout contract.  Keep its renderer-side handle independent from
       * renderer_handle_type, which may be DMA_BUF for WSI. */
      VkExternalMemoryHandleTypeFlagBits win32_renderer_handle_type;
      VkExternalMemoryHandleTypeFlags supported_handle_types;
   } external_memory;

   struct {
      bool fence_exportable;
      bool semaphore_exportable;
      bool semaphore_importable;
   } renderer_sync_fd;

   VkExternalFenceHandleTypeFlags external_fence_handles;
   VkExternalSemaphoreHandleTypeFlags external_binary_semaphore_handles;
   VkExternalSemaphoreHandleTypeFlags external_timeline_semaphore_handles;

   struct wsi_device wsi_device;

   simple_mtx_t mutex;
   struct util_sparse_array format_properties;

   struct vn_image_format_properties_cache image_format_cache;

   bool descriptor_sizes_initialized;
   VkDeviceSize descriptor_sizes[VN_NUM_DESCRIPTOR_TYPES];
};
VK_DEFINE_HANDLE_CASTS(vn_physical_device,
                       base.vk.base,
                       VkPhysicalDevice,
                       VK_OBJECT_TYPE_PHYSICAL_DEVICE)

#define VN_HELIOS_MEMORY_TYPE_DEVICE_LOCAL 0u
#define VN_HELIOS_MEMORY_TYPE_HOST_VISIBLE 1u
#define VN_HELIOS_MEMORY_TYPE_COUNT        2u

static inline uint32_t
vn_physical_device_renderer_memory_type_index(
   const struct vn_physical_device *physical_dev,
   uint32_t guest_memory_type_index)
{
#if DETECT_OS_WINDOWS
   assert(guest_memory_type_index < VN_HELIOS_MEMORY_TYPE_COUNT);
   return physical_dev
      ->helios_renderer_memory_type_indices[guest_memory_type_index];
#else
   (void)physical_dev;
   return guest_memory_type_index;
#endif
}

static inline uint32_t
vn_physical_device_guest_memory_type_bits(
   const struct vn_physical_device *physical_dev,
   uint32_t renderer_memory_type_bits)
{
#if DETECT_OS_WINDOWS
   uint32_t guest_bits = 0;
   for (uint32_t guest = 0; guest < VN_HELIOS_MEMORY_TYPE_COUNT; guest++) {
      const uint32_t renderer =
         physical_dev->helios_renderer_memory_type_indices[guest];
      if (renderer < VK_MAX_MEMORY_TYPES &&
          (renderer_memory_type_bits & (UINT32_C(1) << renderer)))
         guest_bits |= UINT32_C(1) << guest;
   }
   return guest_bits;
#else
   (void)physical_dev;
   return renderer_memory_type_bits;
#endif
}

static inline void
vn_physical_device_sanitize_memory_requirements(
   const struct vn_physical_device *physical_dev,
   VkMemoryRequirements *requirements)
{
   requirements->memoryTypeBits = vn_physical_device_guest_memory_type_bits(
      physical_dev, requirements->memoryTypeBits);
}

/*
 * Helios transports fd-based Vulkan external-memory handle types over the
 * Venus wire; no POSIX fd crosses into the Windows guest.  An explicit
 * DMA_BUF request therefore names the handle type that the renderer must use
 * for both the VkImage and its VkDeviceMemory.  Rewriting only the image to
 * renderer_handle_type produces an invalid mixed external-memory contract and
 * can turn the exported blob into an opaque device fd.
 *
 * Keep this based exclusively on the caller-provided Vulkan handle type.  In
 * particular, image tiling, dimensions, usage, process, and creation order are
 * not resource-identity signals.
 */
static inline bool
vn_preserve_explicit_dmabuf_handle_types(
   const struct vn_physical_device *physical_dev,
   const VkExternalMemoryHandleTypeFlags handle_types)
{
#if DETECT_OS_WINDOWS
   return handle_types == VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT &&
          (physical_dev->external_memory.supported_handle_types &
           VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT);
#else
   (void)physical_dev;
   (void)handle_types;
   return false;
#endif
}

static inline VkExternalMemoryHandleTypeFlagBits
vn_renderer_handle_type_for_guest(
   const struct vn_physical_device *physical_dev,
   const VkExternalMemoryHandleTypeFlags guest_handle_types)
{
#if DETECT_OS_WINDOWS
   /* Both native Win32 payload types resolve to the same renderer handle: they
    * differ only in how the GUEST names the object (a payload this ICD
    * exported vs a helios_umd12 committed resource), and the same WDDM
    * allocation carries the same venus resource either way. */
   if ((guest_handle_types ==
           VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT ||
        guest_handle_types ==
           VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT) &&
       physical_dev->external_memory.win32_renderer_handle_type) {
      return physical_dev->external_memory.win32_renderer_handle_type;
   }
#else
   (void)guest_handle_types;
#endif
   return physical_dev->external_memory.renderer_handle_type;
}

void
vn_physical_device_fini(struct vn_physical_device *physical_dev);

static inline bool
vn_queue_family_can_feedback(struct vn_physical_device *physical_dev,
                             uint32_t queue_family_index)
{
   /* Feedback requires transfer capability, so we must skip feedback cmd pool
    * initialization on incompatible queue families. Meanwhile, rely on the
    * pool_handle for all validity check needed.
    */
   assert(queue_family_index < physical_dev->queue_family_count);
   const struct VkQueueFamilyProperties2 *props =
      &physical_dev->queue_family_properties[queue_family_index];
   const VkQueueFlags transfer_flags =
      VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT;
   return props->queueFamilyProperties.queueFlags & transfer_flags;
}

#endif /* VN_PHYSICAL_DEVICE_H */
