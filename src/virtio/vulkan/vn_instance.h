/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#ifndef VN_INSTANCE_H
#define VN_INSTANCE_H

#include "vn_common.h"

#include "vn_renderer_util.h"

/* require and request at least Vulkan 1.1 at both instance and device levels
 */
#define VN_MIN_RENDERER_VERSION VK_API_VERSION_1_1

/* max advertised version at both instance and device levels */
#if !defined(ANDROID_STRICT) || ANDROID_API_LEVEL >= 36
#define VN_MAX_API_VERSION VK_MAKE_VERSION(1, 4, VK_HEADER_VERSION)
#elif ANDROID_API_LEVEL >= 33
#define VN_MAX_API_VERSION VK_MAKE_VERSION(1, 3, VK_HEADER_VERSION)
#else
#define VN_MAX_API_VERSION VK_MAKE_VERSION(1, 1, VK_HEADER_VERSION)
#endif

struct vn_instance {
   struct vn_instance_base base;

   struct driOptionCache dri_options;
   struct driOptionCache available_dri_options;
   bool enable_wsi_multi_plane_modifiers;

   struct vn_renderer *renderer;

   /* for VN_CS_ENCODER_STORAGE_SHMEM_POOL */
   struct vn_renderer_shmem_pool cs_shmem_pool;

   struct vn_renderer_shmem_pool reply_shmem_pool;

   mtx_t ring_idx_mutex;
   uint64_t ring_idx_used_mask;

   struct {
      struct vn_ring *ring;
      struct list_head tls_rings;

      struct vn_watchdog watchdog;
   } ring;

   /* Between the driver and the app, VN_MAX_API_VERSION is what we advertise
    * and base.base.app_info.api_version is what the app requests.
    *
    * Between the driver and the renderer, renderer_api_version is the api
    * version we request internally, which can be higher than
    * base.base.app_info.api_version.  renderer_version is the instance
    * version we can use internally.
    */
   uint32_t renderer_api_version;
   uint32_t renderer_version;

   bool engine_is_zink;

#if defined(_WIN32)
   /* A5 direct creation passes the adapter and requested HTS1 capacity through
    * the object being constructed.  These are construction facts, never a
    * process-global selector or a post-create lookup key. */
   bool helios_direct_requested;
   uint32_t helios_direct_adapter_luid_low;
   int32_t helios_direct_adapter_luid_high;
   uint32_t helios_direct_endpoint_capacity;
   int32_t helios_direct_create_status;

   /* Non-forgeable A5 ownership tag.  Only the private direct constructor
    * installs it, and every down-call cross-checks it against this instance. */
   struct HeliosTranslatorInstance_T *helios_direct;

   /* A4 owns the submission mode and thread-current outer-scope key here.
    * It is a direct child of this instance, never process-global state. */
   struct vn_helios_submit_instance *helios_submit;

   /* K11 endpoints are 1..64 and are never recycled within a session.
    * Endpoint 1 belongs to the renderer's allocation bootstrap, leaving
    * 2..64 for real queues.  The selected Windows backend mirrors that exact
    * monotonic KMD assignment instead of using the generic reuse bitmap. */
   uint32_t helios_next_ring_idx;
#endif

   struct {
      mtx_t mutex;
      bool initialized;

      struct vn_physical_device *devices;
      uint32_t device_count;
      VkPhysicalDeviceGroupProperties *groups;
      uint32_t group_count;
   } physical_device;
};
VK_DEFINE_HANDLE_CASTS(vn_instance,
                       base.vk.base,
                       VkInstance,
                       VK_OBJECT_TYPE_INSTANCE)

static inline struct vn_renderer_shmem *
vn_instance_cs_shmem_alloc(struct vn_instance *instance,
                           size_t size,
                           size_t *out_offset)
{
   return vn_renderer_shmem_pool_alloc(
      instance->renderer, &instance->cs_shmem_pool, size, out_offset);
}

static inline struct vn_renderer_shmem *
vn_instance_reply_shmem_alloc(struct vn_instance *instance,
                              size_t size,
                              size_t *out_offset)
{
   return vn_renderer_shmem_pool_alloc(
      instance->renderer, &instance->reply_shmem_pool, size, out_offset);
}

static inline int
vn_instance_acquire_ring_idx(struct vn_instance *instance)
{
   mtx_lock(&instance->ring_idx_mutex);
#if defined(_WIN32)
   const uint32_t next = instance->helios_next_ring_idx;
   int ring_idx = next && next < instance->renderer->info.max_timeline_count
                     ? (int)next
                     : -1;
   if (ring_idx > 0)
      instance->helios_next_ring_idx = next + 1;
#else
   int ring_idx = ffsll(~instance->ring_idx_used_mask) - 1;
   if (ring_idx >= instance->renderer->info.max_timeline_count)
      ring_idx = -1;
   if (ring_idx > 0)
      instance->ring_idx_used_mask |= (1ULL << (uint32_t)ring_idx);
#endif
   mtx_unlock(&instance->ring_idx_mutex);

   assert(ring_idx); /* never acquire the dedicated CPU ring */

   /* returns -1 when no vacant rings */
   return ring_idx;
}

static inline void
vn_instance_release_ring_idx(struct vn_instance *instance, uint32_t ring_idx)
{
   assert(ring_idx > 0);

#if defined(_WIN32)
   /* Deliberately consumed until vn_instance/session teardown.  K11 applies
    * the same rule, so a late host completion can never target a new queue. */
   (void)instance;
#else
   mtx_lock(&instance->ring_idx_mutex);
   assert(instance->ring_idx_used_mask & (1ULL << ring_idx));
   instance->ring_idx_used_mask &= ~(1ULL << ring_idx);
   mtx_unlock(&instance->ring_idx_mutex);
#endif
}

#if defined(_WIN32)
/* A5's loader-free constructor.  The exact adapter/capacity travel only on
 * this stack into the instance being allocated; ordinary loader creation
 * continues through vn_CreateInstance with no hidden selector. */
VkResult
vn_helios_create_direct_instance(uint32_t adapter_luid_low,
                                 int32_t adapter_luid_high,
                                 uint32_t endpoint_capacity,
                                 VkInstance *out_instance,
                                 int32_t *out_status);
#endif

#endif /* VN_INSTANCE_H */
