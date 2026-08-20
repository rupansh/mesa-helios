/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#ifndef VN_DEVICE_MEMORY_H
#define VN_DEVICE_MEMORY_H

#include "vn_common.h"

#if DETECT_OS_WINDOWS
#include "helios_resource_association.h"
#include "helios_translation_session.h"

struct vn_helios_outer_progress {
   uint64_t context_generation;
   uint64_t progress_value;
};
#endif

struct vn_device_memory {
   struct vn_device_memory_base base;

   /* non-NULL when mappable or external */
   struct vn_renderer_bo *base_bo;

   /* ensure renderer side resource create is called after vkAllocateMemory
    *
    * 1. driver submits vkAllocateMemory (alloc) via ring for a ring seqno
    * 2. driver submits via vq to wait for above ring to reach the seqno
    * 3. driver creates virtgpu bo from renderer VkDeviceMemory
    *
    * ensure renderer side resource destroy is called after vkAllocateMemory
    *
    * 1. driver submits vkAllocateMemory (import) via ring for a ring seqno
    * 2. driver submits via vq to wait for above ring to reach the seqno
    * 3. driver destroys virtgpu bo
    */
   bool bo_ring_seqno_valid;
   uint32_t bo_ring_seqno;

   /* ensure renderer side vkFreeMemory is called after vkGetMemoryFdKHR
    *
    * 1. driver creates virtgpu bo from renderer VkDeviceMemory
    * 2. driver submits via vq to update the vq seqno
    * 3, driver submits via ring to wait for vq reaching above seqno
    * 4. driver submits vkFreeMemory via ring
    *
    * To be noted: a successful virtgpu mmap implies a roundtrip, so
    * vn_FreeMemory after that no longer has to wait.
    */
   bool bo_roundtrip_seqno_valid;
   uint64_t bo_roundtrip_seqno;

   VkDeviceSize map_end;
   VkDeviceSize map_start;
   bool coherent_cached_mapped;
   bool wsi_buffer_blit_dst;
   struct list_head coherent_cached_link;

#ifdef _WIN32
   /* A7: immutable package-owned association with the exact outer WDDM
    * allocation.  The token is assigned by the owning UMD, never derived from
    * this object's HVM1 allocation, Vulkan handle, or renderer resource id.
    * `helios_outer_link` is present only while the object is in its owning
    * device's bounded live set; FreeMemory removes it before any lower object
    * storage can be reused. */
   HeliosResourceAssociationV1 helios_outer;
   struct list_head helios_outer_link;
   struct list_head helios_deferred_records;
   uint32_t helios_deferred_record_count;
   struct vn_helios_outer_progress
      helios_outer_progress[HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION];
   uint32_t helios_outer_progress_count;
   bool helios_outer_registered;
   bool helios_host_materialized;
   bool helios_free_pending;
   VkAllocationCallbacks helios_free_allocator;

   /* Guest-visible export types must remain distinct from the renderer-side
    * fd/dma-buf type used to create the Venus blob. */
   VkExternalMemoryHandleTypeFlags renderer_export_handle_types;

   /* Native OPAQUE_WIN32 payload: the opened/created WDDM allocation carrying
    * the immutable §10.3 HWA2 descriptor. ⛔ It no longer "owns/retains the
    * Venus resource" — UMD-backing adoption is deleted, and no ICD-side record
    * may name a host resource (§10.3). */
   struct vn_renderer_helios_external_memory *helios_external_memory;

   /* ⛔ `helios_vidmm_{resource,allocation,global_share,cookie}` are DELETED
    * with the VidMm global-share tracker. The mechanism has no successor
    * (K4-CONTRACT §6): nothing was folded into HWA2, and the packed
    * `(cookie << 32) | global_share` attestation was exactly the
    * "independently usable identity" §10.3 forbids. Do not re-add a field
    * here to hold one. */
#endif

   /* only valid when wsi platform is used */
   struct vn_image *dedicated_img;
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_device_memory,
                               base.vk.base,
                               VkDeviceMemory,
                               VK_OBJECT_TYPE_DEVICE_MEMORY)

#if DETECT_OS_WINDOWS
struct vn_helios_memory_binding {
   uint64_t device_generation;
   uint64_t outer_allocation_token;
   uint64_t outer_allocation_bytes;
   uint64_t byte_offset;
   uint64_t byte_length;
   bool valid;
};

#define VN_HELIOS_NO_RESOURCE_OPERAND UINT32_MAX

/* One immutable generated allocation/bind command waiting for the first
 * exact outer batch that names `owner`'s token.  The mutable reservation is
 * lifecycle only: it prevents two concurrent scopes from consuming the same
 * command and is never used as resource identity. */
struct vn_helios_deferred_record {
   struct list_head link;
   struct vn_device_memory *owner;
   uint8_t *payload;
   uint32_t payload_bytes;
   uint32_t resource_operand_offset;
   uint64_t reserved_context_generation;
   uint64_t reserved_batch_id;
   bool allocation_record;
   bool final_free_record;
};

struct vn_helios_deferred_record *
vn_device_memory_helios_record_create(struct vn_device_memory *mem,
                                      void *payload,
                                      size_t payload_bytes,
                                      uint32_t resource_operand_offset,
                                      bool allocation_record,
                                      bool final_free_record);

VkResult
vn_device_memory_helios_record_install(
   struct vn_device *dev,
   struct vn_device_memory *mem,
   struct vn_helios_deferred_record *record);

VkResult
vn_device_memory_helios_records_install(
   struct vn_device *dev,
   struct vn_helios_deferred_record *const *records,
   uint32_t record_count);

void
vn_device_memory_helios_record_destroy(
   struct vn_helios_deferred_record *record);

struct vn_device_memory *
vn_device_memory_helios_binding_memory(
   struct vn_device *dev,
   const struct vn_helios_memory_binding *binding);

bool
vn_device_memory_helios_finalize_pending_free(
   struct vn_device *dev,
   struct vn_device_memory *mem);

VkResult
vn_device_memory_helios_bind(struct vn_device *dev,
                             struct vn_device_memory *mem,
                             VkDeviceSize offset,
                             VkDeviceSize length,
                             struct vn_helios_memory_binding *out);

bool
vn_device_memory_helios_binding_live(
   struct vn_device *dev,
   const struct vn_helios_memory_binding *binding);
#endif

VkResult
vn_device_memory_import_dma_buf(struct vn_device *dev,
                                struct vn_device_memory *mem,
                                const VkMemoryAllocateInfo *alloc_info,
                                int fd);

VkResult
vn_get_memory_dma_buf_properties(struct vn_device *dev,
                                 int fd,
                                 uint32_t *out_mem_type_bits);

void
vn_device_memory_flush_coherent_cached_mappings(struct vn_device *dev);

void
vn_device_memory_invalidate_coherent_cached_mappings(struct vn_device *dev);

void
vn_device_memory_cleanup_coherent_cached_mappings(struct vn_device *dev);

#endif /* VN_DEVICE_MEMORY_H */
