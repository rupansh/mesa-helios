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
