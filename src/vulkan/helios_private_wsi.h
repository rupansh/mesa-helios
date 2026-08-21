/*
 * Copyright © 2026 Helios vGPU
 * SPDX-License-Identifier: MIT
 *
 * The private presentable-image tag call, shared between VK_LAYER_HELIOS_present
 * and the Helios venus ICD.
 *
 * ⛔ THIS FILE IS THE ONE DECLARATION OF THIS ABI.
 *
 * docs/retirement/OWNERSHIP.md §4 settled the analogous question for the
 * translator dispatch table with the standing directive that shared private
 * data has exactly one declaration: left to themselves, two components invent
 * two ABIs that agree until the day they do not. The same rule applies here.
 * The layer and the ICD both include this header; neither declares the name or
 * the signature locally.
 *
 * It lives in src/vulkan/ rather than in the layer's own directory because
 * docs/HELIOS_PRESENT_SYNC_RETIREMENT.md §2 item 8 forbids the ICD from
 * depending on the layer. A header both sides reach without either reaching
 * into the other is the only arrangement that satisfies both rules.
 *
 * WHAT THE CALL IS FOR (§10.7:2571-2588, lane-mesa ambiguity A4)
 *
 * The layer's swapchain images are ordinary VkImages backed by imported D3D12
 * committed resources — not VkSwapchainKHR images, because the layer owns the
 * swapchain and the ICD advertises no WSI on Windows. Without being told, the
 * ICD cannot distinguish them from any other external image, so it cannot know
 * that VK_IMAGE_LAYOUT_PRESENT_SRC_KHR is meaningful for them, and it cannot
 * validate the PRESENT_SRC_KHR -> GENERAL -> foreign-queue release and the
 * reciprocal acquire that presenting them requires.
 *
 * The layer calls this exactly once per slot, after vkBindImageMemory2 and
 * before the image is exposed through vkGetSwapchainImagesKHR. A non-SUCCESS
 * return is a hard refusal of vkCreateSwapchainKHR; the layer never
 * approximates it, and resolves it ONLY through the captured next-layer
 * vkGetDeviceProcAddr — never GetProcAddress, never a global lookup.
 *
 * swapchainId is the layer swapchain's process-unique, monotonically
 * increasing generation id. It is NOT a handle value and is never serialized
 * outside this process (§12.3:3305-3307).
 */

#ifndef HELIOS_PRIVATE_WSI_H
#define HELIOS_PRIVATE_WSI_H

#include <vulkan/vulkan.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HELIOS_SET_PRESENTABLE_IMAGE_NAME "vkSetHeliosPresentableImageHELIOS"

/* B7 uses the otherwise invalid swapchain image index as a candidate marker.
 * The fixed function signature does not change: alias creation installs the
 * generation-only candidate, and alias bind replaces it exactly once with the
 * concrete slot index. */
#define HELIOS_PRESENTABLE_IMAGE_ALIAS_CANDIDATE UINT32_MAX

typedef VkResult(VKAPI_PTR *PFN_vkSetHeliosPresentableImageHELIOS)(
   VkDevice device,
   VkImage image,
   uint64_t swapchainId,
   uint32_t imageIndex);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HELIOS_PRIVATE_WSI_H */
