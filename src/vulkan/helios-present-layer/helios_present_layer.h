/*
 * Copyright © 2026 Helios vGPU
 * SPDX-License-Identifier: MIT
 *
 * VK_LAYER_HELIOS_present — the Helios native-Vulkan WSI layer.
 *
 * Normative source: docs/HELIOS_PRESENT_SYNC_RETIREMENT.md
 *   §2 item 7   (225-259)   what the layer owns and the copy-only profile
 *   §2 item 8   (260-271)   the acyclicity rule: translators never enter here
 *   §10.3       (1121-1188) the exact image and fence import chains
 *   §10.7       (2206-2709) surface ownership, withheld extensions, dispatch
 *                           closure, device creation, singleton device group,
 *                           the alias contract, the copy-only profile table,
 *                           D3D12 object creation, C57, Acquire, Present
 *   §11.4       (3026-3045) the native Vulkan WSI arrow order
 *   §12         (3123-3128) S[i], aliases, per-slot recording objects,
 *                           Ready[i]/Release[i], the DXGI backbuffer set
 *   §12.3       (3309-3329) teardown and the C48 precondition
 *   §10.9       (2846-2880) the failure rows this layer must honour
 *
 * The layer sits ABOVE the Helios ICD and is entered only by ordinary native
 * Vulkan applications through the Vulkan loader. DXVK and vkd3d reach the ICD
 * through the private direct-dispatch entry point and never see this layer
 * (§2 item 8). The layer is therefore allowed to import d3d12.dll/dxgi.dll,
 * which the ICD DLL is not (§13.2).
 */

#ifndef HELIOS_PRESENT_LAYER_H
#define HELIOS_PRESENT_LAYER_H

#ifndef VK_USE_PLATFORM_WIN32_KHR
#define VK_USE_PLATFORM_WIN32_KHR 1
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#ifndef NOMINMAX
#define NOMINMAX 1
#endif

#include <windows.h>

#include <vulkan/vulkan.h>
#include <vulkan/vk_layer.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Layer identity                                                      */
/* ------------------------------------------------------------------ */

#define HELIOS_LAYER_NAME              "VK_LAYER_HELIOS_present"
#define HELIOS_LAYER_DESCRIPTION       "Helios vGPU native Vulkan WSI (D3D12/DXGI flip present)"
#define HELIOS_LAYER_IMPL_VERSION      1u

/* The exact copy-only profile of this generation (§10.7:2447-2487). Nothing
 * outside these constants is ever advertised, approximated, or converted. */
#define HELIOS_WSI_FORMAT              VK_FORMAT_B8G8R8A8_UNORM
#define HELIOS_WSI_COLOR_SPACE         VK_COLOR_SPACE_SRGB_NONLINEAR_KHR
#define HELIOS_WSI_PRESENT_MODE        VK_PRESENT_MODE_FIFO_KHR
#define HELIOS_WSI_MIN_IMAGE_COUNT     2u
#define HELIOS_WSI_MAX_IMAGE_COUNT     16u
#define HELIOS_WSI_MAX_ARRAY_LAYERS    1u
#define HELIOS_WSI_SUPPORTED_USAGE                                            \
   (VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |   \
    VK_IMAGE_USAGE_TRANSFER_DST_BIT)

/* ------------------------------------------------------------------ */
/* The private lower-ICD presentable-image tag call                    */
/* ------------------------------------------------------------------ */
/*
 * ⛔ Declared ONCE, in src/vulkan/helios_private_wsi.h, which the Helios ICD
 * includes too. This file used to carry its own copy of the name and the
 * signature — an ABI with two declarations, which is exactly what the standing
 * directive behind OWNERSHIP.md §4 exists to prevent. The contract, and why
 * the ICD needs to be told at all, is documented there.
 */
#include "vulkan/helios_private_wsi.h"

/* ------------------------------------------------------------------ */
/* Named refusal counters                                              */
/* ------------------------------------------------------------------ */
/*
 * CLAUDE.md rule: "Every skipped/refused path gets a named registry counter or
 * atomic — loud failure over fake success." The layer is user mode, so these
 * are process-wide atomics with a name table. They are dumped to the debugger
 * (OutputDebugStringA) and to stderr at vkDestroyInstance when
 * HELIOS_LAYER_DEBUG is set in the environment, and every individual increment
 * is traced when HELIOS_LAYER_DEBUG=trace.
 */
#define HELIOS_LAYER_COUNTERS(X)                                              \
   X(entry_manifest_mismatch)                                                 \
   X(create_instance_refused_no_link_info)                                    \
   X(create_instance_refused_alloc)                                           \
   X(create_device_refused_no_link_info)                                      \
   X(create_device_refused_api_below_13)                                      \
   X(create_device_refused_not_admitted)                                      \
   X(create_device_refused_no_canonical_queue_record)                         \
   X(create_device_refused_protected_only)                                    \
   X(create_device_refused_queue_capacity)                                    \
   X(create_device_refused_uncopyable_feature_chain)                          \
   X(create_device_refused_device_group)                                      \
   X(create_device_refused_alloc)                                             \
   X(create_device_refused_lower_failed)                                      \
   X(create_device_refused_missing_device_proc)                               \
   X(getqueue_private_index_withheld)                                         \
   X(physdev_refused_api_below_13)                                            \
   X(physdev_refused_no_external_image_import)                                \
   X(physdev_refused_no_external_image_dedicated)                             \
   X(physdev_refused_no_external_semaphore_export)                            \
   X(physdev_refused_no_canonical_family)                                     \
   X(physdev_refused_queue_capacity)                                          \
   X(physdev_refused_no_luid)                                                 \
   X(physdev_refused_no_dxgi_adapter)                                         \
   X(physdev_refused_not_singleton_group)                                     \
   X(surface_create_refused_no_wsi)                                           \
   X(surface_create_refused_bad_hwnd)                                         \
   X(surface_create_refused_pnext)                                            \
   X(surface_create_refused_alloc)                                            \
   X(surface_query_refused_unknown_surface)                                   \
   X(surface_query_refused_unadvertised_pnext)                                \
   X(surface_query_surface_lost)                                              \
   X(swapchain_refused_unknown_surface)                                       \
   X(swapchain_refused_surface_lost)                                          \
   X(swapchain_refused_profile_format)                                        \
   X(swapchain_refused_profile_colorspace)                                    \
   X(swapchain_refused_profile_present_mode)                                  \
   X(swapchain_refused_profile_extent)                                        \
   X(swapchain_refused_profile_zero_extent)                                   \
   X(swapchain_refused_profile_image_count)                                   \
   X(swapchain_refused_profile_array_layers)                                  \
   X(swapchain_refused_profile_usage)                                         \
   X(swapchain_refused_profile_flags)                                         \
   X(swapchain_refused_profile_transform)                                     \
   X(swapchain_refused_profile_alpha)                                         \
   X(swapchain_refused_profile_sharing)                                       \
   X(swapchain_refused_unsupported_pnext)                                     \
   X(swapchain_refused_device_group_mode)                                     \
   X(swapchain_refused_old_swapchain)                                         \
   X(swapchain_refused_tag_call_absent)                                       \
   X(swapchain_refused_d3d12_device)                                          \
   X(swapchain_refused_dxgi_factory)                                          \
   X(swapchain_refused_dxgi_adapter)                                          \
   X(swapchain_refused_dxgi_queue)                                            \
   X(swapchain_refused_dxgi_swapchain)                                        \
   X(swapchain_refused_dxgi_backbuffer)                                       \
   X(swapchain_refused_colorspace_support)                                    \
   X(swapchain_refused_shared_resource)                                       \
   X(swapchain_refused_shared_handle)                                         \
   X(swapchain_refused_semaphore_export)                                      \
   X(swapchain_refused_fence_open)                                            \
   X(swapchain_refused_command_objects)                                       \
   X(swapchain_refused_image_create)                                          \
   X(swapchain_refused_memory_properties)                                     \
   X(swapchain_refused_memory_type)                                           \
   X(swapchain_refused_memory_import)                                         \
   X(swapchain_refused_memory_bind)                                           \
   X(swapchain_refused_semaphore_create)                                      \
   X(swapchain_refused_vk_command_objects)                                    \
   X(swapchain_refused_alloc)                                                 \
   X(acquire_refused_unknown_swapchain)                                       \
   X(acquire_refused_lost)                                                    \
   X(acquire_refused_out_of_date)                                             \
   X(acquire_refused_surface_lost)                                            \
   X(acquire_refused_device_mask)                                             \
   X(acquire_refused_epoch_overflow)                                          \
   X(acquire_refused_submit_failed)                                           \
   X(acquire_not_ready)                                                       \
   X(acquire_timeout)                                                         \
   X(present_refused_queue_not_app_visible)                                   \
   X(present_refused_unknown_swapchain)                                       \
   X(present_refused_image_not_acquired)                                      \
   X(present_refused_unsupported_pnext)                                       \
   X(present_refused_device_group_mode)                                       \
   X(present_refused_device_group_mask)                                       \
   X(present_refused_device_group_count)                                      \
   X(present_refused_out_of_date)                                             \
   X(present_refused_surface_lost)                                            \
   X(present_semaphore_kind_unverified)                                       \
   X(present_submit_failed)                                                   \
   X(present_copy_failed)                                                     \
   X(present_device_lost)                                                     \
   X(present_dxgi_out_of_date)                                                \
   X(teardown_release_wait_failed)                                            \
   X(teardown_barrier_failed)                                                 \
   X(alias_image_refused_unimplemented)                                       \
   X(alias_bind_refused_unimplemented)                                        \
   X(alias_null_form_forwarded)

typedef enum helios_layer_counter_id {
#define HELIOS_COUNTER_ENUM(name) HELIOS_CNT_##name,
   HELIOS_LAYER_COUNTERS(HELIOS_COUNTER_ENUM)
#undef HELIOS_COUNTER_ENUM
      HELIOS_CNT_COUNT
} helios_layer_counter_id;

/* ------------------------------------------------------------------ */
/* Loader-facing exports                                               */
/* ------------------------------------------------------------------ */

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_NegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface *pVersionStruct);

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
helios_layer_GetInstanceProcAddr(VkInstance instance, const char *pName);

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
helios_layer_GetDeviceProcAddr(VkDevice device, const char *pName);

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
helios_layer_GetPhysicalDeviceProcAddr(VkInstance instance, const char *pName);

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateInstanceLayerProperties(uint32_t *pCount,
                                              VkLayerProperties *pProps);

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateInstanceExtensionProperties(const char *pLayerName,
                                                  uint32_t *pCount,
                                                  VkExtensionProperties *pProps);

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateDeviceLayerProperties(VkPhysicalDevice physicalDevice,
                                            uint32_t *pCount,
                                            VkLayerProperties *pProps);

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateDeviceExtensionProperties(VkPhysicalDevice physicalDevice,
                                                const char *pLayerName,
                                                uint32_t *pCount,
                                                VkExtensionProperties *pProps);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HELIOS_PRESENT_LAYER_H */
