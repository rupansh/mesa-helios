/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * Fail-closed Win32 WSI seam while the replacement present layer remains a
 * separate, unstarted lane.  This file intentionally creates no surface or
 * swapchain backend and imports no DXGI, D3D, DComp, or loader thunk.
 */

#include "wsi_common_entrypoints.h"
#include "wsi_common_private.h"

#include "vk_instance.h"

bool
wsi_helios_vehicle_enabled(void)
{
   return false;
}

enum wsi_swapchain_blit_type
wsi_dxgi_image_needs_blit(const struct wsi_device *wsi,
                          const struct wsi_dxgi_image_params *params,
                          VkDevice device)
{
   (void)wsi;
   (void)params;
   (void)device;
   return WSI_SWAPCHAIN_NO_BLIT;
}

VkResult
wsi_dxgi_configure_image(const struct wsi_swapchain *chain,
                         const VkSwapchainCreateInfoKHR *pCreateInfo,
                         const struct wsi_dxgi_image_params *params,
                         struct wsi_image_info *info)
{
   (void)chain;
   (void)pCreateInfo;
   (void)params;
   (void)info;
   return VK_ERROR_EXTENSION_NOT_PRESENT;
}

VKAPI_ATTR VkResult VKAPI_CALL
wsi_CreateWin32SurfaceKHR(VkInstance _instance,
                          const VkWin32SurfaceCreateInfoKHR *pCreateInfo,
                          const VkAllocationCallbacks *pAllocator,
                          VkSurfaceKHR *pSurface)
{
   (void)_instance;
   (void)pCreateInfo;
   (void)pAllocator;
   if (pSurface)
      *pSurface = VK_NULL_HANDLE;
   return VK_ERROR_EXTENSION_NOT_PRESENT;
}

VKAPI_ATTR VkBool32 VKAPI_CALL
wsi_GetPhysicalDeviceWin32PresentationSupportKHR(
   VkPhysicalDevice physicalDevice, uint32_t queueFamilyIndex)
{
   (void)physicalDevice;
   (void)queueFamilyIndex;
   return VK_FALSE;
}

void
wsi_win32_surface_destroy(VkIcdSurfaceBase *icd_surface,
                          VkInstance _instance,
                          const VkAllocationCallbacks *pAllocator)
{
   (void)icd_surface;
   (void)_instance;
   (void)pAllocator;
}

VkResult
wsi_win32_init_wsi(struct wsi_device *wsi_device,
                   const VkAllocationCallbacks *alloc,
                   VkPhysicalDevice physical_device)
{
   (void)alloc;
   (void)physical_device;
   wsi_device->wsi[VK_ICD_WSI_PLATFORM_WIN32] = NULL;
   return VK_SUCCESS;
}

void
wsi_win32_finish_wsi(struct wsi_device *wsi_device,
                     const VkAllocationCallbacks *alloc)
{
   (void)alloc;
   wsi_device->wsi[VK_ICD_WSI_PLATFORM_WIN32] = NULL;
}
