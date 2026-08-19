/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#include "vn_icd.h"

#include "vn_instance.h"

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
vk_icdGetInstanceProcAddr(VkInstance instance, const char *pName)
{
#if defined(_WIN32)
   /* The A5 entry point is resolved on the already-held ICD module.  Vending
    * it through the Vulkan loader would turn the loader into a second route
    * and defeat the module-provenance boundary. */
   if (pName && !strcmp(pName, HELIOS_ICD_CREATE_TRANSLATOR_V1_NAME))
      return NULL;
#endif
   return vn_GetInstanceProcAddr(instance, pName);
}

bool
vn_icd_supports_api_version(uint32_t api_version)
{
   return vk_get_negotiated_icd_version() >= 5 ||
          api_version < VK_API_VERSION_1_1;
}
