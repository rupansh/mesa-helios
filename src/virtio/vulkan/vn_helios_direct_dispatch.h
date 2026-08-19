/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A5 -- the one private, versioned direct-dispatch export.
 */

#ifndef VN_HELIOS_DIRECT_DISPATCH_H
#define VN_HELIOS_DIRECT_DISPATCH_H

#ifdef _WIN32

#include "helios_translator_dispatch.h"
#include <vulkan/vulkan.h>

struct vn_instance;
struct vn_queue;

/* Queue endpoints are registered by the existing A4 per-instance owner.  The
 * direct object stores them at their non-recycled endpoint index; there is no
 * registry, name lookup, PID key, or second session. */
VkResult
vn_helios_direct_register_queue(struct vn_instance *instance,
                                struct vn_queue *queue,
                                uint32_t queue_family,
                                uint32_t queue_index,
                                VkQueueFlags queue_flags);
void
vn_helios_direct_unregister_queue(struct vn_instance *instance,
                                  struct vn_queue *queue);

#endif /* _WIN32 */
#endif /* VN_HELIOS_DIRECT_DISPATCH_H */
