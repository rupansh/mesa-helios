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

/* Internal C60 progress edges.  These are not ICD exports and do not change
 * HeliosTranslatorDispatchV1: a translated synchronous Vulkan entry point
 * names either the live scope on its calling thread or an exact context
 * generation retained on the frontend object that produced the result. */
HeliosTranslatorStatusCode
vn_helios_direct_join_current(struct vn_instance *instance,
                              uint64_t required_progress,
                              HeliosSyncProgressResultV1 *out_result);
HeliosTranslatorStatusCode
vn_helios_direct_join_context(struct vn_instance *instance,
                              uint64_t context_generation,
                              uint64_t required_progress,
                              HeliosSyncProgressResultV1 *out_result);
HeliosTranslatorStatusCode
vn_helios_direct_join_all(struct vn_instance *instance);
HeliosTranslatorStatusCode
vn_helios_direct_query_context(struct vn_instance *instance,
                               uint64_t context_generation,
                               HeliosSyncProgressResultV1 *out_result);

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
