/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A4 -- mode-owned queue recording and normal HNR2 submission.
 *
 * The public Vulkan queue entry points call the four vn_helios_queue_* seams
 * below on Windows.  NORMAL owns one nonzero HVC1 context per real VkQueue;
 * RECORD_ONLY owns no queue context and may append only to the exact live
 * outer scope installed through the direct object passed below.  A5 will
 * expose those direct objects through helios_translator_dispatch.h; it does
 * not get to add another submission path.
 */

#ifndef VN_HELIOS_RECORD_SUBMIT_H
#define VN_HELIOS_RECORD_SUBMIT_H

#ifdef _WIN32

#include "helios_translator_dispatch.h"

struct vn_device_memory;
#include <stdbool.h>
#include <stdint.h>
#include <vulkan/vulkan.h>

struct vn_device;
struct vn_instance;
struct vn_queue;
struct vn_buffer;
struct vn_image;
struct vn_image_view;
struct vn_command_buffer;
struct vn_query_pool;
struct vn_helios_memory_binding;
struct vn_helios_record_context;

enum vn_helios_submission_mode {
   VN_HELIOS_SUBMISSION_MODE_NORMAL = 0,
   VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY =
      HELIOS_TRANSLATOR_SUBMISSION_MODE_RECORD_ONLY,
};

/* Per-vn_instance ownership.  Ordinary loader creation initializes NORMAL.
 * A5 may switch a freshly created, queue-less instance exactly once to
 * RECORD_ONLY through this direct pointer; there is no environment, process,
 * name, module, or global discovery path. */
VkResult
vn_helios_submit_instance_init(struct vn_instance *instance);
void
vn_helios_submit_instance_fini(struct vn_instance *instance);
VkResult
vn_helios_submit_instance_set_record_only(struct vn_instance *instance);
enum vn_helios_submission_mode
vn_helios_submit_instance_mode(const struct vn_instance *instance);

/* Queue lifecycle.  An emulated queue retains the exact owner's context and
 * never creates or destroys a second one. */
VkResult
vn_helios_submit_queue_init(struct vn_device *dev,
                            struct vn_queue *queue,
                            uint32_t queue_family,
                            uint32_t queue_index,
                            struct vn_queue *shared_queue);
void
vn_helios_submit_queue_fini(struct vn_queue *queue);

/* Mode-dispatched queue entry points. */
VkResult
vn_helios_queue_submit(struct vn_queue *queue,
                       uint32_t submit_count,
                       const VkSubmitInfo *submits,
                       VkFence fence);
VkResult
vn_helios_queue_submit2(struct vn_queue *queue,
                        uint32_t submit_count,
                        const VkSubmitInfo2 *submits,
                        VkFence fence);
VkResult
vn_helios_queue_bind_sparse(struct vn_queue *queue,
                            uint32_t bind_count,
                            const VkBindSparseInfo *binds,
                            VkFence fence);
VkResult
vn_helios_queue_wait_idle(struct vn_queue *queue);
VkResult
vn_helios_device_wait_idle(struct vn_device *dev);

/* A4's internal record-only half.  A5 will own lookup-free attachment and
 * merely pass the exact queue/context facts into these functions. */
HeliosTranslatorStatusCode
vn_helios_record_context_create(struct vn_instance *instance,
                                struct vn_queue *queue,
                                uint32_t endpoint_id,
                                uint64_t context_generation,
                                uint32_t context_flags,
                                struct vn_helios_record_context **out_context);
HeliosTranslatorStatusCode
vn_helios_record_context_destroy(struct vn_helios_record_context *context);
HeliosTranslatorStatusCode
vn_helios_record_context_can_destroy(
   struct vn_helios_record_context *context);
HeliosTranslatorStatusCode
vn_helios_record_scope_open(struct vn_helios_record_context *context,
                            HeliosTranslatorScope *out_scope);
HeliosTranslatorStatusCode
vn_helios_record_scope_seal(HeliosTranslatorScope scope,
                            HeliosSealedBatchV1 *out_sealed);
HeliosTranslatorStatusCode
vn_helios_record_scope_copy(HeliosTranslatorScope scope,
                            const HeliosSealedBatchCopyV1 *destination);
HeliosTranslatorStatusCode
vn_helios_record_scope_close(HeliosTranslatorScope scope,
                             const HeliosOuterScopeCloseV1 *close);
bool
vn_helios_record_scope_identity(HeliosTranslatorScope scope,
                                struct vn_instance **out_instance,
                                uint64_t *out_context_generation);
bool
vn_helios_record_current_context(struct vn_instance *instance,
                                 uint64_t *out_context_generation);
void
vn_helios_record_query_refusals(
   const struct vn_instance *instance,
   HeliosTranslatorRefusalCountersV1 *out_counters);
void
vn_helios_record_note_loader_provenance(struct vn_instance *instance);
void
vn_helios_record_note_foreign_handle(struct vn_instance *instance);
void
vn_helios_record_note_withheld_proc(struct vn_instance *instance);
void
vn_helios_record_note_reentrant_join(struct vn_instance *instance);
void
vn_helios_record_note_deferred_use(struct vn_instance *instance);

bool
vn_helios_record_has_active_scope(struct vn_instance *instance);

/* Finish one allocation-backed lifetime entirely inside the calling thread's
 * current outer scope.  The payload consists only of immutable deferred
 * allocation/bind/destroy/free records already owned by `mem`; the sealed use
 * names that exact allocation at offset zero. */
VkResult
vn_helios_record_memory_teardown(struct vn_device *dev,
                                 struct vn_device_memory *mem);

/* A7 command/object classifier.  These helpers never submit.  They append to
 * the exact command buffer's bounded immutable closure, or make that closure
 * permanently incomplete with the first generated opcode that was refused. */
void
vn_helios_cmd_closure_begin(struct vn_command_buffer *cmd);
void
vn_helios_cmd_closure_fini(struct vn_command_buffer *cmd);
void
vn_helios_cmd_refuse(struct vn_command_buffer *cmd, uint32_t opcode);
void
vn_helios_cmd_touch_buffer(struct vn_command_buffer *cmd,
                           VkBuffer buffer,
                           VkDeviceSize offset,
                           VkDeviceSize size,
                           uint32_t access_flags,
                           uint32_t opcode);
void
vn_helios_cmd_touch_image(struct vn_command_buffer *cmd,
                          VkImage image,
                          uint32_t access_flags,
                          uint32_t opcode);
void
vn_helios_cmd_touch_image_view(struct vn_command_buffer *cmd,
                               VkImageView image_view,
                               uint32_t access_flags,
                               uint32_t opcode);
void
vn_helios_cmd_touch_device_address(struct vn_command_buffer *cmd,
                                   VkDeviceAddress address,
                                   VkDeviceSize size,
                                   uint32_t access_flags,
                                   uint32_t opcode);
void
vn_helios_cmd_merge_secondary(struct vn_command_buffer *primary,
                              const struct vn_command_buffer *secondary,
                              uint32_t opcode);
void
vn_helios_cmd_touch_descriptor_set(struct vn_command_buffer *cmd,
                                   VkDescriptorSet descriptor_set,
                                   uint32_t opcode);
void
vn_helios_cmd_touch_descriptor_writes(
   struct vn_command_buffer *cmd,
   uint32_t write_count,
   const VkWriteDescriptorSet *writes,
   uint32_t opcode);
void
vn_helios_cmd_touch_event(struct vn_command_buffer *cmd,
                          VkEvent event,
                          uint32_t opcode);

/* Object-local outer progress.  Query/fence/semaphore synchronous entry
 * points consume these facts through the direct A5 join/query callbacks. */
bool
vn_helios_query_pool_progress(struct vn_query_pool *pool,
                              uint64_t *context_generation,
                              uint64_t *progress_value);

#endif /* _WIN32 */
#endif /* VN_HELIOS_RECORD_SUBMIT_H */
