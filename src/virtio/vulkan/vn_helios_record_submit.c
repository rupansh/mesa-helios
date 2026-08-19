/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A4 -- record-only queue sealing and normal HNR2 queue execution.
 */

#ifdef _WIN32

#include "vn_helios_record_submit.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "venus-protocol/vn_protocol_driver_queue.h"

#include "vn_buffer.h"
#include "vn_cs.h"
#include "vn_device.h"
#include "vn_device_memory.h"
#include "vn_helios_native_kmt.h"
#include "vn_image.h"
#include "vn_instance.h"
#include "vn_queue.h"
#include "vn_renderer.h"

enum helios_record_refusal {
   HELIOS_RECORD_REFUSE_QUEUE_SUBMIT,
   HELIOS_RECORD_REFUSE_QUEUE_SUBMIT2,
   HELIOS_RECORD_REFUSE_QUEUE_BIND_SPARSE,
   HELIOS_RECORD_REFUSE_QUEUE_PRESENT,
   HELIOS_RECORD_REFUSE_QUEUE_WAIT_IDLE,
   HELIOS_RECORD_REFUSE_DEVICE_WAIT_IDLE,
   HELIOS_RECORD_REFUSE_LOADER_PROVENANCE,
   HELIOS_RECORD_REFUSE_CONTROL_CLASS,
   HELIOS_RECORD_REFUSE_DEFERRED_USE,
   HELIOS_RECORD_REFUSE_BATCH_BOUND,
   HELIOS_RECORD_REFUSE_FOREIGN_HANDLE,
   HELIOS_RECORD_REFUSE_WITHHELD_PROC,
   HELIOS_RECORD_REFUSE_REENTRANT_JOIN,
   HELIOS_RECORD_REFUSE_COUNT,
};

struct vn_helios_submit_instance {
   struct vn_instance *instance;
   enum vn_helios_submission_mode mode;
   uint64_t session_generation;
   tss_t scope_key;
   bool scope_key_live;
   mtx_t lock;
   bool lock_live;
   uint32_t live_queue_count;
   bool queue_admission_failed;
   volatile LONG64 refusals[HELIOS_RECORD_REFUSE_COUNT];
};

struct vn_helios_record_context {
   struct vn_helios_submit_instance *owner;
   struct vn_queue *queue;
   uint64_t context_generation;
   uint64_t next_batch_id;
   uint32_t endpoint_id;
   uint32_t context_flags;
   mtx_t lock;
   bool lock_live;
   struct HeliosTranslatorScope_T *active_scope;
};

struct HeliosTranslatorScope_T {
   struct vn_helios_record_context *context;
   thrd_t thread;
   uint64_t batch_id;
   uint8_t *payload;
   uint64_t payload_bytes;
   uint64_t payload_capacity;
   HeliosSealedResourceUseV1 *uses;
   uint32_t use_count;
   uint32_t use_capacity;
   HeliosSealedOperandV1 *operands;
   uint32_t operand_count;
   uint32_t operand_capacity;
   HeliosSealedBatchV1 sealed;
   HeliosTranslatorStatusCode failure;
   bool is_sealed;
};

struct helios_fence_points {
   struct helios_native_fence_point *points;
   uint32_t count;
   uint32_t capacity;
};

struct helios_queue_step {
   void *payload;
   size_t payload_bytes;
   struct helios_hnr2_allocation *allocations;
   uint32_t allocation_count;
   struct helios_fence_points waits;
   struct helios_fence_points signals;
};

static struct vn_helios_submit_instance *
helios_submit_owner(const struct vn_instance *instance)
{
   return instance ? instance->helios_submit : NULL;
}

static void
helios_record_refuse(struct vn_helios_submit_instance *owner,
                     enum helios_record_refusal refusal)
{
   if (owner && (unsigned)refusal < HELIOS_RECORD_REFUSE_COUNT)
      InterlockedIncrement64(&owner->refusals[refusal]);
}

static bool
helios_size_mul(size_t a, size_t b, size_t *out)
{
   if (a && b > SIZE_MAX / a)
      return false;
   *out = a * b;
   return true;
}

static bool
helios_fence_points_add(struct helios_fence_points *points,
                        uint32_t handle,
                        uint64_t value)
{
   if (!handle || points->count >= HELIOS_HOB1_MAX_OPERAND_RECORDS)
      return false;
   if (points->count == points->capacity) {
      uint32_t capacity = points->capacity ? points->capacity * 2u : 8u;
      if (capacity < points->capacity ||
          capacity > HELIOS_HOB1_MAX_OPERAND_RECORDS)
         capacity = HELIOS_HOB1_MAX_OPERAND_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity, sizeof(*points->points), &bytes))
         return false;
      void *new_points = realloc(points->points, bytes);
      if (!new_points)
         return false;
      points->points = new_points;
      points->capacity = capacity;
   }
   points->points[points->count++] = (struct helios_native_fence_point){
      .handle = handle,
      .value = value,
   };
   return true;
}

static void
helios_fence_points_fini(struct helios_fence_points *points)
{
   free(points->points);
   memset(points, 0, sizeof(*points));
}

static void
helios_queue_steps_fini(struct helios_queue_step *steps, uint32_t count)
{
   if (!steps)
      return;
   for (uint32_t i = 0; i < count; i++) {
      helios_fence_points_fini(&steps[i].signals);
      helios_fence_points_fini(&steps[i].waits);
      free(steps[i].allocations);
      free(steps[i].payload);
   }
   free(steps);
}

static bool
helios_queue_step_points_bounded(uint32_t *total,
                                 const struct helios_queue_step *step)
{
   const uint64_t next =
      (uint64_t)*total + step->waits.count + step->signals.count;
   if (next > HELIOS_HOB1_MAX_OPERAND_RECORDS)
      return false;
   *total = (uint32_t)next;
   return true;
}

static bool
helios_object_owned(const struct vk_object_base *object,
                    const struct vn_device *dev)
{
   return object && dev && object->device == &dev->base.vk;
}

static VkResult
helios_classify_semaphore(struct vn_device *dev,
                          VkSemaphore handle,
                          uint64_t value,
                          struct helios_fence_points *points,
                          bool *out_native)
{
   struct vn_semaphore *sem = vn_semaphore_from_handle(handle);
   if (!sem || !helios_object_owned(&sem->base.vk, dev)) {
      helios_record_refuse(helios_submit_owner(dev->instance),
                           HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (!sem->payload)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   *out_native = false;
   if (sem->payload->type != VN_SYNC_TYPE_IMPORTED_WIN32_SYNC)
      return VK_SUCCESS;
   if (!sem->payload->win32_sync)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   const uint32_t native =
      vn_renderer_helios_sync_handle(sem->payload->win32_sync);
   if (!native)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   if (!helios_fence_points_add(points, native, value))
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   *out_native = true;
   return VK_SUCCESS;
}

/* ── per-instance and queue ownership ───────────────────────────────────── */

VkResult
vn_helios_submit_instance_init(struct vn_instance *instance)
{
   if (!instance || !instance->renderer || instance->helios_submit)
      return VK_ERROR_INITIALIZATION_FAILED;

   struct vn_helios_submit_instance *owner = calloc(1, sizeof(*owner));
   if (!owner)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   owner->instance = instance;
   owner->mode = VN_HELIOS_SUBMISSION_MODE_NORMAL;
   owner->session_generation =
      vn_renderer_helios_session_generation(instance->renderer);
   if (!owner->session_generation) {
      free(owner);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   if (tss_create(&owner->scope_key, NULL) != thrd_success) {
      free(owner);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   owner->scope_key_live = true;
   if (mtx_init(&owner->lock, mtx_plain) != thrd_success) {
      tss_delete(owner->scope_key);
      free(owner);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   owner->lock_live = true;
   instance->helios_submit = owner;
   return VK_SUCCESS;
}

void
vn_helios_submit_instance_fini(struct vn_instance *instance)
{
   struct vn_helios_submit_instance *owner = helios_submit_owner(instance);
   if (!owner)
      return;
   mtx_lock(&owner->lock);
   const uint32_t live_queues = owner->live_queue_count;
   mtx_unlock(&owner->lock);
   if (live_queues)
      return;
   if (owner->scope_key_live)
      tss_delete(owner->scope_key);
   if (owner->lock_live)
      mtx_destroy(&owner->lock);
   instance->helios_submit = NULL;
   free(owner);
}

VkResult
vn_helios_submit_instance_set_record_only(struct vn_instance *instance)
{
   struct vn_helios_submit_instance *owner = helios_submit_owner(instance);
   if (!owner)
      return VK_ERROR_INITIALIZATION_FAILED;
   mtx_lock(&owner->lock);
   const bool admissible = owner->mode == VN_HELIOS_SUBMISSION_MODE_NORMAL &&
                           !owner->queue_admission_failed &&
                           owner->live_queue_count == 0 &&
                           tss_get(owner->scope_key) == NULL;
   if (admissible)
      owner->mode = VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY;
   mtx_unlock(&owner->lock);
   return admissible ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED;
}

enum vn_helios_submission_mode
vn_helios_submit_instance_mode(const struct vn_instance *instance)
{
   const struct vn_helios_submit_instance *owner =
      helios_submit_owner(instance);
   return owner ? owner->mode : VN_HELIOS_SUBMISSION_MODE_NORMAL;
}

VkResult
vn_helios_submit_queue_init(struct vn_device *dev,
                            struct vn_queue *queue,
                            uint32_t queue_family,
                            uint32_t queue_index,
                            struct vn_queue *shared_queue)
{
   if (!dev || !queue)
      return VK_ERROR_INITIALIZATION_FAILED;
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   if (!owner)
      return VK_ERROR_INITIALIZATION_FAILED;

   if (mtx_init(&queue->helios_record_mutex, mtx_plain) != thrd_success) {
      mtx_lock(&owner->lock);
      if (owner->mode == VN_HELIOS_SUBMISSION_MODE_NORMAL)
         owner->queue_admission_failed = true;
      mtx_unlock(&owner->lock);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   queue->helios_record_mutex_live = true;

   /* Snapshot the immutable mode and publish the first live queue in one
    * critical section.  Otherwise a concurrent direct-mode selection could
    * observe zero queues after this function had already selected NORMAL and
    * create a raw queue context in a RECORD_ONLY instance. */
   mtx_lock(&owner->lock);
   if (owner->queue_admission_failed) {
      mtx_unlock(&owner->lock);
      mtx_destroy(&queue->helios_record_mutex);
      queue->helios_record_mutex_live = false;
      return VK_ERROR_DEVICE_LOST;
   }
   const enum vn_helios_submission_mode mode = owner->mode;
   owner->live_queue_count++;
   mtx_unlock(&owner->lock);

   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return VK_SUCCESS;

   if (shared_queue) {
      if (!shared_queue->helios_native_context) {
         vn_helios_submit_queue_fini(queue);
         return VK_ERROR_INITIALIZATION_FAILED;
      }
      queue->helios_native_context = shared_queue->helios_native_context;
      queue->helios_native_context_owner = false;
      return VK_SUCCESS;
   }

   const uint32_t device = vn_renderer_helios_device_handle(dev->renderer);
   VkResult result = helios_native_context_create(
      device, HELIOS_NATIVE_CONTEXT_QUEUE, queue_family, queue_index,
      &queue->helios_native_context);
   if (result != VK_SUCCESS) {
      /* The KMT call may have consumed K11's next non-recycled endpoint
       * before a later context/fence allocation failed.  HVC1 has no output
       * endpoint field, so this instance cannot prove that a retry would stay
       * aligned with its own monotonic queue index. */
      mtx_lock(&owner->lock);
      owner->queue_admission_failed = true;
      mtx_unlock(&owner->lock);
      vn_helios_submit_queue_fini(queue);
      return result;
   }
   queue->helios_native_context_owner = true;
   return VK_SUCCESS;
}

void
vn_helios_submit_queue_fini(struct vn_queue *queue)
{
   if (!queue)
      return;
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      dev ? helios_submit_owner(dev->instance) : NULL;

   if (queue->helios_native_context_owner && queue->helios_native_context)
      helios_native_context_destroy(queue->helios_native_context);
   queue->helios_native_context = NULL;
   queue->helios_native_context_owner = false;
   if (queue->helios_record_mutex_live) {
      mtx_destroy(&queue->helios_record_mutex);
      queue->helios_record_mutex_live = false;
   }
   if (owner) {
      mtx_lock(&owner->lock);
      if (owner->live_queue_count)
         owner->live_queue_count--;
      mtx_unlock(&owner->lock);
   }
}

/* ── record-only scopes ─────────────────────────────────────────────────── */

static bool
helios_scope_on_calling_thread(const struct HeliosTranslatorScope_T *scope)
{
   return scope && thrd_equal(scope->thread, thrd_current());
}

static struct HeliosTranslatorScope_T *
helios_current_scope(struct vn_helios_submit_instance *owner)
{
   return owner && owner->scope_key_live ? tss_get(owner->scope_key) : NULL;
}

HeliosTranslatorStatusCode
vn_helios_record_context_create(struct vn_instance *instance,
                                struct vn_queue *queue,
                                uint32_t endpoint_id,
                                uint64_t context_generation,
                                uint32_t context_flags,
                                struct vn_helios_record_context **out_context)
{
   if (!out_context)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   *out_context = NULL;
   struct vn_helios_submit_instance *owner = helios_submit_owner(instance);
   if (!owner || !queue)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   if (!dev || dev->instance != instance)
      return HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
   if (owner->mode != VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return HELIOS_TRANSLATOR_STATUS_SUBMISSION_MODE;
   if (!context_generation)
      return HELIOS_TRANSLATOR_STATUS_CONTEXT_GENERATION;
   if (!endpoint_id ||
       endpoint_id >
          vn_renderer_helios_endpoint_capacity(instance->renderer) ||
       endpoint_id != queue->ring_idx)
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_ENDPOINT;
   if (context_flags != HELIOS_HOB1_FLAG_D3D11_PHYSICAL &&
       context_flags != HELIOS_HOB1_FLAG_D3D12_VIRTUAL)
      return HELIOS_TRANSLATOR_STATUS_CONTEXT_FLAGS;

   struct vn_helios_record_context *context = calloc(1, sizeof(*context));
   if (!context)
      return HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;
   if (mtx_init(&context->lock, mtx_plain) != thrd_success) {
      free(context);
      return HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;
   }
   context->lock_live = true;
   context->owner = owner;
   context->queue = queue;
   context->endpoint_id = endpoint_id;
   context->context_generation = context_generation;
   context->context_flags = context_flags;
   *out_context = context;
   return HELIOS_TRANSLATOR_STATUS_OK;
}

HeliosTranslatorStatusCode
vn_helios_record_context_destroy(struct vn_helios_record_context *context)
{
   if (!context)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   mtx_lock(&context->lock);
   const bool live = context->active_scope != NULL;
   mtx_unlock(&context->lock);
   if (live)
      return HELIOS_TRANSLATOR_STATUS_SCOPE_STILL_LIVE;
   if (context->lock_live)
      mtx_destroy(&context->lock);
   free(context);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

HeliosTranslatorStatusCode
vn_helios_record_scope_open(struct vn_helios_record_context *context,
                            HeliosTranslatorScope *out_scope)
{
   if (!context || !out_scope)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   *out_scope = NULL;
   struct vn_helios_submit_instance *owner = context->owner;
   if (!owner || owner->mode != VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return HELIOS_TRANSLATOR_STATUS_SUBMISSION_MODE;
   if (helios_current_scope(owner))
      return HELIOS_TRANSLATOR_STATUS_SCOPE_ALREADY_OPEN;

   struct HeliosTranslatorScope_T *scope = calloc(1, sizeof(*scope));
   if (!scope)
      return HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;
   scope->context = context;
   scope->thread = thrd_current();

   mtx_lock(&context->lock);
   if (context->active_scope) {
      mtx_unlock(&context->lock);
      free(scope);
      return HELIOS_TRANSLATOR_STATUS_SCOPE_ALREADY_OPEN;
   }
   if (context->next_batch_id == UINT64_MAX) {
      mtx_unlock(&context->lock);
      free(scope);
      return HELIOS_TRANSLATOR_STATUS_BATCH_ID;
   }
   scope->batch_id = ++context->next_batch_id;
   context->active_scope = scope;
   mtx_unlock(&context->lock);

   if (tss_set(owner->scope_key, scope) != thrd_success) {
      mtx_lock(&context->lock);
      context->active_scope = NULL;
      mtx_unlock(&context->lock);
      free(scope);
      return HELIOS_TRANSLATOR_STATUS_SCOPE_FOREIGN_THREAD;
   }
   *out_scope = scope;
   return HELIOS_TRANSLATOR_STATUS_OK;
}

static bool
helios_scope_reserve_payload(struct HeliosTranslatorScope_T *scope,
                             uint64_t bytes)
{
   if (bytes > HELIOS_HOB1_MAX_BYTES ||
       scope->payload_bytes > HELIOS_HOB1_MAX_BYTES - bytes)
      return false;
   const uint64_t required = scope->payload_bytes + bytes;
   if (required <= scope->payload_capacity)
      return true;
   uint64_t capacity =
      scope->payload_capacity ? scope->payload_capacity : 4096;
   while (capacity < required) {
      const uint64_t next = capacity * 2;
      capacity = next > capacity && next <= HELIOS_HOB1_MAX_BYTES
                    ? next
                    : HELIOS_HOB1_MAX_BYTES;
      if (capacity < required)
         return false;
   }
   void *payload = realloc(scope->payload, (size_t)capacity);
   if (!payload)
      return false;
   scope->payload = payload;
   scope->payload_capacity = capacity;
   return true;
}

static VkResult
helios_record_append(struct vn_queue *queue,
                     enum helios_record_refusal refusal,
                     const void *payload,
                     uint64_t payload_bytes)
{
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
   if (!scope) {
      helios_record_refuse(owner, refusal);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (!helios_scope_on_calling_thread(scope)) {
      helios_record_refuse(owner, refusal);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (scope->context->owner != owner || scope->context->queue != queue) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (scope->is_sealed) {
      helios_record_refuse(owner, refusal);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   /* One outer scope is one logical generated queue operation.  The bounded
    * KMD schema consumes one complete queue opcode (whose submit-info array
    * may itself contain many batches); concatenating a second opcode would be
    * trailing bytes, not an executable Venus stream. */
   if (scope->payload_bytes) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   if (!payload || !payload_bytes) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   mtx_lock(&queue->helios_record_mutex);
   const uint64_t assembled =
      HELIOS_HOB1_HEADER_BYTES + scope->payload_bytes + payload_bytes +
      (uint64_t)scope->use_count * HELIOS_HOB1_USE_RECORD_BYTES +
      (uint64_t)scope->operand_count * HELIOS_HOB1_OPERAND_RECORD_BYTES;
   if (assembled > HELIOS_HOB1_MAX_BYTES ||
       !helios_scope_reserve_payload(scope, payload_bytes)) {
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      mtx_unlock(&queue->helios_record_mutex);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   memcpy(scope->payload + scope->payload_bytes, payload,
          (size_t)payload_bytes);
   scope->payload_bytes += payload_bytes;
   mtx_unlock(&queue->helios_record_mutex);
   return VK_SUCCESS;
}

HeliosTranslatorStatusCode
vn_helios_record_scope_seal(HeliosTranslatorScope opaque,
                            HeliosSealedBatchV1 *out_sealed)
{
   struct HeliosTranslatorScope_T *scope = opaque;
   if (!scope || !out_sealed)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   if (!helios_scope_on_calling_thread(scope))
      return HELIOS_TRANSLATOR_STATUS_SCOPE_FOREIGN_THREAD;
   if (out_sealed->struct_bytes != sizeof(*out_sealed))
      return HELIOS_TRANSLATOR_STATUS_STRUCT_BYTES;
   if (out_sealed->abi_version != HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION)
      return HELIOS_TRANSLATOR_STATUS_ABI_VERSION;
   if (scope->is_sealed)
      return HELIOS_TRANSLATOR_STATUS_SCOPE_ALREADY_SEALED;
   if (scope->failure != HELIOS_TRANSLATOR_STATUS_OK)
      return scope->failure;

   struct vn_queue *queue = scope->context->queue;
   mtx_lock(&queue->helios_record_mutex);
   HeliosSealedBatchV1 sealed = {
      .struct_bytes = sizeof(sealed),
      .abi_version = HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION,
      .package_generation = HELIOS_PACKAGE_GENERATION,
      .session_generation = scope->context->owner->session_generation,
      .context_generation = scope->context->context_generation,
      .batch_id = scope->batch_id,
      .payload_bytes = scope->payload_bytes,
      .payload_crc64 = helios_crc64(scope->payload, scope->payload_bytes),
      .endpoint_id = scope->context->endpoint_id,
      .context_flags = scope->context->context_flags,
      .use_count = scope->use_count,
      .operand_count = scope->operand_count,
   };
   const HeliosTranslatorStatusCode checked =
      helios_translator_check_sealed_batch(
         &sealed, HELIOS_PACKAGE_GENERATION,
         scope->context->owner->session_generation,
         scope->context->context_generation, scope->context->endpoint_id,
         scope->context->context_flags);
   if (checked == HELIOS_TRANSLATOR_STATUS_OK) {
      scope->sealed = sealed;
      scope->is_sealed = true;
      *out_sealed = sealed;
   }
   mtx_unlock(&queue->helios_record_mutex);
   return checked;
}

HeliosTranslatorStatusCode
vn_helios_record_scope_copy(HeliosTranslatorScope opaque,
                            const HeliosSealedBatchCopyV1 *destination)
{
   struct HeliosTranslatorScope_T *scope = opaque;
   if (!scope || !destination)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   if (!helios_scope_on_calling_thread(scope))
      return HELIOS_TRANSLATOR_STATUS_SCOPE_FOREIGN_THREAD;
   if (!scope->is_sealed)
      return HELIOS_TRANSLATOR_STATUS_SCOPE_NOT_SEALED;
   const HeliosTranslatorStatusCode checked =
      helios_translator_check_sealed_batch_copy(destination, &scope->sealed);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;
   memcpy(destination->payload, scope->payload,
          (size_t)scope->sealed.payload_bytes);
   if (scope->sealed.use_count)
      memcpy(destination->uses, scope->uses,
             (size_t)scope->sealed.use_count * sizeof(*scope->uses));
   if (scope->sealed.operand_count)
      memcpy(destination->operands, scope->operands,
             (size_t)scope->sealed.operand_count * sizeof(*scope->operands));
   return HELIOS_TRANSLATOR_STATUS_OK;
}

HeliosTranslatorStatusCode
vn_helios_record_scope_close(HeliosTranslatorScope opaque,
                             const HeliosOuterScopeCloseV1 *close)
{
   struct HeliosTranslatorScope_T *scope = opaque;
   if (!scope || !close)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   if (!helios_scope_on_calling_thread(scope))
      return HELIOS_TRANSLATOR_STATUS_SCOPE_FOREIGN_THREAD;
   uint32_t disposition = 0;
   HeliosTranslatorStatusCode checked =
      helios_translator_check_scope_close(close, &disposition);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;
   if (disposition == HELIOS_TRANSLATOR_SCOPE_DISPOSITION_COMMITTED &&
       !scope->is_sealed)
      return HELIOS_TRANSLATOR_STATUS_SCOPE_NOT_SEALED;

   struct vn_helios_record_context *context = scope->context;
   struct vn_helios_submit_instance *owner = context->owner;
   mtx_lock(&context->lock);
   if (context->active_scope != scope) {
      mtx_unlock(&context->lock);
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   }
   /* Never free a scope while its per-instance TLS slot can still name it. */
   if (tss_set(owner->scope_key, NULL) != thrd_success) {
      mtx_unlock(&context->lock);
      return HELIOS_TRANSLATOR_STATUS_SCOPE_FOREIGN_THREAD;
   }
   context->active_scope = NULL;
   mtx_unlock(&context->lock);
   free(scope->operands);
   free(scope->uses);
   free(scope->payload);
   free(scope);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

void
vn_helios_record_query_refusals(const struct vn_instance *instance,
                                HeliosTranslatorRefusalCountersV1 *out)
{
   if (!out)
      return;
   const struct vn_helios_submit_instance *owner =
      helios_submit_owner(instance);
   memset(out, 0, sizeof(*out));
   out->struct_bytes = sizeof(*out);
   out->abi_version = HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION;
   if (!owner)
      return;
#define HELIOS_READ_REFUSAL(field, index)                                    \
   out->field = (uint64_t)InterlockedCompareExchange64(                      \
      (volatile LONG64 *)&owner->refusals[index], 0, 0)
   HELIOS_READ_REFUSAL(queue_submit_without_scope,
                       HELIOS_RECORD_REFUSE_QUEUE_SUBMIT);
   HELIOS_READ_REFUSAL(queue_submit2_without_scope,
                       HELIOS_RECORD_REFUSE_QUEUE_SUBMIT2);
   HELIOS_READ_REFUSAL(queue_bind_sparse_without_scope,
                       HELIOS_RECORD_REFUSE_QUEUE_BIND_SPARSE);
   HELIOS_READ_REFUSAL(queue_present_refused,
                       HELIOS_RECORD_REFUSE_QUEUE_PRESENT);
   HELIOS_READ_REFUSAL(queue_wait_idle_without_scope,
                       HELIOS_RECORD_REFUSE_QUEUE_WAIT_IDLE);
   HELIOS_READ_REFUSAL(device_wait_idle_without_scope,
                       HELIOS_RECORD_REFUSE_DEVICE_WAIT_IDLE);
   HELIOS_READ_REFUSAL(loader_provenance_rejected,
                       HELIOS_RECORD_REFUSE_LOADER_PROVENANCE);
   HELIOS_READ_REFUSAL(control_opcode_class_violation,
                       HELIOS_RECORD_REFUSE_CONTROL_CLASS);
   HELIOS_READ_REFUSAL(deferred_use_without_outer_batch,
                       HELIOS_RECORD_REFUSE_DEFERRED_USE);
   HELIOS_READ_REFUSAL(batch_bound_exceeded,
                       HELIOS_RECORD_REFUSE_BATCH_BOUND);
   HELIOS_READ_REFUSAL(foreign_vulkan_handle_rejected,
                       HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
   HELIOS_READ_REFUSAL(withheld_proc_addr_refused,
                       HELIOS_RECORD_REFUSE_WITHHELD_PROC);
   HELIOS_READ_REFUSAL(reentrant_join_refused,
                       HELIOS_RECORD_REFUSE_REENTRANT_JOIN);
#undef HELIOS_READ_REFUSAL
}

/* ── generated payload execution ────────────────────────────────────────── */

static VkResult
helios_dispatch_payload(struct vn_queue *queue,
                        enum helios_record_refusal refusal,
                        const void *payload,
                        size_t payload_bytes,
                        const struct helios_hnr2_allocation *allocations,
                        uint32_t allocation_count,
                        const struct helios_fence_points *waits,
                        const struct helios_fence_points *signals)
{
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   if (!owner)
      return VK_ERROR_INITIALIZATION_FAILED;

   if (owner->mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      /* A7 will convert outer allocation tokens into sealed uses.  A4 may not
       * leak a process-local HVM1 handle into that ABI or fabricate a token. */
      if (allocation_count) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      return helios_record_append(queue, refusal, payload, payload_bytes);
   }

   if (!queue->helios_native_context)
      return VK_ERROR_DEVICE_LOST;
   const struct helios_hnr2_batch batch = {
      .payload = payload,
      .payload_bytes = payload_bytes,
      .allocations = allocations,
      .allocation_count = allocation_count,
      .patches = NULL,
      .patch_count = 0,
   };
   uint64_t batch_token = 0;
   uint64_t progress = 0;
   return helios_native_context_submit_ordered(
      queue->helios_native_context, waits ? waits->points : NULL,
      waits ? waits->count : 0, &batch, signals ? signals->points : NULL,
      signals ? signals->count : 0, true, &batch_token, &progress);
}

static VkResult
helios_encode_submit1(VkQueue queue,
                      uint32_t count,
                      const VkSubmitInfo *submits,
                      VkFence fence,
                      void **out_payload,
                      size_t *out_bytes)
{
   const size_t bytes = vn_sizeof_vkQueueSubmit(queue, count, submits, fence);
   if (!bytes || bytes > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   void *payload = malloc(bytes);
   if (!payload)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   struct vn_cs_encoder enc = VN_CS_ENCODER_INITIALIZER_LOCAL(payload, bytes);
   vn_encode_vkQueueSubmit(&enc, 0, queue, count, submits, fence);
   if (vn_cs_encoder_get_fatal(&enc) ||
       vn_cs_encoder_get_len(&enc) != bytes) {
      free(payload);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   *out_payload = payload;
   *out_bytes = bytes;
   return VK_SUCCESS;
}

static VkResult
helios_encode_submit2(VkQueue queue,
                      uint32_t count,
                      const VkSubmitInfo2 *submits,
                      VkFence fence,
                      void **out_payload,
                      size_t *out_bytes)
{
   const size_t bytes =
      vn_sizeof_vkQueueSubmit2(queue, count, submits, fence);
   if (!bytes || bytes > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   void *payload = malloc(bytes);
   if (!payload)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   struct vn_cs_encoder enc = VN_CS_ENCODER_INITIALIZER_LOCAL(payload, bytes);
   vn_encode_vkQueueSubmit2(&enc, 0, queue, count, submits, fence);
   if (vn_cs_encoder_get_fatal(&enc) ||
       vn_cs_encoder_get_len(&enc) != bytes) {
      free(payload);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   *out_payload = payload;
   *out_bytes = bytes;
   return VK_SUCCESS;
}

static VkResult
helios_encode_bind_sparse(VkQueue queue,
                          uint32_t count,
                          const VkBindSparseInfo *binds,
                          VkFence fence,
                          void **out_payload,
                          size_t *out_bytes)
{
   const size_t bytes =
      vn_sizeof_vkQueueBindSparse(queue, count, binds, fence);
   if (!bytes || bytes > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   void *payload = malloc(bytes);
   if (!payload)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   struct vn_cs_encoder enc = VN_CS_ENCODER_INITIALIZER_LOCAL(payload, bytes);
   vn_encode_vkQueueBindSparse(&enc, 0, queue, count, binds, fence);
   if (vn_cs_encoder_get_fatal(&enc) ||
       vn_cs_encoder_get_len(&enc) != bytes) {
      free(payload);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   *out_payload = payload;
   *out_bytes = bytes;
   return VK_SUCCESS;
}

/* ── imported-fence stripping and generated-schema copies ──────────────── */

struct helios_submit1_aux {
   VkSemaphore *waits;
   VkPipelineStageFlags *wait_stages;
   VkSemaphore *signals;
   uint64_t *wait_values;
   uint64_t *signal_values;
   uint32_t *group_wait_indices;
   uint32_t *group_command_masks;
   uint32_t *group_signal_indices;
   VkDeviceGroupSubmitInfo group;
   VkProtectedSubmitInfo protected_info;
   VkTimelineSemaphoreSubmitInfo timeline;
   bool have_group;
   bool have_protected;
   bool have_timeline;
};

struct helios_submit1_copy {
   VkSubmitInfo *infos;
   struct helios_submit1_aux *aux;
   uint32_t count;
   struct helios_fence_points waits;
   struct helios_fence_points signals;
};

static void
helios_submit1_copy_fini(struct helios_submit1_copy *copy)
{
   if (copy->aux) {
      for (uint32_t i = 0; i < copy->count; i++) {
         struct helios_submit1_aux *aux = &copy->aux[i];
         free(aux->group_signal_indices);
         free(aux->group_command_masks);
         free(aux->group_wait_indices);
         free(aux->signal_values);
         free(aux->wait_values);
         free(aux->signals);
         free(aux->wait_stages);
         free(aux->waits);
      }
   }
   helios_fence_points_fini(&copy->signals);
   helios_fence_points_fini(&copy->waits);
   free(copy->aux);
   free(copy->infos);
   memset(copy, 0, sizeof(*copy));
}

static VkResult
helios_submit1_scan_pnext(const VkSubmitInfo *src,
                          struct helios_submit1_aux *aux)
{
   vk_foreach_struct_const(ext, src->pNext) {
      switch (ext->sType) {
      case VK_STRUCTURE_TYPE_DEVICE_GROUP_SUBMIT_INFO:
         if (aux->have_group)
            return VK_ERROR_FEATURE_NOT_PRESENT;
         aux->have_group = true;
         break;
      case VK_STRUCTURE_TYPE_PROTECTED_SUBMIT_INFO:
         if (aux->have_protected)
            return VK_ERROR_FEATURE_NOT_PRESENT;
         aux->have_protected = true;
         break;
      case VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO:
         if (aux->have_timeline)
            return VK_ERROR_FEATURE_NOT_PRESENT;
         aux->have_timeline = true;
         break;
      default:
         /* The bounded KMD parser has no schema for this node. */
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
   }
   return VK_SUCCESS;
}

static VkResult
helios_submit1_copy_init(struct vn_device *dev,
                         uint32_t count,
                         const VkSubmitInfo *srcs,
                         struct helios_submit1_copy *copy)
{
   VkResult failure = VK_SUCCESS;
   memset(copy, 0, sizeof(*copy));
   if (count > HELIOS_HOB1_MAX_OPERAND_RECORDS || (count && !srcs))
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   copy->count = count;
   if (!count)
      return VK_SUCCESS;
   copy->infos = calloc(count, sizeof(*copy->infos));
   copy->aux = calloc(count, sizeof(*copy->aux));
   if (!copy->infos || !copy->aux)
      goto oom;

   for (uint32_t b = 0; b < count; b++) {
      const VkSubmitInfo *src = &srcs[b];
      VkSubmitInfo *dst = &copy->infos[b];
      struct helios_submit1_aux *aux = &copy->aux[b];
      if (src->sType != VK_STRUCTURE_TYPE_SUBMIT_INFO ||
          (src->waitSemaphoreCount &&
           (!src->pWaitSemaphores || !src->pWaitDstStageMask)) ||
          (src->commandBufferCount && !src->pCommandBuffers) ||
          (src->signalSemaphoreCount && !src->pSignalSemaphores))
         goto invalid;
      if (helios_submit1_scan_pnext(src, aux) != VK_SUCCESS)
         goto unsupported;

      const VkTimelineSemaphoreSubmitInfo *timeline =
         aux->have_timeline
            ? vk_find_struct_const(src->pNext, TIMELINE_SEMAPHORE_SUBMIT_INFO)
            : NULL;
      const VkDeviceGroupSubmitInfo *group =
         aux->have_group
            ? vk_find_struct_const(src->pNext, DEVICE_GROUP_SUBMIT_INFO)
            : NULL;
      const VkProtectedSubmitInfo *protected_info =
         aux->have_protected
            ? vk_find_struct_const(src->pNext, PROTECTED_SUBMIT_INFO)
            : NULL;
      if (timeline &&
          (timeline->waitSemaphoreValueCount != src->waitSemaphoreCount ||
           timeline->signalSemaphoreValueCount != src->signalSemaphoreCount ||
           (timeline->waitSemaphoreValueCount &&
            !timeline->pWaitSemaphoreValues) ||
           (timeline->signalSemaphoreValueCount &&
            !timeline->pSignalSemaphoreValues)))
         goto invalid;
      if (group &&
          (group->waitSemaphoreCount != src->waitSemaphoreCount ||
           group->commandBufferCount != src->commandBufferCount ||
           group->signalSemaphoreCount != src->signalSemaphoreCount ||
           (group->waitSemaphoreCount &&
            !group->pWaitSemaphoreDeviceIndices) ||
           (group->commandBufferCount && !group->pCommandBufferDeviceMasks) ||
           (group->signalSemaphoreCount &&
            !group->pSignalSemaphoreDeviceIndices)))
         goto invalid;
      if (protected_info && protected_info->protectedSubmit)
         goto unsupported;
      if (group) {
         for (uint32_t i = 0; i < group->waitSemaphoreCount; i++) {
            if (group->pWaitSemaphoreDeviceIndices[i] != 0)
               goto unsupported;
         }
         for (uint32_t i = 0; i < group->signalSemaphoreCount; i++) {
            if (group->pSignalSemaphoreDeviceIndices[i] != 0)
               goto unsupported;
         }
      }

      if (src->waitSemaphoreCount) {
         aux->waits = calloc(src->waitSemaphoreCount, sizeof(*aux->waits));
         aux->wait_stages =
            calloc(src->waitSemaphoreCount, sizeof(*aux->wait_stages));
         aux->wait_values =
            calloc(src->waitSemaphoreCount, sizeof(*aux->wait_values));
         aux->group_wait_indices =
            calloc(src->waitSemaphoreCount, sizeof(*aux->group_wait_indices));
      }
      if (src->signalSemaphoreCount) {
         aux->signals =
            calloc(src->signalSemaphoreCount, sizeof(*aux->signals));
         aux->signal_values =
            calloc(src->signalSemaphoreCount, sizeof(*aux->signal_values));
         aux->group_signal_indices = calloc(
            src->signalSemaphoreCount, sizeof(*aux->group_signal_indices));
      }
      if (src->commandBufferCount)
         aux->group_command_masks = calloc(src->commandBufferCount,
                                           sizeof(*aux->group_command_masks));
      if ((src->waitSemaphoreCount &&
           (!aux->waits || !aux->wait_stages || !aux->wait_values ||
            !aux->group_wait_indices)) ||
          (src->signalSemaphoreCount &&
           (!aux->signals || !aux->signal_values ||
            !aux->group_signal_indices)) ||
          (src->commandBufferCount && !aux->group_command_masks))
         goto oom;

      uint32_t kept_waits = 0;
      for (uint32_t i = 0; i < src->waitSemaphoreCount; i++) {
         struct vn_semaphore *sem =
            vn_semaphore_from_handle(src->pWaitSemaphores[i]);
         if (!sem)
            goto invalid;
         const uint64_t value =
            sem->type == VK_SEMAPHORE_TYPE_TIMELINE
               ? (timeline ? timeline->pWaitSemaphoreValues[i] : UINT64_MAX)
               : 1;
         if (value == UINT64_MAX && !timeline)
            goto invalid;
         bool native = false;
         failure = helios_classify_semaphore(dev, src->pWaitSemaphores[i],
                                             value, &copy->waits, &native);
         if (failure != VK_SUCCESS)
            goto result_fail;
         if (native)
            continue;
         aux->waits[kept_waits] = src->pWaitSemaphores[i];
         aux->wait_stages[kept_waits] = src->pWaitDstStageMask[i];
         if (timeline)
            aux->wait_values[kept_waits] = timeline->pWaitSemaphoreValues[i];
         if (group)
            aux->group_wait_indices[kept_waits] =
               group->pWaitSemaphoreDeviceIndices[i];
         kept_waits++;
      }

      uint32_t kept_signals = 0;
      for (uint32_t i = 0; i < src->signalSemaphoreCount; i++) {
         struct vn_semaphore *sem =
            vn_semaphore_from_handle(src->pSignalSemaphores[i]);
         if (!sem)
            goto invalid;
         const uint64_t value =
            sem->type == VK_SEMAPHORE_TYPE_TIMELINE
               ? (timeline ? timeline->pSignalSemaphoreValues[i] : UINT64_MAX)
               : 1;
         if (value == UINT64_MAX && !timeline)
            goto invalid;
         bool native = false;
         failure = helios_classify_semaphore(dev, src->pSignalSemaphores[i],
                                             value, &copy->signals, &native);
         if (failure != VK_SUCCESS)
            goto result_fail;
         if (native)
            continue;
         aux->signals[kept_signals] = src->pSignalSemaphores[i];
         if (timeline)
            aux->signal_values[kept_signals] =
               timeline->pSignalSemaphoreValues[i];
         if (group)
            aux->group_signal_indices[kept_signals] =
               group->pSignalSemaphoreDeviceIndices[i];
         kept_signals++;
      }

      *dst = *src;
      dst->pNext = NULL;
      dst->waitSemaphoreCount = kept_waits;
      dst->pWaitSemaphores = kept_waits ? aux->waits : NULL;
      dst->pWaitDstStageMask = kept_waits ? aux->wait_stages : NULL;
      dst->signalSemaphoreCount = kept_signals;
      dst->pSignalSemaphores = kept_signals ? aux->signals : NULL;

      const void **tail = &dst->pNext;
      vk_foreach_struct_const(ext, src->pNext) {
         switch (ext->sType) {
         case VK_STRUCTURE_TYPE_DEVICE_GROUP_SUBMIT_INFO:
            aux->group = *(const VkDeviceGroupSubmitInfo *)ext;
            if (src->commandBufferCount) {
               memcpy(aux->group_command_masks,
                      group->pCommandBufferDeviceMasks,
                      (size_t)src->commandBufferCount *
                         sizeof(*aux->group_command_masks));
            }
            aux->group.pNext = NULL;
            aux->group.waitSemaphoreCount = kept_waits;
            aux->group.pWaitSemaphoreDeviceIndices =
               kept_waits ? aux->group_wait_indices : NULL;
            aux->group.pCommandBufferDeviceMasks =
               src->commandBufferCount ? aux->group_command_masks : NULL;
            aux->group.signalSemaphoreCount = kept_signals;
            aux->group.pSignalSemaphoreDeviceIndices =
               kept_signals ? aux->group_signal_indices : NULL;
            *tail = &aux->group;
            tail = &aux->group.pNext;
            break;
         case VK_STRUCTURE_TYPE_PROTECTED_SUBMIT_INFO:
            aux->protected_info = *(const VkProtectedSubmitInfo *)ext;
            aux->protected_info.pNext = NULL;
            *tail = &aux->protected_info;
            tail = &aux->protected_info.pNext;
            break;
         case VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO:
            aux->timeline = *(const VkTimelineSemaphoreSubmitInfo *)ext;
            aux->timeline.pNext = NULL;
            aux->timeline.waitSemaphoreValueCount = kept_waits;
            aux->timeline.pWaitSemaphoreValues =
               kept_waits ? aux->wait_values : NULL;
            aux->timeline.signalSemaphoreValueCount = kept_signals;
            aux->timeline.pSignalSemaphoreValues =
               kept_signals ? aux->signal_values : NULL;
            *tail = &aux->timeline;
            tail = &aux->timeline.pNext;
            break;
         default:
            goto unsupported;
         }
      }
      *tail = NULL;
   }
   return VK_SUCCESS;

result_fail:
   helios_submit1_copy_fini(copy);
   return failure;
unsupported:
   helios_submit1_copy_fini(copy);
   return VK_ERROR_FEATURE_NOT_PRESENT;
invalid:
   helios_submit1_copy_fini(copy);
   return VK_ERROR_INITIALIZATION_FAILED;
oom:
   helios_submit1_copy_fini(copy);
   return VK_ERROR_OUT_OF_HOST_MEMORY;
}

struct helios_submit2_aux {
   VkSemaphoreSubmitInfo *waits;
   VkSemaphoreSubmitInfo *signals;
};

struct helios_submit2_copy {
   VkSubmitInfo2 *infos;
   struct helios_submit2_aux *aux;
   uint32_t count;
   struct helios_fence_points waits;
   struct helios_fence_points signals;
};

static void
helios_submit2_copy_fini(struct helios_submit2_copy *copy)
{
   if (copy->aux) {
      for (uint32_t i = 0; i < copy->count; i++) {
         free(copy->aux[i].signals);
         free(copy->aux[i].waits);
      }
   }
   helios_fence_points_fini(&copy->signals);
   helios_fence_points_fini(&copy->waits);
   free(copy->aux);
   free(copy->infos);
   memset(copy, 0, sizeof(*copy));
}

static VkResult
helios_submit2_copy_init(struct vn_device *dev,
                         uint32_t count,
                         const VkSubmitInfo2 *srcs,
                         struct helios_submit2_copy *copy)
{
   VkResult failure = VK_SUCCESS;
   memset(copy, 0, sizeof(*copy));
   if (count > HELIOS_HOB1_MAX_OPERAND_RECORDS || (count && !srcs))
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   copy->count = count;
   if (!count)
      return VK_SUCCESS;
   copy->infos = calloc(count, sizeof(*copy->infos));
   copy->aux = calloc(count, sizeof(*copy->aux));
   if (!copy->infos || !copy->aux)
      goto oom;

   for (uint32_t b = 0; b < count; b++) {
      const VkSubmitInfo2 *src = &srcs[b];
      VkSubmitInfo2 *dst = &copy->infos[b];
      struct helios_submit2_aux *aux = &copy->aux[b];
      if (src->sType != VK_STRUCTURE_TYPE_SUBMIT_INFO_2 || src->pNext ||
          src->flags ||
          (src->waitSemaphoreInfoCount && !src->pWaitSemaphoreInfos) ||
          (src->commandBufferInfoCount && !src->pCommandBufferInfos) ||
          (src->signalSemaphoreInfoCount && !src->pSignalSemaphoreInfos))
         goto invalid;
      if (src->waitSemaphoreInfoCount)
         aux->waits =
            calloc(src->waitSemaphoreInfoCount, sizeof(*aux->waits));
      if (src->signalSemaphoreInfoCount)
         aux->signals =
            calloc(src->signalSemaphoreInfoCount, sizeof(*aux->signals));
      if ((src->waitSemaphoreInfoCount && !aux->waits) ||
          (src->signalSemaphoreInfoCount && !aux->signals))
         goto oom;

      uint32_t kept_waits = 0;
      for (uint32_t i = 0; i < src->waitSemaphoreInfoCount; i++) {
         const VkSemaphoreSubmitInfo *info = &src->pWaitSemaphoreInfos[i];
         if (info->sType != VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO ||
             info->pNext || info->deviceIndex)
            goto unsupported;
         struct vn_semaphore *sem = vn_semaphore_from_handle(info->semaphore);
         if (!sem)
            goto invalid;
         const uint64_t value =
            sem->type == VK_SEMAPHORE_TYPE_TIMELINE ? info->value : 1;
         bool native = false;
         failure = helios_classify_semaphore(dev, info->semaphore, value,
                                             &copy->waits, &native);
         if (failure != VK_SUCCESS)
            goto result_fail;
         if (!native)
            aux->waits[kept_waits++] = *info;
      }

      uint32_t kept_signals = 0;
      for (uint32_t i = 0; i < src->signalSemaphoreInfoCount; i++) {
         const VkSemaphoreSubmitInfo *info = &src->pSignalSemaphoreInfos[i];
         if (info->sType != VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO ||
             info->pNext || info->deviceIndex)
            goto unsupported;
         struct vn_semaphore *sem = vn_semaphore_from_handle(info->semaphore);
         if (!sem)
            goto invalid;
         const uint64_t value =
            sem->type == VK_SEMAPHORE_TYPE_TIMELINE ? info->value : 1;
         bool native = false;
         failure = helios_classify_semaphore(dev, info->semaphore, value,
                                             &copy->signals, &native);
         if (failure != VK_SUCCESS)
            goto result_fail;
         if (!native)
            aux->signals[kept_signals++] = *info;
      }
      for (uint32_t i = 0; i < src->commandBufferInfoCount; i++) {
         if (src->pCommandBufferInfos[i].sType !=
                VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO ||
             src->pCommandBufferInfos[i].pNext)
            goto unsupported;
      }
      *dst = *src;
      dst->waitSemaphoreInfoCount = kept_waits;
      dst->pWaitSemaphoreInfos = kept_waits ? aux->waits : NULL;
      dst->signalSemaphoreInfoCount = kept_signals;
      dst->pSignalSemaphoreInfos = kept_signals ? aux->signals : NULL;
   }
   return VK_SUCCESS;

result_fail:
   helios_submit2_copy_fini(copy);
   return failure;
unsupported:
   helios_submit2_copy_fini(copy);
   return VK_ERROR_FEATURE_NOT_PRESENT;
invalid:
   helios_submit2_copy_fini(copy);
   return VK_ERROR_INITIALIZATION_FAILED;
oom:
   helios_submit2_copy_fini(copy);
   return VK_ERROR_OUT_OF_HOST_MEMORY;
}

struct helios_sparse_aux {
   VkSemaphore *waits;
   VkSemaphore *signals;
   uint64_t *wait_values;
   uint64_t *signal_values;
   VkDeviceGroupBindSparseInfo group;
   VkTimelineSemaphoreSubmitInfo timeline;
   bool have_group;
   bool have_timeline;
};

struct helios_sparse_copy {
   VkBindSparseInfo *infos;
   struct helios_sparse_aux *aux;
   uint32_t count;
   struct helios_fence_points waits;
   struct helios_fence_points signals;
   struct helios_hnr2_allocation allocations[HELIOS_HNR2_MAX_USE_RECORDS];
   uint32_t allocation_count;
};

static void
helios_sparse_copy_fini(struct helios_sparse_copy *copy)
{
   if (copy->aux) {
      for (uint32_t i = 0; i < copy->count; i++) {
         free(copy->aux[i].signal_values);
         free(copy->aux[i].wait_values);
         free(copy->aux[i].signals);
         free(copy->aux[i].waits);
      }
   }
   helios_fence_points_fini(&copy->signals);
   helios_fence_points_fini(&copy->waits);
   free(copy->aux);
   free(copy->infos);
   memset(copy, 0, sizeof(*copy));
}

static VkResult
helios_sparse_add_memory(struct helios_sparse_copy *copy,
                         struct vn_device *dev,
                         VkDeviceMemory memory)
{
   if (memory == VK_NULL_HANDLE)
      return VK_SUCCESS;
   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   if (!mem || !helios_object_owned(&mem->base.vk.base, dev)) {
      helios_record_refuse(helios_submit_owner(dev->instance),
                           HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (!mem->base_bo || !mem->base_bo->allocation_handle ||
       !mem->base_bo->allocation_generation)
      return VK_ERROR_FEATURE_NOT_PRESENT;
   const uint32_t handle = mem->base_bo->allocation_handle;
   for (uint32_t i = 0; i < copy->allocation_count; i++) {
      if (copy->allocations[i].handle == handle) {
         if (copy->allocations[i].expected_generation !=
             mem->base_bo->allocation_generation)
            return VK_ERROR_DEVICE_LOST;
         copy->allocations[i].access |=
            HELIOS_HNR2_ACCESS_READ | HELIOS_HNR2_ACCESS_WRITE;
         return VK_SUCCESS;
      }
   }
   if (copy->allocation_count >= HELIOS_HNR2_MAX_USE_RECORDS)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   copy->allocations[copy->allocation_count++] =
      (struct helios_hnr2_allocation){
         .handle = handle,
         .access = HELIOS_HNR2_ACCESS_READ | HELIOS_HNR2_ACCESS_WRITE,
         .expected_generation = mem->base_bo->allocation_generation,
      };
   return VK_SUCCESS;
}

static VkResult
helios_sparse_collect_uses(struct helios_sparse_copy *copy,
                           struct vn_device *dev,
                           const VkBindSparseInfo *info)
{
   for (uint32_t i = 0; i < info->bufferBindCount; i++) {
      const VkSparseBufferMemoryBindInfo *object = &info->pBufferBinds[i];
      struct vn_buffer *buffer = vn_buffer_from_handle(object->buffer);
      if (!buffer || !helios_object_owned(&buffer->base.vk, dev)) {
         helios_record_refuse(helios_submit_owner(dev->instance),
                              HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      if (object->bindCount && !object->pBinds)
         return VK_ERROR_INITIALIZATION_FAILED;
      for (uint32_t j = 0; j < object->bindCount; j++) {
         VkResult result =
            helios_sparse_add_memory(copy, dev, object->pBinds[j].memory);
         if (result != VK_SUCCESS)
            return result;
      }
   }
   for (uint32_t i = 0; i < info->imageOpaqueBindCount; i++) {
      const VkSparseImageOpaqueMemoryBindInfo *object =
         &info->pImageOpaqueBinds[i];
      struct vn_image *image = vn_image_from_handle(object->image);
      if (!image || !helios_object_owned(&image->base.vk.base, dev)) {
         helios_record_refuse(helios_submit_owner(dev->instance),
                              HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      if (object->bindCount && !object->pBinds)
         return VK_ERROR_INITIALIZATION_FAILED;
      for (uint32_t j = 0; j < object->bindCount; j++) {
         VkResult result =
            helios_sparse_add_memory(copy, dev, object->pBinds[j].memory);
         if (result != VK_SUCCESS)
            return result;
      }
   }
   for (uint32_t i = 0; i < info->imageBindCount; i++) {
      const VkSparseImageMemoryBindInfo *object = &info->pImageBinds[i];
      struct vn_image *image = vn_image_from_handle(object->image);
      if (!image || !helios_object_owned(&image->base.vk.base, dev)) {
         helios_record_refuse(helios_submit_owner(dev->instance),
                              HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      if (object->bindCount && !object->pBinds)
         return VK_ERROR_INITIALIZATION_FAILED;
      for (uint32_t j = 0; j < object->bindCount; j++) {
         VkResult result =
            helios_sparse_add_memory(copy, dev, object->pBinds[j].memory);
         if (result != VK_SUCCESS)
            return result;
      }
   }
   return VK_SUCCESS;
}

static VkResult
helios_sparse_copy_init(struct vn_device *dev,
                        uint32_t count,
                        const VkBindSparseInfo *srcs,
                        struct helios_sparse_copy *copy)
{
   VkResult failure = VK_SUCCESS;
   memset(copy, 0, sizeof(*copy));
   if (count > HELIOS_HOB1_MAX_OPERAND_RECORDS || (count && !srcs))
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   copy->count = count;
   if (!count)
      return VK_SUCCESS;
   copy->infos = calloc(count, sizeof(*copy->infos));
   copy->aux = calloc(count, sizeof(*copy->aux));
   if (!copy->infos || !copy->aux)
      goto oom;

   for (uint32_t b = 0; b < count; b++) {
      const VkBindSparseInfo *src = &srcs[b];
      VkBindSparseInfo *dst = &copy->infos[b];
      struct helios_sparse_aux *aux = &copy->aux[b];
      if (src->sType != VK_STRUCTURE_TYPE_BIND_SPARSE_INFO ||
          (src->waitSemaphoreCount && !src->pWaitSemaphores) ||
          (src->bufferBindCount && !src->pBufferBinds) ||
          (src->imageOpaqueBindCount && !src->pImageOpaqueBinds) ||
          (src->imageBindCount && !src->pImageBinds) ||
          (src->signalSemaphoreCount && !src->pSignalSemaphores))
         goto invalid;
      vk_foreach_struct_const(ext, src->pNext) {
         if (ext->sType == VK_STRUCTURE_TYPE_DEVICE_GROUP_BIND_SPARSE_INFO) {
            if (aux->have_group)
               goto unsupported;
            aux->have_group = true;
         } else if (ext->sType ==
                    VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO) {
            if (aux->have_timeline)
               goto unsupported;
            aux->have_timeline = true;
         } else {
            goto unsupported;
         }
      }
      const VkTimelineSemaphoreSubmitInfo *timeline =
         aux->have_timeline
            ? vk_find_struct_const(src->pNext, TIMELINE_SEMAPHORE_SUBMIT_INFO)
            : NULL;
      if (timeline &&
          (timeline->waitSemaphoreValueCount != src->waitSemaphoreCount ||
           timeline->signalSemaphoreValueCount != src->signalSemaphoreCount ||
           (timeline->waitSemaphoreValueCount &&
            !timeline->pWaitSemaphoreValues) ||
           (timeline->signalSemaphoreValueCount &&
            !timeline->pSignalSemaphoreValues)))
         goto invalid;
      const VkDeviceGroupBindSparseInfo *group =
         aux->have_group
            ? vk_find_struct_const(src->pNext, DEVICE_GROUP_BIND_SPARSE_INFO)
            : NULL;
      if (group && (group->resourceDeviceIndex || group->memoryDeviceIndex))
         goto unsupported;

      if (src->waitSemaphoreCount) {
         aux->waits = calloc(src->waitSemaphoreCount, sizeof(*aux->waits));
         aux->wait_values =
            calloc(src->waitSemaphoreCount, sizeof(*aux->wait_values));
      }
      if (src->signalSemaphoreCount) {
         aux->signals =
            calloc(src->signalSemaphoreCount, sizeof(*aux->signals));
         aux->signal_values =
            calloc(src->signalSemaphoreCount, sizeof(*aux->signal_values));
      }
      if ((src->waitSemaphoreCount && (!aux->waits || !aux->wait_values)) ||
          (src->signalSemaphoreCount &&
           (!aux->signals || !aux->signal_values)))
         goto oom;

      uint32_t kept_waits = 0;
      for (uint32_t i = 0; i < src->waitSemaphoreCount; i++) {
         struct vn_semaphore *sem =
            vn_semaphore_from_handle(src->pWaitSemaphores[i]);
         if (!sem)
            goto invalid;
         const uint64_t value =
            sem->type == VK_SEMAPHORE_TYPE_TIMELINE
               ? (timeline ? timeline->pWaitSemaphoreValues[i] : UINT64_MAX)
               : 1;
         if (value == UINT64_MAX && !timeline)
            goto invalid;
         bool native = false;
         failure = helios_classify_semaphore(dev, src->pWaitSemaphores[i],
                                             value, &copy->waits, &native);
         if (failure != VK_SUCCESS)
            goto result_fail;
         if (native)
            continue;
         aux->waits[kept_waits] = src->pWaitSemaphores[i];
         if (timeline)
            aux->wait_values[kept_waits] = timeline->pWaitSemaphoreValues[i];
         kept_waits++;
      }
      uint32_t kept_signals = 0;
      for (uint32_t i = 0; i < src->signalSemaphoreCount; i++) {
         struct vn_semaphore *sem =
            vn_semaphore_from_handle(src->pSignalSemaphores[i]);
         if (!sem)
            goto invalid;
         const uint64_t value =
            sem->type == VK_SEMAPHORE_TYPE_TIMELINE
               ? (timeline ? timeline->pSignalSemaphoreValues[i] : UINT64_MAX)
               : 1;
         if (value == UINT64_MAX && !timeline)
            goto invalid;
         bool native = false;
         failure = helios_classify_semaphore(dev, src->pSignalSemaphores[i],
                                             value, &copy->signals, &native);
         if (failure != VK_SUCCESS)
            goto result_fail;
         if (native)
            continue;
         aux->signals[kept_signals] = src->pSignalSemaphores[i];
         if (timeline)
            aux->signal_values[kept_signals] =
               timeline->pSignalSemaphoreValues[i];
         kept_signals++;
      }

      failure = helios_sparse_collect_uses(copy, dev, src);
      if (failure != VK_SUCCESS)
         goto result_fail;
      *dst = *src;
      dst->pNext = NULL;
      dst->waitSemaphoreCount = kept_waits;
      dst->pWaitSemaphores = kept_waits ? aux->waits : NULL;
      dst->signalSemaphoreCount = kept_signals;
      dst->pSignalSemaphores = kept_signals ? aux->signals : NULL;

      const void **tail = &dst->pNext;
      vk_foreach_struct_const(ext, src->pNext) {
         if (ext->sType == VK_STRUCTURE_TYPE_DEVICE_GROUP_BIND_SPARSE_INFO) {
            aux->group = *(const VkDeviceGroupBindSparseInfo *)ext;
            aux->group.pNext = NULL;
            *tail = &aux->group;
            tail = &aux->group.pNext;
         } else {
            aux->timeline = *(const VkTimelineSemaphoreSubmitInfo *)ext;
            aux->timeline.pNext = NULL;
            aux->timeline.waitSemaphoreValueCount = kept_waits;
            aux->timeline.pWaitSemaphoreValues =
               kept_waits ? aux->wait_values : NULL;
            aux->timeline.signalSemaphoreValueCount = kept_signals;
            aux->timeline.pSignalSemaphoreValues =
               kept_signals ? aux->signal_values : NULL;
            *tail = &aux->timeline;
            tail = &aux->timeline.pNext;
         }
      }
      *tail = NULL;
   }
   return VK_SUCCESS;

result_fail:
   helios_sparse_copy_fini(copy);
   return failure;
unsupported:
   helios_sparse_copy_fini(copy);
   return VK_ERROR_FEATURE_NOT_PRESENT;
invalid:
   helios_sparse_copy_fini(copy);
   return VK_ERROR_INITIALIZATION_FAILED;
oom:
   helios_sparse_copy_fini(copy);
   return VK_ERROR_OUT_OF_HOST_MEMORY;
}

static VkResult
helios_record_entry_gate(struct vn_queue *queue,
                         enum helios_record_refusal refusal)
{
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   if (!owner)
      return VK_ERROR_INITIALIZATION_FAILED;
   if (owner->mode != VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return VK_SUCCESS;
   struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
   if (!scope || !helios_scope_on_calling_thread(scope)) {
      helios_record_refuse(owner, refusal);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (scope->context->owner != owner || scope->context->queue != queue) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   return VK_SUCCESS;
}

static VkResult
helios_submit1_deferred_use_gate(struct vn_helios_submit_instance *owner,
                                 uint32_t count,
                                 const VkSubmitInfo *submits)
{
   for (uint32_t i = 0; i < count; i++) {
      if (submits[i].commandBufferCount) {
         /* A7 owns the generated command-buffer/object graph that can prove
          * the complete WDDM allocation closure.  Empty/semaphore-only queue
          * work has an exact empty closure; command-buffer work does not yet.
          */
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
   }
   return VK_SUCCESS;
}

static VkResult
helios_submit2_deferred_use_gate(struct vn_helios_submit_instance *owner,
                                 uint32_t count,
                                 const VkSubmitInfo2 *submits)
{
   for (uint32_t i = 0; i < count; i++) {
      if (submits[i].commandBufferInfoCount) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
   }
   return VK_SUCCESS;
}

static VkResult
helios_validate_fence(struct vn_device *dev, VkFence handle)
{
   if (handle == VK_NULL_HANDLE)
      return VK_SUCCESS;
   struct vn_fence *fence = vn_fence_from_handle(handle);
   if (fence && helios_object_owned(&fence->base.vk, dev))
      return VK_SUCCESS;
   helios_record_refuse(helios_submit_owner(dev->instance),
                        HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
   return VK_ERROR_VALIDATION_FAILED_EXT;
}

/* ── Vulkan queue entry seams ───────────────────────────────────────────── */

VkResult
vn_helios_queue_submit(struct vn_queue *queue,
                       uint32_t submit_count,
                       const VkSubmitInfo *submits,
                       VkFence fence)
{
   VkResult result =
      helios_record_entry_gate(queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT);
   if (result != VK_SUCCESS)
      return result;
   if (submit_count > HELIOS_HOB1_MAX_OPERAND_RECORDS)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   if (submit_count && !submits)
      return VK_ERROR_INITIALIZATION_FAILED;
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   result = helios_validate_fence(dev, fence);
   if (result != VK_SUCCESS)
      return result;
   result = helios_submit1_deferred_use_gate(owner, submit_count, submits);
   if (result != VK_SUCCESS)
      return result;
   const size_t bounded_payload = vn_sizeof_vkQueueSubmit(
      vn_queue_to_handle(queue), submit_count, submits, fence);
   if (!bounded_payload || bounded_payload > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   const enum vn_helios_submission_mode mode =
      vn_helios_submit_instance_mode(dev->instance);
   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      struct helios_submit1_copy copy;
      result = helios_submit1_copy_init(dev, submit_count, submits, &copy);
      if (result != VK_SUCCESS)
         return result;
      if (copy.waits.count || copy.signals.count) {
         /* HOB1 has no native-fence carrier: those dependencies belong to the
          * exact outer D3D context.  A5 may arrange that ordering around the
          * sealed batch, but A4 must not silently discard it or issue KMT
          * work from record-only mode. */
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
         helios_submit1_copy_fini(&copy);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      void *payload = NULL;
      size_t payload_bytes = 0;
      result =
         helios_encode_submit1(vn_queue_to_handle(queue), submit_count,
                               copy.infos, fence, &payload, &payload_bytes);
      if (result == VK_SUCCESS)
         result = helios_dispatch_payload(
            queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT, payload, payload_bytes,
            NULL, 0, &copy.waits, &copy.signals);
      free(payload);
      helios_submit1_copy_fini(&copy);
      return result;
   }
   if (mode == VN_HELIOS_SUBMISSION_MODE_NORMAL) {
      if (!submit_count && fence == VK_NULL_HANDLE)
         return VK_SUCCESS;
      vn_device_memory_flush_coherent_cached_mappings(dev);
   }

   const uint32_t step_count = submit_count ? submit_count : 1;
   struct helios_queue_step *steps = calloc(step_count, sizeof(*steps));
   if (!steps)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   uint32_t point_count = 0;
   uint64_t total_payload_bytes = 0;

   for (uint32_t i = 0; i < step_count; i++) {
      const uint32_t one_count = submit_count ? 1 : 0;
      const VkSubmitInfo *one_submit = submit_count ? &submits[i] : NULL;
      struct helios_submit1_copy copy;
      result = helios_submit1_copy_init(dev, one_count, one_submit, &copy);
      if (result != VK_SUCCESS)
         goto out;
      result = helios_encode_submit1(
         vn_queue_to_handle(queue), one_count, copy.infos,
         i + 1 == step_count ? fence : VK_NULL_HANDLE, &steps[i].payload,
         &steps[i].payload_bytes);
      if (result == VK_SUCCESS) {
         if (steps[i].payload_bytes >
             HELIOS_HNR2_MAX_PAYLOAD_BYTES - total_payload_bytes) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
         } else {
            total_payload_bytes += steps[i].payload_bytes;
         }
      }
      if (result == VK_SUCCESS) {
         steps[i].waits = copy.waits;
         steps[i].signals = copy.signals;
         memset(&copy.waits, 0, sizeof(copy.waits));
         memset(&copy.signals, 0, sizeof(copy.signals));
         if (!helios_queue_step_points_bounded(&point_count, &steps[i]))
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
      }
      helios_submit1_copy_fini(&copy);
      if (result != VK_SUCCESS)
         goto out;
   }

   for (uint32_t i = 0; i < step_count; i++) {
      result = helios_dispatch_payload(
         queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT, steps[i].payload,
         steps[i].payload_bytes, NULL, 0, &steps[i].waits, &steps[i].signals);
      if (result != VK_SUCCESS)
         break;
   }

out:
   helios_queue_steps_fini(steps, step_count);
   return result;
}

VkResult
vn_helios_queue_submit2(struct vn_queue *queue,
                        uint32_t submit_count,
                        const VkSubmitInfo2 *submits,
                        VkFence fence)
{
   VkResult result =
      helios_record_entry_gate(queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT2);
   if (result != VK_SUCCESS)
      return result;
   if (submit_count > HELIOS_HOB1_MAX_OPERAND_RECORDS)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   if (submit_count && !submits)
      return VK_ERROR_INITIALIZATION_FAILED;
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   result = helios_validate_fence(dev, fence);
   if (result != VK_SUCCESS)
      return result;
   result = helios_submit2_deferred_use_gate(owner, submit_count, submits);
   if (result != VK_SUCCESS)
      return result;
   const size_t bounded_payload = vn_sizeof_vkQueueSubmit2(
      vn_queue_to_handle(queue), submit_count, submits, fence);
   if (!bounded_payload || bounded_payload > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   const enum vn_helios_submission_mode mode =
      vn_helios_submit_instance_mode(dev->instance);
   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      struct helios_submit2_copy copy;
      result = helios_submit2_copy_init(dev, submit_count, submits, &copy);
      if (result != VK_SUCCESS)
         return result;
      if (copy.waits.count || copy.signals.count) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
         helios_submit2_copy_fini(&copy);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      void *payload = NULL;
      size_t payload_bytes = 0;
      result =
         helios_encode_submit2(vn_queue_to_handle(queue), submit_count,
                               copy.infos, fence, &payload, &payload_bytes);
      if (result == VK_SUCCESS)
         result = helios_dispatch_payload(
            queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT2, payload, payload_bytes,
            NULL, 0, &copy.waits, &copy.signals);
      free(payload);
      helios_submit2_copy_fini(&copy);
      return result;
   }
   if (mode == VN_HELIOS_SUBMISSION_MODE_NORMAL) {
      if (!submit_count && fence == VK_NULL_HANDLE)
         return VK_SUCCESS;
      vn_device_memory_flush_coherent_cached_mappings(dev);
   }

   const uint32_t step_count = submit_count ? submit_count : 1;
   struct helios_queue_step *steps = calloc(step_count, sizeof(*steps));
   if (!steps)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   uint32_t point_count = 0;
   uint64_t total_payload_bytes = 0;

   for (uint32_t i = 0; i < step_count; i++) {
      const uint32_t one_count = submit_count ? 1 : 0;
      const VkSubmitInfo2 *one_submit = submit_count ? &submits[i] : NULL;
      struct helios_submit2_copy copy;
      result = helios_submit2_copy_init(dev, one_count, one_submit, &copy);
      if (result != VK_SUCCESS)
         goto out;
      result = helios_encode_submit2(
         vn_queue_to_handle(queue), one_count, copy.infos,
         i + 1 == step_count ? fence : VK_NULL_HANDLE, &steps[i].payload,
         &steps[i].payload_bytes);
      if (result == VK_SUCCESS) {
         if (steps[i].payload_bytes >
             HELIOS_HNR2_MAX_PAYLOAD_BYTES - total_payload_bytes) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
         } else {
            total_payload_bytes += steps[i].payload_bytes;
         }
      }
      if (result == VK_SUCCESS) {
         steps[i].waits = copy.waits;
         steps[i].signals = copy.signals;
         memset(&copy.waits, 0, sizeof(copy.waits));
         memset(&copy.signals, 0, sizeof(copy.signals));
         if (!helios_queue_step_points_bounded(&point_count, &steps[i]))
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
      }
      helios_submit2_copy_fini(&copy);
      if (result != VK_SUCCESS)
         goto out;
   }

   for (uint32_t i = 0; i < step_count; i++) {
      result = helios_dispatch_payload(
         queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT2, steps[i].payload,
         steps[i].payload_bytes, NULL, 0, &steps[i].waits, &steps[i].signals);
      if (result != VK_SUCCESS)
         break;
   }

out:
   helios_queue_steps_fini(steps, step_count);
   return result;
}

VkResult
vn_helios_queue_bind_sparse(struct vn_queue *queue,
                            uint32_t bind_count,
                            const VkBindSparseInfo *binds,
                            VkFence fence)
{
   VkResult result =
      helios_record_entry_gate(queue, HELIOS_RECORD_REFUSE_QUEUE_BIND_SPARSE);
   if (result != VK_SUCCESS)
      return result;
   if (bind_count > HELIOS_HOB1_MAX_OPERAND_RECORDS)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   if (bind_count && !binds)
      return VK_ERROR_INITIALIZATION_FAILED;
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   result = helios_validate_fence(dev, fence);
   if (result != VK_SUCCESS)
      return result;
   const size_t bounded_payload = vn_sizeof_vkQueueBindSparse(
      vn_queue_to_handle(queue), bind_count, binds, fence);
   if (!bounded_payload || bounded_payload > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   const enum vn_helios_submission_mode mode =
      vn_helios_submit_instance_mode(dev->instance);
   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      struct helios_sparse_copy copy;
      result = helios_sparse_copy_init(dev, bind_count, binds, &copy);
      if (result != VK_SUCCESS)
         return result;
      if (copy.waits.count || copy.signals.count) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
         helios_sparse_copy_fini(&copy);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      void *payload = NULL;
      size_t payload_bytes = 0;
      result = helios_encode_bind_sparse(vn_queue_to_handle(queue),
                                         bind_count, copy.infos, fence,
                                         &payload, &payload_bytes);
      if (result == VK_SUCCESS)
         result = helios_dispatch_payload(
            queue, HELIOS_RECORD_REFUSE_QUEUE_BIND_SPARSE, payload,
            payload_bytes, copy.allocations, copy.allocation_count,
            &copy.waits, &copy.signals);
      free(payload);
      helios_sparse_copy_fini(&copy);
      return result;
   }
   if (mode == VN_HELIOS_SUBMISSION_MODE_NORMAL) {
      if (!bind_count && fence == VK_NULL_HANDLE)
         return VK_SUCCESS;
      vn_device_memory_flush_coherent_cached_mappings(dev);
   }

   const uint32_t step_count = bind_count ? bind_count : 1;
   struct helios_queue_step *steps = calloc(step_count, sizeof(*steps));
   if (!steps)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   uint32_t point_count = 0;
   uint64_t total_payload_bytes = 0;

   for (uint32_t i = 0; i < step_count; i++) {
      const uint32_t one_count = bind_count ? 1 : 0;
      const VkBindSparseInfo *one_bind = bind_count ? &binds[i] : NULL;
      struct helios_sparse_copy copy;
      result = helios_sparse_copy_init(dev, one_count, one_bind, &copy);
      if (result != VK_SUCCESS)
         goto out;
      result = helios_encode_bind_sparse(
         vn_queue_to_handle(queue), one_count, copy.infos,
         i + 1 == step_count ? fence : VK_NULL_HANDLE, &steps[i].payload,
         &steps[i].payload_bytes);
      if (result == VK_SUCCESS) {
         if (steps[i].payload_bytes >
             HELIOS_HNR2_MAX_PAYLOAD_BYTES - total_payload_bytes) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
         } else {
            total_payload_bytes += steps[i].payload_bytes;
         }
      }
      if (result == VK_SUCCESS && copy.allocation_count) {
         const size_t bytes =
            (size_t)copy.allocation_count * sizeof(*steps[i].allocations);
         steps[i].allocations = malloc(bytes);
         if (!steps[i].allocations) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
         } else {
            memcpy(steps[i].allocations, copy.allocations, bytes);
            steps[i].allocation_count = copy.allocation_count;
         }
      }
      if (result == VK_SUCCESS) {
         steps[i].waits = copy.waits;
         steps[i].signals = copy.signals;
         memset(&copy.waits, 0, sizeof(copy.waits));
         memset(&copy.signals, 0, sizeof(copy.signals));
         if (!helios_queue_step_points_bounded(&point_count, &steps[i]))
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
      }
      helios_sparse_copy_fini(&copy);
      if (result != VK_SUCCESS)
         goto out;
   }

   for (uint32_t i = 0; i < step_count; i++) {
      result = helios_dispatch_payload(
         queue, HELIOS_RECORD_REFUSE_QUEUE_BIND_SPARSE, steps[i].payload,
         steps[i].payload_bytes, steps[i].allocations,
         steps[i].allocation_count, &steps[i].waits, &steps[i].signals);
      if (result != VK_SUCCESS)
         break;
   }

out:
   helios_queue_steps_fini(steps, step_count);
   return result;
}

VkResult
vn_helios_queue_wait_idle(struct vn_queue *queue)
{
   if (!queue)
      return VK_ERROR_INITIALIZATION_FAILED;
   VkResult result =
      helios_record_entry_gate(queue, HELIOS_RECORD_REFUSE_QUEUE_WAIT_IDLE);
   if (result != VK_SUCCESS)
      return result;
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   if (owner->mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      /* The record-only join is the A5 host-callback cut-and-reopen edge.  A4
       * owns the live-scope gate, but does not invent a lower queue or
       * control C51 completion while that callback is deliberately still
       * unexposed. */
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
      return VK_ERROR_FEATURE_NOT_PRESENT;
   }
   if (!queue->helios_native_context)
      return VK_ERROR_DEVICE_LOST;
   return helios_native_queue_wait_idle(queue->helios_native_context);
}

VkResult
vn_helios_device_wait_idle(struct vn_device *dev)
{
   if (!dev)
      return VK_ERROR_INITIALIZATION_FAILED;
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   if (!owner)
      return VK_ERROR_INITIALIZATION_FAILED;
   if (owner->mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      /* Device idle has the same A5 cut-and-reopen dependency as queue idle,
       * and is still scope-gated rather than silently promoted to control. */
      struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
      if (!scope || !helios_scope_on_calling_thread(scope)) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEVICE_WAIT_IDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      if (vn_device_from_vk(scope->context->queue->base.vk.base.device) !=
          dev) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
      return VK_ERROR_FEATURE_NOT_PRESENT;
   }

   struct helios_native_context **contexts =
      calloc(MAX2(dev->queue_count, 1), sizeof(*contexts));
   if (!contexts)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   uint32_t count = 0;
   for (uint32_t i = 0; i < dev->queue_count; i++) {
      struct vn_queue *queue = &dev->queues[i];
      if (queue->helios_native_context_owner && queue->helios_native_context)
         contexts[count++] = queue->helios_native_context;
   }
   const VkResult result = helios_native_device_wait_idle(contexts, count);
   free(contexts);
   return result;
}

#endif /* _WIN32 */
