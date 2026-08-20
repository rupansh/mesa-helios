/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A5 -- one direct-owned record-only translator object per vn_instance.
 */

#ifdef _WIN32

#include "vn_helios_direct_dispatch.h"

#include <stdlib.h>
#include <string.h>
#include <windows.h>

#include "vulkan/helios_private_wsi.h"
#include "vn_helios_record_submit.h"
#include "vn_instance.h"
#include "vn_queue.h"
#include "vn_renderer.h"

enum helios_direct_context_state {
   HELIOS_DIRECT_CONTEXT_PENDING,
   HELIOS_DIRECT_CONTEXT_ATTACHING,
   HELIOS_DIRECT_CONTEXT_ATTACHED,
   HELIOS_DIRECT_CONTEXT_DETACHING,
   /* The outer context is gone, but its final, actually-joined HQC1 result is
    * retained until instance teardown.  Frontend objects can therefore finish
    * an exact producer join without turning a dead generation into a lookup or
    * a synthetic completion.  One context is admitted per non-reused endpoint,
    * so this tombstone set remains bounded by the fixed endpoint table. */
   HELIOS_DIRECT_CONTEXT_RETIRED,
};

struct helios_direct_endpoint {
   HeliosTranslationEndpointV1 desc;
   struct vn_queue *queue;
   bool ever_registered;
   bool live;
};

struct helios_direct_context {
   struct helios_direct_context *next;
   struct vn_helios_record_context *record;
   struct vn_queue *queue;
   void *host_context_cookie;
   uint64_t generation;
   uint64_t last_progress;
   HeliosSyncProgressResultV1 retired_result;
   uint32_t endpoint_id;
   uint32_t context_flags;
   uint32_t active_host_calls;
   enum helios_direct_context_state state;
};

struct HeliosTranslatorInstance_T {
   struct vn_instance *instance;
   HeliosTranslatorHostCallbacksV1 host;
   HeliosTranslatorDispatchV1 dispatch;
   struct helios_direct_endpoint
      endpoints[HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION + 1u];
   struct helios_direct_context *contexts;
   uint64_t last_context_generation;
   uint32_t context_count;
   uint32_t endpoint_count;
   mtx_t lock;
   bool lock_live;
   cnd_t context_idle;
   bool context_idle_live;
   tss_t join_key;
   bool join_key_live;
   bool poisoned;
   bool destroying;
};

static struct HeliosTranslatorInstance_T *
helios_direct_from_handle(HeliosTranslatorHandle handle)
{
   struct HeliosTranslatorInstance_T *direct = handle;
   if (!direct || !direct->instance ||
       direct->instance->helios_direct != direct ||
       vn_helios_submit_instance_mode(direct->instance) !=
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return NULL;
   return direct;
}

static struct HeliosTranslatorInstance_T *
helios_direct_from_vk_instance(void *vk_instance)
{
   if (!vk_instance)
      return NULL;
   struct vn_instance *instance =
      vn_instance_from_handle((VkInstance)vk_instance);
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance ||
       vn_helios_submit_instance_mode(instance) !=
          VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY)
      return NULL;
   return direct;
}

static HMODULE
helios_module_from_address(const void *address)
{
   HMODULE module = NULL;
   if (!address ||
       !GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                             GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          (LPCWSTR)address, &module))
      return NULL;
   return module;
}

static bool
helios_direct_withheld_proc(const char *name)
{
   if (!name || !strcmp(name, "vkCreateInstance") ||
       !strcmp(name, "vkGetInstanceProcAddr") ||
       !strcmp(name, HELIOS_SET_PRESENTABLE_IMAGE_NAME))
      return true;

   /* These are API-name classes, not resource identity.  Withhold the whole
    * lower WSI surface so an extension added to vk.xml cannot accidentally
    * become a second present path without changing this explicit boundary. */
   return strstr(name, "Surface") || strstr(name, "Swapchain") ||
          strstr(name, "Present") || strstr(name, "AcquireNextImage");
}

static uint32_t
helios_direct_engine_class(VkQueueFlags flags)
{
   if (flags & VK_QUEUE_GRAPHICS_BIT)
      return HELIOS_ENGINE_CLASS_GRAPHICS;
   if (flags & VK_QUEUE_COMPUTE_BIT)
      return HELIOS_ENGINE_CLASS_COMPUTE;
   if (flags & VK_QUEUE_TRANSFER_BIT)
      return HELIOS_ENGINE_CLASS_COPY;
   return 0;
}

VkResult
vn_helios_direct_register_queue(struct vn_instance *instance,
                                struct vn_queue *queue,
                                uint32_t queue_family,
                                uint32_t queue_index,
                                VkQueueFlags queue_flags)
{
   if (!instance || !queue)
      return VK_ERROR_INITIALIZATION_FAILED;
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance)
      return VK_ERROR_INITIALIZATION_FAILED;

   const uint32_t endpoint_id = queue->ring_idx;
   const uint32_t endpoint_capacity =
      vn_renderer_helios_endpoint_capacity(instance->renderer);
   const uint32_t engine_class = helios_direct_engine_class(queue_flags);
   if (queue->emulated || endpoint_id < 2 ||
       endpoint_id > endpoint_capacity || !engine_class)
      return VK_ERROR_INITIALIZATION_FAILED;

   mtx_lock(&direct->lock);
   struct helios_direct_endpoint *endpoint =
      &direct->endpoints[endpoint_id];
   const bool admissible = !direct->poisoned && !direct->destroying &&
                           !endpoint->ever_registered && !endpoint->live &&
                           direct->endpoint_count < endpoint_capacity;
   if (admissible) {
      endpoint->desc = (HeliosTranslationEndpointV1){
         .endpoint_id = endpoint_id,
         .engine_class = engine_class,
         .queue_family = queue_family,
         .queue_index = queue_index,
      };
      endpoint->queue = queue;
      endpoint->ever_registered = true;
      endpoint->live = true;
      direct->endpoint_count++;
   }
   mtx_unlock(&direct->lock);
   return admissible ? VK_SUCCESS : VK_ERROR_INITIALIZATION_FAILED;
}

void
vn_helios_direct_unregister_queue(struct vn_instance *instance,
                                  struct vn_queue *queue)
{
   if (!instance || !queue)
      return;
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance)
      return;

   mtx_lock(&direct->lock);
   const uint32_t endpoint_id = queue->ring_idx;
   if (endpoint_id <= HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION) {
      struct helios_direct_endpoint *endpoint =
         &direct->endpoints[endpoint_id];
      if (endpoint->live && endpoint->queue == queue) {
         for (struct helios_direct_context *context = direct->contexts;
              context; context = context->next) {
            if (context->endpoint_id == endpoint_id &&
                context->state != HELIOS_DIRECT_CONTEXT_RETIRED) {
               direct->poisoned = true;
               break;
            }
         }
         endpoint->live = false;
         endpoint->queue = NULL;
         if (direct->endpoint_count)
            direct->endpoint_count--;
      }
   }
   mtx_unlock(&direct->lock);
}

static PFN_helios_translator_void_function HELIOS_TRANSLATOR_CALL
helios_direct_get_instance_proc_addr(void *vk_instance, const char *name)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_vk_instance(vk_instance);
   if (!direct)
      return NULL;
   if (helios_direct_withheld_proc(name)) {
      vn_helios_record_note_withheld_proc(direct->instance);
      return NULL;
   }

   PFN_vkVoidFunction proc =
      vn_GetInstanceProcAddr((VkInstance)vk_instance, name);
   if (!proc)
      return NULL;
   HMODULE module = helios_module_from_address((const void *)proc);
   if (!module || (const void *)module != direct->dispatch.icd_module_base) {
      vn_helios_record_note_loader_provenance(direct->instance);
      return NULL;
   }
   return (PFN_helios_translator_void_function)proc;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_enumerate_endpoints(
   HeliosTranslatorHandle handle,
   uint32_t *endpoint_count,
   HeliosTranslationEndpointV1 *endpoints,
   uint32_t endpoint_bytes)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct || !endpoint_count)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   if (endpoint_bytes != HELIOS_TRANSLATOR_ENDPOINT_BYTES)
      return HELIOS_TRANSLATOR_STATUS_STRUCT_BYTES;

   mtx_lock(&direct->lock);
   if (direct->poisoned || direct->destroying) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
   }
   const uint32_t count = direct->endpoint_count;
   if (!endpoints) {
      *endpoint_count = count;
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_OK;
   }
   if (*endpoint_count < count) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;
   }
   uint32_t written = 0;
   for (uint32_t i = 1; i <= HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION; i++) {
      if (direct->endpoints[i].live)
         endpoints[written++] = direct->endpoints[i].desc;
   }
   *endpoint_count = written;
   mtx_unlock(&direct->lock);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

static struct helios_direct_context *
helios_direct_find_context(struct HeliosTranslatorInstance_T *direct,
                           uint64_t generation)
{
   for (struct helios_direct_context *context = direct->contexts; context;
        context = context->next) {
      if (context->generation == generation)
         return context;
   }
   return NULL;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_build_queue_attach(HeliosTranslatorHandle handle,
                                 const HeliosQueueAttachRequestV1 *request,
                                 HeliosQueueAttachV1 *out_hqa1,
                                 uint32_t out_hqa1_bytes)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct || !out_hqa1)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   if (out_hqa1_bytes != HELIOS_TRANSLATOR_HQA1_BYTES)
      return HELIOS_TRANSLATOR_STATUS_STRUCT_BYTES;
   memset(out_hqa1, 0, sizeof(*out_hqa1));
   HeliosTranslatorStatusCode checked =
      helios_translator_check_queue_attach_request(request);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;

   struct helios_direct_context *context = calloc(1, sizeof(*context));
   if (!context)
      return HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;

   mtx_lock(&direct->lock);
   if (direct->poisoned || direct->destroying) {
      checked = HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
      goto unlock;
   }
   if (direct->context_count >= HELIOS_HNR2_MAX_USE_RECORDS) {
      checked = HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;
      goto unlock;
   }
   if (request->context_generation <= direct->last_context_generation ||
       helios_direct_find_context(direct, request->context_generation)) {
      checked = HELIOS_TRANSLATOR_STATUS_CONTEXT_GENERATION;
      goto unlock;
   }
   if (request->endpoint_id > HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION ||
       !direct->endpoints[request->endpoint_id].live) {
      checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ENDPOINT;
      goto unlock;
   }
   struct helios_direct_endpoint *endpoint =
      &direct->endpoints[request->endpoint_id];
   if (endpoint->desc.engine_class != request->engine_class) {
      checked = HELIOS_TRANSLATOR_STATUS_ENGINE_CLASS;
      goto unlock;
   }
   for (struct helios_direct_context *known = direct->contexts; known;
        known = known->next) {
      if (known->endpoint_id == request->endpoint_id) {
         /* Endpoints 2..64 are monotonic and non-reused.  A second context on
          * one endpoint would make later object progress ambiguous. */
         checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ENDPOINT;
         goto unlock;
      }
   }

   if (vn_renderer_helios_build_queue_attach(
          direct->instance->renderer, &endpoint->desc, request,
          out_hqa1) != VK_SUCCESS) {
      checked = HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
      goto unlock;
   }
   context->queue = endpoint->queue;
   context->generation = request->context_generation;
   context->endpoint_id = request->endpoint_id;
   context->context_flags = request->context_flags;
   context->state = HELIOS_DIRECT_CONTEXT_PENDING;
   context->next = direct->contexts;
   direct->contexts = context;
   direct->context_count++;
   direct->last_context_generation = request->context_generation;
   mtx_unlock(&direct->lock);
   return HELIOS_TRANSLATOR_STATUS_OK;

unlock:
   mtx_unlock(&direct->lock);
   free(context);
   memset(out_hqa1, 0, sizeof(*out_hqa1));
   return checked;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_attach_outer_context(HeliosTranslatorHandle handle,
                                   const HeliosOuterContextAttachV1 *attach)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   HeliosTranslatorStatusCode checked =
      helios_translator_check_context_attach(attach);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;

   mtx_lock(&direct->lock);
   struct helios_direct_context *context =
      helios_direct_find_context(direct, attach->context_generation);
   if (!context) {
      checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   } else if (context->endpoint_id != attach->endpoint_id) {
      checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ENDPOINT;
   } else if (context->context_flags != attach->context_flags) {
      checked = HELIOS_TRANSLATOR_STATUS_CONTEXT_FLAGS;
   } else if (context->state != HELIOS_DIRECT_CONTEXT_PENDING) {
      checked = context->state == HELIOS_DIRECT_CONTEXT_ATTACHED
                   ? HELIOS_TRANSLATOR_STATUS_CONTEXT_ALREADY_ATTACHED
                   : HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   } else if (direct->poisoned || direct->destroying) {
      checked = HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
   } else {
      context->state = HELIOS_DIRECT_CONTEXT_ATTACHING;
      checked = HELIOS_TRANSLATOR_STATUS_OK;
   }
   mtx_unlock(&direct->lock);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;

   struct vn_helios_record_context *record = NULL;
   checked = vn_helios_record_context_create(
      direct->instance, context->queue, context->endpoint_id,
      context->generation, context->context_flags, &record);

   mtx_lock(&direct->lock);
   if (checked == HELIOS_TRANSLATOR_STATUS_OK &&
       context->state == HELIOS_DIRECT_CONTEXT_ATTACHING) {
      context->record = record;
      context->host_context_cookie = attach->host_context_cookie;
      context->state = HELIOS_DIRECT_CONTEXT_ATTACHED;
   } else {
      context->state = HELIOS_DIRECT_CONTEXT_PENDING;
   }
   mtx_unlock(&direct->lock);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK && record)
      (void)vn_helios_record_context_destroy(record);
   return checked;
}

static HeliosTranslatorStatusCode
helios_direct_join(struct HeliosTranslatorInstance_T *direct,
                   struct helios_direct_context *context,
                   uint64_t required_progress,
                   HeliosSyncProgressResultV1 *out_result)
{
   if (tss_get(direct->join_key)) {
      vn_helios_record_note_reentrant_join(direct->instance);
      return HELIOS_TRANSLATOR_STATUS_REENTRANT_JOIN;
   }
   if (tss_set(direct->join_key, direct) != thrd_success)
      return HELIOS_TRANSLATOR_STATUS_HOST_CALLBACK_FAILED;

   const HeliosSyncProgressJoinV1 request = {
      .struct_bytes = sizeof(request),
      .abi_version = HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION,
      .required_progress_value = required_progress,
      .context_generation = context->generation,
   };
   HeliosSyncProgressResultV1 result;
   memset(&result, 0, sizeof(result));
   HeliosTranslatorStatusCode status = direct->host.sync_progress_join(
      context->host_context_cookie, &request, &result);
   (void)tss_set(direct->join_key, NULL);
   if (status < HELIOS_TRANSLATOR_STATUS_OK ||
       status > HELIOS_TRANSLATOR_STATUS_MAX)
      return HELIOS_TRANSLATOR_STATUS_HOST_CALLBACK_FAILED;
   if (status != HELIOS_TRANSLATOR_STATUS_OK)
      return status;
   status = helios_translator_check_join_result(&result, required_progress);
   if (status == HELIOS_TRANSLATOR_STATUS_OK && out_result)
      *out_result = result;
   return status;
}

static struct helios_direct_context *
helios_direct_acquire_context(struct HeliosTranslatorInstance_T *direct,
                              uint64_t context_generation)
{
   struct helios_direct_context *context = NULL;
   mtx_lock(&direct->lock);
   context = helios_direct_find_context(direct, context_generation);
   if (!context || context->state != HELIOS_DIRECT_CONTEXT_ATTACHED ||
       context->active_host_calls == UINT32_MAX || direct->poisoned ||
       direct->destroying) {
      context = NULL;
   } else {
      context->active_host_calls++;
   }
   mtx_unlock(&direct->lock);
   return context;
}

static void
helios_direct_release_context(struct HeliosTranslatorInstance_T *direct,
                              struct helios_direct_context *context)
{
   mtx_lock(&direct->lock);
   if (context->active_host_calls)
      context->active_host_calls--;
   if (!context->active_host_calls && direct->context_idle_live)
      cnd_broadcast(&direct->context_idle);
   mtx_unlock(&direct->lock);
}

HeliosTranslatorStatusCode
vn_helios_direct_join_context(struct vn_instance *instance,
                              uint64_t context_generation,
                              uint64_t required_progress,
                              HeliosSyncProgressResultV1 *out_result)
{
   if (!instance || !context_generation)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance)
      return HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
   mtx_lock(&direct->lock);
   struct helios_direct_context *known =
      helios_direct_find_context(direct, context_generation);
   if (known && known->state == HELIOS_DIRECT_CONTEXT_RETIRED &&
       !direct->poisoned && !direct->destroying) {
      const HeliosSyncProgressResultV1 result = known->retired_result;
      mtx_unlock(&direct->lock);
      const HeliosTranslatorStatusCode status =
         helios_translator_check_join_result(&result, required_progress);
      if (status == HELIOS_TRANSLATOR_STATUS_OK && out_result)
         *out_result = result;
      return status;
   }
   mtx_unlock(&direct->lock);
   struct helios_direct_context *context =
      helios_direct_acquire_context(direct, context_generation);
   if (!context)
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   const HeliosTranslatorStatusCode status =
      helios_direct_join(direct, context, required_progress, out_result);
   helios_direct_release_context(direct, context);
   return status;
}

HeliosTranslatorStatusCode
vn_helios_direct_join_current(struct vn_instance *instance,
                              uint64_t required_progress,
                              HeliosSyncProgressResultV1 *out_result)
{
   uint64_t context_generation = 0;
   if (!vn_helios_record_current_context(instance, &context_generation))
      return HELIOS_TRANSLATOR_STATUS_NO_OUTER_SCOPE;
   return vn_helios_direct_join_context(instance, context_generation,
                                        required_progress, out_result);
}

HeliosTranslatorStatusCode
vn_helios_direct_join_all(struct vn_instance *instance)
{
   if (!instance)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance)
      return HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;

   uint64_t generations[HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION];
   uint64_t progress[HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION];
   uint32_t count = 0;
   mtx_lock(&direct->lock);
   if (direct->poisoned || direct->destroying) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
   }
   for (struct helios_direct_context *context = direct->contexts; context;
        context = context->next) {
      if (context->state != HELIOS_DIRECT_CONTEXT_ATTACHED)
         continue;
      if (count == ARRAY_SIZE(generations)) {
         mtx_unlock(&direct->lock);
         return HELIOS_TRANSLATOR_STATUS_ENDPOINT_CAPACITY;
      }
      generations[count] = context->generation;
      progress[count] = context->last_progress;
      count++;
   }
   mtx_unlock(&direct->lock);

   for (uint32_t i = 0; i < count; i++) {
      HeliosSyncProgressResultV1 result;
      memset(&result, 0, sizeof(result));
      const HeliosTranslatorStatusCode status =
         vn_helios_direct_join_context(instance, generations[i], progress[i],
                                       &result);
      if (status != HELIOS_TRANSLATOR_STATUS_OK)
         return status;
      if (result.flags & HELIOS_TRANSLATOR_PROGRESS_FLAG_DEVICE_LOST)
         return HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
   }
   return HELIOS_TRANSLATOR_STATUS_OK;
}

HeliosTranslatorStatusCode
vn_helios_direct_query_context(struct vn_instance *instance,
                               uint64_t context_generation,
                               HeliosSyncProgressResultV1 *out_result)
{
   if (!instance || !context_generation || !out_result)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance)
      return HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
   mtx_lock(&direct->lock);
   struct helios_direct_context *known =
      helios_direct_find_context(direct, context_generation);
   if (known && known->state == HELIOS_DIRECT_CONTEXT_RETIRED &&
       !direct->poisoned && !direct->destroying) {
      *out_result = known->retired_result;
      mtx_unlock(&direct->lock);
      return helios_translator_check_query_result(out_result);
   }
   mtx_unlock(&direct->lock);
   struct helios_direct_context *context =
      helios_direct_acquire_context(direct, context_generation);
   if (!context)
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   memset(out_result, 0, sizeof(*out_result));
   HeliosTranslatorStatusCode status = direct->host.sync_progress_query(
      context->host_context_cookie, context->generation, out_result);
   if (status < HELIOS_TRANSLATOR_STATUS_OK ||
       status > HELIOS_TRANSLATOR_STATUS_MAX)
      status = HELIOS_TRANSLATOR_STATUS_HOST_CALLBACK_FAILED;
   else if (status == HELIOS_TRANSLATOR_STATUS_OK)
      status = helios_translator_check_query_result(out_result);
   helios_direct_release_context(direct, context);
   return status;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_detach_outer_context(HeliosTranslatorHandle handle,
                                   uint64_t context_generation)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct || !context_generation)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;

   mtx_lock(&direct->lock);
   struct helios_direct_context *context =
      helios_direct_find_context(direct, context_generation);
   if (!context) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   }
   if (context->state == HELIOS_DIRECT_CONTEXT_ATTACHING ||
       context->state == HELIOS_DIRECT_CONTEXT_DETACHING) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   }
   const bool attached =
      context->state == HELIOS_DIRECT_CONTEXT_ATTACHED;
   context->state = HELIOS_DIRECT_CONTEXT_DETACHING;
   while (context->active_host_calls)
      cnd_wait(&direct->context_idle, &direct->lock);
   mtx_unlock(&direct->lock);

   HeliosTranslatorStatusCode checked = HELIOS_TRANSLATOR_STATUS_OK;
   HeliosSyncProgressResultV1 retired_result;
   memset(&retired_result, 0, sizeof(retired_result));
   if (attached) {
      checked = vn_helios_record_context_can_destroy(context->record);
      if (checked == HELIOS_TRANSLATOR_STATUS_OK)
         checked = helios_direct_join(direct, context,
                                      context->last_progress,
                                      &retired_result);
      if (checked == HELIOS_TRANSLATOR_STATUS_OK)
         checked = vn_helios_record_context_destroy(context->record);
   }
   if (checked != HELIOS_TRANSLATOR_STATUS_OK) {
      mtx_lock(&direct->lock);
      context->state = attached ? HELIOS_DIRECT_CONTEXT_ATTACHED
                                : HELIOS_DIRECT_CONTEXT_PENDING;
      mtx_unlock(&direct->lock);
      return checked;
   }

   mtx_lock(&direct->lock);
   context->record = NULL;
   context->queue = NULL;
   context->host_context_cookie = NULL;
   context->retired_result = retired_result;
   context->state = HELIOS_DIRECT_CONTEXT_RETIRED;
   if (direct->context_count)
      direct->context_count--;
   mtx_unlock(&direct->lock);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_open_outer_scope(HeliosTranslatorHandle handle,
                               const HeliosOuterScopeBeginV1 *begin,
                               HeliosTranslatorScope *out_scope)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct || !out_scope)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   *out_scope = NULL;
   HeliosTranslatorStatusCode checked =
      helios_translator_check_scope_begin(begin);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;

   mtx_lock(&direct->lock);
   struct helios_direct_context *context =
      helios_direct_find_context(direct, begin->context_generation);
   if (!context || context->state != HELIOS_DIRECT_CONTEXT_ATTACHED) {
      checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   } else if (context->endpoint_id != begin->endpoint_id) {
      checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ENDPOINT;
   } else if (direct->poisoned || direct->destroying) {
      checked = HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
   } else {
      checked = HELIOS_TRANSLATOR_STATUS_OK;
   }
   struct vn_helios_record_context *record =
      checked == HELIOS_TRANSLATOR_STATUS_OK ? context->record : NULL;
   mtx_unlock(&direct->lock);
   return checked == HELIOS_TRANSLATOR_STATUS_OK
             ? vn_helios_record_scope_open(record, out_scope)
             : checked;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_seal_outer_scope(HeliosTranslatorScope scope,
                               HeliosSealedBatchV1 *out_sealed)
{
   return vn_helios_record_scope_seal(scope, out_sealed);
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_copy_sealed_batch(
   HeliosTranslatorScope scope,
   const HeliosSealedBatchCopyV1 *destination)
{
   return vn_helios_record_scope_copy(scope, destination);
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_close_outer_scope(HeliosTranslatorScope scope,
                                const HeliosOuterScopeCloseV1 *close)
{
   struct vn_instance *instance = NULL;
   uint64_t context_generation = 0;
   if (!vn_helios_record_scope_identity(scope, &instance,
                                        &context_generation))
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   struct HeliosTranslatorInstance_T *direct = instance->helios_direct;
   if (!direct || direct->instance != instance)
      return HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;

   uint32_t disposition = 0;
   HeliosTranslatorStatusCode checked =
      helios_translator_check_scope_close(close, &disposition);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;

   mtx_lock(&direct->lock);
   struct helios_direct_context *context =
      helios_direct_find_context(direct, context_generation);
   if (!context || context->state != HELIOS_DIRECT_CONTEXT_ATTACHED) {
      checked = HELIOS_TRANSLATOR_STATUS_UNKNOWN_CONTEXT;
   } else if (disposition == HELIOS_TRANSLATOR_SCOPE_DISPOSITION_COMMITTED &&
              close->progress_value <= context->last_progress) {
      checked = HELIOS_TRANSLATOR_STATUS_PROGRESS_VALUE;
   } else {
      checked = HELIOS_TRANSLATOR_STATUS_OK;
   }
   mtx_unlock(&direct->lock);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;

   checked = vn_helios_record_scope_close(scope, close);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;
   if (disposition == HELIOS_TRANSLATOR_SCOPE_DISPOSITION_COMMITTED) {
      mtx_lock(&direct->lock);
      context = helios_direct_find_context(direct, context_generation);
      if (context && context->state == HELIOS_DIRECT_CONTEXT_ATTACHED)
         context->last_progress = close->progress_value;
      else
         direct->poisoned = true;
      mtx_unlock(&direct->lock);
   }
   return HELIOS_TRANSLATOR_STATUS_OK;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_query_refusal_counters(
   HeliosTranslatorHandle handle,
   HeliosTranslatorRefusalCountersV1 *out_counters)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct || !out_counters)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   if (out_counters->struct_bytes != sizeof(*out_counters))
      return HELIOS_TRANSLATOR_STATUS_STRUCT_BYTES;
   if (out_counters->abi_version != HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION)
      return HELIOS_TRANSLATOR_STATUS_ABI_VERSION;
   vn_helios_record_query_refusals(direct->instance, out_counters);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

static HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_direct_destroy_instance(HeliosTranslatorHandle handle)
{
   struct HeliosTranslatorInstance_T *direct =
      helios_direct_from_handle(handle);
   if (!direct)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;

   mtx_lock(&direct->lock);
   if (direct->context_count || direct->endpoint_count) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_CONTEXT_STILL_ATTACHED;
   }
   if (direct->destroying) {
      mtx_unlock(&direct->lock);
      return HELIOS_TRANSLATOR_STATUS_SESSION_POISONED;
   }
   direct->destroying = true;
   struct vn_instance *instance = direct->instance;
   struct helios_direct_context *contexts = direct->contexts;
   direct->contexts = NULL;
   instance->helios_direct = NULL;
   direct->instance = NULL;
   mtx_unlock(&direct->lock);

   vn_DestroyInstance(vn_instance_to_handle(instance), NULL);
   while (contexts) {
      struct helios_direct_context *next = contexts->next;
      free(contexts);
      contexts = next;
   }
   if (direct->join_key_live)
      tss_delete(direct->join_key);
   if (direct->context_idle_live)
      cnd_destroy(&direct->context_idle);
   if (direct->lock_live)
      mtx_destroy(&direct->lock);
   free(direct);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

static void
helios_direct_reset_output(HeliosTranslatorInstanceV1 *out_instance,
                           uint32_t struct_bytes,
                           uint32_t abi_version)
{
   memset(out_instance, 0, sizeof(*out_instance));
   out_instance->struct_bytes = struct_bytes;
   out_instance->abi_version = abi_version;
}

HeliosTranslatorStatusCode HELIOS_TRANSLATOR_CALL
helios_icd_create_translator_v1(
   const HeliosTranslatorCreateInfoV1 *create_info,
   HeliosTranslatorInstanceV1 *out_instance)
{
   HeliosTranslatorStatusCode checked =
      helios_translator_check_create_info(create_info,
                                          HELIOS_PACKAGE_GENERATION);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;
   checked = helios_translator_check_host_callbacks(
      create_info->host_callbacks, HELIOS_PACKAGE_GENERATION);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;
   if (!out_instance)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   const uint32_t out_struct_bytes = out_instance->struct_bytes;
   const uint32_t out_abi_version = out_instance->abi_version;
   if (out_struct_bytes != sizeof(*out_instance))
      return HELIOS_TRANSLATOR_STATUS_STRUCT_BYTES;
   if (out_abi_version != HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION)
      return HELIOS_TRANSLATOR_STATUS_ABI_VERSION;
   helios_direct_reset_output(out_instance, out_struct_bytes,
                              out_abi_version);

   VkInstance vk_instance = VK_NULL_HANDLE;
   int32_t create_status = HELIOS_TRANSLATOR_STATUS_SESSION_INIT;
   VkResult result = vn_helios_create_direct_instance(
      create_info->adapter_luid_low, create_info->adapter_luid_high,
      create_info->requested_endpoint_capacity, &vk_instance,
      &create_status);
   if (result != VK_SUCCESS || !vk_instance) {
      if (create_status <= HELIOS_TRANSLATOR_STATUS_OK ||
          create_status > HELIOS_TRANSLATOR_STATUS_MAX)
         create_status = HELIOS_TRANSLATOR_STATUS_SESSION_INIT;
      return create_status;
   }

   struct vn_instance *instance = vn_instance_from_handle(vk_instance);
   struct HeliosTranslatorInstance_T *direct = calloc(1, sizeof(*direct));
   if (!direct) {
      vn_DestroyInstance(vk_instance, NULL);
      return HELIOS_TRANSLATOR_STATUS_SESSION_CAPACITY;
   }
   direct->instance = instance;
   direct->host = *create_info->host_callbacks;
   if (mtx_init(&direct->lock, mtx_plain) != thrd_success) {
      free(direct);
      vn_DestroyInstance(vk_instance, NULL);
      return HELIOS_TRANSLATOR_STATUS_SESSION_CAPACITY;
   }
   direct->lock_live = true;
   if (cnd_init(&direct->context_idle) != thrd_success) {
      mtx_destroy(&direct->lock);
      free(direct);
      vn_DestroyInstance(vk_instance, NULL);
      return HELIOS_TRANSLATOR_STATUS_SESSION_CAPACITY;
   }
   direct->context_idle_live = true;
   if (tss_create(&direct->join_key, NULL) != thrd_success) {
      cnd_destroy(&direct->context_idle);
      mtx_destroy(&direct->lock);
      free(direct);
      vn_DestroyInstance(vk_instance, NULL);
      return HELIOS_TRANSLATOR_STATUS_SESSION_CAPACITY;
   }
   direct->join_key_live = true;

   HMODULE module =
      helios_module_from_address((const void *)helios_icd_create_translator_v1);
   if (!module) {
      tss_delete(direct->join_key);
      cnd_destroy(&direct->context_idle);
      mtx_destroy(&direct->lock);
      free(direct);
      vn_DestroyInstance(vk_instance, NULL);
      return HELIOS_TRANSLATOR_STATUS_LOADER_PROVENANCE;
   }
   direct->dispatch = (HeliosTranslatorDispatchV1){
      .struct_bytes = sizeof(direct->dispatch),
      .abi_version = HELIOS_TRANSLATOR_DISPATCH_ABI_VERSION,
      .package_generation = HELIOS_PACKAGE_GENERATION,
      .icd_module_base = (const void *)module,
      .get_instance_proc_addr = helios_direct_get_instance_proc_addr,
      .enumerate_endpoints = helios_direct_enumerate_endpoints,
      .build_queue_attach = helios_direct_build_queue_attach,
      .attach_outer_context = helios_direct_attach_outer_context,
      .detach_outer_context = helios_direct_detach_outer_context,
      .open_outer_scope = helios_direct_open_outer_scope,
      .seal_outer_scope = helios_direct_seal_outer_scope,
      .copy_sealed_batch = helios_direct_copy_sealed_batch,
      .close_outer_scope = helios_direct_close_outer_scope,
      .query_refusal_counters = helios_direct_query_refusal_counters,
      .destroy_instance = helios_direct_destroy_instance,
   };
   instance->helios_direct = direct;

   out_instance->handle = direct;
   out_instance->dispatch = &direct->dispatch;
   out_instance->vk_instance = vk_instance;
   out_instance->session_generation =
      vn_renderer_helios_session_generation(instance->renderer);
   out_instance->endpoint_capacity =
      vn_renderer_helios_endpoint_capacity(instance->renderer);
   out_instance->submission_mode = HELIOS_TRANSLATOR_SUBMISSION_MODE_RECORD_ONLY;

   checked = helios_translator_check_instance(out_instance);
   if (checked == HELIOS_TRANSLATOR_STATUS_OK)
      checked = helios_translator_check_dispatch(
         out_instance->dispatch, HELIOS_PACKAGE_GENERATION);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK) {
      (void)helios_direct_destroy_instance(direct);
      helios_direct_reset_output(out_instance, out_struct_bytes,
                                 out_abi_version);
   }
   return checked;
}

#endif /* _WIN32 */
