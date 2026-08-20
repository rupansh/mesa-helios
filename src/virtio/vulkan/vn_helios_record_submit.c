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
#include "vn_command_buffer.h"
#include "vn_cs.h"
#include "vn_descriptor_set.h"
#include "vn_device.h"
#include "vn_device_memory.h"
#include "vn_helios_direct_dispatch.h"
#include "vn_helios_native_kmt.h"
#include "vn_image.h"
#include "vn_instance.h"
#include "vn_physical_device.h"
#include "vn_query_pool.h"
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

struct helios_scope_signal {
   struct vn_semaphore *semaphore;
   uint64_t value;
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
   struct vn_helios_deferred_record **deferred_records;
   uint32_t deferred_record_count;
   uint32_t deferred_record_capacity;
   struct vn_query_pool **query_pools;
   uint32_t query_pool_count;
   uint32_t query_pool_capacity;
   struct vn_event **events;
   uint32_t event_count;
   uint32_t event_capacity;
   struct vn_fence *fence;
   struct helios_scope_signal *signals;
   uint32_t signal_count;
   uint32_t signal_capacity;
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

struct helios_command_streams {
   struct vn_command_buffer **commands;
   uint32_t count;
   uint32_t capacity;
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

/* ── A7 command-buffer allocation closure ──────────────────────────────── */

static struct vn_device *
helios_cmd_device(const struct vn_command_buffer *cmd)
{
   return cmd && cmd->base.vk.pool
             ? vn_device_from_vk(cmd->base.vk.pool->base.device)
             : NULL;
}

void
vn_helios_cmd_closure_fini(struct vn_command_buffer *cmd)
{
   if (!cmd)
      return;
   free(cmd->builder.helios_uses);
   free(cmd->builder.helios_secondaries);
   free(cmd->builder.helios_events);
   cmd->builder.helios_uses = NULL;
   cmd->builder.helios_use_count = 0;
   cmd->builder.helios_use_capacity = 0;
   cmd->builder.helios_secondaries = NULL;
   cmd->builder.helios_secondary_count = 0;
   cmd->builder.helios_secondary_capacity = 0;
   cmd->builder.helios_events = NULL;
   cmd->builder.helios_event_count = 0;
   cmd->builder.helios_event_capacity = 0;
   cmd->builder.helios_refusal_opcode = 0;
   cmd->builder.helios_closure_complete = false;
}

void
vn_helios_cmd_closure_begin(struct vn_command_buffer *cmd)
{
   if (!cmd)
      return;
   /* Begin follows vn_cmd_reset, but keeping this idempotent makes an error
    * path incapable of retaining a previous recording's token closure. */
   free(cmd->builder.helios_uses);
   free(cmd->builder.helios_secondaries);
   free(cmd->builder.helios_events);
   cmd->builder.helios_uses = NULL;
   cmd->builder.helios_use_count = 0;
   cmd->builder.helios_use_capacity = 0;
   cmd->builder.helios_secondaries = NULL;
   cmd->builder.helios_secondary_count = 0;
   cmd->builder.helios_secondary_capacity = 0;
   cmd->builder.helios_events = NULL;
   cmd->builder.helios_event_count = 0;
   cmd->builder.helios_event_capacity = 0;
   cmd->builder.helios_refusal_opcode = 0;
   cmd->builder.helios_closure_complete = true;
}

void
vn_helios_cmd_refuse(struct vn_command_buffer *cmd, uint32_t opcode)
{
   if (!cmd)
      return;
   if (cmd->builder.helios_closure_complete)
      cmd->builder.helios_refusal_opcode = opcode ? opcode : UINT32_MAX;
   cmd->builder.helios_closure_complete = false;
}

static bool
helios_cmd_add_binding(struct vn_command_buffer *cmd,
                       const struct vn_helios_memory_binding *binding,
                       uint32_t access_flags,
                       uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   if (!dev || !cmd->builder.helios_closure_complete || !binding ||
       !binding->valid || !access_flags ||
       (access_flags & ~HELIOS_HOB1_ACCESS_MASK) ||
       ((access_flags & HELIOS_HOB1_ACCESS_PRIMARY_WRITE) &&
        !(access_flags & HELIOS_HOB1_ACCESS_WRITE)) ||
       !vn_device_memory_helios_binding_live(dev, binding)) {
      vn_helios_cmd_refuse(cmd, opcode);
      return false;
   }

   const uint64_t binding_end =
      binding->byte_offset + binding->byte_length;
   if (binding_end < binding->byte_offset ||
       binding_end > binding->outer_allocation_bytes) {
      vn_helios_cmd_refuse(cmd, opcode);
      return false;
   }

   for (uint32_t i = 0; i < cmd->builder.helios_use_count; i++) {
      struct vn_helios_command_use *use = &cmd->builder.helios_uses[i];
      if (use->binding.outer_allocation_token !=
          binding->outer_allocation_token)
         continue;
      if (use->binding.device_generation != binding->device_generation ||
          use->binding.outer_allocation_bytes !=
             binding->outer_allocation_bytes) {
         vn_helios_cmd_refuse(cmd, opcode);
         return false;
      }
      const uint64_t old_end =
         use->binding.byte_offset + use->binding.byte_length;
      const uint64_t start = MIN2(use->binding.byte_offset,
                                  binding->byte_offset);
      const uint64_t end = MAX2(old_end, binding_end);
      if (old_end < use->binding.byte_offset || end < start ||
          end > binding->outer_allocation_bytes) {
         vn_helios_cmd_refuse(cmd, opcode);
         return false;
      }
      use->binding.byte_offset = start;
      use->binding.byte_length = end - start;
      use->access_flags |= access_flags;
      return true;
   }

   if (cmd->builder.helios_use_count == HELIOS_HOB1_MAX_USE_RECORDS) {
      vn_helios_cmd_refuse(cmd, opcode);
      return false;
   }
   if (cmd->builder.helios_use_count ==
       cmd->builder.helios_use_capacity) {
      uint32_t capacity = cmd->builder.helios_use_capacity
                             ? cmd->builder.helios_use_capacity * 2u
                             : 16u;
      if (capacity < cmd->builder.helios_use_capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity, sizeof(*cmd->builder.helios_uses),
                           &bytes)) {
         vn_helios_cmd_refuse(cmd, opcode);
         return false;
      }
      void *new_uses = realloc(cmd->builder.helios_uses, bytes);
      if (!new_uses) {
         vn_helios_cmd_refuse(cmd, opcode);
         return false;
      }
      cmd->builder.helios_uses = new_uses;
      cmd->builder.helios_use_capacity = capacity;
   }
   cmd->builder.helios_uses[cmd->builder.helios_use_count++] =
      (struct vn_helios_command_use){
         .binding = *binding,
         .access_flags = access_flags,
      };
   return true;
}

void
vn_helios_cmd_touch_buffer(struct vn_command_buffer *cmd,
                           VkBuffer handle,
                           VkDeviceSize offset,
                           VkDeviceSize size,
                           uint32_t access_flags,
                           uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   struct vn_buffer *buf = vn_buffer_from_handle(handle);
   if (!dev || !buf || buf->base.vk.device != &dev->base.vk ||
       !buf->helios_binding.valid || offset > buf->helios_binding.byte_length) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   const uint64_t available = buf->helios_binding.byte_length - offset;
   const uint64_t length = size == VK_WHOLE_SIZE ? available : size;
   if (!length || length > available) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   /* HOB1 identity is allocation-level.  Validate the command's subrange but
    * retain the complete exact VkBuffer binding so D3D11 never fabricates a
    * byte-offset identity and D3D12 still names a bounded heap interval. */
   (void)helios_cmd_add_binding(cmd, &buf->helios_binding, access_flags,
                                opcode);
}

void
vn_helios_cmd_touch_image(struct vn_command_buffer *cmd,
                          VkImage handle,
                          uint32_t access_flags,
                          uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   struct vn_image *img = vn_image_from_handle(handle);
   if (!dev || !img || img->base.vk.base.device != &dev->base.vk) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   if (img->helios_presentable.tagged &&
       (access_flags & HELIOS_HOB1_ACCESS_WRITE))
      access_flags |= HELIOS_HOB1_ACCESS_PRIMARY_WRITE;
   bool found = false;
   for (uint32_t plane = 0; plane < ARRAY_SIZE(img->helios_bindings);
        plane++) {
      if (!img->helios_bindings[plane].valid)
         continue;
      found = true;
      if (!helios_cmd_add_binding(cmd, &img->helios_bindings[plane],
                                  access_flags, opcode))
         return;
   }
   if (!found)
      vn_helios_cmd_refuse(cmd, opcode);
}

void
vn_helios_cmd_touch_image_view(struct vn_command_buffer *cmd,
                               VkImageView handle,
                               uint32_t access_flags,
                               uint32_t opcode)
{
   struct vn_image_view *view = vn_image_view_from_handle(handle);
   struct vn_device *dev = helios_cmd_device(cmd);
   if (!view || !dev || view->base.vk.device != &dev->base.vk ||
       !view->image) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   vn_helios_cmd_touch_image(cmd, vn_image_to_handle((struct vn_image *)view->image),
                             access_flags, opcode);
}

void
vn_helios_cmd_touch_device_address(struct vn_command_buffer *cmd,
                                   VkDeviceAddress address,
                                   VkDeviceSize size,
                                   uint32_t access_flags,
                                   uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   if (!dev || !address || !size || size == VK_WHOLE_SIZE ||
       address > UINT64_MAX - size) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   struct vn_buffer *match = NULL;
   VkDeviceSize match_offset = 0;
   const uint64_t end = address + size;
   simple_mtx_lock(&dev->mutex);
   list_for_each_entry(struct vn_buffer, buf, &dev->helios_address_buffers,
                       helios_address_link) {
      if (!buf->helios_device_address ||
          buf->helios_device_address > UINT64_MAX -
             buf->helios_binding.byte_length)
         continue;
      const uint64_t buf_end =
         buf->helios_device_address + buf->helios_binding.byte_length;
      if (address >= buf->helios_device_address && end <= buf_end) {
         if (match) {
            match = NULL;
            break;
         }
         match = buf;
         match_offset = address - buf->helios_device_address;
      }
   }
   simple_mtx_unlock(&dev->mutex);
   if (!match) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   vn_helios_cmd_touch_buffer(cmd, vn_buffer_to_handle(match), match_offset,
                              size, access_flags, opcode);
}

static bool
helios_cmd_add_event(struct vn_command_buffer *cmd,
                     struct vn_event *event,
                     uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   if (!dev || !event || !cmd->builder.helios_closure_complete ||
       !helios_object_owned(&event->base.vk, dev)) {
      vn_helios_cmd_refuse(cmd, opcode);
      return false;
   }
   for (uint32_t i = 0; i < cmd->builder.helios_event_count; i++) {
      if (cmd->builder.helios_events[i] == event)
         return true;
   }
   if (cmd->builder.helios_event_count == HELIOS_HOB1_MAX_USE_RECORDS) {
      vn_helios_cmd_refuse(cmd, opcode);
      return false;
   }
   if (cmd->builder.helios_event_count ==
       cmd->builder.helios_event_capacity) {
      uint32_t capacity = cmd->builder.helios_event_capacity
                             ? cmd->builder.helios_event_capacity * 2u
                             : 8u;
      if (capacity < cmd->builder.helios_event_capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity,
                           sizeof(*cmd->builder.helios_events), &bytes)) {
         vn_helios_cmd_refuse(cmd, opcode);
         return false;
      }
      void *events = realloc(cmd->builder.helios_events, bytes);
      if (!events) {
         vn_helios_cmd_refuse(cmd, opcode);
         return false;
      }
      cmd->builder.helios_events = events;
      cmd->builder.helios_event_capacity = capacity;
   }
   cmd->builder.helios_events[cmd->builder.helios_event_count++] = event;
   return true;
}

void
vn_helios_cmd_touch_event(struct vn_command_buffer *cmd,
                          VkEvent handle,
                          uint32_t opcode)
{
   struct vn_event *event = vn_event_from_handle(handle);
   (void)helios_cmd_add_event(cmd, event, opcode);
}

void
vn_helios_cmd_merge_secondary(struct vn_command_buffer *primary,
                              const struct vn_command_buffer *secondary,
                              uint32_t opcode)
{
   if (!primary || !secondary || primary == secondary ||
       helios_cmd_device(primary) != helios_cmd_device(secondary) ||
       secondary->base.vk.state != MESA_VK_COMMAND_BUFFER_STATE_EXECUTABLE ||
       !secondary->builder.helios_closure_complete) {
      vn_helios_cmd_refuse(primary,
                           secondary && secondary->builder.helios_refusal_opcode
                              ? secondary->builder.helios_refusal_opcode
                              : opcode);
      return;
   }
   for (uint32_t i = 0; i < secondary->builder.helios_use_count; i++) {
      const struct vn_helios_command_use *use =
         &secondary->builder.helios_uses[i];
      if (!helios_cmd_add_binding(primary, &use->binding, use->access_flags,
                                  opcode))
         return;
   }
   for (uint32_t i = 0; i < secondary->builder.helios_event_count; i++) {
      if (!helios_cmd_add_event(primary,
                                secondary->builder.helios_events[i],
                                opcode))
         return;
   }

   for (uint32_t i = 0; i < primary->builder.helios_secondary_count; i++) {
      if (primary->builder.helios_secondaries[i] == secondary)
         return;
   }
   if (primary->builder.helios_secondary_count ==
       primary->builder.helios_secondary_capacity) {
      uint32_t capacity = primary->builder.helios_secondary_capacity
                             ? primary->builder.helios_secondary_capacity * 2u
                             : 8u;
      if (capacity < primary->builder.helios_secondary_capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (primary->builder.helios_secondary_count ==
             HELIOS_HOB1_MAX_USE_RECORDS ||
          !helios_size_mul(capacity,
                           sizeof(*primary->builder.helios_secondaries),
                           &bytes)) {
         vn_helios_cmd_refuse(primary, opcode);
         return;
      }
      void *new_secondaries =
         realloc(primary->builder.helios_secondaries, bytes);
      if (!new_secondaries) {
         vn_helios_cmd_refuse(primary, opcode);
         return;
      }
      primary->builder.helios_secondaries = new_secondaries;
      primary->builder.helios_secondary_capacity = capacity;
   }
   primary->builder.helios_secondaries[
      primary->builder.helios_secondary_count++] =
      (struct vn_command_buffer *)secondary;
}

void
vn_helios_cmd_touch_descriptor_set(struct vn_command_buffer *cmd,
                                   VkDescriptorSet handle,
                                   uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   struct vn_descriptor_set *set =
      vn_descriptor_set_from_handle(handle);
   if (!dev || !set || set->base.vk.device != &dev->base.vk ||
       !set->helios_closure_complete) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   for (uint32_t i = 0; i < set->helios_slot_count; i++) {
      const struct vn_helios_descriptor_slot *slot =
         &set->helios_slots[i];
      if (!slot->binding_count)
         continue;
      if (!slot->access_flags ||
          slot->binding_count > ARRAY_SIZE(slot->bindings)) {
         vn_helios_cmd_refuse(cmd, opcode);
         return;
      }
      for (uint32_t j = 0; j < slot->binding_count; j++) {
         if (!helios_cmd_add_binding(cmd, &slot->bindings[j],
                                     slot->access_flags, opcode))
            return;
      }
   }
}

void
vn_helios_cmd_touch_descriptor_writes(
   struct vn_command_buffer *cmd,
   uint32_t write_count,
   const VkWriteDescriptorSet *writes,
   uint32_t opcode)
{
   struct vn_device *dev = helios_cmd_device(cmd);
   if (!dev || (write_count && !writes)) {
      vn_helios_cmd_refuse(cmd, opcode);
      return;
   }
   /* Push descriptors have no persistent set object.  Build one bounded
    * temporary slot per element with the same classification rules as an
    * ordinary descriptor set, then merge it immediately into the command. */
   for (uint32_t w = 0; w < write_count; w++) {
      const VkWriteDescriptorSet *write = &writes[w];
      for (uint32_t i = 0; i < write->descriptorCount; i++) {
         switch (write->descriptorType) {
         case VK_DESCRIPTOR_TYPE_SAMPLER:
         case VK_DESCRIPTOR_TYPE_INLINE_UNIFORM_BLOCK:
            break;
         case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
         case VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE:
         case VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT:
         case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE: {
            if (!write->pImageInfo) {
               vn_helios_cmd_refuse(cmd, opcode);
               return;
            }
            const VkImageView view = write->pImageInfo[i].imageView;
            if (view)
               vn_helios_cmd_touch_image_view(
                  cmd, view,
                  write->descriptorType == VK_DESCRIPTOR_TYPE_STORAGE_IMAGE
                     ? HELIOS_HOB1_ACCESS_READ | HELIOS_HOB1_ACCESS_WRITE
                     : HELIOS_HOB1_ACCESS_READ,
                  opcode);
            break;
         }
         case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
         case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC:
         case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
         case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC: {
            if (!write->pBufferInfo) {
               vn_helios_cmd_refuse(cmd, opcode);
               return;
            }
            const VkDescriptorBufferInfo *info = &write->pBufferInfo[i];
            if (info->buffer)
               vn_helios_cmd_touch_buffer(
                  cmd, info->buffer, info->offset, info->range,
                  write->descriptorType == VK_DESCRIPTOR_TYPE_STORAGE_BUFFER ||
                        write->descriptorType ==
                           VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC
                     ? HELIOS_HOB1_ACCESS_READ | HELIOS_HOB1_ACCESS_WRITE
                     : HELIOS_HOB1_ACCESS_READ,
                  opcode);
            break;
         }
         case VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER:
         case VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER: {
            if (!write->pTexelBufferView) {
               vn_helios_cmd_refuse(cmd, opcode);
               return;
            }
            struct vn_buffer_view *view =
               vn_buffer_view_from_handle(write->pTexelBufferView[i]);
            if (view && view->helios_buffer)
               vn_helios_cmd_touch_buffer(
                  cmd, vn_buffer_to_handle(view->helios_buffer), 0,
                  VK_WHOLE_SIZE,
                  write->descriptorType ==
                        VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER
                     ? HELIOS_HOB1_ACCESS_READ | HELIOS_HOB1_ACCESS_WRITE
                     : HELIOS_HOB1_ACCESS_READ,
                  opcode);
            else if (write->pTexelBufferView[i]) {
               vn_helios_cmd_refuse(cmd, opcode);
               return;
            }
            break;
         }
         default:
            vn_helios_cmd_refuse(cmd, opcode);
            return;
         }
         if (!cmd->builder.helios_closure_complete)
            return;
      }
   }
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

   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      if (queue_family >= dev->physical_device->queue_family_count) {
         vn_helios_submit_queue_fini(queue);
         return VK_ERROR_INITIALIZATION_FAILED;
      }
      const VkQueueFlags queue_flags =
         dev->physical_device->queue_family_properties[queue_family]
            .queueFamilyProperties.queueFlags;
      VkResult result = vn_helios_direct_register_queue(
         dev->instance, queue, queue_family, queue_index, queue_flags);
      if (result != VK_SUCCESS)
         vn_helios_submit_queue_fini(queue);
      return result;
   }

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

   if (dev)
      vn_helios_direct_unregister_queue(dev->instance, queue);

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
vn_helios_record_context_can_destroy(
   struct vn_helios_record_context *context)
{
   if (!context)
      return HELIOS_TRANSLATOR_STATUS_NULL_ARGUMENT;
   mtx_lock(&context->lock);
   const bool live = context->active_scope != NULL;
   mtx_unlock(&context->lock);
   return live ? HELIOS_TRANSLATOR_STATUS_SCOPE_STILL_LIVE
               : HELIOS_TRANSLATOR_STATUS_OK;
}

HeliosTranslatorStatusCode
vn_helios_record_context_destroy(struct vn_helios_record_context *context)
{
   const HeliosTranslatorStatusCode checked =
      vn_helios_record_context_can_destroy(context);
   if (checked != HELIOS_TRANSLATOR_STATUS_OK)
      return checked;
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

static bool
helios_scope_reserve_uses(struct HeliosTranslatorScope_T *scope,
                          uint32_t count)
{
   if (count > HELIOS_HOB1_MAX_USE_RECORDS)
      return false;
   if (count <= scope->use_capacity)
      return true;
   size_t bytes;
   if (!helios_size_mul(count, sizeof(*scope->uses), &bytes))
      return false;
   void *uses = realloc(scope->uses, bytes);
   if (!uses)
      return false;
   scope->uses = uses;
   scope->use_capacity = count;
   return true;
}

static bool
helios_scope_reserve_operands(struct HeliosTranslatorScope_T *scope,
                              uint32_t count)
{
   if (count > HELIOS_HOB1_MAX_OPERAND_RECORDS)
      return false;
   if (count <= scope->operand_capacity)
      return true;
   size_t bytes;
   if (!helios_size_mul(count, sizeof(*scope->operands), &bytes))
      return false;
   void *operands = realloc(scope->operands, bytes);
   if (!operands)
      return false;
   scope->operands = operands;
   scope->operand_capacity = count;
   return true;
}

static bool
helios_scope_reserve_deferred(struct HeliosTranslatorScope_T *scope,
                              uint32_t count)
{
   if (count > HELIOS_HOB1_MAX_USE_RECORDS)
      return false;
   if (count <= scope->deferred_record_capacity)
      return true;
   size_t bytes;
   if (!helios_size_mul(count, sizeof(*scope->deferred_records), &bytes))
      return false;
   void *records = realloc(scope->deferred_records, bytes);
   if (!records)
      return false;
   scope->deferred_records = records;
   scope->deferred_record_capacity = count;
   return true;
}

static bool
helios_scope_add_query_pool(struct HeliosTranslatorScope_T *scope,
                            struct vn_device *dev,
                            struct vn_query_pool *pool)
{
   if (!pool || !helios_object_owned(&pool->base.vk, dev))
      return false;
   for (uint32_t i = 0; i < scope->query_pool_count; i++) {
      if (scope->query_pools[i] == pool)
         return true;
   }
   if (scope->query_pool_count == HELIOS_HOB1_MAX_USE_RECORDS)
      return false;
   if (scope->query_pool_count == scope->query_pool_capacity) {
      uint32_t capacity = scope->query_pool_capacity
                             ? scope->query_pool_capacity * 2u
                             : 8u;
      if (capacity < scope->query_pool_capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity, sizeof(*scope->query_pools), &bytes))
         return false;
      void *pools = realloc(scope->query_pools, bytes);
      if (!pools)
         return false;
      scope->query_pools = pools;
      scope->query_pool_capacity = capacity;
   }
   scope->query_pools[scope->query_pool_count++] = pool;
   return true;
}

static bool
helios_scope_add_event(struct HeliosTranslatorScope_T *scope,
                       struct vn_device *dev,
                       struct vn_event *event)
{
   if (!event || !helios_object_owned(&event->base.vk, dev))
      return false;
   for (uint32_t i = 0; i < scope->event_count; i++) {
      if (scope->events[i] == event)
         return true;
   }
   if (scope->event_count == HELIOS_HOB1_MAX_USE_RECORDS)
      return false;
   if (scope->event_count == scope->event_capacity) {
      uint32_t capacity = scope->event_capacity
                             ? scope->event_capacity * 2u
                             : 8u;
      if (capacity < scope->event_capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity, sizeof(*scope->events), &bytes))
         return false;
      void *events = realloc(scope->events, bytes);
      if (!events)
         return false;
      scope->events = events;
      scope->event_capacity = capacity;
   }
   scope->events[scope->event_count++] = event;
   return true;
}

static bool
helios_scope_add_signal(struct HeliosTranslatorScope_T *scope,
                        struct vn_device *dev,
                        struct vn_semaphore *sem,
                        uint64_t value)
{
   if (!sem || !helios_object_owned(&sem->base.vk, dev) ||
       (sem->type == VK_SEMAPHORE_TYPE_TIMELINE && !value))
      return false;
   for (uint32_t i = 0; i < scope->signal_count; i++) {
      if (scope->signals[i].semaphore != sem)
         continue;
      if (sem->type == VK_SEMAPHORE_TYPE_BINARY ||
          value > scope->signals[i].value)
         scope->signals[i].value = value;
      return true;
   }
   if (scope->signal_count == HELIOS_HOB1_MAX_USE_RECORDS)
      return false;
   if (scope->signal_count == scope->signal_capacity) {
      uint32_t capacity = scope->signal_capacity
                             ? scope->signal_capacity * 2u
                             : 8u;
      if (capacity < scope->signal_capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity, sizeof(*scope->signals), &bytes))
         return false;
      void *signals = realloc(scope->signals, bytes);
      if (!signals)
         return false;
      scope->signals = signals;
      scope->signal_capacity = capacity;
   }
   scope->signals[scope->signal_count++] = (struct helios_scope_signal){
      .semaphore = sem,
      .value = value,
   };
   return true;
}

static struct vn_device_memory *
helios_find_outer_memory_locked(
   struct vn_device *dev,
   const struct vn_helios_memory_binding *binding)
{
   list_for_each_entry(struct vn_device_memory, mem,
                       &dev->helios_outer_allocations,
                       helios_outer_link) {
      if (mem->helios_outer_registered &&
          mem->helios_outer.device_generation == binding->device_generation &&
          mem->helios_outer.outer_allocation_token ==
             binding->outer_allocation_token &&
          mem->helios_outer.outer_allocation_bytes ==
             binding->outer_allocation_bytes)
         return mem;
   }
   return NULL;
}

static VkResult
helios_record_append(struct vn_queue *queue,
                     enum helios_record_refusal refusal,
                     const void *payload,
                     uint64_t payload_bytes,
                     const struct vn_helios_command_use *command_uses,
                     uint32_t command_use_count,
                     const struct helios_command_streams *streams,
                     bool deferred_only)
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
   /* One append still owns one logical queue operation, but its immutable A7
    * stream is deliberately multi-command: deferred allocation/bind records,
    * complete Begin..End command-buffer recordings, then one final queue op. */
   if (scope->payload_bytes) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   if ((!deferred_only && (!payload || !payload_bytes)) ||
       (deferred_only && (payload || payload_bytes || streams ||
                          command_use_count != 1))) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   if (command_use_count > HELIOS_HOB1_MAX_USE_RECORDS ||
       (command_use_count && !command_uses)) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   if (streams && streams->count && !streams->commands) {
      scope->failure = HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }

   uint64_t command_bytes = 0;
   if (streams) {
      for (uint32_t i = 0; i < streams->count; i++) {
         const struct vn_command_buffer *cmd = streams->commands[i];
         if (!cmd || helios_cmd_device(cmd) != dev ||
             cmd->base.vk.state != MESA_VK_COMMAND_BUFFER_STATE_EXECUTABLE ||
             !cmd->builder.helios_closure_complete ||
             vn_cs_encoder_get_fatal(&cmd->cs)) {
            scope->failure = HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
            helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
            return VK_ERROR_VALIDATION_FAILED_EXT;
         }
         uint64_t exact = 0;
         for (uint32_t b = 0; b < cmd->cs.buffer_count; b++) {
            const struct vn_cs_encoder_buffer *buffer = &cmd->cs.buffers[b];
            if (!buffer->base || !buffer->committed_size ||
                exact > UINT64_MAX - buffer->committed_size) {
               scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
               helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
               return VK_ERROR_OUT_OF_HOST_MEMORY;
            }
            exact += buffer->committed_size;
         }
         if (!exact || exact != vn_cs_encoder_get_len(&cmd->cs) ||
             command_bytes > UINT64_MAX - exact) {
            scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
            helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
            return VK_ERROR_OUT_OF_HOST_MEMORY;
         }
         command_bytes += exact;

         list_for_each_entry(struct vn_cmd_query_record, query,
                             &cmd->builder.query_records, head) {
            if (!helios_scope_add_query_pool(scope, dev,
                                             query->query_pool)) {
               scope->failure =
                  HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
               helios_record_refuse(owner,
                                    HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
               return VK_ERROR_VALIDATION_FAILED_EXT;
            }
         }
         for (uint32_t e = 0; e < cmd->builder.helios_event_count; e++) {
            if (!helios_scope_add_event(scope, dev,
                                        cmd->builder.helios_events[e])) {
               scope->failure =
                  HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
               helios_record_refuse(owner,
                                    HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
               return VK_ERROR_VALIDATION_FAILED_EXT;
            }
         }
      }
   }

   if (!helios_scope_reserve_uses(scope, command_use_count)) {
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   for (uint32_t i = 0; i < command_use_count; i++) {
      const struct vn_helios_command_use *command_use = &command_uses[i];
      if (!vn_device_memory_helios_binding_live(dev,
                                                &command_use->binding)) {
         scope->failure = HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      for (uint32_t j = 0; j < i; j++) {
         if (command_uses[j].binding.outer_allocation_token ==
             command_use->binding.outer_allocation_token) {
            scope->failure = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ALLOCATION_TOKEN;
            helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
            return VK_ERROR_VALIDATION_FAILED_EXT;
         }
      }
      HeliosSealedResourceUseV1 sealed_use = {
         .outer_allocation_token =
            command_use->binding.outer_allocation_token,
         .byte_length = command_use->binding.byte_length,
         .byte_offset = command_use->binding.byte_offset,
         .access_flags = command_use->access_flags,
         .reserved0 = 0,
         .operand_count = 0,
         .first_operand = 0,
         .reserved1 = 0,
      };
      const HeliosTranslatorStatusCode use_status =
         helios_translator_check_sealed_use(&sealed_use,
                                            scope->context->context_flags);
      if (use_status != HELIOS_TRANSLATOR_STATUS_OK) {
         scope->failure = use_status;
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      scope->uses[i] = sealed_use;
   }
   scope->use_count = command_use_count;

   uint32_t deferred_count = 0;
   uint32_t final_free_count = 0;
   uint32_t operand_count = 0;
   uint64_t deferred_bytes = 0;
   simple_mtx_lock(&dev->mutex);
   for (uint32_t i = 0; i < command_use_count; i++) {
      const struct vn_device_memory *mem =
         helios_find_outer_memory_locked(dev, &command_uses[i].binding);
      if (!mem) {
         simple_mtx_unlock(&dev->mutex);
         scope->failure = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ALLOCATION_TOKEN;
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      list_for_each_entry(struct vn_helios_deferred_record, record,
                          &mem->helios_deferred_records, link) {
         if (record->owner != mem || record->reserved_context_generation ||
             record->reserved_batch_id ||
             deferred_count == HELIOS_HOB1_MAX_USE_RECORDS ||
             deferred_bytes > HELIOS_HOB1_MAX_BYTES - record->payload_bytes) {
            simple_mtx_unlock(&dev->mutex);
            scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
            helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
            return VK_ERROR_OUT_OF_HOST_MEMORY;
         }
         deferred_count++;
         deferred_bytes += record->payload_bytes;
         final_free_count += record->final_free_record;
         if ((!deferred_only && record->final_free_record) ||
             (record->final_free_record && !mem->helios_free_pending)) {
            simple_mtx_unlock(&dev->mutex);
            scope->failure = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ALLOCATION_TOKEN;
            helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
            return VK_ERROR_VALIDATION_FAILED_EXT;
         }
         if (record->resource_operand_offset !=
             VN_HELIOS_NO_RESOURCE_OPERAND) {
            uint32_t zero = UINT32_MAX;
            memcpy(&zero,
                   record->payload + record->resource_operand_offset,
                   sizeof(zero));
            if (zero != 0) {
               simple_mtx_unlock(&dev->mutex);
               scope->failure =
                  HELIOS_TRANSLATOR_STATUS_PAYLOAD_PLACEHOLDER_NON_ZERO;
               helios_record_refuse(owner,
                                    HELIOS_RECORD_REFUSE_DEFERRED_USE);
               return VK_ERROR_VALIDATION_FAILED_EXT;
            }
            if (operand_count == HELIOS_HOB1_MAX_OPERAND_RECORDS) {
               simple_mtx_unlock(&dev->mutex);
               scope->failure =
                  HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
               helios_record_refuse(owner,
                                    HELIOS_RECORD_REFUSE_BATCH_BOUND);
               return VK_ERROR_OUT_OF_HOST_MEMORY;
            }
            operand_count++;
         }
      }
   }
   if (deferred_only && (deferred_count == 0 || final_free_count != 1)) {
      simple_mtx_unlock(&dev->mutex);
      scope->failure = HELIOS_TRANSLATOR_STATUS_UNKNOWN_ALLOCATION_TOKEN;
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   if (command_bytes > HELIOS_HOB1_MAX_BYTES - deferred_bytes ||
       payload_bytes >
          HELIOS_HOB1_MAX_BYTES - deferred_bytes - command_bytes) {
      simple_mtx_unlock(&dev->mutex);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   const uint64_t total_payload =
      deferred_bytes + command_bytes + payload_bytes;
   const uint64_t assembled = HELIOS_HOB1_HEADER_BYTES + total_payload +
      (uint64_t)command_use_count * HELIOS_HOB1_USE_RECORD_BYTES +
      (uint64_t)operand_count * HELIOS_HOB1_OPERAND_RECORD_BYTES;
   if (assembled > HELIOS_HOB1_MAX_BYTES ||
       !helios_scope_reserve_deferred(scope, deferred_count) ||
       !helios_scope_reserve_operands(scope, operand_count) ||
       !helios_scope_reserve_payload(scope, total_payload)) {
      simple_mtx_unlock(&dev->mutex);
      scope->failure = HELIOS_TRANSLATOR_STATUS_BATCH_BOUND_EXCEEDED;
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_BATCH_BOUND);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   uint64_t write_offset = 0;
   scope->deferred_record_count = 0;
   scope->operand_count = 0;
   for (uint32_t i = 0; i < command_use_count; i++) {
      struct vn_device_memory *mem =
         helios_find_outer_memory_locked(dev, &command_uses[i].binding);
      assert(mem);
      scope->uses[i].first_operand = scope->operand_count;
      list_for_each_entry(struct vn_helios_deferred_record, record,
                          &mem->helios_deferred_records, link) {
         record->reserved_context_generation =
            scope->context->context_generation;
         record->reserved_batch_id = scope->batch_id;
         scope->deferred_records[scope->deferred_record_count++] = record;
         memcpy(scope->payload + write_offset, record->payload,
                record->payload_bytes);
         if (record->resource_operand_offset !=
             VN_HELIOS_NO_RESOURCE_OPERAND) {
            uint32_t zero = UINT32_MAX;
            memcpy(&zero,
                   record->payload + record->resource_operand_offset,
                   sizeof(zero));
            assert(zero == 0);
            scope->operands[scope->operand_count++] =
               (HeliosSealedOperandV1){
                  .payload_relative_offset =
                     (uint32_t)(write_offset +
                                record->resource_operand_offset),
                  .use_index = i,
                  .operand_kind =
                     HELIOS_HOB1_OPERAND_KIND_GENERATED_RESOURCE,
                  .encoded_width = HELIOS_HOB1_OPERAND_WIDTH_4,
                  .reserved = 0,
               };
            scope->uses[i].operand_count++;
         }
         write_offset += record->payload_bytes;
      }
   }
   simple_mtx_unlock(&dev->mutex);

   mtx_lock(&queue->helios_record_mutex);
   if (streams) {
      for (uint32_t i = 0; i < streams->count; i++) {
         const struct vn_command_buffer *cmd = streams->commands[i];
         for (uint32_t b = 0; b < cmd->cs.buffer_count; b++) {
            const struct vn_cs_encoder_buffer *buffer = &cmd->cs.buffers[b];
            memcpy(scope->payload + write_offset, buffer->base,
                   buffer->committed_size);
            write_offset += buffer->committed_size;
         }
      }
   }
   if (payload_bytes) {
      memcpy(scope->payload + write_offset, payload,
             (size_t)payload_bytes);
      write_offset += payload_bytes;
   }
   assert(write_offset == total_payload);
   scope->payload_bytes = total_payload;
   mtx_unlock(&queue->helios_record_mutex);
   return VK_SUCCESS;
}

VkResult
vn_helios_record_memory_teardown(struct vn_device *dev,
                                 struct vn_device_memory *mem)
{
   if (!dev || !mem || mem->base.vk.base.device != &dev->base.vk ||
       !mem->helios_outer_registered || !mem->helios_free_pending ||
       !mem->helios_outer.device_generation ||
       !mem->helios_outer.outer_allocation_token ||
       !mem->helios_outer.outer_allocation_bytes) {
      vn_helios_record_note_deferred_use(dev ? dev->instance : NULL);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(dev->instance);
   struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
   if (!scope || !scope->context || scope->context->owner != owner ||
       !scope->context->queue) {
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   const struct vn_helios_command_use use = {
      .binding = {
         .device_generation = mem->helios_outer.device_generation,
         .outer_allocation_token =
            mem->helios_outer.outer_allocation_token,
         .outer_allocation_bytes =
            mem->helios_outer.outer_allocation_bytes,
         .byte_offset = 0,
         .byte_length = mem->helios_outer.outer_allocation_bytes,
         .valid = true,
      },
      .access_flags = HELIOS_HOB1_ACCESS_READ |
                      HELIOS_HOB1_ACCESS_WRITE,
   };
   return helios_record_append(
      scope->context->queue, HELIOS_RECORD_REFUSE_DEFERRED_USE,
      NULL, 0, &use, 1, NULL, true);
}

bool
vn_helios_record_has_active_scope(struct vn_instance *instance)
{
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(instance);
   return owner && helios_current_scope(owner) != NULL;
}

static VkResult
helios_scope_track_fence_and_submit1_signals(
   struct vn_queue *queue,
   VkFence fence_handle,
   uint32_t submit_count,
   const VkSubmitInfo *submits)
{
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner = helios_submit_owner(dev->instance);
   struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
   if (!scope || scope->context->queue != queue)
      return VK_ERROR_VALIDATION_FAILED_EXT;
   if (fence_handle) {
      struct vn_fence *fence = vn_fence_from_handle(fence_handle);
      if (!fence || !helios_object_owned(&fence->base.vk, dev) ||
          (scope->fence && scope->fence != fence))
         goto invalid;
      scope->fence = fence;
   }
   for (uint32_t s = 0; s < submit_count; s++) {
      const VkSubmitInfo *submit = &submits[s];
      const VkTimelineSemaphoreSubmitInfo *timeline =
         vk_find_struct_const(submit->pNext,
                              TIMELINE_SEMAPHORE_SUBMIT_INFO);
      for (uint32_t i = 0; i < submit->signalSemaphoreCount; i++) {
         struct vn_semaphore *sem =
            vn_semaphore_from_handle(submit->pSignalSemaphores[i]);
         if (sem && sem->payload &&
             sem->payload->type == VN_SYNC_TYPE_IMPORTED_WIN32_SYNC)
            continue;
         const uint64_t value =
            sem && sem->type == VK_SEMAPHORE_TYPE_TIMELINE
               ? (timeline ? timeline->pSignalSemaphoreValues[i] : 0)
               : 1;
         if (!helios_scope_add_signal(scope, dev, sem, value))
            goto invalid;
      }
   }
   return VK_SUCCESS;

invalid:
   scope->failure = HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
   helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
   return VK_ERROR_VALIDATION_FAILED_EXT;
}

static VkResult
helios_scope_track_fence_and_submit2_signals(
   struct vn_queue *queue,
   VkFence fence_handle,
   uint32_t submit_count,
   const VkSubmitInfo2 *submits)
{
   struct vn_device *dev = vn_device_from_vk(queue->base.vk.base.device);
   struct vn_helios_submit_instance *owner = helios_submit_owner(dev->instance);
   struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
   if (!scope || scope->context->queue != queue)
      return VK_ERROR_VALIDATION_FAILED_EXT;
   if (fence_handle) {
      struct vn_fence *fence = vn_fence_from_handle(fence_handle);
      if (!fence || !helios_object_owned(&fence->base.vk, dev) ||
          (scope->fence && scope->fence != fence))
         goto invalid;
      scope->fence = fence;
   }
   for (uint32_t s = 0; s < submit_count; s++) {
      const VkSubmitInfo2 *submit = &submits[s];
      for (uint32_t i = 0; i < submit->signalSemaphoreInfoCount; i++) {
         const VkSemaphoreSubmitInfo *info =
            &submit->pSignalSemaphoreInfos[i];
         struct vn_semaphore *sem =
            vn_semaphore_from_handle(info->semaphore);
         if (sem && sem->payload &&
             sem->payload->type == VN_SYNC_TYPE_IMPORTED_WIN32_SYNC)
            continue;
         const uint64_t value =
            sem && sem->type == VK_SEMAPHORE_TYPE_TIMELINE ? info->value : 1;
         if (!helios_scope_add_signal(scope, dev, sem, value))
            goto invalid;
      }
   }
   return VK_SUCCESS;

invalid:
   scope->failure = HELIOS_TRANSLATOR_STATUS_FOREIGN_VULKAN_HANDLE;
   helios_record_refuse(owner, HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
   return VK_ERROR_VALIDATION_FAILED_EXT;
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
   if (scope->payload_bytes == 0)
      return HELIOS_TRANSLATOR_STATUS_SCOPE_EMPTY;

   struct vn_queue *queue = scope->context->queue;
   mtx_lock(&queue->helios_record_mutex);
   uint32_t next_operand = 0;
   for (uint32_t i = 0; i < scope->use_count; i++) {
      if (scope->uses[i].first_operand != next_operand) {
         mtx_unlock(&queue->helios_record_mutex);
         return HELIOS_TRANSLATOR_STATUS_OPERAND_ENCODING;
      }
      const HeliosTranslatorStatusCode checked_use =
         helios_translator_check_sealed_use(
            &scope->uses[i], scope->context->context_flags);
      if (checked_use != HELIOS_TRANSLATOR_STATUS_OK) {
         mtx_unlock(&queue->helios_record_mutex);
         return checked_use;
      }
      next_operand += scope->uses[i].operand_count;
   }
   if (next_operand != scope->operand_count) {
      mtx_unlock(&queue->helios_record_mutex);
      return HELIOS_TRANSLATOR_STATUS_OPERAND_ENCODING;
   }
   for (uint32_t i = 0; i < scope->operand_count; i++) {
      const HeliosTranslatorStatusCode checked_operand =
         helios_translator_check_sealed_operand(
            &scope->operands[i], scope->payload_bytes, scope->use_count);
      if (checked_operand != HELIOS_TRANSLATOR_STATUS_OK) {
         mtx_unlock(&queue->helios_record_mutex);
         return checked_operand;
      }
      const HeliosTranslatorStatusCode checked_zero =
         helios_translator_check_operand_payload_zero(
            &scope->operands[i], scope->payload, scope->payload_bytes);
      if (checked_zero != HELIOS_TRANSLATOR_STATUS_OK) {
         mtx_unlock(&queue->helios_record_mutex);
         return checked_zero;
      }
   }
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

static bool
helios_scope_retire_deferred(struct HeliosTranslatorScope_T *scope,
                             bool committed)
{
   if (!scope->deferred_record_count)
      return true;
   struct vn_device *dev = vn_device_from_vk(
      scope->context->queue->base.vk.base.device);
   if (!dev)
      return false;

   struct vn_device_memory *pending_free = NULL;

   simple_mtx_lock(&dev->mutex);
   for (uint32_t i = 0; i < scope->deferred_record_count; i++) {
      const struct vn_helios_deferred_record *record =
         scope->deferred_records[i];
      if (!record || !record->owner || list_is_empty(&record->link) ||
          !record->owner->helios_outer_registered ||
          record->reserved_context_generation !=
             scope->context->context_generation ||
          record->reserved_batch_id != scope->batch_id) {
         simple_mtx_unlock(&dev->mutex);
         return false;
      }
      if (record->final_free_record) {
         if (pending_free || !record->owner->helios_free_pending) {
            simple_mtx_unlock(&dev->mutex);
            return false;
         }
         pending_free = record->owner;
      }
   }
   for (uint32_t i = 0; i < scope->deferred_record_count; i++) {
      struct vn_helios_deferred_record *record =
         scope->deferred_records[i];
      record->reserved_context_generation = 0;
      record->reserved_batch_id = 0;
      if (committed) {
         struct vn_device_memory *mem = record->owner;
         list_delinit(&record->link);
         assert(mem->helios_deferred_record_count > 0);
         mem->helios_deferred_record_count--;
         if (record->allocation_record) {
            assert(!mem->helios_host_materialized);
            mem->helios_host_materialized = true;
         }
      }
   }
   simple_mtx_unlock(&dev->mutex);

   if (committed) {
      for (uint32_t i = 0; i < scope->deferred_record_count; i++)
         vn_device_memory_helios_record_destroy(
            scope->deferred_records[i]);
   }
   scope->deferred_record_count = 0;
   if (committed && pending_free &&
       !vn_device_memory_helios_finalize_pending_free(dev, pending_free))
      return false;
   return true;
}

static bool
helios_scope_note_allocation_progress(struct HeliosTranslatorScope_T *scope,
                                      uint64_t progress_value)
{
   if (!scope || !scope->context || !scope->context->queue ||
       !scope->context->context_generation || !progress_value)
      return false;
   struct vn_device *dev = vn_device_from_vk(
      scope->context->queue->base.vk.base.device);
   if (!dev)
      return false;

   const uint64_t context_generation =
      scope->context->context_generation;
   bool complete = true;
   simple_mtx_lock(&dev->mutex);
   for (uint32_t i = 0; i < scope->use_count && complete; i++) {
      const HeliosSealedResourceUseV1 *use = &scope->uses[i];
      struct vn_device_memory *match = NULL;
      list_for_each_entry(struct vn_device_memory, mem,
                          &dev->helios_outer_allocations,
                          helios_outer_link) {
         if (mem->helios_outer.outer_allocation_token ==
                use->outer_allocation_token) {
            if (match) {
               complete = false;
               break;
            }
            match = mem;
         }
      }
      if (!complete || !match) {
         complete = false;
         break;
      }

      uint32_t slot = 0;
      while (slot < match->helios_outer_progress_count &&
             match->helios_outer_progress[slot].context_generation !=
                context_generation)
         slot++;
      if (slot == match->helios_outer_progress_count) {
         if (slot == ARRAY_SIZE(match->helios_outer_progress)) {
            complete = false;
            break;
         }
         match->helios_outer_progress_count++;
         match->helios_outer_progress[slot].context_generation =
            context_generation;
      }
      match->helios_outer_progress[slot].progress_value =
         MAX2(match->helios_outer_progress[slot].progress_value,
              progress_value);
   }
   simple_mtx_unlock(&dev->mutex);
   return complete;
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
   if (disposition == HELIOS_TRANSLATOR_SCOPE_DISPOSITION_COMMITTED &&
       !helios_scope_note_allocation_progress(scope, close->progress_value))
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_ALLOCATION_TOKEN;
   if (!helios_scope_retire_deferred(
          scope,
          disposition == HELIOS_TRANSLATOR_SCOPE_DISPOSITION_COMMITTED))
      return HELIOS_TRANSLATOR_STATUS_UNKNOWN_ALLOCATION_TOKEN;

   if (disposition == HELIOS_TRANSLATOR_SCOPE_DISPOSITION_COMMITTED) {
      const uint64_t context_generation =
         scope->context->context_generation;
      const uint64_t progress = close->progress_value;
      for (uint32_t i = 0; i < scope->query_pool_count; i++) {
         struct vn_query_pool *pool = scope->query_pools[i];
         simple_mtx_lock(&pool->mutex);
         pool->helios_context_generation = context_generation;
         pool->helios_progress_value = progress;
         simple_mtx_unlock(&pool->mutex);
      }
      for (uint32_t i = 0; i < scope->event_count; i++) {
         struct vn_event *event = scope->events[i];
         simple_mtx_lock(&event->helios_progress_mtx);
         event->helios_context_generation = context_generation;
         event->helios_progress_value = progress;
         simple_mtx_unlock(&event->helios_progress_mtx);
      }
      if (scope->fence) {
         simple_mtx_lock(&scope->fence->helios_progress_mtx);
         scope->fence->helios_context_generation = context_generation;
         scope->fence->helios_progress_value = progress;
         scope->fence->helios_locally_signaled = false;
         simple_mtx_unlock(&scope->fence->helios_progress_mtx);
      }
      for (uint32_t i = 0; i < scope->signal_count; i++) {
         struct vn_semaphore *sem = scope->signals[i].semaphore;
         simple_mtx_lock(&sem->helios_progress_mtx);
         sem->helios_context_generation = context_generation;
         sem->helios_progress_value = progress;
         sem->helios_progress_signal_value = scope->signals[i].value;
         simple_mtx_unlock(&sem->helios_progress_mtx);
      }
   }

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
   free(scope->deferred_records);
   free(scope->query_pools);
   free(scope->events);
   free(scope->signals);
   free(scope->uses);
   free(scope->payload);
   free(scope);
   return HELIOS_TRANSLATOR_STATUS_OK;
}

bool
vn_helios_query_pool_progress(struct vn_query_pool *pool,
                              uint64_t *context_generation,
                              uint64_t *progress_value)
{
   if (!pool || !context_generation || !progress_value)
      return false;
   simple_mtx_lock(&pool->mutex);
   *context_generation = pool->helios_context_generation;
   *progress_value = pool->helios_progress_value;
   simple_mtx_unlock(&pool->mutex);
   return *context_generation != 0 && *progress_value != 0;
}

bool
vn_helios_record_scope_identity(HeliosTranslatorScope opaque,
                                struct vn_instance **out_instance,
                                uint64_t *out_context_generation)
{
   struct HeliosTranslatorScope_T *scope = opaque;
   if (!scope || !scope->context || !scope->context->owner ||
       !out_instance || !out_context_generation)
      return false;
   *out_instance = scope->context->owner->instance;
   *out_context_generation = scope->context->context_generation;
   return *out_instance != NULL && *out_context_generation != 0;
}

bool
vn_helios_record_current_context(struct vn_instance *instance,
                                 uint64_t *out_context_generation)
{
   if (!instance || !out_context_generation)
      return false;
   struct vn_helios_submit_instance *owner =
      helios_submit_owner(instance);
   struct HeliosTranslatorScope_T *scope = helios_current_scope(owner);
   if (!scope || !helios_scope_on_calling_thread(scope) ||
       !scope->context || scope->context->owner != owner ||
       !scope->context->context_generation)
      return false;
   *out_context_generation = scope->context->context_generation;
   return true;
}

void
vn_helios_record_note_loader_provenance(struct vn_instance *instance)
{
   helios_record_refuse(helios_submit_owner(instance),
                        HELIOS_RECORD_REFUSE_LOADER_PROVENANCE);
}

void
vn_helios_record_note_foreign_handle(struct vn_instance *instance)
{
   helios_record_refuse(helios_submit_owner(instance),
                        HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
}

void
vn_helios_record_note_withheld_proc(struct vn_instance *instance)
{
   helios_record_refuse(helios_submit_owner(instance),
                        HELIOS_RECORD_REFUSE_WITHHELD_PROC);
}

void
vn_helios_record_note_reentrant_join(struct vn_instance *instance)
{
   helios_record_refuse(helios_submit_owner(instance),
                        HELIOS_RECORD_REFUSE_REENTRANT_JOIN);
}

void
vn_helios_record_note_deferred_use(struct vn_instance *instance)
{
   if (instance)
      helios_record_refuse(helios_submit_owner(instance),
                           HELIOS_RECORD_REFUSE_DEFERRED_USE);
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
      /* QueueSubmit/QueueSubmit2 use the complete A7 classifier and bypass
       * this generic payload helper.  Any other allocation-bearing opcode is
       * incomplete closure: it may not leak a local HVM1 handle or fabricate
       * an outer token. */
      if (allocation_count) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      return helios_record_append(queue, refusal, payload, payload_bytes,
                                  NULL, 0, NULL, false);
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
helios_join_current(struct vn_device *dev)
{
   HeliosSyncProgressResultV1 progress;
   memset(&progress, 0, sizeof(progress));
   const HeliosTranslatorStatusCode status =
      vn_helios_direct_join_current(dev->instance, 0, &progress);
   if (status == HELIOS_TRANSLATOR_STATUS_OK)
      return (progress.flags & HELIOS_TRANSLATOR_PROGRESS_FLAG_DEVICE_LOST)
                ? VK_ERROR_DEVICE_LOST
                : VK_SUCCESS;
   if (status == HELIOS_TRANSLATOR_STATUS_SESSION_POISONED ||
       status == HELIOS_TRANSLATOR_STATUS_HOST_CALLBACK_FAILED)
      return VK_ERROR_DEVICE_LOST;
   helios_record_refuse(helios_submit_owner(dev->instance),
                        status == HELIOS_TRANSLATOR_STATUS_REENTRANT_JOIN
                           ? HELIOS_RECORD_REFUSE_REENTRANT_JOIN
                           : HELIOS_RECORD_REFUSE_CONTROL_CLASS);
   return VK_ERROR_VALIDATION_FAILED_EXT;
}

static bool
helios_submit_use_add(struct vn_helios_command_use **uses,
                      uint32_t *count,
                      uint32_t *capacity,
                      const struct vn_helios_command_use *candidate)
{
   for (uint32_t i = 0; i < *count; i++) {
      struct vn_helios_command_use *use = &(*uses)[i];
      if (use->binding.outer_allocation_token !=
          candidate->binding.outer_allocation_token)
         continue;
      if (use->binding.device_generation !=
             candidate->binding.device_generation ||
          use->binding.outer_allocation_bytes !=
             candidate->binding.outer_allocation_bytes)
         return false;
      const uint64_t old_end =
         use->binding.byte_offset + use->binding.byte_length;
      const uint64_t candidate_end =
         candidate->binding.byte_offset + candidate->binding.byte_length;
      const uint64_t start = MIN2(use->binding.byte_offset,
                                  candidate->binding.byte_offset);
      const uint64_t end = MAX2(old_end, candidate_end);
      if (old_end < use->binding.byte_offset ||
          candidate_end < candidate->binding.byte_offset || end < start ||
          end > use->binding.outer_allocation_bytes)
         return false;
      use->binding.byte_offset = start;
      use->binding.byte_length = end - start;
      use->access_flags |= candidate->access_flags;
      return true;
   }

   if (*count == HELIOS_HOB1_MAX_USE_RECORDS)
      return false;
   if (*count == *capacity) {
      uint32_t next = *capacity ? *capacity * 2u : 16u;
      if (next < *capacity || next > HELIOS_HOB1_MAX_USE_RECORDS)
         next = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(next, sizeof(**uses), &bytes))
         return false;
      void *new_uses = realloc(*uses, bytes);
      if (!new_uses)
         return false;
      *uses = new_uses;
      *capacity = next;
   }
   (*uses)[(*count)++] = *candidate;
   return true;
}

static void
helios_command_streams_fini(struct helios_command_streams *streams)
{
   free(streams->commands);
   memset(streams, 0, sizeof(*streams));
}

static VkResult
helios_command_streams_add_recursive(
   struct vn_device *dev,
   struct helios_command_streams *streams,
   struct vn_command_buffer *cmd,
   struct vn_command_buffer **stack,
   uint32_t depth)
{
   if (!cmd || depth == 64 || helios_cmd_device(cmd) != dev ||
       cmd->base.vk.state != MESA_VK_COMMAND_BUFFER_STATE_EXECUTABLE ||
       !cmd->builder.helios_closure_complete ||
       vn_cs_encoder_get_fatal(&cmd->cs) ||
       !vn_cs_encoder_get_len(&cmd->cs))
      return VK_ERROR_VALIDATION_FAILED_EXT;
   for (uint32_t i = 0; i < depth; i++) {
      if (stack[i] == cmd)
         return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   for (uint32_t i = 0; i < streams->count; i++) {
      if (streams->commands[i] == cmd)
         return VK_SUCCESS;
   }

   stack[depth] = cmd;
   for (uint32_t i = 0; i < cmd->builder.helios_secondary_count; i++) {
      VkResult result = helios_command_streams_add_recursive(
         dev, streams, cmd->builder.helios_secondaries[i], stack,
         depth + 1);
      if (result != VK_SUCCESS)
         return result;
   }

   if (streams->count == HELIOS_HOB1_MAX_USE_RECORDS)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   if (streams->count == streams->capacity) {
      uint32_t capacity = streams->capacity ? streams->capacity * 2u : 16u;
      if (capacity < streams->capacity ||
          capacity > HELIOS_HOB1_MAX_USE_RECORDS)
         capacity = HELIOS_HOB1_MAX_USE_RECORDS;
      size_t bytes;
      if (!helios_size_mul(capacity, sizeof(*streams->commands), &bytes))
         return VK_ERROR_OUT_OF_HOST_MEMORY;
      void *commands = realloc(streams->commands, bytes);
      if (!commands)
         return VK_ERROR_OUT_OF_HOST_MEMORY;
      streams->commands = commands;
      streams->capacity = capacity;
   }
   streams->commands[streams->count++] = cmd;
   return VK_SUCCESS;
}

static VkResult
helios_collect_command_buffer_uses(
   struct vn_device *dev,
   VkCommandBuffer handle,
   struct vn_helios_command_use **uses,
   uint32_t *count,
   uint32_t *capacity,
   struct helios_command_streams *streams)
{
   struct vn_command_buffer *cmd =
      vn_command_buffer_from_handle(handle);
   if (!cmd || cmd->base.vk.pool->base.device != &dev->base.vk ||
       cmd->base.vk.state == MESA_VK_COMMAND_BUFFER_STATE_INVALID ||
       !cmd->builder.helios_closure_complete) {
      helios_record_refuse(helios_submit_owner(dev->instance),
                           HELIOS_RECORD_REFUSE_DEFERRED_USE);
      return VK_ERROR_VALIDATION_FAILED_EXT;
   }
   struct vn_command_buffer *stack[64];
   VkResult stream_result = helios_command_streams_add_recursive(
      dev, streams, cmd, stack, 0);
   if (stream_result != VK_SUCCESS) {
      helios_record_refuse(helios_submit_owner(dev->instance),
                           stream_result == VK_ERROR_OUT_OF_HOST_MEMORY
                              ? HELIOS_RECORD_REFUSE_BATCH_BOUND
                              : HELIOS_RECORD_REFUSE_DEFERRED_USE);
      return stream_result;
   }
   for (uint32_t i = 0; i < cmd->builder.helios_use_count; i++) {
      const struct vn_helios_command_use *candidate =
         &cmd->builder.helios_uses[i];
      if (!vn_device_memory_helios_binding_live(dev,
                                                &candidate->binding)) {
         helios_record_refuse(helios_submit_owner(dev->instance),
                              HELIOS_RECORD_REFUSE_FOREIGN_HANDLE);
         return VK_ERROR_VALIDATION_FAILED_EXT;
      }
      if (!helios_submit_use_add(uses, count, capacity, candidate)) {
         helios_record_refuse(helios_submit_owner(dev->instance),
                              HELIOS_RECORD_REFUSE_BATCH_BOUND);
         return VK_ERROR_OUT_OF_HOST_MEMORY;
      }
   }
   return VK_SUCCESS;
}

static VkResult
helios_submit1_collect_uses(struct vn_device *dev,
                            uint32_t submit_count,
                            const VkSubmitInfo *submits,
                            struct vn_helios_command_use **out_uses,
                            uint32_t *out_count,
                            struct helios_command_streams *out_streams)
{
   struct vn_helios_command_use *uses = NULL;
   uint32_t count = 0;
   uint32_t capacity = 0;
   for (uint32_t s = 0; s < submit_count; s++) {
      for (uint32_t i = 0; i < submits[s].commandBufferCount; i++) {
         const VkResult result = helios_collect_command_buffer_uses(
            dev, submits[s].pCommandBuffers[i], &uses, &count, &capacity,
            out_streams);
         if (result != VK_SUCCESS) {
            free(uses);
            helios_command_streams_fini(out_streams);
            return result;
         }
      }
   }
   *out_uses = uses;
   *out_count = count;
   return VK_SUCCESS;
}

static VkResult
helios_submit2_collect_uses(struct vn_device *dev,
                            uint32_t submit_count,
                            const VkSubmitInfo2 *submits,
                            struct vn_helios_command_use **out_uses,
                            uint32_t *out_count,
                            struct helios_command_streams *out_streams)
{
   struct vn_helios_command_use *uses = NULL;
   uint32_t count = 0;
   uint32_t capacity = 0;
   for (uint32_t s = 0; s < submit_count; s++) {
      for (uint32_t i = 0; i < submits[s].commandBufferInfoCount; i++) {
         const VkResult result = helios_collect_command_buffer_uses(
            dev, submits[s].pCommandBufferInfos[i].commandBuffer,
            &uses, &count, &capacity, out_streams);
         if (result != VK_SUCCESS) {
            free(uses);
            helios_command_streams_fini(out_streams);
            return result;
         }
      }
   }
   *out_uses = uses;
   *out_count = count;
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
   const size_t bounded_payload = vn_sizeof_vkQueueSubmit(
      vn_queue_to_handle(queue), submit_count, submits, fence);
   if (!bounded_payload || bounded_payload > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   const enum vn_helios_submission_mode mode =
      vn_helios_submit_instance_mode(dev->instance);
   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      struct vn_helios_command_use *command_uses = NULL;
      uint32_t command_use_count = 0;
      struct helios_command_streams command_streams = { 0 };
      result = helios_submit1_collect_uses(
         dev, submit_count, submits, &command_uses, &command_use_count,
         &command_streams);
      if (result != VK_SUCCESS)
         return result;
      struct helios_submit1_copy copy;
      result = helios_submit1_copy_init(dev, submit_count, submits, &copy);
      if (result != VK_SUCCESS) {
         helios_command_streams_fini(&command_streams);
         free(command_uses);
         return result;
      }
      if (copy.waits.count || copy.signals.count) {
         /* HOB1 has no native-fence carrier: those dependencies belong to the
          * exact outer D3D context.  A5 may arrange that ordering around the
          * sealed batch, but A4 must not silently discard it or issue KMT
          * work from record-only mode. */
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
         helios_submit1_copy_fini(&copy);
         helios_command_streams_fini(&command_streams);
         free(command_uses);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      void *payload = NULL;
      size_t payload_bytes = 0;
      result =
         helios_encode_submit1(vn_queue_to_handle(queue), submit_count,
                               copy.infos, fence, &payload, &payload_bytes);
      if (result == VK_SUCCESS)
         result = helios_record_append(
            queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT, payload, payload_bytes,
            command_uses, command_use_count, &command_streams, false);
      if (result == VK_SUCCESS)
         result = helios_scope_track_fence_and_submit1_signals(
            queue, fence, submit_count, copy.infos);
      free(payload);
      helios_command_streams_fini(&command_streams);
      free(command_uses);
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
   const size_t bounded_payload = vn_sizeof_vkQueueSubmit2(
      vn_queue_to_handle(queue), submit_count, submits, fence);
   if (!bounded_payload || bounded_payload > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   const enum vn_helios_submission_mode mode =
      vn_helios_submit_instance_mode(dev->instance);
   if (mode == VN_HELIOS_SUBMISSION_MODE_RECORD_ONLY) {
      struct vn_helios_command_use *command_uses = NULL;
      uint32_t command_use_count = 0;
      struct helios_command_streams command_streams = { 0 };
      result = helios_submit2_collect_uses(
         dev, submit_count, submits, &command_uses, &command_use_count,
         &command_streams);
      if (result != VK_SUCCESS)
         return result;
      struct helios_submit2_copy copy;
      result = helios_submit2_copy_init(dev, submit_count, submits, &copy);
      if (result != VK_SUCCESS) {
         helios_command_streams_fini(&command_streams);
         free(command_uses);
         return result;
      }
      if (copy.waits.count || copy.signals.count) {
         helios_record_refuse(owner, HELIOS_RECORD_REFUSE_CONTROL_CLASS);
         helios_submit2_copy_fini(&copy);
         helios_command_streams_fini(&command_streams);
         free(command_uses);
         return VK_ERROR_FEATURE_NOT_PRESENT;
      }
      void *payload = NULL;
      size_t payload_bytes = 0;
      result =
         helios_encode_submit2(vn_queue_to_handle(queue), submit_count,
                               copy.infos, fence, &payload, &payload_bytes);
      if (result == VK_SUCCESS)
         result = helios_record_append(
            queue, HELIOS_RECORD_REFUSE_QUEUE_SUBMIT2, payload, payload_bytes,
            command_uses, command_use_count, &command_streams, false);
      if (result == VK_SUCCESS)
         result = helios_scope_track_fence_and_submit2_signals(
            queue, fence, submit_count, copy.infos);
      free(payload);
      helios_command_streams_fini(&command_streams);
      free(command_uses);
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
      /* Windows exposes no sparse queue or sparse residency feature in this
       * package generation.  A call smuggled through a private proc is an
       * unsupported A7 opcode, never a reason to submit local HVM1 identities
       * on HVC1 or to invent an outer sparse carrier. */
      helios_record_refuse(owner, HELIOS_RECORD_REFUSE_DEFERRED_USE);
      return VK_ERROR_FEATURE_NOT_PRESENT;
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
      return helios_join_current(dev);
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
      /* Device idle is entered from one runtime-approved outer callback, but
       * its Vulkan meaning covers every queue of this exact device. */
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
      const HeliosTranslatorStatusCode status =
         vn_helios_direct_join_all(dev->instance);
      if (status == HELIOS_TRANSLATOR_STATUS_OK)
         return VK_SUCCESS;
      if (status == HELIOS_TRANSLATOR_STATUS_SESSION_POISONED ||
          status == HELIOS_TRANSLATOR_STATUS_HOST_CALLBACK_FAILED)
         return VK_ERROR_DEVICE_LOST;
      helios_record_refuse(owner,
                           status == HELIOS_TRANSLATOR_STATUS_REENTRANT_JOIN
                              ? HELIOS_RECORD_REFUSE_REENTRANT_JOIN
                              : HELIOS_RECORD_REFUSE_CONTROL_CLASS);
      return VK_ERROR_VALIDATION_FAILED_EXT;
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
