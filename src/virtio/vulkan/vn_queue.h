/*
 * Copyright 2019 Google LLC
 * SPDX-License-Identifier: MIT
 *
 * based in part on anv and radv which are:
 * Copyright © 2015 Intel Corporation
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 */

#ifndef VN_QUEUE_H
#define VN_QUEUE_H

#include "vn_common.h"

struct vn_queue {
   struct vn_queue_base base;

   /* emulated queue shares base queue id and ring_idx with another queue */
   bool emulated;

   /* whether this queue supports venus feedback */
   bool can_feedback;

   /* only used if renderer supports multiple timelines */
   uint32_t ring_idx;

   /* wait fence used for vn_QueueWaitIdle */
   VkFence wait_fence;

   /* semaphore for gluing vkQueueSubmit feedback commands to
    * vkQueueBindSparse
    */
   VkSemaphore sparse_semaphore;
   uint64_t sparse_semaphore_counter;

   /* for vn_queue_submission storage */
   struct vn_cached_storage storage;

   /* for async queue present */
   struct {
      /* Protects VkQueue host access except async present states. */
      simple_mtx_t queue_mutex;
      /* Protects state transitions: initialized, pending and join. */
      mtx_t mutex;
      /* Wake up async present thread upon presentation. */
      cnd_t cond;
      /* This is the async present thread. */
      thrd_t thread;
      /* Avoid extra locking on async present thread. */
      pid_t tid;
      /* Track whether the async present thread has been initialized. */
      bool initialized;
      /* Track whether the present is still pending acquired. */
      bool pending;
      /* Track whether to join the async present thread. */
      bool join;
      /* This is a deep copy of the requested presentation. */
      VkPresentInfoKHR *info;
      /* Track the result of the presentation. */
      VkResult result;
      /* This is used by vtest to properly wait before present. */
      VkFence fence;
   } async_present;
};
VK_DEFINE_HANDLE_CASTS(vn_queue, base.vk.base, VkQueue, VK_OBJECT_TYPE_QUEUE)

enum vn_sync_type {
   /* no payload */
   VN_SYNC_TYPE_INVALID,

   /* device object */
   VN_SYNC_TYPE_DEVICE_ONLY,

   /* payload is an imported sync file */
   VN_SYNC_TYPE_IMPORTED_SYNC_FD,

#if defined(_WIN32)
   /* payload is an imported WDDM monitored-fence synchronization object */
   VN_SYNC_TYPE_IMPORTED_WIN32_SYNC,
#endif
};

struct vn_sync_payload {
   enum vn_sync_type type;

   /* If type is VN_SYNC_TYPE_IMPORTED_SYNC_FD, fd is a sync file. */
   int fd;

#if defined(_WIN32)
   struct vn_renderer_sync *win32_sync;
#endif
};

/* For external fences and external semaphores submitted to be signaled. The
 * Vulkan spec guarantees those external syncs are on permanent payload.
 */
struct vn_sync_payload_external {
   /* ring_idx of the last queue submission */
   uint32_t ring_idx;
   /* valid when NO_ASYNC_QUEUE_SUBMIT perf option is not used */
   bool ring_seqno_valid;
   /* ring seqno of the last queue submission */
   uint32_t ring_seqno;
};

struct vn_feedback_slot;

struct vn_fence {
   struct vn_object_base base;

   struct vn_sync_payload *payload;

   struct vn_sync_payload permanent;
   struct vn_sync_payload temporary;

   struct {
      /* non-NULL if VN_PERF_NO_FENCE_FEEDBACK is disabled */
      struct vn_feedback_slot *slot;
      VkCommandBuffer *commands;

      /* Indicate whether the fence status in the feedback slot is pollable.
       * When pollable is false, the fence feedback has been suspended and the
       * slot won't be signaled to VK_SUCCESS.
       * - suspend: submit on queues not supporting feedback
       * - resume: vn_ResetFences will reset pollable to true
       */
      bool pollable;
   } feedback;

   bool is_external;
   struct vn_sync_payload_external external_payload;

#if DETECT_OS_WINDOWS
   VkExternalSemaphoreHandleTypeFlags external_handle_types;
#endif
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_fence,
                               base.vk,
                               VkFence,
                               VK_OBJECT_TYPE_FENCE)

struct vn_semaphore {
   struct vn_object_base base;

   VkSemaphoreType type;

#if DETECT_OS_WINDOWS
   /* Serializes host counter query+CPU signal when a WDDM-imported timeline
    * is mirrored into its otherwise independent host VkSemaphore.  The pair
    * must be atomic with respect to another guest thread mirroring the same
    * value, because duplicate vkSignalSemaphore values are invalid. */
   simple_mtx_t helios_host_signal_mtx;

   /* Helios stale-signal guard (atomic; no lock — counter_mtx is only
    * initialized when a feedback slot is allocated, which win32 exported
    * timelines may skip). The greatest timeline value ever forwarded to the
    * HOST for this semaphore, across BOTH the queue-submit signal path and the
    * explicit vn_SignalSemaphore forward. Seeded to the initial value at
    * create. Consulted by vn_SignalSemaphore to drop a stale explicit signal
    * (value <= this) that the queue-submit path already superseded — such a
    * forward would trip VUID-VkSemaphoreSignalInfo-value-03258 on the host and
    * drop the renderer context (dwm -> terminal DEVICE_LOST / black desktop).
    * Timeline monotonicity makes skipping it safe: the timeline has already
    * reached at least `value`, so no progress is lost. */
   uint64_t helios_max_forwarded_host_value;
#endif

   struct vn_sync_payload *payload;

   struct vn_sync_payload permanent;
   struct vn_sync_payload temporary;

   struct {
      /* non-NULL if VN_PERF_NO_SEMAPHORE_FEEDBACK is disabled */
      struct vn_feedback_slot *slot;

      /* Lists of allocated vn_semaphore_feedback_cmd
       *
       * On submission prepare, sfb cmd is cache allocated from the free list
       * and is moved to the pending list after initialization.
       *
       * On submission cleanup, sfb cmds of the owner semaphores are checked
       * and cached to the free list if they have been "signaled", which is
       * proxyed via the src slot value having been reached.
       */
      struct list_head pending_cmds;
      struct list_head free_cmds;
      uint32_t free_cmd_count;

      /* Lock for accessing free/pending sfb cmds */
      simple_mtx_t cmd_mtx;

      /* Indicate whether the timeline semaphore counter value in the feedback
       * slot is pollable. When pollable is false, the semaphore feedback has
       * been suspended and the slot won't be signaled to the pending counter.
       * - suspend: submit on queues not supporting feedback
       * - resume if any of below occurs:
       *   - vn_SignalSemaphore
       *   - when the queried counter value is no smaller than the suspended
       *     counter value
       */
      bool pollable;

      /* When feedback is active, signaled_counter is the cached counter value
       * to track if an async sem wait call is needed.
       *
       * When feedback is suspended, suspended_counter tracks the greatest
       * signal counter value submitted on queues not supporting feedback.
       *
       * They share the same storage and the value is monotonic.
       */
      union {
         uint64_t signaled_counter;
         uint64_t suspended_counter;
      };

      /* Helios forward-progress deadline (guarded by counter_mtx): when the
       * slot counter sits unchanged past the deadline while a submitted
       * signal op is pending, the renderer context is treated as lost even
       * if it keeps answering counter queries with stale success (the host
       * driver may never report VK_ERROR_DEVICE_LOST after killing the GPU
       * channel). stall_since_ns == 0 means "not stalled".
       *
       * stale_strikes counts CONSECUTIVE deadline windows whose synchronous
       * renderer probe showed zero counter movement; the loss latch requires
       * VN_HELIOS_SEM_DEADLINE_STRIKES of them. One 8 s stale-success window
       * is NOT proof of death under the C3/M3.4 async transport — a
       * validate-slow host under boot/login churn legitimately stalls a
       * semaphore that long (2026-07-04: the single-window latch killed dwm
       * at Present #329 on an otherwise healthy host, freezing the desktop
       * and starting an IddCx WUDFHost kill/recover loop). A genuinely dead
       * channel (the Xid-109 zombie this deadline exists for) shows zero
       * movement forever and still latches after strikes x deadline. Reset
       * whenever the counter moves. */
      int64_t stall_since_ns;
      uint64_t stall_counter;
      uint32_t stale_strikes;

      /* Helios strike attribution (diag only; guarded by counter_mtx): the
       * queue and value of the most recent submitted signal op targeting
       * this semaphore, recorded at submission prepare, so a sem-deadline
       * strike names the queue whose work is not completing instead of an
       * anonymous slot value (18th session: dwm+explorer+WUDFHost strike
       * ~1/min on a healthy boot; which semaphore/queue was unknown). */
      uint64_t last_signal_queue_id;
      uint32_t last_signal_family;
      uint32_t last_signal_ring_idx;
      uint64_t last_signal_value;
      int64_t last_signal_ns;
      VkSemaphore last_signal_wait_sem_handle;
      uint64_t last_signal_wait_sem_id;
      uint64_t last_signal_wait_value;

      /* Start of the current continuous period in which at least one signal
       * op is pending host-side with no observed counter movement (guarded by
       * cmd_mtx; 0 = not armed). Armed when the pending list gains an entry,
       * re-armed (or cleared if the list drained) when progress recycles
       * pending cmds, restarted on every strike.
       *
       * This is what the sem-deadline must measure — NOT wall time since the
       * counter last moved: a timeline wait legally begins long before its
       * signal is submitted (wait-before-signal), and an idle desktop parks
       * such waits for many seconds. 18th-session attribution showed every
       * observed strike had sig_age_ms <= 14: the deadline was firing on
       * idle waits the moment the NEXT frame's signal was submitted, which
       * is how a healthy login-churn boot accumulated 4 windows and tripped
       * the DEVICE_LOST latch (the 2026-07-05 freeze cascade's first act). */
      int64_t pending_signal_since_ns;

      /* Lock for checking if an async sem wait call is needed based on
       * the current counter value and signaled_counter to ensure async
       * wait order across threads.
       *
       * Also lock to protect suspended_counter and pollable updates.
       */
      simple_mtx_t counter_mtx;
   } feedback;

   bool is_external;
   struct vn_sync_payload_external external_payload;

#if defined(_WIN32)
   VkExternalSemaphoreHandleTypeFlags external_handle_types;

   /* Nonzero only after helios_venus_register_present_stream accepted this
    * exact OPAQUE_WIN32 exported timeline.  It is not inferred from a handle,
    * queue, process, or creation order. */
   uint64_t helios_present_stream_cookie;
#endif
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_semaphore,
                               base.vk,
                               VkSemaphore,
                               VK_OBJECT_TYPE_SEMAPHORE)

struct vn_event {
   struct vn_object_base base;

   /* non-NULL if below are satisfied:
    * - event is created without VK_EVENT_CREATE_DEVICE_ONLY_BIT
    * - VN_PERF_NO_EVENT_FEEDBACK is disabled
    */
   struct vn_feedback_slot *feedback_slot;
};
VK_DEFINE_NONDISP_HANDLE_CASTS(vn_event,
                               base.vk,
                               VkEvent,
                               VK_OBJECT_TYPE_EVENT)

#endif /* VN_QUEUE_H */
