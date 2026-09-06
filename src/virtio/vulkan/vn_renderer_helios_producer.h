/* Included by vn_renderer_helios.c after its private backend definitions.
 * The sole versioned ICD implementation of the allocation producer ABI. */
#include "helios_producer_abi.h"

struct helios_producer_binding {
   struct helios *renderer;
   struct vn_device *device;
   const struct helios_producer_status *status;
   uint64_t token, generation;
   volatile LONG references;
   HANDLE event;
   mtx_t wait_mutex;
};

static volatile LONG helios_producer_refused;

static int32_t
helios_producer_error(void)
{
   const LONG n = InterlockedIncrement(&helios_producer_refused);
   if (n == 1 || n % 64 == 0)
      helios_diag("producer interface refused operation (x%ld)", n);
   return VK_ERROR_DEVICE_LOST;
}

static struct helios_producer_control
helios_producer_request(uint32_t op, uint64_t binding)
{
   struct helios_producer_control r = {0};
   r.magic = HELIOS_ESCAPE_MAGIC;
   r.command = HELIOS_PRODUCER_ESCAPE;
   r.header_version = HELIOS_ESCAPE_VERSION;
   r.header_size = sizeof(r);
   r.version = HELIOS_PRODUCER_ABI;
   r.op = op;
   r.binding = binding;
   return r;
}

bool helios_venus_register_present_stream(VkDevice device, VkSemaphore semaphore, uint64_t *cookie);

static int32_t
helios_producer_stream(uintptr_t device, uint64_t semaphore, uint32_t *ctx, uint64_t *cookie)
{
   if (!device || !semaphore || !ctx || !cookie) return helios_producer_error();
   *ctx = 0; *cookie = 0;
   struct vn_device *dev = vn_device_from_handle((VkDevice)device);
   struct helios *h = (struct helios *)dev->renderer;
   struct vn_semaphore *sem = vn_semaphore_from_handle((VkSemaphore)(uintptr_t)semaphore);
   if (sem->base.vk.device != &dev->base.vk || !h || h->instance != dev->instance || !h->ctx_id ||
       !helios_venus_register_present_stream((VkDevice)device, (VkSemaphore)semaphore, cookie))
      return helios_producer_error();
   *ctx = h->ctx_id;
   return VK_SUCCESS;
}

static int32_t
helios_producer_bind(uintptr_t device, uint32_t allocation, void **out)
{
   if (!out) return helios_producer_error();
   *out = NULL;
   if (!device || !allocation) return helios_producer_error();
   struct vn_device *dev = vn_device_from_handle((VkDevice)device);
   struct helios *h = (struct helios *)dev->renderer;
   if (!h || h->instance != dev->instance || !h->device || !h->ctx_id)
      return helios_producer_error();
   struct helios_producer_binding *b = calloc(1, sizeof(*b));
   if (!b) return VK_ERROR_OUT_OF_HOST_MEMORY;
   b->event = CreateEventW(NULL, FALSE, FALSE, NULL);
   if (!b->event) { free(b); return VK_ERROR_OUT_OF_HOST_MEMORY; }
   if (mtx_init(&b->wait_mutex, mtx_plain) != thrd_success) {
      CloseHandle(b->event); free(b); return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   struct helios_producer_control r = helios_producer_request(HP_MAP, 0);
   mtx_lock(&h->dev_mutex);
   bool ok = true;
   if (!h->producer_status) {
      ok = helios_escape(h, &r, sizeof(r)) && r.user_va && r.size == 8192 * 64;
      if (ok) h->producer_status = (void *)(uintptr_t)r.user_va;
   }
   r = helios_producer_request(HP_BIND, 0);
   r.allocation = allocation;
   ok = ok && helios_escape(h, &r, sizeof(r));
   mtx_unlock(&h->dev_mutex);
   if (!ok || !r.binding || !r.generation || r.slot >= 8192) {
      mtx_destroy(&b->wait_mutex); CloseHandle(b->event); free(b);
      return helios_producer_error();
   }
   b->renderer = h; b->device = dev; b->token = r.binding;
   b->generation = r.generation; b->status = &h->producer_status[r.slot];
   b->references = 1;
   *out = b;
   return VK_SUCCESS;
}

static int32_t
helios_producer_status(void *binding, struct helios_producer_snapshot *out)
{
   struct helios_producer_binding *b = binding;
   if (!b || !out) return helios_producer_error();
   const struct helios_producer_status *s = b->status;
   for (unsigned i = 0; i < 8; i++) {
      /* Loads only: InterlockedCompareExchange would WRITE the read-only MDL. */
      const uint64_t before = __atomic_load_n(&s->sequence, __ATOMIC_ACQUIRE);
      if (before & 1) continue;
      struct helios_producer_snapshot snap = {
         .generation = __atomic_load_n(&s->generation, __ATOMIC_RELAXED),
         .announced = __atomic_load_n(&s->announced, __ATOMIC_RELAXED),
         .completed = __atomic_load_n(&s->completed, __ATOMIC_RELAXED),
         .status = __atomic_load_n(&s->status, __ATOMIC_RELAXED),
      };
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      if (__atomic_load_n(&s->sequence, __ATOMIC_ACQUIRE) != before) continue;
      *out = snap;
      if (snap.generation != b->generation || snap.status != 0)
         return helios_producer_error();
      return VK_SUCCESS;
   }
   return VK_NOT_READY; /* Contention is never an unpublished/ready snapshot. */
}

static int32_t
helios_producer_publish(void *binding, uint64_t semaphore, uint64_t value, uint64_t *epoch)
{
   struct helios_producer_binding *b = binding;
   if (!b || !semaphore || !value || value > UINT32_MAX || !epoch)
      return helios_producer_error();
   struct vn_semaphore *sem = vn_semaphore_from_handle((VkSemaphore)(uintptr_t)semaphore);
   if (sem->base.vk.device != &b->device->base.vk || !sem->helios_present_stream_cookie)
      return helios_producer_error();
   struct helios_producer_control r = helios_producer_request(HP_PUBLISH, b->token);
   r.ctx_id = b->renderer->ctx_id;
   r.stream_cookie = sem->helios_present_stream_cookie;
   r.value = value;
   if (!helios_escape(b->renderer, &r, sizeof(r)) || !r.epoch)
      return helios_producer_error();
   *epoch = r.epoch;
   return VK_SUCCESS;
}

static int32_t
helios_producer_wait(void *binding, uint64_t epoch, uint64_t timeout_ns, uintptr_t cancel_event)
{
   struct helios_producer_binding *b = binding;
   if (!b) return helios_producer_error();
   struct helios_producer_snapshot snap;
   int32_t result = helios_producer_status(b, &snap);
   if (result < 0) return result;
   if (result == VK_SUCCESS && epoch <= snap.completed) return VK_SUCCESS;
   /* Serialize use of this binding's reusable event, never dev_mutex. Each
    * command-list dependency retains the binding throughout this operation. */
   const uint64_t start = GetTickCount64();
   const uint64_t budget_ms = timeout_ns == UINT64_MAX ? UINT64_MAX :
      timeout_ns / 1000000 + !!(timeout_ns % 1000000);
   // Include contention on the reusable event in this call's timeout/cancel
   // budget. A second waiter must not get stuck behind an infinite first wait.
   while (mtx_trylock(&b->wait_mutex) != thrd_success) {
      if (cancel_event && WaitForSingleObject((HANDLE)cancel_event, 0) != WAIT_TIMEOUT)
         return helios_producer_error();
      if (GetTickCount64() - start >= budget_ms) return VK_TIMEOUT;
      Sleep(1);
   }
   ResetEvent(b->event);
   struct helios_producer_control r = helios_producer_request(HP_WAIT, b->token);
   r.epoch = epoch; r.event = (uintptr_t)b->event;
   if (!helios_escape(b->renderer, &r, sizeof(r))) {
      result = helios_producer_error();
   } else if (r.state == 0) {
      result = VK_SUCCESS;
   } else if (r.state == 1) {
      const uint64_t elapsed_ms = GetTickCount64() - start;
      const uint64_t ms = elapsed_ms >= budget_ms ? 0 : budget_ms - elapsed_ms;
      const DWORD timeout = timeout_ns == UINT64_MAX ? INFINITE : (DWORD)MIN2(ms, (uint64_t)INFINITE - 1);
      HANDLE events[2] = { b->event, (HANDLE)cancel_event };
      const DWORD waited = WaitForMultipleObjects(cancel_event ? 2 : 1, events, FALSE, timeout);
      r.op = HP_CANCEL;
      const bool cancelled = helios_escape(b->renderer, &r, sizeof(r));
      if (!cancelled || waited == WAIT_FAILED || waited == WAIT_OBJECT_0 + 1) {
         result = helios_producer_error();
      } else {
         result = helios_producer_status(b, &snap);
         if (result == VK_SUCCESS)
            result = epoch <= snap.completed ? VK_SUCCESS : VK_TIMEOUT;
         if (result == VK_NOT_READY) result = VK_TIMEOUT;
      }
   } else {
      result = helios_producer_error();
   }
   mtx_unlock(&b->wait_mutex);
   return result;
}

static void helios_producer_retain(void *binding)
{
   if (binding) InterlockedIncrement(&((struct helios_producer_binding *)binding)->references);
}

static void helios_producer_release(void *binding)
{
   struct helios_producer_binding *b = binding;
   if (!b || InterlockedDecrement(&b->references)) return;
   struct helios_producer_control r = helios_producer_request(HP_RELEASE, b->token);
   if (!helios_escape(b->renderer, &r, sizeof(r))) helios_producer_error();
   mtx_destroy(&b->wait_mutex); CloseHandle(b->event); free(b);
}

static void helios_producer_abort(void *binding)
{
   struct helios_producer_binding *b = binding;
   if (!b) return;
   struct helios_producer_control r = helios_producer_request(HP_ABORT, b->token);
   if (!helios_escape(b->renderer, &r, sizeof(r))) helios_producer_error();
}

__declspec(dllexport) int32_t
helios_venus_producer_interface(uint32_t version, struct helios_producer_api_v1 *api);

__declspec(dllexport) int32_t
helios_venus_producer_interface(uint32_t version, struct helios_producer_api_v1 *api)
{
   if (!api || version != HELIOS_PRODUCER_ABI) return VK_ERROR_INCOMPATIBLE_DRIVER;
   *api = (struct helios_producer_api_v1) {
      .version = HELIOS_PRODUCER_ABI, .size = sizeof(*api),
      .stream = helios_producer_stream,
      .bind = helios_producer_bind, .publish = helios_producer_publish,
      .status = helios_producer_status, .wait = helios_producer_wait,
      .retain = helios_producer_retain, .release = helios_producer_release,
      .abort = helios_producer_abort,
   };
   return VK_SUCCESS;
}
