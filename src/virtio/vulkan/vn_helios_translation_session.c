/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A1 -- the HTS1 translation session. See vn_helios_translation_session.h.
 */

#ifdef _WIN32

#include "vn_helios_translation_session.h"

#include <stdio.h>
#include <string.h>

#include "vn_helios_hwa2.h" /* vn_renderer_helios_diag_log */

/* Teardown ignores every status by design -- there is nothing left to do about
 * a failed destroy, and the handles are dropped either way. `(void)` does not
 * suppress warn_unused_result, so route them through here. */
#define HELIOS_IGNORE_STATUS(call)                                             \
   do {                                                                        \
      NTSTATUS ignored_ = (call);                                              \
      (void)ignored_;                                                          \
   } while (0)

/* ── CRC-64/ECMA-182 ────────────────────────────────────────────────────────
 *
 * ⛔ HAND-MIRRORED from protocol/src/wddm.rs::crc64_ecma_update. Nothing links
 * the two, so helios_crc64_self_test() checks the published check value at
 * session create and refuses rather than shipping a silent transcription slip.
 */
static uint64_t helios_crc64_table[256];
static bool helios_crc64_ready;

static void
helios_crc64_init(void)
{
   if (helios_crc64_ready)
      return;
   for (unsigned n = 0; n < 256; n++) {
      uint64_t c = (uint64_t)n << 56;
      for (unsigned bit = 0; bit < 8; bit++)
         c = (c & 0x8000000000000000ull) ? ((c << 1) ^ HELIOS_CRC64_ECMA182_POLY)
                                         : (c << 1);
      helios_crc64_table[n] = c;
   }
   helios_crc64_ready = true;
}

static uint64_t
helios_crc64(const void *bytes, size_t len)
{
   const uint8_t *p = (const uint8_t *)bytes;
   uint64_t crc = HELIOS_CRC64_ECMA182_INIT;
   helios_crc64_init();
   for (size_t i = 0; i < len; i++)
      crc = helios_crc64_table[(uint8_t)((crc >> 56) ^ p[i])] ^ (crc << 8);
   return crc ^ HELIOS_CRC64_ECMA182_XOROUT;
}

static bool
helios_crc64_self_test(void)
{
   return helios_crc64("123456789", 9) == HELIOS_CRC64_ECMA182_CHECK;
}

/* ── refusal census ─────────────────────────────────────────────────────────*/

static const char *const helios_session_site_names[HELIOS_SESSION_REFUSE_SITE_COUNT] = {
   [HELIOS_SESSION_REFUSE_ADAPTER] = "adapter",
   [HELIOS_SESSION_REFUSE_DEVICE] = "device",
   [HELIOS_SESSION_REFUSE_CONTEXT] = "control_context",
   [HELIOS_SESSION_REFUSE_CONTEXT_MINIMA] = "context_minima",
   [HELIOS_SESSION_REFUSE_POOL_CREATE] = "pool_create",
   [HELIOS_SESSION_REFUSE_POOL_RESIDENT] = "pool_resident",
   [HELIOS_SESSION_REFUSE_POOL_LOCK] = "pool_lock",
   [HELIOS_SESSION_REFUSE_FENCE] = "fence",
   [HELIOS_SESSION_REFUSE_INIT_RENDER] = "init_render",
   [HELIOS_SESSION_REFUSE_INIT_WAIT] = "init_wait",
   [HELIOS_SESSION_REFUSE_INIT_REPLY] = "init_reply",
   [HELIOS_SESSION_REFUSE_SLOT_EXHAUSTED] = "slot_exhausted",
   [HELIOS_SESSION_REFUSE_CONTROL_RENDER] = "control_render",
   [HELIOS_SESSION_REFUSE_CONTROL_WAIT] = "control_wait",
   [HELIOS_SESSION_REFUSE_CONTROL_REPLY] = "control_reply",
   [HELIOS_SESSION_REFUSE_PAYLOAD_TOO_LARGE] = "payload_too_large",
};

static volatile LONG helios_session_counters[HELIOS_SESSION_REFUSE_SITE_COUNT];

static void
helios_session_refuse(enum helios_session_refusal_site site, const char *detail,
                      unsigned status)
{
   if ((unsigned)site >= HELIOS_SESSION_REFUSE_SITE_COUNT)
      return;
   const long n = InterlockedIncrement(&helios_session_counters[site]);
   /* 1-then-every-256: a caller that retries per frame must not make the diag
    * log itself the failure. */
   if (n == 1 || (n % 256) == 0)
      vn_renderer_helios_diag_log(
         "HTS1 session REFUSED at %s count=%ld status=0x%08x: %s",
         helios_session_site_names[site], n, status, detail ? detail : "");
}

long
helios_session_refusals(enum helios_session_refusal_site site)
{
   if ((unsigned)site >= HELIOS_SESSION_REFUSE_SITE_COUNT)
      return 0;
   return helios_session_counters[site];
}

void
helios_session_refusal_summary(char *out, size_t out_bytes)
{
   size_t used = 0;
   if (!out || out_bytes == 0)
      return;
   out[0] = '\0';
   for (unsigned s = 0; s < HELIOS_SESSION_REFUSE_SITE_COUNT; s++) {
      const long n = helios_session_counters[s];
      if (!n)
         continue;
      const int w = snprintf(out + used, out_bytes - used, "%s%s=%ld",
                             used ? " " : "", helios_session_site_names[s], n);
      if (w <= 0 || (size_t)w >= out_bytes - used)
         return;
      used += (size_t)w;
   }
}

/* ── the session object ─────────────────────────────────────────────────────*/

struct helios_reply_slot {
   uint64_t generation; /* nonzero, strictly increasing; 0 = never checked out */
   uint64_t offset;     /* byte offset of this slot inside the pool */
   bool in_use;
};

struct helios_translation_session {
   D3DKMT_HANDLE adapter;
   D3DKMT_HANDLE device;
   D3DKMT_HANDLE context;

   /* Buffers dxgkrnl hands back from CreateContext/Render. Re-adopted after
    * EVERY Render, per §10.7 ("adopts the returned next command/allocation
    * buffers and their actual sizes before recording again"). */
   void *command_buffer;
   uint32_t command_buffer_bytes;
   D3DDDI_ALLOCATIONLIST *allocation_list;
   uint32_t allocation_list_entries;
   D3DDDI_PATCHLOCATIONLIST *patch_list;
   uint32_t patch_list_entries;

   /* Role-1 reply pool: one allocation, resident, Lock2-mapped for life. */
   D3DKMT_HANDLE pool;
   void *pool_cpu;
   D3DKMT_HANDLE paging_queue;
   D3DKMT_HANDLE paging_fence;

   /* C51 completion for the control context. */
   D3DKMT_HANDLE fence;
   uint64_t fence_value;

   struct helios_reply_slot slots[HELIOS_HVM1_REPLY_SLOT_COUNT];
   uint64_t next_slot_generation;
   uint64_t next_batch_token;

   CRITICAL_SECTION lock;

   uint64_t session_generation;
   uint64_t capability_low;
   uint64_t capability_high;
   uint32_t endpoint_capacity;
   uint32_t capset;
};

/* ── slot checkout ──────────────────────────────────────────────────────────*/

static bool
helios_slot_acquire(struct helios_translation_session *s, unsigned *out_index,
                    uint64_t *out_generation)
{
   bool got = false;
   EnterCriticalSection(&s->lock);
   for (unsigned i = 0; i < HELIOS_HVM1_REPLY_SLOT_COUNT; i++) {
      if (s->slots[i].in_use)
         continue;
      s->slots[i].in_use = true;
      s->slots[i].generation = s->next_slot_generation++;
      *out_index = i;
      *out_generation = s->slots[i].generation;
      got = true;
      break;
   }
   LeaveCriticalSection(&s->lock);
   return got;
}

static void
helios_slot_release(struct helios_translation_session *s, unsigned index)
{
   EnterCriticalSection(&s->lock);
   if (index < HELIOS_HVM1_REPLY_SLOT_COUNT)
      s->slots[index].in_use = false;
   LeaveCriticalSection(&s->lock);
}

/* ── the finite HNR2 control transaction ────────────────────────────────────
 *
 * One fragment, BEGIN|COMMIT|HAS_REPLY, one use record naming the checked-out
 * reply slot writable, zero patch records. ⛔ This carrier may never name an
 * outer D3D allocation (§10.4): the slot is the only entry it ever lists.
 */
static VkResult
helios_transact(struct helios_translation_session *s, const void *payload,
                uint64_t payload_bytes, void *reply, uint64_t reply_capacity,
                uint64_t *reply_bytes, int32_t *reply_status,
                uint64_t *snapshot_generation, bool *more,
                enum helios_session_refusal_site render_site,
                enum helios_session_refusal_site wait_site,
                enum helios_session_refusal_site reply_site)
{
   if (reply_bytes)
      *reply_bytes = 0;
   if (more)
      *more = false;

   const uint64_t max_chunk =
      reply_capacity > HELIOS_HVR1_MAX_CHUNK_BYTES ? HELIOS_HVR1_MAX_CHUNK_BYTES
                                                   : reply_capacity;
   const uint64_t reply_capacity_bytes = HELIOS_HVR1_HEADER_SIZE + max_chunk;

   /* Reserve COMMIT metadata BEFORE choosing the payload length (§10.7). */
   const uint64_t metadata = HELIOS_HNR2_HEADER_SIZE + HELIOS_HNR2_USE_RECORD_SIZE;
   if (payload_bytes == 0 || payload_bytes > HELIOS_HNR2_MAX_PAYLOAD_BYTES ||
       metadata + payload_bytes > (uint64_t)s->command_buffer_bytes) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_PAYLOAD_TOO_LARGE,
                            "control payload does not fit one fragment", 0);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   if (reply_capacity_bytes > HELIOS_HVM1_REPLY_SLOT_BYTES) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_PAYLOAD_TOO_LARGE,
                            "reply capacity exceeds one slot", 0);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   unsigned slot_index = 0;
   uint64_t slot_generation = 0;
   if (!helios_slot_acquire(s, &slot_index, &slot_generation)) {
      /* §10.7: exhaustion returns resource failure and never waits or spills. */
      helios_session_refuse(HELIOS_SESSION_REFUSE_SLOT_EXHAUSTED,
                            "all four reply slots outstanding", 0);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   VkResult result = VK_ERROR_INITIALIZATION_FAILED;
   uint8_t *cmd = (uint8_t *)s->command_buffer;
   const uint32_t use_offset = (uint32_t)(HELIOS_HNR2_HEADER_SIZE + payload_bytes);
   const uint32_t use_offset_aligned = (use_offset + 7u) & ~7u;
   const uint32_t total = use_offset_aligned + HELIOS_HNR2_USE_RECORD_SIZE;

   memset(cmd, 0, total);
   memcpy(cmd + HELIOS_HNR2_HEADER_SIZE, payload, (size_t)payload_bytes);

   /* The one COMMIT use record: allocation-list index 0, WRITE. */
   {
      uint8_t *u = cmd + use_offset_aligned;
      const uint32_t index = 0, access = HELIOS_HNR2_ACCESS_WRITE;
      const uint64_t expected_generation = 0; /* the KMD's own pool allocation */
      const uint32_t first_patch = 0, patch_count = 0;
      memcpy(u + 0, &index, 4);
      memcpy(u + 4, &access, 4);
      memcpy(u + 8, &expected_generation, 8);
      memcpy(u + 16, &first_patch, 4);
      memcpy(u + 20, &patch_count, 4);
   }

   HeliosNativeRenderV2 h;
   memset(&h, 0, sizeof(h));
   h.magic = HELIOS_HNR2_MAGIC;
   h.abi_version = HELIOS_HNR2_ABI_VERSION;
   h.header_size = HELIOS_HNR2_HEADER_SIZE;
   h.package_generation = HELIOS_PACKAGE_GENERATION;
   h.batch_token = ++s->next_batch_token;
   h.total_payload_bytes = payload_bytes;
   h.fragment_payload_offset = 0;
   h.fragment_payload_bytes = (uint32_t)payload_bytes;
   h.fragment_index = 0;
   h.fragment_count = 1;
   h.use_record_offset = use_offset_aligned;
   h.use_record_count = 1;
   h.patch_record_offset = 0;
   h.patch_record_count = 0;
   h.reply_allocation_list_index = 0;
   h.flags = HELIOS_HNR2_FLAG_BEGIN | HELIOS_HNR2_FLAG_COMMIT |
             HELIOS_HNR2_FLAG_HAS_REPLY;
   h.reply_offset = s->slots[slot_index].offset;
   h.reply_capacity_bytes = reply_capacity_bytes;
   h.reply_slot_generation = slot_generation;
   h.full_payload_crc64 = helios_crc64(payload, (size_t)payload_bytes);
   memcpy(cmd, &h, sizeof(h));
   /* Fragment CRC covers the emitted fragment with this field zero. */
   {
      const uint64_t zero = 0;
      memcpy(cmd + 88, &zero, 8);
      const uint64_t frag = helios_crc64(cmd, total);
      memcpy(cmd + 88, &frag, 8);
   }

   s->allocation_list[0].hAllocation = s->pool;
   s->allocation_list[0].Value = 0;
   s->allocation_list[0].WriteOperation = 1;

   D3DKMT_RENDER render;
   memset(&render, 0, sizeof(render));
   render.hContext = s->context;
   render.CommandOffset = 0;
   render.CommandLength = total;
   render.AllocationCount = 1;
   render.PatchLocationCount = 0;
   render.NewCommandBufferSize = HELIOS_HVC1_DMA_BUFFER_BYTES;
   render.NewAllocationListSize = HELIOS_HVC1_ALLOCATION_LIST_ENTRIES;
   render.NewPatchLocationListSize = HELIOS_HVC1_PATCH_LOCATION_ENTRIES;

   NTSTATUS st = D3DKMTRender(&render);
   if (st != 0) {
      /* §10.7: a failed Render discards the assembler, loses the context, and
       * ignores the returned buffers rather than retrying on another transport. */
      helios_session_refuse(render_site, "D3DKMTRender", (unsigned)st);
      helios_slot_release(s, slot_index);
      return VK_ERROR_DEVICE_LOST;
   }

   s->command_buffer = render.pNewCommandBuffer;
   s->command_buffer_bytes = render.NewCommandBufferSize;
   s->allocation_list = render.pNewAllocationList;
   s->allocation_list_entries = render.NewAllocationListSize;
   s->patch_list = render.pNewPatchLocationList;
   s->patch_list_entries = render.NewPatchLocationListSize;

   /* C51: enqueue the GPU-side signal behind the Render we just queued, then
    * one event-backed CPU wait. Never a shared-memory sample. */
   const uint64_t target = ++s->fence_value;
   D3DKMT_SIGNALSYNCHRONIZATIONOBJECTFROMGPU sig;
   memset(&sig, 0, sizeof(sig));
   sig.hContext = s->context;
   sig.ObjectCount = 1;
   sig.ObjectHandleArray = &s->fence;
   sig.MonitoredFenceValueArray = &target;
   st = D3DKMTSignalSynchronizationObjectFromGpu(&sig);
   if (st != 0) {
      helios_session_refuse(wait_site, "SignalSynchronizationObjectFromGpu",
                            (unsigned)st);
      helios_slot_release(s, slot_index);
      return VK_ERROR_DEVICE_LOST;
   }

   D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU wait;
   memset(&wait, 0, sizeof(wait));
   wait.hDevice = s->device;
   wait.ObjectCount = 1;
   wait.ObjectHandleArray = &s->fence;
   wait.FenceValueArray = &target;
   wait.hAsyncEvent = NULL; /* synchronous: the call itself blocks */
   st = D3DKMTWaitForSynchronizationObjectFromCpu(&wait);
   if (st != 0) {
      helios_session_refuse(wait_site, "WaitForSynchronizationObjectFromCpu",
                            (unsigned)st);
      helios_slot_release(s, slot_index);
      return VK_ERROR_DEVICE_LOST;
   }

   /* ── read the HVR1 the KMD published into our slot ───────────────────── */
   const uint8_t *slot = (const uint8_t *)s->pool_cpu + s->slots[slot_index].offset;
   HeliosVenusReplyV1 r;
   memcpy(&r, slot, sizeof(r));

   char why[192];
   if (r.magic != HELIOS_HVR1_MAGIC || r.version != HELIOS_HVR1_VERSION ||
       r.header_size != HELIOS_HVR1_HEADER_SIZE) {
      snprintf(why, sizeof(why), "header magic=0x%08x version=%u size=%u",
               (unsigned)r.magic, (unsigned)r.version, (unsigned)r.header_size);
      goto reply_refused;
   }
   if (r.package_generation != HELIOS_PACKAGE_GENERATION) {
      snprintf(why, sizeof(why), "package generation mismatch");
      goto reply_refused;
   }
   if (r.slot_generation != slot_generation || r.batch_token != h.batch_token) {
      /* A stale slot answer is the one thing this generation pair exists to
       * catch; never accept it as this transaction's result. */
      snprintf(why, sizeof(why), "stale reply: slot gen/batch token mismatch");
      goto reply_refused;
   }
   if (s->session_generation && r.session_generation != s->session_generation) {
      snprintf(why, sizeof(why), "session generation mismatch");
      goto reply_refused;
   }
   if ((r.flags & ~HELIOS_HVR1_FLAG_MASK) != 0 ||
       (r.flags & HELIOS_HVR1_FLAG_MASK) == 0 ||
       (r.flags & HELIOS_HVR1_FLAG_MASK) == HELIOS_HVR1_FLAG_MASK) {
      snprintf(why, sizeof(why), "flags=0x%x is not exactly one of MORE/FINAL",
               (unsigned)r.flags);
      goto reply_refused;
   }
   if ((uint64_t)r.chunk_bytes > max_chunk ||
       (uint64_t)r.chunk_bytes > r.total_bytes) {
      snprintf(why, sizeof(why), "chunk_bytes=%u exceeds capacity or total",
               (unsigned)r.chunk_bytes);
      goto reply_refused;
   }

   if (r.chunk_bytes && reply)
      memcpy(reply, slot + HELIOS_HVR1_HEADER_SIZE, r.chunk_bytes);
   if (reply_bytes)
      *reply_bytes = r.chunk_bytes;
   if (reply_status)
      *reply_status = r.status;
   if (snapshot_generation)
      *snapshot_generation = r.snapshot_generation;
   if (more)
      *more = (r.flags & HELIOS_HVR1_FLAG_MORE) != 0;
   result = VK_SUCCESS;
   helios_slot_release(s, slot_index);
   return result;

reply_refused:
   helios_session_refuse(reply_site, why, 0);
   helios_slot_release(s, slot_index);
   return VK_ERROR_DEVICE_LOST;
}

VkResult
helios_session_control(struct helios_translation_session *s, const void *payload,
                       uint64_t payload_bytes, void *reply,
                       uint64_t reply_capacity, uint64_t *reply_bytes,
                       int32_t *reply_status, uint64_t *snapshot_generation,
                       bool *more)
{
   if (!s || !payload)
      return VK_ERROR_INITIALIZATION_FAILED;
   return helios_transact(s, payload, payload_bytes, reply, reply_capacity,
                          reply_bytes, reply_status, snapshot_generation, more,
                          HELIOS_SESSION_REFUSE_CONTROL_RENDER,
                          HELIOS_SESSION_REFUSE_CONTROL_WAIT,
                          HELIOS_SESSION_REFUSE_CONTROL_REPLY);
}

VkResult
helios_session_control_continue(struct helios_translation_session *s,
                                uint64_t snapshot_generation,
                                uint64_t expected_offset, void *reply,
                                uint64_t reply_capacity, uint64_t *reply_bytes,
                                int32_t *reply_status, bool *more)
{
   if (!s || !snapshot_generation)
      return VK_ERROR_INITIALIZATION_FAILED;

   const uint64_t max_chunk =
      reply_capacity > HELIOS_HVR1_MAX_CHUNK_BYTES ? HELIOS_HVR1_MAX_CHUNK_BYTES
                                                   : reply_capacity;
   HeliosVenusReplyContinuationV1 cont;
   memset(&cont, 0, sizeof(cont));
   cont.package_generation = HELIOS_PACKAGE_GENERATION;
   cont.session_generation = s->session_generation;
   cont.snapshot_generation = snapshot_generation;
   cont.expected_offset = expected_offset;
   cont.max_chunk_bytes = (uint32_t)max_chunk;

   uint64_t got_snapshot = 0;
   return helios_transact(s, &cont, sizeof(cont), reply, reply_capacity,
                          reply_bytes, reply_status, &got_snapshot, more,
                          HELIOS_SESSION_REFUSE_CONTROL_RENDER,
                          HELIOS_SESSION_REFUSE_CONTROL_WAIT,
                          HELIOS_SESSION_REFUSE_CONTROL_REPLY);
}

/* ── create / destroy ───────────────────────────────────────────────────────*/

static void
helios_session_teardown(struct helios_translation_session *s)
{
   if (s->pool_cpu) {
      D3DKMT_UNLOCK2 u = { .hDevice = s->device, .hAllocation = s->pool };
      HELIOS_IGNORE_STATUS(D3DKMTUnlock2(&u));
      s->pool_cpu = NULL;
   }
   if (s->pool) {
      D3DKMT_DESTROYALLOCATION2 d;
      memset(&d, 0, sizeof(d));
      d.hDevice = s->device;
      d.hResource = 0;
      d.phAllocationList = &s->pool;
      d.AllocationCount = 1;
      HELIOS_IGNORE_STATUS(D3DKMTDestroyAllocation2(&d));
      s->pool = 0;
   }
   if (s->paging_queue) {
      D3DDDI_DESTROYPAGINGQUEUE dq = { .hPagingQueue = s->paging_queue };
      HELIOS_IGNORE_STATUS(D3DKMTDestroyPagingQueue(&dq));
      s->paging_queue = 0;
   }
   if (s->fence) {
      D3DKMT_DESTROYSYNCHRONIZATIONOBJECT ds = { .hSyncObject = s->fence };
      HELIOS_IGNORE_STATUS(D3DKMTDestroySynchronizationObject(&ds));
      s->fence = 0;
   }
   if (s->context) {
      D3DKMT_DESTROYCONTEXT dc = { .hContext = s->context };
      HELIOS_IGNORE_STATUS(D3DKMTDestroyContext(&dc));
      s->context = 0;
   }
   if (s->device) {
      D3DKMT_DESTROYDEVICE dd;
      memset(&dd, 0, sizeof(dd));
      dd.hDevice = s->device;
      HELIOS_IGNORE_STATUS(D3DKMTDestroyDevice(&dd));
      s->device = 0;
   }
   if (s->adapter) {
      D3DKMT_CLOSEADAPTER ca = { .hAdapter = s->adapter };
      HELIOS_IGNORE_STATUS(D3DKMTCloseAdapter(&ca));
      s->adapter = 0;
   }
}

void
helios_translation_session_destroy(struct helios_translation_session *s)
{
   if (!s)
      return;
   helios_session_teardown(s);
   DeleteCriticalSection(&s->lock);
   free(s);
}

VkResult
helios_translation_session_create(LUID adapter_luid,
                                  uint32_t requested_endpoint_capacity,
                                  struct helios_translation_session **out)
{
   if (!out)
      return VK_ERROR_INITIALIZATION_FAILED;
   *out = NULL;

   if (!helios_crc64_self_test()) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_INIT_REPLY,
                            "CRC-64/ECMA-182 transcription failed its check value",
                            0);
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   if (requested_endpoint_capacity == 0 ||
       requested_endpoint_capacity > HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION)
      return VK_ERROR_INITIALIZATION_FAILED;

   struct helios_translation_session *s = calloc(1, sizeof(*s));
   if (!s)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   InitializeCriticalSection(&s->lock);
   s->next_slot_generation = 1;

   NTSTATUS st;

   D3DKMT_OPENADAPTERFROMLUID open = { .AdapterLuid = adapter_luid };
   st = D3DKMTOpenAdapterFromLuid(&open);
   if (st != 0) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_ADAPTER, "OpenAdapterFromLuid",
                            (unsigned)st);
      goto fail;
   }
   s->adapter = open.hAdapter;

   D3DKMT_CREATEDEVICE cd;
   memset(&cd, 0, sizeof(cd));
   cd.hAdapter = s->adapter;
   st = D3DKMTCreateDevice(&cd);
   if (st != 0) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_DEVICE, "CreateDevice",
                            (unsigned)st);
      goto fail;
   }
   s->device = cd.hDevice;

   /* The one HVC1 control context: both ordinals UINT32_MAX mean control. */
   HeliosVulkanContextV1 hvc1;
   memset(&hvc1, 0, sizeof(hvc1));
   hvc1.magic = HELIOS_HVC1_MAGIC;
   hvc1.abi_version = HELIOS_HVC1_ABI_VERSION;
   hvc1.struct_size = HELIOS_HVC1_SIZE;
   hvc1.package_generation = HELIOS_PACKAGE_GENERATION;
   hvc1.capset = HELIOS_NATIVE_RENDER_CAPSET;
   hvc1.mode = HELIOS_HVC1_MODE_FINITE_HNR2_RENDER;
   hvc1.queue_family = HELIOS_HVC1_CONTROL_ORDINAL;
   hvc1.queue_index = HELIOS_HVC1_CONTROL_ORDINAL;

   D3DKMT_CREATECONTEXT cc;
   memset(&cc, 0, sizeof(cc));
   cc.hDevice = s->device;
   cc.NodeOrdinal = HELIOS_HVC1_NODE_ORDINAL;
   cc.EngineAffinity = HELIOS_HVC1_ENGINE_AFFINITY;
   cc.ClientHint = D3DKMT_CLIENTHINT_VULKAN;
   cc.pPrivateDriverData = &hvc1;
   cc.PrivateDriverDataSize = sizeof(hvc1);
   st = D3DKMTCreateContext(&cc);
   if (st != 0) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_CONTEXT, "CreateContext",
                            (unsigned)st);
      goto fail;
   }
   s->context = cc.hContext;
   s->command_buffer = cc.pCommandBuffer;
   s->command_buffer_bytes = cc.CommandBufferSize;
   s->allocation_list = cc.pAllocationList;
   s->allocation_list_entries = cc.AllocationListSize;
   s->patch_list = cc.pPatchLocationList;
   s->patch_list_entries = cc.PatchLocationListSize;

   /* §10.7: creation fails if dxgkrnl does not return non-null buffers of at
    * least the advertised minima. Loud, because a short list silently truncates
    * a manifest later. */
   if (!s->command_buffer || s->command_buffer_bytes < HELIOS_HVC1_DMA_BUFFER_BYTES ||
       !s->allocation_list ||
       s->allocation_list_entries < HELIOS_HVC1_ALLOCATION_LIST_ENTRIES ||
       !s->patch_list ||
       s->patch_list_entries < HELIOS_HVC1_PATCH_LOCATION_ENTRIES) {
      char why[192];
      snprintf(why, sizeof(why),
               "cmd=%p/%u alloc=%p/%u patch=%p/%u below the advertised minima",
               s->command_buffer, s->command_buffer_bytes, (void *)s->allocation_list,
               s->allocation_list_entries, (void *)s->patch_list,
               s->patch_list_entries);
      helios_session_refuse(HELIOS_SESSION_REFUSE_CONTEXT_MINIMA, why, 0);
      goto fail;
   }

   /* The role-1 reply pool. pSystemMem=NULL: the KMD owns the backing. */
   HeliosVenusMemoryAllocationV1 hvm1;
   memset(&hvm1, 0, sizeof(hvm1));
   hvm1.magic = HELIOS_HVM1_MAGIC;
   hvm1.abi_version = HELIOS_HVM1_ABI_VERSION;
   hvm1.struct_size = HELIOS_HVM1_SIZE;
   hvm1.package_generation = HELIOS_PACKAGE_GENERATION;
   hvm1.object_generation = 0; /* zero in, nonzero out */
   hvm1.byte_size = HELIOS_HVM1_REPLY_POOL_BYTES;
   hvm1.role = HELIOS_HVM1_ROLE_REPLY_POOL;
   hvm1.access = HELIOS_HVM1_ACCESS_CPU_READ | HELIOS_HVM1_ACCESS_HOST_WRITE;
   hvm1.cache_policy = HELIOS_HVM1_CACHE_WRITE_COMBINED;

   D3DDDI_ALLOCATIONINFO2 info;
   memset(&info, 0, sizeof(info));
   info.pSystemMem = NULL;
   info.pPrivateDriverData = &hvm1;
   info.PrivateDriverDataSize = sizeof(hvm1);

   D3DKMT_CREATEALLOCATION ca2;
   memset(&ca2, 0, sizeof(ca2));
   ca2.hDevice = s->device;
   ca2.NumAllocations = 1;
   ca2.pAllocationInfo2 = &info;
   st = D3DKMTCreateAllocation2(&ca2);
   if (st != 0) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_POOL_CREATE,
                            "CreateAllocation2(role-1 reply pool)", (unsigned)st);
      goto fail;
   }
   s->pool = info.hAllocation;

   D3DKMT_CREATEPAGINGQUEUE pq;
   memset(&pq, 0, sizeof(pq));
   pq.hDevice = s->device;
   pq.Priority = D3DDDI_PAGINGQUEUE_PRIORITY_NORMAL;
   st = D3DKMTCreatePagingQueue(&pq);
   if (st != 0) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_POOL_RESIDENT,
                            "CreatePagingQueue", (unsigned)st);
      goto fail;
   }
   s->paging_queue = pq.hPagingQueue;
   s->paging_fence = pq.hSyncObject;

   D3DDDI_MAKERESIDENT mr;
   memset(&mr, 0, sizeof(mr));
   mr.hPagingQueue = s->paging_queue;
   mr.NumAllocations = 1;
   mr.AllocationList = &s->pool;
   mr.PriorityList = NULL;
   st = D3DKMTMakeResident(&mr);
   if (st != 0 && st != (NTSTATUS)0x00000103L /* STATUS_PENDING */) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_POOL_RESIDENT, "MakeResident",
                            (unsigned)st);
      goto fail;
   }
   if (mr.PagingFenceValue) {
      /* Residency is not complete until the paging fence reaches this value;
       * INIT lists the pool writable, so waiting here is mandatory. */
      const uint64_t v = mr.PagingFenceValue;
      D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU pw;
      memset(&pw, 0, sizeof(pw));
      pw.hDevice = s->device;
      pw.ObjectCount = 1;
      pw.ObjectHandleArray = &s->paging_fence;
      pw.FenceValueArray = &v;
      st = D3DKMTWaitForSynchronizationObjectFromCpu(&pw);
      if (st != 0) {
         helios_session_refuse(HELIOS_SESSION_REFUSE_POOL_RESIDENT,
                               "paging fence wait", (unsigned)st);
         goto fail;
      }
   }

   D3DKMT_LOCK2 lock;
   memset(&lock, 0, sizeof(lock));
   lock.hDevice = s->device;
   lock.hAllocation = s->pool;
   st = D3DKMTLock2(&lock);
   if (st != 0 || !lock.pData) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_POOL_LOCK, "Lock2",
                            (unsigned)st);
      goto fail;
   }
   s->pool_cpu = lock.pData;
   for (unsigned i = 0; i < HELIOS_HVM1_REPLY_SLOT_COUNT; i++)
      s->slots[i].offset = (uint64_t)i * HELIOS_HVM1_REPLY_SLOT_BYTES;

   /* The control context's own unshared monitored fence (C51). */
   D3DKMT_CREATESYNCHRONIZATIONOBJECT2 cs;
   memset(&cs, 0, sizeof(cs));
   cs.hDevice = s->device;
   cs.Info.Type = D3DDDI_MONITORED_FENCE;
   cs.Info.MonitoredFence.InitialFenceValue = 0;
   st = D3DKMTCreateSynchronizationObject2(&cs);
   if (st != 0) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_FENCE,
                            "CreateSynchronizationObject2(monitored)",
                            (unsigned)st);
      goto fail;
   }
   s->fence = cs.hSyncObject;

   /* ── the finite INIT ─────────────────────────────────────────────────── */
   HeliosTranslationSessionInitV1 init;
   memset(&init, 0, sizeof(init));
   init.magic = HELIOS_HTS1_INIT_MAGIC;
   init.abi_version = HELIOS_HTS1_ABI_VERSION;
   init.struct_size = HELIOS_HTS1_INIT_SIZE;
   init.package_generation = HELIOS_PACKAGE_GENERATION;
   init.capset = HELIOS_NATIVE_RENDER_CAPSET;
   init.requested_endpoint_capacity = requested_endpoint_capacity;

   HeliosTranslationSessionReplyV1 reply;
   uint64_t reply_bytes = 0;
   int32_t reply_status = 0;
   bool more = false;
   VkResult r = helios_transact(
      s, &init, sizeof(init), &reply, sizeof(reply), &reply_bytes, &reply_status,
      NULL, &more, HELIOS_SESSION_REFUSE_INIT_RENDER,
      HELIOS_SESSION_REFUSE_INIT_WAIT, HELIOS_SESSION_REFUSE_INIT_REPLY);
   if (r != VK_SUCCESS)
      goto fail;

   {
      char why[192];
      if (reply_bytes != sizeof(reply) || more) {
         snprintf(why, sizeof(why), "INIT reply is %llu bytes (more=%u)",
                  (unsigned long long)reply_bytes, more ? 1u : 0u);
         goto init_refused;
      }
      if (reply.magic != HELIOS_HTS1_REPLY_MAGIC ||
          reply.abi_version != HELIOS_HTS1_ABI_VERSION ||
          reply.struct_size != HELIOS_HTS1_REPLY_SIZE) {
         snprintf(why, sizeof(why), "reply header magic=0x%08x abi=%u size=%u",
                  (unsigned)reply.magic, (unsigned)reply.abi_version,
                  (unsigned)reply.struct_size);
         goto init_refused;
      }
      if (reply.package_generation != HELIOS_PACKAGE_GENERATION) {
         snprintf(why, sizeof(why), "reply package generation mismatch");
         goto init_refused;
      }
      if (reply.session_generation == 0) {
         snprintf(why, sizeof(why), "session generation is zero");
         goto init_refused;
      }
      if (reply.capset != HELIOS_NATIVE_RENDER_CAPSET) {
         snprintf(why, sizeof(why), "host capset %u is not %u",
                  (unsigned)reply.capset, HELIOS_NATIVE_RENDER_CAPSET);
         goto init_refused;
      }
      if (reply.endpoint_capacity == 0 ||
          reply.endpoint_capacity > requested_endpoint_capacity) {
         snprintf(why, sizeof(why), "endpoint capacity %u is not 1..=%u",
                  (unsigned)reply.endpoint_capacity, requested_endpoint_capacity);
         goto init_refused;
      }
      if (reply.reserved != 0) {
         snprintf(why, sizeof(why), "reserved is nonzero");
         goto init_refused;
      }
      if (0) {
      init_refused:
         helios_session_refuse(HELIOS_SESSION_REFUSE_INIT_REPLY, why, 0);
         goto fail;
      }
   }

   s->session_generation = reply.session_generation;
   s->capability_low = reply.capability_low;
   s->capability_high = reply.capability_high;
   s->endpoint_capacity = reply.endpoint_capacity;
   s->capset = reply.capset;

   /* ⛔ The capability is deliberately absent from this line. */
   vn_renderer_helios_diag_log(
      "HTS1 session ready: generation=%llu endpoints=%u capset=%u pool=0x%x",
      (unsigned long long)s->session_generation, s->endpoint_capacity, s->capset,
      (unsigned)s->pool);

   *out = s;
   return VK_SUCCESS;

fail:
   helios_translation_session_destroy(s);
   return VK_ERROR_INITIALIZATION_FAILED;
}

/* ── accessors ──────────────────────────────────────────────────────────────*/

uint64_t
helios_session_generation(const struct helios_translation_session *s)
{
   return s ? s->session_generation : 0;
}

uint32_t
helios_session_endpoint_capacity(const struct helios_translation_session *s)
{
   return s ? s->endpoint_capacity : 0;
}

uint32_t
helios_session_capset(const struct helios_translation_session *s)
{
   return s ? s->capset : 0;
}

void
helios_session_capability(const struct helios_translation_session *s,
                          uint64_t *low, uint64_t *high)
{
   if (low)
      *low = s ? s->capability_low : 0;
   if (high)
      *high = s ? s->capability_high : 0;
}

D3DKMT_HANDLE
helios_session_device_handle(const struct helios_translation_session *s)
{
   return s ? s->device : 0;
}

#endif /* _WIN32 */
