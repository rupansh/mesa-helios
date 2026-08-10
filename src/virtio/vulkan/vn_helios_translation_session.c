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

#include "vn_helios_hwa2.h"       /* vn_renderer_helios_diag_log */
#include "vn_helios_native_kmt.h" /* A2: the HVC1 context and HNR2 encoder */

/* Teardown ignores every status by design -- there is nothing left to do about
 * a failed destroy, and the handles are dropped either way. `(void)` does not
 * suppress warn_unused_result, so route them through here. */
#define HELIOS_IGNORE_STATUS(call)                                             \
   do {                                                                        \
      NTSTATUS ignored_ = (call);                                              \
      (void)ignored_;                                                          \
   } while (0)

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

   /* The one HVC1 control context: A2 owns its Render/fence/buffer state. */
   struct helios_native_context *control;

   /* Role-1 reply pool: one allocation, resident, Lock2-mapped for life. */
   D3DKMT_HANDLE pool;
   void *pool_cpu;
   uint64_t pool_generation; /* HVM1 object_generation the KMD returned */
   D3DKMT_HANDLE paging_queue;
   D3DKMT_HANDLE paging_fence;

   struct helios_reply_slot slots[HELIOS_HVM1_REPLY_SLOT_COUNT];
   uint64_t next_slot_generation;

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

   if (payload_bytes == 0 || payload_bytes > HELIOS_HNR2_MAX_PAYLOAD_BYTES) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_PAYLOAD_TOO_LARGE,
                            "control payload is empty or above the batch bound",
                            0);
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

   /* The only allocation this carrier ever lists: its own checked-out slot,
    * writable, because §10.4 makes the reply target the sole legal write of the
    * pure-control class. */
   const struct helios_hnr2_allocation slot_use = {
      .handle = s->pool,
      .access = HELIOS_HNR2_ACCESS_WRITE,
      .expected_generation = s->pool_generation,
   };
   struct helios_hnr2_batch batch;
   memset(&batch, 0, sizeof(batch));
   batch.payload = payload;
   batch.payload_bytes = payload_bytes;
   batch.allocations = &slot_use;
   batch.allocation_count = 1;
   batch.has_reply = true;
   batch.reply_allocation_index = 0;
   batch.reply_offset = s->slots[slot_index].offset;
   batch.reply_capacity_bytes = reply_capacity_bytes;
   batch.reply_slot_generation = slot_generation;

   uint64_t batch_token = 0;
   uint64_t progress = 0;
   VkResult vr = helios_native_context_submit(s->control, &batch, true,
                                              &batch_token, &progress);
   if (vr != VK_SUCCESS) {
      /* §10.7: a failed Render discards the assembler and loses the context;
       * A2 has already marked it so, and there is no other transport. */
      helios_session_refuse(render_site, "native submit", (unsigned)vr);
      helios_slot_release(s, slot_index);
      return vr;
   }

   /* C51: one CPU wait on the GPU-side signal A2 enqueued behind the Render.
    * Never a shared-memory sample. */
   vr = helios_native_context_wait(s->control, progress,
                                   HELIOS_NATIVE_WAIT_INFINITE);
   if (vr != VK_SUCCESS) {
      helios_session_refuse(wait_site, "C51 wait", (unsigned)vr);
      helios_slot_release(s, slot_index);
      return vr;
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
   if (r.slot_generation != slot_generation || r.batch_token != batch_token) {
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
   /* First, because destroying the context JOINS its outstanding C51 milestone
    * (§10.7 "drains/cancels every HNR2 use" before the allocation goes): the
    * only writer of the reply pool is a Render on this context. */
   if (s->control) {
      helios_native_context_destroy(s->control);
      s->control = NULL;
   }
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

   /* The one HVC1 control context: A2 builds it, validates the DXGK_CONTEXTINFO
    * minima, and owns its monitored progress fence. */
   {
      const VkResult ctx = helios_native_context_create(
         s->device, HELIOS_NATIVE_CONTEXT_CONTROL, 0, 0, &s->control);
      if (ctx != VK_SUCCESS) {
         helios_session_refuse(HELIOS_SESSION_REFUSE_CONTEXT,
                               "native control context", (unsigned)ctx);
         goto fail;
      }
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

   /* CreateAllocation copies the per-allocation private data back, and the
    * generation it returns is what every use record naming this pool must
    * carry: a zero one is `Hnr2TableReject::AllocationGenerationZero` and would
    * be refused on the wire, so refuse here where the reason is visible. */
   s->pool_generation = hvm1.object_generation;
   if (!s->pool_generation) {
      helios_session_refuse(HELIOS_SESSION_REFUSE_POOL_CREATE,
                            "reply pool came back with object_generation=0", 0);
      goto fail;
   }

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
