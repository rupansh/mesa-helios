/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * based in part on Mesa's vn_renderer_vtest.c / vn_renderer_virtgpu.c which are:
 * Copyright 2019 Google LLC
 *
 * Helios vn_renderer backend — Windows ICD over the Helios KMD's DeviceIoControl
 * channel (ARCH.md §5, icd/PHASE5_HANDOVER.md). This replaces venus's Linux
 * virtgpu-DRM backend: every vn_renderer op below maps onto one of the six
 * Helios IOCTLs on GUID_DEVINTERFACE_HELIOS. Everything above vn_renderer (the
 * byte-correct vn_protocol_driver_* encoder, vn_ring/vn_cs/vn_instance) is reused
 * unmodified; the host venus decoder (virglrenderer) is also Mesa, so the wire is
 * compatible.
 *
 * Structural template: vn_renderer_vtest.c. Blob/submit/sync *semantics*:
 * vn_renderer_virtgpu.c. The Helios IOCTL surface (codes, structs) mirrors
 * protocol/src/{ioctl.rs,escape.rs,virtio_gpu.rs} byte-for-byte.
 *
 * KMD semantics that shape this backend (PHASE5_HANDOVER §2; KMD Phase 4e):
 *   - SUBMIT_VENUS is NON-BLOCKING: the KMD queues the submission and returns
 *     immediately (multiple submits in flight at once). ops.submit records each
 *     sync target as pending on the batch fence. ops.wait advances sync values
 *     only after WAIT_FENCE confirms retirement, preserving present ordering
 *     without returning to a synchronous submit channel.
 *   - SUBMIT_VENUS is METHOD_IN_DIRECT: the fixed header rides lpInBuffer
 *     (buffered) and the variable Venus cs rides lpOutBuffer (read-locked MDL) —
 *     see kmd/src/ioctl.rs::handle_submit_venus.
 *   - There is no per-resource RESOURCE_UNREF / UNMAP_BLOB IOCTL yet, so blob
 *     resources and their user mappings are reclaimed at handle-close
 *     (EvtFileCleanup) / CTX_DESTROY. shmem/bo destroy here only frees the guest
 *     tracking struct (the shmem cache keeps churn — and thus the leak — bounded).
 */

/* WIN32_LEAN_AND_MEAN is already defined on the Mesa build command line. */
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h> /* Phase-7 gate res_id surfacing (throwaway) */
#include <stdlib.h>
#include <string.h>

#include "vn_renderer_internal.h"

#include "util/cache_ops.h"

#include <windows.h>
#include <setupapi.h>

/* ── Helios device interface GUID (protocol/src/ioctl.rs) ───────────────────── */
/* {C8F84237-CD89-48F5-AFC5-32944524625C} */
static const GUID GUID_DEVINTERFACE_HELIOS = {
   0xC8F84237,
   0xCD89,
   0x48F5,
   { 0xAF, 0xC5, 0x32, 0x94, 0x45, 0x24, 0x62, 0x5C }
};

/* ── Helios IOCTL codes (protocol/src/ioctl.rs) ────────────────────────────── */
#define IOCTL_HELIOS_CTX_CREATE   0x0022E400u
#define IOCTL_HELIOS_CTX_DESTROY  0x0022E404u
#define IOCTL_HELIOS_SUBMIT_VENUS 0x0022E409u /* METHOD_IN_DIRECT */
#define IOCTL_HELIOS_ALLOC_BLOB   0x0022E40Cu
#define IOCTL_HELIOS_MAP_BLOB     0x0022E410u
#define IOCTL_HELIOS_WAIT_FENCE   0x0022E414u

/* ── Escape payload structs (protocol/src/escape.rs) — repr(C), padding-free ─── */
#define HELIOS_ESCAPE_MAGIC   0x48454C53u /* 'HELS' */
#define HELIOS_ESCAPE_VERSION 1u

#define HELIOS_ESCAPE_SUBMIT_VENUS 0x0001u
#define HELIOS_ESCAPE_CTX_CREATE   0x0002u
#define HELIOS_ESCAPE_CTX_DESTROY  0x0003u
#define HELIOS_ESCAPE_ALLOC_BLOB   0x0004u
#define HELIOS_ESCAPE_MAP_BLOB     0x0005u
#define HELIOS_ESCAPE_WAIT_FENCE   0x0006u

#define HELIOS_MAP_CACHE_CACHED    0x00000001u
#define HELIOS_MAP_CACHE_UNCACHED  0x00000002u
#define HELIOS_MAP_CACHE_WC        0x00000003u

/* virtio-gpu constants the backend needs (protocol/src/virtio_gpu.rs) */
#define VIRTIO_GPU_CAPSET_VENUS          4u
#define VIRTIO_GPU_BLOB_MEM_HOST3D       2u
#define VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE 1u

struct helios_escape_header {
   uint32_t magic;    /* == HELIOS_ESCAPE_MAGIC */
   uint32_t cmd_type; /* one of HELIOS_ESCAPE_* */
   uint32_t version;  /* == HELIOS_ESCAPE_VERSION */
   uint32_t size;     /* total escape buffer size in bytes */
};

struct helios_escape_ctx_create {
   struct helios_escape_header hdr;
   uint32_t capset_id;  /* in:  VIRTIO_GPU_CAPSET_VENUS */
   uint32_t out_ctx_id; /* out: assigned context id */
};

struct helios_escape_ctx_destroy {
   struct helios_escape_header hdr;
   uint32_t ctx_id;
   uint32_t padding;
};

struct helios_escape_submit_venus {
   struct helios_escape_header hdr;
   uint64_t fence_id;
   uint32_t ctx_id;
   uint32_t buffer_size;
   uint32_t ring_idx; /* venus per-queue host timeline (0 = CPU/primary ring) */
   uint32_t _pad;
};

struct helios_escape_alloc_blob {
   struct helios_escape_header hdr;
   uint64_t size;            /* in:  blob size in bytes */
   uint64_t blob_id;         /* in:  venus device-memory id backing the blob (0 = none) */
   uint32_t blob_flags;      /* in:  VIRTIO_GPU_BLOB_FLAG_* */
   uint32_t blob_mem;        /* in:  VIRTIO_GPU_BLOB_MEM_* */
   uint32_t ctx_id;          /* in:  owning context */
   uint32_t out_resource_id; /* out: assigned resource id */
};

struct helios_escape_map_blob {
   struct helios_escape_header hdr;
   uint64_t out_user_va; /* out: user-mode virtual address of the mapping */
   uint32_t resource_id; /* in:  blob to map */
   uint32_t map_cache;   /* in/out: requested/effective VIRTIO_GPU_MAP_CACHE_* */
};

struct helios_escape_wait_fence {
   struct helios_escape_header hdr;
   uint64_t fence_id;
   uint64_t timeout_ns;
};

/* Wire-size guards mirroring protocol/src/escape.rs const _: () asserts. */
_Static_assert(sizeof(struct helios_escape_header) == 16, "hdr size");
_Static_assert(sizeof(struct helios_escape_ctx_create) == 24, "ctx_create size");
_Static_assert(sizeof(struct helios_escape_ctx_destroy) == 24, "ctx_destroy size");
_Static_assert(sizeof(struct helios_escape_submit_venus) == 40, "submit size");
_Static_assert(sizeof(struct helios_escape_alloc_blob) == 48, "alloc_blob size");
_Static_assert(sizeof(struct helios_escape_map_blob) == 32, "map_blob size");
_Static_assert(sizeof(struct helios_escape_wait_fence) == 32, "wait_fence size");

/* ── Backend private structs (vtest pattern: base is the first member) ──────── */

struct helios_shmem {
   struct vn_renderer_shmem base;
};

struct helios_bo {
   struct vn_renderer_bo base;
   uint32_t blob_flags;
   uint32_t map_cache;
   VkMemoryPropertyFlags memory_flags;
};

#define HELIOS_SYNC_PENDING_MAX 256
#define HELIOS_WAIT_FENCE_STACK_MAX 256

struct helios_sync_pending {
   uint64_t val;
   uint64_t fence_id;
   bool complete;
};

struct helios_sync {
   struct vn_renderer_sync base;
   /* Last value known to have retired on the host. Pending target values are kept
    * ordered so a later out-of-order fence cannot make an older frame appear
    * complete. */
   uint64_t val;
   uint32_t pending_count;
   struct helios_sync_pending pending[HELIOS_SYNC_PENDING_MAX];
};

enum helios_ioctl_stat {
   HELIOS_STAT_CTX_CREATE = 0,
   HELIOS_STAT_CTX_DESTROY,
   HELIOS_STAT_SUBMIT,
   HELIOS_STAT_ALLOC_BLOB,
   HELIOS_STAT_MAP_BLOB,
   HELIOS_STAT_WAIT_FENCE,
   HELIOS_STAT_COUNT,
};

struct helios_perf_ioctl {
   uint64_t calls;
   uint64_t failures;
   uint64_t bytes_in;
   uint64_t bytes_out;
   int64_t ticks;
};

struct helios_perf_stats {
   bool enabled;
   bool dumped;
   bool live;
   LARGE_INTEGER qpc_freq;
   struct helios_perf_ioctl ioctl[HELIOS_STAT_COUNT];
   uint64_t submit_calls;
   uint64_t submit_batches;
   uint64_t submit_empty_batches;
   uint64_t submit_syncs;
   uint64_t submit_cs_bytes;
   uint64_t wait_calls;
   uint64_t wait_fast;
   uint64_t wait_slow;
   uint64_t wait_timeout;
   uint64_t shmem_cache_hits;
   uint64_t shmem_creates;
   uint64_t bo_creates;
   uint64_t bo_maps;
   uint64_t bo_map_cached;
   uint64_t bo_map_wc;
   uint64_t bo_map_uncached;
   uint64_t bo_map_unknown;
};

struct helios {
   struct vn_renderer base;

   struct vn_instance *instance;

   /* The device-interface handle + the lock serializing all IOCTLs on it.
    * MAP_BLOB in particular must be serialized and issued from the process that
    * opened the handle (this process) — see kmd/src/ioctl.rs::handle_map_blob. */
   mtx_t dev_mutex;
   HANDLE dev;

   uint32_t ctx_id;
   uint64_t next_fence_id; /* monotonic, under dev_mutex */

   struct vn_renderer_shmem_cache shmem_cache;
   struct helios_perf_stats perf;
};

static struct helios *helios_perf_at_exit_renderer;
static bool helios_perf_at_exit_registered;

static void helios_perf_write(struct helios *helios, bool final);
static bool helios_ioctl_wait_fence(struct helios *helios,
                                    uint64_t fence_id,
                                    uint64_t timeout_ns);

/* ── IOCTL helpers ─────────────────────────────────────────────────────────── */

static void
helios_hdr_init(struct helios_escape_header *hdr, uint32_t cmd_type, uint32_t size)
{
   hdr->magic = HELIOS_ESCAPE_MAGIC;
   hdr->cmd_type = cmd_type;
   hdr->version = HELIOS_ESCAPE_VERSION;
   hdr->size = size;
}

static enum helios_ioctl_stat
helios_ioctl_stat_from_code(uint32_t code)
{
   switch (code) {
   case IOCTL_HELIOS_CTX_CREATE:
      return HELIOS_STAT_CTX_CREATE;
   case IOCTL_HELIOS_CTX_DESTROY:
      return HELIOS_STAT_CTX_DESTROY;
   case IOCTL_HELIOS_SUBMIT_VENUS:
      return HELIOS_STAT_SUBMIT;
   case IOCTL_HELIOS_ALLOC_BLOB:
      return HELIOS_STAT_ALLOC_BLOB;
   case IOCTL_HELIOS_MAP_BLOB:
      return HELIOS_STAT_MAP_BLOB;
   case IOCTL_HELIOS_WAIT_FENCE:
      return HELIOS_STAT_WAIT_FENCE;
   default:
      return HELIOS_STAT_COUNT;
   }
}

/* One DeviceIoControl round-trip. For METHOD_BUFFERED ops, `in`/`out` are the
 * in/out copies of the escape struct (the I/O manager double-buffers). For the
 * SUBMIT_VENUS METHOD_IN_DIRECT op, `out` carries the read-locked Venus cs bytes
 * (the KMD reads them via the input MDL). Returns false on a Win32 failure. */
static bool
helios_ioctl(struct helios *helios,
             uint32_t code,
             void *in,
             uint32_t in_size,
             void *out,
             uint32_t out_size)
{
   DWORD returned = 0;
   LARGE_INTEGER t0 = { 0 };
   const enum helios_ioctl_stat stat = helios_ioctl_stat_from_code(code);
   if (helios->perf.enabled)
      QueryPerformanceCounter(&t0);

   const BOOL ok = DeviceIoControl(helios->dev, code, in, in_size, out, out_size,
                                   &returned, NULL);

   if (helios->perf.enabled && stat < HELIOS_STAT_COUNT) {
      LARGE_INTEGER t1;
      QueryPerformanceCounter(&t1);
      struct helios_perf_ioctl *s = &helios->perf.ioctl[stat];
      s->calls++;
      s->bytes_in += in_size;
      s->bytes_out += out_size;
      s->ticks += t1.QuadPart - t0.QuadPart;
      if (!ok)
         s->failures++;
      if (helios->perf.live)
         helios_perf_write(helios, false);
   }

   if (!ok) {
      vn_log(helios->instance, "Helios IOCTL 0x%x failed: Win32 error %lu", code,
             (unsigned long)GetLastError());
      return false;
   }
   return true;
}

static bool
helios_ioctl_ctx_create(struct helios *helios, uint32_t capset_id, uint32_t *out_ctx_id)
{
   struct helios_escape_ctx_create req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_CTX_CREATE, sizeof(req));
   req.capset_id = capset_id;

   struct helios_escape_ctx_create out = req;
   if (!helios_ioctl(helios, IOCTL_HELIOS_CTX_CREATE, &req, sizeof(req), &out,
                     sizeof(out)))
      return false;

   *out_ctx_id = out.out_ctx_id;
   return true;
}

static void
helios_ioctl_ctx_destroy(struct helios *helios, uint32_t ctx_id)
{
   struct helios_escape_ctx_destroy req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_CTX_DESTROY, sizeof(req));
   req.ctx_id = ctx_id;
   helios_ioctl(helios, IOCTL_HELIOS_CTX_DESTROY, &req, sizeof(req), NULL, 0);
}

/* SUBMIT_VENUS (METHOD_IN_DIRECT). Caller MUST hold dev_mutex (next_fence_id +
 * ordering). The cs bytes ride lpOutBuffer, which the KMD retrieves via
 * WdfRequestRetrieveOutputBuffer (METHOD_IN_DIRECT read-locks that buffer's MDL)
 * and only reads — see handle_submit_venus. NON-BLOCKING (KMD Phase 4e): the KMD
 * queues the submission and returns immediately; completion is observed later via
 * WAIT_FENCE. `*out_fence_id` (if non-NULL) receives the assigned fence id so the
 * caller can record it on the batch's syncs for ops.wait. */
static bool
helios_ioctl_submit_cs(struct helios *helios,
                       const void *cs_data,
                       size_t cs_size,
                       uint32_t ring_idx,
                       uint64_t *out_fence_id)
{
   if (cs_size == 0 || cs_size > UINT32_MAX)
      return false;

   const uint64_t fence_id = ++helios->next_fence_id;

   struct helios_escape_submit_venus hdr = { 0 };
   helios_hdr_init(&hdr.hdr, HELIOS_ESCAPE_SUBMIT_VENUS, sizeof(hdr));
   hdr.fence_id = fence_id;
   hdr.ctx_id = helios->ctx_id;
   hdr.buffer_size = (uint32_t)cs_size;
   hdr.ring_idx = ring_idx;

   /* lpOutBuffer carries the cs; the KMD only reads it (IN_DIRECT read-lock), so
    * casting away const is safe. */
   const bool ok = helios_ioctl(helios, IOCTL_HELIOS_SUBMIT_VENUS, &hdr,
                                sizeof(hdr), (void *)(uintptr_t)cs_data,
                                (uint32_t)cs_size);
   if (ok && out_fence_id)
      *out_fence_id = fence_id;
   return ok;
}

/* ALLOC_BLOB. Caller MUST hold dev_mutex. Returns the resource id, or 0 on
 * failure (a valid resource id is always > 0). */
static uint32_t
helios_ioctl_alloc_blob(struct helios *helios,
                        uint32_t blob_mem,
                        uint32_t blob_flags,
                        uint64_t blob_id,
                        uint64_t size)
{
   struct helios_escape_alloc_blob req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_ALLOC_BLOB, sizeof(req));
   req.size = size;
   req.blob_id = blob_id;
   req.blob_flags = blob_flags;
   req.blob_mem = blob_mem;
   req.ctx_id = helios->ctx_id;

   struct helios_escape_alloc_blob out = req;
   if (!helios_ioctl(helios, IOCTL_HELIOS_ALLOC_BLOB, &req, sizeof(req), &out,
                     sizeof(out)))
      return 0;

   return out.out_resource_id;
}

/* MAP_BLOB. Caller MUST hold dev_mutex (the KMD requires serialized maps from
 * the opening process). Returns the user VA, or 0 on failure. */
static uint64_t
helios_ioctl_map_blob(struct helios *helios,
                      uint32_t resource_id,
                      uint32_t requested_map_cache,
                      uint32_t *out_map_cache)
{
   struct helios_escape_map_blob req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_MAP_BLOB, sizeof(req));
   req.resource_id = resource_id;
   req.map_cache = requested_map_cache;

   struct helios_escape_map_blob out = req;
   if (!helios_ioctl(helios, IOCTL_HELIOS_MAP_BLOB, &req, sizeof(req), &out,
                     sizeof(out)))
      return 0;

   if (out_map_cache)
      *out_map_cache = out.map_cache;
   return out.out_user_va;
}

static bool
helios_ioctl_wait_fence(struct helios *helios, uint64_t fence_id, uint64_t timeout_ns)
{
   struct helios_escape_wait_fence req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_WAIT_FENCE, sizeof(req));
   req.fence_id = fence_id;
   req.timeout_ns = timeout_ns;
   return helios_ioctl(helios, IOCTL_HELIOS_WAIT_FENCE, &req, sizeof(req), NULL, 0);
}

static void
helios_sync_retire_locked(struct helios_sync *sync)
{
   uint32_t n = 0;
   while (n < sync->pending_count && sync->pending[n].complete)
      n++;

   if (!n)
      return;

   const uint64_t val = sync->pending[n - 1].val;
   if (sync->val < val)
      sync->val = val;

   sync->pending_count -= n;
   if (sync->pending_count) {
      memmove(sync->pending, sync->pending + n,
              sync->pending_count * sizeof(sync->pending[0]));
   }
}

static bool
helios_sync_append_locked(struct helios_sync *sync, uint64_t val, uint64_t fence_id)
{
   if (!fence_id) {
      if (sync->val < val)
         sync->val = val;
      sync->pending_count = 0;
      return true;
   }

   if (sync->pending_count >= HELIOS_SYNC_PENDING_MAX)
      return false;

   sync->pending[sync->pending_count++] = (struct helios_sync_pending) {
      .val = val,
      .fence_id = fence_id,
      .complete = false,
   };
   return true;
}

static void
helios_sync_mark_fence_locked(struct helios_sync *sync, uint64_t fence_id)
{
   for (uint32_t i = 0; i < sync->pending_count; i++) {
      if (sync->pending[i].fence_id == fence_id)
         sync->pending[i].complete = true;
   }
   helios_sync_retire_locked(sync);
}

static bool
helios_wait_fence_list_contains(const uint64_t *fences, uint32_t count, uint64_t fence_id)
{
   for (uint32_t i = 0; i < count; i++) {
      if (fences[i] == fence_id)
         return true;
   }
   return false;
}

static void
helios_perf_init(struct helios *helios)
{
   char enabled[8];
   if (!GetEnvironmentVariableA("HELIOS_PERF", enabled, sizeof(enabled)))
      return;

   helios->perf.enabled = true;
   helios->perf.live =
      GetEnvironmentVariableA("HELIOS_PERF_LIVE", enabled, sizeof(enabled)) != 0;
   QueryPerformanceFrequency(&helios->perf.qpc_freq);
}

static double
helios_perf_ms(const struct helios *helios, int64_t ticks)
{
   if (!helios->perf.qpc_freq.QuadPart)
      return 0.0;
   return (double)ticks * 1000.0 / (double)helios->perf.qpc_freq.QuadPart;
}

static void
helios_perf_write(struct helios *helios, bool final)
{
   FILE *f = stderr;
   char path[MAX_PATH];
   if (GetEnvironmentVariableA("HELIOS_PERF_FILE", path, sizeof(path))) {
      FILE *opened = fopen(path, "a");
      if (opened)
         f = opened;
   }

   static const char *names[HELIOS_STAT_COUNT] = {
      "ctx_create",
      "ctx_destroy",
      "submit",
      "alloc_blob",
      "map_blob",
      "wait_fence",
   };

   fprintf(f, "Helios perf summary (%s)\n", final ? "final" : "live");
   fprintf(f,
           "submit_calls=%llu batches=%llu empty_batches=%llu syncs=%llu cs_bytes=%llu\n",
           (unsigned long long)helios->perf.submit_calls,
           (unsigned long long)helios->perf.submit_batches,
           (unsigned long long)helios->perf.submit_empty_batches,
           (unsigned long long)helios->perf.submit_syncs,
           (unsigned long long)helios->perf.submit_cs_bytes);
   fprintf(f, "wait_calls=%llu fast=%llu slow=%llu timeout=%llu\n",
           (unsigned long long)helios->perf.wait_calls,
           (unsigned long long)helios->perf.wait_fast,
           (unsigned long long)helios->perf.wait_slow,
           (unsigned long long)helios->perf.wait_timeout);
   fprintf(f, "shmem_creates=%llu shmem_cache_hits=%llu bo_creates=%llu bo_maps=%llu\n",
           (unsigned long long)helios->perf.shmem_creates,
           (unsigned long long)helios->perf.shmem_cache_hits,
           (unsigned long long)helios->perf.bo_creates,
           (unsigned long long)helios->perf.bo_maps);
   fprintf(f, "bo_map_cache cached=%llu wc=%llu uncached=%llu unknown=%llu\n",
           (unsigned long long)helios->perf.bo_map_cached,
           (unsigned long long)helios->perf.bo_map_wc,
           (unsigned long long)helios->perf.bo_map_uncached,
           (unsigned long long)helios->perf.bo_map_unknown);

   for (uint32_t i = 0; i < HELIOS_STAT_COUNT; i++) {
      const struct helios_perf_ioctl *s = &helios->perf.ioctl[i];
      if (!s->calls)
         continue;
      fprintf(f,
              "ioctl.%s calls=%llu failures=%llu ms=%.3f avg_us=%.3f bytes_in=%llu bytes_out=%llu\n",
              names[i],
              (unsigned long long)s->calls,
              (unsigned long long)s->failures,
              helios_perf_ms(helios, s->ticks),
              helios_perf_ms(helios, s->ticks) * 1000.0 / (double)s->calls,
              (unsigned long long)s->bytes_in,
              (unsigned long long)s->bytes_out);
   }

   fprintf(f, "\n");
   if (f != stderr)
      fclose(f);
}

static void
helios_perf_dump(struct helios *helios)
{
   if (!helios->perf.enabled || helios->perf.dumped)
      return;
   helios->perf.dumped = true;
   helios_perf_write(helios, true);
}

static void
helios_perf_dump_at_exit(void)
{
   if (helios_perf_at_exit_renderer)
      helios_perf_dump(helios_perf_at_exit_renderer);
}

/* ── Device discovery + open (mirrors probe/src/main.rs::open_helios) ───────── */

static HANDLE
helios_open_device(struct vn_instance *instance)
{
   HDEVINFO dev_info =
      SetupDiGetClassDevsW(&GUID_DEVINTERFACE_HELIOS, NULL, NULL,
                           DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
   if (dev_info == INVALID_HANDLE_VALUE) {
      vn_log(instance, "SetupDiGetClassDevs failed: %lu",
             (unsigned long)GetLastError());
      return INVALID_HANDLE_VALUE;
   }

   SP_DEVICE_INTERFACE_DATA ifd = { 0 };
   ifd.cbSize = sizeof(ifd);
   if (!SetupDiEnumDeviceInterfaces(dev_info, NULL, &GUID_DEVINTERFACE_HELIOS, 0,
                                    &ifd)) {
      vn_log(instance, "no GUID_DEVINTERFACE_HELIOS instance present: %lu",
             (unsigned long)GetLastError());
      SetupDiDestroyDeviceInfoList(dev_info);
      return INVALID_HANDLE_VALUE;
   }

   /* First call: required detail buffer size. */
   DWORD required = 0;
   SetupDiGetDeviceInterfaceDetailW(dev_info, &ifd, NULL, 0, &required, NULL);
   if (required == 0) {
      vn_log(instance, "interface detail size query failed: %lu",
             (unsigned long)GetLastError());
      SetupDiDestroyDeviceInfoList(dev_info);
      return INVALID_HANDLE_VALUE;
   }

   SP_DEVICE_INTERFACE_DETAIL_DATA_W *detail = calloc(1, required);
   if (!detail) {
      SetupDiDestroyDeviceInfoList(dev_info);
      return INVALID_HANDLE_VALUE;
   }
   /* cbSize is the FIXED header size, NOT the buffer size (8 on x64). */
   detail->cbSize = sizeof(*detail);

   HANDLE dev = INVALID_HANDLE_VALUE;
   if (SetupDiGetDeviceInterfaceDetailW(dev_info, &ifd, detail, required, NULL,
                                        NULL)) {
      dev = CreateFileW(detail->DevicePath, GENERIC_READ | GENERIC_WRITE,
                        FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0,
                        NULL);
      if (dev == INVALID_HANDLE_VALUE)
         vn_log(instance, "CreateFile on Helios device failed: %lu",
                (unsigned long)GetLastError());
   } else {
      vn_log(instance, "SetupDiGetDeviceInterfaceDetail failed: %lu",
             (unsigned long)GetLastError());
   }

   free(detail);
   SetupDiDestroyDeviceInfoList(dev_info);
   return dev;
}

/* ── ops ───────────────────────────────────────────────────────────────────── */

static VkResult
helios_submit(struct vn_renderer *renderer, const struct vn_renderer_submit *submit)
{
   struct helios *helios = (struct helios *)renderer;
   VkResult result = VK_SUCCESS;

   mtx_lock(&helios->dev_mutex);
   if (helios->perf.enabled)
      helios->perf.submit_calls++;
   for (uint32_t i = 0; i < submit->batch_count; i++) {
      const struct vn_renderer_submit_batch *batch = &submit->batches[i];

      if (helios->perf.enabled) {
         helios->perf.submit_batches++;
         helios->perf.submit_syncs += batch->sync_count;
         helios->perf.submit_cs_bytes += batch->cs_size;
         if (!batch->cs_size)
            helios->perf.submit_empty_batches++;
      }

      uint64_t fence_id = 0;
      if (batch->cs_size) {
         if (!helios_ioctl_submit_cs(helios, batch->cs_data, batch->cs_size,
                                     batch->ring_idx, &fence_id)) {
            result = VK_ERROR_DEVICE_LOST;
            break;
         }
      }

      for (uint32_t j = 0; j < batch->sync_count; j++) {
         struct helios_sync *sync = (struct helios_sync *)batch->syncs[j];
         if (!helios_sync_append_locked(sync, batch->sync_values[j], fence_id)) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
            break;
         }
      }
      if (result != VK_SUCCESS)
         break;
   }
   mtx_unlock(&helios->dev_mutex);

   return result;
}

static VkResult
helios_wait(struct vn_renderer *renderer, const struct vn_renderer_wait *wait)
{
   struct helios *helios = (struct helios *)renderer;
   uint64_t stack_wait_fences[HELIOS_WAIT_FENCE_STACK_MAX];
   uint64_t *wait_fences = stack_wait_fences;
   uint32_t wait_fence_count = 0;
   VkResult result = VK_SUCCESS;

   if (helios->perf.enabled)
      helios->perf.wait_calls++;

   mtx_lock(&helios->dev_mutex);
   bool satisfied = !wait->wait_any; /* wait_all starts true, wait_any starts false */
   for (uint32_t i = 0; i < wait->sync_count; i++) {
      const struct helios_sync *sync = (const struct helios_sync *)wait->syncs[i];
      const bool reached = sync->val >= wait->sync_values[i];
      if (wait->wait_any) {
         satisfied = satisfied || reached;
      } else {
         satisfied = satisfied && reached;
      }
   }
   mtx_unlock(&helios->dev_mutex);

   if (satisfied) {
      if (helios->perf.enabled)
         helios->perf.wait_fast++;
      return VK_SUCCESS;
   }
   if (wait->timeout == 0) {
      if (helios->perf.enabled)
         helios->perf.wait_timeout++;
      return VK_TIMEOUT;
   }

   const uint32_t wait_fence_capacity =
      wait->sync_count * HELIOS_SYNC_PENDING_MAX;
   if (wait_fence_capacity > HELIOS_WAIT_FENCE_STACK_MAX) {
      wait_fences = calloc(wait_fence_capacity, sizeof(*wait_fences));
      if (!wait_fences)
         return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   mtx_lock(&helios->dev_mutex);
   for (uint32_t i = 0; i < wait->sync_count; i++) {
      const struct helios_sync *sync = (const struct helios_sync *)wait->syncs[i];
      if (sync->val >= wait->sync_values[i])
         continue;

      for (uint32_t j = 0; j < sync->pending_count; j++) {
         const struct helios_sync_pending *pending = &sync->pending[j];
         if (pending->val > wait->sync_values[i])
            break;
         if (!pending->complete &&
             wait_fence_count < wait_fence_capacity &&
             !helios_wait_fence_list_contains(wait_fences, wait_fence_count,
                                              pending->fence_id))
            wait_fences[wait_fence_count++] = pending->fence_id;
      }
   }
   mtx_unlock(&helios->dev_mutex);

   if (!wait_fence_count) {
      if (helios->perf.enabled)
         helios->perf.wait_timeout++;
      if (wait_fences != stack_wait_fences)
         free(wait_fences);
      return VK_TIMEOUT; /* nothing identifiable to wait on */
   }

   if (helios->perf.enabled)
      helios->perf.wait_slow++;

   for (uint32_t i = 0; i < wait_fence_count; i++) {
      if (!helios_ioctl_wait_fence(helios, wait_fences[i], wait->timeout)) {
         result = VK_TIMEOUT;
         break;
      }
   }

   if (result == VK_SUCCESS) {
      mtx_lock(&helios->dev_mutex);
      for (uint32_t i = 0; i < wait->sync_count; i++) {
         struct helios_sync *sync = (struct helios_sync *)wait->syncs[i];
         for (uint32_t j = 0; j < wait_fence_count; j++) {
            helios_sync_mark_fence_locked(sync, wait_fences[j]);
         }
      }
      mtx_unlock(&helios->dev_mutex);
   } else if (helios->perf.enabled) {
      helios->perf.wait_timeout++;
   }

   if (wait_fences != stack_wait_fences)
      free(wait_fences);
   return result;
}

/* ── shmem ops ─────────────────────────────────────────────────────────────── */

/* Cache-eviction / final teardown callback. There is no RESOURCE_UNREF / unmap
 * IOCTL yet, so the host resource + its user mapping persist until handle close
 * (KMD EvtFileCleanup) / CTX_DESTROY; we only free the guest tracking struct. */
static void
helios_shmem_destroy_now(struct vn_renderer *renderer, struct vn_renderer_shmem *shmem)
{
   (void)renderer;
   free(shmem); /* base is the first member of struct helios_shmem */
}

static void
helios_shmem_destroy(struct vn_renderer *renderer, struct vn_renderer_shmem *shmem)
{
   struct helios *helios = (struct helios *)renderer;

   if (vn_renderer_shmem_cache_add(&helios->shmem_cache, shmem))
      return;

   helios_shmem_destroy_now(&helios->base, shmem);
}

static struct vn_renderer_shmem *
helios_shmem_create(struct vn_renderer *renderer, size_t size)
{
   struct helios *helios = (struct helios *)renderer;

   struct vn_renderer_shmem *cached_shmem =
      vn_renderer_shmem_cache_get(&helios->shmem_cache, size);
   if (cached_shmem) {
      if (helios->perf.enabled)
         helios->perf.shmem_cache_hits++;
      cached_shmem->refcount = VN_REFCOUNT_INIT(1);
      return cached_shmem;
   }

   if (helios->perf.enabled)
      helios->perf.shmem_creates++;

   /* The command-stream ring + cs/reply pools need genuinely host-coherent,
    * mappable memory the renderer can both read and write (vn_ring head/status).
    * A HOST3D mappable blob with blob_id=0 is exactly that (virtgpu shmem path);
    * the host-visible window is mapped into this process by MAP_BLOB. */
   mtx_lock(&helios->dev_mutex);
   const uint32_t res_id =
      helios_ioctl_alloc_blob(helios, VIRTIO_GPU_BLOB_MEM_HOST3D,
                              VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE, 0, size);
   uint64_t user_va = 0;
   if (res_id)
      user_va = helios_ioctl_map_blob(helios, res_id, 0, NULL);
   mtx_unlock(&helios->dev_mutex);

   if (!res_id || !user_va) {
      vn_log(helios->instance, "shmem create failed (res_id=%u, mapped=%d)", res_id,
             user_va != 0);
      return NULL;
   }

   struct helios_shmem *shmem = calloc(1, sizeof(*shmem));
   if (!shmem)
      return NULL;

   shmem->base.refcount = VN_REFCOUNT_INIT(1);
   shmem->base.res_id = res_id;
   shmem->base.mmap_size = size;
   shmem->base.mmap_ptr = (void *)(uintptr_t)user_va;

   return &shmem->base;
}

/* ── bo ops ────────────────────────────────────────────────────────────────── */

static VkResult
helios_bo_create_from_device_memory(
   struct vn_renderer *renderer,
   struct vn_renderer_submit_batch *batch,
   VkDeviceSize size,
   vn_object_id mem_id,
   VkMemoryPropertyFlags flags,
   VkExternalMemoryHandleTypeFlags external_handles,
   struct vn_renderer_bo **out_bo)
{
   struct helios *helios = (struct helios *)renderer;

   if (helios->perf.enabled)
      helios->perf.bo_creates++;

   /* Helios has no external-handle / dma-buf sharing; mappable iff host-visible. */
   uint32_t blob_flags = 0;
   if (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
      blob_flags |= VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE;

   mtx_lock(&helios->dev_mutex);
   /* The optional batch orders the host's vkAllocateMemory before the blob binds
    * to it ("venus: let resource_create_blob wait for mem alloc"). Submit it
    * first (synchronous) so the device memory exists when ALLOC_BLOB references
    * it via blob_id=mem_id. */
   if (batch) {
      uint64_t fence_id = 0;
      if (batch->cs_size &&
          !helios_ioctl_submit_cs(helios, batch->cs_data, batch->cs_size,
                                  batch->ring_idx, &fence_id)) {
         mtx_unlock(&helios->dev_mutex);
         return VK_ERROR_DEVICE_LOST;
      }
      /* Optimistically signal the batch's syncs and record its fence id (matches
       * helios_submit; a sync-only batch must still advance). The subsequent
       * ALLOC_BLOB(blob_id=mem_id) round-trips synchronously through the KMD, which
       * quiesces in-flight submits first — so by the time the blob binds, this
       * batch's vkAllocateMemory has actually completed on the host. */
      for (uint32_t j = 0; j < batch->sync_count; j++) {
         struct helios_sync *sync = (struct helios_sync *)batch->syncs[j];
         helios_sync_append_locked(sync, batch->sync_values[j], 0);
      }
   }

   const uint32_t res_id = helios_ioctl_alloc_blob(
      helios, VIRTIO_GPU_BLOB_MEM_HOST3D, blob_flags, mem_id, size);
   mtx_unlock(&helios->dev_mutex);

   if (!res_id)
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;

   /* Surface "<res_id> <size>" for the IDD fast path and present gate tests.
    * Use the Win32 environment API instead of CRT getenv(): the ICD may be linked
    * with a different/static CRT than its caller, but the process environment is
    * shared by the OS. */
   {
      char gate_file[MAX_PATH];
      if (GetEnvironmentVariableA("HELIOS_GATE_RESID_FILE", gate_file,
                                  sizeof(gate_file))) {
         FILE *f = fopen(gate_file, "a");
         if (f) {
            fprintf(f, "%u %llu\n", res_id, (unsigned long long)size);
            fclose(f);
         }
      }
   }

   struct helios_bo *bo = calloc(1, sizeof(*bo));
   if (!bo)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   bo->base.refcount = VN_REFCOUNT_INIT(1);
   bo->base.res_id = res_id;
   bo->base.mmap_size = (blob_flags & VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE) ? size : 0;
   bo->blob_flags = blob_flags;
   bo->memory_flags = flags;

   *out_bo = &bo->base;
   return VK_SUCCESS;
}

static void *
helios_bo_map(struct vn_renderer *renderer, struct vn_renderer_bo *_bo, void *placed_addr)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_bo *bo = (struct helios_bo *)_bo;
   const bool mappable = bo->blob_flags & VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE;

   if (helios->perf.enabled)
      helios->perf.bo_maps++;

   /* placed_addr (VK_EXT_map_memory_placed) is unsupported by MAP_BLOB; ignore. */
   (void)placed_addr;

   /* map-once; the vtable contract allows this to be non-thread-safe but we
    * serialize anyway (MAP_BLOB must be serialized in the KMD). */
   mtx_lock(&helios->dev_mutex);
   if (!bo->base.mmap_ptr && mappable) {
      uint32_t map_cache = 0;
      const uint32_t requested_map_cache = bo->base.prefer_cached_map ?
         HELIOS_MAP_CACHE_CACHED : HELIOS_MAP_CACHE_WC;
      const uint64_t va =
         helios_ioctl_map_blob(helios, bo->base.res_id, requested_map_cache,
                               &map_cache);
      if (va) {
         bo->base.mmap_ptr = (void *)(uintptr_t)va;
         bo->map_cache = map_cache;
         if (helios->perf.enabled) {
            switch (map_cache) {
            case HELIOS_MAP_CACHE_CACHED:
               helios->perf.bo_map_cached++;
               break;
            case HELIOS_MAP_CACHE_WC:
               helios->perf.bo_map_wc++;
               break;
            case HELIOS_MAP_CACHE_UNCACHED:
               helios->perf.bo_map_uncached++;
               break;
            default:
               helios->perf.bo_map_unknown++;
               break;
            }
         }
      }
   }
   mtx_unlock(&helios->dev_mutex);

   return bo->base.mmap_ptr;
}

static bool
helios_bo_destroy(struct vn_renderer *renderer, struct vn_renderer_bo *bo)
{
   (void)renderer;
   /* No RESOURCE_UNREF / unmap IOCTL yet — host resource + mapping persist until
    * handle close / CTX_DESTROY (PHASE5_HANDOVER §9 #6). Free the guest struct. */
   free(bo);
   return true;
}

static void
helios_bo_flush(struct vn_renderer *renderer,
                struct vn_renderer_bo *bo,
                VkDeviceSize offset,
                VkDeviceSize size)
{
   (void)renderer;
   if (!bo->mmap_ptr || !size || !util_has_cache_ops())
      return;

   util_flush_range((char *)bo->mmap_ptr + offset, size);
}

static void
helios_bo_invalidate(struct vn_renderer *renderer,
                     struct vn_renderer_bo *bo,
                     VkDeviceSize offset,
                     VkDeviceSize size)
{
   (void)renderer;
   if (!bo->mmap_ptr || !size || !util_has_cache_ops())
      return;

   util_flush_inval_range((char *)bo->mmap_ptr + offset, size);
}

/* ── sync ops (synchronous-submit CPU counter; PHASE5_HANDOVER §4) ──────────── */

static VkResult
helios_sync_create(struct vn_renderer *renderer,
                   uint64_t initial_val,
                   uint32_t flags,
                   struct vn_renderer_sync **out_sync)
{
   (void)renderer;
   (void)flags;

   struct helios_sync *sync = calloc(1, sizeof(*sync));
   if (!sync)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   sync->base.sync_id = 0; /* unused: Helios does not carry host sync ids on the wire */
   sync->val = initial_val;

   *out_sync = &sync->base;
   return VK_SUCCESS;
}

static void
helios_sync_destroy(struct vn_renderer *renderer, struct vn_renderer_sync *_sync)
{
   (void)renderer;
   free(_sync);
}

static VkResult
helios_sync_reset(struct vn_renderer *renderer,
                  struct vn_renderer_sync *_sync,
                  uint64_t initial_val)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_sync *sync = (struct helios_sync *)_sync;

   mtx_lock(&helios->dev_mutex);
   sync->val = initial_val;
   sync->pending_count = 0;
   mtx_unlock(&helios->dev_mutex);
   return VK_SUCCESS;
}

static VkResult
helios_sync_read(struct vn_renderer *renderer,
                 struct vn_renderer_sync *_sync,
                 uint64_t *val)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_sync *sync = (struct helios_sync *)_sync;
   uint64_t fences[HELIOS_SYNC_PENDING_MAX];
   uint32_t fence_count = 0;

   mtx_lock(&helios->dev_mutex);
   for (uint32_t i = 0; i < sync->pending_count; i++) {
      const uint64_t fence_id = sync->pending[i].fence_id;
      if (!sync->pending[i].complete &&
          !helios_wait_fence_list_contains(fences, fence_count, fence_id))
         fences[fence_count++] = fence_id;
   }
   mtx_unlock(&helios->dev_mutex);

   for (uint32_t i = 0; i < fence_count; i++) {
      if (helios_ioctl_wait_fence(helios, fences[i], 0)) {
         mtx_lock(&helios->dev_mutex);
         helios_sync_mark_fence_locked(sync, fences[i]);
         mtx_unlock(&helios->dev_mutex);
      } else {
         break;
      }
   }

   mtx_lock(&helios->dev_mutex);
   *val = sync->val;
   mtx_unlock(&helios->dev_mutex);
   return VK_SUCCESS;
}

static VkResult
helios_sync_write(struct vn_renderer *renderer,
                  struct vn_renderer_sync *_sync,
                  uint64_t val)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_sync *sync = (struct helios_sync *)_sync;

   mtx_lock(&helios->dev_mutex);
   sync->val = val;
   sync->pending_count = 0;
   mtx_unlock(&helios->dev_mutex);
   return VK_SUCCESS;
}

/* ── init / teardown ───────────────────────────────────────────────────────── */

static void
helios_init_renderer_info(struct helios *helios)
{
   struct vn_renderer_info *info = &helios->base.info;

   /* Helios has no GET_CAPSET IOCTL, so these are hardcoded (PHASE5_HANDOVER §5);
    * the host already negotiated the venus capset at CTX_CREATE, and the layers
    * above clamp over-reported versions down. The hard gate is wire_format_version
    * == vn_info_wire_format_version() (==1); vk_xml_version must be >= 1.1 and is
    * clamped to vn_info_vk_xml_version(). */
   info->wire_format_version = 1;
   info->vk_xml_version = VK_MAKE_API_VERSION(0, 1, 4, 343);
   info->vk_ext_command_serialization_spec_version = 1;
   info->vk_mesa_venus_protocol_spec_version = 4;

   /* mask1[0] bit0 clear => "all extensions supported by the renderer protocol"
    * (venus_hw.h:44, vn_cs.c:21). All-zero = maximally permissive. */
   memset(info->vk_extension_mask, 0, sizeof(info->vk_extension_mask));

   /* Per-queue host timelines. venus binds each VkQueue to a distinct ring_idx
    * (1..max-1; ring_idx 0 is the reserved CPU timeline) via
    * VkDeviceQueueTimelineInfoMESA in the queue-create cs — which the KMD forwards
    * transparently, so the host render server demuxes the timelines. A value of 1
    * makes vn_instance_acquire_ring_idx fail for the FIRST queue (`1 >= 1`), so
    * vkCreateDevice fails. The real virtgpu backend hardcodes 64 ("implied by
    * CONTEXT_INIT"); mirror it. NOTE: the synchronous SUBMIT_VENUS path does not
    * yet propagate batch->ring_idx into the virtio submit header
    * (VIRTIO_GPU_FLAG_INFO_RING_IDX); fences still resolve because each submit
    * blocks to host completion, but per-ring async fencing is a later refinement. */
   info->max_timeline_count = 64;

   info->has_dma_buf_import = false;
   info->has_external_sync = false;
   info->has_implicit_fencing = false;
   info->has_guest_vram = false;

   info->pci.vendor_id = 0x1af4;
   info->pci.device_id = 0x1050;
   info->pci.has_bus_info = false;

   /* LUID for DXVK/VKD3D D3DKMT interop is a Phase 6 concern. */
   info->id.has_luid = false;
}

static void
helios_destroy(struct vn_renderer *renderer, const VkAllocationCallbacks *alloc)
{
   struct helios *helios = (struct helios *)renderer;

   helios_perf_dump(helios);
   if (helios_perf_at_exit_renderer == helios)
      helios_perf_at_exit_renderer = NULL;

   vn_renderer_shmem_cache_fini(&helios->shmem_cache);

   if (helios->dev != INVALID_HANDLE_VALUE && helios->dev != NULL) {
      if (helios->ctx_id)
         helios_ioctl_ctx_destroy(helios, helios->ctx_id);
      CloseHandle(helios->dev);
   }

   mtx_destroy(&helios->dev_mutex);

   vk_free(alloc, helios);
}

static VkResult
helios_init(struct helios *helios)
{
   mtx_init(&helios->dev_mutex, mtx_plain);
   helios_perf_init(helios);
   if (helios->perf.enabled) {
      helios_perf_at_exit_renderer = helios;
      if (!helios_perf_at_exit_registered) {
         atexit(helios_perf_dump_at_exit);
         helios_perf_at_exit_registered = true;
      }
   }

   helios->dev = helios_open_device(helios->instance);
   if (helios->dev == INVALID_HANDLE_VALUE)
      return VK_ERROR_INITIALIZATION_FAILED;

   /* Create the single venus virtio-gpu context up front so it exists before the
    * first shmem/submit (analog of vtest_vcmd_context_init). */
   if (!helios_ioctl_ctx_create(helios, VIRTIO_GPU_CAPSET_VENUS, &helios->ctx_id) ||
       helios->ctx_id == 0) {
      vn_log(helios->instance, "CTX_CREATE(VENUS) failed or returned ctx_id 0");
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   vn_renderer_shmem_cache_init(&helios->shmem_cache, &helios->base,
                                helios_shmem_destroy_now);

   helios_init_renderer_info(helios);

   helios->base.ops.destroy = helios_destroy;
   helios->base.ops.submit = helios_submit;
   helios->base.ops.wait = helios_wait;

   helios->base.shmem_ops.create = helios_shmem_create;
   helios->base.shmem_ops.destroy = helios_shmem_destroy;

   helios->base.bo_ops.create_from_device_memory =
      helios_bo_create_from_device_memory;
   helios->base.bo_ops.create_from_dma_buf = NULL;
   helios->base.bo_ops.destroy = helios_bo_destroy;
   helios->base.bo_ops.export_dma_buf = NULL;
   helios->base.bo_ops.export_sync_file = NULL;
   helios->base.bo_ops.map = helios_bo_map;
   helios->base.bo_ops.flush = helios_bo_flush;
   helios->base.bo_ops.invalidate = helios_bo_invalidate;

   helios->base.sync_ops.create = helios_sync_create;
   helios->base.sync_ops.create_from_syncobj = NULL;
   helios->base.sync_ops.destroy = helios_sync_destroy;
   helios->base.sync_ops.export_syncobj = NULL;
   helios->base.sync_ops.reset = helios_sync_reset;
   helios->base.sync_ops.read = helios_sync_read;
   helios->base.sync_ops.write = helios_sync_write;

   return VK_SUCCESS;
}

VkResult
vn_renderer_create_helios(struct vn_instance *instance,
                          const VkAllocationCallbacks *alloc,
                          struct vn_renderer **renderer)
{
   struct helios *helios = vk_zalloc(alloc, sizeof(*helios), VN_DEFAULT_ALIGN,
                                     VK_SYSTEM_ALLOCATION_SCOPE_INSTANCE);
   if (!helios)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   helios->instance = instance;
   helios->dev = INVALID_HANDLE_VALUE;

   VkResult result = helios_init(helios);
   if (result != VK_SUCCESS) {
      helios_destroy(&helios->base, alloc);
      return result;
   }

   *renderer = &helios->base;
   return VK_SUCCESS;
}

/* ── vtn link stubs ────────────────────────────────────────────────────────── */
/*
 * src/vulkan/util/vk_util.c defines vk_spec_info_to_nir_spirv(), which references
 * the SPIR-V->NIR specialization helpers below. That object gets pulled into the
 * venus ICD for its *other* vk_util helpers, dragging in these undefined symbols
 * — but venus never compiles SPIR-V to NIR on the guest (it forwards SPIR-V to the
 * host), so vk_spec_info_to_nir_spirv is dead code on this transport. Provide
 * no-op stubs to satisfy the linker without pulling in libvtn (the whole SPIR-V
 * compiler + NIR). Returning NULL is handled gracefully (vk_util.c:94). The real
 * fix, if ever needed, is to link idep_vtn.
 *
 * Prototypes precede the definitions to satisfy -Werror=missing-prototypes
 * without including <nir_spirv.h> (which would pull the NIR headers).
 */
struct nir_spirv_specialization;

struct nir_spirv_specialization *vtn_alloc_specialization(uint32_t num_entries);
bool vtn_add_specialization_entry(struct nir_spirv_specialization *spec,
                                  uint32_t slot,
                                  uint32_t entry_id,
                                  uint32_t entry_size,
                                  const void *entry_data,
                                  bool defined_on_module);
void vtn_free_specialization(struct nir_spirv_specialization *spec);

struct nir_spirv_specialization *
vtn_alloc_specialization(uint32_t num_entries)
{
   (void)num_entries;
   return NULL;
}

bool
vtn_add_specialization_entry(struct nir_spirv_specialization *spec,
                             uint32_t slot,
                             uint32_t entry_id,
                             uint32_t entry_size,
                             const void *entry_data,
                             bool defined_on_module)
{
   (void)spec;
   (void)slot;
   (void)entry_id;
   (void)entry_size;
   (void)entry_data;
   (void)defined_on_module;
   return false;
}

void
vtn_free_specialization(struct nir_spirv_specialization *spec)
{
   (void)spec;
}
