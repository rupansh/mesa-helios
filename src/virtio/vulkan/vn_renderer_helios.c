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
 *   - Blob destroy releases the KMD/host resource immediately. The shmem cache
 *     still keeps command-ring blobs alive until eviction/final teardown.
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
#include <time.h>
#include <stdarg.h>
#include <wchar.h> /* wcsstr — adapter description match in helios_open_d3dkmt */

#include "vn_renderer_internal.h"
#include "vn_device.h"
#include "vn_device_memory.h"
#include "vn_instance.h" /* helios_venus_instance_ctx_id (instance-scoped export) */
#include "vn_physical_device.h"
/* helios_venus_queue_gpu_fence: decodes a VkQueue to its venus per-queue
 * ring_idx (vn_queue.h) and orders its fence behind the guest ring's submitted
 * seqno (vn_ring.h + the vkWaitRingSeqnoMESA encoder). */
#include "vn_queue.h"
#include "vn_ring.h"

#include "venus-protocol/vn_protocol_driver_transport.h"

#include "util/cache_ops.h"
#include "util/u_thread.h" /* retire thread (external-sync GPU-completion signal) */

#include <windows.h>
#include <setupapi.h>
/* Gate 5a: the venus transport reaches the kmd_render WDDM adapter through the
 * D3DKMT thunks (gdi32 exports) instead of DeviceIoControl. mingw-w64 ships no
 * D3DKMT headers, so the build adds the vendored WDK headers
 * (icd/win-build/wdk-include, via -I in meson.build); this is the real
 * d3dkmthk.h, so the D3DKMT_* structs are the authoritative OS ABI. */
/* d3dkmthk.h's D3DKMT* prototypes return NTSTATUS, but the header pulls no
 * header that defines it and mingw's <windows.h> doesn't either. Provide it,
 * guarded by the SDK's _NTDEF_ so we never double-define if <ntdef.h> is in. */
#ifndef _NTDEF_
typedef LONG NTSTATUS, *PNTSTATUS;
#endif
#include <d3dkmthk.h>

struct _OBJECT_ATTRIBUTES {
   ULONG Length;
   HANDLE RootDirectory;
   PVOID ObjectName;
   ULONG Attributes;
   PVOID SecurityDescriptor;
   PVOID SecurityQualityOfService;
};

/* Minimal UNICODE_STRING (ntdef.h ABI) for OBJECT_ATTRIBUTES.ObjectName —
 * named WDDM sync sharing (mingw pulls no ntdef/winternl here). */
struct helios_unicode_string {
   USHORT Length;
   USHORT MaximumLength;
   WCHAR *Buffer;
};

/* ── Helios IOCTL codes (protocol/src/ioctl.rs) ────────────────────────────── */
/* Retained only for the not-yet-ported verbs that still route through the
 * fail-clean helios_ioctl() stub (submit/blob/wait). The System-class device
 * interface GUID + SetupDi open path are gone — the WDDM adapter is reached via
 * D3DKMT (helios_open_d3dkmt). */
#define IOCTL_HELIOS_CTX_CREATE   0x0022E400u
#define IOCTL_HELIOS_CTX_DESTROY  0x0022E404u
#define IOCTL_HELIOS_SUBMIT_VENUS 0x0022E409u /* METHOD_IN_DIRECT */
#define IOCTL_HELIOS_ALLOC_BLOB   0x0022E40Cu
#define IOCTL_HELIOS_MAP_BLOB     0x0022E410u
#define IOCTL_HELIOS_WAIT_FENCE   0x0022E414u
#define IOCTL_HELIOS_RELEASE_BLOB 0x0022E41Cu

/* ── Escape payload structs (protocol/src/escape.rs) — repr(C), padding-free ─── */
#define HELIOS_ESCAPE_MAGIC   0x48454C53u /* 'HELS' */
#define HELIOS_ESCAPE_VERSION 1u

#define HELIOS_ESCAPE_SUBMIT_VENUS 0x0001u
#define HELIOS_ESCAPE_CTX_CREATE   0x0002u
#define HELIOS_ESCAPE_CTX_DESTROY  0x0003u
#define HELIOS_ESCAPE_ALLOC_BLOB   0x0004u
#define HELIOS_ESCAPE_MAP_BLOB     0x0005u
#define HELIOS_ESCAPE_WAIT_FENCE   0x0006u
#define HELIOS_ESCAPE_RELEASE_BLOB 0x0008u
#define HELIOS_ESCAPE_ATTACH_RESOURCE 0x0009u
#define HELIOS_ESCAPE_REGISTER_FENCE_EVENT   0x000Bu
#define HELIOS_ESCAPE_UNREGISTER_FENCE_EVENT 0x000Cu
#define HELIOS_ESCAPE_QUERY_SCANOUT           0x000Du
#define HELIOS_ESCAPE_PRESENT_STREAM           0x0010u

#define HELIOS_PRESENT_STREAM_OP_REGISTER   1u
#define HELIOS_PRESENT_STREAM_OP_UNREGISTER 2u

/* helios_escape_fence_event.out_state values (protocol/src/escape.rs). */
#define HELIOS_FENCE_EVENT_REGISTERED       0u
#define HELIOS_FENCE_EVENT_ALREADY_COMPLETE 1u
#define HELIOS_FENCE_EVENT_PROBE_ACK        2u
#define HELIOS_FENCE_EVENT_CANCELLED        3u
#define HELIOS_FENCE_EVENT_NOT_FOUND        4u
/* Local sentinel: the escape itself failed (never a wire value). */
#define HELIOS_FENCE_EVENT_ESCAPE_FAILED    ~0u

#define HELIOS_MAP_CACHE_CACHED    0x00000001u
#define HELIOS_MAP_CACHE_UNCACHED  0x00000002u
#define HELIOS_MAP_CACHE_WC        0x00000003u

#define HELIOS_WDDM_MAGIC               0x4857444Du /* 'HWDM' */
#define HELIOS_WDDM_VERSION             1u
#define HELIOS_WDDM_ALLOC_KIND_DEVICE_MEMORY 1u
#define HELIOS_WDDM_ALLOC_KIND_TRACKING 3u
#define HELIOS_WDDM_BLOB_FLAG_NONLOCAL_TRACKING 0x40000000u
#define HELIOS_WDDM_IDENTITY_MAGIC      0x4849444Eu /* 'HIDN' */
#define HELIOS_WDDM_IDENTITY_VERSION    1u
#define HELIOS_STATUS_PENDING           ((NTSTATUS)0x00000103L)

#ifndef OBJ_INHERIT
#define OBJ_INHERIT 0x00000002L
#endif
#ifndef OBJ_CASE_INSENSITIVE
#define OBJ_CASE_INSENSITIVE 0x00000040L
#endif

/* virtio-gpu constants the backend needs (protocol/src/virtio_gpu.rs) */
#define VIRTIO_GPU_CAPSET_VENUS          4u
#define VIRTIO_GPU_BLOB_MEM_HOST3D       2u
#define VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE 1u
#define VIRTIO_GPU_BLOB_FLAG_USE_SHAREABLE 2u

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
   uint32_t present_value32; /* 0 = legacy; nonzero tags fence_id as cookie */
};

struct helios_escape_present_stream {
   struct helios_escape_header hdr;
   uint64_t cookie; /* out on REGISTER; exact in on UNREGISTER */
   uint32_t ctx_id;
   uint32_t op;
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

struct helios_escape_release_blob {
   struct helios_escape_header hdr;
   uint32_t ctx_id;
   uint32_t resource_id;
   uint32_t flags;
   uint32_t padding;
};

struct helios_escape_attach_resource {
   struct helios_escape_header hdr;
   uint32_t ctx_id;
   uint32_t resource_id;
};

struct helios_wddm_alloc_private {
   uint64_t blob_id;
   uint64_t size;
   uint32_t magic;
   uint32_t version;
   uint32_t blob_mem;
   uint32_t blob_flags;
   uint32_t ctx_id;
   uint32_t map_cache;
   uint32_t kind;
   uint32_t adopt_resource_id;
};

struct helios_wddm_alloc_meta {
   uint32_t width;
   uint32_t height;
   uint32_t format;
   uint32_t pitch;
   uint32_t bind_flags;
   uint32_t misc_flags;
   uint64_t venus_alloc_size;
   uint32_t memory_type_index;
   uint32_t dxgi_format;
   uint64_t plane_offset;
};

struct helios_wddm_open_identity {
   uint64_t venus_alloc_size;
   uint64_t blob_size;
   uint32_t magic;
   uint32_t version;
   uint32_t resource_id;
   uint32_t memory_type_index;
   uint32_t ctx_id;
   uint32_t kind;
   uint32_t reserved[2];
};

struct helios_wddm_external_private {
   struct helios_wddm_alloc_private alloc;
   struct helios_wddm_alloc_meta meta;
};

/* C3/M3.4 async transport: WAIT_FENCE v2 (40 bytes). `fence_id` is the WIRE
 * fence id the KMD wrote back into the SUBMIT_VENUS escape buffer. The caller
 * MUST pre-set out_completed = 1: the old synchronous KMD validates the shape
 * and returns without writing the buffer, and 1 then correctly reports
 * "complete" (its submits were host-complete by return). The async KMD blocks
 * (PASSIVE KEVENT) until completion or timeout_ns and writes the verdict. */
struct helios_escape_wait_fence {
   struct helios_escape_header hdr;
   uint64_t fence_id;
   uint64_t timeout_ns;
   uint32_t out_completed; /* out: 1 = complete, 0 = timed out */
   uint32_t _pad;
};

/* KMD 22.22.54+ usermode fence events (PSC WS2): REGISTER parks an event
 * handle for one-shot KeSetEvent at wire-fence retirement; the wait happens in
 * USERMODE (WaitForSingleObject), so no thread ever parks inside a blocking
 * escape and the dxgkrnl escape layer never convoys this process's
 * SUBMIT_VENUS escapes behind a wait (measured 24th session: 2.9 ms → µs).
 * UNREGISTER cancels after a usermode timeout; NOT_FOUND + a signaled event =
 * the retirement raced the timeout (complete); NOT_FOUND + unsignaled = the
 * registration was purged (transport teardown) — failure, never fake success.
 * REGISTER with fence_id == 0 && event_handle == 0 is the capability probe. */
struct helios_escape_fence_event {
   struct helios_escape_header hdr;
   uint64_t fence_id;     /* in: wire fence id */
   uint64_t event_handle; /* in: usermode event HANDLE, zero-extended */
   uint32_t out_state;    /* out: HELIOS_FENCE_EVENT_* */
   uint32_t _pad;
};

struct helios_escape_query_scanout {
   struct helios_escape_header hdr;
   uint64_t out_alloc_size;
   uint32_t out_resource_id;
   uint32_t out_width;
   uint32_t out_height;
   uint32_t out_dxgi_format;
   uint32_t out_pitch;
   uint32_t out_plane_offset;
   uint32_t out_memory_type_index;
   uint32_t out_generation;
   uint32_t reserved[2];
};

/* Stable export ABI used by the in-process Helios DXVK bridge. */
struct helios_venus_scanout_info {
   uint64_t alloc_size;
   uint32_t resource_id;
   uint32_t width;
   uint32_t height;
   uint32_t dxgi_format;
   uint32_t pitch;
   uint32_t plane_offset;
   uint32_t memory_type_index;
   uint32_t generation;
};

/* Wire-size guards mirroring protocol/src/escape.rs const _: () asserts. */
_Static_assert(sizeof(struct helios_escape_header) == 16, "hdr size");
_Static_assert(sizeof(struct helios_escape_ctx_create) == 24, "ctx_create size");
_Static_assert(sizeof(struct helios_escape_ctx_destroy) == 24, "ctx_destroy size");
_Static_assert(sizeof(struct helios_escape_submit_venus) == 40, "submit size");
_Static_assert(sizeof(struct helios_escape_present_stream) == 32,
               "present_stream size");
_Static_assert(sizeof(struct helios_escape_alloc_blob) == 48, "alloc_blob size");
_Static_assert(sizeof(struct helios_escape_map_blob) == 32, "map_blob size");
_Static_assert(sizeof(struct helios_escape_release_blob) == 32, "release_blob size");
_Static_assert(sizeof(struct helios_escape_attach_resource) == 24, "attach_resource size");
_Static_assert(sizeof(struct helios_escape_wait_fence) == 40, "wait_fence size");
_Static_assert(sizeof(struct helios_escape_fence_event) == 40, "fence_event size");
_Static_assert(sizeof(struct helios_escape_query_scanout) == 64, "query_scanout size");
_Static_assert(sizeof(struct helios_venus_scanout_info) == 40, "scanout_info size");
_Static_assert(sizeof(struct helios_wddm_alloc_private) == 48,
               "wddm alloc private size");
_Static_assert(sizeof(struct helios_wddm_alloc_meta) == 48,
               "wddm alloc meta size");
_Static_assert(sizeof(struct helios_wddm_open_identity) == 48,
               "wddm open identity size");
_Static_assert(sizeof(struct helios_wddm_external_private) == 96,
               "wddm external private size");

/* ── Backend private structs (vtest pattern: base is the first member) ──────── */

struct helios_shmem {
   struct vn_renderer_shmem base;
   uint32_t ctx_id;
};

struct helios_bo {
   struct vn_renderer_bo base;
   uint32_t ctx_id;
   uint32_t blob_flags;
   uint32_t map_cache;
   VkMemoryPropertyFlags memory_flags;
   bool resource_released;
};

struct vn_renderer_helios_external_memory {
   D3DKMT_HANDLE resource;
   D3DKMT_HANDLE allocation;
   HANDLE export_handle;
   bool export_handle_inheritable;
};

static bool
helios_nt_object_path(const WCHAR *name,
                      WCHAR *buf,
                      size_t buf_chars,
                      struct helios_unicode_string *out_us);

#define HELIOS_SYNC_PENDING_MAX 256
#define HELIOS_WAIT_FENCE_STACK_MAX 256

/* Upper bound on the venus ring_idx helios_venus_queue_gpu_fence will put on
 * the wire. It is EQUAL to helios_init_renderer_info's info->max_timeline_count
 * (64), which is the bound vn_instance_acquire_ring_idx enforces
 * (vn_instance.h:106) — so a ring_idx at or above this is not a live timeline
 * and is refused rather than submitted. 64 also stays inside the KMD's
 * `ring_idx.min(u8::MAX as u32) as u8` truncation (`enqueue_submit_inner`,
 * kmd_render gpu/mod.rs:3541), so a value that passes the bound check reaches
 * the wire unchanged.
 *
 * ⚠ This used to be `_SLOTS` and size a per-ring fence cache on `struct
 * helios`. That cache was deleted (see helios_venus_queue_gpu_fence): its key
 * could never match. The BOUND is the part that was load-bearing. */
#define HELIOS_QUEUE_GPU_FENCE_RING_LIMIT 64

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
   D3DKMT_HANDLE wddm_local;
   D3DKMT_HANDLE wddm_global;
   void *wddm_cpu_va;
   /* GPU-written vn feedback-slot counter of the owning EXPORTED timeline
    * semaphore (feedback-shadow retire, WS2). Written under dev_mutex by
    * vn_renderer_helios_sync_set_feedback; the retire thread re-loads it
    * every poll iteration WITHOUT the mutex (aligned pointer loads are
    * atomic; NULLed at semaphore destroy BEFORE the slot returns to the
    * feedback pool). NULL = observe via the wire fence as before. */
   const volatile uint64_t *feedback_counter;
   /* NT handle created by D3DKMTShareObjects with an object NAME (export
    * with VkExportSemaphoreWin32HandleInfoKHR::name). Held open so the name
    * stays resolvable for consumers; closed on final unref. */
   void *nt_named_handle;
   /* Reference count under dev_mutex: 1 for the vn_renderer_sync owner, +1 per
    * queued retire-thread entry. The WDDM handles close and the struct frees on
    * the LAST unref (a queued retire entry may legally outlive ops.destroy). */
   uint32_t refs;
};

/* One queued "signal the shared WDDM fence when this wire fence retires" work
 * item for the retire thread. Holds a reference on `sync`. */
struct helios_retire_entry {
   struct helios_retire_entry *next;
   struct helios_sync *sync;
   uint64_t fence_id;
   /* The sync value this entry's signal reaches — the feedback-shadow
    * retire polls the sync's feedback counter against it. */
   uint64_t val;
   /* QPC at enqueue (== the wire fence's submit, same call path) — the
    * retire thread computes submit→retirement-observed latency from it
    * (WS2 copy-latency decomposition). */
   int64_t enqueue_qpc;
};

enum helios_ioctl_stat {
   HELIOS_STAT_CTX_CREATE = 0,
   HELIOS_STAT_CTX_DESTROY,
   HELIOS_STAT_SUBMIT,
   HELIOS_STAT_ALLOC_BLOB,
   HELIOS_STAT_MAP_BLOB,
   HELIOS_STAT_RELEASE_BLOB,
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
   /* helios_submit sub-phases (24th session, the 2.9 ms win32-signal
    * renderer submit): dev_mutex acquisition wait vs the SUBMIT_VENUS
    * escape itself vs sync bookkeeping. QPC ticks. */
   int64_t submit_mutex_ticks;
   int64_t submit_escape_ticks;
   int64_t submit_sync_ticks;
   uint64_t wait_calls;
   uint64_t wait_fast;
   uint64_t wait_slow;
   uint64_t wait_timeout;
   /* Wire-fence submit→retirement-observed latency, measured by the retire
    * thread (external-sync signals only — one per vehicle/producer present).
    * Splits the bridge's copy-lat: a fat leg here = venus submit → host
    * completion → fence event; a thin leg = the delay is in guest dispatch
    * before submit or in the waiter hop after the WDDM signal. */
   uint64_t retire_lat_count;
   int64_t retire_lat_ticks;
   int64_t retire_lat_ticks_max;
   /* Distribution shape (µs buckets: <1000, <3000, <6000, <10000, <20000,
    * rest) — uniform-over-[0,10ms] indicts a host-side 10 ms fence poll;
    * a tight spike = a fixed pipeline delay. */
   uint64_t retire_lat_hist[6];
   /* Feedback-shadow retire outcomes: fast = completion observed via the
    * feedback slot; fallback = a slot existed but the poll budget expired
    * or the slot detached mid-poll (wire path served); wire = no slot
    * (non-timeline / gate off / import-only sync). */
   uint64_t retire_fb_fast;
   uint64_t retire_fb_fallback;
   uint64_t retire_fb_wire;
   uint64_t shmem_cache_hits;
   uint64_t shmem_creates;
   uint64_t bo_creates;
   uint64_t bo_maps;
   uint64_t bo_map_cached;
   uint64_t bo_map_wc;
   uint64_t bo_map_uncached;
   uint64_t bo_map_unknown;
};

/* One record per accepted REGISTER.  This lives under dev_mutex, alongside the
 * escape channel itself: that makes semaphore destruction and renderer/context
 * teardown race-free without a process/global identity heuristic. */
struct helios_present_stream {
   uint64_t cookie;
   struct helios_present_stream *next;
};

struct helios {
   struct vn_renderer base;

   struct vn_instance *instance;

   /* The device-interface handle + the lock serializing all IOCTLs on it.
    * MAP_BLOB in particular must be serialized and issued from the process that
    * opened the handle (this process) — see kmd/src/ioctl.rs::handle_map_blob. */
   mtx_t dev_mutex;
   /* Legacy System-class IOCTL handle. Gate 5a retires this transport: it stays
    * INVALID_HANDLE_VALUE so any not-yet-ported verb (submit/blob/wait via
    * helios_ioctl) fails cleanly until it moves onto the D3DKMT path. */
   HANDLE dev;

   /* WDDM D3DKMT handle set for the kmd_render adapter (Gate 5a). The adapter is
    * mandatory (carries the DxgkDdiEscape control channel); device/context are
    * best-effort here and become load-bearing for Stage 2/3 (CreateAllocation /
    * Render). */
   D3DKMT_HANDLE adapter;
   D3DKMT_HANDLE device;
   D3DKMT_HANDLE context;
   D3DKMT_HANDLE paging_queue;
   D3DKMT_HANDLE paging_sync;
   LUID adapter_luid;
   bool has_adapter_address;
   D3DKMT_ADAPTERADDRESS adapter_address;

   uint32_t ctx_id;
   uint64_t next_fence_id; /* monotonic, under dev_mutex */

   /* Reused D3DKMTEscape staging for helios_ioctl_submit_cs. dev_mutex
    * serializes every ops.submit, and the thunk consumes the private buffer
    * synchronously before returning.
    *
    * ⚠ THIS BUFFER IS THE REASON helios_ioctl_submit_cs REQUIRES THE MUTEX —
    * not any wire ordering. helios_venus_queue_gpu_fence deliberately does NOT
    * use it (it stages ~104 bytes on its own stack) precisely so it can issue
    * its escape with dev_mutex dropped; see the argument there. */
   uint8_t *submit_buf;
   size_t submit_buf_capacity;

   /* Retire thread (WS1 #4): a wire fence recorded on a SHARED sync (external
    * win32 semaphore) must signal the shared WDDM monitored fence when it
    * retires at host GPU completion — otherwise a cross-process consumer's
    * monitored-fence wait completes only if the PRODUCER happens to wait on or
    * read its own semaphore, which dwm's present path never does. The thread
    * blocks in WAIT_FENCE (PASSIVE, in the KMD) per queued entry, then marks
    * the fence on the sync (which signals the WDDM fence in retire order).
    * Started lazily on the first enqueue; joined in helios_destroy. */
   mtx_t retire_mutex;
   cnd_t retire_cond;
   thrd_t retire_thread;
   bool retire_thread_live;
   bool retire_stop;
   struct helios_retire_entry *retire_head;
   struct helios_retire_entry *retire_tail;
   uint32_t retire_depth;
   /* Manual-reset event SET by helios_destroy alongside retire_stop, so the
    * retire thread's event-path fence wait (WaitForMultipleObjects) unparks
    * promptly for the join. NULL when unavailable (fallback slice waits
    * re-check retire_stop each slice and need no event). */
   HANDLE retire_stop_event;

   /* KMD 22.22.54+ usermode fence events (REGISTER_FENCE_EVENT probe ack at
    * init). When false every wire-fence wait uses the blocking WAIT_FENCE
    * escape (correct, but parked escapes convoy this process's submits). */
   bool fence_events_supported;

   /* Registered monotonic present streams not yet explicitly unregistered.
    * Drained best-effort before CTX_DESTROY. */
   struct helios_present_stream *present_streams;

   struct vn_renderer_shmem_cache shmem_cache;
   struct helios_perf_stats perf;
};

/* Process-wide fence-event wait telemetry (printed by helios_perf_write; kept
 * as interlocked statics because the wait path deliberately holds NO lock). */
static volatile LONG helios_fence_event_waits;     /* event-path parks */
static volatile LONG helios_fence_event_immediate; /* ALREADY_COMPLETE */
static volatile LONG helios_fence_event_raced;     /* signal raced the timeout */
static volatile LONG helios_fence_event_timeouts;  /* deadline give-ups */
static volatile LONG helios_fence_event_fallbacks; /* event path refused →
                                                    * blocking-escape wait */
static volatile LONG helios_fence_event_lost;      /* NOT_FOUND + unsignaled
                                                    * (teardown purge) — loud */

/* GetModuleHandleExW(..._PIN) refusals — see helios_pin_module. Expected 0;
 * non-zero means the per-device handle leak below is still live. */
static volatile LONG helios_module_pin_failures;

/* Process-global state, audited 2026-07-06 (23rd session) for the two-live-
 * VkInstance shape the dcomp present vehicle introduces (a Vulkan app's own
 * instance + the vehicle's DXVK->ICD stack in the SAME process):
 *  - helios_perf_at_exit_renderer/_registered: perf-dump-only; last
 *    perf-enabled renderer wins the atexit dump. Harmless (HELIOS_PERF off by
 *    default), left as-is.
 *  - helios_current_ctx_id: last-writer-wins across instances, and destroy
 *    only clears it when it still matches the dying renderer. Ambiguous with
 *    two instances — bridge callers must use the handle-based
 *    helios_venus_instance_ctx_id() below; the process-global form stays for
 *    single-instance probes only. */
static struct helios *helios_perf_at_exit_renderer;
static bool helios_perf_at_exit_registered;
static uint32_t helios_current_ctx_id;
/* Private DLL exports are invoked directly rather than through Vulkan loader
 * dispatch, so an application VkInstance is a loader wrapper and cannot be
 * decoded with vn_instance_from_handle().  The bridge asks for the context on
 * the same thread that synchronously created its DxvkInstance; retain that
 * renderer identity per thread so a concurrent instance create cannot replace
 * it as it can with helios_current_ctx_id. */
static _Thread_local uint32_t helios_calling_thread_ctx_id;
/* The scanout query is called through a private DLL export, not through the
 * Vulkan loader dispatch table.  A VkInstance supplied by DXVK must therefore
 * never be decoded with vn_instance_from_handle here: if the export was found
 * in another loaded ICD image, or instance teardown raced the call, that turns
 * into an invalid renderer/dev_mutex pointer (DWM AV in mtx_lock).
 *
 * The query is adapter-global, so any live Helios renderer in this module is
 * sufficient.  Hold this process-wide lock across the short escape so destroy
 * cannot invalidate the selected renderer.  Last-created wins; if it dies
 * while an older renderer remains, queries fail cleanly until the next device
 * creation instead of guessing across instances. */
static SRWLOCK helios_current_renderer_lock = SRWLOCK_INIT;
static struct helios *helios_current_renderer;

/* Open a diag log under C:\ProgramData\Helios. The restricted IddCx host
 * process (which loads this ICD via DXVK to open the IDD swapchain surface)
 * cannot write C:\Windows\Temp, so its ICD diag lines vanished. ProgramData is
 * standard-user writable. Best effort: returns NULL on failure. */
static FILE *
helios_diag_fopen(const char *name)
{
   static bool dir_made = false;
   if (!dir_made) {
      CreateDirectoryA("C:\\ProgramData\\Helios", NULL);
      dir_made = true;
   }
   char path[MAX_PATH];
   snprintf(path, sizeof(path), "C:\\ProgramData\\Helios\\%s", name);
   return fopen(path, "a");
}

static void
helios_diag(const char *fmt, ...)
{
   FILE *f = helios_diag_fopen("helios_icd_diag.log");
   if (!f)
      return;

   fprintf(f, "%lld pid=%lu ", (long long)time(NULL),
           (unsigned long)GetCurrentProcessId());
   va_list ap;
   va_start(ap, fmt);
   vfprintf(f, fmt, ap);
   va_end(ap);
   fputc('\n', f);
   fclose(f);
}

/* Non-static diag entry for venus core code (vn_queue.c's forward-progress
 * deadline): dwm/WUDFHost stderr is invisible, so loss-latch decisions must
 * land in the ProgramData diag log or they can never be post-mortemed. */
void vn_renderer_helios_diag_log(const char *fmt, ...);

void
vn_renderer_helios_diag_log(const char *fmt, ...)
{
   FILE *f = helios_diag_fopen("helios_icd_diag.log");
   if (!f)
      return;

   fprintf(f, "%lld pid=%lu ", (long long)time(NULL),
           (unsigned long)GetCurrentProcessId());
   va_list ap;
   va_start(ap, fmt);
   vfprintf(f, fmt, ap);
   va_end(ap);
   fputc('\n', f);
   fclose(f);
}

__declspec(dllexport) uint32_t helios_venus_current_ctx_id(void);
__declspec(dllexport) uint32_t helios_venus_instance_ctx_id(VkInstance instance);
__declspec(dllexport) uint64_t helios_venus_memory_id(VkDeviceMemory memory);
__declspec(dllexport) uint32_t helios_venus_memory_res_id(VkDeviceMemory memory);
__declspec(dllexport) uint32_t helios_venus_memory_transfer_resource_ownership(VkDeviceMemory memory);
__declspec(dllexport) bool helios_venus_memory_alloc_info(VkDeviceMemory memory,
                                                          uint64_t *out_alloc_size,
                                                          uint32_t *out_memory_type_index);
__declspec(dllexport) bool helios_venus_memory_vidmm_tracked(VkDeviceMemory memory);
__declspec(dllexport) uint64_t helios_venus_memory_vidmm_global_identity(VkDeviceMemory memory);
__declspec(dllexport) bool helios_venus_memory_open_vidmm_tracker(VkDeviceMemory memory,
                                                                  uint64_t global_identity);
__declspec(dllexport) bool helios_venus_query_scanout(
   VkInstance instance, struct helios_venus_scanout_info *out_info);
__declspec(dllexport) bool helios_venus_queue_gpu_fence(VkQueue queue,
                                                        uint64_t *out_wire_fence);

__declspec(dllexport) uint32_t
helios_venus_current_ctx_id(void)
{
   return helios_current_ctx_id;
}

/* Context id for the renderer synchronously created by this calling thread.
 * `instance` is retained for the stable bridge ABI but is intentionally opaque:
 * this export is called directly, not through Vulkan loader dispatch, so the
 * supplied handle is the loader's wrapper rather than a vn_instance. */
__declspec(dllexport) uint32_t
helios_venus_instance_ctx_id(VkInstance instance)
{
   (void)instance;
   return helios_calling_thread_ctx_id;
}

__declspec(dllexport) uint64_t
helios_venus_memory_id(VkDeviceMemory memory)
{
   if (memory == VK_NULL_HANDLE)
      return 0;

   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   return mem->base.id;
}

__declspec(dllexport) uint32_t
helios_venus_memory_res_id(VkDeviceMemory memory)
{
   if (memory == VK_NULL_HANDLE)
      return 0;

   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   return mem->base_bo ? mem->base_bo->res_id : 0;
}

/* C1 allocation identity: the exact vkAllocateMemory parameters of this
 * memory object, recorded into the WDDM allocation's private-data trailer so
 * a cross-process opener imports the shared resource with the CREATOR's
 * allocation size and memory type (vkr's OPAQUE-fd import requires an
 * exact-size match; the host only accepts the exported handle in the
 * creator's memory type). Returns false for a null/unbacked memory. */
__declspec(dllexport) bool
helios_venus_memory_alloc_info(VkDeviceMemory memory,
                               uint64_t *out_alloc_size,
                               uint32_t *out_memory_type_index)
{
   if (out_alloc_size)
      *out_alloc_size = 0;
   if (out_memory_type_index)
      *out_memory_type_index = 0;
   if (memory == VK_NULL_HANDLE)
      return false;

   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   if (out_alloc_size)
      *out_alloc_size = mem->base.vk.size;
   if (out_memory_type_index)
      *out_memory_type_index = mem->base.vk.memory_type_index;
   return true;
}

/* Kept for old bridge DLLs, but deliberately never attest the legacy
 * process-local lifetime. They then retain the adopted allocation's full
 * conservative charge instead of recreating creator-exit under-reporting. */
__declspec(dllexport) bool
helios_venus_memory_vidmm_tracked(VkDeviceMemory memory)
{
   (void)memory;
   return false;
}

/* System-wide share of the exact tracker described above. A nonzero result is
 * stronger than the legacy boolean attestation: another process can open the
 * same WDDM allocation and keep its one VidMm charge alive after this process
 * releases the creator-side VkDeviceMemory. */
__declspec(dllexport) uint64_t
helios_venus_memory_vidmm_global_identity(VkDeviceMemory memory)
{
   if (memory == VK_NULL_HANDLE)
      return 0;

   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   return mem->helios_vidmm_resource && mem->helios_vidmm_allocation &&
          mem->helios_vidmm_global_share && mem->helios_vidmm_cookie
             ? ((uint64_t)mem->helios_vidmm_cookie << 32) |
                  mem->helios_vidmm_global_share
             : 0;
}

/* Retain a creator's shared VidMm tracker on an imported VkDeviceMemory. */
__declspec(dllexport) bool
helios_venus_memory_open_vidmm_tracker(VkDeviceMemory memory,
                                       uint64_t global_identity)
{
   const uint32_t global_share = (uint32_t)global_identity;
   const uint32_t expected_cookie = (uint32_t)(global_identity >> 32);
   if (memory == VK_NULL_HANDLE || !global_share || !expected_cookie)
      return false;

   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   struct vk_device *vk_dev = mem->base.vk.base.device;
   if (!vk_dev)
      return false;
   struct vn_device *dev = vn_device_from_vk(vk_dev);
   struct helios *helios = (struct helios *)dev->renderer;
   mtx_lock(&helios->dev_mutex);
   const bool already_complete =
      mem->helios_vidmm_resource && mem->helios_vidmm_allocation &&
      mem->helios_vidmm_global_share && mem->helios_vidmm_cookie;
   const bool already_partial =
      mem->helios_vidmm_resource || mem->helios_vidmm_allocation ||
      mem->helios_vidmm_global_share || mem->helios_vidmm_cookie;
   mtx_unlock(&helios->dev_mutex);
   if (already_complete || already_partial)
      return already_complete;

   const VkMemoryType *memory_type =
      &dev->physical_device->memory_properties
          .memoryTypes[mem->base.vk.memory_type_index];
   const VkMemoryHeap *memory_heap =
      &dev->physical_device->memory_properties
          .memoryHeaps[memory_type->heapIndex];
   const bool device_local =
      memory_heap->flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT;
   uint32_t resource = 0;
   uint32_t allocation = 0;
   uint32_t retained_share = global_share;
   uint32_t retained_cookie = expected_cookie;
   bool retained = vn_renderer_helios_vidmm_open_shared(
      dev->renderer, global_share, expected_cookie, mem->base.vk.size,
      device_local, &resource, &allocation);

   /* The creator can release its last tracker reference while this process is
    * opening the payload. Preserve accounting in that race by creating a new
    * full-size tracker in the imported memory's actual heap. Concurrent or
    * repeated fallback owners can conservatively over-report. */
   if (!retained) {
      retained = vn_renderer_helios_vidmm_alloc(
         dev->renderer, mem->base.vk.size, device_local, &resource,
         &allocation, &retained_share, &retained_cookie);
      if (!retained)
         return false;
      helios_diag("VidMm global tracker 0x%x unavailable; created fallback 0x%x",
                  global_share, retained_share);
   }

   /* The bridge normally calls once per freshly imported memory. Keep the
    * install serialized anyway: a duplicate concurrent call frees its local
    * reference instead of overwriting and leaking the winner's handles. */
   mtx_lock(&helios->dev_mutex);
   const bool empty = !mem->helios_vidmm_resource &&
                      !mem->helios_vidmm_allocation &&
                      !mem->helios_vidmm_global_share &&
                      !mem->helios_vidmm_cookie;
   if (empty) {
      mem->helios_vidmm_resource = resource;
      mem->helios_vidmm_allocation = allocation;
      mem->helios_vidmm_global_share = retained_share;
      mem->helios_vidmm_cookie = retained_cookie;
   }
   const bool complete = mem->helios_vidmm_resource &&
                         mem->helios_vidmm_allocation &&
                         mem->helios_vidmm_global_share &&
                         mem->helios_vidmm_cookie;
   mtx_unlock(&helios->dev_mutex);
   if (!empty)
      vn_renderer_helios_vidmm_free(dev->renderer, resource, allocation);
   return complete;
}

__declspec(dllexport) uint32_t
helios_venus_memory_transfer_resource_ownership(VkDeviceMemory memory)
{
   if (memory == VK_NULL_HANDLE)
      return 0;

   struct vn_device_memory *mem = vn_device_memory_from_handle(memory);
   if (!mem->base_bo || !mem->base_bo->res_id)
      return 0;

   struct helios_bo *bo = (struct helios_bo *)mem->base_bo;
   bo->resource_released = true;
   helios_diag("memory_transfer_resource_ownership mem=%p res=%u ctx=%u",
               (void *)mem, mem->base_bo->res_id, bo->ctx_id);
   return mem->base_bo->res_id;
}

static VkResult
helios_wddm_sync_create(struct vn_renderer *renderer,
                        uint64_t initial_val,
                        bool nt_shared,
                        D3DKMT_HANDLE *out_local,
                        D3DKMT_HANDLE *out_global,
                        void **out_cpu_va)
{
   struct helios *helios = (struct helios *)renderer;

   if (!helios->device)
      return VK_ERROR_INITIALIZATION_FAILED;

   D3DKMT_CREATESYNCHRONIZATIONOBJECT2 create;
   memset(&create, 0, sizeof(create));
   create.hDevice = helios->device;
   /*
    * WDDM-sync-redesign M1: the adapter is now raised to WDDM 3.2 + GpuMmu (GPU
    * virtual addressing), so dxgkrnl should accept D3DDDI_MONITORED_FENCE — which
    * exposes a CPU-readable fence-value VA + a GPU VA, the basis for a real
    * cross-process completion fence (CPU signal/wait verbs + a host VkSemaphore).
    * Try a monitored fence first; fall back to the legacy D3DDDI_FENCE (no CPU VA)
    * if the adapter still rejects it, so the stack keeps working either way.
    */
   /* Monitored fences can ONLY be NT-security-shared: dxgkrnl rejects
    * Shared=1 without NtSecuritySharing with 0xc000000d (proven live
    * 2026-07-06), i.e. there is no global/KMT DWORD flavor of a monitored
    * fence. Cross-process rendezvous without handle duplication therefore
    * uses NAMED NT sharing (vn_renderer_helios_sync_share_named /
    * D3DKMTOpenSyncObjectNtHandleFromName). nt_shared=false is kept only
    * for completeness and fails loudly below. */
   create.Info.Type = D3DDDI_MONITORED_FENCE;
   create.Info.Flags.Shared = 1;
   create.Info.Flags.NtSecuritySharing = nt_shared ? 1 : 0;
   create.Info.MonitoredFence.InitialFenceValue = initial_val;

   NTSTATUS st = D3DKMTCreateSynchronizationObject2(&create);
   if (st == 0) {
      *out_local = create.hSyncObject;
      *out_global = create.Info.SharedHandle;
      if (out_cpu_va)
         *out_cpu_va = create.Info.MonitoredFence.FenceValueCPUVirtualAddress;
      helios_diag(
         "sync_create ok MONITORED-fence local=0x%x shared=0x%x nt=%u initial=%llu cpuva=%p gpuva=0x%llx",
         (unsigned)*out_local, (unsigned)*out_global, nt_shared ? 1u : 0u,
         (unsigned long long)initial_val,
         create.Info.MonitoredFence.FenceValueCPUVirtualAddress,
         (unsigned long long)create.Info.MonitoredFence.FenceValueGPUVirtualAddress);
      return VK_SUCCESS;
   }
   /* NO legacy-D3DDDI_FENCE fallback: the retire thread signals and the
    * cross-process consumer waits via the FromCpu verbs, which require a
    * monitored fence. A legacy fence "succeeding" here produces a sync whose
    * waits hang forever (proven live 2026-07-06: monitored+Shared-without-
    * NtSecuritySharing is rejected 0xc000000d, the legacy fallback engaged,
    * and the KMT ring probe wedged in an unbounded kernel wait). Loud
    * failure over fake success. */
   helios_diag("sync_create MONITORED rejected status=0x%08x nt=%u dev=0x%x — refusing (no legacy fallback)",
               (unsigned)st, nt_shared ? 1u : 0u, (unsigned)helios->device);
   return VK_ERROR_OUT_OF_DEVICE_MEMORY;
}

static VkResult
helios_wddm_sync_open_kmt(struct vn_renderer *renderer,
                          D3DKMT_HANDLE global,
                          D3DKMT_HANDLE *out_local,
                          void **out_cpu_va)
{
   UNUSED struct helios *helios = (struct helios *)renderer;

   D3DKMT_OPENSYNCHRONIZATIONOBJECT open;
   memset(&open, 0, sizeof(open));
   open.hSharedHandle = global;

   const NTSTATUS st = D3DKMTOpenSynchronizationObject(&open);
   if (st != 0) {
      helios_diag("sync_open_kmt failed status=0x%08x global=0x%x",
                  (unsigned)st, (unsigned)global);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   *out_local = open.hSyncObject;
   if (out_cpu_va)
      *out_cpu_va = NULL;
   return VK_SUCCESS;
}

static VkResult
helios_wddm_sync_open_nt(struct vn_renderer *renderer,
                         void *nt_handle,
                         D3DKMT_HANDLE *out_local,
                         void **out_cpu_va)
{
   struct helios *helios = (struct helios *)renderer;

   D3DKMT_OPENSYNCOBJECTFROMNTHANDLE2 open2;
   memset(&open2, 0, sizeof(open2));
   open2.hNtHandle = nt_handle;
   open2.hDevice = helios->device;

   NTSTATUS st = D3DKMTOpenSyncObjectFromNtHandle2(&open2);
   if (st == 0) {
      *out_local = open2.hSyncObject;
      if (out_cpu_va)
         *out_cpu_va = open2.MonitoredFence.FenceValueCPUVirtualAddress;
      helios_diag("sync_open_nt2 ok local=0x%x handle=%p dev=0x%x cpu_va=%p gpu_va=0x%llx",
                  (unsigned)*out_local, nt_handle, (unsigned)helios->device,
                  open2.MonitoredFence.FenceValueCPUVirtualAddress,
                  (unsigned long long)open2.MonitoredFence.FenceValueGPUVirtualAddress);
      return VK_SUCCESS;
   }

   D3DKMT_OPENSYNCOBJECTFROMNTHANDLE open;
   memset(&open, 0, sizeof(open));
   open.hNtHandle = nt_handle;
   st = D3DKMTOpenSyncObjectFromNtHandle(&open);
   if (st != 0) {
      helios_diag("sync_open_nt failed status=0x%08x handle=%p dev=0x%x",
                  (unsigned)st, nt_handle, (unsigned)helios->device);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   *out_local = open.hSyncObject;
   if (out_cpu_va)
      *out_cpu_va = NULL;
   return VK_SUCCESS;
}

static VkResult
helios_wddm_sync_share_nt(struct vn_renderer *renderer,
                          D3DKMT_HANDLE local,
                          void **out_handle)
{
   UNUSED struct helios *helios = (struct helios *)renderer;
   HANDLE handle = NULL;
   D3DKMT_HANDLE object = local;
   OBJECT_ATTRIBUTES attr;
   memset(&attr, 0, sizeof(attr));
   attr.Length = sizeof(attr);

   const NTSTATUS st =
      D3DKMTShareObjects(1, &object, &attr, GENERIC_ALL, &handle);
   if (st != 0) {
      helios_diag("sync_share_nt failed status=0x%08x local=0x%x",
                  (unsigned)st, (unsigned)local);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   *out_handle = handle;
   return VK_SUCCESS;
}

static void
helios_wddm_sync_destroy(UNUSED struct vn_renderer *renderer,
                         D3DKMT_HANDLE local)
{
   if (!local)
      return;

   D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy;
   memset(&destroy, 0, sizeof(destroy));
   destroy.hSyncObject = local;
   (void)D3DKMTDestroySynchronizationObject(&destroy);
}

static VkResult
helios_wddm_sync_signal(struct vn_renderer *renderer,
                        D3DKMT_HANDLE local,
                        uint64_t value)
{
   struct helios *helios = (struct helios *)renderer;
   D3DKMT_HANDLE object = local;
   UINT64 fence_value = value;

   D3DKMT_SIGNALSYNCHRONIZATIONOBJECTFROMCPU cpu_signal;
   memset(&cpu_signal, 0, sizeof(cpu_signal));
   cpu_signal.hDevice = helios->device;
   cpu_signal.ObjectCount = 1;
   cpu_signal.ObjectHandleArray = &object;
   cpu_signal.FenceValueArray = &fence_value;

   NTSTATUS st = D3DKMTSignalSynchronizationObjectFromCpu(&cpu_signal);
   if (st == 0)
      return VK_SUCCESS;

   D3DKMT_SIGNALSYNCHRONIZATIONOBJECT2 signal;
   memset(&signal, 0, sizeof(signal));
   signal.hContext = helios->context;
   signal.ObjectCount = 1;
   signal.ObjectHandleArray[0] = object;
   signal.Fence.FenceValue = fence_value;

   st = D3DKMTSignalSynchronizationObject2(&signal);
   if (st != 0) {
      helios_diag("sync_signal failed status=0x%08x local=0x%x value=%llu dev=0x%x",
                  (unsigned)st, (unsigned)local,
                  (unsigned long long)value, (unsigned)helios->device);
      return VK_ERROR_DEVICE_LOST;
   }

   return VK_SUCCESS;
}

static DWORD
helios_timeout_ns_to_ms(uint64_t timeout)
{
   const uint64_t ns_per_ms = 1000ull * 1000ull;
   const uint64_t ms = timeout / ns_per_ms +
                       ((timeout % ns_per_ms) ? 1ull : 0ull);

   if (ms >= INFINITE)
      return INFINITE - 1;
   return (DWORD)ms;
}

static VkResult
helios_wddm_sync_wait(struct vn_renderer *renderer,
                      D3DKMT_HANDLE local,
                      uint64_t value,
                      uint64_t timeout)
{
   struct helios *helios = (struct helios *)renderer;
   D3DKMT_HANDLE object = local;
   UINT64 fence_value = value;

   D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU cpu_wait;
   memset(&cpu_wait, 0, sizeof(cpu_wait));
   cpu_wait.hDevice = helios->device;
   cpu_wait.ObjectCount = 1;
   cpu_wait.ObjectHandleArray = &object;
   cpu_wait.FenceValueArray = &fence_value;

   if (timeout == UINT64_MAX) {
      const NTSTATUS cpu_st = D3DKMTWaitForSynchronizationObjectFromCpu(&cpu_wait);
      if (cpu_st == 0)
         return VK_SUCCESS;

      D3DKMT_WAITFORSYNCHRONIZATIONOBJECT2 wait2;
      memset(&wait2, 0, sizeof(wait2));
      wait2.hContext = helios->context;
      wait2.ObjectCount = 1;
      wait2.ObjectHandleArray[0] = object;
      wait2.Fence.FenceValue = fence_value;

      const NTSTATUS st = D3DKMTWaitForSynchronizationObject2(&wait2);
      if (st != 0) {
         helios_diag("sync_wait2 failed status=0x%08x local=0x%x value=%llu dev=0x%x ctx=0x%x",
                     (unsigned)st, (unsigned)local,
                     (unsigned long long)value, (unsigned)helios->device,
                     (unsigned)helios->context);
         return VK_TIMEOUT;
      }

      return VK_SUCCESS;
   }

   HANDLE event = CreateEventW(NULL, TRUE, FALSE, NULL);
   if (!event)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   cpu_wait.hAsyncEvent = event;
   const NTSTATUS cpu_st = D3DKMTWaitForSynchronizationObjectFromCpu(&cpu_wait);
   if (cpu_st != 0) {
      CloseHandle(event);
      return VK_TIMEOUT;
   }

   const DWORD wait_ms = helios_timeout_ns_to_ms(timeout);
   const DWORD wr = WaitForSingleObject(event, wait_ms);
   CloseHandle(event);

   return wr == WAIT_OBJECT_0 ? VK_SUCCESS : VK_TIMEOUT;
}

/* Feedback-shadow retire gate: HELIOS_RETIRE_FEEDBACK absent or "1" = ON
 * (default), "0" = off. Gates BOTH the feedback-slot allocation for exported
 * timeline semaphores (vn_semaphore_feedback_init) and the retire thread's
 * feedback poll — off restores the pure wire-fence behavior. */
bool
vn_renderer_helios_retire_feedback_enabled(void)
{
   static int enabled = -1;
   if (enabled < 0) {
      char v[8];
      enabled = GetEnvironmentVariableA("HELIOS_RETIRE_FEEDBACK", v,
                                        sizeof(v)) && v[0] == '0'
                   ? 0
                   : 1;
   }
   return enabled == 1;
}

void
vn_renderer_helios_sync_set_feedback(struct vn_renderer *renderer,
                                     struct vn_renderer_sync *sync,
                                     const volatile uint64_t *counter_va)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_sync *hsync = (struct helios_sync *)sync;

   mtx_lock(&helios->dev_mutex);
   hsync->feedback_counter = counter_va;
   mtx_unlock(&helios->dev_mutex);
}

/* Opt-in gate for the per-op shmem/submit trace logs (HELIOS_SUBMIT_TRACE):
 * unconditional fopen+fprintf+fclose per submission/shmem-op measurably taxed
 * the hottest ICD paths (33 MB + 1.9 MB written in one desktop session,
 * PSC WS2 2026-07-05). Read once per process. */
static bool
helios_trace_io_enabled(void)
{
   static int enabled = -1;
   if (enabled < 0) {
      char v[8];
      enabled = GetEnvironmentVariableA("HELIOS_SUBMIT_TRACE", v, sizeof(v)) ? 1 : 0;
   }
   return enabled == 1;
}

static void
helios_trace_shmem(const char *event,
                   D3DKMT_HANDLE device,
                   D3DKMT_HANDLE context,
                   uint32_t ctx_id,
                   uint32_t res_id,
                   uint64_t size,
                   uint64_t user_va)
{
   if (!helios_trace_io_enabled())
      return;

   FILE *f = helios_diag_fopen("helios_icd_shmem.log");
   if (!f)
      return;

   fprintf(f, "%lld pid=%lu %s dev=0x%x kctx=0x%x ctx=%u res=%u size=%llu va=0x%llx\n",
           (long long)time(NULL), (unsigned long)GetCurrentProcessId(), event,
           (unsigned)device, (unsigned)context,
           ctx_id, res_id, (unsigned long long)size,
           (unsigned long long)user_va);
   fclose(f);
}

static void
helios_trace_submit(struct helios *helios,
                    const void *cs_data,
                    size_t cs_size,
                    uint32_t ring_idx,
                    uint64_t fence_id)
{
   if (!helios_trace_io_enabled())
      return;

   FILE *f = helios_diag_fopen("helios_icd_submit.log");
   if (!f)
      return;

   const uint32_t *words = (const uint32_t *)cs_data;
   const uint32_t w0 = cs_size >= 4 ? words[0] : 0;
   const uint32_t w1 = cs_size >= 8 ? words[1] : 0;
   const uint32_t w2 = cs_size >= 12 ? words[2] : 0;
   const uint32_t w3 = cs_size >= 16 ? words[3] : 0;
   fprintf(f,
           "%lld pid=%lu submit dev=0x%x kctx=0x%x ctx=%u fence=%llu ring=%u size=%llu words=%08x %08x %08x %08x\n",
           (long long)time(NULL), (unsigned long)GetCurrentProcessId(),
           (unsigned)helios->device, (unsigned)helios->context, helios->ctx_id,
           (unsigned long long)fence_id, ring_idx, (unsigned long long)cs_size,
           w0, w1, w2, w3);
   fclose(f);
}

static LONG CALLBACK
helios_vectored_exception_handler(PEXCEPTION_POINTERS ep)
{
   if (!ep || !ep->ExceptionRecord || !ep->ContextRecord ||
       ep->ExceptionRecord->ExceptionCode != EXCEPTION_ACCESS_VIOLATION)
      return EXCEPTION_CONTINUE_SEARCH;

   FILE *f = helios_diag_fopen("helios_icd_av.log");
   if (!f)
      return EXCEPTION_CONTINUE_SEARCH;

   const ULONG_PTR fault =
      ep->ExceptionRecord->NumberParameters >= 2
         ? ep->ExceptionRecord->ExceptionInformation[1]
         : 0;
   CONTEXT *c = ep->ContextRecord;
   fprintf(f,
           "%lld pid=%lu av code=0x%08lx ip=0x%llx fault=0x%llx "
           "rax=0x%llx rbx=0x%llx rcx=0x%llx rdx=0x%llx "
           "rsi=0x%llx rdi=0x%llx r8=0x%llx r9=0x%llx\n",
           (long long)time(NULL), (unsigned long)GetCurrentProcessId(),
           (unsigned long)ep->ExceptionRecord->ExceptionCode,
           (unsigned long long)c->Rip, (unsigned long long)fault,
           (unsigned long long)c->Rax, (unsigned long long)c->Rbx,
           (unsigned long long)c->Rcx, (unsigned long long)c->Rdx,
           (unsigned long long)c->Rsi, (unsigned long long)c->Rdi,
           (unsigned long long)c->R8, (unsigned long long)c->R9);
   fclose(f);
   return EXCEPTION_CONTINUE_SEARCH;
}

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
   case IOCTL_HELIOS_RELEASE_BLOB:
      return HELIOS_STAT_RELEASE_BLOB;
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

/* One D3DKMTEscape round-trip carrying a Helios escape struct as adapter-scoped
 * driver-private data. The escape buffer is in/out: the KMD's DxgkDdiEscape
 * validates the helios_escape_header and writes any out-fields (e.g. out_ctx_id)
 * back into the same buffer, which the runtime reflects to user mode. Returns
 * false on a D3DKMT failure.
 *
 * LOCKING. helios_escape_ex itself needs no lock: it reads only
 * helios->{adapter,device,context}, written at renderer create. Callers that
 * stage into helios->submit_buf, or that mutate renderer state across the
 * round-trip, hold dev_mutex for THAT reason. The callers which stage on their
 * own stack take no lock — helios_wait_fence_blocking, helios_escape_fence_event
 * and helios_submit_gpu_fence_cs.
 * ⚠ This used to read "Caller MUST hold dev_mutex (except WAIT_FENCE — see
 * helios_escape_no_hw)". There has never been a helios_escape_no_hw in this
 * file, and by then two callers already took no lock.
 *
 * `hardware_access` maps to D3DDDI_ESCAPEFLAGS.HardwareAccess. HardwareAccess=1
 * escapes serialize EXCLUSIVELY on the dxgkrnl adapter lock — that is the lock
 * the 2026-07-04 dump caught a WUDFHost device-create Escape queued behind for
 * 30+ s. A blocking WAIT_FENCE must therefore pass 0 (it touches no hardware —
 * the KMD parks the thread on a KEVENT) so it never convoys other escapes. */
static bool
helios_escape_no_adapter_sync_enabled(void)
{
   static int enabled = -1;
   if (enabled < 0) {
      char value[8];
      enabled = GetEnvironmentVariableA("HELIOS_ESCAPE_NO_ADAPTER_SYNC", value,
                                        sizeof(value)) &&
                       value[0] && value[0] != '0';
   }
   return enabled > 0;
}

static bool
helios_escape_ex(struct helios *helios, void *buf, uint32_t size, bool hardware_access)
{
   if (!helios->adapter) {
      helios_diag("escape skipped: no adapter size=%u", size);
      return false;
   }

   D3DKMT_ESCAPE esc;
   memset(&esc, 0, sizeof(esc));
   esc.hAdapter = helios->adapter; /* KMD escape is adapter-scoped (design §8.3) */
   esc.hDevice = helios->device;   /* pass the device too (some OS builds require it) */
   esc.hContext = helios->context;
   esc.Type = D3DKMT_ESCAPE_DRIVERPRIVATE;
   esc.Flags.HardwareAccess = hardware_access ? 1 : 0;
   /* Opt-in A/B only. Enabling this did not improve measured frame cadence and
    * a subsequent cold boot produced black output, so production behavior
    * keeps dxgkrnl's default adapter synchronization. */
   esc.Flags.NoAdapterSynchronization =
      !hardware_access && helios_escape_no_adapter_sync_enabled() ? 1 : 0;
   esc.pPrivateDriverData = buf;
   esc.PrivateDriverDataSize = size;

   const NTSTATUS st = D3DKMTEscape(&esc);
   if (st != 0) {
      vn_log(helios->instance, "Helios D3DKMTEscape failed: status 0x%08x",
             (unsigned)st);
      helios_diag("D3DKMTEscape failed status=0x%08x adapter=0x%x device=0x%x size=%u",
                  (unsigned)st, (unsigned)helios->adapter,
                  (unsigned)helios->device, size);
      fprintf(stderr, "HELIOS[gate5a]: D3DKMTEscape status=0x%08x (adapter=0x%x device=0x%x size=%u)\n",
              (unsigned)st, (unsigned)helios->adapter, (unsigned)helios->device, size);
      return false;
   }
   return true;
}

/* HardwareAccess for the default escape wrapper — now 0 (26th session).
 *
 * HardwareAccess=1 makes DxgkEscape take the dxgkrnl adapter CORE resource
 * EXCLUSIVE via DXGADAPTER::AcquireCoreResourceExclusive, which FIRST runs
 * DXGPROCESS::FlushAllDevice -> DXGDEVICE::FlushScheduler ->
 * dxgmms2!VidSchWaitForCompletionEvent — i.e. it WAITS FOR EVERY CONTEXT
 * QUEUE OF THE PROCESS TO DRAIN while holding the core resource. With the
 * kernel flip-wait parking a context queue on a monitored fence whose
 * signal comes from user mode, that is a guaranteed process-wide deadlock:
 * the escape holds the core resource and waits for the queue; every
 * D3DKMTSignalSynchronizationObjectFromCpu (dxvk fence waiter, flip-kwait
 * watchdog, the ICD retire thread) convoys SHARED behind it; the queue can
 * never drain (MEMORY.DMP 2026-07-07, !locks: one escape thread owning 3
 * ERESOURCEs exclusive, contention 26, VidSchWaitForCompletionEvent 3m13s;
 * same lock held a WUDFHost device-create escape 30+ s, 2026-07-04).
 *
 * Every Helios escape is software: the KMD stages the stream and rings the
 * virtio doorbell in its own BAR under its own spinlock — nothing needs
 * dxgkrnl's exclusive adapter serialization, and WAIT_FENCE + the
 * fence-event escapes have shipped with HardwareAccess=0 since the
 * 24th/25th sessions. `HELIOS_ESCAPE_HW=1` (env, read once) restores the
 * old exclusive behavior as a no-rebuild kill switch. */
static bool
helios_escape_hw_forced(void)
{
   static int forced = -1;
   if (forced < 0) {
      char value[8];
      forced = GetEnvironmentVariableA("HELIOS_ESCAPE_HW", value,
                                       sizeof(value)) &&
               value[0] && value[0] != '0';
      if (forced)
         helios_diag("escape: HELIOS_ESCAPE_HW=1 — exclusive-lock escapes "
                     "restored (flip-kwait deadlock class re-armed)");
   }
   return forced > 0;
}

static bool
helios_escape(struct helios *helios, void *buf, uint32_t size)
{
   return helios_escape_ex(helios, buf, size, helios_escape_hw_forced());
}

/* REGISTER/UNREGISTER are intentionally synchronous control-plane escapes.
 * Per-frame attribution remains on the already-existing SUBMIT_VENUS escape;
 * do not add a renderer batch or an escape to the frame path.  Callers hold
 * dev_mutex so the context id and the renderer-owned registration list are
 * stable for the full request. */
static bool
helios_ioctl_present_stream(struct helios *helios,
                            uint32_t op,
                            uint64_t *inout_cookie)
{
   if (!helios->ctx_id || !inout_cookie)
      return false;

   struct helios_escape_present_stream req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_PRESENT_STREAM, sizeof(req));
   req.cookie = *inout_cookie;
   req.ctx_id = helios->ctx_id;
   req.op = op;
   if (!helios_escape(helios, &req, sizeof(req)))
      return false;

   if (op == HELIOS_PRESENT_STREAM_OP_REGISTER && !req.cookie)
      return false;
   *inout_cookie = req.cookie;
   return true;
}

bool
vn_renderer_helios_present_stream_register(struct vn_renderer *renderer,
                                           uint64_t *out_cookie)
{
   if (out_cookie)
      *out_cookie = 0;
   if (!renderer || !out_cookie)
      return false;

   struct helios_present_stream *entry = calloc(1, sizeof(*entry));
   if (!entry)
      return false;

   struct helios *helios = (struct helios *)renderer;
   uint64_t cookie = 0;
   mtx_lock(&helios->dev_mutex);
   /* One Venus context has one UMD-created present timeline.  Reject a
    * different semaphore rather than trying to infer a winner or allowing
    * two cookies to make the marker's stream identity ambiguous.  Once its
    * VkSemaphore is destroyed, unregister clears this record and a later
    * explicit registration is possible. */
   const bool ok = !helios->present_streams &&
      helios_ioctl_present_stream(helios, HELIOS_PRESENT_STREAM_OP_REGISTER,
                                  &cookie);
   if (ok) {
      entry->cookie = cookie;
      entry->next = helios->present_streams;
      helios->present_streams = entry;
      *out_cookie = cookie;
   }
   mtx_unlock(&helios->dev_mutex);

   if (!ok)
      free(entry);
   return ok;
}

void
vn_renderer_helios_present_stream_unregister(struct vn_renderer *renderer,
                                             uint64_t cookie)
{
   if (!renderer || !cookie)
      return;

   struct helios *helios = (struct helios *)renderer;
   mtx_lock(&helios->dev_mutex);
   struct helios_present_stream **link = &helios->present_streams;
   while (*link && (*link)->cookie != cookie)
      link = &(*link)->next;
   if (*link) {
      struct helios_present_stream *entry = *link;
      *link = entry->next;
      /* Best effort: a context/device teardown may already have reclaimed the
       * stream.  The local record must still go away so no later teardown
       * guesses at a stale cookie. */
      (void)helios_ioctl_present_stream(
         helios, HELIOS_PRESENT_STREAM_OP_UNREGISTER, &cookie);
      free(entry);
   }
   mtx_unlock(&helios->dev_mutex);
}

static void
helios_present_stream_unregister_all_locked(struct helios *helios)
{
   while (helios->present_streams) {
      struct helios_present_stream *entry = helios->present_streams;
      helios->present_streams = entry->next;
      uint64_t cookie = entry->cookie;
      (void)helios_ioctl_present_stream(
         helios, HELIOS_PRESENT_STREAM_OP_UNREGISTER, &cookie);
      free(entry);
   }
}

/* Query the adapter-owned LINEAR VidPn primary through one live renderer.
 * `instance` is retained for the stable bridge ABI but is intentionally opaque
 * here; see helios_current_renderer above. */
__declspec(dllexport) bool
helios_venus_query_scanout(VkInstance instance,
                           struct helios_venus_scanout_info *out_info)
{
   if (!out_info)
      return false;
   memset(out_info, 0, sizeof(*out_info));
   (void)instance;

   AcquireSRWLockExclusive(&helios_current_renderer_lock);
   struct helios *helios = helios_current_renderer;
   if (!helios) {
      ReleaseSRWLockExclusive(&helios_current_renderer_lock);
      return false;
   }

   struct helios_escape_query_scanout req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_QUERY_SCANOUT, sizeof(req));
   mtx_lock(&helios->dev_mutex);
   const bool ok = helios_escape(helios, &req, sizeof(req));
   mtx_unlock(&helios->dev_mutex);
   ReleaseSRWLockExclusive(&helios_current_renderer_lock);
   if (!ok || !req.out_resource_id || !req.out_alloc_size ||
       !req.out_width || !req.out_height || !req.out_pitch)
      return false;

   out_info->alloc_size = req.out_alloc_size;
   out_info->resource_id = req.out_resource_id;
   out_info->width = req.out_width;
   out_info->height = req.out_height;
   out_info->dxgi_format = req.out_dxgi_format;
   out_info->pitch = req.out_pitch;
   out_info->plane_offset = req.out_plane_offset;
   out_info->memory_type_index = req.out_memory_type_index;
   out_info->generation = req.out_generation;
   helios_diag("query_scanout res=%u %ux%u pitch=%u off=%u alloc=%llu mti=%u gen=%u",
               out_info->resource_id, out_info->width, out_info->height,
               out_info->pitch, out_info->plane_offset,
               (unsigned long long)out_info->alloc_size,
               out_info->memory_type_index, out_info->generation);
   return true;
}

static bool
helios_ioctl_ctx_create(struct helios *helios, uint32_t capset_id, uint32_t *out_ctx_id)
{
   struct helios_escape_ctx_create req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_CTX_CREATE, sizeof(req));
   req.capset_id = capset_id;

   /* Escape is in/out: out_ctx_id comes back in the same buffer. */
   if (!helios_escape(helios, &req, sizeof(req)))
      return false;

   *out_ctx_id = req.out_ctx_id;
   return true;
}

static void
helios_ioctl_ctx_destroy(struct helios *helios, uint32_t ctx_id)
{
   struct helios_escape_ctx_destroy req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_CTX_DESTROY, sizeof(req));
   req.ctx_id = ctx_id;
   helios_escape(helios, &req, sizeof(req));
}

static void
helios_close_d3dkmt_handles(D3DKMT_HANDLE adapter,
                            D3DKMT_HANDLE device,
                            D3DKMT_HANDLE context)
{
   if (context) {
      D3DKMT_DESTROYCONTEXT dc;
      memset(&dc, 0, sizeof(dc));
      dc.hContext = context;
      (void)D3DKMTDestroyContext(&dc);
   }
   if (device) {
      D3DKMT_DESTROYDEVICE dd;
      memset(&dd, 0, sizeof(dd));
      dd.hDevice = device;
      (void)D3DKMTDestroyDevice(&dd);
   }
   if (adapter) {
      D3DKMT_CLOSEADAPTER ca;
      memset(&ca, 0, sizeof(ca));
      ca.hAdapter = adapter;
      (void)D3DKMTCloseAdapter(&ca);
   }
}

static bool
helios_probe_d3dkmt_adapter(struct helios *helios,
                            D3DKMT_HANDLE adapter,
                            D3DKMT_HANDLE *out_device,
                            D3DKMT_HANDLE *out_context)
{
   struct vn_instance *instance = helios->instance;
   D3DKMT_HANDLE old_adapter = helios->adapter;
   D3DKMT_HANDLE old_device = helios->device;
   D3DKMT_HANDLE old_context = helios->context;

   *out_device = 0;
   *out_context = 0;

   D3DKMT_CREATEDEVICE cd;
   memset(&cd, 0, sizeof(cd));
   cd.hAdapter = adapter;
   NTSTATUS st = D3DKMTCreateDevice(&cd);
   helios_diag("probe D3DKMTCreateDevice adapter=0x%x status=0x%08x hDevice=0x%x",
               (unsigned)adapter, (unsigned)st, (unsigned)cd.hDevice);
   if (st != 0) {
      vn_log(instance, "D3DKMTCreateDevice probe failed: status 0x%08x",
             (unsigned)st);
      return false;
   }
   *out_device = cd.hDevice;

   D3DKMT_CREATECONTEXT cc;
   memset(&cc, 0, sizeof(cc));
   cc.hDevice = cd.hDevice;
   cc.NodeOrdinal = 0;
   cc.EngineAffinity = 0;
   st = D3DKMTCreateContext(&cc);
   helios_diag("probe D3DKMTCreateContext device=0x%x status=0x%08x hContext=0x%x",
               (unsigned)cd.hDevice, (unsigned)st, (unsigned)cc.hContext);
   if (st == 0)
      *out_context = cc.hContext;
   else
      vn_log(instance, "D3DKMTCreateContext probe failed: status 0x%08x",
             (unsigned)st);

   helios->adapter = adapter;
   helios->device = *out_device;
   helios->context = *out_context;

   uint32_t probe_ctx_id = 0;
   const bool ok = helios_ioctl_ctx_create(helios, VIRTIO_GPU_CAPSET_VENUS,
                                           &probe_ctx_id) &&
                   probe_ctx_id != 0;
   helios_diag("probe CTX_CREATE adapter=0x%x device=0x%x context=0x%x ok=%u ctx_id=%u",
               (unsigned)adapter, (unsigned)*out_device,
               (unsigned)*out_context, ok ? 1u : 0u, probe_ctx_id);
   if (ok)
      helios_ioctl_ctx_destroy(helios, probe_ctx_id);

   helios->adapter = old_adapter;
   helios->device = old_device;
   helios->context = old_context;
   return ok;
}

/* SUBMIT_VENUS — ASYNC (C3/M3.4). Caller MUST hold dev_mutex (ordering). The
 * escape returns at QUEUE time; the KMD assigns a globally-unique WIRE fence id
 * and writes it back into the escape buffer's fence_id, which `*out_fence_id`
 * receives so the caller records it on the batch's syncs for ops.wait /
 * WAIT_FENCE. (Against a legacy synchronous KMD the field comes back unchanged
 * — our locally-assigned id — and its WAIT_FENCE no-op keeps the old
 * "submit returned ⇒ done" semantics.) */
static bool
helios_ioctl_submit_cs(struct helios *helios,
                       const void *cs_data,
                       size_t cs_size,
                       uint32_t ring_idx,
                       uint64_t present_cookie,
                       uint32_t present_value32,
                       uint64_t *out_fence_id)
{
   if (cs_size == 0 || cs_size > UINT32_MAX)
      return false;

   const uint64_t local_fence_id = ++helios->next_fence_id;
   const bool tagged_present = present_cookie && present_value32;

   struct helios_escape_submit_venus hdr = { 0 };
   helios_hdr_init(&hdr.hdr, HELIOS_ESCAPE_SUBMIT_VENUS, sizeof(hdr));
   /* The tagged form deliberately reuses the pre-existing 40-byte header:
    * cookie in fence_id on input, value32 in the former reserved word.  The
    * KMD overwrites fence_id with its ordinary fresh wire fence on return. */
   hdr.fence_id = tagged_present ? present_cookie : local_fence_id;
   hdr.ctx_id = helios->ctx_id;
   hdr.buffer_size = (uint32_t)cs_size;
   hdr.ring_idx = ring_idx;
   hdr.present_value32 = tagged_present ? present_value32 : 0;
   helios_trace_submit(helios, cs_data, cs_size, ring_idx, hdr.fence_id);

   /* Over D3DKMTEscape the venus stream rides INSIDE the escape buffer, directly
    * after the fixed header (the KMD reads it at buf[sizeof(hdr)..]); there is no
    * IN_DIRECT side buffer. Stage header+cs into one contiguous buffer. */
   const size_t total = sizeof(hdr) + cs_size;
   if (total > UINT32_MAX)
      return false;
   if (total > helios->submit_buf_capacity) {
      size_t capacity = helios->submit_buf_capacity
                           ? helios->submit_buf_capacity
                           : 4096;
      while (capacity < total) {
         if (capacity > UINT32_MAX / 2u) {
            capacity = total;
            break;
         }
         capacity *= 2;
      }
      uint8_t *grown = realloc(helios->submit_buf, capacity);
      if (!grown)
         return false;
      helios->submit_buf = grown;
      helios->submit_buf_capacity = capacity;
   }
   uint8_t *buf = helios->submit_buf;
   memcpy(buf, &hdr, sizeof(hdr));
   memcpy(buf + sizeof(hdr), cs_data, cs_size);
   const bool ok = helios_escape(helios, buf, (uint32_t)total);
   uint64_t wire_fence_id = local_fence_id;
   if (ok) {
      /* The KMD wrote the assigned wire fence id back into the header. */
      struct helios_escape_submit_venus out;
      memcpy(&out, buf, sizeof(out));
      if (out.fence_id)
         wire_fence_id = out.fence_id;
   }
   if (ok && out_fence_id)
      *out_fence_id = wire_fence_id;
   return ok;
}

/* ── D3D12 GPU-completion boundary — helios_venus_queue_gpu_fence ──────────── */

/* Log on a power-of-two decay (1st, 2nd, 4th, … occurrence). Deliberately a
 * second copy of vn_ring.c:117-125's helper, which is static there: the rule is
 * three tokens and exporting a diag helper out of vn_ring.c to share it would
 * be the larger change. */
static bool
helios_qgf_should_log(uint32_t n)
{
   return (n & (n - 1)) == 0;
}

/* Refusal/success accounting for the export below.
 *
 * ⚠ SAY IT OUT LOUD: these are PROCESS-LOCAL statics whose only sink is the
 * ProgramData diag log. That is a strictly WEAKER instrument than the named
 * registry counter every KMD-side refusal on this driver gets — it does not
 * survive the process, it cannot be read from another process, and it is
 * invisible to the gate scripts. The ICD has no registry-counter channel and
 * inventing one (a new escape) for a refusal path would dwarf the export.
 *
 * What IS observable adapter-side is the SUCCESS path, and it has TWO readings.
 * The better one is `EscSubRing` (ESCAPE_SUBMIT_RING_COUNT, kmd_render
 * virtio/ctrl.rs:1447-1449): it is incremented only on the escape path, so it
 * is guest-originated by construction, unlike the adapter-wide
 * RING_SUBMIT_COUNT/`RngSub` (gpu/mod.rs:3560) which three internal KMD
 * producers also reach (scanout/windowed-BLT copies use SCANOUT_RING_IDX = 1,
 * gpu/mod.rs:409).
 * ⚠ `EscSubRing` is still not specific to THIS export — venus's own external
 * semaphore-signal batches (vn_queue.c:1719-1734, vn_queue.c:3018-3033) are
 * ring>=1 escapes too — so attributing a delta to this export requires
 * differencing against an otherwise identical run with the caller's boundary
 * disabled (its gpu_wire_fence == 0 arm), not a single before/after read.
 *
 * ⚠ `helios_qgf_issued` HAD NO SINK AT ALL until the decayed success log below
 * was added: it was incremented and never printed, i.e. it was not an
 * instrument. The comment above claimed the diag log was the sink for all of
 * these; that was true only of the refusal counters. */
static uint32_t helios_qgf_refused_args;
static uint32_t helios_qgf_refused_object;
static uint32_t helios_qgf_refused_ring0;
static uint32_t helios_qgf_refused_state;
static uint32_t helios_qgf_refused_encode;
static uint32_t helios_qgf_refused_submit;
static uint32_t helios_qgf_refused_unassigned;
static uint32_t helios_qgf_issued;

/* Bound on the barrier cs. One vkWaitRingSeqnoMESA encodes far inside this;
 * it sizes BOTH the encoder's local buffer and the escape's stack staging, so
 * the two cannot drift apart. */
#define HELIOS_QGF_CS_MAX_BYTES 64u

/* Inbound fence_id for the barrier submit — see helios_submit_gpu_fence_cs. */
#define HELIOS_QGF_FENCE_SENTINEL UINT64_MAX

/* The wire-fence submit for the export below: ONE SUBMIT_VENUS escape staged
 * entirely on the STACK, so it needs no lock.
 *
 * ⛔ WHY NOT helios_ioctl_submit_cs. That function stages into the SHARED
 * helios->submit_buf and bumps the shared helios->next_fence_id, so its
 * contract is "caller holds dev_mutex" — and dev_mutex serialises EVERY venus
 * submit in the process. Holding it across this escape is defect A3
 * (docs/dx12/PENDING.md §1): on backpressure the KMD retries the enqueue in
 * 1 ms slices for up to ENQUEUE_RETRY_MAX_MS = 5_000 ms
 * (kmd_render/src/virtio/ctrl.rs:100, :1414-1464) before it will even report
 * QueueFull, and QueueFull is reachable at frame rate — CTRL_QUEUE_SIZE = 64
 * descriptors (gpu/mod.rs:83) with >= 2 per chain, while every fence minted
 * here stays in flight until host GPU COMPLETION by design. The symptom is a
 * process-wide venus stall and no error anywhere.
 *
 * WHAT dev_mutex ACTUALLY PROTECTS ON THE SUBMIT PATH, item by item, because
 * "(ordering)" in helios_ioctl_submit_cs's header is too vague to reason from:
 *   1. helios->submit_buf / submit_buf_capacity — reused, realloc'd staging.
 *      Not touched here: this stages on the stack.
 *   2. helios->next_fence_id — `++`. Not touched here: see the sentinel below.
 *   3. helios->{adapter,device,context,ctx_id} read by helios_escape_ex.
 *      Written at renderer create and by the init-time probe path
 *      (helios_probe_* — before any VkQueue exists); never on a live device.
 *      Their lifetime here is the CALLER's: umd12 holds the vkd3d
 *      ID3D12CommandQueue, hence the VkDevice, hence the VkInstance and its
 *      one `struct helios`, live across this call.
 *   4. Wire ORDER between this escape and any other SUBMIT_VENUS. NOT a
 *      requirement, and the barrier is why: the cs is a vkWaitRingSeqnoMESA
 *      that makes the host's ctx dispatch thread consume the guest ring up to
 *      `ring_seqno` BEFORE the fence attaches to the VkQueue. Whatever else is
 *      queued ahead of or behind us on that ctx cannot shrink the set covered
 *      by our seqno. Reordering can only ever make a fence cover MORE.
 *   5. Nothing about the seqno read. The venus ring is written under
 *      ring->mutex (vn_ring.c:493, :879, :926), never dev_mutex — so holding
 *      dev_mutex across the read never ordered us against a ring writer
 *      either.
 *
 * PRECEDENT, not novelty: helios_wait_fence_blocking and
 * helios_escape_fence_event already issue stack-staged escapes with no
 * dev_mutex, from arbitrary threads, concurrently with ops.submit.
 *
 * ⭐ LEGACY-KMD DETECTION IS EXACT HERE. The KMD ignores the inbound fence_id
 * whenever present_value32 == 0 (`escape_submit_venus`, kmd_render
 * ddi/escape.rs:1228-1259) and always writes its assigned wire fence back on
 * the success path, so an UNCHANGED sentinel means "this KMD assigned no wire
 * fence". UINT64_MAX is unreachable as a real id: they are
 * `1 + instance*2^32 + k` (NEXT_WIRE_FENCE_BASE = 1, WIRE_FENCE_INSTANCE_STRIDE
 * = 1 << 32 — gpu/mod.rs:1377, :1388), so aliasing needs 2^32 transport inits
 * AND 2^32 fences inside one of them. This replaces a comparison against
 * helios->next_fence_id which needed the mutex, could false-refuse on numeric
 * aliasing, and — the point — read shared state.
 *
 * Returns whether the ESCAPE itself succeeded. *out_fence_id receives whatever
 * the KMD left in the header — which is still HELIOS_QGF_FENCE_SENTINEL if it
 * assigned nothing, and 0 if the escape failed. Keeping the raw value is what
 * lets the caller keep "escape failed" and "no wire fence assigned" in separate
 * counters instead of collapsing them. */
static bool
helios_submit_gpu_fence_cs(struct helios *helios,
                           const void *cs_data,
                           uint32_t cs_size,
                           uint32_t ring_idx,
                           uint64_t *out_fence_id)
{
   *out_fence_id = 0;
   if (cs_size == 0 || cs_size > HELIOS_QGF_CS_MAX_BYTES)
      return false;

   struct helios_escape_submit_venus hdr = { 0 };
   helios_hdr_init(&hdr.hdr, HELIOS_ESCAPE_SUBMIT_VENUS, sizeof(hdr));
   hdr.fence_id = HELIOS_QGF_FENCE_SENTINEL;
   hdr.ctx_id = helios->ctx_id;
   hdr.buffer_size = cs_size;
   hdr.ring_idx = ring_idx;
   hdr.present_value32 = 0; /* untagged: the KMD ignores the inbound fence_id */

   /* header + barrier cs, contiguous. Over D3DKMTEscape the venus stream rides
    * INSIDE the escape buffer directly after the fixed header (the KMD reads it
    * at buf[sizeof(hdr)..]); there is no IN_DIRECT side buffer. */
   uint8_t buf[sizeof(struct helios_escape_submit_venus) +
               HELIOS_QGF_CS_MAX_BYTES];
   memcpy(buf, &hdr, sizeof(hdr));
   memcpy(buf + sizeof(hdr), cs_data, cs_size);

   const bool ok = helios_escape(helios, buf, (uint32_t)(sizeof(hdr) + cs_size));
   if (ok) {
      struct helios_escape_submit_venus out;
      memcpy(&out, buf, sizeof(out));
      *out_fence_id = out.fence_id;
   }

   /* Traced AFTER the escape, unlike helios_ioctl_submit_cs, so the line
    * carries the ASSIGNED id and correlates with the KMD's wire fence. A failed
    * escape traces fence=0; a KMD that assigned nothing traces the sentinel,
    * which is itself the recognisable signature of that case. */
   helios_trace_submit(helios, cs_data, cs_size, ring_idx, *out_fence_id);
   return ok;
}

/* Mint a wire fence that retires at host GPU COMPLETION of everything so far
 * submitted to `queue`, and hand back its id so a caller can order a WDDM DMA
 * packet behind real GPU work (protocol's HeliosD3D12SubmitCmd::gpu_wire_fence;
 * docs/dx12/KMD_IMPACT.md §14a).
 *
 * WHY THIS HAS TO EXIST — a ring-0 wire fence is not a GPU boundary, and for
 * vkd3d's D3D12 traffic it is not even a venus-decode boundary:
 *   - The KMD sets VIRTIO_GPU_FLAG_INFO_RING_IDX only for ring_idx != 0
 *     (`enqueue_submit_inner`, kmd_render gpu/mod.rs:3539-3542), so a ring-0
 *     submit takes QEMU's LEGACY fence branch (qemu-helios
 *     hw/display/virtio-gpu-virgl.c:1186) —
 *     virgl_renderer_create_fence, which IGNORES ctx_id and routes to
 *     vrend_renderer_create_ctx0_fence: a glFenceSync on the host GL context,
 *     disjoint from the venus context this driver actually uses.
 *   - And a plain vkQueueSubmit issues NO virtio submit at all. Its cs is
 *     written into the shared ring's memory (vn_ring.c:630-636); the only
 *     virtio traffic is the vkNotifyRingMESA doorbell, which is hardcoded to
 *     ring 0 (vn_renderer_util.h:26-35) and is suppressed outright while the
 *     host ring is not idle (vn_ring.c:672-689). A watermark of "every wire
 *     fence enqueued so far" can therefore be already satisfied at the instant
 *     of a D3D12 submission.
 * With ring_idx >= 1 the host instead attaches the fence to the VkQueue bound
 * to that timeline and signals it from its per-queue retire thread, which is
 * genuine GPU completion.
 *
 * CALLER CONTRACT — call this AFTER the submission to be covered has reached
 * the host driver, i.e. after the engine's own vkQueueSubmit returned (for
 * vkd3d: after the VKD3D_SUBMISSION_DRAIN handshake inside
 * vkd3d_acquire_vk_queue). The seqno read below is the ring's submitted-up-to
 * position AT CALL TIME; calling too early is the one way to get a fence that
 * covers less than the caller believes. Reading a LARGER seqno is harmless
 * (it only over-orders); reading a stale smaller one is the correctness
 * hazard, and only the caller's own ordering prevents it.
 *
 * COST, STATED PLAINLY: one SUBMIT_VENUS escape and one wire fence PER CALL.
 * There is no cache and no coalescing — see the argument at the submit below.
 * At 3 queues × 2 ExecuteCommandLists × 200 fps that is ~1200 in-flight-until-
 * GPU-completion fences per second, which is the pressure `PENDING.md` §4 names
 * as "per-fence completion unbatched at ~1800/s". If that becomes the limit, the
 * lever is rate-limiting in the CALLER (one boundary per frame) or a wider
 * descriptor budget in the KMD — not a seqno-keyed cache here, which cannot be
 * made sound.
 *
 * Every refusal returns false without touching the wire. *out_wire_fence is
 * always written (0 on failure) and 0 is a legal "no boundary" value for the
 * caller: failing here must stay SURVIVABLE, never fatal. */
__declspec(dllexport) bool
helios_venus_queue_gpu_fence(VkQueue queue, uint64_t *out_wire_fence)
{
   if (out_wire_fence)
      *out_wire_fence = 0;
   if (!queue || !out_wire_fence) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_args);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: null arg queue=%p out=%p (x%u)",
                     (void *)queue, (void *)out_wire_fence, n);
      return false;
   }

   /* HANDLE-CAST SAFETY. This export is resolved by DLL export name and called
    * DIRECTLY, not through Vulkan loader dispatch, so what arrives is whatever
    * the caller holds. That is decodable for a dispatchable VkQueue and is NOT
    * for a VkInstance, and the difference is structural rather than lucky:
    * mesa places VK_LOADER_DATA as the FIRST member of vk_object_base
    * (src/vulkan/runtime/vk_object.h:43-44) precisely so the loader can
    * overwrite that one word with its dispatch-table pointer IN PLACE — the
    * handle VALUE stays this driver's own object pointer. A VkInstance is by
    * contrast a genuine loader-owned wrapper and can never be decoded here,
    * which is why helios_venus_instance_ctx_id refuses to try (see its comment
    * above). helios_venus_register_present_stream already decodes a
    * directly-supplied VkDevice this way and is proven live.
    *
    * What no check here can defend against is a handle belonging to a
    * DIFFERENT ICD image in a multi-driver process: merely reading it is a
    * fault risk before any validation can run. So the checks below fail CLOSED
    * on everything still visible, and the real containment lives on the
    * consumer side — umd_common's find_venus_icd_module() selects ONE coherent
    * ICD module and resolves every export from that same module. */
   struct vn_queue *vnq = vn_queue_from_handle(queue);
   if (vnq->base.vk.base.type != VK_OBJECT_TYPE_QUEUE) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_object);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: queue %p is not a VkQueue "
                     "(type=%d) (x%u)",
                     (void *)queue, (int)vnq->base.vk.base.type, n);
      return false;
   }

   struct vk_device *dev_vk = vnq->base.vk.base.device;
   if (!dev_vk) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_object);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: queue %p has no device (x%u)",
                     (void *)queue, n);
      return false;
   }

   struct vn_device *dev = vn_device_from_handle(vk_device_to_handle(dev_vk));
   struct helios *helios = (struct helios *)dev->renderer;
   /* Two INDEPENDENT pointer paths must agree on the instance: dev->instance
    * (walked from the queue) and helios->instance (recorded at renderer
    * create). A foreign or freed handle does not survive that, and it is the
    * cheapest available proof that this really is one of THIS module's
    * renderers before we take its mutex — the failure mode the scanout
    * export's comment describes (an invalid renderer/dev_mutex pointer, DWM AV
    * inside mtx_lock). */
   if (dev->base.vk.base.type != VK_OBJECT_TYPE_DEVICE || !dev->primary_ring ||
       !helios || !dev->instance || helios->instance != dev->instance ||
       !helios->ctx_id) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_state);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: device/renderer state "
                     "dev=%p ring=%p renderer=%p inst=%p/%p ctx=%u (x%u)",
                     (void *)dev, (void *)dev->primary_ring, (void *)helios,
                     (void *)dev->instance,
                     (void *)(helios ? helios->instance : NULL),
                     helios ? helios->ctx_id : 0u, n);
      return false;
   }

   /* ⛔ REFUSE ring 0 UNCONDITIONALLY — this is the worst failure available on
    * this path, and it is silent. vkr_context_submit_fence fails the fence for
    * an unbound or out-of-range ring, so write_context_fence never fires, the
    * virtio ctrl response for our SUBMIT_3D is never written, and the KMD's
    * in-flight AsyncVenus entry becomes IMMORTAL: async_retired_up_to then
    * reports false for every watermark above it, DMA_COMPLETED is never
    * delivered again, and dxgkrnl resubmits into the same wedge — the TDR loop
    * documented on `latch_failed_and_fail_inflight` (kmd_render
    * gpu/mod.rs:3561-3579) — on an adapter-GLOBAL FIFO, i.e. it takes DWM
    * down with it.
    *
    * A live queue cannot actually be on ring 0 (vn_instance_acquire_ring_idx
    * never hands out 0 and asserts it, vn_instance.h:108-112), and a device
    * whose first queue could not acquire a timeline fails vkCreateDevice
    * outright. This is therefore defence in depth — written as real code and
    * not an assert(), because assert() is compiled out of the shipping ICD and
    * assurance that vanishes in release is not assurance. */
   const uint32_t ring_idx = vnq->ring_idx;
   if (ring_idx == 0 || ring_idx >= HELIOS_QUEUE_GPU_FENCE_RING_LIMIT) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_ring0);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: ring_idx=%u out of range "
                     "[1,%u) — a ring-0 fence would be a DECODE boundary and "
                     "an unbound ring would wedge the WDDM FIFO (x%u)",
                     ring_idx, (unsigned)HELIOS_QUEUE_GPU_FENCE_RING_LIMIT, n);
      return false;
   }

   /* The guest ring's submitted-up-to position. ⚠ RING-GLOBAL, not per-queue:
    * one primary ring carries every queue's submissions on this device
    * (dev->primary_ring is always instance->ring.ring — vn_device.c:513 — never
    * a TLS ring). That is SAFE and deliberate: this value is >= the seqno of
    * this queue's own last submission, so the barrier below over-orders (it
    * also waits for unrelated commands to be DISPATCHED, never for them to
    * complete) and can never under-order. Do NOT "fix" this into a per-queue
    * field expecting a tighter boundary — a tighter seqno would buy nothing,
    * because the ring>=1 fence covers the whole VkQueue regardless, and it
    * would reintroduce the race the barrier exists to close.
    *
    * 0 means a torn ring (vn_ring.c:373-383); it is also what a 32-bit byte
    * counter reads at the instant it wraps, so refusing here can cost one
    * fence per 4 GiB of ring traffic. Counted, and the caller's zero arm
    * absorbs it. */
   const uint32_t ring_seqno = vn_ring_current_seqno(dev->primary_ring);
   if (!ring_seqno) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_state);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: ring seqno 0 (torn ring or "
                     "seqno wrap) ring_idx=%u (x%u)", ring_idx, n);
      return false;
   }

   /* A NON-EMPTY cs is MANDATORY. helios_submit_gpu_fence_cs refuses cs_size
    * == 0, and the KMD's escape handler refuses a zero payload too
    * (`escape_submit_venus`, kmd_render ddi/escape.rs:1224-1226), so an empty
    * cs yields NO wire fence at all.
    *
    * vkWaitRingSeqnoMESA is not filler. Our cs is decoded on the host's
    * CONTEXT dispatch thread while the guest's vkQueueSubmit sits in the
    * shared ring consumed by a SEPARATE per-context ring thread; nothing
    * orders those two. Without the barrier the empty fence submit can reach
    * the VkQueue before the guest's real submit does, the host fence signals
    * against an empty queue, and DMA_COMPLETED claims work that has not run.
    * The host's dispatcher also calls vkr_ring_notify() on this command BEFORE
    * waiting, which is what makes the barrier safe against the suppressed
    * doorbell above: it wakes the very ring it is about to wait on, so a
    * parked ring cannot deadlock the wait. */
   uint32_t local_data[HELIOS_QGF_CS_MAX_BYTES / sizeof(uint32_t)];
   struct vn_cs_encoder local_enc =
      VN_CS_ENCODER_INITIALIZER_LOCAL(local_data, sizeof(local_data));
   vn_encode_vkWaitRingSeqnoMESA(&local_enc, 0,
                                 vn_ring_get_id(dev->primary_ring),
                                 ring_seqno);
   const size_t cs_size = vn_cs_encoder_get_len(&local_enc);
   if (cs_size == 0 || cs_size > sizeof(local_data)) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_encode);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: cs encode produced %llu bytes "
                     "(cap %llu) (x%u)",
                     (unsigned long long)cs_size,
                     (unsigned long long)sizeof(local_data), n);
      return false;
   }

   /* ⛔ NO LOCK IS TAKEN FROM HERE ON, and NO FENCE IS CACHED. Both are
    * deliberate; the full argument is on helios_submit_gpu_fence_cs.
    *
    * WHY THERE IS NO CACHE (defect A2, docs/dx12/PENDING.md §1). A per-timeline
    * cache keyed on `ring_seqno` used to live here, justified as sparing a
    * "submit-once-then-wait queue" a fence per ExecuteCommandLists. It could
    * never hit, and the key is why:
    *   - vn_ring_current_seqno returns the ring's shared TAIL (vn_ring.c:373-383)
    *     — a byte counter for the whole per-instance primary ring.
    *   - vkd3d_release_vk_queue issues a REAL vkQueueSubmit2 (one
    *     signalSemaphoreInfo, no command buffers — vkd3d-proton-helios
    *     libs/vkd3d/command.c:25591-25620) immediately AFTER the UMD samples
    *     this fence. Its cs is encoded into that same ring, advancing the tail.
    *   ⇒ ECL N+1 always reads a larger seqno than ECL N cached. Zero hits, ever.
    * ⛔ And the key cannot be made tolerant soundly. The tail is RING-GLOBAL, so
    * "advanced by only the release's empty batch" is indistinguishable from
    * "advanced by another VkQueue's real work on the same ring": accepting a
    * near-miss would return a fence that UNDER-orders, the one hazard this
    * export exists to prevent. The sound key would be a per-VkQueue submission
    * count, and venus core has none (struct vn_queue, vn_queue.h) — adding one
    * is a venus-core change, not an ICD-renderer change.
    * ⚠ Deleting it also removes a LATENT bug: ring_idx is recycled
    * (vn_instance_release_ring_idx, vn_instance.h:118-127) and nothing
    * invalidated the slot, so a hit condition that ever started working would
    * hand out a fence minted on a destroyed queue's timeline. */
   uint64_t fence_id = 0;
   if (!helios_submit_gpu_fence_cs(helios, local_data, (uint32_t)cs_size,
                                   ring_idx, &fence_id)) {
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_submit);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: SUBMIT_VENUS escape failed "
                     "ring_idx=%u seqno=%u (x%u)",
                     ring_idx, ring_seqno, n);
      return false;
   }
   if (fence_id == HELIOS_QGF_FENCE_SENTINEL || !fence_id) {
      /* The escape succeeded but the KMD wrote no wire fence back — a legacy
       * synchronous KMD. ⚠ It must NOT be papered over with a guest-local id:
       * a small local id can slip past the KMD's
       * `gpu_fence_id < next_wire_fence` validation and read as an
       * already-retired boundary — fake success, the one outcome this export
       * must never produce. Refuse instead; the caller's zero arm absorbs it. */
      const uint32_t n = p_atomic_inc_return(&helios_qgf_refused_unassigned);
      if (helios_qgf_should_log(n))
         helios_diag("queue_gpu_fence refused: this KMD assigned no wire fence "
                     "(header came back %s) — not a GPU boundary (x%u)",
                     fence_id ? "unchanged" : "zeroed", n);
      return false;
   }

   *out_wire_fence = fence_id;
   const uint32_t issued = p_atomic_inc_return(&helios_qgf_issued);
   /* The ONLY sink for helios_qgf_issued. Power-of-two decayed like every other
    * line here, so the cost is ~log2(N) lines for the WHOLE process — ~17 for a
    * minute at 1200 fences/s, then one more per doubling — and it prints the id
    * so a boundary can be correlated with the KMD's wire fence and with
    * helios_icd_submit.log. ⚠ Being a decayed log and not an accumulator, the
    * last line's `(xN)` is the count; do not read line COUNT as fence count. */
   if (helios_qgf_should_log(issued))
      helios_diag("queue_gpu_fence issued: ring_idx=%u seqno=%u fence=%llu (x%u)",
                  ring_idx, ring_seqno, (unsigned long long)fence_id, issued);
   return true;
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

   /* D3DKMTEscape writes the out-fields back into `req` in place. */
   if (!helios_escape(helios, &req, sizeof(req)))
      return 0;

   return req.out_resource_id;
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

   /* D3DKMTEscape writes the out-fields back into `req` in place. */
   if (!helios_escape(helios, &req, sizeof(req)))
      return 0;

   if (out_map_cache)
      *out_map_cache = req.map_cache;
   return req.out_user_va;
}

/* Blocking fence wait (C3/M3.4): blocks in the KMD (PASSIVE KEVENT) until the
 * wire fence completes on the virtio used ring or timeout_ns elapses;
 * timeout_ns==0 is a poll. Returns whether the fence is COMPLETE. Called
 * WITHOUT dev_mutex (waits must not block submits) and with HardwareAccess=0
 * (a blocking escape must never hold dxgkrnl's exclusive adapter lock — the
 * 30 s Escape-convoy mechanism). out_completed is pre-set to 1 so a legacy
 * synchronous KMD (which returns without writing the buffer) reads as complete
 * — matching its "submit returned ⇒ done" semantics.
 *
 * CONVOY WARNING (measured 24th session): even with HardwareAccess=0, a thread
 * PARKED inside this escape serializes the process's other escapes (submits)
 * at the dxgkrnl escape layer. This is therefore only the poll path + the
 * fallback for KMDs without fence events / refused registrations. */
static bool
helios_wait_fence_blocking(struct helios *helios, uint64_t fence_id, uint64_t timeout_ns)
{
   struct helios_escape_wait_fence req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_WAIT_FENCE, sizeof(req));
   req.fence_id = fence_id;
   req.timeout_ns = timeout_ns;
   req.out_completed = 1;
   if (!helios_escape_ex(helios, &req, sizeof(req), false))
      return false;
   return req.out_completed != 0;
}

/* ── usermode fence-event waits (KMD 22.22.54+, PSC WS2) ──────────────────────
 * register-event → WaitForSingleObject → cancel. The KMD's retirement DPC
 * KeSetEvents the registered event, so fence observation is interrupt-latency
 * (µs) instead of escape-park + slice-poll latency (ms), and NO thread ever
 * parks inside an escape — the escape-park submit convoy class is dead. */

/* Cap a single usermode event wait; mirrors the KMD's WAIT_FENCE_MAX_MS bound
 * so an event whose registration can never signal (transport death) parks a
 * thread no longer than the blocking path would have. */
#define HELIOS_EVENT_WAIT_MAX_NS (120ull * 1000 * 1000 * 1000)

/* One manual-reset event per waiting thread, created on demand, closed by the
 * tss destructor at thread exit. Reset before every registration (one-shot
 * KMD semantics leave it signaled after each completed wait). */
static once_flag helios_fence_event_tss_once = ONCE_FLAG_INIT;
static tss_t helios_fence_event_tss;
static bool helios_fence_event_tss_ok;

static void
helios_fence_event_tss_dtor(void *ev)
{
   if (ev)
      CloseHandle((HANDLE)ev);
}

static void
helios_fence_event_tss_init(void)
{
   helios_fence_event_tss_ok =
      tss_create(&helios_fence_event_tss, helios_fence_event_tss_dtor) ==
      thrd_success;
}

static HANDLE
helios_fence_event_get(void)
{
   call_once(&helios_fence_event_tss_once, helios_fence_event_tss_init);
   if (!helios_fence_event_tss_ok)
      return NULL;
   HANDLE ev = (HANDLE)tss_get(helios_fence_event_tss);
   if (!ev) {
      ev = CreateEventW(NULL, /*bManualReset*/ TRUE, FALSE, NULL);
      if (ev && tss_set(helios_fence_event_tss, ev) != thrd_success) {
         CloseHandle(ev);
         ev = NULL;
      }
   }
   return ev;
}

/* One REGISTER/UNREGISTER_FENCE_EVENT escape (non-blocking, HardwareAccess=0,
 * no dev_mutex — touches no context state). Returns the KMD's out_state, or
 * HELIOS_FENCE_EVENT_ESCAPE_FAILED if the escape itself failed. */
static uint32_t
helios_escape_fence_event(struct helios *helios,
                          uint32_t cmd_type,
                          uint64_t fence_id,
                          HANDLE event)
{
   struct helios_escape_fence_event req = { 0 };
   helios_hdr_init(&req.hdr, cmd_type, sizeof(req));
   req.fence_id = fence_id;
   req.event_handle = (uint64_t)(uintptr_t)event;
   req.out_state = HELIOS_FENCE_EVENT_ESCAPE_FAILED;
   if (!helios_escape_ex(helios, &req, sizeof(req), false))
      return HELIOS_FENCE_EVENT_ESCAPE_FAILED;
   return req.out_state;
}

/* Cancel a registration after a usermode timeout. Returns whether the fence
 * is COMPLETE (the retirement raced our timeout). Distinguishes the three
 * legal shapes loudly; a lost registration (purged unsignaled at transport
 * teardown) reports INCOMPLETE — the caller's own deadline semantics apply. */
static bool
helios_fence_event_cancel(struct helios *helios, uint64_t fence_id, HANDLE ev)
{
   const uint32_t un = helios_escape_fence_event(
      helios, HELIOS_ESCAPE_UNREGISTER_FENCE_EVENT, fence_id, ev);
   if (un == HELIOS_FENCE_EVENT_CANCELLED)
      return false; /* removed before signaling — a real timeout */
   if (un == HELIOS_FENCE_EVENT_NOT_FOUND) {
      if (WaitForSingleObject(ev, 0) == WAIT_OBJECT_0) {
         InterlockedIncrement(&helios_fence_event_raced);
         return true; /* the drain consumed it: signal raced the timeout */
      }
      InterlockedIncrement(&helios_fence_event_lost);
      helios_diag("fence-event registration LOST for wire fence %llu "
                  "(not found + unsignaled — transport teardown?)",
                  (unsigned long long)fence_id);
      return false;
   }
   /* The unregister escape failed. On a live transport this cannot happen
    * (the verb only takes the device lock); a dead transport never signals
    * (teardown derefs without signaling), so a later reuse of the per-thread
    * event cannot be woken by this stale registration. */
   InterlockedIncrement(&helios_fence_event_lost);
   helios_diag("fence-event UNREGISTER escape failed for wire fence %llu",
               (unsigned long long)fence_id);
   return false;
}

/* Event-path fence wait: register → WaitForSingleObject → cancel on timeout.
 * Falls back to the blocking escape wait if the event machinery is refused
 * (table full, no event, escape failure) — correct either way, counted. */
static bool
helios_event_wait_fence(struct helios *helios, uint64_t fence_id, uint64_t timeout_ns)
{
   HANDLE ev = helios_fence_event_get();
   if (!ev || !ResetEvent(ev)) {
      InterlockedIncrement(&helios_fence_event_fallbacks);
      return helios_wait_fence_blocking(helios, fence_id, timeout_ns);
   }

   const uint32_t state = helios_escape_fence_event(
      helios, HELIOS_ESCAPE_REGISTER_FENCE_EVENT, fence_id, ev);
   if (state == HELIOS_FENCE_EVENT_ALREADY_COMPLETE) {
      InterlockedIncrement(&helios_fence_event_immediate);
      return true;
   }
   if (state != HELIOS_FENCE_EVENT_REGISTERED) {
      /* Refused (table full / invalid / escape failure): blocking fallback. */
      InterlockedIncrement(&helios_fence_event_fallbacks);
      return helios_wait_fence_blocking(helios, fence_id, timeout_ns);
   }

   InterlockedIncrement(&helios_fence_event_waits);
   const uint64_t bounded_ns =
      timeout_ns < HELIOS_EVENT_WAIT_MAX_NS ? timeout_ns : HELIOS_EVENT_WAIT_MAX_NS;
   if (WaitForSingleObject(ev, helios_timeout_ns_to_ms(bounded_ns)) ==
       WAIT_OBJECT_0)
      return true;

   if (helios_fence_event_cancel(helios, fence_id, ev))
      return true;
   InterlockedIncrement(&helios_fence_event_timeouts);
   return false;
}

/* Wire-fence wait dispatcher. Polls (timeout_ns == 0) stay on the escape —
 * they never park, so they cannot convoy — as does everything when the KMD
 * lacks fence events (probed once at init; loud diag there). */
static bool
helios_ioctl_wait_fence(struct helios *helios, uint64_t fence_id, uint64_t timeout_ns)
{
   if (!helios->fence_events_supported || timeout_ns == 0)
      return helios_wait_fence_blocking(helios, fence_id, timeout_ns);
   return helios_event_wait_fence(helios, fence_id, timeout_ns);
}

/* Capability probe (init, once): REGISTER with fence_id == 0 && handle == 0.
 * A supporting KMD (22.22.54+) answers PROBE_ACK; older KMDs fail the escape
 * with STATUS_NOT_IMPLEMENTED. Quiet D3DKMTEscape (helios_escape_ex would log
 * an alarming failure line against every old KMD); exactly one diag line
 * either way. */
static void
helios_probe_fence_events(struct helios *helios)
{
   helios->fence_events_supported = false;

   struct helios_escape_fence_event req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_REGISTER_FENCE_EVENT, sizeof(req));
   req.out_state = HELIOS_FENCE_EVENT_ESCAPE_FAILED;

   D3DKMT_ESCAPE esc;
   memset(&esc, 0, sizeof(esc));
   esc.hAdapter = helios->adapter;
   esc.hDevice = helios->device;
   esc.hContext = helios->context;
   esc.Type = D3DKMT_ESCAPE_DRIVERPRIVATE;
   esc.pPrivateDriverData = &req;
   esc.PrivateDriverDataSize = sizeof(req);

   const NTSTATUS st = D3DKMTEscape(&esc);
   helios->fence_events_supported =
      st == 0 && req.out_state == HELIOS_FENCE_EVENT_PROBE_ACK;
   if (helios->fence_events_supported)
      helios_diag("fence-events: KMD probe ACK — usermode event waits live "
                  "(escape-park convoy path retired)");
   else
      helios_diag("fence-events: KMD UNSUPPORTED (status=0x%08x state=%u) — "
                  "blocking escape-wait fallback (convoy-prone)",
                  (unsigned)st, req.out_state);
}

static void
helios_ioctl_release_blob(struct helios *helios, uint32_t ctx_id, uint32_t resource_id)
{
   if (!ctx_id || !resource_id)
      return;

   char trace[8];
   if (GetEnvironmentVariableA("HELIOS_RELEASE_TRACE", trace, sizeof(trace)))
      fprintf(stderr, "Helios release_blob ctx=%u res=%u\n", ctx_id, resource_id);

   struct helios_escape_release_blob req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_RELEASE_BLOB, sizeof(req));
   req.ctx_id = ctx_id;
   req.resource_id = resource_id;
   helios_escape(helios, &req, sizeof(req));
}

static bool
helios_ioctl_attach_resource(struct helios *helios, uint32_t ctx_id, uint32_t resource_id)
{
   if (!ctx_id || !resource_id)
      return false;

   struct helios_escape_attach_resource req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_ATTACH_RESOURCE, sizeof(req));
   req.ctx_id = ctx_id;
   req.resource_id = resource_id;
   return helios_escape(helios, &req, sizeof(req));
}

static void
helios_sync_retire_locked(struct vn_renderer *renderer, struct helios_sync *sync)
{
   uint32_t n = 0;
   while (n < sync->pending_count && sync->pending[n].complete)
      n++;

   if (!n)
      return;

   const uint64_t val = sync->pending[n - 1].val;
   if (sync->val < val) {
      sync->val = val;
      if (sync->wddm_local)
         (void)helios_wddm_sync_signal(renderer, sync->wddm_local, sync->val);
   }

   sync->pending_count -= n;
   if (sync->pending_count) {
      memmove(sync->pending, sync->pending + n,
              sync->pending_count * sizeof(sync->pending[0]));
   }
}

static bool
helios_sync_append_locked(struct vn_renderer *renderer,
                          struct helios_sync *sync,
                          uint64_t val,
                          uint64_t fence_id)
{
   if (!fence_id) {
      /* No wire fence to order on (a sync-only batch). With nothing pending the
       * value advances immediately. With signal ops still in flight it must
       * QUEUE BEHIND them instead: the old behavior (advance + clear pendings)
       * blind-signaled the sync past unretired GPU work — an external consumer
       * reading the shared WDDM fence would then read stale pixels (the exact
       * early-signal poison WS1 #4 exists to kill). */
      if (!sync->pending_count) {
         if (sync->val < val) {
            sync->val = val;
            if (sync->wddm_local)
               (void)helios_wddm_sync_signal(renderer, sync->wddm_local,
                                             sync->val);
         }
         return true;
      }
      if (sync->pending_count >= HELIOS_SYNC_PENDING_MAX)
         return false;
      sync->pending[sync->pending_count++] = (struct helios_sync_pending) {
         .val = val,
         .fence_id = 0,
         .complete = true, /* retires with (not before) its predecessors */
      };
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
helios_sync_mark_fence_locked(struct vn_renderer *renderer,
                              struct helios_sync *sync,
                              uint64_t fence_id)
{
   for (uint32_t i = 0; i < sync->pending_count; i++) {
      if (sync->pending[i].fence_id == fence_id)
         sync->pending[i].complete = true;
   }
   helios_sync_retire_locked(renderer, sync);
}

/* ── external-sync retire thread ───────────────────────────────────────────── */

/* Per-fence retire deadline: far above any legitimate GPU work. On expiry the
 * WDDM fence is deliberately left UNSIGNALED (loud diag): a consumer timing
 * out on a real stall beats a consumer reading unfinished pixels.
 *
 * Event path (fence_events_supported): ONE WaitForMultipleObjects on
 * {fence event, retire_stop_event} bounded by HELIOS_RETIRE_DEADLINE_MS —
 * sane give-up math, prompt destroy-join, and zero escape parks.
 *
 * Fallback path (old KMD): bounded WAIT_FENCE slices. Slice length trades
 * give-up granularity against escape-park duration: a parked WAIT_FENCE
 * escape serializes against this process's SUBMIT_VENUS escapes at the
 * dxgkrnl escape layer (measured 24th session: Doom's win32-signal submit
 * escape averaged 2.9 ms behind the ICD1 retire thread's 250 ms parks;
 * dwm/sw-path submits without parked waits run at 3-5 µs). Short slices
 * bound the convoy; the retire deadline stays slices × slice = 60 s. */
#define HELIOS_RETIRE_DEADLINE_MS 60000
#define HELIOS_RETIRE_SLICE_NS (2ull * 1000 * 1000)
#define HELIOS_RETIRE_MAX_SLICES 30000 /* 60 s */

/* Outcome of the retire thread's event-path wait for one entry. */
enum helios_retire_wait {
   HELIOS_RETIRE_WAIT_COMPLETE,
   HELIOS_RETIRE_WAIT_TIMEOUT,  /* deadline expired — give up loudly */
   HELIOS_RETIRE_WAIT_STOPPED,  /* retire_stop_event — destroy join */
   HELIOS_RETIRE_WAIT_FALLBACK, /* event machinery refused — use slices */
};

static enum helios_retire_wait
helios_retire_event_wait(struct helios *helios, uint64_t fence_id)
{
   HANDLE ev = helios_fence_event_get();
   if (!ev || !helios->retire_stop_event || !ResetEvent(ev)) {
      InterlockedIncrement(&helios_fence_event_fallbacks);
      return HELIOS_RETIRE_WAIT_FALLBACK;
   }

   const uint32_t state = helios_escape_fence_event(
      helios, HELIOS_ESCAPE_REGISTER_FENCE_EVENT, fence_id, ev);
   if (state == HELIOS_FENCE_EVENT_ALREADY_COMPLETE) {
      InterlockedIncrement(&helios_fence_event_immediate);
      return HELIOS_RETIRE_WAIT_COMPLETE;
   }
   if (state != HELIOS_FENCE_EVENT_REGISTERED) {
      InterlockedIncrement(&helios_fence_event_fallbacks);
      return HELIOS_RETIRE_WAIT_FALLBACK;
   }

   InterlockedIncrement(&helios_fence_event_waits);
   const HANDLE handles[2] = { ev, helios->retire_stop_event };
   const DWORD wr =
      WaitForMultipleObjects(2, handles, FALSE, HELIOS_RETIRE_DEADLINE_MS);
   if (wr == WAIT_OBJECT_0)
      return HELIOS_RETIRE_WAIT_COMPLETE;

   /* Stop or deadline: cancel the registration either way (a completion that
    * raced in still reports COMPLETE so the sync is marked before exit). */
   const bool complete = helios_fence_event_cancel(helios, fence_id, ev);
   if (complete)
      return HELIOS_RETIRE_WAIT_COMPLETE;
   if (wr == WAIT_OBJECT_0 + 1)
      return HELIOS_RETIRE_WAIT_STOPPED;
   InterlockedIncrement(&helios_fence_event_timeouts);
   return HELIOS_RETIRE_WAIT_TIMEOUT;
}

/* Caller holds dev_mutex. Returns whether the struct must be freed (caller
 * frees OUTSIDE the lock). */
static bool
helios_sync_unref_locked(struct vn_renderer *renderer, struct helios_sync *sync)
{
   assert(sync->refs > 0);
   if (--sync->refs)
      return false;
   if (sync->wddm_local)
      helios_wddm_sync_destroy(renderer, sync->wddm_local);
   if (sync->nt_named_handle) {
      CloseHandle(sync->nt_named_handle);
      sync->nt_named_handle = NULL;
   }
   sync->wddm_local = 0;
   sync->wddm_global = 0;
   sync->wddm_cpu_va = NULL;
   return true;
}

static int
helios_sync_retire_thread(void *arg)
{
   struct helios *helios = arg;
   struct vn_renderer *renderer = &helios->base;

   u_thread_setname("helios-retire");

   mtx_lock(&helios->retire_mutex);
   while (true) {
      while (!helios->retire_stop && !helios->retire_head)
         cnd_wait(&helios->retire_cond, &helios->retire_mutex);
      if (helios->retire_stop && !helios->retire_head)
         break;

      struct helios_retire_entry *entry = helios->retire_head;
      helios->retire_head = entry->next;
      if (!helios->retire_head)
         helios->retire_tail = NULL;
      helios->retire_depth--;
      mtx_unlock(&helios->retire_mutex);

      bool complete = false;
      /* Feedback-shadow fast path (WS2 wire-fence latency workaround): the
       * exported semaphore's feedback slot is written BY THE GPU with the
       * signaled value as part of the same submission that signals it, in
       * host-coherent shmem — observable here sub-ms after completion,
       * bypassing the wire-fence response leg (measured 10-20 ms through
       * QEMU's fence delivery; Doom fps ceiling). Poll ladder: yield ~2 ms,
       * Sleep(0) to ~8 ms, then 1 ms sleeps to a 50 ms budget; on budget
       * expiry or slot detach (semaphore destroy) fall back to the wire
       * path below — never trust a stale pointer past one iteration. */
      int fb_outcome = 0; /* 0 = no slot, 1 = fast, 2 = fallback */
      if (!helios->retire_stop &&
          vn_renderer_helios_retire_feedback_enabled() &&
          entry->sync->feedback_counter) {
         LARGE_INTEGER t0, now, freq;
         QueryPerformanceFrequency(&freq);
         QueryPerformanceCounter(&t0);
         fb_outcome = 2;
         while (!helios->retire_stop) {
            const volatile uint64_t *fb = entry->sync->feedback_counter;
            if (!fb)
               break; /* detached mid-poll — wire path serves */
            if (*fb >= entry->val) {
               complete = true;
               fb_outcome = 1;
               break;
            }
            QueryPerformanceCounter(&now);
            const int64_t us =
               (now.QuadPart - t0.QuadPart) * 1000000 / freq.QuadPart;
            if (us > 50000)
               break; /* budget — wire path serves */
            if (us < 2000) {
               for (uint32_t p = 0; p < 64; p++)
                  YieldProcessor();
               SwitchToThread();
            } else if (us < 8000) {
               Sleep(0);
            } else {
               Sleep(1);
            }
         }
      }
      if (!complete && !helios->retire_stop) {
         bool handled = false;
         if (helios->fence_events_supported) {
            switch (helios_retire_event_wait(helios, entry->fence_id)) {
            case HELIOS_RETIRE_WAIT_COMPLETE:
               complete = true;
               handled = true;
               break;
            case HELIOS_RETIRE_WAIT_STOPPED:
               handled = true;
               break;
            case HELIOS_RETIRE_WAIT_TIMEOUT:
               handled = true;
               helios_diag("retire-thread GIVING UP on wire fence %llu after "
                           "%u ms event wait — shared sync stays UNSIGNALED "
                           "(sem=%p)",
                           (unsigned long long)entry->fence_id,
                           HELIOS_RETIRE_DEADLINE_MS, (void *)entry->sync);
               break;
            case HELIOS_RETIRE_WAIT_FALLBACK:
               break; /* slice loop below */
            }
         }
         if (!handled) {
            uint32_t slices = 0;
            while (!helios->retire_stop && slices < HELIOS_RETIRE_MAX_SLICES) {
               if (helios_wait_fence_blocking(helios, entry->fence_id,
                                              HELIOS_RETIRE_SLICE_NS)) {
                  complete = true;
                  break;
               }
               slices++;
            }
            if (!complete && !helios->retire_stop) {
               helios_diag("retire-thread GIVING UP on wire fence %llu after %u "
                           "slices — shared sync stays UNSIGNALED (sem=%p)",
                           (unsigned long long)entry->fence_id,
                           HELIOS_RETIRE_MAX_SLICES, (void *)entry->sync);
            }
         }
      }

      bool free_sync;
      mtx_lock(&helios->dev_mutex);
      if (helios->perf.enabled) {
         if (fb_outcome == 1)
            helios->perf.retire_fb_fast++;
         else if (fb_outcome == 2)
            helios->perf.retire_fb_fallback++;
         else
            helios->perf.retire_fb_wire++;
      }
      if (complete) {
         helios_sync_mark_fence_locked(renderer, entry->sync, entry->fence_id);
         if (helios->perf.enabled) {
            LARGE_INTEGER now;
            QueryPerformanceCounter(&now);
            const int64_t dt = now.QuadPart - entry->enqueue_qpc;
            helios->perf.retire_lat_count++;
            helios->perf.retire_lat_ticks += dt;
            if (dt > helios->perf.retire_lat_ticks_max)
               helios->perf.retire_lat_ticks_max = dt;
            const int64_t us = helios->perf.qpc_freq.QuadPart
               ? dt * 1000000 / helios->perf.qpc_freq.QuadPart
               : 0;
            const uint32_t bucket =
               us < 1000 ? 0 : us < 3000 ? 1 : us < 6000 ? 2 :
               us < 10000 ? 3 : us < 20000 ? 4 : 5;
            helios->perf.retire_lat_hist[bucket]++;
         }
      }
      free_sync = helios_sync_unref_locked(renderer, entry->sync);
      mtx_unlock(&helios->dev_mutex);
      if (free_sync)
         free(entry->sync);
      free(entry);

      mtx_lock(&helios->retire_mutex);
   }
   mtx_unlock(&helios->retire_mutex);
   return 0;
}

/* Queue "signal the shared WDDM fence when `fence_id` retires" for `sync`.
 * Caller holds dev_mutex (takes retire_mutex inside; the retire thread never
 * holds retire_mutex while taking dev_mutex, so the order is deadlock-free).
 * Takes a reference on `sync`. */
static bool
helios_retire_enqueue_locked(struct helios *helios,
                             struct helios_sync *sync,
                             uint64_t fence_id,
                             uint64_t val)
{
   struct helios_retire_entry *entry = malloc(sizeof(*entry));
   if (!entry)
      return false;
   entry->next = NULL;
   entry->sync = sync;
   entry->fence_id = fence_id;
   entry->val = val;
   {
      LARGE_INTEGER now;
      QueryPerformanceCounter(&now);
      entry->enqueue_qpc = now.QuadPart;
   }

   mtx_lock(&helios->retire_mutex);
   if (!helios->retire_thread_live) {
      if (u_thread_create(&helios->retire_thread, helios_sync_retire_thread,
                          helios) != thrd_success) {
         mtx_unlock(&helios->retire_mutex);
         free(entry);
         helios_diag("retire-thread creation FAILED — external sync %p cannot "
                     "signal at GPU completion", (void *)sync);
         return false;
      }
      helios->retire_thread_live = true;
   }
   sync->refs++;
   if (helios->retire_tail)
      helios->retire_tail->next = entry;
   else
      helios->retire_head = entry;
   helios->retire_tail = entry;
   helios->retire_depth++;
   if (helios->retire_depth > 256 && !(helios->retire_depth & 0x3F))
      helios_diag("retire queue depth %u — external signals outrunning host "
                  "completion", helios->retire_depth);
   cnd_signal(&helios->retire_cond);
   mtx_unlock(&helios->retire_mutex);
   return true;
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
      "release_blob",
      "wait_fence",
   };

   fprintf(f, "Helios perf summary pid=%lu epoch=%lld (%s)\n",
           (unsigned long)GetCurrentProcessId(), (long long)time(NULL),
           final ? "final" : "live");
   fprintf(f,
           "submit_calls=%llu batches=%llu empty_batches=%llu syncs=%llu cs_bytes=%llu\n",
           (unsigned long long)helios->perf.submit_calls,
           (unsigned long long)helios->perf.submit_batches,
           (unsigned long long)helios->perf.submit_empty_batches,
           (unsigned long long)helios->perf.submit_syncs,
           (unsigned long long)helios->perf.submit_cs_bytes);
   if (helios->perf.submit_calls) {
      fprintf(f,
              "submit_phases mutex_avg_us=%.3f escape_avg_us=%.3f sync_avg_us=%.3f\n",
              helios_perf_ms(helios, helios->perf.submit_mutex_ticks) * 1000.0 /
                 (double)helios->perf.submit_calls,
              helios_perf_ms(helios, helios->perf.submit_escape_ticks) * 1000.0 /
                 (double)helios->perf.submit_calls,
              helios_perf_ms(helios, helios->perf.submit_sync_ticks) * 1000.0 /
                 (double)helios->perf.submit_calls);
   }
   fprintf(f, "wait_calls=%llu fast=%llu slow=%llu timeout=%llu\n",
           (unsigned long long)helios->perf.wait_calls,
           (unsigned long long)helios->perf.wait_fast,
           (unsigned long long)helios->perf.wait_slow,
           (unsigned long long)helios->perf.wait_timeout);
   if (helios->perf.retire_lat_count) {
      fprintf(f,
              "retire_lat n=%llu avg_us=%.1f max_us=%.1f "
              "hist_ms[<1,1-3,3-6,6-10,10-20,20+]=%llu/%llu/%llu/%llu/%llu/%llu\n",
              (unsigned long long)helios->perf.retire_lat_count,
              helios_perf_ms(helios, helios->perf.retire_lat_ticks) * 1000.0 /
                 (double)helios->perf.retire_lat_count,
              helios_perf_ms(helios, helios->perf.retire_lat_ticks_max) *
                 1000.0,
              (unsigned long long)helios->perf.retire_lat_hist[0],
              (unsigned long long)helios->perf.retire_lat_hist[1],
              (unsigned long long)helios->perf.retire_lat_hist[2],
              (unsigned long long)helios->perf.retire_lat_hist[3],
              (unsigned long long)helios->perf.retire_lat_hist[4],
              (unsigned long long)helios->perf.retire_lat_hist[5]);
      fprintf(f, "retire_fb fast=%llu fallback=%llu wire=%llu\n",
              (unsigned long long)helios->perf.retire_fb_fast,
              (unsigned long long)helios->perf.retire_fb_fallback,
              (unsigned long long)helios->perf.retire_fb_wire);
   }
   fprintf(f,
           "fence_events supported=%d waits=%ld imm=%ld raced=%ld timeouts=%ld "
           "fallbacks=%ld lost=%ld\n",
           helios->fence_events_supported ? 1 : 0, helios_fence_event_waits,
           helios_fence_event_immediate, helios_fence_event_raced,
           helios_fence_event_timeouts, helios_fence_event_fallbacks,
           helios_fence_event_lost);
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

/* ── Adapter discovery + open (Gate 5a: WDDM D3DKMT, kmd_render) ────────────── */

/* Enumerate WDDM adapters via the KMT thunks (no DXGI/COM needed in C), match
 * the Helios render adapter by its registry description, open it, and create a
 * device + context on it. On success fills helios->{adapter,device,context,
 * adapter_luid}. The adapter handle carries the DxgkDdiEscape control channel
 * the venus context rides; device/context are created here for the Stage 2/3
 * CreateAllocation / Render path (best-effort for Stage 1). */
static bool
helios_open_d3dkmt(struct helios *helios)
{
   struct vn_instance *instance = helios->instance;
   helios_diag("helios_open_d3dkmt enter");

   D3DKMT_ENUMADAPTERS2 ea;
   memset(&ea, 0, sizeof(ea));
   /* First call with NULL pAdapters returns the adapter count. */
   NTSTATUS st = D3DKMTEnumAdapters2(&ea);
   helios_diag("D3DKMTEnumAdapters2 count status=0x%08x count=%u",
               (unsigned)st, (unsigned)ea.NumAdapters);
   if (st != 0 || ea.NumAdapters == 0) {
      vn_log(instance, "D3DKMTEnumAdapters2 count failed: status 0x%08x count %u",
             (unsigned)st, (unsigned)ea.NumAdapters);
      return false;
   }
   ea.pAdapters = calloc(ea.NumAdapters, sizeof(D3DKMT_ADAPTERINFO));
   if (!ea.pAdapters)
      return false;
   st = D3DKMTEnumAdapters2(&ea);
   helios_diag("D3DKMTEnumAdapters2 list status=0x%08x count=%u",
               (unsigned)st, (unsigned)ea.NumAdapters);
   if (st != 0) {
      vn_log(instance, "D3DKMTEnumAdapters2 failed: status 0x%08x", (unsigned)st);
      free(ea.pAdapters);
      return false;
   }

   D3DKMT_HANDLE chosen_adapter = 0;
   D3DKMT_HANDLE chosen_device = 0;
   D3DKMT_HANDLE chosen_context = 0;
   LUID chosen_luid = { 0 };
   for (UINT i = 0; i < ea.NumAdapters; i++) {
      const D3DKMT_HANDLE h = ea.pAdapters[i].hAdapter;

      D3DKMT_ADAPTERREGISTRYINFO reg;
      memset(&reg, 0, sizeof(reg));
      D3DKMT_QUERYADAPTERINFO qai;
      memset(&qai, 0, sizeof(qai));
      qai.hAdapter = h;
      qai.Type = KMTQAITYPE_ADAPTERREGISTRYINFO;
      qai.pPrivateDriverData = &reg;
      qai.PrivateDriverDataSize = sizeof(reg);

      const NTSTATUS qst = D3DKMTQueryAdapterInfo(&qai);
      const bool query_ok = qst == 0;
      const bool virtio_match = query_ok &&
                                wcsstr(reg.AdapterString, L"VIRTIO GPU") != NULL;
      const bool helios_match = query_ok &&
                                wcsstr(reg.AdapterString, L"Helios") != NULL;
      const bool name_match = helios_match || virtio_match;
      /* The CTX_CREATE probe is the authoritative Helios discriminator: it runs
       * the Helios-private D3DKMTEscape handshake that only the Helios KMD
       * answers (foreign adapters cleanly reject it). The registry AdapterString
       * query is only a hint and is unreliable on some boots — it has been
       * observed returning STATUS_OBJECT_NAME_NOT_FOUND (0xc0000034) for every
       * adapter. So probe name-identified candidates first, and fall back to
       * probing every adapter when the name query failed; never gate discovery
       * solely on a fragile registry string. */
      const bool try_candidate = chosen_adapter == 0 && (name_match || !query_ok);
      helios_diag("adapter[%u] h=0x%x query=0x%08x name='%ls' match=%u virtio=%u helios=%u try=%u luid=%08lx:%08lx",
                  i, (unsigned)h, (unsigned)qst,
                  qst == 0 ? reg.AdapterString : L"<query-failed>",
                  name_match ? 1u : 0u, virtio_match ? 1u : 0u,
                  helios_match ? 1u : 0u, try_candidate ? 1u : 0u,
                  (unsigned long)ea.pAdapters[i].AdapterLuid.HighPart,
                  (unsigned long)ea.pAdapters[i].AdapterLuid.LowPart);

      if (try_candidate) {
         D3DKMT_HANDLE device = 0;
         D3DKMT_HANDLE context = 0;
         if (helios_probe_d3dkmt_adapter(helios, h, &device, &context)) {
            chosen_adapter = h;
            chosen_device = device;
            chosen_context = context;
            chosen_luid = ea.pAdapters[i].AdapterLuid;
            helios_diag("adapter[%u] selected after CTX_CREATE probe", i);
         } else {
            helios_diag("adapter[%u] rejected by CTX_CREATE probe", i);
            helios_close_d3dkmt_handles(h, device, context);
         }
      }

      if (h != chosen_adapter) {
         /* Release adapters we won't use (D3DKMTEnumAdapters2 opens each one). */
         if (try_candidate) {
            /* Probe rejection already closed the candidate handle set. */
         } else {
            D3DKMT_CLOSEADAPTER ca;
            memset(&ca, 0, sizeof(ca));
            ca.hAdapter = h;
            (void)D3DKMTCloseAdapter(&ca);
         }
      } else {
         helios_diag("adapter[%u] keeping selected handle 0x%x", i, (unsigned)h);
      }
   }
   free(ea.pAdapters);

   if (chosen_adapter == 0) {
      vn_log(instance, "no Helios WDDM adapter passed D3DKMT CTX_CREATE probe");
      helios_diag("no Helios adapter passed CTX_CREATE probe");
      return false;
   }
   helios->adapter = chosen_adapter;
   helios->device = chosen_device;
   helios->context = chosen_context;
   helios->adapter_luid = chosen_luid;

   /* A renderer physical device reports the host GPU's PCI address.  That
    * address is not part of the guest topology and cannot be used to match
    * this Vulkan device with the WDDM adapter.  Query the guest address from
    * Dxgkrnl so VK_EXT_pci_bus_info (and OpenCL implementations layered on
    * Vulkan) identify the same adapter as DXGI.  Render-only adapters use the
    * WDDM 2.4 query; retain the legacy query as a compatibility fallback. */
   const KMTQUERYADAPTERINFOTYPE address_types[] = {
      KMTQAITYPE_ADAPTERADDRESS_RENDER,
      KMTQAITYPE_ADAPTERADDRESS,
   };
   for (uint32_t i = 0; i < ARRAY_SIZE(address_types); i++) {
      D3DKMT_ADAPTERADDRESS address;
      memset(&address, 0, sizeof(address));
      D3DKMT_QUERYADAPTERINFO address_query;
      memset(&address_query, 0, sizeof(address_query));
      address_query.hAdapter = chosen_adapter;
      address_query.Type = address_types[i];
      address_query.pPrivateDriverData = &address;
      address_query.PrivateDriverDataSize = sizeof(address);
      st = D3DKMTQueryAdapterInfo(&address_query);
      helios_diag("adapter address query type=%u status=0x%08x",
                  (unsigned)address_types[i], (unsigned)st);
      if (st == 0) {
         helios->adapter_address = address;
         helios->has_adapter_address = true;
         helios_diag("guest adapter PCI address 0000:%02x:%02x.%u",
                     (unsigned)address.BusNumber,
                     (unsigned)address.DeviceNumber,
                     (unsigned)address.FunctionNumber);
         break;
      }
   }

   D3DKMT_CREATEPAGINGQUEUE create_queue;
   memset(&create_queue, 0, sizeof(create_queue));
   create_queue.hDevice = helios->device;
   create_queue.Priority = D3DDDI_PAGINGQUEUE_PRIORITY_NORMAL;
   st = D3DKMTCreatePagingQueue(&create_queue);
   if (st == 0) {
      helios->paging_queue = create_queue.hPagingQueue;
      helios->paging_sync = create_queue.hSyncObject;
   } else {
      /* Accounting is diagnostic metadata, never a reason to make Vulkan
       * initialization fail against an older KMD/OS. */
      helios_diag("D3DKMTCreatePagingQueue failed status=0x%08x; "
                  "VidMm usage tracking disabled",
                  (unsigned)st);
   }
   /* Gate 5a bring-up breadcrumb (stderr): adapter opened by LUID. */
   fprintf(stderr, "HELIOS[gate5a]: opened Helios WDDM adapter hAdapter=0x%x luid=%08lx:%08lx\n",
           (unsigned)chosen_adapter, (unsigned long)chosen_luid.HighPart,
           (unsigned long)chosen_luid.LowPart);

   fprintf(stderr, "HELIOS[gate5a]: D3DKMT device=0x%x context=0x%x\n",
           (unsigned)helios->device, (unsigned)helios->context);
   return true;
}

static void
helios_vidmm_evict_locked(struct helios *helios,
                          D3DKMT_HANDLE allocation_handle)
{
   if (allocation_handle && helios->device) {
      D3DKMT_EVICT evict;
      memset(&evict, 0, sizeof(evict));
      evict.hDevice = helios->device;
      evict.NumAllocations = 1;
      evict.AllocationList = &allocation_handle;
      const NTSTATUS status = D3DKMTEvict(&evict);
      if (status != 0)
         helios_diag("VidMm Evict failed status=0x%08x", (unsigned)status);
   }
}

static void
helios_vidmm_destroy_locked(struct helios *helios,
                            D3DKMT_HANDLE resource_handle,
                            D3DKMT_HANDLE allocation_handle)
{
   if (helios->device && (resource_handle || allocation_handle)) {
      D3DKMT_DESTROYALLOCATION destroy;
      memset(&destroy, 0, sizeof(destroy));
      destroy.hDevice = helios->device;
      if (resource_handle) {
         destroy.hResource = resource_handle;
      } else {
         destroy.phAllocationList = &allocation_handle;
         destroy.AllocationCount = 1;
      }
      const NTSTATUS status = D3DKMTDestroyAllocation(&destroy);
      if (status != 0)
         helios_diag("VidMm DestroyAllocation failed status=0x%08x",
                     (unsigned)status);
   }
}

enum helios_vidmm_residency {
   HELIOS_VIDMM_NOT_ACQUIRED,
   HELIOS_VIDMM_RESIDENT,
   HELIOS_VIDMM_RESIDENT_WAIT_FAILED,
};

static enum helios_vidmm_residency
helios_vidmm_make_resident_locked(struct helios *helios,
                                  D3DKMT_HANDLE allocation,
                                  uint64_t size)
{
   UINT priority = D3DDDI_ALLOCATIONPRIORITY_MAXIMUM;
   D3DDDI_MAKERESIDENT resident;
   memset(&resident, 0, sizeof(resident));
   resident.hPagingQueue = helios->paging_queue;
   resident.NumAllocations = 1;
   resident.AllocationList = &allocation;
   resident.PriorityList = &priority;
   NTSTATUS st = D3DKMTMakeResident(&resident);
   if (st != 0 && st != HELIOS_STATUS_PENDING) {
      helios_diag("VidMm MakeResident size=%llu failed status=0x%08x",
                  (unsigned long long)size, (unsigned)st);
      return HELIOS_VIDMM_NOT_ACQUIRED;
   }

   if (resident.PagingFenceValue) {
      UINT64 value = resident.PagingFenceValue;
      D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU wait;
      memset(&wait, 0, sizeof(wait));
      wait.hDevice = helios->device;
      wait.ObjectCount = 1;
      wait.ObjectHandleArray = &helios->paging_sync;
      wait.FenceValueArray = &value;
      st = D3DKMTWaitForSynchronizationObjectFromCpu(&wait);
      if (st != 0) {
         helios_diag("VidMm paging wait size=%llu failed status=0x%08x",
                     (unsigned long long)size, (unsigned)st);
         return HELIOS_VIDMM_RESIDENT_WAIT_FAILED;
      }
   }
   return HELIOS_VIDMM_RESIDENT;
}

static uint32_t
helios_vidmm_new_cookie(struct helios *helios)
{
   static volatile LONG serial;
   LARGE_INTEGER counter;
   QueryPerformanceCounter(&counter);
   uint64_t mixed = (uint64_t)counter.QuadPart ^
                    ((uint64_t)GetCurrentProcessId() << 32) ^
                    (uintptr_t)helios ^ (uint32_t)InterlockedIncrement(&serial);
   mixed ^= mixed >> 33;
   mixed *= UINT64_C(0xff51afd7ed558ccd);
   mixed ^= mixed >> 33;
   uint32_t cookie = (uint32_t)mixed;
   return cookie ? cookie : 1;
}

bool
vn_renderer_helios_vidmm_alloc(struct vn_renderer *renderer,
                               uint64_t size,
                               bool device_local,
                               uint32_t *resource_handle,
                               uint32_t *allocation_handle,
                               uint32_t *global_share,
                               uint32_t *tracker_cookie)
{
   struct helios *helios = (struct helios *)renderer;
   if (!resource_handle || !allocation_handle || !global_share ||
       !tracker_cookie)
      return false;
   *resource_handle = 0;
   *allocation_handle = 0;
   *global_share = 0;
   *tracker_cookie = 0;

   if (!size || !helios->device || !helios->ctx_id ||
       !helios->paging_queue || !helios->paging_sync)
      return false;

   struct helios_wddm_alloc_private private_data;
   memset(&private_data, 0, sizeof(private_data));
   private_data.size = size;
   private_data.magic = HELIOS_WDDM_MAGIC;
   private_data.version = HELIOS_WDDM_VERSION;
   private_data.ctx_id = helios->ctx_id;
   private_data.kind = HELIOS_WDDM_ALLOC_KIND_TRACKING;
   private_data.map_cache = helios_vidmm_new_cookie(helios);
   if (!device_local)
      private_data.blob_flags = HELIOS_WDDM_BLOB_FLAG_NONLOCAL_TRACKING;

   D3DDDI_ALLOCATIONINFO2 allocation_info;
   memset(&allocation_info, 0, sizeof(allocation_info));
   allocation_info.pPrivateDriverData = &private_data;
   allocation_info.PrivateDriverDataSize = sizeof(private_data);

   D3DKMT_CREATEALLOCATION create;
   memset(&create, 0, sizeof(create));
   create.hDevice = helios->device;
   create.Flags.CreateResource = 1;
   /* A legacy KMT global share names one WDDM resource system-wide. Importers
    * open this same tracker rather than creating another full-size charge. */
   create.Flags.CreateShared = 1;
   create.Flags.AllowNotZeroed = 1;
   create.NumAllocations = 1;
   create.pAllocationInfo2 = &allocation_info;

   mtx_lock(&helios->dev_mutex);
   NTSTATUS st = D3DKMTCreateAllocation2(&create);
   if (st != 0) {
      mtx_unlock(&helios->dev_mutex);
      helios_diag("VidMm CreateAllocation2 size=%llu failed status=0x%08x",
                  (unsigned long long)size, (unsigned)st);
      return false;
   }

   D3DKMT_HANDLE allocation = allocation_info.hAllocation;
   enum helios_vidmm_residency residency = HELIOS_VIDMM_NOT_ACQUIRED;
   if (create.hResource && allocation && create.hGlobalShare)
      residency = helios_vidmm_make_resident_locked(helios, allocation, size);
   if (!create.hResource || !allocation || !create.hGlobalShare ||
       residency != HELIOS_VIDMM_RESIDENT) {
      if (residency != HELIOS_VIDMM_NOT_ACQUIRED)
         helios_vidmm_evict_locked(helios, allocation);
      helios_vidmm_destroy_locked(helios, create.hResource, allocation);
      mtx_unlock(&helios->dev_mutex);
      helios_diag("VidMm shared tracker invalid resource=0x%x allocation=0x%x global=0x%x",
                  (unsigned)create.hResource, (unsigned)allocation,
                  (unsigned)create.hGlobalShare);
      return false;
   }

   *resource_handle = create.hResource;
   *allocation_handle = allocation;
   *global_share = create.hGlobalShare;
   *tracker_cookie = private_data.map_cache;
   mtx_unlock(&helios->dev_mutex);
   return true;
}

#define HELIOS_VIDMM_PRIVATE_DATA_LIMIT (1024u * 1024u)

bool
vn_renderer_helios_vidmm_open_shared(struct vn_renderer *renderer,
                                     uint32_t global_share,
                                     uint32_t expected_cookie,
                                     uint64_t expected_size,
                                     bool expected_device_local,
                                     uint32_t *resource_handle,
                                     uint32_t *allocation_handle)
{
   struct helios *helios = (struct helios *)renderer;
   if (!global_share || !expected_cookie || !expected_size ||
       !resource_handle || !allocation_handle)
      return false;
   *resource_handle = 0;
   *allocation_handle = 0;
   if (!helios->device || !helios->paging_queue || !helios->paging_sync)
      return false;

   D3DKMT_QUERYRESOURCEINFO query;
   memset(&query, 0, sizeof(query));
   query.hDevice = helios->device;
   query.hGlobalShare = global_share;

   mtx_lock(&helios->dev_mutex);
   NTSTATUS st = D3DKMTQueryResourceInfo(&query);
   if (st != 0 || query.NumAllocations != 1 ||
       query.PrivateRuntimeDataSize > HELIOS_VIDMM_PRIVATE_DATA_LIMIT ||
       query.ResourcePrivateDriverDataSize > HELIOS_VIDMM_PRIVATE_DATA_LIMIT ||
       query.TotalPrivateDriverDataSize > HELIOS_VIDMM_PRIVATE_DATA_LIMIT) {
      mtx_unlock(&helios->dev_mutex);
      helios_diag("VidMm QueryResourceInfo global=0x%x failed status=0x%08x allocs=%u private=%u/%u/%u",
                  global_share, (unsigned)st, query.NumAllocations,
                  query.PrivateRuntimeDataSize,
                  query.ResourcePrivateDriverDataSize,
                  query.TotalPrivateDriverDataSize);
      return false;
   }

   void *runtime_data = query.PrivateRuntimeDataSize
                           ? calloc(1, query.PrivateRuntimeDataSize) : NULL;
   void *resource_private = query.ResourcePrivateDriverDataSize
                              ? calloc(1, query.ResourcePrivateDriverDataSize) : NULL;
   void *total_private = query.TotalPrivateDriverDataSize
                           ? calloc(1, query.TotalPrivateDriverDataSize) : NULL;
   if ((query.PrivateRuntimeDataSize && !runtime_data) ||
       (query.ResourcePrivateDriverDataSize && !resource_private) ||
       (query.TotalPrivateDriverDataSize && !total_private)) {
      free(runtime_data);
      free(resource_private);
      free(total_private);
      mtx_unlock(&helios->dev_mutex);
      return false;
   }

   if (query.PrivateRuntimeDataSize) {
      query.pPrivateRuntimeData = runtime_data;
      st = D3DKMTQueryResourceInfo(&query);
      if (st != 0) {
         free(runtime_data);
         free(resource_private);
         free(total_private);
         mtx_unlock(&helios->dev_mutex);
         helios_diag("VidMm QueryResourceInfo data global=0x%x failed status=0x%08x",
                     global_share, (unsigned)st);
         return false;
      }
   }

   D3DDDI_OPENALLOCATIONINFO allocation_info;
   memset(&allocation_info, 0, sizeof(allocation_info));
   D3DKMT_OPENRESOURCE open;
   memset(&open, 0, sizeof(open));
   open.hDevice = helios->device;
   open.hGlobalShare = global_share;
   open.NumAllocations = 1;
   open.pOpenAllocationInfo = &allocation_info;
   open.pPrivateRuntimeData = runtime_data;
   open.PrivateRuntimeDataSize = query.PrivateRuntimeDataSize;
   open.pResourcePrivateDriverData = resource_private;
   open.ResourcePrivateDriverDataSize = query.ResourcePrivateDriverDataSize;
   open.pTotalPrivateDriverDataBuffer = total_private;
   open.TotalPrivateDriverDataBufferSize = query.TotalPrivateDriverDataSize;
   st = D3DKMTOpenResource(&open);
   D3DKMT_HANDLE allocation = allocation_info.hAllocation;
   struct helios_wddm_alloc_private private_data;
   memset(&private_data, 0, sizeof(private_data));
   if (allocation_info.PrivateDriverDataSize >= sizeof(private_data) &&
       allocation_info.pPrivateDriverData)
      memcpy(&private_data, allocation_info.pPrivateDriverData,
             sizeof(private_data));
   const bool expected_nonlocal = !expected_device_local;
   const bool tracker_matches =
      private_data.magic == HELIOS_WDDM_MAGIC &&
      private_data.version == HELIOS_WDDM_VERSION &&
      private_data.kind == HELIOS_WDDM_ALLOC_KIND_TRACKING &&
      private_data.map_cache == expected_cookie &&
      private_data.size == expected_size &&
      !!(private_data.blob_flags & HELIOS_WDDM_BLOB_FLAG_NONLOCAL_TRACKING) ==
         expected_nonlocal;
   enum helios_vidmm_residency residency = HELIOS_VIDMM_NOT_ACQUIRED;
   if (st == 0 && open.hResource && allocation && tracker_matches)
      residency = helios_vidmm_make_resident_locked(
         helios, allocation, expected_size);
   if (residency == HELIOS_VIDMM_RESIDENT) {
      *resource_handle = open.hResource;
      *allocation_handle = allocation;
   } else {
      if (residency != HELIOS_VIDMM_NOT_ACQUIRED)
         helios_vidmm_evict_locked(helios, allocation);
      if (open.hResource || allocation)
         helios_vidmm_destroy_locked(helios, open.hResource, allocation);
      helios_diag("VidMm OpenResource global=0x%x failed/mismatched status=0x%08x resource=0x%x allocation=0x%x",
                  global_share, (unsigned)st, (unsigned)open.hResource,
                  (unsigned)allocation);
   }
   free(runtime_data);
   free(resource_private);
   free(total_private);
   mtx_unlock(&helios->dev_mutex);
   return *resource_handle != 0 && *allocation_handle != 0;
}

void
vn_renderer_helios_vidmm_free(struct vn_renderer *renderer,
                              uint32_t resource_handle,
                              uint32_t allocation_handle)
{
   struct helios *helios = (struct helios *)renderer;
   if (!resource_handle || !allocation_handle)
      return;

   mtx_lock(&helios->dev_mutex);
   helios_vidmm_evict_locked(helios, allocation_handle);
   helios_vidmm_destroy_locked(helios, resource_handle, allocation_handle);
   mtx_unlock(&helios->dev_mutex);
}

static void
helios_external_memory_destroy_locked(
   struct helios *helios,
   struct vn_renderer_helios_external_memory *external)
{
   if (external->export_handle) {
      CloseHandle(external->export_handle);
      external->export_handle = NULL;
   }

   if (external->resource || external->allocation) {
      helios_vidmm_destroy_locked(helios, external->resource,
                                  external->allocation);
      external->resource = 0;
      external->allocation = 0;
   }
}

VkResult
vn_renderer_helios_external_memory_create(
   struct vn_renderer *renderer,
   struct vn_renderer_bo *_bo,
   uint64_t memory_id,
   uint64_t allocation_size,
   uint32_t memory_type_index,
   struct vn_renderer_helios_external_memory **out_external)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_bo *bo = (struct helios_bo *)_bo;
   *out_external = NULL;

   if (!helios->device || !helios->ctx_id || !bo || !bo->base.res_id ||
       !memory_id || !allocation_size)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   struct vn_renderer_helios_external_memory *external =
      calloc(1, sizeof(*external));
   if (!external)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   struct helios_wddm_external_private private_data;
   memset(&private_data, 0, sizeof(private_data));
   private_data.alloc.blob_id = memory_id;
   private_data.alloc.size = allocation_size;
   private_data.alloc.magic = HELIOS_WDDM_MAGIC;
   private_data.alloc.version = HELIOS_WDDM_VERSION;
   private_data.alloc.blob_mem = VIRTIO_GPU_BLOB_MEM_HOST3D;
   private_data.alloc.blob_flags = VIRTIO_GPU_BLOB_FLAG_USE_SHAREABLE;
   private_data.alloc.ctx_id = helios->ctx_id;
   private_data.alloc.map_cache = HELIOS_MAP_CACHE_CACHED;
   private_data.alloc.kind = HELIOS_WDDM_ALLOC_KIND_DEVICE_MEMORY;
   private_data.alloc.adopt_resource_id = bo->base.res_id;
   private_data.meta.venus_alloc_size = allocation_size;
   private_data.meta.memory_type_index = memory_type_index;

   D3DDDI_ALLOCATIONINFO2 allocation_info;
   memset(&allocation_info, 0, sizeof(allocation_info));
   allocation_info.pPrivateDriverData = &private_data;
   allocation_info.PrivateDriverDataSize = sizeof(private_data);

   D3DKMT_CREATEALLOCATION create;
   memset(&create, 0, sizeof(create));
   create.hDevice = helios->device;
   create.Flags.CreateResource = 1;
   create.Flags.CreateShared = 1;
   create.Flags.NtSecuritySharing = 1;
   create.Flags.AllowNotZeroed = 1;
   create.NumAllocations = 1;
   create.pAllocationInfo2 = &allocation_info;

   mtx_lock(&helios->dev_mutex);
   const NTSTATUS st = D3DKMTCreateAllocation2(&create);
   if (st == 0 && create.hResource && allocation_info.hAllocation) {
      /* The KMD atomically re-owns this resource id for the allocation.  The
       * BO must never send RELEASE_BLOB after this point. */
      bo->resource_released = true;
      external->resource = create.hResource;
      external->allocation = allocation_info.hAllocation;
   } else {
      if (st == 0) {
         /* A successful call with incomplete output has still passed through
          * the KMD adoption path, so treat the BO lifetime as transferred. */
         bo->resource_released = true;
         external->resource = create.hResource;
         external->allocation = allocation_info.hAllocation;
         helios_external_memory_destroy_locked(helios, external);
      }
      helios_diag("external memory CreateAllocation2 failed status=0x%08x resource=0x%x allocation=0x%x res_id=%u",
                  (unsigned)st, (unsigned)create.hResource,
                  (unsigned)allocation_info.hAllocation, bo->base.res_id);
   }
   mtx_unlock(&helios->dev_mutex);

   if (!external->resource || !external->allocation) {
      free(external);
      return st == 0 ? VK_ERROR_INVALID_EXTERNAL_HANDLE
                     : VK_ERROR_OUT_OF_DEVICE_MEMORY;
   }

   helios_diag("external memory adopted res=%u resource=0x%x allocation=0x%x size=%llu type=%u",
               bo->base.res_id, (unsigned)external->resource,
               (unsigned)external->allocation,
               (unsigned long long)allocation_size, memory_type_index);
   *out_external = external;
   return VK_SUCCESS;
}

static void
helios_external_memory_destroy_aux_handles(
   D3DKMT_HANDLE keyed_mutex,
   D3DKMT_HANDLE sync_object)
{
   if (keyed_mutex) {
      D3DKMT_DESTROYKEYEDMUTEX destroy;
      memset(&destroy, 0, sizeof(destroy));
      destroy.hKeyedMutex = keyed_mutex;
      const NTSTATUS st = D3DKMTDestroyKeyedMutex(&destroy);
      if (st != 0)
         helios_diag("external memory keyed-mutex destroy failed status=0x%08x handle=0x%x",
                     (unsigned)st, (unsigned)keyed_mutex);
   }
   if (sync_object) {
      D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy;
      memset(&destroy, 0, sizeof(destroy));
      destroy.hSyncObject = sync_object;
      const NTSTATUS st = D3DKMTDestroySynchronizationObject(&destroy);
      if (st != 0)
         helios_diag("external memory sync-object destroy failed status=0x%08x handle=0x%x",
                     (unsigned)st, (unsigned)sync_object);
   }
}

VkResult
vn_renderer_helios_external_memory_open(
   struct vn_renderer *renderer,
   const VkImportMemoryWin32HandleInfoKHR *import_info,
   uint64_t allocation_size,
   uint32_t memory_type_index,
   uint32_t *out_resource_id,
   uint64_t *out_allocation_size,
   uint32_t *out_memory_type_index,
   struct vn_renderer_helios_external_memory **out_external)
{
   struct helios *helios = (struct helios *)renderer;
   *out_resource_id = 0;
   *out_allocation_size = 0;
   *out_memory_type_index = UINT32_MAX;
   *out_external = NULL;

   /*
    * Two guest handle types reach the same carrier, because on this stack they
    * name the same object. OPAQUE_WIN32 is a payload this ICD exported.
    * D3D12_RESOURCE is a committed resource created by helios_umd12, and the
    * fused heap+resource DDI arm sets VKD3D_HEAP_FLAG_HELIOS_VENUS_EXPORT on
    * *every* committed create (umd12/src/forward12/resource12.rs) precisely so
    * the memory is allocator-dedicated and venus-exportable — so its WDDM
    * allocation carries the same helios_wddm_open_identity, naming the same
    * venus resource. The host never learns which D3D API named it.
    *
    * They are NOT interchangeable to the caller: Vulkan keeps them in separate
    * compatibility classes, and vn_sanitize_image_format_properties reports
    * each as compatible only with itself.
    */
   const bool d3d12_resource =
      import_info &&
      import_info->handleType ==
         VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT;
   if (!helios->device || !import_info ||
       (import_info->handleType !=
           VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT &&
        !d3d12_resource) ||
       (!import_info->handle && !import_info->name))
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   /* VkMemoryAllocateInfo::allocationSize is ignored for D3D12_RESOURCE_BIT —
    * the implementation takes the size from Windows (§10.3, and the Vulkan
    * spec's import rules). The identity is that size here, so a caller-supplied
    * 0 means "whatever the allocation says" rather than a mismatch. */
   const bool size_from_identity = d3d12_resource && allocation_size == 0;

   HANDLE nt_handle = import_info->handle;
   bool close_nt_handle = false;
   WCHAR path[512];
   struct helios_unicode_string us;
   OBJECT_ATTRIBUTES attr;
   memset(&attr, 0, sizeof(attr));

   mtx_lock(&helios->dev_mutex);
   if (!nt_handle) {
      if (!helios_nt_object_path(import_info->name, path, ARRAY_SIZE(path),
                                 &us)) {
         mtx_unlock(&helios->dev_mutex);
         return VK_ERROR_INVALID_EXTERNAL_HANDLE;
      }

      attr.Length = sizeof(attr);
      attr.ObjectName = &us;
      attr.Attributes = OBJ_CASE_INSENSITIVE;
      D3DKMT_OPENNTHANDLEFROMNAME open_name;
      memset(&open_name, 0, sizeof(open_name));
      open_name.dwDesiredAccess = SHARED_ALLOCATION_ALL_ACCESS;
      open_name.pObjAttrib = &attr;
      const NTSTATUS name_st = D3DKMTOpenNtHandleFromName(&open_name);
      if (name_st != 0 || !open_name.hNtHandle) {
         mtx_unlock(&helios->dev_mutex);
         helios_diag("external memory open name failed status=0x%08x",
                     (unsigned)name_st);
         return VK_ERROR_INVALID_EXTERNAL_HANDLE;
      }
      nt_handle = open_name.hNtHandle;
      close_nt_handle = true;
   }

   D3DKMT_QUERYRESOURCEINFOFROMNTHANDLE query;
   memset(&query, 0, sizeof(query));
   query.hDevice = helios->device;
   query.hNtHandle = nt_handle;
   NTSTATUS st = D3DKMTQueryResourceInfoFromNtHandle(&query);
   if (st != 0 || query.NumAllocations != 1 ||
       query.PrivateRuntimeDataSize > HELIOS_VIDMM_PRIVATE_DATA_LIMIT ||
       query.ResourcePrivateDriverDataSize > HELIOS_VIDMM_PRIVATE_DATA_LIMIT ||
       query.TotalPrivateDriverDataSize > HELIOS_VIDMM_PRIVATE_DATA_LIMIT) {
      if (close_nt_handle)
         CloseHandle(nt_handle);
      mtx_unlock(&helios->dev_mutex);
      helios_diag("external memory query failed status=0x%08x allocs=%u private=%u/%u/%u",
                  (unsigned)st, query.NumAllocations,
                  query.PrivateRuntimeDataSize,
                  query.ResourcePrivateDriverDataSize,
                  query.TotalPrivateDriverDataSize);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   void *runtime_data = query.PrivateRuntimeDataSize
                           ? calloc(1, query.PrivateRuntimeDataSize) : NULL;
   void *resource_private = query.ResourcePrivateDriverDataSize
                              ? calloc(1, query.ResourcePrivateDriverDataSize)
                              : NULL;
   void *total_private = query.TotalPrivateDriverDataSize
                           ? calloc(1, query.TotalPrivateDriverDataSize) : NULL;
   if ((query.PrivateRuntimeDataSize && !runtime_data) ||
       (query.ResourcePrivateDriverDataSize && !resource_private) ||
       (query.TotalPrivateDriverDataSize && !total_private)) {
      free(runtime_data);
      free(resource_private);
      free(total_private);
      if (close_nt_handle)
         CloseHandle(nt_handle);
      mtx_unlock(&helios->dev_mutex);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   if (query.PrivateRuntimeDataSize) {
      query.pPrivateRuntimeData = runtime_data;
      st = D3DKMTQueryResourceInfoFromNtHandle(&query);
   }

   D3DDDI_OPENALLOCATIONINFO2 allocation_info;
   memset(&allocation_info, 0, sizeof(allocation_info));
   D3DKMT_OPENRESOURCEFROMNTHANDLE open;
   memset(&open, 0, sizeof(open));
   open.hDevice = helios->device;
   open.hNtHandle = nt_handle;
   open.NumAllocations = 1;
   open.pOpenAllocationInfo2 = &allocation_info;
   open.PrivateRuntimeDataSize = query.PrivateRuntimeDataSize;
   open.pPrivateRuntimeData = runtime_data;
   open.ResourcePrivateDriverDataSize =
      query.ResourcePrivateDriverDataSize;
   open.pResourcePrivateDriverData = resource_private;
   open.TotalPrivateDriverDataBufferSize = query.TotalPrivateDriverDataSize;
   open.pTotalPrivateDriverDataBuffer = total_private;
   if (st == 0)
      st = D3DKMTOpenResourceFromNtHandle(&open);

   struct helios_wddm_open_identity identity;
   memset(&identity, 0, sizeof(identity));
   if (st == 0 && allocation_info.pPrivateDriverData &&
       allocation_info.PrivateDriverDataSize >= sizeof(identity)) {
      memcpy(&identity, allocation_info.pPrivateDriverData, sizeof(identity));
   }

   const uint64_t effective_size =
      size_from_identity ? identity.venus_alloc_size : allocation_size;
   const bool identity_valid =
      identity.magic == HELIOS_WDDM_IDENTITY_MAGIC &&
      identity.version == HELIOS_WDDM_IDENTITY_VERSION &&
      identity.kind == HELIOS_WDDM_ALLOC_KIND_DEVICE_MEMORY &&
      identity.resource_id != 0 && identity.blob_size >= effective_size &&
      identity.venus_alloc_size == effective_size &&
      /* UINT32_MAX = "report it, do not check it". Only
       * vkGetMemoryWin32HandlePropertiesKHR passes this: it is the call whose
       * whole job is to discover the memory type, so it cannot supply one. */
      (memory_type_index == UINT32_MAX ||
       identity.memory_type_index == memory_type_index);

   struct vn_renderer_helios_external_memory *external = NULL;
   if (st == 0 && open.hResource && allocation_info.hAllocation &&
       identity_valid) {
      external = calloc(1, sizeof(*external));
      if (external) {
         external->resource = open.hResource;
         external->allocation = allocation_info.hAllocation;
      }
   }

   if (!external && (open.hResource || allocation_info.hAllocation))
      helios_vidmm_destroy_locked(helios, open.hResource,
                                  allocation_info.hAllocation);
   helios_external_memory_destroy_aux_handles(open.hKeyedMutex,
                                              open.hSyncObject);
   free(runtime_data);
   free(resource_private);
   free(total_private);
   if (close_nt_handle)
      CloseHandle(nt_handle);
   mtx_unlock(&helios->dev_mutex);

   if (!external) {
      helios_diag("external memory open failed status=0x%08x resource=0x%x allocation=0x%x identity=%u/%u/%u size=%llu/%llu type=%u/%u",
                  (unsigned)st, (unsigned)open.hResource,
                  (unsigned)allocation_info.hAllocation, identity.magic,
                  identity.version, identity.resource_id,
                  (unsigned long long)identity.venus_alloc_size,
                  (unsigned long long)effective_size,
                  identity.memory_type_index, memory_type_index);
      return st == 0 && identity_valid ? VK_ERROR_OUT_OF_HOST_MEMORY
                                      : VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   helios_diag("external memory opened res=%u resource=0x%x allocation=0x%x size=%llu type=%u",
               identity.resource_id, (unsigned)external->resource,
               (unsigned)external->allocation,
               (unsigned long long)allocation_size, memory_type_index);
   *out_resource_id = identity.resource_id;
   *out_allocation_size = effective_size;
   *out_memory_type_index = identity.memory_type_index;
   *out_external = external;
   return VK_SUCCESS;
}

VkResult
vn_renderer_helios_external_memory_prepare_export(
   struct vn_renderer *renderer,
   struct vn_renderer_helios_external_memory *external,
   const VkExportMemoryWin32HandleInfoKHR *export_info)
{
   struct helios *helios = (struct helios *)renderer;
   if (!external || !external->allocation)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   if (external->export_handle)
      return VK_SUCCESS;

   WCHAR path[512];
   struct helios_unicode_string us;
   OBJECT_ATTRIBUTES attr;
   memset(&attr, 0, sizeof(attr));
   attr.Length = sizeof(attr);
   attr.Attributes = OBJ_CASE_INSENSITIVE;

   if (export_info && export_info->name) {
      if (!helios_nt_object_path(export_info->name, path, ARRAY_SIZE(path),
                                 &us))
         return VK_ERROR_INVALID_EXTERNAL_HANDLE;
      attr.ObjectName = &us;
   }
   if (export_info && export_info->pAttributes) {
      attr.SecurityDescriptor =
         export_info->pAttributes->lpSecurityDescriptor;
      if (export_info->pAttributes->bInheritHandle)
         attr.Attributes |= OBJ_INHERIT;
   }

   const DWORD access = export_info ? export_info->dwAccess
                                    : SHARED_ALLOCATION_ALL_ACCESS;
   HANDLE handle = NULL;
   mtx_lock(&helios->dev_mutex);
   const NTSTATUS st =
      D3DKMTShareObjects(1, &external->resource, &attr, access, &handle);
   if (st == 0 && handle) {
      external->export_handle = handle;
      external->export_handle_inheritable =
         export_info && export_info->pAttributes &&
         export_info->pAttributes->bInheritHandle;
   }
   mtx_unlock(&helios->dev_mutex);

   if (st != 0 || !handle) {
      helios_diag("external memory ShareObjects failed status=0x%08x resource=0x%x",
                  (unsigned)st, (unsigned)external->resource);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }
   return VK_SUCCESS;
}

VkResult
vn_renderer_helios_external_memory_get_handle(
   struct vn_renderer *renderer,
   struct vn_renderer_helios_external_memory *external,
   void **out_handle)
{
   (void)renderer;
   *out_handle = NULL;
   if (!external || !external->export_handle)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   HANDLE duplicate = NULL;
   if (!DuplicateHandle(GetCurrentProcess(), external->export_handle,
                        GetCurrentProcess(), &duplicate, 0,
                        external->export_handle_inheritable,
                        DUPLICATE_SAME_ACCESS))
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   *out_handle = duplicate;
   return VK_SUCCESS;
}

void
vn_renderer_helios_external_memory_destroy(
   struct vn_renderer *renderer,
   struct vn_renderer_helios_external_memory *external)
{
   if (!external)
      return;

   struct helios *helios = (struct helios *)renderer;
   mtx_lock(&helios->dev_mutex);
   helios_external_memory_destroy_locked(helios, external);
   mtx_unlock(&helios->dev_mutex);
   free(external);
}

/* ── ops ───────────────────────────────────────────────────────────────────── */

static VkResult
helios_submit(struct vn_renderer *renderer, const struct vn_renderer_submit *submit)
{
   struct helios *helios = (struct helios *)renderer;
   VkResult result = VK_SUCCESS;
   const bool perf = helios->perf.enabled;
   LARGE_INTEGER t0 = { 0 }, t1 = { 0 }, t2 = { 0 };

   if (perf)
      QueryPerformanceCounter(&t0);
   mtx_lock(&helios->dev_mutex);
   if (perf) {
      QueryPerformanceCounter(&t1);
      helios->perf.submit_mutex_ticks += t1.QuadPart - t0.QuadPart;
      helios->perf.submit_calls++;
   }
   for (uint32_t i = 0; i < submit->batch_count; i++) {
      const struct vn_renderer_submit_batch *batch = &submit->batches[i];

      if (perf) {
         helios->perf.submit_batches++;
         helios->perf.submit_syncs += batch->sync_count;
         helios->perf.submit_cs_bytes += batch->cs_size;
         if (!batch->cs_size)
            helios->perf.submit_empty_batches++;
      }

      uint64_t fence_id = 0;
      if (batch->cs_size) {
         if (perf)
            QueryPerformanceCounter(&t1);
         const bool ok = helios_ioctl_submit_cs(
            helios, batch->cs_data, batch->cs_size, batch->ring_idx,
            batch->present_cookie, batch->present_value32,
            &fence_id);
         if (perf) {
            QueryPerformanceCounter(&t2);
            helios->perf.submit_escape_ticks += t2.QuadPart - t1.QuadPart;
         }
         if (!ok) {
            result = VK_ERROR_DEVICE_LOST;
            break;
         }
      }

      if (perf)
         QueryPerformanceCounter(&t1);
      for (uint32_t j = 0; j < batch->sync_count; j++) {
         struct helios_sync *sync = (struct helios_sync *)batch->syncs[j];
         if (!helios_sync_append_locked(renderer, sync, batch->sync_values[j],
                                        fence_id)) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
            break;
         }
         /* A SHARED sync must signal its WDDM fence at wire-fence retirement
          * without relying on this process ever waiting on it — hand the
          * (sync, fence) pair to the retire thread. */
         if (fence_id && sync->wddm_local &&
             !helios_retire_enqueue_locked(helios, sync, fence_id,
                                           batch->sync_values[j])) {
            result = VK_ERROR_OUT_OF_HOST_MEMORY;
            break;
         }
      }
      if (perf) {
         QueryPerformanceCounter(&t2);
         helios->perf.submit_sync_ticks += t2.QuadPart - t1.QuadPart;
      }
      if (result != VK_SUCCESS)
         break;
   }
   /* Periodic summary: atexit never runs under taskkill /F, so the phase
    * telemetry must land during the run. Bounded: one multi-line append
    * per 512 renderer submits. */
   const bool write_summary =
      perf && (helios->perf.submit_calls & 511) == 0;
   mtx_unlock(&helios->dev_mutex);
   if (write_summary)
      helios_perf_write(helios, false);

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
      struct helios_sync *sync = (struct helios_sync *)wait->syncs[i];
      if (sync->wddm_cpu_va) {
         const uint64_t wddm_val = *(const volatile uint64_t *)sync->wddm_cpu_va;
         if (sync->val < wddm_val)
            sync->val = wddm_val;
      }
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
      result = VK_SUCCESS;
      for (uint32_t i = 0; i < wait->sync_count; i++) {
         D3DKMT_HANDLE wddm_local = 0;
         struct helios_sync *sync = (struct helios_sync *)wait->syncs[i];
         mtx_lock(&helios->dev_mutex);
         if (sync->wddm_cpu_va) {
            const uint64_t wddm_val = *(const volatile uint64_t *)sync->wddm_cpu_va;
            if (sync->val < wddm_val)
               sync->val = wddm_val;
         }
         if (sync->val >= wait->sync_values[i]) {
            mtx_unlock(&helios->dev_mutex);
            continue;
         }
         if (!sync->wddm_local) {
            mtx_unlock(&helios->dev_mutex);
            result = VK_TIMEOUT;
            break;
         }
         wddm_local = sync->wddm_local;
         mtx_unlock(&helios->dev_mutex);

         result = helios_wddm_sync_wait(renderer, wddm_local,
                                        wait->sync_values[i], wait->timeout);
         if (result != VK_SUCCESS)
            break;
         mtx_lock(&helios->dev_mutex);
         sync = (struct helios_sync *)wait->syncs[i];
         if (sync->val < wait->sync_values[i])
            sync->val = wait->sync_values[i];
         mtx_unlock(&helios->dev_mutex);
         if (wait->wait_any)
            break;
      }
      if (result != VK_SUCCESS && helios->perf.enabled) {
         mtx_lock(&helios->dev_mutex);
         helios->perf.wait_timeout++;
         mtx_unlock(&helios->dev_mutex);
      }
      if (wait_fences != stack_wait_fences)
         free(wait_fences);
      return result;
   }

   if (helios->perf.enabled) {
      mtx_lock(&helios->dev_mutex);
      helios->perf.wait_slow++;
      mtx_unlock(&helios->dev_mutex);
   }

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
            helios_sync_mark_fence_locked(renderer, sync, wait_fences[j]);
         }
      }
      mtx_unlock(&helios->dev_mutex);
   } else if (helios->perf.enabled) {
      mtx_lock(&helios->dev_mutex);
      helios->perf.wait_timeout++;
      mtx_unlock(&helios->dev_mutex);
   }

   if (wait_fences != stack_wait_fences)
      free(wait_fences);
   return result;
}

/* ── shmem ops ─────────────────────────────────────────────────────────────── */

/* Cache-eviction / final teardown callback. Cached shmems intentionally remain
 * live until they reach this callback; only then release the KMD/host blob. */
static void
helios_shmem_destroy_now(struct vn_renderer *renderer, struct vn_renderer_shmem *shmem)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_shmem *hshmem = (struct helios_shmem *)shmem;

   mtx_lock(&helios->dev_mutex);
   helios_ioctl_release_blob(helios, hshmem->ctx_id, shmem->res_id);
   mtx_unlock(&helios->dev_mutex);
   helios_trace_shmem("release", helios->device, helios->context,
                      hshmem->ctx_id, shmem->res_id, shmem->mmap_size,
                      (uint64_t)(uintptr_t)shmem->mmap_ptr);

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
   helios_trace_shmem("create", helios->device, helios->context, helios->ctx_id,
                      res_id, size, user_va);

   if (!res_id || !user_va) {
      vn_log(helios->instance, "shmem create failed (res_id=%u, mapped=%d)", res_id,
             user_va != 0);
      if (res_id) {
         mtx_lock(&helios->dev_mutex);
         helios_ioctl_release_blob(helios, helios->ctx_id, res_id);
         mtx_unlock(&helios->dev_mutex);
      }
      return NULL;
   }

   struct helios_shmem *shmem = calloc(1, sizeof(*shmem));
   if (!shmem) {
      mtx_lock(&helios->dev_mutex);
      helios_ioctl_release_blob(helios, helios->ctx_id, res_id);
      mtx_unlock(&helios->dev_mutex);
      return NULL;
   }

   shmem->base.refcount = VN_REFCOUNT_INIT(1);
   shmem->base.res_id = res_id;
   shmem->base.mmap_size = size;
   shmem->base.mmap_ptr = (void *)(uintptr_t)user_va;
   shmem->ctx_id = helios->ctx_id;

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

   /* Match the virtgpu/vtest Venus renderer contract: external memory must make
    * the HOST3D blob shareable so virglrenderer can materialize a dma-buf for
    * scanout/import paths. Mappability remains tied to host-visible memory. */
   uint32_t blob_flags = 0;
   if (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
      blob_flags |= VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE;
   if (external_handles)
      blob_flags |= VIRTIO_GPU_BLOB_FLAG_USE_SHAREABLE;

   mtx_lock(&helios->dev_mutex);
   /* The optional batch orders the host's vkAllocateMemory before the blob binds
    * to it ("venus: let resource_create_blob wait for mem alloc"). Submit it
    * first (synchronous) so the device memory exists when ALLOC_BLOB references
    * it via blob_id=mem_id. */
   if (batch) {
      uint64_t fence_id = 0;
      if (batch->cs_size &&
          !helios_ioctl_submit_cs(helios, batch->cs_data, batch->cs_size,
                                  batch->ring_idx, batch->present_cookie,
                                  batch->present_value32, &fence_id)) {
         mtx_unlock(&helios->dev_mutex);
         return VK_ERROR_DEVICE_LOST;
      }
      /* Record the batch's syncs against its real wire fence (a sync-only
       * batch has fence_id 0 and advances immediately when nothing is
       * pending, else queues behind the in-flight signal ops). The
       * subsequent ALLOC_BLOB(blob_id=mem_id) is ordered by virglrenderer's
       * "resource_create_blob waits for mem alloc" host-side wait — the same
       * mechanism the upstream async virtgpu backend relies on — so the blob
       * binds only after this batch's vkAllocateMemory executes. */
      for (uint32_t j = 0; j < batch->sync_count; j++) {
         struct helios_sync *sync = (struct helios_sync *)batch->syncs[j];
         helios_sync_append_locked(renderer, sync, batch->sync_values[j],
                                   fence_id);
         if (fence_id && sync->wddm_local)
            (void)helios_retire_enqueue_locked(helios, sync, fence_id,
                                               batch->sync_values[j]);
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
   if (!bo) {
      mtx_lock(&helios->dev_mutex);
      helios_ioctl_release_blob(helios, helios->ctx_id, res_id);
      mtx_unlock(&helios->dev_mutex);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   bo->base.refcount = VN_REFCOUNT_INIT(1);
   bo->base.res_id = res_id;
   bo->base.mmap_size = (blob_flags & VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE) ? size : 0;
   bo->ctx_id = helios->ctx_id;
   bo->blob_flags = blob_flags;
   bo->memory_flags = flags;

   *out_bo = &bo->base;
   return VK_SUCCESS;
}

static VkResult
helios_bo_create_from_resource_id(struct vn_renderer *renderer,
                                  VkDeviceSize size,
                                  uint32_t res_id,
                                  VkMemoryPropertyFlags flags,
                                  struct vn_renderer_bo **out_bo)
{
   struct helios *helios = (struct helios *)renderer;

   if (!res_id)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   mtx_lock(&helios->dev_mutex);
   const bool attached =
      helios_ioctl_attach_resource(helios, helios->ctx_id, res_id);
   mtx_unlock(&helios->dev_mutex);

   if (!attached)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   struct helios_bo *bo = calloc(1, sizeof(*bo));
   if (!bo)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   bo->base.refcount = VN_REFCOUNT_INIT(1);
   bo->base.res_id = res_id;
   bo->base.mmap_size = (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) ? size : 0;
   bo->ctx_id = helios->ctx_id;
   bo->memory_flags = flags;
   bo->resource_released = true;

   {
      FILE *f = helios_diag_fopen("helios_icd_diag.log");
      if (f) {
         fprintf(f, "%lld pid=%lu import_resource ctx=%u res=%u size=%llu flags=0x%x\n",
                 (long long)time(NULL), (unsigned long)GetCurrentProcessId(),
                 helios->ctx_id, res_id, (unsigned long long)size,
                 (unsigned)flags);
         fclose(f);
      }
   }

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
   struct helios_bo *hbo = (struct helios_bo *)bo;

   if (!hbo->resource_released)
      vn_renderer_bo_release_resource(renderer, bo);

   free(hbo);
   return true;
}

static void
helios_bo_release_resource(struct vn_renderer *renderer, struct vn_renderer_bo *bo)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_bo *hbo = (struct helios_bo *)bo;

   if (hbo->resource_released)
      return;

   mtx_lock(&helios->dev_mutex);
   helios_ioctl_release_blob(helios, hbo->ctx_id, bo->res_id);
   mtx_unlock(&helios->dev_mutex);
   hbo->resource_released = true;
}

/* CPU cache-line ops are only meaningful for a CACHED guest mapping that
 * aliases a non-WB host mapping. WC/UC guest mappings never hold cache lines
 * (write-combine buffers drain on the submit syscall's serialization), and
 * when the memory type is HOST_COHERENT|HOST_CACHED the host map_info is
 * CACHED (vkr reports CACHED iff coherent && cached), so guest WB over host
 * WB is hardware-coherent under KVM and clflush is pure overhead — measured
 * ~175 us/frame in the WSI present invalidate sweep alone. */
static bool
helios_bo_needs_cache_ops(const struct helios_bo *bo)
{
   if (bo->map_cache != HELIOS_MAP_CACHE_CACHED)
      return false;

   const VkMemoryPropertyFlags host_wb = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |
                                         VK_MEMORY_PROPERTY_HOST_CACHED_BIT;
   return (bo->memory_flags & host_wb) != host_wb;
}

static void
helios_bo_flush(struct vn_renderer *renderer,
                struct vn_renderer_bo *bo,
                VkDeviceSize offset,
                VkDeviceSize size)
{
   (void)renderer;
   if (!bo->mmap_ptr || !size || !util_has_cache_ops() ||
       !helios_bo_needs_cache_ops((const struct helios_bo *)bo))
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
   if (!bo->mmap_ptr || !size || !util_has_cache_ops() ||
       !helios_bo_needs_cache_ops((const struct helios_bo *)bo))
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
   struct helios_sync *sync = calloc(1, sizeof(*sync));
   if (!sync)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   sync->base.sync_id = 0; /* unused: Helios does not carry host sync ids on the wire */
   sync->val = initial_val;
   sync->refs = 1;
   if (flags & VN_RENDERER_SYNC_SHAREABLE) {
      VkResult result =
         helios_wddm_sync_create(renderer, initial_val, true,
                                 &sync->wddm_local, &sync->wddm_global,
                                 &sync->wddm_cpu_va);
      if (result != VK_SUCCESS) {
         free(sync);
         return result;
      }
   }

   *out_sync = &sync->base;
   return VK_SUCCESS;
}

static void
helios_sync_destroy(struct vn_renderer *renderer, struct vn_renderer_sync *_sync)
{
   struct helios *helios = (struct helios *)renderer;
   struct helios_sync *sync = (struct helios_sync *)_sync;

   /* Drop the owner reference; a queued retire entry may still hold one, in
    * which case the retire thread frees the struct after its WAIT_FENCE. */
   mtx_lock(&helios->dev_mutex);
   const bool free_sync = helios_sync_unref_locked(renderer, sync);
   mtx_unlock(&helios->dev_mutex);

   if (free_sync)
      free(sync);
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
   if (sync->wddm_local)
      (void)helios_wddm_sync_signal(renderer, sync->wddm_local, initial_val);
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
         helios_sync_mark_fence_locked(renderer, sync, fences[i]);
         mtx_unlock(&helios->dev_mutex);
      } else {
         break;
      }
   }

   mtx_lock(&helios->dev_mutex);
   if (sync->wddm_cpu_va) {
      const uint64_t wddm_val = *(const volatile uint64_t *)sync->wddm_cpu_va;
      if (sync->val < wddm_val)
         sync->val = wddm_val;
   }
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
   if (sync->wddm_local)
      (void)helios_wddm_sync_signal(renderer, sync->wddm_local, val);
   mtx_unlock(&helios->dev_mutex);
   return VK_SUCCESS;
}

VkResult
vn_renderer_helios_sync_create_from_win32(
   struct vn_renderer *renderer,
   VkExternalSemaphoreHandleTypeFlagBits handle_type,
   void *handle,
   struct vn_renderer_sync **out_sync)
{
   struct helios_sync *sync = calloc(1, sizeof(*sync));
   if (!sync)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   sync->refs = 1;
   VkResult result;
   switch (handle_type) {
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT:
   /* A D3D12 fence on this stack IS a Helios WDDM monitored fence: vkd3d
    * creates ID3D12Fence as a Vulkan timeline semaphore and shares it through
    * this ICD's own Win32 export (libs/vkd3d/command.c:626-644,
    * device.c:7687). So the NT handle a D3D12_FENCE_BIT import receives is the
    * same object an OPAQUE_WIN32 import receives, and the same open applies.
    * That identity is a property of Helios, not of Vulkan — an NT handle from
    * a different D3D12 implementation would not be one of ours, and
    * helios_wddm_sync_open_nt refuses it. */
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT:
      result = helios_wddm_sync_open_nt(renderer, handle, &sync->wddm_local,
                                        &sync->wddm_cpu_va);
      break;
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_KMT_BIT:
      sync->wddm_global = HandleToULong(handle);
      result = helios_wddm_sync_open_kmt(renderer, sync->wddm_global,
                                         &sync->wddm_local,
                                         &sync->wddm_cpu_va);
      break;
   default:
      free(sync);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   if (result != VK_SUCCESS) {
      free(sync);
      return result;
   }

   sync->base.sync_id = 0;
   if (sync->wddm_cpu_va)
      sync->val = *(const volatile uint64_t *)sync->wddm_cpu_va;
   *out_sync = &sync->base;
   return VK_SUCCESS;
}

/* Translate Win32 kernel-object names into the NT object-manager paths used by
 * D3DKMTShareObjects/D3DKMTOpenNtHandleFromName:
 *   Global\\X -> \\BaseNamedObjects\\X
 *   Local\\X  -> \\Sessions\\<id>\\BaseNamedObjects\\X (or
 *                \\BaseNamedObjects\\X for session 0)
 *   X         -> the same current-session directory as Local\\X
 *   \\...     -> used verbatim.
 * This matches Win32 named-object namespace rules and makes Vulkan's optional
 * name field work for ordinary desktop applications as well as services. */
static bool
helios_nt_object_path(const WCHAR *name, WCHAR *buf, size_t buf_chars,
                      struct helios_unicode_string *out_us)
{
   static const WCHAR global_prefix[] = L"Global\\";
   static const WCHAR local_prefix[] = L"Local\\";
   const size_t global_prefix_len = ARRAY_SIZE(global_prefix) - 1;
   const size_t local_prefix_len = ARRAY_SIZE(local_prefix) - 1;
   size_t len;

   if (!name)
      return false;

   if (wcsncmp(name, global_prefix, global_prefix_len) == 0) {
      static const WCHAR bno[] = L"\\BaseNamedObjects\\";
      const size_t bno_len = ARRAY_SIZE(bno) - 1;
      const WCHAR *rest = name + global_prefix_len;
      const size_t rest_len = wcslen(rest);
      if (!rest_len || bno_len + rest_len + 1 > buf_chars)
         return false;
      memcpy(buf, bno, bno_len * sizeof(WCHAR));
      memcpy(buf + bno_len, rest, (rest_len + 1) * sizeof(WCHAR));
      len = bno_len + rest_len;
   } else if (name[0] == L'\\') {
      len = wcslen(name);
      if (!len || len + 1 > buf_chars)
         return false;
      memcpy(buf, name, (len + 1) * sizeof(WCHAR));
   } else {
      const WCHAR *rest = name;
      if (wcsncmp(name, local_prefix, local_prefix_len) == 0)
         rest += local_prefix_len;
      const size_t rest_len = wcslen(rest);
      DWORD session_id = 0;
      if (!rest_len ||
          !ProcessIdToSessionId(GetCurrentProcessId(), &session_id))
         return false;
      const int prefix_len = session_id == 0
         ? swprintf(buf, buf_chars, L"\\BaseNamedObjects\\")
         : swprintf(buf, buf_chars,
                    L"\\Sessions\\%lu\\BaseNamedObjects\\",
                    (unsigned long)session_id);
      if (prefix_len < 0 || (size_t)prefix_len + rest_len + 1 > buf_chars)
         return false;
      memcpy(buf + prefix_len, rest, (rest_len + 1) * sizeof(WCHAR));
      len = (size_t)prefix_len + rest_len;
   }

   if (len > UINT16_MAX / sizeof(WCHAR) - 1)
      return false;

   out_us->Buffer = buf;
   out_us->Length = (USHORT)(len * sizeof(WCHAR));
   out_us->MaximumLength = (USHORT)((len + 1) * sizeof(WCHAR));
   return true;
}

VkResult
vn_renderer_helios_sync_share_named(struct vn_renderer *renderer,
                                    struct vn_renderer_sync *_sync,
                                    const void *name,
                                    const void *security_attributes)
{
   struct helios_sync *sync = (struct helios_sync *)_sync;
   const SECURITY_ATTRIBUTES *sa = security_attributes;

   if (!sync->wddm_local)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   if (sync->nt_named_handle) /* already published once; names are per-object */
      return VK_SUCCESS;

   WCHAR path[192];
   struct helios_unicode_string us;
   if (!helios_nt_object_path((const WCHAR *)name, path,
                              ARRAY_SIZE(path), &us)) {
      helios_diag("sync_share_named: invalid or overlong object name");
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   OBJECT_ATTRIBUTES attr;
   memset(&attr, 0, sizeof(attr));
   attr.Length = sizeof(attr);
   attr.ObjectName = &us;
   if (sa && sa->lpSecurityDescriptor)
      attr.SecurityDescriptor = sa->lpSecurityDescriptor;

   HANDLE handle = NULL;
   D3DKMT_HANDLE object = sync->wddm_local;
   const NTSTATUS st =
      D3DKMTShareObjects(1, &object, &attr, GENERIC_ALL, &handle);
   if (st != 0) {
      helios_diag("sync_share_named failed status=0x%08x local=0x%x",
                  (unsigned)st, (unsigned)sync->wddm_local);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   /* Keep the handle open: the kernel name lives only as long as a handle
    * referencing it does. Closed on the sync's final unref. */
   sync->nt_named_handle = handle;
   helios_diag("sync_share_named ok local=0x%x handle=%p (named NT share)",
               (unsigned)sync->wddm_local, handle);
   return VK_SUCCESS;
}

VkResult
vn_renderer_helios_sync_create_from_win32_name(struct vn_renderer *renderer,
                                               const void *name,
                                               struct vn_renderer_sync **out_sync)
{
   WCHAR path[192];
   struct helios_unicode_string us;
   if (!helios_nt_object_path((const WCHAR *)name, path,
                              ARRAY_SIZE(path), &us)) {
      helios_diag("sync_open_by_name: refused name (must be Global\\* or an "
                  "absolute NT path)");
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   OBJECT_ATTRIBUTES attr;
   memset(&attr, 0, sizeof(attr));
   attr.Length = sizeof(attr);
   attr.ObjectName = &us;

   D3DKMT_OPENSYNCOBJECTNTHANDLEFROMNAME open_name;
   memset(&open_name, 0, sizeof(open_name));
   open_name.dwDesiredAccess = GENERIC_ALL;
   open_name.pObjAttrib = &attr;

   const NTSTATUS st = D3DKMTOpenSyncObjectNtHandleFromName(&open_name);
   if (st != 0) {
      helios_diag("sync_open_by_name failed status=0x%08x", (unsigned)st);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   struct helios_sync *sync = calloc(1, sizeof(*sync));
   if (!sync) {
      CloseHandle(open_name.hNtHandle);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   sync->refs = 1;
   VkResult result = helios_wddm_sync_open_nt(renderer, open_name.hNtHandle,
                                              &sync->wddm_local,
                                              &sync->wddm_cpu_va);
   /* Unlike the imported-NT-handle path (where the caller transfers handle
    * ownership per the Vulkan spec), this handle is ours: close it once the
    * D3DKMT local handle references the object. */
   CloseHandle(open_name.hNtHandle);

   if (result != VK_SUCCESS) {
      free(sync);
      return result;
   }

   sync->base.sync_id = 0;
   if (sync->wddm_cpu_va)
      sync->val = *(const volatile uint64_t *)sync->wddm_cpu_va;
   *out_sync = &sync->base;
   helios_diag("sync_open_by_name ok local=0x%x val=%llu",
               (unsigned)sync->wddm_local, (unsigned long long)sync->val);
   return VK_SUCCESS;
}

VkResult
vn_renderer_helios_sync_export_win32(
   struct vn_renderer *renderer,
   struct vn_renderer_sync *_sync,
   VkExternalSemaphoreHandleTypeFlagBits handle_type,
   void **out_handle)
{
   struct helios_sync *sync = (struct helios_sync *)_sync;

   if (!sync->wddm_local) {
      VkResult result =
         helios_wddm_sync_create(renderer, sync->val,
                                 handle_type ==
                                    VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT,
                                 &sync->wddm_local, &sync->wddm_global,
                                 &sync->wddm_cpu_va);
      if (result != VK_SUCCESS)
         return result;
   }

   switch (handle_type) {
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT:
   /* The NT share is the same for a D3D12 fence: the exported object is this
    * WDDM monitored fence either way, and the handle type only records what
    * the importer will call it. This is the export half vkd3d requires before
    * it will create a shared ID3D12Fence at all. */
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT:
      return helios_wddm_sync_share_nt(renderer, sync->wddm_local, out_handle);
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_KMT_BIT:
      if (!sync->wddm_global)
         return VK_ERROR_INVALID_EXTERNAL_HANDLE;
      *out_handle = ULongToHandle(sync->wddm_global);
      return VK_SUCCESS;
   default:
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }
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
   info->has_external_sync = true;
   info->has_implicit_fencing = false;
   info->has_guest_vram = false;

   info->pci.vendor_id = 0x1af4;
   info->pci.device_id = 0x1050;
   /* Never fall back to the renderer's host PCI address.  Use the WDDM
    * adapter address when Dxgkrnl supplied it; otherwise hide the extension
    * because no address is more truthful than leaking an unrelated host BDF. */
   info->pci.hide_renderer_bus_info = true;
   info->pci.has_bus_info = helios->has_adapter_address;
   if (helios->has_adapter_address) {
      info->pci.props = (VkPhysicalDevicePCIBusInfoPropertiesEXT){
         .sType =
            VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PCI_BUS_INFO_PROPERTIES_EXT,
         .pciDomain = 0,
         .pciBus = helios->adapter_address.BusNumber,
         .pciDevice = helios->adapter_address.DeviceNumber,
         .pciFunction = helios->adapter_address.FunctionNumber,
      };
   }

   /* Report the guest WDDM adapter's LUID as VkPhysicalDeviceIDProperties::
    * deviceLUID, exactly as a native Windows Vulkan driver does. helios->adapter_luid
    * was captured from D3DKMTEnumAdapters2 in helios_open_d3dkmt (which runs before
    * this) and IS the same LUID DXGI/D3DKMT reports for our kmd_render adapter
    * (EnumAdapters1 AdapterLuid). Without this, deviceLUIDValid was false and no
    * VkPhysicalDevice carried the guest adapter LUID, so any app that selects an
    * adapter via DXGI and then matches it into Vulkan by deviceLUID (3DMark/UL
    * benchmarks, dxvk findAdapterByLuid, dxvk_adapter D3DKMTOpenAdapterFromLuid)
    * could not find the Vulkan device — "VkPhysicalDevice with device LUID X not
    * found". node_mask=1 mirrors a single-node adapter. LUID bytes are copied
    * verbatim (LowPart then HighPart); consumers memcpy them straight back into a
    * LUID struct. A zero LUID means the OS never assigned one — keep it invalid
    * rather than advertise a bogus all-zero identity. */
   _Static_assert(sizeof(helios->adapter_luid) == sizeof(info->id.luid),
                  "WDDM LUID size must equal Vulkan deviceLUID size");
   if (helios->adapter_luid.LowPart != 0 || helios->adapter_luid.HighPart != 0) {
      info->id.has_luid = true;
      info->id.node_mask = 1;
      memcpy(info->id.luid, &helios->adapter_luid, sizeof(info->id.luid));
      helios_diag("renderer info: deviceLUID=%08lx:%08lx node_mask=1",
                  (unsigned long)helios->adapter_luid.HighPart,
                  (unsigned long)helios->adapter_luid.LowPart);
   } else {
      info->id.has_luid = false;
      helios_diag("renderer info: WDDM adapter LUID is zero; deviceLUIDValid=false");
   }
}

static void
helios_destroy(struct vn_renderer *renderer, const VkAllocationCallbacks *alloc)
{
   struct helios *helios = (struct helios *)renderer;

   AcquireSRWLockExclusive(&helios_current_renderer_lock);
   if (helios_current_renderer == helios)
      helios_current_renderer = NULL;
   ReleaseSRWLockExclusive(&helios_current_renderer_lock);

   helios_perf_dump(helios);
   if (helios_perf_at_exit_renderer == helios)
      helios_perf_at_exit_renderer = NULL;

   /* Stop the retire thread BEFORE tearing down the escape channel: it blocks
    * only in bounded WAIT_FENCE slices and re-checks retire_stop between them,
    * so the join is prompt. Entries still queued drain WITHOUT marking (the
    * device is going away); their sync references drop in the worker. */
   mtx_lock(&helios->retire_mutex);
   helios->retire_stop = true;
   const bool join_retire = helios->retire_thread_live;
   cnd_signal(&helios->retire_cond);
   mtx_unlock(&helios->retire_mutex);
   /* Unpark an event-path fence wait (WaitForMultipleObjects) promptly. */
   if (helios->retire_stop_event)
      SetEvent(helios->retire_stop_event);
   if (join_retire)
      thrd_join(helios->retire_thread, NULL);
   if (helios->retire_stop_event) {
      CloseHandle(helios->retire_stop_event);
      helios->retire_stop_event = NULL;
   }

   /* Vulkan callers normally destroy every semaphore before VkDevice, but the
    * registered stream table is also a context-teardown backstop.  UNREGISTER
    * while the D3DKMT context is still valid; a failed best-effort escape does
    * not retain a stale local cookie. */
   mtx_lock(&helios->dev_mutex);
   helios_present_stream_unregister_all_locked(helios);
   mtx_unlock(&helios->dev_mutex);

   vn_renderer_shmem_cache_fini(&helios->shmem_cache);

   if (helios->ctx_id)
      helios_ioctl_ctx_destroy(helios, helios->ctx_id); /* CTX_DESTROY via escape */
   if (helios_current_ctx_id == helios->ctx_id)
      helios_current_ctx_id = 0;
   if (helios_calling_thread_ctx_id == helios->ctx_id)
      helios_calling_thread_ctx_id = 0;

   /* WDDM D3DKMT teardown (reverse of helios_open_d3dkmt). */
   if (helios->paging_queue) {
      D3DDDI_DESTROYPAGINGQUEUE destroy_queue;
      memset(&destroy_queue, 0, sizeof(destroy_queue));
      destroy_queue.hPagingQueue = helios->paging_queue;
      const NTSTATUS status = D3DKMTDestroyPagingQueue(&destroy_queue);
      if (status != 0)
         helios_diag("D3DKMTDestroyPagingQueue failed status=0x%08x",
                     (unsigned)status);
   }
   if (helios->context) {
      D3DKMT_DESTROYCONTEXT dc;
      memset(&dc, 0, sizeof(dc));
      dc.hContext = helios->context;
      (void)D3DKMTDestroyContext(&dc);
   }
   if (helios->device) {
      D3DKMT_DESTROYDEVICE dd;
      memset(&dd, 0, sizeof(dd));
      dd.hDevice = helios->device;
      (void)D3DKMTDestroyDevice(&dd);
   }
   if (helios->adapter) {
      D3DKMT_CLOSEADAPTER ca;
      memset(&ca, 0, sizeof(ca));
      ca.hAdapter = helios->adapter;
      (void)D3DKMTCloseAdapter(&ca);
   }

   /* The legacy IOCTL handle is never opened on the D3DKMT path, but guard it in
    * case a future transport reuses it. */
   if (helios->dev != INVALID_HANDLE_VALUE && helios->dev != NULL)
      CloseHandle(helios->dev);

   cnd_destroy(&helios->retire_cond);
   mtx_destroy(&helios->retire_mutex);
   mtx_destroy(&helios->dev_mutex);
   free(helios->submit_buf);

   vk_free(alloc, helios);
}

/* Hold this DLL in the process for as long as the process lives.
 *
 * The Vulkan loader FreeLibrary()s an ICD when its last VkInstance is
 * destroyed, and DXVK builds and tears down a VkInstance per D3D11 device, so
 * this module is loaded and unloaded ONCE PER DEVICE. Measured, not assumed:
 * a probe that creates and releases one D3D11 device sees this module appear
 * in and vanish from EnumProcessModules within that single call.
 *
 * Nothing releases a module's process- or thread-lifetime state on unload.
 * The loader closes no handles the module opened, and thread-specific-storage
 * destructors run at THREAD exit, not at module unload — so on a thread that
 * outlives the module (every caller's thread does) they never run at all.
 * Five kernel handles were therefore stranded per device, dead linear with no
 * plateau, of which only one belongs to code in this file:
 *
 *   Event      helios_fence_event_get (below) — the per-thread fence event
 *              held in tss, whose destructor the unload skips
 *   Event    \ winpthreads registering the caller's NATIVE thread: an event
 *   Thread   / plus a DuplicateHandle of the thread, freed at thread exit
 *   Semaphore x2  libgcc emutls_init -> __gthread_mutex_lock, i.e. the mutex
 *              objects winpthreads mints lazily for a static initialiser
 *
 * Four of the five are inside the mingw runtime we link statically, so no
 * DLL_PROCESS_DETACH hook we could write reaches them, and the fifth belongs
 * to threads this module does not own. Pinning is what makes the
 * process-lifetime assumption those statics are already written against true.
 * The deeper fix is upstream of here — a DXVK that shares one VkInstance
 * across D3D11 devices would stop the churn at its source, and would also
 * stop paying loader enumeration and ICD load per device — but that is a
 * different change with a different blast radius.
 *
 * Idempotent by construction (PIN on an already-pinned module is a no-op);
 * call_once keeps it to one syscall.
 */
static void
helios_pin_module(void)
{
   HMODULE self = NULL;
   if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                             GET_MODULE_HANDLE_EX_FLAG_PIN,
                          (LPCWSTR)(const void *)&helios_pin_module, &self)) {
      helios_diag("module pinned hmodule=%p", (void *)self);
      return;
   }
   /* Counted, not silent: without the pin every device strands five handles. */
   InterlockedIncrement(&helios_module_pin_failures);
   helios_diag("module PIN FAILED err=%lu — the per-device handle leak is live",
               (unsigned long)GetLastError());
}

static once_flag helios_pin_once = ONCE_FLAG_INIT;

static VkResult
helios_init(struct helios *helios)
{
   helios_diag("helios_init enter");
   call_once(&helios_pin_once, helios_pin_module);
   mtx_init(&helios->dev_mutex, mtx_plain);
   mtx_init(&helios->retire_mutex, mtx_plain);
   cnd_init(&helios->retire_cond);
   helios_perf_init(helios);
   if (helios->perf.enabled) {
      helios_perf_at_exit_renderer = helios;
      if (!helios_perf_at_exit_registered) {
         atexit(helios_perf_dump_at_exit);
         helios_perf_at_exit_registered = true;
      }
   }

   if (!helios_open_d3dkmt(helios)) {
      helios_diag("helios_open_d3dkmt failed");
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   /* Versioned ICD↔KMD capability handshake (no deploy-order assumptions):
    * one quiet probe decides event waits vs the blocking-escape fallback. */
   helios_probe_fence_events(helios);
   helios->retire_stop_event = CreateEventW(NULL, /*bManualReset*/ TRUE, FALSE, NULL);
   if (helios->fence_events_supported && !helios->retire_stop_event)
      helios_diag("retire_stop_event creation FAILED — retire thread will use "
                  "the slice-wait fallback");

   /* Create the single venus virtio-gpu context up front so it exists before the
    * first shmem/submit (analog of vtest_vcmd_context_init). */
   if (!helios_ioctl_ctx_create(helios, VIRTIO_GPU_CAPSET_VENUS, &helios->ctx_id) ||
       helios->ctx_id == 0) {
      vn_log(helios->instance, "CTX_CREATE(VENUS) failed or returned ctx_id 0");
      helios_diag("CTX_CREATE failed ctx_id=%u", helios->ctx_id);
      fprintf(stderr, "HELIOS[gate5a]: CTX_CREATE(VENUS) over D3DKMTEscape FAILED\n");
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   helios_diag("CTX_CREATE OK ctx_id=%u", helios->ctx_id);
   /* Gate 5a STAGE-1 SUCCESS SIGNAL: the venus context came up over D3DKMTEscape
    * → the KMD's DxgkDdiEscape handled CTX_CREATE against the kmd_render adapter. */
   fprintf(stderr, "HELIOS[gate5a]: CTX_CREATE(VENUS) over D3DKMTEscape OK ctx_id=%u\n",
           helios->ctx_id);
   helios_current_ctx_id = helios->ctx_id;
   helios_calling_thread_ctx_id = helios->ctx_id;

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
   helios->base.bo_ops.create_from_resource_id =
      helios_bo_create_from_resource_id;
   helios->base.bo_ops.destroy = helios_bo_destroy;
   helios->base.bo_ops.release_resource = helios_bo_release_resource;
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

   AcquireSRWLockExclusive(&helios_current_renderer_lock);
   helios_current_renderer = helios;
   ReleaseSRWLockExclusive(&helios_current_renderer_lock);

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
