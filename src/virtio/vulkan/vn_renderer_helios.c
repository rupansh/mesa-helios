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
#include "vn_device_memory.h"

#include "util/cache_ops.h"

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

#define HELIOS_MAP_CACHE_CACHED    0x00000001u
#define HELIOS_MAP_CACHE_UNCACHED  0x00000002u
#define HELIOS_MAP_CACHE_WC        0x00000003u

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
_Static_assert(sizeof(struct helios_escape_release_blob) == 32, "release_blob size");
_Static_assert(sizeof(struct helios_escape_attach_resource) == 24, "attach_resource size");
_Static_assert(sizeof(struct helios_escape_wait_fence) == 32, "wait_fence size");

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
   D3DKMT_HANDLE wddm_local;
   D3DKMT_HANDLE wddm_global;
   void *wddm_cpu_va;
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
   LUID adapter_luid;

   uint32_t ctx_id;
   uint64_t next_fence_id; /* monotonic, under dev_mutex */

   struct vn_renderer_shmem_cache shmem_cache;
   struct helios_perf_stats perf;
};

static struct helios *helios_perf_at_exit_renderer;
static bool helios_perf_at_exit_registered;
static uint32_t helios_current_ctx_id;

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

__declspec(dllexport) uint32_t helios_venus_current_ctx_id(void);
__declspec(dllexport) uint64_t helios_venus_memory_id(VkDeviceMemory memory);
__declspec(dllexport) uint32_t helios_venus_memory_res_id(VkDeviceMemory memory);
__declspec(dllexport) uint32_t helios_venus_memory_transfer_resource_ownership(VkDeviceMemory memory);

__declspec(dllexport) uint32_t
helios_venus_current_ctx_id(void)
{
   return helios_current_ctx_id;
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
   create.Info.Type = D3DDDI_MONITORED_FENCE;
   create.Info.Flags.Shared = nt_shared ? 1 : 0;
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
   helios_diag("sync_create MONITORED rejected status=0x%08x nt=%u; falling back to legacy fence",
               (unsigned)st, nt_shared ? 1u : 0u);

   memset(&create, 0, sizeof(create));
   create.hDevice = helios->device;
   create.Info.Type = D3DDDI_FENCE;
   create.Info.Flags.Shared = nt_shared ? 1 : 0;
   create.Info.Flags.NtSecuritySharing = nt_shared ? 1 : 0;
   create.Info.Fence.FenceValue = initial_val;

   st = D3DKMTCreateSynchronizationObject2(&create);
   if (st != 0) {
      helios_diag("sync_create failed status=0x%08x nt=%u initial=%llu dev=0x%x",
                  (unsigned)st, nt_shared ? 1u : 0u,
                  (unsigned long long)initial_val, (unsigned)helios->device);
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;
   }

   *out_local = create.hSyncObject;
   *out_global = create.Info.SharedHandle;
   if (out_cpu_va)
      *out_cpu_va = NULL;
   helios_diag("sync_create ok legacy-fence local=0x%x shared=0x%x nt=%u initial=%llu",
               (unsigned)*out_local, (unsigned)*out_global, nt_shared ? 1u : 0u,
               (unsigned long long)initial_val);
   return VK_SUCCESS;
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

static void
helios_trace_shmem(const char *event,
                   D3DKMT_HANDLE device,
                   D3DKMT_HANDLE context,
                   uint32_t ctx_id,
                   uint32_t res_id,
                   uint64_t size,
                   uint64_t user_va)
{
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
 * back into the same buffer, which the runtime reflects to user mode. Caller
 * MUST hold dev_mutex. Returns false on a D3DKMT failure. */
static bool
helios_escape(struct helios *helios, void *buf, uint32_t size)
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
   esc.Flags.HardwareAccess = 1;
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
   helios_trace_submit(helios, cs_data, cs_size, ring_idx, fence_id);

   /* Over D3DKMTEscape the venus stream rides INSIDE the escape buffer, directly
    * after the fixed header (the KMD reads it at buf[sizeof(hdr)..]); there is no
    * IN_DIRECT side buffer. Stage header+cs into one contiguous buffer. */
   const size_t total = sizeof(hdr) + cs_size;
   if (total > UINT32_MAX)
      return false;
   uint8_t *buf = malloc(total);
   if (!buf)
      return false;
   memcpy(buf, &hdr, sizeof(hdr));
   memcpy(buf + sizeof(hdr), cs_data, cs_size);
   const bool ok = helios_escape(helios, buf, (uint32_t)total);
   free(buf);

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

static bool
helios_ioctl_wait_fence(struct helios *helios, uint64_t fence_id, uint64_t timeout_ns)
{
   struct helios_escape_wait_fence req = { 0 };
   helios_hdr_init(&req.hdr, HELIOS_ESCAPE_WAIT_FENCE, sizeof(req));
   req.fence_id = fence_id;
   req.timeout_ns = timeout_ns;
   return helios_escape(helios, &req, sizeof(req));
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
      if (sync->val < val) {
         sync->val = val;
         if (sync->wddm_local)
            (void)helios_wddm_sync_signal(renderer, sync->wddm_local,
                                          sync->val);
      }
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
   /* Gate 5a bring-up breadcrumb (stderr): adapter opened by LUID. */
   fprintf(stderr, "HELIOS[gate5a]: opened Helios WDDM adapter hAdapter=0x%x luid=%08lx:%08lx\n",
           (unsigned)chosen_adapter, (unsigned long)chosen_luid.HighPart,
           (unsigned long)chosen_luid.LowPart);

   fprintf(stderr, "HELIOS[gate5a]: D3DKMT device=0x%x context=0x%x\n",
           (unsigned)helios->device, (unsigned)helios->context);
   return true;
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
         if (!helios_sync_append_locked(renderer, sync, batch->sync_values[j],
                                        fence_id)) {
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
         helios_sync_append_locked(renderer, sync, batch->sync_values[j], 0);
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

   mtx_lock(&helios->dev_mutex);
   if (sync->wddm_local)
      helios_wddm_sync_destroy(renderer, sync->wddm_local);
   sync->wddm_local = 0;
   sync->wddm_global = 0;
   sync->wddm_cpu_va = NULL;
   mtx_unlock(&helios->dev_mutex);

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

   VkResult result;
   switch (handle_type) {
   case VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT:
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

   if (helios->ctx_id)
      helios_ioctl_ctx_destroy(helios, helios->ctx_id); /* CTX_DESTROY via escape */
   if (helios_current_ctx_id == helios->ctx_id)
      helios_current_ctx_id = 0;

   /* WDDM D3DKMT teardown (reverse of helios_open_d3dkmt). */
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

   mtx_destroy(&helios->dev_mutex);

   vk_free(alloc, helios);
}

static VkResult
helios_init(struct helios *helios)
{
   helios_diag("helios_init enter");
   mtx_init(&helios->dev_mutex, mtx_plain);
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
