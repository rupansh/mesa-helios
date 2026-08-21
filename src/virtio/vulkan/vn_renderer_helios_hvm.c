/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A3 -- escape-free Windows renderer ownership.
 *
 * One renderer owns one HTS1 session.  Every allocation visible to this file
 * is a process-local WDDM handle paired with its immutable KMD generation.
 * Host resource identifiers never enter this address space or the Venus wire.
 */

#ifdef _WIN32

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <windows.h>
#include <winternl.h>

#include "vn_renderer.h"
#ifndef _NTDEF_
typedef LONG NTSTATUS, *PNTSTATUS;
#endif
#include <d3dkmthk.h>

#include "venus-protocol/vn_protocol_driver_device_memory.h"
#include "venus-protocol/vn_protocol_driver_transport.h"

#include "vn_cs.h"
#include "vn_helios_hwa2.h"
#include "vn_helios_native_kmt.h"
#include "vn_helios_translation_session.h"
#include "vn_instance.h"

#define HELIOS_CPU_VISIBLE_MAX_BYTES (UINT64_C(1024) * UINT64_C(4096))
#define HELIOS_PRIVATE_DATA_LIMIT    (1024u * 1024u)
#define HELIOS_SESSION_ENDPOINTS     64u
#define HELIOS_PAGE_BYTES            UINT64_C(4096)

#define HELIOS_IGNORE_STATUS(call)                                           \
   do {                                                                      \
      NTSTATUS ignored_ = (call);                                            \
      (void)ignored_;                                                        \
   } while (0)

struct helios_allocation {
   D3DKMT_HANDLE resource;
   D3DKMT_HANDLE allocation;
   uint64_t generation;
   uint64_t size;
   uint32_t role;
   void *cpu;
   bool imported;
};

struct helios_shmem {
   struct vn_renderer_shmem base;
   struct helios_allocation allocation;
};

struct helios_bo {
   struct vn_renderer_bo base;
   struct helios_allocation allocation;
};

struct helios_sync {
   struct vn_renderer_sync base;
   struct helios_allocation feedback;
   D3DKMT_HANDLE sync;
   D3DDDI_NATIVEFENCEMAPPING mapping;
   uint64_t native_generation;
   bool native;
};

struct vn_renderer_helios_external_memory {
   D3DKMT_HANDLE resource;
   D3DKMT_HANDLE allocation;
};

struct helios {
   struct vn_renderer base;
   struct vn_instance *instance;
   struct helios_translation_session *session;
   struct helios_native_context *bootstrap;
   D3DKMT_HANDLE device;
   D3DKMT_HANDLE paging_queue;
   D3DKMT_HANDLE paging_fence;
   LUID adapter_luid;
   D3DKMT_ADAPTERADDRESS adapter_address;
   bool has_adapter_address;
   CRITICAL_SECTION allocation_lock;
   bool allocation_lock_live;
   CRITICAL_SECTION bootstrap_lock;
   bool bootstrap_lock_live;
};

static inline struct helios *
helios_from_renderer(struct vn_renderer *renderer)
{
   return (struct helios *)renderer;
}

static inline const struct helios *
helios_from_renderer_const(const struct vn_renderer *renderer)
{
   return (const struct helios *)renderer;
}

void
vn_renderer_helios_diag_log(const char *fmt, ...)
{
   char line[1024];
   va_list ap;
   va_start(ap, fmt);
   int n = vsnprintf(line, sizeof(line) - 2, fmt, ap);
   va_end(ap);
   if (n < 0)
      return;
   size_t used = (size_t)n < sizeof(line) - 2 ? (size_t)n : sizeof(line) - 2;
   line[used++] = '\n';
   line[used] = '\0';
   OutputDebugStringA(line);
}

static uint64_t
helios_round_page(uint64_t size)
{
   if (!size || size > UINT32_MAX)
      return 0;
   const uint64_t rounded =
      (size + HELIOS_PAGE_BYTES - 1) & ~(HELIOS_PAGE_BYTES - 1);
   return rounded >= size && rounded <= UINT32_MAX ? rounded : 0;
}

static void
helios_close_enum_adapter(D3DKMT_HANDLE adapter)
{
   if (!adapter)
      return;
   D3DKMT_CLOSEADAPTER close = { .hAdapter = adapter };
   HELIOS_IGNORE_STATUS(D3DKMTCloseAdapter(&close));
}

/* Direct OS-owned adapter discovery.  There is no probe Escape and no
 * fallback: exactly one adapter must identify itself as Helios through
 * registry info. */
static VkResult
helios_find_adapter(struct helios *helios)
{
   D3DKMT_ENUMADAPTERS2 enumerate;
   memset(&enumerate, 0, sizeof(enumerate));
   NTSTATUS st = D3DKMTEnumAdapters2(&enumerate);
   if (st != 0 || !enumerate.NumAdapters)
      return VK_ERROR_INITIALIZATION_FAILED;

   D3DKMT_ADAPTERINFO *adapters =
      calloc(enumerate.NumAdapters, sizeof(*adapters));
   if (!adapters)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   enumerate.pAdapters = adapters;
   st = D3DKMTEnumAdapters2(&enumerate);
   if (st != 0) {
      free(adapters);
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   uint32_t matches = 0;
   for (uint32_t i = 0; i < enumerate.NumAdapters; i++) {
      D3DKMT_ADAPTERREGISTRYINFO registry;
      memset(&registry, 0, sizeof(registry));
      D3DKMT_QUERYADAPTERINFO query;
      memset(&query, 0, sizeof(query));
      query.hAdapter = adapters[i].hAdapter;
      query.Type = KMTQAITYPE_ADAPTERREGISTRYINFO;
      query.pPrivateDriverData = &registry;
      query.PrivateDriverDataSize = sizeof(registry);
      if (D3DKMTQueryAdapterInfo(&query) == 0 &&
          wcsstr(registry.AdapterString, L"Helios")) {
         matches++;
         helios->adapter_luid = adapters[i].AdapterLuid;

         D3DKMT_ADAPTERADDRESS address;
         memset(&address, 0, sizeof(address));
         query.Type = KMTQAITYPE_ADAPTERADDRESS_RENDER;
         query.pPrivateDriverData = &address;
         query.PrivateDriverDataSize = sizeof(address);
         if (D3DKMTQueryAdapterInfo(&query) == 0) {
            helios->adapter_address = address;
            helios->has_adapter_address = true;
         }
      }
   }
   for (uint32_t i = 0; i < enumerate.NumAdapters; i++)
      helios_close_enum_adapter(adapters[i].hAdapter);
   free(adapters);

   if (matches != 1 || (helios->adapter_luid.LowPart == 0 &&
                        helios->adapter_luid.HighPart == 0)) {
      memset(&helios->adapter_luid, 0, sizeof(helios->adapter_luid));
      return VK_ERROR_INITIALIZATION_FAILED;
   }
   return VK_SUCCESS;
}

/* A5 direct instances do not discover an adapter.  Open exactly the supplied
 * LUID once to prove that it is available and to collect diagnostic PCI
 * address data, then let the HTS1 session open that same LUID for ownership. */
static VkResult
helios_select_direct_adapter(struct helios *helios,
                             uint32_t luid_low,
                             int32_t luid_high)
{
   LUID luid = {
      .LowPart = luid_low,
      .HighPart = luid_high,
   };
   if (luid.LowPart == 0 && luid.HighPart == 0)
      return VK_ERROR_INITIALIZATION_FAILED;

   D3DKMT_OPENADAPTERFROMLUID open = { .AdapterLuid = luid };
   if (D3DKMTOpenAdapterFromLuid(&open) != 0 || !open.hAdapter)
      return VK_ERROR_INITIALIZATION_FAILED;

   helios->adapter_luid = luid;
   D3DKMT_ADAPTERADDRESS address;
   memset(&address, 0, sizeof(address));
   D3DKMT_QUERYADAPTERINFO query;
   memset(&query, 0, sizeof(query));
   query.hAdapter = open.hAdapter;
   query.Type = KMTQAITYPE_ADAPTERADDRESS_RENDER;
   query.pPrivateDriverData = &address;
   query.PrivateDriverDataSize = sizeof(address);
   if (D3DKMTQueryAdapterInfo(&query) == 0) {
      helios->adapter_address = address;
      helios->has_adapter_address = true;
   }
   helios_close_enum_adapter(open.hAdapter);
   return VK_SUCCESS;
}

static VkResult
helios_wait_paging(struct helios *helios, uint64_t value)
{
   if (!value)
      return VK_SUCCESS;
   D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU wait;
   memset(&wait, 0, sizeof(wait));
   wait.hDevice = helios->device;
   wait.ObjectCount = 1;
   wait.ObjectHandleArray = &helios->paging_fence;
   wait.FenceValueArray = &value;
   return D3DKMTWaitForSynchronizationObjectFromCpu(&wait) == 0
             ? VK_SUCCESS
             : VK_ERROR_DEVICE_LOST;
}

static void
helios_allocation_destroy_locked(struct helios *helios,
                                 struct helios_allocation *allocation)
{
   if (allocation->cpu) {
      D3DKMT_UNLOCK2 unlock = {
         .hDevice = helios->device,
         .hAllocation = allocation->allocation,
      };
      HELIOS_IGNORE_STATUS(D3DKMTUnlock2(&unlock));
      allocation->cpu = NULL;
   }
   if (allocation->resource) {
      D3DKMT_DESTROYALLOCATION2 destroy;
      memset(&destroy, 0, sizeof(destroy));
      destroy.hDevice = helios->device;
      destroy.hResource = allocation->resource;
      HELIOS_IGNORE_STATUS(D3DKMTDestroyAllocation2(&destroy));
   }
   memset(allocation, 0, sizeof(*allocation));
}

static void
helios_allocation_destroy(struct helios *helios,
                          struct helios_allocation *allocation)
{
   EnterCriticalSection(&helios->allocation_lock);
   helios_allocation_destroy_locked(helios, allocation);
   LeaveCriticalSection(&helios->allocation_lock);
}

static VkResult
helios_allocation_create(struct helios *helios,
                         uint64_t requested_size,
                         uint32_t role,
                         struct helios_allocation *out)
{
   memset(out, 0, sizeof(*out));
   const uint64_t size = helios_round_page(requested_size);
   const bool cpu_visible = role != HELIOS_HVM1_ROLE_VULKAN_DEVICE_LOCAL;
   if (!size || (cpu_visible && size > HELIOS_CPU_VISIBLE_MAX_BYTES) ||
       (role != HELIOS_HVM1_ROLE_VULKAN_HOST_VISIBLE &&
        role != HELIOS_HVM1_ROLE_FEEDBACK &&
        role != HELIOS_HVM1_ROLE_VULKAN_DEVICE_LOCAL))
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;

   HeliosVenusMemoryAllocationV1 hvm1;
   memset(&hvm1, 0, sizeof(hvm1));
   hvm1.magic = HELIOS_HVM1_MAGIC;
   hvm1.abi_version = HELIOS_HVM1_ABI_VERSION;
   hvm1.struct_size = HELIOS_HVM1_SIZE;
   hvm1.package_generation = HELIOS_PACKAGE_GENERATION;
   hvm1.byte_size = size;
   hvm1.role = role;
   hvm1.cache_policy = cpu_visible ? HELIOS_HVM1_CACHE_WRITE_COMBINED
                                   : HELIOS_HVM1_CACHE_NOT_CPU_VISIBLE;
   switch (role) {
   case HELIOS_HVM1_ROLE_VULKAN_HOST_VISIBLE:
      hvm1.access =
         HELIOS_HVM1_ACCESS_CPU_READ | HELIOS_HVM1_ACCESS_CPU_WRITE |
         HELIOS_HVM1_ACCESS_HOST_READ | HELIOS_HVM1_ACCESS_HOST_WRITE;
      break;
   case HELIOS_HVM1_ROLE_FEEDBACK:
      hvm1.access =
         HELIOS_HVM1_ACCESS_CPU_READ | HELIOS_HVM1_ACCESS_HOST_WRITE;
      break;
   case HELIOS_HVM1_ROLE_VULKAN_DEVICE_LOCAL:
      hvm1.access =
         HELIOS_HVM1_ACCESS_HOST_READ | HELIOS_HVM1_ACCESS_HOST_WRITE;
      break;
   default:
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   D3DDDI_ALLOCATIONINFO2 info;
   memset(&info, 0, sizeof(info));
   info.pSystemMem = NULL;
   info.pPrivateDriverData = &hvm1;
   info.PrivateDriverDataSize = sizeof(hvm1);

   D3DKMT_CREATEALLOCATION create;
   memset(&create, 0, sizeof(create));
   create.hDevice = helios->device;
   create.NumAllocations = 1;
   create.pAllocationInfo2 = &info;
   create.Flags.CreateResource = 1;
   create.Flags.CreateShared = 1;
   create.Flags.NtSecuritySharing = 1;

   EnterCriticalSection(&helios->allocation_lock);
   NTSTATUS st = D3DKMTCreateAllocation2(&create);
   if (st != 0 || !create.hResource || !info.hAllocation ||
       !hvm1.object_generation) {
      if (create.hResource) {
         struct helios_allocation partial = {
            .resource = create.hResource,
            .allocation = info.hAllocation,
         };
         helios_allocation_destroy_locked(helios, &partial);
      }
      LeaveCriticalSection(&helios->allocation_lock);
      return st == (NTSTATUS)0xC00000BBL ? VK_ERROR_FEATURE_NOT_PRESENT
                                         : VK_ERROR_OUT_OF_DEVICE_MEMORY;
   }

   struct helios_allocation allocation = {
      .resource = create.hResource,
      .allocation = info.hAllocation,
      .generation = hvm1.object_generation,
      .size = size,
      .role = role,
   };

   D3DDDI_MAKERESIDENT resident;
   memset(&resident, 0, sizeof(resident));
   resident.hPagingQueue = helios->paging_queue;
   resident.NumAllocations = 1;
   resident.AllocationList = &allocation.allocation;
   st = D3DKMTMakeResident(&resident);
   if (st != 0 && st != (NTSTATUS)0x00000103L) {
      helios_allocation_destroy_locked(helios, &allocation);
      LeaveCriticalSection(&helios->allocation_lock);
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;
   }
   VkResult result = helios_wait_paging(helios, resident.PagingFenceValue);
   if (result != VK_SUCCESS) {
      helios_allocation_destroy_locked(helios, &allocation);
      LeaveCriticalSection(&helios->allocation_lock);
      return result;
   }

   if (cpu_visible) {
      D3DKMT_LOCK2 lock = {
         .hDevice = helios->device,
         .hAllocation = allocation.allocation,
      };
      st = D3DKMTLock2(&lock);
      if (st != 0 || !lock.pData) {
         helios_allocation_destroy_locked(helios, &allocation);
         LeaveCriticalSection(&helios->allocation_lock);
         return VK_ERROR_MEMORY_MAP_FAILED;
      }
      allocation.cpu = lock.pData;
   }
   LeaveCriticalSection(&helios->allocation_lock);
   *out = allocation;
   return VK_SUCCESS;
}

static struct vn_renderer_shmem *
helios_shmem_create(struct vn_renderer *renderer, size_t requested_size)
{
   struct helios *helios = helios_from_renderer(renderer);
   struct helios_shmem *shmem = calloc(1, sizeof(*shmem));
   if (!shmem)
      return NULL;
   VkResult result = helios_allocation_create(
      helios, requested_size, HELIOS_HVM1_ROLE_VULKAN_HOST_VISIBLE,
      &shmem->allocation);
   if (result != VK_SUCCESS) {
      free(shmem);
      return NULL;
   }
   shmem->base.refcount = VN_REFCOUNT_INIT(1);
   shmem->base.allocation_handle = shmem->allocation.allocation;
   shmem->base.allocation_generation = shmem->allocation.generation;
   shmem->base.allocation_role = shmem->allocation.role;
   shmem->base.mmap_size = shmem->allocation.size;
   shmem->base.mmap_ptr = shmem->allocation.cpu;
   return &shmem->base;
}

static void
helios_shmem_destroy(struct vn_renderer *renderer,
                     struct vn_renderer_shmem *base)
{
   struct helios_shmem *shmem = (struct helios_shmem *)base;
   helios_allocation_destroy(helios_from_renderer(renderer),
                             &shmem->allocation);
   free(shmem);
}

static VkResult
helios_bo_create_from_device_memory(
   struct vn_renderer *renderer,
   struct vn_renderer_submit_batch *batch,
   VkDeviceSize size,
   vn_object_id memory_id,
   VkMemoryPropertyFlags flags,
   VkExternalMemoryHandleTypeFlags external_handles,
   struct vn_renderer_bo **out_bo)
{
   *out_bo = NULL;
   if (batch || !memory_id || external_handles)
      return VK_ERROR_FEATURE_NOT_PRESENT;
   struct helios_bo *bo = calloc(1, sizeof(*bo));
   if (!bo)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   const uint32_t role = (flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
                            ? HELIOS_HVM1_ROLE_VULKAN_HOST_VISIBLE
                            : HELIOS_HVM1_ROLE_VULKAN_DEVICE_LOCAL;
   VkResult result = helios_allocation_create(helios_from_renderer(renderer),
                                              size, role, &bo->allocation);
   if (result != VK_SUCCESS) {
      free(bo);
      return result;
   }
   bo->base.refcount = VN_REFCOUNT_INIT(1);
   bo->base.allocation_handle = bo->allocation.allocation;
   bo->base.allocation_generation = bo->allocation.generation;
   bo->base.allocation_role = bo->allocation.role;
   bo->base.mmap_size = bo->allocation.cpu ? bo->allocation.size : 0;
   bo->base.mmap_ptr = bo->allocation.cpu;
   *out_bo = &bo->base;
   return VK_SUCCESS;
}

static bool
helios_bo_destroy(struct vn_renderer *renderer, struct vn_renderer_bo *base)
{
   struct helios_bo *bo = (struct helios_bo *)base;
   if (!bo->allocation.imported)
      helios_allocation_destroy(helios_from_renderer(renderer),
                                &bo->allocation);
   free(bo);
   return true;
}

static void *
helios_bo_map(struct vn_renderer *renderer,
              struct vn_renderer_bo *base,
              void *placed_address)
{
   (void)renderer;
   struct helios_bo *bo = (struct helios_bo *)base;
   if (placed_address ||
       bo->allocation.role == HELIOS_HVM1_ROLE_VULKAN_DEVICE_LOCAL ||
       bo->allocation.imported)
      return NULL;
   return bo->allocation.cpu;
}

static void
helios_bo_cache_op(struct vn_renderer *renderer,
                   struct vn_renderer_bo *bo,
                   VkDeviceSize offset,
                   VkDeviceSize size)
{
   (void)renderer;
   (void)bo;
   (void)offset;
   (void)size;
   /* Roles 1-3 are WC and the exposed host-visible type is coherent. */
}

static VkResult
helios_sync_create(struct vn_renderer *renderer,
                   uint64_t initial_value,
   uint32_t flags,
   struct vn_renderer_sync **out_sync)
{
   *out_sync = NULL;
   const uint32_t known_flags =
      VN_RENDERER_SYNC_SHAREABLE | VN_RENDERER_SYNC_BINARY;
   if ((flags & ~known_flags) ||
       ((flags & VN_RENDERER_SYNC_SHAREABLE) &&
        (flags & VN_RENDERER_SYNC_BINARY)))
      return VK_ERROR_FEATURE_NOT_PRESENT;
   struct helios_sync *sync = calloc(1, sizeof(*sync));
   if (!sync)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   if (flags & VN_RENDERER_SYNC_SHAREABLE) {
      struct helios *helios = helios_from_renderer(renderer);
      D3DKMT_CREATESYNCHRONIZATIONOBJECT2 create;
      memset(&create, 0, sizeof(create));
      create.hDevice = helios->device;
      create.Info.Type = D3DDDI_MONITORED_FENCE;
      create.Info.Flags.Shared = 1;
      create.Info.Flags.NtSecuritySharing = 1;
      create.Info.MonitoredFence.InitialFenceValue = initial_value;
      create.Info.MonitoredFence.EngineAffinity = 1u;
      const NTSTATUS st = D3DKMTCreateSynchronizationObject2(&create);
      if (st != 0 || !create.hSyncObject ||
          !create.Info.MonitoredFence.FenceValueCPUVirtualAddress ||
          !create.Info.MonitoredFence.FenceValueGPUVirtualAddress) {
         if (create.hSyncObject) {
            D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy = {
               .hSyncObject = create.hSyncObject,
            };
            HELIOS_IGNORE_STATUS(
               D3DKMTDestroySynchronizationObject(&destroy));
         }
         free(sync);
         return VK_ERROR_OUT_OF_DEVICE_MEMORY;
      }
      sync->sync = create.hSyncObject;
      sync->mapping.CurrentValueCpuVa =
         create.Info.MonitoredFence.FenceValueCPUVirtualAddress;
      sync->mapping.CurrentValueGpuVa =
         create.Info.MonitoredFence.FenceValueGPUVirtualAddress;
      sync->native = true;
      sync->base.allocation_handle = create.hSyncObject;
      *out_sync = &sync->base;
      return VK_SUCCESS;
   }

   if (initial_value) {
      free(sync);
      return VK_ERROR_FEATURE_NOT_PRESENT;
   }
   VkResult result = helios_allocation_create(
      helios_from_renderer(renderer), HELIOS_PAGE_BYTES,
      HELIOS_HVM1_ROLE_FEEDBACK, &sync->feedback);
   if (result != VK_SUCCESS) {
      free(sync);
      return result;
   }
   sync->base.allocation_handle = sync->feedback.allocation;
   sync->base.allocation_generation = sync->feedback.generation;
   *out_sync = &sync->base;
   return VK_SUCCESS;
}

static void
helios_sync_destroy(struct vn_renderer *renderer,
                    struct vn_renderer_sync *base)
{
   if (!base)
      return;
   struct helios_sync *sync = (struct helios_sync *)base;
   if (sync->sync) {
      D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy = {
         .hSyncObject = sync->sync,
      };
      HELIOS_IGNORE_STATUS(D3DKMTDestroySynchronizationObject(&destroy));
      sync->sync = 0;
   }
   if (sync->feedback.resource)
      helios_allocation_destroy(helios_from_renderer(renderer),
                                &sync->feedback);
   free(sync);
}

static VkResult
helios_sync_reset(struct vn_renderer *renderer,
                  struct vn_renderer_sync *sync,
                  uint64_t initial_value)
{
   (void)renderer;
   (void)sync;
   (void)initial_value;
   return VK_ERROR_FEATURE_NOT_PRESENT;
}

static VkResult
helios_sync_read(struct vn_renderer *renderer,
                 struct vn_renderer_sync *base,
                 uint64_t *value)
{
   (void)renderer;
   struct helios_sync *sync = (struct helios_sync *)base;
   if (sync->native && sync->mapping.CurrentValueCpuVa) {
      memcpy(value, sync->mapping.CurrentValueCpuVa, sizeof(*value));
      return VK_SUCCESS;
   }
   if (!sync->native && sync->feedback.cpu) {
      memcpy(value, sync->feedback.cpu, sizeof(*value));
      return VK_SUCCESS;
   }
   return VK_ERROR_DEVICE_LOST;
}

static VkResult
helios_sync_write(struct vn_renderer *renderer,
                  struct vn_renderer_sync *base,
                  uint64_t value)
{
   struct helios_sync *sync = (struct helios_sync *)base;
   if (!sync->native || !sync->sync)
      return VK_ERROR_FEATURE_NOT_PRESENT;
   struct helios *helios = helios_from_renderer(renderer);
   D3DKMT_HANDLE object = sync->sync;
   UINT64 fence_value = value;
   D3DKMT_SIGNALSYNCHRONIZATIONOBJECTFROMCPU signal;
   memset(&signal, 0, sizeof(signal));
   signal.hDevice = helios->device;
   signal.ObjectCount = 1;
   signal.ObjectHandleArray = &object;
   signal.FenceValueArray = &fence_value;
   return D3DKMTSignalSynchronizationObjectFromCpu(&signal) == 0
             ? VK_SUCCESS
             : VK_ERROR_DEVICE_LOST;
}

static uint16_t
helios_load_u16(const uint8_t *bytes, size_t offset)
{
   uint16_t value;
   memcpy(&value, bytes + offset, sizeof(value));
   return value;
}

static uint32_t
helios_load_u32(const uint8_t *bytes, size_t offset)
{
   uint32_t value;
   memcpy(&value, bytes + offset, sizeof(value));
   return value;
}

static uint64_t
helios_load_u64(const uint8_t *bytes, size_t offset)
{
   uint64_t value;
   memcpy(&value, bytes + offset, sizeof(value));
   return value;
}

static void
helios_store_u16(uint8_t *bytes, size_t offset, uint16_t value)
{
   memcpy(bytes + offset, &value, sizeof(value));
}

static void
helios_store_u32(uint8_t *bytes, size_t offset, uint32_t value)
{
   memcpy(bytes + offset, &value, sizeof(value));
}

static void
helios_store_u64(uint8_t *bytes, size_t offset, uint64_t value)
{
   memcpy(bytes + offset, &value, sizeof(value));
}

VkResult
vn_renderer_helios_sync_create_from_win32(
   struct vn_renderer *renderer,
   VkExternalSemaphoreHandleTypeFlagBits handle_type,
   void *handle,
   struct vn_renderer_sync **out_sync)
{
   enum {
      HNF1_MAGIC = 0x31464e48u,
      HNF1_ABI = 1,
      HNF1_BYTES = 64,
      HNF1_FLAG_SHARED = 1,
   };
   *out_sync = NULL;
   if (!handle ||
       handle_type != VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   struct helios *helios = helios_from_renderer(renderer);
   D3DKMT_OPENNATIVEFENCEFROMNTHANDLE open;
   memset(&open, 0, sizeof(open));
   open.hNtHandle = handle;
   open.hDevice = helios->device;
   open.EngineAffinity = 1u;
   open.Flags.Value = 0;
   open.Flags.Shared = 1;
   open.Flags.NtSecuritySharing = 1;
   helios_store_u32(open.PrivateDriverData, 0, HNF1_MAGIC);
   helios_store_u16(open.PrivateDriverData, 4, HNF1_ABI);
   helios_store_u16(open.PrivateDriverData, 6, HNF1_BYTES);
   helios_store_u64(open.PrivateDriverData, 8, HELIOS_PACKAGE_GENERATION);
   helios_store_u32(open.PrivateDriverData, 24, 0);
   helios_store_u32(open.PrivateDriverData, 28, HNF1_FLAG_SHARED);
   int64_t luid = 0;
   memcpy(&luid, &helios->adapter_luid, sizeof(luid));
   helios_store_u64(open.PrivateDriverData, 32, (uint64_t)luid);

   NTSTATUS st = D3DKMTOpenNativeFenceFromNtHandle(&open);
   const uint64_t generation = helios_load_u64(open.PrivateDriverData, 16);
   uint8_t reserved_or = 0;
   for (size_t i = 40; i < HNF1_BYTES; i++)
      reserved_or |= open.PrivateDriverData[i];
   for (size_t i = 0; i < sizeof(open.NativeFenceMapping.Reserved); i++)
      reserved_or |= open.NativeFenceMapping.Reserved[i];
   for (size_t i = 0; i < sizeof(open.Reserved); i++)
      reserved_or |= open.Reserved[i];
   const uintptr_t current_cpu =
      (uintptr_t)open.NativeFenceMapping.CurrentValueCpuVa;
   if (st != 0 || !open.hSyncObject ||
       !open.NativeFenceMapping.CurrentValueCpuVa ||
       !open.NativeFenceMapping.CurrentValueGpuVa ||
       !open.NativeFenceMapping.MonitoredValueGpuVa ||
       (current_cpu & (sizeof(uint64_t) - 1)) ||
       (open.NativeFenceMapping.CurrentValueGpuVa & (sizeof(uint64_t) - 1)) ||
       (open.NativeFenceMapping.MonitoredValueGpuVa &
        (sizeof(uint64_t) - 1)) ||
       helios_load_u32(open.PrivateDriverData, 0) != HNF1_MAGIC ||
       helios_load_u16(open.PrivateDriverData, 4) != HNF1_ABI ||
       helios_load_u16(open.PrivateDriverData, 6) != HNF1_BYTES ||
       helios_load_u64(open.PrivateDriverData, 8) !=
          HELIOS_PACKAGE_GENERATION ||
       !generation || helios_load_u32(open.PrivateDriverData, 24) != 0 ||
       helios_load_u32(open.PrivateDriverData, 28) != HNF1_FLAG_SHARED ||
       helios_load_u64(open.PrivateDriverData, 32) != (uint64_t)luid ||
       reserved_or) {
      if (open.hSyncObject) {
         D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy = {
            .hSyncObject = open.hSyncObject,
         };
         HELIOS_IGNORE_STATUS(D3DKMTDestroySynchronizationObject(&destroy));
      }
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   struct helios_sync *sync = calloc(1, sizeof(*sync));
   if (!sync) {
      D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy = {
         .hSyncObject = open.hSyncObject,
      };
      HELIOS_IGNORE_STATUS(D3DKMTDestroySynchronizationObject(&destroy));
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   sync->base.allocation_handle = open.hSyncObject;
   sync->base.allocation_generation = generation;
   sync->sync = open.hSyncObject;
   sync->mapping = open.NativeFenceMapping;
   sync->native_generation = generation;
   sync->native = true;
   *out_sync = &sync->base;
   return VK_SUCCESS;
}

uint32_t
vn_renderer_helios_sync_handle(const struct vn_renderer_sync *base)
{
   const struct helios_sync *sync = (const struct helios_sync *)base;
   return sync && sync->native ? sync->sync : 0;
}

VkResult
vn_renderer_helios_sync_export_win32(
   struct vn_renderer *renderer,
   struct vn_renderer_sync *base,
   VkExternalSemaphoreHandleTypeFlagBits handle_type,
   void **out_handle)
{
   (void)renderer;
   if (out_handle)
      *out_handle = NULL;
   struct helios_sync *sync = (struct helios_sync *)base;
   if (!out_handle || !sync || !sync->native || !sync->sync ||
       handle_type !=
          VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   D3DKMT_HANDLE object = sync->sync;
   OBJECT_ATTRIBUTES attributes;
   memset(&attributes, 0, sizeof(attributes));
   attributes.Length = sizeof(attributes);
   HANDLE handle = NULL;
   const NTSTATUS st =
      D3DKMTShareObjects(1, &object, &attributes, GENERIC_ALL, &handle);
   if (st != 0 || !handle) {
      if (handle)
         CloseHandle(handle);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }
   *out_handle = handle;
   return VK_SUCCESS;
}

static void
helios_destroy_auxiliary_open_handles(D3DKMT_HANDLE keyed_mutex,
                                      D3DKMT_HANDLE sync_object)
{
   if (sync_object) {
      D3DKMT_DESTROYSYNCHRONIZATIONOBJECT destroy = {
         .hSyncObject = sync_object,
      };
      HELIOS_IGNORE_STATUS(D3DKMTDestroySynchronizationObject(&destroy));
   }
   if (keyed_mutex) {
      D3DKMT_DESTROYKEYEDMUTEX destroy = { .hKeyedMutex = keyed_mutex };
      HELIOS_IGNORE_STATUS(D3DKMTDestroyKeyedMutex(&destroy));
   }
}

VkResult
vn_renderer_helios_external_memory_open(
   struct vn_renderer *renderer,
   const VkImportMemoryWin32HandleInfoKHR *import_info,
   uint64_t allocation_size,
   uint64_t *out_allocation_size,
   HeliosWddmAllocationDescV2 *out_desc,
   struct vn_renderer_bo **out_bo,
   struct vn_renderer_helios_external_memory **out_external)
{
   *out_allocation_size = 0;
   if (out_desc)
      memset(out_desc, 0, sizeof(*out_desc));
   *out_bo = NULL;
   *out_external = NULL;
   if (!import_info ||
       import_info->handleType !=
          VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT ||
       !import_info->handle || import_info->name)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   struct helios *helios = helios_from_renderer(renderer);
   D3DKMT_QUERYRESOURCEINFOFROMNTHANDLE query;
   memset(&query, 0, sizeof(query));
   query.hDevice = helios->device;
   query.hNtHandle = import_info->handle;
   NTSTATUS st = D3DKMTQueryResourceInfoFromNtHandle(&query);
   if (st != 0 || query.NumAllocations != 1 ||
       query.PrivateRuntimeDataSize > HELIOS_PRIVATE_DATA_LIMIT ||
       query.ResourcePrivateDriverDataSize > HELIOS_PRIVATE_DATA_LIMIT ||
       query.TotalPrivateDriverDataSize > HELIOS_PRIVATE_DATA_LIMIT)
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;

   const uint32_t runtime_bytes = query.PrivateRuntimeDataSize;
   const uint32_t resource_private_bytes =
      query.ResourcePrivateDriverDataSize;
   const uint32_t total_private_bytes = query.TotalPrivateDriverDataSize;
   void *runtime = query.PrivateRuntimeDataSize
                      ? calloc(1, query.PrivateRuntimeDataSize)
                      : NULL;
   if (runtime_bytes && !runtime)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   query.pPrivateRuntimeData = runtime;
   if (runtime_bytes)
      st = D3DKMTQueryResourceInfoFromNtHandle(&query);
   if (st != 0 || query.NumAllocations != 1 ||
       query.PrivateRuntimeDataSize != runtime_bytes ||
       query.ResourcePrivateDriverDataSize != resource_private_bytes ||
       query.TotalPrivateDriverDataSize != total_private_bytes) {
      free(runtime);
      return VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   void *resource_private =
      resource_private_bytes ? calloc(1, resource_private_bytes) : NULL;
   void *total_private =
      total_private_bytes ? calloc(1, total_private_bytes) : NULL;
   if ((resource_private_bytes && !resource_private) ||
       (total_private_bytes && !total_private)) {
      free(total_private);
      free(resource_private);
      free(runtime);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   D3DDDI_OPENALLOCATIONINFO2 allocation_info[1];
   memset(allocation_info, 0, sizeof(allocation_info));
   D3DKMT_OPENRESOURCEFROMNTHANDLE open;
   memset(&open, 0, sizeof(open));
   open.hDevice = helios->device;
   open.hNtHandle = import_info->handle;
   open.NumAllocations = 1;
   open.pOpenAllocationInfo2 = allocation_info;
   open.PrivateRuntimeDataSize = runtime_bytes;
   open.pPrivateRuntimeData = runtime;
   open.ResourcePrivateDriverDataSize = resource_private_bytes;
   open.pResourcePrivateDriverData = resource_private;
   open.TotalPrivateDriverDataBufferSize = total_private_bytes;
   open.pTotalPrivateDriverDataBuffer = total_private;
   st = D3DKMTOpenResourceFromNtHandle(&open);

   HeliosWddmAllocationDescV2 desc;
   memset(&desc, 0, sizeof(desc));
   struct helios_hwa2_reject_detail reject;
   memset(&reject, 0, sizeof(reject));
   const bool desc_valid =
      st == 0 && open.hResource && open.NumAllocations == 1 &&
      allocation_info[0].hAllocation &&
      helios_hwa2_from_private_data(allocation_info[0].pPrivateDriverData,
                                    allocation_info[0].PrivateDriverDataSize,
                                    total_private, total_private_bytes, &desc,
                                    &reject) &&
      helios_hwa2_validate_create_output(&desc, HELIOS_PACKAGE_GENERATION,
                                         &reject);
   const bool admitted =
      desc_valid &&
      (desc.allocation_kind == HELIOS_HWA2_KIND_BUFFER ||
       desc.allocation_kind == HELIOS_HWA2_KIND_IMAGE) &&
      (desc.flags &
       (HELIOS_HWA2_FLAG_SHARED | HELIOS_HWA2_FLAG_RESOURCE_ASSOCIATED)) ==
         (HELIOS_HWA2_FLAG_SHARED | HELIOS_HWA2_FLAG_RESOURCE_ASSOCIATED) &&
      desc.byte_size && !open.hKeyedMutex && !open.hSyncObject;

   /* Vulkan ignores allocationSize for D3D12_RESOURCE imports.  Keeping it in
    * the signature makes that deliberate and prevents a later caller from
    * quietly treating it as an HWA2 size check. */
   (void)allocation_size;

   struct helios_bo *bo = NULL;
   struct vn_renderer_helios_external_memory *external = NULL;
   if (admitted) {
      bo = calloc(1, sizeof(*bo));
      external = calloc(1, sizeof(*external));
   }
   if (!admitted || !bo || !external) {
      free(bo);
      free(external);
      helios_destroy_auxiliary_open_handles(open.hKeyedMutex,
                                            open.hSyncObject);
      if (open.hResource) {
         D3DKMT_DESTROYALLOCATION2 destroy;
         memset(&destroy, 0, sizeof(destroy));
         destroy.hDevice = helios->device;
         destroy.hResource = open.hResource;
         HELIOS_IGNORE_STATUS(D3DKMTDestroyAllocation2(&destroy));
      }
      free(total_private);
      free(resource_private);
      free(runtime);
      if (!desc_valid && st == 0)
         helios_hwa2_note_reject("a3_c57_open", &reject);
      return admitted ? VK_ERROR_OUT_OF_HOST_MEMORY
                      : VK_ERROR_INVALID_EXTERNAL_HANDLE;
   }

   bo->allocation = (struct helios_allocation){
      .resource = open.hResource,
      .allocation = allocation_info[0].hAllocation,
      .generation = desc.allocation_generation,
      .size = desc.byte_size,
      .role = 0,
      .imported = true,
   };
   bo->base.refcount = VN_REFCOUNT_INIT(1);
   bo->base.allocation_handle = bo->allocation.allocation;
   bo->base.allocation_generation = bo->allocation.generation;
   bo->base.allocation_role = 0;
   external->resource = open.hResource;
   external->allocation = allocation_info[0].hAllocation;

   free(total_private);
   free(resource_private);
   free(runtime);
   *out_allocation_size = desc.byte_size;
   if (out_desc)
      *out_desc = desc;
   *out_bo = &bo->base;
   *out_external = external;
   return VK_SUCCESS;
}

void
vn_renderer_helios_external_memory_destroy(
   struct vn_renderer *renderer,
   struct vn_renderer_helios_external_memory *external)
{
   if (!external)
      return;
   struct helios *helios = helios_from_renderer(renderer);
   EnterCriticalSection(&helios->allocation_lock);
   if (external->resource) {
      D3DKMT_DESTROYALLOCATION2 destroy;
      memset(&destroy, 0, sizeof(destroy));
      destroy.hDevice = helios->device;
      destroy.hResource = external->resource;
      HELIOS_IGNORE_STATUS(D3DKMTDestroyAllocation2(&destroy));
   }
   LeaveCriticalSection(&helios->allocation_lock);
   free(external);
}

VkResult
vn_renderer_helios_allocate_memory(struct vn_renderer *renderer,
                                   VkDevice device,
                                   const VkMemoryAllocateInfo *alloc_info,
                                   VkDeviceMemory *memory,
                                   struct vn_renderer_bo *base_bo)
{
   struct helios *helios = helios_from_renderer(renderer);
   struct helios_bo *bo = (struct helios_bo *)base_bo;
   if (!alloc_info || !memory || !bo || !bo->allocation.allocation ||
       !bo->allocation.generation || !bo->allocation.size)
      return VK_ERROR_INITIALIZATION_FAILED;

   const VkImportMemoryResourceInfoMESA import = {
      .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_RESOURCE_INFO_MESA,
      .pNext = alloc_info->pNext,
      .resourceId = 0,
   };
   const VkMemoryAllocateInfo local = {
      .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      .pNext = &import,
      .allocationSize = bo->allocation.size,
      .memoryTypeIndex = alloc_info->memoryTypeIndex,
   };
   const VkCommandStreamDescriptionMESA reply_stream = {
      .resourceId = 0,
      .offset = 0,
      .size = 24,
   };
   const size_t set_reply_size =
      vn_sizeof_vkSetReplyCommandStreamMESA(&reply_stream);
   const size_t allocate_size =
      vn_sizeof_vkAllocateMemory(device, &local, NULL, memory);
   if (set_reply_size != 36 ||
       allocate_size > HELIOS_HNR2_MAX_PAYLOAD_BYTES - set_reply_size)
      return VK_ERROR_INITIALIZATION_FAILED;
   const size_t payload_size = set_reply_size + allocate_size;
   uint8_t *payload = malloc(payload_size);
   if (!payload)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   struct vn_cs_encoder encoder =
      VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_size);
   vn_encode_vkSetReplyCommandStreamMESA(&encoder, 0, &reply_stream);
   vn_encode_vkAllocateMemory(&encoder, VK_COMMAND_GENERATE_REPLY_BIT_EXT,
                              device, &local, NULL, memory);
   if (vn_cs_encoder_get_fatal(&encoder) ||
       vn_cs_encoder_get_len(&encoder) != payload_size) {
      free(payload);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   /* Generated nesting is prefix-recursive.  With ImportResource outermost,
    * its uint32 operand follows exactly the encoded original pNext chain. */
   const size_t import_operand_offset =
      76 + vn_sizeof_VkMemoryAllocateInfo_pnext(alloc_info->pNext);
   if (import_operand_offset > UINT32_MAX) {
      free(payload);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   uint8_t raw_reply[24];
   uint64_t raw_reply_bytes = 0;
   int32_t reply_status = VK_ERROR_DEVICE_LOST;
   EnterCriticalSection(&helios->bootstrap_lock);
   VkResult result =
      helios->bootstrap
         ? helios_session_execute_allocate(
              helios->session, helios->bootstrap, payload, payload_size,
              (uint32_t)import_operand_offset, bo->allocation.allocation,
              bo->allocation.generation, raw_reply, sizeof(raw_reply),
              &raw_reply_bytes, &reply_status)
         : VK_ERROR_DEVICE_LOST;
   if (result != VK_SUCCESS && helios->bootstrap) {
      /* Render may have been accepted before its progress signal/wait failed.
       * Destroying this exact context is the only permitted cancel/drain; the
       * caller may revoke the target allocation only after it returns. */
      helios_native_context_destroy(helios->bootstrap);
      helios->bootstrap = NULL;
   }
   LeaveCriticalSection(&helios->bootstrap_lock);
   free(payload);
   if (result != VK_SUCCESS)
      return result;
   if (raw_reply_bytes != sizeof(raw_reply))
      return VK_ERROR_DEVICE_LOST;

   struct vn_cs_decoder decoder =
      VN_CS_DECODER_INITIALIZER(raw_reply, sizeof(raw_reply));
   VkDeviceMemory returned_memory = *memory;
   result = vn_decode_vkAllocateMemory_reply(&decoder, device, &local, NULL,
                                             &returned_memory);
   if ((int32_t)result != reply_status ||
       (result == VK_SUCCESS && returned_memory != *memory))
      return VK_ERROR_DEVICE_LOST;
   return result;
}

VkResult
vn_renderer_helios_control_generated(struct vn_renderer *renderer,
                                     void *payload,
                                     size_t payload_size,
                                     void *reply,
                                     size_t reply_size,
                                     int32_t *reply_status)
{
   struct helios *helios = helios_from_renderer(renderer);
   if (!helios || !helios->session || !payload || !payload_size || !reply ||
       !reply_size)
      return VK_ERROR_INITIALIZATION_FAILED;
   uint64_t reply_bytes = 0;
   return helios_session_control_generated(
      helios->session, payload, payload_size, reply, reply_size,
      &reply_bytes, reply_status);
}

VkResult
vn_renderer_helios_control_no_reply(struct vn_renderer *renderer,
                                    const void *payload,
                                    size_t payload_size)
{
   struct helios *helios = helios_from_renderer(renderer);
   if (!helios || !helios->session)
      return VK_ERROR_DEVICE_LOST;
   return helios_session_control_no_reply(helios->session, payload,
                                          payload_size);
}

VkResult
vn_renderer_helios_free_memory(struct vn_renderer *renderer,
                               VkDevice device,
                               VkDeviceMemory memory,
                               struct vn_renderer_bo *base_bo)
{
   struct helios *helios = helios_from_renderer(renderer);
   struct helios_bo *bo = (struct helios_bo *)base_bo;
   if (!bo || !bo->allocation.allocation || !bo->allocation.generation)
      return VK_ERROR_INITIALIZATION_FAILED;
   const size_t payload_size = vn_sizeof_vkFreeMemory(device, memory, NULL);
   uint8_t local_payload[64];
   uint8_t *payload = local_payload;
   if (payload_size > sizeof(local_payload)) {
      payload = malloc(payload_size);
      if (!payload)
         return VK_ERROR_OUT_OF_HOST_MEMORY;
   }
   struct vn_cs_encoder encoder =
      VN_CS_ENCODER_INITIALIZER_LOCAL(payload, payload_size);
   vn_encode_vkFreeMemory(&encoder, 0, device, memory, NULL);
   const struct helios_hnr2_allocation use = {
      .handle = bo->allocation.allocation,
      .access = HELIOS_HNR2_ACCESS_READ | HELIOS_HNR2_ACCESS_WRITE,
      .expected_generation = bo->allocation.generation,
   };
   const struct helios_hnr2_batch batch = {
      .payload = payload,
      .payload_bytes = payload_size,
      .allocations = &use,
      .allocation_count = 1,
   };
   uint64_t batch_token = 0;
   uint64_t progress = 0;
   EnterCriticalSection(&helios->bootstrap_lock);
   VkResult result = helios->bootstrap ? helios_native_context_submit(
                                            helios->bootstrap, &batch, true,
                                            &batch_token, &progress)
                                       : VK_ERROR_DEVICE_LOST;
   if (result == VK_SUCCESS)
      result = helios_native_context_wait(helios->bootstrap, progress,
                                          HELIOS_NATIVE_WAIT_INFINITE);
   if (result != VK_SUCCESS && helios->bootstrap) {
      /* See AllocateMemory above: context destruction precedes allocation
       * revocation even when the terminal host result cannot be observed. */
      helios_native_context_destroy(helios->bootstrap);
      helios->bootstrap = NULL;
   }
   LeaveCriticalSection(&helios->bootstrap_lock);
   if (payload != local_payload)
      free(payload);
   return result;
}

uint64_t
vn_renderer_helios_session_generation(const struct vn_renderer *renderer)
{
   return helios_session_generation(
      helios_from_renderer_const(renderer)->session);
}

uint32_t
vn_renderer_helios_endpoint_capacity(const struct vn_renderer *renderer)
{
   return helios_session_endpoint_capacity(
      helios_from_renderer_const(renderer)->session);
}

uint32_t
vn_renderer_helios_device_handle(const struct vn_renderer *renderer)
{
   const struct helios *helios = helios_from_renderer_const(renderer);
   return helios && helios->session
             ? helios_session_device_handle(helios->session)
             : 0;
}

VkResult
vn_renderer_helios_local_heap_size(const struct vn_renderer *renderer,
                                   uint64_t *out_size)
{
   const struct helios *helios = helios_from_renderer_const(renderer);
   return helios && helios->session
             ? helios_session_local_heap_size(helios->session, out_size)
             : VK_ERROR_INITIALIZATION_FAILED;
}

VkResult
vn_renderer_helios_build_queue_attach(
   const struct vn_renderer *renderer,
   const HeliosTranslationEndpointV1 *endpoint,
   const HeliosQueueAttachRequestV1 *request,
   HeliosQueueAttachV1 *out_hqa1)
{
   if (!renderer || !endpoint || !request || !out_hqa1)
      return VK_ERROR_INITIALIZATION_FAILED;
   const struct helios *helios = helios_from_renderer_const(renderer);
   if (!helios->session)
      return VK_ERROR_DEVICE_LOST;

   uint64_t capability_low = 0;
   uint64_t capability_high = 0;
   helios_session_capability(helios->session, &capability_low,
                             &capability_high);
   const uint64_t session_generation =
      helios_session_generation(helios->session);
   if ((!capability_low && !capability_high) || !session_generation)
      return VK_ERROR_DEVICE_LOST;

   *out_hqa1 = (HeliosQueueAttachV1){
      .magic = HELIOS_HQA1_MAGIC,
      .abi_version = HELIOS_HQA1_ABI_VERSION,
      .struct_size = HELIOS_HQA1_SIZE,
      .package_generation = HELIOS_PACKAGE_GENERATION,
      .session_generation = session_generation,
      .capability_low = capability_low,
      .capability_high = capability_high,
      .endpoint_id = endpoint->endpoint_id,
      .engine_class = endpoint->engine_class,
      .queue_family = endpoint->queue_family,
      .queue_index = endpoint->queue_index,
      .context_generation = request->context_generation,
      .flags = request->context_flags,
   };
   return VK_SUCCESS;
}

static VkResult
helios_refuse_generic_submit(struct vn_renderer *renderer,
                             const struct vn_renderer_submit *submit)
{
   (void)renderer;
   (void)submit;
   /* A7 owns generated generic-object classification.  Never fall back to a
    * ring, Escape, or allocation-free submit in this intermediate build. */
   return VK_ERROR_DEVICE_LOST;
}

static VkResult
helios_refuse_generic_wait(struct vn_renderer *renderer,
                           const struct vn_renderer_wait *wait)
{
   (void)renderer;
   (void)wait;
   return VK_ERROR_DEVICE_LOST;
}

static void
helios_renderer_info_init(struct helios *helios)
{
   struct vn_renderer_info *info = &helios->base.info;
   info->wire_format_version = 1;
   info->vk_xml_version = VK_MAKE_API_VERSION(0, 1, 4, 343);
   info->vk_ext_command_serialization_spec_version = 1;
   info->vk_mesa_venus_protocol_spec_version = 4;
   memset(info->vk_extension_mask, 0, sizeof(info->vk_extension_mask));
   /* `max_timeline_count` is an exclusive ring-index limit and ring zero is
    * control-only.  K11 publishes 64 nonzero endpoints, hence 65 indices;
    * the dedicated allocation bootstrap consumes endpoint 1 and real queues
    * consume the remaining 2..64 in exact creation order. */
   info->max_timeline_count =
      helios_session_endpoint_capacity(helios->session) + 1;
   info->has_dma_buf_import = false;
   info->has_external_sync = true;
   info->has_implicit_fencing = false;
   info->has_guest_vram = false;
   info->pci.vendor_id = 0x1af4;
   info->pci.device_id = 0x1050;
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
   info->id.has_luid = true;
   info->id.node_mask = 1;
   memcpy(info->id.luid, &helios->adapter_luid, sizeof(info->id.luid));
}

static void
helios_destroy(struct vn_renderer *renderer,
               const VkAllocationCallbacks *alloc)
{
   struct helios *helios = helios_from_renderer(renderer);
   if (helios->bootstrap_lock_live) {
      EnterCriticalSection(&helios->bootstrap_lock);
      if (helios->bootstrap) {
         helios_native_context_destroy(helios->bootstrap);
         helios->bootstrap = NULL;
      }
      LeaveCriticalSection(&helios->bootstrap_lock);
   }
   if (helios->paging_queue) {
      D3DDDI_DESTROYPAGINGQUEUE destroy = {
         .hPagingQueue = helios->paging_queue,
      };
      HELIOS_IGNORE_STATUS(D3DKMTDestroyPagingQueue(&destroy));
      helios->paging_queue = 0;
   }
   if (helios->session) {
      helios_translation_session_destroy(helios->session);
      helios->session = NULL;
   }
   if (helios->bootstrap_lock_live) {
      DeleteCriticalSection(&helios->bootstrap_lock);
      helios->bootstrap_lock_live = false;
   }
   if (helios->allocation_lock_live) {
      DeleteCriticalSection(&helios->allocation_lock);
      helios->allocation_lock_live = false;
   }
   vk_free(alloc, helios);
}

VkResult
vn_renderer_create_helios(struct vn_instance *instance,
                          const VkAllocationCallbacks *alloc,
                          struct vn_renderer **out_renderer)
{
   *out_renderer = NULL;
   struct helios *helios = vk_zalloc(alloc, sizeof(*helios), VN_DEFAULT_ALIGN,
                                     VK_SYSTEM_ALLOCATION_SCOPE_INSTANCE);
   if (!helios)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   helios->instance = instance;
   InitializeCriticalSection(&helios->allocation_lock);
   helios->allocation_lock_live = true;
   InitializeCriticalSection(&helios->bootstrap_lock);
   helios->bootstrap_lock_live = true;

   const bool direct = instance->helios_direct_requested;
   const uint32_t endpoint_capacity =
      direct ? instance->helios_direct_endpoint_capacity
             : HELIOS_SESSION_ENDPOINTS;
   if (direct)
      instance->helios_direct_create_status =
         HELIOS_TRANSLATOR_STATUS_ADAPTER_UNAVAILABLE;
   VkResult result =
      direct ? helios_select_direct_adapter(
                  helios, instance->helios_direct_adapter_luid_low,
                  instance->helios_direct_adapter_luid_high)
             : helios_find_adapter(helios);
   if (result == VK_SUCCESS && direct)
      instance->helios_direct_create_status =
         HELIOS_TRANSLATOR_STATUS_SESSION_INIT;
   if (result == VK_SUCCESS)
      result = helios_translation_session_create(
         helios->adapter_luid, endpoint_capacity, &helios->session);
   if (result == VK_SUCCESS) {
      helios->device = helios_session_device_handle(helios->session);
      result = helios_native_context_create(helios->device,
                                            HELIOS_NATIVE_CONTEXT_QUEUE, 0, 0,
                                            &helios->bootstrap);
   }
   if (result == VK_SUCCESS) {
      D3DKMT_CREATEPAGINGQUEUE create;
      memset(&create, 0, sizeof(create));
      create.hDevice = helios->device;
      create.Priority = D3DDDI_PAGINGQUEUE_PRIORITY_NORMAL;
      NTSTATUS st = D3DKMTCreatePagingQueue(&create);
      if (st != 0 || !create.hPagingQueue || !create.hSyncObject) {
         result = VK_ERROR_INITIALIZATION_FAILED;
      } else {
         helios->paging_queue = create.hPagingQueue;
         helios->paging_fence = create.hSyncObject;
      }
   }
   if (result != VK_SUCCESS) {
      if (direct && result == VK_ERROR_OUT_OF_HOST_MEMORY)
         instance->helios_direct_create_status =
            HELIOS_TRANSLATOR_STATUS_SESSION_CAPACITY;
      helios_destroy(&helios->base, alloc);
      return result;
   }

   if (direct)
      instance->helios_direct_create_status = HELIOS_TRANSLATOR_STATUS_OK;

   helios_renderer_info_init(helios);
   helios->base.ops.destroy = helios_destroy;
   helios->base.ops.submit = helios_refuse_generic_submit;
   helios->base.ops.wait = helios_refuse_generic_wait;
   helios->base.shmem_ops.create = helios_shmem_create;
   helios->base.shmem_ops.destroy = helios_shmem_destroy;
   helios->base.bo_ops.create_from_device_memory =
      helios_bo_create_from_device_memory;
   helios->base.bo_ops.create_from_dma_buf = NULL;
   helios->base.bo_ops.destroy = helios_bo_destroy;
   helios->base.bo_ops.release_resource = NULL;
   helios->base.bo_ops.export_dma_buf = NULL;
   helios->base.bo_ops.export_sync_file = NULL;
   helios->base.bo_ops.map = helios_bo_map;
   helios->base.bo_ops.flush = helios_bo_cache_op;
   helios->base.bo_ops.invalidate = helios_bo_cache_op;
   helios->base.sync_ops.create = helios_sync_create;
   helios->base.sync_ops.create_from_syncobj = NULL;
   helios->base.sync_ops.destroy = helios_sync_destroy;
   helios->base.sync_ops.export_syncobj = NULL;
   helios->base.sync_ops.reset = helios_sync_reset;
   helios->base.sync_ops.read = helios_sync_read;
   helios->base.sync_ops.write = helios_sync_write;

   *out_renderer = &helios->base;
   return VK_SUCCESS;
}

/* venus never compiles SPIR-V in the guest, but vk_util's object needs these
 * link symbols in the lean Windows ICD. */
struct nir_spirv_specialization;
struct nir_spirv_specialization *
vtn_alloc_specialization(uint32_t count);
bool
vtn_add_specialization_entry(struct nir_spirv_specialization *spec,
                             uint32_t slot,
                             uint32_t entry_id,
                             uint32_t entry_size,
                             const void *entry_data,
                             bool defined_on_module);
void
vtn_free_specialization(struct nir_spirv_specialization *spec);

struct nir_spirv_specialization *
vtn_alloc_specialization(uint32_t count)
{
   (void)count;
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

#endif /* _WIN32 */
