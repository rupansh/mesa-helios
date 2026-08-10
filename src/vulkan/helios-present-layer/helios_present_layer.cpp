/*
 * Copyright © 2026 Helios vGPU
 * SPDX-License-Identifier: MIT
 *
 * VK_LAYER_HELIOS_present — the Helios native-Vulkan WSI layer.
 *
 * See helios_present_layer.h for the normative-source map. Every "§" reference
 * below is a line range in docs/HELIOS_PRESENT_SYNC_RETIREMENT.md.
 *
 * Structure of this file:
 *   1. counters, logging, small helpers
 *   2. next-chain dispatch tables
 *   3. layer objects (instance, physical-device admission, device, surface,
 *      swapchain, slot) and their registries
 *   4. the entry-point manifest and its byte-for-byte self-check
 *   5. instance creation/destruction and extension enumeration
 *   6. physical-device admission and the intercepted physical-device queries
 *   7. surface creation/destruction and the copy-only profile queries
 *   8. device creation/destruction, the private helper queue
 *   9. swapchain creation: D3D12/DXGI objects, C57 canonical import, fences
 *  10. Acquire
 *  11. Present
 *  12. swapchain destruction / teardown
 *  13. the C45 alias-image surface (deliberately refused this generation)
 *  14. vkGetInstanceProcAddr / vkGetDeviceProcAddr / negotiate
 */

#include "helios_present_layer.h"

#define D3D12_IGNORE_SDK_LAYERS
#include <dxgi1_6.h>
#include <directx/d3d12.h>
#include <dxguids/dxguids.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/* ==================================================================== */
/* 1. Counters, logging, helpers                                        */
/* ==================================================================== */

static const char *const helios_counter_names[HELIOS_CNT_COUNT] = {
#define HELIOS_COUNTER_NAME(name) #name,
   HELIOS_LAYER_COUNTERS(HELIOS_COUNTER_NAME)
#undef HELIOS_COUNTER_NAME
};

static std::atomic<uint64_t> helios_counters[HELIOS_CNT_COUNT];

/* 0 = unknown, 1 = off, 2 = on, 3 = trace */
static std::atomic<int> helios_debug_state{0};

static int
helios_debug_level(void)
{
   int s = helios_debug_state.load(std::memory_order_relaxed);
   if (s != 0)
      return s - 1;

   char buf[32];
   DWORD n = GetEnvironmentVariableA("HELIOS_LAYER_DEBUG", buf, sizeof(buf));
   int level = 0;
   if (n > 0 && n < sizeof(buf)) {
      if (_stricmp(buf, "trace") == 0)
         level = 2;
      else if (buf[0] != '0')
         level = 1;
   }
   helios_debug_state.store(level + 1, std::memory_order_relaxed);
   return level;
}

static void
helios_log(const char *fmt, ...)
{
   if (helios_debug_level() == 0)
      return;

   char msg[1024];
   va_list ap;
   va_start(ap, fmt);
   int n = vsnprintf(msg, sizeof(msg) - 2, fmt, ap);
   va_end(ap);
   if (n < 0)
      return;
   if ((size_t)n > sizeof(msg) - 2)
      n = (int)sizeof(msg) - 2;
   msg[n] = '\n';
   msg[n + 1] = '\0';
   OutputDebugStringA(msg);
   fputs(msg, stderr);
}

/*
 * Every refusal in this layer goes through helios_refuse(). It bumps the named
 * counter, traces when asked, and returns the caller's chosen VkResult so the
 * refusal is one expression at the call site. Loud failure over fake success.
 */
static VkResult
helios_refuse(helios_layer_counter_id id, VkResult result, const char *detail)
{
   helios_counters[id].fetch_add(1, std::memory_order_relaxed);
   if (helios_debug_level() >= 2) {
      helios_log("[helios-wsi] REFUSE %s (%d)%s%s", helios_counter_names[id],
                 (int)result, detail ? ": " : "", detail ? detail : "");
   }
   return result;
}

static void
helios_count(helios_layer_counter_id id)
{
   helios_counters[id].fetch_add(1, std::memory_order_relaxed);
}

static void
helios_dump_counters(const char *why)
{
   if (helios_debug_level() == 0)
      return;
   helios_log("[helios-wsi] counters (%s):", why);
   for (int i = 0; i < HELIOS_CNT_COUNT; i++) {
      uint64_t v = helios_counters[i].load(std::memory_order_relaxed);
      if (v)
         helios_log("[helios-wsi]   %-46s %llu", helios_counter_names[i],
                    (unsigned long long)v);
   }
}

static inline bool
streq(const char *a, const char *b)
{
   return a && b && strcmp(a, b) == 0;
}

/* Two-call enumeration contract, used by every array-returning query. */
template <typename T>
static VkResult
helios_write_array(const T *src, uint32_t src_count, T *dst, uint32_t *dst_count)
{
   if (dst == nullptr) {
      *dst_count = src_count;
      return VK_SUCCESS;
   }
   uint32_t n = *dst_count < src_count ? *dst_count : src_count;
   for (uint32_t i = 0; i < n; i++)
      dst[i] = src[i];
   *dst_count = n;
   return n < src_count ? VK_INCOMPLETE : VK_SUCCESS;
}

static const void *
helios_find_pnext(const void *chain, VkStructureType type)
{
   for (const VkBaseInStructure *s = (const VkBaseInStructure *)chain; s;
        s = s->pNext) {
      if (s->sType == type)
         return s;
   }
   return nullptr;
}

template <typename T>
static void
helios_com_release(T *&p)
{
   if (p) {
      p->Release();
      p = nullptr;
   }
}

/* ==================================================================== */
/* 2. Next-chain dispatch tables                                        */
/* ==================================================================== */

struct HeliosInstanceDispatch {
   PFN_vkGetInstanceProcAddr GetInstanceProcAddr;
   PFN_vkDestroyInstance DestroyInstance;
   PFN_vkCreateDevice CreateDevice;
   PFN_vkEnumerateDeviceExtensionProperties EnumerateDeviceExtensionProperties;
   PFN_vkEnumeratePhysicalDevices EnumeratePhysicalDevices;
   PFN_vkEnumeratePhysicalDeviceGroups EnumeratePhysicalDeviceGroups;
   PFN_vkGetPhysicalDeviceProperties GetPhysicalDeviceProperties;
   PFN_vkGetPhysicalDeviceProperties2 GetPhysicalDeviceProperties2;
   PFN_vkGetPhysicalDeviceQueueFamilyProperties GetPhysicalDeviceQueueFamilyProperties;
   PFN_vkGetPhysicalDeviceQueueFamilyProperties2 GetPhysicalDeviceQueueFamilyProperties2;
   PFN_vkGetPhysicalDeviceImageFormatProperties2 GetPhysicalDeviceImageFormatProperties2;
   PFN_vkGetPhysicalDeviceExternalSemaphoreProperties GetPhysicalDeviceExternalSemaphoreProperties;
};

struct HeliosDeviceDispatch {
   PFN_vkGetDeviceProcAddr GetDeviceProcAddr;
   PFN_vkDestroyDevice DestroyDevice;
   PFN_vkDeviceWaitIdle DeviceWaitIdle;
   PFN_vkGetDeviceQueue GetDeviceQueue;
   PFN_vkGetDeviceQueue2 GetDeviceQueue2;
   PFN_vkQueueSubmit2 QueueSubmit2;
   PFN_vkQueueWaitIdle QueueWaitIdle;
   PFN_vkCreateImage CreateImage;
   PFN_vkDestroyImage DestroyImage;
   PFN_vkBindImageMemory2 BindImageMemory2;
   PFN_vkGetImageMemoryRequirements2 GetImageMemoryRequirements2;
   PFN_vkAllocateMemory AllocateMemory;
   PFN_vkFreeMemory FreeMemory;
   PFN_vkGetMemoryWin32HandlePropertiesKHR GetMemoryWin32HandlePropertiesKHR;
   PFN_vkCreateSemaphore CreateSemaphore;
   PFN_vkDestroySemaphore DestroySemaphore;
   PFN_vkImportSemaphoreWin32HandleKHR ImportSemaphoreWin32HandleKHR;
   PFN_vkCreateFence CreateFence;
   PFN_vkDestroyFence DestroyFence;
   PFN_vkWaitForFences WaitForFences;
   PFN_vkResetFences ResetFences;
   PFN_vkCreateCommandPool CreateCommandPool;
   PFN_vkDestroyCommandPool DestroyCommandPool;
   PFN_vkResetCommandPool ResetCommandPool;
   PFN_vkAllocateCommandBuffers AllocateCommandBuffers;
   PFN_vkBeginCommandBuffer BeginCommandBuffer;
   PFN_vkEndCommandBuffer EndCommandBuffer;
   PFN_vkCmdPipelineBarrier2 CmdPipelineBarrier2;
   /* The private lower-ICD presentable-image tag (ambiguity A4). */
   PFN_vkSetHeliosPresentableImageHELIOS SetHeliosPresentableImage;
};

/* ==================================================================== */
/* 3. Layer objects                                                     */
/* ==================================================================== */

struct HeliosPhysDevInfo {
   bool computed = false;
   bool admitted = false;
   const char *reject_reason = "not evaluated";
   uint32_t api_version = 0;
   uint32_t canonical_family = UINT32_MAX;
   uint32_t canonical_queue_count = 0;
   LUID luid = {};
   uint8_t device_uuid[VK_UUID_SIZE] = {};
   uint8_t driver_uuid[VK_UUID_SIZE] = {};
};

struct HeliosInstance {
   VkInstance instance = VK_NULL_HANDLE;
   HeliosInstanceDispatch disp = {};
   PFN_GetPhysicalDeviceProcAddr next_gpdpa = nullptr;
   uint32_t app_api_version = VK_API_VERSION_1_0;
   bool surface_enabled = false;      /* VK_KHR_surface */
   bool win32_surface_enabled = false;/* VK_KHR_win32_surface */
   bool surface_caps2_enabled = false;/* VK_KHR_get_surface_capabilities2 */
   /* Observed, not consumed: these two are the app's, and they travel down to
    * the ICD untouched. They exist here only so the KHR aliases of the
    * core-1.1 entry points can be gated on them. */
   bool device_group_creation_enabled = false; /* VK_KHR_device_group_creation */
   bool physdev_props2_enabled = false;        /* VK_KHR_get_physical_device_properties2 */
   IDXGIFactory4 *dxgi_factory = nullptr;
   std::mutex lock;
   std::unordered_map<VkPhysicalDevice, HeliosPhysDevInfo> phys;

   bool wsi_enabled() const { return surface_enabled && win32_surface_enabled; }
};

struct HeliosSurface {
   static const uint64_t kMagic = 0x48454c53554246ull; /* "HELSURF" */
   uint64_t magic = kMagic;
   HeliosInstance *inst = nullptr;
   HWND hwnd = nullptr;
   HINSTANCE hinstance = nullptr;
};

/* §10.7:2516-2518 — the nine slot states. */
enum HeliosSlotState {
   HELIOS_SLOT_NEVER_USED = 0,
   HELIOS_SLOT_AVAILABLE,
   HELIOS_SLOT_ACQUIRED,
   HELIOS_SLOT_PRESENT_QUEUED,
   HELIOS_SLOT_D3D_COPY,
   HELIOS_SLOT_DXGI_OWNED,
   HELIOS_SLOT_RELEASE_QUEUED,
   HELIOS_SLOT_ALIAS_ONLY,
   HELIOS_SLOT_LOST,
};

struct HeliosSlot {
   /* D3D12 side */
   ID3D12Resource *s = nullptr;             /* S[i] */
   HANDLE h = nullptr;                      /* H[i], retained for slot lifetime */
   ID3D12Fence *ready = nullptr;            /* Ready[i]  — Vulkan signals */
   ID3D12Fence *release = nullptr;          /* Release[i] — D3D signals */
   ID3D12CommandAllocator *alloc = nullptr;
   ID3D12GraphicsCommandList *list = nullptr;
   HANDLE release_event = nullptr;          /* SetEventOnCompletion target */

   /* Vulkan side */
   VkImage image = VK_NULL_HANDLE;
   VkDeviceMemory memory = VK_NULL_HANDLE;
   VkSemaphore ready_sem = VK_NULL_HANDLE;
   VkSemaphore release_sem = VK_NULL_HANDLE;
   VkCommandPool pool = VK_NULL_HANDLE;
   VkCommandBuffer acquire_cb = VK_NULL_HANDLE;
   VkCommandBuffer release_cb = VK_NULL_HANDLE;

   /* State */
   HeliosSlotState state = HELIOS_SLOT_NEVER_USED;
   uint64_t epoch = 0;        /* current/target epoch e[i] */
   uint64_t retired_epoch = 0;/* last epoch whose Release completed */
   uint32_t device_mask = 1;  /* mask recorded by the last Acquire (C45) */
   bool externally_owned = false; /* last transfer was to QUEUE_FAMILY_EXTERNAL */
   std::mutex lock;           /* covers this slot's recording objects (A13) */
};

struct HeliosDevice;

struct HeliosSwapchain {
   static const uint64_t kMagic = 0x48454c535743ull; /* "HELSWC" */
   uint64_t magic = kMagic;
   uint64_t id = 0;               /* process-unique generation id */
   HeliosDevice *dev = nullptr;
   HeliosSurface *surf = nullptr;
   HWND hwnd = nullptr;

   VkExtent2D extent = {};
   uint32_t image_count = 0;
   VkImageUsageFlags usage = 0;
   VkSharingMode sharing_mode = VK_SHARING_MODE_EXCLUSIVE;
   std::vector<uint32_t> queue_families;

   IDXGISwapChain3 *dxgi = nullptr;
   ID3D12CommandQueue *queue = nullptr;      /* this swapchain's own D3D queue */
   std::vector<ID3D12Resource *> backbuffers;/* B[0..N) — refs held */

   std::vector<HeliosSlot> slots;

   bool retired = false;  /* replaced through oldSwapchain */
   bool lost = false;     /* device removed / surface lost */
   bool destroying = false;
   HANDLE state_event = nullptr; /* wakes a blocked Acquire on loss/destroy */
   uint32_t next_hint = 0;
   std::mutex lock;
};

struct HeliosAppQueue {
   uint32_t family = UINT32_MAX;
   uint32_t index = UINT32_MAX;
   VkDeviceQueueCreateFlags flags = 0;
};

struct HeliosDevice {
   VkDevice device = VK_NULL_HANDLE;
   VkPhysicalDevice phys = VK_NULL_HANDLE;
   HeliosInstance *inst = nullptr;
   HeliosDeviceDispatch disp = {};
   PFN_vkSetDeviceLoaderData set_device_loader_data = nullptr;

   bool wsi_enabled = false;
   /* Observed, not consumed — see HeliosInstance's pair. Gates the KHR alias
    * of the bind-memory2 entry points, which were promoted to core 1.1. */
   bool bind_memory2_khr_enabled = false; /* VK_KHR_bind_memory2 */
   uint32_t canonical_family = UINT32_MAX;
   uint32_t private_queue_index = UINT32_MAX;
   VkQueue helper_queue = VK_NULL_HANDLE;
   std::mutex helper_lock;   /* §10.7:2602-2604: covers only layer calls to it */

   /* D3D12 objects, created lazily at the first swapchain and released with
    * the device so nothing outlives the VkDevice (the 54TH-session lesson: a
    * never-freed global is a per-device leak). */
   ID3D12Device *d3d12 = nullptr;
   std::mutex d3d12_lock;

   std::mutex lock;
   std::unordered_map<VkQueue, HeliosAppQueue> app_queues;
   std::unordered_set<HeliosSwapchain *> swapchains;
};

/* ---- registries ---------------------------------------------------- */
/*
 * Dispatchable Vulkan handles begin with a loader dispatch-table pointer; the
 * conventional layer key is that pointer. Non-dispatchable handles the layer
 * itself mints (surfaces, swapchains) are pointers cast to uint64_t and are
 * additionally validated against these registries, so a foreign or stale
 * handle is refused rather than dereferenced (§10.9:2857).
 */
static std::mutex helios_registry_lock;
static std::unordered_map<void *, HeliosInstance *> helios_instances;
static std::unordered_map<void *, HeliosDevice *> helios_devices;
static std::unordered_set<HeliosSurface *> helios_surfaces;
static std::unordered_set<HeliosSwapchain *> helios_swapchains;
static std::atomic<uint64_t> helios_next_swapchain_id{1};

static inline void *
helios_key(const void *dispatchable)
{
   return *(void **)dispatchable;
}

static HeliosInstance *
helios_instance_of(VkInstance instance)
{
   if (!instance)
      return nullptr;
   std::lock_guard<std::mutex> g(helios_registry_lock);
   auto it = helios_instances.find(helios_key(instance));
   return it == helios_instances.end() ? nullptr : it->second;
}

static HeliosInstance *
helios_instance_of_phys(VkPhysicalDevice phys)
{
   if (!phys)
      return nullptr;
   std::lock_guard<std::mutex> g(helios_registry_lock);
   auto it = helios_instances.find(helios_key(phys));
   return it == helios_instances.end() ? nullptr : it->second;
}

static HeliosDevice *
helios_device_of(VkDevice device)
{
   if (!device)
      return nullptr;
   std::lock_guard<std::mutex> g(helios_registry_lock);
   auto it = helios_devices.find(helios_key(device));
   return it == helios_devices.end() ? nullptr : it->second;
}

static HeliosDevice *
helios_device_of_queue(VkQueue queue)
{
   if (!queue)
      return nullptr;
   std::lock_guard<std::mutex> g(helios_registry_lock);
   auto it = helios_devices.find(helios_key(queue));
   return it == helios_devices.end() ? nullptr : it->second;
}

static HeliosSurface *
helios_surface_of(VkSurfaceKHR surface)
{
   if (surface == VK_NULL_HANDLE)
      return nullptr;
   HeliosSurface *s = (HeliosSurface *)(uintptr_t)surface;
   std::lock_guard<std::mutex> g(helios_registry_lock);
   if (helios_surfaces.find(s) == helios_surfaces.end())
      return nullptr;
   return s->magic == HeliosSurface::kMagic ? s : nullptr;
}

static HeliosSwapchain *
helios_swapchain_of(VkSwapchainKHR swapchain)
{
   if (swapchain == VK_NULL_HANDLE)
      return nullptr;
   HeliosSwapchain *s = (HeliosSwapchain *)(uintptr_t)swapchain;
   std::lock_guard<std::mutex> g(helios_registry_lock);
   if (helios_swapchains.find(s) == helios_swapchains.end())
      return nullptr;
   return s->magic == HeliosSwapchain::kMagic ? s : nullptr;
}

static inline VkSurfaceKHR
helios_surface_handle(HeliosSurface *s)
{
   return (VkSurfaceKHR)(uintptr_t)s;
}

static inline VkSwapchainKHR
helios_swapchain_handle(HeliosSwapchain *s)
{
   return (VkSwapchainKHR)(uintptr_t)s;
}

/* ---- window helpers ------------------------------------------------ */

static bool
helios_window_alive(HWND hwnd)
{
   return hwnd != nullptr && IsWindow(hwnd);
}

/* Returns false when the HWND is gone. A live but minimized/zero-sized window
 * yields (0,0), which is the exact value the surface queries must report
 * (§10.7:2465-2466). */
static bool
helios_client_extent(HWND hwnd, VkExtent2D *out)
{
   RECT r;
   if (!helios_window_alive(hwnd) || !GetClientRect(hwnd, &r))
      return false;
   LONG w = r.right - r.left;
   LONG h = r.bottom - r.top;
   out->width = w > 0 ? (uint32_t)w : 0u;
   out->height = h > 0 ? (uint32_t)h : 0u;
   return true;
}

/* ==================================================================== */
/* 4. The entry-point manifest                                          */
/* ==================================================================== */
/*
 * §10.7:2250-2267 fixes exactly which names vkGetInstanceProcAddr and
 * vkGetDeviceProcAddr return the layer chain for, and requires "a generated
 * entry-point manifest ... compared byte-for-byte with these lists".
 *
 * The comparison is implemented literally: kInstanceChainNames/kDeviceChainNames
 * are transcribed from the prose, the dispatch tables below are what the
 * proc-address functions actually serve, and helios_verify_entry_manifest()
 * compares the two at vkNegotiateLoaderLayerInterfaceVersion. A mismatch
 * refuses to negotiate — the layer never loads with a surface it cannot prove.
 */

struct HeliosEntry {
   const char *name;
   PFN_vkVoidFunction fn;
   /* See the HELIOS_GATE_* enum below. */
   int gate;
   /* True when the entry's first parameter is a VkPhysicalDevice.
    *
    * §10.7's dispatch closure distinguishes them because the loader resolves
    * physical-device functions through GetPhysicalDeviceProcAddr as well as
    * GetInstanceProcAddr, and a layer that answers one and not the other drops
    * out of the chain for exactly those entry points. */
   bool phys;
};

enum {
   HELIOS_GATE_ALWAYS = 0,
   HELIOS_GATE_SURFACE = 1,          /* VK_KHR_surface */
   HELIOS_GATE_WIN32_SURFACE = 2,    /* VK_KHR_win32_surface */
   HELIOS_GATE_CAPS2 = 3,            /* VK_KHR_get_surface_capabilities2 */
   HELIOS_GATE_CORE_1_1 = 4,         /* app apiVersion >= 1.1 */
   HELIOS_GATE_SWAPCHAIN = 5,        /* VK_KHR_swapchain, on the device */
   /* The three below gate KHR ALIASES of entry points that were promoted to
    * core 1.1. The alias is resolvable only while its extension is enabled —
    * an app on 1.1 gets the un-suffixed name through HELIOS_GATE_CORE_1_1 and
    * the suffixed one only if it asked for the extension — so these are
    * deliberately NOT folded into HELIOS_GATE_CORE_1_1.
    *
    * ⚠ Unlike the gates above, these name extensions this layer does not own.
    * They are OBSERVED during CreateInstance/CreateDevice and passed through to
    * the lower ICD, never consumed. */
   HELIOS_GATE_DEVICE_GROUP_CREATION_KHR = 6, /* VK_KHR_device_group_creation */
   HELIOS_GATE_PHYSDEV_PROPS2_KHR = 7,        /* VK_KHR_get_physical_device_properties2 */
   HELIOS_GATE_BIND_MEMORY2_KHR = 8,          /* VK_KHR_bind_memory2, on the device */
};

/* The two lists, transcribed from §10.7:2250-2262. Order is the prose order. */
static const char *const kInstanceChainNames[] = {
   "vkGetInstanceProcAddr",
   "vkCreateInstance",
   "vkDestroyInstance",
   "vkEnumerateInstanceExtensionProperties",
   "vkEnumerateDeviceExtensionProperties",
   "vkCreateWin32SurfaceKHR",
   "vkDestroySurfaceKHR",
   "vkGetPhysicalDeviceSurfaceSupportKHR",
   "vkGetPhysicalDeviceSurfaceCapabilitiesKHR",
   "vkGetPhysicalDeviceSurfaceFormatsKHR",
   "vkGetPhysicalDeviceSurfacePresentModesKHR",
   "vkGetPhysicalDeviceWin32PresentationSupportKHR",
   "vkGetPhysicalDeviceSurfaceCapabilities2KHR",
   "vkGetPhysicalDeviceSurfaceFormats2KHR",
   "vkEnumeratePhysicalDeviceGroups",
   "vkEnumeratePhysicalDeviceGroupsKHR",
   "vkGetPhysicalDevicePresentRectanglesKHR",
   "vkGetPhysicalDeviceQueueFamilyProperties",
   "vkGetPhysicalDeviceQueueFamilyProperties2",
   "vkGetPhysicalDeviceQueueFamilyProperties2KHR",
   "vkCreateDevice",
};

static const char *const kDeviceChainNames[] = {
   "vkGetDeviceProcAddr",
   "vkDestroyDevice",
   "vkCreateSwapchainKHR",
   "vkDestroySwapchainKHR",
   "vkGetSwapchainImagesKHR",
   "vkAcquireNextImageKHR",
   "vkAcquireNextImage2KHR",
   "vkQueuePresentKHR",
   "vkGetDeviceGroupPresentCapabilitiesKHR",
   "vkGetDeviceGroupSurfacePresentModesKHR",
   "vkGetDeviceQueue",
   "vkGetDeviceQueue2",
   "vkCreateImage",
   "vkDestroyImage",
   "vkBindImageMemory2",
   "vkBindImageMemory2KHR",
};

/* Filled in at the bottom of the file, where every handler is defined. */
static const HeliosEntry *helios_instance_entries(uint32_t *count);
static const HeliosEntry *helios_device_entries(uint32_t *count);

static bool
helios_verify_entry_manifest(void)
{
   uint32_t n_inst = 0, n_dev = 0;
   const HeliosEntry *inst = helios_instance_entries(&n_inst);
   const HeliosEntry *dev = helios_device_entries(&n_dev);

   const uint32_t want_inst =
      (uint32_t)(sizeof(kInstanceChainNames) / sizeof(kInstanceChainNames[0]));
   const uint32_t want_dev =
      (uint32_t)(sizeof(kDeviceChainNames) / sizeof(kDeviceChainNames[0]));

   if (n_inst != want_inst || n_dev != want_dev) {
      helios_refuse(HELIOS_CNT_entry_manifest_mismatch,
                    VK_ERROR_INITIALIZATION_FAILED, "table size");
      return false;
   }
   for (uint32_t i = 0; i < want_inst; i++) {
      if (!streq(inst[i].name, kInstanceChainNames[i]) || inst[i].fn == nullptr) {
         helios_refuse(HELIOS_CNT_entry_manifest_mismatch,
                       VK_ERROR_INITIALIZATION_FAILED, kInstanceChainNames[i]);
         return false;
      }
   }
   for (uint32_t i = 0; i < want_dev; i++) {
      if (!streq(dev[i].name, kDeviceChainNames[i]) || dev[i].fn == nullptr) {
         helios_refuse(HELIOS_CNT_entry_manifest_mismatch,
                       VK_ERROR_INITIALIZATION_FAILED, kDeviceChainNames[i]);
         return false;
      }
   }
   return true;
}

/* ==================================================================== */
/* 5. Advertised extensions and instance creation                       */
/* ==================================================================== */

/* §10.7:2237-2240 — exactly these three at instance scope, and nothing else.
 * Every name in §10.7:2224-2233's withheld list is absent by construction. */
static const VkExtensionProperties kInstanceExtensions[] = {
   { VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_SURFACE_SPEC_VERSION },
   { VK_KHR_WIN32_SURFACE_EXTENSION_NAME, VK_KHR_WIN32_SURFACE_SPEC_VERSION },
   { VK_KHR_GET_SURFACE_CAPABILITIES_2_EXTENSION_NAME,
     VK_KHR_GET_SURFACE_CAPABILITIES_2_SPEC_VERSION },
};

/* §10.7:2240-2241 — VK_KHR_swapchain only, and only for an admitted device. */
static const VkExtensionProperties kDeviceExtensions[] = {
   { VK_KHR_SWAPCHAIN_EXTENSION_NAME, VK_KHR_SWAPCHAIN_SPEC_VERSION },
};

/*
 * Names the layer must strip from the lower create arrays and from any merged
 * enumeration result: the lower Helios ICD deliberately exposes no WSI on
 * Windows (§10.7:2209-2210, §10.7:2246-2248), so a stale lower name would let
 * an application reach a swapchain the layer does not own.
 *
 * (There was a `helios_is_layer_owned_instance_extension` here, testing the
 * three names in kInstanceExtensions. It was dead — those three are a strict
 * SUBSET of the kStale list below, which is what actually does the stripping,
 * and the CreateInstance loop cannot use it anyway because it has to know
 * WHICH of the three matched in order to set the right want_* flag. Found by
 * the first real build, -Wunused-function.)
 */
static bool
helios_is_stale_lower_wsi_name(const char *name)
{
   static const char *const kStale[] = {
      VK_KHR_SURFACE_EXTENSION_NAME,
      VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
      VK_KHR_GET_SURFACE_CAPABILITIES_2_EXTENSION_NAME,
      VK_KHR_SWAPCHAIN_EXTENSION_NAME,
      VK_KHR_SWAPCHAIN_MUTABLE_FORMAT_EXTENSION_NAME,
      VK_KHR_DISPLAY_SWAPCHAIN_EXTENSION_NAME,
      VK_KHR_SHARED_PRESENTABLE_IMAGE_EXTENSION_NAME,
      VK_KHR_INCREMENTAL_PRESENT_EXTENSION_NAME,
      VK_KHR_PRESENT_ID_EXTENSION_NAME,
      VK_KHR_PRESENT_WAIT_EXTENSION_NAME,
      VK_EXT_HDR_METADATA_EXTENSION_NAME,
      VK_EXT_SWAPCHAIN_COLOR_SPACE_EXTENSION_NAME,
      VK_EXT_FULL_SCREEN_EXCLUSIVE_EXTENSION_NAME,
      VK_EXT_SURFACE_MAINTENANCE_1_EXTENSION_NAME,
      VK_EXT_SWAPCHAIN_MAINTENANCE_1_EXTENSION_NAME,
      VK_GOOGLE_DISPLAY_TIMING_EXTENSION_NAME,
      VK_EXT_DISPLAY_CONTROL_EXTENSION_NAME,
   };
   for (const char *s : kStale)
      if (streq(name, s))
         return true;
   return false;
}

static VkLayerInstanceCreateInfo *
helios_find_instance_chain_info(const VkInstanceCreateInfo *info,
                                VkLayerFunction func)
{
   VkLayerInstanceCreateInfo *ci = (VkLayerInstanceCreateInfo *)info->pNext;
   while (ci && !(ci->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO &&
                  ci->function == func))
      ci = (VkLayerInstanceCreateInfo *)ci->pNext;
   return ci;
}

static VkLayerDeviceCreateInfo *
helios_find_device_chain_info(const VkDeviceCreateInfo *info,
                              VkLayerFunction func)
{
   VkLayerDeviceCreateInfo *ci = (VkLayerDeviceCreateInfo *)info->pNext;
   while (ci && !(ci->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO &&
                  ci->function == func))
      ci = (VkLayerDeviceCreateInfo *)ci->pNext;
   return ci;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_CreateInstance(const VkInstanceCreateInfo *pCreateInfo,
                      const VkAllocationCallbacks *pAllocator,
                      VkInstance *pInstance)
{
   VkLayerInstanceCreateInfo *chain =
      helios_find_instance_chain_info(pCreateInfo, VK_LAYER_LINK_INFO);
   if (!chain || !chain->u.pLayerInfo)
      return helios_refuse(HELIOS_CNT_create_instance_refused_no_link_info,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);

   PFN_vkGetInstanceProcAddr next_gipa =
      chain->u.pLayerInfo->pfnNextGetInstanceProcAddr;
   PFN_GetPhysicalDeviceProcAddr next_gpdpa =
      chain->u.pLayerInfo->pfnNextGetPhysicalDeviceProcAddr;
   /* Loader-owned memory, not application memory: advancing the link is the
    * documented layer protocol. */
   chain->u.pLayerInfo = chain->u.pLayerInfo->pNext;

   PFN_vkCreateInstance next_create =
      (PFN_vkCreateInstance)next_gipa(VK_NULL_HANDLE, "vkCreateInstance");
   if (!next_create)
      return helios_refuse(HELIOS_CNT_create_instance_refused_no_link_info,
                           VK_ERROR_INITIALIZATION_FAILED, "next vkCreateInstance");

   /* Consume this layer's instance extensions: they are removed from the
    * copied lower array (§10.7:2246-2248). The copy is ours; the caller's
    * array is never written. */
   bool want_surface = false, want_win32 = false, want_caps2 = false;
   /* ⚠ OBSERVED, NOT CONSUMED. These two are not this layer's extensions: they
    * belong to the app and the lower ICD implements them, so they must reach
    * `lower_exts` unchanged. We record them only to gate the KHR aliases of
    * the entry points they were promoted from — dropping either from the lower
    * list would disable a working extension to answer a naming question. */
   bool want_device_group_creation = false, want_physdev_props2 = false;
   std::vector<const char *> lower_exts;
   lower_exts.reserve(pCreateInfo->enabledExtensionCount);
   for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
      const char *name = pCreateInfo->ppEnabledExtensionNames[i];
      if (streq(name, VK_KHR_SURFACE_EXTENSION_NAME)) {
         want_surface = true;
         continue;
      }
      if (streq(name, VK_KHR_WIN32_SURFACE_EXTENSION_NAME)) {
         want_win32 = true;
         continue;
      }
      if (streq(name, VK_KHR_GET_SURFACE_CAPABILITIES_2_EXTENSION_NAME)) {
         want_caps2 = true;
         continue;
      }
      if (streq(name, VK_KHR_DEVICE_GROUP_CREATION_EXTENSION_NAME))
         want_device_group_creation = true; /* falls through to lower_exts */
      else if (streq(name, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME))
         want_physdev_props2 = true;        /* falls through to lower_exts */
      if (helios_is_stale_lower_wsi_name(name))
         continue;
      lower_exts.push_back(name);
   }

   VkInstanceCreateInfo lower = *pCreateInfo;
   lower.enabledExtensionCount = (uint32_t)lower_exts.size();
   lower.ppEnabledExtensionNames = lower_exts.empty() ? nullptr : lower_exts.data();

   VkResult res = next_create(&lower, pAllocator, pInstance);
   if (res != VK_SUCCESS)
      return res;

   HeliosInstance *inst = new (std::nothrow) HeliosInstance();
   if (!inst) {
      PFN_vkDestroyInstance destroy =
         (PFN_vkDestroyInstance)next_gipa(*pInstance, "vkDestroyInstance");
      if (destroy)
         destroy(*pInstance, pAllocator);
      *pInstance = VK_NULL_HANDLE;
      return helios_refuse(HELIOS_CNT_create_instance_refused_alloc,
                           VK_ERROR_OUT_OF_HOST_MEMORY, nullptr);
   }

   inst->instance = *pInstance;
   inst->next_gpdpa = next_gpdpa;
   inst->surface_enabled = want_surface;
   inst->win32_surface_enabled = want_win32;
   inst->surface_caps2_enabled = want_caps2 && want_surface;
   inst->device_group_creation_enabled = want_device_group_creation;
   inst->physdev_props2_enabled = want_physdev_props2;
   inst->app_api_version =
      (pCreateInfo->pApplicationInfo && pCreateInfo->pApplicationInfo->apiVersion)
         ? pCreateInfo->pApplicationInfo->apiVersion
         : VK_API_VERSION_1_0;

#define HELIOS_GIPA(field, name)                                              \
   inst->disp.field = (PFN_vk##field)next_gipa(*pInstance, name)
   inst->disp.GetInstanceProcAddr = next_gipa;
   HELIOS_GIPA(DestroyInstance, "vkDestroyInstance");
   HELIOS_GIPA(CreateDevice, "vkCreateDevice");
   HELIOS_GIPA(EnumerateDeviceExtensionProperties,
               "vkEnumerateDeviceExtensionProperties");
   HELIOS_GIPA(EnumeratePhysicalDevices, "vkEnumeratePhysicalDevices");
   HELIOS_GIPA(EnumeratePhysicalDeviceGroups, "vkEnumeratePhysicalDeviceGroups");
   HELIOS_GIPA(GetPhysicalDeviceProperties, "vkGetPhysicalDeviceProperties");
   HELIOS_GIPA(GetPhysicalDeviceProperties2, "vkGetPhysicalDeviceProperties2");
   HELIOS_GIPA(GetPhysicalDeviceQueueFamilyProperties,
               "vkGetPhysicalDeviceQueueFamilyProperties");
   HELIOS_GIPA(GetPhysicalDeviceQueueFamilyProperties2,
               "vkGetPhysicalDeviceQueueFamilyProperties2");
   HELIOS_GIPA(GetPhysicalDeviceImageFormatProperties2,
               "vkGetPhysicalDeviceImageFormatProperties2");
   HELIOS_GIPA(GetPhysicalDeviceExternalSemaphoreProperties,
               "vkGetPhysicalDeviceExternalSemaphoreProperties");
#undef HELIOS_GIPA

   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_instances[helios_key(*pInstance)] = inst;
   }
   helios_log("[helios-wsi] instance %p created (surface=%d win32=%d caps2=%d)",
              (void *)*pInstance, (int)want_surface, (int)want_win32,
              (int)want_caps2);
   return VK_SUCCESS;
}

static VKAPI_ATTR void VKAPI_CALL
helios_DestroyInstance(VkInstance instance, const VkAllocationCallbacks *pAllocator)
{
   HeliosInstance *inst = helios_instance_of(instance);
   if (!inst)
      return;

   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_instances.erase(helios_key(instance));
   }
   helios_com_release(inst->dxgi_factory);

   PFN_vkDestroyInstance destroy = inst->disp.DestroyInstance;
   delete inst;
   if (destroy)
      destroy(instance, pAllocator);

   helios_dump_counters("vkDestroyInstance");
}

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateInstanceLayerProperties(uint32_t *pCount,
                                              VkLayerProperties *pProps)
{
   VkLayerProperties props = {};
   strncpy(props.layerName, HELIOS_LAYER_NAME, sizeof(props.layerName) - 1);
   strncpy(props.description, HELIOS_LAYER_DESCRIPTION,
           sizeof(props.description) - 1);
   props.specVersion = VK_HEADER_VERSION_COMPLETE;
   props.implementationVersion = HELIOS_LAYER_IMPL_VERSION;
   return helios_write_array(&props, 1u, pProps, pCount);
}

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateDeviceLayerProperties(VkPhysicalDevice, uint32_t *pCount,
                                            VkLayerProperties *pProps)
{
   return helios_layer_EnumerateInstanceLayerProperties(pCount, pProps);
}

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_EnumerateInstanceExtensionProperties(const char *pLayerName,
                                                  uint32_t *pCount,
                                                  VkExtensionProperties *pProps)
{
   /* There is no next chain at this point, so pLayerName==NULL can only report
    * this layer's own instance extensions; the loader merges the ICD's. Both
    * accepted forms implement the two-call/VK_INCOMPLETE contract
    * (§10.7:2241-2245). Any other layer name is not ours. */
   if (pLayerName != nullptr && !streq(pLayerName, HELIOS_LAYER_NAME))
      return VK_ERROR_LAYER_NOT_PRESENT;

   const uint32_t n =
      (uint32_t)(sizeof(kInstanceExtensions) / sizeof(kInstanceExtensions[0]));
   return helios_write_array(kInstanceExtensions, n, pProps, pCount);
}

/* ==================================================================== */
/* 6. Physical-device admission and the intercepted queries             */
/* ==================================================================== */

static IDXGIFactory4 *
helios_dxgi_factory(HeliosInstance *inst)
{
   std::lock_guard<std::mutex> g(inst->lock);
   if (!inst->dxgi_factory) {
      IDXGIFactory4 *f = nullptr;
      if (FAILED(CreateDXGIFactory2(0, IID_PPV_ARGS(&f))) || !f) {
         helios_refuse(HELIOS_CNT_swapchain_refused_dxgi_factory,
                       VK_ERROR_INITIALIZATION_FAILED, "CreateDXGIFactory2");
         return nullptr;
      }
      inst->dxgi_factory = f;
   }
   return inst->dxgi_factory;
}

/* Caller owns the returned adapter reference. */
static IDXGIAdapter1 *
helios_adapter_by_luid(HeliosInstance *inst, LUID luid)
{
   IDXGIFactory4 *factory = helios_dxgi_factory(inst);
   if (!factory)
      return nullptr;
   IDXGIAdapter1 *adapter = nullptr;
   if (FAILED(factory->EnumAdapterByLuid(luid, IID_PPV_ARGS(&adapter))))
      return nullptr;
   return adapter;
}

static bool
helios_lower_has_device_extension(HeliosInstance *inst, VkPhysicalDevice phys,
                                  const char *name)
{
   if (!inst->disp.EnumerateDeviceExtensionProperties)
      return false;
   uint32_t n = 0;
   if (inst->disp.EnumerateDeviceExtensionProperties(phys, nullptr, &n, nullptr) !=
       VK_SUCCESS)
      return false;
   std::vector<VkExtensionProperties> props(n);
   if (n && inst->disp.EnumerateDeviceExtensionProperties(phys, nullptr, &n,
                                                          props.data()) < 0)
      return false;
   for (uint32_t i = 0; i < n; i++)
      if (streq(props[i].extensionName, name))
         return true;
   return false;
}

/*
 * The complete pre-advertisement admission gate (§10.7:2456, §10.9:2856).
 * Failure of ANY row makes the surface report no support; nothing is
 * approximated and no fallback path exists.
 */
static const HeliosPhysDevInfo &
helios_admit(HeliosInstance *inst, VkPhysicalDevice phys)
{
   std::unique_lock<std::mutex> g(inst->lock);
   HeliosPhysDevInfo &info = inst->phys[phys];
   if (info.computed)
      return info;
   info.computed = true;
   info.admitted = false;
   g.unlock();

   /* --- Vulkan 1.3 --- */
   VkPhysicalDeviceProperties props = {};
   if (!inst->disp.GetPhysicalDeviceProperties) {
      info.reject_reason = "no vkGetPhysicalDeviceProperties";
      return info;
   }
   inst->disp.GetPhysicalDeviceProperties(phys, &props);
   info.api_version = props.apiVersion;
   if (VK_API_VERSION_MAJOR(props.apiVersion) < 1 ||
       (VK_API_VERSION_MAJOR(props.apiVersion) == 1 &&
        VK_API_VERSION_MINOR(props.apiVersion) < 3)) {
      info.reject_reason = "lower device is not Vulkan 1.3";
      helios_refuse(HELIOS_CNT_physdev_refused_api_below_13, VK_ERROR_INITIALIZATION_FAILED,
                    info.reject_reason);
      return info;
   }

   /* --- LUID / UUIDs --- */
   VkPhysicalDeviceIDProperties id = {};
   id.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES;
   VkPhysicalDeviceProperties2 props2 = {};
   props2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
   props2.pNext = &id;
   if (!inst->disp.GetPhysicalDeviceProperties2) {
      info.reject_reason = "no vkGetPhysicalDeviceProperties2";
      return info;
   }
   inst->disp.GetPhysicalDeviceProperties2(phys, &props2);
   if (!id.deviceLUIDValid) {
      info.reject_reason = "lower device reports no LUID";
      helios_refuse(HELIOS_CNT_physdev_refused_no_luid, VK_ERROR_INITIALIZATION_FAILED,
                    info.reject_reason);
      return info;
   }
   memcpy(&info.luid, id.deviceLUID, sizeof(info.luid));
   memcpy(info.device_uuid, id.deviceUUID, VK_UUID_SIZE);
   memcpy(info.driver_uuid, id.driverUUID, VK_UUID_SIZE);

   /* --- the exact adapter must exist in DXGI (§10.7:2456 LUID row) --- */
   IDXGIAdapter1 *adapter = helios_adapter_by_luid(inst, info.luid);
   if (!adapter) {
      info.reject_reason = "no DXGI adapter with that LUID";
      helios_refuse(HELIOS_CNT_physdev_refused_no_dxgi_adapter,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }
   adapter->Release();

   /* --- canonical unprotected queue family with q >= 2 (§10.7:2284-2287) --- */
   uint32_t qn = 0;
   inst->disp.GetPhysicalDeviceQueueFamilyProperties(phys, &qn, nullptr);
   std::vector<VkQueueFamilyProperties> qprops(qn);
   if (qn)
      inst->disp.GetPhysicalDeviceQueueFamilyProperties(phys, &qn, qprops.data());
   uint32_t canonical = UINT32_MAX;
   for (uint32_t i = 0; i < qn; i++) {
      if ((qprops[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) &&
          (qprops[i].queueFlags & VK_QUEUE_TRANSFER_BIT ||
           qprops[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
         canonical = i;
         break;
      }
   }
   if (canonical == UINT32_MAX) {
      info.reject_reason = "no canonical unprotected graphics family";
      helios_refuse(HELIOS_CNT_physdev_refused_no_canonical_family,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }
   info.canonical_family = canonical;
   info.canonical_queue_count = qprops[canonical].queueCount;
   if (info.canonical_queue_count < 2) {
      info.reject_reason = "canonical family reports q < 2";
      helios_refuse(HELIOS_CNT_physdev_refused_queue_capacity,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }

   /* --- the lower Win32 external extensions must exist (§10.7:2276-2277) --- */
   if (!helios_lower_has_device_extension(inst, phys,
                                          VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME) ||
       !helios_lower_has_device_extension(inst, phys,
                                          VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME)) {
      info.reject_reason = "lower ICD lacks the Win32 external extensions";
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_image_import,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }

   /* --- exact external-image query: IMPORTABLE and DEDICATED_ONLY for the
    *     immutable C37 tuple, with the union of all three usages
    *     (§10.7:2433-2434, §10.7:2460) --- */
   VkPhysicalDeviceExternalImageFormatInfo ext_fmt_info = {};
   ext_fmt_info.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_IMAGE_FORMAT_INFO;
   ext_fmt_info.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT;

   VkPhysicalDeviceImageFormatInfo2 fmt_info = {};
   fmt_info.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_FORMAT_INFO_2;
   fmt_info.pNext = &ext_fmt_info;
   fmt_info.format = HELIOS_WSI_FORMAT;
   fmt_info.type = VK_IMAGE_TYPE_2D;
   fmt_info.tiling = VK_IMAGE_TILING_OPTIMAL;
   fmt_info.usage = HELIOS_WSI_SUPPORTED_USAGE;
   fmt_info.flags = 0;

   VkExternalImageFormatProperties ext_props = {};
   ext_props.sType = VK_STRUCTURE_TYPE_EXTERNAL_IMAGE_FORMAT_PROPERTIES;
   VkImageFormatProperties2 fmt_props = {};
   fmt_props.sType = VK_STRUCTURE_TYPE_IMAGE_FORMAT_PROPERTIES_2;
   fmt_props.pNext = &ext_props;

   /*
    * The two ways this query can fail are a LAYER defect and an ICD gap
    * respectively, and they were originally reported by the same string. A
    * refusal that cannot tell "could not run the check" from "the check said
    * no" sends the reader to the wrong repository: measured on 2026-08-10, it
    * cost a separate probe (tools/vk_external_handle_probe.cpp) to establish
    * that the entry point was present and the driver had answered
    * FORMAT_NOT_SUPPORTED. `reject_reason` is cached in inst->phys and must
    * stay a string literal, so the VkResult is named by a switch rather than
    * formatted.
    */
   if (!inst->disp.GetPhysicalDeviceImageFormatProperties2) {
      info.reject_reason = "no vkGetPhysicalDeviceImageFormatProperties2 "
                           "(layer dispatch defect, not an ICD capability)";
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_image_import,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }
   VkResult fmt_query =
      inst->disp.GetPhysicalDeviceImageFormatProperties2(phys, &fmt_info, &fmt_props);
   if (fmt_query != VK_SUCCESS) {
      switch (fmt_query) {
      case VK_ERROR_FORMAT_NOT_SUPPORTED:
         info.reject_reason =
            "external image query: FORMAT_NOT_SUPPORTED - the lower ICD does not "
            "support D3D12_RESOURCE_BIT for the C37 tuple";
         break;
      case VK_ERROR_OUT_OF_HOST_MEMORY:
         info.reject_reason = "external image query: OUT_OF_HOST_MEMORY";
         break;
      case VK_ERROR_OUT_OF_DEVICE_MEMORY:
         info.reject_reason = "external image query: OUT_OF_DEVICE_MEMORY";
         break;
      default:
         info.reject_reason = "external image query failed with an unexpected VkResult";
         break;
      }
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_image_import,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }
   const VkExternalMemoryFeatureFlags mem_features =
      ext_props.externalMemoryProperties.externalMemoryFeatures;
   if (!(mem_features & VK_EXTERNAL_MEMORY_FEATURE_IMPORTABLE_BIT)) {
      info.reject_reason = "D3D12_RESOURCE_BIT not IMPORTABLE";
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_image_import,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }
   if (!(mem_features & VK_EXTERNAL_MEMORY_FEATURE_DEDICATED_ONLY_BIT)) {
      /* §10.3:1150-1151 requires the query to report mandatory DEDICATED_ONLY
       * behaviour. Anything else is a different import contract. */
      info.reject_reason = "D3D12_RESOURCE_BIT not DEDICATED_ONLY";
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_image_dedicated,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }

   /* --- exact external-semaphore query: D3D12_FENCE_BIT IMPORTABLE --- */
   /*
    * The query must describe the semaphore this layer will actually create.
    * §10.3:1170-1178 creates Ready[i]/Release[i] with
    * VkSemaphoreTypeCreateInfo{TIMELINE, initialValue=0} and then imports
    * D3D12_FENCE_BIT into them, so the capability question is about a TIMELINE
    * semaphore. Omitting this chain asks about a BINARY one — a semaphore this
    * layer never creates — and a driver is entitled to answer differently:
    * measured on the Helios ICD 2026-08-10, D3D12_FENCE_BIT is importable for
    * timeline semaphores and absent for binary ones, so the unchained query
    * refused a device that satisfies the contract.
    */
   VkSemaphoreTypeCreateInfo sem_type_info = {};
   sem_type_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
   sem_type_info.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
   sem_type_info.initialValue = 0;

   VkPhysicalDeviceExternalSemaphoreInfo sem_info = {};
   sem_info.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_SEMAPHORE_INFO;
   sem_info.pNext = &sem_type_info;
   sem_info.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT;
   VkExternalSemaphoreProperties sem_props = {};
   sem_props.sType = VK_STRUCTURE_TYPE_EXTERNAL_SEMAPHORE_PROPERTIES;
   if (!inst->disp.GetPhysicalDeviceExternalSemaphoreProperties) {
      info.reject_reason = "no vkGetPhysicalDeviceExternalSemaphoreProperties";
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_semaphore_import,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }
   inst->disp.GetPhysicalDeviceExternalSemaphoreProperties(phys, &sem_info,
                                                           &sem_props);
   if (!(sem_props.externalSemaphoreFeatures &
         VK_EXTERNAL_SEMAPHORE_FEATURE_IMPORTABLE_BIT)) {
      info.reject_reason = "D3D12_FENCE_BIT not IMPORTABLE";
      helios_refuse(HELIOS_CNT_physdev_refused_no_external_semaphore_import,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return info;
   }

   /* --- singleton device group (C45, §10.7:2320-2322) --- */
   if (inst->disp.EnumeratePhysicalDeviceGroups) {
      uint32_t gn = 0;
      if (inst->disp.EnumeratePhysicalDeviceGroups(inst->instance, &gn, nullptr) ==
             VK_SUCCESS &&
          gn) {
         std::vector<VkPhysicalDeviceGroupProperties> groups(gn);
         for (auto &gp : groups)
            gp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_GROUP_PROPERTIES;
         if (inst->disp.EnumeratePhysicalDeviceGroups(inst->instance, &gn,
                                                      groups.data()) >= 0) {
            bool singleton = false;
            for (uint32_t i = 0; i < gn; i++) {
               for (uint32_t j = 0; j < groups[i].physicalDeviceCount; j++) {
                  if (groups[i].physicalDevices[j] == phys) {
                     singleton = groups[i].physicalDeviceCount == 1;
                     break;
                  }
               }
            }
            if (!singleton) {
               info.reject_reason = "physical device is not a one-member group";
               helios_refuse(HELIOS_CNT_physdev_refused_not_singleton_group,
                             VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
               return info;
            }
         }
      }
   }

   info.admitted = true;
   info.reject_reason = "admitted";
   helios_log("[helios-wsi] physical device %p admitted: canonical family %u, q=%u",
              (void *)phys, info.canonical_family, info.canonical_queue_count);
   return info;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_EnumerateDeviceExtensionProperties(VkPhysicalDevice physicalDevice,
                                          const char *pLayerName,
                                          uint32_t *pCount,
                                          VkExtensionProperties *pProps)
{
   HeliosInstance *inst = helios_instance_of_phys(physicalDevice);
   const bool admitted =
      inst && inst->wsi_enabled() && helios_admit(inst, physicalDevice).admitted;

   if (pLayerName != nullptr) {
      if (!streq(pLayerName, HELIOS_LAYER_NAME))
         return VK_ERROR_LAYER_NOT_PRESENT;
      const uint32_t n = admitted ? 1u : 0u;
      return helios_write_array(kDeviceExtensions, n, pProps, pCount);
   }

   if (!inst || !inst->disp.EnumerateDeviceExtensionProperties)
      return VK_ERROR_INITIALIZATION_FAILED;

   /* Merge: preserve unrelated lower extensions, remove every stale lower
    * Win32-surface/swapchain name, add ours without duplicates
    * (§10.7:2241-2245). */
   uint32_t lower_n = 0;
   VkResult r = inst->disp.EnumerateDeviceExtensionProperties(physicalDevice, nullptr,
                                                              &lower_n, nullptr);
   if (r != VK_SUCCESS)
      return r;
   std::vector<VkExtensionProperties> lower(lower_n);
   if (lower_n) {
      r = inst->disp.EnumerateDeviceExtensionProperties(physicalDevice, nullptr,
                                                        &lower_n, lower.data());
      if (r < 0)
         return r;
      lower.resize(lower_n);
   }

   std::vector<VkExtensionProperties> merged;
   merged.reserve(lower.size() + 1);
   for (const VkExtensionProperties &e : lower) {
      if (helios_is_stale_lower_wsi_name(e.extensionName))
         continue;
      merged.push_back(e);
   }
   if (admitted)
      merged.push_back(kDeviceExtensions[0]);

   return helios_write_array(merged.data(), (uint32_t)merged.size(), pProps, pCount);
}

/*
 * §10.7:2287-2288 — for a WSI-enabled instance both queue-family-properties
 * entry points report q-1 app-visible queues for the canonical family and
 * otherwise preserve the lower properties. The hidden queue is the layer's
 * private helper.
 */
static void
helios_hide_private_queue(HeliosInstance *inst, VkPhysicalDevice phys,
                          uint32_t first, uint32_t count,
                          VkQueueFamilyProperties *props, size_t stride)
{
   if (!inst->wsi_enabled())
      return;
   const HeliosPhysDevInfo &info = helios_admit(inst, phys);
   if (!info.admitted)
      return;
   for (uint32_t i = 0; i < count; i++) {
      const uint32_t family = first + i;
      if (family != info.canonical_family)
         continue;
      VkQueueFamilyProperties *p =
         (VkQueueFamilyProperties *)((char *)props + (size_t)i * stride);
      if (p->queueCount > 0)
         p->queueCount -= 1;
   }
}

static VKAPI_ATTR void VKAPI_CALL
helios_GetPhysicalDeviceQueueFamilyProperties(VkPhysicalDevice physicalDevice,
                                              uint32_t *pCount,
                                              VkQueueFamilyProperties *pProps)
{
   HeliosInstance *inst = helios_instance_of_phys(physicalDevice);
   if (!inst || !inst->disp.GetPhysicalDeviceQueueFamilyProperties)
      return;
   inst->disp.GetPhysicalDeviceQueueFamilyProperties(physicalDevice, pCount, pProps);
   if (pProps)
      helios_hide_private_queue(inst, physicalDevice, 0, *pCount, pProps,
                                sizeof(VkQueueFamilyProperties));
}

static VKAPI_ATTR void VKAPI_CALL
helios_GetPhysicalDeviceQueueFamilyProperties2(VkPhysicalDevice physicalDevice,
                                               uint32_t *pCount,
                                               VkQueueFamilyProperties2 *pProps)
{
   HeliosInstance *inst = helios_instance_of_phys(physicalDevice);
   if (!inst || !inst->disp.GetPhysicalDeviceQueueFamilyProperties2)
      return;
   inst->disp.GetPhysicalDeviceQueueFamilyProperties2(physicalDevice, pCount, pProps);
   if (pProps)
      helios_hide_private_queue(inst, physicalDevice, 0, *pCount,
                                &pProps[0].queueFamilyProperties,
                                sizeof(VkQueueFamilyProperties2));
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_EnumeratePhysicalDeviceGroups(VkInstance instance, uint32_t *pCount,
                                     VkPhysicalDeviceGroupProperties *pProps)
{
   HeliosInstance *inst = helios_instance_of(instance);
   if (!inst || !inst->disp.EnumeratePhysicalDeviceGroups)
      return VK_ERROR_INITIALIZATION_FAILED;
   /* The layer neither invents nor merges groups; C45 admission rejects any
    * non-singleton group for WSI use (helios_admit). */
   return inst->disp.EnumeratePhysicalDeviceGroups(instance, pCount, pProps);
}

/* ==================================================================== */
/* 7. Surface ownership and the copy-only profile queries               */
/* ==================================================================== */

static VKAPI_ATTR VkResult VKAPI_CALL
helios_CreateWin32SurfaceKHR(VkInstance instance,
                             const VkWin32SurfaceCreateInfoKHR *pCreateInfo,
                             const VkAllocationCallbacks *pAllocator,
                             VkSurfaceKHR *pSurface)
{
   HeliosInstance *inst = helios_instance_of(instance);
   if (!inst || !inst->wsi_enabled())
      return helios_refuse(HELIOS_CNT_surface_create_refused_no_wsi,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (!pCreateInfo || pCreateInfo->flags != 0)
      return helios_refuse(HELIOS_CNT_surface_create_refused_pnext,
                           VK_ERROR_INITIALIZATION_FAILED, "nonzero flags");
   /* No surface-create extension structure is advertised this generation. */
   if (pCreateInfo->pNext != nullptr)
      return helios_refuse(HELIOS_CNT_surface_create_refused_pnext,
                           VK_ERROR_INITIALIZATION_FAILED, "unadvertised pNext");
   if (!helios_window_alive(pCreateInfo->hwnd))
      return helios_refuse(HELIOS_CNT_surface_create_refused_bad_hwnd,
                           VK_ERROR_INITIALIZATION_FAILED, "HWND is not a window");

   HeliosSurface *surf = new (std::nothrow) HeliosSurface();
   if (!surf)
      return helios_refuse(HELIOS_CNT_surface_create_refused_alloc,
                           VK_ERROR_OUT_OF_HOST_MEMORY, nullptr);
   surf->inst = inst;
   surf->hwnd = pCreateInfo->hwnd;
   surf->hinstance = pCreateInfo->hinstance;

   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_surfaces.insert(surf);
   }
   *pSurface = helios_surface_handle(surf);
   (void)pAllocator;
   return VK_SUCCESS;
}

static VKAPI_ATTR void VKAPI_CALL
helios_DestroySurfaceKHR(VkInstance instance, VkSurfaceKHR surface,
                         const VkAllocationCallbacks *pAllocator)
{
   (void)instance;
   (void)pAllocator;
   HeliosSurface *surf = helios_surface_of(surface);
   if (!surf)
      return;
   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_surfaces.erase(surf);
   }
   surf->magic = 0;
   delete surf;
}

/* Resolves a surface handle and its liveness in one step. Returns:
 *   VK_SUCCESS               live layer surface, *out set
 *   VK_ERROR_SURFACE_LOST_KHR  the HWND is gone
 *   VK_ERROR_INITIALIZATION_FAILED  not our handle at all */
static VkResult
helios_resolve_surface(VkSurfaceKHR surface, HeliosSurface **out)
{
   HeliosSurface *s = helios_surface_of(surface);
   if (!s)
      return helios_refuse(HELIOS_CNT_surface_query_refused_unknown_surface,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   *out = s;
   if (!helios_window_alive(s->hwnd))
      return helios_refuse(HELIOS_CNT_surface_query_surface_lost,
                           VK_ERROR_SURFACE_LOST_KHR, nullptr);
   return VK_SUCCESS;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDeviceSurfaceSupportKHR(VkPhysicalDevice physicalDevice,
                                          uint32_t queueFamilyIndex,
                                          VkSurfaceKHR surface,
                                          VkBool32 *pSupported)
{
   *pSupported = VK_FALSE;
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(surface, &surf);
   if (r != VK_SUCCESS)
      return r == VK_ERROR_SURFACE_LOST_KHR ? r : VK_ERROR_SURFACE_LOST_KHR;

   HeliosInstance *inst = helios_instance_of_phys(physicalDevice);
   if (!inst || !inst->wsi_enabled())
      return VK_SUCCESS;
   const HeliosPhysDevInfo &info = helios_admit(inst, physicalDevice);
   /* §10.7:2590-2591 — support only for the canonical unprotected family. */
   if (info.admitted && queueFamilyIndex == info.canonical_family)
      *pSupported = VK_TRUE;
   return VK_SUCCESS;
}

static VKAPI_ATTR VkBool32 VKAPI_CALL
helios_GetPhysicalDeviceWin32PresentationSupportKHR(VkPhysicalDevice physicalDevice,
                                                    uint32_t queueFamilyIndex)
{
   HeliosInstance *inst = helios_instance_of_phys(physicalDevice);
   if (!inst || !inst->wsi_enabled())
      return VK_FALSE;
   const HeliosPhysDevInfo &info = helios_admit(inst, physicalDevice);
   return (info.admitted && queueFamilyIndex == info.canonical_family) ? VK_TRUE
                                                                       : VK_FALSE;
}

static void
helios_fill_surface_caps(const VkExtent2D &extent, VkSurfaceCapabilitiesKHR *caps)
{
   /* §10.7:2458-2460 — every field is a constant of the profile except the
    * current client extent, which min and max both equal. */
   caps->minImageCount = HELIOS_WSI_MIN_IMAGE_COUNT;
   caps->maxImageCount = HELIOS_WSI_MAX_IMAGE_COUNT;
   caps->currentExtent = extent;
   caps->minImageExtent = extent;
   caps->maxImageExtent = extent;
   caps->maxImageArrayLayers = HELIOS_WSI_MAX_ARRAY_LAYERS;
   caps->supportedTransforms = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
   caps->currentTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
   caps->supportedCompositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
   caps->supportedUsageFlags = HELIOS_WSI_SUPPORTED_USAGE;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDeviceSurfaceCapabilitiesKHR(VkPhysicalDevice physicalDevice,
                                               VkSurfaceKHR surface,
                                               VkSurfaceCapabilitiesKHR *pCaps)
{
   (void)physicalDevice;
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(surface, &surf);
   if (r != VK_SUCCESS)
      return VK_ERROR_SURFACE_LOST_KHR;

   VkExtent2D extent = {};
   if (!helios_client_extent(surf->hwnd, &extent))
      return helios_refuse(HELIOS_CNT_surface_query_surface_lost,
                           VK_ERROR_SURFACE_LOST_KHR, nullptr);
   memset(pCaps, 0, sizeof(*pCaps));
   helios_fill_surface_caps(extent, pCaps);
   return VK_SUCCESS;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDeviceSurfaceCapabilities2KHR(
   VkPhysicalDevice physicalDevice,
   const VkPhysicalDeviceSurfaceInfo2KHR *pSurfaceInfo,
   VkSurfaceCapabilities2KHR *pCaps)
{
   /* §10.7:2221-2222 — capabilities2 is admitted only when it returns the same
    * base profile and rejects every unadvertised extension structure. An input
    * chain can only carry such a structure this generation, so any non-NULL
    * input pNext is refused. Output-chain structures the layer does not
    * populate are left untouched (omitted), never approximated. */
   if (pSurfaceInfo->pNext != nullptr)
      return helios_refuse(HELIOS_CNT_surface_query_refused_unadvertised_pnext,
                           VK_ERROR_SURFACE_LOST_KHR, "capabilities2 input pNext");
   return helios_GetPhysicalDeviceSurfaceCapabilitiesKHR(
      physicalDevice, pSurfaceInfo->surface, &pCaps->surfaceCapabilities);
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDeviceSurfaceFormatsKHR(VkPhysicalDevice physicalDevice,
                                          VkSurfaceKHR surface, uint32_t *pCount,
                                          VkSurfaceFormatKHR *pFormats)
{
   (void)physicalDevice;
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(surface, &surf);
   if (r != VK_SUCCESS)
      return VK_ERROR_SURFACE_LOST_KHR;

   const VkSurfaceFormatKHR only = { HELIOS_WSI_FORMAT, HELIOS_WSI_COLOR_SPACE };
   return helios_write_array(&only, 1u, pFormats, pCount);
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDeviceSurfaceFormats2KHR(
   VkPhysicalDevice physicalDevice,
   const VkPhysicalDeviceSurfaceInfo2KHR *pSurfaceInfo, uint32_t *pCount,
   VkSurfaceFormat2KHR *pFormats)
{
   if (pSurfaceInfo->pNext != nullptr)
      return helios_refuse(HELIOS_CNT_surface_query_refused_unadvertised_pnext,
                           VK_ERROR_SURFACE_LOST_KHR, "formats2 input pNext");
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(pSurfaceInfo->surface, &surf);
   if (r != VK_SUCCESS)
      return VK_ERROR_SURFACE_LOST_KHR;
   (void)physicalDevice;

   if (!pFormats) {
      *pCount = 1;
      return VK_SUCCESS;
   }
   if (*pCount == 0)
      return VK_INCOMPLETE;
   /* Preserve the caller's output pNext chain; only surfaceFormat is written. */
   pFormats[0].surfaceFormat.format = HELIOS_WSI_FORMAT;
   pFormats[0].surfaceFormat.colorSpace = HELIOS_WSI_COLOR_SPACE;
   *pCount = 1;
   return VK_SUCCESS;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDeviceSurfacePresentModesKHR(VkPhysicalDevice physicalDevice,
                                               VkSurfaceKHR surface,
                                               uint32_t *pCount,
                                               VkPresentModeKHR *pModes)
{
   (void)physicalDevice;
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(surface, &surf);
   if (r != VK_SUCCESS)
      return VK_ERROR_SURFACE_LOST_KHR;

   /* §10.7:2231-2232 — no mailbox, immediate, FIFO-relaxed, shared, or
    * latest-ready mode is ever enumerated. */
   const VkPresentModeKHR only = HELIOS_WSI_PRESENT_MODE;
   return helios_write_array(&only, 1u, pModes, pCount);
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetPhysicalDevicePresentRectanglesKHR(VkPhysicalDevice physicalDevice,
                                             VkSurfaceKHR surface,
                                             uint32_t *pCount, VkRect2D *pRects)
{
   (void)physicalDevice;
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(surface, &surf);
   if (r != VK_SUCCESS)
      return VK_ERROR_SURFACE_LOST_KHR;

   VkExtent2D extent = {};
   if (!helios_client_extent(surf->hwnd, &extent))
      return helios_refuse(HELIOS_CNT_surface_query_surface_lost,
                           VK_ERROR_SURFACE_LOST_KHR, nullptr);

   /* §10.7:2329-2331 — exactly one non-overlapping rectangle covering the
    * current client extent, refreshed on every call. */
   VkRect2D rect = {};
   rect.offset.x = 0;
   rect.offset.y = 0;
   rect.extent = extent;
   return helios_write_array(&rect, 1u, pRects, pCount);
}

/* ==================================================================== */
/* 8. Device creation                                                   */
/* ==================================================================== */

/*
 * §10.7:2278-2280 requires timelineSemaphore and synchronization2 to be
 * VK_TRUE in the lower create chain, while "never mutating application memory
 * or inserting duplicate feature structures".
 *
 * Those two requirements together mean: if the caller already supplies a
 * structure that governs the feature and sets it FALSE, the layer must clone
 * the chain prefix up to and including that node rather than write through the
 * caller's pointer. Cloning needs the node's size, so this table exists. An
 * unknown sType *before* a node that must be modified is a hard refusal
 * (create_device_refused_uncopyable_feature_chain) — never a silent mutation
 * and never a duplicate structure.
 */
static size_t
helios_pnext_size(VkStructureType type)
{
   switch (type) {
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2:
      return sizeof(VkPhysicalDeviceFeatures2);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES:
      return sizeof(VkPhysicalDeviceVulkan11Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES:
      return sizeof(VkPhysicalDeviceVulkan12Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES:
      return sizeof(VkPhysicalDeviceVulkan13Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_4_FEATURES:
      return sizeof(VkPhysicalDeviceVulkan14Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES:
      return sizeof(VkPhysicalDeviceTimelineSemaphoreFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SYNCHRONIZATION_2_FEATURES:
      return sizeof(VkPhysicalDeviceSynchronization2Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES:
      return sizeof(VkPhysicalDeviceDynamicRenderingFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES:
      return sizeof(VkPhysicalDeviceDescriptorIndexingFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES:
      return sizeof(VkPhysicalDeviceBufferDeviceAddressFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGELESS_FRAMEBUFFER_FEATURES:
      return sizeof(VkPhysicalDeviceImagelessFramebufferFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_8BIT_STORAGE_FEATURES:
      return sizeof(VkPhysicalDevice8BitStorageFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES:
      return sizeof(VkPhysicalDevice16BitStorageFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_FLOAT16_INT8_FEATURES:
      return sizeof(VkPhysicalDeviceShaderFloat16Int8Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES:
      return sizeof(VkPhysicalDeviceScalarBlockLayoutFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_UNIFORM_BUFFER_STANDARD_LAYOUT_FEATURES:
      return sizeof(VkPhysicalDeviceUniformBufferStandardLayoutFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_HOST_QUERY_RESET_FEATURES:
      return sizeof(VkPhysicalDeviceHostQueryResetFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SEPARATE_DEPTH_STENCIL_LAYOUTS_FEATURES:
      return sizeof(VkPhysicalDeviceSeparateDepthStencilLayoutsFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_SUBGROUP_EXTENDED_TYPES_FEATURES:
      return sizeof(VkPhysicalDeviceShaderSubgroupExtendedTypesFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_MEMORY_MODEL_FEATURES:
      return sizeof(VkPhysicalDeviceVulkanMemoryModelFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MAINTENANCE_4_FEATURES:
      return sizeof(VkPhysicalDeviceMaintenance4Features);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_INLINE_UNIFORM_BLOCK_FEATURES:
      return sizeof(VkPhysicalDeviceInlineUniformBlockFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PIPELINE_CREATION_CACHE_CONTROL_FEATURES:
      return sizeof(VkPhysicalDevicePipelineCreationCacheControlFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PRIVATE_DATA_FEATURES:
      return sizeof(VkPhysicalDevicePrivateDataFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_DEMOTE_TO_HELPER_INVOCATION_FEATURES:
      return sizeof(VkPhysicalDeviceShaderDemoteToHelperInvocationFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_INTEGER_DOT_PRODUCT_FEATURES:
      return sizeof(VkPhysicalDeviceShaderIntegerDotProductFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_TERMINATE_INVOCATION_FEATURES:
      return sizeof(VkPhysicalDeviceShaderTerminateInvocationFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SUBGROUP_SIZE_CONTROL_FEATURES:
      return sizeof(VkPhysicalDeviceSubgroupSizeControlFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TEXTURE_COMPRESSION_ASTC_HDR_FEATURES:
      return sizeof(VkPhysicalDeviceTextureCompressionASTCHDRFeatures);
   case VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ZERO_INITIALIZE_WORKGROUP_MEMORY_FEATURES:
      return sizeof(VkPhysicalDeviceZeroInitializeWorkgroupMemoryFeatures);
   case VK_STRUCTURE_TYPE_DEVICE_GROUP_DEVICE_CREATE_INFO:
      return sizeof(VkDeviceGroupDeviceCreateInfo);
   case VK_STRUCTURE_TYPE_DEVICE_MEMORY_OVERALLOCATION_CREATE_INFO_AMD:
      return sizeof(VkDeviceMemoryOverallocationCreateInfoAMD);
   /* Nodes that can legitimately precede a consumed WSI structure in an image
    * create or bind chain. */
   case VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO:
      return sizeof(VkExternalMemoryImageCreateInfo);
   case VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO:
      return sizeof(VkImageFormatListCreateInfo);
   case VK_STRUCTURE_TYPE_IMAGE_STENCIL_USAGE_CREATE_INFO:
      return sizeof(VkImageStencilUsageCreateInfo);
   case VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_DEVICE_GROUP_INFO:
      return sizeof(VkBindImageMemoryDeviceGroupInfo);
   case VK_STRUCTURE_TYPE_BIND_IMAGE_PLANE_MEMORY_INFO:
      return sizeof(VkBindImagePlaneMemoryInfo);
   case VK_STRUCTURE_TYPE_BIND_MEMORY_STATUS:
      return sizeof(VkBindMemoryStatus);
   case VK_STRUCTURE_TYPE_IMAGE_SWAPCHAIN_CREATE_INFO_KHR:
      return sizeof(VkImageSwapchainCreateInfoKHR);
   case VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_SWAPCHAIN_INFO_KHR:
      return sizeof(VkBindImageMemorySwapchainInfoKHR);
   default:
      return 0;
   }
}

/*
 * Removing one node from a caller-owned pNext chain without writing to
 * application memory. Only the prefix in front of the victim is cloned; the
 * tail after it is shared unchanged. An unknown sType in that prefix is a
 * refusal, never a mutation.
 */
struct HeliosStrippedChain {
   std::vector<std::vector<char>> nodes;
   const void *head = nullptr;
};

static bool
helios_strip_pnext(const void *chain_head, const VkBaseInStructure *victim,
                   HeliosStrippedChain *out)
{
   std::vector<const VkBaseInStructure *> chain;
   int victim_idx = -1;
   for (const VkBaseInStructure *s = (const VkBaseInStructure *)chain_head; s;
        s = s->pNext) {
      if (s == victim)
         victim_idx = (int)chain.size();
      chain.push_back(s);
   }
   if (victim_idx < 0) {
      out->head = chain_head;
      return true;
   }
   if (victim_idx == 0) {
      out->head = victim->pNext;
      return true;
   }

   out->nodes.resize((size_t)victim_idx);
   for (int i = 0; i < victim_idx; i++) {
      size_t sz = helios_pnext_size(chain[i]->sType);
      if (sz == 0)
         return false;
      out->nodes[i].resize(sz);
      memcpy(out->nodes[i].data(), chain[i], sz);
   }
   for (int i = 0; i + 1 < victim_idx; i++)
      ((VkBaseInStructure *)out->nodes[i].data())->pNext =
         (const VkBaseInStructure *)out->nodes[i + 1].data();
   ((VkBaseInStructure *)out->nodes[victim_idx - 1].data())->pNext = victim->pNext;
   out->head = out->nodes[0].data();
   return true;
}

struct HeliosFeatureChain {
   /* Backing store for the cloned prefix; nodes point into it. */
   std::vector<std::vector<char>> nodes;
   VkPhysicalDeviceTimelineSemaphoreFeatures ts = {};
   VkPhysicalDeviceSynchronization2Features sync2 = {};
   const void *head = nullptr;
};

static bool
helios_feature_governs_timeline(VkStructureType t)
{
   return t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES ||
          t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
}

static bool
helios_feature_governs_sync2(VkStructureType t)
{
   return t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SYNCHRONIZATION_2_FEATURES ||
          t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
}

static VkBool32 *
helios_timeline_slot(void *node, VkStructureType t)
{
   if (t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES)
      return &((VkPhysicalDeviceTimelineSemaphoreFeatures *)node)->timelineSemaphore;
   if (t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES)
      return &((VkPhysicalDeviceVulkan12Features *)node)->timelineSemaphore;
   return nullptr;
}

static VkBool32 *
helios_sync2_slot(void *node, VkStructureType t)
{
   if (t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SYNCHRONIZATION_2_FEATURES)
      return &((VkPhysicalDeviceSynchronization2Features *)node)->synchronization2;
   if (t == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES)
      return &((VkPhysicalDeviceVulkan13Features *)node)->synchronization2;
   return nullptr;
}

/* Builds the lower create chain. Returns false on a refusal already counted. */
static bool
helios_build_feature_chain(const void *app_chain, HeliosFeatureChain *out)
{
   /* Index the chain and locate the governing nodes. */
   std::vector<const VkBaseInStructure *> chain;
   int ts_idx = -1, sync2_idx = -1;
   for (const VkBaseInStructure *s = (const VkBaseInStructure *)app_chain; s;
        s = s->pNext) {
      if (helios_feature_governs_timeline(s->sType))
         ts_idx = (int)chain.size();
      if (helios_feature_governs_sync2(s->sType))
         sync2_idx = (int)chain.size();
      chain.push_back(s);
   }

   bool need_ts_write = false, need_sync2_write = false;
   if (ts_idx >= 0) {
      const VkBaseInStructure *n = chain[ts_idx];
      const VkBool32 *slot =
         helios_timeline_slot((void *)n, n->sType);
      need_ts_write = slot && *slot != VK_TRUE;
   }
   if (sync2_idx >= 0) {
      const VkBaseInStructure *n = chain[sync2_idx];
      const VkBool32 *slot = helios_sync2_slot((void *)n, n->sType);
      need_sync2_write = slot && *slot != VK_TRUE;
   }

   /* Deepest node that must be rewritten. */
   int deepest = -1;
   if (need_ts_write)
      deepest = ts_idx;
   if (need_sync2_write && sync2_idx > deepest)
      deepest = sync2_idx;

   const void *head = app_chain;

   if (deepest >= 0) {
      out->nodes.resize((size_t)deepest + 1);
      for (int i = 0; i <= deepest; i++) {
         size_t sz = helios_pnext_size(chain[i]->sType);
         if (sz == 0) {
            helios_refuse(HELIOS_CNT_create_device_refused_uncopyable_feature_chain,
                          VK_ERROR_INITIALIZATION_FAILED,
                          "unknown pNext sType before a feature that must be forced");
            return false;
         }
         out->nodes[i].resize(sz);
         memcpy(out->nodes[i].data(), chain[i], sz);
      }
      /* Relink the clones; the tail past `deepest` is shared unmodified. */
      for (int i = 0; i < deepest; i++) {
         ((VkBaseInStructure *)out->nodes[i].data())->pNext =
            (const VkBaseInStructure *)out->nodes[i + 1].data();
      }
      ((VkBaseInStructure *)out->nodes[deepest].data())->pNext = chain[deepest]->pNext;

      if (need_ts_write) {
         void *n = out->nodes[ts_idx].data();
         *helios_timeline_slot(n, chain[ts_idx]->sType) = VK_TRUE;
      }
      if (need_sync2_write) {
         void *n = out->nodes[sync2_idx].data();
         *helios_sync2_slot(n, chain[sync2_idx]->sType) = VK_TRUE;
      }
      head = out->nodes[0].data();
   }

   /* Nothing governs the feature: prepend our own structure. Prepending is not
    * a duplicate — there is no other structure that carries this bit. */
   if (ts_idx < 0) {
      out->ts.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
      out->ts.timelineSemaphore = VK_TRUE;
      out->ts.pNext = (void *)head;
      head = &out->ts;
   }
   if (sync2_idx < 0) {
      out->sync2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SYNCHRONIZATION_2_FEATURES;
      out->sync2.synchronization2 = VK_TRUE;
      out->sync2.pNext = (void *)head;
      head = &out->sync2;
   }

   out->head = head;
   return true;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_CreateDevice(VkPhysicalDevice physicalDevice,
                    const VkDeviceCreateInfo *pCreateInfo,
                    const VkAllocationCallbacks *pAllocator, VkDevice *pDevice)
{
   HeliosInstance *inst = helios_instance_of_phys(physicalDevice);
   VkLayerDeviceCreateInfo *chain =
      helios_find_device_chain_info(pCreateInfo, VK_LAYER_LINK_INFO);
   if (!inst || !chain || !chain->u.pLayerInfo)
      return helios_refuse(HELIOS_CNT_create_device_refused_no_link_info,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);

   PFN_vkGetInstanceProcAddr next_gipa =
      chain->u.pLayerInfo->pfnNextGetInstanceProcAddr;
   PFN_vkGetDeviceProcAddr next_gdpa = chain->u.pLayerInfo->pfnNextGetDeviceProcAddr;
   chain->u.pLayerInfo = chain->u.pLayerInfo->pNext;

   PFN_vkCreateDevice next_create =
      (PFN_vkCreateDevice)next_gipa(inst->instance, "vkCreateDevice");
   if (!next_create || !next_gdpa)
      return helios_refuse(HELIOS_CNT_create_device_refused_no_link_info,
                           VK_ERROR_INITIALIZATION_FAILED, "next vkCreateDevice");

   VkLayerDeviceCreateInfo *cb_info =
      helios_find_device_chain_info(pCreateInfo, VK_LOADER_DATA_CALLBACK);

   /* Does the application want the layer's WSI on this device? */
   bool wsi = false;
   /* Observed, not consumed — see the instance pair. */
   bool want_bind_memory2 = false;
   std::vector<const char *> lower_exts;
   lower_exts.reserve(pCreateInfo->enabledExtensionCount + 2);
   for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
      const char *name = pCreateInfo->ppEnabledExtensionNames[i];
      if (streq(name, VK_KHR_SWAPCHAIN_EXTENSION_NAME)) {
         wsi = true; /* consumed here; the lower ICD exposes no WSI */
         continue;
      }
      if (streq(name, VK_KHR_BIND_MEMORY_2_EXTENSION_NAME))
         want_bind_memory2 = true; /* falls through to lower_exts */
      if (helios_is_stale_lower_wsi_name(name))
         continue;
      lower_exts.push_back(name);
   }

   const HeliosPhysDevInfo &info = helios_admit(inst, physicalDevice);
   HeliosFeatureChain features;
   VkDeviceCreateInfo lower = *pCreateInfo;
   std::vector<VkDeviceQueueCreateInfo> queues;
   std::vector<float> priorities;
   uint32_t private_index = UINT32_MAX;

   if (wsi) {
      if (!info.admitted)
         return helios_refuse(HELIOS_CNT_create_device_refused_not_admitted,
                              VK_ERROR_EXTENSION_NOT_PRESENT, info.reject_reason);

      /* C45 singleton device group (§10.7:2320-2322). */
      const VkDeviceGroupDeviceCreateInfo *dg =
         (const VkDeviceGroupDeviceCreateInfo *)helios_find_pnext(
            pCreateInfo->pNext, VK_STRUCTURE_TYPE_DEVICE_GROUP_DEVICE_CREATE_INFO);
      if (dg && !(dg->physicalDeviceCount == 1 &&
                  dg->pPhysicalDevices[0] == physicalDevice))
         return helios_refuse(HELIOS_CNT_create_device_refused_device_group,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "not a one-member group naming this device");

      /* §10.7:2289-2296 — an original flags=0, nonzero-count record for the
       * canonical family is required; the layer clones exactly that record and
       * appends one queue at priority 1.0f. It never manufactures a new flags
       * class the application did not request. */
      int canon_idx = -1;
      for (uint32_t i = 0; i < pCreateInfo->queueCreateInfoCount; i++) {
         const VkDeviceQueueCreateInfo &q = pCreateInfo->pQueueCreateInfos[i];
         if (q.queueFamilyIndex == info.canonical_family && q.flags == 0 &&
             q.queueCount > 0) {
            canon_idx = (int)i;
            break;
         }
      }
      if (canon_idx < 0) {
         bool protected_only = false;
         for (uint32_t i = 0; i < pCreateInfo->queueCreateInfoCount; i++) {
            const VkDeviceQueueCreateInfo &q = pCreateInfo->pQueueCreateInfos[i];
            if (q.queueFamilyIndex == info.canonical_family &&
                (q.flags & VK_DEVICE_QUEUE_CREATE_PROTECTED_BIT))
               protected_only = true;
         }
         return helios_refuse(protected_only
                                 ? HELIOS_CNT_create_device_refused_protected_only
                                 : HELIOS_CNT_create_device_refused_no_canonical_queue_record,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "no flags=0 canonical-family queue record");
      }

      queues.assign(pCreateInfo->pQueueCreateInfos,
                    pCreateInfo->pQueueCreateInfos + pCreateInfo->queueCreateInfoCount);
      VkDeviceQueueCreateInfo &canon = queues[(size_t)canon_idx];
      if (canon.queueCount + 1u > info.canonical_queue_count)
         return helios_refuse(HELIOS_CNT_create_device_refused_queue_capacity,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "app queue count plus the helper exceeds q");

      private_index = canon.queueCount; /* the appended queue's lower index */
      priorities.assign(canon.pQueuePriorities,
                        canon.pQueuePriorities + canon.queueCount);
      priorities.push_back(1.0f);
      canon.queueCount += 1;
      canon.pQueuePriorities = priorities.data();
      /* canon.pNext is preserved verbatim: the clone keeps the exact record's
       * compatible pNext policy (§10.7:2295). */

      lower.queueCreateInfoCount = (uint32_t)queues.size();
      lower.pQueueCreateInfos = queues.data();

      lower_exts.push_back(VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME);
      lower_exts.push_back(VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME);

      if (!helios_build_feature_chain(pCreateInfo->pNext, &features))
         return VK_ERROR_INITIALIZATION_FAILED;
      lower.pNext = features.head;
   }

   /* De-duplicate the extension list without reordering the caller's names. */
   {
      std::vector<const char *> dedup;
      dedup.reserve(lower_exts.size());
      for (const char *n : lower_exts) {
         bool seen = false;
         for (const char *m : dedup)
            if (streq(n, m)) {
               seen = true;
               break;
            }
         if (!seen)
            dedup.push_back(n);
      }
      lower_exts.swap(dedup);
   }
   lower.enabledExtensionCount = (uint32_t)lower_exts.size();
   lower.ppEnabledExtensionNames = lower_exts.empty() ? nullptr : lower_exts.data();

   VkResult res = next_create(physicalDevice, &lower, pAllocator, pDevice);
   if (res != VK_SUCCESS) {
      helios_count(HELIOS_CNT_create_device_refused_lower_failed);
      return res;
   }

   HeliosDevice *dev = new (std::nothrow) HeliosDevice();
   if (!dev) {
      PFN_vkDestroyDevice destroy =
         (PFN_vkDestroyDevice)next_gdpa(*pDevice, "vkDestroyDevice");
      if (destroy)
         destroy(*pDevice, pAllocator);
      *pDevice = VK_NULL_HANDLE;
      return helios_refuse(HELIOS_CNT_create_device_refused_alloc,
                           VK_ERROR_OUT_OF_HOST_MEMORY, nullptr);
   }
   dev->device = *pDevice;
   dev->phys = physicalDevice;
   dev->inst = inst;
   dev->wsi_enabled = wsi;
   dev->bind_memory2_khr_enabled = want_bind_memory2;
   dev->canonical_family = info.canonical_family;
   dev->private_queue_index = private_index;
   if (cb_info)
      dev->set_device_loader_data = cb_info->u.pfnSetDeviceLoaderData;

#define HELIOS_GDPA(field, name)                                              \
   dev->disp.field = (PFN_vk##field)next_gdpa(*pDevice, name)
   dev->disp.GetDeviceProcAddr = next_gdpa;
   HELIOS_GDPA(DestroyDevice, "vkDestroyDevice");
   HELIOS_GDPA(DeviceWaitIdle, "vkDeviceWaitIdle");
   HELIOS_GDPA(GetDeviceQueue, "vkGetDeviceQueue");
   HELIOS_GDPA(GetDeviceQueue2, "vkGetDeviceQueue2");
   HELIOS_GDPA(QueueSubmit2, "vkQueueSubmit2");
   HELIOS_GDPA(QueueWaitIdle, "vkQueueWaitIdle");
   HELIOS_GDPA(CreateImage, "vkCreateImage");
   HELIOS_GDPA(DestroyImage, "vkDestroyImage");
   HELIOS_GDPA(BindImageMemory2, "vkBindImageMemory2");
   HELIOS_GDPA(GetImageMemoryRequirements2, "vkGetImageMemoryRequirements2");
   HELIOS_GDPA(AllocateMemory, "vkAllocateMemory");
   HELIOS_GDPA(FreeMemory, "vkFreeMemory");
   HELIOS_GDPA(GetMemoryWin32HandlePropertiesKHR,
               "vkGetMemoryWin32HandlePropertiesKHR");
   HELIOS_GDPA(CreateSemaphore, "vkCreateSemaphore");
   HELIOS_GDPA(DestroySemaphore, "vkDestroySemaphore");
   HELIOS_GDPA(ImportSemaphoreWin32HandleKHR, "vkImportSemaphoreWin32HandleKHR");
   HELIOS_GDPA(CreateFence, "vkCreateFence");
   HELIOS_GDPA(DestroyFence, "vkDestroyFence");
   HELIOS_GDPA(WaitForFences, "vkWaitForFences");
   HELIOS_GDPA(ResetFences, "vkResetFences");
   HELIOS_GDPA(CreateCommandPool, "vkCreateCommandPool");
   HELIOS_GDPA(DestroyCommandPool, "vkDestroyCommandPool");
   HELIOS_GDPA(ResetCommandPool, "vkResetCommandPool");
   HELIOS_GDPA(AllocateCommandBuffers, "vkAllocateCommandBuffers");
   HELIOS_GDPA(BeginCommandBuffer, "vkBeginCommandBuffer");
   HELIOS_GDPA(EndCommandBuffer, "vkEndCommandBuffer");
   HELIOS_GDPA(CmdPipelineBarrier2, "vkCmdPipelineBarrier2");
#undef HELIOS_GDPA
   dev->disp.SetHeliosPresentableImage =
      (PFN_vkSetHeliosPresentableImageHELIOS)next_gdpa(
         *pDevice, HELIOS_SET_PRESENTABLE_IMAGE_NAME);

   if (wsi) {
      /*
       * The application's own apiVersion, checked before the proc table,
       * because it is the usual reason the table has holes and the two are
       * easy to confuse. The layer's WSI path needs core 1.3 entry points
       * (vkQueueSubmit2, vkCmdPipelineBarrier2); the loader returns NULL from
       * vkGetDeviceProcAddr for core functions above VkApplicationInfo::
       * apiVersion, so a 1.1 application produces a NULL vkQueueSubmit2 on a
       * 1.4 device. Reporting that as "missing device proc" blames the driver
       * for the application's declaration.
       *
       * This is what create_device_refused_api_below_13 was named for. It had
       * never been incremented, because the check did not exist —
       * physdev_refused_api_below_13 covers the DEVICE's version, which is a
       * different question with a different answer.
       */
      if (VK_API_VERSION_MAJOR(inst->app_api_version) < 1 ||
          (VK_API_VERSION_MAJOR(inst->app_api_version) == 1 &&
           VK_API_VERSION_MINOR(inst->app_api_version) < 3)) {
         PFN_vkDestroyDevice destroy = dev->disp.DestroyDevice;
         delete dev;
         if (destroy)
            destroy(*pDevice, pAllocator);
         *pDevice = VK_NULL_HANDLE;
         return helios_refuse(HELIOS_CNT_create_device_refused_api_below_13,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "application requested an apiVersion below 1.3");
      }

      /* Every function the WSI path uses must exist before any NT object is
       * created (§10.7:2281-2282). */
      /* Paired with their names: a bare pointer array reports only THAT
       * something is missing, and the reader then has to guess which of the
       * twenty it was. Naming it is the difference between a refusal that
       * ends the investigation and one that starts it. */
      const struct { const void *fn; const char *name; } required[] = {
         { (const void *)dev->disp.QueueSubmit2, "vkQueueSubmit2" },
         { (const void *)dev->disp.CreateImage, "vkCreateImage" },
         { (const void *)dev->disp.DestroyImage, "vkDestroyImage" },
         { (const void *)dev->disp.BindImageMemory2, "vkBindImageMemory2" },
         { (const void *)dev->disp.GetImageMemoryRequirements2, "vkGetImageMemoryRequirements2" },
         { (const void *)dev->disp.AllocateMemory, "vkAllocateMemory" },
         { (const void *)dev->disp.FreeMemory, "vkFreeMemory" },
         { (const void *)dev->disp.GetMemoryWin32HandlePropertiesKHR, "vkGetMemoryWin32HandlePropertiesKHR" },
         { (const void *)dev->disp.CreateSemaphore, "vkCreateSemaphore" },
         { (const void *)dev->disp.DestroySemaphore, "vkDestroySemaphore" },
         { (const void *)dev->disp.ImportSemaphoreWin32HandleKHR, "vkImportSemaphoreWin32HandleKHR" },
         { (const void *)dev->disp.CreateCommandPool, "vkCreateCommandPool" },
         { (const void *)dev->disp.DestroyCommandPool, "vkDestroyCommandPool" },
         { (const void *)dev->disp.ResetCommandPool, "vkResetCommandPool" },
         { (const void *)dev->disp.AllocateCommandBuffers, "vkAllocateCommandBuffers" },
         { (const void *)dev->disp.BeginCommandBuffer, "vkBeginCommandBuffer" },
         { (const void *)dev->disp.EndCommandBuffer, "vkEndCommandBuffer" },
         { (const void *)dev->disp.CmdPipelineBarrier2, "vkCmdPipelineBarrier2" },
         { (const void *)dev->disp.CreateFence, "vkCreateFence" },
         { (const void *)dev->disp.DestroyFence, "vkDestroyFence" },
         { (const void *)dev->disp.WaitForFences, "vkWaitForFences" },
         { (const void *)dev->disp.ResetFences, "vkResetFences" },
      };
      for (const auto &r : required) {
         if (r.fn == nullptr) {
            PFN_vkDestroyDevice destroy = dev->disp.DestroyDevice;
            delete dev;
            if (destroy)
               destroy(*pDevice, pAllocator);
            *pDevice = VK_NULL_HANDLE;
            return helios_refuse(HELIOS_CNT_create_device_refused_missing_device_proc,
                                 VK_ERROR_INITIALIZATION_FAILED, r.name);
         }
      }

      /* Retrieve the private helper queue at the appended index and never
       * expose it (§10.7:2301-2303). */
      dev->disp.GetDeviceQueue(*pDevice, info.canonical_family, private_index,
                               &dev->helper_queue);
      if (dev->helper_queue && dev->set_device_loader_data)
         dev->set_device_loader_data(*pDevice, dev->helper_queue);
      if (!dev->helper_queue) {
         PFN_vkDestroyDevice destroy = dev->disp.DestroyDevice;
         delete dev;
         if (destroy)
            destroy(*pDevice, pAllocator);
         *pDevice = VK_NULL_HANDLE;
         return helios_refuse(HELIOS_CNT_create_device_refused_queue_capacity,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "helper queue not retrievable");
      }
   }

   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_devices[helios_key(*pDevice)] = dev;
   }
   helios_log("[helios-wsi] device %p created (wsi=%d canonical=%u private=%u)",
              (void *)*pDevice, (int)wsi, info.canonical_family, private_index);
   return VK_SUCCESS;
}

static void helios_destroy_swapchain_locked(HeliosSwapchain *sc);

static VKAPI_ATTR void VKAPI_CALL
helios_DestroyDevice(VkDevice device, const VkAllocationCallbacks *pAllocator)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev)
      return;

   /* Any swapchain still alive at device destruction is an application error;
    * the layer still drains and destroys its own objects rather than leaking
    * D3D/NT state. */
   std::vector<HeliosSwapchain *> leftovers;
   {
      std::lock_guard<std::mutex> g(dev->lock);
      leftovers.assign(dev->swapchains.begin(), dev->swapchains.end());
      dev->swapchains.clear();
   }
   for (HeliosSwapchain *sc : leftovers) {
      {
         std::lock_guard<std::mutex> g(helios_registry_lock);
         helios_swapchains.erase(sc);
      }
      helios_destroy_swapchain_locked(sc);
      delete sc;
   }

   {
      std::lock_guard<std::mutex> g(dev->d3d12_lock);
      helios_com_release(dev->d3d12);
   }

   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_devices.erase(helios_key(device));
   }
   PFN_vkDestroyDevice destroy = dev->disp.DestroyDevice;
   delete dev;
   if (destroy)
      destroy(device, pAllocator);
}

static VKAPI_ATTR void VKAPI_CALL
helios_GetDeviceQueue(VkDevice device, uint32_t queueFamilyIndex,
                      uint32_t queueIndex, VkQueue *pQueue)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->disp.GetDeviceQueue) {
      *pQueue = VK_NULL_HANDLE;
      return;
   }
   /* §10.7:2302-2304 — the private index is never exposed through either
    * app-facing getter. */
   if (dev->wsi_enabled && queueFamilyIndex == dev->canonical_family &&
       queueIndex == dev->private_queue_index) {
      *pQueue = VK_NULL_HANDLE;
      helios_count(HELIOS_CNT_getqueue_private_index_withheld);
      return;
   }
   dev->disp.GetDeviceQueue(device, queueFamilyIndex, queueIndex, pQueue);
   if (*pQueue) {
      std::lock_guard<std::mutex> g(dev->lock);
      dev->app_queues[*pQueue] = { queueFamilyIndex, queueIndex, 0 };
   }
}

static VKAPI_ATTR void VKAPI_CALL
helios_GetDeviceQueue2(VkDevice device, const VkDeviceQueueInfo2 *pQueueInfo,
                       VkQueue *pQueue)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->disp.GetDeviceQueue2) {
      *pQueue = VK_NULL_HANDLE;
      return;
   }
   if (dev->wsi_enabled && pQueueInfo->queueFamilyIndex == dev->canonical_family &&
       pQueueInfo->queueIndex == dev->private_queue_index) {
      *pQueue = VK_NULL_HANDLE;
      helios_count(HELIOS_CNT_getqueue_private_index_withheld);
      return;
   }
   dev->disp.GetDeviceQueue2(device, pQueueInfo, pQueue);
   if (*pQueue) {
      std::lock_guard<std::mutex> g(dev->lock);
      dev->app_queues[*pQueue] = { pQueueInfo->queueFamilyIndex,
                                   pQueueInfo->queueIndex, pQueueInfo->flags };
   }
}

/* ---- C45 singleton device-group Present queries -------------------- */

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetDeviceGroupPresentCapabilitiesKHR(
   VkDevice device, VkDeviceGroupPresentCapabilitiesKHR *pCaps)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->wsi_enabled)
      return VK_ERROR_INITIALIZATION_FAILED;
   memset(pCaps->presentMask, 0, sizeof(pCaps->presentMask));
   pCaps->presentMask[0] = 1;
   pCaps->modes = VK_DEVICE_GROUP_PRESENT_MODE_LOCAL_BIT_KHR;
   return VK_SUCCESS;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetDeviceGroupSurfacePresentModesKHR(
   VkDevice device, VkSurfaceKHR surface,
   VkDeviceGroupPresentModeFlagsKHR *pModes)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->wsi_enabled)
      return VK_ERROR_INITIALIZATION_FAILED;
   HeliosSurface *surf = nullptr;
   VkResult r = helios_resolve_surface(surface, &surf);
   if (r != VK_SUCCESS)
      return VK_ERROR_SURFACE_LOST_KHR;
   *pModes = VK_DEVICE_GROUP_PRESENT_MODE_LOCAL_BIT_KHR;
   return VK_SUCCESS;
}

/* ==================================================================== */
/* 9. Swapchain creation                                                */
/* ==================================================================== */

static ID3D12Device *
helios_d3d12_device(HeliosDevice *dev)
{
   std::lock_guard<std::mutex> g(dev->d3d12_lock);
   if (dev->d3d12)
      return dev->d3d12;

   const HeliosPhysDevInfo &info = helios_admit(dev->inst, dev->phys);
   if (!info.admitted) {
      helios_refuse(HELIOS_CNT_swapchain_refused_d3d12_device,
                    VK_ERROR_INITIALIZATION_FAILED, info.reject_reason);
      return nullptr;
   }
   IDXGIAdapter1 *adapter = helios_adapter_by_luid(dev->inst, info.luid);
   if (!adapter) {
      helios_refuse(HELIOS_CNT_swapchain_refused_dxgi_adapter,
                    VK_ERROR_INITIALIZATION_FAILED, "EnumAdapterByLuid");
      return nullptr;
   }
   /* §10.7:2489-2490 — the D3D12 device is created on the exact adapter and
    * runs over the selected ordinary-context translator path (UMD12/vkd3d). */
   ID3D12Device *d3d = nullptr;
   HRESULT hr = D3D12CreateDevice(adapter, D3D_FEATURE_LEVEL_11_0,
                                  IID_PPV_ARGS(&d3d));
   adapter->Release();
   if (FAILED(hr) || !d3d) {
      helios_refuse(HELIOS_CNT_swapchain_refused_d3d12_device,
                    VK_ERROR_INITIALIZATION_FAILED, "D3D12CreateDevice");
      return nullptr;
   }
   dev->d3d12 = d3d;
   return d3d;
}

/* Releases every object a slot owns. Safe on a partially built slot. */
static void
helios_slot_teardown(HeliosDevice *dev, HeliosSlot &slot)
{
   if (slot.pool != VK_NULL_HANDLE && dev->disp.DestroyCommandPool) {
      dev->disp.DestroyCommandPool(dev->device, slot.pool, nullptr);
      slot.pool = VK_NULL_HANDLE;
      slot.acquire_cb = VK_NULL_HANDLE;
      slot.release_cb = VK_NULL_HANDLE;
   }
   if (slot.ready_sem != VK_NULL_HANDLE && dev->disp.DestroySemaphore) {
      dev->disp.DestroySemaphore(dev->device, slot.ready_sem, nullptr);
      slot.ready_sem = VK_NULL_HANDLE;
   }
   if (slot.release_sem != VK_NULL_HANDLE && dev->disp.DestroySemaphore) {
      dev->disp.DestroySemaphore(dev->device, slot.release_sem, nullptr);
      slot.release_sem = VK_NULL_HANDLE;
   }
   if (slot.image != VK_NULL_HANDLE && dev->disp.DestroyImage) {
      dev->disp.DestroyImage(dev->device, slot.image, nullptr);
      slot.image = VK_NULL_HANDLE;
   }
   /* Freeing the imported VkDeviceMemory is what makes the lower ICD close its
    * C57 resource with D3DKMTDestroyAllocation2 (§10.7:2554-2556); H[i] is
    * closed afterwards, in that order. */
   if (slot.memory != VK_NULL_HANDLE && dev->disp.FreeMemory) {
      dev->disp.FreeMemory(dev->device, slot.memory, nullptr);
      slot.memory = VK_NULL_HANDLE;
   }
   helios_com_release(slot.list);
   helios_com_release(slot.alloc);
   helios_com_release(slot.ready);
   helios_com_release(slot.release);
   helios_com_release(slot.s);
   if (slot.h) {
      CloseHandle(slot.h);
      slot.h = nullptr;
   }
   if (slot.release_event) {
      CloseHandle(slot.release_event);
      slot.release_event = nullptr;
   }
}

/* Creates one D3D12 fence, exports it, imports it as a permanent timeline
 * semaphore, and closes the transient handle (§10.3:1180-1188, §10.7:2563-2570). */
static bool
helios_import_fence(HeliosDevice *dev, ID3D12Device *d3d, ID3D12Fence **out_fence,
                    VkSemaphore *out_sem)
{
   ID3D12Fence *fence = nullptr;
   if (FAILED(d3d->CreateFence(0, D3D12_FENCE_FLAG_SHARED, IID_PPV_ARGS(&fence))) ||
       !fence) {
      helios_refuse(HELIOS_CNT_swapchain_refused_fence_create,
                    VK_ERROR_INITIALIZATION_FAILED, "CreateFence");
      return false;
   }
   HANDLE h = nullptr;
   if (FAILED(d3d->CreateSharedHandle(fence, nullptr, GENERIC_ALL, nullptr, &h)) ||
       !h) {
      fence->Release();
      helios_refuse(HELIOS_CNT_swapchain_refused_fence_share,
                    VK_ERROR_INITIALIZATION_FAILED, "CreateSharedHandle(fence)");
      return false;
   }

   VkSemaphoreTypeCreateInfo type = {};
   type.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
   type.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
   type.initialValue = 0;
   VkSemaphoreCreateInfo sci = {};
   sci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
   sci.pNext = &type;

   VkSemaphore sem = VK_NULL_HANDLE;
   if (dev->disp.CreateSemaphore(dev->device, &sci, nullptr, &sem) != VK_SUCCESS) {
      CloseHandle(h);
      fence->Release();
      helios_refuse(HELIOS_CNT_swapchain_refused_semaphore_create,
                    VK_ERROR_INITIALIZATION_FAILED, nullptr);
      return false;
   }

   VkImportSemaphoreWin32HandleInfoKHR imp = {};
   imp.sType = VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_WIN32_HANDLE_INFO_KHR;
   imp.semaphore = sem;
   imp.flags = 0; /* permanent import */
   imp.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT;
   imp.handle = h;
   imp.name = nullptr;
   VkResult r = dev->disp.ImportSemaphoreWin32HandleKHR(dev->device, &imp);

   /* Win32 semaphore import does not transfer NT-handle ownership, so closing
    * the transient handle after a successful import is mandatory
    * (§10.3:1186-1188). */
   CloseHandle(h);
   if (r != VK_SUCCESS) {
      dev->disp.DestroySemaphore(dev->device, sem, nullptr);
      fence->Release();
      helios_refuse(HELIOS_CNT_swapchain_refused_semaphore_import,
                    VK_ERROR_INITIALIZATION_FAILED, nullptr);
      return false;
   }

   *out_fence = fence;
   *out_sem = sem;
   return true;
}

/* The canonical per-slot import chain (§10.3:1143-1158, §10.7:2520-2534). */
static bool
helios_import_image(HeliosDevice *dev, HeliosSwapchain *sc, HeliosSlot &slot,
                    uint32_t index)
{
   VkExternalMemoryImageCreateInfo emici = {};
   emici.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
   emici.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT;

   VkImageCreateInfo ici = {};
   ici.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
   ici.pNext = &emici;
   ici.flags = 0;
   ici.imageType = VK_IMAGE_TYPE_2D;
   ici.format = HELIOS_WSI_FORMAT;
   ici.extent.width = sc->extent.width;
   ici.extent.height = sc->extent.height;
   ici.extent.depth = 1;
   ici.mipLevels = 1;
   ici.arrayLayers = 1;
   ici.samples = VK_SAMPLE_COUNT_1_BIT;
   ici.tiling = VK_IMAGE_TILING_OPTIMAL;
   ici.usage = sc->usage;
   ici.sharingMode = sc->sharing_mode;
   ici.queueFamilyIndexCount =
      sc->sharing_mode == VK_SHARING_MODE_CONCURRENT
         ? (uint32_t)sc->queue_families.size()
         : 0;
   ici.pQueueFamilyIndices = ici.queueFamilyIndexCount ? sc->queue_families.data()
                                                       : nullptr;
   ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

   if (dev->disp.CreateImage(dev->device, &ici, nullptr, &slot.image) != VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_image_create,
                    VK_ERROR_INITIALIZATION_FAILED, nullptr);
      return false;
   }

   VkMemoryWin32HandlePropertiesKHR mprops = {};
   mprops.sType = VK_STRUCTURE_TYPE_MEMORY_WIN32_HANDLE_PROPERTIES_KHR;
   if (dev->disp.GetMemoryWin32HandlePropertiesKHR(
          dev->device, VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT, slot.h,
          &mprops) != VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_memory_properties,
                    VK_ERROR_INITIALIZATION_FAILED, nullptr);
      return false;
   }

   VkImageMemoryRequirementsInfo2 mri = {};
   mri.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_REQUIREMENTS_INFO_2;
   mri.image = slot.image;
   VkMemoryRequirements2 mreq = {};
   mreq.sType = VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2;
   dev->disp.GetImageMemoryRequirements2(dev->device, &mri, &mreq);

   const uint32_t compatible =
      mreq.memoryRequirements.memoryTypeBits & mprops.memoryTypeBits;
   if (compatible == 0) {
      helios_refuse(HELIOS_CNT_swapchain_refused_memory_type,
                    VK_ERROR_INITIALIZATION_FAILED,
                    "no memory type is compatible with the D3D12 resource handle");
      return false;
   }
   uint32_t type_index = 0;
   while (((compatible >> type_index) & 1u) == 0)
      type_index++;

   VkMemoryDedicatedAllocateInfo dedicated = {};
   dedicated.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
   dedicated.image = slot.image;
   dedicated.buffer = VK_NULL_HANDLE;

   VkImportMemoryWin32HandleInfoKHR import = {};
   import.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR;
   import.pNext = &dedicated;
   import.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT;
   import.handle = slot.h;
   import.name = nullptr;

   VkMemoryAllocateInfo mai = {};
   mai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
   mai.pNext = &import;
   /* §10.7:2530-2532: the requirement size supplies a valid nonzero field, but
    * Windows remains authoritative because the D3D12_RESOURCE_BIT contract
    * ignores allocationSize. */
   mai.allocationSize = mreq.memoryRequirements.size;
   mai.memoryTypeIndex = type_index;

   if (dev->disp.AllocateMemory(dev->device, &mai, nullptr, &slot.memory) !=
       VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_memory_import,
                    VK_ERROR_INITIALIZATION_FAILED, nullptr);
      return false;
   }

   VkBindImageMemoryInfo bind = {};
   bind.sType = VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_INFO;
   bind.image = slot.image;
   bind.memory = slot.memory;
   bind.memoryOffset = 0;
   if (dev->disp.BindImageMemory2(dev->device, 1, &bind) != VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_memory_bind,
                    VK_ERROR_INITIALIZATION_FAILED, nullptr);
      return false;
   }

   /* The presentable-image tag, before the image can be exposed
    * (§10.7:2581-2588). Its absence already refused swapchain creation. */
   if (dev->disp.SetHeliosPresentableImage(dev->device, slot.image, sc->id, index) !=
       VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_tag_call_absent,
                    VK_ERROR_INITIALIZATION_FAILED, "tag call rejected the image");
      return false;
   }
   return true;
}

static bool
helios_create_slot_recording_objects(HeliosDevice *dev, ID3D12Device *d3d,
                                     HeliosSlot &slot)
{
   if (FAILED(d3d->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                          IID_PPV_ARGS(&slot.alloc)))) {
      helios_refuse(HELIOS_CNT_swapchain_refused_command_objects,
                    VK_ERROR_INITIALIZATION_FAILED, "CreateCommandAllocator");
      return false;
   }
   if (FAILED(d3d->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, slot.alloc,
                                     nullptr, IID_PPV_ARGS(&slot.list)))) {
      helios_refuse(HELIOS_CNT_swapchain_refused_command_objects,
                    VK_ERROR_INITIALIZATION_FAILED, "CreateCommandList");
      return false;
   }
   /* A fresh list is open; close it so Acquire's uniform Reset is always legal. */
   slot.list->Close();

   slot.release_event = CreateEventA(nullptr, FALSE, FALSE, nullptr);
   if (!slot.release_event) {
      helios_refuse(HELIOS_CNT_swapchain_refused_command_objects,
                    VK_ERROR_INITIALIZATION_FAILED, "CreateEvent");
      return false;
   }

   VkCommandPoolCreateInfo cpci = {};
   cpci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
   cpci.flags = 0; /* reset as a whole pool, never per buffer */
   cpci.queueFamilyIndex = dev->canonical_family;
   if (dev->disp.CreateCommandPool(dev->device, &cpci, nullptr, &slot.pool) !=
       VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_vk_command_objects,
                    VK_ERROR_INITIALIZATION_FAILED, "vkCreateCommandPool");
      return false;
   }
   VkCommandBufferAllocateInfo cbai = {};
   cbai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
   cbai.commandPool = slot.pool;
   cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
   cbai.commandBufferCount = 2;
   VkCommandBuffer cbs[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
   if (dev->disp.AllocateCommandBuffers(dev->device, &cbai, cbs) != VK_SUCCESS) {
      helios_refuse(HELIOS_CNT_swapchain_refused_vk_command_objects,
                    VK_ERROR_INITIALIZATION_FAILED, "vkAllocateCommandBuffers");
      return false;
   }
   if (dev->set_device_loader_data) {
      dev->set_device_loader_data(dev->device, cbs[0]);
      dev->set_device_loader_data(dev->device, cbs[1]);
   }
   slot.acquire_cb = cbs[0];
   slot.release_cb = cbs[1];
   return true;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_CreateSwapchainKHR(VkDevice device,
                          const VkSwapchainCreateInfoKHR *pCreateInfo,
                          const VkAllocationCallbacks *pAllocator,
                          VkSwapchainKHR *pSwapchain)
{
   (void)pAllocator;
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->wsi_enabled)
      return helios_refuse(HELIOS_CNT_swapchain_refused_unknown_surface,
                           VK_ERROR_INITIALIZATION_FAILED, "device has no layer WSI");

   HeliosSurface *surf = nullptr;
   VkResult sr = helios_resolve_surface(pCreateInfo->surface, &surf);
   if (sr != VK_SUCCESS)
      return sr == VK_ERROR_SURFACE_LOST_KHR
                ? helios_refuse(HELIOS_CNT_swapchain_refused_surface_lost,
                                VK_ERROR_SURFACE_LOST_KHR, nullptr)
                : helios_refuse(HELIOS_CNT_swapchain_refused_unknown_surface,
                                VK_ERROR_SURFACE_LOST_KHR, nullptr);

   /* ---- the copy-only profile, every row an admission condition ---- */
   if (pCreateInfo->flags != 0)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_flags,
                           VK_ERROR_INITIALIZATION_FAILED, "flags must be zero");
   if (pCreateInfo->imageFormat != HELIOS_WSI_FORMAT)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_format,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->imageColorSpace != HELIOS_WSI_COLOR_SPACE)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_colorspace,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->presentMode != HELIOS_WSI_PRESENT_MODE)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_present_mode,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->imageArrayLayers != HELIOS_WSI_MAX_ARRAY_LAYERS)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_array_layers,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->minImageCount < HELIOS_WSI_MIN_IMAGE_COUNT ||
       pCreateInfo->minImageCount > HELIOS_WSI_MAX_IMAGE_COUNT)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_image_count,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->preTransform != VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_transform,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->compositeAlpha != VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_alpha,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   if (pCreateInfo->imageUsage == 0 ||
       (pCreateInfo->imageUsage & ~(VkImageUsageFlags)HELIOS_WSI_SUPPORTED_USAGE))
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_usage,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);

   /* Only the C45 singleton device-group structure is admitted in pNext. */
   for (const VkBaseInStructure *s = (const VkBaseInStructure *)pCreateInfo->pNext;
        s; s = s->pNext) {
      if (s->sType == VK_STRUCTURE_TYPE_DEVICE_GROUP_SWAPCHAIN_CREATE_INFO_KHR) {
         const VkDeviceGroupSwapchainCreateInfoKHR *dg =
            (const VkDeviceGroupSwapchainCreateInfoKHR *)s;
         if (dg->modes != VK_DEVICE_GROUP_PRESENT_MODE_LOCAL_BIT_KHR)
            return helios_refuse(HELIOS_CNT_swapchain_refused_device_group_mode,
                                 VK_ERROR_INITIALIZATION_FAILED,
                                 "modes must be exactly LOCAL");
         continue;
      }
      return helios_refuse(HELIOS_CNT_swapchain_refused_unsupported_pnext,
                           VK_ERROR_INITIALIZATION_FAILED,
                           "unadvertised swapchain create structure");
   }

   /* Sharing mode (§10.7:2591-2594). */
   std::vector<uint32_t> families;
   if (pCreateInfo->imageSharingMode == VK_SHARING_MODE_CONCURRENT) {
      bool has_canonical = false;
      for (uint32_t i = 0; i < pCreateInfo->queueFamilyIndexCount; i++) {
         families.push_back(pCreateInfo->pQueueFamilyIndices[i]);
         if (pCreateInfo->pQueueFamilyIndices[i] == dev->canonical_family)
            has_canonical = true;
      }
      if (!has_canonical)
         return helios_refuse(HELIOS_CNT_swapchain_refused_profile_sharing,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "concurrent list omits the canonical family");
   } else if (pCreateInfo->imageSharingMode != VK_SHARING_MODE_EXCLUSIVE) {
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_sharing,
                           VK_ERROR_INITIALIZATION_FAILED, "unknown sharing mode");
   }

   /* Extent must equal the current nonzero client extent (§10.7:2458, 2465). */
   VkExtent2D client = {};
   if (!helios_client_extent(surf->hwnd, &client))
      return helios_refuse(HELIOS_CNT_swapchain_refused_surface_lost,
                           VK_ERROR_SURFACE_LOST_KHR, nullptr);
   if (client.width == 0 || client.height == 0)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_zero_extent,
                           VK_ERROR_INITIALIZATION_FAILED, "client extent is (0,0)");
   if (pCreateInfo->imageExtent.width != client.width ||
       pCreateInfo->imageExtent.height != client.height)
      return helios_refuse(HELIOS_CNT_swapchain_refused_profile_extent,
                           VK_ERROR_INITIALIZATION_FAILED,
                           "imageExtent differs from the client extent");

   /* oldSwapchain must be one of ours on this device and surface. */
   HeliosSwapchain *old_sc = nullptr;
   if (pCreateInfo->oldSwapchain != VK_NULL_HANDLE) {
      old_sc = helios_swapchain_of(pCreateInfo->oldSwapchain);
      if (!old_sc || old_sc->dev != dev || old_sc->surf != surf)
         return helios_refuse(HELIOS_CNT_swapchain_refused_old_swapchain,
                              VK_ERROR_INITIALIZATION_FAILED, nullptr);
   }

   if (!dev->disp.SetHeliosPresentableImage)
      return helios_refuse(HELIOS_CNT_swapchain_refused_tag_call_absent,
                           VK_ERROR_INITIALIZATION_FAILED,
                           HELIOS_SET_PRESENTABLE_IMAGE_NAME
                           " is not provided by the lower chain");

   ID3D12Device *d3d = helios_d3d12_device(dev);
   if (!d3d)
      return VK_ERROR_INITIALIZATION_FAILED;

   HeliosSwapchain *sc = new (std::nothrow) HeliosSwapchain();
   if (!sc)
      return helios_refuse(HELIOS_CNT_swapchain_refused_alloc,
                           VK_ERROR_OUT_OF_HOST_MEMORY, nullptr);
   sc->id = helios_next_swapchain_id.fetch_add(1, std::memory_order_relaxed);
   sc->dev = dev;
   sc->surf = surf;
   sc->hwnd = surf->hwnd;
   sc->extent = client;
   sc->image_count = pCreateInfo->minImageCount;
   sc->usage = pCreateInfo->imageUsage;
   sc->sharing_mode = pCreateInfo->imageSharingMode;
   sc->queue_families = families;
   sc->state_event = CreateEventA(nullptr, TRUE, FALSE, nullptr);

   /* ---- D3D12 queue + DXGI flip-model swapchain --------------------- */
   D3D12_COMMAND_QUEUE_DESC qdesc = {};
   qdesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
   qdesc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
   qdesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
   qdesc.NodeMask = 0;
   if (FAILED(d3d->CreateCommandQueue(&qdesc, IID_PPV_ARGS(&sc->queue)))) {
      helios_destroy_swapchain_locked(sc);
      delete sc;
      return helios_refuse(HELIOS_CNT_swapchain_refused_dxgi_queue,
                           VK_ERROR_INITIALIZATION_FAILED, "CreateCommandQueue");
   }

   IDXGIFactory4 *factory = helios_dxgi_factory(dev->inst);
   if (!factory) {
      helios_destroy_swapchain_locked(sc);
      delete sc;
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   DXGI_SWAP_CHAIN_DESC1 scd = {};
   scd.Width = sc->extent.width;
   scd.Height = sc->extent.height;
   scd.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
   scd.Stereo = FALSE;
   scd.SampleDesc.Count = 1;
   scd.SampleDesc.Quality = 0;
   scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
   scd.BufferCount = sc->image_count;
   scd.Scaling = DXGI_SCALING_NONE;
   scd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
   scd.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
   scd.Flags = 0; /* no tearing flag (§10.7:2461) */

   IDXGISwapChain1 *sc1 = nullptr;
   HRESULT hr = factory->CreateSwapChainForHwnd(sc->queue, sc->hwnd, &scd, nullptr,
                                                nullptr, &sc1);
   if (FAILED(hr) || !sc1) {
      helios_destroy_swapchain_locked(sc);
      delete sc;
      return helios_refuse(HELIOS_CNT_swapchain_refused_dxgi_swapchain,
                           VK_ERROR_INITIALIZATION_FAILED, "CreateSwapChainForHwnd");
   }
   hr = sc1->QueryInterface(IID_PPV_ARGS(&sc->dxgi));
   sc1->Release();
   if (FAILED(hr) || !sc->dxgi) {
      helios_destroy_swapchain_locked(sc);
      delete sc;
      return helios_refuse(HELIOS_CNT_swapchain_refused_dxgi_swapchain,
                           VK_ERROR_INITIALIZATION_FAILED, "IDXGISwapChain3");
   }

   /* C38 colour space (§10.7:2457). */
   UINT cs_support = 0;
   if (FAILED(sc->dxgi->CheckColorSpaceSupport(
          DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709, &cs_support)) ||
       !(cs_support & DXGI_SWAP_CHAIN_COLOR_SPACE_SUPPORT_FLAG_PRESENT) ||
       FAILED(sc->dxgi->SetColorSpace1(DXGI_COLOR_SPACE_RGB_FULL_G22_NONE_P709))) {
      helios_destroy_swapchain_locked(sc);
      delete sc;
      return helios_refuse(HELIOS_CNT_swapchain_refused_colorspace_support,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   }

   /* Hold one reference per real backbuffer; released before DXGI teardown. */
   sc->backbuffers.resize(sc->image_count, nullptr);
   for (uint32_t j = 0; j < sc->image_count; j++) {
      if (FAILED(sc->dxgi->GetBuffer(j, IID_PPV_ARGS(&sc->backbuffers[j])))) {
         helios_destroy_swapchain_locked(sc);
         delete sc;
         return helios_refuse(HELIOS_CNT_swapchain_refused_dxgi_backbuffer,
                              VK_ERROR_INITIALIZATION_FAILED, "GetBuffer");
      }
   }

   /* ---- per-slot S[i], H[i], fences, imports, recording objects ----- */
   /* ⚠ NOT `slots.resize(n)`. A HeliosSlot owns the `std::mutex` that covers
    * its recording objects (A13), so it is neither copyable nor movable, and
    * `resize` requires MoveInsertable. The sized constructor requires only
    * DefaultInsertable and value-initialises the elements in place, which is
    * what a fixed-size slot array wants anyway — the count is decided once,
    * here, and never changes for the life of the swapchain. */
   sc->slots = std::vector<HeliosSlot>(sc->image_count);
   D3D12_HEAP_PROPERTIES heap = {};
   heap.Type = D3D12_HEAP_TYPE_DEFAULT;
   heap.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
   heap.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
   heap.CreationNodeMask = 1;
   heap.VisibleNodeMask = 1;

   D3D12_RESOURCE_DESC rd = {};
   rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
   rd.Alignment = 0;
   rd.Width = sc->extent.width;
   rd.Height = sc->extent.height;
   rd.DepthOrArraySize = 1;
   rd.MipLevels = 1;
   rd.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
   rd.SampleDesc.Count = 1;
   rd.SampleDesc.Quality = 0;
   rd.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
   rd.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

   for (uint32_t i = 0; i < sc->image_count; i++) {
      HeliosSlot &slot = sc->slots[i];

      if (FAILED(d3d->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_SHARED, &rd,
                                              D3D12_RESOURCE_STATE_COMMON, nullptr,
                                              IID_PPV_ARGS(&slot.s)))) {
         helios_destroy_swapchain_locked(sc);
         delete sc;
         return helios_refuse(HELIOS_CNT_swapchain_refused_shared_resource,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "CreateCommittedResource");
      }
      /* Exactly one retained, unnamed, non-inheritable handle per S[i]. */
      if (FAILED(d3d->CreateSharedHandle(slot.s, nullptr, GENERIC_ALL, nullptr,
                                         &slot.h)) ||
          !slot.h) {
         helios_destroy_swapchain_locked(sc);
         delete sc;
         return helios_refuse(HELIOS_CNT_swapchain_refused_shared_handle,
                              VK_ERROR_INITIALIZATION_FAILED, "CreateSharedHandle");
      }
      if (!helios_import_fence(dev, d3d, &slot.ready, &slot.ready_sem) ||
          !helios_import_fence(dev, d3d, &slot.release, &slot.release_sem) ||
          !helios_create_slot_recording_objects(dev, d3d, slot) ||
          !helios_import_image(dev, sc, slot, i)) {
         helios_destroy_swapchain_locked(sc);
         delete sc;
         return VK_ERROR_INITIALIZATION_FAILED;
      }
      slot.state = HELIOS_SLOT_NEVER_USED;
   }

   /* §10.7:2478-2480 — the new set exists before the old acquisition/present
    * state is atomically retired. */
   if (old_sc) {
      std::lock_guard<std::mutex> g(old_sc->lock);
      old_sc->retired = true;
      if (old_sc->state_event)
         SetEvent(old_sc->state_event);
   }

   {
      std::lock_guard<std::mutex> g(dev->lock);
      dev->swapchains.insert(sc);
   }
   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_swapchains.insert(sc);
   }
   *pSwapchain = helios_swapchain_handle(sc);
   helios_log("[helios-wsi] swapchain %llu created %ux%u N=%u",
              (unsigned long long)sc->id, sc->extent.width, sc->extent.height,
              sc->image_count);
   return VK_SUCCESS;
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_GetSwapchainImagesKHR(VkDevice device, VkSwapchainKHR swapchain,
                             uint32_t *pCount, VkImage *pImages)
{
   (void)device;
   HeliosSwapchain *sc = helios_swapchain_of(swapchain);
   if (!sc)
      return helios_refuse(HELIOS_CNT_acquire_refused_unknown_swapchain,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   std::lock_guard<std::mutex> g(sc->lock);
   std::vector<VkImage> images;
   images.reserve(sc->slots.size());
   for (const HeliosSlot &s : sc->slots)
      images.push_back(s.image);
   return helios_write_array(images.data(), (uint32_t)images.size(), pImages, pCount);
}

/* ==================================================================== */
/* 10. Acquire                                                          */
/* ==================================================================== */

/* Records one image barrier into `cb`. The layer only ever needs single-image
 * ownership/layout transitions, so this covers acquire, release and the
 * teardown reciprocal. */
static VkResult
helios_record_barrier(HeliosDevice *dev, VkCommandBuffer cb, VkImage image,
                      VkImageLayout old_layout, VkImageLayout new_layout,
                      uint32_t src_family, uint32_t dst_family,
                      VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access,
                      VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access)
{
   VkCommandBufferBeginInfo begin = {};
   begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
   begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
   VkResult r = dev->disp.BeginCommandBuffer(cb, &begin);
   if (r != VK_SUCCESS)
      return r;

   VkImageMemoryBarrier2 b = {};
   b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
   b.srcStageMask = src_stage;
   b.srcAccessMask = src_access;
   b.dstStageMask = dst_stage;
   b.dstAccessMask = dst_access;
   b.oldLayout = old_layout;
   b.newLayout = new_layout;
   b.srcQueueFamilyIndex = src_family;
   b.dstQueueFamilyIndex = dst_family;
   b.image = image;
   b.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
   b.subresourceRange.baseMipLevel = 0;
   b.subresourceRange.levelCount = 1;
   b.subresourceRange.baseArrayLayer = 0;
   b.subresourceRange.layerCount = 1;

   VkDependencyInfo dep = {};
   dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
   dep.imageMemoryBarrierCount = 1;
   dep.pImageMemoryBarriers = &b;
   dev->disp.CmdPipelineBarrier2(cb, &dep);

   return dev->disp.EndCommandBuffer(cb);
}

/* Refreshes RELEASE_QUEUED slots whose Release[i]=e has actually completed.
 * O(1) per slot; never a table scan outside this swapchain (§10.7:2611-2614). */
static void
helios_refresh_slots(HeliosSwapchain *sc)
{
   for (HeliosSlot &slot : sc->slots) {
      if (slot.state != HELIOS_SLOT_RELEASE_QUEUED || !slot.release)
         continue;
      if (slot.release->GetCompletedValue() >= slot.epoch) {
         slot.retired_epoch = slot.epoch;
         slot.state = HELIOS_SLOT_AVAILABLE;
      }
   }
}

/* Swapchain status shared by Acquire and Present. */
static VkResult
helios_swapchain_status(HeliosSwapchain *sc)
{
   if (sc->lost)
      return VK_ERROR_DEVICE_LOST;
   if (sc->destroying)
      return VK_ERROR_OUT_OF_DATE_KHR;
   if (!helios_window_alive(sc->hwnd))
      return VK_ERROR_SURFACE_LOST_KHR;
   if (sc->retired)
      return VK_ERROR_OUT_OF_DATE_KHR;
   VkExtent2D now = {};
   if (!helios_client_extent(sc->hwnd, &now))
      return VK_ERROR_SURFACE_LOST_KHR;
   /* §10.7:2466-2469 — a changed dimension makes the swapchain incompatible;
    * this generation never reports VK_SUBOPTIMAL_KHR to keep copying. */
   if (now.width != sc->extent.width || now.height != sc->extent.height)
      return VK_ERROR_OUT_OF_DATE_KHR;
   return VK_SUCCESS;
}

static VkResult
helios_acquire(HeliosDevice *dev, HeliosSwapchain *sc, uint64_t timeout,
               VkSemaphore semaphore, VkFence fence, uint32_t device_mask,
               uint32_t *pImageIndex)
{
   const ULONGLONG start_ms = GetTickCount64();

   for (;;) {
      uint32_t chosen = UINT32_MAX;
      uint64_t prev_epoch = 0;
      uint64_t wait_epoch = 0;
      uint32_t wait_slot = UINT32_MAX;

      {
         std::lock_guard<std::mutex> g(sc->lock);
         VkResult st = helios_swapchain_status(sc);
         if (st != VK_SUCCESS) {
            helios_count(st == VK_ERROR_DEVICE_LOST ? HELIOS_CNT_acquire_refused_lost
                         : st == VK_ERROR_SURFACE_LOST_KHR
                            ? HELIOS_CNT_acquire_refused_surface_lost
                            : HELIOS_CNT_acquire_refused_out_of_date);
            return st;
         }

         helios_refresh_slots(sc);

         const uint32_t n = (uint32_t)sc->slots.size();
         for (uint32_t k = 0; k < n; k++) {
            uint32_t i = (sc->next_hint + k) % n;
            HeliosSlot &slot = sc->slots[i];
            if (slot.state == HELIOS_SLOT_NEVER_USED ||
                slot.state == HELIOS_SLOT_AVAILABLE) {
               chosen = i;
               break;
            }
         }

         if (chosen == UINT32_MAX) {
            if (timeout == 0) {
               return helios_refuse(HELIOS_CNT_acquire_not_ready, VK_NOT_READY,
                                    nullptr);
            }
            /* One D3D queue per swapchain, so a later Release cannot complete
             * first: the oldest enqueued RELEASE_QUEUED slot is the next one
             * to become selectable (§10.7:2635-2638). */
            uint64_t oldest = UINT64_MAX;
            for (uint32_t i = 0; i < n; i++) {
               HeliosSlot &slot = sc->slots[i];
               if (slot.state == HELIOS_SLOT_RELEASE_QUEUED && slot.epoch < oldest) {
                  oldest = slot.epoch;
                  wait_slot = i;
                  wait_epoch = slot.epoch;
               }
            }
         } else {
            HeliosSlot &slot = sc->slots[chosen];
            prev_epoch = slot.retired_epoch;
            if (slot.epoch == UINT64_MAX - 1)
               return helios_refuse(HELIOS_CNT_acquire_refused_epoch_overflow,
                                    VK_ERROR_OUT_OF_DATE_KHR,
                                    "epoch would wrap; recreate the swapchain");
            slot.epoch += 1;
            slot.state = HELIOS_SLOT_ACQUIRED;
            slot.device_mask = device_mask;
            sc->next_hint = (chosen + 1) % n;
         }
      }

      if (chosen == UINT32_MAX) {
         /* No slot is selectable and no Release is pending: nothing can make
          * progress except Present, destruction or loss. Wait on the bounded
          * state-change event only; never poll, never sleep in a loop. */
         HANDLE waits[2];
         DWORD nwaits = 0;
         if (wait_slot != UINT32_MAX) {
            HeliosSlot &slot = sc->slots[wait_slot];
            if (FAILED(slot.release->SetEventOnCompletion(wait_epoch,
                                                          slot.release_event)))
               return helios_refuse(HELIOS_CNT_acquire_refused_lost,
                                    VK_ERROR_DEVICE_LOST, "SetEventOnCompletion");
            waits[nwaits++] = slot.release_event;
         }
         if (sc->state_event)
            waits[nwaits++] = sc->state_event;
         if (nwaits == 0)
            return helios_refuse(HELIOS_CNT_acquire_timeout, VK_TIMEOUT, nullptr);

         DWORD ms = INFINITE;
         if (timeout != UINT64_MAX) {
            const uint64_t elapsed_ms = GetTickCount64() - start_ms;
            const uint64_t total_ms = timeout / 1000000ull;
            ms = (elapsed_ms >= total_ms) ? 0
                                          : (DWORD)(total_ms - elapsed_ms);
         }
         DWORD w = WaitForMultipleObjects(nwaits, waits, FALSE, ms);
         if (w == WAIT_TIMEOUT)
            return helios_refuse(HELIOS_CNT_acquire_timeout, VK_TIMEOUT, nullptr);
         if (w == WAIT_FAILED)
            return helios_refuse(HELIOS_CNT_acquire_refused_lost,
                                 VK_ERROR_DEVICE_LOST, "WaitForMultipleObjects");
         /* Revalidate state and the exact value once, then retry selection. */
         continue;
      }

      /* ---- the selected slot ---- */
      HeliosSlot &slot = sc->slots[chosen];
      VkResult r;
      {
         std::lock_guard<std::mutex> sg(slot.lock);

         /* Reset the slot's recording objects. Legal now, and only now,
          * because the preceding Release completed (§10.7:2617-2619). */
         if (slot.alloc && slot.list) {
            slot.alloc->Reset();
            slot.list->Reset(slot.alloc, nullptr);
            slot.list->Close();
         }
         if (dev->disp.ResetCommandPool(dev->device, slot.pool, 0) != VK_SUCCESS)
            return helios_refuse(HELIOS_CNT_acquire_refused_submit_failed,
                                 VK_ERROR_DEVICE_LOST, "vkResetCommandPool");

         const bool first_use = !slot.externally_owned;
         r = helios_record_barrier(
            dev, slot.acquire_cb, slot.image,
            first_use ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL,
            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            first_use ? VK_QUEUE_FAMILY_IGNORED : VK_QUEUE_FAMILY_EXTERNAL,
            first_use ? VK_QUEUE_FAMILY_IGNORED : dev->canonical_family,
            VK_PIPELINE_STAGE_2_NONE, 0, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);
      }
      if (r != VK_SUCCESS)
         return helios_refuse(HELIOS_CNT_acquire_refused_submit_failed,
                              VK_ERROR_DEVICE_LOST, "record acquire barrier");

      VkSemaphoreSubmitInfo wait_sem = {};
      wait_sem.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
      wait_sem.semaphore = slot.release_sem;
      wait_sem.value = prev_epoch;
      wait_sem.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

      VkSemaphoreSubmitInfo signal_sem = {};
      signal_sem.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
      signal_sem.semaphore = semaphore;
      signal_sem.value = 0; /* app object is binary */
      signal_sem.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

      VkCommandBufferSubmitInfo cbi = {};
      cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
      cbi.commandBuffer = slot.acquire_cb;

      VkSubmitInfo2 si = {};
      si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
      si.waitSemaphoreInfoCount = prev_epoch > 0 ? 1u : 0u;
      si.pWaitSemaphoreInfos = prev_epoch > 0 ? &wait_sem : nullptr;
      si.commandBufferInfoCount = 1;
      si.pCommandBufferInfos = &cbi;
      si.signalSemaphoreInfoCount = semaphore != VK_NULL_HANDLE ? 1u : 0u;
      si.pSignalSemaphoreInfos = semaphore != VK_NULL_HANDLE ? &signal_sem : nullptr;

      {
         std::lock_guard<std::mutex> hg(dev->helper_lock);
         r = dev->disp.QueueSubmit2(dev->helper_queue, 1, &si, fence);
      }
      if (r != VK_SUCCESS) {
         std::lock_guard<std::mutex> g(sc->lock);
         slot.state = HELIOS_SLOT_LOST;
         sc->lost = true;
         if (sc->state_event)
            SetEvent(sc->state_event);
         return helios_refuse(HELIOS_CNT_acquire_refused_submit_failed,
                              VK_ERROR_DEVICE_LOST, "helper queue submit");
      }

      *pImageIndex = chosen;
      return VK_SUCCESS;
   }
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_AcquireNextImageKHR(VkDevice device, VkSwapchainKHR swapchain,
                           uint64_t timeout, VkSemaphore semaphore, VkFence fence,
                           uint32_t *pImageIndex)
{
   HeliosDevice *dev = helios_device_of(device);
   HeliosSwapchain *sc = helios_swapchain_of(swapchain);
   if (!dev || !sc || sc->dev != dev)
      return helios_refuse(HELIOS_CNT_acquire_refused_unknown_swapchain,
                           VK_ERROR_OUT_OF_DATE_KHR, nullptr);
   return helios_acquire(dev, sc, timeout, semaphore, fence, 1u, pImageIndex);
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_AcquireNextImage2KHR(VkDevice device,
                            const VkAcquireNextImageInfoKHR *pAcquireInfo,
                            uint32_t *pImageIndex)
{
   HeliosDevice *dev = helios_device_of(device);
   HeliosSwapchain *sc = helios_swapchain_of(pAcquireInfo->swapchain);
   if (!dev || !sc || sc->dev != dev)
      return helios_refuse(HELIOS_CNT_acquire_refused_unknown_swapchain,
                           VK_ERROR_OUT_OF_DATE_KHR, nullptr);
   /* §10.7:2629-2632 — only deviceMask 1 is accepted, and it is recorded on
    * the acquired slot so an explicit device-group Present can be checked
    * before its waits are consumed. */
   if (pAcquireInfo->deviceMask != 1u)
      return helios_refuse(HELIOS_CNT_acquire_refused_device_mask,
                           VK_ERROR_INITIALIZATION_FAILED, "deviceMask must be 1");
   if (pAcquireInfo->pNext != nullptr)
      return helios_refuse(HELIOS_CNT_acquire_refused_device_mask,
                           VK_ERROR_INITIALIZATION_FAILED,
                           "unadvertised acquire pNext");
   return helios_acquire(dev, sc, pAcquireInfo->timeout, pAcquireInfo->semaphore,
                         pAcquireInfo->fence, pAcquireInfo->deviceMask, pImageIndex);
}

/* ==================================================================== */
/* 11. Present                                                          */
/* ==================================================================== */

static VkResult
helios_map_present_hresult(HRESULT hr)
{
   if (hr == S_OK)
      return VK_SUCCESS;
   if (hr == DXGI_ERROR_DEVICE_REMOVED || hr == DXGI_ERROR_DEVICE_RESET ||
       hr == DXGI_ERROR_DEVICE_HUNG || hr == DXGI_ERROR_DRIVER_INTERNAL_ERROR)
      return VK_ERROR_DEVICE_LOST;
   /* The selected flip-model swapchain is not expected to return
    * DXGI_STATUS_OCCLUDED; if the runtime does so contrary to the flip-model
    * status contract, retire the swapchain as out of date rather than invent
    * an occlusion protocol (§10.7:2470-2474). */
   if (hr == DXGI_STATUS_OCCLUDED || hr == DXGI_STATUS_MODE_CHANGED ||
       hr == DXGI_STATUS_MODE_CHANGE_IN_PROGRESS)
      return VK_ERROR_OUT_OF_DATE_KHR;
   if (SUCCEEDED(hr))
      return VK_SUCCESS;
   return VK_ERROR_DEVICE_LOST;
}

static D3D12_RESOURCE_BARRIER
helios_transition(ID3D12Resource *res, D3D12_RESOURCE_STATES from,
                  D3D12_RESOURCE_STATES to)
{
   D3D12_RESOURCE_BARRIER b = {};
   b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
   b.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
   b.Transition.pResource = res;
   b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
   b.Transition.StateBefore = from;
   b.Transition.StateAfter = to;
   return b;
}

struct HeliosPresentItem {
   HeliosSwapchain *sc;
   uint32_t index;
   uint64_t epoch;
};

static VKAPI_ATTR VkResult VKAPI_CALL
helios_QueuePresentKHR(VkQueue queue, const VkPresentInfoKHR *pPresentInfo)
{
   HeliosDevice *dev = helios_device_of_queue(queue);
   if (!dev || !dev->wsi_enabled)
      return helios_refuse(HELIOS_CNT_present_refused_queue_not_app_visible,
                           VK_ERROR_INITIALIZATION_FAILED, "no layer device");

   /* §10.7:2648-2652 — the queue must be one of the application-visible
    * flags=0 handles created from the original canonical-family record. The
    * private helper, a protected queue, a different-family queue, a foreign
    * queue or an unrequested index fails without consuming a semaphore. */
   {
      std::lock_guard<std::mutex> g(dev->lock);
      auto it = dev->app_queues.find(queue);
      if (it == dev->app_queues.end() || it->second.flags != 0 ||
          it->second.family != dev->canonical_family ||
          queue == dev->helper_queue)
         return helios_refuse(HELIOS_CNT_present_refused_queue_not_app_visible,
                              VK_ERROR_INITIALIZATION_FAILED, nullptr);
   }

   /* Only the C45 singleton device-group structure is admitted in pNext. */
   const VkDeviceGroupPresentInfoKHR *dg = nullptr;
   for (const VkBaseInStructure *s = (const VkBaseInStructure *)pPresentInfo->pNext;
        s; s = s->pNext) {
      if (s->sType == VK_STRUCTURE_TYPE_DEVICE_GROUP_PRESENT_INFO_KHR) {
         dg = (const VkDeviceGroupPresentInfoKHR *)s;
         continue;
      }
      return helios_refuse(HELIOS_CNT_present_refused_unsupported_pnext,
                           VK_ERROR_INITIALIZATION_FAILED, nullptr);
   }
   if (dg) {
      if (dg->mode != VK_DEVICE_GROUP_PRESENT_MODE_LOCAL_BIT_KHR)
         return helios_refuse(HELIOS_CNT_present_refused_device_group_mode,
                              VK_ERROR_INITIALIZATION_FAILED, nullptr);
      if (dg->swapchainCount != 0 &&
          dg->swapchainCount != pPresentInfo->swapchainCount)
         return helios_refuse(HELIOS_CNT_present_refused_device_group_count,
                              VK_ERROR_INITIALIZATION_FAILED, nullptr);
      for (uint32_t k = 0; k < dg->swapchainCount; k++)
         if (dg->pDeviceMasks[k] != 1u)
            return helios_refuse(HELIOS_CNT_present_refused_device_group_mask,
                                 VK_ERROR_INITIALIZATION_FAILED, nullptr);
   }

   /*
    * STUB: the layer cannot prove that every pWaitSemaphores entry is binary.
    * §10.7:2653 requires that validation, but §10.7:2257-2262 fixes the device
    * dispatch closure byte-for-byte and vkCreateSemaphore is not in it, so the
    * layer never sees semaphore creation and has no non-invasive way to
    * classify a VkSemaphore. Vulkan's own VUIDs (vkQueuePresentKHR
    * pWaitSemaphores-03267/03268) already require binary semaphores here. The
    * unverified check is counted on every Present rather than being claimed.
    */
   if (pPresentInfo->waitSemaphoreCount)
      helios_count(HELIOS_CNT_present_semaphore_kind_unverified);

   /* ---- validate every swapchain/image before consuming any wait ---- */
   std::vector<HeliosPresentItem> items;
   items.reserve(pPresentInfo->swapchainCount);
   for (uint32_t k = 0; k < pPresentInfo->swapchainCount; k++) {
      HeliosSwapchain *sc = helios_swapchain_of(pPresentInfo->pSwapchains[k]);
      if (!sc || sc->dev != dev)
         return helios_refuse(HELIOS_CNT_present_refused_unknown_swapchain,
                              VK_ERROR_INITIALIZATION_FAILED, nullptr);
      const uint32_t i = pPresentInfo->pImageIndices[k];
      std::lock_guard<std::mutex> g(sc->lock);
      if (i >= sc->slots.size() || sc->slots[i].state != HELIOS_SLOT_ACQUIRED)
         return helios_refuse(HELIOS_CNT_present_refused_image_not_acquired,
                              VK_ERROR_INITIALIZATION_FAILED, nullptr);
      if (dg && dg->swapchainCount && sc->slots[i].device_mask != dg->pDeviceMasks[k])
         return helios_refuse(HELIOS_CNT_present_refused_device_group_mask,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "mask differs from the last Acquire");
      items.push_back({ sc, i, sc->slots[i].epoch });
   }

   /* Per-swapchain status is reported, not fatal: an out-of-date swapchain
    * still must not have its waits consumed twice, so status is resolved
    * before the single common submission. */
   std::vector<VkResult> results(pPresentInfo->swapchainCount, VK_SUCCESS);
   bool any_presentable = false;
   for (uint32_t k = 0; k < items.size(); k++) {
      std::lock_guard<std::mutex> g(items[k].sc->lock);
      VkResult st = helios_swapchain_status(items[k].sc);
      results[k] = st;
      if (st == VK_SUCCESS)
         any_presentable = true;
      else if (st == VK_ERROR_OUT_OF_DATE_KHR)
         helios_count(HELIOS_CNT_present_refused_out_of_date);
      else if (st == VK_ERROR_SURFACE_LOST_KHR)
         helios_count(HELIOS_CNT_present_refused_surface_lost);
   }

   /* ---- one lower vkQueueSubmit2 on the application's queue ---- */
   std::vector<VkSemaphoreSubmitInfo> waits;
   waits.reserve(pPresentInfo->waitSemaphoreCount);
   for (uint32_t k = 0; k < pPresentInfo->waitSemaphoreCount; k++) {
      VkSemaphoreSubmitInfo w = {};
      w.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
      w.semaphore = pPresentInfo->pWaitSemaphores[k];
      w.value = 0;
      w.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
      waits.push_back(w);
   }

   std::vector<VkCommandBufferSubmitInfo> cbs;
   std::vector<VkSemaphoreSubmitInfo> signals;
   cbs.reserve(items.size());
   signals.reserve(items.size());

   for (uint32_t k = 0; k < items.size(); k++) {
      if (results[k] != VK_SUCCESS)
         continue;
      HeliosSlot &slot = items[k].sc->slots[items[k].index];
      VkResult r;
      {
         std::lock_guard<std::mutex> sg(slot.lock);
         r = helios_record_barrier(
            dev, slot.release_cb, slot.image, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            VK_IMAGE_LAYOUT_GENERAL, dev->canonical_family,
            VK_QUEUE_FAMILY_EXTERNAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            VK_ACCESS_2_MEMORY_WRITE_BIT, VK_PIPELINE_STAGE_2_NONE, 0);
      }
      if (r != VK_SUCCESS) {
         helios_count(HELIOS_CNT_present_submit_failed);
         return VK_ERROR_DEVICE_LOST;
      }
      VkCommandBufferSubmitInfo cbi = {};
      cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
      cbi.commandBuffer = slot.release_cb;
      cbs.push_back(cbi);

      VkSemaphoreSubmitInfo sig = {};
      sig.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
      sig.semaphore = slot.ready_sem;
      sig.value = items[k].epoch;
      sig.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
      signals.push_back(sig);
   }

   if (!any_presentable) {
      /* Nothing to present: no wait is consumed and no image changes. Report
       * the per-swapchain results and the first error as the aggregate. */
      if (pPresentInfo->pResults)
         for (uint32_t k = 0; k < items.size(); k++)
            pPresentInfo->pResults[k] = results[k];
      for (VkResult r : results)
         if (r != VK_SUCCESS)
            return r;
      return VK_SUCCESS;
   }

   VkSubmitInfo2 si = {};
   si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
   si.waitSemaphoreInfoCount = (uint32_t)waits.size();
   si.pWaitSemaphoreInfos = waits.empty() ? nullptr : waits.data();
   si.commandBufferInfoCount = (uint32_t)cbs.size();
   si.pCommandBufferInfos = cbs.empty() ? nullptr : cbs.data();
   si.signalSemaphoreInfoCount = (uint32_t)signals.size();
   si.pSignalSemaphoreInfos = signals.empty() ? nullptr : signals.data();

   if (dev->disp.QueueSubmit2(queue, 1, &si, VK_NULL_HANDLE) != VK_SUCCESS) {
      helios_count(HELIOS_CNT_present_submit_failed);
      return VK_ERROR_DEVICE_LOST;
   }

   for (uint32_t k = 0; k < items.size(); k++) {
      if (results[k] != VK_SUCCESS)
         continue;
      std::lock_guard<std::mutex> g(items[k].sc->lock);
      HeliosSlot &slot = items[k].sc->slots[items[k].index];
      slot.state = HELIOS_SLOT_PRESENT_QUEUED;
      slot.externally_owned = true;
   }

   /* ---- the per-(swapchain,image) D3D chain, in input order ----
    * No layer mutex is held across these D3D/DXGI calls (§10.7:2604). */
   bool device_lost = false;
   for (uint32_t k = 0; k < items.size(); k++) {
      if (results[k] != VK_SUCCESS)
         continue;
      HeliosSwapchain *sc = items[k].sc;
      HeliosSlot &slot = sc->slots[items[k].index];
      const uint64_t epoch = items[k].epoch;

      sc->queue->Wait(slot.ready, epoch);

      const UINT j = sc->dxgi->GetCurrentBackBufferIndex();
      if (j >= sc->backbuffers.size() || !sc->backbuffers[j]) {
         helios_count(HELIOS_CNT_present_copy_failed);
         results[k] = VK_ERROR_DEVICE_LOST;
         device_lost = true;
         continue;
      }
      ID3D12Resource *back = sc->backbuffers[j];

      HRESULT hr = S_OK;
      {
         std::lock_guard<std::mutex> sg(slot.lock);
         hr = slot.list->Reset(slot.alloc, nullptr);
         if (SUCCEEDED(hr)) {
            D3D12_RESOURCE_BARRIER pre[2] = {
               helios_transition(slot.s, D3D12_RESOURCE_STATE_COMMON,
                                 D3D12_RESOURCE_STATE_COPY_SOURCE),
               helios_transition(back, D3D12_RESOURCE_STATE_PRESENT,
                                 D3D12_RESOURCE_STATE_COPY_DEST),
            };
            slot.list->ResourceBarrier(2, pre);
            slot.list->CopyResource(back, slot.s);
            D3D12_RESOURCE_BARRIER post[2] = {
               helios_transition(slot.s, D3D12_RESOURCE_STATE_COPY_SOURCE,
                                 D3D12_RESOURCE_STATE_COMMON),
               helios_transition(back, D3D12_RESOURCE_STATE_COPY_DEST,
                                 D3D12_RESOURCE_STATE_PRESENT),
            };
            slot.list->ResourceBarrier(2, post);
            hr = slot.list->Close();
         }
      }
      if (FAILED(hr)) {
         helios_count(HELIOS_CNT_present_copy_failed);
         results[k] = VK_ERROR_DEVICE_LOST;
         device_lost = true;
         continue;
      }

      ID3D12CommandList *lists[1] = { slot.list };
      sc->queue->ExecuteCommandLists(1, lists);
      if (FAILED(sc->queue->Signal(slot.release, epoch))) {
         helios_count(HELIOS_CNT_present_copy_failed);
         results[k] = VK_ERROR_DEVICE_LOST;
         device_lost = true;
         continue;
      }

      hr = sc->dxgi->Present(1, 0);
      results[k] = helios_map_present_hresult(hr);
      if (results[k] == VK_ERROR_OUT_OF_DATE_KHR)
         helios_count(HELIOS_CNT_present_dxgi_out_of_date);
      if (results[k] == VK_ERROR_DEVICE_LOST) {
         helios_count(HELIOS_CNT_present_device_lost);
         device_lost = true;
      }

      std::lock_guard<std::mutex> g(sc->lock);
      slot.state = HELIOS_SLOT_RELEASE_QUEUED;
      if (results[k] == VK_ERROR_OUT_OF_DATE_KHR)
         sc->retired = true;
      if (results[k] == VK_ERROR_DEVICE_LOST)
         sc->lost = true;
      if (sc->state_event)
         SetEvent(sc->state_event);
   }

   if (pPresentInfo->pResults)
      for (uint32_t k = 0; k < items.size(); k++)
         pPresentInfo->pResults[k] = results[k];

   /*
    * §10.7:2691-2695 — after the common release has transferred ownership, a
    * failure that leaves the layer unable to produce the matching D3D
    * Release/acquire path for every affected image is a device loss, not an
    * "unaffected" report.
    */
   if (device_lost)
      return VK_ERROR_DEVICE_LOST;
   for (VkResult r : results)
      if (r != VK_SUCCESS)
         return r;
   return VK_SUCCESS;
}

/* ==================================================================== */
/* 12. Teardown                                                         */
/* ==================================================================== */

/* Waits, on the CPU, for one slot's Release[i]=e. Bounded by an event; never
 * polls. Returns false only on a D3D failure. */
static bool
helios_wait_release(HeliosSlot &slot)
{
   if (!slot.release || slot.epoch == 0)
      return true;
   if (slot.release->GetCompletedValue() >= slot.epoch)
      return true;
   if (FAILED(slot.release->SetEventOnCompletion(slot.epoch, slot.release_event)))
      return false;
   return WaitForSingleObject(slot.release_event, INFINITE) == WAIT_OBJECT_0;
}

/*
 * §12.3:3309-3319. The layer relies on the application's C48 precondition for
 * its own recorded work; it drains only what it owns: in-flight D3D Release
 * and DXGI Present, then the reciprocal EXTERNAL -> canonical-family barrier
 * for every slot left in external ownership.
 */
static void
helios_drain_swapchain(HeliosSwapchain *sc)
{
   HeliosDevice *dev = sc->dev;
   if (!dev)
      return;

   for (HeliosSlot &slot : sc->slots) {
      if (slot.state == HELIOS_SLOT_RELEASE_QUEUED ||
          slot.state == HELIOS_SLOT_PRESENT_QUEUED ||
          slot.state == HELIOS_SLOT_D3D_COPY ||
          slot.state == HELIOS_SLOT_DXGI_OWNED) {
         if (!helios_wait_release(slot))
            helios_count(HELIOS_CNT_teardown_release_wait_failed);
      }
   }

   /* One D3D queue quiescence point before any resource is released. */
   if (sc->queue && !sc->slots.empty() && sc->slots[0].release) {
      ID3D12Fence *f = sc->slots[0].release;
      const uint64_t target = f->GetCompletedValue() + 1;
      if (SUCCEEDED(sc->queue->Signal(f, target)) &&
          f->GetCompletedValue() < target) {
         if (SUCCEEDED(f->SetEventOnCompletion(target, sc->slots[0].release_event)))
            WaitForSingleObject(sc->slots[0].release_event, INFINITE);
      }
      /* Keep the slot's epoch bookkeeping consistent with the bumped fence. */
      sc->slots[0].epoch = target;
      sc->slots[0].retired_epoch = target;
   }

   /* Reciprocal ownership barrier for every externally owned slot. A never
    * presented Vulkan-owned slot needs no invented transition
    * (§12.3:3315-3318). */
   if (dev->helper_queue && dev->disp.QueueSubmit2) {
      std::vector<VkCommandBufferSubmitInfo> cbs;
      for (HeliosSlot &slot : sc->slots) {
         if (!slot.externally_owned || slot.state == HELIOS_SLOT_LOST)
            continue;
         if (dev->disp.ResetCommandPool(dev->device, slot.pool, 0) != VK_SUCCESS ||
             helios_record_barrier(dev, slot.acquire_cb, slot.image,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_QUEUE_FAMILY_EXTERNAL, dev->canonical_family,
                                   VK_PIPELINE_STAGE_2_NONE, 0,
                                   VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                   VK_ACCESS_2_MEMORY_READ_BIT |
                                      VK_ACCESS_2_MEMORY_WRITE_BIT) != VK_SUCCESS) {
            helios_count(HELIOS_CNT_teardown_barrier_failed);
            continue;
         }
         VkCommandBufferSubmitInfo cbi = {};
         cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
         cbi.commandBuffer = slot.acquire_cb;
         cbs.push_back(cbi);
      }
      if (!cbs.empty()) {
         VkFenceCreateInfo fci = {};
         fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
         VkFence fence = VK_NULL_HANDLE;
         if (dev->disp.CreateFence(dev->device, &fci, nullptr, &fence) !=
             VK_SUCCESS) {
            helios_count(HELIOS_CNT_teardown_barrier_failed);
         } else {
            VkSubmitInfo2 si = {};
            si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
            si.commandBufferInfoCount = (uint32_t)cbs.size();
            si.pCommandBufferInfos = cbs.data();
            VkResult r;
            {
               std::lock_guard<std::mutex> hg(dev->helper_lock);
               r = dev->disp.QueueSubmit2(dev->helper_queue, 1, &si, fence);
            }
            if (r != VK_SUCCESS)
               helios_count(HELIOS_CNT_teardown_barrier_failed);
            else
               dev->disp.WaitForFences(dev->device, 1, &fence, VK_TRUE, UINT64_MAX);
            dev->disp.DestroyFence(dev->device, fence, nullptr);
         }
      }
   }
}

/* Releases everything the swapchain owns. Callable on a partially built
 * swapchain (the create path uses it as its unwind). */
static void
helios_destroy_swapchain_locked(HeliosSwapchain *sc)
{
   HeliosDevice *dev = sc->dev;
   sc->destroying = true;
   if (sc->state_event)
      SetEvent(sc->state_event);

   helios_drain_swapchain(sc);

   if (dev) {
      for (HeliosSlot &slot : sc->slots)
         helios_slot_teardown(dev, slot);
   }
   sc->slots.clear();

   /* Every backbuffer reference is released before DXGI destruction. */
   for (ID3D12Resource *&b : sc->backbuffers)
      helios_com_release(b);
   sc->backbuffers.clear();

   helios_com_release(sc->dxgi);
   helios_com_release(sc->queue);
   if (sc->state_event) {
      CloseHandle(sc->state_event);
      sc->state_event = nullptr;
   }
   sc->magic = 0;
}

static VKAPI_ATTR void VKAPI_CALL
helios_DestroySwapchainKHR(VkDevice device, VkSwapchainKHR swapchain,
                           const VkAllocationCallbacks *pAllocator)
{
   (void)pAllocator;
   HeliosDevice *dev = helios_device_of(device);
   HeliosSwapchain *sc = helios_swapchain_of(swapchain);
   if (!dev || !sc || sc->dev != dev)
      return;

   {
      std::lock_guard<std::mutex> g(helios_registry_lock);
      helios_swapchains.erase(sc);
   }
   {
      std::lock_guard<std::mutex> g(dev->lock);
      dev->swapchains.erase(sc);
   }
   helios_destroy_swapchain_locked(sc);
   delete sc;
}

/* ==================================================================== */
/* 13. C45 swapchain-memory alias images                                */
/* ==================================================================== */
/*
 * STUB: unit B7 of the Mesa lane brief is not implemented in this changeset.
 *
 * What IS implemented is the null-swapchain form, which §10.7:2356-2365 makes
 * a pure pass-through: the layer consumes and strips only that no-op
 * structure, preserves every other pNext item, and forwards an ordinary lower
 * create/bind. That path is exercised by ordinary applications and must not
 * regress.
 *
 * The non-null alias form (VUIDs 00995/01630/01631/01644, the dedicated
 * D3D12_RESOURCE_BIT re-import of H[i] per alias, ALIAS_ONLY survival past
 * swapchain destruction) is REFUSED, loudly and per call, with the counters
 * alias_image_refused_unimplemented / alias_bind_refused_unimplemented. It is
 * never approximated with ordinary memory or a fake lower swapchain — that is
 * exactly what §10.9:2858 forbids.
 */

static VKAPI_ATTR VkResult VKAPI_CALL
helios_CreateImage(VkDevice device, const VkImageCreateInfo *pCreateInfo,
                   const VkAllocationCallbacks *pAllocator, VkImage *pImage)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->disp.CreateImage)
      return VK_ERROR_INITIALIZATION_FAILED;

   const VkImageSwapchainCreateInfoKHR *sci =
      (const VkImageSwapchainCreateInfoKHR *)helios_find_pnext(
         pCreateInfo->pNext, VK_STRUCTURE_TYPE_IMAGE_SWAPCHAIN_CREATE_INFO_KHR);
   if (!sci)
      return dev->disp.CreateImage(device, pCreateInfo, pAllocator, pImage);

   if (sci->swapchain != VK_NULL_HANDLE)
      return helios_refuse(HELIOS_CNT_alias_image_refused_unimplemented,
                           VK_ERROR_INITIALIZATION_FAILED,
                           "swapchain-memory alias images are not implemented "
                           "in this generation");

   /* Null form: consume and strip only that structure, preserve the rest. */
   helios_count(HELIOS_CNT_alias_null_form_forwarded);
   HeliosStrippedChain stripped;
   if (!helios_strip_pnext(pCreateInfo->pNext, (const VkBaseInStructure *)sci,
                           &stripped))
      return helios_refuse(HELIOS_CNT_alias_image_refused_unimplemented,
                           VK_ERROR_INITIALIZATION_FAILED,
                           "cannot unlink VkImageSwapchainCreateInfoKHR from "
                           "behind an unknown pNext structure");
   VkImageCreateInfo copy = *pCreateInfo;
   copy.pNext = stripped.head;
   return dev->disp.CreateImage(device, &copy, pAllocator, pImage);
}

static VKAPI_ATTR void VKAPI_CALL
helios_DestroyImage(VkDevice device, VkImage image,
                    const VkAllocationCallbacks *pAllocator)
{
   HeliosDevice *dev = helios_device_of(device);
   if (dev && dev->disp.DestroyImage)
      dev->disp.DestroyImage(device, image, pAllocator);
}

static VKAPI_ATTR VkResult VKAPI_CALL
helios_BindImageMemory2(VkDevice device, uint32_t bindInfoCount,
                        const VkBindImageMemoryInfo *pBindInfos)
{
   HeliosDevice *dev = helios_device_of(device);
   if (!dev || !dev->disp.BindImageMemory2)
      return VK_ERROR_INITIALIZATION_FAILED;

   bool any_alias = false;
   for (uint32_t i = 0; i < bindInfoCount; i++) {
      const VkBindImageMemorySwapchainInfoKHR *bs =
         (const VkBindImageMemorySwapchainInfoKHR *)helios_find_pnext(
            pBindInfos[i].pNext,
            VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_SWAPCHAIN_INFO_KHR);
      if (bs && bs->swapchain != VK_NULL_HANDLE)
         any_alias = true;
   }
   if (any_alias)
      return helios_refuse(HELIOS_CNT_alias_bind_refused_unimplemented,
                           VK_ERROR_INITIALIZATION_FAILED,
                           "swapchain-memory alias bind is not implemented in "
                           "this generation");

   /* No non-null alias entry. The null form is consumed and stripped, and the
    * ordinary lower bind is forwarded with the caller's memory and offset
    * (§10.7:2359-2363). Input order and per-bind output chains are preserved
    * by construction: only the consumed node is removed. */
   std::vector<VkBindImageMemoryInfo> copies;
   std::vector<HeliosStrippedChain> chains(bindInfoCount);
   std::vector<char> stripped(bindInfoCount, 0);
   bool any_stripped = false;
   for (uint32_t i = 0; i < bindInfoCount; i++) {
      const VkBindImageMemorySwapchainInfoKHR *bs =
         (const VkBindImageMemorySwapchainInfoKHR *)helios_find_pnext(
            pBindInfos[i].pNext,
            VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_SWAPCHAIN_INFO_KHR);
      if (!bs)
         continue;
      any_stripped = true;
      if (!helios_strip_pnext(pBindInfos[i].pNext, (const VkBaseInStructure *)bs,
                              &chains[i]))
         return helios_refuse(HELIOS_CNT_alias_bind_refused_unimplemented,
                              VK_ERROR_INITIALIZATION_FAILED,
                              "cannot unlink VkBindImageMemorySwapchainInfoKHR "
                              "from behind an unknown pNext structure");
      stripped[i] = 1;
   }
   if (!any_stripped)
      return dev->disp.BindImageMemory2(device, bindInfoCount, pBindInfos);

   helios_count(HELIOS_CNT_alias_null_form_forwarded);
   copies.assign(pBindInfos, pBindInfos + bindInfoCount);
   for (uint32_t i = 0; i < bindInfoCount; i++)
      if (stripped[i])
         copies[i].pNext = chains[i].head;
   return dev->disp.BindImageMemory2(device, bindInfoCount, copies.data());
}

/* ==================================================================== */
/* 14. Dispatch closure                                                 */
/* ==================================================================== */

#define HELIOS_FN(f) (PFN_vkVoidFunction)(void (*)(void))(f)

static const HeliosEntry kInstanceEntries[] = {
   { "vkGetInstanceProcAddr", HELIOS_FN(helios_layer_GetInstanceProcAddr),
     HELIOS_GATE_ALWAYS, false },
   { "vkCreateInstance", HELIOS_FN(helios_CreateInstance), HELIOS_GATE_ALWAYS,
     false },
   { "vkDestroyInstance", HELIOS_FN(helios_DestroyInstance), HELIOS_GATE_ALWAYS,
     false },
   { "vkEnumerateInstanceExtensionProperties",
     HELIOS_FN(helios_layer_EnumerateInstanceExtensionProperties),
     HELIOS_GATE_ALWAYS, false },
   { "vkEnumerateDeviceExtensionProperties",
     HELIOS_FN(helios_EnumerateDeviceExtensionProperties), HELIOS_GATE_ALWAYS,
     true },
   { "vkCreateWin32SurfaceKHR", HELIOS_FN(helios_CreateWin32SurfaceKHR),
     HELIOS_GATE_WIN32_SURFACE, false },
   { "vkDestroySurfaceKHR", HELIOS_FN(helios_DestroySurfaceKHR),
     HELIOS_GATE_SURFACE, false },
   { "vkGetPhysicalDeviceSurfaceSupportKHR",
     HELIOS_FN(helios_GetPhysicalDeviceSurfaceSupportKHR), HELIOS_GATE_SURFACE,
     true },
   { "vkGetPhysicalDeviceSurfaceCapabilitiesKHR",
     HELIOS_FN(helios_GetPhysicalDeviceSurfaceCapabilitiesKHR),
     HELIOS_GATE_SURFACE, true },
   { "vkGetPhysicalDeviceSurfaceFormatsKHR",
     HELIOS_FN(helios_GetPhysicalDeviceSurfaceFormatsKHR), HELIOS_GATE_SURFACE,
     true },
   { "vkGetPhysicalDeviceSurfacePresentModesKHR",
     HELIOS_FN(helios_GetPhysicalDeviceSurfacePresentModesKHR),
     HELIOS_GATE_SURFACE, true },
   { "vkGetPhysicalDeviceWin32PresentationSupportKHR",
     HELIOS_FN(helios_GetPhysicalDeviceWin32PresentationSupportKHR),
     HELIOS_GATE_WIN32_SURFACE, true },
   { "vkGetPhysicalDeviceSurfaceCapabilities2KHR",
     HELIOS_FN(helios_GetPhysicalDeviceSurfaceCapabilities2KHR),
     HELIOS_GATE_CAPS2, true },
   { "vkGetPhysicalDeviceSurfaceFormats2KHR",
     HELIOS_FN(helios_GetPhysicalDeviceSurfaceFormats2KHR), HELIOS_GATE_CAPS2,
     true },
   { "vkEnumeratePhysicalDeviceGroups",
     HELIOS_FN(helios_EnumeratePhysicalDeviceGroups), HELIOS_GATE_CORE_1_1, false },
   { "vkEnumeratePhysicalDeviceGroupsKHR",
     HELIOS_FN(helios_EnumeratePhysicalDeviceGroups),
     HELIOS_GATE_DEVICE_GROUP_CREATION_KHR, false },
   { "vkGetPhysicalDevicePresentRectanglesKHR",
     HELIOS_FN(helios_GetPhysicalDevicePresentRectanglesKHR), HELIOS_GATE_SURFACE,
     true },
   { "vkGetPhysicalDeviceQueueFamilyProperties",
     HELIOS_FN(helios_GetPhysicalDeviceQueueFamilyProperties), HELIOS_GATE_ALWAYS,
     true },
   { "vkGetPhysicalDeviceQueueFamilyProperties2",
     HELIOS_FN(helios_GetPhysicalDeviceQueueFamilyProperties2),
     HELIOS_GATE_CORE_1_1, true },
   { "vkGetPhysicalDeviceQueueFamilyProperties2KHR",
     HELIOS_FN(helios_GetPhysicalDeviceQueueFamilyProperties2),
     HELIOS_GATE_PHYSDEV_PROPS2_KHR, true },
   { "vkCreateDevice", HELIOS_FN(helios_CreateDevice), HELIOS_GATE_ALWAYS, true },
};

static const HeliosEntry kDeviceEntries[] = {
   { "vkGetDeviceProcAddr", HELIOS_FN(helios_layer_GetDeviceProcAddr),
     HELIOS_GATE_ALWAYS, false },
   { "vkDestroyDevice", HELIOS_FN(helios_DestroyDevice), HELIOS_GATE_ALWAYS, false },
   { "vkCreateSwapchainKHR", HELIOS_FN(helios_CreateSwapchainKHR),
     HELIOS_GATE_SWAPCHAIN, false },
   { "vkDestroySwapchainKHR", HELIOS_FN(helios_DestroySwapchainKHR),
     HELIOS_GATE_SWAPCHAIN, false },
   { "vkGetSwapchainImagesKHR", HELIOS_FN(helios_GetSwapchainImagesKHR),
     HELIOS_GATE_SWAPCHAIN, false },
   { "vkAcquireNextImageKHR", HELIOS_FN(helios_AcquireNextImageKHR),
     HELIOS_GATE_SWAPCHAIN, false },
   { "vkAcquireNextImage2KHR", HELIOS_FN(helios_AcquireNextImage2KHR),
     HELIOS_GATE_SWAPCHAIN, false },
   { "vkQueuePresentKHR", HELIOS_FN(helios_QueuePresentKHR), HELIOS_GATE_SWAPCHAIN,
     false },
   { "vkGetDeviceGroupPresentCapabilitiesKHR",
     HELIOS_FN(helios_GetDeviceGroupPresentCapabilitiesKHR), HELIOS_GATE_SWAPCHAIN,
     false },
   { "vkGetDeviceGroupSurfacePresentModesKHR",
     HELIOS_FN(helios_GetDeviceGroupSurfacePresentModesKHR), HELIOS_GATE_SWAPCHAIN,
     false },
   { "vkGetDeviceQueue", HELIOS_FN(helios_GetDeviceQueue), HELIOS_GATE_ALWAYS,
     false },
   { "vkGetDeviceQueue2", HELIOS_FN(helios_GetDeviceQueue2), HELIOS_GATE_CORE_1_1,
     false },
   { "vkCreateImage", HELIOS_FN(helios_CreateImage), HELIOS_GATE_ALWAYS, false },
   { "vkDestroyImage", HELIOS_FN(helios_DestroyImage), HELIOS_GATE_ALWAYS, false },
   { "vkBindImageMemory2", HELIOS_FN(helios_BindImageMemory2), HELIOS_GATE_CORE_1_1,
     false },
   { "vkBindImageMemory2KHR", HELIOS_FN(helios_BindImageMemory2),
     HELIOS_GATE_BIND_MEMORY2_KHR, false },
};

#undef HELIOS_FN

static const HeliosEntry *
helios_instance_entries(uint32_t *count)
{
   *count = (uint32_t)(sizeof(kInstanceEntries) / sizeof(kInstanceEntries[0]));
   return kInstanceEntries;
}

static const HeliosEntry *
helios_device_entries(uint32_t *count)
{
   *count = (uint32_t)(sizeof(kDeviceEntries) / sizeof(kDeviceEntries[0]));
   return kDeviceEntries;
}

static bool
helios_instance_gate_open(const HeliosInstance *inst, int gate)
{
   switch (gate) {
   case HELIOS_GATE_ALWAYS:
      return true;
   case HELIOS_GATE_SURFACE:
      return inst && inst->surface_enabled;
   case HELIOS_GATE_WIN32_SURFACE:
      return inst && inst->wsi_enabled();
   case HELIOS_GATE_CAPS2:
      return inst && inst->surface_enabled && inst->surface_caps2_enabled;
   case HELIOS_GATE_CORE_1_1:
      return inst && VK_API_VERSION_MINOR(inst->app_api_version) >= 1;
   case HELIOS_GATE_DEVICE_GROUP_CREATION_KHR:
      return inst && inst->device_group_creation_enabled;
   case HELIOS_GATE_PHYSDEV_PROPS2_KHR:
      return inst && inst->physdev_props2_enabled;
   default:
      return false;
   }
}

static bool
helios_device_gate_open(const HeliosDevice *dev, int gate)
{
   switch (gate) {
   case HELIOS_GATE_ALWAYS:
      return true;
   case HELIOS_GATE_SWAPCHAIN:
      return dev && dev->wsi_enabled;
   case HELIOS_GATE_CORE_1_1:
      return dev && dev->inst &&
             VK_API_VERSION_MINOR(dev->inst->app_api_version) >= 1;
   case HELIOS_GATE_BIND_MEMORY2_KHR:
      return dev && dev->bind_memory2_khr_enabled;
   default:
      return false;
   }
}

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
helios_layer_GetInstanceProcAddr(VkInstance instance, const char *pName)
{
   if (!pName)
      return nullptr;

   HeliosInstance *inst = helios_instance_of(instance);

   for (const HeliosEntry &e : kInstanceEntries) {
      if (!streq(pName, e.name))
         continue;
      /* Global commands are answerable with a null instance; everything else
       * needs the instance to exist and its gate to be open. */
      if (instance == VK_NULL_HANDLE)
         return (streq(pName, "vkCreateInstance") ||
                 streq(pName, "vkEnumerateInstanceExtensionProperties") ||
                 streq(pName, "vkGetInstanceProcAddr"))
                   ? e.fn
                   : nullptr;
      return helios_instance_gate_open(inst, e.gate) ? e.fn : nullptr;
   }

   /* vkGetInstanceProcAddr must also serve the layer's device commands so the
    * loader can build its trampoline table; per-device enablement is enforced
    * by vkGetDeviceProcAddr. */
   if (instance != VK_NULL_HANDLE) {
      for (const HeliosEntry &e : kDeviceEntries) {
         if (streq(pName, e.name))
            return e.fn;
      }
   }

   /* The loader's own layer-enumeration exports are not part of the dispatch
    * closure and are answered from the DLL export table, not from here. */
   if (instance == VK_NULL_HANDLE || !inst || !inst->disp.GetInstanceProcAddr)
      return nullptr;
   return inst->disp.GetInstanceProcAddr(instance, pName);
}

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
helios_layer_GetDeviceProcAddr(VkDevice device, const char *pName)
{
   if (!pName || device == VK_NULL_HANDLE)
      return nullptr;
   HeliosDevice *dev = helios_device_of(device);
   if (!dev)
      return nullptr;

   for (const HeliosEntry &e : kDeviceEntries) {
      if (!streq(pName, e.name))
         continue;
      return helios_device_gate_open(dev, e.gate) ? e.fn : nullptr;
   }
   if (!dev->disp.GetDeviceProcAddr)
      return nullptr;
   return dev->disp.GetDeviceProcAddr(device, pName);
}

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
helios_layer_GetPhysicalDeviceProcAddr(VkInstance instance, const char *pName)
{
   if (!pName)
      return nullptr;
   HeliosInstance *inst = helios_instance_of(instance);
   for (const HeliosEntry &e : kInstanceEntries) {
      if (!e.phys || !streq(pName, e.name))
         continue;
      return helios_instance_gate_open(inst, e.gate) ? e.fn : nullptr;
   }
   if (inst && inst->next_gpdpa)
      return inst->next_gpdpa(instance, pName);
   return nullptr;
}

VKAPI_ATTR VkResult VKAPI_CALL
helios_layer_NegotiateLoaderLayerInterfaceVersion(
   VkNegotiateLayerInterface *pVersionStruct)
{
   if (!pVersionStruct ||
       pVersionStruct->sType != LAYER_NEGOTIATE_INTERFACE_STRUCT)
      return VK_ERROR_INITIALIZATION_FAILED;

   /* §10.7:2264-2267 — the generated manifest is compared byte-for-byte with
    * the documented lists before the layer agrees to load. A mismatch means an
    * application could bypass the virtual WSI, so the layer refuses. */
   if (!helios_verify_entry_manifest())
      return VK_ERROR_INITIALIZATION_FAILED;

   if (pVersionStruct->loaderLayerInterfaceVersion >
       CURRENT_LOADER_LAYER_INTERFACE_VERSION)
      pVersionStruct->loaderLayerInterfaceVersion =
         CURRENT_LOADER_LAYER_INTERFACE_VERSION;
   if (pVersionStruct->loaderLayerInterfaceVersion <
       MIN_SUPPORTED_LOADER_LAYER_INTERFACE_VERSION)
      return VK_ERROR_INITIALIZATION_FAILED;

   pVersionStruct->pfnGetInstanceProcAddr = helios_layer_GetInstanceProcAddr;
   pVersionStruct->pfnGetDeviceProcAddr = helios_layer_GetDeviceProcAddr;
   pVersionStruct->pfnGetPhysicalDeviceProcAddr =
      helios_layer_GetPhysicalDeviceProcAddr;
   return VK_SUCCESS;
}

/* ---- DLL exports (see helios_present_layer.def) --------------------- */

extern "C" {

__declspec(dllexport) VKAPI_ATTR VkResult VKAPI_CALL
vkNegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface *pVersionStruct)
{
   return helios_layer_NegotiateLoaderLayerInterfaceVersion(pVersionStruct);
}

__declspec(dllexport) VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
vkGetInstanceProcAddr(VkInstance instance, const char *pName)
{
   return helios_layer_GetInstanceProcAddr(instance, pName);
}

__declspec(dllexport) VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
vkGetDeviceProcAddr(VkDevice device, const char *pName)
{
   return helios_layer_GetDeviceProcAddr(device, pName);
}

__declspec(dllexport) VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL
vk_layerGetPhysicalDeviceProcAddr(VkInstance instance, const char *pName)
{
   return helios_layer_GetPhysicalDeviceProcAddr(instance, pName);
}

__declspec(dllexport) VKAPI_ATTR VkResult VKAPI_CALL
vkEnumerateInstanceLayerProperties(uint32_t *pCount, VkLayerProperties *pProps)
{
   return helios_layer_EnumerateInstanceLayerProperties(pCount, pProps);
}

__declspec(dllexport) VKAPI_ATTR VkResult VKAPI_CALL
vkEnumerateInstanceExtensionProperties(const char *pLayerName, uint32_t *pCount,
                                       VkExtensionProperties *pProps)
{
   return helios_layer_EnumerateInstanceExtensionProperties(pLayerName, pCount,
                                                            pProps);
}

__declspec(dllexport) VKAPI_ATTR VkResult VKAPI_CALL
vkEnumerateDeviceLayerProperties(VkPhysicalDevice physicalDevice, uint32_t *pCount,
                                 VkLayerProperties *pProps)
{
   return helios_layer_EnumerateDeviceLayerProperties(physicalDevice, pCount,
                                                      pProps);
}

__declspec(dllexport) VKAPI_ATTR VkResult VKAPI_CALL
vkEnumerateDeviceExtensionProperties(VkPhysicalDevice physicalDevice,
                                     const char *pLayerName, uint32_t *pCount,
                                     VkExtensionProperties *pProps)
{
   return helios_EnumerateDeviceExtensionProperties(physicalDevice, pLayerName,
                                                    pCount, pProps);
}

} /* extern "C" */
