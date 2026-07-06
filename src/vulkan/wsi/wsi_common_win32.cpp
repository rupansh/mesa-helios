/*
 * Copyright © 2015 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <assert.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h> /* helios_wsi_vehicle_diag */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h> /* helios_wsi_vehicle_diag timestamps */

#include "util/cnd_monotonic.h"
#include "util/os_time.h"
#include "util/timespec.h"
#include "util/u_thread.h"
#include "vk_format.h"
#include "vk_instance.h"
#include "vk_physical_device.h"
#include "vk_util.h"
#include "wsi_common_entrypoints.h"
#include "wsi_common_private.h"

#define D3D12_IGNORE_SDK_LAYERS
#include <dxgi1_4.h>
#include <directx/d3d12.h>
#include <dxguids/dxguids.h>

#include <d3d11.h> /* Helios dcomp present vehicle (road 4) */
#include <dcomp.h>

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"      // warning: cast to pointer from integer of different size
#endif

struct wsi_win32;

struct wsi_win32 {
   struct wsi_interface                     base;

   struct wsi_device *wsi;

   const VkAllocationCallbacks *alloc;
   VkPhysicalDevice physical_device;
   struct {
      IDXGIFactory4 *factory;
      IDCompositionDevice *dcomp;
   } dxgi;
};

struct helios_win32_wsi_perf {
   bool initialized;
   bool enabled;
   uint64_t interval;
   uint64_t frames;
   uint64_t direct_frames;
   uint64_t copy_ns;
   uint64_t stretch_ns;
   uint64_t get_dc_ns;
};

static struct helios_win32_wsi_perf helios_win32_wsi_perf;
static bool helios_win32_wsi_direct_map_initialized;
static bool helios_win32_wsi_direct_map;

/* Helios dcomp present vehicle — loud process-wide counters, reported in the
 * HELIOS_WSI_PERF line and in the failure diag lines. Every skipped/refused
 * vehicle path moves one. (The vehicle itself is defined further down.) */
static volatile LONG helios_vehicle_creates;        /* worker attempts */
static volatile LONG helios_vehicle_ready;          /* READY latches */
static volatile LONG helios_vehicle_create_fails;   /* FAILED latches (init) */
static volatile LONG helios_vehicle_export_miss;    /* UMD exports unresolved */
static volatile LONG helios_vehicle_presents;       /* vehicle Present() OK */
static volatile LONG helios_vehicle_present_fails;  /* Present()/copy errors */
static volatile LONG helios_vehicle_fallbacks;      /* presents served sw while
                                                     * INIT/FAILED on a vehicle-
                                                     * requested chain */
static volatile LONG helios_vehicle_wait_timeouts;  /* wait_last_present bounded
                                                     * timeouts (unit 3) */

static bool
helios_win32_wsi_direct_map_enabled(void)
{
   if (!helios_win32_wsi_direct_map_initialized) {
      char value[64];
      helios_win32_wsi_direct_map_initialized = true;
      helios_win32_wsi_direct_map =
         GetEnvironmentVariableA("HELIOS_WSI_DIRECT_MAP", value, sizeof(value)) &&
         value[0] && value[0] != '0';
   }

   return helios_win32_wsi_direct_map;
}

static void
helios_win32_wsi_perf_init(void)
{
   if (helios_win32_wsi_perf.initialized)
      return;

   helios_win32_wsi_perf.initialized = true;
   helios_win32_wsi_perf.interval = 300;

   char value[64];
   helios_win32_wsi_perf.enabled =
      GetEnvironmentVariableA("HELIOS_WSI_PERF", value, sizeof(value)) &&
      value[0] && value[0] != '0';

   if (GetEnvironmentVariableA("HELIOS_WSI_PERF_INTERVAL", value, sizeof(value))) {
      char *end = NULL;
      unsigned long long parsed = strtoull(value, &end, 10);
      if (end && *end == '\0' && parsed > 0)
         helios_win32_wsi_perf.interval = parsed;
   }
}

static void
helios_win32_wsi_perf_write(void)
{
   FILE *f = stderr;
   char path[MAX_PATH];
   if (GetEnvironmentVariableA("HELIOS_WSI_PERF_FILE", path, sizeof(path))) {
      FILE *opened = fopen(path, "a");
      if (opened)
         f = opened;
   }

   fprintf(f,
           "Helios WSI win32 frames=%" PRIu64 " direct=%" PRIu64
           " copy_ms=%.3f copy_avg_us=%.3f"
           " getdc_ms=%.3f getdc_avg_us=%.3f"
           " stretch_ms=%.3f stretch_avg_us=%.3f"
           " vehicle: ready=%ld creates=%ld fails=%ld exp_miss=%ld"
           " presents=%ld pfails=%ld fallbacks=%ld wait_to=%ld\n",
           helios_win32_wsi_perf.frames,
           helios_win32_wsi_perf.direct_frames,
           (double)helios_win32_wsi_perf.copy_ns / 1000000.0,
           helios_win32_wsi_perf.frames ?
              (double)helios_win32_wsi_perf.copy_ns / 1000.0 /
              (double)helios_win32_wsi_perf.frames : 0.0,
           (double)helios_win32_wsi_perf.get_dc_ns / 1000000.0,
           helios_win32_wsi_perf.frames ?
              (double)helios_win32_wsi_perf.get_dc_ns / 1000.0 /
              (double)helios_win32_wsi_perf.frames : 0.0,
           (double)helios_win32_wsi_perf.stretch_ns / 1000000.0,
           helios_win32_wsi_perf.frames ?
              (double)helios_win32_wsi_perf.stretch_ns / 1000.0 /
              (double)helios_win32_wsi_perf.frames : 0.0,
           helios_vehicle_ready, helios_vehicle_creates,
           helios_vehicle_create_fails, helios_vehicle_export_miss,
           helios_vehicle_presents, helios_vehicle_present_fails,
           helios_vehicle_fallbacks, helios_vehicle_wait_timeouts);

   if (f != stderr)
      fclose(f);
}

static void
helios_win32_wsi_perf_note_frame(bool direct, uint64_t copy_ns,
                                 uint64_t get_dc_ns, uint64_t stretch_ns)
{
   helios_win32_wsi_perf_init();
   if (!helios_win32_wsi_perf.enabled)
      return;

   helios_win32_wsi_perf.frames++;
   if (direct)
      helios_win32_wsi_perf.direct_frames++;
   helios_win32_wsi_perf.copy_ns += copy_ns;
   helios_win32_wsi_perf.get_dc_ns += get_dc_ns;
   helios_win32_wsi_perf.stretch_ns += stretch_ns;

   if (helios_win32_wsi_perf.frames % helios_win32_wsi_perf.interval == 0)
      helios_win32_wsi_perf_write();
}

/* ---------------------------------------------------------------------------
 * Helios dcomp present vehicle (WS2 road 4, 23rd session).
 *
 * A D3D11 device on OUR adapter + CreateSwapChainForComposition(FLIP_*) +
 * DirectComposition binding to the app's HWND gives Vulkan swapchains a real
 * hardware flip-model present: DXGI/dcomp mint the win32k present tokens the
 * ICD cannot mint itself (see ROADMAP WS2 — every redirected-token model is
 * runtime-private), while the pixels move GPU-side (the vehicle's DXVK
 * imports the ICD frame by resid and copies it into the backbuffer; unit 2).
 * Mechanism proven live by tools/dcomp_present_probe.cpp (1023 flip presents,
 * dwm composed).
 *
 * Lifecycle contract (the residual-risk containment):
 *  - ALL vehicle work — LoadLibrary d3d11/dxgi/dcomp, D3D11CreateDevice,
 *    factory, composition swapchain, dcomp target/visual — runs on a
 *    DEDICATED worker thread kicked off at vkCreateSwapchainKHR, outside
 *    every ICD instance/device/ring lock. The libraries load lazily there,
 *    never at DllMain.
 *  - Until the vehicle is READY, presents flow through the existing async
 *    sw worker; the swap-in is visible only in counters.
 *  - Any failure latches WSI_VEHICLE_FAILED for that swapchain (loud counter
 *    + one diag line); the sw path keeps serving it. No retry within a
 *    swapchain's lifetime — a recreate (resize) starts a fresh attempt.
 *  - The worker also RELEASES the vehicle COM objects: after READY it parks
 *    on a condvar; swapchain destroy signals stop and joins. The nested
 *    D3D11→helios_umd→DXVK→ICD2 teardown therefore never runs on an ICD1
 *    thread. The dcomp target/visual are per-SURFACE (owned/released by the
 *    surface, like the in-tree dxgi path); the dcomp device + DXGI factory
 *    are process-lifetime.
 *  - Kill switch: HELIOS_WSI_DCOMP_PRESENT (default OFF for bring-up).
 *
 * ICD singleton audit for the nested ICD2 stack (23rd session): TLS rings
 * are keyed per instance, gate5a adapter handles / retire thread / diag
 * handles are per-renderer; the one ambiguous global (last-writer-wins
 * helios_current_ctx_id) got an instance-scoped export the UMD bridge uses.
 */

typedef int32_t (*helios_umd_set_present_source_fn)(
   uint32_t resid, uint64_t fence_value, uint32_t width, uint32_t height,
   uint32_t dxgi_format, uint64_t alloc_size, uint32_t memory_type_index);
typedef int32_t (*helios_umd_wait_last_present_fn)(uint32_t timeout_us);

enum wsi_win32_vehicle_state {
   WSI_VEHICLE_OFF = 0, /* knob off / not applicable to this chain */
   WSI_VEHICLE_INIT,    /* worker building the D3D11/dcomp stack */
   WSI_VEHICLE_READY,   /* vehicle presents may be routed */
   WSI_VEHICLE_FAILED,  /* latched off; the sw path serves this chain */
};

struct wsi_win32_vehicle {
   volatile LONG state; /* enum wsi_win32_vehicle_state */

   thrd_t thread;
   bool thread_started;
   mtx_t mutex; /* guards stop + cond (valid iff thread_started) */
   cnd_t cond;
   bool stop;

   /* Immutable snapshot of the swapchain create parameters for the worker
    * (create_info does not outlive vkCreateSwapchainKHR). */
   HWND hwnd;
   uint32_t width;
   uint32_t height;
   DXGI_FORMAT format;
   DXGI_ALPHA_MODE alpha_mode;
   uint32_t buffer_count;
   bool allow_tearing;

   /* Created AND released on the worker thread. */
   ID3D11Device *dev;
   ID3D11DeviceContext *ctx;
   IDXGISwapChain3 *sc;

   /* In-process UMD exports (unit 2 of the road-4 design), resolved on the
    * worker from the already-loaded helios_umd module. */
   helios_umd_set_present_source_fn set_source;
   helios_umd_wait_last_present_fn wait_present;

   /* SetContent+Commit are deferred to the first successful vehicle present
    * (mirrors the dxgi path's current_swapchain switch): until then the sw
    * GDI blits keep painting the hwnd, so content flows from frame 1. */
   bool content_bound;
};

static bool
wsi_win32_vehicle_enabled(void)
{
   static int cached = -1;
   if (cached < 0) {
      char value[8] = "";
      cached = GetEnvironmentVariableA("HELIOS_WSI_DCOMP_PRESENT", value,
                                       sizeof(value)) &&
               value[0] && value[0] != '0';
   }
   return cached > 0;
}

/* dwm/steam stderr is invisible; vehicle state changes must land in the
 * ProgramData ICD diag log (same file + line shape as vn_renderer_helios.c's
 * helios_diag; duplicated here because wsi_common must not link against the
 * venus renderer directly). Best effort. */
static void
helios_wsi_vehicle_diag(const char *fmt, ...)
{
   CreateDirectoryA("C:\\ProgramData\\Helios", NULL);
   FILE *f = fopen("C:\\ProgramData\\Helios\\helios_icd_diag.log", "a");
   if (!f)
      return;
   fprintf(f, "%lld pid=%lu wsi-vehicle: ", (long long)time(NULL),
           (unsigned long)GetCurrentProcessId());
   va_list ap;
   va_start(ap, fmt);
   vfprintf(f, fmt, ap);
   va_end(ap);
   fputc('\n', f);
   fclose(f);
}

/* Process-lifetime vehicle runtime: the d3d11/dxgi/dcomp modules, the DXGI
 * factory and the (rendering-device-less) dcomp device. Built lazily on a
 * vehicle worker thread — never at DllMain, never on an app thread. The
 * mutex also serializes per-surface dcomp target/visual creation. */
struct wsi_win32_vehicle_runtime {
   mtx_t mutex;
   bool init_attempted;
   bool usable;
   PFN_D3D11_CREATE_DEVICE create_device;
   IDXGIFactory4 *factory;
   IDCompositionDevice *dcomp;

   wsi_win32_vehicle_runtime()
   {
      mtx_init(&mutex, mtx_plain);
      init_attempted = false;
      usable = false;
      create_device = NULL;
      factory = NULL;
      dcomp = NULL;
   }
};

static struct wsi_win32_vehicle_runtime *
wsi_win32_vehicle_runtime_get(void)
{
   /* C++11 magic static: thread-safe construction, process lifetime. */
   static wsi_win32_vehicle_runtime rt;
   return &rt;
}

/* Caller holds rt->mutex. One attempt per process; failure is permanent
 * (missing system DLLs will not appear later). */
static bool
wsi_win32_vehicle_runtime_init_locked(struct wsi_win32_vehicle_runtime *rt)
{
   if (rt->init_attempted)
      return rt->usable;
   rt->init_attempted = true;

   HMODULE d3d11_mod = LoadLibraryA("d3d11.dll");
   HMODULE dxgi_mod = LoadLibraryA("dxgi.dll");
   HMODULE dcomp_mod = LoadLibraryA("dcomp.dll");
   if (!d3d11_mod || !dxgi_mod || !dcomp_mod) {
      helios_wsi_vehicle_diag("runtime FAILED: LoadLibrary d3d11=%p dxgi=%p dcomp=%p",
                              (void *)d3d11_mod, (void *)dxgi_mod, (void *)dcomp_mod);
      return false;
   }

   rt->create_device =
      (PFN_D3D11_CREATE_DEVICE)GetProcAddress(d3d11_mod, "D3D11CreateDevice");

   typedef HRESULT(WINAPI *PFN_CREATE_DXGI_FACTORY2)(UINT, REFIID, void **);
   PFN_CREATE_DXGI_FACTORY2 create_factory2 =
      (PFN_CREATE_DXGI_FACTORY2)GetProcAddress(dxgi_mod, "CreateDXGIFactory2");

   typedef HRESULT(STDAPICALLTYPE *PFN_DCOMP_CREATE_DEVICE)(IDXGIDevice *,
                                                            REFIID, void **);
   PFN_DCOMP_CREATE_DEVICE dcomp_create =
      (PFN_DCOMP_CREATE_DEVICE)GetProcAddress(dcomp_mod,
                                              "DCompositionCreateDevice");

   if (!rt->create_device || !create_factory2 || !dcomp_create) {
      helios_wsi_vehicle_diag("runtime FAILED: exports d3d11=%p factory2=%p dcomp=%p",
                              (void *)rt->create_device, (void *)create_factory2,
                              (void *)dcomp_create);
      return false;
   }

   HRESULT hr = create_factory2(0, IID_PPV_ARGS(&rt->factory));
   if (FAILED(hr) || !rt->factory) {
      helios_wsi_vehicle_diag("runtime FAILED: CreateDXGIFactory2 hr=0x%08lx",
                              (unsigned long)hr);
      return false;
   }

   /* dcomp needs NO rendering device (dzn-proven; NULL = system compositor
    * device) — this is what makes the whole road viable. */
   hr = dcomp_create(NULL, IID_PPV_ARGS(&rt->dcomp));
   if (FAILED(hr) || !rt->dcomp) {
      helios_wsi_vehicle_diag("runtime FAILED: DCompositionCreateDevice hr=0x%08lx",
                              (unsigned long)hr);
      rt->factory->Release();
      rt->factory = NULL;
      return false;
   }

   rt->usable = true;
   return true;
}

/* Vulkan surface format -> vehicle backbuffer format. Flip-model swapchains
 * refuse SRGB formats; UNORM + raw bytes matches what the sw path presents
 * today (the bits are sRGB-encoded either way, dwm treats composition
 * surfaces as sRGB content). Unsupported -> DXGI_FORMAT_UNKNOWN = the chain
 * latches FAILED (counted), sw path serves it. */
static DXGI_FORMAT
wsi_win32_vehicle_dxgi_format(VkFormat format)
{
   switch (format) {
   case VK_FORMAT_B8G8R8A8_UNORM:
   case VK_FORMAT_B8G8R8A8_SRGB:
      return DXGI_FORMAT_B8G8R8A8_UNORM;
   case VK_FORMAT_R8G8B8A8_UNORM:
      return DXGI_FORMAT_R8G8B8A8_UNORM;
   default:
      return DXGI_FORMAT_UNKNOWN;
   }
}

enum wsi_win32_image_state {
   WSI_IMAGE_IDLE,
   WSI_IMAGE_DRAWING,
   WSI_IMAGE_QUEUED,
};

struct wsi_win32_image {
   struct wsi_image base;
   enum wsi_win32_image_state state;
   struct wsi_win32_swapchain *chain;
   struct {
      ID3D12Resource *swapchain_res;
   } dxgi;
   struct {
      HDC dc;
      HBITMAP bmp;
      int bmp_row_pitch;
      void *ppvBits;
   } sw;
};

struct wsi_win32_surface {
   VkIcdSurfaceWin32 base;

   /* The first time a swapchain is created against this surface, a DComp
    * target/visual will be created for it and that swapchain will be bound.
    * When a new swapchain is created, we delay changing the visual's content
    * until that swapchain has completed its first present once, otherwise the
    * window will flash white. When the currently-bound swapchain is destroyed,
    * the visual's content is unset.
    */
   IDCompositionTarget *target;
   IDCompositionVisual *visual;
   struct wsi_win32_swapchain *current_swapchain;
};

struct wsi_win32_swapchain {
   struct wsi_swapchain         base;
   IDXGISwapChain3            *dxgi;
   struct wsi_win32           *wsi;
   wsi_win32_surface          *surface;
   mtx_t                      acquire_mutex;
   struct u_cnd_monotonic     acquire_cond;
   uint64_t                     flip_sequence;
   VkResult                     status;
   VkExtent2D                 extent;
   HWND wnd;
   HDC chain_dc;
   struct wsi_win32_vehicle   vehicle;
   struct wsi_win32_image     images[0];
};

/* Resolve an in-process helios_umd export. The UMD module name is versioned
 * (helios_umd_<hash>.dll), so walk the loaded modules instead of naming it;
 * it is guaranteed loaded by the time this runs — the vehicle's
 * D3D11CreateDevice on the Helios adapter loaded it. (Same pattern as the
 * UMD bridge's find_export_in_loaded_modules for the ICD exports.) */
static void *
wsi_win32_vehicle_find_umd_export(const char *name)
{
   typedef BOOL(WINAPI * PFN_ENUM_PROCESS_MODULES)(HANDLE, HMODULE *, DWORD,
                                                   LPDWORD);
   PFN_ENUM_PROCESS_MODULES enum_modules =
      (PFN_ENUM_PROCESS_MODULES)GetProcAddress(
         GetModuleHandleA("kernel32.dll"), "K32EnumProcessModules");
   if (!enum_modules)
      return NULL;

   HMODULE modules[1024];
   DWORD needed = 0;
   if (!enum_modules(GetCurrentProcess(), modules, sizeof(modules), &needed))
      return NULL;

   DWORD count = MIN2(needed / sizeof(HMODULE), ARRAY_SIZE(modules));
   for (DWORD i = 0; i < count; i++) {
      void *fn = (void *)GetProcAddress(modules[i], name);
      if (fn)
         return fn;
   }
   return NULL;
}

static void
wsi_win32_vehicle_release_com(struct wsi_win32_vehicle *v)
{
   if (v->sc) {
      v->sc->Release();
      v->sc = NULL;
   }
   if (v->ctx) {
      v->ctx->Release();
      v->ctx = NULL;
   }
   if (v->dev) {
      v->dev->Release();
      v->dev = NULL;
   }
}

static bool
wsi_win32_vehicle_stop_requested(struct wsi_win32_vehicle *v)
{
   mtx_lock(&v->mutex);
   bool stop = v->stop;
   mtx_unlock(&v->mutex);
   return stop;
}

/* Build the vehicle on the worker thread. Returns true when READY; on
 * failure everything partially created is released and FAILED is latched
 * (loud). Checks the stop flag between the expensive steps so destroy during
 * init cancels promptly. */
static bool
wsi_win32_vehicle_build(struct wsi_win32_swapchain *chain)
{
   struct wsi_win32_vehicle *v = &chain->vehicle;
   struct wsi_win32_vehicle_runtime *rt = wsi_win32_vehicle_runtime_get();
   const char *stage = "runtime";
   HRESULT hr = S_OK;

   mtx_lock(&rt->mutex);
   bool rt_ok = wsi_win32_vehicle_runtime_init_locked(rt);
   mtx_unlock(&rt->mutex);
   if (!rt_ok)
      goto fail;

   if (wsi_win32_vehicle_stop_requested(v))
      goto stopped;

   {
      stage = "D3D11CreateDevice";
      D3D_FEATURE_LEVEL fl = (D3D_FEATURE_LEVEL)0;
      /* Default adapter = the Helios render adapter (the only one). BGRA
       * support is required for B8G8R8A8 swapchains (probe-proven recipe). */
      hr = rt->create_device(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL,
                             D3D11_CREATE_DEVICE_BGRA_SUPPORT, NULL, 0,
                             D3D11_SDK_VERSION, &v->dev, &fl, &v->ctx);
      if (FAILED(hr) || !v->dev || !v->ctx)
         goto fail;
   }

   if (wsi_win32_vehicle_stop_requested(v))
      goto stopped;

   {
      stage = "CreateSwapChainForComposition";
      DXGI_SWAP_CHAIN_DESC1 desc = {};
      desc.Width = v->width;
      desc.Height = v->height;
      desc.Format = v->format;
      desc.SampleDesc.Count = 1;
      desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
      desc.BufferCount = v->buffer_count;
      desc.Scaling = DXGI_SCALING_STRETCH;
      desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_SEQUENTIAL;
      desc.AlphaMode = v->alpha_mode;
      desc.Flags = v->allow_tearing ? DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0;

      IDXGISwapChain1 *sc1 = NULL;
      hr = rt->factory->CreateSwapChainForComposition(v->dev, &desc, NULL,
                                                      &sc1);
      if (FAILED(hr) || !sc1)
         goto fail;
      hr = sc1->QueryInterface(IID_PPV_ARGS(&v->sc));
      sc1->Release();
      if (FAILED(hr) || !v->sc)
         goto fail;
   }

   /* Per-surface dcomp target/visual (surface-owned; released in
    * wsi_win32_surface_destroy — all chains are gone by then). Serialized
    * against other chains' workers on the runtime mutex. Content binding is
    * deferred to the first vehicle present. */
   {
      stage = "dcomp target/visual";
      wsi_win32_surface *surface = chain->surface;
      mtx_lock(&rt->mutex);
      if (!surface->target)
         hr = rt->dcomp->CreateTargetForHwnd(v->hwnd, FALSE, &surface->target);
      if (SUCCEEDED(hr) && surface->target && !surface->visual) {
         hr = rt->dcomp->CreateVisual(&surface->visual);
         if (SUCCEEDED(hr) && surface->visual)
            hr = surface->target->SetRoot(surface->visual);
      }
      mtx_unlock(&rt->mutex);
      if (FAILED(hr) || !surface->target || !surface->visual)
         goto fail;
   }

   /* The UMD exports land with road-4 unit 2; their absence is a distinct
    * loud failure so an ICD/UMD deploy skew reads unambiguously. */
   {
      stage = "helios_umd exports";
      v->set_source = (helios_umd_set_present_source_fn)
         wsi_win32_vehicle_find_umd_export("helios_umd_set_present_source");
      v->wait_present = (helios_umd_wait_last_present_fn)
         wsi_win32_vehicle_find_umd_export("helios_umd_wait_last_present");
      if (!v->set_source || !v->wait_present) {
         InterlockedIncrement(&helios_vehicle_export_miss);
         hr = E_NOINTERFACE;
         goto fail;
      }
   }

   {
      /* Adapter identity for the READY diag line — the vehicle MUST be on the
       * Helios adapter; a WARP/other-adapter device would silently change the
       * whole data path. */
      WCHAR desc_str[128] = L"?";
      IDXGIDevice *dxgi_dev = NULL;
      if (SUCCEEDED(v->dev->QueryInterface(IID_PPV_ARGS(&dxgi_dev)))) {
         IDXGIAdapter *adapter = NULL;
         if (SUCCEEDED(dxgi_dev->GetAdapter(&adapter))) {
            DXGI_ADAPTER_DESC ad = {};
            if (SUCCEEDED(adapter->GetDesc(&ad)))
               memcpy(desc_str, ad.Description, sizeof(desc_str));
            adapter->Release();
         }
         dxgi_dev->Release();
      }
      helios_wsi_vehicle_diag(
         "READY chain=%p hwnd=%p %ux%u fmt=%u buffers=%u tearing=%d adapter=%ls",
         (void *)chain, (void *)v->hwnd, v->width, v->height, (unsigned)v->format,
         v->buffer_count, v->allow_tearing ? 1 : 0, desc_str);
   }

   InterlockedIncrement(&helios_vehicle_ready);
   InterlockedExchange(&v->state, WSI_VEHICLE_READY);
   return true;

fail:
   helios_wsi_vehicle_diag("FAILED chain=%p stage='%s' hr=0x%08lx %ux%u fmt=%u",
                           (void *)chain, stage, (unsigned long)hr, v->width,
                           v->height, (unsigned)v->format);
   InterlockedIncrement(&helios_vehicle_create_fails);
   wsi_win32_vehicle_release_com(v);
   InterlockedExchange(&v->state, WSI_VEHICLE_FAILED);
   return false;

stopped:
   wsi_win32_vehicle_release_com(v);
   InterlockedExchange(&v->state, WSI_VEHICLE_FAILED);
   return false;
}

static int
wsi_win32_vehicle_thread(void *arg)
{
   struct wsi_win32_swapchain *chain = (struct wsi_win32_swapchain *)arg;
   struct wsi_win32_vehicle *v = &chain->vehicle;

   u_thread_setname("helios-vehicle");
   InterlockedIncrement(&helios_vehicle_creates);

   if (wsi_win32_vehicle_build(chain)) {
      /* Park until destroy: the COM release (and the nested DXVK->ICD2
       * teardown it triggers) must run on THIS thread, never on an ICD1
       * teardown path. */
      mtx_lock(&v->mutex);
      while (!v->stop)
         cnd_wait(&v->cond, &v->mutex);
      mtx_unlock(&v->mutex);

      wsi_win32_vehicle_release_com(v);
   }
   return 0;
}

/* Kick the vehicle worker at swapchain create. Never blocks; never touches
 * D3D on this thread. Any refusal latches FAILED with a counter + diag line
 * and the chain stays on the sw path. */
static void
wsi_win32_vehicle_start(struct wsi_win32_swapchain *chain,
                        const VkSwapchainCreateInfoKHR *create_info)
{
   struct wsi_win32_vehicle *v = &chain->vehicle;

   v->state = WSI_VEHICLE_OFF;
   if (!wsi_win32_vehicle_enabled() || chain->dxgi)
      return;

   DXGI_FORMAT format = wsi_win32_vehicle_dxgi_format(create_info->imageFormat);
   if (format == DXGI_FORMAT_UNKNOWN) {
      helios_wsi_vehicle_diag("REFUSED chain=%p unsupported VkFormat %u",
                              (void *)chain, (unsigned)create_info->imageFormat);
      InterlockedIncrement(&helios_vehicle_create_fails);
      v->state = WSI_VEHICLE_FAILED;
      return;
   }

   VkIcdSurfaceWin32 *win32_surface = (VkIcdSurfaceWin32 *)create_info->surface;
   v->hwnd = win32_surface->hwnd;
   v->width = MAX2(create_info->imageExtent.width, 1u);
   v->height = MAX2(create_info->imageExtent.height, 1u);
   v->format = format;
   /* BufferCount >= 3: flip-model needs headroom so Present never blocks on
    * the compositor holding a buffer (design note, road 4). */
   v->buffer_count = MAX2(3u, create_info->minImageCount);
   v->allow_tearing =
      chain->base.present_mode == VK_PRESENT_MODE_IMMEDIATE_KHR;
   switch (create_info->compositeAlpha) {
   case VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR:
      v->alpha_mode = DXGI_ALPHA_MODE_PREMULTIPLIED;
      break;
   case VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR:
      v->alpha_mode = DXGI_ALPHA_MODE_STRAIGHT;
      break;
   default:
      /* OPAQUE and INHERIT: composition swapchains take IGNORE. */
      v->alpha_mode = DXGI_ALPHA_MODE_IGNORE;
      break;
   }

   if (mtx_init(&v->mutex, mtx_plain) != thrd_success) {
      InterlockedIncrement(&helios_vehicle_create_fails);
      v->state = WSI_VEHICLE_FAILED;
      return;
   }
   if (cnd_init(&v->cond) != thrd_success) {
      mtx_destroy(&v->mutex);
      InterlockedIncrement(&helios_vehicle_create_fails);
      v->state = WSI_VEHICLE_FAILED;
      return;
   }

   v->state = WSI_VEHICLE_INIT;
   if (thrd_create(&v->thread, wsi_win32_vehicle_thread, chain) !=
       thrd_success) {
      helios_wsi_vehicle_diag("REFUSED chain=%p thrd_create failed",
                              (void *)chain);
      InterlockedIncrement(&helios_vehicle_create_fails);
      cnd_destroy(&v->cond);
      mtx_destroy(&v->mutex);
      v->state = WSI_VEHICLE_FAILED;
      return;
   }
   v->thread_started = true;
}

/* Destroy-side teardown: signal stop, join (the worker releases the COM
 * stack on its own thread), then destroy the sync primitives. Must run
 * BEFORE the chain memory is freed — the worker dereferences the chain. */
static void
wsi_win32_vehicle_finish(struct wsi_win32_swapchain *chain)
{
   struct wsi_win32_vehicle *v = &chain->vehicle;

   if (!v->thread_started)
      return;

   mtx_lock(&v->mutex);
   v->stop = true;
   cnd_signal(&v->cond);
   mtx_unlock(&v->mutex);

   thrd_join(v->thread, NULL);

   cnd_destroy(&v->cond);
   mtx_destroy(&v->mutex);
   v->thread_started = false;
}

VKAPI_ATTR VkBool32 VKAPI_CALL
wsi_GetPhysicalDeviceWin32PresentationSupportKHR(VkPhysicalDevice physicalDevice,
                                                 uint32_t queueFamilyIndex)
{
   VK_FROM_HANDLE(vk_physical_device, pdevice, physicalDevice);
   struct wsi_device *wsi_device = pdevice->wsi_device;
   return (wsi_device->queue_supports_blit & BITFIELD64_BIT(queueFamilyIndex)) != 0;
}

VKAPI_ATTR VkResult VKAPI_CALL
wsi_CreateWin32SurfaceKHR(VkInstance _instance,
                          const VkWin32SurfaceCreateInfoKHR *pCreateInfo,
                          const VkAllocationCallbacks *pAllocator,
                          VkSurfaceKHR *pSurface)
{
   VK_FROM_HANDLE(vk_instance, instance, _instance);
   wsi_win32_surface *surface;

   assert(pCreateInfo->sType == VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR);

   surface = (wsi_win32_surface *)vk_zalloc2(&instance->alloc, pAllocator, sizeof(*surface), 8,
                        VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);

   if (surface == NULL)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   surface->base.base.platform = VK_ICD_WSI_PLATFORM_WIN32;

   surface->base.hinstance = pCreateInfo->hinstance;
   surface->base.hwnd = pCreateInfo->hwnd;

   *pSurface = VkIcdSurfaceBase_to_handle(&surface->base.base);

   return VK_SUCCESS;
}

void
wsi_win32_surface_destroy(VkIcdSurfaceBase *icd_surface, VkInstance _instance,
                          const VkAllocationCallbacks *pAllocator)
{
   VK_FROM_HANDLE(vk_instance, instance, _instance);
   wsi_win32_surface *surface = (wsi_win32_surface *)icd_surface;
   if (surface->visual)
      surface->visual->Release();
   if (surface->target)
      surface->target->Release();
   vk_free2(&instance->alloc, pAllocator, icd_surface);
}

static VkResult
wsi_win32_surface_get_support(VkIcdSurfaceBase *surface,
                              struct wsi_device *wsi_device,
                              uint32_t queueFamilyIndex,
                              VkBool32* pSupported)
{
   *pSupported = true;

   return VK_SUCCESS;
}

static VkResult
wsi_win32_surface_get_capabilities(VkIcdSurfaceBase *surf,
                                   struct wsi_device *wsi_device,
                                   VkSurfaceCapabilitiesKHR* caps)
{
   VkIcdSurfaceWin32 *surface = (VkIcdSurfaceWin32 *)surf;

   RECT win_rect;
   if (!GetClientRect(surface->hwnd, &win_rect))
      return VK_ERROR_SURFACE_LOST_KHR;

   caps->minImageCount = 1;

   if (!wsi_device->sw && wsi_device->win32.get_d3d12_command_queue) {
      /* DXGI doesn't support random presenting order (images need to
       * be presented in the order they were acquired), so we can't
       * expose more than two image per swapchain.
       */
      caps->minImageCount = caps->maxImageCount = 2;
   } else {
      caps->minImageCount = 1;
      /* Software callbacke, there is no real maximum */
      caps->maxImageCount = 0;
   }

   caps->currentExtent = {
      (uint32_t)win_rect.right - (uint32_t)win_rect.left,
      (uint32_t)win_rect.bottom - (uint32_t)win_rect.top
   };
   caps->minImageExtent = { 1u, 1u };
   caps->maxImageExtent = {
      wsi_device->maxImageDimension2D,
      wsi_device->maxImageDimension2D,
   };

   caps->supportedTransforms = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
   caps->currentTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
   caps->maxImageArrayLayers = 1;

   caps->supportedCompositeAlpha =
      VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR |
      VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR |
      VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR;

   caps->supportedUsageFlags = wsi_caps_get_image_usage();

   VK_FROM_HANDLE(vk_physical_device, pdevice, wsi_device->pdevice);
   if (pdevice->supported_extensions.EXT_attachment_feedback_loop_layout)
      caps->supportedUsageFlags |= VK_IMAGE_USAGE_ATTACHMENT_FEEDBACK_LOOP_BIT_EXT;

   return VK_SUCCESS;
}

static VkResult
wsi_win32_surface_get_capabilities2(VkIcdSurfaceBase *surface,
                                    struct wsi_device *wsi_device,
                                    const void *info_next,
                                    VkSurfaceCapabilities2KHR* caps)
{
   assert(caps->sType == VK_STRUCTURE_TYPE_SURFACE_CAPABILITIES_2_KHR);

   const VkSurfacePresentModeKHR *present_mode =
      (const VkSurfacePresentModeKHR *)vk_find_struct_const(info_next, SURFACE_PRESENT_MODE_KHR);

   VkResult result =
      wsi_win32_surface_get_capabilities(surface, wsi_device,
                                      &caps->surfaceCapabilities);

   vk_foreach_struct(ext, caps->pNext) {
      switch (ext->sType) {
      case VK_STRUCTURE_TYPE_SURFACE_PROTECTED_CAPABILITIES_KHR: {
         VkSurfaceProtectedCapabilitiesKHR *protected_cap = (VkSurfaceProtectedCapabilitiesKHR *)ext;
         protected_cap->supportsProtected = VK_FALSE;
         break;
      }

      case VK_STRUCTURE_TYPE_SURFACE_PRESENT_SCALING_CAPABILITIES_KHR: {
         /* Unsupported. */
         VkSurfacePresentScalingCapabilitiesEXT *scaling =
            (VkSurfacePresentScalingCapabilitiesEXT *)ext;
         scaling->supportedPresentScaling = 0;
         scaling->supportedPresentGravityX = 0;
         scaling->supportedPresentGravityY = 0;
         scaling->minScaledImageExtent = caps->surfaceCapabilities.minImageExtent;
         scaling->maxScaledImageExtent = caps->surfaceCapabilities.maxImageExtent;
         break;
      }

      case VK_STRUCTURE_TYPE_SURFACE_PRESENT_MODE_COMPATIBILITY_KHR: {
         /* Unsupported, just report the input present mode. */
         VkSurfacePresentModeCompatibilityKHR *compat =
            (VkSurfacePresentModeCompatibilityKHR *)ext;
         if (compat->pPresentModes) {
            if (compat->presentModeCount) {
               assert(present_mode);
               compat->pPresentModes[0] = present_mode->presentMode;
               compat->presentModeCount = 1;
            }
         } else {
            if (!present_mode)
               wsi_common_vk_warn_once("Use of VkSurfacePresentModeCompatibilityKHR "
                                       "without a VkSurfacePresentModeKHR set. This is an "
                                       "application bug.\n");
            compat->presentModeCount = 1;
         }
         break;
      }

      case VK_STRUCTURE_TYPE_PRESENT_TIMING_SURFACE_CAPABILITIES_EXT: {
         VkPresentTimingSurfaceCapabilitiesEXT *wait = (VkPresentTimingSurfaceCapabilitiesEXT *)ext;

         wait->presentStageQueries = 0;
         wait->presentTimingSupported = VK_FALSE;
         wait->presentAtAbsoluteTimeSupported = VK_FALSE;
         wait->presentAtRelativeTimeSupported = VK_FALSE;
         break;
      }

      default:
         /* Ignored */
         break;
      }
   }

   return result;
}


static const struct {
   VkFormat     format;
} available_surface_formats[] = {
   { VK_FORMAT_B8G8R8A8_UNORM },
   { VK_FORMAT_R8G8B8A8_UNORM },
   { VK_FORMAT_B8G8R8A8_SRGB },
};


static void
get_sorted_vk_formats(struct wsi_device *wsi_device, VkFormat *sorted_formats)
{
   for (unsigned i = 0; i < ARRAY_SIZE(available_surface_formats); i++)
      sorted_formats[i] = available_surface_formats[i].format;

   if (wsi_device->force_bgra8_unorm_first) {
      for (unsigned i = 0; i < ARRAY_SIZE(available_surface_formats); i++) {
         if (sorted_formats[i] == VK_FORMAT_B8G8R8A8_UNORM) {
            sorted_formats[i] = sorted_formats[0];
            sorted_formats[0] = VK_FORMAT_B8G8R8A8_UNORM;
            break;
         }
      }
   }
}

static VkResult
wsi_win32_surface_get_formats(VkIcdSurfaceBase *icd_surface,
                              struct wsi_device *wsi_device,
                              uint32_t* pSurfaceFormatCount,
                              VkSurfaceFormatKHR* pSurfaceFormats)
{
   VK_OUTARRAY_MAKE_TYPED(VkSurfaceFormatKHR, out, pSurfaceFormats, pSurfaceFormatCount);

   VkFormat sorted_formats[ARRAY_SIZE(available_surface_formats)];
   get_sorted_vk_formats(wsi_device, sorted_formats);

   for (unsigned i = 0; i < ARRAY_SIZE(sorted_formats); i++) {
      vk_outarray_append_typed(VkSurfaceFormatKHR, &out, f) {
         f->format = sorted_formats[i];
         f->colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
      }
   }

   return vk_outarray_status(&out);
}

static VkResult
wsi_win32_surface_get_formats2(VkIcdSurfaceBase *icd_surface,
                               struct wsi_device *wsi_device,
                               const void *info_next,
                               uint32_t* pSurfaceFormatCount,
                               VkSurfaceFormat2KHR* pSurfaceFormats)
{
   VK_OUTARRAY_MAKE_TYPED(VkSurfaceFormat2KHR, out, pSurfaceFormats, pSurfaceFormatCount);

   VkFormat sorted_formats[ARRAY_SIZE(available_surface_formats)];
   get_sorted_vk_formats(wsi_device, sorted_formats);

   for (unsigned i = 0; i < ARRAY_SIZE(sorted_formats); i++) {
      vk_outarray_append_typed(VkSurfaceFormat2KHR, &out, f) {
         assert(f->sType == VK_STRUCTURE_TYPE_SURFACE_FORMAT_2_KHR);
         f->surfaceFormat.format = sorted_formats[i];
         f->surfaceFormat.colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
      }
   }

   return vk_outarray_status(&out);
}

static const VkPresentModeKHR present_modes_gdi[] = {
   VK_PRESENT_MODE_FIFO_KHR,
};
static const VkPresentModeKHR present_modes_dxgi[] = {
   VK_PRESENT_MODE_IMMEDIATE_KHR,
   VK_PRESENT_MODE_MAILBOX_KHR,
   VK_PRESENT_MODE_FIFO_KHR,
};

static VkResult
wsi_win32_surface_get_present_modes(VkIcdSurfaceBase *surface,
                                    struct wsi_device *wsi_device,
                                    uint32_t* pPresentModeCount,
                                    VkPresentModeKHR* pPresentModes)
{
   const VkPresentModeKHR *array;
   size_t array_size;
   if (wsi_device->sw || !wsi_device->win32.get_d3d12_command_queue) {
      array = present_modes_gdi;
      array_size = ARRAY_SIZE(present_modes_gdi);
   } else {
      array = present_modes_dxgi;
      array_size = ARRAY_SIZE(present_modes_dxgi);
   }

   if (pPresentModes == NULL) {
      *pPresentModeCount = array_size;
      return VK_SUCCESS;
   }

   *pPresentModeCount = MIN2(*pPresentModeCount, array_size);
   typed_memcpy(pPresentModes, array, *pPresentModeCount);

   if (*pPresentModeCount < array_size)
      return VK_INCOMPLETE;
   else
      return VK_SUCCESS;
}

static VkResult
wsi_win32_surface_get_present_rectangles(VkIcdSurfaceBase *surface,
                                      struct wsi_device *wsi_device,
                                      uint32_t* pRectCount,
                                      VkRect2D* pRects)
{
   VK_OUTARRAY_MAKE_TYPED(VkRect2D, out, pRects, pRectCount);

   vk_outarray_append_typed(VkRect2D, &out, rect) {
      /* We don't know a size so just return the usual "I don't know." */
      *rect = {
         { 0, 0 },
         { UINT32_MAX, UINT32_MAX },
      };
   }

   return vk_outarray_status(&out);
}

static VkResult
wsi_create_dxgi_image_mem(const struct wsi_swapchain *drv_chain,
                          const struct wsi_image_info *info,
                          struct wsi_image *image)
{
   struct wsi_win32_swapchain *chain = (struct wsi_win32_swapchain *)drv_chain;
   const struct wsi_device *wsi = chain->base.wsi;

   assert(chain->base.blit.type != WSI_SWAPCHAIN_BUFFER_BLIT);

   struct wsi_win32_image *win32_image =
      container_of(image, struct wsi_win32_image, base);
   uint32_t image_idx =
      ((uintptr_t)win32_image - (uintptr_t)chain->images) /
      sizeof(*win32_image);
   if (FAILED(chain->dxgi->GetBuffer(image_idx,
                                     IID_PPV_ARGS(&win32_image->dxgi.swapchain_res))))
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;

   VkResult result =
      wsi->win32.create_image_memory(chain->base.device,
                                     win32_image->dxgi.swapchain_res,
                                     &chain->base.alloc,
                                     chain->base.blit.type == WSI_SWAPCHAIN_NO_BLIT ?
                                     &image->memory : &image->blit.memory);
   if (result != VK_SUCCESS)
      return result;

   if (chain->base.blit.type == WSI_SWAPCHAIN_NO_BLIT)
      return VK_SUCCESS;

   VkImageCreateInfo create = info->create;

   create.usage &= ~VK_IMAGE_USAGE_STORAGE_BIT;
   create.initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

   result = wsi->CreateImage(chain->base.device, &create,
                             &chain->base.alloc, &image->blit.image);
   if (result != VK_SUCCESS)
      return result;

   result = wsi->BindImageMemory(chain->base.device, image->blit.image,
                                 image->blit.memory, 0);
   if (result != VK_SUCCESS)
      return result;

   VkMemoryRequirements reqs;
   wsi->GetImageMemoryRequirements(chain->base.device, image->image, &reqs);

   const VkMemoryDedicatedAllocateInfo memory_dedicated_info = {
      VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO,
      nullptr,
      image->blit.image,
      VK_NULL_HANDLE,
   };
   const VkMemoryAllocateInfo memory_info = {
      VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
      &memory_dedicated_info,
      reqs.size,
      info->select_image_memory_type(wsi, reqs.memoryTypeBits),
   };

   return wsi->AllocateMemory(chain->base.device, &memory_info,
                              &chain->base.alloc, &image->memory);
}

enum wsi_swapchain_blit_type
wsi_dxgi_image_needs_blit(const struct wsi_device *wsi,
                          const struct wsi_dxgi_image_params *params,
                          VkDevice device)
{
   if (wsi->win32.requires_blits && wsi->win32.requires_blits(device))
      return WSI_SWAPCHAIN_IMAGE_BLIT;
   else if (params->storage_image)
      return WSI_SWAPCHAIN_IMAGE_BLIT;
   return WSI_SWAPCHAIN_NO_BLIT;
}

VkResult
wsi_dxgi_configure_image(const struct wsi_swapchain *chain,
                         const VkSwapchainCreateInfoKHR *pCreateInfo,
                         const struct wsi_dxgi_image_params *params,
                         struct wsi_image_info *info)
{
   VkResult result =
      wsi_configure_image(chain, pCreateInfo, 0, info);
   if (result != VK_SUCCESS)
      return result;

   info->create_mem = wsi_create_dxgi_image_mem;

   if (chain->blit.type != WSI_SWAPCHAIN_NO_BLIT) {
      wsi_configure_image_blit_image(chain, info);
      info->select_image_memory_type = wsi_select_device_memory_type;
      info->select_blit_dst_memory_type = wsi_select_device_memory_type;
   }

   return VK_SUCCESS;
}

static VkResult
wsi_win32_image_init(VkDevice device_h,
                     struct wsi_win32_swapchain *chain,
                     const VkSwapchainCreateInfoKHR *create_info,
                     const VkAllocationCallbacks *allocator,
                     struct wsi_win32_image *image)
{
   VkResult result = wsi_create_image(&chain->base, &chain->base.image_info,
                                      &image->base);
   if (result != VK_SUCCESS)
      return result;

   VkIcdSurfaceWin32 *win32_surface = (VkIcdSurfaceWin32 *)create_info->surface;
   chain->wnd = win32_surface->hwnd;
   image->chain = chain;

   if (chain->dxgi)
      return VK_SUCCESS;

   HDC wnd_dc = GetDC(chain->wnd);
   if (!wnd_dc)
      return VK_ERROR_SURFACE_LOST_KHR;

   image->sw.dc = CreateCompatibleDC(wnd_dc);
   ReleaseDC(chain->wnd, wnd_dc);
   if (!image->sw.dc)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   HBITMAP bmp = NULL;

   BITMAPINFO info = { 0 };
   info.bmiHeader.biSize = sizeof(BITMAPINFO);
   info.bmiHeader.biWidth = create_info->imageExtent.width;
   info.bmiHeader.biHeight = -create_info->imageExtent.height;
   info.bmiHeader.biPlanes = 1;
   info.bmiHeader.biBitCount = 32;
   info.bmiHeader.biCompression = BI_RGB;

   bmp = CreateDIBSection(image->sw.dc, &info, DIB_RGB_COLORS, &image->sw.ppvBits, NULL, 0);
   if (!bmp || !image->sw.ppvBits)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   SelectObject(image->sw.dc, bmp);

   BITMAP header;
   int status = GetObject(bmp, sizeof(BITMAP), &header);
   (void)status;
   image->sw.bmp_row_pitch = header.bmWidthBytes;
   image->sw.bmp = bmp;

   return VK_SUCCESS;
}

static void
wsi_win32_image_finish(struct wsi_win32_swapchain *chain,
                       const VkAllocationCallbacks *allocator,
                       struct wsi_win32_image *image)
{
   if (image->dxgi.swapchain_res)
      image->dxgi.swapchain_res->Release();

   if (image->sw.dc)
      DeleteDC(image->sw.dc);
   if(image->sw.bmp)
      DeleteObject(image->sw.bmp);
   wsi_destroy_image(&chain->base, &image->base);
}

static VkResult
wsi_win32_swapchain_destroy(struct wsi_swapchain *drv_chain,
                            const VkAllocationCallbacks *allocator)
{
   struct wsi_win32_swapchain *chain =
      (struct wsi_win32_swapchain *) drv_chain;

   /* First: the vehicle worker dereferences the chain — stop and join it
    * before anything else is torn down. */
   wsi_win32_vehicle_finish(chain);

   for (uint32_t i = 0; i < chain->base.image_count; i++)
      wsi_win32_image_finish(chain, allocator, &chain->images[i]);

   if (chain->surface->current_swapchain == chain)
      chain->surface->current_swapchain = NULL;

   if (chain->dxgi)
      chain->dxgi->Release();

   wsi_swapchain_finish(&chain->base);

   u_cnd_monotonic_destroy(&chain->acquire_cond);
   mtx_destroy(&chain->acquire_mutex);

   vk_free(allocator, chain);
   return VK_SUCCESS;
}

static struct wsi_image *
wsi_win32_get_wsi_image(struct wsi_swapchain *drv_chain,
                        uint32_t image_index)
{
   struct wsi_win32_swapchain *chain =
      (struct wsi_win32_swapchain *) drv_chain;

   return &chain->images[image_index].base;
}

static void
wsi_win32_set_image_idle(struct wsi_win32_swapchain *chain,
                         struct wsi_win32_image *image)
{
   if (!chain->dxgi)
      mtx_lock(&chain->acquire_mutex);

   image->state = WSI_IMAGE_IDLE;

   if (!chain->dxgi) {
      u_cnd_monotonic_broadcast(&chain->acquire_cond);
      mtx_unlock(&chain->acquire_mutex);
   }
}

static VkResult
wsi_win32_release_images(struct wsi_swapchain *drv_chain,
                         uint32_t count, const uint32_t *indices)
{
   struct wsi_win32_swapchain *chain =
      (struct wsi_win32_swapchain *)drv_chain;

   if (chain->status == VK_ERROR_SURFACE_LOST_KHR)
      return chain->status;

   for (uint32_t i = 0; i < count; i++) {
      uint32_t index = indices[i];
      assert(index < chain->base.image_count);
      assert(chain->images[index].state == WSI_IMAGE_DRAWING);
      wsi_win32_set_image_idle(chain, &chain->images[index]);
   }

   return VK_SUCCESS;
}

static bool
wsi_win32_find_idle_image(struct wsi_win32_swapchain *chain,
                          uint32_t *out_image_index)
{
   for (uint32_t i = 0; i < chain->base.image_count; i++) {
      if (chain->images[i].state == WSI_IMAGE_IDLE) {
         *out_image_index = i;
         chain->images[i].state = WSI_IMAGE_DRAWING;
         return true;
      }
   }
   return false;
}

static VkResult
wsi_win32_acquire_idle_cpu_image_locked(struct wsi_win32_swapchain *chain,
                                        const VkAcquireNextImageInfoKHR *info,
                                        uint32_t *out_image_index)
{
   if (wsi_win32_find_idle_image(chain, out_image_index))
      return VK_SUCCESS;

   if (info->timeout == 0)
      return VK_NOT_READY;

   const uint64_t abs_timeout = os_time_get_absolute_timeout(info->timeout);
   struct timespec abs_timespec;
   timespec_from_nsec(&abs_timespec, abs_timeout);
   do {
      int ret = u_cnd_monotonic_timedwait(
         &chain->acquire_cond, &chain->acquire_mutex, &abs_timespec);
      if (ret == thrd_timedout)
         return VK_TIMEOUT;
      else if (ret != thrd_success)
         return VK_ERROR_OUT_OF_DATE_KHR;
   } while (!wsi_win32_find_idle_image(chain, out_image_index));

   return VK_SUCCESS;
}

static inline VkResult
wsi_win32_acquire_idle_cpu_image(struct wsi_win32_swapchain *chain,
                                 const VkAcquireNextImageInfoKHR *info,
                                 uint32_t *out_image_index)
{
   mtx_lock(&chain->acquire_mutex);
   VkResult result = wsi_win32_acquire_idle_cpu_image_locked(chain, info,
                                                             out_image_index);
   mtx_unlock(&chain->acquire_mutex);
   return result;
}

static VkResult
wsi_win32_acquire_next_image(struct wsi_swapchain *drv_chain,
                             const VkAcquireNextImageInfoKHR *info,
                             uint32_t *image_index)
{
   struct wsi_win32_swapchain *chain =
      (struct wsi_win32_swapchain *)drv_chain;

   /* Bail early if the swapchain is broken */
   if (chain->status != VK_SUCCESS)
      return chain->status;

   /* acquire timeout has to be explicitly handled for sw wsi */
   if (!chain->dxgi)
      return wsi_win32_acquire_idle_cpu_image(chain, info, image_index);

   if (wsi_win32_find_idle_image(chain, image_index))
      return VK_SUCCESS;

   assert(chain->dxgi);
   uint32_t index = chain->dxgi->GetCurrentBackBufferIndex();
   if (chain->images[index].state == WSI_IMAGE_DRAWING) {
      index = (index + 1) % chain->base.image_count;
      assert(chain->images[index].state == WSI_IMAGE_QUEUED);
   }
   if (chain->wsi->wsi->WaitForFences(chain->base.device, 1,
                                      &chain->base.fences[index],
                                      false, info->timeout) != VK_SUCCESS)
      return VK_TIMEOUT;

   *image_index = index;
   chain->images[index].state = WSI_IMAGE_DRAWING;
   return VK_SUCCESS;
}

static VkResult
wsi_win32_queue_present_dxgi(struct wsi_win32_swapchain *chain,
                             struct wsi_win32_image *image,
                             const VkPresentRegionKHR *damage)
{
   uint32_t rect_count = damage ? damage->rectangleCount : 0;
   STACK_ARRAY(RECT, rects, rect_count);

   for (uint32_t r = 0; r < rect_count; r++) {
      rects[r].left = damage->pRectangles[r].offset.x;
      rects[r].top = damage->pRectangles[r].offset.y;
      rects[r].right = damage->pRectangles[r].offset.x + damage->pRectangles[r].extent.width;
      rects[r].bottom = damage->pRectangles[r].offset.y + damage->pRectangles[r].extent.height;
   }

   DXGI_PRESENT_PARAMETERS params = {
      rect_count,
      rects,
   };

   image->state = WSI_IMAGE_QUEUED;
   UINT sync_interval = chain->base.present_mode == VK_PRESENT_MODE_FIFO_KHR ? 1 : 0;
   UINT present_flags = chain->base.present_mode == VK_PRESENT_MODE_IMMEDIATE_KHR ?
      DXGI_PRESENT_ALLOW_TEARING : 0;

   HRESULT hres = chain->dxgi->Present1(sync_interval, present_flags, &params);
   switch (hres) {
   case DXGI_ERROR_DEVICE_REMOVED: return VK_ERROR_DEVICE_LOST;
   case E_OUTOFMEMORY: return VK_ERROR_OUT_OF_DEVICE_MEMORY;
   default:
      if (FAILED(hres))
         return VK_ERROR_OUT_OF_HOST_MEMORY;
      break;
   }

   if (chain->surface->current_swapchain != chain) {
      chain->surface->visual->SetContent(chain->dxgi);
      chain->wsi->dxgi.dcomp->Commit();
      chain->surface->current_swapchain = chain;
   }

   /* Mark the other image idle */
   chain->status = VK_SUCCESS;
   return VK_SUCCESS;
}

static VkResult
wsi_win32_queue_present(struct wsi_swapchain *drv_chain,
                        uint32_t image_index,
                        uint64_t present_id,
                        const VkPresentRegionKHR *damage)
{
   struct wsi_win32_swapchain *chain = (struct wsi_win32_swapchain *) drv_chain;
   assert(image_index < chain->base.image_count);
   struct wsi_win32_image *image = &chain->images[image_index];

   assert(image->state == WSI_IMAGE_DRAWING);

   if (chain->dxgi)
      return wsi_win32_queue_present_dxgi(chain, image, damage);

   const uint32_t src_row_pitch = image->base.row_pitches[0];
   const bool can_present_cpu_map_directly =
      helios_win32_wsi_direct_map_enabled() &&
      image->base.cpu_map &&
      src_row_pitch % 4 == 0 &&
      src_row_pitch / 4 >= chain->extent.width &&
      src_row_pitch / 4 <= LONG_MAX;

   const void *present_bits = image->base.cpu_map;
   LONG present_bitmap_width = (LONG)(src_row_pitch / 4);
   bool present_from_shadow_dib = false;
   uint64_t helios_copy_ns = 0;
   uint64_t helios_get_dc_ns = 0;
   uint64_t helios_stretch_ns = 0;

   if (!can_present_cpu_map_directly) {
      uint64_t helios_start_ns = os_time_get_nano();
      char *ptr = (char *)image->base.cpu_map;
      char *dptr = (char *)image->sw.ppvBits;

      for (unsigned h = 0; h < chain->extent.height; h++) {
         memcpy(dptr, ptr, chain->extent.width * 4);
         dptr += image->sw.bmp_row_pitch;
         ptr += src_row_pitch;
      }

      present_bits = image->sw.ppvBits;
      present_bitmap_width = (LONG)chain->extent.width;
      present_from_shadow_dib = true;
      helios_copy_ns = os_time_get_nano() - helios_start_ns;
   }

   uint64_t helios_start_ns = os_time_get_nano();
   HDC wnd_dc = GetDC(chain->wnd);
   helios_get_dc_ns = os_time_get_nano() - helios_start_ns;
   if (!wnd_dc) {
      chain->status = VK_ERROR_SURFACE_LOST_KHR;
      return chain->status;
   }

   BITMAPINFO info = { 0 };
   info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
   info.bmiHeader.biWidth = present_bitmap_width;
   info.bmiHeader.biHeight = -(LONG)chain->extent.height;
   info.bmiHeader.biPlanes = 1;
   info.bmiHeader.biBitCount = 32;
   info.bmiHeader.biCompression = BI_RGB;

   helios_start_ns = os_time_get_nano();
   int copied;
   if (present_from_shadow_dib) {
      copied = BitBlt(wnd_dc, 0, 0, chain->extent.width, chain->extent.height,
                      image->sw.dc, 0, 0, SRCCOPY) ? (int)chain->extent.height : 0;
   } else {
      copied = StretchDIBits(wnd_dc, 0, 0, chain->extent.width,
                             chain->extent.height, 0, 0, chain->extent.width,
                             chain->extent.height, present_bits, &info,
                             DIB_RGB_COLORS, SRCCOPY);
   }
   helios_stretch_ns = os_time_get_nano() - helios_start_ns;
   if (copied == 0 || copied == (int)GDI_ERROR) {
      fprintf(stderr,
              "wsi/win32: GDI present failed, ret=%d, GetLastError=%lu, dst=%p, extent=%ux%u\n",
              copied, (unsigned long)GetLastError(), wnd_dc,
              chain->extent.width, chain->extent.height);
      chain->status = VK_ERROR_MEMORY_MAP_FAILED;
   }
   ReleaseDC(chain->wnd, wnd_dc);
   helios_win32_wsi_perf_note_frame(can_present_cpu_map_directly,
                                    helios_copy_ns, helios_get_dc_ns,
                                    helios_stretch_ns);

   wsi_win32_set_image_idle(chain, image);

   return chain->status;
}

static VkResult
wsi_win32_surface_create_swapchain_dxgi(
   wsi_win32_surface *surface,
   VkDevice device,
   struct wsi_win32 *wsi,
   const VkSwapchainCreateInfoKHR *create_info,
   struct wsi_win32_swapchain *chain)
{
   IDXGIFactory4 *factory = wsi->dxgi.factory;
   ID3D12CommandQueue *queue =
      (ID3D12CommandQueue *)wsi->wsi->win32.get_d3d12_command_queue(device);

   DXGI_ALPHA_MODE alpha_mode;
   switch (create_info->compositeAlpha) {
   case VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR:
      alpha_mode = DXGI_ALPHA_MODE_IGNORE;
      break;
   case VK_COMPOSITE_ALPHA_PRE_MULTIPLIED_BIT_KHR:
      alpha_mode = DXGI_ALPHA_MODE_PREMULTIPLIED;
      break;
   case VK_COMPOSITE_ALPHA_POST_MULTIPLIED_BIT_KHR:
      alpha_mode = DXGI_ALPHA_MODE_STRAIGHT;
      break;
   default:
      alpha_mode = DXGI_ALPHA_MODE_UNSPECIFIED;
      break;
   }

   DXGI_SWAP_CHAIN_DESC1 desc = {
      create_info->imageExtent.width,
      create_info->imageExtent.height,
      DXGI_FORMAT_B8G8R8A8_UNORM,
      create_info->imageArrayLayers > 1,  // Stereo
      { 1 },                              // SampleDesc
      0,                                  // Usage (filled in below)
      create_info->minImageCount,
      DXGI_SCALING_STRETCH,
      DXGI_SWAP_EFFECT_FLIP_SEQUENTIAL,
      alpha_mode,
      chain->base.present_mode == VK_PRESENT_MODE_IMMEDIATE_KHR ?
         DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0u
   };

   if (create_info->imageUsage &
       (VK_IMAGE_USAGE_SAMPLED_BIT |
        VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT))
      desc.BufferUsage |= DXGI_USAGE_SHADER_INPUT;

   if (create_info->imageUsage & VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
      desc.BufferUsage |= DXGI_USAGE_RENDER_TARGET_OUTPUT;

   IDXGISwapChain1 *swapchain1;
   if (FAILED(factory->CreateSwapChainForComposition(queue, &desc, NULL, &swapchain1)) ||
       FAILED(swapchain1->QueryInterface(&chain->dxgi)))
      return VK_ERROR_INITIALIZATION_FAILED;

   swapchain1->Release();

   if (!surface->target &&
       FAILED(wsi->dxgi.dcomp->CreateTargetForHwnd(surface->base.hwnd, false, &surface->target)))
      return VK_ERROR_INITIALIZATION_FAILED;

   if (!surface->visual) {
      if (FAILED(wsi->dxgi.dcomp->CreateVisual(&surface->visual)) ||
          FAILED(surface->target->SetRoot(surface->visual)) ||
          FAILED(surface->visual->SetContent(chain->dxgi)) ||
          FAILED(wsi->dxgi.dcomp->Commit()))
         return VK_ERROR_INITIALIZATION_FAILED;

      surface->current_swapchain = chain;
   }
   return VK_SUCCESS;
}

static VkResult
wsi_win32_surface_create_swapchain(
   VkIcdSurfaceBase *icd_surface,
   VkDevice device,
   struct wsi_device *wsi_device,
   const VkSwapchainCreateInfoKHR *create_info,
   const VkAllocationCallbacks *allocator,
   struct wsi_swapchain **swapchain_out)
{
   wsi_win32_surface *surface = (wsi_win32_surface *)icd_surface;
   struct wsi_win32 *wsi =
      (struct wsi_win32 *) wsi_device->wsi[VK_ICD_WSI_PLATFORM_WIN32];

   assert(create_info->sType == VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR);

   /* Helios async present: with the fence-wait + blit on the worker thread,
    * the swapchain depth is the app's run-ahead budget — at the requested 2-3
    * images the app stalls in acquire before the GPU can pipeline and the
    * worker's fence wait degenerates to a full GPU-frame-time serialization
    * (Doom: 160 fps). Creating MORE images than minImageCount is spec-legal
    * (apps must query vkGetSwapchainImagesKHR), but PROVEN app-hostile:
    * idTech sizes its per-image arrays to the count it requested — an
    * unconditional bump to 5 crashed Doom at renderer init with an unhandled
    * C++ FatalError (2026-07-06, Crash.00003/00004). Opt-in only:
    * HELIOS_WSI_EXTRA_IMAGES=N adds N images for engines known to re-query. */
   unsigned num_images = create_info->minImageCount;
   if (wsi_device->sw && wsi_helios_async_present_enabled()) {
      static int extra_images = -1;
      if (extra_images < 0) {
         char extra_env[8] = "";
         GetEnvironmentVariableA("HELIOS_WSI_EXTRA_IMAGES", extra_env,
                                 sizeof(extra_env));
         extra_images = (extra_env[0] >= '0' && extra_env[0] <= '9')
                           ? atoi(extra_env) : 0;
         extra_images = CLAMP(extra_images, 0, 8);
      }
      num_images += (unsigned)extra_images;
   }
   struct wsi_win32_swapchain *chain;
   size_t size = sizeof(*chain) + num_images * sizeof(chain->images[0]);

   chain = (wsi_win32_swapchain *)vk_zalloc(allocator, size,
                     8, VK_SYSTEM_ALLOCATION_SCOPE_OBJECT);

   if (chain == NULL)
      return VK_ERROR_OUT_OF_HOST_MEMORY;

   int ret = mtx_init(&chain->acquire_mutex, mtx_plain);
   if (ret != thrd_success) {
      vk_free(allocator, chain);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   ret = u_cnd_monotonic_init(&chain->acquire_cond);
   if (ret != thrd_success) {
      mtx_destroy(&chain->acquire_mutex);
      vk_free(allocator, chain);
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   struct wsi_dxgi_image_params dxgi_image_params = {
      { WSI_IMAGE_TYPE_DXGI },
   };
   dxgi_image_params.storage_image = (create_info->imageUsage & VK_IMAGE_USAGE_STORAGE_BIT) != 0;

   struct wsi_cpu_image_params cpu_image_params = {
      { WSI_IMAGE_TYPE_CPU },
   };

   bool supports_dxgi = wsi->dxgi.factory &&
                        wsi->dxgi.dcomp &&
                        wsi->wsi->win32.get_d3d12_command_queue;
   struct wsi_base_image_params *image_params = supports_dxgi ?
      &dxgi_image_params.base : &cpu_image_params.base;

   VkResult result = wsi_swapchain_init(wsi_device, &chain->base, device,
                                        create_info, image_params,
                                        allocator);
   if (result != VK_SUCCESS) {
      u_cnd_monotonic_destroy(&chain->acquire_cond);
      mtx_destroy(&chain->acquire_mutex);
      vk_free(allocator, chain);
      return result;
   }

   chain->base.destroy = wsi_win32_swapchain_destroy;
   chain->base.get_wsi_image = wsi_win32_get_wsi_image;
   chain->base.acquire_next_image = wsi_win32_acquire_next_image;
   chain->base.release_images = wsi_win32_release_images;
   chain->base.queue_present = wsi_win32_queue_present;
   chain->base.present_mode = wsi_swapchain_get_present_mode(wsi_device, create_info);
   chain->extent = create_info->imageExtent;

   chain->wsi = wsi;
   chain->status = VK_SUCCESS;

   chain->surface = surface;

   if (image_params->image_type == WSI_IMAGE_TYPE_DXGI) {
      result = wsi_win32_surface_create_swapchain_dxgi(surface, device, wsi, create_info, chain);
      if (result != VK_SUCCESS)
         goto fail;
   }

   for (uint32_t image = 0; image < num_images; image++) {
      result = wsi_win32_image_init(device, chain,
                                    create_info, allocator,
                                    &chain->images[image]);
      if (result != VK_SUCCESS)
         goto fail;

      chain->base.image_count++;
   }

   /* Helios dcomp present vehicle: kick the dedicated worker (no D3D work on
    * this thread). Until it latches READY, presents flow through the sw
    * path exactly as before. */
   wsi_win32_vehicle_start(chain, create_info);

   *swapchain_out = &chain->base;

   return VK_SUCCESS;

fail:
   if (surface->visual) {
      surface->visual->SetContent(NULL);
      surface->current_swapchain = NULL;
      wsi->dxgi.dcomp->Commit();
   }
   wsi_win32_swapchain_destroy(&chain->base, allocator);
   return result;
}

static IDXGIFactory4 *
dxgi_get_factory(bool debug)
{
   HMODULE dxgi_mod = LoadLibraryA("DXGI.DLL");
   if (!dxgi_mod) {
      return NULL;
   }

   typedef HRESULT(WINAPI *PFN_CREATE_DXGI_FACTORY2)(UINT flags, REFIID riid, void **ppFactory);
   PFN_CREATE_DXGI_FACTORY2 CreateDXGIFactory2;

   CreateDXGIFactory2 = (PFN_CREATE_DXGI_FACTORY2)GetProcAddress(dxgi_mod, "CreateDXGIFactory2");
   if (!CreateDXGIFactory2) {
      return NULL;
   }

   UINT flags = 0;
   if (debug)
      flags |= DXGI_CREATE_FACTORY_DEBUG;

   IDXGIFactory4 *factory;
   HRESULT hr = CreateDXGIFactory2(flags, IID_PPV_ARGS(&factory));
   if (FAILED(hr)) {
      return NULL;
   }

   return factory;
}

static IDCompositionDevice *
dcomp_get_device()
{
   HMODULE dcomp_mod = LoadLibraryA("DComp.DLL");
   if (!dcomp_mod) {
      return NULL;
   }

   typedef HRESULT (STDAPICALLTYPE *PFN_DCOMP_CREATE_DEVICE)(IDXGIDevice *, REFIID, void **);
   PFN_DCOMP_CREATE_DEVICE DCompositionCreateDevice;

   DCompositionCreateDevice = (PFN_DCOMP_CREATE_DEVICE)GetProcAddress(dcomp_mod, "DCompositionCreateDevice");
   if (!DCompositionCreateDevice) {
      return NULL;
   }

   IDCompositionDevice *device;
   HRESULT hr = DCompositionCreateDevice(NULL, IID_PPV_ARGS(&device));
   if (FAILED(hr)) {
      return NULL;
   }

   return device;
}

VkResult
wsi_win32_init_wsi(struct wsi_device *wsi_device,
                   const VkAllocationCallbacks *alloc,
                   VkPhysicalDevice physical_device)
{
   struct wsi_win32 *wsi;
   VkResult result;

   wsi = (wsi_win32 *)vk_zalloc(alloc, sizeof(*wsi), 8,
                   VK_SYSTEM_ALLOCATION_SCOPE_INSTANCE);
   if (!wsi) {
      result = VK_ERROR_OUT_OF_HOST_MEMORY;
      goto fail;
   }

   wsi->physical_device = physical_device;
   wsi->alloc = alloc;
   wsi->wsi = wsi_device;

   if (!wsi_device->sw) {
      wsi->dxgi.factory = dxgi_get_factory(WSI_DEBUG & WSI_DEBUG_DXGI);
      if (!wsi->dxgi.factory) {
         vk_free(alloc, wsi);
         result = VK_ERROR_INITIALIZATION_FAILED;
         goto fail;
      }
      wsi->dxgi.dcomp = dcomp_get_device();
      if (!wsi->dxgi.dcomp) {
         wsi->dxgi.factory->Release();
         vk_free(alloc, wsi);
         result = VK_ERROR_INITIALIZATION_FAILED;
         goto fail;
      }
   }

   wsi->base.get_support = wsi_win32_surface_get_support;
   wsi->base.get_capabilities2 = wsi_win32_surface_get_capabilities2;
   wsi->base.get_formats = wsi_win32_surface_get_formats;
   wsi->base.get_formats2 = wsi_win32_surface_get_formats2;
   wsi->base.get_present_modes = wsi_win32_surface_get_present_modes;
   wsi->base.get_present_rectangles = wsi_win32_surface_get_present_rectangles;
   wsi->base.create_swapchain = wsi_win32_surface_create_swapchain;

   wsi_device->wsi[VK_ICD_WSI_PLATFORM_WIN32] = &wsi->base;

   return VK_SUCCESS;

fail:
   wsi_device->wsi[VK_ICD_WSI_PLATFORM_WIN32] = NULL;

   return result;
}

void
wsi_win32_finish_wsi(struct wsi_device *wsi_device,
                  const VkAllocationCallbacks *alloc)
{
   struct wsi_win32 *wsi =
      (struct wsi_win32 *)wsi_device->wsi[VK_ICD_WSI_PLATFORM_WIN32];
   if (!wsi)
      return;

   if (wsi->dxgi.factory)
      wsi->dxgi.factory->Release();
   if (wsi->dxgi.dcomp)
      wsi->dxgi.dcomp->Release();

   vk_free(alloc, wsi);
}
