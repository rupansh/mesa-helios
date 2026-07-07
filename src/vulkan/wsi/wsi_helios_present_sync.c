/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * WS1 #4 present-sync seqlock table — ICD-side writer. See the header for
 * the contract; the layout below must stay byte-identical to
 * dxvk-helios/src/dxvk/dxvk_helios_present_sync.cpp.
 */

#include "wsi_helios_present_sync.h"

#ifdef _WIN32

#include <stdio.h>
#include <string.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#define HPS_MAGIC 0x31535048u /* 'HPS1' */
#define HPS_SLOT_COUNT 64u

struct hps_header {
   uint32_t magic;
   uint32_t slot_count;
   uint32_t reserved[6];
};

/* 32 bytes; `seq` odd while a write is in flight. `resid` is CAS-claimed
 * once and never returns to 0 while the file lives; stale slots are only
 * recycled via producer-pid liveness. */
struct hps_slot {
   volatile LONG seq;
   uint32_t resid;
   uint32_t pid;
   uint32_t fence_id;
   volatile LONG64 value;
   /* Producer process CREATION TIME (FILETIME as u64): pid liveness alone is
    * wrong for recycling — the table file persists across boots and Windows
    * reuses pids, so a dead producer's slot can look alive forever. A pid
    * whose current creation time differs from this stamp is a reused pid;
    * legacy slots carry 0 here and are recycled on first pressure. Layout
    * mirrored in dxvk-helios/src/dxvk/dxvk_helios_present_sync.cpp. */
   uint64_t producer_start;
};

_Static_assert(sizeof(struct hps_header) == 32, "HPS header ABI");
_Static_assert(sizeof(struct hps_slot) == 32, "HPS slot ABI");

static struct hps_header *hps_header;
static struct hps_slot *hps_slots;
static INIT_ONCE hps_once = INIT_ONCE_STATIC_INIT;

uint32_t
wsi_helios_present_sync_alloc_fence_id(void)
{
   static volatile LONG counter;
   return 0x80000000u | (uint32_t)InterlockedIncrement(&counter);
}

static BOOL CALLBACK
hps_init_mapping(INIT_ONCE *once, void *param, void **context)
{
   (void)once;
   (void)param;
   (void)context;

   char path[MAX_PATH];
   if (!GetEnvironmentVariableA("HELIOS_PRESENT_SYNC_PATH", path,
                                sizeof(path)))
      snprintf(path, sizeof(path),
               "C:\\ProgramData\\Helios\\helios_present_sync.bin");

   HANDLE file =
      CreateFileA(path, GENERIC_READ | GENERIC_WRITE,
                  FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                  NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
   if (file == INVALID_HANDLE_VALUE)
      return TRUE; /* once-init done; table stays unavailable (loud caller) */

   const uint32_t size =
      sizeof(struct hps_header) + HPS_SLOT_COUNT * sizeof(struct hps_slot);

   HANDLE mapping = CreateFileMappingA(file, NULL, PAGE_READWRITE, 0, size,
                                       NULL);
   CloseHandle(file);
   if (!mapping)
      return TRUE;

   void *view = MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
   CloseHandle(mapping); /* the view keeps the section referenced */
   if (!view)
      return TRUE;

   struct hps_header *header = view;

   /* First mapper stamps the header; racers spin briefly on a partial
    * stamp. A wrong-versioned file is refused (never guessed at). */
   LONG prev = InterlockedCompareExchange((volatile LONG *)&header->magic,
                                          (LONG)HPS_MAGIC, 0);
   if (prev != 0 && (uint32_t)prev != HPS_MAGIC) {
      UnmapViewOfFile(view);
      return TRUE;
   }
   if (prev == 0)
      header->slot_count = HPS_SLOT_COUNT;
   for (uint32_t spin = 0; header->slot_count == 0 && spin < 4096; spin++)
      Sleep(0);
   if (header->slot_count != HPS_SLOT_COUNT) {
      UnmapViewOfFile(view);
      return TRUE;
   }

   hps_slots = (struct hps_slot *)(header + 1);
   hps_header = header;
   return TRUE;
}

static uint64_t
hps_self_start(void)
{
   static volatile LONG64 cached;
   LONG64 v = InterlockedCompareExchange64(&cached, 0, 0);
   if (v)
      return (uint64_t)v;
   FILETIME creation = {0}, exit_t = {0}, kernel = {0}, user = {0};
   if (GetProcessTimes(GetCurrentProcess(), &creation, &exit_t, &kernel, &user))
      v = (LONG64)(((uint64_t)creation.dwHighDateTime << 32) |
                   creation.dwLowDateTime);
   InterlockedCompareExchange64(&cached, v, 0);
   return (uint64_t)v;
}

/* A slot's producer is alive iff its pid exists AND the process' creation
 * time matches the stamp the producer wrote — anything else (dead pid,
 * reused pid, legacy zero stamp) makes the slot recyclable. A shielded
 * process (ACCESS_DENIED) is conservatively treated as alive. */
static bool
hps_producer_alive(const struct hps_slot *slot)
{
   const uint32_t pid = slot->pid;
   if (!pid)
      return false;
   HANDLE proc = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
   if (!proc)
      return GetLastError() == ERROR_ACCESS_DENIED; /* exists, shielded */
   DWORD code = 0;
   bool alive = GetExitCodeProcess(proc, &code) && code == STILL_ACTIVE;
   if (alive) {
      FILETIME creation = {0}, exit_t = {0}, kernel = {0}, user = {0};
      if (GetProcessTimes(proc, &creation, &exit_t, &kernel, &user)) {
         const uint64_t start = ((uint64_t)creation.dwHighDateTime << 32) |
                                creation.dwLowDateTime;
         alive = (start == slot->producer_start);
      }
   }
   CloseHandle(proc);
   return alive;
}

static struct hps_slot *
hps_find_slot(uint32_t resid)
{
   for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
      if (hps_slots[i].resid == resid)
         return &hps_slots[i];
   }
   return NULL;
}

static struct hps_slot *
hps_claim_slot(uint32_t resid)
{
   for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
      struct hps_slot *slot = &hps_slots[i];
      if (slot->resid == 0 &&
          InterlockedCompareExchange((volatile LONG *)&slot->resid,
                                     (LONG)resid, 0) == 0)
         return slot;
   }

   /* Recycle a dead/reused-pid producer's slot (its fence name is
    * unresolvable anyway). */
   for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
      struct hps_slot *slot = &hps_slots[i];
      const uint32_t old = slot->resid;
      if (old != 0 && !hps_producer_alive(slot) &&
          InterlockedCompareExchange((volatile LONG *)&slot->resid,
                                     (LONG)resid, (LONG)old) == (LONG)old)
         return slot;
   }

   return NULL;
}

bool
wsi_helios_present_sync_publish(uint32_t resid, uint32_t pid,
                                uint32_t fence_id, uint64_t value)
{
   InitOnceExecuteOnce(&hps_once, hps_init_mapping, NULL, NULL);

   if (!hps_slots || !resid)
      return false;

   struct hps_slot *slot = hps_find_slot(resid);
   if (!slot)
      slot = hps_claim_slot(resid);
   if (!slot)
      return false;

   /* Seqlock write; one producer per resid past the claim. */
   InterlockedIncrement(&slot->seq); /* -> odd */
   slot->pid = pid;
   slot->fence_id = fence_id;
   slot->value = (LONG64)value;
   slot->producer_start = hps_self_start();
   InterlockedIncrement(&slot->seq); /* -> even */
   return true;
}

#endif /* _WIN32 */
