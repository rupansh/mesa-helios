/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * WS1 #4 present-sync seqlock table — ICD-side writer. Layout must stay
 * byte-identical to dxvk-helios/src/dxvk/dxvk_helios_present_sync.cpp.
 */

#include "wsi_helios_present_sync.h"

#ifdef _WIN32

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#define HPS_MAGIC 0x32535048u /* 'HPS2' */
#define HPS_SLOT_COUNT 4096u
#define HPS_MAPPING_BYTES 131104u

struct hps_header {
   uint32_t magic;
   uint32_t slot_count;
   uint32_t reserved[6];
};

struct hps_slot {
   volatile LONG seq;
   uint32_t resid;
   uint32_t pid;
   uint32_t fence_id;
   volatile LONG64 value;
   uint64_t producer_start;
};

_Static_assert(sizeof(struct hps_header) == 32, "HPS header ABI");
_Static_assert(sizeof(struct hps_slot) == 32, "HPS slot ABI");
_Static_assert(sizeof(struct hps_header) +
                  HPS_SLOT_COUNT * sizeof(struct hps_slot) ==
                  HPS_MAPPING_BYTES,
               "HPS mapping ABI");
_Static_assert(sizeof(struct hps_header) % _Alignof(struct hps_slot) == 0,
               "HPS slot alignment");
_Static_assert(offsetof(struct hps_slot, seq) == 0, "HPS seq offset");
_Static_assert(offsetof(struct hps_slot, resid) == sizeof(LONG), "HPS resid offset");
_Static_assert(_Alignof(struct hps_slot) >= _Alignof(LONG64), "HPS pair alignment");

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
               "C:\\ProgramData\\Helios\\helios_present_sync_v2.bin");

   HANDLE file = CreateFileA(path, GENERIC_READ | GENERIC_WRITE,
                             FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                             NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
   if (file == INVALID_HANDLE_VALUE)
      return TRUE;

   HANDLE mapping = CreateFileMappingA(file, NULL, PAGE_READWRITE, 0,
                                       HPS_MAPPING_BYTES, NULL);
   CloseHandle(file);
   if (!mapping)
      return TRUE;

   void *view = MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0,
                              HPS_MAPPING_BYTES);
   CloseHandle(mapping);
   if (!view)
      return TRUE;

   struct hps_header *header = view;
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

uint64_t
wsi_helios_present_sync_process_start(void)
{
   return hps_self_start();
}

static bool
hps_producer_alive(uint32_t pid, uint64_t producer_start)
{
   if (!pid)
      return false;
   HANDLE proc = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
   if (!proc)
      return GetLastError() != ERROR_INVALID_PARAMETER;
   DWORD code = 0;
   if (!GetExitCodeProcess(proc, &code)) {
      CloseHandle(proc);
      return true;
   }
   if (code != STILL_ACTIVE) {
      CloseHandle(proc);
      return false;
   }
   FILETIME creation = {0}, exit_t = {0}, kernel = {0}, user = {0};
   if (!GetProcessTimes(proc, &creation, &exit_t, &kernel, &user)) {
      CloseHandle(proc);
      return true;
   }
   const uint64_t start = ((uint64_t)creation.dwHighDateTime << 32) |
                          creation.dwLowDateTime;
   CloseHandle(proc);
   return start == producer_start;
}

static uint64_t
hps_pack_state(LONG seq, uint32_t resid)
{
   return ((uint64_t)resid << 32) | (uint32_t)seq;
}

static LONG
hps_state_seq(uint64_t state)
{
   return (LONG)(uint32_t)state;
}

static uint32_t
hps_state_resid(uint64_t state)
{
   return (uint32_t)(state >> 32);
}

static uint64_t
hps_read_state(const struct hps_slot *slot)
{
   return (uint64_t)InterlockedCompareExchange64(
      (volatile LONG64 *)(volatile LONG *)&slot->seq, 0, 0);
}

static bool
hps_try_claim(struct hps_slot *slot, uint64_t expected_state,
              uint32_t new_resid, LONG *write_seq)
{
   const LONG seq = hps_state_seq(expected_state);
   if (seq & 1)
      return false;
   const LONG locked = (LONG)((uint32_t)seq + 1u);
   const uint64_t desired = hps_pack_state(locked, new_resid);
   if ((uint64_t)InterlockedCompareExchange64((volatile LONG64 *)&slot->seq,
                                               (LONG64)desired,
                                               (LONG64)expected_state) != expected_state)
      return false;
   *write_seq = locked;
   return true;
}

static bool
hps_unlock(struct hps_slot *slot, LONG write_seq, uint32_t resid)
{
   const uint64_t expected = hps_pack_state(write_seq, resid);
   const uint64_t unlocked =
      hps_pack_state((LONG)((uint32_t)write_seq + 1u), resid);
   return (uint64_t)InterlockedCompareExchange64((volatile LONG64 *)&slot->seq,
                                                  (LONG64)unlocked,
                                                  (LONG64)expected) == expected;
}

static bool
hps_free(struct hps_slot *slot, LONG unlocked_seq, uint32_t resid)
{
   const uint64_t expected = hps_pack_state(unlocked_seq, resid);
   const uint64_t freed = hps_pack_state(unlocked_seq, 0);
   return (uint64_t)InterlockedCompareExchange64((volatile LONG64 *)&slot->seq,
                                                  (LONG64)freed,
                                                  (LONG64)expected) == expected;
}

static void
hps_write_payload(struct hps_slot *slot, uint32_t pid, uint32_t fence_id,
                  uint64_t value, uint64_t producer_start)
{
   slot->pid = pid;
   slot->fence_id = fence_id;
   slot->value = (LONG64)value;
   slot->producer_start = producer_start;
}

static void
hps_clear_payload(struct hps_slot *slot)
{
   slot->pid = 0;
   slot->fence_id = 0;
   slot->value = 0;
   slot->producer_start = 0;
}

bool
wsi_helios_present_sync_publish(uint32_t resid, uint32_t pid,
                                uint32_t fence_id, uint64_t value)
{
   InitOnceExecuteOnce(&hps_once, hps_init_mapping, NULL, NULL);
   if (!hps_slots || !resid)
      return false;
   const uint64_t start = hps_self_start();
   if (!pid || !start)
      return false;

   for (uint32_t attempt = 0; attempt < 8; attempt++) {
      for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
         struct hps_slot *slot = &hps_slots[i];
         const uint64_t state = hps_read_state(slot);
         if (hps_state_resid(state) != resid || (hps_state_seq(state) & 1))
            continue;
         LONG write_seq = 0;
         if (!hps_try_claim(slot, state, resid, &write_seq))
            continue;
         if (slot->resid == resid) {
            hps_write_payload(slot, pid, fence_id, value, start);
            return hps_unlock(slot, write_seq, resid);
         }
         if (!hps_unlock(slot, write_seq, resid))
            return false;
      }

      for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
         struct hps_slot *slot = &hps_slots[i];
         const uint64_t state = hps_read_state(slot);
         if (hps_state_resid(state) != 0 || (hps_state_seq(state) & 1))
            continue;
         LONG write_seq = 0;
         if (!hps_try_claim(slot, state, resid, &write_seq))
            continue;
         if (slot->resid == resid) {
            hps_write_payload(slot, pid, fence_id, value, start);
            return hps_unlock(slot, write_seq, resid);
         }
         if (!hps_unlock(slot, write_seq, resid))
            return false;
      }

      for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
         struct hps_slot *slot = &hps_slots[i];
         const uint64_t state = hps_read_state(slot);
         if (hps_state_resid(state) == 0 || (hps_state_seq(state) & 1))
            continue;
         const uint32_t old_pid = slot->pid;
         const uint64_t old_start = (uint64_t)InterlockedCompareExchange64(
            (volatile LONG64 *)&slot->producer_start, 0, 0);
         if (hps_read_state(slot) != state || hps_producer_alive(old_pid, old_start))
            continue;
         LONG write_seq = 0;
         if (!hps_try_claim(slot, state, resid, &write_seq))
            continue;
         if (slot->resid == resid) {
            hps_write_payload(slot, pid, fence_id, value, start);
            return hps_unlock(slot, write_seq, resid);
         }
         if (!hps_unlock(slot, write_seq, resid))
            return false;
      }
   }
   return false;
}

bool
wsi_helios_present_sync_release(uint32_t resid, uint32_t fence_id)
{
   InitOnceExecuteOnce(&hps_once, hps_init_mapping, NULL, NULL);
   if (!hps_slots || !resid || !fence_id)
      return false;
   const uint32_t pid = (uint32_t)GetCurrentProcessId();
   const uint64_t start = hps_self_start();
   if (!pid || !start)
      return false;

   for (uint32_t attempt = 0; attempt < 8; attempt++) {
      for (uint32_t i = 0; i < HPS_SLOT_COUNT; i++) {
         struct hps_slot *slot = &hps_slots[i];
         const uint64_t state = hps_read_state(slot);
         if (hps_state_resid(state) != resid || (hps_state_seq(state) & 1))
            continue;
         LONG write_seq = 0;
         if (!hps_try_claim(slot, state, resid, &write_seq))
            continue;
         const bool owned = slot->pid == pid && slot->producer_start == start &&
                            slot->fence_id == fence_id;
         if (!owned) {
            if (!hps_unlock(slot, write_seq, resid))
               return false;
            return false;
         }
         hps_clear_payload(slot);
         if (!hps_unlock(slot, write_seq, resid))
            return false;
         return hps_free(slot, (LONG)((uint32_t)write_seq + 1u), resid);
      }
   }
   return false;
}

#endif /* _WIN32 */
