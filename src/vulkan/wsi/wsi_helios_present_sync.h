/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * WS1 #4 present-sync seqlock table — ICD-side WRITER.
 *
 * Byte-compatible port of the producer half of
 * dxvk-helios/src/dxvk/dxvk_helios_present_sync.cpp (wire format HPS1:
 * 32-byte header, 64 x 32-byte slots, CAS-claimed resid keys, seqlock
 * odd-while-writing). The table file is
 * C:\ProgramData\Helios\helios_present_sync.bin (override:
 * HELIOS_PRESENT_SYNC_PATH) and must NEVER be deleted while mapped —
 * existing views split-brain until every mapper restarts.
 *
 * The dcomp present vehicle publishes (frame resid -> pid, fence id, value)
 * here so the vehicle DXVK's copy-time consumer wait can import the named
 * fence Global\HeliosPresentFence_<pid>_<fenceId> and bound-wait for the
 * frame's GPU completion before copying.
 */

#ifndef WSI_HELIOS_PRESENT_SYNC_H
#define WSI_HELIOS_PRESENT_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate a process-unique fence id in the ICD's HALF of the id space.
 * The UMD/dxvk producer allocates ids from a plain 1..n counter in ITS
 * module; the ICD writer lives in a different DLL with its own counter, and
 * both name fences Global\HeliosPresentFence_<pid>_<fenceId> — a collision
 * makes the second create fail (proven live with per-pid-only names). The
 * high bit partitions the namespaces. */
uint32_t wsi_helios_present_sync_alloc_fence_id(void);

/* Seqlock-publish (resid -> pid, fence_id, value). Returns false when the
 * table is unavailable or full (counted by the caller — loud, not fatal:
 * consumers then take their bounded no-slot path). */
bool wsi_helios_present_sync_publish(uint32_t resid, uint32_t pid,
                                     uint32_t fence_id, uint64_t value);

#ifdef __cplusplus
}
#endif

#endif /* WSI_HELIOS_PRESENT_SYNC_H */
