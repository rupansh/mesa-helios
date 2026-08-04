/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * WS1 #4 present-sync seqlock table — ICD-side WRITER.
 *
 * Byte-compatible port of the producer half of
 * dxvk-helios/src/dxvk/dxvk_helios_present_sync.cpp (wire format HPS2:
 * 32-byte header, 4096 x 32-byte slots (131104 bytes total), atomic
 * (seq,resid) claims and a seqlock writer epoch). The table file is
 * C:\ProgramData\Helios\helios_present_sync_v2.bin (override:
 * HELIOS_PRESENT_SYNC_PATH) and must NEVER be deleted while mapped —
 * existing views split-brain until every mapper restarts.
 */

#ifndef WSI_HELIOS_PRESENT_SYNC_H
#define WSI_HELIOS_PRESENT_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t wsi_helios_present_sync_alloc_fence_id(void);

/* Exact process generation stored in every HPS2 publication and embedded in
 * its named fence. Zero means the generation could not be queried. */
uint64_t wsi_helios_present_sync_process_start(void);

bool wsi_helios_present_sync_publish(uint32_t resid, uint32_t pid,
                                     uint32_t fence_id, uint64_t value);

bool wsi_helios_present_sync_release(uint32_t resid, uint32_t fence_id);

#ifdef __cplusplus
}
#endif

#endif /* WSI_HELIOS_PRESENT_SYNC_H */
