/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * HWA2 — the §10.3 immutable create-time allocation descriptor, as the ICD
 * reads and writes it.
 *
 * ⛔ THE RECORD ITSELF IS NOT DECLARED HERE. `protocol/include/helios_wddm.h`
 * is the mechanical C mirror of `protocol/src/wddm.rs`, and it is the ONLY
 * declaration this ICD uses. The standing project directive is that shared
 * private data has one declaration, in `protocol/`; the pre-retirement ICD
 * violated it by hand-copying four records into `vn_renderer_helios.c` with
 * nothing but `sizeof` asserts, so a field REORDER inside the right number of
 * bytes passed every check in the tree (K4-CONTRACT §7 names that pair the
 * third cross-repo atomic pair, and lane-mesa A0 requires offset asserts rather
 * than size asserts). Including the protocol header makes its own
 * `HELIOS_WDDM_STATIC_ASSERT` block — one `offsetof` assert per field, plus the
 * 168-byte total and the flag-mask identity — a compile error on this side the
 * moment `protocol/` moves a field. That is the cross-check the old mirror did
 * not have in either direction.
 *
 * WHAT THIS FILE ADDS: the protocol header is deliberately NOT a validator (see
 * its own "WHAT THIS HEADER IS NOT" block) — it is constants, structs and
 * assertions. The rules live in Rust: `HeliosWddmAllocationDescV2::
 * {from_private_data, validate, validate_create_input, validate_create_output}`.
 * A C consumer must reproduce them, so this is that transcription, in the same
 * order, with the same refusal names, so a reader who knows one knows the
 * other. Any divergence is a bug in this file, never in the Rust.
 *
 * TWO-STAGE, exactly like HVM1 and HOC1 (K4-CONTRACT §1):
 *   - create INPUT  — what a UMD/ICD may legally hand `pfnAllocateCb`:
 *                     `allocation_generation == 0`, and the two KMD-owned flag
 *                     bits (DIRECT_FLIP_COMPATIBLE, D3D12_RUNTIME_PRIMARY)
 *                     clear. Everything else is written exactly and echoed.
 *   - create OUTPUT — what the KMD wrote back and every opener treats as const:
 *                     `allocation_generation != 0`. This is what `validate()`
 *                     means in the Rust, and it is what a C57 open reads.
 * The KMD refuses a create rather than correcting a field, so a wrong value on
 * the input side is a hard failure, not a silent fixup.
 */

#ifndef VN_HELIOS_HWA2_H
#define VN_HELIOS_HWA2_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The single declaration of the record. See the meson.build note for how the
 * include path reaches `protocol/include` across the submodule boundary. */
#include "helios_wddm.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Mirror of Rust `HeliosAllocDescRejection`, flattened to a code plus up to
 * three numbers because C has no payload-carrying enums. The NAME is the
 * load-bearing part: a refusal that names nothing has cost this project a round
 * trip three times in one session, so every refusal below prints its code name,
 * what was found and what was expected.
 *
 * Append-only, exactly like the Rust enum. Do not renumber.
 */
enum helios_hwa2_reject {
   HELIOS_HWA2_OK = 0,
   /* The (pPrivateDriverData, PrivateDriverDataSize) pair is not exactly
    * HELIOS_HWA2_BYTES. §10.3's truncation arm — and the first thing that runs,
    * so a split deploy (legacy 48/96-byte producer against an HWA2 reader, or
    * the reverse) is diagnosed here rather than as a mysterious magic
    * mismatch. */
   HELIOS_HWA2_REJECT_PRIVATE_DATA_SIZE,
   /* The per-allocation private-data pointer does not lie wholly inside the
    * queried pTotalPrivateDriverDataBuffer arena. Doc §10.9's failure row
    * requires refusing "per-allocation private data outside or larger than the
    * queried pTotalPrivateDriverDataBuffer"; the pre-retirement reader never
    * checked it. */
   HELIOS_HWA2_REJECT_PRIVATE_DATA_OUTSIDE_ARENA,
   HELIOS_HWA2_REJECT_MAGIC,
   HELIOS_HWA2_REJECT_ABI_VERSION,
   HELIOS_HWA2_REJECT_STRUCT_SIZE,
   HELIOS_HWA2_REJECT_PACKAGE_GENERATION,
   /* Output stage: the KMD-assigned generation is zero. */
   HELIOS_HWA2_REJECT_ALLOCATION_GENERATION_ZERO,
   /* Input stage: the creator set a generation only the KMD may assign. */
   HELIOS_HWA2_REJECT_ALLOCATION_GENERATION_NONZERO_ON_INPUT,
   /* Input stage: the creator set a KMD-owned flag bit (§1.1). */
   HELIOS_HWA2_REJECT_KMD_OWNED_FLAG_SET_ON_INPUT,
   HELIOS_HWA2_REJECT_BYTE_SIZE_ZERO,
   HELIOS_HWA2_REJECT_RESERVED_NONZERO,
   HELIOS_HWA2_REJECT_UNKNOWN_ALLOCATION_KIND,
   HELIOS_HWA2_REJECT_UNKNOWN_FLAG_BITS,
   HELIOS_HWA2_REJECT_UNKNOWN_BIND_BITS,
   HELIOS_HWA2_REJECT_UNKNOWN_MISC_BITS,
   HELIOS_HWA2_REJECT_UNKNOWN_SWIZZLE_CLASS,
   HELIOS_HWA2_REJECT_UNKNOWN_MEMORY_CLASS,
   HELIOS_HWA2_REJECT_IMAGE_GEOMETRY_ZERO,
   HELIOS_HWA2_REJECT_NON_IMAGE_GEOMETRY_NONZERO,
   HELIOS_HWA2_REJECT_IMAGE_DXGI_FORMAT_UNKNOWN,
   HELIOS_HWA2_REJECT_NON_IMAGE_DXGI_FORMAT_SET,
   HELIOS_HWA2_REJECT_NON_IMAGE_D3D_DDI_FORMAT_SET,
   HELIOS_HWA2_REJECT_PLANE_COUNT_OUT_OF_RANGE,
   HELIOS_HWA2_REJECT_IMAGE_PLANE_COUNT_ZERO,
   HELIOS_HWA2_REJECT_NON_IMAGE_PLANE_COUNT_NONZERO,
   HELIOS_HWA2_REJECT_UNUSED_PLANE_NONZERO,
   HELIOS_HWA2_REJECT_PLANE_ROW_PITCH_ZERO,
   HELIOS_HWA2_REJECT_PLANE_SLICE_PITCH_ZERO,
   HELIOS_HWA2_REJECT_PLANE_ROW_PITCH_EXCEEDS_SLICE_PITCH,
   HELIOS_HWA2_REJECT_PLANE_RANGE_OVERFLOW,
   HELIOS_HWA2_REJECT_PLANE_RANGE_EXCEEDS_BYTE_SIZE,
   HELIOS_HWA2_REJECT_D3D12_RUNTIME_PRIMARY_WITHOUT_PRIMARY,
   HELIOS_HWA2_REJECT_D3D12_RUNTIME_PRIMARY_NOT_SENTINEL,
   HELIOS_HWA2_REJECT_PRIMARY_VIDPN_SOURCE_NOT_CONCRETE,
   HELIOS_HWA2_REJECT_NON_PRIMARY_VIDPN_SOURCE_NOT_SENTINEL,
   HELIOS_HWA2_REJECT_STANDARD_ALLOCATION_TYPE_ZERO,
   HELIOS_HWA2_REJECT_STANDARD_FLAG_ON_NON_IMAGE_KIND,
   HELIOS_HWA2_REJECT_STANDARD_ALLOCATION_TYPE_WITHOUT_STANDARD_FLAG,
   HELIOS_HWA2_REJECT_STANDARD_KIND_WITHOUT_STANDARD_FLAG,
   HELIOS_HWA2_REJECT_DIRECT_FLIP_WITHOUT_PRIMARY,
   HELIOS_HWA2_REJECT_DIRECT_FLIP_PROTECTED,
   HELIOS_HWA2_REJECT_DIRECT_FLIP_CROSS_ADAPTER,
   HELIOS_HWA2_REJECT_DIRECT_FLIP_PLANE_COUNT_NOT_ONE,
   HELIOS_HWA2_REJECT_DIRECT_FLIP_UNSUPPORTED_SWIZZLE_CLASS,
   HELIOS_HWA2_REJECT_MEMORY_CLASS_CPU_VISIBILITY_MISMATCH,
   HELIOS_HWA2_REJECT_COUNT,
};

/* The three numbers a refusal may carry. `field` names the geometry field or
 * the plane index the Rust variant carries; it is a static string, never
 * allocated, and is NULL when the variant carries none. */
struct helios_hwa2_reject_detail {
   enum helios_hwa2_reject code;
   uint64_t found;
   uint64_t expected;
   const char *field;
};

const char *helios_hwa2_reject_name(enum helios_hwa2_reject code);

/*
 * Mirror of Rust `HeliosWddmAllocationDescV2::from_private_data`.
 *
 * Copies rather than casts, for the reason the Rust doc gives: the runtime's
 * buffer carries no alignment promise, and reading 168 bytes through a cast
 * pointer out of a shorter or misaligned buffer is an out-of-bounds read this
 * layer could not catch. Every consumer must enter through here.
 *
 * `arena`/`arena_bytes` are the queried `pTotalPrivateDriverDataBuffer` and its
 * size; pass NULL/0 only when there is genuinely no arena to check against, and
 * say why at the call site. When an arena IS supplied, `bytes` must lie wholly
 * inside it (doc §10.9).
 *
 * Returns false and fills `out_reject` on refusal. `out_desc` is untouched on
 * refusal.
 */
bool helios_hwa2_from_private_data(const void *bytes,
                                   size_t byte_count,
                                   const void *arena,
                                   size_t arena_bytes,
                                   HeliosWddmAllocationDescV2 *out_desc,
                                   struct helios_hwa2_reject_detail *out_reject);

/*
 * Mirror of Rust `validate_create_output` — and of `validate()`, which is the
 * same thing under its older name. This is what an opener applies: identity →
 * generations → vocabulary → geometry → planes → cross-field rules.
 */
bool helios_hwa2_validate_create_output(
   const HeliosWddmAllocationDescV2 *desc,
   uint64_t package_generation,
   struct helios_hwa2_reject_detail *out_reject);

/*
 * Mirror of Rust `validate_create_input` — what a creator may legally SEND.
 * Identical to the output stage except that `allocation_generation` must be
 * zero and the two KMD-owned flag bits must be clear (K4-CONTRACT §1.1).
 */
bool helios_hwa2_validate_create_input(
   const HeliosWddmAllocationDescV2 *desc,
   uint64_t package_generation,
   struct helios_hwa2_reject_detail *out_reject);

/*
 * Record a refusal: bump the named per-code counter and write one diagnosable
 * line naming the site, the code, and the numbers. `site` is a short static
 * string ("c57_open", "export_create", …).
 */
void helios_hwa2_note_reject(const char *site,
                             const struct helios_hwa2_reject_detail *reject);

long helios_hwa2_reject_count(enum helios_hwa2_reject code);

/* "size=3 magic=1" for every nonzero counter, or "none". For the perf dump. */
void helios_hwa2_reject_summary(char *buf, size_t buf_bytes);

/* ── The §10.3 identity gap: what HWA2 deliberately does not carry ────────────
 *
 * The retired 48-byte `HeliosWddmOpenIdentity` carried a host virtio/venus
 * `resource_id` at offset 24 and a Vulkan `memory_type_index` at offset 28.
 * NEITHER HAS AN HWA2 COUNTERPART AND NEITHER MAY BE ADDED — §10.3 via
 * `protocol/src/wddm.rs:355-361`: "No host resource token, resid, PID, process
 * handle, synchronization object, mutable value, or independently usable
 * identity. A host resid may live SOLELY inside the KMD allocation object; no
 * UMD, ICD, batch, or private descriptor can name or supply one."
 *
 * The replacement is not a field, it is a mechanism, and it is **mesa unit A3**:
 * the ICD stops naming host resources at all, names allocations by their index
 * in the `D3DDDI_ALLOCATIONLIST` plus HWA2's `allocation_generation` as the
 * anti-stale token, and the KMD patches the host resid into operand slots the
 * ICD wrote as zero (`HeliosNativeRenderPatch`). Until A3 lands, every path
 * that used to consume one of those two values refuses here — loudly, counted,
 * naming A3 — and never fabricates, substitutes or falls back.
 *
 * ⛔ An ICD in this state CANNOT import a Win32/D3D12 payload. That is the
 * retirement's intended intermediate state (K4-CONTRACT §5), not a regression
 * to be worked around.
 */
enum helios_a3_gap_site {
   /* vn_device_memory_import_win32: the payload validated, but there is no
    * legal way to tell the host which object it is. */
   HELIOS_A3_GAP_IMPORT_WIN32,
   /* vn_GetMemoryWin32HandlePropertiesKHR: the payload no longer records a
    * Vulkan memory type, and inventing one would be answering a question with
    * a guess the subsequent import cannot honour. */
   HELIOS_A3_GAP_HANDLE_PROPERTIES,
   /* vn_renderer_helios_external_memory_create: the export used to hand the
    * KMD `adopt_resource_id` so it would re-own this BO's venus resource.
    * That is UMD-backing adoption, which K4 deletes outright. */
   HELIOS_A3_GAP_EXPORT_ADOPTION,
   /* The retired VidMm tracker exports, kept as symbols so an older bridge DLL
    * still resolves them, and refusing so it can never believe an attestation
    * that no longer exists (K4-CONTRACT §6: no successor). */
   HELIOS_A3_GAP_VIDMM_TRACKER,
   HELIOS_A3_GAP_SITE_COUNT,
};

/* Bump the site's counter and log it. The first refusal at a site logs the full
 * explanation; after that one line every 256 refusals, so a hot import loop
 * cannot turn the diag log into the failure. */
void helios_a3_gap_refuse(enum helios_a3_gap_site site);
long helios_a3_gap_refusals(enum helios_a3_gap_site site);
const char *helios_a3_gap_site_name(enum helios_a3_gap_site site);
/* "import_win32=12 handle_properties=1" for every nonzero site, or "none". */
void helios_a3_gap_summary(char *buf, size_t buf_bytes);

/* Implemented in vn_renderer_helios.c; declared here so this TU can reach the
 * one diag sink the whole ICD writes (C:\ProgramData\Helios\helios_icd_diag.log
 * — dwm and WUDFHost have no visible stderr, so a refusal that only reaches
 * stderr cannot be post-mortemed). */
void vn_renderer_helios_diag_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* VN_HELIOS_HWA2_H */
