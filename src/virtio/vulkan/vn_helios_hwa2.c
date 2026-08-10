/*
 * Copyright 2026 Helios vGPU project
 * SPDX-License-Identifier: MIT
 *
 * HWA2 parse + validate for the ICD. See vn_helios_hwa2.h for why this exists
 * and what it must stay in step with.
 *
 * ⛔ THIS FILE IS A TRANSCRIPTION, NOT A DESIGN. Every check below exists in
 * `protocol/src/wddm.rs` first — `HeliosWddmAllocationDescV2::from_private_data`
 * and `::validate` (the create-OUTPUT validator), plus the create-INPUT stage
 * rules K4-CONTRACT §1.1 defines. The order is the Rust's order, deliberately:
 * a reader comparing the two must be able to walk them side by side, and the
 * FIRST failure is the informative one, so identity precedes vocabulary and
 * vocabulary precedes cross-field rules. If the Rust and this file ever
 * disagree, the Rust wins and this file is the bug.
 *
 * ⚠ There is no compile-time link between the two — the assertions in
 * `protocol/include/helios_wddm.h` pin the LAYOUT, and nothing pins the RULES.
 * `tools/retirement-gates.sh` is the right place for a diff gate over these two
 * bodies; until it has one, this pairing is maintained by review.
 */

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "vn_helios_hwa2.h"

#include <stdio.h>
#include <string.h>

#include <windows.h>

/* ── the two kind predicates, mirroring the Rust const fns ──────────────────
 *
 * ⚠⚠ The kind enum is the trap in this whole re-point. The retired record's
 * `kind == 1` meant HELIOS_WDDM_ALLOC_KIND_DEVICE_MEMORY; HWA2's
 * `allocation_kind == 1` means HELIOS_HWA2_KIND_BUFFER. A mechanical swap of
 * the struct that leaves an `== 1` comparison standing keeps compiling and
 * silently starts asking a different question. Nothing in this file compares a
 * kind to a bare integer for that reason. */
static bool
helios_hwa2_kind_is_image(uint32_t kind)
{
   return kind == HELIOS_HWA2_KIND_IMAGE ||
          kind == HELIOS_HWA2_KIND_STANDARD_PRIMARY ||
          kind == HELIOS_HWA2_KIND_STANDARD_SHADOW ||
          kind == HELIOS_HWA2_KIND_STANDARD_STAGING;
}

static bool
helios_hwa2_kind_is_standard(uint32_t kind)
{
   return kind == HELIOS_HWA2_KIND_STANDARD_PRIMARY ||
          kind == HELIOS_HWA2_KIND_STANDARD_SHADOW ||
          kind == HELIOS_HWA2_KIND_STANDARD_STAGING;
}

static bool
helios_hwa2_swizzle_is_direct_flip_capable(uint32_t swizzle_class)
{
   return swizzle_class == HELIOS_HWA2_SWIZZLE_LINEAR;
}

static const char *const helios_hwa2_reject_names[HELIOS_HWA2_REJECT_COUNT] = {
   [HELIOS_HWA2_OK] = "ok",
   [HELIOS_HWA2_REJECT_PRIVATE_DATA_SIZE] = "private_data_size",
   [HELIOS_HWA2_REJECT_PRIVATE_DATA_OUTSIDE_ARENA] = "private_data_outside_arena",
   [HELIOS_HWA2_REJECT_MAGIC] = "magic",
   [HELIOS_HWA2_REJECT_ABI_VERSION] = "abi_version",
   [HELIOS_HWA2_REJECT_STRUCT_SIZE] = "struct_size",
   [HELIOS_HWA2_REJECT_PACKAGE_GENERATION] = "package_generation",
   [HELIOS_HWA2_REJECT_ALLOCATION_GENERATION_ZERO] = "allocation_generation_zero",
   [HELIOS_HWA2_REJECT_ALLOCATION_GENERATION_NONZERO_ON_INPUT] =
      "allocation_generation_nonzero_on_input",
   [HELIOS_HWA2_REJECT_KMD_OWNED_FLAG_SET_ON_INPUT] = "kmd_owned_flag_set_on_input",
   [HELIOS_HWA2_REJECT_BYTE_SIZE_ZERO] = "byte_size_zero",
   [HELIOS_HWA2_REJECT_RESERVED_NONZERO] = "reserved_nonzero",
   [HELIOS_HWA2_REJECT_UNKNOWN_ALLOCATION_KIND] = "unknown_allocation_kind",
   [HELIOS_HWA2_REJECT_UNKNOWN_FLAG_BITS] = "unknown_flag_bits",
   [HELIOS_HWA2_REJECT_UNKNOWN_BIND_BITS] = "unknown_bind_bits",
   [HELIOS_HWA2_REJECT_UNKNOWN_MISC_BITS] = "unknown_misc_bits",
   [HELIOS_HWA2_REJECT_UNKNOWN_SWIZZLE_CLASS] = "unknown_swizzle_class",
   [HELIOS_HWA2_REJECT_UNKNOWN_MEMORY_CLASS] = "unknown_memory_class",
   [HELIOS_HWA2_REJECT_IMAGE_GEOMETRY_ZERO] = "image_geometry_zero",
   [HELIOS_HWA2_REJECT_NON_IMAGE_GEOMETRY_NONZERO] = "non_image_geometry_nonzero",
   [HELIOS_HWA2_REJECT_IMAGE_DXGI_FORMAT_UNKNOWN] = "image_dxgi_format_unknown",
   [HELIOS_HWA2_REJECT_NON_IMAGE_DXGI_FORMAT_SET] = "non_image_dxgi_format_set",
   [HELIOS_HWA2_REJECT_NON_IMAGE_D3D_DDI_FORMAT_SET] = "non_image_d3d_ddi_format_set",
   [HELIOS_HWA2_REJECT_PLANE_COUNT_OUT_OF_RANGE] = "plane_count_out_of_range",
   [HELIOS_HWA2_REJECT_IMAGE_PLANE_COUNT_ZERO] = "image_plane_count_zero",
   [HELIOS_HWA2_REJECT_NON_IMAGE_PLANE_COUNT_NONZERO] = "non_image_plane_count_nonzero",
   [HELIOS_HWA2_REJECT_UNUSED_PLANE_NONZERO] = "unused_plane_nonzero",
   [HELIOS_HWA2_REJECT_PLANE_ROW_PITCH_ZERO] = "plane_row_pitch_zero",
   [HELIOS_HWA2_REJECT_PLANE_SLICE_PITCH_ZERO] = "plane_slice_pitch_zero",
   [HELIOS_HWA2_REJECT_PLANE_ROW_PITCH_EXCEEDS_SLICE_PITCH] =
      "plane_row_pitch_exceeds_slice_pitch",
   [HELIOS_HWA2_REJECT_PLANE_RANGE_OVERFLOW] = "plane_range_overflow",
   [HELIOS_HWA2_REJECT_PLANE_RANGE_EXCEEDS_BYTE_SIZE] = "plane_range_exceeds_byte_size",
   [HELIOS_HWA2_REJECT_D3D12_RUNTIME_PRIMARY_WITHOUT_PRIMARY] =
      "d3d12_runtime_primary_without_primary",
   [HELIOS_HWA2_REJECT_D3D12_RUNTIME_PRIMARY_NOT_SENTINEL] =
      "d3d12_runtime_primary_not_sentinel",
   [HELIOS_HWA2_REJECT_PRIMARY_VIDPN_SOURCE_NOT_CONCRETE] =
      "primary_vidpn_source_not_concrete",
   [HELIOS_HWA2_REJECT_NON_PRIMARY_VIDPN_SOURCE_NOT_SENTINEL] =
      "non_primary_vidpn_source_not_sentinel",
   [HELIOS_HWA2_REJECT_STANDARD_ALLOCATION_TYPE_ZERO] = "standard_allocation_type_zero",
   [HELIOS_HWA2_REJECT_STANDARD_FLAG_ON_NON_IMAGE_KIND] =
      "standard_flag_on_non_image_kind",
   [HELIOS_HWA2_REJECT_STANDARD_ALLOCATION_TYPE_WITHOUT_STANDARD_FLAG] =
      "standard_allocation_type_without_standard_flag",
   [HELIOS_HWA2_REJECT_STANDARD_KIND_WITHOUT_STANDARD_FLAG] =
      "standard_kind_without_standard_flag",
   [HELIOS_HWA2_REJECT_DIRECT_FLIP_WITHOUT_PRIMARY] = "direct_flip_without_primary",
   [HELIOS_HWA2_REJECT_DIRECT_FLIP_PROTECTED] = "direct_flip_protected",
   [HELIOS_HWA2_REJECT_DIRECT_FLIP_CROSS_ADAPTER] = "direct_flip_cross_adapter",
   [HELIOS_HWA2_REJECT_DIRECT_FLIP_PLANE_COUNT_NOT_ONE] =
      "direct_flip_plane_count_not_one",
   [HELIOS_HWA2_REJECT_DIRECT_FLIP_UNSUPPORTED_SWIZZLE_CLASS] =
      "direct_flip_unsupported_swizzle_class",
   [HELIOS_HWA2_REJECT_MEMORY_CLASS_CPU_VISIBILITY_MISMATCH] =
      "memory_class_cpu_visibility_mismatch",
};

const char *
helios_hwa2_reject_name(enum helios_hwa2_reject code)
{
   if ((unsigned)code >= HELIOS_HWA2_REJECT_COUNT)
      return "unknown_reject_code";
   const char *name = helios_hwa2_reject_names[code];
   return name ? name : "unnamed_reject_code";
}

/* Fill `out_reject` and return false. Every refusal arm goes through here so no
 * arm can forget to name itself. */
static bool
helios_hwa2_reject(struct helios_hwa2_reject_detail *out_reject,
                   enum helios_hwa2_reject code,
                   uint64_t found,
                   uint64_t expected,
                   const char *field)
{
   if (out_reject) {
      out_reject->code = code;
      out_reject->found = found;
      out_reject->expected = expected;
      out_reject->field = field;
   }
   return false;
}

static bool
helios_hwa2_accept(struct helios_hwa2_reject_detail *out_reject)
{
   if (out_reject) {
      out_reject->code = HELIOS_HWA2_OK;
      out_reject->found = 0;
      out_reject->expected = 0;
      out_reject->field = NULL;
   }
   return true;
}

bool
helios_hwa2_from_private_data(const void *bytes,
                              size_t byte_count,
                              const void *arena,
                              size_t arena_bytes,
                              HeliosWddmAllocationDescV2 *out_desc,
                              struct helios_hwa2_reject_detail *out_reject)
{
   if (!out_desc)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PRIVATE_DATA_SIZE, 0,
                                HELIOS_HWA2_BYTES, "out_desc");

   /* ── THE SPLIT-DEPLOY ARM, AND IT RUNS FIRST ──────────────────────────────
    *
    * The KMD and this ICD ship separately, so the two halves of the package
    * can differ across an install. An exact size test is the earliest and
    * cheapest place that difference becomes visible, and it is the one the
    * protocol's own reader uses: `from_private_data` rejects any length that
    * is not exactly 168 by name. `>=` would be worse than useless here — the
    * pre-retirement reader used `>=` and would happily accept a 200-byte
    * buffer that `protocol/` itself refuses, so the guest and the kernel would
    * disagree about whether the same bytes were admissible.
    *
    * A legacy 48-byte HeliosWddmOpenIdentity or 96-byte AdoptedAllocPrivate
    * lands here, not on the magic, and gets a message that says so. There is
    * NO negotiation and there must not be: §10.3 — "a mismatch is a hard
    * reject and never selects a legacy parser". */
   if (byte_count != (size_t)HELIOS_HWA2_BYTES || !bytes) {
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PRIVATE_DATA_SIZE,
                                (uint64_t)byte_count, HELIOS_HWA2_BYTES, NULL);
   }

   /* ── the arena containment check the doc requires and the old reader lacked
    *
    * dxgkrnl addresses each D3DDDI_OPENALLOCATIONINFO2's pPrivateDriverData
    * INSIDE the caller's pTotalPrivateDriverDataBuffer. §10.9's failure row
    * requires refusing "per-allocation private data outside or larger than the
    * queried pTotalPrivateDriverDataBuffer". The pointer arithmetic is done on
    * uintptr_t so no pointer ever moves out of its object, and the end is
    * computed by subtraction rather than addition so it cannot wrap. */
   if (arena) {
      const uintptr_t arena_begin = (uintptr_t)arena;
      const uintptr_t data_begin = (uintptr_t)bytes;
      if (arena_bytes < (size_t)HELIOS_HWA2_BYTES || data_begin < arena_begin ||
          data_begin - arena_begin > arena_bytes - (size_t)HELIOS_HWA2_BYTES) {
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_PRIVATE_DATA_OUTSIDE_ARENA,
                                   (uint64_t)(data_begin - arena_begin),
                                   (uint64_t)arena_bytes, NULL);
      }
   }

   /* Copy, never cast: the runtime's buffer carries no alignment promise. */
   memcpy(out_desc, bytes, sizeof(*out_desc));
   return helios_hwa2_accept(out_reject);
}

/* The shared core of both stages, so a rule can never be enforced on one path
 * and not the other — the same structure `protocol/src/wddm.rs` uses. */
static bool
helios_hwa2_validate_common(const HeliosWddmAllocationDescV2 *desc,
                            uint64_t package_generation,
                            bool create_input,
                            struct helios_hwa2_reject_detail *out_reject)
{
   if (!desc)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_MAGIC, 0,
                                HELIOS_HWA2_MAGIC, "desc");

   /* ── identity ─────────────────────────────────────────────────────────── */
   if (desc->magic != HELIOS_HWA2_MAGIC)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_MAGIC, desc->magic,
                                HELIOS_HWA2_MAGIC, NULL);
   if (desc->abi_version != HELIOS_HWA2_ABI_VERSION)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_ABI_VERSION,
                                desc->abi_version, HELIOS_HWA2_ABI_VERSION, NULL);
   if (desc->struct_size != HELIOS_HWA2_BYTES)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_STRUCT_SIZE,
                                desc->struct_size, HELIOS_HWA2_BYTES, NULL);
   /* Zero is never a live package generation on either side: a zeroed buffer
    * must not be admitted just because the caller has not established the
    * package yet. This is the field designed to catch a split deploy that the
    * size and magic both survive, and the pre-retirement reader checked
    * nothing like it. */
   if (desc->package_generation != package_generation || desc->package_generation == 0)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PACKAGE_GENERATION,
                                desc->package_generation, package_generation, NULL);

   /* ── generations: the one field whose rule differs by stage ───────────── */
   if (create_input) {
      if (desc->allocation_generation != 0)
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_ALLOCATION_GENERATION_NONZERO_ON_INPUT,
            desc->allocation_generation, 0, NULL);
   } else if (desc->allocation_generation == 0) {
      return helios_hwa2_reject(out_reject,
                                HELIOS_HWA2_REJECT_ALLOCATION_GENERATION_ZERO, 0, 0,
                                NULL);
   }

   if (desc->reserved != 0)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_RESERVED_NONZERO,
                                desc->reserved, 0, NULL);
   if (desc->byte_size == 0)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_BYTE_SIZE_ZERO, 0, 0,
                                NULL);

   /* ── vocabulary ───────────────────────────────────────────────────────── */
   if (desc->allocation_kind == HELIOS_HWA2_KIND_INVALID ||
       desc->allocation_kind > HELIOS_HWA2_KIND_MAX) {
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_UNKNOWN_ALLOCATION_KIND,
                                desc->allocation_kind, HELIOS_HWA2_KIND_MAX, NULL);
   }
   if (desc->flags & ~HELIOS_HWA2_FLAG_MASK)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_UNKNOWN_FLAG_BITS,
                                desc->flags & ~HELIOS_HWA2_FLAG_MASK,
                                HELIOS_HWA2_FLAG_MASK, NULL);
   /* Stage rule: the KMD owns DIRECT_FLIP_COMPATIBLE and D3D12_RUNTIME_PRIMARY.
    * A creator that sets either is claiming a decision only the kernel can make
    * (§10.3: "the bit and sentinel are cross-validated and neither is inferred
    * by an opener"), and the KMD refuses the create rather than clearing it. */
   if (create_input && (desc->flags & HELIOS_HWA2_FLAG_KMD_OWNED_MASK)) {
      return helios_hwa2_reject(out_reject,
                                HELIOS_HWA2_REJECT_KMD_OWNED_FLAG_SET_ON_INPUT,
                                desc->flags & HELIOS_HWA2_FLAG_KMD_OWNED_MASK, 0,
                                NULL);
   }
   if (desc->bind_flags & ~HELIOS_HWA2_BIND_MASK)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_UNKNOWN_BIND_BITS,
                                desc->bind_flags & ~HELIOS_HWA2_BIND_MASK,
                                HELIOS_HWA2_BIND_MASK, NULL);
   if (desc->misc_flags & ~HELIOS_HWA2_MISC_MASK)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_UNKNOWN_MISC_BITS,
                                desc->misc_flags & ~HELIOS_HWA2_MISC_MASK,
                                HELIOS_HWA2_MISC_MASK, NULL);
   if (desc->swizzle_class == HELIOS_HWA2_SWIZZLE_INVALID ||
       desc->swizzle_class > HELIOS_HWA2_SWIZZLE_MAX) {
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_UNKNOWN_SWIZZLE_CLASS,
                                desc->swizzle_class, HELIOS_HWA2_SWIZZLE_MAX, NULL);
   }
   if (desc->memory_class == HELIOS_HWA2_MEMORY_INVALID ||
       desc->memory_class > HELIOS_HWA2_MEMORY_MAX) {
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_UNKNOWN_MEMORY_CLASS,
                                desc->memory_class, HELIOS_HWA2_MEMORY_MAX, NULL);
   }

   /* ── geometry, per kind class ─────────────────────────────────────────────
    *
    * "zero only for a non-image allocation kind", read fail-closed in BOTH
    * directions: an image must carry every one of these, a non-image must carry
    * none of them. */
   const bool image = helios_hwa2_kind_is_image(desc->allocation_kind);
   const struct {
      const char *name;
      uint32_t value;
   } geometry[] = {
      { "width", desc->width },
      { "height", desc->height },
      { "depth_or_array_size", desc->depth_or_array_size },
      { "mip_levels", desc->mip_levels },
      { "sample_count", desc->sample_count },
   };
   for (size_t i = 0; i < sizeof(geometry) / sizeof(geometry[0]); i++) {
      if (image) {
         if (geometry[i].value == 0)
            return helios_hwa2_reject(out_reject,
                                      HELIOS_HWA2_REJECT_IMAGE_GEOMETRY_ZERO, 0, 0,
                                      geometry[i].name);
      } else if (geometry[i].value != 0) {
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_NON_IMAGE_GEOMETRY_NONZERO,
                                   geometry[i].value, 0, geometry[i].name);
      }
   }
   /* Sample quality is meaningless without a sample count, and the sample count
    * is already required zero above for a non-image. */
   if (!image && desc->sample_quality != 0)
      return helios_hwa2_reject(out_reject,
                                HELIOS_HWA2_REJECT_NON_IMAGE_GEOMETRY_NONZERO,
                                desc->sample_quality, 0, "sample_quality");

   if (image) {
      if (desc->dxgi_format == 0 /* DXGI_FORMAT_UNKNOWN */)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_IMAGE_DXGI_FORMAT_UNKNOWN, 0, 0,
                                   NULL);
   } else {
      if (desc->dxgi_format != 0)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_NON_IMAGE_DXGI_FORMAT_SET,
                                   desc->dxgi_format, 0, NULL);
      if (desc->d3d_ddi_format != 0)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_NON_IMAGE_D3D_DDI_FORMAT_SET,
                                   desc->d3d_ddi_format, 0, NULL);
   }

   /* ── planes ───────────────────────────────────────────────────────────── */
   if (desc->plane_count > HELIOS_HWA2_MAX_PLANES)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PLANE_COUNT_OUT_OF_RANGE,
                                desc->plane_count, HELIOS_HWA2_MAX_PLANES, NULL);
   if (image && desc->plane_count == 0)
      return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_IMAGE_PLANE_COUNT_ZERO, 0,
                                0, NULL);
   if (!image && desc->plane_count != 0)
      return helios_hwa2_reject(out_reject,
                                HELIOS_HWA2_REJECT_NON_IMAGE_PLANE_COUNT_NONZERO,
                                desc->plane_count, 0, NULL);
   for (uint32_t p = 0; p < HELIOS_HWA2_MAX_PLANES; p++) {
      const HeliosWddmPlaneRecordV2 plane = desc->planes[p];
      if (p >= desc->plane_count) {
         if (plane.offset != 0 || plane.row_pitch != 0 || plane.slice_pitch != 0)
            return helios_hwa2_reject(out_reject,
                                      HELIOS_HWA2_REJECT_UNUSED_PLANE_NONZERO, p,
                                      desc->plane_count, NULL);
         continue;
      }
      if (plane.row_pitch == 0)
         return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PLANE_ROW_PITCH_ZERO,
                                   p, 0, NULL);
      if (plane.slice_pitch == 0)
         return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PLANE_SLICE_PITCH_ZERO,
                                   p, 0, NULL);
      if (plane.row_pitch > plane.slice_pitch)
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_PLANE_ROW_PITCH_EXCEEDS_SLICE_PITCH, p,
            plane.slice_pitch, NULL);
      /* A used plane's range is offset + slice_pitch — never row_pitch * height,
       * which over-counts a chroma plane. Overflow-checked by subtraction so
       * the sum is never formed. */
      if (plane.offset > UINT64_MAX - (uint64_t)plane.slice_pitch)
         return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_PLANE_RANGE_OVERFLOW,
                                   p, 0, NULL);
      if (plane.offset + (uint64_t)plane.slice_pitch > desc->byte_size)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_PLANE_RANGE_EXCEEDS_BYTE_SIZE, p,
                                   desc->byte_size, NULL);
   }

   /* ── C44: the runtime-primary bit and the VidPn sentinel, cross-validated ─
    *
    * ⛔ Neither is inferred by an opener. Both are checked together so a reader
    * can never accept one without the other. */
   const bool primary = (desc->flags & HELIOS_HWA2_FLAG_PRIMARY) != 0;
   if (desc->flags & HELIOS_HWA2_FLAG_D3D12_RUNTIME_PRIMARY) {
      if (!primary)
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_D3D12_RUNTIME_PRIMARY_WITHOUT_PRIMARY,
            desc->flags, 0, NULL);
      if (desc->vidpn_source != HELIOS_D3DDDI_ID_UNINITIALIZED)
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_D3D12_RUNTIME_PRIMARY_NOT_SENTINEL,
            desc->vidpn_source, HELIOS_D3DDDI_ID_UNINITIALIZED, NULL);
   } else if (primary) {
      if (desc->vidpn_source == HELIOS_D3DDDI_ID_UNINITIALIZED)
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_PRIMARY_VIDPN_SOURCE_NOT_CONCRETE,
            desc->vidpn_source, 0, NULL);
   } else if (desc->vidpn_source != HELIOS_D3DDDI_ID_UNINITIALIZED) {
      return helios_hwa2_reject(out_reject,
                                HELIOS_HWA2_REJECT_NON_PRIMARY_VIDPN_SOURCE_NOT_SENTINEL,
                                desc->vidpn_source, HELIOS_D3DDDI_ID_UNINITIALIZED,
                                NULL);
   }

   /* ── standard-allocation coupling ─────────────────────────────────────── */
   const bool standard = (desc->flags & HELIOS_HWA2_FLAG_STANDARD) != 0;
   if (standard) {
      if (desc->standard_allocation_type == 0)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_STANDARD_ALLOCATION_TYPE_ZERO, 0,
                                   0, NULL);
      if (!image)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_STANDARD_FLAG_ON_NON_IMAGE_KIND,
                                   desc->allocation_kind, 0, NULL);
   } else {
      if (desc->standard_allocation_type != 0)
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_STANDARD_ALLOCATION_TYPE_WITHOUT_STANDARD_FLAG,
            desc->standard_allocation_type, 0, NULL);
      if (helios_hwa2_kind_is_standard(desc->allocation_kind))
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_STANDARD_KIND_WITHOUT_STANDARD_FLAG,
                                   desc->allocation_kind, 0, NULL);
   }

   /* ── Direct Flip eligibility ──────────────────────────────────────────────
    * Unreachable on the create-input stage, where the bit is already refused
    * above; kept here because the output stage reaches it and both stages share
    * this body. */
   if (desc->flags & HELIOS_HWA2_FLAG_DIRECT_FLIP_COMPATIBLE) {
      if (!primary)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_DIRECT_FLIP_WITHOUT_PRIMARY,
                                   desc->flags, 0, NULL);
      if (desc->flags & HELIOS_HWA2_FLAG_PROTECTED)
         return helios_hwa2_reject(out_reject, HELIOS_HWA2_REJECT_DIRECT_FLIP_PROTECTED,
                                   desc->flags, 0, NULL);
      if (desc->flags & HELIOS_HWA2_FLAG_CROSS_ADAPTER)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_DIRECT_FLIP_CROSS_ADAPTER,
                                   desc->flags, 0, NULL);
      if (desc->plane_count != 1)
         return helios_hwa2_reject(out_reject,
                                   HELIOS_HWA2_REJECT_DIRECT_FLIP_PLANE_COUNT_NOT_ONE,
                                   desc->plane_count, 1, NULL);
      if (!helios_hwa2_swizzle_is_direct_flip_capable(desc->swizzle_class))
         return helios_hwa2_reject(
            out_reject, HELIOS_HWA2_REJECT_DIRECT_FLIP_UNSUPPORTED_SWIZZLE_CLASS,
            desc->swizzle_class, HELIOS_HWA2_SWIZZLE_LINEAR, NULL);
   }

   /* ── memory class vs CPU visibility ───────────────────────────────────── */
   const bool cpu_visible = (desc->flags & HELIOS_HWA2_FLAG_CPU_VISIBLE) != 0;
   bool mismatch = false;
   if (desc->memory_class == HELIOS_HWA2_MEMORY_CPU_VISIBLE)
      mismatch = !cpu_visible;
   else if (desc->memory_class == HELIOS_HWA2_MEMORY_DEVICE_LOCAL)
      mismatch = cpu_visible;
   if (mismatch)
      return helios_hwa2_reject(out_reject,
                                HELIOS_HWA2_REJECT_MEMORY_CLASS_CPU_VISIBILITY_MISMATCH,
                                desc->memory_class, desc->flags, NULL);

   return helios_hwa2_accept(out_reject);
}

bool
helios_hwa2_validate_create_output(const HeliosWddmAllocationDescV2 *desc,
                                   uint64_t package_generation,
                                   struct helios_hwa2_reject_detail *out_reject)
{
   return helios_hwa2_validate_common(desc, package_generation,
                                      /* create_input */ false, out_reject);
}

bool
helios_hwa2_validate_create_input(const HeliosWddmAllocationDescV2 *desc,
                                  uint64_t package_generation,
                                  struct helios_hwa2_reject_detail *out_reject)
{
   return helios_hwa2_validate_common(desc, package_generation,
                                      /* create_input */ true, out_reject);
}

/* ── refusal counters ──────────────────────────────────────────────────────
 *
 * Interlocked statics rather than per-renderer fields: a refusal can happen on
 * a probe path that holds no renderer (vkGetMemoryWin32HandlePropertiesKHR
 * releases its payload immediately) and the counters must survive it. Signed
 * LONG because that is what InterlockedIncrement takes. */
static volatile LONG helios_hwa2_reject_counters[HELIOS_HWA2_REJECT_COUNT];

void
helios_hwa2_note_reject(const char *site,
                        const struct helios_hwa2_reject_detail *reject)
{
   if (!reject || reject->code == HELIOS_HWA2_OK ||
       (unsigned)reject->code >= HELIOS_HWA2_REJECT_COUNT)
      return;

   InterlockedIncrement(&helios_hwa2_reject_counters[reject->code]);
   vn_renderer_helios_diag_log(
      "HWA2 REFUSED site=%s reason=%s found=0x%llx(%llu) expected=0x%llx(%llu) field=%s"
      " -- this ICD reads the 168-byte 10.3 HWA2 descriptor; a legacy 48-byte"
      " HeliosWddmOpenIdentity or 96-byte AdoptedAllocPrivate producer is a SPLIT"
      " DEPLOY (KMD/UMD and ICD from different packages), not a corrupt payload."
      " There is no fallback parser by design.",
      site ? site : "?", helios_hwa2_reject_name(reject->code),
      (unsigned long long)reject->found, (unsigned long long)reject->found,
      (unsigned long long)reject->expected, (unsigned long long)reject->expected,
      reject->field ? reject->field : "-");
}

long
helios_hwa2_reject_count(enum helios_hwa2_reject code)
{
   if ((unsigned)code >= HELIOS_HWA2_REJECT_COUNT)
      return 0;
   return helios_hwa2_reject_counters[code];
}

void
helios_hwa2_reject_summary(char *buf, size_t buf_bytes)
{
   if (!buf || !buf_bytes)
      return;
   buf[0] = '\0';
   size_t used = 0;
   bool any = false;
   for (unsigned code = 1; code < HELIOS_HWA2_REJECT_COUNT; code++) {
      const long count = helios_hwa2_reject_counters[code];
      if (!count)
         continue;
      const int written =
         snprintf(buf + used, buf_bytes - used, "%s%s=%ld", any ? " " : "",
                  helios_hwa2_reject_name((enum helios_hwa2_reject)code), count);
      /* snprintf returns what it WOULD have written; stop on truncation rather
       * than advancing `used` past the buffer. */
      if (written < 0 || (size_t)written >= buf_bytes - used)
         return;
      used += (size_t)written;
      any = true;
   }
   if (!any)
      snprintf(buf, buf_bytes, "none");
}

/* ── the §10.3 identity gap (mesa unit A3) ─────────────────────────────────── */

static const char *const helios_a3_gap_site_names[HELIOS_A3_GAP_SITE_COUNT] = {
   [HELIOS_A3_GAP_IMPORT_WIN32] = "import_win32",
   [HELIOS_A3_GAP_HANDLE_PROPERTIES] = "handle_properties",
   [HELIOS_A3_GAP_EXPORT_ADOPTION] = "export_adoption",
   [HELIOS_A3_GAP_VIDMM_TRACKER] = "vidmm_tracker",
};

/* What each site can no longer obtain, and what will supply it. A refusal that
 * names nothing is why this project has burned round trips; these strings are
 * the instrument, so keep them specific. */
static const char *const helios_a3_gap_site_reasons[HELIOS_A3_GAP_SITE_COUNT] = {
   [HELIOS_A3_GAP_IMPORT_WIN32] =
      "the payload's HWA2 descriptor validated, but HWA2 carries NO host resource id"
      " (resource_id, offset 24 of the retired HeliosWddmOpenIdentity) and 10.3"
      " forbids the ICD naming a host resource at all, so there is nothing legal to"
      " put in VkImportMemoryResourceInfoMESA::resourceId. SUPPLIED BY: mesa unit A3"
      " -- the ICD names allocations by D3DDDI_ALLOCATIONLIST index plus HWA2"
      " allocation_generation, and the KMD patches the host resid in from"
      " HeliosNativeRenderPatch",
   [HELIOS_A3_GAP_HANDLE_PROPERTIES] =
      "HWA2 carries NO Vulkan memory_type_index (retired trailer offset 28); its"
      " memory_class is a 3-value protocol enum and 10.7 says a C57 D3D12_RESOURCE"
      " import is never exposed through an ordinary memory type. Answering with a"
      " guessed type would promise an import that the A3 gap then refuses."
      " SUPPLIED BY: mesa units A6 (the two-type table) and A8, after A3",
   [HELIOS_A3_GAP_EXPORT_ADOPTION] =
      "the export used to hand the KMD adopt_resource_id so it would re-own this BO's"
      " venus resource. That is UMD-backing adoption, which K4 deletes and 10.3"
      " forbids reintroducing under another name. SUPPLIED BY: mesa unit A3 -- the KMD"
      " owns the venus allocation (HVM1) and the ICD stops exporting one it named",
   [HELIOS_A3_GAP_VIDMM_TRACKER] =
      "the VidMm global-share tracker and its HELIOS_WDDM_ALLOC_KIND_TRACKING"
      " allocation are deleted with UMD-backing adoption and have NO successor"
      " (K4-CONTRACT 6): nothing was folded into HWA2 and nobody should look for it."
      " The export is retained only so an older bridge DLL resolves it and gets a"
      " refusal instead of a stale attestation",
};

static volatile LONG helios_a3_gap_counters[HELIOS_A3_GAP_SITE_COUNT];

const char *
helios_a3_gap_site_name(enum helios_a3_gap_site site)
{
   if ((unsigned)site >= HELIOS_A3_GAP_SITE_COUNT)
      return "unknown_site";
   return helios_a3_gap_site_names[site];
}

void
helios_a3_gap_refuse(enum helios_a3_gap_site site)
{
   if ((unsigned)site >= HELIOS_A3_GAP_SITE_COUNT)
      return;

   const long count = InterlockedIncrement(&helios_a3_gap_counters[site]);
   /* The first refusal at a site carries the whole explanation; after that one
    * line every 256, because a compositor that retries an import per frame
    * would otherwise make the diag log itself the failure. */
   if (count == 1 || (count % 256) == 0) {
      vn_renderer_helios_diag_log(
         "A3 GAP REFUSAL site=%s count=%ld: %s. !! An ICD in this state cannot import"
         " a Win32/D3D12 payload; that is the retirement's intended intermediate"
         " state (K4-CONTRACT 5), not a regression to work around.",
         helios_a3_gap_site_name(site), count, helios_a3_gap_site_reasons[site]);
   }
}

long
helios_a3_gap_refusals(enum helios_a3_gap_site site)
{
   if ((unsigned)site >= HELIOS_A3_GAP_SITE_COUNT)
      return 0;
   return helios_a3_gap_counters[site];
}

void
helios_a3_gap_summary(char *buf, size_t buf_bytes)
{
   if (!buf || !buf_bytes)
      return;
   buf[0] = '\0';
   size_t used = 0;
   bool any = false;
   for (unsigned site = 0; site < HELIOS_A3_GAP_SITE_COUNT; site++) {
      const long count = helios_a3_gap_counters[site];
      if (!count)
         continue;
      const int written =
         snprintf(buf + used, buf_bytes - used, "%s%s=%ld", any ? " " : "",
                  helios_a3_gap_site_names[site], count);
      if (written < 0 || (size_t)written >= buf_bytes - used)
         return;
      used += (size_t)written;
      any = true;
   }
   if (!any)
      snprintf(buf, buf_bytes, "none");
}
