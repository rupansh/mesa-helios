/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A2 -- the native KMT lane (§10.7 "Normal-ICD KMT execution carrier").
 *
 * Two halves. The upper one is PURE: the HNR2 fragment planner/encoder and
 * CRC-64/ECMA-182, no D3DKMT and no Windows, so tools/hnr2_encode_probe.c can
 * compile and RUN it on Linux against protocol/'s own validator. The lower one
 * is the HVC1 context: create/destroy, the Render call shape, the per-context
 * monitored progress fence, and the C51 joins.
 *
 * ⛔ The wire records are protocol/'s, never this file's. See OWNERSHIP.md §1.
 */

#ifndef VN_HELIOS_NATIVE_KMT_H
#define VN_HELIOS_NATIVE_KMT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "helios_native_render.h"

/* ── CRC-64/ECMA-182 ────────────────────────────────────────────────────────
 *
 * The ICD's ONE copy. Parameters come from protocol/include/helios_wddm.h; the
 * self test runs the published check value at session create and refuses, which
 * is what makes a hand-carried table checkable.
 */
uint64_t helios_crc64(const void *bytes, size_t len);
bool helios_crc64_self_test(void);

/* ── the HNR2 encoder ───────────────────────────────────────────────────────*/

/*
 * One entry of a batch's allocation manifest. It becomes both one
 * `D3DDDI_ALLOCATIONLIST` entry and one 24-byte HNR2 use record at the same
 * index, which is how "exactly once in each, with the same WriteOperation"
 * holds by construction instead of by a cross-check.
 */
struct helios_hnr2_allocation {
   /* D3DKMT_HANDLE. uint32_t so this half compiles without d3dkmthk.h; the
    * Windows half static-asserts the two are the same width. */
   uint32_t handle;
   /* HELIOS_HNR2_ACCESS_READ / _WRITE. Nonzero. */
   uint32_t access;
   /* The HVM1 object_generation the KMD returned at create. Nonzero: a zero
    * generation is `Hnr2TableReject::AllocationGenerationZero`. */
   uint64_t expected_generation;
};

/*
 * One typed operand occurrence whose payload bytes the encoder's caller wrote
 * as zero. `encoded_width` is DERIVED from `operand_kind`, so the pair cannot
 * disagree (`Hnr2TableReject::PatchOperandWidthMismatch` is unreachable from
 * here), and the records may arrive in any order -- the encoder groups them
 * into the contiguous per-use runs the validator requires.
 */
struct helios_hnr2_patch_input {
   uint32_t payload_offset; /* within the REASSEMBLED payload, 4-aligned */
   uint32_t allocation_index;
   uint16_t operand_kind; /* HELIOS_HNR2_OPERAND_KIND_* */
};

struct helios_hnr2_batch {
   const void *payload;
   uint64_t payload_bytes;

   const struct helios_hnr2_allocation *allocations;
   uint32_t allocation_count;
   const struct helios_hnr2_patch_input *patches;
   uint32_t patch_count;

   /* The reply descriptor, all zero without a reply. `reply_allocation_index`
    * must name a WRITE entry of the manifest above -- §10.7's "valid writable
    * index". */
   bool has_reply;
   uint32_t reply_allocation_index;
   uint64_t reply_offset;
   uint64_t reply_capacity_bytes;
   uint64_t reply_slot_generation;
};

enum helios_hnr2_encode_status {
   HELIOS_HNR2_ENCODE_OK = 0,
   HELIOS_HNR2_ENCODE_NO_PAYLOAD,
   HELIOS_HNR2_ENCODE_PAYLOAD_TOO_LARGE,
   HELIOS_HNR2_ENCODE_TOO_MANY_FRAGMENTS,
   HELIOS_HNR2_ENCODE_TOO_MANY_USES,
   HELIOS_HNR2_ENCODE_TOO_MANY_PATCHES,
   HELIOS_HNR2_ENCODE_PATCH_WITHOUT_USE,
   HELIOS_HNR2_ENCODE_METADATA_TOO_LARGE,
   HELIOS_HNR2_ENCODE_BAD_ALLOCATION,
   HELIOS_HNR2_ENCODE_BAD_PATCH,
   HELIOS_HNR2_ENCODE_BAD_REPLY,
   HELIOS_HNR2_ENCODE_BAD_FRAGMENT_INDEX,
   HELIOS_HNR2_ENCODE_BUFFER_TOO_SMALL,
   HELIOS_HNR2_ENCODE_BAD_TOKEN,
   HELIOS_HNR2_ENCODE_STATUS_COUNT,
};

const char *helios_hnr2_encode_status_name(enum helios_hnr2_encode_status s);

/*
 * The whole fragment plan, computed once before the first Render.
 *
 * ⭐ It is planned against the ADVERTISED FLOOR (HELIOS_HVC1_DMA_BUFFER_BYTES),
 * never against the buffer dxgkrnl happens to have returned. Dxgkrnl may hand
 * back a different size after every Render, and a plan that moved with it could
 * need a fragment count it has already committed to in fragment 0. §10.7's own
 * worst-case arithmetic (15,965,184 <= 64*262,144) is computed on the floor, so
 * planning on the floor is what the 64-fragment bound already assumes.
 */
struct helios_hnr2_plan {
   uint16_t fragment_count;
   uint32_t commit_metadata_bytes; /* 112 + uses*24 + patches*16 */
   uint64_t full_payload_crc64;
   uint32_t fragment_payload_bytes[HELIOS_HNR2_MAX_FRAGMENTS];
   uint64_t fragment_payload_offset[HELIOS_HNR2_MAX_FRAGMENTS];
};

/* Validates the whole batch and computes the fragment split. Nothing after this
 * can fail for a reason this did not check. */
enum helios_hnr2_encode_status
helios_hnr2_plan_batch(const struct helios_hnr2_batch *batch,
                       uint32_t command_buffer_floor,
                       struct helios_hnr2_plan *out);

/*
 * Encode one fragment into `command_buffer` in the canonical layout
 * (header, use table, patch table, payload -- see helios_native_render.h).
 * `*out_command_length` is the exact `D3DKMT_RENDER::CommandLength`.
 */
enum helios_hnr2_encode_status helios_hnr2_encode_fragment(
   const struct helios_hnr2_batch *batch, const struct helios_hnr2_plan *plan,
   uint16_t fragment_index, uint64_t batch_token, void *command_buffer,
   uint32_t command_buffer_bytes, uint32_t *out_command_length);

/* Exact `D3DKMT_RENDER::AllocationCount` for one fragment: the complete
 * manifest on COMMIT, zero everywhere else (`Hnr2Reject::AllocationListNotEmpty`
 * is a context loss, not a warning). */
uint32_t helios_hnr2_fragment_allocation_count(
   const struct helios_hnr2_batch *batch, const struct helios_hnr2_plan *plan,
   uint16_t fragment_index);

/* ── the HVC1 context ───────────────────────────────────────────────────────*/

#ifdef _WIN32

#include <vulkan/vulkan.h>
#include <windows.h>

/* Same guard as the Windows renderer: d3dkmthk.h's prototypes return NTSTATUS
 * but nothing it includes defines it, and mingw's <windows.h> does not either. */
#ifndef _NTDEF_
typedef LONG NTSTATUS, *PNTSTATUS;
#endif
#include <d3dkmthk.h>

struct helios_native_context;

/* One imported monitored-fence dependency.  The handle is the exact
 * process-local object returned by D3DKMTOpenNativeFenceFromNtHandle; it is
 * never serialized into HNR2. */
struct helios_native_fence_point {
   uint32_t handle;
   uint64_t value;
};

enum helios_native_context_kind {
   /* Both HVC1 queue ordinals UINT32_MAX. One per raw KMT device; carries only
    * C60 pure control, and its C51 value is NEVER a GPU-complete fact. */
   HELIOS_NATIVE_CONTEXT_CONTROL,
   /* One per real lower VkQueue. */
   HELIOS_NATIVE_CONTEXT_QUEUE,
};

/*
 * Create one HVC1 context on an already-open raw KMT device and its own
 * unshared monitored progress fence. Validates the returned `DXGK_CONTEXTINFO`
 * minima and refuses rather than recording into a short list later.
 */
VkResult helios_native_context_create(D3DKMT_HANDLE device,
                                      enum helios_native_context_kind kind,
                                      uint32_t queue_family,
                                      uint32_t queue_index,
                                      struct helios_native_context **out);

void helios_native_context_destroy(struct helios_native_context *c);

D3DKMT_HANDLE
helios_native_context_handle(const struct helios_native_context *c);

/*
 * One finite HNR2 batch: plan, then one D3DKMTRender per fragment with the
 * returned buffers re-adopted after each, then optionally the C51 progress
 * signal enqueued behind it on this same context.
 *
 * ⛔ Any failed Render loses the context permanently -- there is no other
 * transport to fall back to, and every later call on it refuses.
 */
VkResult helios_native_context_submit(struct helios_native_context *c,
                                      const struct helios_hnr2_batch *batch,
                                      bool signal_progress,
                                      uint64_t *out_batch_token,
                                      uint64_t *out_progress_value);

/* Queue form of submit.  Imported native-fence waits, every HNR2 fragment,
 * imported native-fence signals, and the optional C51 signal are enqueued in
 * exactly that order while holding the one context lock.  No CPU wait, helper
 * context, or renderer timeline participates. */
VkResult helios_native_context_submit_ordered(
   struct helios_native_context *c,
   const struct helios_native_fence_point *waits,
   uint32_t wait_count,
   const struct helios_hnr2_batch *batch,
   const struct helios_native_fence_point *signals,
   uint32_t signal_count,
   bool signal_progress,
   uint64_t *out_batch_token,
   uint64_t *out_progress_value);

/*
 * C51 CPU waits. `timeout_ns == UINT64_MAX` blocks inside KMT with no event;
 * a finite wait uses an event that stays ARMED across a timeout (§10.7) so a
 * later wait cannot miss the signal it already asked for.
 */
#define HELIOS_NATIVE_WAIT_INFINITE UINT64_MAX
VkResult helios_native_context_wait(struct helios_native_context *c,
                                    uint64_t value, uint64_t timeout_ns);

/* Enqueue the next progress value and return it, without waiting. */
VkResult helios_native_context_signal(struct helios_native_context *c,
                                      uint64_t *out_value);

/* §10.7: `vkQueueWaitIdle` waits the latest C51 value on this context -- so a
 * batch submitted fire-and-forget gets one signal enqueued first. */
VkResult helios_native_queue_wait_idle(struct helios_native_context *c);

/*
 * §10.7: snapshot every live nonzero queue context, arm/wait each latest
 * milestone without holding a lock across KMT, and fail the device if any
 * fails. ⛔ A control C51 value is never substituted for this join, so a
 * control context in the array is refused.
 */
VkResult helios_native_device_wait_idle(struct helios_native_context *const *contexts,
                                        uint32_t count);

/* Refusal census, printed beside A1's. */
enum helios_native_refusal_site {
   HELIOS_NATIVE_REFUSE_CONTEXT,
   HELIOS_NATIVE_REFUSE_CONTEXT_MINIMA,
   HELIOS_NATIVE_REFUSE_FENCE,
   HELIOS_NATIVE_REFUSE_ENCODE,
   HELIOS_NATIVE_REFUSE_LIST_CAPACITY,
   HELIOS_NATIVE_REFUSE_RESIZE,
   HELIOS_NATIVE_REFUSE_RENDER,
   HELIOS_NATIVE_REFUSE_BUFFER_ADOPT,
   HELIOS_NATIVE_REFUSE_SIGNAL,
   HELIOS_NATIVE_REFUSE_WAIT,
   HELIOS_NATIVE_REFUSE_WAIT_ARMS,
   HELIOS_NATIVE_REFUSE_NATIVE_FENCE_WAIT,
   HELIOS_NATIVE_REFUSE_NATIVE_FENCE_SIGNAL,
   HELIOS_NATIVE_REFUSE_CONTEXT_LOST,
   HELIOS_NATIVE_REFUSE_SITE_COUNT,
};

long helios_native_refusals(enum helios_native_refusal_site site);
void helios_native_refusal_summary(char *out, size_t out_bytes);

#endif /* _WIN32 */
#endif /* VN_HELIOS_NATIVE_KMT_H */
