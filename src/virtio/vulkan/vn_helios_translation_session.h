/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A1 -- the HTS1 translation session (§10.4 "HTS1 pre-queue control and
 * outer-context attachment", §10.7 "Normal-ICD KMT execution carrier").
 *
 * One of these exists per vn_instance. It owns the raw KMT device, the single
 * HVC1 control context, the role-1 HVM1 reply pool, and the session identity the
 * KMD returns from the finite HNR2 INIT. A2 creates its queue contexts on this
 * device; A3 allocates through it; the outer UMD attaches to it with HQA1.
 *
 * ⛔ The wire records are protocol/'s, never this file's. See OWNERSHIP.md §1.
 */

#ifndef VN_HELIOS_TRANSLATION_SESSION_H
#define VN_HELIOS_TRANSLATION_SESSION_H

#ifdef _WIN32

#include <stdbool.h>
#include <stdint.h>

#include <vulkan/vulkan.h>
#include <windows.h>

/* d3dkmthk.h's prototypes return NTSTATUS but it pulls no header that defines
 * it, and mingw's <windows.h> does not either. Same guard as
 * the selected Windows renderer. */
#ifndef _NTDEF_
typedef LONG NTSTATUS, *PNTSTATUS;
#endif
#include <d3dkmthk.h>

#include "helios_native_render.h"
#include "helios_translation_session.h"

struct helios_translation_session;
struct helios_native_context;

/*
 * Build the session: open the adapter, create the raw KMT device, create the one
 * HVC1 control context, create + make resident + Lock2-map the 4 MiB role-1
 * reply pool, then run the finite INIT and capture what it returns.
 *
 * `requested_endpoint_capacity` is 1..=HELIOS_HTS1_MAX_ENDPOINTS_PER_SESSION; the
 * reply is authoritative and may grant fewer.
 *
 * ⛔ Any failure unwinds completely and the caller MUST fail translator/device
 * creation: §10.4 forbids attaching a session late.
 */
VkResult helios_translation_session_create(
   LUID adapter_luid, uint32_t requested_endpoint_capacity,
   struct helios_translation_session **out);

void helios_translation_session_destroy(struct helios_translation_session *s);

/* Session identity, valid only after a successful create. */
uint64_t helios_session_generation(const struct helios_translation_session *s);
uint32_t helios_session_endpoint_capacity(
   const struct helios_translation_session *s);
uint32_t helios_session_capset(const struct helios_translation_session *s);

/*
 * The 128-bit admission nonce. ⛔ NEVER log it, hash it, derive from it, or put
 * it anywhere but HQA1's create-context private data (§10.4: "unpredictable
 * admission nonces, not resource IDs").
 */
void helios_session_capability(const struct helios_translation_session *s,
                               uint64_t *low, uint64_t *high);

/* The raw KMT device A2's queue contexts are created on. */
D3DKMT_HANDLE helios_session_device_handle(
   const struct helios_translation_session *s);

/* Exact WDDM local-segment capacity backing the one A6 HLM1 Vulkan heap.
 * This is an OS-owned adapter query on this session's directly owned adapter;
 * there is no registry estimate or renderer-heap substitution. */
VkResult helios_session_local_heap_size(
   const struct helios_translation_session *s, uint64_t *out_size);

/*
 * Execute the one reply-bearing normal-context operation admitted by A3:
 *
 *   vkSetReplyCommandStreamMESA(resourceId = 0) +
 *   vkAllocateMemory(VK_COMMAND_GENERATE_REPLY_BIT_EXT,
 *                    VkImportMemoryResourceInfoMESA(resourceId = 0))
 *
 * `payload` is a caller-owned, generated-Venus byte stream.  The session fills
 * only the SetReply offset/size fields after checking the fixed generated
 * layout; both host-resource operands remain zero.  `import_operand_offset`
 * names the generated import operand and is repeated as the second typed patch.
 * The first use is always this session's checked-out role-1 slot, and the
 * second is exactly `target_allocation`/`target_generation`.
 *
 * Completion is the HVR1 publication produced only after the exact host
 * AllocateMemory reply.  A C51 wait on `context` orders the CPU decode; no
 * control-context milestone is substituted for it.
 */
VkResult helios_session_execute_allocate(
   struct helios_translation_session *s,
   struct helios_native_context *context,
   void *payload,
   uint64_t payload_bytes,
   uint32_t import_operand_offset,
   D3DKMT_HANDLE target_allocation,
   uint64_t target_generation,
   void *raw_reply,
   uint64_t raw_reply_capacity,
   uint64_t *raw_reply_bytes,
   int32_t *reply_status);

/*
 * One bounded C60 pure-control transaction on the HVC1 control context: encode
 * `payload` as a finite HNR2 batch with HAS_REPLY into a checked-out reply slot,
 * Render it, wait on the C51 event, and copy the HVR1 chunk out.
 *
 * `reply_status` receives HVR1's signed status (VK_INCOMPLETE and friends are
 * legal outcomes of a bounded rule, not errors). A result larger than
 * `reply_capacity` is continued by calling helios_session_control_continue with
 * the returned `snapshot_generation` and `total_bytes` and the accumulated
 * offset -- all three are checked against the next reply's own header, so a
 * caller that loses track is refused rather than spliced.
 *
 * ⛔ The payload may not name an outer D3D allocation. The only allocation this
 * carrier ever lists is the session's own reply slot.
 *
 * ⚠ BLOCKS when all four reply slots are checked out (§10.7: "the caller drops
 * the lock and event-waits ... then retries"). It is not a resource failure --
 * several entry points that reach it return `void` and could not report one.
 */
VkResult helios_session_control(struct helios_translation_session *s,
                                const void *payload, uint64_t payload_bytes,
                                void *reply, uint64_t reply_capacity,
                                uint64_t *reply_bytes, uint64_t *total_bytes,
                                int32_t *reply_status,
                                uint64_t *snapshot_generation, bool *more);

/*
 * Fetch the next chunk of an already-computed result. Never re-executes the
 * original operation and only ever asks for the exact next offset (§C66).
 *
 * `expected_offset` and `expected_total_bytes` are what the previous chunk
 * produced, and both are required: slot generation and batch token prove only
 * that a reply belongs to THIS Render, never that it is the requested range of
 * the requested snapshot.
 */
VkResult helios_session_control_continue(struct helios_translation_session *s,
                                         uint64_t snapshot_generation,
                                         uint64_t expected_offset,
                                         uint64_t expected_total_bytes,
                                         void *reply, uint64_t reply_capacity,
                                         uint64_t *reply_bytes,
                                         int32_t *reply_status, bool *more);

/* Refusal census, printed beside the A3 gaps.
 *
 * ⚠ CONTEXT_MINIMA and FENCE moved to A2 with the context itself and are no
 * longer incremented here; the equivalents are
 * HELIOS_NATIVE_REFUSE_CONTEXT_MINIMA / _FENCE. They stay so a stored counter
 * name keeps meaning what it meant. */
enum helios_session_refusal_site {
   HELIOS_SESSION_REFUSE_ADAPTER,
   HELIOS_SESSION_REFUSE_DEVICE,
   HELIOS_SESSION_REFUSE_CONTEXT,
   HELIOS_SESSION_REFUSE_CONTEXT_MINIMA,
   HELIOS_SESSION_REFUSE_POOL_CREATE,
   HELIOS_SESSION_REFUSE_POOL_RESIDENT,
   HELIOS_SESSION_REFUSE_POOL_LOCK,
   HELIOS_SESSION_REFUSE_FENCE,
   HELIOS_SESSION_REFUSE_INIT_RENDER,
   HELIOS_SESSION_REFUSE_INIT_WAIT,
   HELIOS_SESSION_REFUSE_INIT_REPLY,
   HELIOS_SESSION_REFUSE_SLOT_EXHAUSTED,
   HELIOS_SESSION_REFUSE_CONTROL_RENDER,
   HELIOS_SESSION_REFUSE_CONTROL_WAIT,
   HELIOS_SESSION_REFUSE_CONTROL_REPLY,
   HELIOS_SESSION_REFUSE_PAYLOAD_TOO_LARGE,
   HELIOS_SESSION_REFUSE_EXEC_ALLOC_RENDER,
   HELIOS_SESSION_REFUSE_EXEC_ALLOC_WAIT,
   HELIOS_SESSION_REFUSE_EXEC_ALLOC_REPLY,
   HELIOS_SESSION_REFUSE_SITE_COUNT,
};

long helios_session_refusals(enum helios_session_refusal_site site);
void helios_session_refusal_summary(char *out, size_t out_bytes);

#endif /* _WIN32 */
#endif /* VN_HELIOS_TRANSLATION_SESSION_H */
