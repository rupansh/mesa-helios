/*
 * Copyright 2026 Helios
 * SPDX-License-Identifier: MIT
 *
 * A2 -- the native KMT lane. See vn_helios_native_kmt.h.
 */

#include "vn_helios_native_kmt.h"

#include <string.h>

#include "helios_wddm.h" /* HELIOS_CRC64_ECMA182_* -- the ONE declaration */

/* ══ the pure half: CRC-64/ECMA-182 and the HNR2 encoder ═══════════════════ */

/*
 * Generated, not derived at run time: a lazily built table is a data race on
 * every ICD thread that submits, and its only cure (a Windows InitOnce) would
 * not compile in the half this file shares with the Linux probe.
 *
 *   python3 -c 'P=0x42F0E1EBA9EA3693;M=(1<<64)-1
 *   [print(hex((lambda c:[c:=((c<<1)^P)&M if c>>63 else (c<<1)&M for _ in
 *   range(8)][-1])(n<<56))) for n in range(256)]'
 *
 * helios_crc64_self_test() runs the published check value against it, so a
 * mistyped entry cannot ship silently.
 */
static const uint64_t helios_crc64_table[256] = {
   UINT64_C(0x0000000000000000), UINT64_C(0x42F0E1EBA9EA3693), UINT64_C(0x85E1C3D753D46D26), UINT64_C(0xC711223CFA3E5BB5),
   UINT64_C(0x493366450E42ECDF), UINT64_C(0x0BC387AEA7A8DA4C), UINT64_C(0xCCD2A5925D9681F9), UINT64_C(0x8E224479F47CB76A),
   UINT64_C(0x9266CC8A1C85D9BE), UINT64_C(0xD0962D61B56FEF2D), UINT64_C(0x17870F5D4F51B498), UINT64_C(0x5577EEB6E6BB820B),
   UINT64_C(0xDB55AACF12C73561), UINT64_C(0x99A54B24BB2D03F2), UINT64_C(0x5EB4691841135847), UINT64_C(0x1C4488F3E8F96ED4),
   UINT64_C(0x663D78FF90E185EF), UINT64_C(0x24CD9914390BB37C), UINT64_C(0xE3DCBB28C335E8C9), UINT64_C(0xA12C5AC36ADFDE5A),
   UINT64_C(0x2F0E1EBA9EA36930), UINT64_C(0x6DFEFF5137495FA3), UINT64_C(0xAAEFDD6DCD770416), UINT64_C(0xE81F3C86649D3285),
   UINT64_C(0xF45BB4758C645C51), UINT64_C(0xB6AB559E258E6AC2), UINT64_C(0x71BA77A2DFB03177), UINT64_C(0x334A9649765A07E4),
   UINT64_C(0xBD68D2308226B08E), UINT64_C(0xFF9833DB2BCC861D), UINT64_C(0x388911E7D1F2DDA8), UINT64_C(0x7A79F00C7818EB3B),
   UINT64_C(0xCC7AF1FF21C30BDE), UINT64_C(0x8E8A101488293D4D), UINT64_C(0x499B3228721766F8), UINT64_C(0x0B6BD3C3DBFD506B),
   UINT64_C(0x854997BA2F81E701), UINT64_C(0xC7B97651866BD192), UINT64_C(0x00A8546D7C558A27), UINT64_C(0x4258B586D5BFBCB4),
   UINT64_C(0x5E1C3D753D46D260), UINT64_C(0x1CECDC9E94ACE4F3), UINT64_C(0xDBFDFEA26E92BF46), UINT64_C(0x990D1F49C77889D5),
   UINT64_C(0x172F5B3033043EBF), UINT64_C(0x55DFBADB9AEE082C), UINT64_C(0x92CE98E760D05399), UINT64_C(0xD03E790CC93A650A),
   UINT64_C(0xAA478900B1228E31), UINT64_C(0xE8B768EB18C8B8A2), UINT64_C(0x2FA64AD7E2F6E317), UINT64_C(0x6D56AB3C4B1CD584),
   UINT64_C(0xE374EF45BF6062EE), UINT64_C(0xA1840EAE168A547D), UINT64_C(0x66952C92ECB40FC8), UINT64_C(0x2465CD79455E395B),
   UINT64_C(0x3821458AADA7578F), UINT64_C(0x7AD1A461044D611C), UINT64_C(0xBDC0865DFE733AA9), UINT64_C(0xFF3067B657990C3A),
   UINT64_C(0x711223CFA3E5BB50), UINT64_C(0x33E2C2240A0F8DC3), UINT64_C(0xF4F3E018F031D676), UINT64_C(0xB60301F359DBE0E5),
   UINT64_C(0xDA050215EA6C212F), UINT64_C(0x98F5E3FE438617BC), UINT64_C(0x5FE4C1C2B9B84C09), UINT64_C(0x1D14202910527A9A),
   UINT64_C(0x93366450E42ECDF0), UINT64_C(0xD1C685BB4DC4FB63), UINT64_C(0x16D7A787B7FAA0D6), UINT64_C(0x5427466C1E109645),
   UINT64_C(0x4863CE9FF6E9F891), UINT64_C(0x0A932F745F03CE02), UINT64_C(0xCD820D48A53D95B7), UINT64_C(0x8F72ECA30CD7A324),
   UINT64_C(0x0150A8DAF8AB144E), UINT64_C(0x43A04931514122DD), UINT64_C(0x84B16B0DAB7F7968), UINT64_C(0xC6418AE602954FFB),
   UINT64_C(0xBC387AEA7A8DA4C0), UINT64_C(0xFEC89B01D3679253), UINT64_C(0x39D9B93D2959C9E6), UINT64_C(0x7B2958D680B3FF75),
   UINT64_C(0xF50B1CAF74CF481F), UINT64_C(0xB7FBFD44DD257E8C), UINT64_C(0x70EADF78271B2539), UINT64_C(0x321A3E938EF113AA),
   UINT64_C(0x2E5EB66066087D7E), UINT64_C(0x6CAE578BCFE24BED), UINT64_C(0xABBF75B735DC1058), UINT64_C(0xE94F945C9C3626CB),
   UINT64_C(0x676DD025684A91A1), UINT64_C(0x259D31CEC1A0A732), UINT64_C(0xE28C13F23B9EFC87), UINT64_C(0xA07CF2199274CA14),
   UINT64_C(0x167FF3EACBAF2AF1), UINT64_C(0x548F120162451C62), UINT64_C(0x939E303D987B47D7), UINT64_C(0xD16ED1D631917144),
   UINT64_C(0x5F4C95AFC5EDC62E), UINT64_C(0x1DBC74446C07F0BD), UINT64_C(0xDAAD56789639AB08), UINT64_C(0x985DB7933FD39D9B),
   UINT64_C(0x84193F60D72AF34F), UINT64_C(0xC6E9DE8B7EC0C5DC), UINT64_C(0x01F8FCB784FE9E69), UINT64_C(0x43081D5C2D14A8FA),
   UINT64_C(0xCD2A5925D9681F90), UINT64_C(0x8FDAB8CE70822903), UINT64_C(0x48CB9AF28ABC72B6), UINT64_C(0x0A3B7B1923564425),
   UINT64_C(0x70428B155B4EAF1E), UINT64_C(0x32B26AFEF2A4998D), UINT64_C(0xF5A348C2089AC238), UINT64_C(0xB753A929A170F4AB),
   UINT64_C(0x3971ED50550C43C1), UINT64_C(0x7B810CBBFCE67552), UINT64_C(0xBC902E8706D82EE7), UINT64_C(0xFE60CF6CAF321874),
   UINT64_C(0xE224479F47CB76A0), UINT64_C(0xA0D4A674EE214033), UINT64_C(0x67C58448141F1B86), UINT64_C(0x253565A3BDF52D15),
   UINT64_C(0xAB1721DA49899A7F), UINT64_C(0xE9E7C031E063ACEC), UINT64_C(0x2EF6E20D1A5DF759), UINT64_C(0x6C0603E6B3B7C1CA),
   UINT64_C(0xF6FAE5C07D3274CD), UINT64_C(0xB40A042BD4D8425E), UINT64_C(0x731B26172EE619EB), UINT64_C(0x31EBC7FC870C2F78),
   UINT64_C(0xBFC9838573709812), UINT64_C(0xFD39626EDA9AAE81), UINT64_C(0x3A28405220A4F534), UINT64_C(0x78D8A1B9894EC3A7),
   UINT64_C(0x649C294A61B7AD73), UINT64_C(0x266CC8A1C85D9BE0), UINT64_C(0xE17DEA9D3263C055), UINT64_C(0xA38D0B769B89F6C6),
   UINT64_C(0x2DAF4F0F6FF541AC), UINT64_C(0x6F5FAEE4C61F773F), UINT64_C(0xA84E8CD83C212C8A), UINT64_C(0xEABE6D3395CB1A19),
   UINT64_C(0x90C79D3FEDD3F122), UINT64_C(0xD2377CD44439C7B1), UINT64_C(0x15265EE8BE079C04), UINT64_C(0x57D6BF0317EDAA97),
   UINT64_C(0xD9F4FB7AE3911DFD), UINT64_C(0x9B041A914A7B2B6E), UINT64_C(0x5C1538ADB04570DB), UINT64_C(0x1EE5D94619AF4648),
   UINT64_C(0x02A151B5F156289C), UINT64_C(0x4051B05E58BC1E0F), UINT64_C(0x87409262A28245BA), UINT64_C(0xC5B073890B687329),
   UINT64_C(0x4B9237F0FF14C443), UINT64_C(0x0962D61B56FEF2D0), UINT64_C(0xCE73F427ACC0A965), UINT64_C(0x8C8315CC052A9FF6),
   UINT64_C(0x3A80143F5CF17F13), UINT64_C(0x7870F5D4F51B4980), UINT64_C(0xBF61D7E80F251235), UINT64_C(0xFD913603A6CF24A6),
   UINT64_C(0x73B3727A52B393CC), UINT64_C(0x31439391FB59A55F), UINT64_C(0xF652B1AD0167FEEA), UINT64_C(0xB4A25046A88DC879),
   UINT64_C(0xA8E6D8B54074A6AD), UINT64_C(0xEA16395EE99E903E), UINT64_C(0x2D071B6213A0CB8B), UINT64_C(0x6FF7FA89BA4AFD18),
   UINT64_C(0xE1D5BEF04E364A72), UINT64_C(0xA3255F1BE7DC7CE1), UINT64_C(0x64347D271DE22754), UINT64_C(0x26C49CCCB40811C7),
   UINT64_C(0x5CBD6CC0CC10FAFC), UINT64_C(0x1E4D8D2B65FACC6F), UINT64_C(0xD95CAF179FC497DA), UINT64_C(0x9BAC4EFC362EA149),
   UINT64_C(0x158E0A85C2521623), UINT64_C(0x577EEB6E6BB820B0), UINT64_C(0x906FC95291867B05), UINT64_C(0xD29F28B9386C4D96),
   UINT64_C(0xCEDBA04AD0952342), UINT64_C(0x8C2B41A1797F15D1), UINT64_C(0x4B3A639D83414E64), UINT64_C(0x09CA82762AAB78F7),
   UINT64_C(0x87E8C60FDED7CF9D), UINT64_C(0xC51827E4773DF90E), UINT64_C(0x020905D88D03A2BB), UINT64_C(0x40F9E43324E99428),
   UINT64_C(0x2CFFE7D5975E55E2), UINT64_C(0x6E0F063E3EB46371), UINT64_C(0xA91E2402C48A38C4), UINT64_C(0xEBEEC5E96D600E57),
   UINT64_C(0x65CC8190991CB93D), UINT64_C(0x273C607B30F68FAE), UINT64_C(0xE02D4247CAC8D41B), UINT64_C(0xA2DDA3AC6322E288),
   UINT64_C(0xBE992B5F8BDB8C5C), UINT64_C(0xFC69CAB42231BACF), UINT64_C(0x3B78E888D80FE17A), UINT64_C(0x7988096371E5D7E9),
   UINT64_C(0xF7AA4D1A85996083), UINT64_C(0xB55AACF12C735610), UINT64_C(0x724B8ECDD64D0DA5), UINT64_C(0x30BB6F267FA73B36),
   UINT64_C(0x4AC29F2A07BFD00D), UINT64_C(0x08327EC1AE55E69E), UINT64_C(0xCF235CFD546BBD2B), UINT64_C(0x8DD3BD16FD818BB8),
   UINT64_C(0x03F1F96F09FD3CD2), UINT64_C(0x41011884A0170A41), UINT64_C(0x86103AB85A2951F4), UINT64_C(0xC4E0DB53F3C36767),
   UINT64_C(0xD8A453A01B3A09B3), UINT64_C(0x9A54B24BB2D03F20), UINT64_C(0x5D45907748EE6495), UINT64_C(0x1FB5719CE1045206),
   UINT64_C(0x919735E51578E56C), UINT64_C(0xD367D40EBC92D3FF), UINT64_C(0x1476F63246AC884A), UINT64_C(0x568617D9EF46BED9),
   UINT64_C(0xE085162AB69D5E3C), UINT64_C(0xA275F7C11F7768AF), UINT64_C(0x6564D5FDE549331A), UINT64_C(0x279434164CA30589),
   UINT64_C(0xA9B6706FB8DFB2E3), UINT64_C(0xEB46918411358470), UINT64_C(0x2C57B3B8EB0BDFC5), UINT64_C(0x6EA7525342E1E956),
   UINT64_C(0x72E3DAA0AA188782), UINT64_C(0x30133B4B03F2B111), UINT64_C(0xF7021977F9CCEAA4), UINT64_C(0xB5F2F89C5026DC37),
   UINT64_C(0x3BD0BCE5A45A6B5D), UINT64_C(0x79205D0E0DB05DCE), UINT64_C(0xBE317F32F78E067B), UINT64_C(0xFCC19ED95E6430E8),
   UINT64_C(0x86B86ED5267CDBD3), UINT64_C(0xC4488F3E8F96ED40), UINT64_C(0x0359AD0275A8B6F5), UINT64_C(0x41A94CE9DC428066),
   UINT64_C(0xCF8B0890283E370C), UINT64_C(0x8D7BE97B81D4019F), UINT64_C(0x4A6ACB477BEA5A2A), UINT64_C(0x089A2AACD2006CB9),
   UINT64_C(0x14DEA25F3AF9026D), UINT64_C(0x562E43B4931334FE), UINT64_C(0x913F6188692D6F4B), UINT64_C(0xD3CF8063C0C759D8),
   UINT64_C(0x5DEDC41A34BBEEB2), UINT64_C(0x1F1D25F19D51D821), UINT64_C(0xD80C07CD676F8394), UINT64_C(0x9AFCE626CE85B507),
};

uint64_t
helios_crc64(const void *bytes, size_t len)
{
   const uint8_t *p = (const uint8_t *)bytes;
   uint64_t crc = HELIOS_CRC64_ECMA182_INIT;
   for (size_t i = 0; i < len; i++)
      crc = helios_crc64_table[(uint8_t)((crc >> 56) ^ p[i])] ^ (crc << 8);
   return crc ^ HELIOS_CRC64_ECMA182_XOROUT;
}

bool
helios_crc64_self_test(void)
{
   return helios_crc64("123456789", 9) == HELIOS_CRC64_ECMA182_CHECK;
}

/* ── encoder ────────────────────────────────────────────────────────────────*/

static const char *const helios_hnr2_status_names[HELIOS_HNR2_ENCODE_STATUS_COUNT] = {
   [HELIOS_HNR2_ENCODE_OK] = "ok",
   [HELIOS_HNR2_ENCODE_NO_PAYLOAD] = "no_payload",
   [HELIOS_HNR2_ENCODE_PAYLOAD_TOO_LARGE] = "payload_too_large",
   [HELIOS_HNR2_ENCODE_TOO_MANY_FRAGMENTS] = "too_many_fragments",
   [HELIOS_HNR2_ENCODE_TOO_MANY_USES] = "too_many_uses",
   [HELIOS_HNR2_ENCODE_TOO_MANY_PATCHES] = "too_many_patches",
   [HELIOS_HNR2_ENCODE_PATCH_WITHOUT_USE] = "patch_without_use",
   [HELIOS_HNR2_ENCODE_METADATA_TOO_LARGE] = "metadata_too_large",
   [HELIOS_HNR2_ENCODE_BAD_ALLOCATION] = "bad_allocation",
   [HELIOS_HNR2_ENCODE_BAD_PATCH] = "bad_patch",
   [HELIOS_HNR2_ENCODE_BAD_REPLY] = "bad_reply",
   [HELIOS_HNR2_ENCODE_BAD_FRAGMENT_INDEX] = "bad_fragment_index",
   [HELIOS_HNR2_ENCODE_BUFFER_TOO_SMALL] = "buffer_too_small",
   [HELIOS_HNR2_ENCODE_BAD_TOKEN] = "bad_token",
};

const char *
helios_hnr2_encode_status_name(enum helios_hnr2_encode_status s)
{
   if ((unsigned)s >= HELIOS_HNR2_ENCODE_STATUS_COUNT)
      return "unknown";
   return helios_hnr2_status_names[s];
}

/* Mirror of protocol's `hnr2_operand_width`. Zero means "outside the generated
 * vocabulary", which is a refusal and never a default width. */
static uint16_t
helios_hnr2_operand_width(uint16_t operand_kind)
{
   switch (operand_kind) {
   case HELIOS_HNR2_OPERAND_KIND_HOST_RESOURCE_ID32:
      return HELIOS_HNR2_OPERAND_WIDTH_32;
   case HELIOS_HNR2_OPERAND_KIND_HOST_RESOURCE_ID64:
      return HELIOS_HNR2_OPERAND_WIDTH_64;
   default:
      return 0;
   }
}

static enum helios_hnr2_encode_status
helios_hnr2_validate_reply(const struct helios_hnr2_batch *b)
{
   if (!b->has_reply) {
      if (b->reply_allocation_index || b->reply_offset ||
          b->reply_capacity_bytes || b->reply_slot_generation)
         return HELIOS_HNR2_ENCODE_BAD_REPLY;
      return HELIOS_HNR2_ENCODE_OK;
   }

   if (b->reply_allocation_index >= b->allocation_count)
      return HELIOS_HNR2_ENCODE_BAD_REPLY;
   if (!(b->allocations[b->reply_allocation_index].access &
         HELIOS_HNR2_ACCESS_WRITE))
      return HELIOS_HNR2_ENCODE_BAD_REPLY;
   if (b->reply_slot_generation == 0)
      return HELIOS_HNR2_ENCODE_BAD_REPLY;
   if (b->reply_capacity_bytes < HELIOS_HVR1_HEADER_SIZE ||
       b->reply_capacity_bytes >
          (uint64_t)HELIOS_HVR1_HEADER_SIZE + HELIOS_HVR1_MAX_CHUNK_BYTES)
      return HELIOS_HNR2_ENCODE_BAD_REPLY;
   if (b->reply_offset % HELIOS_HNR2_REPLY_OFFSET_ALIGN)
      return HELIOS_HNR2_ENCODE_BAD_REPLY;

   const uint64_t slot = b->reply_offset / HELIOS_HVM1_REPLY_SLOT_BYTES;
   if (slot >= HELIOS_HVM1_REPLY_SLOT_COUNT)
      return HELIOS_HNR2_ENCODE_BAD_REPLY;
   /* No overflow: capacity is already bounded by 80 + 15 MiB. */
   if (b->reply_offset + b->reply_capacity_bytes >
       (slot + 1) * HELIOS_HVM1_REPLY_SLOT_BYTES)
      return HELIOS_HNR2_ENCODE_BAD_REPLY;
   return HELIOS_HNR2_ENCODE_OK;
}

enum helios_hnr2_encode_status
helios_hnr2_plan_batch(const struct helios_hnr2_batch *b,
                       uint32_t command_buffer_floor,
                       struct helios_hnr2_plan *out)
{
   if (!b || !out)
      return HELIOS_HNR2_ENCODE_NO_PAYLOAD;
   memset(out, 0, sizeof(*out));

   if (!b->payload || b->payload_bytes == 0)
      return HELIOS_HNR2_ENCODE_NO_PAYLOAD;
   if (b->payload_bytes > HELIOS_HNR2_MAX_PAYLOAD_BYTES)
      return HELIOS_HNR2_ENCODE_PAYLOAD_TOO_LARGE;
   if (b->allocation_count > HELIOS_HNR2_MAX_USE_RECORDS)
      return HELIOS_HNR2_ENCODE_TOO_MANY_USES;
   if (b->patch_count > HELIOS_HNR2_MAX_PATCH_RECORDS)
      return HELIOS_HNR2_ENCODE_TOO_MANY_PATCHES;
   if (b->allocation_count && !b->allocations)
      return HELIOS_HNR2_ENCODE_BAD_ALLOCATION;
   if (b->patch_count && !b->patches)
      return HELIOS_HNR2_ENCODE_BAD_PATCH;
   if (b->patch_count && !b->allocation_count)
      return HELIOS_HNR2_ENCODE_PATCH_WITHOUT_USE;

   for (uint32_t i = 0; i < b->allocation_count; i++) {
      const struct helios_hnr2_allocation *a = &b->allocations[i];
      if (!a->handle || !a->expected_generation)
         return HELIOS_HNR2_ENCODE_BAD_ALLOCATION;
      if (!a->access || (a->access & ~(uint32_t)HELIOS_HNR2_ACCESS_MASK))
         return HELIOS_HNR2_ENCODE_BAD_ALLOCATION;
   }

   for (uint32_t i = 0; i < b->patch_count; i++) {
      const struct helios_hnr2_patch_input *p = &b->patches[i];
      if (p->allocation_index >= b->allocation_count)
         return HELIOS_HNR2_ENCODE_BAD_PATCH;
      const uint16_t width = helios_hnr2_operand_width(p->operand_kind);
      if (!width)
         return HELIOS_HNR2_ENCODE_BAD_PATCH;
      if (p->payload_offset % HELIOS_HNR2_OPERAND_ALIGN)
         return HELIOS_HNR2_ENCODE_BAD_PATCH;
      if ((uint64_t)p->payload_offset + width > b->payload_bytes)
         return HELIOS_HNR2_ENCODE_BAD_PATCH;
   }

   const enum helios_hnr2_encode_status reply = helios_hnr2_validate_reply(b);
   if (reply != HELIOS_HNR2_ENCODE_OK)
      return reply;

   /* At most 112 + 4096*24 + 8192*16 = 229,488 -- inside u32 by construction. */
   const uint32_t metadata =
      HELIOS_HNR2_HEADER_SIZE +
      b->allocation_count * HELIOS_HNR2_USE_RECORD_SIZE +
      b->patch_count * HELIOS_HNR2_PATCH_RECORD_SIZE;

   if (command_buffer_floor < HELIOS_HVC1_DMA_BUFFER_BYTES)
      return HELIOS_HNR2_ENCODE_BUFFER_TOO_SMALL;
   /* The COMMIT fragment must still be able to carry one payload byte. */
   if ((uint64_t)metadata + 1 > command_buffer_floor)
      return HELIOS_HNR2_ENCODE_METADATA_TOO_LARGE;

   const uint32_t nonfinal_capacity =
      command_buffer_floor - HELIOS_HNR2_HEADER_SIZE;

   uint64_t remaining = b->payload_bytes;
   uint64_t offset = 0;
   uint32_t count = 0;
   while (remaining) {
      if (count >= HELIOS_HNR2_MAX_FRAGMENTS)
         return HELIOS_HNR2_ENCODE_TOO_MANY_FRAGMENTS;
      uint64_t take;
      if (remaining + metadata <= command_buffer_floor) {
         take = remaining; /* this is the COMMIT fragment */
      } else {
         take = nonfinal_capacity;
         /* A non-final fragment may never exhaust the payload
          * (`Hnr2Reject::NonFinalFragmentExhaustsPayload`); with a large COMMIT
          * metadata block, `nonfinal_capacity` can exceed what is left. */
         if (take >= remaining)
            take = remaining - 1;
      }
      out->fragment_payload_offset[count] = offset;
      out->fragment_payload_bytes[count] = (uint32_t)take;
      offset += take;
      remaining -= take;
      count++;
   }

   out->fragment_count = (uint16_t)count;
   out->commit_metadata_bytes = metadata;
   out->full_payload_crc64 = helios_crc64(b->payload, (size_t)b->payload_bytes);
   return HELIOS_HNR2_ENCODE_OK;
}

uint32_t
helios_hnr2_fragment_allocation_count(const struct helios_hnr2_batch *b,
                                      const struct helios_hnr2_plan *plan,
                                      uint16_t fragment_index)
{
   if (!b || !plan || fragment_index + 1 != plan->fragment_count)
      return 0;
   return b->allocation_count;
}

static void
helios_hnr2_read_use(const uint8_t *base, uint32_t i, HeliosNativeRenderUse *u)
{
   memcpy(u, base + (size_t)i * HELIOS_HNR2_USE_RECORD_SIZE, sizeof(*u));
}

static void
helios_hnr2_write_use(uint8_t *base, uint32_t i, const HeliosNativeRenderUse *u)
{
   memcpy(base + (size_t)i * HELIOS_HNR2_USE_RECORD_SIZE, u, sizeof(*u));
}

/*
 * Group the caller's patches into the contiguous per-use runs
 * `validate_commit_tables` requires, in use order, with no scratch memory: the
 * use records' own `patch_count` fields serve as the counting array and then as
 * the placement cursor, so they hold the right value when the third pass ends.
 */
static void
helios_hnr2_write_tables(const struct helios_hnr2_batch *b, uint8_t *use_base,
                         uint8_t *patch_base)
{
   for (uint32_t i = 0; i < b->allocation_count; i++) {
      HeliosNativeRenderUse u;
      memset(&u, 0, sizeof(u));
      u.allocation_list_index = i;
      u.access_flags = b->allocations[i].access;
      u.expected_allocation_generation = b->allocations[i].expected_generation;
      helios_hnr2_write_use(use_base, i, &u);
   }

   for (uint32_t i = 0; i < b->patch_count; i++) {
      HeliosNativeRenderUse u;
      helios_hnr2_read_use(use_base, b->patches[i].allocation_index, &u);
      u.patch_count++;
      helios_hnr2_write_use(use_base, b->patches[i].allocation_index, &u);
   }

   uint32_t running = 0;
   for (uint32_t i = 0; i < b->allocation_count; i++) {
      HeliosNativeRenderUse u;
      helios_hnr2_read_use(use_base, i, &u);
      u.first_patch = running;
      running += u.patch_count;
      u.patch_count = 0;
      helios_hnr2_write_use(use_base, i, &u);
   }

   for (uint32_t i = 0; i < b->patch_count; i++) {
      const struct helios_hnr2_patch_input *in = &b->patches[i];
      HeliosNativeRenderUse u;
      helios_hnr2_read_use(use_base, in->allocation_index, &u);
      const uint32_t slot = u.first_patch + u.patch_count;
      u.patch_count++;
      helios_hnr2_write_use(use_base, in->allocation_index, &u);

      HeliosNativeRenderPatch p;
      memset(&p, 0, sizeof(p));
      p.payload_offset = in->payload_offset;
      p.allocation_list_index = in->allocation_index;
      p.operand_kind = in->operand_kind;
      p.encoded_width = helios_hnr2_operand_width(in->operand_kind);
      memcpy(patch_base + (size_t)slot * HELIOS_HNR2_PATCH_RECORD_SIZE, &p,
             sizeof(p));
   }
}

enum helios_hnr2_encode_status
helios_hnr2_encode_fragment(const struct helios_hnr2_batch *b,
                            const struct helios_hnr2_plan *plan,
                            uint16_t fragment_index, uint64_t batch_token,
                            void *command_buffer, uint32_t command_buffer_bytes,
                            uint32_t *out_command_length)
{
   if (!b || !plan || !command_buffer || !out_command_length)
      return HELIOS_HNR2_ENCODE_NO_PAYLOAD;
   *out_command_length = 0;
   if (batch_token == 0)
      return HELIOS_HNR2_ENCODE_BAD_TOKEN;
   if (plan->fragment_count == 0 || fragment_index >= plan->fragment_count)
      return HELIOS_HNR2_ENCODE_BAD_FRAGMENT_INDEX;

   const bool begin = fragment_index == 0;
   const bool commit = fragment_index + 1 == plan->fragment_count;
   const bool has_reply = commit && b->has_reply;
   const uint32_t payload_bytes = plan->fragment_payload_bytes[fragment_index];
   const uint32_t metadata =
      commit ? plan->commit_metadata_bytes : HELIOS_HNR2_HEADER_SIZE;
   const uint64_t command_bytes = (uint64_t)metadata + payload_bytes;

   if (command_bytes > command_buffer_bytes)
      return HELIOS_HNR2_ENCODE_BUFFER_TOO_SMALL;

   uint8_t *out = (uint8_t *)command_buffer;
   memset(out, 0, (size_t)command_bytes);

   const uint32_t use_bytes =
      commit ? b->allocation_count * HELIOS_HNR2_USE_RECORD_SIZE : 0;
   const uint32_t patch_bytes =
      commit ? b->patch_count * HELIOS_HNR2_PATCH_RECORD_SIZE : 0;

   if (commit && b->allocation_count)
      helios_hnr2_write_tables(b, out + HELIOS_HNR2_HEADER_SIZE,
                               out + HELIOS_HNR2_HEADER_SIZE + use_bytes);

   const uint8_t *payload_src = (const uint8_t *)b->payload +
                                plan->fragment_payload_offset[fragment_index];
   memcpy(out + metadata, payload_src, payload_bytes);

   HeliosNativeRenderV2 h;
   memset(&h, 0, sizeof(h));
   h.magic = HELIOS_HNR2_MAGIC;
   h.abi_version = HELIOS_HNR2_ABI_VERSION;
   h.header_size = HELIOS_HNR2_HEADER_SIZE;
   h.package_generation = HELIOS_PACKAGE_GENERATION;
   h.batch_token = batch_token;
   h.total_payload_bytes = b->payload_bytes;
   h.fragment_payload_offset = plan->fragment_payload_offset[fragment_index];
   h.fragment_payload_bytes = payload_bytes;
   h.fragment_index = fragment_index;
   h.fragment_count = plan->fragment_count;
   /* An empty table is offset ZERO, never a degenerate pointer into the middle
    * of the buffer -- the same spelling the "before COMMIT" rule uses. */
   h.use_record_offset = use_bytes ? HELIOS_HNR2_HEADER_SIZE : 0;
   h.use_record_count = commit ? b->allocation_count : 0;
   h.patch_record_offset =
      patch_bytes ? HELIOS_HNR2_HEADER_SIZE + use_bytes : 0;
   h.patch_record_count = commit ? b->patch_count : 0;
   h.reply_allocation_list_index =
      has_reply ? b->reply_allocation_index
                : HELIOS_HNR2_NO_REPLY_ALLOCATION_INDEX;
   h.flags = (begin ? HELIOS_HNR2_FLAG_BEGIN : 0u) |
             (commit ? HELIOS_HNR2_FLAG_COMMIT : 0u) |
             (has_reply ? HELIOS_HNR2_FLAG_HAS_REPLY : 0u);
   h.reply_offset = has_reply ? b->reply_offset : 0;
   h.reply_capacity_bytes = has_reply ? b->reply_capacity_bytes : 0;
   h.reply_slot_generation = has_reply ? b->reply_slot_generation : 0;
   /* ⛔ The ABI's chosen domains, stated identically in helios_native_render.h:
    * fragment_crc64 is THIS FRAGMENT'S payload slice, full_payload_crc64 the
    * whole reassembled payload. Both are corruption diagnostics; no validator
    * on either side reads them. */
   h.fragment_crc64 = helios_crc64(payload_src, payload_bytes);
   h.full_payload_crc64 = commit ? plan->full_payload_crc64 : 0;
   memcpy(out, &h, sizeof(h));

   *out_command_length = (uint32_t)command_bytes;
   return HELIOS_HNR2_ENCODE_OK;
}

/* ══ the Windows half: the HVC1 context ════════════════════════════════════ */

#ifdef _WIN32

#include <stdio.h>
#include <stdlib.h>

#include "vn_helios_hwa2.h" /* vn_renderer_helios_diag_log */

/* The manifest carries D3DKMT handles as uint32_t so the encoder above can be
 * compiled and run off Windows. */
_Static_assert(sizeof(D3DKMT_HANDLE) == sizeof(uint32_t),
               "the HNR2 manifest stores D3DKMT_HANDLE as uint32_t");

#define HELIOS_IGNORE_STATUS(call)                                             \
   do {                                                                        \
      NTSTATUS ignored_ = (call);                                              \
      (void)ignored_;                                                          \
   } while (0)

/* One outstanding finite C51 wait. §10.7 requires the armed event object to be
 * RETAINED after a timeout "until signal or device loss", so an arm is never
 * cancelled -- only recycled once its own value is known reached and nobody is
 * inside its wait. Manual-reset because "value V was reached" is monotone: it
 * must stay true and must wake every waiter, not one. */
struct helios_native_wait_arm {
   HANDLE event;
   uint64_t value;
   uint32_t waiters;
   bool armed;
};

#define HELIOS_NATIVE_MAX_WAIT_ARMS 32

struct helios_native_context {
   D3DKMT_HANDLE device;
   D3DKMT_HANDLE context;
   enum helios_native_context_kind kind;

   /* Re-adopted after EVERY Render (§10.7). */
   void *command_buffer;
   uint32_t command_buffer_bytes;
   D3DDDI_ALLOCATIONLIST *allocation_list;
   uint32_t allocation_list_entries;
   D3DDDI_PATCHLOCATIONLIST *patch_list;
   uint32_t patch_list_entries;

   D3DKMT_HANDLE fence;
   uint64_t enqueued;  /* highest C51 value signalled on this context */
   uint64_t completed; /* highest value proven reached */
   uint64_t next_batch_token;
   bool renders_since_signal;
   bool lost;

   struct helios_native_wait_arm arms[HELIOS_NATIVE_MAX_WAIT_ARMS];

   CRITICAL_SECTION lock;
};

/* ── refusal census ─────────────────────────────────────────────────────────*/

static const char *const helios_native_site_names[HELIOS_NATIVE_REFUSE_SITE_COUNT] = {
   [HELIOS_NATIVE_REFUSE_CONTEXT] = "context",
   [HELIOS_NATIVE_REFUSE_CONTEXT_MINIMA] = "context_minima",
   [HELIOS_NATIVE_REFUSE_FENCE] = "fence",
   [HELIOS_NATIVE_REFUSE_ENCODE] = "encode",
   [HELIOS_NATIVE_REFUSE_LIST_CAPACITY] = "list_capacity",
   [HELIOS_NATIVE_REFUSE_RESIZE] = "resize",
   [HELIOS_NATIVE_REFUSE_RENDER] = "render",
   [HELIOS_NATIVE_REFUSE_BUFFER_ADOPT] = "buffer_adopt",
   [HELIOS_NATIVE_REFUSE_SIGNAL] = "signal",
   [HELIOS_NATIVE_REFUSE_WAIT] = "wait",
   [HELIOS_NATIVE_REFUSE_WAIT_ARMS] = "wait_arms",
   [HELIOS_NATIVE_REFUSE_NATIVE_FENCE_WAIT] = "native_fence_wait",
   [HELIOS_NATIVE_REFUSE_NATIVE_FENCE_SIGNAL] = "native_fence_signal",
   [HELIOS_NATIVE_REFUSE_CONTEXT_LOST] = "context_lost",
};

static volatile LONG helios_native_counters[HELIOS_NATIVE_REFUSE_SITE_COUNT];

static void
helios_native_refuse(enum helios_native_refusal_site site, const char *detail,
                     unsigned status)
{
   if ((unsigned)site >= HELIOS_NATIVE_REFUSE_SITE_COUNT)
      return;
   const long n = InterlockedIncrement(&helios_native_counters[site]);
   /* 1-then-every-256: a caller that retries per frame must not make the diag
    * log itself the failure. */
   if (n == 1 || (n % 256) == 0)
      vn_renderer_helios_diag_log(
         "HNR2 context REFUSED at %s count=%ld status=0x%08x: %s",
         helios_native_site_names[site], n, status, detail ? detail : "");
}

long
helios_native_refusals(enum helios_native_refusal_site site)
{
   if ((unsigned)site >= HELIOS_NATIVE_REFUSE_SITE_COUNT)
      return 0;
   return helios_native_counters[site];
}

void
helios_native_refusal_summary(char *out, size_t out_bytes)
{
   size_t used = 0;
   if (!out || out_bytes == 0)
      return;
   out[0] = '\0';
   for (unsigned s = 0; s < HELIOS_NATIVE_REFUSE_SITE_COUNT; s++) {
      const long n = helios_native_counters[s];
      if (!n)
         continue;
      const int w = snprintf(out + used, out_bytes - used, "%s%s=%ld",
                             used ? " " : "", helios_native_site_names[s], n);
      if (w <= 0 || (size_t)w >= out_bytes - used)
         return;
      used += (size_t)w;
   }
}

/* ── context state ──────────────────────────────────────────────────────────*/

/* Every failure on this context is terminal: §10.7 says a failed Render
 * "discards the incomplete assembler, loses the queue/context, and ignores
 * returned buffers rather than resubmitting through another transport". */
static VkResult
helios_native_lose(struct helios_native_context *c,
                   enum helios_native_refusal_site site, const char *detail,
                   unsigned status)
{
   c->lost = true;
   helios_native_refuse(site, detail, status);
   return VK_ERROR_DEVICE_LOST;
}

static bool
helios_native_adopt(struct helios_native_context *c, void *command_buffer,
                    uint32_t command_buffer_bytes,
                    D3DDDI_ALLOCATIONLIST *allocation_list,
                    uint32_t allocation_list_entries,
                    D3DDDI_PATCHLOCATIONLIST *patch_list,
                    uint32_t patch_list_entries)
{
   /* The advertised DMA buffer is a FLOOR, and the fragment plan was computed
    * on it: a smaller buffer would silently invalidate a plan already committed
    * to in fragment 0's `fragment_count`. */
   if (!command_buffer || command_buffer_bytes < HELIOS_HVC1_DMA_BUFFER_BYTES ||
       !allocation_list || !patch_list)
      return false;
   c->command_buffer = command_buffer;
   c->command_buffer_bytes = command_buffer_bytes;
   c->allocation_list = allocation_list;
   c->allocation_list_entries = allocation_list_entries;
   c->patch_list = patch_list;
   c->patch_list_entries = patch_list_entries;
   return true;
}

VkResult
helios_native_context_create(D3DKMT_HANDLE device,
                             enum helios_native_context_kind kind,
                             uint32_t queue_family, uint32_t queue_index,
                             struct helios_native_context **out)
{
   if (!out)
      return VK_ERROR_INITIALIZATION_FAILED;
   *out = NULL;
   if (!device)
      return VK_ERROR_INITIALIZATION_FAILED;

   /* Exactly one of the two ordinals carrying the control sentinel is
    * `Hvc1Reject::QueueOrdinalMixed` -- never a guess in either direction. */
   if (kind == HELIOS_NATIVE_CONTEXT_CONTROL) {
      queue_family = HELIOS_HVC1_CONTROL_ORDINAL;
      queue_index = HELIOS_HVC1_CONTROL_ORDINAL;
   } else if (queue_family == HELIOS_HVC1_CONTROL_ORDINAL ||
              queue_index == HELIOS_HVC1_CONTROL_ORDINAL) {
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT,
                           "queue context carries the control ordinal", 0);
      return VK_ERROR_INITIALIZATION_FAILED;
   }

   struct helios_native_context *c = calloc(1, sizeof(*c));
   if (!c)
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   InitializeCriticalSection(&c->lock);
   c->device = device;
   c->kind = kind;

   HeliosVulkanContextV1 hvc1;
   memset(&hvc1, 0, sizeof(hvc1));
   hvc1.magic = HELIOS_HVC1_MAGIC;
   hvc1.abi_version = HELIOS_HVC1_ABI_VERSION;
   hvc1.struct_size = HELIOS_HVC1_SIZE;
   hvc1.package_generation = HELIOS_PACKAGE_GENERATION;
   hvc1.capset = HELIOS_NATIVE_RENDER_CAPSET;
   hvc1.mode = HELIOS_HVC1_MODE_FINITE_HNR2_RENDER;
   hvc1.queue_family = queue_family;
   hvc1.queue_index = queue_index;

   D3DKMT_CREATECONTEXT cc;
   memset(&cc, 0, sizeof(cc));
   cc.hDevice = device;
   cc.NodeOrdinal = HELIOS_HVC1_NODE_ORDINAL;
   cc.EngineAffinity = HELIOS_HVC1_ENGINE_AFFINITY;
   cc.Flags.Value = HELIOS_HVC1_CREATE_CONTEXT_FLAGS;
   cc.ClientHint = D3DKMT_CLIENTHINT_VULKAN;
   cc.pPrivateDriverData = &hvc1;
   cc.PrivateDriverDataSize = sizeof(hvc1);
   NTSTATUS st = D3DKMTCreateContext(&cc);
   if (st != 0) {
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT, "CreateContext",
                           (unsigned)st);
      goto fail;
   }
   c->context = cc.hContext;

   if (!helios_native_adopt(c, cc.pCommandBuffer, cc.CommandBufferSize,
                            cc.pAllocationList, cc.AllocationListSize,
                            cc.pPatchLocationList, cc.PatchLocationListSize) ||
       cc.AllocationListSize < HELIOS_HVC1_ALLOCATION_LIST_ENTRIES ||
       cc.PatchLocationListSize < HELIOS_HVC1_PATCH_LOCATION_ENTRIES) {
      char why[192];
      snprintf(why, sizeof(why),
               "cmd=%p/%u alloc=%p/%u patch=%p/%u below the advertised minima",
               cc.pCommandBuffer, cc.CommandBufferSize,
               (void *)cc.pAllocationList, cc.AllocationListSize,
               (void *)cc.pPatchLocationList, cc.PatchLocationListSize);
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT_MINIMA, why, 0);
      goto fail;
   }

   /* The one unshared monitored progress fence (§10.7). Every flag is stated:
    * TopOfPipeline=0 is what makes the value mean "the promised completion
    * class happened", not "the buffer was copied". */
   D3DKMT_CREATESYNCHRONIZATIONOBJECT2 cs;
   memset(&cs, 0, sizeof(cs));
   cs.hDevice = device;
   cs.Info.Type = D3DDDI_MONITORED_FENCE;
   cs.Info.Flags.NoSignalMaxValueOnTdr = 1;
   cs.Info.Flags.NoGPUAccess = 1;
   cs.Info.MonitoredFence.InitialFenceValue = 0;
   /* ⛔ A BIT MASK over the one exposed node -- NOT HELIOS_HVC1_ENGINE_AFFINITY,
    * which is the zero-based ordinal CreateContext takes. §10.7 states both and
    * they are different structures; never propagate one to the other site. */
   cs.Info.MonitoredFence.EngineAffinity = HELIOS_HVC1_FENCE_ENGINE_AFFINITY;
   st = D3DKMTCreateSynchronizationObject2(&cs);
   if (st != 0) {
      helios_native_refuse(HELIOS_NATIVE_REFUSE_FENCE,
                           "CreateSynchronizationObject2(monitored)",
                           (unsigned)st);
      goto fail;
   }
   c->fence = cs.hSyncObject;

   *out = c;
   return VK_SUCCESS;

fail:
   helios_native_context_destroy(c);
   return VK_ERROR_INITIALIZATION_FAILED;
}

static VkResult helios_native_wait_locked_release(struct helios_native_context *c,
                                                  uint64_t value,
                                                  uint64_t timeout_ns);

void
helios_native_context_destroy(struct helios_native_context *c)
{
   if (!c)
      return;

   /* §10.7: orderly destruction joins this context's own latest milestone
    * first. A lost context has nothing to join and must not block. */
   EnterCriticalSection(&c->lock);
   const bool join = !c->lost && c->fence && c->enqueued > c->completed;
   const uint64_t value = c->enqueued;
   if (join)
      (void)helios_native_wait_locked_release(c, value,
                                              HELIOS_NATIVE_WAIT_INFINITE);
   else
      LeaveCriticalSection(&c->lock);

   /* A lost context can still own an accepted Render or imported-fence
    * operation even though no trustworthy C51 value remains to join.  Cancel
    * and drain that exact context before revoking the progress object any of
    * its already-enqueued signals can still reference. */
   if (c->context) {
      D3DKMT_DESTROYCONTEXT dc = { .hContext = c->context };
      HELIOS_IGNORE_STATUS(D3DKMTDestroyContext(&dc));
      c->context = 0;
   }
   if (c->fence) {
      D3DKMT_DESTROYSYNCHRONIZATIONOBJECT ds = { .hSyncObject = c->fence };
      HELIOS_IGNORE_STATUS(D3DKMTDestroySynchronizationObject(&ds));
      c->fence = 0;
   }
   for (unsigned i = 0; i < HELIOS_NATIVE_MAX_WAIT_ARMS; i++) {
      if (c->arms[i].event)
         CloseHandle(c->arms[i].event);
   }
   DeleteCriticalSection(&c->lock);
   free(c);
}

D3DKMT_HANDLE
helios_native_context_handle(const struct helios_native_context *c)
{
   return c ? c->context : 0;
}

/* ── Render ─────────────────────────────────────────────────────────────────*/

/*
 * §10.7's resize transition: a separate Render whose only nonzero flags are the
 * two resize bits, carrying no application work. All three returned pointers and
 * sizes are adopted EVEN ON FAILURE, and no over-capacity batch is recorded
 * until the requested capacity is actually present.
 *
 * ⚠ Cross-lane request 3 (lane-mesa.md): `DxgkDdiRender` must accept this
 * zero-length Render as an explicit counted no-op. Until K6 does, this path
 * fails loudly here rather than truncating a manifest.
 */
static bool
helios_native_resize_lists(struct helios_native_context *c,
                           uint32_t allocation_entries, uint32_t patch_entries)
{
   D3DKMT_RENDER render;
   memset(&render, 0, sizeof(render));
   render.hContext = c->context;
   render.CommandOffset = 0;
   render.CommandLength = 0;
   render.AllocationCount = 0;
   render.PatchLocationCount = 0;
   render.NewCommandBufferSize = HELIOS_HVC1_DMA_BUFFER_BYTES;
   render.NewAllocationListSize = allocation_entries;
   render.NewPatchLocationListSize = patch_entries;
   render.Flags.ResizeAllocationList = 1;
   render.Flags.ResizePatchLocationList = 1;

   const NTSTATUS st = D3DKMTRender(&render);
   const bool adopted = helios_native_adopt(
      c, render.pNewCommandBuffer, render.NewCommandBufferSize,
      render.pNewAllocationList, render.NewAllocationListSize,
      render.pNewPatchLocationList, render.NewPatchLocationListSize);
   if (st != 0) {
      helios_native_refuse(HELIOS_NATIVE_REFUSE_RESIZE, "D3DKMTRender(resize)",
                           (unsigned)st);
      return false;
   }
   if (!adopted) {
      helios_native_refuse(HELIOS_NATIVE_REFUSE_BUFFER_ADOPT,
                           "resize returned buffers below the minima", 0);
      return false;
   }
   return true;
}

/* Enqueue the next progress value behind everything already submitted. Caller
 * holds the lock. */
static VkResult
helios_native_signal_locked(struct helios_native_context *c, uint64_t *out_value)
{
   if (c->enqueued == UINT64_MAX)
      return helios_native_lose(c, HELIOS_NATIVE_REFUSE_SIGNAL,
                                "progress value exhausted", 0);
   const uint64_t target = c->enqueued + 1;
   D3DKMT_SIGNALSYNCHRONIZATIONOBJECTFROMGPU sig;
   memset(&sig, 0, sizeof(sig));
   sig.hContext = c->context;
   sig.ObjectCount = 1;
   sig.ObjectHandleArray = &c->fence;
   sig.MonitoredFenceValueArray = &target;
   const NTSTATUS st = D3DKMTSignalSynchronizationObjectFromGpu(&sig);
   if (st != 0)
      return helios_native_lose(c, HELIOS_NATIVE_REFUSE_SIGNAL,
                                "SignalSynchronizationObjectFromGpu",
                                (unsigned)st);
   c->enqueued = target;
   c->renders_since_signal = false;
   if (out_value)
      *out_value = target;
   return VK_SUCCESS;
}

static VkResult
helios_native_enqueue_fence_points_locked(
   struct helios_native_context *c,
   const struct helios_native_fence_point *points,
   uint32_t count,
   bool signal)
{
   for (uint32_t i = 0; i < count; i++) {
      if (!points[i].handle)
         return helios_native_lose(
            c, signal ? HELIOS_NATIVE_REFUSE_NATIVE_FENCE_SIGNAL
                      : HELIOS_NATIVE_REFUSE_NATIVE_FENCE_WAIT,
            "zero imported native-fence handle", 0);

      D3DKMT_HANDLE handle = points[i].handle;
      uint64_t value = points[i].value;
      NTSTATUS st;
      if (signal) {
         D3DKMT_SIGNALSYNCHRONIZATIONOBJECTFROMGPU op;
         memset(&op, 0, sizeof(op));
         op.hContext = c->context;
         op.ObjectCount = 1;
         op.ObjectHandleArray = &handle;
         op.MonitoredFenceValueArray = &value;
         st = D3DKMTSignalSynchronizationObjectFromGpu(&op);
      } else {
         D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMGPU op;
         memset(&op, 0, sizeof(op));
         op.hContext = c->context;
         op.ObjectCount = 1;
         op.ObjectHandleArray = &handle;
         op.MonitoredFenceValueArray = &value;
         st = D3DKMTWaitForSynchronizationObjectFromGpu(&op);
      }
      if (st != 0)
         return helios_native_lose(
            c, signal ? HELIOS_NATIVE_REFUSE_NATIVE_FENCE_SIGNAL
                      : HELIOS_NATIVE_REFUSE_NATIVE_FENCE_WAIT,
            signal ? "SignalSynchronizationObjectFromGpu(imported)"
                   : "WaitForSynchronizationObjectFromGpu(imported)",
            (unsigned)st);
   }
   return VK_SUCCESS;
}

VkResult
helios_native_context_submit_ordered(
   struct helios_native_context *c,
   const struct helios_native_fence_point *waits,
   uint32_t wait_count,
   const struct helios_hnr2_batch *batch,
   const struct helios_native_fence_point *signals,
   uint32_t signal_count,
   bool signal_progress,
   uint64_t *out_batch_token,
   uint64_t *out_progress_value)
{
   if (!c || !batch || (wait_count && !waits) || (signal_count && !signals))
      return VK_ERROR_INITIALIZATION_FAILED;
   if (out_batch_token)
      *out_batch_token = 0;
   if (out_progress_value)
      *out_progress_value = 0;

   /* Planning is pure and includes a CRC over the whole payload: do it before
    * taking the context lock. */
   struct helios_hnr2_plan plan;
   const enum helios_hnr2_encode_status planned =
      helios_hnr2_plan_batch(batch, HELIOS_HVC1_DMA_BUFFER_BYTES, &plan);
   if (planned != HELIOS_HNR2_ENCODE_OK) {
      helios_native_refuse(HELIOS_NATIVE_REFUSE_ENCODE,
                           helios_hnr2_encode_status_name(planned), 0);
      /* A batch above the bound returns the Vulkan operation's permitted
       * host-memory failure (§10.7), not a device loss. */
      return VK_ERROR_OUT_OF_HOST_MEMORY;
   }

   EnterCriticalSection(&c->lock);
   if (c->lost) {
      LeaveCriticalSection(&c->lock);
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT_LOST, "submit", 0);
      return VK_ERROR_DEVICE_LOST;
   }

   /* KMD owns exactly 64 immutable executor slots per context.  Every queue
    * call requests a C51 milestone, so waiting the oldest unretired milestone
    * before admitting the 65th is exact bounded backpressure, not polling and
    * not a synthetic completion.  The wait helper releases the lock. */
   while (signal_progress &&
          c->enqueued - c->completed >=
             HELIOS_HNR2_MAX_OUTSTANDING_SUBMISSIONS) {
      const uint64_t retire =
         c->enqueued - HELIOS_HNR2_MAX_OUTSTANDING_SUBMISSIONS + 1;
      const VkResult waited = helios_native_wait_locked_release(
         c, retire, HELIOS_NATIVE_WAIT_INFINITE);
      if (waited != VK_SUCCESS)
         return waited;
      EnterCriticalSection(&c->lock);
      if (c->lost) {
         LeaveCriticalSection(&c->lock);
         return VK_ERROR_DEVICE_LOST;
      }
   }

   /* The KMD emits one output patch location per use record, so the COMMIT
    * needs both lists at least `allocation_count` long. */
   if (batch->allocation_count > c->allocation_list_entries ||
       batch->allocation_count > c->patch_list_entries) {
      if (!helios_native_resize_lists(c, HELIOS_HVC1_ALLOCATION_LIST_ENTRIES,
                                      HELIOS_HVC1_PATCH_LOCATION_ENTRIES) ||
          batch->allocation_count > c->allocation_list_entries ||
          batch->allocation_count > c->patch_list_entries) {
         char why[128];
         snprintf(why, sizeof(why), "need %u entries, have alloc=%u patch=%u",
                  batch->allocation_count, c->allocation_list_entries,
                  c->patch_list_entries);
         helios_native_refuse(HELIOS_NATIVE_REFUSE_LIST_CAPACITY, why, 0);
         LeaveCriticalSection(&c->lock);
         return VK_ERROR_OUT_OF_HOST_MEMORY;
      }
   }

   if (c->next_batch_token == UINT64_MAX) {
      const VkResult r = helios_native_lose(
         c, HELIOS_NATIVE_REFUSE_ENCODE, "batch token exhausted", 0);
      LeaveCriticalSection(&c->lock);
      return r;
   }

   /* Nothing above this point submitted queue work.  Imported waits come
    * only after every pure bound check and the optional list resize, so a
    * preflight refusal cannot leave an otherwise-live context blocked on a
    * dependency for a batch the caller was told did not submit. */
   VkResult result = helios_native_enqueue_fence_points_locked(
      c, waits, wait_count, false);
   if (result != VK_SUCCESS) {
      LeaveCriticalSection(&c->lock);
      return result;
   }

   const uint64_t token = c->next_batch_token + 1;

   for (uint16_t f = 0; f < plan.fragment_count; f++) {
      uint32_t command_length = 0;
      const enum helios_hnr2_encode_status encoded = helios_hnr2_encode_fragment(
         batch, &plan, f, token, c->command_buffer, c->command_buffer_bytes,
         &command_length);
      if (encoded != HELIOS_HNR2_ENCODE_OK) {
         const VkResult r =
            helios_native_lose(c, HELIOS_NATIVE_REFUSE_ENCODE,
                               helios_hnr2_encode_status_name(encoded), f);
         LeaveCriticalSection(&c->lock);
         return r;
      }

      const uint32_t allocation_count =
         helios_hnr2_fragment_allocation_count(batch, &plan, f);
      /* Re-checked against the CURRENTLY ADOPTED list, not the one the gate
       * above saw: `NewAllocationListSize` is "in: requested / out: actual"
       * (d3dkmthk.h), the fragment loop re-adopts after every Render, and §10.7
       * itself speaks of "a smaller list actually returned by Dxgkrnl". If that
       * ever happens mid-batch the manifest write below would run off the end of
       * a dxgkrnl-mapped buffer, so it is a context loss rather than a resize:
       * fragments 0..f-1 are already in the KMD's one open assembler. */
      if (allocation_count > c->allocation_list_entries ||
          allocation_count > c->patch_list_entries) {
         char why[128];
         snprintf(why, sizeof(why),
                  "list shrank mid-batch: COMMIT needs %u, adopted alloc=%u patch=%u",
                  allocation_count, c->allocation_list_entries,
                  c->patch_list_entries);
         const VkResult r =
            helios_native_lose(c, HELIOS_NATIVE_REFUSE_LIST_CAPACITY, why, 0);
         LeaveCriticalSection(&c->lock);
         return r;
      }
      for (uint32_t i = 0; i < allocation_count; i++) {
         c->allocation_list[i].Value = 0;
         c->allocation_list[i].hAllocation = batch->allocations[i].handle;
         c->allocation_list[i].WriteOperation =
            (batch->allocations[i].access & HELIOS_HNR2_ACCESS_WRITE) ? 1 : 0;
      }

      D3DKMT_RENDER render;
      memset(&render, 0, sizeof(render));
      render.hContext = c->context;
      render.CommandOffset = 0;
      render.CommandLength = command_length;
      render.AllocationCount = allocation_count;
      render.PatchLocationCount = 0;
      render.NewCommandBufferSize = HELIOS_HVC1_DMA_BUFFER_BYTES;
      render.NewAllocationListSize = HELIOS_HVC1_ALLOCATION_LIST_ENTRIES;
      render.NewPatchLocationListSize = HELIOS_HVC1_PATCH_LOCATION_ENTRIES;

      const NTSTATUS st = D3DKMTRender(&render);
      if (st != 0) {
         char why[128];
         snprintf(why, sizeof(why), "D3DKMTRender fragment %u/%u len=%u", f,
                  plan.fragment_count, command_length);
         const VkResult r = helios_native_lose(c, HELIOS_NATIVE_REFUSE_RENDER,
                                               why, (unsigned)st);
         LeaveCriticalSection(&c->lock);
         return r;
      }
      c->renders_since_signal = true;

      if (!helios_native_adopt(c, render.pNewCommandBuffer,
                               render.NewCommandBufferSize,
                               render.pNewAllocationList,
                               render.NewAllocationListSize,
                               render.pNewPatchLocationList,
                               render.NewPatchLocationListSize)) {
         const VkResult r = helios_native_lose(
            c, HELIOS_NATIVE_REFUSE_BUFFER_ADOPT,
            "Render returned buffers below the advertised minima", 0);
         LeaveCriticalSection(&c->lock);
         return r;
      }
   }

   /* Only now: a token is "accepted by this context" exactly when its whole
    * batch reached the KMD, so a failed batch cannot burn one and make the next
    * one look non-increasing. */
   c->next_batch_token = token;
   if (out_batch_token)
      *out_batch_token = token;

   result = helios_native_enqueue_fence_points_locked(
      c, signals, signal_count, true);
   if (result != VK_SUCCESS) {
      LeaveCriticalSection(&c->lock);
      return result;
   }

   if (signal_progress)
      result = helios_native_signal_locked(c, out_progress_value);

   LeaveCriticalSection(&c->lock);
   return result;
}

VkResult
helios_native_context_submit(struct helios_native_context *c,
                             const struct helios_hnr2_batch *batch,
                             bool signal_progress, uint64_t *out_batch_token,
                             uint64_t *out_progress_value)
{
   return helios_native_context_submit_ordered(
      c, NULL, 0, batch, NULL, 0, signal_progress, out_batch_token,
      out_progress_value);
}

VkResult
helios_native_context_signal(struct helios_native_context *c,
                             uint64_t *out_value)
{
   if (!c)
      return VK_ERROR_INITIALIZATION_FAILED;
   EnterCriticalSection(&c->lock);
   if (c->lost) {
      LeaveCriticalSection(&c->lock);
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT_LOST, "signal", 0);
      return VK_ERROR_DEVICE_LOST;
   }
   const VkResult r = helios_native_signal_locked(c, out_value);
   LeaveCriticalSection(&c->lock);
   return r;
}

/* ── C51 CPU waits ──────────────────────────────────────────────────────────*/

static uint32_t
helios_native_timeout_ms(uint64_t timeout_ns)
{
   /* Split rather than `+999999`: a near-UINT64_MAX Vulkan timeout would
    * overflow the round-up and come back as a near-zero wait. */
   const uint64_t ms =
      timeout_ns / 1000000u + ((timeout_ns % 1000000u) ? 1u : 0u);
   /* INFINITE is 0xFFFFFFFF: a finite Vulkan timeout must never round INTO it. */
   return ms >= INFINITE ? INFINITE - 1u : (uint32_t)ms;
}

/* Collect any arm whose event already fired. Caller holds the lock. */
static void
helios_native_reap_arms(struct helios_native_context *c)
{
   for (unsigned i = 0; i < HELIOS_NATIVE_MAX_WAIT_ARMS; i++) {
      struct helios_native_wait_arm *a = &c->arms[i];
      if (!a->armed || !a->event)
         continue;
      if (WaitForSingleObject(a->event, 0) != WAIT_OBJECT_0)
         continue;
      if (a->value > c->completed)
         c->completed = a->value;
      if (a->waiters == 0) {
         ResetEvent(a->event);
         a->armed = false;
      }
   }
}

/*
 * Wait for `value` with the context lock HELD on entry; the lock is always
 * released before returning, and is never held across a KMT wait or a
 * WaitForSingleObject (§13.3).
 */
static VkResult
helios_native_wait_locked_release(struct helios_native_context *c,
                                  uint64_t value, uint64_t timeout_ns)
{
   helios_native_reap_arms(c);
   if (c->completed >= value) {
      LeaveCriticalSection(&c->lock);
      return VK_SUCCESS;
   }
   if (value > c->enqueued) {
      /* Nothing will ever signal it; an infinite wait here would hang the
       * application thread forever. */
      const VkResult r =
         helios_native_lose(c, HELIOS_NATIVE_REFUSE_WAIT,
                            "wait for a value this context never enqueued", 0);
      LeaveCriticalSection(&c->lock);
      return r;
   }

   if (timeout_ns == HELIOS_NATIVE_WAIT_INFINITE) {
      const D3DKMT_HANDLE fence = c->fence;
      const D3DKMT_HANDLE device = c->device;
      LeaveCriticalSection(&c->lock);

      D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU wait;
      memset(&wait, 0, sizeof(wait));
      wait.hDevice = device;
      wait.ObjectCount = 1;
      wait.ObjectHandleArray = &fence;
      wait.FenceValueArray = &value;
      wait.hAsyncEvent = NULL; /* the call itself blocks */
      const NTSTATUS st = D3DKMTWaitForSynchronizationObjectFromCpu(&wait);

      EnterCriticalSection(&c->lock);
      if (st != 0) {
         const VkResult r =
            helios_native_lose(c, HELIOS_NATIVE_REFUSE_WAIT,
                               "WaitForSynchronizationObjectFromCpu",
                               (unsigned)st);
         LeaveCriticalSection(&c->lock);
         return r;
      }
      if (value > c->completed)
         c->completed = value;
      LeaveCriticalSection(&c->lock);
      return VK_SUCCESS;
   }

   /* Finite: an event-backed arm. Reuse the arm for this exact value if one is
    * already outstanding -- two waiters then share one arm, which is why the
    * event is manual-reset. */
   struct helios_native_wait_arm *arm = NULL;
   for (unsigned i = 0; i < HELIOS_NATIVE_MAX_WAIT_ARMS; i++) {
      if (c->arms[i].armed && c->arms[i].value == value) {
         arm = &c->arms[i];
         break;
      }
   }
   if (!arm) {
      for (unsigned i = 0; i < HELIOS_NATIVE_MAX_WAIT_ARMS; i++) {
         if (!c->arms[i].armed) {
            arm = &c->arms[i];
            break;
         }
      }
      if (!arm) {
         /* Every arm is still outstanding. There is no fallback: an unarmed
          * finite wait would either spin or lie. */
         helios_native_refuse(HELIOS_NATIVE_REFUSE_WAIT_ARMS,
                              "all finite-wait arms outstanding", 0);
         LeaveCriticalSection(&c->lock);
         return VK_ERROR_OUT_OF_HOST_MEMORY;
      }
      if (!arm->event) {
         arm->event = CreateEventW(NULL, TRUE /* manual reset */, FALSE, NULL);
         if (!arm->event) {
            helios_native_refuse(HELIOS_NATIVE_REFUSE_WAIT_ARMS, "CreateEvent",
                                 GetLastError());
            LeaveCriticalSection(&c->lock);
            return VK_ERROR_OUT_OF_HOST_MEMORY;
         }
      }

      D3DKMT_WAITFORSYNCHRONIZATIONOBJECTFROMCPU wait;
      memset(&wait, 0, sizeof(wait));
      wait.hDevice = c->device;
      wait.ObjectCount = 1;
      wait.ObjectHandleArray = &c->fence;
      wait.FenceValueArray = &value;
      wait.hAsyncEvent = arm->event;
      const NTSTATUS st = D3DKMTWaitForSynchronizationObjectFromCpu(&wait);
      if (st != 0) {
         const VkResult r = helios_native_lose(
            c, HELIOS_NATIVE_REFUSE_WAIT,
            "WaitForSynchronizationObjectFromCpu(event)", (unsigned)st);
         LeaveCriticalSection(&c->lock);
         return r;
      }
      arm->value = value;
      arm->armed = true;
   }

   arm->waiters++;
   HANDLE event = arm->event;
   LeaveCriticalSection(&c->lock);

   const DWORD w = WaitForSingleObject(event, helios_native_timeout_ms(timeout_ns));

   EnterCriticalSection(&c->lock);
   arm->waiters--;
   VkResult result;
   if (w == WAIT_OBJECT_0) {
      if (value > c->completed)
         c->completed = value;
      if (arm->waiters == 0) {
         ResetEvent(arm->event);
         arm->armed = false;
      }
      result = VK_SUCCESS;
   } else if (w == WAIT_TIMEOUT) {
      /* §10.7: the armed event object is retained after timeout until signal or
       * device loss -- the arm stays, so the next wait cannot miss it. */
      result = VK_TIMEOUT;
   } else {
      result = helios_native_lose(c, HELIOS_NATIVE_REFUSE_WAIT,
                                  "WaitForSingleObject", GetLastError());
   }
   LeaveCriticalSection(&c->lock);
   return result;
}

VkResult
helios_native_context_wait(struct helios_native_context *c, uint64_t value,
                           uint64_t timeout_ns)
{
   if (!c)
      return VK_ERROR_INITIALIZATION_FAILED;
   EnterCriticalSection(&c->lock);
   if (c->lost) {
      LeaveCriticalSection(&c->lock);
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT_LOST, "wait", 0);
      return VK_ERROR_DEVICE_LOST;
   }
   if (value == 0) {
      LeaveCriticalSection(&c->lock);
      return VK_SUCCESS;
   }
   return helios_native_wait_locked_release(c, value, timeout_ns);
}

VkResult
helios_native_queue_wait_idle(struct helios_native_context *c)
{
   if (!c)
      return VK_ERROR_INITIALIZATION_FAILED;
   EnterCriticalSection(&c->lock);
   if (c->lost) {
      LeaveCriticalSection(&c->lock);
      helios_native_refuse(HELIOS_NATIVE_REFUSE_CONTEXT_LOST, "queue_wait_idle",
                           0);
      return VK_ERROR_DEVICE_LOST;
   }
   /* §10.7 waits "the latest C51 value on that queue's context" -- so work
    * submitted fire-and-forget needs one enqueued before there IS a latest. */
   if (c->renders_since_signal) {
      const VkResult r = helios_native_signal_locked(c, NULL);
      if (r != VK_SUCCESS) {
         LeaveCriticalSection(&c->lock);
         return r;
      }
   }
   const uint64_t value = c->enqueued;
   if (value == 0) {
      LeaveCriticalSection(&c->lock);
      return VK_SUCCESS;
   }
   return helios_native_wait_locked_release(c, value,
                                            HELIOS_NATIVE_WAIT_INFINITE);
}

VkResult
helios_native_device_wait_idle(struct helios_native_context *const *contexts,
                               uint32_t count)
{
   if (count && !contexts)
      return VK_ERROR_INITIALIZATION_FAILED;
   for (uint32_t i = 0; i < count; i++) {
      struct helios_native_context *c = contexts[i];
      if (!c)
         return VK_ERROR_INITIALIZATION_FAILED;
      if (c->kind != HELIOS_NATIVE_CONTEXT_QUEUE) {
         /* ⛔ "A control C51 value is never substituted for this join." */
         helios_native_refuse(HELIOS_NATIVE_REFUSE_WAIT,
                              "device idle named the control context", 0);
         return VK_ERROR_DEVICE_LOST;
      }
      /* Vulkan already requires every VkQueue of the device to be externally
       * synchronised here, so snapshotting and joining one context at a time
       * cannot miss work that was legal to submit. */
      const VkResult r = helios_native_queue_wait_idle(c);
      if (r != VK_SUCCESS)
         return r;
   }
   return VK_SUCCESS;
}

#endif /* _WIN32 */
