/*
 * Copyright 2026 The Mesa 3D Graphics Library
 * SPDX-License-Identifier: MIT
 */

#ifndef ZINK_GLINTEROP_H
#define ZINK_GLINTEROP_H

#include <stdint.h>

/* Private data returned through mesa_glinterop_export_in::out_driver_data.
 * Keep this ABI free of pointers and Vulkan typedefs so out-of-tree consumers
 * can validate it before interpreting Vulkan values.
 */
#define ZINK_GLINTEROP_EXPORT_INFO_MAGIC 0x314c475aU /* "ZGL1" */
#define ZINK_GLINTEROP_EXPORT_INFO_VERSION 2

enum zink_glinterop_object_type {
   ZINK_GLINTEROP_OBJECT_IMAGE = 1,
   ZINK_GLINTEROP_OBJECT_BUFFER = 2,
};

struct zink_glinterop_export_info {
   uint32_t magic;
   uint32_t version;
   uint32_t struct_size;
   uint32_t object_type;

   uint32_t handle_type;
   uint32_t create_flags;
   uint32_t image_type;
   uint32_t format;
   uint32_t width;
   uint32_t height;
   uint32_t depth;
   uint32_t mip_levels;
   uint32_t array_layers;
   uint32_t samples;
   uint32_t tiling;
   uint32_t usage;
   uint32_t sharing_mode;
   uint32_t layout;
   uint32_t released_queue_family;
   uint32_t memory_type_index;

   uint64_t allocation_size;
   uint64_t memory_offset;
};

#endif
