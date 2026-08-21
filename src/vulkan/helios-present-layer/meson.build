# Copyright © 2026 Helios vGPU
# SPDX-License-Identifier: MIT
#
# VK_LAYER_HELIOS_present — the Helios native-Vulkan WSI layer.
#
# ⛔ WHY THIS IS ITS OWN DIRECTORY AND NOT PART OF src/vulkan/wsi.
#
# The layer sits ABOVE the Helios ICD and must never be linked into it.
# docs/HELIOS_PRESENT_SYNC_RETIREMENT.md §2 item 8 makes that an acyclicity
# rule, not a preference: DXVK and vkd3d reach the ICD through the private
# direct-dispatch entry point and must never enter this layer. The layer is
# also allowed to import d3d12.dll/dxgi.dll, which §13.2 forbids the ICD DLL
# from doing — so a build arrangement that could put these objects into
# `idep_vulkan_wsi`, and from there into an ICD, would silently violate both.
# Keeping the sources in a directory whose only product is the layer's own
# shared_library makes that mistake hard rather than merely discouraged.

helios_present_layer_files = files(
  'helios_present_layer.cpp',
)

# Windows-only by construction: the sources include <windows.h>,
# <directx/d3d12.h> and the D3D12/DXGI COM interfaces. There is no non-Windows
# arm to fall back to, so the caller is expected to have gated on
# with_platform_windows before entering this subdir.
vklayer_helios_present = shared_library(
  'VkLayer_HELIOS_present',
  helios_present_layer_files,
  # mingw would otherwise emit libVkLayer_HELIOS_present.dll, which does not
  # match the `library_path` in the manifest and is not the Windows convention.
  # A mismatch here is silent: the loader simply never finds the layer.
  name_prefix : '',
  cpp_args : ['-D_CRT_SECURE_NO_WARNINGS'],
  gnu_symbol_visibility : 'hidden',
  # The loader surface is fixed.  Do not rely on MinGW auto-export, which can
  # expose implementation helpers and silently widen the layer ABI.
  vs_module_defs : 'helios_present_layer.def',
  dependencies : [
    idep_vulkan_util,
    idep_mesautil,
    dep_dxheaders,
    cpp.find_library('d3d12'),
    cpp.find_library('dxgi'),
    cpp.find_library('dxguid'),
  ],
  include_directories : [inc_include, inc_util, inc_src],
  install : true,
)

# The manifest the Vulkan loader reads. Two copies, matching every other Mesa
# layer: an installed one naming the library by bare filename (resolved from the
# loader's search path) and a build-tree one naming the absolute path, so
# `meson devenv` can load the layer without installing it.
configure_file(
  configuration : {'library_path' : 'VkLayer_HELIOS_present.dll'},
  input : 'VkLayer_HELIOS_present.json.in',
  output : '@BASENAME@',
  install : true,
  install_dir : join_paths(get_option('datadir'), 'vulkan', 'implicit_layer.d'),
  install_tag : 'runtime',
)

configure_file(
  configuration : {'library_path' : vklayer_helios_present.full_path()},
  input : 'VkLayer_HELIOS_present.json.in',
  output : '@PLAINNAME@_dev.json',
)

devenv.append('VK_ADD_IMPLICIT_LAYER_PATH', meson.current_build_dir())
