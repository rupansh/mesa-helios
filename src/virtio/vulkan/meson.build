# Copyright 2019 Google LLC
# SPDX-License-Identifier: MIT
#
# based in part on anv and radv which are:
# Copyright © 2017 Intel Corporation

vn_entrypoints = custom_target(
  'vn_entrypoints',
  input : [vk_entrypoints_gen, vk_api_xml],
  output : ['vn_entrypoints.h', 'vn_entrypoints.c'],
  command : [
    prog_python, '@INPUT0@', '--xml', '@INPUT1@', '--proto', '--weak',
    '--out-h', '@OUTPUT0@', '--out-c', '@OUTPUT1@', '--prefix', 'vn',
    '--beta', with_vulkan_beta.to_string()
  ],
)

icd_file_name = libname_prefix + 'vulkan_virtio.' + libname_suffix

virtio_icd = custom_target(
  'virtio_icd',
  input : [vk_icd_gen, vk_api_xml],
  output : 'virtio_icd.' + vulkan_manifest_suffix,
  command : [
    prog_python, '@INPUT0@',
    '--api-version', '1.4', '--xml', '@INPUT1@',
    '--sizeof-pointer', sizeof_pointer,
    '--icd-lib-path', vulkan_icd_lib_path,
    '--icd-filename', icd_file_name,
    '--out', '@OUTPUT@',
  ],
  build_by_default : true,
  install_dir : with_vulkan_icd_dir,
  install_tag : 'runtime',
  install : true,
)

_dev_icdname = 'virtio_devenv_icd.@0@.json'.format(host_machine.cpu())
_dev_icd = custom_target(
  'virtio_devenv_icd',
  input : [vk_icd_gen, vk_api_xml],
  output : _dev_icdname,
  command : [
    prog_python, '@INPUT0@',
    '--api-version', '1.4', '--xml', '@INPUT1@',
    '--sizeof-pointer', sizeof_pointer,
    '--icd-lib-path', meson.current_build_dir(),
    '--icd-filename', icd_file_name,
    '--out', '@OUTPUT@',
  ],
  build_by_default : true,
)

devenv.append('VK_DRIVER_FILES', _dev_icd.full_path())

libvn_files = files(
  'vn_acceleration_structure.c',
  'vn_buffer.c',
  'vn_command_buffer.c',
  'vn_common.c',
  'vn_cs.c',
  'vn_descriptor_heap.c',
  'vn_descriptor_set.c',
  'vn_device.c',
  'vn_device_memory.c',
  'vn_feedback.c',
  'vn_host_copy.c',
  'vn_icd.c',
  'vn_image.c',
  'vn_instance.c',
  'vn_physical_device.c',
  'vn_pipeline.c',
  'vn_query_pool.c',
  'vn_queue.c',
  'vn_render_pass.c',
  'vn_ring.c',
  'vn_renderer_internal.c',
  'vn_renderer_util.c',
)

vn_incs = [
  inc_include,
  inc_src,
  inc_virtio,
]

vn_deps = [
  dep_thread,
  idep_mesautil,
  idep_vulkan_util,
  idep_vulkan_lite_runtime,
  idep_vulkan_wsi,
  idep_xmlconfig,
]

vn_c_args = [
  no_override_init_args,
  soong_compat_c_args,
] + cc.get_supported_arguments([
  '-Werror=switch',
])

vn_link_args = [
  ld_args_bsymbolic,
  ld_args_gc_sections,
  vulkan_icd_link_args,
]

vn_libs = []

if not with_platform_windows
  libvn_files += files('vn_renderer_vtest.c')
endif

if system_has_kms_drm
  libvn_files += files('vn_renderer_virtgpu.c')
  vn_deps += dep_libdrm
endif

if with_platform_windows
  # Escape-free per-instance HVM1 backend.  Only stock D3DKMT thunks are linked;
  # the selected backend has no IOCTL/blob/name/resource-id carrier.
  libvn_files += files('vn_renderer_helios_hvm.c')
  # HWA2 parse/validate. Separate TU because it is a transcription of
  # protocol/src/wddm.rs's validator and must be reviewable against it without
  # 5000 lines of transport in the way.
  libvn_files += files('vn_helios_hwa2.c')
  # A1: the HTS1 translation session (raw KMT device, HVC1 control context,
  # role-1 reply pool, finite INIT, C51 reply). Its own TU because A2 and A3
  # both build on it and it is single-owner per the lane brief.
  libvn_files += files('vn_helios_translation_session.c')
  # A2: the native KMT lane (HVC1 contexts, the HNR2 encoder/fragmenter, the
  # Render call shape, the per-context monitored fence). Its encoder half is
  # platform-independent on purpose -- tools/hnr2_encode_probe.c compiles and
  # RUNS it on Linux against protocol/'s own validator.
  libvn_files += files('vn_helios_native_kmt.c')
  # A4: exact per-instance mode ownership.  Record-only queue entry points
  # synchronously seal bounded Venus payloads; normal mode submits the same
  # generated queue opcode subset through the queue's nonzero HVC1 context.
  libvn_files += files('vn_helios_record_submit.c')
  vn_deps += cc.find_library('gdi32')
  # Current mingw-w64's gdi32 import library predates the WDDM 3.1 thunk even
  # though the stock target gdi32.dll exports it.  Generate a one-symbol import
  # library at configure time; this remains an ordinary static import (and is
  # intentionally not a LoadLibrary/GetProcAddress compatibility path).
  helios_dlltool = find_program('dlltool')
  helios_gdi32_native_fence_import = meson.current_build_dir() / 'libhelios_gdi32_native_fence.a'
  run_command(
    helios_dlltool,
    '-d', meson.current_source_dir() / 'helios_gdi32_native_fence.def',
    '-l', helios_gdi32_native_fence_import,
    '-D', 'GDI32.dll',
    check : true,
  )
  vn_link_args += helios_gdi32_native_fence_import
  # mingw-w64 ships no D3DKMT headers; use the real WDK ones (d3dkmthk.h +
  # d3dkmdt.h + d3dukmdt.h, the only WDK-only headers in the chain — they pull
  # only winapifamily.h, which mingw has) vendored under icd/win-build/wdk-include.
  # Real headers = authoritative ABI (they self-default DXGKDDI_INTERFACE_VERSION
  # to WDDM3_2, matching the OS), so no hand-shimmed D3DKMT structs to keep in sync.
  vn_c_args += '-I' + get_option('helios-wdk-include')

  # protocol/include/helios_wddm.h — the ONE C declaration of the Helios D3D-side
  # wire records (HWA2/HOB1/HOS1/HOC1), mirroring protocol/src/wddm.rs. The
  # standing directive is that shared private data has one declaration, in
  # protocol/, and before this edge existed the ICD hand-copied four of those
  # records into the retired renderer backend with nothing but `sizeof` asserts — so a
  # field reorder inside the right number of bytes passed every check on this
  # side. Including the real header brings its per-field `offsetof` assertions
  # with it, which is the compile-time cross-check K4-CONTRACT §7 requires of
  # this cross-repo pair.
  #
  # The path crosses the submodule boundary: helios_wddm.h lives in the PARENT
  # repository and icd/mesa is a submodule of it. This is the same shape as
  # helios-wdk-include above (default 'Z:/icd/win-build/wdk-include'), which is
  # also outside this subproject, so the mechanism is not new — but it is derived
  # rather than optioned, because adding a meson option means editing
  # icd/mesa/meson.options, which this lane does not own. The derivation is
  # `<mesa source root>/../../protocol/include`, and mesa is always checked out at
  # <helios>/icd/mesa; win_meson builds it straight from the Z:\ share, where the
  # parent tree is present, so the relative walk resolves on both platforms.
  #
  # ⚠ It is checked, not assumed: a wrong path would otherwise surface as a
  # confusing "helios_wddm.h: No such file" from a source file, or worse, as a
  # stale copy found elsewhere on the include path.
  helios_protocol_include = meson.project_source_root() / '..' / '..' / 'protocol' / 'include'
  helios_fs = import('fs')
  if not helios_fs.exists(helios_protocol_include / 'helios_wddm.h')
    error('Helios: protocol/include/helios_wddm.h not found at "' +
          helios_protocol_include + '". icd/mesa must be built from inside the ' +
          'helios-vgpu tree (it is a submodule at <helios>/icd/mesa); the HWA2 ' +
          'allocation descriptor has exactly one declaration and this ICD may ' +
          'not carry a local copy of it.')
  endif
  vn_c_args += '-I' + helios_protocol_include
endif

if with_platform_wayland or with_platform_x11 or \
   (system_has_kms_drm and not with_platform_android) or \
   with_platform_windows
  libvn_files += files('vn_wsi.c')
  vn_c_args += '-DVN_USE_WSI_PLATFORM'
endif

if with_platform_wayland
  vn_deps += dep_wayland_client
endif

if with_platform_x11
  vn_deps += dep_xcb_dri3
endif

if with_platform_android
  libvn_files += files('vn_android.c')
  vn_deps += [dep_android, idep_u_gralloc]
endif

libvulkan_virtio = shared_library(
  'vulkan_virtio',
  [libvn_files, vn_entrypoints, sha1_h],
  include_directories : vn_incs,
  link_with : vn_libs,
  dependencies : vn_deps,
  c_args : vn_c_args,
  link_args : vn_link_args,
  link_depends : vulkan_icd_link_depends,
  # Export the 3 vk_icd* loader entrypoints on Windows (gnu_symbol_visibility
  # 'hidden' otherwise hides vk_icdGetInstanceProcAddr). Ignored on ELF.
  vs_module_defs : vulkan_api_def,
  gnu_symbol_visibility : 'hidden',
  # On Windows libname_prefix='' so the manifest expects vulkan_virtio.dll; mingw
  # would otherwise produce libvulkan_virtio.dll. '' on Windows, default elsewhere.
  name_prefix : host_machine.system() == 'windows' ? '' : [],
  install : true,
)
