# SPDX-License-Identifier: CC0-1.0

subdir('algorithms')
subdir('data')

ipa_name = 'ipa_nxp_neo'

neo_ipa_libs = [
    nxp_ipa_cam_helper_lib,
]

neo_ipa_sources = files([
    'ipa_context.cpp',
    'neo_ipa.cpp',
])

neo_ipa_sources += neo_ipa_algorithms

mod = shared_module(ipa_name,
                    [neo_ipa_sources, libcamera_generated_ipa_headers],
                    name_prefix : '',
                    include_directories : [ipa_includes, libipa_includes],
                    dependencies : libcamera_private,
                    link_with : libipa,
                    link_whole : neo_ipa_libs,
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@'],
                  install : false,
                  build_by_default : true)
endif

ipa_names += ipa_name
