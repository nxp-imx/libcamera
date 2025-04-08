# SPDX-License-Identifier: CC0-1.0

subdir('algorithms')
subdir('data')

ipa_name = 'ipa_nxp_neo'

neo_ipa_sources = files([
    'ipa_context.cpp',
    'neo_ipa.cpp',
])

neo_ipa_sources += neo_ipa_algorithms

gen_version = meson.project_source_root() / 'utils' / 'gen-version.sh'

# Use vcs_tag() and not configure_file() or run_command(), to ensure that the
# version gets updated with every ninja build and not just at meson setup time.
ipa_version_h = vcs_tag(command : [gen_version, meson.project_build_root(), meson.project_source_root()],
                        input : 'neo_ipa_version.h.in',
                        output : 'neo_ipa_version.h',
                        fallback : meson.project_version())

neo_ipa_sources += ipa_version_h

neo_ipa_deps = [
    libcamera_private,
    libipa_dep,
    nxp_ipa_cam_helper_dep,
]

mod = shared_module(ipa_name, neo_ipa_sources,
                    name_prefix : '',
                    include_directories : [ipa_includes],
                    dependencies : [neo_ipa_deps],
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
