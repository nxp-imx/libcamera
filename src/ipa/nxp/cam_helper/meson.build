# SPDX-License-Identifier: CC0-1.0

nxp_ipa_cam_helper_sources = files([
    'camera_helper.cpp',
    'camera_helper_mx95mbcam.cpp',
    'camera_helper_ov5640.cpp',
    'camera_helper_os08a20.cpp',
    'md_parser_ox.cpp',
])

# include path to md_parser.h
ipa_rpi_cam_helper = '..' / '..' / 'rpi' / 'cam_helper'

nxp_ipa_cam_helper_includes = [
    libipa_includes,
    ipa_rpi_cam_helper,
]

nxp_ipa_cam_helper_deps = [
    libcamera_private,
]

nxp_ipa_cam_helper_lib = static_library('nxp_ipa_cam_helper', nxp_ipa_cam_helper_sources,
                                         include_directories : nxp_ipa_cam_helper_includes,
                                         dependencies : nxp_ipa_cam_helper_deps)
