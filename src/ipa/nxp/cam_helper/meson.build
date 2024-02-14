# SPDX-License-Identifier: CC0-1.0

nxp_ipa_cam_helper_sources = files([
    'camera_helper.cpp',
    'camera_helper_ov5640.cpp',
    'camera_helper_mx95mbcam.cpp',
])

nxp_ipa_cam_helper_includes = [
    libipa_includes,
]

nxp_ipa_cam_helper_deps = [
    libcamera_private,
]

nxp_ipa_cam_helper_lib = static_library('nxp_ipa_cam_helper', nxp_ipa_cam_helper_sources,
                                         include_directories : nxp_ipa_cam_helper_includes,
                                         dependencies : nxp_ipa_cam_helper_deps)
