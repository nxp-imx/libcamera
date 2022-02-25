# SPDX-License-Identifier: CC0-1.0

androir_root = get_option('android_root')

if androir_root=='auto'
android_includes = ([
    include_directories('hardware/libhardware/include/'),
    include_directories('metadata/'),
    include_directories('system/core/include'),
])
else
android_includes = ([
    include_directories('hardware/libhardware/include/'),
    include_directories('metadata/'),
    include_directories('system/core/include'),
    include_directories(androir_root / 'external/libexif'),
    include_directories(androir_root / 'external/libjpeg-turbo'),
    include_directories(androir_root / 'external/libyuv/files/include'),
    include_directories(androir_root / 'external/libyaml/include'),
])
endif
