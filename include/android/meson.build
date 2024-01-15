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
    include_directories(androir_root / 'frameworks/native/libs/ui/include'),
    include_directories(androir_root / 'system/core/libutils/include'),
    include_directories(androir_root / 'vendor/nxp-opensource/imx/display/display'),
    include_directories(androir_root / 'vendor/nxp-opensource/imx/include'),
    include_directories(androir_root / 'vendor/nxp/wsialloc'),
    include_directories(androir_root / 'vendor/nxp/wsialloc/android/src'),
    include_directories(androir_root / 'vendor/nxp/wsialloc/android/src/include'),
    include_directories(androir_root / 'system/libbase/include'),
    include_directories(androir_root / 'system/logging/liblog/include/log'),
    include_directories(androir_root / 'system/logging/liblog/include'),
])
endif
