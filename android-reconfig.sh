ANDROID_ROOT=/home/fanghui/share_home2/android-14-imx95
PRODUCT_PATH=$ANDROID_ROOT/out/target/product/evk_95
NDK_LIB_PATH=./src/android/ndk-lib

mkdir -p $NDK_LIB_PATH
cp -f $PRODUCT_PATH/system/lib64/libexif.so $NDK_LIB_PATH
cp -f $PRODUCT_PATH/system/lib64/libjpeg.so $NDK_LIB_PATH
cp -f $PRODUCT_PATH/system/lib64/libyuv.so $NDK_LIB_PATH
cp -f $PRODUCT_PATH/system/lib64/libyaml.so $NDK_LIB_PATH

~/.local/bin/meson build --cross-file android-meson.cross --sysconfdir /vendor/etc/configs -Dpipelines=imx8-isi -Dandroid=enabled -Dandroid_root=$ANDROID_ROOT -Dtest=true --reconfigure
