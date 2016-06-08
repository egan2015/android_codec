#!/bin/sh
ANDROID_NDK_ROOT=$NDK_HOME
PREBUILT=$ANDROID_NDK_ROOT/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64
PLATFORM=$ANDROID_NDK_ROOT/platforms/android-23/arch-arm
ARM_INC=$PLATFORM/usr/include
ARM_LIB=$PLATFORM/usr/lib
ARM_LIBO=$PREBUILT/lib/gcc/arm-linux-androideabi/4.9

./configure --enable-pic --enable-strip --enable-static --cross-prefix=$PREBUILT/bin/arm-linux-androideabi- --sysroot=$PLATFORM --host=arm-linux --prefix=../build/x264 --extra-cflags="-march=armv7-a -mtune=cortex-a8 -mfloat-abi=softfp -mfpu=neon -D__ARM_ARCH_7__ -D__ARM_ARCH_7A__"

