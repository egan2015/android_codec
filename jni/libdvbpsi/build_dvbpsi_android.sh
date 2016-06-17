#!/bin/sh
ANDROID_NDK_ROOT=$NDK_HOME
CROSS_PREFIX=$ANDROID_NDK_ROOT/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64/bin/arm-linux-androideabi-
PLATFORM=$ANDROID_NDK_ROOT/platforms/android-9/arch-arm
OPTIMIZE_CFLAGS="-mfloat-abi=softfp -mfpu=vfpv3-d16 -marm -march=armv7-a -mthumb -D__thumb__"

./configure --prefix=/Users/firebolt/workspaces/android-codec/jni/build/dvbpsi \
  --with-sysroot=$PLATFORM \
  --host=arm-linux \
  CC="${CROSS_PREFIX}gcc --sysroot=$PLATFORM" \
  CXX="${CROSS_PREFIX}g++ --sysroot=$PLATFORM" \
  RANLIB="${CROSS_PREFIX}ranlib" \
  AR="${CROSS_PREFIX}ar" \
  STRIP="${CROSS_PREFIX}strip" \
  NM="${CROSS_PREFIX}nm" \
  CFLAGS="-O3 $OPTIMIZE_CFLAGS --sysroot=$PLATFORM" \
  CXXFLAGS="-O3 $OPTIMIZE_CFLAGS --sysroot=$PLATFORM"
