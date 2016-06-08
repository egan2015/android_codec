#!/bin/sh
ANDROID_NDK_ROOT=$NDK_HOME
PREBUILT=$ANDROID_NDK_ROOT/toolchains/arm-linux-androideabi-4.9/prebuilt/darwin-x86_64
PLATFORM=$ANDROID_NDK_ROOT/platforms/android-23/arch-arm
ARM_INC=$PLATFORM/usr/include
ARM_LIB=$PLATFORM/usr/lib
ARM_LIBO=$PREBUILT/lib/gcc/arm-linux-androideabi/4.9
X264_BUILD=/Users/firebolt/workspaces/android-codec/jni/build/x264
FAAC_BUILD=/Users/firebolt/workspaces/android-codec/jni/build/faac

./configure --target-os=linux \
  --arch=arm  \
  --cpu=armv7-a \
  --enable-cross-compile \
  --cc=$PREBUILT/bin/arm-linux-androideabi-gcc \
  --cross-prefix=$PREBUILT/bin/arm-linux-androideabi- \
  --sysroot=$PLATFORM
  #--nm=$PREBUILT/bin/arm-linux-androideabi-nm \
  --extra-cflags="-fPIC -DANDROID -mfpu=neon -mfloat-abi=softfp -I$PLATFORM/usr/include -I$X264_BUILD/include -I$FAAC_BUILD/include" \
  --enable-asm \
  --disable-yasm \
  --enable-static \
  --disable-shared \
  --enable-small \
  --enable-gpl \
  --enable-version3 \
  --enable-nonfree \
  --enable-neon \
  --disable-endecoders \
  --enable-libx264 \
  --enable-libfdk-aac \
  --disable-ffmpeg \
  --disable-ffplay \
  --disable-ffserver \
  --disable-ffprobe \
  --prefix=../build/ffmpeg \
  --extra-ldflags="-Wl,-T,$PREBUILT/arm-linux-androideabi/lib/ldscripts/armelf_linux_eabi.x -Wl,-rpath-link=$PLATFORM/usr/lib -L$PLATFORM/usr/lib -nostdlib $PREBUILT/lib/gcc/arm-linux-androideabi/4.9/crtbegin.o $PREBUILT/lib/gcc/arm-linux-androideabi/4.9/crtend.o -lc -lm -ld -L$X264_BUILD/lib -L$FAAC_BUILD/lib"
  

#sed -i 's/HAVE_LRINT 0/HAVE_LRINT 1/g' config.h 
#sed -i 's/HAVE_LRINTF 0/HAVE_LRINTF 1/g' config.h
#sed -i 's/HAVE_ROUND 0/HAVE_ROUND 1/g' config.h
#sed -i 's/HAVE_ROUNDF 0/HAVE_ROUNDF 1/g' config.h
#sed -i 's/HAVE_TRUNC 0/HAVE_TRUNC 1/g' config.h
#sed -i 's/HAVE_TRUNCF 0/HAVE_TRUNCF 1/g' config.h
#sed -i 's/restrict restrict/restrict/g' config.h
#sed -i 's/HAVE_CBRT 0/HAVE_CBRT 1/g' config.h
#sed -i 's/HAVE_CBRTF 0/HAVE_CBRTF 1/g' config.h
#sed -i 's/HAVE_ISINF 0/HAVE_ISINF 1/g' config.h
#sed -i 's/HAVE_ISNAN 0/HAVE_ISNAN 1/g' config.h
#sed -i 's/HAVE_SINF 0/HAVE_SINF 1/g' config.h
#sed -i 's/HAVE_RINT 0/HAVE_RINT 1/g' config.h
