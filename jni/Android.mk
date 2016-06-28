LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_CFLAGS := -std=c99
LOCAL_MODULE := jxcodec
LOCAL_SRC_FILES := codec.c \
				   sout.c \
				   tsmux/block.c \
				   tsmux/pes.c \
				   tsmux/csa.c \
				   tsmux/mtime.c \
				   tsmux/tsmux.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)/build/x264/include \
  $(LOCAL_PATH)/build/faac/include \
  $(LOCAL_PATH)/build/dvbpsi/include \
  $(LOCAL_PATH)/tsmux

LOCAL_LDLIBS := -llog -ldl \
    $(LOCAL_PATH)/build/x264/lib/libx264.a \
	$(LOCAL_PATH)/build/faac/lib/libfdk-aac.a \
	$(LOCAL_PATH)/build/dvbpsi/lib/libdvbpsi.a
LOCAL_STATIC_LIBRARIES := libx264 libfdk-aac libdvbpsi
include $(BUILD_SHARED_LIBRARY)


