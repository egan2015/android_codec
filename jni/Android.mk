LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := jxcodec
LOCAL_SRC_FILES := codec.c
LOCAL_C_INCLUDES := $(LOCAL_PATH)/build/x264/include \
  $(LOCAL_PATH)/build/faac/include

LOCAL_LDLIBS := -llog -ldl $(LOCAL_PATH)/build/x264/lib/libx264.a \
	$(LOCAL_PATH)/build/faac/lib/libfdk-aac.a
LOCAL_STATIC_LIBRARIES := libx264
include $(BUILD_SHARED_LIBRARY)


