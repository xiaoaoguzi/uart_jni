

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := test.c
LOCAL_MODULE    := libserial
LOCAL_MODULE_TAGS := optional
LOCAL_PRELINK_MODULE := false
LOCAL_SHARED_LIBRARIES := libc

include $(BUILD_SHARED_LIBRARY)
