LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_SRC_FILES:= \
	termcap.c \
	tparam.c \
	version.c \

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../ \
	$(LOCAL_PATH) \

LOCAL_CFLAGS:= \
	-DANDROID_STUB \
	-DHAVE_FCNTL_H \
	-DSTDC_HEADERS \
	-DHAVE_STRING_H \
	-DHAVE_UNISTD_H \

LOCAL_MODULE:=libtermcap_static

include $(BUILD_STATIC_LIBRARY)
