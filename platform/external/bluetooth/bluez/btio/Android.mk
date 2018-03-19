LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

#
# libbtio
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	btio.c

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\" \

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../gdbus \
	$(call include-path-for, glib) \

LOCAL_SHARED_LIBRARIES := \
	libcutils \
	libglib \
	libbluetooth \

LOCAL_MODULE:=libbtio

LOCAL_MODULE_TAGS:=optional

include $(BUILD_SHARED_LIBRARY)
