LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

# A2DP plugin

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	a2dp.c \
	avdtp.c \
	control.c \
	device.c \
	gateway.c \
	headset.c \
	ipc.c \
	main.c \
	manager.c \
	media.c \
	module-bluetooth-sink.c \
	sink.c \
	source.c \
	telephony-dummy.c \
	transport.c \
	unix.c \
	avrcp.c \
	avctp.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\" \
	-DCONFIGDIR=\"/etc/bluetooth\" \
	-DANDROID \
	-D__S_IFREG=0100000  # missing from bionic stat.h

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../gdbus \
	$(LOCAL_PATH)/../src \
	$(LOCAL_PATH)/../btio \
	$(call include-path-for, glib) \
	$(call include-path-for, dbus)

LOCAL_SHARED_LIBRARIES := \
	libbluetooth \
	libbluetoothd \
	libbtio \
	libdbus \
	libglib \
	liblog

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/bluez-plugin
LOCAL_UNSTRIPPED_PATH := $(TARGET_OUT_SHARED_LIBRARIES_UNSTRIPPED)/bluez-plugin
LOCAL_MODULE := audio

include $(BUILD_SHARED_LIBRARY)

#
# liba2dp
# This is linked to Audioflinger so **LGPL only**

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	android_audio_hw.c \
	liba2dp.c \
	ipc.c \
	../sbc/sbc_primitives.c \
	../sbc/sbc_primitives_neon.c

ifeq ($(TARGET_ARCH),x86)
LOCAL_SRC_FILES+= \
	../sbc/sbc_primitives_mmx.c \
	../sbc/sbc.c
else
LOCAL_SRC_FILES+= \
	../sbc/sbc.c.arm \
	../sbc/sbc_primitives_armv6.c
endif

# to improve SBC performance

ifeq ($(BLUETI_ENHANCEMENT), true)

LOCAL_CFLAGS:= -funroll-loops \
               -DCONFIGDIR=\"/etc/bluetooth\" \
  
LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../sbc \
        ../../../../frameworks/base/include \
	$(LOCAL_PATH)/../lib \
	$(call include-path-for, glib)

LOCAL_SHARED_LIBRARIES := \
	libcutils libbluetooth libglib

else

LOCAL_CFLAGS:= -funroll-loops
               
LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../sbc \
        ../../../../frameworks/base/include \
	$(LOCAL_PATH)/../lib \

LOCAL_SHARED_LIBRARIES := \
	libcutils libbluetooth
	
endif

ifneq ($(wildcard system/bluetooth/legacy.mk),)
LOCAL_STATIC_LIBRARIES := \
	libpower

LOCAL_MODULE := liba2dp
else
LOCAL_SHARED_LIBRARIES += \
	libpower

LOCAL_MODULE := audio.a2dp.default
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
endif

LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)
