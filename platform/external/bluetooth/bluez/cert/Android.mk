LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

# Certification plugin

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	main.c \
	server.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\" \
	-DCONFIGDIR=\"/etc/bluetooth\"

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../attrib \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(LOCAL_PATH)/../gdbus \
	$(call include-path-for, glib) \
	$(call include-path-for, dbus)

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libbtio \
	libcutils \
	libdbus \
	libexpat \
	libglib

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/bluez-plugin
LOCAL_UNSTRIPPED_PATH := $(TARGET_OUT_SHARED_LIBRARIES_UNSTRIPPED)/bluez-plugin
LOCAL_MODULE := certification
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)
