LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

# HID plugin

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	device.c \
	fakehid.c \
	main.c \
	manager.c \
	server.c \
	hog_manager.c \
	hog_device.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\" \
	-DCONFIGDIR=\"/etc/bluetooth\"

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(LOCAL_PATH)/../gdbus \
	$(LOCAL_PATH)/../attrib \
	$(call include-path-for, glib) \
	$(call include-path-for, glib)/glib \
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
LOCAL_MODULE := input

include $(BUILD_SHARED_LIBRARY)
