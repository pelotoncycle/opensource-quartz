LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

#
# libplugin
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	hciops.c \
	mgmtops.c \
	dbusoob.c \
	adaptername.c \
	service.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-DBLUETOOTH_PLUGIN_BUILTIN \
	-DANDROID_EXPAND_NAME \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\"

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
        $(LOCAL_PATH)/../gdbus \
        $(LOCAL_PATH)/../src \
        $(call include-path-for, glib) \
        $(call include-path-for, dbus) \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libcutils \
	libdbus \
	libglib

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/bluez-plugin
LOCAL_UNSTRIPPED_PATH := $(TARGET_OUT_SHARED_LIBRARIES_UNSTRIPPED)/bluez-plugin
LOCAL_MODULE:=libbuiltinplugin

include $(BUILD_STATIC_LIBRARY)
