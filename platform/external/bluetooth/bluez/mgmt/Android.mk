LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

#
# mgmt
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	main.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, glib) \
	$(call include-path-for, glib)/glib

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=mgmt

include $(BUILD_EXECUTABLE)
