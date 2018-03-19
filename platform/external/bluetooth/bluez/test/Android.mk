LOCAL_PATH:= $(call my-dir)
BUILD_BTIOTEST:=0
BUILD_HCIEMU:=0
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)
TIBLUZCOMPILEDATE := $(shell date +%F_%T)
#
# hstest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	hstest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=hstest
include $(BUILD_EXECUTABLE)

#
# l2test
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-DCOMPILEDATE=\"$(TIBLUZCOMPILEDATE)\"

LOCAL_SRC_FILES:= \
	l2test.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=l2test

include $(BUILD_EXECUTABLE)

#
# rctest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	rctest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=rctest

include $(BUILD_EXECUTABLE)


#
# scotest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	scotest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=scotest

include $(BUILD_EXECUTABLE)

#
# agent
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	agent.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, dbus)

LOCAL_SHARED_LIBRARIES := \
	libdbus

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=agent

include $(BUILD_EXECUTABLE)

#
# attest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	attest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=attest

include $(BUILD_EXECUTABLE)

#
# avtest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	avtest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=avtest

include $(BUILD_EXECUTABLE)

#
# bdaddr
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	bdaddr.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=bdaddr

include $(BUILD_EXECUTABLE)

ifeq ($(BUILD_BTIOTEST),1)
#
# btiotest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	btiotest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, glib) \
	$(call include-path-for, glib)\glib


LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libglib

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=btiotest

include $(BUILD_EXECUTABLE)
endif #BTIOTEST


ifeq ($(BUILD_HCIEMU),1)
#
# hciemu
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	hciemu.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, glib) \
	$(call include-path-for, glib)\glib

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libc \
	libcutils

LOCAL_STATIC_LIBRARIES := \
	libglib_static

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=hciemu

include $(BUILD_EXECUTABLE)
endif #BUILD_HCIEMU

#
# lmptest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	lmptest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=lmptest

include $(BUILD_EXECUTABLE)

#
# sdptest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	sdptest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=sdptest

include $(BUILD_EXECUTABLE)

#
# gaptest
#
#
# hstest
#

include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_SRC_FILES:= \
	gaptest.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(call include-path-for, dbus)

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth libdbus

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=gaptest

include $(BUILD_EXECUTABLE)


#
# mcaptool
#
# libmcapapp
# mcaptool
#

include $(CLEAR_VARS)

LOCAL_HEALTH_DIR=../health/
LOCAL_MCAP_DIR:=$(LOCAL_HEALTH_DIR)test/
	
LOCAL_SRC_FILES:= \
	../health/mcap.c \
	../health/mcap_sync.c\
 	$(LOCAL_MCAP_DIR)mcap_app.c \
	$(LOCAL_MCAP_DIR)mcap_utils.c \
	$(LOCAL_MCAP_DIR)mcap_main.c \
	$(LOCAL_MCAP_DIR)mcap_manager.c \


LOCAL_CFLAGS:= \
	-DVERSION=\"$(TIBLUEZVER)\" \

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(LOCAL_PATH)/../gdbus \
	$(call include-path-for, glib) \
	$(call include-path-for, dbus) \
	$(LOCAL_MCAP_DIR) \
	$(LOCAL_PATH)/../health/test \


LOCAL_SHARED_LIBRARIES := \
	libbluetoothd \
	libbluetooth \
	libbtio \
	libdbus \
	libcutils \
	libglib 
#	libbluetooth-health


LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/bluez-plugin
LOCAL_UNSTRIPPED_PATH := $(TARGET_OUT_SHARED_LIBRARIES_UNSTRIPPED)/bluez-plugin
LOCAL_MODULE:=mcapapp
LOCAL_MODULE_TAGS:=optional

include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_CFLAGS:= \
	    -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-DSTORAGEDIR=\"/data/misc/bluetoothd\" \
	-DCONFIGDIR=\"/etc/bluetooth\"

LOCAL_SRC_FILES:= \
	../health/test/mcap_tool.c

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../health \
	$(LOCAL_PATH)/../btio \
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \
	$(LOCAL_PATH)/../gdbus \
	$(call include-path-for, glib) \
	$(call include-path-for, dbus)


#	libbluetoothd libbluetooth
LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth libdbus libglib  libbtio

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=mcaptool



#
# bneppkttool
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	bneppkttool.c \

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
        -DVERSION=\"$(TIBLUEZVER)\" \
	-DCONFIGDIR=\"/etc/bluetooth\" \
        -DNEED_PPOLL

LOCAL_C_INCLUDES:= \
	$(call include-path-for, glib) \

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=bneppkttool

include $(BUILD_EXECUTABLE)

