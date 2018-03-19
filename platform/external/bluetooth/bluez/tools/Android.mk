LOCAL_PATH:= $(call my-dir)
TIBLUEZVER := $(shell cat $(LOCAL_PATH)/../ti_bluez_version;)

#
# avinfo
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	avinfo.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=avinfo

include $(BUILD_EXECUTABLE)

#
# sdptool
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	sdptool.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" -fpermissive

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE:=sdptool

include $(BUILD_EXECUTABLE)

#
# hciconfig
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	csr.c \
	csr_h4.c \
	hciconfig.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DSTORAGEDIR=\"/tmp\" \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=hciconfig

include $(BUILD_EXECUTABLE)

#
# hcitool
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	hcitool.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DSTORAGEDIR=\"/tmp\" \
	-DVERSION=\"$(TIBLUEZVER)\"

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := eng
LOCAL_MODULE:=hcitool

include $(BUILD_EXECUTABLE)

#
# l2ping
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	l2ping.c

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=l2ping

include $(BUILD_EXECUTABLE)

#
# hciattach
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	hciattach.c \
	hciattach_ath3k.c \
	hciattach_qualcomm.c \
	hciattach_st.c \
	hciattach_ti.c \
	hciattach_tialt.c \

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
	-DVERSION=\"$(TIBLUEZVER)\" \
	-D__BSD_VISIBLE=1 \
	-DCONFIGDIR=\"/etc/bluetooth\" \
        -DNEED_PPOLL

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetoothd libbluetooth

LOCAL_MODULE:=hciattach

include $(BUILD_EXECUTABLE)

#
# rfcomm
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        kword.c \
        rfcomm.c \
        parser.c \
        lexer.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
        -DVERSION=\"$(TIBLUEZVER)\" \
	-DCONFIGDIR=\"/etc/bluetooth\" \
        -DNEED_PPOLL

LOCAL_C_INCLUDES:= \
        $(LOCAL_PATH)/../src \
        $(LOCAL_PATH)/../lib

LOCAL_SHARED_LIBRARIES := \
        libbluetooth

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=rfcomm

include $(BUILD_EXECUTABLE)

ifeq ($(BOARD_HAVE_BLUETOOTH_CSR),true)
#
# bccmd
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	bccmd.c \
	csr.c \
	csr_hci.c \
	csr_bcsp.c \
	csr_h4.c \
	csr_3wire.c \
	ubcsp.c

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
        -DVERSION=\"$(TIBLUEZVER)\" 

LOCAL_C_INCLUDES:=\
	$(LOCAL_PATH)/../lib \
	$(LOCAL_PATH)/../src \

LOCAL_SHARED_LIBRARIES := \
	libbluetooth libbluetoothd

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE:=bccmd
include $(BUILD_EXECUTABLE)
endif

#
# gatttool
#

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
        ../attrib/gatttool.c \
	../attrib/utils.c \
	../attrib/interactive.c \

LOCAL_CFLAGS:= \
        -Wno-missing-field-initializers \
        -DVERSION=\"$(TIBLUEZVER)\" \
	-DCONFIGDIR=\"/etc/bluetooth\" \
        -DNEED_PPOLL

LOCAL_C_INCLUDES:= \
        $(LOCAL_PATH)/../src \
        $(LOCAL_PATH)/../lib \
        $(LOCAL_PATH)/../btio \
	$(call include-path-for, glib) \
        $(LOCAL_PATH)/../attrib \
        $(LOCAL_PATH)/../../readline
	

LOCAL_STATIC_LIBRARIES := \
	libreadline_static \
	libhistory_static \
	libtermcap_static\

LOCAL_SHARED_LIBRARIES := \
        libbluetooth \
	libbluetoothd \
	libglib \
	libbtio

LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:=gatttool


include $(BUILD_EXECUTABLE)

