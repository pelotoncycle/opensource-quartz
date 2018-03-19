LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

#
# libreadline
#

LOCAL_SRC_FILES:= \
	readline.c\
	funmap.c \
	keymaps.c \
	vi_mode.c \
	parens.c \
	rltty.c \
	bind.c \
	isearch.c \
	kill.c \
	undo.c \
	input.c \
	callback.c \
	terminal.c \
	nls.c \
	search.c \
	text.c \
	misc.c \
	compat.c \
	display.c \
	signals.c \
	complete.c \
	emacs_keymap.c \
	vi_keymap.c \
	savestring.c \
	util.c \
	macro.c \
	xmalloc.c \
	xfree.c \
	tilde.c \

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../ \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/readline \

LOCAL_CFLAGS:= \
	-DANDROID_STUB \
	-DRL_LIBRARY_VERSION='"6.2"' \
	-DHAVE_CONFIG_H \

LOCAL_MODULE:=libreadline_static

include $(BUILD_STATIC_LIBRARY)

#
# libhistory
#
include $(CLEAR_VARS)
LOCAL_SRC_FILES:= \
	history.c \
	histsearch.c \
	histexpand.c \
	histfile.c \
	shell.c \
	mbutil.c \

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/../ \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/readline \

LOCAL_CFLAGS:= \
	-DANDROID_STUB \
	-DRL_LIBRARY_VERSION='"6.2"' \
	-DHAVE_CONFIG_H \

LOCAL_MODULE:=libhistory_static

include $(BUILD_STATIC_LIBRARY)
