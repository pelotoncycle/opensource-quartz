/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2004-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef __LOG_H
#define __LOG_H

void info(const char *format, ...) __attribute__((format(printf, 1, 2)));
void warn(const char *format, ...) __attribute__((format(printf, 1, 2)));
void error(const char *format, ...) __attribute__((format(printf, 1, 2)));

void btd_debug(const char *format, ...) __attribute__((format(printf, 1, 2)));

void __btd_log_init(const char *debug, int detach);
void __btd_log_cleanup(void);
void __btd_toggle_debug(void);

struct btd_debug_desc {
	const char *file;
#define BTD_DEBUG_FLAG_DEFAULT (0)
#define BTD_DEBUG_FLAG_PRINT   (1 << 0)
	unsigned int flags;
} __attribute__((aligned(8)));

void __btd_enable_debug(struct btd_debug_desc *start,
					struct btd_debug_desc *stop);

#define LOGE ALOGE
#define LOGW ALOGW
#define LOGV ALOGV
#define LOGD ALOGD
#define LOGI ALOGI

#ifdef ANDROID
#define LOG_TAG "BtDebug"
#include <utils/Log.h>
#else
#define LOGD bt_debug
#endif

/**
 * DBG:
 * @fmt: format string
 * @arg...: list of arguments
 *
 * Simple macro around btd_debug() which also include the function
 * name it is called in.
 */
#define DBG(fmt, arg...) do { \
	static struct btd_debug_desc __btd_debug_desc \
	__attribute__((used, section("__debug"), aligned(8))) = { \
		.file = __FILE__, .flags = BTD_DEBUG_FLAG_DEFAULT, \
	}; \
	LOGD("%s:%s() " fmt,  __FILE__, __FUNCTION__ , ## arg); \
} while (0)

/**
 * INFO:
 * @fmt: format string
 * @arg...: list of arguments
 *
 * Simple macro around info() which also include the function
 * name it is called in.
 */
#define INFO(fmt, arg...) do { \
	info("%s:%s() " fmt,  __FILE__, __FUNCTION__ , ## arg); \
} while (0)

/**
 * ERROR:
 * @fmt: format string
 * @arg...: list of arguments
 *
 * Simple macro around error() which also include the function
 * name it is called in.
 */
#define ERROR(fmt, arg...) do { \
	error("%s:%s() " fmt,  __FILE__, __FUNCTION__ , ## arg); \
} while (0)

#endif
