/*
 * config_defaults.h - sane defaults for everyone
 *
 * Copyright (c) 2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _CONFIG_DEFAULTS_H_
#define _CONFIG_DEFAULTS_H_

/* James Wu: we use aboot instead of bootm */
#if 0
/* Support bootm-ing different OSes */
#define CONFIG_BOOTM_LINUX 1
#define CONFIG_BOOTM_NETBSD 1
#define CONFIG_BOOTM_RTEMS 1
#else
#define CONFIG_ANDROID_BOOT	1
#define CONFIG_ABOOT_CMDLINE_SIZE	1024
#endif

#define CONFIG_GZIP 1
#define CONFIG_ZLIB 1

#ifndef CONFIG_SPL_BUILD
#define CONFIG_PARTITIONS 1
#endif

#endif
