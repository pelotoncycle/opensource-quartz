/*
 * include/linux/android_boot.h
 *
 * Copyright (C) 2012 InnoComm Mobile Technology Corp.
 * Author: James Wu <james.wu@innocomm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifndef _LINUX_ANDROID_BOOT_H
#define _LINUX_ANDROID_BOOT_H

/* Android Boot Reason (InnoComm Proprietary Feature) */
#define ANDROID_BOOT_REASON_NORMAL			0x55667701	/* Normal Android Boot */
#define ANDROID_BOOT_REASON_CHARGER			0x55667702	/* Power-off Charging */
#define ANDROID_BOOT_REASON_RECOVERY		0x55667703	/* Android Recovery Boot */
#define ANDROID_BOOT_REASON_ALARM			0x55667705	/* Power-on Alarm */
#define ANDROID_BOOT_REASON_PCBA			0x55667706	/* Android Recovery Boot for PCBA */

/* Android Boot Mode (Android Official Feature) */
#define ANDROID_BOOT_MODE_NORMAL			0x77665501	/* Android Normal Mode */
#define ANDROID_BOOT_MODE_FACTORY			0x77665502	/* Android Factory Mode */
#define ANDROID_BOOT_MODE_CHARGER			0x77665503	/* Android Power-off Charger Mode */

#ifdef CONFIG_ANDROID_BOOT_REASON

/**
 * Get the current ANDROID_BOOT_REASON_XXXXXX
 */
u32 android_boot_get_reason(void);

/**
 * Get the current ANDROID_BOOT_MODE_XXXXXX
 */
u32 android_boot_get_mode(void);

/**
 * Get the carrier name
 */
const char *android_boot_get_carrier(void);

/**
 * Get the Android serial number
 */
const char *android_boot_get_serialno(void);

/**
 * Get the wifi mac address
 */
const char *android_boot_get_wifimac(void);

/**
 * Get the ethernet mac address
 */
const char *android_boot_get_ethaddr(void);

#else /* !CONFIG_ANDROID_BOOT_REASON */

#define android_boot_get_reason()			(ANDROID_BOOT_REASON_NORMAL)
#define android_boot_get_mode()				(ANDROID_BOOT_MODE_NORMAL)
#define android_boot_get_carrier()			(NULL)
#define android_boot_get_serialno()			(NULL)
#define android_boot_get_wifimac()			(NULL)
#define android_boot_get_ethaddr()			(NULL)

#endif /* CONFIG_ANDROID_BOOT_REASON */

#endif /* _LINUX_ANDROID_BOOT_H */
