/*
 * linux/drivers/staging/android/android_boot.c
 *
 * Copyright (C) 2012 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/android_boot.h>
#include <asm/setup.h>

#define ANDROID_BOOT_CARRIER_LENGTH		32
#define ANDROID_BOOT_SERIALNO_LENGTH	20
#define ANDROID_BOOT_WIFIMAC_LENGTH		12
#define ANDROID_BOOT_ETHADDR_LENGTH		12

struct android_boot_t {
	const char* str;
	u32 value;
};

static const struct android_boot_t reasons[] = {
	{ "normal",		ANDROID_BOOT_REASON_NORMAL		},
	{ "charger",	ANDROID_BOOT_REASON_CHARGER		},
	{ "recovery",	ANDROID_BOOT_REASON_RECOVERY	},
	{ "alarm",		ANDROID_BOOT_REASON_ALARM		},
	{ "pcba",		ANDROID_BOOT_REASON_PCBA		},
};

static const struct android_boot_t modes[] = {
	{ "normal",		ANDROID_BOOT_MODE_NORMAL		},
	{ "charger",	ANDROID_BOOT_MODE_CHARGER		},
	{ "factory",	ANDROID_BOOT_MODE_FACTORY		},
	{ "factory2",	ANDROID_BOOT_MODE_FACTORY		},
};

static u32 android_boot_reason = ANDROID_BOOT_REASON_NORMAL;
static u32 android_boot_mode = ANDROID_BOOT_MODE_NORMAL;

static char android_boot_carrier[ANDROID_BOOT_CARRIER_LENGTH+1];
static char android_boot_serialno[ANDROID_BOOT_SERIALNO_LENGTH+1];
static char android_boot_wifimac[ANDROID_BOOT_WIFIMAC_LENGTH+1];
static char android_boot_ethaddr[ANDROID_BOOT_ETHADDR_LENGTH+1];

static int __init android_boot_iboot_setup(char *opt)
{
	return 1;
}
__setup("iboot=", android_boot_iboot_setup);

static int __init android_boot_console_setup(char *opt)
{
	return 1;
}
__setup("androidboot.console=", android_boot_console_setup);

static int __init android_boot_bootloader_setup(char *opt)
{
	return 1;
}
__setup("androidboot.bootloader=", android_boot_bootloader_setup);

static int __init android_boot_carrier_setup(char *opt)
{
	if (strlen(opt) > ANDROID_BOOT_CARRIER_LENGTH)
		pr_warning("android_boot: carrier too long\n");

	strncpy(android_boot_carrier, opt, ANDROID_BOOT_CARRIER_LENGTH);
	android_boot_carrier[ANDROID_BOOT_CARRIER_LENGTH] = '\0';

#ifdef CONFIG_ANDROID_BOOT_REASON_DEBUG
	pr_info("android_boot: carrier %s\n", android_boot_carrier);
#endif /* CONFIG_ANDROID_BOOT_REASON_DEBUG */

	return 1;
}
__setup("androidboot.carrier=", android_boot_carrier_setup);

static int __init android_boot_serialno_setup(char *opt)
{
	if (strlen(opt) > ANDROID_BOOT_SERIALNO_LENGTH)
		pr_warning("android_boot: serialno too long\n");

	strncpy(android_boot_serialno, opt, ANDROID_BOOT_SERIALNO_LENGTH);
	android_boot_serialno[ANDROID_BOOT_SERIALNO_LENGTH] = '\0';

#ifdef CONFIG_ANDROID_BOOT_REASON_DEBUG
	pr_info("android_boot: serialno %s\n", android_boot_serialno);
#endif /* CONFIG_ANDROID_BOOT_REASON_DEBUG */

	return 1;
}
__setup("androidboot.serialno=", android_boot_serialno_setup);

static int __init android_boot_wifimac_setup(char *opt)
{
	if (strlen(opt) > ANDROID_BOOT_WIFIMAC_LENGTH)
		pr_warning("android_boot: wifimac too long\n");

	strncpy(android_boot_wifimac, opt, ANDROID_BOOT_WIFIMAC_LENGTH);
	android_boot_wifimac[ANDROID_BOOT_WIFIMAC_LENGTH] = '\0';

#ifdef CONFIG_ANDROID_BOOT_REASON_DEBUG
	pr_info("android_boot: wifimac %s\n", android_boot_wifimac);
#endif /* CONFIG_ANDROID_BOOT_REASON_DEBUG */

	return 1;
}
__setup("androidboot.wifimac=", android_boot_wifimac_setup);

static int __init android_boot_ethaddr_setup(char *opt)
{
	if (strlen(opt) > ANDROID_BOOT_ETHADDR_LENGTH)
		pr_warning("android_boot: ethaddr too long\n");

	strncpy(android_boot_ethaddr, opt, ANDROID_BOOT_ETHADDR_LENGTH);
	android_boot_ethaddr[ANDROID_BOOT_ETHADDR_LENGTH] = '\0';

#ifdef CONFIG_ANDROID_BOOT_REASON_DEBUG
	pr_info("android_boot: ethaddr %s\n", android_boot_ethaddr);
#endif /* CONFIG_ANDROID_BOOT_REASON_DEBUG */

	return 1;
}
__setup("androidboot.ethaddr=", android_boot_ethaddr_setup);

static int __init android_boot_mode_setup(char *opt)
{
	int i;
	
	for (i = 0 ; i < ARRAY_SIZE(modes) ; i++) {
		if (!strcmp(modes[i].str, (const char*)opt)) {
			android_boot_mode = modes[i].value;
#ifdef CONFIG_ANDROID_BOOT_REASON_DEBUG
			pr_info("android_boot: mode %s\n", opt);
#endif /* CONFIG_ANDROID_BOOT_REASON_DEBUG */
			return 1;
		}
	}

	pr_warning("android_boot: unknown mode %s\n", opt);
	android_boot_mode = ANDROID_BOOT_MODE_NORMAL;

	return 1;
}
__setup("androidboot.mode=", android_boot_mode_setup);

static int __init android_boot_reason_setup(char *opt)
{
	int i;
	
	for (i = 0 ; i < ARRAY_SIZE(reasons) ; i++) {
		if (!strcmp(reasons[i].str, (const char*)opt)) {
			android_boot_reason = reasons[i].value;
#ifdef CONFIG_ANDROID_BOOT_REASON_DEBUG
			pr_info("android_boot: reason %s\n", opt);
#endif /* CONFIG_ANDROID_BOOT_REASON_DEBUG */
			return 1;
		}
	}

	pr_warning("android_boot: unknown reason %s\n", opt);
	android_boot_reason = ANDROID_BOOT_REASON_RECOVERY;

	return 1;
}
__setup("androidboot.reason=", android_boot_reason_setup);

u32 android_boot_get_reason(void)
{
	return android_boot_reason;
}
EXPORT_SYMBOL(android_boot_get_reason);

u32 android_boot_get_mode(void)
{
	return android_boot_mode;
}
EXPORT_SYMBOL(android_boot_get_mode);

const char *android_boot_get_carrier(void)
{
	return android_boot_carrier;
}
EXPORT_SYMBOL(android_boot_get_carrier);

const char *android_boot_get_serialno(void)
{
	return android_boot_serialno;
}
EXPORT_SYMBOL(android_boot_get_serialno);

const char *android_boot_get_wifimac(void)
{
	return android_boot_wifimac;
}
EXPORT_SYMBOL(android_boot_get_wifimac);

const char *android_boot_get_ethaddr(void)
{
	return android_boot_ethaddr;
}
EXPORT_SYMBOL(android_boot_get_ethaddr);
