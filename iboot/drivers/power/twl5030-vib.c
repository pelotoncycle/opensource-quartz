/*
 * (C) Copyright 2012
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <errno.h>

#include <twl4030.h>

#include <icom/vibrator.h>

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

#define CONFIG_TWL5030_VIB_DEBUG
/*#define CONFIG_TWL5030_VIB_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_TWL5030_VIB_DEBUG
#define CONFIG_TWL5030_VIB_DEBUG
#endif
#ifndef CONFIG_TWL5030_VIB_VERBOSE_DEBUG
#define CONFIG_TWL5030_VIB_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL5030_VIB_DEBUG
#define VIB_DPRINT(fmt, args...) \
	do {printf("[vib] " fmt, ##args);} while (0)
#define VIB_DPUTS(fmt) \
	do {puts("[vib] " fmt);} while (0)
#else /* CONFIG_TWL5030_VIB_DEBUG */
#define VIB_DPRINT(fmt, args...) \
	do {} while (0)
#define VIB_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL5030_VIB_DEBUG */

#ifdef CONFIG_TWL5030_VIB_VERBOSE_DEBUG
#define VIB_VPRINT(fmt, args...) \
	do {printf("[vib] " fmt, ##args);} while (0)
#define VIB_VPUTS(fmt) \
	do {puts("[vib] " fmt);} while (0)
#else /* CONFIG_TWL5030_VIB_VERBOSE_DEBUG */
#define VIB_VPRINT(fmt, args...) \
	do {} while (0)
#define VIB_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL5030_VIB_VERBOSE_DEBUG */

#define VIB_PRINT(fmt, args...) \
	do {printf("vib: " fmt, ##args);} while (0)
#define VIB_PUTS(fmt) \
	do {puts("vib: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define VIB_ERR(fmt, args...) \
	do {printf("VIB: " fmt, ##args);} while (0)

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

void vibrate(unsigned int ms)
{
}

/*----------------------------------------------------------------------*/

void twl5030_vib_init(void)
{
}
