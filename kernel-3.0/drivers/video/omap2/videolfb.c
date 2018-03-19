/*
 * File: drivers/video/omap2/videolfb.c
 *
 * Copyright (C) 2012 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <asm/setup.h>
#include <mach/io.h>

#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT

static int _videolfb_is_enabled = 0;
static u32 _phys_videolfb_start = 0;
static u32 _phys_videolfb_size = 0;

int videolfb_is_enabled(void)
{
	return _videolfb_is_enabled;
}

u32 videolfb_get_base(void)
{
	return _phys_videolfb_start;
}

u32 videolfb_get_size(void)
{
	return _phys_videolfb_size;
}

static int __init parse_tag_videolfb(const struct tag *tag)
{
	_phys_videolfb_start = tag->u.videolfb.lfb_base;
	_phys_videolfb_size = tag->u.videolfb.lfb_size;

	if ((_phys_videolfb_start != 0) && (_phys_videolfb_size != 0)) {
		_videolfb_is_enabled = 1;
		if (_phys_videolfb_size != PAGE_ALIGN(_phys_videolfb_size))
			pr_warning("Bootloader FB size (%u) is not page-aligned\n", _phys_videolfb_size);
	} else
		_videolfb_is_enabled = 0;

#ifdef DEBUG
	pr_info("Bootloader FB: 0x%08X (%u)\n", _phys_videolfb_start, _phys_videolfb_size);
#endif /* DEBUG */

    return 0;
}
__tagtable(ATAG_VIDEOLFB, parse_tag_videolfb);

#else /* !CONFIG_FB_OMAP_BOOTLOADER_INIT */

/* Remove "Ignoring unrecognised tag 0x54410008" message */
static int __init parse_tag_videolfb(const struct tag *tag)
{
    return 0;
}
__tagtable(ATAG_VIDEOLFB, parse_tag_videolfb);

#endif /* CONFIG_FB_OMAP_BOOTLOADER_INIT */
