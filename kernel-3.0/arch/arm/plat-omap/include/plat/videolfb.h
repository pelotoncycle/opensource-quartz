/*
 * File: arch/arm/plat-omap/include/videolfb.h
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

#ifndef __OMAP_DSS_VIDEOLFB_H
#define __OMAP_DSS_VIDEOLFB_H

#include <asm/types.h>

#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
extern int videolfb_is_enabled(void);
extern u32 videolfb_get_base(void);
extern u32 videolfb_get_size(void);
#else /* CONFIG_FB_OMAP_BOOTLOADER_INIT */
#define	videolfb_is_enabled()	(0)
#define	videolfb_get_base()		(0)
#define	videolfb_get_size()		(0)
#endif /* CONFIG_FB_OMAP_BOOTLOADER_INIT */

#endif /* __OMAP_DSS_VIDEOLFB_H */
