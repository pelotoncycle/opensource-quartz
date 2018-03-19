/*
 * VRAM manager for OMAP
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __OMAP_VRAM_H__
#define __OMAP_VRAM_H__

#include <linux/types.h>

#define OMAP_VRAM_MEMTYPE_SDRAM		0
#define OMAP_VRAM_MEMTYPE_SRAM		1
#define OMAP_VRAM_MEMTYPE_MAX		1

#if 0
extern int omap_vram_add_region(unsigned long paddr, size_t size);
#endif
extern int omap_vram_free(unsigned long paddr, size_t size);
extern int omap_vram_alloc(int mtype, size_t size, unsigned long *paddr);
extern int omap_vram_reserve(unsigned long paddr, size_t size);
#if 0
extern void omap_vram_get_info(unsigned long *vram, unsigned long *free_vram,
		unsigned long *largest_free_block);
#endif

#if 0
#ifdef CONFIG_OMAP2_VRAM
ulong omap_vram_reserve_sdram_memblock(ulong addr);
#else
static inline ulong omap_vram_reserve_sdram_memblock(ulong addr) { return addr; }
#endif
#endif

#endif
