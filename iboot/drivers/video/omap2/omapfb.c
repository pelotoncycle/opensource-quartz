/*
 * linux/drivers/video/omap2/dss/core.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * Some code and ideas taken from drivers/video/omap/ driver
 * by Imre Deak.
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

#define DSS_SUBSYS_NAME "OMAPFB"

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>
#include <video_fb.h>

#include <icom/omapdss.h>

#include "dss.h"
#include "dss_features.h"
#include "vram.h"

/*----------------------------------------------------------------------*/

#define HW_ALIGN		32

/*----------------------------------------------------------------------*/

#define MAX_OMAPFB_DISPLAYS		2
#define MAX_OMAPFB_NUM		2
/* max number of overlays to which a framebuffer data can be direct */
#if 0
#define OMAPFB_MAX_OVL_PER_FB 3
#else
#define OMAPFB_MAX_OVL_PER_FB 1
#endif

#if !defined(CONFIG_FB_OMAP2_NUM_FBS) || (CONFIG_FB_OMAP2_NUM_FBS < 1) || (CONFIG_FB_OMAP2_NUM_FBS > MAX_OMAPFB_NUM)
#error "Invalid CONFIG_FB_OMAP2_NUM_FBS\n"
#endif /* CONFIG_FB_OMAP2_NUM_FBS */

/*----------------------------------------------------------------------*/

struct omapfb2_device;

struct omapfb2_mem_region {
	int				id;
	u32		paddr;
	void	*vaddr;
#if 0
	struct vrfb	vrfb;
#endif
	unsigned long	size;
#if 0
	u8		type;		/* OMAPFB_PLANE_MEM_* */
	bool		alloc;		/* allocated by the driver */
	bool		map;		/* kernel mapped by the driver */
	atomic_t	map_count;
	struct rw_semaphore lock;
	atomic_t	lock_count;
#endif
};

struct omapfb_info {
	int id;
	struct omapfb2_mem_region *region;
	int num_overlays;
	struct omap_overlay *overlays[OMAPFB_MAX_OVL_PER_FB];
	struct omapfb2_device *fbdev;
	enum omap_dss_rotation_type rotation_type;
	u8 rotation[OMAPFB_MAX_OVL_PER_FB];
	bool mirror;
};

struct omapfb2_device {
#if 0
	struct device *dev;
	struct mutex  mtx;

	u32 pseudo_palette[17];

	int state;
#endif

	unsigned num_fbs;
#if 0
	struct fb_info *fbs[10];
	struct omapfb2_mem_region regions[10];
#else
	struct omapfb_info *fbs[MAX_OMAPFB_NUM];
	struct omapfb2_mem_region regions[MAX_OMAPFB_NUM];
#endif

	unsigned num_displays;
	struct omap_dss_device *displays[MAX_OMAPFB_DISPLAYS];
	unsigned num_overlays;
	struct omap_overlay *overlays[MAX_DSS_OVERLAYS];
	unsigned num_managers;
	struct omap_overlay_manager *managers[MAX_DSS_MANAGERS];

#if 0
	unsigned num_bpp_overrides;
	struct {
		struct omap_dss_device *dssdev;
		u8 bpp;
	} bpp_overrides[10];
#endif
};

/*----------------------------------------------------------------------*/

/*
 * The Graphic Device
 */
static GraphicDevice ctfb __attribute__ ((section (".data")));
static struct omap_dss_device *curr_display __attribute__ ((section (".data"))) = NULL;

static struct omapfb2_device *omapfb_dev __attribute__ ((section (".data"))) = NULL;
static struct omap_dss_board_info *dss_board_info __attribute__ ((section (".data"))) = NULL;

/*----------------------------------------------------------------------*/

/* find the display connected to this fb, if any */
static inline struct omap_dss_device *fb2display(struct omapfb_info *ofbi)
{
	int i;

	/* XXX: returns the display connected to first attached overlay */
	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ofbi->overlays[i]->manager)
			return ofbi->overlays[i]->manager->device;
	}

	return NULL;
}

static inline int omapfb_overlay_enable(struct omap_overlay *ovl,
		int enable)
{
	struct omap_overlay_info info;

	ovl->get_overlay_info(ovl, &info);
	if (info.enabled == enable)
		return 0;
	info.enabled = enable;
	return ovl->set_overlay_info(ovl, &info);
}

/*----------------------------------------------------------------------*/

static u32 omapfb_get_region_paddr(const struct omapfb_info *ofbi)
{
#if 0
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		return ofbi->region->vrfb.paddr[0];
	else
#endif
		return ofbi->region->paddr;
}

static void *omapfb_get_region_vaddr(const struct omapfb_info *ofbi)
{
#if 0
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		return ofbi->region->vrfb.vaddr[0];
	else
#endif
		return ofbi->region->vaddr;
}

#if 0
int omapfb_mode_to_dss_mode(struct fb_var_screeninfo *var,
		enum omap_color_mode *mode)
{
	enum omap_color_mode dssmode;
	int i;

	/* first match with nonstd field */
	if (var->nonstd) {
		for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
			struct omapfb_colormode *m = &omapfb_colormodes[i];
			if (var->nonstd == m->nonstd) {
				assign_colormode_to_var(var, m);
				*mode = m->dssmode;
				return 0;
			}
		}

		return -EINVAL;
	}

	/* then try exact match of bpp and colors */
	for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
		struct omapfb_colormode *m = &omapfb_colormodes[i];
		if (cmp_var_to_colormode(var, m)) {
			assign_colormode_to_var(var, m);
			*mode = m->dssmode;
			return 0;
		}
	}

	/* match with bpp if user has not filled color fields
	 * properly */
	switch (var->bits_per_pixel) {
	case 1:
		dssmode = OMAP_DSS_COLOR_CLUT1;
		break;
	case 2:
		dssmode = OMAP_DSS_COLOR_CLUT2;
		break;
	case 4:
		dssmode = OMAP_DSS_COLOR_CLUT4;
		break;
	case 8:
		dssmode = OMAP_DSS_COLOR_CLUT8;
		break;
	case 12:
		dssmode = OMAP_DSS_COLOR_RGB12U;
		break;
	case 16:
		dssmode = OMAP_DSS_COLOR_RGB16;
		break;
	case 24:
		dssmode = OMAP_DSS_COLOR_RGB24P;
		break;
	case 32:
		dssmode = OMAP_DSS_COLOR_ARGB32;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(omapfb_colormodes); ++i) {
		struct omapfb_colormode *m = &omapfb_colormodes[i];
		if (dssmode == m->dssmode) {
			assign_colormode_to_var(var, m);
			*mode = m->dssmode;
			return 0;
		}
	}

	return -EINVAL;
}
#else
int omapfb_mode_to_dss_mode(GraphicDevice *pGD,
		enum omap_color_mode *mode)
{
	/* match with bpp if user has not filled color fields
	 * properly */
	switch (pGD->gdfIndex) {
	case GDF_16BIT_565RGB:
		*mode = OMAP_DSS_COLOR_RGB16;
		break;
	case GDF_32BIT_X888RGB:
		*mode = OMAP_DSS_COLOR_ARGB32;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif

/*----------------------------------------------------------------------*/

#if 0
static inline int omapfb_get_recommended_bpp(struct omapfb2_device *fbdev,
		struct omap_dss_device *dssdev)
{
	int i;

	BUG_ON(dssdev->driver->get_recommended_bpp == NULL);

	for (i = 0; i < fbdev->num_bpp_overrides; ++i) {
		if (dssdev == fbdev->bpp_overrides[i].dssdev)
			return fbdev->bpp_overrides[i].bpp;
	}

	return dssdev->driver->get_recommended_bpp(dssdev);
}
#endif

/*----------------------------------------------------------------------*/

/* setup overlay according to the fb */
int omapfb_setup_overlay(struct omapfb_info *ofbi, struct omap_overlay *ovl,
		u16 posx, u16 posy, u16 outw, u16 outh)
{
	int r = 0;
#if 0
	struct fb_var_screeninfo *var = &fbi->var;
	struct fb_fix_screeninfo *fix = &fbi->fix;
#else
	GraphicDevice *pGD = (GraphicDevice *) & ctfb;
#endif
	enum omap_color_mode mode = 0;
	u32 data_start_p = 0;
	void *data_start_v = NULL;
	struct omap_overlay_info info;
	int xres, yres;
	int screen_width;
	int mirror;
#if 0
	int rotation = var->rotate;
	int i;
#else
	int rotation;
#endif

#if 0
	WARN_ON(!atomic_read(&ofbi->region->lock_count));
#endif

#if 0
	for (i = 0; i < ofbi->num_overlays; i++) {
		if (ovl != ofbi->overlays[i])
			continue;

		rotation = (rotation + ofbi->rotation[i]) % 4;
		break;
	}
#else
	rotation = 0;
#endif

	DSSDBG("setup_overlay %d, posx %d, posy %d, outw %d, outh %d\n", ofbi->id,
			posx, posy, outw, outh);

#if 0
	if (rotation == FB_ROTATE_CW || rotation == FB_ROTATE_CCW) {
		xres = var->yres;
		yres = var->xres;
	} else {
		xres = var->xres;
		yres = var->yres;
	}
#else
	xres = pGD->winSizeX;
	yres = pGD->winSizeY;
#endif

#if 0
	if (ofbi->region->size)
		omapfb_calc_addr(ofbi, var, fix, rotation,
				 &data_start_p, &data_start_v);
#else
	if (ofbi->region->size) {
		data_start_p = omapfb_get_region_paddr(ofbi);
		data_start_v = omapfb_get_region_vaddr(ofbi);
	}
#endif

#if 0
	r = omapfb_mode_to_dss_mode(var, &mode);
#else
	r = omapfb_mode_to_dss_mode(pGD, &mode);
#endif
	if (r) {
		DSSDBG("omapfb_mode_to_dss_mode failed");
		goto err;
	}

#if 0
	switch (var->nonstd) {
	case OMAPFB_COLOR_YUV422:
	case OMAPFB_COLOR_YUY422:
		if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
			screen_width = fix->line_length
				/ (var->bits_per_pixel >> 2);
			break;
		}
	default:
		screen_width = fix->line_length / (var->bits_per_pixel >> 3);
		break;
	}
#else
	{
		/* SGX requires stride to be a multiple of 32 pixels */
		screen_width = pGD->plnSizeX;
	}
#endif

	ovl->get_overlay_info(ovl, &info);

#if 0
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB)
		mirror = 0;
	else
		mirror = ofbi->mirror;
#else
	mirror = ofbi->mirror;
#endif

	info.paddr = data_start_p;
	info.vaddr = data_start_v;
	info.screen_width = screen_width;
	info.width = xres;
	info.height = yres;
	info.color_mode = mode;
	info.rotation_type = ofbi->rotation_type;
	info.rotation = rotation;
	info.mirror = mirror;

	info.pos_x = posx;
	info.pos_y = posy;
	info.out_width = outw;
	info.out_height = outh;

	r = ovl->set_overlay_info(ovl, &info);
	if (r) {
		DSSDBG("ovl->setup_overlay_info failed\n");
		goto err;
	}

	return 0;

err:
	DSSDBG("setup_overlay failed\n");
	return r;
}

/* apply var to the overlay */
int omapfb_apply_changes(struct omapfb_info *ofbi, int init)
{
	int r = 0;
#if 0
	struct fb_var_screeninfo *var = &fbi->var;
#else
	GraphicDevice *pGD = (GraphicDevice *) & ctfb;
#endif
	struct omap_overlay *ovl;
	u16 posx, posy;
	u16 outw, outh;
	int i;

#if 0
#ifdef DEBUG
	if (omapfb_test_pattern)
		fill_fb(fbi);
#endif
#endif

#if 0
	WARN_ON(!atomic_read(&ofbi->region->lock_count));
#endif

	for (i = 0; i < ofbi->num_overlays; i++) {
		ovl = ofbi->overlays[i];

		DSSDBG("apply_changes, fb %d, ovl %d\n", ofbi->id, ovl->id);

		if (ofbi->region->size == 0) {
			/* the fb is not available. disable the overlay */
			omapfb_overlay_enable(ovl, 0);
			if (!init && ovl->manager)
				ovl->manager->apply(ovl->manager);
			continue;
		}

		if (init || (ovl->caps & OMAP_DSS_OVL_CAP_SCALE) == 0) {
#if 0
			int rotation = (var->rotate + ofbi->rotation[i]) % 4;
			if (rotation == FB_ROTATE_CW ||
					rotation == FB_ROTATE_CCW) {
				outw = var->yres;
				outh = var->xres;
			} else {
				outw = var->xres;
				outh = var->yres;
			}
#else
			outw = pGD->winSizeX;
			outh = pGD->winSizeY;
#endif
		} else {
			outw = ovl->info.out_width;
			outh = ovl->info.out_height;
		}

		if (init) {
			posx = 0;
			posy = 0;
		} else {
			posx = ovl->info.pos_x;
			posy = ovl->info.pos_y;
		}

		r = omapfb_setup_overlay(ofbi, ovl, posx, posy, outw, outh);
		if (r)
			goto err;

		if (!init && ovl->manager) {
			struct omap_dss_device *dev;
			struct omap_dss_driver *drv;

			ovl->manager->apply(ovl->manager);

			drv = ovl->manager->device->driver;
			dev = ovl->manager->device;

			if (dev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE &&
				drv->get_update_mode(dev) != OMAP_DSS_UPDATE_AUTO)
				return drv->update(dev, 0, 0,
							dev->panel.timings.x_res,
							dev->panel.timings.y_res);
		}
	}
	return 0;
err:
	DSSDBG("apply_changes failed\n");
	return r;
}

/*----------------------------------------------------------------------*/

static void omapfb_free_fbmem(struct omapfb_info *ofbi)
{
#if 0
	struct omapfb2_device *fbdev = ofbi->fbdev;
#endif
	struct omapfb2_mem_region *rg;

	rg = ofbi->region;

#if 0
	WARN_ON(atomic_read(&rg->map_count));
#endif

	if (rg->paddr)
		if (omap_vram_free(rg->paddr, rg->size))
			DSSERR("VRAM FREE failed\n");

	if (rg->vaddr)
#if 0
		iounmap(rg->vaddr);
#else
		rg->vaddr = (void*)NULL;
#endif

#if 0
	if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
		/* unmap the 0 angle rotation */
		if (rg->vrfb.vaddr[0]) {
			iounmap(rg->vrfb.vaddr[0]);
			omap_vrfb_release_ctx(&rg->vrfb);
			rg->vrfb.vaddr[0] = NULL;
		}
	}
#endif

	rg->vaddr = NULL;
	rg->paddr = 0;
#if 0
	rg->alloc = 0;
#endif
	rg->size = 0;
}

static int omapfb_free_all_fbmem(struct omapfb2_device *fbdev)
{
	int i;

	DSSVDBG("free all fbmem\n");

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = fbdev->fbs[i];
		omapfb_free_fbmem(ofbi);
#if 0
		clear_fb_info(fbi);
#endif
	}

	return 0;
}

static int omapfb_alloc_fbmem(struct omapfb_info *ofbi, unsigned long size,
		unsigned long paddr)
{
#if 0
	struct omapfb2_device *fbdev = ofbi->fbdev;
#endif
	struct omapfb2_mem_region *rg;
	void *vaddr;
	int r;

	rg = ofbi->region;

	rg->paddr = 0;
	rg->vaddr = NULL;
#if 0
	memset(&rg->vrfb, 0, sizeof rg->vrfb);
#endif
	rg->size = 0;
#if 0
	rg->type = 0;
	rg->alloc = false;
	rg->map = false;
#endif

	size = PAGE_ALIGN(size);

	if (!paddr) {
		DSSVDBG("allocating %lu bytes for fb %d\n", size, ofbi->id);
		r = omap_vram_alloc(OMAP_VRAM_MEMTYPE_SDRAM, size, &paddr);
	} else {
		DSSVDBG("reserving %lu bytes at %lx for fb %d\n", size, paddr,
				ofbi->id);
		r = omap_vram_reserve(paddr, size);
	}

	if (r) {
		DSSERR("failed to allocate framebuffer\n");
		return -ENOMEM;
	}

#if 0
	if (ofbi->rotation_type != OMAP_DSS_ROT_VRFB) {
		vaddr = ioremap_wc(paddr, size);

		if (!vaddr) {
			dev_err(fbdev->dev, "failed to ioremap framebuffer\n");
			omap_vram_free(paddr, size);
			return -ENOMEM;
		}

		DBG("allocated VRAM paddr %lx, vaddr %p\n", paddr, vaddr);
	} else {
		r = omap_vrfb_request_ctx(&rg->vrfb);
		if (r) {
			dev_err(fbdev->dev, "vrfb create ctx failed\n");
			return r;
		}

		vaddr = NULL;
	}
#else
	vaddr = (void*)paddr;
	DSSVDBG("allocated VRAM paddr %lx, vaddr %p\n", paddr, vaddr);
#endif

	rg->paddr = paddr;
	rg->vaddr = vaddr;
	rg->size = size;
#if 0
	rg->alloc = 1;
#endif

	return 0;
}

/* allocate fbmem using display resolution as reference */
static int omapfb_alloc_fbmem_display(struct omapfb_info *ofbi, unsigned long size,
		unsigned long paddr)
{
#if 0
	struct omapfb2_device *fbdev = ofbi->fbdev;
#endif
	struct omap_dss_device *display;
	int bytespp;

	display = fb2display(ofbi);

	if (!display)
		return 0;

	DSSVDBG("allocating fbmem for display '%s'\n", display->driver_name);

#if 0
	switch (omapfb_get_recommended_bpp(fbdev, display)) {
#else
	switch (display->driver->get_recommended_bpp(display)) {
#endif
	case 16:
		bytespp = 2;
		break;
	case 24:
		bytespp = 4;
		break;
	default:
		bytespp = 4;
		break;
	}

	if (!size) {
		u16 w, h;

		display->driver->get_resolution(display, &w, &h);

#if 0
		if (ofbi->rotation_type == OMAP_DSS_ROT_VRFB) {
			size = max(omap_vrfb_min_phys_size(w, h, bytespp),
					omap_vrfb_min_phys_size(h, w, bytespp));

			DBG("adjusting fb mem size for VRFB, %u -> %lu\n",
					w * h * bytespp, size);
		} else {
			size = w * h * bytespp;
		}
#else
		/* SGX requires stride to be a multiple of 32 pixels */
		size = ALIGN(w, HW_ALIGN) * h * bytespp;
		size = PAGE_ALIGN(size) * 2;
#endif
	}

	if (!size)
		return 0;

	return omapfb_alloc_fbmem(ofbi, size, paddr);
}

static int omapfb_allocate_all_fbs(struct omapfb2_device *fbdev)
{
	int i, r;
	unsigned long vram_sizes[MAX_OMAPFB_NUM];
	unsigned long vram_paddrs[MAX_OMAPFB_NUM];

	memset(&vram_sizes, 0, sizeof(vram_sizes));
	memset(&vram_paddrs, 0, sizeof(vram_paddrs));

#if 0
	if (fbdev->dev->platform_data) {
		struct omapfb_platform_data *opd;
		opd = fbdev->dev->platform_data;
		for (i = 0; i < opd->mem_desc.region_cnt; ++i) {
			if (!vram_sizes[i]) {
				unsigned long size;
				unsigned long paddr;

				size = opd->mem_desc.region[i].size;
				paddr = opd->mem_desc.region[i].paddr;

				vram_sizes[i] = size;
				vram_paddrs[i] = paddr;
			}
		}
	}
#endif

	for (i = 0; i < fbdev->num_fbs; i++) {
		/* allocate memory automatically only for fb0, or if
		 * excplicitly defined with vram or plat data option */
		if (i == 0 || vram_sizes[i] != 0) {
			r = omapfb_alloc_fbmem_display(fbdev->fbs[i],
					vram_sizes[i], vram_paddrs[i]);

			if (r)
				return r;
		}
	}

#ifdef CONFIG_OMAP2_DSS_VERBOSE_DEBUG_SUPPORT
	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = fbdev->fbs[i];
		struct omapfb2_mem_region *rg;
		rg = ofbi->region;

		DSSVDBG("region%d phys %08x virt %p size=%lu\n",
				i,
				rg->paddr,
				rg->vaddr,
				rg->size);
	}
#endif /* CONFIG_OMAP2_DSS_VERBOSE_DEBUG_SUPPORT */

	return 0;
}

/* initialize fb_info, var, fix to something sane based on the display */
static int omapfb_fb_init(struct omapfb2_device *fbdev, struct omapfb_info *ofbi)
{
#if 0
	struct fb_var_screeninfo *var = &fbi->var;
#else
	GraphicDevice *pGD = (GraphicDevice *) & ctfb;
#endif
	struct omap_dss_device *display = fb2display(ofbi);
	int r = 0;

	if (ofbi->region->size == 0) {
		return -EINVAL;
	}

	if (display) {
		u16 w, h;

		display->driver->get_resolution(display, &w, &h);
#if 0
		if (rotation == FB_ROTATE_CW ||
				rotation == FB_ROTATE_CCW) {
			var->xres = h;
			var->yres = w;
		} else {
			var->xres = w;
			var->yres = h;
		}

		var->xres_virtual = var->xres;
		var->yres_virtual = var->yres;

		if (!var->bits_per_pixel) {
			switch (omapfb_get_recommended_bpp(fbdev, display)) {
			case 16:
				var->bits_per_pixel = 16;
				break;
			case 24:
				var->bits_per_pixel = 32;
				break;
			default:
				dev_err(fbdev->dev, "illegal display "
						"bpp\n");
				return -EINVAL;
			}
		}
#else
		pGD->winSizeX = w;
		pGD->winSizeY = h;
		/* SGX requires stride to be a multiple of 32 pixels */
		pGD->plnSizeX = ALIGN(w, HW_ALIGN); /* stride */
		pGD->plnSizeY = h;

		switch (display->driver->get_recommended_bpp(display)) {
		case 16:
			pGD->gdfBytesPP = 2;
			pGD->gdfIndex = GDF_16BIT_565RGB;
			break;
		case 24:
			pGD->gdfBytesPP = 4;
			pGD->gdfIndex = GDF_32BIT_X888RGB;
			break;
		default:
			DSSERR("illegal display bpp\n");
			return -EINVAL;
		}

		/* Video buffer */
		pGD->vprBase = ofbi->region->paddr;
		/* Framebuffer */
		pGD->memSize = pGD->plnSizeX * pGD->plnSizeY * pGD->gdfBytesPP;
		pGD->memSize = PAGE_ALIGN(pGD->memSize);
		pGD->frameAdrs = ofbi->region->paddr + pGD->memSize;

		snprintf(pGD->modeIdent, sizeof(pGD->modeIdent), "%ux%ux%u (%u@0x%08x)",
				pGD->winSizeX, pGD->winSizeY, pGD->gdfBytesPP, pGD->memSize, pGD->vprBase);
		if (display->driver_name)
			printf("Display: %s %s\n", display->driver_name, pGD->modeIdent);
		else
			printf("Display: %s\n", pGD->modeIdent);
#endif
	} else {
		DSSDBG("no display\n");
		r = -EINVAL;
	}

	return r;
}

static void omapfb_free_resources(struct omapfb2_device *fbdev)
{
	int i;

	DSSVDBG("free_resources\n");

	if (fbdev == NULL)
		return;

#if 0
	for (i = 0; i < fbdev->num_fbs; i++)
		unregister_framebuffer(fbdev->fbs[i]);
#endif

	/* free the reserved fbmem */
	omapfb_free_all_fbmem(fbdev);

#if 0
	for (i = 0; i < fbdev->num_fbs; i++) {
		fbinfo_cleanup(fbdev, fbdev->fbs[i]);
		framebuffer_release(fbdev->fbs[i]);
	}
#endif

	for (i = 0; i < fbdev->num_displays; i++) {
		if (fbdev->displays[i]->state != OMAP_DSS_DISPLAY_DISABLED)
			fbdev->displays[i]->driver->disable(fbdev->displays[i]);
#if 0
		omap_dss_put_device(fbdev->displays[i]);
#endif
	}

#if 0
	dev_set_drvdata(fbdev->dev, NULL);
#endif
	free(fbdev);
}

static int omapfb_create_framebuffers(struct omapfb2_device *fbdev)
{
	int r, i;

	fbdev->num_fbs = 0;

	DSSVDBG("create %d framebuffers\n",	CONFIG_FB_OMAP2_NUM_FBS);

	/* allocate fb_infos */
	for (i = 0; i < CONFIG_FB_OMAP2_NUM_FBS; i++) {
		struct omapfb_info *ofbi;

		ofbi = memalign(ARCH_DMA_MINALIGN, sizeof(struct omapfb_info));
		if (ofbi == NULL) {
			DSSDBG("unable to allocate memory for plane info\n");
			return -ENOMEM;
		}

		memset(ofbi, 0, sizeof(struct omapfb_info));

		fbdev->fbs[i] = ofbi;

		ofbi->fbdev = fbdev;
		ofbi->id = i;

		ofbi->region = &fbdev->regions[i];
		ofbi->region->id = i;

		/* assign these early, so that fb alloc can use them */
#if 0
		ofbi->rotation_type = def_vrfb ? OMAP_DSS_ROT_VRFB :
			OMAP_DSS_ROT_DMA;
		ofbi->mirror = def_mirror;
#else
		ofbi->rotation_type = OMAP_DSS_ROT_DMA;
		ofbi->mirror = 0;
#endif

		fbdev->num_fbs++;
	}

	DSSVDBG("fb_infos allocated\n");

	/* assign overlays for the fbs */
	for (i = 0; i < min(fbdev->num_fbs, fbdev->num_overlays); i++) {
		struct omapfb_info *ofbi = fbdev->fbs[i];

		ofbi->overlays[0] = fbdev->overlays[i];
		ofbi->num_overlays = 1;
	}

	/* allocate fb memories */
	r = omapfb_allocate_all_fbs(fbdev);
	if (r) {
		DSSDBG("failed to allocate fbmem\n");
		return r;
	}

	DSSVDBG("fbmems allocated\n");

	/* setup fb_infos */
	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = fbdev->fbs[i];

#if 0
		omapfb_get_mem_region(ofbi->region);
#endif
		r = omapfb_fb_init(fbdev, ofbi);
#if 0
		omapfb_put_mem_region(ofbi->region);
#endif

		if (r) {
			DSSDBG("failed to setup fb_info\n");
			return r;
		}
	}

	DSSVDBG("fb_infos initialized\n");

#if 0
	for (i = 0; i < fbdev->num_fbs; i++) {
		r = register_framebuffer(fbdev->fbs[i]);
		if (r != 0) {
			dev_err(fbdev->dev,
				"registering framebuffer %d failed\n", i);
			return r;
		}
	}

	DBG("framebuffers registered\n");
#endif

	for (i = 0; i < fbdev->num_fbs; i++) {
		struct omapfb_info *ofbi = fbdev->fbs[i];

#if 0
		omapfb_get_mem_region(ofbi->region);
#endif
		r = omapfb_apply_changes(ofbi, 1);
#if 0
		omapfb_put_mem_region(ofbi->region);
#endif

		if (r) {
			DSSERR("failed to change mode\n");
			return r;
		}
	}

	/* Enable fb0 */
	if (fbdev->num_fbs > 0) {
		struct omapfb_info *ofbi = fbdev->fbs[0];

		if (ofbi->num_overlays > 0) {
			struct omap_overlay *ovl = ofbi->overlays[0];

			r = omapfb_overlay_enable(ovl, 1);

			if (r) {
				DSSDBG("failed to enable overlay\n");
				return r;
			}
		}
	}

	DSSVDBG("create_framebuffers done\n");

	return 0;
}

/*----------------------------------------------------------------------*/

static int omapfb_init_display(struct omapfb2_device *fbdev,
		struct omap_dss_device *dssdev)
{
	struct omap_dss_driver *dssdrv = dssdev->driver;
	int r;

	r = dssdrv->enable(dssdev);
	if (r) {
		DSSWARN("failed to enable display '%s'\n", dssdev->name);
		return r;
	}

	if (dssdev->caps & OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE) {
		u16 w, h;
		if (dssdrv->enable_te) {
			r = dssdrv->enable_te(dssdev, 1);
			if (r) {
				DSSERR("failed to set TE\n");
				return r;
			}
		}

		if (dssdrv->set_update_mode) {
			r = dssdrv->set_update_mode(dssdev,
					OMAP_DSS_UPDATE_MANUAL);
			if (r) {
				DSSERR("failed to set update mode\n");
				return r;
			}
		}

		dssdrv->get_resolution(dssdev, &w, &h);
		r = dssdrv->update(dssdev, 0, 0, w, h);
		if (r) {
			DSSERR("failed to update display\n");
			return r;
		}
	} else {
		if (dssdrv->set_update_mode) {
			r = dssdrv->set_update_mode(dssdev,
					OMAP_DSS_UPDATE_AUTO);
			if (r) {
				DSSERR("failed to set update mode\n");
				return r;
			}
		}
	}

	return 0;
}

/*----------------------------------------------------------------------*/

static int omapfb_probe(void)
{
	struct omapfb2_device *fbdev = NULL;
	int r = 0;
	int i;
	struct omap_overlay *ovl;
	struct omap_dss_device *def_display;
	struct omap_dss_device *dssdev;

	DSSVDBG("omapfb_probe\n");

	dss_board_info = omap_dss_get_board_info();
	if (dss_board_info == NULL) {
		r = -EINVAL;
		goto err0;
	}

	if (dss_board_info->num_devices > MAX_OMAPFB_DISPLAYS) {
		DSSDBG("too many displays (%d)\n", dss_board_info->num_devices);
		r = -EUSERS;
		goto err0;
	}

	fbdev = memalign(ARCH_DMA_MINALIGN, sizeof(struct omapfb2_device));
	if (fbdev == NULL) {
		r = -ENOMEM;
		goto err0;
	}
	memset(fbdev, 0, sizeof(struct omapfb2_device));

	fbdev->num_displays = 0;
	for (i = 0; i < dss_board_info->num_devices; ++i) {
		dssdev = dss_board_info->devices[i];
		if (!dssdev->driver) {
			DSSDBG("no driver for display '%s'\n", dssdev->name);
			r = -ENODEV;
		}
		fbdev->displays[fbdev->num_displays++] = dssdev;
	}

	if (r)
		goto cleanup;

	if (fbdev->num_displays == 0) {
		DSSDBG("no displays\n");
		r = -EINVAL;
		goto cleanup;
	}

	fbdev->num_overlays = omap_dss_get_num_overlays();
	for (i = 0; i < fbdev->num_overlays; i++)
		fbdev->overlays[i] = omap_dss_get_overlay(i);

	fbdev->num_managers = omap_dss_get_num_overlay_managers();
	for (i = 0; i < fbdev->num_managers; i++)
		fbdev->managers[i] = omap_dss_get_overlay_manager(i);

	r = omapfb_create_framebuffers(fbdev);
	if (r)
		goto cleanup;

#if 0
	for (i = 0; i < fbdev->num_managers; i++) {
		struct omap_overlay_manager *mgr;
		mgr = fbdev->managers[i];
		r = mgr->apply(mgr);
		if (r)
			DSSDBG("failed to apply dispc config\n");
	}

	DSSVDBG("mgr->apply'ed\n");
#endif

	/* gfx overlay should be the default one. find a display
	 * connected to that, and use it as default display */
	ovl = omap_dss_get_overlay(0);
	if (ovl->manager && ovl->manager->device) {
		def_display = ovl->manager->device;
	} else {
		DSSWARN("cannot find default display\n");
		def_display = NULL;
	}

	curr_display = def_display;

	if (def_display) {
#if 0
		struct omapfb_info *ofbi = fbdev->fbs[0];
#endif

		r = omapfb_init_display(fbdev, def_display);
		if (r) {
			DSSERR("failed to init default display\n");
			goto cleanup;
		}

#if 0
		r = -ENODEV;
		if (fb2display(ofbi) == def_display)
			r = omapfb_apply_changes(ofbi, 0);
		if (r)
			DSSERR("failed to apply default display: %d\n", r);
#endif
	}

#if 0
	DSSVDBG("create sysfs for fbs\n");
	r = omapfb_create_sysfs(fbdev);
	if (r) {
		DSSERR("failed to create sysfs entries\n");
		goto cleanup;
	}
#endif

	omapfb_dev = fbdev;
	DSSVDBG("omapfb initialized\n");

	return 0;

cleanup:
	omapfb_free_resources(fbdev);
err0:
	curr_display = NULL;
	DSSERR("failed to setup omapfb (%d)\n", r);
	return r;
}

/*----------------------------------------------------------------------*/

static int omapfb_pan_display(struct omapfb_info *ofbi)
{
	int r;

	DSSVDBG("pan_display(%d)\n", ofbi->id);

#if 0
	if (var->xoffset == fbi->var.xoffset &&
	    var->yoffset == fbi->var.yoffset)
		return 0;

	new_var = fbi->var;
	new_var.xoffset = var->xoffset;
	new_var.yoffset = var->yoffset;

	fbi->var = new_var;

	omapfb_get_mem_region(ofbi->region);

	r = omapfb_apply_changes(fbi, 0);

	omapfb_put_mem_region(ofbi->region);
#else
	r = omapfb_apply_changes(ofbi, 0);
#endif

	return r;
}

/*----------------------------------------------------------------------*/

static inline int __board_display_init(void)
{
	return -ENODEV;
}

/**
 * Initialize the display drivers for UI.
 *
 * Returns none-zero value if the error occurred
 */
int board_display_init(void)
		__attribute__((weak, alias("__board_display_init")));

/*----------------------------------------------------------------------*/

/* U-Boot cfb_console.c functions */

/* returns GraphicDevice struct or NULL */
void *video_hw_init(void)
{
	int r;

	memset(&ctfb, 0, sizeof(ctfb));

	/* init the OMAP DSS & panel drivers */
	r = board_display_init();
	if (r) {
		DSSERR("failed to init board display (%d)\n", r);
		return NULL;
	}

	/* init the OMAP FB driver */
	r = omapfb_probe();
	if (r)
		return NULL;

	return &ctfb;
}

#ifdef CONFIG_FB_OMAP2_ROTATE180
static void omapfb_rotate180(void *dst, const void *src, unsigned int length)
{
    unsigned int length4 = length >> 2;
    register unsigned int *dst32 = ((unsigned int*)dst) + length4 - 1;
    register unsigned int *src32 = (unsigned int*)src;
    register unsigned int i = 0;

	if (ctfb.gdfBytesPP == 4) {
    	while (i < length4) {
        	*dst32-- = *src32++;
        	i++;
    	}
	} else if (ctfb.gdfBytesPP == 2) {
    	while (i < length4) {
        	*dst32-- = ((*src32 >> 16) & 0x0FFFF) | ((*src32 & 0x0FFFF) << 16);
        	src32++;
        	i++;
    	}
	}
}
#endif /* CONFIG_FB_OMAP2_ROTATE180 */

#ifdef CONFIG_VIDEO_FLUSH
void video_flush(void)
{
	if (curr_display && curr_display->state == OMAP_DSS_DISPLAY_ACTIVE
			&& omapfb_dev && omapfb_dev->num_fbs > 0) {
		struct omapfb_info *ofbi = omapfb_dev->fbs[0];

		if (ofbi->num_overlays > 0) {
			int r;

#ifdef CONFIG_FB_OMAP2_ROTATE180
			omapfb_rotate180((void*)ctfb.vprBase, (void*)ctfb.frameAdrs, ctfb.memSize);
#else
			memcpy((void*)ctfb.vprBase, (void*)ctfb.frameAdrs, (size_t)ctfb.memSize);
#endif /* CONFIG_FB_OMAP2_ROTATE180 */
#if 0
			flush_cache((unsigned long)ctfb.vprBase, ctfb.memSize);
#else
			/* for large transfers, where speed matters most, it's
			 * more efficient to flush the entire dcache using
			 * flush_dcache_all() rather than using flush_dcache_range()
			 */
			flush_dcache_all();
#endif

			r = omapfb_pan_display(ofbi);
			if (r) {
				DSSERR("failed to flush display (%d)\n", r);
				return;
			}
		}
	}
}
#endif /* CONFIG_VIDEO_FLUSH */

#ifdef CONFIG_VIDEO_BRIGHTNESS
static int curr_brightness __attribute__ ((section (".data"))) = 0;

void video_set_brightness(int brightness)
{
	curr_brightness = brightness;

	if (curr_display && curr_display->set_backlight)
		curr_display->set_backlight(curr_display, brightness);
}

int video_get_brightness(void)
{
	return curr_brightness;
}
#endif /* CONFIG_VIDEO_BRIGHTNESS */

#if 0
void video_set_lut(unsigned int index,	/* color number */
		    unsigned char r,	/* red */
		    unsigned char g,	/* green */
		    unsigned char b	/* blue */
		    )
{
	/* We don't support 8-bit display */
	return;
}
#endif

/*----------------------------------------------------------------------*/

u32 videolfb_get_fbbase(void)
{
	if (curr_display && curr_display->state == OMAP_DSS_DISPLAY_ACTIVE)
		return (u32)ctfb.vprBase;
	return 0;
}

u32 videolfb_get_fbsize(void)
{
	if (curr_display && curr_display->state == OMAP_DSS_DISPLAY_ACTIVE)
		return (u32)ctfb.memSize;
	return 0;
}

/*----------------------------------------------------------------------*/

