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

#define DSS_SUBSYS_NAME "CORE"

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <command.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/compiler.h> 

#include <asm/arch/cpu.h>
#include <asm/arch/clocks.h>
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX)
#include <asm/arch/sys_proto.h>
#endif /* CONFIG_OMAP44XX || CONFIG_OMAP54XX */

#include <icom/omapdss.h>

#include "dss.h"
#include "dss_features.h"

/*----------------------------------------------------------------------*/

static struct omap_dss_board_info *dss_board_info __attribute__ ((section (".data"))) = NULL;
static int dss_initiated __attribute__ ((section (".data"))) = 0;

/*----------------------------------------------------------------------*/

#if 0
static struct {
	struct platform_device *pdev;

	struct regulator *vdds_dsi_reg;
	struct regulator *vdds_sdi_reg;
} core;

static char *def_disp_name;
module_param_named(def_disp, def_disp_name, charp, 0);
MODULE_PARM_DESC(def_disp, "default display name");

#ifdef DEBUG
unsigned int dss_debug;
module_param_named(debug, dss_debug, bool, 0644);
#endif

static int omap_dss_register_device(struct omap_dss_device *);
static void omap_dss_unregister_device(struct omap_dss_device *);

/* REGULATORS */

struct regulator *dss_get_vdds_dsi(void)
{
	struct regulator *reg;

	if (core.vdds_dsi_reg != NULL)
		return core.vdds_dsi_reg;

	reg = regulator_get(&core.pdev->dev, "vdds_dsi");
	if (!IS_ERR(reg))
		core.vdds_dsi_reg = reg;

	return reg;
}

struct regulator *dss_get_vdds_sdi(void)
{
	struct regulator *reg;

	if (core.vdds_sdi_reg != NULL)
		return core.vdds_sdi_reg;

	reg = regulator_get(&core.pdev->dev, "vdds_sdi");
	if (!IS_ERR(reg))
		core.vdds_sdi_reg = reg;

	return reg;
}
#endif

/*----------------------------------------------------------------------*/

/*#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_OMAP2_DSS_DEBUG_SUPPORT)*/
#if 0
static int dss_debug_show(struct seq_file *s, void *unused)
{
	void (*func)(struct seq_file *) = s->private;
	func(s);
	return 0;
}

static int dss_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dss_debug_show, inode->i_private);
}

static const struct file_operations dss_debug_fops = {
	.open           = dss_debug_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static struct dentry *dss_debugfs_dir;

static int dss_initialize_debugfs(void)
{
	dss_debugfs_dir = debugfs_create_dir("omapdss", NULL);
	if (IS_ERR(dss_debugfs_dir)) {
		int err = PTR_ERR(dss_debugfs_dir);
		dss_debugfs_dir = NULL;
		return err;
	}

	debugfs_create_file("clk", S_IRUGO, dss_debugfs_dir,
			&dss_debug_dump_clocks, &dss_debug_fops);

#ifdef CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS
	debugfs_create_file("dispc_irq", S_IRUGO, dss_debugfs_dir,
			&dispc_dump_irqs, &dss_debug_fops);
#endif

#if defined(CONFIG_OMAP2_DSS_DSI) && defined(CONFIG_OMAP2_DSS_COLLECT_IRQ_STATS)
	dsi_create_debugfs_files_irq(dss_debugfs_dir, &dss_debug_fops);
#endif

	debugfs_create_file("dss", S_IRUGO, dss_debugfs_dir,
			&dss_dump_regs, &dss_debug_fops);
	debugfs_create_file("dispc", S_IRUGO, dss_debugfs_dir,
			&dispc_dump_regs, &dss_debug_fops);
#ifdef CONFIG_OMAP2_DSS_RFBI
	debugfs_create_file("rfbi", S_IRUGO, dss_debugfs_dir,
			&rfbi_dump_regs, &dss_debug_fops);
#endif
#ifdef CONFIG_OMAP2_DSS_DSI
	dsi_create_debugfs_files_reg(dss_debugfs_dir, &dss_debug_fops);
#endif
#ifdef CONFIG_OMAP2_DSS_VENC
	debugfs_create_file("venc", S_IRUGO, dss_debugfs_dir,
			&venc_dump_regs, &dss_debug_fops);
#endif
	debugfs_create_file("hdmi", S_IRUGO, dss_debugfs_dir,
			&hdmi_dump_regs, &dss_debug_fops);
	return 0;
}

static void dss_uninitialize_debugfs(void)
{
	if (dss_debugfs_dir)
		debugfs_remove_recursive(dss_debugfs_dir);
}
#else /* CONFIG_DEBUG_FS && CONFIG_OMAP2_DSS_DEBUG_SUPPORT */
static inline int dss_initialize_debugfs(void)
{
	return 0;
}
static inline void dss_uninitialize_debugfs(void)
{
}
#endif /* CONFIG_DEBUG_FS && CONFIG_OMAP2_DSS_DEBUG_SUPPORT */

/*----------------------------------------------------------------------*/

#if 0
/*
 * The value of HIGH_RES_TPUT corresponds to one dispc pipe layer of
 * 1920x1080x4(bpp)x60(Hz) = ~500000(MiB/s). We add another 100000
 * for the other partial screen pipes. This is above the threshold for
 * selecting the higher OPP and L3 frequency, so it's "as fast" as we
 * can go so covers the higest supported resolution.
 */
#if 0
#define HIGH_RES_TPUT 600000 /* MiB/s */
#else
#define HIGH_RES_TPUT 800000 /* MiB/s */
#endif

#if 0
static void omap_dss_request_bandwidth(struct omap_dss_device *display)
{
	struct device *dss_dev;

	if (display->panel.timings.x_res * display->panel.timings.y_res >=
							(1080 * 1920)) {
		dss_dev = omap_hwmod_name_get_dev("dss_core");
		if (dss_dev)
			omap_pm_set_min_bus_tput(dss_dev,
						 OCP_INITIATOR_AGENT,
						 HIGH_RES_TPUT);
		else
			DSSDBG("Failed to set L3 bus speed\n");
	}
}

#if 0
static void omap_dss_reset_bandwidth(void)
{
	struct device *dss_dev;
	dss_dev = omap_hwmod_name_get_dev("dss_core");
	if (IS_ERR_OR_NULL(dss_dev))
		return;
	omap_pm_set_min_bus_tput(dss_dev,
				 OCP_INITIATOR_AGENT, -1);
}
#else
static void omap_dss_reset_bandwidth(struct omap_dss_device *display)
{
	struct device *dss_dev;
	dss_dev = omap_hwmod_name_get_dev("dss_core");
	if (IS_ERR_OR_NULL(dss_dev))
		return;
	if (display->panel.timings.x_res * display->panel.timings.y_res >=
							(1080 * 1920)) {
		omap_pm_set_min_bus_tput(dss_dev,
				OCP_INITIATOR_AGENT, -1);
	}
}
#endif
#else
void omap_dss_request_high_bandwidth(struct device *dss_dev)
{
	if (IS_ERR_OR_NULL(dss_dev))
		DSSERR("%s: wrong dss_dev pointer\n", __func__);
	else if (!omap_pm_set_min_bus_tput(dss_dev,
					OCP_INITIATOR_AGENT, HIGH_RES_TPUT))
		return;
	DSSDBG("Failed to set high L3 bus speed\n");
}

void omap_dss_reset_high_bandwidth(struct device *dss_dev)
{
	if (IS_ERR_OR_NULL(dss_dev))
		DSSERR("%s: wrong dss_dev pointer\n", __func__);
	else if (!omap_pm_set_min_bus_tput(dss_dev, OCP_INITIATOR_AGENT, -1))
		return;
	DSSDBG("Failed to reset high L3 bus speed\n");
}
#endif
#endif

/*----------------------------------------------------------------------*/

/* PLATFORM DEVICE */
static int omap_dss_probe(struct platform_device *pdev)
{
#if 0
	struct omap_dss_board_info *pdata = pdev->dev.platform_data;
	int r;
	int i;

	core.pdev = pdev;
#else
	int r;
#endif

	dss_features_init();

	dss_init_overlay_managers(pdev);
	dss_init_overlays(pdev);

#if 0
	if (dss_has_feature(FEAT_OVL_WB))
		dss_init_writeback(pdev);
#endif

	r = dss_init_platform_driver();
	if (r) {
		DSSERR("Failed to initialize DSS platform driver\n");
		goto err_dss;
	}

	r = dispc_init_platform_driver();
	if (r) {
		DSSERR("Failed to initialize dispc platform driver\n");
		goto err_dispc;
	}

#if 0
	r = rfbi_init_platform_driver();
	if (r) {
		DSSERR("Failed to initialize rfbi platform driver\n");
		goto err_rfbi;
	}

	r = venc_init_platform_driver();
	if (r) {
		DSSERR("Failed to initialize venc platform driver\n");
		goto err_venc;
	}
#endif

#ifdef CONFIG_OMAP2_DSS_DSI
	r = dsi_init_platform_driver();
	if (r) {
		DSSERR("Failed to initialize DSI platform driver\n");
		goto err_dsi;
	}
#endif /* CONFIG_OMAP2_DSS_DSI */

#if 0
	r = hdmi_init_platform_driver();
	if (r) {
		DSSERR("Failed to initialize hdmi\n");
		goto err_hdmi;
	}

	r = dss_initialize_debugfs();
	if (r)
		goto err_debugfs;

	for (i = 0; i < pdata->num_devices; ++i) {
		struct omap_dss_device *dssdev = pdata->devices[i];

		if (def_disp_name && strcmp(def_disp_name, dssdev->name) == 0)
			pdata->default_device = dssdev;

		r = omap_dss_register_device(dssdev);
		if (r) {
			DSSERR("device %d %s register failed %d\n", i,
				dssdev->name ?: "unnamed", r);

			while (--i >= 0)
				omap_dss_unregister_device(pdata->devices[i]);

			goto err_register;
		}
	}

#ifdef CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT
	omap_dss_backlight_init(&pdev->dev);
#endif /* CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT */
#endif

	return 0;

#if 0
err_register:
	dss_uninitialize_debugfs();
err_debugfs:
	hdmi_uninit_platform_driver();
err_hdmi:
	dsi_uninit_platform_driver();
#endif
#ifdef CONFIG_OMAP2_DSS_DSI
err_dsi:
#if 0
	venc_uninit_platform_driver();
err_venc:
#endif
	dispc_uninit_platform_driver();
#endif /* CONFIG_OMAP2_DSS_DSI */
err_dispc:
#if 0
	rfbi_uninit_platform_driver();
err_rfbi:
#endif
	dss_uninit_platform_driver();
err_dss:

	return r;
}

#if 0
static int omap_dss_remove(struct platform_device *pdev)
{
	struct omap_dss_board_info *pdata = pdev->dev.platform_data;
	int i;

#ifdef CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT
	omap_dss_backlight_exit();
#endif /* CONFIG_OMAP2_DSS_ANDROID_BACKLIGHT */

	dss_uninitialize_debugfs();

	venc_uninit_platform_driver();
	dispc_uninit_platform_driver();
	rfbi_uninit_platform_driver();
	dsi_uninit_platform_driver();
	hdmi_uninit_platform_driver();
	dss_uninit_platform_driver();

	if (dss_has_feature(FEAT_OVL_WB))
		dss_uninit_writeback(pdev);
	dss_uninit_overlays(pdev);
	dss_uninit_overlay_managers(pdev);

	for (i = 0; i < pdata->num_devices; ++i)
		omap_dss_unregister_device(pdata->devices[i]);

	return 0;
}
#endif

#if 0
static void omap_dss_shutdown(struct platform_device *pdev)
{
	DSSDBG("shutdown\n");
	dss_disable_all_devices();
}

static int omap_dss_suspend(struct platform_device *pdev, pm_message_t state)
{
	DSSDBG("suspend %d\n", state.event);

	return dss_suspend_all_devices();
}

static int omap_dss_resume(struct platform_device *pdev)
{
	DSSDBG("resume\n");

	return dss_resume_all_devices();
}
#endif

#if 0
static struct platform_driver omap_dss_driver = {
	.probe          = omap_dss_probe,
	.remove         = omap_dss_remove,
	.shutdown	= omap_dss_shutdown,
	.suspend	= omap_dss_suspend,
	.resume		= omap_dss_resume,
	.driver         = {
		.name   = "omapdss",
		.owner  = THIS_MODULE,
	},
};
#endif

#if 0
/* BUS */
static int dss_bus_match(struct device *dev, struct device_driver *driver)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);

	DSSDBG("bus_match. dev %s/%s, drv %s\n",
			dev_name(dev), dssdev->driver_name, driver->name);

	return strcmp(dssdev->driver_name, driver->name) == 0;
}
#endif

#if 0
static ssize_t device_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			dssdev->name ?
			dssdev->name : "");
}

static struct device_attribute default_dev_attrs[] = {
	__ATTR(name, S_IRUGO, device_name_show, NULL),
	__ATTR_NULL,
};

static ssize_t driver_name_show(struct device_driver *drv, char *buf)
{
	struct omap_dss_driver *dssdrv = to_dss_driver(drv);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			dssdrv->driver.name ?
			dssdrv->driver.name : "");
}
static struct driver_attribute default_drv_attrs[] = {
	__ATTR(name, S_IRUGO, driver_name_show, NULL),
	__ATTR_NULL,
};
#endif

#if 0
static struct bus_type dss_bus_type = {
	.name = "omapdss",
	.match = dss_bus_match,
	.dev_attrs = default_dev_attrs,
	.drv_attrs = default_drv_attrs,
};

static void dss_bus_release(struct device *dev)
{
	DSSDBG("bus_release\n");
}

static struct device dss_bus = {
	.release = dss_bus_release,
};

struct bus_type *dss_get_bus(void)
{
	return &dss_bus_type;
}
#endif

/*----------------------------------------------------------------------*/

static void dss_enable_all_clocks(void)
{
#if defined(CONFIG_OMAP44XX)
	/* CD_DSS (l3_dss_clkdm) (clkdm_clk_enable/clkdm_wakeup) */
	omap_clk_domain(&prcm->cm_dss_clkstctrl, CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
	omap_pwrdm_wait_transition(&prcm->pm_dss_pwrstctrl);

	/* Enable dss_dss_fck */
	omap_clk_optional_enable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_DSSCLK);

	/* Enable DSS clock module (dss_fck) */
	omap_clk_enable(&prcm->cm_dss_dss_clkctrl, MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	/* Enable dss_sys_clk */
	omap_clk_optional_enable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_SYS_CLK);
	/* Enable dss_tv_clk */
	omap_clk_optional_enable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_TV_CLK);
	/* Enable dss_48mhz_clk */
	omap_clk_optional_enable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_48MHZ_CLK);

	DSSVDBG("%s: PM_DSS_PWRSTCTRL 0x%08x, PM_DSS_PWRSTST 0x%08x\n", __func__,
			__raw_readl(&prcm->pm_dss_pwrstctrl), __raw_readl(&prcm->pm_dss_pwrstst));
	DSSVDBG("%s: CM_DSS_CLKSTCTRL 0x%08x, CM_DSS_DSS_CLKCTRL 0x%08x\n", __func__,
			__raw_readl(&prcm->cm_dss_clkstctrl), __raw_readl(&prcm->cm_dss_dss_clkctrl));
#else
#error "dss_enable_all_clocks() not implemented\n"
#endif /* CONFIG_OMAP44XX */
}

static void dss_disable_all_clocks(void)
{
#if defined(CONFIG_OMAP44XX)
	/* Disable dss_sys_clk */
	omap_clk_optional_disable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_SYS_CLK);
	/* Disable dss_tv_clk */
	omap_clk_optional_disable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_TV_CLK);
	/* Disable dss_48mhz_clk */
	omap_clk_optional_disable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_48MHZ_CLK);

	/* Disable DSS clock module (dss_fck) */
	omap_clk_disable(&prcm->cm_dss_dss_clkctrl);

	/* Disable dss_dss_fck */
	omap_clk_optional_disable(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_DSSCLK);

	/* CD_DSS (l3_dss_clkdm) (clkdm_clk_disable/clkdm_sleep) */
	omap_clk_domain(&prcm->cm_dss_clkstctrl, CD_CLKCTRL_CLKTRCTRL_SW_SLEEP);
	omap_pwrdm_wait_transition(&prcm->pm_dss_pwrstctrl);

	if (omap_pwrdm_read_pwrst(&prcm->pm_dss_pwrstctrl) != PWRDM_POWER_OFF) {
		DSSERR("%s: PD_DSS is not OFF\n", __func__);
#ifdef CONFIG_OMAP4_PRCM_DEBUG
		prcmdebug_dump(PRCMDEBUG_ON);
#else
		DSSINFO("%s: PM_DSS_PWRSTCTRL 0x%08x, PM_DSS_PWRSTST 0x%08x\n", __func__,
			__raw_readl(&prcm->pm_dss_pwrstctrl), __raw_readl(&prcm->pm_dss_pwrstst));
		DSSINFO("%s: CM_DSS_CLKSTCTRL 0x%08x, CM_DSS_DSS_CLKCTRL 0x%08x\n", __func__,
			__raw_readl(&prcm->cm_dss_clkstctrl), __raw_readl(&prcm->cm_dss_dss_clkctrl));
#endif
	}

#else
#error "dss_disable_all_clocks() not implemented\n"
#endif /* CONFIG_OMAP44XX */
}

static void dispc_disable_outputs(void)
{
#define LCD_EN_MASK		(0x1 << 0)
#define DIGIT_EN_MASK		(0x1 << 1)

#define FRAMEDONE_IRQ_SHIFT	0
#define EVSYNC_EVEN_IRQ_SHIFT	2
#define EVSYNC_ODD_IRQ_SHIFT	3
#define FRAMEDONE2_IRQ_SHIFT	22
#define FRAMEDONETV_IRQ_SHIFT	24
#define FRAMEDONE3_IRQ_SHIFT	30

/*
 * FRAMEDONE_IRQ_TIMEOUT: how long (in milliseconds) to wait during DISPC
 *     reset before deciding that something has gone wrong
 */
#define FRAMEDONE_IRQ_TIMEOUT		100

#if defined(CONFIG_OMAP44XX)

#define DISPC_IRQSTATUS		(DISPC_BASE + 0x0018)
#define DISPC_CONTROL1		(DISPC_BASE + 0x0040)
#define DISPC_CONTROL2		(DISPC_BASE + 0x0238)

	u32 v, irq_mask = 0;
	bool lcd_en, digit_en, lcd2_en;
	ulong start;

	/* store value of LCDENABLE and DIGITENABLE bits */
	v = __raw_readl(DISPC_CONTROL1);
	lcd_en = v & LCD_EN_MASK;
	digit_en = v & DIGIT_EN_MASK;

	/* store value of LCDENABLE for LCD2 */
	v = __raw_readl(DISPC_CONTROL2);
	lcd2_en = v & LCD_EN_MASK;

	if (!(lcd_en | digit_en | lcd2_en))
		return; /* no managers currently enabled */

	/*
	 * If any manager was enabled, we need to disable it before
	 * DSS clocks are disabled or DISPC module is reset
	 */
	if (lcd_en)
		irq_mask |= 1 << FRAMEDONE_IRQ_SHIFT;

	if (digit_en)
		irq_mask |= 1 << FRAMEDONETV_IRQ_SHIFT;

	if (lcd2_en)
		irq_mask |= 1 << FRAMEDONE2_IRQ_SHIFT;

	/*
	 * clear any previous FRAMEDONE, FRAMEDONETV,
	 * EVSYNC_EVEN/ODD, FRAMEDONE2 or FRAMEDONE3 interrupts
	 */
	__raw_writel(irq_mask, DISPC_IRQSTATUS);

	/* disable LCD and TV managers */
	v = __raw_readl(DISPC_CONTROL1);
	v &= ~(LCD_EN_MASK | DIGIT_EN_MASK);
	__raw_writel(v, DISPC_CONTROL1);

	/* disable LCD2 manager */
	v = __raw_readl(DISPC_CONTROL2);
	v &= ~LCD_EN_MASK;
	__raw_writel(v, DISPC_CONTROL2);

	start = get_timer(0);
	while ((__raw_readl(DISPC_IRQSTATUS) & irq_mask) != irq_mask) {
		if (get_timer(0) - start > FRAMEDONE_IRQ_TIMEOUT) {
			DSSERR("%s: timedout (0x%08X,0x%08X,0x%08X)\n", __func__,
					__raw_readl(DISPC_CONTROL1),
					__raw_readl(DISPC_CONTROL2),
					__raw_readl(DISPC_IRQSTATUS));
			break;
		}
		barrier();
	}

#undef DISPC_IRQSTATUS
#undef DISPC_CONTROL1
#undef DISPC_CONTROL2

#else
#error "dispc_disable_outputs() not implemented\n"
#endif
}

static int dss_reset(void)
{
	dispc_disable_outputs();

#if defined(CONFIG_OMAP44XX)

#define DSS_CONTROL			(DSS_BASE + 0x0040)
#define DSS_SYSSTATUS		(DSS_BASE + 0x0014)
#define	SYSS_RESETDONE		(1 << 0)

	u32 start = 1 + (LDELAY << 1);

	/*
	 * clear DSS_CONTROL register to switch DSS clock sources to
	 * PRCM clock, if any
	 */
	__raw_writel(0, DSS_CONTROL);

	while (!(__raw_readl(DSS_SYSSTATUS) & SYSS_RESETDONE)) {
		if (--start == 0) {
			DSSDBG("DSS_SYSSTATUS(0x%08x)=0x%08x\n", DSS_SYSSTATUS, __raw_readl(DSS_SYSSTATUS));
			return -ETIMEDOUT;
		}
		barrier();
	}

	return 0;
#else
#error "dss_reset() not implemented\n"
#endif
}

static void omap_dss_reset(void)
{
	int r = 0;

	dss_enable_all_clocks();

	r = dss_reset();
	if (r)
		DSSERR("DSS RST err %d\n", r);

#if defined(CONFIG_OMAP44XX)
#define DISPC_SYSCONFIG			(DSS_BASE + 0x1010)
#define DISPC_SYSSTATUS			(DSS_BASE + 0x1014)
	r = ocp_softreset(DISPC_SYSCONFIG, DISPC_SYSSTATUS,
			SYSC_MSTANDBY_SMART | SYSC_SIDLE_SMART | SYSC_ENWAKEUP | SYSC_AUTOIDLE);
	if (r)
		DSSERR("DISPC RST err %d\n", r);

#define	DSI1_SYSCONFIG			(DSS_BASE + 0x4010)
#define	DSI1_SYSSTATUS			(DSS_BASE + 0x4014)
	r = ocp_softreset(DSI1_SYSCONFIG, DSI1_SYSSTATUS,
			SYSC_MSTANDBY_FORCE | SYSC_SIDLE_SMART | SYSC_ENWAKEUP | SYSC_AUTOIDLE);
	if (r)
		DSSERR("DSI1 RST err %d\n", r);

#define	DSI2_SYSCONFIG			(DSS_BASE + 0x5010)
#define	DSI2_SYSSTATUS			(DSS_BASE + 0x5014)
	r = ocp_softreset(DSI2_SYSCONFIG, DSI2_SYSSTATUS,
			SYSC_MSTANDBY_FORCE | SYSC_SIDLE_SMART | SYSC_ENWAKEUP | SYSC_AUTOIDLE);
	if (r)
		DSSERR("DSI2 RST err %d\n", r);

#define	HDMI_SYSCONFIG			(DSS_BASE + 0x6010)
	r = ocp_softreset_type2(HDMI_SYSCONFIG, TYPE2_SYSC_SIDLE_SMART_WAKEUP);
	if (r)
		DSSERR("HDMI RST err %d\n", r);

#define	RFBI_SYSCONFIG			(DSS_BASE + 0x2010)
#define	RFBI_SYSSTATUS			(DSS_BASE + 0x2014)
	r = ocp_softreset(RFBI_SYSCONFIG, RFBI_SYSSTATUS,
			SYSC_MSTANDBY_FORCE | SYSC_SIDLE_SMART | SYSC_AUTOIDLE);
	if (r)
		DSSERR("RFBI RST err %d\n", r);

#else
#error "omap_dss_reset() not implemented\n"
#endif

	dss_disable_all_clocks();
}

/*----------------------------------------------------------------------*/

int omap_display_init(struct omap_dss_board_info *board_data)
{
	int r = 0;

	dss_board_info = NULL;
	dss_initiated = 0;

	omap_dss_reset();

	if (!board_data || board_data->num_devices <= 0) {
		DSSDBG("no board info available\n");
		return -EINVAL;
	}

	dss_board_info = board_data;

	r = omap_dss_probe(NULL);
	if (!r)
		dss_initiated = 1;

	return r;
}

void omap_display_shutdown(void)
{
	if (dss_initiated && dss_board_info) {
		struct omap_dss_device *dssdev;
		int i;

		for (i = 0; i < dss_board_info->num_devices; ++i) {
			dssdev = dss_board_info->devices[i];
			if (dssdev->driver && dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
				DSSVDBG("driver_shutdown: dev %s, drv %s\n",
						dssdev->driver_name, dssdev->driver->driver_name);

				dssdev->driver->disable(dssdev);
				if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED)
					printf("Display: %s disabled\n", dssdev->driver->driver_name);
			}
		}
		dss_runtime_put();

		dss_initiated = 0;
	}
}

/*----------------------------------------------------------------------*/

struct omap_dss_board_info* omap_dss_get_board_info(void)
{
	return dss_board_info;
}

/*----------------------------------------------------------------------*/

/* DRIVER */
static int dss_driver_probe(struct omap_dss_driver *dssdrv, struct omap_dss_device *dssdev)
{
	int r;
#if 0
	struct omap_dss_driver *dssdrv = to_dss_driver(dev->driver);
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct omap_dss_board_info *pdata = core.pdev->dev.platform_data;
#else
	struct omap_dss_board_info *pdata = dss_board_info;
#endif
	bool force;

	DSSVDBG("driver_probe: dev %s, drv %s\n",
			dssdev->driver_name, dssdrv->driver_name);

	dss_init_device(dssdev);

	force = pdata->default_device == dssdev;
	dss_recheck_connections(dssdev, force);

	r = dssdrv->probe(dssdev);

	if (r) {
		DSSERR("driver '%s' probe failed: %d\n", dssdrv->driver_name, r);
		dss_uninit_device(dssdev);
		return r;
	}

	DSSVDBG("probe done for device '%s'\n", dssdev->driver_name);

	dssdev->driver = dssdrv;

	return 0;
}

#if 0
static int dss_driver_remove(struct omap_dss_driver *dssdrv, struct omap_dss_device *dssdev)
{
#if 0
	struct omap_dss_driver *dssdrv = to_dss_driver(dev->driver);
	struct omap_dss_device *dssdev = to_dss_device(dev);
#endif

	DSSVDBG("driver_remove: dev %s, drv %s\n",
			dssdev->driver_name, dssdrv->driver_name);

	dssdrv->remove(dssdev);

	dss_uninit_device(dssdev);

	dssdev->driver = NULL;

	return 0;
}
#endif

static void omap_dss_driver_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		blocking_notifier_call_chain(&dssdev->state_notifiers,
					OMAP_DSS_DISPLAY_DISABLED, dssdev);
	dssdev->driver->disable_orig(dssdev);
	dssdev->first_vsync = false;
}

static int omap_dss_driver_enable(struct omap_dss_device *dssdev)
{
	int r;
#if 0
	omap_dss_request_bandwidth(dssdev);
#endif
	r = dssdev->driver->enable_orig(dssdev);
	if (!r && dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		blocking_notifier_call_chain(&dssdev->state_notifiers,
					OMAP_DSS_DISPLAY_ACTIVE, dssdev);
		printf("Display: %s enabled\n", dssdev->driver->driver_name);
	}

	return r;
}

#if 0
static int omap_dss_driver_suspend(struct omap_dss_device *dssdev)
{
	int r = dssdev->driver->suspend_orig(dssdev);
#if 0
#if 0
	omap_dss_reset_bandwidth();
#else
	omap_dss_reset_bandwidth(dssdev);
#endif
#endif
	return r;
}
#endif

int omap_dss_register_driver(struct omap_dss_driver *dssdriver)
{
#if 0
	dssdriver->driver.bus = &dss_bus_type;
	dssdriver->driver.probe = dss_driver_probe;
	dssdriver->driver.remove = dss_driver_remove;
#else
	struct omap_dss_device *dssdev = NULL;
	int i;

	if (!dssdriver->driver_name) {
		DSSDBG("no driver name for display driver\n");
		return -EINVAL;
	}

	if (dss_board_info) {
		for (i = 0; i < dss_board_info->num_devices; ++i) {
			dssdev = dss_board_info->devices[i];
			if (!strcmp(dssdriver->driver_name, dssdev->driver_name))
				goto dssdev_found;
		}
	}

	DSSERR("no dss device for display '%s'\n", dssdriver->driver_name);
	return -ENODEV;
#endif

dssdev_found:
	if (dssdriver->get_resolution == NULL)
		dssdriver->get_resolution = omapdss_default_get_resolution;
	if (dssdriver->get_recommended_bpp == NULL)
		dssdriver->get_recommended_bpp =
			omapdss_default_get_recommended_bpp;

	dssdriver->disable_orig = dssdriver->disable;
	dssdriver->disable = omap_dss_driver_disable;
	dssdriver->enable_orig = dssdriver->enable;
	dssdriver->enable = omap_dss_driver_enable;

#if 0
	dssdriver->suspend_orig = dssdriver->suspend;
	dssdriver->suspend = omap_dss_driver_suspend;

	return driver_register(&dssdriver->driver);
#else
	return dss_driver_probe(dssdriver, dssdev);
#endif
}
EXPORT_SYMBOL(omap_dss_register_driver);

#if 0
void omap_dss_unregister_driver(struct omap_dss_driver *dssdriver)
{
#if 0
	driver_unregister(&dssdriver->driver);
#else
	struct omap_dss_device *dssdev = NULL;
	int i;

	if (!dssdriver->driver_name) {
		DSSDBG("no driver name for display driver\n");
		return;
	}

	if (dss_board_info) {
		for (i = 0; i < dss_board_info->num_devices; ++i) {
			dssdev = dss_board_info->devices[i];
			if (dssdev->driver == dssdriver)
				goto dssdev_found;
		}
	}

	DSSERR("no registered dss device for display '%s'\n", dssdriver->driver_name);
	return;

dssdev_found:
	dss_driver_remove(dssdriver, dssdev);
#endif
}
EXPORT_SYMBOL(omap_dss_unregister_driver);
#endif

/*----------------------------------------------------------------------*/

#if 0
/* DEVICE */
static void reset_device(struct device *dev, int check)
{
	u8 *dev_p = (u8 *)dev;
	u8 *dev_end = dev_p + sizeof(*dev);
	void *saved_pdata;

	saved_pdata = dev->platform_data;
	if (check) {
		/*
		 * Check if there is any other setting than platform_data
		 * in struct device; warn that these will be reset by our
		 * init.
		 */
		dev->platform_data = NULL;
		while (dev_p < dev_end) {
			if (*dev_p) {
				WARN("%s: struct device fields will be "
						"discarded\n",
				     __func__);
				break;
			}
			dev_p++;
		}
	}
	memset(dev, 0, sizeof(*dev));
	dev->platform_data = saved_pdata;
}


static void omap_dss_dev_release(struct device *dev)
{
	reset_device(dev, 0);
}

static int omap_dss_register_device(struct omap_dss_device *dssdev)
{
	static int dev_num;

	WARN_ON(!dssdev->driver_name);

	reset_device(&dssdev->dev, 1);
	dssdev->dev.bus = &dss_bus_type;
	dssdev->dev.parent = &dss_bus;
	dssdev->dev.release = omap_dss_dev_release;
	dev_set_name(&dssdev->dev, "display%d", dev_num++);
	return device_register(&dssdev->dev);
}

static void omap_dss_unregister_device(struct omap_dss_device *dssdev)
{
	device_unregister(&dssdev->dev);
}

/*----------------------------------------------------------------------*/

/* BUS */
static int omap_dss_bus_register(void)
{
	int r;

	r = bus_register(&dss_bus_type);
	if (r) {
		DSSERR("bus register failed\n");
		return r;
	}

	dev_set_name(&dss_bus, "omapdss");
	r = device_register(&dss_bus);
	if (r) {
		DSSERR("bus driver register failed\n");
		bus_unregister(&dss_bus_type);
		return r;
	}

	return 0;
}

/* INIT */

#ifdef CONFIG_OMAP2_DSS_MODULE
static void omap_dss_bus_unregister(void)
{
	device_unregister(&dss_bus);

	bus_unregister(&dss_bus_type);
}

static int __init omap_dss_init(void)
{
	int r;

	r = omap_dss_bus_register();
	if (r)
		return r;

	r = platform_driver_register(&omap_dss_driver);
	if (r) {
		omap_dss_bus_unregister();
		return r;
	}

	return 0;
}

static void __exit omap_dss_exit(void)
{
	if (core.vdds_dsi_reg != NULL) {
		regulator_put(core.vdds_dsi_reg);
		core.vdds_dsi_reg = NULL;
	}

	if (core.vdds_sdi_reg != NULL) {
		regulator_put(core.vdds_sdi_reg);
		core.vdds_sdi_reg = NULL;
	}

	platform_driver_unregister(&omap_dss_driver);

	omap_dss_bus_unregister();
}

module_init(omap_dss_init);
module_exit(omap_dss_exit);
#else
static int __init omap_dss_init(void)
{
	return omap_dss_bus_register();
}

static int __init omap_dss_init2(void)
{
	return platform_driver_register(&omap_dss_driver);
}

core_initcall(omap_dss_init);
device_initcall(omap_dss_init2);
#endif

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("OMAP2/3 Display Subsystem");
MODULE_LICENSE("GPL v2");
#endif

/*----------------------------------------------------------------------*/

#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY > 0)
/* dss_display */
static int do_display_disable(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	omap_display_shutdown();
	return 0;
}

U_BOOT_CMD(
	dss_disable, 1, 0,	do_display_disable,
	"Disable DSS display",
	""
);
#endif /* CONFIG_BOOTDELAY */

#ifdef CONFIG_OMAP2_DSS_DEBUG_SUPPORT

#ifdef CONFIG_OMAP2_DSS_DSI
void dsi1_dump_regs(void);
void dsi2_dump_regs(void);

/* dsi1 */
static int do_dump_dsi1_regs(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	puts("----------------------------------------------\n");
	dsi1_dump_regs();
	puts("----------------------------------------------\n");
	return 0;
}

U_BOOT_CMD(
	dsi1, 1, 0,	do_dump_dsi1_regs,
	"Dump OMAP DSI1 registers",
	""
);

/* dsi2 */
static int do_dump_dsi2_regs(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	puts("----------------------------------------------\n");
	dsi2_dump_regs();
	puts("----------------------------------------------\n");
	return 0;
}

U_BOOT_CMD(
	dsi2, 1, 0,	do_dump_dsi2_regs,
	"Dump OMAP DSI1 registers",
	""
);
#endif /* CONFIG_OMAP2_DSS_DSI */

/* dispc */
static int do_dump_dispc_regs(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	puts("----------------------------------------------\n");
	dispc_dump_regs();
	puts("----------------------------------------------\n");
	return 0;
}

U_BOOT_CMD(
	dispc, 1, 0,	do_dump_dispc_regs,
	"Dump OMAP DISPC registers",
	""
);

/* dss */
static int do_dump_dss_regs(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	puts("----------------------------------------------\n");
	dss_dump_regs();
	puts("----------------------------------------------\n");
	return 0;
}

U_BOOT_CMD(
	dss, 1, 0,	do_dump_dss_regs,
	"Dump OMAP DSS registers",
	""
);

/*----------------------------------------------------------------------*/

/* dssclk */
static int do_dump_dss_clks(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	dss_debug_dump_clocks();
	return 0;
}

U_BOOT_CMD(
	dssclk, 1, 0,	do_dump_dss_clks,
	"Dump OMAP DSS clocks",
	""
);

#endif /* CONFIG_OMAP2_DSS_DEBUG_SUPPORT */
