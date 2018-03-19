/*
 *
 * Clock initialization for OMAP4
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * Based on previous work by:
 *	Santosh Shilimkar <santosh.shilimkar@ti.com>
 *	Rajendra Nayak <rnayak@ti.com>
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
#include <common.h>
#include <asm/omap_common.h>
#include <asm/gpio.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/utils.h>
#include <asm/omap_gpio.h>

/* James Wu: Console is not available until gd->have_console is 1 */
#if 0
#ifndef CONFIG_SPL_BUILD
/*
 * printing to console doesn't work unless
 * this code is executed from SPL
 */
#define printf(fmt, args...)
#define puts(s)
#endif
#endif

struct omap4_prcm_regs *const prcm = (struct omap4_prcm_regs *)0x4A004100;

#if 0

const u32 sys_clk_array[8] = {
	12000000,	       /* 12 MHz */
	13000000,	       /* 13 MHz */
	16800000,	       /* 16.8 MHz */
	19200000,	       /* 19.2 MHz */
	26000000,	       /* 26 MHz */
	27000000,	       /* 27 MHz */
	38400000,	       /* 38.4 MHz */
};

/*
 * The M & N values in the following tables are created using the
 * following tool:
 * tools/omap/clocks_get_m_n.c
 * Please use this tool for creating the table for any new frequency.
 */

/* dpll locked at 1400 MHz MPU clk at 700 MHz(OPP100) - DCC OFF */
static const struct dpll_params mpu_dpll_params_1400mhz[NUM_SYS_CLKS] = {
	{175, 2, 1, -1, -1, -1, -1, -1},	/* 12 MHz   */
	{700, 12, 1, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{125, 2, 1, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{401, 10, 1, -1, -1, -1, -1, -1},	/* 19.2 MHz */
	{350, 12, 1, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{700, 26, 1, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{638, 34, 1, -1, -1, -1, -1, -1}	/* 38.4 MHz */
};

/* dpll locked at 1584 MHz - MPU clk at 792 MHz(OPP Turbo 4430) */
static const struct dpll_params mpu_dpll_params_1600mhz[NUM_SYS_CLKS] = {
	{200, 2, 1, -1, -1, -1, -1, -1},	/* 12 MHz   */
	{800, 12, 1, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{619, 12, 1, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{125, 2, 1, -1, -1, -1, -1, -1},	/* 19.2 MHz */
	{400, 12, 1, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{800, 26, 1, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{125, 5, 1, -1, -1, -1, -1, -1}		/* 38.4 MHz */
};

/* dpll locked at 1200 MHz - MPU clk at 600 MHz */
static const struct dpll_params mpu_dpll_params_1200mhz[NUM_SYS_CLKS] = {
	{50, 0, 1, -1, -1, -1, -1, -1},		/* 12 MHz   */
	{600, 12, 1, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{250, 6, 1, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{125, 3, 1, -1, -1, -1, -1, -1},	/* 19.2 MHz */
	{300, 12, 1, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{200, 8, 1, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{125, 7, 1, -1, -1, -1, -1, -1}		/* 38.4 MHz */
};

static const struct dpll_params core_dpll_params_1600mhz[NUM_SYS_CLKS] = {
	{200, 2, 1, 5, 8, 4, 6, 5},	/* 12 MHz   */
	{800, 12, 1, 5, 8, 4, 6, 5},	/* 13 MHz   */
	{619, 12, 1, 5, 8, 4, 6, 5},	/* 16.8 MHz */
	{125, 2, 1, 5, 8, 4, 6, 5},	/* 19.2 MHz */
	{400, 12, 1, 5, 8, 4, 6, 5},	/* 26 MHz   */
	{800, 26, 1, 5, 8, 4, 6, 5},	/* 27 MHz   */
	{125, 5, 1, 5, 8, 4, 6, 5}	/* 38.4 MHz */
};

static const struct dpll_params core_dpll_params_es1_1524mhz[NUM_SYS_CLKS] = {
	{127, 1, 1, 5, 8, 4, 6, 5},	/* 12 MHz   */
	{762, 12, 1, 5, 8, 4, 6, 5},	/* 13 MHz   */
	{635, 13, 1, 5, 8, 4, 6, 5},	/* 16.8 MHz */
	{635, 15, 1, 5, 8, 4, 6, 5},	/* 19.2 MHz */
	{381, 12, 1, 5, 8, 4, 6, 5},	/* 26 MHz   */
	{254, 8, 1, 5, 8, 4, 6, 5},	/* 27 MHz   */
	{496, 24, 1, 5, 8, 4, 6, 5}	/* 38.4 MHz */
};

static const struct dpll_params
		core_dpll_params_es2_1600mhz_ddr200mhz[NUM_SYS_CLKS] = {
	{200, 2, 2, 5, 8, 4, 6, 5},	/* 12 MHz   */
	{800, 12, 2, 5, 8, 4, 6, 5},	/* 13 MHz   */
	{619, 12, 2, 5, 8, 4, 6, 5},	/* 16.8 MHz */
	{125, 2, 2, 5, 8, 4, 6, 5},	/* 19.2 MHz */
	{400, 12, 2, 5, 8, 4, 6, 5},	/* 26 MHz   */
	{800, 26, 2, 5, 8, 4, 6, 5},	/* 27 MHz   */
	{125, 5, 2, 5, 8, 4, 6, 5}	/* 38.4 MHz */
};

static const struct dpll_params per_dpll_params_1536mhz[NUM_SYS_CLKS] = {
	{64, 0, 8, 6, 12, 9, 4, 5},	/* 12 MHz   */
	{768, 12, 8, 6, 12, 9, 4, 5},	/* 13 MHz   */
	{320, 6, 8, 6, 12, 9, 4, 5},	/* 16.8 MHz */
	{40, 0, 8, 6, 12, 9, 4, 5},	/* 19.2 MHz */
	{384, 12, 8, 6, 12, 9, 4, 5},	/* 26 MHz   */
	{256, 8, 8, 6, 12, 9, 4, 5},	/* 27 MHz   */
	{20, 0, 8, 6, 12, 9, 4, 5}	/* 38.4 MHz */
};

static const struct dpll_params iva_dpll_params_1862mhz[NUM_SYS_CLKS] = {
	{931, 11, -1, -1, 4, 7, -1, -1},	/* 12 MHz   */
	{931, 12, -1, -1, 4, 7, -1, -1},	/* 13 MHz   */
	{665, 11, -1, -1, 4, 7, -1, -1},	/* 16.8 MHz */
	{727, 14, -1, -1, 4, 7, -1, -1},	/* 19.2 MHz */
	{931, 25, -1, -1, 4, 7, -1, -1},	/* 26 MHz   */
	{931, 26, -1, -1, 4, 7, -1, -1},	/* 27 MHz   */
	{291, 11, -1, -1, 4, 7, -1, -1}		/* 38.4 MHz */
};

/* ABE M & N values with sys_clk as source */
static const struct dpll_params
		abe_dpll_params_sysclk_196608khz[NUM_SYS_CLKS] = {
	{49, 5, 1, 1, -1, -1, -1, -1},	/* 12 MHz   */
	{68, 8, 1, 1, -1, -1, -1, -1},	/* 13 MHz   */
	{35, 5, 1, 1, -1, -1, -1, -1},	/* 16.8 MHz */
	{46, 8, 1, 1, -1, -1, -1, -1},	/* 19.2 MHz */
	{34, 8, 1, 1, -1, -1, -1, -1},	/* 26 MHz   */
	{29, 7, 1, 1, -1, -1, -1, -1},	/* 27 MHz   */
	{64, 24, 1, 1, -1, -1, -1, -1}	/* 38.4 MHz */
};

/* ABE M & N values with 32K clock as source */
static const struct dpll_params abe_dpll_params_32k_196608khz = {
	750, 0, 1, 1, -1, -1, -1, -1
};

static const struct dpll_params usb_dpll_params_1920mhz[NUM_SYS_CLKS] = {
	{80, 0, 2, -1, -1, -1, -1, -1},		/* 12 MHz   */
	{960, 12, 2, -1, -1, -1, -1, -1},	/* 13 MHz   */
	{400, 6, 2, -1, -1, -1, -1, -1},	/* 16.8 MHz */
	{50, 0, 2, -1, -1, -1, -1, -1},		/* 19.2 MHz */
	{480, 12, 2, -1, -1, -1, -1, -1},	/* 26 MHz   */
	{320, 8, 2, -1, -1, -1, -1, -1},	/* 27 MHz   */
	{25, 0, 2, -1, -1, -1, -1, -1}		/* 38.4 MHz */
};

#else

/* James Wu: We support 38.4 MHz SYS_CLK only */

/** Refer to Kernel OPP configurations.
 * OMAP4430: MPU 300MHz(OPP50), 600MHz(OPP100), 800MHz(OPPTURBO), 1008MHz(OPPNITRO)
 * OMAP4460: MPU 350MHz(OPP50), 700MHz(OPP100), 920MHz(OPPTURBO)
 * OMAP4470: MPU 396.8MHz(OPP50), 800MHz(OPP100), 1100MHz(OPPTURBO, DCC on)
 */
#if defined(CONFIG_OMAP44XX_MPU_OPP50)
/* dpll locked at 793.6 MHz MPU clk at 396.8 MHz(OPP50 4470) */
static const struct dpll_params mpu_dpll_params_793p6mhz = {
	31, 2, 1, -1, -1, -1, -1, -1
};

/* dpll locked at 700 MHz MPU clk at 350 MHz(OPP50 4460) */
static const struct dpll_params mpu_dpll_params_700mhz = {
	875, 95, 1, -1, -1, -1, -1, -1
};

/* dpll locked at 600 MHz - MPU clk at 300 MHz(OPP50 4430) */
static const struct dpll_params mpu_dpll_params_600mhz = {
	125, 15, 1, -1, -1, -1, -1, -1
};
#elif defined(CONFIG_OMAP44XX_MPU_OPPTURBO)
/* dpll locked at 2200 MHz MPU clk at 1100 MHz(OPPTURBO 4470) - DCC ON */
static const struct dpll_params mpu_dpll_params_2200mhz = {
	1375, 47, 1, -1, -1, -1, -1, -1	/* 38.4 MHz */
};

/* dpll locked at 1840 MHz MPU clk at 920 MHz(OPPTURBO 4460) - DCC OFF */
static const struct dpll_params mpu_dpll_params_1840mhz = {
	575, 23, 1, -1, -1, -1, -1, -1
};

#ifdef CONFIG_OMAP4430_MPU_OPPNITRO
/* dpll locked at 2016 MHz MPU clk at 1008 MHz(OPPNITRO 4430) - DCC OFF */
static const struct dpll_params mpu_dpll_params_2016mhz = {
	105, 3, 1, -1, -1, -1, -1, -1	/* 38.4 MHz */
};
#else
/* dpll locked at 1600 MHz MPU clk at 800 MHz(OPPTURBO 4430) - DCC OFF */
static const struct dpll_params mpu_dpll_params_1600mhz = {
	125, 5, 1, -1, -1, -1, -1, -1	/* 38.4 MHz */
};
#endif /* CONFIG_OMAP4430_MPU_OPPNITRO */
#else
/* dpll locked at 1600 MHz MPU clk at 800 MHz(OPP100 4470) - DCC OFF */
static const struct dpll_params mpu_dpll_params_1600mhz = {
	125, 5, 1, -1, -1, -1, -1, -1	/* 38.4 MHz */
};

/* dpll locked at 1400 MHz MPU clk at 700 MHz(OPP100 4460) - DCC OFF */
static const struct dpll_params mpu_dpll_params_1400mhz = {
	638, 34, 1, -1, -1, -1, -1, -1
};

/* dpll locked at 1200 MHz - MPU clk at 600 MHz(OPP100 4430) */
static const struct dpll_params mpu_dpll_params_1200mhz = {
	125, 7, 1, -1, -1, -1, -1, -1
};
#endif /* CONFIG_OMAP44XX_MPU_OPP50 */

#ifdef CONFIG_OMAP44XX_CORE_233MHZ
static const struct dpll_params core_dpll_params_ddr466mhz = {
	/* CORE OPP119 1866666666Hz (DDR@466MHz) */
#if 0
	875, 35, 1/*M2*/, 6/*M3:SCRM 311.1MHz*/, 8/*M4*/, 4/*M5*/, 6/*M6:EMU 311.1MHz*/, 5/*M7:BB2D 373.3MHz*/
#else
	875, 35, 1/*M2*/, 6/*M3:SCRM 311.1MHz*/, 8/*M4*/, 4/*M5*/, 9/*M6:EMU 207.4MHz*/, 10/*M7:BB2D 186.6MHz*/
#endif
};
#endif /* CONFIG_OMAP44XX_CORE_233MHZ */

static const struct dpll_params core_dpll_params_1600mhz = {
	/* CORE OPP100 (DDR@400MHz) */
#if 0
	125, 5, 1/*M2*/, 5/*M3:SCRM 320MHz*/, 8/*M4*/, 4/*M5*/, 6/*M6:EMU 266.7MHz*/, 6/*M7:SGX_FCLK/BB2D 266.7MHz*/
#else
	125, 5, 1/*M2*/, 5/*M3:SCRM 320MHz*/, 8/*M4*/, 4/*M5*/, 8/*M6:EMU 200MHz*/, 12/*M7:SGX_FCLK/BB2D 133.3MHz*/
#endif
};

static const struct dpll_params per_dpll_params_1536mhz = {
#if 0
	20, 0, 8/*M2:96M*/, 6/*M3:SCRM*/, 12/*M4:128M*/, 9/*M5:DSS_FCLK*/, 4/*M6:MPU_M3/BB2D 384MHz*/, 5/*M7:SGX_FCLK 307.2MHz*/
#else
	20, 0, 8/*M2:96M*/, 6/*M3:SCRM*/, 12/*M4:128M*/, 9/*M5:DSS_FCLK*/, 8/*M6:MPU_M3/BB2D 192MHz*/, 10/*M7:SGX_FCLK 153.6MHz*/
#endif
};

static const struct dpll_params iva_dpll_params_1862mhz = {
#if defined(CONFIG_OMAP44XX_MPU_OPPTURBO)
	/* James Wu: OMAP44XX IVA at OPP100 (OPP Dependencies with MPU OPPTRUBO) */
#if 0
	291, 11, -1, -1, 4/*M4:DSP*/, 7/*M5:IVAHD*/, -1, -1
#else
	97, 3, -1, -1, 4/*M4:DSP*/, 7/*M5:IVAHD*/, -1, -1
#endif
#else
	/* James Wu: (1862.4MHz) OMAP44XX IVA at OPP50 */
	97, 3, -1, -1, 8/*M4:DSP*/, 14/*M5:IVAHD*/, -1, -1
#endif /* CONFIG_OMAP44XX_MPU_OPPTURBO */
};

#ifdef CONFIG_SYS_OMAP_ABE_SYSCK
/* ABE M & N values with sys_clk as source */
static const struct dpll_params abe_dpll_params_sysclk_196608khz = {
	64, 24, 1, 1, -1, -1, -1, -1
};
#else
/* ABE M & N values with 32K clock as source */
static const struct dpll_params abe_dpll_params_32k_196608khz = {
	750, 0, 1, 1, -1, -1, -1, -1
};
#endif /* CONFIG_SYS_OMAP_ABE_SYSCK */

static const struct dpll_params usb_dpll_params_1920mhz = {
	25, 0, 2, -1, -1, -1, -1, -1
};

#endif

void setup_post_dividers(u32 *const base, const struct dpll_params *params)
{
	struct dpll_regs *const dpll_regs = (struct dpll_regs *)base;

	/* Setup post-dividers */
	if (params->m2 >= 0)
		writel(params->m2, &dpll_regs->cm_div_m2_dpll);
	if (params->m3 >= 0)
		writel(params->m3, &dpll_regs->cm_div_m3_dpll);
	if (params->m4 >= 0)
		writel(params->m4, &dpll_regs->cm_div_m4_dpll);
	if (params->m5 >= 0)
		writel(params->m5, &dpll_regs->cm_div_m5_dpll);
	if (params->m6 >= 0)
		writel(params->m6, &dpll_regs->cm_div_m6_dpll);
	if (params->m7 >= 0)
		writel(params->m7, &dpll_regs->cm_div_m7_dpll);
}

/*
 * Lock MPU dpll
 *
 * Resulting MPU frequencies:
 * 4430 ES1.0	: 600 MHz
 * 4430 ES2.x	: 792 MHz (OPP Turbo)
 * 4460		: 920 MHz (OPP Turbo) - DCC disabled
 */
const struct dpll_params *get_mpu_dpll_params(void)
{
#if 0
	u32 omap_rev, sysclk_ind;

	omap_rev = omap_revision();
	sysclk_ind = get_sys_clk_index();

	if (omap_rev == OMAP4430_ES1_0)
		return &mpu_dpll_params_1200mhz[sysclk_ind];
	else if (omap_rev < OMAP4460_ES1_0)
		return &mpu_dpll_params_1600mhz[sysclk_ind];
	else
		return &mpu_dpll_params_1400mhz[sysclk_ind];
#else
	u32 omap_rev = omap_revision();

/**
 * OMAP4430: MPU 300MHz(OPP50), 600MHz(OPP100), 800MHz(OPPTURBO), 1008MHz(OPPNITRO)
 * OMAP4460: MPU 350MHz(OPP50), 700MHz(OPP100), 920MHz(OPPTURBO)
 * OMAP4470: MPU 400MHz(OPP50), 800MHz(OPP100), 1100MHz(OPPTURBO, DCC on)
 */
#if defined(CONFIG_OMAP44XX_MPU_OPP50)
	/* MPU OPP50 */
	if (omap_rev < OMAP4460_ES1_0)
		/* OMAP4430 */
		return &mpu_dpll_params_600mhz;
	else if (omap_rev < OMAP4470_ES1_0)
		/* OMAP4460 */
		return &mpu_dpll_params_700mhz;
	else
		/* OMAP4470 */
		return &mpu_dpll_params_793p6mhz;
#elif defined(CONFIG_OMAP44XX_MPU_OPPTURBO)
	if (omap_rev < OMAP4460_ES1_0)
		/* OMAP4430 */
#ifdef CONFIG_OMAP4430_MPU_OPPNITRO
		return &mpu_dpll_params_2016mhz;
#else
		return &mpu_dpll_params_1600mhz;
#endif /* CONFIG_OMAP4430_MPU_OPPNITRO */
	else if (omap_rev < OMAP4470_ES1_0)
		/* OMAP4460 */
		return &mpu_dpll_params_1840mhz;
	else
		/* OMAP4470 */
		return &mpu_dpll_params_2200mhz;
#else
	/* MPU OPP100 */
	if (omap_rev < OMAP4460_ES1_0)
		/* OMAP4430 */
		return &mpu_dpll_params_1200mhz;
	else if (omap_rev < OMAP4470_ES1_0)
		/* OMAP4460 */
		return &mpu_dpll_params_1400mhz;
	else
		/* OMAP4470 */
		return &mpu_dpll_params_1600mhz;
#endif /* CONFIG_OMAP44XX_MPU_OPP50 */

#endif
}

const struct dpll_params *get_core_dpll_params(void)
{
#if 0
	u32 sysclk_ind = get_sys_clk_index();

	switch (omap_revision()) {
	case OMAP4430_ES1_0:
		return &core_dpll_params_es1_1524mhz[sysclk_ind];
	case OMAP4430_ES2_0:
	case OMAP4430_SILICON_ID_INVALID:
		 /* safest */
		return &core_dpll_params_es2_1600mhz_ddr200mhz[sysclk_ind];
	default:
		return &core_dpll_params_1600mhz[sysclk_ind];
	}
#else

#ifdef CONFIG_OMAP44XX_CORE_233MHZ
	u32 omap_rev = omap_revision();

	if (omap_rev >= OMAP4470_ES1_0)
		return &core_dpll_params_ddr466mhz;
	else
#endif /* CONFIG_OMAP44XX_CORE_233MHZ */
		return &core_dpll_params_1600mhz;

#endif
}


const struct dpll_params *get_per_dpll_params(void)
{
#if 0
	u32 sysclk_ind = get_sys_clk_index();
	return &per_dpll_params_1536mhz[sysclk_ind];
#else
	return &per_dpll_params_1536mhz;
#endif
}

const struct dpll_params *get_iva_dpll_params(void)
{
#if 0
	u32 sysclk_ind = get_sys_clk_index();
	return &iva_dpll_params_1862mhz[sysclk_ind];
#else
	return &iva_dpll_params_1862mhz;
#endif
}

const struct dpll_params *get_usb_dpll_params(void)
{
#if 0
	u32 sysclk_ind = get_sys_clk_index();
	return &usb_dpll_params_1920mhz[sysclk_ind];
#else
	return &usb_dpll_params_1920mhz;
#endif
}

const struct dpll_params *get_abe_dpll_params(void)
{
#ifdef CONFIG_SYS_OMAP_ABE_SYSCK
#if 0
	u32 sysclk_ind = get_sys_clk_index();
	return &abe_dpll_params_sysclk_196608khz[sysclk_ind];
#else
	return &abe_dpll_params_sysclk_196608khz;
#endif
#else
	return &abe_dpll_params_32k_196608khz;
#endif /* CONFIG_SYS_OMAP_ABE_SYSCK */
}

/*
 * Setup the voltages for vdd_mpu, vdd_core, and vdd_iva
 * We set the maximum voltages allowed here because Smart-Reflex is not
 * enabled in bootloader. Voltage initialization in the kernel will set
 * these to the nominal values after enabling Smart-Reflex
 */
#ifdef CONFIG_TWL6032_POWER
static void do_scale_vcores(u32 volt_mpu, u32 volt_iva, u32 volt_core)
{
	omap_vc_init(PRM_VC_I2C_CHANNEL_FREQ_KHZ);

	/**
	 * http://dev.omapzoom.org/?p=bootloader/x-loader.git;a=commit;h=9e1e8bf563c72844153c7f166bad39011ea4ad8d
	 *
	 * OMAP4 requires that parent domains scale ahead of dependent domains.
	 * This is due to the restrictions in timing closure. To ensure
	 * a consistent behavior accross all OMAP4 SoC, ensure that
	 * vdd_core scale first, then vdd_mpu and finally vdd_iva.
	 */

	/* TWL6032 */
	/* VCORE 3 - supplies vdd_core */
	do_scale_vcore(SMPS_6032_REG_ADDR_VCORE3, volt_core);
	/* VCORE 1 - supplies vdd_mpu */
	do_scale_vcore(SMPS_6032_REG_ADDR_VCORE1, volt_mpu);
	/* VCORE 2 - supplies vdd_iva */
	do_scale_vcore(SMPS_6032_REG_ADDR_VCORE2, volt_iva);
}
#else /* !CONFIG_TWL6032_POWER */
static void do_scale_vcores(u32 volt_mpu, u32 volt_iva, u32 volt_core)
{
	u32 omap_rev;

	omap_vc_init(PRM_VC_I2C_CHANNEL_FREQ_KHZ);

	omap_rev = omap_revision();

	/*
	 * Scale Voltage rails:
	 * 1. VDD_CORE
	 * 3. VDD_MPU
	 * 3. VDD_IVA
	 */
	if (omap_rev < OMAP4460_ES1_0) {
		/*
		 * OMAP4430:
		 * VDD_CORE = TWL6030 VCORE3
		 * VDD_MPU = TWL6030 VCORE1
		 * VDD_IVA = TWL6030 VCORE2
		 */
		/* TWL6030 VCORE 3 - supplies vdd_core */
		do_scale_vcore(SMPS_REG_ADDR_VCORE3, volt_core);
		/* TWL6030 VCORE 1 - supplies vdd_mpu */
		do_scale_vcore(SMPS_REG_ADDR_VCORE1, volt_mpu);
		/* TWL6030 VCORE 2 - supplies vdd_iva */
		do_scale_vcore(SMPS_REG_ADDR_VCORE2, volt_iva);
	} else if (omap_rev < OMAP4470_ES1_0) {
		/*
		 * OMAP4460:
		 * VDD_CORE = TWL6030 VCORE1
		 * VDD_MPU = TPS62361
		 * VDD_IVA = TWL6030 VCORE2
		 */

		/* TWL6030 VCORE 1 - supplies vdd_core */
		do_scale_vcore(SMPS_REG_ADDR_VCORE1, volt_core);
		/* TPS62361 - supplies vdd_mpu on 4460 */
		do_scale_tps62361(TPS62361_REG_ADDR_SET1, volt_mpu);
		/* TWL6030 VCORE 2 - supplies vdd_iva */
		do_scale_vcore(SMPS_REG_ADDR_VCORE2, volt_iva);
		/* TWL6030 VCORE 3 - not connected */
	} else {
		puts("OMAP4470 uses TWL6032\n");
		hang();
	}
}
#endif /* CONFIG_TWL6032_POWER */

void scale_vcores(void)
{
#if 0
	u32 volt, omap_rev;

	omap_vc_init(PRM_VC_I2C_CHANNEL_FREQ_KHZ);

	omap_rev = omap_revision();

	/*
	 * Scale Voltage rails:
	 * 1. VDD_CORE
	 * 3. VDD_MPU
	 * 3. VDD_IVA
	 */
	if (omap_rev < OMAP4460_ES1_0) {
		/*
		 * OMAP4430:
		 * VDD_CORE = TWL6030 VCORE3
		 * VDD_MPU = TWL6030 VCORE1
		 * VDD_IVA = TWL6030 VCORE2
		 */
		volt = 1200;
		do_scale_vcore(SMPS_REG_ADDR_VCORE3, volt);

		/*
		 * note on VDD_MPU:
		 * Setting a high voltage for Nitro mode as smart reflex is not
		 * enabled. We use the maximum possible value in the AVS range
		 * because the next higher voltage in the discrete range
		 * (code >= 0b111010) is way too high.
		 */
		volt = 1325;
		do_scale_vcore(SMPS_REG_ADDR_VCORE1, volt);
		volt = 1200;
		do_scale_vcore(SMPS_REG_ADDR_VCORE2, volt);

	} else {
		/*
		 * OMAP4460:
		 * VDD_CORE = TWL6030 VCORE1
		 * VDD_MPU = TPS62361
		 * VDD_IVA = TWL6030 VCORE2
		 */
		volt = 1200;
		do_scale_vcore(SMPS_REG_ADDR_VCORE1, volt);
		/* TPS62361 */
		volt = 1203;
		do_scale_tps62361(TPS62361_VSEL0_GPIO,
				  TPS62361_REG_ADDR_SET1, volt);
		/* VCORE 2 - supplies vdd_iva */
		volt = 1200;
		do_scale_vcore(SMPS_REG_ADDR_VCORE2, volt);
	}
#else
	u32 omap_rev = omap_revision();

	/* James Wu: follow values in OMAP4430/4460/4470 DM Operating Condition Addendum.
	 * OMAP4430 v0.9:
	 *	MPU:
	 *		OPP50:		1.025
	 *		OPP100:		1.200
	 *		OPPTB:		1.325
	 *		OPPNT:		1.388
	 *	IVA:
	 *		OPP50:		0.950
	 *		OPP100:		1.114
	 *	CORE:
	 *		OPP100:		1.127
	 *
	 * OMAP4460 v0.6:
	 *	MPU:
	 *		OPP50:		1.025
	 *		OPP100:		1.203
	 *		OPPTB:		1.317
	 *	IVA:
	 *		OPP50:		0.950
	 *		OPP100:		1.114
	 *	CORE:
	 *		OPP100:		1.127
	 *
	 * OMAP4470 v0.7:
	 *	MPU:
	 *		OPP50:		1.037
	 *		OPP100:		1.200
	 *		OPPTB:		1.312
	 *	IVA:
	 *		OPP50:		0.962
	 *		OPP100:		1.137
	 *	CORE:
	 *		OPP100:		1.126
	 *		OPP119:		1.190
	 */
#if defined(CONFIG_OMAP44XX_MPU_OPP50)

	/* MPU OPP50 */
	if (omap_rev < OMAP4470_ES1_0)
		/* OMAP4430/OMAP4460: MPU OPP50 ; IVA OPP50 ; CORE OPP100 */
		do_scale_vcores(1025, 950, 1127);
	else
#ifdef CONFIG_OMAP44XX_CORE_233MHZ
		/* OMAP4470: MPU OPP50 ; IVA OPP50 ; CORE OPP119 */
		do_scale_vcores(1037, 962, 1190);
#else
		/* OMAP4470: MPU OPP50 ; IVA OPP50 ; CORE OPP100 */
		do_scale_vcores(1037, 962, 1126);
#endif /* CONFIG_OMAP44XX_CORE_233MHZ */

#elif defined(CONFIG_OMAP44XX_MPU_OPPTURBO)

	/* MPU OPPTURBO */
	if (omap_rev < OMAP4460_ES1_0)
#ifdef CONFIG_OMAP4430_MPU_OPPNITRO
		/* OMAP4430: MPU OPPNITRO; IVA OPP100 ; CORE OPP100 */
		do_scale_vcores(1388, 1114, 1127);
#else
		/* OMAP4430: MPU OPPTURBO ; IVA OPP100 ; CORE OPP100 */
		do_scale_vcores(1325, 1114, 1127);
#endif /* CONFIG_OMAP4430_MPU_OPPNITRO */
	else if (omap_rev < OMAP4470_ES1_0)
		/* OMAP4460: MPU OPPTURBO ; IVA OPP100 ; CORE OPP100 */
		do_scale_vcores(1317, 1114, 1127);
	else
#ifdef CONFIG_OMAP44XX_CORE_233MHZ
		/* OMAP4470: MPU OPPTURBO ; IVA OPP100 ; CORE OPP119 */
		do_scale_vcores(1312, 1137, 1190);
#else
		/* OMAP4470: MPU OPPTURBO ; IVA OPP100 ; CORE OPP100 */
		do_scale_vcores(1312, 1137, 1126);
#endif /* CONFIG_OMAP44XX_CORE_233MHZ */

#else

	/* MPU OPP100 */
	if (omap_rev < OMAP4460_ES1_0)
		/* OMAP4430: MPU OPP100 ; IVA OPP50 ; CORE OPP100 */
		do_scale_vcores(1200, 950, 1127);
	else if (omap_rev < OMAP4470_ES1_0)
		/* OMAP4460: MPU OPP100 ; IVA OPP50 ; CORE OPP100 */
		do_scale_vcores(1203, 950, 1127);
	else
#ifdef CONFIG_OMAP44XX_CORE_233MHZ
		/* OMAP4470: MPU OPP100 ; IVA OPP50 ; CORE OPP119 */
		do_scale_vcores(1200, 962, 1190);
#else
		/* OMAP4470: MPU OPP100 ; IVA OPP50 ; CORE OPP100 */
		do_scale_vcores(1200, 962, 1126);
#endif /* CONFIG_OMAP44XX_CORE_233MHZ */

#endif /* CONFIG_OMAP44XX_MPU_OPP50 */

	/* James Wu:
	 * Delay to wait for the SMPS voltage ready.
	 * SMPS uV range / SMPS slew rate(4000uV).
	 *
	 * sdelay(2000) := 1 us (Enough loops assuming a maximum of 2GHz)
	 */
	sdelay(2000 * 132/*us*/);
#endif
}

u32 get_offset_code(u32 offset)
{
/*
 * TWL6030 & TWL6032:
 * The voltage range in minimum value: 700-1400mV range (12.5mV step)
 * The voltage range in typical value: 709-1418mV range (12.66mV step)
 */
#ifdef CONFIG_TWL6032_POWER
	u32 volt_mv, offset_code, step = PHOENIX_SMPS_STEP_VOLT_STD_MODE_UV;

	volt_mv = offset / 1000;
	offset -= PHOENIX_SMPS_BASE_VOLT_STD_MODE_WITH_OFFSET_UV;

	offset_code = (offset + step - 1) / step;
	/* The code starts at 1 not 0 */
	offset_code++;

	if (offset_code > 0x39) {
		if (volt_mv <= 1367)		/* 1367.4mV */
			offset_code = 0x3A;
		else if (volt_mv <= 1519)	/* 1519.3mv */
			offset_code = 0x3B;
		else if (volt_mv <= 1823)	/* 1823.1mv */
			offset_code = 0x3C;
		else if (volt_mv <= 1924)	/* 1924.4mv */
			offset_code = 0x3D;
		else						/* 2127.0mv */
			offset_code = 0x3E;
	}

	return offset_code;
#else /* !CONFIG_TWL6032_POWER */
	u32 offset_code, step = 12660; /* 12.66 mV represented in uV */

	if (omap_revision() == OMAP4430_ES1_0)
		offset -= PHOENIX_SMPS_BASE_VOLT_STD_MODE_UV;
	else
		offset -= PHOENIX_SMPS_BASE_VOLT_STD_MODE_WITH_OFFSET_UV;

	offset_code = (offset + step - 1) / step;

	/* The code starts at 1 not 0 */
	return ++offset_code;
#endif /* CONFIG_TWL6032_POWER */
}

/*
 * Enable essential clock domains, modules and
 * do some additional special settings needed
 */
void enable_basic_clocks(void)
{
	u32 *const clk_domains_essential[] = {
		&prcm->cm_l4per_clkstctrl,
		&prcm->cm_l3init_clkstctrl,
		&prcm->cm_memif_clkstctrl,
		&prcm->cm_l4cfg_clkstctrl,
		0
	};

	u32 *const clk_modules_hw_auto_essential[] = {
		&prcm->cm_l3_2_gpmc_clkctrl,
		&prcm->cm_memif_emif_1_clkctrl,
		&prcm->cm_memif_emif_2_clkctrl,
		&prcm->cm_l4cfg_l4_cfg_clkctrl,
#if 0
		&prcm->cm_wkup_gpio1_clkctrl,
		&prcm->cm_l4per_gpio2_clkctrl,
		&prcm->cm_l4per_gpio3_clkctrl,
		&prcm->cm_l4per_gpio4_clkctrl,
		&prcm->cm_l4per_gpio5_clkctrl,
		&prcm->cm_l4per_gpio6_clkctrl,
#endif
		0
	};

	u32 *const clk_modules_explicit_en_essential[] = {
#if 0
		&prcm->cm_wkup_gptimer1_clkctrl,
#endif
		&prcm->cm_l3init_hsmmc1_clkctrl,
		&prcm->cm_l3init_hsmmc2_clkctrl,
#if 0
		&prcm->cm_l4per_gptimer2_clkctrl,
#endif
#if 0
		&prcm->cm_wkup_wdtimer2_clkctrl,
#endif
#if (CONFIG_CONS_INDEX == 1)
		&prcm->cm_l4per_uart1_clkctrl,
#elif (CONFIG_CONS_INDEX == 2)
		&prcm->cm_l4per_uart2_clkctrl,
#elif (CONFIG_CONS_INDEX == 3)
		&prcm->cm_l4per_uart3_clkctrl,
#elif (CONFIG_CONS_INDEX == 4)
		&prcm->cm_l4per_uart4_clkctrl,
#endif
		0
	};

#if 0
	/* Enable optional additional functional clock for GPIO4 */
	setbits_le32(&prcm->cm_l4per_gpio4_clkctrl,
			GPIO4_CLKCTRL_OPTFCLKEN_MASK);
#endif

	/* Enable 96 MHz clock for MMC1 & MMC2 */
	setbits_le32(&prcm->cm_l3init_hsmmc1_clkctrl,
			HSMMC_CLKCTRL_CLKSEL_MASK);
	setbits_le32(&prcm->cm_l3init_hsmmc2_clkctrl,
			HSMMC_CLKCTRL_CLKSEL_MASK);

#if 0
	/* Select 32KHz clock as the source of GPTIMER1 */
	setbits_le32(&prcm->cm_wkup_gptimer1_clkctrl,
			GPTIMER1_CLKCTRL_CLKSEL_MASK);
#endif

#if 0
	/* Enable optional 48M functional clock for USB  PHY */
	setbits_le32(&prcm->cm_l3init_usbphy_clkctrl,
			USBPHY_CLKCTRL_OPTFCLKEN_PHY_48M_MASK);
#endif

	do_enable_clocks(clk_domains_essential,
			 clk_modules_hw_auto_essential,
			 clk_modules_explicit_en_essential,
			 1);

	/* Refer to Kernel, CD_L3_INIT (SW_WKUP) (clkdm_clk_enable/clkdm_wakeup) */
	omap_clk_domain(&prcm->cm_l3init_clkstctrl, CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
}

#if 0
void enable_basic_uboot_clocks(void)
{
	u32 *const clk_domains_essential[] = {
		0
	};

	u32 *const clk_modules_hw_auto_essential[] = {
		&prcm->cm_l3init_hsusbotg_clkctrl,
		&prcm->cm_l3init_usbphy_clkctrl,
		&prcm->cm_l3init_usbphy_clkctrl,
		&prcm->cm_clksel_usb_60mhz,
		&prcm->cm_l3init_hsusbtll_clkctrl,
		0
	};

	u32 *const clk_modules_explicit_en_essential[] = {
		&prcm->cm_l4per_mcspi1_clkctrl,
		&prcm->cm_l4per_i2c1_clkctrl,
		&prcm->cm_l4per_i2c2_clkctrl,
		&prcm->cm_l4per_i2c3_clkctrl,
		&prcm->cm_l4per_i2c4_clkctrl,
		&prcm->cm_l3init_hsusbhost_clkctrl,
		0
	};

	do_enable_clocks(clk_domains_essential,
			 clk_modules_hw_auto_essential,
			 clk_modules_explicit_en_essential,
			 1);
}
#endif

/* James Wu: Don't turn on non-essential clocks */
#if 0
/*
 * Enable non-essential clock domains, modules and
 * do some additional special settings needed
 */
void enable_non_essential_clocks(void)
{
	u32 *const clk_domains_non_essential[] = {
		&prcm->cm_mpu_m3_clkstctrl,
		&prcm->cm_ivahd_clkstctrl,
		&prcm->cm_dsp_clkstctrl,
		&prcm->cm_dss_clkstctrl,
		&prcm->cm_sgx_clkstctrl,
		&prcm->cm1_abe_clkstctrl,
		&prcm->cm_c2c_clkstctrl,
		&prcm->cm_cam_clkstctrl,
		&prcm->cm_dss_clkstctrl,
		&prcm->cm_sdma_clkstctrl,
		0
	};

	u32 *const clk_modules_hw_auto_non_essential[] = {
		&prcm->cm_l3instr_l3_3_clkctrl,
		&prcm->cm_l3instr_l3_instr_clkctrl,
		&prcm->cm_l3instr_intrconn_wp1_clkctrl,
		&prcm->cm_l3init_hsi_clkctrl,
		0
	};

	u32 *const clk_modules_explicit_en_non_essential[] = {
		&prcm->cm1_abe_aess_clkctrl,
		&prcm->cm1_abe_pdm_clkctrl,
		&prcm->cm1_abe_dmic_clkctrl,
		&prcm->cm1_abe_mcasp_clkctrl,
		&prcm->cm1_abe_mcbsp1_clkctrl,
		&prcm->cm1_abe_mcbsp2_clkctrl,
		&prcm->cm1_abe_mcbsp3_clkctrl,
		&prcm->cm1_abe_slimbus_clkctrl,
		&prcm->cm1_abe_timer5_clkctrl,
		&prcm->cm1_abe_timer6_clkctrl,
		&prcm->cm1_abe_timer7_clkctrl,
		&prcm->cm1_abe_timer8_clkctrl,
		&prcm->cm1_abe_wdt3_clkctrl,
		&prcm->cm_l4per_gptimer9_clkctrl,
		&prcm->cm_l4per_gptimer10_clkctrl,
		&prcm->cm_l4per_gptimer11_clkctrl,
		&prcm->cm_l4per_gptimer3_clkctrl,
		&prcm->cm_l4per_gptimer4_clkctrl,
		&prcm->cm_l4per_hdq1w_clkctrl,
		&prcm->cm_l4per_mcbsp4_clkctrl,
		&prcm->cm_l4per_mcspi2_clkctrl,
		&prcm->cm_l4per_mcspi3_clkctrl,
		&prcm->cm_l4per_mcspi4_clkctrl,
		&prcm->cm_l4per_mmcsd3_clkctrl,
		&prcm->cm_l4per_mmcsd4_clkctrl,
		&prcm->cm_l4per_mmcsd5_clkctrl,
#if 0
		&prcm->cm_l4per_uart1_clkctrl,
		&prcm->cm_l4per_uart2_clkctrl,
		&prcm->cm_l4per_uart4_clkctrl,
#endif
		&prcm->cm_wkup_keyboard_clkctrl,
		&prcm->cm_wkup_wdtimer2_clkctrl,
		&prcm->cm_cam_iss_clkctrl,
		&prcm->cm_cam_fdif_clkctrl,
		&prcm->cm_dss_dss_clkctrl,
		&prcm->cm_sgx_sgx_clkctrl,
		0
	};

	/* Enable optional functional clock for ISS */
	setbits_le32(&prcm->cm_cam_iss_clkctrl, ISS_CLKCTRL_OPTFCLKEN_MASK);

	/* Enable all optional functional clocks of DSS */
	setbits_le32(&prcm->cm_dss_dss_clkctrl, DSS_CLKCTRL_OPTFCLKEN_MASK);

	do_enable_clocks(clk_domains_non_essential,
			 clk_modules_hw_auto_non_essential,
			 clk_modules_explicit_en_non_essential,
			 0);

	/* Put camera module in no sleep mode */
	clrsetbits_le32(&prcm->cm_cam_clkstctrl, MODULE_CLKCTRL_MODULEMODE_MASK,
			CD_CLKCTRL_CLKTRCTRL_NO_SLEEP <<
			MODULE_CLKCTRL_MODULEMODE_SHIFT);
}
#endif
