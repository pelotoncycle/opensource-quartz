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

#if 0
static inline u32 __get_sys_clk_index(void)
{
	u32 ind;
	/*
	 * For ES1 the ROM code calibration of sys clock is not reliable
	 * due to hw issue. So, use hard-coded value. If this value is not
	 * correct for any board over-ride this function in board file
	 * From ES2.0 onwards you will get this information from
	 * CM_SYS_CLKSEL
	 */
	if (omap_revision() == OMAP4430_ES1_0)
		ind = OMAP_SYS_CLK_IND_38_4_MHZ;
	else {
		/* SYS_CLKSEL - 1 to match the dpll param array indices */
		ind = (readl(&prcm->cm_sys_clksel) &
			CM_SYS_CLKSEL_SYS_CLKSEL_MASK) - 1;
	}
	return ind;
}

u32 get_sys_clk_index(void)
	__attribute__ ((weak, alias("__get_sys_clk_index")));

u32 get_sys_clk_freq(void)
{
	u8 index = get_sys_clk_index();
	return sys_clk_array[index];
}

#else

u32 get_sys_clk_freq(void)
{
#if defined(CONFIG_OMAP44XX)
	return 38400000; /* 38.4 MHz */
#elif defined(CONFIG_OMAP54XX)
#error "get_sys_clk_freq() is not implemented\n"
#else
#error "get_sys_clk_freq() is not implemented\n"
#endif
}
#endif

static inline void do_allow_idle_dpll(u32 *const base)
{
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	clrsetbits_le32(&dpll_regs->cm_autoidle_dpll,
			DPLL_AUTOIDLE_AUTOMODE_MASK,
			DPLL_AUTOIDLE_LOW_POWER_STOP <<
			DPLL_AUTOIDLE_AUTOMODE_SHIFT);
}

static inline void do_bypass_dpll(u32 *const base)
{
	struct dpll_regs *dpll_regs = (struct dpll_regs *)base;

	/**
	 * For type A DPLLs:
	 *   - MN bypass mode (0x4)
	 *   - Idle Low Power bypass mode (0x5)
	 *   - Idle Fast Relock bypass mode (0x6)
	 *   - DPLL_MPU, DPLL_IVA, DPLL_CORE, DPLL_PER & DPLL_ABE
	 * For type B DPLLs:
	 *   - Low Power Stop mode (0x1)
	 *   - MN bypass mode (0x4)
	 *   - Idle Low Power bypass mode (0x5)
	 *   - DPLL_USB
	 * DPLL_IVA does not support Idle Fast Relock bypass mode (0x6).
	 */
#if 0
	clrsetbits_le32(&dpll_regs->cm_clkmode_dpll,
			CM_CLKMODE_DPLL_DPLL_EN_MASK,
			DPLL_EN_FAST_RELOCK_BYPASS <<
			CM_CLKMODE_DPLL_EN_SHIFT);
#else
	clrsetbits_le32(&dpll_regs->cm_clkmode_dpll,
			CM_CLKMODE_DPLL_DPLL_EN_MASK,
			DPLL_EN_MN_BYPASS <<
			CM_CLKMODE_DPLL_EN_SHIFT);
#endif
}

static inline void wait_for_bypass(u32 *const base)
{
	struct dpll_regs *const dpll_regs = (struct dpll_regs *)base;

	if (!wait_on_value(ST_DPLL_CLK_MASK, 0, &dpll_regs->cm_idlest_dpll,
				LDELAY)) {
#if 0
		printf("Bypassing DPLL failed %p\n", base);
#else
		printf("DPLL bypass err %p\n", base);
#endif
	}
}

static inline void do_lock_dpll(u32 *const base)
{
	struct dpll_regs *const dpll_regs = (struct dpll_regs *)base;

	clrsetbits_le32(&dpll_regs->cm_clkmode_dpll,
		      CM_CLKMODE_DPLL_DPLL_EN_MASK,
		      DPLL_EN_LOCK << CM_CLKMODE_DPLL_EN_SHIFT);
}

static inline void wait_for_lock(u32 *const base)
{
	struct dpll_regs *const dpll_regs = (struct dpll_regs *)base;

	if (!wait_on_value(ST_DPLL_CLK_MASK, ST_DPLL_CLK_MASK,
		&dpll_regs->cm_idlest_dpll, LDELAY)) {
#if 0
		printf("DPLL locking failed for %p\n", base);
#else
		printf("DPLL lock err %p\n", base);
#endif
		hang();
	}
}

inline u32 check_for_lock(u32 *const base)
{
	struct dpll_regs *const dpll_regs = (struct dpll_regs *)base;
	u32 lock = readl(&dpll_regs->cm_idlest_dpll) & ST_DPLL_CLK_MASK;

	return lock;
}

static void do_setup_dpll(u32 *const base, const struct dpll_params *params,
				u8 lock, char *dpll)
{
	u32 temp, M, N;
	struct dpll_regs *const dpll_regs = (struct dpll_regs *)base;

	temp = readl(&dpll_regs->cm_clksel_dpll);

	if (check_for_lock(base)) {
		/*
		 * The Dpll has already been locked by rom code using CH.
		 * Check if M,N are matching with Ideal nominal opp values.
		 * If matches, skip the rest otherwise relock.
		 */
		M = (temp & CM_CLKSEL_DPLL_M_MASK) >> CM_CLKSEL_DPLL_M_SHIFT;
		N = (temp & CM_CLKSEL_DPLL_N_MASK) >> CM_CLKSEL_DPLL_N_SHIFT;
		if ((M != (params->m)) || (N != (params->n))) {
			debug("\n %s Dpll locked, but not for ideal M = %d,"
				"N = %d values, current values are M = %d,"
				"N= %d" , dpll, params->m, params->n,
				M, N);
		} else {
			/* Dpll locked with ideal values for nominal opps. */
			debug("\n %s Dpll already locked with ideal"
						"nominal opp values", dpll);
			goto setup_post_dividers;
		}
	}

	bypass_dpll(base);

	/* Set M & N */
	temp &= ~CM_CLKSEL_DPLL_M_MASK;
	temp |= (params->m << CM_CLKSEL_DPLL_M_SHIFT) & CM_CLKSEL_DPLL_M_MASK;

	temp &= ~CM_CLKSEL_DPLL_N_MASK;
	temp |= (params->n << CM_CLKSEL_DPLL_N_SHIFT) & CM_CLKSEL_DPLL_N_MASK;

	writel(temp, &dpll_regs->cm_clksel_dpll);

	/* Lock */
	if (lock)
		do_lock_dpll(base);

setup_post_dividers:
	setup_post_dividers(base, params);

	/* Wait till the DPLL locks */
	if (lock)
		wait_for_lock(base);
}

u32 omap_ddr_clk(void)
{
	u32 ddr_clk, sys_clk_khz, omap_rev, divider;
	const struct dpll_params *core_dpll_params;

	omap_rev = omap_revision();
	sys_clk_khz = get_sys_clk_freq() / 1000;

	core_dpll_params = get_core_dpll_params();

	debug("sys_clk %d\n", sys_clk_khz * 1000);

	/* Find Core DPLL locked frequency first */
	ddr_clk = sys_clk_khz * 2 * core_dpll_params->m /
			(core_dpll_params->n + 1);

	if (omap_rev < OMAP5430_ES1_0) {
		/*
		 * DDR frequency is PHY_ROOT_CLK/2
		 * PHY_ROOT_CLK = Fdpll/2/M2
		 */
		divider = 4;
	} else {
		/*
		 * DDR frequency is PHY_ROOT_CLK
		 * PHY_ROOT_CLK = Fdpll/2/M2
		 */
		divider = 2;
	}

	ddr_clk = ddr_clk / divider / core_dpll_params->m2;
	ddr_clk *= 1000;	/* convert to Hz */
	debug("ddr_clk %d\n", ddr_clk);

	return ddr_clk;
}

#if defined(CONFIG_OMAP44XX_MPU_OPPTURBO)
static struct dpll_params omap4_mpu_dpll_params_clkoutx2_m3 = {
	-1, -1, -1, -1, -1, -1, -1, -1	/* 38.4 MHz */
};
#endif /* CONFIG_OMAP44XX_MPU_OPPTURBO */

/*
 * Lock MPU dpll
 *
 * Resulting MPU frequencies:
 * 4430 ES1.0	: 600 MHz
 * 4430 ES2.x	: 792 MHz (OPP Turbo)
 * 4460		: 920 MHz (OPP Turbo) - DCC disabled
 */
void configure_mpu_dpll(void)
{
#if 0
	const struct dpll_params *params;
	struct dpll_regs *mpu_dpll_regs;
	u32 omap_rev;
	omap_rev = omap_revision();

	/*
	 * DCC and clock divider settings for 4460.
	 * DCC is required, if more than a certain frequency is required.
	 * For, 4460 > 1GHZ.
	 *     5430 > 1.4GHZ.
	 */
	if ((omap_rev >= OMAP4460_ES1_0) && (omap_rev < OMAP5430_ES1_0)) {
		mpu_dpll_regs =
			(struct dpll_regs *)&prcm->cm_clkmode_dpll_mpu;
		bypass_dpll(&prcm->cm_clkmode_dpll_mpu);
		clrbits_le32(&prcm->cm_mpu_mpu_clkctrl,
			MPU_CLKCTRL_CLKSEL_EMIF_DIV_MODE_MASK);
		setbits_le32(&prcm->cm_mpu_mpu_clkctrl,
			MPU_CLKCTRL_CLKSEL_ABE_DIV_MODE_MASK);
		clrbits_le32(&mpu_dpll_regs->cm_clksel_dpll,
			CM_CLKSEL_DCC_EN_MASK);
	}

	setbits_le32(&prcm->cm_mpu_mpu_clkctrl,
		MPU_CLKCTRL_CLKSEL_EMIF_DIV_MODE_MASK);
	setbits_le32(&prcm->cm_mpu_mpu_clkctrl,
		MPU_CLKCTRL_CLKSEL_ABE_DIV_MODE_MASK);

	params = get_mpu_dpll_params();

#else

	const struct dpll_params *params;
	struct dpll_regs *mpu_dpll_regs;
	u32 omap_rev = omap_revision();
#ifdef CONFIG_OMAP44XX
	u32 sys_clk_khz, mpu_clkout_m2_khz;
#endif

	params = get_mpu_dpll_params();

#ifdef CONFIG_OMAP44XX
	/* Dump MPU DPLL locked frequency */
	sys_clk_khz = get_sys_clk_freq() / 1000;
	mpu_clkout_m2_khz = sys_clk_khz * params->m / (params->n + 1);
#ifdef CONFIG_OMAP44XX_CORE_233MHZ
	printf("MPU@%uKHz CORE@233MHz\n", mpu_clkout_m2_khz);
#else
	printf("MPU@%uKHz CORE@200MHz\n", mpu_clkout_m2_khz);
#endif
#endif /* CONFIG_OMAP44XX */

	if ((omap_rev >= OMAP4460_ES1_0) && (omap_rev < OMAP5430_ES1_0)) {
		u32 v;

		mpu_dpll_regs =
			(struct dpll_regs *)&prcm->cm_clkmode_dpll_mpu;
		bypass_dpll(&prcm->cm_clkmode_dpll_mpu);

		/**
		 * OMAP4430: MPU 300MHz(OPP50), 600MHz(OPP100), 800MHz(OPPTURBO), 1008MHz(OPPNITRO)
		 * OMAP4460: MPU 350MHz(OPP50), 700MHz(OPP100), 920MHz(OPPTURBO)
		 * OMAP4470: MPU 396.8MHz(OPP50), 800MHz(OPP100), 1100MHz(OPPTURBO, DCC on)
		 */

		/*
		 * The interconnect frequency to EMIF should
		 * be switched between MPU clk divide by 4 (for
		 * frequencies higher than 920Mhz) and MPU clk divide
		 * by 2 (for frequencies lower than or equal to 920Mhz)
		 * Also the async bridge to ABE must be MPU clk divide
		 * by 8 for MPU clk > 748Mhz and MPU clk divide by 4
		 * for lower frequencies.
		 */
		v = __raw_readl(&prcm->cm_mpu_mpu_clkctrl);
		if (mpu_clkout_m2_khz > 920000)
			/* EMIF: MPU DPLL clock divided by 4 */
			v |= MPU_CLKCTRL_CLKSEL_EMIF_DIV_MODE_MASK;
		else
			/* EMIF: MPU DPLL clock divided by 2 */
			v &= ~MPU_CLKCTRL_CLKSEL_EMIF_DIV_MODE_MASK;
		if (mpu_clkout_m2_khz > 748000)
			/* ABE: MPU DPLL clock divided by 8 */
			v |= MPU_CLKCTRL_CLKSEL_ABE_DIV_MODE_MASK;
		else
			/* ABE: MPU DPLL clock divided by 4 */
			v &= ~MPU_CLKCTRL_CLKSEL_ABE_DIV_MODE_MASK;
		__raw_writel(v, &prcm->cm_mpu_mpu_clkctrl);

		/*
		 * A direct transition from a locked mode with DCC_EN=1 to a manual bypass
		 * mode is not allowed, because DPLL CLKOUTX2_M3 output does not support
		 * bypass mode. An intermediate state, with DCC_EN=0 and DPLL locked, must
		 * be programmed between these modes.
		 */
		/*
		 * To obtain MPU DPLL frequency higher than 1GHz, On OMAP4470,
		 * DCC (Duty Cycle Correction) needs to be enabled.
		 * And needs to be kept disabled for < 1 Ghz.
		 *
		 * OMAP4460 has a HW issue with DCC so until proper WA is found
		 * DCC shouldn't be used at any frequency.
		 */
		if (omap_rev < OMAP4470_ES1_0 || mpu_clkout_m2_khz <= 1000000) {
			/* disable DCC */
			clrbits_le32(&mpu_dpll_regs->cm_clksel_dpll,
				CM_CLKSEL_DCC_EN_MASK);
			debug("DCC is disabled\n");
		} else {
#if defined(CONFIG_OMAP44XX_MPU_OPPTURBO)
			/*
			 * On OMAP4470, the MPU clk for frequencies higher than 1Ghz
			 * is sourced from CLKOUTX2_M3, instead of CLKOUT_M2, while
			 * value of M3 is fixed to 1. Hence for frequencies higher
			 * than 1 Ghz, lock the DPLL at half the rate so the
			 * CLKOUTX2_M3 then matches the requested rate.
			 */
			memcpy(&omap4_mpu_dpll_params_clkoutx2_m3, params, sizeof(struct dpll_params));
			omap4_mpu_dpll_params_clkoutx2_m3.n = (params->n + 1) * 2 - 1; /* CLKOUTX2_M3 */
			params = &omap4_mpu_dpll_params_clkoutx2_m3;
#if 0
			mpu_clkout_m2_khz = sys_clk_khz * params->m / (params->n + 1);
			printf("mpu CLKOUT_M2 %d KHz: m %d, n %d\n", mpu_clkout_m2_khz, params->m, params->n);
#else
			debug("mpu CLKOUTX2_M3 %d KHz: m %d, n %d\n", mpu_clkout_m2_khz, params->m, params->n);
#endif

			v = __raw_readl(&mpu_dpll_regs->cm_clksel_dpll);
			v &= ~CM_CLKSEL_DCC_COUNT_MAX_MASK;
			v |= (5 << CM_CLKSEL_DCC_COUNT_MAX_SHIFT);
			__raw_writel(v, &mpu_dpll_regs->cm_clksel_dpll);

			/* enable DCC */
			v |= CM_CLKSEL_DCC_EN_MASK;
			__raw_writel(v, &mpu_dpll_regs->cm_clksel_dpll);
			debug("DCC is enabled\n");
#endif
		}
	}

#endif

	do_setup_dpll(&prcm->cm_clkmode_dpll_mpu, params, DPLL_LOCK, "mpu");
	debug("MPU DPLL locked\n");
}

/*#ifdef CONFIG_USB_EHCI_OMAP*/
static void setup_usb_dpll(void)
{
	const struct dpll_params *params;
	u32 sys_clk_khz, sd_div, num, den;

	sys_clk_khz = get_sys_clk_freq() / 1000;
	/*
	 * USB:
	 * USB dpll is J-type. Need to set DPLL_SD_DIV for jitter correction
	 * DPLL_SD_DIV = CEILING ([DPLL_MULT/(DPLL_DIV+1)]* CLKINP / 250)
	 *      - where CLKINP is sys_clk in MHz
	 * Use CLKINP in KHz and adjust the denominator accordingly so
	 * that we have enough accuracy and at the same time no overflow
	 */
	params = get_usb_dpll_params();
	num = params->m * sys_clk_khz;
	den = (params->n + 1) * 250 * 1000;
	num += den - 1;
	sd_div = num / den;
	clrsetbits_le32(&prcm->cm_clksel_dpll_usb,
			CM_CLKSEL_DPLL_DPLL_SD_DIV_MASK,
			sd_div << CM_CLKSEL_DPLL_DPLL_SD_DIV_SHIFT);

	/* Select the 60Mhz clock 480/8 = 60 */
	setbits_le32(&prcm->cm_clksel_usb_60mhz, 0x01);

	/* Now setup the dpll with the regular function */
	do_setup_dpll(&prcm->cm_clkmode_dpll_usb, params, DPLL_LOCK, "usb");
}
/*#endif*/

static void setup_dplls(void)
{
	u32 temp;
	const struct dpll_params *params;

	debug("setup_dplls\n");

	/* CORE dpll */
	params = get_core_dpll_params();	/* default - safest */
	/*
	 * Do not lock the core DPLL now. Just set it up.
	 * Core DPLL will be locked after setting up EMIF
	 * using the FREQ_UPDATE method(freq_update_core())
	 */
	if (omap_revision() != OMAP5432_ES1_0)
		do_setup_dpll(&prcm->cm_clkmode_dpll_core, params,
							DPLL_NO_LOCK, "core");
	else
		do_setup_dpll(&prcm->cm_clkmode_dpll_core, params,
							DPLL_LOCK, "core");
	/* Set the ratios for CORE_CLK, L3_CLK, L4_CLK */
	temp = (CLKSEL_CORE_X2_DIV_1 << CLKSEL_CORE_SHIFT) |
	    (CLKSEL_L3_CORE_DIV_2 << CLKSEL_L3_SHIFT) |
	    (CLKSEL_L4_L3_DIV_2 << CLKSEL_L4_SHIFT);
	writel(temp, &prcm->cm_clksel_core);
	debug("Core DPLL configured\n");

	/* lock PER dpll */
	params = get_per_dpll_params();
	do_setup_dpll(&prcm->cm_clkmode_dpll_per,
			params, DPLL_LOCK, "per");
	debug("PER DPLL locked\n");

	/* MPU dpll */
	configure_mpu_dpll();

#if 0
#ifdef CONFIG_USB_EHCI_OMAP
	setup_usb_dpll();
#endif
#else
	setup_usb_dpll();
#endif
}

/*#ifdef CONFIG_SYS_CLOCKS_ENABLE_ALL*/
static void setup_non_essential_dplls(void)
{
	u32 abe_ref_clk;
	const struct dpll_params *params;

	/* IVA */
	clrsetbits_le32(&prcm->cm_bypclk_dpll_iva,
		CM_BYPCLK_DPLL_IVA_CLKSEL_MASK, DPLL_IVA_CLKSEL_CORE_X2_DIV_2);

	params = get_iva_dpll_params();
	do_setup_dpll(&prcm->cm_clkmode_dpll_iva, params, DPLL_LOCK, "iva");

	/* Allow auto idle for IVA DPLL because we don't use it in Bootloader */
	do_allow_idle_dpll(&prcm->cm_clkmode_dpll_iva);

	/* Configure ABE dpll */
	params = get_abe_dpll_params();
#ifdef CONFIG_SYS_OMAP_ABE_SYSCK
	abe_ref_clk = CM_ABE_PLL_REF_CLKSEL_CLKSEL_SYSCLK;
#else
	abe_ref_clk = CM_ABE_PLL_REF_CLKSEL_CLKSEL_32KCLK;
	/*
	 * We need to enable some additional options to achieve
	 * 196.608MHz from 32768 Hz
	 */
	setbits_le32(&prcm->cm_clkmode_dpll_abe,
			CM_CLKMODE_DPLL_DRIFTGUARD_EN_MASK|
			CM_CLKMODE_DPLL_RELOCK_RAMP_EN_MASK|
			CM_CLKMODE_DPLL_LPMODE_EN_MASK|
			CM_CLKMODE_DPLL_REGM4XEN_MASK);
	/* Spend 4 REFCLK cycles at each stage */
	clrsetbits_le32(&prcm->cm_clkmode_dpll_abe,
			CM_CLKMODE_DPLL_RAMP_RATE_MASK,
			1 << CM_CLKMODE_DPLL_RAMP_RATE_SHIFT);
#endif

	/* Select the right reference clk */
	clrsetbits_le32(&prcm->cm_abe_pll_ref_clksel,
			CM_ABE_PLL_REF_CLKSEL_CLKSEL_MASK,
			abe_ref_clk << CM_ABE_PLL_REF_CLKSEL_CLKSEL_SHIFT);
	/* Lock the dpll */
	do_setup_dpll(&prcm->cm_clkmode_dpll_abe, params, DPLL_LOCK, "abe");

#ifndef CONFIG_SYS_OMAP_ABE_SYSCK
	/* OMAP4430: Fix errata i723 (DPLL ABE Warm Reset Issue) */
	/* http://dev.omapzoom.org/?p=bootloader/x-loader.git;a=commit;h=9aca526af910a146a3e50639b1d6d5e43c945533 */
	if (omap_revision() < OMAP4460_ES1_0) {
		/* Set M & N values again for DPLL_ABE in warm reset scenario.*/
		u32 temp;

		/* Set M & N */
		temp = readl(&prcm->cm_clkmode_dpll_abe);
		temp &= ~CM_CLKSEL_DPLL_M_MASK;
		temp |= ((params->m << CM_CLKSEL_DPLL_M_SHIFT) & CM_CLKSEL_DPLL_M_MASK);
		temp &= ~CM_CLKSEL_DPLL_N_MASK;
		temp |= ((params->n << CM_CLKSEL_DPLL_N_SHIFT) & CM_CLKSEL_DPLL_N_MASK);
		writel(temp, &prcm->cm_clkmode_dpll_abe);
	}
#endif /* !CONFIG_SYS_OMAP_ABE_SYSCK */

	/* Allow auto idle for ABE DPLL because we don't use it in Bootloader */
	do_allow_idle_dpll(&prcm->cm_clkmode_dpll_abe);
}
/*#endif*/

#ifdef CONFIG_TWL6032_POWER
#else /* !CONFIG_TWL6032_POWER */
void do_scale_tps62361(int gpio, u32 reg, u32 volt_mv)
{
	u32 step;
	int ret = 0;

	/* See if we can first get the GPIO if needed */
	if (gpio >= 0)
		ret = gpio_request(gpio, "TPS62361_VSEL0_GPIO");
	if (ret < 0) {
		printf("%s: gpio %d request failed %d\n", __func__, gpio, ret);
		gpio = -1;
	}

	/* Pull the GPIO low to select SET0 register, while we program SET1 */
	if (gpio >= 0)
		gpio_direction_output(gpio, 0);

	step = volt_mv - TPS62361_BASE_VOLT_MV;
	step /= 10;

	debug("do_scale_tps62361: volt - %d step - 0x%x\n", volt_mv, step);
	if (omap_vc_bypass_send_value(TPS62361_I2C_SLAVE_ADDR, reg, step))
		puts("Scaling voltage failed for vdd_mpu from TPS\n");

	/* Pull the GPIO high to select SET1 register */
	if (gpio >= 0)
		gpio_direction_output(gpio, 1);
}
#endif /* CONFIG_TWL6032_POWER */

void do_scale_vcore(u32 vcore_reg, u32 volt_mv)
{
	u32 offset_code;
	u32 offset = volt_mv;

	/* convert to uV for better accuracy in the calculations */
	offset *= 1000;

	offset_code = get_offset_code(offset);

	debug("do_scale_vcore: volt - %d offset_code - 0x%x\n", volt_mv,
		offset_code);

	if (omap_vc_bypass_send_value(SMPS_I2C_SLAVE_ADDR,
				vcore_reg, offset_code))
#if 0
		printf("Scaling voltage failed for 0x%x\n", vcore_reg);
#else
		printf("volt scale err 0x%x\n", vcore_reg);
#endif
}

static inline void enable_clock_domain(u32 *const clkctrl_reg, u32 enable_mode)
{
	clrsetbits_le32(clkctrl_reg, CD_CLKCTRL_CLKTRCTRL_MASK,
			enable_mode << CD_CLKCTRL_CLKTRCTRL_SHIFT);
	(void)__raw_readl(clkctrl_reg); /* Flush the post write */
	debug("Enable clock domain - %p\n", clkctrl_reg);
}

static inline void wait_for_clk_enable(u32 *clkctrl_addr)
{
	u32 clkctrl, idlest = MODULE_CLKCTRL_IDLEST_DISABLED;
	u32 bound = LDELAY;

	while ((idlest == MODULE_CLKCTRL_IDLEST_DISABLED) ||
		(idlest == MODULE_CLKCTRL_IDLEST_TRANSITIONING)) {

		clkctrl = readl(clkctrl_addr);
		idlest = (clkctrl & MODULE_CLKCTRL_IDLEST_MASK) >>
			 MODULE_CLKCTRL_IDLEST_SHIFT;
		if (--bound == 0) {
#if 0
			printf("Clock enable failed for 0x%p idlest 0x%x\n",
				clkctrl_addr, clkctrl);
#else
			printf("clock enable err 0x%p=0x%x,%d\n", clkctrl_addr, clkctrl, idlest);
#endif
			return;
		}
	}
}

static inline void enable_clock_module(u32 *const clkctrl_addr, u32 enable_mode,
				u32 wait_for_enable)
{
	clrsetbits_le32(clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_MASK,
			enable_mode << MODULE_CLKCTRL_MODULEMODE_SHIFT);
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	debug("Enable clock module - %p\n", clkctrl_addr);
	if (wait_for_enable)
		wait_for_clk_enable(clkctrl_addr);
}

#ifndef CONFIG_SPL_BUILD
/* PWRDM_POWER_OFF/../PWRDM_POWER_ON */
u32 omap_pwrdm_read_pwrst(u32 *prm_offset)
{
	void *pm_pwstst_reg = ((void*)prm_offset) + OMAP4_PM_PWSTST;
	u32 v;

	v = __raw_readl(pm_pwstst_reg);
	v &= OMAP_POWERSTATEST_MASK;
	v >>= OMAP_POWERSTATEST_SHIFT;

	return v;
}

void omap_pwrdm_wait_transition(u32 *prm_offset)
{
	void *pm_pwstst_reg = ((void*)prm_offset) + OMAP4_PM_PWSTST;

	if (!wait_on_value(OMAP_INTRANSITION_MASK, 0, pm_pwstst_reg, (LDELAY << 1))) {
		printf("pwrdm_wait err 0x%p=0x%x\n",
			pm_pwstst_reg, __raw_readl(pm_pwstst_reg));
	}
}
#endif

void omap_clk_domain(u32 *clkctrl_reg, u32 enable_mode)
{
	clrsetbits_le32(clkctrl_reg, CD_CLKCTRL_CLKTRCTRL_MASK,
			enable_mode << CD_CLKCTRL_CLKTRCTRL_SHIFT);
	(void)__raw_readl(clkctrl_reg); /* Flush the post write */
	debug("Enable clock domain %p\n", clkctrl_reg);
}

void omap_clk_enable(u32 *clkctrl_addr, u32 enable_mode)
{
	clrsetbits_le32(clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_MASK,
			enable_mode << MODULE_CLKCTRL_MODULEMODE_SHIFT);
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	debug("Enable clock module %p\n", clkctrl_addr);
	wait_for_clk_enable(clkctrl_addr);
}

void omap_clk_disable(u32 *clkctrl_addr)
{
	clrsetbits_le32(clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_DISABLE << MODULE_CLKCTRL_MODULEMODE_SHIFT);
	/* Must flush the post write TWICE to make sure the clock module is disabled */
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	debug("Disable clock module %p\n", clkctrl_addr);
}

#ifndef CONFIG_SPL_BUILD
/* Enable one optional clock at one time */
void omap_clk_optional_enable(u32 *clkctrl_addr, u32 opt_clk)
{
	setbits_le32(clkctrl_addr, opt_clk);
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	debug("Enable opt clock %p: %x\n", clkctrl_addr, opt_clk);
}

/* Disable one optional clock at one time */
void omap_clk_optional_disable(u32 *clkctrl_addr, u32 opt_clk)
{
	clrbits_le32(clkctrl_addr, opt_clk);
	/* Must flush the post write TWICE to make sure this optional clock is disabled before disabling its clock module */
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	(void)__raw_readl(clkctrl_addr); /* Flush the post write */
	debug("Disable opt clock %p: %x\n", clkctrl_addr, opt_clk);
}
#endif

void freq_update_core(void)
{
	u32 freq_config1 = 0;
	const struct dpll_params *core_dpll_params;
	u32 omap_rev = omap_revision();

	core_dpll_params = get_core_dpll_params();
	/* Put EMIF clock domain in sw wakeup mode */
	enable_clock_domain(&prcm->cm_memif_clkstctrl,
				CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
	wait_for_clk_enable(&prcm->cm_memif_emif_1_clkctrl);
	wait_for_clk_enable(&prcm->cm_memif_emif_2_clkctrl);

	freq_config1 = SHADOW_FREQ_CONFIG1_FREQ_UPDATE_MASK |
	    SHADOW_FREQ_CONFIG1_DLL_RESET_MASK;

	freq_config1 |= (DPLL_EN_LOCK << SHADOW_FREQ_CONFIG1_DPLL_EN_SHIFT) &
				SHADOW_FREQ_CONFIG1_DPLL_EN_MASK;

	freq_config1 |= (core_dpll_params->m2 <<
			SHADOW_FREQ_CONFIG1_M2_DIV_SHIFT) &
			SHADOW_FREQ_CONFIG1_M2_DIV_MASK;

	writel(freq_config1, &prcm->cm_shadow_freq_config1);
	if (!wait_on_value(SHADOW_FREQ_CONFIG1_FREQ_UPDATE_MASK, 0,
				&prcm->cm_shadow_freq_config1, LDELAY)) {
#if 0
		puts("FREQ UPDATE procedure failed!!");
#else
		puts("ERR: CORE freq update");
#endif
		hang();
	}

	/*
	 * Putting EMIF in HW_AUTO is seen to be causing issues with
	 * EMIF clocks and the master DLL. Put EMIF in SW_WKUP
	 * in OMAP5430 ES1.0 silicon
	 */
	if (omap_rev != OMAP5430_ES1_0) {
		/* Put EMIF clock domain back in hw auto mode */
		enable_clock_domain(&prcm->cm_memif_clkstctrl,
					CD_CLKCTRL_CLKTRCTRL_HW_AUTO);
		wait_for_clk_enable(&prcm->cm_memif_emif_1_clkctrl);
		wait_for_clk_enable(&prcm->cm_memif_emif_2_clkctrl);
	}
}

void bypass_dpll(u32 *const base)
{
	do_bypass_dpll(base);
	wait_for_bypass(base);
}

#if 0
void lock_dpll(u32 *const base)
{
	do_lock_dpll(base);
	wait_for_lock(base);
}
#endif

void setup_clocks_for_console(void)
{
	/* Do not add any spl_debug prints in this function */
	clrsetbits_le32(&prcm->cm_l4per_clkstctrl, CD_CLKCTRL_CLKTRCTRL_MASK,
			CD_CLKCTRL_CLKTRCTRL_SW_WKUP <<
			CD_CLKCTRL_CLKTRCTRL_SHIFT);

#if 0
	/* Enable all UARTs - console will be on one of them */
	clrsetbits_le32(&prcm->cm_l4per_uart1_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN <<
			MODULE_CLKCTRL_MODULEMODE_SHIFT);

	clrsetbits_le32(&prcm->cm_l4per_uart2_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN <<
			MODULE_CLKCTRL_MODULEMODE_SHIFT);

	clrsetbits_le32(&prcm->cm_l4per_uart3_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN <<
			MODULE_CLKCTRL_MODULEMODE_SHIFT);

	clrsetbits_le32(&prcm->cm_l4per_uart3_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_MASK,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN <<
			MODULE_CLKCTRL_MODULEMODE_SHIFT);
#else
#if (CONFIG_CONS_INDEX == 1)
	enable_clock_module(&prcm->cm_l4per_uart1_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);
#elif (CONFIG_CONS_INDEX == 2)
	enable_clock_module(&prcm->cm_l4per_uart2_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);
#elif (CONFIG_CONS_INDEX == 3)
	enable_clock_module(&prcm->cm_l4per_uart3_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN, 1);
#endif
#endif

	clrsetbits_le32(&prcm->cm_l4per_clkstctrl, CD_CLKCTRL_CLKTRCTRL_MASK,
			CD_CLKCTRL_CLKTRCTRL_HW_AUTO <<
			CD_CLKCTRL_CLKTRCTRL_SHIFT);
}

void do_enable_clocks(u32 *const *clk_domains,
			    u32 *const *clk_modules_hw_auto,
			    u32 *const *clk_modules_explicit_en,
			    u8 wait_for_enable)
{
	u32 i, max = 100;

	/* Put the clock domains in SW_WKUP mode */
	for (i = 0; (i < max) && clk_domains[i]; i++) {
		enable_clock_domain(clk_domains[i],
				    CD_CLKCTRL_CLKTRCTRL_SW_WKUP);
	}

	/* Clock modules that need to be put in HW_AUTO */
	for (i = 0; (i < max) && clk_modules_hw_auto[i]; i++) {
		enable_clock_module(clk_modules_hw_auto[i],
				    MODULE_CLKCTRL_MODULEMODE_HW_AUTO,
				    wait_for_enable);
	};

	/* Clock modules that need to be put in SW_EXPLICIT_EN mode */
	for (i = 0; (i < max) && clk_modules_explicit_en[i]; i++) {
		enable_clock_module(clk_modules_explicit_en[i],
				    MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN,
				    wait_for_enable);
	};

	/* Put the clock domains in HW_AUTO mode now */
	for (i = 0; (i < max) && clk_domains[i]; i++) {
		enable_clock_domain(clk_domains[i],
				    CD_CLKCTRL_CLKTRCTRL_HW_AUTO);
	}
}

void prcm_init(void)
{
	switch (omap_hw_init_context()) {
	case OMAP_INIT_CONTEXT_SPL:
	case OMAP_INIT_CONTEXT_UBOOT_FROM_NOR:
	case OMAP_INIT_CONTEXT_UBOOT_AFTER_CH:
		enable_basic_clocks();
		scale_vcores();
		setup_dplls();
#if defined(CONFIG_OMAP54XX)
#ifdef CONFIG_SYS_CLOCKS_ENABLE_ALL
		setup_non_essential_dplls();
		enable_non_essential_clocks();
#endif
#else
		setup_non_essential_dplls();
		/* James Wu: Don't turn on non-essential clocks */
#endif
		break;
	default:
		break;
	}

#if defined(CONFIG_OMAP54XX)
	if (OMAP_INIT_CONTEXT_SPL != omap_hw_init_context())
		enable_basic_uboot_clocks();
#else
#endif
}
