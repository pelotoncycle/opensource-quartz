/*
 *
 * Common functions for OMAP4 based boards
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Aneesh V	<aneesh@ti.com>
 *	Steve Sakoman	<steve@sakoman.com>
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
#include <asm/armv7.h>
#include <asm/arch/cpu.h>
#include <asm/arch/sys_proto.h>
#include <asm/sizes.h>
#include <asm/emif.h>
#include <asm/arch/gpio.h>
#include <asm/arch/omap_rom.h>

DECLARE_GLOBAL_DATA_PTR;

u32 public_rom_base = PUBLIC_API_BASE_4430;

u32 *const omap_si_rev = (u32 *)OMAP4_SRAM_SCRATCH_OMAP4_REV;

static const struct gpio_bank gpio_bank_44xx[6] = {
	{ (void *)OMAP44XX_GPIO1_BASE, METHOD_GPIO_24XX },
	{ (void *)OMAP44XX_GPIO2_BASE, METHOD_GPIO_24XX },
	{ (void *)OMAP44XX_GPIO3_BASE, METHOD_GPIO_24XX },
	{ (void *)OMAP44XX_GPIO4_BASE, METHOD_GPIO_24XX },
	{ (void *)OMAP44XX_GPIO5_BASE, METHOD_GPIO_24XX },
	{ (void *)OMAP44XX_GPIO6_BASE, METHOD_GPIO_24XX },
};

const struct gpio_bank *const omap_gpio_bank = gpio_bank_44xx;

#ifdef CONFIG_SPL_BUILD

/*#define DEBUG_LPDDR2IO*/

#ifdef DEBUG_LPDDR2IO
#define LPDDR2IO_DEBUG(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#else
#define LPDDR2IO_DEBUG(fmt, args...) \
	do {} while (0)
#endif /* DEBUG_LPDDR2IO */

#define LPDDR_WD_PULL_DOWN				0x02

/*
 * Trim value has to be written to CONTROL_EFUSE_2 according to
 * OMAP4430 errata i684 (version B)
 * OMAP4430 units with ProdID[51:50]=11 are not affected
 */
#define OMAP4_LPDDR2_I684_FIX_VALUE	0x004E4000
#define OMAP4_PROD_ID_I684_MASK		0x000C0000

/*
 * Some tuning of IOs for optimal power and performance
 */
void do_io_settings(void)
{
#if 0
	u32 lpddr2io;
#endif
	struct control_lpddr2io_regs *lpddr2io_regs =
		(struct control_lpddr2io_regs *)LPDDR2_IO_REGS_BASE;
	struct omap_sys_ctrl_regs *const ctrl =
		(struct omap_sys_ctrl_regs *)SYSCTRL_GENERAL_CORE_BASE;
	u32 val;

	u32 omap4_rev = omap_revision();

#if 0
	if (omap4_rev == OMAP4430_ES1_0)
		lpddr2io = CONTROL_LPDDR2IO_SLEW_125PS_DRV8_PULL_DOWN;
	else if (omap4_rev == OMAP4430_ES2_0)
		lpddr2io = CONTROL_LPDDR2IO_SLEW_325PS_DRV8_GATE_KEEPER;
	else
		lpddr2io = CONTROL_LPDDR2IO_SLEW_315PS_DRV12_PULL_DOWN;

	/* EMIF1 */
	writel(lpddr2io, &lpddr2io_regs->control_lpddr2io1_0);
	writel(lpddr2io, &lpddr2io_regs->control_lpddr2io1_1);
	/* No pull for GR10 as per hw team's recommendation */
	writel(lpddr2io & ~LPDDR2IO_GR10_WD_MASK,
		&lpddr2io_regs->control_lpddr2io1_2);
	writel(CONTROL_LPDDR2IO_3_VAL, &lpddr2io_regs->control_lpddr2io1_3);

	/* EMIF2 */
	writel(lpddr2io, &lpddr2io_regs->control_lpddr2io2_0);
	writel(lpddr2io, &lpddr2io_regs->control_lpddr2io2_1);
	/* No pull for GR10 as per hw team's recommendation */
	writel(lpddr2io & ~LPDDR2IO_GR10_WD_MASK,
		&lpddr2io_regs->control_lpddr2io2_2);
	writel(CONTROL_LPDDR2IO_3_VAL, &lpddr2io_regs->control_lpddr2io2_3);
#else

	/* James Wu: refer to X-Loader __ddr_init() in sdram_elpida.c */
	/* Pull Dn enabled for "Weak driver control" on LPDDR
	 * Interface.
	 */
	if (omap4_rev >= OMAP4460_ES1_0) {
		writel(0x9c9c9c9c, &lpddr2io_regs->control_lpddr2io1_0);
		writel(0x9c9c9c9c, &lpddr2io_regs->control_lpddr2io1_1);
		writel(0x9c989c00, &lpddr2io_regs->control_lpddr2io1_2);
		writel(0xa0888c03, &lpddr2io_regs->control_lpddr2io1_3);
		writel(0x9c9c9c9c, &lpddr2io_regs->control_lpddr2io2_0);
		writel(0x9c9c9c9c, &lpddr2io_regs->control_lpddr2io2_1);
		writel(0x9c989c00, &lpddr2io_regs->control_lpddr2io2_2);
		writel(0xa0888c03, &lpddr2io_regs->control_lpddr2io2_3);
	}

	/* James Wu: refer to Kernel syscontrol_setup_regs() in pm44xx.c */
	/* Disable LPDDR VREF manual control and enable Auto control */
	val = readl(&lpddr2io_regs->control_lpddr2io1_3);
	val &= ~(OMAP4_LPDDR21_VREF_EN_CA_MASK | OMAP4_LPDDR21_VREF_EN_DQ_MASK);
	val |= OMAP4_LPDDR21_VREF_AUTO_EN_CA_MASK | OMAP4_LPDDR21_VREF_AUTO_EN_DQ_MASK;
	writel(val, &lpddr2io_regs->control_lpddr2io1_3);
	LPDDR2IO_DEBUG("lpddr2io1_3=0x%08X\n", val);

	val = readl(&lpddr2io_regs->control_lpddr2io2_3);
	val &= ~(OMAP4_LPDDR21_VREF_EN_CA_MASK | OMAP4_LPDDR21_VREF_EN_DQ_MASK);
	val |= OMAP4_LPDDR21_VREF_AUTO_EN_CA_MASK | OMAP4_LPDDR21_VREF_AUTO_EN_DQ_MASK;
	writel(val, &lpddr2io_regs->control_lpddr2io2_3);
	LPDDR2IO_DEBUG("lpddr2io2_3=0x%08X\n\n", val);

	/* James Wu: refer to Kernel syscontrol_lpddr_clk_io_errata(true) */
	val = readl(&lpddr2io_regs->control_lpddr2io1_2);
	val &= ~OMAP4_LPDDR2IO1_GR10_WD_MASK;
	writel(val, &lpddr2io_regs->control_lpddr2io1_2);
	LPDDR2IO_DEBUG("lpddr2io1_2=0x%08X\n", val);

	val = readl(&lpddr2io_regs->control_lpddr2io2_2);
	val &= ~OMAP4_LPDDR2IO2_GR10_WD_MASK;
	writel(val, &lpddr2io_regs->control_lpddr2io2_2);
	LPDDR2IO_DEBUG("lpddr2io2_2=0x%08X\n\n", val);

#ifdef CONFIG_OMAP44XX_CORE_233MHZ
	/* James Wu: refer to Kernel syscontrol_lpddr2io_config_update_466_mhz() */
	/*
	 * According to the OMAP4470 LPDDR interface configuration
	 * update for 466MHz Slew Rate should be set to “FASTEST”
	 * and Impedance Control to “Drv12”:
	 * - CONTROL_LPDDR2IOx_2[LPDDR2IO1_GR10_SR] = 0
	 * - CONTROL_LPDDR2IOx_2[LPDDR2IO1_GR10_I] = 7
	 * where x=[1-2]
	 */
	if (omap4_rev >= OMAP4470_ES1_0) {
		/* Setup LPDDR2IO1_2 */
		val = readl(&lpddr2io_regs->control_lpddr2io1_2);
		val &= ~OMAP4_LPDDR2IO1_GR10_SR_MASK;
		val |= OMAP4_LPDDR2IO1_GR10_I_MASK;
		writel(val, &lpddr2io_regs->control_lpddr2io1_2);
		LPDDR2IO_DEBUG("lpddr2io1_2=0x%08X\n", val);

		/* Setup LPDDR2IO2_2 */
		val = readl(&lpddr2io_regs->control_lpddr2io2_2);
		val &= ~OMAP4_LPDDR2IO2_GR10_SR_MASK;
		val |= OMAP4_LPDDR2IO2_GR10_I_MASK;
		writel(val, &lpddr2io_regs->control_lpddr2io2_2);
		LPDDR2IO_DEBUG("lpddr2io2_2=0x%08X\n\n", val);
	}
#endif /* CONFIG_OMAP44XX_CORE_233MHZ */

	LPDDR2IO_DEBUG("lpddr2io1_0=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io1_0));
	LPDDR2IO_DEBUG("lpddr2io1_1=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io1_1));
	LPDDR2IO_DEBUG("lpddr2io1_2=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io1_2));
	LPDDR2IO_DEBUG("lpddr2io1_3=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io1_3));
	LPDDR2IO_DEBUG("lpddr2io2_0=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io2_0));
	LPDDR2IO_DEBUG("lpddr2io2_1=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io2_1));
	LPDDR2IO_DEBUG("lpddr2io2_2=0x%08X\n", readl(&lpddr2io_regs->control_lpddr2io2_2));
	LPDDR2IO_DEBUG("lpddr2io2_3=0x%08X\n\n", readl(&lpddr2io_regs->control_lpddr2io2_3));

#endif

#if 0
	/*
	 * Some of these settings (TRIM values) come from eFuse and are
	 * in turn programmed in the eFuse at manufacturing time after
	 * calibration of the device. Do the software over-ride only if
	 * the device is not correctly trimmed
	 */
	if (!(readl(&ctrl->control_std_fuse_opp_bgap) & 0xFFFF)) {

		writel(LDOSRAM_VOLT_CTRL_OVERRIDE,
			&ctrl->control_ldosram_iva_voltage_ctrl);

		writel(LDOSRAM_VOLT_CTRL_OVERRIDE,
			&ctrl->control_ldosram_mpu_voltage_ctrl);

		writel(LDOSRAM_VOLT_CTRL_OVERRIDE,
			&ctrl->control_ldosram_core_voltage_ctrl);
	}

	/*
	 * Over-ride the register
	 *	i. unconditionally for all 4430
	 *	ii. only if un-trimmed for 4460
	 */
	if (!readl(&ctrl->control_efuse_1))
		writel(CONTROL_EFUSE_1_OVERRIDE, &ctrl->control_efuse_1);

	if ((omap4_rev < OMAP4460_ES1_0) || !readl(&ctrl->control_efuse_2))
		writel(CONTROL_EFUSE_2_OVERRIDE, &ctrl->control_efuse_2);
#else

	/* James Wu: refer to Kernel omap4_ldo_trim_configure() */

	/* if not trimmed, we set force overide, insted of efuse. */
	if (!(readl(&ctrl->control_std_fuse_opp_bgap) & OMAP4_STD_FUSE_OPP_BGAP_MASK_LSB)) {
		/* Fill in recommended values */
		val = 0x0f << OMAP4_LDOSRAMCORE_ACTMODE_VSET_OUT_SHIFT;
		val |= OMAP4_LDOSRAMCORE_ACTMODE_MUX_CTRL_MASK;
		val |= 0x1 << OMAP4_LDOSRAMCORE_RETMODE_VSET_OUT_SHIFT;
		val |= OMAP4_LDOSRAMCORE_RETMODE_MUX_CTRL_MASK;

		writel(val, &ctrl->control_ldosram_mpu_voltage_ctrl);
		writel(val, &ctrl->control_ldosram_core_voltage_ctrl);
		writel(val, &ctrl->control_ldosram_iva_voltage_ctrl);
	}

	/* OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_1 is reserved for 4470 */
	if (omap4_rev < OMAP4470_ES1_0) {
		/* For all trimmed and untrimmed write recommended value */
		val =  0x10 << OMAP4_AVDAC_TRIM_BYTE0_SHIFT;
		val |=  0x01 << OMAP4_AVDAC_TRIM_BYTE1_SHIFT;
		val |=  0x4d << OMAP4_AVDAC_TRIM_BYTE2_SHIFT;
		val |=  0x1C << OMAP4_AVDAC_TRIM_BYTE3_SHIFT;
		writel(val, &ctrl->control_efuse_1);
	}

	/*
	 * Errata i684 (revision B)
	 * Impacts all OMAP4430ESx.y trimmed and untrimmed excluding units
	 * with with ProdID[51:50]=11
	 * OMAP4460/70 are not impacted.
	 *
	 * ProdID:
	 * 51 50
	 * 0  0  Incorrect trim, SW WA needed.
	 * 0  1  Fixed test program issue of overlapping of LPDDR & SmartIO
	 *	 efuse fields, SW WA needed for LPDDR.
	 * 1  1  New LPDDR trim formula to compensate for vertical vs horizontal
	 *	 cell layout.  No overwrite required.
	 */
	if (omap4_rev < OMAP4460_ES1_0) {
		/* DDR I/O Trim override as per erratum i684 */
		u32 prod_id = readl(&ctrl->control_std_fuse_prod_id_1) & OMAP4_PROD_ID_I684_MASK;
		if (prod_id != OMAP4_PROD_ID_I684_MASK)
			writel(OMAP4_LPDDR2_I684_FIX_VALUE, &ctrl->control_efuse_2);
	}

#if 0 /* OMAP4460 */
	/* Required for DPLL_MPU to lock at 2.4 GHz */
	if (dpll_trim_override)
		omap_ctrl_writel(0x29, OMAP4_CTRL_MODULE_CORE_DPLL_NWELL_TRIM_0);
#endif

#endif
}
#endif

/* dummy fuction for omap4 */
void config_data_eye_leveling_samples(u32 emif_base)
{
}

void init_omap_revision(void)
{
#if 0

	/*
	 * For some of the ES2/ES1 boards ID_CODE is not reliable:
	 * Also, ES1 and ES2 have different ARM revisions
	 * So use ARM revision for identification
	 */
	unsigned int arm_rev = cortex_rev();

	switch (arm_rev) {
	case MIDR_CORTEX_A9_R0P1:
		*omap_si_rev = OMAP4430_ES1_0;
		break;
	case MIDR_CORTEX_A9_R1P2:
		switch (readl(CONTROL_ID_CODE)) {
		case OMAP4_CONTROL_ID_CODE_ES2_0:
			*omap_si_rev = OMAP4430_ES2_0;
			break;
		case OMAP4_CONTROL_ID_CODE_ES2_1:
			*omap_si_rev = OMAP4430_ES2_1;
			break;
		case OMAP4_CONTROL_ID_CODE_ES2_2:
			*omap_si_rev = OMAP4430_ES2_2;
			break;
		default:
			*omap_si_rev = OMAP4430_ES2_0;
			break;
		}
		break;
	case MIDR_CORTEX_A9_R1P3:
		*omap_si_rev = OMAP4430_ES2_3;
		break;
	case MIDR_CORTEX_A9_R2P10:
		switch (readl(CONTROL_ID_CODE)) {
		case OMAP4460_CONTROL_ID_CODE_ES1_1:
			*omap_si_rev = OMAP4460_ES1_1;
			break;
		case OMAP4460_CONTROL_ID_CODE_ES1_0:
		default:
			*omap_si_rev = OMAP4460_ES1_0;
			break;
		}
		break;
	default:
		*omap_si_rev = OMAP4430_SILICON_ID_INVALID;
		break;
	}
#else

	u32 omap4_rev = OMAP4430_SILICON_ID_INVALID;
	/*
	 * For some of the ES2/ES1 boards ID_CODE is not reliable:
	 * Also, ES1 and ES2 have different ARM revisions
	 * So use ARM revision for identification
	 */
	unsigned int arm_rev = cortex_rev();

	switch (arm_rev) {
	case MIDR_CORTEX_A9_R0P1:
		omap4_rev = OMAP4430_ES1_0;
		break;
	case MIDR_CORTEX_A9_R1P2:
		switch (readl(CONTROL_ID_CODE)) {
		case OMAP4_CONTROL_ID_CODE_ES2_0:
			omap4_rev = OMAP4430_ES2_0;
			break;
		case OMAP4_CONTROL_ID_CODE_ES2_1:
			omap4_rev = OMAP4430_ES2_1;
			break;
		case OMAP4_CONTROL_ID_CODE_ES2_2:
			omap4_rev = OMAP4430_ES2_2;
			break;
		default:
			omap4_rev = OMAP4430_ES2_0;
			break;
		}
		break;
	case MIDR_CORTEX_A9_R1P3:
		omap4_rev = OMAP4430_ES2_3;
		break;
	case MIDR_CORTEX_A9_R2P10:
		switch (readl(CONTROL_ID_CODE)) {
		case OMAP4460_CONTROL_ID_CODE_ES1_0:
			omap4_rev = OMAP4460_ES1_0;
			break;
		case OMAP4460_CONTROL_ID_CODE_ES1_1:
			omap4_rev = OMAP4460_ES1_1;
			break;
		case OMAP4470_CONTROL_ID_CODE_ES1_0:
			omap4_rev = OMAP4470_ES1_0;
			break;
		default:
			omap4_rev = OMAP4470_ES1_0;
			break;
		}
		break;
	default:
		break;
	}
	*omap_si_rev = omap4_rev;

	if (omap4_rev >= OMAP4460_ES1_0)
		public_rom_base = PUBLIC_API_BASE_4460;

#endif
}

#ifndef CONFIG_SPL_BUILD

#ifndef CONFIG_SYS_L2CACHE_OFF

#define OMAP44XX_L2CACHE_BASE		0x48242000
#define L2X0_CACHE_ID				(OMAP44XX_L2CACHE_BASE + 0x000)
#define L2X0_CTRL					(OMAP44XX_L2CACHE_BASE + 0x100)
#define L2X0_AUX_CTRL				(OMAP44XX_L2CACHE_BASE + 0x104)
#define L2X0_LOCKDOWN_WAY_D0		(OMAP44XX_L2CACHE_BASE + 0x900)
#define L2X0_LOCKDOWN_WAY_D1		(OMAP44XX_L2CACHE_BASE + 0x908)
#define L2X0_LOCKDOWN_WAY_I0		(OMAP44XX_L2CACHE_BASE + 0x904)
#define L2X0_LOCKDOWN_WAY_I1		(OMAP44XX_L2CACHE_BASE + 0x90C)
#define L2X0_PREFETCH_CTRL			(OMAP44XX_L2CACHE_BASE + 0xF60)

/* L2X0_AUX_CTRL */
#define L2X0_AUX_CTRL_ASSOCIATIVITY_SHIFT	16
#define L2X0_AUX_CTRL_WAY_SIZE_SHIFT		17
#define L2X0_AUX_CTRL_WAY_SIZE_MASK		(0x7 << 17)
#define L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT	22
#define L2X0_AUX_CTRL_NS_LOCKDOWN_SHIFT		26
#define L2X0_AUX_CTRL_NS_INT_CTRL_SHIFT		27
#define L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT	28
#define L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT	29
#define L2X0_AUX_CTRL_EARLY_BRESP_SHIFT		30

/* L2X0_PREFETCH_CTRL */
#define L2X0_PREFETCH_DATA_PREFETCH_SHIFT	28
#define L2X0_PREFETCH_INTSTR_PREFETCH_SHIFT	29
#define L2X0_PREFETCH_DOUBLE_LINEFILL_SHIFT	30

#define L2X0_POR_OFFSET_VALUE	0x5
#define L2X0_POR_OFFSET_MASK	0x1f

/* Refer to Linux kernel omap_l2_cache_init() */
void omap4_l2_cache_init(void)
{
	u32 omap4_rev = omap_revision();
	u32 aux_ctrl;
	u32 por_ctrl;

	/*
	 * 16-way associativity, parity disabled
	 * Way size - 32KB (es1.0)
	 * Way size - 64KB (es2.0 +)
	 */
	aux_ctrl = __raw_readl(L2X0_AUX_CTRL);
	/*
	 * Drop instruction prefetch hint since it degrades the
	 * the performance.
	 */
	aux_ctrl |= ((0x3 << L2X0_AUX_CTRL_WAY_SIZE_SHIFT) |
			(1 << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT) |
			(1 << L2X0_AUX_CTRL_EARLY_BRESP_SHIFT));
	/* Enable L2 data prefetch */
	if (omap4_rev != OMAP4460_ES1_0)
		aux_ctrl |= (1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT);
	omap_smc_rom(ROM_SERVICE_PL310_AUXCR, aux_ctrl);

	/* Setup POR Control register */
	por_ctrl = __raw_readl(L2X0_PREFETCH_CTRL);
	/*
	 * Double linefill is available only on OMAP4460 L2X0.
	 * It may cause single cache line memory corruption, leave it disabled
	 * on all devices
	 */
	por_ctrl &= ~(1 << L2X0_PREFETCH_DOUBLE_LINEFILL_SHIFT);
	if (omap4_rev != OMAP4460_ES1_0) {
		por_ctrl &= ~L2X0_POR_OFFSET_MASK;
		por_ctrl |= L2X0_POR_OFFSET_VALUE;
	}

#if 0
	/* Set POR through PPA service only in EMU/HS devices */
	if (get_device_type() != GP_DEVICE)
		omap4_secure_dispatcher(PPA_SERVICE_PL310_POR, 0x7, 1,
				por_ctrl, 0, 0, 0);
	else /*if (omap4_rev >= OMAP4430_ES2_1)*/
		omap_smc_rom(ROM_SERVICE_PL310_POR, por_ctrl);
#else
	if (get_device_type() != GP_DEVICE)
		/* Set PL310 Prefetch Offset Register w/PPA svc*/
		omap_smc_ppa(PPA_SERVICE_PL310_POR, 0, 0x7, 1, por_ctrl);
	else
		omap_smc_rom(ROM_SERVICE_PL310_POR, por_ctrl);
#endif

	/*
	 * FIXME: Temporary WA for OMAP4460 stability issue.
	 * Lock-down specific L2 cache ways which  makes effective
	 * L2 size as 512 KB instead of 1 MB
	 */
	if (omap4_rev == OMAP4460_ES1_0) {
		u32 lockdown = 0xa5a5;
		__raw_writel(lockdown, L2X0_LOCKDOWN_WAY_D0);
		__raw_writel(lockdown, L2X0_LOCKDOWN_WAY_D1);
		__raw_writel(lockdown, L2X0_LOCKDOWN_WAY_I0);
		__raw_writel(lockdown, L2X0_LOCKDOWN_WAY_I1);
	}
}

void v7_outer_cache_enable(void)
{
#if 0
	set_pl310_ctrl_reg(1);
#else
	omap_smc_rom(ROM_SERVICE_PL310_ENABLE, 1);
#endif
}

void v7_outer_cache_disable(void)
{
#if 0
	set_pl310_ctrl_reg(0);
#else
	omap_smc_rom(ROM_SERVICE_PL310_ENABLE, 0);
#endif
}
#endif /* CONFIG_SYS_L2CACHE_OFF */

/* Refer to Kernel "omap4_sec_dispatcher()".
 * omap4_sec_dispatcher: Routine to dispatch low power secure
 * service routines
 *
 * @idx: The HAL API index
 * @flag: The flag indicating criticality of operation
 * @nargs: Number of valid arguments out of four.
 * @arg1, arg2, arg3 args4: Parameters passed to secure API
 *
 * Return the error value on success/failure
 */
u32 omap4_secure_dispatcher(u32 idx, u32 flag, u32 nargs, u32 arg1, u32 arg2,
							u32 arg3, u32 arg4)
{
	u32 ret;
	u32 param[5];

	param[0] = nargs;
	param[1] = arg1;
	param[2] = arg2;
	param[3] = arg3;
	param[4] = arg4;

	omap_clk_domain(&prcm->cm_l4sec_clkstctrl, CD_CLKCTRL_CLKTRCTRL_SW_WKUP);

	flush_dcache_all();
	flush_dcache_range((unsigned long)param, (unsigned long)(param + 5));

	ret = omap_smc2(idx, flag, (u32)param);

	omap_clk_domain(&prcm->cm_l4sec_clkstctrl, CD_CLKCTRL_CLKTRCTRL_HW_AUTO);

	return ret;
}

#endif /* !CONFIG_SPL_BUILD */
