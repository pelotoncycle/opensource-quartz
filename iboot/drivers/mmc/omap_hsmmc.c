/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Sukumar Ghorai <s-ghorai@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <mmc.h>
#include <part.h>
#include <i2c.h>
#include <twl4030.h>
#include <twl6030.h>
#include <twl6035.h>
#include <asm/io.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>

#include <exports.h>
#include <icom/regulator.h>

/* common definitions for all OMAPs */
#define SYSCTL_SRC	(1 << 25)
#define SYSCTL_SRD	(1 << 26)

#if 0
/* If we fail after 1 second wait, something is really bad */
#define MAX_RETRY_MS	1000

static int mmc_read_data(struct hsmmc *mmc_base, char *buf, unsigned int size);
static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
			unsigned int siz);

static struct mmc hsmmc_dev[2];
#else
#define MAX_RETRY_MS	HSMMC_MAX_RETRY_MS_TIMEOUT

static int mmc_read_data(struct hsmmc *mmc_base, char *buf,
			unsigned int blksize, unsigned int blks);
#ifndef CONFIG_SPL_BUILD
static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
			unsigned int blksize, unsigned int blks);
#endif /* !CONFIG_SPL_BUILD */

#ifdef OMAP_HSMMC3_BASE
static struct mmc hsmmc_dev[3];
#else
static struct mmc hsmmc_dev[2];
#endif
#endif

#if defined(CONFIG_OMAP44XX) && defined(CONFIG_TWL6030_POWER)
static void omap4_vmmc_pbias_config(struct mmc *mmc)
{
#if 0

	u32 value = 0;
	struct omap_sys_ctrl_regs *const ctrl =
		(struct omap_sys_ctrl_regs *) SYSCTRL_GENERAL_CORE_BASE;


	value = readl(&ctrl->control_pbiaslite);
	value &= ~(MMC1_PBIASLITE_PWRDNZ | MMC1_PWRDNZ);
	writel(value, &ctrl->control_pbiaslite);
	/* set VMMC to 3V */
	twl6030_power_mmc_init();
	value = readl(&ctrl->control_pbiaslite);
	value |= MMC1_PBIASLITE_VMODE | MMC1_PBIASLITE_PWRDNZ | MMC1_PWRDNZ;
	writel(value, &ctrl->control_pbiaslite);

#else

#if defined(CONFIG_TWL6030_POWER)
#ifdef CONFIG_TWL6032_POWER
#define REGULATOR_VMMC	TWL6032_LDO5
#define REGULATOR_VEMMC	TWL6032_LDO2
#else
#define REGULATOR_VMMC	TWL6030_VMMC
#define REGULATOR_VEMMC	TWL6030_VAUX1
#endif
#endif /* CONFIG_TWL6030_POWER */

	if ((u32)mmc->priv == OMAP_HSMMC1_BASE) {
		/* MMC1 */
		struct omap_sys_ctrl_regs *const ctrl =
				(struct omap_sys_ctrl_regs *)SYSCTRL_GENERAL_CORE_BASE;
		u32 value;
		ulong start;

		/* PBIAS config needed for MMC1 only */

		value = readl(&ctrl->control_pbiaslite);
		value &= ~(MMC1_PBIASLITE_PWRDNZ | MMC1_PWRDNZ | MMC1_PBIASLITE_VMODE);
		writel(value, &ctrl->control_pbiaslite);

		/* set VMMC to 3.0V */
#if defined(CONFIG_TWL6030_POWER)
		regulator_disable(REGULATOR_VMMC);
		udelay(1000); /* wait for voltage off */
		regulator_set_voltage(REGULATOR_VMMC, 3000);
		regulator_enable(REGULATOR_VMMC);
#endif /* CONFIG_TWL6030_POWER */

		value = readl(&ctrl->control_pbiaslite);
		value |= MMC1_PBIASLITE_VMODE | MMC1_PBIASLITE_PWRDNZ | MMC1_PWRDNZ;
		writel(value, &ctrl->control_pbiaslite);

		start = get_timer(0);
		while (readl(&ctrl->control_pbiaslite) & MMC1_PBIASLITE_VMODE_ERROR) {
			udelay(100);
			if (get_timer(start) > 10)
				break;
		}
		value = readl(&ctrl->control_pbiaslite);
		if (value & MMC1_PBIASLITE_VMODE_ERROR) {
			printf("MMC1 PBIAS err 0x%x\n", value);
			value &= ~(MMC1_PWRDNZ);
			writel(value, &ctrl->control_pbiaslite);
		}

		/**
		 * SDMMC1_PUSTRENGTH_GRP0: sdmmc1_clk;
		 * SDMMC1_DR0_SPEEDCTRL: sdmmc1_clk; sdmmc1_cmd; sdmmc1_dat0; sdmmc1_dat1;
		 *
		 * SDMMC1_PUSTRENGTH_GRP1: sdmmc1_cmd;
		 * SDMMC1_DR1_SPEEDCTRL: sdmmc1_dat4-sdmmc1_dat7
		 *
		 * SDMMC1_PUSTRENGTH_GRP2: sdmmc1_dat0; sdmmc1_dat1; sdmmc1_dat2; sdmmc1_dat3;
		 * SDMMC1_DR2_SPEEDCTRL: sdmmc1_dat2; sdmmc1_dat3;
		 *
		 * SDMMC1_PUSTRENGTH_GRP3: sdmmc1_dat4-sdmmc1_dat7
		 */
		value = readl(&ctrl->control_mmc1);
#if 0
		value |= SDMMC1_PUSTRENGTH_GRP0 | SDMMC1_PUSTRENGTH_GRP1;
		value &= ~(SDMMC1_PUSTRENGTH_GRP2 | SDMMC1_PUSTRENGTH_GRP3);
		value |= SDMMC1_DR0_SPEEDCTRL | SDMMC1_DR1_SPEEDCTRL | SDMMC1_DR2_SPEEDCTRL;
#else
		value |= SDMMC1_DR0_SPEEDCTRL | SDMMC1_DR2_SPEEDCTRL |
				SDMMC1_PUSTRENGTH_GRP0 | SDMMC1_PUSTRENGTH_GRP1 | SDMMC1_PUSTRENGTH_GRP2;
		if (mmc->host_caps & MMC_MODE_8BIT)
			value |= SDMMC1_DR1_SPEEDCTRL | SDMMC1_PUSTRENGTH_GRP3;
		else
			value &= ~(SDMMC1_DR1_SPEEDCTRL | SDMMC1_PUSTRENGTH_GRP3);
#endif
		writel(value, &ctrl->control_mmc1);
	} else if ((u32)mmc->priv == OMAP_HSMMC2_BASE) {
		/* set eMMC to 2.9V */
#if defined(CONFIG_TWL6030_POWER)
#if 0
		regulator_disable(REGULATOR_VEMMC);
		udelay(1000); /* wait for voltage off */
#endif
		regulator_set_voltage(REGULATOR_VEMMC, 2900);
		regulator_enable(REGULATOR_VEMMC);
#endif /* CONFIG_TWL6030_POWER */
	}

#endif
}
#endif

#if defined(CONFIG_OMAP54XX) && defined(CONFIG_TWL6035_POWER)
static void omap5_pbias_config(struct mmc *mmc)
{
	u32 value = 0;
	struct omap_sys_ctrl_regs *const ctrl =
		(struct omap_sys_ctrl_regs *) SYSCTRL_GENERAL_CORE_BASE;

	value = readl(&ctrl->control_pbias);
	value &= ~(SDCARD_PWRDNZ | SDCARD_BIAS_PWRDNZ);
	value |= SDCARD_BIAS_HIZ_MODE;
	writel(value, &ctrl->control_pbias);

	twl6035_mmc1_poweron_ldo();

	value = readl(&ctrl->control_pbias);
	value &= ~SDCARD_BIAS_HIZ_MODE;
	value |= SDCARD_PBIASLITE_VMODE | SDCARD_PWRDNZ | SDCARD_BIAS_PWRDNZ;
	writel(value, &ctrl->control_pbias);

	value = readl(&ctrl->control_pbias);
	if (value & (1 << 23)) {
		value &= ~(SDCARD_PWRDNZ | SDCARD_BIAS_PWRDNZ);
		value |= SDCARD_BIAS_HIZ_MODE;
		writel(value, &ctrl->control_pbias);
	}
}
#endif

unsigned char mmc_board_init(struct mmc *mmc)
{
#if defined(CONFIG_OMAP34XX)
	t2_t *t2_base = (t2_t *)T2_BASE;
	struct prcm *prcm_base = (struct prcm *)PRCM_BASE;
	u32 pbias_lite;

	pbias_lite = readl(&t2_base->pbias_lite);
	pbias_lite &= ~(PBIASLITEPWRDNZ1 | PBIASLITEPWRDNZ0);
	writel(pbias_lite, &t2_base->pbias_lite);
#endif
#if defined(CONFIG_TWL4030_POWER)
	twl4030_power_mmc_init();
	mdelay(100);	/* ramp-up delay from Linux code */
#endif
#if defined(CONFIG_OMAP34XX)
	writel(pbias_lite | PBIASLITEPWRDNZ1 |
		PBIASSPEEDCTRL0 | PBIASLITEPWRDNZ0,
		&t2_base->pbias_lite);

	writel(readl(&t2_base->devconf0) | MMCSDIO1ADPCLKISEL,
		&t2_base->devconf0);

	writel(readl(&t2_base->devconf1) | MMCSDIO2ADPCLKISEL,
		&t2_base->devconf1);

	/* Change from default of 52MHz to 26MHz if necessary */
	if (!(mmc->host_caps & MMC_MODE_HS_52MHz))
		writel(readl(&t2_base->ctl_prog_io1) & ~CTLPROGIO1SPEEDCTRL,
			&t2_base->ctl_prog_io1);

	writel(readl(&prcm_base->fclken1_core) |
		EN_MMC1 | EN_MMC2 | EN_MMC3,
		&prcm_base->fclken1_core);

	writel(readl(&prcm_base->iclken1_core) |
		EN_MMC1 | EN_MMC2 | EN_MMC3,
		&prcm_base->iclken1_core);
#endif

#if defined(CONFIG_OMAP44XX) && defined(CONFIG_TWL6030_POWER)
#if 0
	/* PBIAS config needed for MMC1 only */
	if (mmc->block_dev.dev == 0)
#endif
		omap4_vmmc_pbias_config(mmc);
#endif
#if defined(CONFIG_OMAP54XX) && defined(CONFIG_TWL6035_POWER)
	if (mmc->block_dev.dev == 0)
		omap5_pbias_config(mmc);
#endif

	return 0;
}

/*
 * Start clock to the card
 */
static void omap_hsmmc_start_clock(struct hsmmc *mmc_base)
{
	mmc_reg_out(&mmc_base->sysctl, CEN_MASK, CEN_ENABLE);
}

/*
 * Stop clock to the card
 */
static void omap_hsmmc_stop_clock(struct hsmmc *mmc_base)
{
	mmc_reg_out(&mmc_base->sysctl, CEN_MASK, CEN_DISABLE);
	if (readl(&mmc_base->sysctl) & CEN_ENABLE)
		puts("MMC clk not stopped\n");
}

static void omap_hsmmc_enable_irq(struct hsmmc *mmc_base, struct mmc_cmd *cmd)
{
	unsigned int irq_mask = INT_EN_MASK;

	/* Disable timeout for erases */
	if (cmd && cmd->cmdidx == MMC_CMD_ERASE)
		irq_mask &= ~IE_DTO;

	writel(0xFFFFFFFF, &mmc_base->stat);
	(void)readl(&mmc_base->stat); /* Flush the post write */
	writel(irq_mask, &mmc_base->ise);
	writel(irq_mask, &mmc_base->ie);
}

static void omap_hsmmc_disable_irq(struct hsmmc *mmc_base)
{
	writel(0, &mmc_base->ise);
	writel(0, &mmc_base->ie);
	writel(0xFFFFFFFF, &mmc_base->stat);
	(void)readl(&mmc_base->stat); /* Flush the post write */
}

/* Calculate divisor for the given clock frequency */
static inline u16 calc_divisor(struct mmc *mmc)
{
	uint clock = mmc->clock;
	u16 dsor = 0;

	if (clock > mmc->f_max)
		clock = mmc->f_max;

	if (clock < mmc->f_min)
		clock = mmc->f_min;

	dsor = DIV_ROUND_UP(MMC_CLOCK_REFERENCE * 1000000, clock);
	if (dsor > 250)
		dsor = 250;

	return dsor;
}

static void omap_hsmmc_set_clock(struct mmc *mmc)
{
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	unsigned long regval;
	unsigned long clkdiv;
	ulong start;

	omap_hsmmc_stop_clock(mmc_base);

	clkdiv = calc_divisor(mmc);

	regval = readl(&mmc_base->sysctl);
	regval = regval & ~(CLKD_MASK | DTO_MASK);
	regval = regval | (clkdiv << 6) | DTO_15THDTO;
	writel(regval, &mmc_base->sysctl);
	writel(readl(&mmc_base->sysctl) | ICE_MASK, &mmc_base->sysctl);

	/* Wait till the ICS bit is set */
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & ICS_MASK) != ICS_MASK) {
		if (get_timer(start) > HSMMC_MAX_RETRY_MS_TIMEOUT)
			break;
	}
	if ((readl(&mmc_base->sysctl) & ICS_MASK) != ICS_MASK)
		printf("%s: ics timedout\n", __func__);

#if defined(CONFIG_AM33XX)
	/* Don't enable HSPE on OMAP4 */
	/*
	 * Enable High-Speed Support
	 * Pre-Requisites
	 *	- Controller should support High-Speed-Enable Bit
	 *	- Controller should not be using DDR Mode
	 *	- Controller should advertise that it supports High Speed
	 *	  in capabilities register
	 *	- MMC/SD clock coming out of controller > 25MHz
	 */
	if ((readl(&mmc_base->capa) & HSS_SUPPORT) == HSS_SUPPORT) {
		regval = readl(&mmc_base->hctl);
		if (clkdiv && ((MMC_CLOCK_REFERENCE * 1000000) / clkdiv) > 25000000)
			regval |= HSPE_ENABLE;
		else
			regval &= ~HSPE_ENABLE;
		writel(regval, &mmc_base->hctl);
	}
#endif /* CONFIG_AM33XX */

	omap_hsmmc_start_clock(mmc_base);
}

static void omap_hsmmc_set_bus_width(struct mmc *mmc)
{
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	u32 con;

	con = readl(&mmc_base->con);
	if (mmc->card_caps & MMC_MODE_DDR_52MHz)
		con |= DDR_ENABLE;
	else
		con &= ~DDR_ENABLE;
	/* configue bus width */
	switch (mmc->bus_width) {
	case 8:
		writel(con | DTW_8_BITMODE, &mmc_base->con);
		break;

	case 4:
		writel(con & ~DTW_8_BITMODE, &mmc_base->con);
		writel(readl(&mmc_base->hctl) | DTW_4_BITMODE,
			&mmc_base->hctl);
		break;

	case 1:
	default:
		writel(con & ~DTW_8_BITMODE, &mmc_base->con);
		writel(readl(&mmc_base->hctl) & ~DTW_4_BITMODE,
			&mmc_base->hctl);
		break;
	}
}

static void set_data_timeout(struct hsmmc *mmc_base)
{
	mmc_reg_out(&mmc_base->sysctl, DTO_MASK, DTO_15THDTO);
}

#if defined(CONFIG_OMAP44XX)
static int
omap_hsmmc_errata_i761(struct mmc *mmc, struct mmc_cmd *cmd)
{
	if ((cmd->resp_type & MMC_RSP_R1) == MMC_RSP_R1
			|| (IS_SD(mmc) && (cmd->resp_type & MMC_RSP_R5))) {
		struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
		u32 rsp10, csre;

		rsp10 = readl(&mmc_base->rsp10);
		csre = readl(&mmc_base->csre);
		return rsp10 & csre;
	}
	return 0;
}
#endif /* CONFIG_OMAP44XX */

#if 0
void mmc_init_stream(struct hsmmc *mmc_base)
{
	ulong start;

	writel(readl(&mmc_base->con) | INIT_INITSTREAM, &mmc_base->con);

	writel(MMC_CMD0, &mmc_base->cmd);
	start = get_timer(0);
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc!\n", __func__);
			return;
		}
	}
	writel(CC_MASK, &mmc_base->stat)
		;
	writel(MMC_CMD0, &mmc_base->cmd)
		;
	start = get_timer(0);
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc2!\n", __func__);
			return;
		}
	}
	writel(readl(&mmc_base->con) & ~INIT_INITSTREAM, &mmc_base->con);
}
#else
static void mmc_init_stream(struct hsmmc *mmc_base)
{
	ulong start;
	unsigned int retry;

	omap_hsmmc_enable_irq(mmc_base, NULL);

	writel(readl(&mmc_base->con) | INIT_INITSTREAM, &mmc_base->con);
	writel(MMC_CMD0, &mmc_base->cmd);

	retry = HSMMC_WAIT_STAT_RETRY;
_retry_cc:
	start = get_timer(0);
	while (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (get_timer(start) > HSMMC_WAIT_STAT_MS_TIMEOUT)
			break;
	}
	if (!(readl(&mmc_base->stat) & CC_MASK)) {
		if (retry != HSMMC_WAIT_STAT_RETRY)
			printf("[%u]%s: cc timedout 0x%x\n", retry, __func__, readl(&mmc_base->stat));
		UART_FIFO_FLUSH_ONCE();
		if (--retry)
			goto _retry_cc;
	}

	writel(readl(&mmc_base->con) & ~INIT_INITSTREAM, &mmc_base->con);

	omap_hsmmc_disable_irq(mmc_base);
}
#endif

static int mmc_init_setup(struct mmc *mmc)
{
#if 0
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	unsigned int reg_val;
	unsigned int dsor;
	ulong start;

	mmc_board_init(mmc);

	writel(readl(&mmc_base->sysconfig) | MMC_SOFTRESET,
		&mmc_base->sysconfig);
	start = get_timer(0);
	while ((readl(&mmc_base->sysstatus) & RESETDONE) == 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for cc2!\n", __func__);
			return TIMEOUT;
		}
	}
	writel(readl(&mmc_base->sysctl) | SOFTRESETALL, &mmc_base->sysctl);
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & SOFTRESETALL) != 0x0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for softresetall!\n",
				__func__);
			return TIMEOUT;
		}
	}
	writel(DTW_1_BITMODE | SDBP_PWROFF | SDVS_3V0, &mmc_base->hctl);
	writel(readl(&mmc_base->capa) | VS30_3V0SUP | VS18_1V8SUP,
		&mmc_base->capa);

	reg_val = readl(&mmc_base->con) & RESERVED_MASK;

	writel(CTPL_MMC_SD | reg_val | WPP_ACTIVEHIGH | CDP_ACTIVEHIGH |
		MIT_CTO | DW8_1_4BITMODE | MODE_FUNC | STR_BLOCK |
		HR_NOHOSTRESP | INIT_NOINIT | NOOPENDRAIN, &mmc_base->con);

	dsor = 240;
	mmc_reg_out(&mmc_base->sysctl, (ICE_MASK | DTO_MASK | CEN_MASK),
		(ICE_STOP | DTO_15THDTO | CEN_DISABLE));
	mmc_reg_out(&mmc_base->sysctl, ICE_MASK | CLKD_MASK,
		(dsor << CLKD_OFFSET) | ICE_OSCILLATE);
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & ICS_MASK) == ICS_NOTREADY) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for ics!\n", __func__);
			return TIMEOUT;
		}
	}
	writel(readl(&mmc_base->sysctl) | CEN_ENABLE, &mmc_base->sysctl);

	writel(readl(&mmc_base->hctl) | SDBP_PWRON, &mmc_base->hctl);

	writel(IE_BADA | IE_CERR | IE_DEB | IE_DCRC | IE_DTO | IE_CIE |
		IE_CEB | IE_CCRC | IE_CTO | IE_BRR | IE_BWR | IE_TC | IE_CC,
		&mmc_base->ie);

	mmc_init_stream(mmc_base);
#else
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	u32 hctl, capa, value;
	ulong start;

	mmc_board_init(mmc);

	/* Wait for hardware reset */
	start = get_timer(0);
	while ((readl(&mmc_base->sysstatus) & RESETDONE) == 0) {
		if (get_timer(start) > HSMMC_MAX_RETRY_MS_TIMEOUT)
			break;
	}
	if ((readl(&mmc_base->sysstatus) & RESETDONE) == 0)
		printf("%s: reset1 timedout\n", __func__);

	/* Do software reset */
	writel(readl(&mmc_base->sysconfig) | MMC_SOFTRESET,
		&mmc_base->sysconfig);
	start = get_timer(0);
	while ((readl(&mmc_base->sysstatus) & RESETDONE) == 0) {
		if (get_timer(start) > HSMMC_MAX_RETRY_MS_TIMEOUT)
			break;
	}
	if ((readl(&mmc_base->sysstatus) & RESETDONE) == 0)
		printf("%s: reset2 timedout\n", __func__);

	/* SYSCTL[24] SRA bit has the same action on the design as the SOFTRESET bit */

	if (OMAP_HSMMC1_BASE == (u32)mmc_base) {
		hctl = SDVS_3V0;
		capa = VS30_3V0SUP | VS18_1V8SUP;
	} else {
		hctl = SDVS_1V8;
		capa = VS18_1V8SUP;
	}

	value = readl(&mmc_base->hctl) & ~SDVS_MASK;
	writel(value | hctl, &mmc_base->hctl);

	value = readl(&mmc_base->capa);
	writel(value | capa, &mmc_base->capa);

	/* Set the controller to AUTO IDLE mode */
	value = readl(&mmc_base->sysconfig) & ~MMC_SOFTRESET;
	writel(value | MMC_AUTOIDLE, &mmc_base->sysconfig);

	/* Set SD bus power bit */
	writel(readl(&mmc_base->hctl) | SDBP_PWRON, &mmc_base->hctl);
	start = get_timer(0);
	while (!(readl(&mmc_base->hctl) & SDBP_PWRON)) {
		if (get_timer(start) > HSMMC_MAX_RETRY_MS_TIMEOUT)
			break;
	}
	if (!(readl(&mmc_base->hctl) & SDBP_PWRON))
		printf("%s: SDBP timedout\n", __func__);

	omap_hsmmc_disable_irq(mmc_base);
#endif

	return 0;
}

/*
 * MMC controller internal finite state machine reset
 *
 * Used to reset command or data internal state machines, using respectively
 * SRC or SRD bit of SYSCTL register
 */
static void mmc_reset_controller_fsm(struct hsmmc *mmc_base, u32 bit)
{
#if 0
	ulong start;

	mmc_reg_out(&mmc_base->sysctl, bit, bit);

	/*
	 * CMD(DAT) lines reset procedures are slightly different
	 * for OMAP3 and OMAP4(AM335x,OMAP5,DRA7xx).
	 * According to OMAP3 TRM:
	 * Set SRC(SRD) bit in MMCHS_SYSCTL register to 0x1 and wait until it
	 * returns to 0x0.
	 * According to OMAP4(AM335x,OMAP5,DRA7xx) TRMs, CMD(DATA) lines reset
	 * procedure steps must be as follows:
	 * 1. Initiate CMD(DAT) line reset by writing 0x1 to SRC(SRD) bit in
	 *    MMCHS_SYSCTL register (SD_SYSCTL for AM335x).
	 * 2. Poll the SRC(SRD) bit until it is set to 0x1.
	 * 3. Wait until the SRC (SRD) bit returns to 0x0
	 *    (reset procedure is completed).
	 */
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX) || \
	defined(CONFIG_AM33XX)
	if (!(readl(&mmc_base->sysctl) & bit)) {
		start = get_timer(0);
		while (!(readl(&mmc_base->sysctl) & bit)) {
			if (get_timer(0) - start > MAX_RETRY_MS)
				return;
		}
	}
#endif
	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & bit) != 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for sysctl %x to clear\n",
				__func__, bit);
			return;
		}
	}
#else
	ulong start;

	mmc_reg_out(&mmc_base->sysctl, bit, bit);

	/*
	 * CMD(DAT) lines reset procedures are slightly different
	 * for OMAP3 and OMAP4(AM335x,OMAP5,DRA7xx).
	 * According to OMAP3 TRM:
	 * Set SRC(SRD) bit in MMCHS_SYSCTL register to 0x1 and wait until it
	 * returns to 0x0.
	 * According to OMAP4(AM335x,OMAP5,DRA7xx) TRMs, CMD(DATA) lines reset
	 * procedure steps must be as follows:
	 * 1. Initiate CMD(DAT) line reset by writing 0x1 to SRC(SRD) bit in
	 *    MMCHS_SYSCTL register (SD_SYSCTL for AM335x).
	 * 2. Poll the SRC(SRD) bit until it is set to 0x1.
	 * 3. Wait until the SRC (SRD) bit returns to 0x0
	 *    (reset procedure is completed).
	 */
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX) || \
	defined(CONFIG_AM33XX)
	if (!(readl(&mmc_base->sysctl) & bit)) {
		start = get_timer(0);
		while (!(readl(&mmc_base->sysctl) & bit)) {
			if (get_timer(start) > HSMMC_MAX_RETRY_MS_TIMEOUT)
				break;
		}
		if (!(readl(&mmc_base->sysctl) & bit))
			return;
	}
#endif

	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & bit) != 0) {
		if (get_timer(start) > HSMMC_MAX_RETRY_MS_TIMEOUT)
			break;
	}
	if ((readl(&mmc_base->sysctl) & bit) != 0)
		printf("%s: FSM bit %x timedout\n", __func__, bit);
#endif
}

static int mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			struct mmc_data *data)
{
#if 0
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	unsigned int flags, mmc_stat;
	ulong start;

	start = get_timer(0);
	while ((readl(&mmc_base->pstate) & (DATI_MASK | CMDI_MASK)) != 0) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting on cmd inhibit to clear\n",
					__func__);
			return TIMEOUT;
		}
	}
	writel(0xFFFFFFFF, &mmc_base->stat);
	start = get_timer(0);
	while (readl(&mmc_base->stat)) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for STAT (%x) to clear\n",
				__func__, readl(&mmc_base->stat));
			return TIMEOUT;
		}
	}
	/*
	 * CMDREG
	 * CMDIDX[13:8]	: Command index
	 * DATAPRNT[5]	: Data Present Select
	 * ENCMDIDX[4]	: Command Index Check Enable
	 * ENCMDCRC[3]	: Command CRC Check Enable
	 * RSPTYP[1:0]
	 *	00 = No Response
	 *	01 = Length 136
	 *	10 = Length 48
	 *	11 = Length 48 Check busy after response
	 */
	/* Delay added before checking the status of frq change
	 * retry not supported by mmc.c(core file)
	 */
	if (cmd->cmdidx == SD_CMD_APP_SEND_SCR)
		udelay(50000); /* wait 50 ms */

	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		flags = 0;
	else if (cmd->resp_type & MMC_RSP_136)
		flags = RSP_TYPE_LGHT136 | CICE_NOCHECK;
	else if (cmd->resp_type & MMC_RSP_BUSY)
		flags = RSP_TYPE_LGHT48B;
	else
		flags = RSP_TYPE_LGHT48;

	/* enable default flags */
	flags =	flags | (CMD_TYPE_NORMAL | CICE_NOCHECK | CCCE_NOCHECK |
			MSBS_SGLEBLK | ACEN_DISABLE | BCE_DISABLE | DE_DISABLE);

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= CCCE_CHECK;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= CICE_CHECK;

	if (data) {
		if ((cmd->cmdidx == MMC_CMD_READ_MULTIPLE_BLOCK) ||
			 (cmd->cmdidx == MMC_CMD_WRITE_MULTIPLE_BLOCK)) {
			flags |= (MSBS_MULTIBLK | BCE_ENABLE);
			data->blocksize = 512;
			writel(data->blocksize | (data->blocks << 16),
							&mmc_base->blk);
		} else
			writel(data->blocksize | NBLK_STPCNT, &mmc_base->blk);

		if (data->flags & MMC_DATA_READ)
			flags |= (DP_DATA | DDIR_READ);
		else
			flags |= (DP_DATA | DDIR_WRITE);
	}

	writel(cmd->cmdarg, &mmc_base->arg);
	udelay(20);		/* To fix "No status update" error on eMMC */
	writel((cmd->cmdidx << 24) | flags, &mmc_base->cmd);

	start = get_timer(0);
	do {
		mmc_stat = readl(&mmc_base->stat);
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s : timeout: No status update\n", __func__);
			return TIMEOUT;
		}
	} while (!mmc_stat);

	if ((mmc_stat & IE_CTO) != 0) {
		mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
		return TIMEOUT;
	} else if ((mmc_stat & ERRI_MASK) != 0)
		return -1;

	if (mmc_stat & CC_MASK) {
		writel(CC_MASK, &mmc_base->stat);
		if (cmd->resp_type & MMC_RSP_PRESENT) {
			if (cmd->resp_type & MMC_RSP_136) {
				/* response type 2 */
				cmd->response[3] = readl(&mmc_base->rsp10);
				cmd->response[2] = readl(&mmc_base->rsp32);
				cmd->response[1] = readl(&mmc_base->rsp54);
				cmd->response[0] = readl(&mmc_base->rsp76);
			} else
				/* response types 1, 1b, 3, 4, 5, 6 */
				cmd->response[0] = readl(&mmc_base->rsp10);
		}
	}

	if (data && (data->flags & MMC_DATA_READ)) {
		mmc_read_data(mmc_base,	data->dest,
				data->blocksize * data->blocks);
	} else if (data && (data->flags & MMC_DATA_WRITE)) {
		mmc_write_data(mmc_base, data->src,
				data->blocksize * data->blocks);
	}
#else
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	unsigned int flags, mmc_stat;
	ulong start;
	int end_cmd = 0, cmd_err = 0;
	unsigned int retry;

	retry = HSMMC_CLEAR_PSTATE_RETRY;
_retry_pstate:
	start = get_timer(0);
	while (readl(&mmc_base->pstate) & (DATI_MASK | CMDI_MASK)) {
		if (get_timer(start) > HSMMC_CLEAR_PSTATE_MS_TIMEOUT)
			break;
	}
	mmc_stat = readl(&mmc_base->pstate);
	if (mmc_stat & (DATI_MASK | CMDI_MASK)) {
		if (retry != HSMMC_CLEAR_PSTATE_RETRY)
			printf("[%u]%s: clear PSTAT 0x%x timedout\n", retry, __func__, mmc_stat);
		if ((mmc_stat & CMDI_MASK) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
		if ((mmc_stat & DATI_MASK) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		UART_FIFO_FLUSH_ONCE();
		if (--retry)
			goto _retry_pstate;
		else
			return TIMEOUT;
	}

	retry = HSMMC_CLEAR_STAT_RETRY;
_retry_empty_stat:
	writel(0xFFFFFFFF, &mmc_base->stat);
	mmc_stat = readl(&mmc_base->stat); /* Flush the post write */
	start = get_timer(0);
	while (readl(&mmc_base->stat)) {
		if (get_timer(start) > HSMMC_CLEAR_STAT_MS_TIMEOUT)
			break;
	}
	mmc_stat = readl(&mmc_base->stat);
	if (mmc_stat) {
		printf("[%u]%s: clear STAT 0x%x timedout\n", retry, __func__, mmc_stat);
		UART_FIFO_FLUSH_ONCE();
		if (--retry)
			goto _retry_empty_stat;
		else
			return TIMEOUT;
	}

	if (data && !data->blocks) {
		printf("%s: no block\n", __func__);
		return 0;
	}

	omap_hsmmc_enable_irq(mmc_base, cmd);

	/*
	 * CMDREG
	 * CMDIDX[13:8]	: Command index
	 * DATAPRNT[5]	: Data Present Select
	 * ENCMDIDX[4]	: Command Index Check Enable
	 * ENCMDCRC[3]	: Command CRC Check Enable
	 * RSPTYP[1:0]
	 *	00 = No Response
	 *	01 = Length 136
	 *	10 = Length 48
	 *	11 = Length 48 Check busy after response
	 */
#if 0
	/* Delay added before checking the status of frq change
	 * retry not supported by mmc.c(core file)
	 */
	if (cmd->cmdidx == SD_CMD_APP_SEND_SCR)
		udelay(50000); /* wait 50 ms */
#endif

	if (!(cmd->resp_type & MMC_RSP_PRESENT))
		flags = 0;
	else if (cmd->resp_type & MMC_RSP_136)
		flags = RSP_TYPE_LGHT136 | CICE_NOCHECK;
	else if (cmd->resp_type & MMC_RSP_BUSY)
		flags = RSP_TYPE_LGHT48B;
	else
		flags = RSP_TYPE_LGHT48;

	/* Sync the value of "flags" with Kernel omap_hsmmc.c */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		flags |= (0x3 << 22);

	if (data) {
		/* Sync BLK register value with Kernel omap_hsmmc.c */
		writel(data->blocksize | (data->blocks << 16), &mmc_base->blk);
		set_data_timeout(mmc_base);

		/* Sync the value of "flags" with Kernel omap_hsmmc.c */
		flags |= DP_DATA | MSBS_MULTIBLK | BCE_ENABLE;
		if (data->flags & MMC_DATA_READ) {
			flags |= DDIR_READ;
		} else {
			flags &= ~(DDIR_READ);
		}
	} else {
		/* Sync BLK register value with Kernel omap_hsmmc.c */
		writel(0, &mmc_base->blk);
		set_data_timeout(mmc_base);
	}

	writel(cmd->cmdarg, &mmc_base->arg);
	writel((cmd->cmdidx << 24) | flags, &mmc_base->cmd);

	retry = HSMMC_WAIT_STAT_RETRY;
_retry_stat:
	start = get_timer(0);
	while (readl(&mmc_base->stat) == 0) {
		if (get_timer(start) > HSMMC_WAIT_STAT_MS_TIMEOUT)
			break;
	}
	mmc_stat = readl(&mmc_base->stat);
	if (mmc_stat == 0) {
		if (retry != HSMMC_WAIT_STAT_RETRY)
			printf("[%u]%s: (cmd%d) no STAT timedout\n", retry, __func__, cmd->cmdidx);
		UART_FIFO_FLUSH_ONCE();
		if (--retry)
			goto _retry_stat;
		else {
			omap_hsmmc_disable_irq(mmc_base);
			return TIMEOUT;
		}
	}

	if (mmc_stat & CC_MASK) {
		writel(CC_MASK, &mmc_base->stat);
		end_cmd = 1;
	}
	if (mmc_stat & ERRI_MASK) {
		if (mmc_stat & (IE_CTO | IE_CCRC)) {
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
			if (mmc_stat & IE_CTO) {
#ifdef CONFIG_MMC_TRACE
				printf("%s: (cmd%d) STAT err CTO\n", __func__, cmd->cmdidx);
#endif
				cmd_err = TIMEOUT;
			} else {
				printf("%s: (cmd%d) STAT err 0x%x\n", __func__, cmd->cmdidx, mmc_stat);
				cmd_err = COMM_ERR;
			}
			end_cmd = 1;
			if (data)
				mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);
		} else if (!data) {
			if (mmc_stat & (IE_DTO | IE_DCRC | IE_DEB))
				mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

			printf("%s: (cmd%d) STAT err 0x%x\n", __func__, cmd->cmdidx, mmc_stat);
			cmd_err = COMM_ERR;
			end_cmd = 1;
		}
	}

#if defined(CONFIG_OMAP44XX)
	if (omap_hsmmc_errata_i761(mmc, cmd)) {
		printf("%s: (cmd%d) i761 CARD err 0x%x\n", __func__, cmd->cmdidx, mmc_stat);
		cmd_err = UNUSABLE_ERR;
		end_cmd = 1;
	}
#endif /* CONFIG_OMAP44XX */

	if (end_cmd) {
		if (cmd->resp_type & MMC_RSP_PRESENT) {
			if (cmd->resp_type & MMC_RSP_136) {
				/* response type 2 */
				cmd->response[3] = readl(&mmc_base->rsp10);
				cmd->response[2] = readl(&mmc_base->rsp32);
				cmd->response[1] = readl(&mmc_base->rsp54);
				cmd->response[0] = readl(&mmc_base->rsp76);
			} else
				/* response types 1, 1b, 3, 4, 5, 6 */
				cmd->response[0] = readl(&mmc_base->rsp10);
		}
		if (cmd_err) {
			omap_hsmmc_disable_irq(mmc_base);
			return cmd_err;
		}
	}

	if (data) {
		int ret = 0;
		if (data->flags & MMC_DATA_READ) {
			ret = mmc_read_data(mmc_base, data->dest,
						data->blocksize, data->blocks);
#ifndef CONFIG_SPL_BUILD
		} else if (data->flags & MMC_DATA_WRITE) {
			ret = mmc_write_data(mmc_base, data->src,
						data->blocksize, data->blocks);
#endif /* !CONFIG_SPL_BUILD */
		}
		if (ret) {
			omap_hsmmc_disable_irq(mmc_base);
			return ret;
		}
	}

	omap_hsmmc_disable_irq(mmc_base);
#endif

	return 0;
}

#if 0
static int mmc_read_data(struct hsmmc *mmc_base, char *buf, unsigned int size)
{
	unsigned int *output_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count;

	/*
	 * Start Polled Read
	 */
	count = (size > MMCSD_SECTOR_SIZE) ? MMCSD_SECTOR_SIZE : size;
	count /= 4;

	while (size) {
		ulong start = get_timer(0);
		do {
			mmc_stat = readl(&mmc_base->stat);
			if (get_timer(0) - start > MAX_RETRY_MS) {
				printf("%s: timedout waiting for status!\n",
						__func__);
				return TIMEOUT;
			}
		} while (mmc_stat == 0);

		if ((mmc_stat & (IE_DTO | IE_DCRC | IE_DEB)) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if ((mmc_stat & ERRI_MASK) != 0)
			return 1;

		if (mmc_stat & BRR_MASK) {
			unsigned int k;

			writel(readl(&mmc_base->stat) | BRR_MASK,
				&mmc_base->stat);
			for (k = 0; k < count; k++) {
				*output_buf = readl(&mmc_base->data);
				output_buf++;
			}
			size -= (count*4);
		}

		if (mmc_stat & BWR_MASK)
			writel(readl(&mmc_base->stat) | BWR_MASK,
				&mmc_base->stat);

		if (mmc_stat & TC_MASK) {
			writel(readl(&mmc_base->stat) | TC_MASK,
				&mmc_base->stat);
			break;
		}
	}
	return 0;
}
#else
static int mmc_read_data(struct hsmmc *mmc_base, char *buf,
			unsigned int blksize, unsigned int blks)
{
	unsigned int *output_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count = blksize >> 2;

	while (blks) {
		unsigned int retry;
		ulong start;

		retry = HSMMC_WAIT_STAT_RETRY;
_retry:
		start = get_timer(0);
		while (readl(&mmc_base->stat) == 0) {
			if (get_timer(start) > HSMMC_WAIT_STAT_MS_TIMEOUT)
				break;
		}
		mmc_stat = readl(&mmc_base->stat);
		if (mmc_stat == 0) {
			if (retry != HSMMC_WAIT_STAT_RETRY)
				printf("[%u]%s: no STAT timedout\n", retry, __func__);
			UART_FIFO_FLUSH_ONCE();
			if (--retry)
				goto _retry;
			else {
				mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);
				printf("%s: STAT err 0x%08x;0x%08x;%u\n", __func__,
						(u32)buf, (u32)output_buf, blks);
				return TIMEOUT;
			}
		}
		writel(mmc_stat, &mmc_base->stat);

		if (mmc_stat & (IE_CTO | IE_CCRC))
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
		if (mmc_stat & (IE_DTO | IE_DCRC | IE_DEB))
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if (mmc_stat & ERRI_MASK) {
			printf("%s: STAT err 0x%x\n", __func__, mmc_stat);
			(void)readl(&mmc_base->stat); /* Flush the post write */
			return COMM_ERR;
		}

		if (mmc_stat & BRR_MASK) {
			unsigned int k;

			for (k = 0; k < count; k++) {
				*output_buf = readl(&mmc_base->data);
				output_buf++;
			}
			blks--;
		}

		if (mmc_stat & TC_MASK) {
			(void)readl(&mmc_base->stat); /* Flush the post write */
			break;
		}
	}
	return 0;
}
#endif

#if 0
static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
				unsigned int size)
{
	unsigned int *input_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count;

	/*
	 * Start Polled Write
	 */
	count = (size > MMCSD_SECTOR_SIZE) ? MMCSD_SECTOR_SIZE : size;
	count /= 4;

	while (size) {
		ulong start = get_timer(0);
		do {
			mmc_stat = readl(&mmc_base->stat);
			if (get_timer(0) - start > MAX_RETRY_MS) {
				printf("%s: timedout waiting for status!\n",
						__func__);
				return TIMEOUT;
			}
		} while (mmc_stat == 0);

		if ((mmc_stat & (IE_DTO | IE_DCRC | IE_DEB)) != 0)
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if ((mmc_stat & ERRI_MASK) != 0)
			return 1;

		if (mmc_stat & BWR_MASK) {
			unsigned int k;

			writel(readl(&mmc_base->stat) | BWR_MASK,
					&mmc_base->stat);
			for (k = 0; k < count; k++) {
				writel(*input_buf, &mmc_base->data);
				input_buf++;
			}
			size -= (count*4);
		}

		if (mmc_stat & BRR_MASK)
			writel(readl(&mmc_base->stat) | BRR_MASK,
				&mmc_base->stat);

		if (mmc_stat & TC_MASK) {
			writel(readl(&mmc_base->stat) | TC_MASK,
				&mmc_base->stat);
			break;
		}
	}
	return 0;
}
#else
#ifndef CONFIG_SPL_BUILD
static int mmc_write_data(struct hsmmc *mmc_base, const char *buf,
			unsigned int blksize, unsigned int blks)
{
	unsigned int *input_buf = (unsigned int *)buf;
	unsigned int mmc_stat;
	unsigned int count = blksize >> 2;

	while (blks) {
		unsigned int retry;
		ulong start;

		retry = HSMMC_WAIT_STAT_RETRY;
_retry:
		start = get_timer(0);
		while (readl(&mmc_base->stat) == 0) {
			if (get_timer(start) > HSMMC_WAIT_STAT_MS_TIMEOUT)
				break;
		}
		mmc_stat = readl(&mmc_base->stat);
		if (mmc_stat == 0) {
			if (retry != HSMMC_WAIT_STAT_RETRY)
				printf("[%u]%s: no STAT timedout\n", retry, __func__);
			UART_FIFO_FLUSH_ONCE();
			if (--retry)
				goto _retry;
			else {
				mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);
				printf("%s: STAT err 0x%08x;0x%08x;%u\n", __func__,
						(u32)buf, (u32)input_buf, blks);
				return TIMEOUT;
			}
		}
		writel(mmc_stat, &mmc_base->stat);

		if (mmc_stat & (IE_CTO | IE_CCRC))
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRC);
		if (mmc_stat & (IE_DTO | IE_DCRC | IE_DEB))
			mmc_reset_controller_fsm(mmc_base, SYSCTL_SRD);

		if (mmc_stat & ERRI_MASK) {
			printf("%s: STAT err 0x%x\n", __func__, mmc_stat);
			(void)readl(&mmc_base->stat); /* Flush the post write */
			return COMM_ERR;
		}

		if (mmc_stat & BWR_MASK) {
			unsigned int k;

			for (k = 0; k < count; k++) {
				writel(*input_buf, &mmc_base->data);
				input_buf++;
			}
			blks--;
		}

		if (mmc_stat & TC_MASK) {
			(void)readl(&mmc_base->stat); /* Flush the post write */
			break;
		}
	}
	return 0;
}
#endif /* !CONFIG_SPL_BUILD */
#endif

static void mmc_set_ios(struct mmc *mmc)
{
#if 0
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;
	unsigned int dsor = 0;
	ulong start;

	/* configue bus width */
	switch (mmc->bus_width) {
	case 8:
		writel(readl(&mmc_base->con) | DTW_8_BITMODE,
			&mmc_base->con);
		break;

	case 4:
		writel(readl(&mmc_base->con) & ~DTW_8_BITMODE,
			&mmc_base->con);
		writel(readl(&mmc_base->hctl) | DTW_4_BITMODE,
			&mmc_base->hctl);
		break;

	case 1:
	default:
		writel(readl(&mmc_base->con) & ~DTW_8_BITMODE,
			&mmc_base->con);
		writel(readl(&mmc_base->hctl) & ~DTW_4_BITMODE,
			&mmc_base->hctl);
		break;
	}

	/* configure clock with 96Mhz system clock.
	 */
	if (mmc->clock != 0) {
		dsor = (MMC_CLOCK_REFERENCE * 1000000 / mmc->clock);
		if ((MMC_CLOCK_REFERENCE * 1000000) / dsor > mmc->clock)
			dsor++;
	}

	mmc_reg_out(&mmc_base->sysctl, (ICE_MASK | DTO_MASK | CEN_MASK),
				(ICE_STOP | DTO_15THDTO | CEN_DISABLE));

	mmc_reg_out(&mmc_base->sysctl, ICE_MASK | CLKD_MASK,
				(dsor << CLKD_OFFSET) | ICE_OSCILLATE);

	start = get_timer(0);
	while ((readl(&mmc_base->sysctl) & ICS_MASK) == ICS_NOTREADY) {
		if (get_timer(0) - start > MAX_RETRY_MS) {
			printf("%s: timedout waiting for ics!\n", __func__);
			return;
		}
	}
	writel(readl(&mmc_base->sysctl) | CEN_ENABLE, &mmc_base->sysctl);
#else
	static int do_send_init_stream = 1;
	struct hsmmc *mmc_base = (struct hsmmc *)mmc->priv;

#if defined(CONFIG_AM33XX)
{
	u32 reg_val;
	/*
	* Set
	*	Debounce filter value to max
	*	Write protect polarity to Active low level
	*/
	reg_val = readl(&mmc_base->con);
	reg_val &= ~(DVAL_MASK | WPP_MASK);
	reg_val |= (DVAL_MAX | WPP_ACT_LOW);
	writel(reg_val, &mmc_base->con);
}
#endif /* CONFIG_AM33XX */

	omap_hsmmc_set_bus_width(mmc);

	omap_hsmmc_set_clock(mmc);

	if (do_send_init_stream) {
		do_send_init_stream = 0;

		omap_hsmmc_enable_irq(mmc_base, NULL);
		mmc_init_stream(mmc_base);
		omap_hsmmc_disable_irq(mmc_base);
	}
#endif
}

int omap_mmc_init(int dev_index, uint host_caps_mask, uint f_max)
{
	struct mmc *mmc;
	uint host_caps_val = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS |
			     MMC_MODE_HC | MMC_MODE_WAIT_WHILE_BUSY;

	mmc = &hsmmc_dev[dev_index];

#if 0
	sprintf(mmc->name, "OMAP SD/MMC");
#else
	sprintf(mmc->name, "hsmmc%d", dev_index + 1);
#endif
	mmc->send_cmd = mmc_send_cmd;
	mmc->set_ios = mmc_set_ios;
	mmc->init = mmc_init_setup;
	mmc->getcd = NULL;

	switch (dev_index) {
	case 0:
		mmc->priv = (struct hsmmc *)OMAP_HSMMC1_BASE;
		break;
#ifdef OMAP_HSMMC2_BASE
	case 1:
		mmc->priv = (struct hsmmc *)OMAP_HSMMC2_BASE;
		/* Enable 8-bit interface for eMMC */
		host_caps_val |= MMC_MODE_8BIT;
#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_OMAP44XX)
		host_caps_val |= MMC_MODE_DDR_52MHz;
#endif
		break;
#endif
#ifdef OMAP_HSMMC3_BASE
	case 2:
		mmc->priv = (struct hsmmc *)OMAP_HSMMC3_BASE;
		break;
#endif
	default:
		mmc->priv = (struct hsmmc *)OMAP_HSMMC1_BASE;
		return 1;
	}
#if 0
	mmc->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	mmc->host_caps = (MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS |
				MMC_MODE_HC) & ~host_caps_mask;
#else
	mmc->voltages = MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32;
	mmc->host_caps = host_caps_val & ~host_caps_mask;
#endif

	mmc->f_min = 400000;

	if (f_max != 0)
		mmc->f_max = f_max;
	else {
		if (mmc->host_caps & MMC_MODE_HS) {
			if (mmc->host_caps & MMC_MODE_HS_52MHz)
				mmc->f_max = 52000000;
			else
				mmc->f_max = 26000000;
		} else
			mmc->f_max = 20000000;
	}

	mmc->b_max = 0;

#if defined(CONFIG_OMAP34XX)
	/*
	 * Silicon revs 2.1 and older do not support multiblock transfers.
	 */
	if ((get_cpu_family() == CPU_OMAP34XX) && (get_cpu_rev() <= CPU_3XX_ES21))
		mmc->b_max = 1;
#endif

	mmc_register(mmc);

	return 0;
}
