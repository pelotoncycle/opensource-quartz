/*
 * Driver for OMAP4 McSPI
 *
 * (C) Copyright 2013
 * InnoComm Mobile Technology Corp., <www.innocomm.com>
 *
 * Author :
 *  James Wu <james.wu@innocomm.com>
 *
 * Codes and ideas taken from linux/drivers/spi/omap2_mcspi.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <malloc.h>
#include <linux/compat.h>
#include <compiler.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mcspi.h>

/*----------------------------------------------------------------------*/

#ifndef barrier
/* Optimization barrier */
/* The "volatile" is due to gcc bugs */
#define barrier() __asm__ __volatile__("": : :"memory")
#endif

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

/*----------------------------------------------------------------------*/

/*#define CONFIG_OMAP2_MCSPI_DEBUG*/
/*#define CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_OMAP2_MCSPI_DEBUG
#define CONFIG_OMAP2_MCSPI_DEBUG
#endif /* CONFIG_OMAP2_MCSPI_DEBUG */
#endif /* DEBUG */

/*----------------------------------------------------------------------*/

#ifdef CONFIG_OMAP2_MCSPI_DEBUG
#define MCSPI_CS_PRINT(spi, x...)	{printf("McSPI%u<cs%u>: ",spi->slave.bus,spi->slave.cs); printf(x);}
#define MCSPI_PRINT(spi, x...)	{printf("McSPI%u: ",spi->slave.bus); printf(x);}
#define MCSPI_BUGPRINT(_expr) \
		printf("(" #_expr "): failure at " __FILE__ "#%d:%s()!\n", __LINE__, __FUNCTION__); \
		panic("BUG!");
#define MCSPI_BUG_ON(expr) if(!!(expr)){MCSPI_BUGPRINT(expr);}
#else /* CONFIG_OMAP2_MCSPI_DEBUG */
#define MCSPI_CS_PRINT(spi, x...) do{}while(0)
#define MCSPI_PRINT(spi, x...) do{}while(0)
#define MCSPI_BUGPRINT(_expr) do{}while(0)
#define MCSPI_BUG_ON(expr) do{}while(0)
#endif /* CONFIG_OMAP2_MCSPI_DEBUG */

#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
#define MCSPI_CS_VPRINT(spi, x...)	{printf("McSPI%u<cs%u>: ",spi->slave.bus,spi->slave.cs); printf(x);}
#define MCSPI_VPRINT(x...) printf("McSPI: " x)
#else /* CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG */
#define MCSPI_CS_VPRINT(spi, x...) do{}while(0)
#define MCSPI_VPRINT(x...) do{}while(0)
#endif /* CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG */

#define MCSPI_ERRPRINT(spi, x...)	{printf("McSPI%u<cs%u>: ",spi->slave.bus,spi->slave.cs); printf(x);}
#define MCSPI_ERR(x...)	{printf("McSPI: " x);}

/*----------------------------------------------------------------------*/

/* McSPI registers (offset) */
#define OMAP2_MCSPI_REVISION		0x00
#define OMAP2_MCSPI_SYSCONFIG		0x10
#define OMAP2_MCSPI_SYSSTATUS		0x14
#define OMAP2_MCSPI_IRQSTATUS		0x18
#define OMAP2_MCSPI_IRQENABLE		0x1c
#define OMAP2_MCSPI_WAKEUPENABLE	0x20
#define OMAP2_MCSPI_SYST			0x24
#define OMAP2_MCSPI_MODULCTRL		0x28
#define OMAP2_MCSPI_XFERLEVEL		0x7c

/* per-channel banks, 0x14 bytes each, first is: */
#define OMAP2_MCSPI_CHCONF0		0x2C
#define OMAP2_MCSPI_CHSTAT0		0x30
#define OMAP2_MCSPI_CHCTRL0		0x34
#define OMAP2_MCSPI_TX0			0x38
#define OMAP2_MCSPI_RX0			0x3C

/* per-register bit values & masks */
/* OMAP2_MCSPI_SYSCONFIG */
#define OMAP2_MCSPI_SYSCONFIG_SMARTIDLE	BIT(4)
#define OMAP2_MCSPI_SYSCONFIG_ENAWAKEUP	BIT(2)
#define OMAP2_MCSPI_SYSCONFIG_AUTOIDLE	BIT(0)
#define OMAP2_MCSPI_SYSCONFIG_SOFTRESET	BIT(1)
/* OMAP2_MCSPI_SYSSTATUS */
#define OMAP2_MCSPI_SYSSTATUS_RESETDONE	BIT(0)
/* OMAP2_MCSPI_IRQSTATUS */
#define OMAP2_MCSPI_IRQSTATUS_EOW				(1 << 17)
#define OMAP2_MCSPI_IRQSTATUS_WKS				(1 << 16)
#define OMAP2_MCSPI_IRQSTATUS_RX3_OVERFLOW	(1 << 15)
#define OMAP2_MCSPI_IRQSTATUS_RX3_FULL		(1 << 14)
#define OMAP2_MCSPI_IRQSTATUS_TX3_UNDERFLOW	(1 << 13)
#define OMAP2_MCSPI_IRQSTATUS_TX3_EMPTY		(1 << 12)
#define OMAP2_MCSPI_IRQSTATUS_RX2_OVERFLOW	(1 << 11)
#define OMAP2_MCSPI_IRQSTATUS_RX2_FULL		(1 << 10)
#define OMAP2_MCSPI_IRQSTATUS_TX2_UNDERFLOW	(1 << 9)
#define OMAP2_MCSPI_IRQSTATUS_TX2_EMPTY		(1 << 8)
#define OMAP2_MCSPI_IRQSTATUS_RX1_OVERFLOW	(1 << 7)
#define OMAP2_MCSPI_IRQSTATUS_RX1_FULL		(1 << 6)
#define OMAP2_MCSPI_IRQSTATUS_TX1_UNDERFLOW	(1 << 5)
#define OMAP2_MCSPI_IRQSTATUS_TX1_EMPTY		(1 << 4)
#define OMAP2_MCSPI_IRQSTATUS_RX0_OVERFLOW	(1 << 3)
#define OMAP2_MCSPI_IRQSTATUS_RX0_FULL		(1 << 2)
#define OMAP2_MCSPI_IRQSTATUS_TX0_UNDERFLOW	(1 << 1)
#define OMAP2_MCSPI_IRQSTATUS_TX0_EMPTY		(1 << 0)

/* OMAP2_MCSPI_MODULCTRL */
#define OMAP2_MCSPI_MODULCTRL_SINGLE	BIT(0)
#define OMAP2_MCSPI_MODULCTRL_MS	BIT(2)
#define OMAP2_MCSPI_MODULCTRL_STEST	BIT(3)

#define OMAP2_MCSPI_CHCONF_PHA		BIT(0)
#define OMAP2_MCSPI_CHCONF_POL		BIT(1)
#define OMAP2_MCSPI_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP2_MCSPI_CHCONF_EPOL		BIT(6)
#define OMAP2_MCSPI_CHCONF_WL_MASK	(0x1f << 7)
#define OMAP2_MCSPI_CHCONF_TRM_RX_ONLY	BIT(12)
#define OMAP2_MCSPI_CHCONF_TRM_TX_ONLY	BIT(13)
#define OMAP2_MCSPI_CHCONF_TRM_MASK	(0x03 << 12)
#define OMAP2_MCSPI_CHCONF_DMAW		BIT(14)
#define OMAP2_MCSPI_CHCONF_DMAR		BIT(15)
#define OMAP2_MCSPI_CHCONF_DPE0		BIT(16)
#define OMAP2_MCSPI_CHCONF_DPE1		BIT(17)
#define OMAP2_MCSPI_CHCONF_IS		BIT(18)
#define OMAP2_MCSPI_CHCONF_TURBO	BIT(19)
#define OMAP2_MCSPI_CHCONF_FORCE	BIT(20)


#define OMAP2_MCSPI_CHSTAT_RXS		BIT(0)
#define OMAP2_MCSPI_CHSTAT_TXS		BIT(1)
#define OMAP2_MCSPI_CHSTAT_EOT		BIT(2)

#define OMAP2_MCSPI_CHCTRL_EN		BIT(0)

#define OMAP2_MCSPI_WAKEUPENABLE_WKEN	BIT(0)

/*----------------------------------------------------------------------*/

struct omap2_mcspi_platform {
	unsigned short num_cs;
	void* bus_base;
	int initiated;
	int clk_reference;
#if defined(CONFIG_OMAP34XX)
	/* OMAP3 clock registers */
	const unsigned int	*fclk_addr;
	const unsigned int	*iclk_addr;
	const unsigned int	*idlest_addr;
	const unsigned int	clk_bit;
#error "Not Implemented"
#elif defined(CONFIG_OMAP44XX)
	/* OMAP4 clock register */
	const unsigned int	*clkctrl_addr;
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif
};

struct omap2_mcspi {
	struct spi_slave slave;
	unsigned int bus;
	unsigned int mode;
	unsigned char bits_per_word;
	void* bus_base;
	void* cs_base;
	unsigned int clk_divisor;

	/* Context save and restore shadow register */
	u32	chconf0;
	u32 sysconfig;
	u32 modulctrl;
	u32 wakeupenable;
};

/*----------------------------------------------------------------------*/

static struct omap2_mcspi_platform mcspis[OMAP2_MCSPI_MAX_CTRL] __attribute__ ((section (".data"))) = {
#if defined(CONFIG_OMAP34XX)
#error "Not Implemented"
#elif defined(CONFIG_OMAP44XX)
	[0] = {
		.num_cs			= 4,
		.bus_base		= (void*) MCSPI1_BASE,
		.clkctrl_addr	= (const unsigned int*)0x4A0094F0, /* CM_L4PER_MCSPI1_CLKCTRL */
		.initiated		= 0,
		.clk_reference	= 0,
	},
	[1] = {
		.num_cs			= 2,
		.bus_base		= (void*) MCSPI2_BASE,
		.clkctrl_addr	= (const unsigned int*)0x4A0094F8, /* CM_L4PER_MCSPI2_CLKCTRL */
		.initiated		= 0,
		.clk_reference	= 0,
	},
	[2] = {
		.num_cs			= 1,
		.bus_base		= (void*) MCSPI3_BASE,
		.clkctrl_addr	= (const unsigned int*)0x4A009500, /* CM_L4PER_MCSPI3_CLKCTRL */
		.initiated		= 0,
		.clk_reference	= 0,
	},
	[3] = {
		.num_cs			= 1,
		.bus_base		= (void*) MCSPI4_BASE,
		.clkctrl_addr	= (const unsigned int*)0x4A009508, /* CM_L4PER_MCSPI4_CLKCTRL */
		.initiated		= 0,
		.clk_reference	= 0,
	},
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif
};

/*----------------------------------------------------------------------*/

static inline void _mcspi_disable_clocks(struct omap2_mcspi_platform *plat)
{
#if defined(CONFIG_OMAP34XX)
#error "Not Implemented"
#elif defined(CONFIG_OMAP44XX)
	if (plat->clkctrl_addr)
		omap_clk_disable((u32 *)plat->clkctrl_addr);
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif
}

static inline void _mcspi_enable_clocks(struct omap2_mcspi_platform *plat)
{
#if defined(CONFIG_OMAP34XX)
#error "Not Implemented"
#elif defined(CONFIG_OMAP44XX)
	if (plat->clkctrl_addr)
		omap_clk_enable((u32 *)plat->clkctrl_addr, MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented"
#else
#error "Not Implemented"
#endif
}

/*----------------------------------------------------------------------*/

static inline struct omap2_mcspi* _to_omap2_mcspi(struct spi_slave *slave)
{
	return container_of(slave, struct omap2_mcspi, slave);
}

static void omap2_mcspi_disable_clocks(struct omap2_mcspi *mcspi)
{
	struct omap2_mcspi_platform *plat = &(mcspis[mcspi->bus]);

	MCSPI_VPRINT("%s\n", __func__);

	plat->clk_reference--;
	if (plat->clk_reference == 0) {
		MCSPI_PRINT(mcspi, "disable clocks\n");
		_mcspi_disable_clocks(plat);
	}
}

static int omap2_mcspi_enable_clocks(struct omap2_mcspi *mcspi)
{
	struct omap2_mcspi_platform *plat = &(mcspis[mcspi->bus]);
	int ret = 0;

	MCSPI_VPRINT("%s\n", __func__);

	if (plat->clk_reference == 0) {
		MCSPI_PRINT(mcspi, "enable clocks\n");
		_mcspi_enable_clocks(plat);
		ret = 1;
	}
	plat->clk_reference++;

	return ret;
}

/*----------------------------------------------------------------------*/

#define MOD_REG_BIT(val, mask, set) do { \
	if (set) \
		val |= mask; \
	else \
		val &= ~mask; \
} while (0)

static inline void mcspi_write_reg(struct omap2_mcspi *mcspi,
		int idx, u32 val)
{
#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
	printf("%s: 0x%08x=0x%08x\n", __func__, (u32)(mcspi->bus_base + idx), val);
#endif
	__raw_writel(val, mcspi->bus_base + idx);
}

static inline u32 mcspi_read_reg(struct omap2_mcspi *mcspi, int idx)
{
#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
	u32 val = __raw_readl(mcspi->bus_base + idx);
	printf("%s: 0x%08x=0x%08x\n", __func__, (u32)(mcspi->bus_base + idx), val);
	return val;
#else
	return __raw_readl(mcspi->bus_base + idx);
#endif
}

static inline void mcspi_write_cs_reg(struct omap2_mcspi *mcspi,
		int idx, u32 val)
{
#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
	printf("%s: 0x%08x=0x%08x\n", __func__, (u32)(mcspi->cs_base + idx), val);
#endif
	__raw_writel(val, mcspi->cs_base +  idx);
}

static inline u32 mcspi_read_cs_reg(struct omap2_mcspi *mcspi, int idx)
{
#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
	u32 val = __raw_readl(mcspi->cs_base + idx);
	printf("%s: 0x%08x=0x%08x\n", __func__, (u32)(mcspi->cs_base + idx), val);
	return val;
#else
	return __raw_readl(mcspi->cs_base + idx);
#endif
}

static inline unsigned int mcspi_cached_chconf0(struct omap2_mcspi *spi)
{
#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
	printf("%s: chconf0=0x%08x\n", __func__, spi->chconf0);
#endif
	return spi->chconf0;
}

static inline void mcspi_write_chconf0(struct omap2_mcspi *spi, unsigned int value)
{
	spi->chconf0 = value;
#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
	printf("%s: chconf0=0x%08x\n", __func__, spi->chconf0);
#endif
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, value);
	mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
}

/*----------------------------------------------------------------------*/

static void omap2_mcspi_restore_ctx(struct omap2_mcspi *spi)
{
	MCSPI_VPRINT("%s\n", __func__);

	/* McSPI: context restore */
	mcspi_write_reg(spi, OMAP2_MCSPI_MODULCTRL, spi->modulctrl);

	mcspi_write_reg(spi, OMAP2_MCSPI_SYSCONFIG, spi->sysconfig);

	mcspi_write_reg(spi, OMAP2_MCSPI_WAKEUPENABLE, spi->wakeupenable);

	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, spi->chconf0);
}

static void omap2_mcspi_runtime_resume(struct omap2_mcspi *spi)
{
	MCSPI_VPRINT("%s\n", __func__);

	if (omap2_mcspi_enable_clocks(spi)) {
		omap2_mcspi_restore_ctx(spi);
	}
}

static inline void omap2_mcspi_master_setup(struct omap2_mcspi *spi)
{
	u32 l;

	MCSPI_VPRINT("%s\n", __func__);

	/* enable wakeup */
	mcspi_write_reg(spi, OMAP2_MCSPI_WAKEUPENABLE, OMAP2_MCSPI_WAKEUPENABLE_WKEN);
	spi->wakeupenable = OMAP2_MCSPI_WAKEUPENABLE_WKEN;

	MCSPI_CS_VPRINT(spi, "setup single-channel master mode\n");
	/* setup when switching from (reset default) slave mode
	 * to single-channel master mode
	 */
	l = mcspi_read_reg(spi, OMAP2_MCSPI_MODULCTRL);
	MOD_REG_BIT(l, OMAP2_MCSPI_MODULCTRL_STEST, 0);
	MOD_REG_BIT(l, OMAP2_MCSPI_MODULCTRL_MS, 0);
	MOD_REG_BIT(l, OMAP2_MCSPI_MODULCTRL_SINGLE, 1);
	mcspi_write_reg(spi, OMAP2_MCSPI_MODULCTRL, l);

	spi->modulctrl = l;
}

static int omap2_mcspi_reset(struct omap2_mcspi *spi)
{
	int ret;

	MCSPI_VPRINT("%d: SOFT RESET\n", spi->slave.bus);
	/* Refer to Kernel (Kernel 3.0 uses omap_hwmod_sysc_type2) */
#if 0
	ret = ocp_softreset((u32)(spi->bus_base + OMAP2_MCSPI_SYSCONFIG),
				(u32)(spi->bus_base + OMAP2_MCSPI_SYSSTATUS),
				SYSC_AUTOIDLE | SYSC_ENWAKEUP | SYSC_SIDLE_SMART);
	spi->sysconfig = SYSC_AUTOIDLE | SYSC_ENWAKEUP | SYSC_SIDLE_SMART;
#else
	ret = ocp_softreset_type2((u32)(spi->bus_base + OMAP2_MCSPI_SYSCONFIG),
				TYPE2_SYSC_SIDLE_SMART_WAKEUP);
	spi->sysconfig = TYPE2_SYSC_SIDLE_SMART_WAKEUP;
#endif
	if (ret) {
		MCSPI_ERRPRINT(spi, "reset err %d (SYSS 0x%08x, SYSC 0x%08x)\n", ret,
			mcspi_read_reg(spi, OMAP2_MCSPI_SYSSTATUS), mcspi_read_reg(spi, OMAP2_MCSPI_SYSCONFIG));
	} else {
		MCSPI_VPRINT("%d: SYSC 0x%08x=0x%08x (0x%08x)\n", spi->slave.bus,
			(u32)(spi->bus_base + OMAP2_MCSPI_SYSCONFIG),
			mcspi_read_reg(spi, OMAP2_MCSPI_SYSCONFIG),
			spi->sysconfig);
	}

	return ret;
}

/*----------------------------------------------------------------------*/

static inline void omap2_mcspi_set_enable(struct omap2_mcspi *spi, int enable)
{
	u32 l;

	MCSPI_VPRINT("%s: %d\n", __func__, enable);

	l = enable ? OMAP2_MCSPI_CHCTRL_EN : 0;
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCTRL0, l);
	/* Flash post-writes */
	mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCTRL0);
}

static inline void omap2_mcspi_force_cs(struct omap2_mcspi *spi, int cs_active)
{
	u32 l;

	MCSPI_VPRINT("%s: %d\n", __func__, cs_active);

	l = mcspi_cached_chconf0(spi);
	MOD_REG_BIT(l, OMAP2_MCSPI_CHCONF_FORCE, cs_active);
	mcspi_write_chconf0(spi, l);
}

/*----------------------------------------------------------------------*/

static int mcspi_wait_for_reg_bit(void *reg, unsigned int bit)
{
	ulong start = get_timer(0);

	MCSPI_VPRINT("%s: 0x%08x, 0x%08x\n", __func__, (u32)reg, bit);

	while (!(__raw_readl(reg) & bit)) {
		if ((get_timer(0) - start) > 1000/*ms*/) {
#ifdef CONFIG_OMAP2_MCSPI_DEBUG
			MCSPI_ERR("%s: 0x%08x=0x%08x (bit 0x%08x)\n", __func__, (u32)reg, __raw_readl(reg), bit);
#endif
			return -ETIMEDOUT;
		}
		barrier();
	}

	return 0;
}

static unsigned int omap2_mcspi_txrx_pio(struct omap2_mcspi *spi, unsigned int bitlen,
		const void *dout, void *din)
{
	unsigned int c;
	unsigned int chconf;
	void *tx_reg;
	void *rx_reg;
	void *chstat_reg;

	MCSPI_VPRINT("%s: start\n", __func__);

	c = bitlen;

	chconf = mcspi_cached_chconf0(spi);

	/* We store the pre-calculated register addresses on stack to speed
	 * up the transfer loop. */
	tx_reg = spi->cs_base + OMAP2_MCSPI_TX0;
	rx_reg = spi->cs_base + OMAP2_MCSPI_RX0;
	chstat_reg = spi->cs_base + OMAP2_MCSPI_CHSTAT0;

	if (c < spi->bits_per_word)
		return 0;

	if (spi->bits_per_word <= 8) {
		const unsigned char *tx;
		unsigned char *rx;

		tx = (const unsigned char *)dout;
		rx = (unsigned char*)din;

		if ((c & 0x07) != 0) {
#ifdef CONFIG_OMAP2_MCSPI_DEBUG
			MCSPI_CS_PRINT(spi, "the bit length must be 8-bit aligned for %d bits per word\n", spi->bits_per_word);
#else
			MCSPI_ERRPRINT(spi, "invalid bitlen\n");
#endif /* CONFIG_OMAP2_MCSPI_DEBUG */
			panic("BUG!");
			return 0;
		}

		do {
			MCSPI_CS_VPRINT(spi, "transfer: remnant bits: %u\n", c);
			c -= 8;

			if (tx) {
				if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_TXS)) {
					MCSPI_ERRPRINT(spi, "TXS timed out\n");
					c += 8;
					goto out;
				}
				MCSPI_CS_VPRINT(spi, "write-%d 0x%02X\n", spi->bits_per_word, *tx);
				__raw_writel(*tx, tx_reg);
				tx++;
			}

			if (rx) {
				if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_RXS)) {
					MCSPI_ERRPRINT(spi, "RXS timed out\n");
					c += 8;
					goto out;
				}
				if (c == 0 && dout == NULL)
					omap2_mcspi_set_enable(spi, 0);
				*rx = __raw_readl(rx_reg);
				MCSPI_CS_VPRINT(spi, "read-%d 0x%02X\n", spi->bits_per_word, *rx);
				rx++;
			}
		} while (c >= 8);
	} else if (spi->bits_per_word <= 16) {
		const unsigned short *tx;
		unsigned short *rx;

		tx = (const unsigned short*)dout;
		rx = (unsigned short*)din;

		if ((c & 0x0F) != 0) {
#ifdef CONFIG_OMAP2_MCSPI_DEBUG
			MCSPI_CS_PRINT(spi, "the bit length must be 16-bit aligned for %d bits per word\n", spi->bits_per_word);
#else
			MCSPI_ERRPRINT(spi, "invalid bitlen\n");
#endif /* CONFIG_OMAP2_MCSPI_DEBUG */
			panic("BUG!");
			return 0;
		}

		do {
			MCSPI_CS_VPRINT(spi, "transfer: remnant bits: %u\n", c);
			c -= 16;

			if (tx) {
				if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_TXS)) {
					MCSPI_ERRPRINT(spi, "TXS timed out\n");
					c += 16;
					goto out;
				}
				MCSPI_CS_VPRINT(spi, "write-%d 0x%04X\n", spi->bits_per_word, *tx);
				__raw_writel(*tx, tx_reg);
				tx++;
			}

			if (rx) {
				if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_RXS)) {
					MCSPI_ERRPRINT(spi, "RXS timed out\n");
					c += 16;
					goto out;
				}
				if (c == 0 && dout == NULL)
					omap2_mcspi_set_enable(spi, 0);
				*rx = __raw_readl(rx_reg);
				MCSPI_CS_VPRINT(spi, "read-%d 0x%04X\n", spi->bits_per_word, *rx);
				rx++;
			}
		} while (c >= 16);
	} else if (spi->bits_per_word <= 32) {
		const unsigned int *tx;
		unsigned int *rx;

		tx = (const unsigned int*)dout;
		rx = (unsigned int*)din;

		if ((c & 0x01F) != 0) {
#ifdef CONFIG_OMAP2_MCSPI_DEBUG
			MCSPI_CS_PRINT(spi, "the bit length must be 32-bit aligned for %d bits per word\n", spi->bits_per_word);
#else
			MCSPI_ERRPRINT(spi, "invalid bitlen\n");
#endif /* CONFIG_OMAP2_MCSPI_DEBUG */
			panic("BUG!");
			return 0;
		}

		do {
			MCSPI_CS_VPRINT(spi, "transfer: remnant bits: %u\n", c);
			c -= 32;

			if (tx) {
				if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_TXS)) {
					MCSPI_ERRPRINT(spi, "TXS timed out\n");
					c += 32;
					goto out;
				}
				MCSPI_CS_VPRINT(spi, "write-%d 0x%08x\n", spi->bits_per_word, *tx);
				__raw_writel(*tx, tx_reg);
				tx++;
			}

			if (rx) {
				if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_RXS)) {
					MCSPI_ERRPRINT(spi, "RXS timed out\n");
					c += 32;
					goto out;
				}
				if (c == 0 && dout == NULL)
					omap2_mcspi_set_enable(spi, 0);
				*rx = __raw_readl(rx_reg);
				MCSPI_CS_VPRINT(spi, "read-%d 0x%08x\n", spi->bits_per_word, *rx);
				rx++;
			}
		} while (c >= 32);
	} else {
		MCSPI_ERRPRINT(spi, "Unsupported word length (%d bits)\n", spi->bits_per_word);
	}

	/* for TX_ONLY mode, be sure all words have shifted out */
	if (din == NULL) {
		if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_TXS)) {
			MCSPI_ERRPRINT(spi, "TXS timed out\n");
		} else if (mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_EOT)) {
			MCSPI_ERRPRINT(spi, "EOT timed out\n");
		}

		/* disable chan to purge rx datas received in TX_ONLY transfer,
		 * otherwise these rx datas will affect the direct following
		 * RX_ONLY transfer.
		 */
		omap2_mcspi_set_enable(spi, 0);
	}

out:
	omap2_mcspi_set_enable(spi, 1);
	MCSPI_VPRINT("%s: end\n", __func__);
	return (bitlen - c);
}

/*----------------------------------------------------------------------*/

static u32 omap2_mcspi_calc_divisor(u32 speed_hz)
{
	u32 div;

	MCSPI_VPRINT("%s\n", __func__);

	for (div = 0; div < 15; div++)
		if (speed_hz >= (OMAP2_MCSPI_MAX_FREQ >> div))
			return div;

	return 15;
}

static void omap2_mcspi_setup_transfer(struct omap2_mcspi *spi)
{
	u32 l;

	MCSPI_VPRINT("%s\n", __func__);

	l = mcspi_cached_chconf0(spi);

	/* standard 4-wire master mode:  SCK, MOSI/out, MISO/in, nCS
	 * REVISIT: this controller could support SPI_3WIRE mode.
	 */
	l &= ~(OMAP2_MCSPI_CHCONF_IS | OMAP2_MCSPI_CHCONF_DPE1);
	l |= OMAP2_MCSPI_CHCONF_DPE0;

	/* word length */
	l &= ~OMAP2_MCSPI_CHCONF_WL_MASK;
	l |= (spi->bits_per_word - 1) << 7;

	/* set chipselect polarity; manage with FORCE */
	if (!(spi->mode & SPI_CS_HIGH))
		l |= OMAP2_MCSPI_CHCONF_EPOL;	/* active-low; normal */
	else
		l &= ~OMAP2_MCSPI_CHCONF_EPOL;	/* active-high */

	/* set clock divisor */
	l &= ~OMAP2_MCSPI_CHCONF_CLKD_MASK;
	l |= (spi->clk_divisor) << 2;

	/* set SPI mode 0..3 */
	if (spi->mode & SPI_CPOL)
		l |= OMAP2_MCSPI_CHCONF_POL;
	else
		l &= ~OMAP2_MCSPI_CHCONF_POL;
	if (spi->mode & SPI_CPHA)
		l |= OMAP2_MCSPI_CHCONF_PHA;
	else
		l &= ~OMAP2_MCSPI_CHCONF_PHA;

	mcspi_write_chconf0(spi, l);

	MCSPI_CS_PRINT(spi, "setup: %d bits per word, mode %d, speed %d, sample %s edge, clk %s\n",
			spi->bits_per_word, spi->mode, OMAP2_MCSPI_MAX_FREQ >> spi->clk_divisor,
			(spi->mode & SPI_CPHA) ? "trailing" : "leading",
			(spi->mode & SPI_CPOL) ? "inverted" : "normal");
}

/*----------------------------------------------------------------------*/

/*=====================================================================*/
/*                         Public Functions                            */
/*=====================================================================*/

void spi_init(void)
{
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct omap2_mcspi *spi;

	if (!spi_cs_is_valid(bus, cs)) {
		MCSPI_ERR("bus %d, cs %d is invalid!\n", bus, cs);
		return NULL;
	}

	spi = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(struct omap2_mcspi));
	if (!spi) {
		MCSPI_ERR("out of memory!\n");
		return NULL;
	}
	memset(spi, 0x0, sizeof(struct omap2_mcspi));

	spi->slave.bus = bus;
	spi->slave.cs = cs;
	spi->mode = mode;
	spi->bus = (--bus);
	spi->bits_per_word = 0;
	spi->bus_base = mcspis[bus].bus_base;
	spi->cs_base = (spi->bus_base) + (0x14 * cs);
	spi->chconf0 = 0;

	if (max_hz) {
		max_hz = min_t(unsigned int, max_hz, OMAP2_MCSPI_MAX_FREQ);
		spi->clk_divisor = omap2_mcspi_calc_divisor(max_hz);
	} else
		spi->clk_divisor = 15;

	(void)omap2_mcspi_enable_clocks(spi);

	if (!mcspis[spi->bus].initiated) {
		if (omap2_mcspi_reset(spi)) {
			omap2_mcspi_disable_clocks(spi);
			free(spi);
			return NULL;
		}
		mcspis[spi->bus].initiated = 1;
	}
#if defined(CONFIG_OMAP44XX)
	spi->bus_base += OMAP4_MCSPI_REG_OFFSET;
	spi->cs_base += OMAP4_MCSPI_REG_OFFSET;
#endif

#ifdef CONFIG_OMAP2_MCSPI_VERBOSE_DEBUG
{
	u32 rev = mcspi_read_reg(spi, OMAP2_MCSPI_REVISION);

	MCSPI_CS_PRINT(spi, "bus_base: 0x%08x, cs_base: 0x%08x\n", (u32)spi->bus_base, (u32)spi->cs_base);
	printf("OMAP McSPI%d: revision %d.%d\n", spi->slave.bus,
			(rev & 0x0F0) >> 4, (rev & 0x0F));
	MCSPI_CS_PRINT(spi, "speed %d Hz (div: %u)\n", OMAP2_MCSPI_MAX_FREQ >> spi->clk_divisor, spi->clk_divisor);
}
#endif

	omap2_mcspi_master_setup(spi);

	return &spi->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	omap2_mcspi_disable_clocks(spi);

	if (mcspis[spi->bus].clk_reference) {
		MCSPI_ERRPRINT(spi, "clocks not disabled!\n");
	}

	free(spi);
}

int spi_claim_bus(struct spi_slave *slave)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	MCSPI_BUG_ON((spi->bits_per_word < 4) || (spi->bits_per_word > 32));

	omap2_mcspi_runtime_resume(spi);
	omap2_mcspi_setup_transfer(spi);

	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	omap2_mcspi_disable_clocks(spi);

	if (mcspis[spi->bus].clk_reference > 1) {
		MCSPI_ERRPRINT(spi, "clocks not disabled!\n");
	}
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
		void *din, unsigned long flags)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);
	int ret = 0;
	unsigned int chconf;

	MCSPI_BUG_ON(mcspis[spi->bus].clk_reference == 0);

	if ((flags & SPI_XFER_ONLY) != SPI_XFER_ONLY) {
		omap2_mcspi_set_enable(spi, 1);
	}

	if (flags & SPI_XFER_BEGIN)
		omap2_mcspi_force_cs(spi, 1);

	if ((flags & SPI_XFER_ONLY) != SPI_XFER_ONLY) {
		chconf = mcspi_cached_chconf0(spi);
		chconf &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
		chconf &= ~OMAP2_MCSPI_CHCONF_TURBO;

		if (dout == NULL) {			/* Receive-Only Mode */
			MCSPI_CS_VPRINT(spi, "receive-only mode\n");
			chconf |= OMAP2_MCSPI_CHCONF_TRM_RX_ONLY;
		} else if (din == NULL) {	/* Transmit-Only Mode */
			MCSPI_CS_VPRINT(spi, "transmit-only mode\n");
			chconf |= OMAP2_MCSPI_CHCONF_TRM_TX_ONLY;
		} else {
			MCSPI_CS_VPRINT(spi, "transmit-receive mode\n");
		}

		mcspi_write_chconf0(spi, chconf);
	}

	if (bitlen) {
		unsigned int count;

		/* RX_ONLY mode needs dummy data in TX reg */
		if (dout == NULL)
			__raw_writel(0, spi->cs_base + OMAP2_MCSPI_TX0);

		count = omap2_mcspi_txrx_pio(spi, bitlen, dout, din);
		if (bitlen != count) {
			ret = -EIO;
			MCSPI_ERRPRINT(spi, "xfer err: %u<->%u\n", bitlen, count);
		}
	}

#if 0
	if ((flags & SPI_XFER_ONLY) != SPI_XFER_ONLY) {
		/* McSPI: fix for rx error due to channel noise */
		chconf = mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
		if (chconf & OMAP2_MCSPI_CHCONF_FORCE)
			omap2_mcspi_set_enable(spi, 0);
	}
#endif

	if (flags & SPI_XFER_END)
		omap2_mcspi_force_cs(spi, 0);

	if ((flags & SPI_XFER_ONLY) != SPI_XFER_ONLY)
		omap2_mcspi_set_enable(spi, 0);

	return ret;
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	if (bus > 0 && bus <= OMAP2_MCSPI_MAX_CTRL) {
		bus--;
		return (cs < mcspis[bus].num_cs);
	} else {
		return 0;
	}
}

void spi_cs_activate(struct spi_slave *slave)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	MCSPI_BUG_ON(mcspis[spi->bus].clk_reference == 0);

	omap2_mcspi_force_cs(spi, 1);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	MCSPI_BUG_ON(mcspis[spi->bus].clk_reference == 0);

	omap2_mcspi_force_cs(spi, 0);
}

/*----------------------------------------------------------------------*/

void omap_mcspi_setup_bits_per_word(struct spi_slave *slave, unsigned char bits_per_word)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	MCSPI_BUG_ON((bits_per_word < 4) || (bits_per_word > 32));
	spi->bits_per_word = bits_per_word;
}

void omap_mcspi_master_setup(struct spi_slave *slave)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	MCSPI_BUG_ON(mcspis[spi->bus].clk_reference == 0);

	omap2_mcspi_setup_transfer(spi);
}

void omap_mcspi_xfer_setup(struct spi_slave *slave, int enable, int has_dout,
	int has_din, unsigned long flags)
{
	struct omap2_mcspi *spi = _to_omap2_mcspi(slave);

	MCSPI_BUG_ON(mcspis[spi->bus].clk_reference == 0);

	if (enable) {
		unsigned int chconf;

		omap2_mcspi_set_enable(spi, 1);

		if (flags & SPI_XFER_BEGIN)
			omap2_mcspi_force_cs(spi, 1);

		chconf = mcspi_cached_chconf0(spi);
		chconf &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
		chconf &= ~OMAP2_MCSPI_CHCONF_TURBO;

		if (!has_dout) {			/* Receive-Only Mode */
			MCSPI_CS_VPRINT(spi, "receive-only mode\n");
			chconf |= OMAP2_MCSPI_CHCONF_TRM_RX_ONLY;
		} else if (!has_din) {	/* Transmit-Only Mode */
			MCSPI_CS_VPRINT(spi, "transmit-only mode\n");
			chconf |= OMAP2_MCSPI_CHCONF_TRM_TX_ONLY;
		} else {
			MCSPI_CS_VPRINT(spi, "transmit-receive mode\n");
		}

		mcspi_write_chconf0(spi, chconf);
	} else {
		if (flags & SPI_XFER_END)
			omap2_mcspi_force_cs(spi, 0);

		omap2_mcspi_set_enable(spi, 0);
	}
}
