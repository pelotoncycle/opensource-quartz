/*
 * Basic I2C functions
 *
 * Copyright (c) 2004 Texas Instruments
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Jian Zhang jzhang@ti.com, Texas Instruments
 *
 * Copyright (c) 2003 Wolfgang Denk, wd@denx.de
 * Rewritten to fit into the current U-Boot framework
 *
 * Adapted for OMAP2420 I2C, r-woodruff2@ti.com
 *
 */

#include <common.h>

#include <asm/arch/i2c.h>
#include <asm/io.h>

#include "omap24xx_i2c.h"

#if 0

DECLARE_GLOBAL_DATA_PTR;

#define I2C_TIMEOUT	1000

static void wait_for_bb(void);
static u16 wait_for_pin(void);
static void flush_fifo(void);

/*
 * For SPL boot some boards need i2c before SDRAM is initialised so force
 * variables to live in SRAM
 */
static struct i2c __attribute__((section (".data"))) *i2c_base =
					(struct i2c *)I2C_DEFAULT_BASE;
static unsigned int __attribute__((section (".data"))) bus_initialized[I2C_BUS_MAX] =
					{ [0 ... (I2C_BUS_MAX-1)] = 0 };
static unsigned int __attribute__((section (".data"))) current_bus = 0;

void i2c_init(int speed, int slaveadd)
{
	int psc, fsscll, fssclh;
	int hsscll = 0, hssclh = 0;
	u32 scll, sclh;
	int timeout = I2C_TIMEOUT;

	/* Only handle standard, fast and high speeds */
	if ((speed != OMAP_I2C_STANDARD) &&
	    (speed != OMAP_I2C_FAST_MODE) &&
	    (speed != OMAP_I2C_HIGH_SPEED)) {
		printf("Error : I2C unsupported speed %d\n", speed);
		return;
	}

	psc = I2C_IP_CLK / I2C_INTERNAL_SAMPLING_CLK;
	psc -= 1;
	if (psc < I2C_PSC_MIN) {
		printf("Error : I2C unsupported prescalar %d\n", psc);
		return;
	}

	if (speed == OMAP_I2C_HIGH_SPEED) {
		/* High speed */

		/* For first phase of HS mode */
		fsscll = fssclh = I2C_INTERNAL_SAMPLING_CLK /
			(2 * OMAP_I2C_FAST_MODE);

		fsscll -= I2C_HIGHSPEED_PHASE_ONE_SCLL_TRIM;
		fssclh -= I2C_HIGHSPEED_PHASE_ONE_SCLH_TRIM;
		if (((fsscll < 0) || (fssclh < 0)) ||
		    ((fsscll > 255) || (fssclh > 255))) {
			puts("Error : I2C initializing first phase clock\n");
			return;
		}

		/* For second phase of HS mode */
		hsscll = hssclh = I2C_INTERNAL_SAMPLING_CLK / (2 * speed);

		hsscll -= I2C_HIGHSPEED_PHASE_TWO_SCLL_TRIM;
		hssclh -= I2C_HIGHSPEED_PHASE_TWO_SCLH_TRIM;
		if (((fsscll < 0) || (fssclh < 0)) ||
		    ((fsscll > 255) || (fssclh > 255))) {
			puts("Error : I2C initializing second phase clock\n");
			return;
		}

		scll = (unsigned int)hsscll << 8 | (unsigned int)fsscll;
		sclh = (unsigned int)hssclh << 8 | (unsigned int)fssclh;

	} else {
		/* Standard and fast speed */
		fsscll = fssclh = I2C_INTERNAL_SAMPLING_CLK / (2 * speed);

		fsscll -= I2C_FASTSPEED_SCLL_TRIM;
		fssclh -= I2C_FASTSPEED_SCLH_TRIM;
		if (((fsscll < 0) || (fssclh < 0)) ||
		    ((fsscll > 255) || (fssclh > 255))) {
			puts("Error : I2C initializing clock\n");
			return;
		}

		scll = (unsigned int)fsscll;
		sclh = (unsigned int)fssclh;
	}

	if (readw(&i2c_base->con) & I2C_CON_EN) {
		writew(0, &i2c_base->con);
		udelay(50000);
	}

	writew(0x2, &i2c_base->sysc); /* for ES2 after soft reset */
	udelay(1000);

	writew(I2C_CON_EN, &i2c_base->con);
	while (!(readw(&i2c_base->syss) & I2C_SYSS_RDONE) && timeout--) {
		if (timeout <= 0) {
			puts("ERROR: Timeout in soft-reset\n");
			return;
		}
		udelay(1000);
	}

	writew(0, &i2c_base->con);
	writew(psc, &i2c_base->psc);
	writew(scll, &i2c_base->scll);
	writew(sclh, &i2c_base->sclh);

	/* own address */
	writew(slaveadd, &i2c_base->oa);
	writew(I2C_CON_EN, &i2c_base->con);

	/* have to enable intrrupts or OMAP i2c module doesn't work */
	writew(I2C_IE_XRDY_IE | I2C_IE_RRDY_IE | I2C_IE_ARDY_IE |
		I2C_IE_NACK_IE | I2C_IE_AL_IE, &i2c_base->ie);
	udelay(1000);
	flush_fifo();
	writew(0xFFFF, &i2c_base->stat);
	writew(0, &i2c_base->cnt);

	if (gd->flags & GD_FLG_RELOC)
		bus_initialized[current_bus] = 1;
}

static int i2c_read_byte(u8 devaddr, u16 regoffset, u8 alen, u8 *value)
{
	int i2c_error = 0;
	u16 status;
	int i = 2 - alen;
	u8 tmpbuf[2] = {(regoffset) >> 8, regoffset & 0xff};
	u16 w;

	/* wait until bus not busy */
	wait_for_bb();

	/* one byte only */
	writew(alen, &i2c_base->cnt);
	/* set slave address */
	writew(devaddr, &i2c_base->sa);
	/* no stop bit needed here */
	writew(I2C_CON_EN | I2C_CON_MST | I2C_CON_STT |
	      I2C_CON_TRX, &i2c_base->con);

	/* send register offset */
	while (1) {
		status = wait_for_pin();
		if (status == 0 || status & I2C_STAT_NACK) {
			i2c_error = 1;
			goto read_exit;
		}
		if (status & I2C_STAT_XRDY) {
			w = tmpbuf[i++];
#if !(defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) || \
	defined(CONFIG_OMAP44XX) || defined(CONFIG_AM33XX))
			w |= tmpbuf[i++] << 8;
#endif
			writew(w, &i2c_base->data);
			writew(I2C_STAT_XRDY, &i2c_base->stat);
		}
		if (status & I2C_STAT_ARDY) {
			writew(I2C_STAT_ARDY, &i2c_base->stat);
			break;
		}
	}

	/* set slave address */
	writew(devaddr, &i2c_base->sa);
	/* read one byte from slave */
	writew(1, &i2c_base->cnt);
	/* need stop bit here */
	writew(I2C_CON_EN | I2C_CON_MST |
		I2C_CON_STT | I2C_CON_STP,
		&i2c_base->con);

	/* receive data */
	while (1) {
		status = wait_for_pin();
		if (status == 0 || status & I2C_STAT_NACK) {
			i2c_error = 1;
			goto read_exit;
		}
		if (status & I2C_STAT_RRDY) {
#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) || \
	defined(CONFIG_OMAP44XX) || defined(CONFIG_AM33XX)
			*value = readb(&i2c_base->data);
#else
			*value = readw(&i2c_base->data);
#endif
			writew(I2C_STAT_RRDY, &i2c_base->stat);
		}
		if (status & I2C_STAT_ARDY) {
			writew(I2C_STAT_ARDY, &i2c_base->stat);
			break;
		}
	}

read_exit:
	flush_fifo();
	writew(0xFFFF, &i2c_base->stat);
	writew(0, &i2c_base->cnt);
	return i2c_error;
}

static void flush_fifo(void)
{	u16 stat;

	/* note: if you try and read data when its not there or ready
	 * you get a bus error
	 */
	while (1) {
		stat = readw(&i2c_base->stat);
		if (stat == I2C_STAT_RRDY) {
#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) || \
	defined(CONFIG_OMAP44XX) || defined(CONFIG_AM33XX)
			readb(&i2c_base->data);
#else
			readw(&i2c_base->data);
#endif
			writew(I2C_STAT_RRDY, &i2c_base->stat);
			udelay(1000);
		} else
			break;
	}
}

int i2c_probe(uchar chip)
{
	u16 status;
	int res = 1; /* default = fail */

	if (chip == readw(&i2c_base->oa))
		return res;

	/* wait until bus not busy */
	wait_for_bb();

	/* try to read one byte */
	writew(1, &i2c_base->cnt);
	/* set slave address */
	writew(chip, &i2c_base->sa);
	/* stop bit needed here */
	writew (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP, &i2c_base->con);

	while (1) {
		status = wait_for_pin();
		if (status == 0 || status & I2C_STAT_AL) {
			res = 1;
			goto probe_exit;
		}
		if (status & I2C_STAT_NACK) {
			res = 1;
			writew(0xff, &i2c_base->stat);
			writew (readw (&i2c_base->con) | I2C_CON_STP, &i2c_base->con);
			wait_for_bb ();
			break;
		}
		if (status & I2C_STAT_ARDY) {
			writew(I2C_STAT_ARDY, &i2c_base->stat);
			break;
		}
		if (status & I2C_STAT_RRDY) {
			res = 0;
#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) || \
    defined(CONFIG_OMAP44XX) || defined(CONFIG_AM33XX)
			readb(&i2c_base->data);
#else
			readw(&i2c_base->data);
#endif
			writew(I2C_STAT_RRDY, &i2c_base->stat);
		}
	}

probe_exit:
	flush_fifo();
	/* don't allow any more data in... we don't want it. */
	writew(0, &i2c_base->cnt);
	writew(0xFFFF, &i2c_base->stat);
	return res;
}

int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int i;

	if (alen > 2) {
		printf("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > (1 << 16)) {
		puts("I2C read: address out of range\n");
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (i2c_read_byte(chip, addr + i, alen, &buffer[i])) {
			puts("I2C read: I/O error\n");
			i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
			return 1;
		}
	}

	return 0;
}

int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int i;
	u16 status;
	int i2c_error = 0;
	u16 w;
	u8 tmpbuf[2] = {addr >> 8, addr & 0xff};

	if (alen > 2) {
		printf("I2C write: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > (1 << 16)) {
		printf("I2C write: address 0x%x + 0x%x out of range\n",
				addr, len);
		return 1;
	}

	/* wait until bus not busy */
	wait_for_bb();

	/* start address phase - will write regoffset + len bytes data */
	/* TODO consider case when !CONFIG_OMAP243X/34XX/44XX */
	writew(alen + len, &i2c_base->cnt);
	/* set slave address */
	writew(chip, &i2c_base->sa);
	/* stop bit needed here */
	writew(I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX |
		I2C_CON_STP, &i2c_base->con);

	/* Send address and data */
	for (i = -alen; i < len; i++) {
		status = wait_for_pin();

		if (status == 0 || status & I2C_STAT_NACK) {
			i2c_error = 1;
			printf("i2c error waiting for data ACK (status=0x%x)\n",
					status);
			goto write_exit;
		}

		if (status & I2C_STAT_XRDY) {
			w = (i < 0) ? tmpbuf[2+i] : buffer[i];
#if !(defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) || \
	defined(CONFIG_OMAP44XX) || defined(CONFIG_AM33XX))
			w |= ((++i < 0) ? tmpbuf[2+i] : buffer[i]) << 8;
#endif
			writew(w, &i2c_base->data);
			writew(I2C_STAT_XRDY, &i2c_base->stat);
		} else {
			i2c_error = 1;
			printf("i2c bus not ready for Tx (i=%d)\n", i);
			goto write_exit;
		}
	}

write_exit:
	flush_fifo();
	writew(0xFFFF, &i2c_base->stat);
	return i2c_error;
}

static void wait_for_bb(void)
{
	int timeout = I2C_TIMEOUT;
	u16 stat;

	writew(0xFFFF, &i2c_base->stat);	/* clear current interrupts...*/
	while ((stat = readw(&i2c_base->stat) & I2C_STAT_BB) && timeout--) {
		writew(stat, &i2c_base->stat);
		udelay(1000);
	}

	if (timeout <= 0) {
		printf("timed out in wait_for_bb: I2C_STAT=%x\n",
			readw(&i2c_base->stat));
	}
	writew(0xFFFF, &i2c_base->stat);	 /* clear delayed stuff*/
}

static u16 wait_for_pin(void)
{
	u16 status;
	int timeout = I2C_TIMEOUT;

	do {
		udelay(1000);
		status = readw(&i2c_base->stat);
	} while (!(status &
		   (I2C_STAT_ROVR | I2C_STAT_XUDF | I2C_STAT_XRDY |
		    I2C_STAT_RRDY | I2C_STAT_ARDY | I2C_STAT_NACK |
		    I2C_STAT_AL)) && timeout--);

	if (timeout <= 0) {
		printf("timed out in wait_for_pin: I2C_STAT=%x\n",
			readw(&i2c_base->stat));
		writew(0xFFFF, &i2c_base->stat);
		status = 0;
	}

	return status;
}

int i2c_set_bus_num(unsigned int bus)
{
	if ((bus < 0) || (bus >= I2C_BUS_MAX)) {
		printf("Bad bus: %d\n", bus);
		return -1;
	}

#if I2C_BUS_MAX == 3
	if (bus == 2)
		i2c_base = (struct i2c *)I2C_BASE3;
	else
#endif
	if (bus == 1)
		i2c_base = (struct i2c *)I2C_BASE2;
	else
		i2c_base = (struct i2c *)I2C_BASE1;

	current_bus = bus;

	if (!bus_initialized[current_bus])
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);

	return 0;
}

unsigned int i2c_get_bus_num(void)
{
	return current_bus;
}

#else

/**
 * James Wu:
 *   We refer to i2c-omap.c in Linux Kernel 2.6.35 & 3.0 and
 *   rewrite OMAP3/4 I2C driver.
 */

#if !defined(CONFIG_OMAP34XX) && !defined(CONFIG_OMAP44XX) \
		&& !defined(CONFIG_OMAP54XX)
#error "Only support OMAP3/OMAP4/OMAP5 I2C!\n"
#endif

#include <errno.h>
#include <asm/io.h>
#include <asm/arch/clocks.h>
#include <watchdog.h>

/*#define CONFIG_OMAP_I2C_DEBUG*/
/*#define CONFIG_OMAP_I2C_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_OMAP_I2C_DEBUG
#define CONFIG_OMAP_I2C_DEBUG
#endif /* CONFIG_OMAP_I2C_DEBUG */
#endif /* DEBUG */

#ifdef CONFIG_OMAP_I2C_DEBUG
#define I2C_BUS_DPRINT(i2c, fmt, args...) \
	do {printf("I2C%d: " fmt, i2c->bus, ##args);} while (0)
#define I2C_DPRINT(fmt, args...) \
	do {printf("I2C: " fmt, ##args);} while (0)
#else /* CONFIG_OMAP_I2C_DEBUG */
#define I2C_BUS_DPRINT(i2c, fmt, args...) \
	do {} while (0)
#define I2C_DPRINT(fmt, args...) \
	do {} while (0)
#endif /* CONFIG_OMAP_I2C_DEBUG */

#if defined(CONFIG_OMAP_I2C_DEBUG) && defined(CONFIG_OMAP_I2C_VERBOSE_DEBUG)
#define I2C_BUS_VPRINT(i2c, fmt, args...) \
	do {printf("I2C%d: " fmt, i2c->bus, ##args);} while (0)
#define I2C_VPRINT(fmt, args...) \
	do {printf("I2C: " fmt, ##args);} while (0)
#else /* CONFIG_OMAP_I2C_DEBUG, CONFIG_OMAP_I2C_VERBOSE_DEBUG */
#define I2C_BUS_VPRINT(i2c, fmt, args...) \
	do {} while (0)
#define I2C_VPRINT(fmt, args...) \
	do {} while (0)
#endif /* CONFIG_OMAP_I2C_DEBUG, CONFIG_OMAP_I2C_VERBOSE_DEBUG */

#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_OMAP_I2C_DEBUG)
#define I2C_BUS_ERRPRINT(i2c, fmt, args...) \
	do {printf("I2C%d: " fmt, i2c->bus, ##args);} while (0)
#define I2C_ERRPRINT(fmt, args...) \
	do {printf("I2C: " fmt, ##args);} while (0)
#else
#define I2C_BUS_ERRPRINT(i2c, fmt, args...) \
	do {} while (0)
#define I2C_ERRPRINT(fmt, args...) \
	do {} while (0)
#endif /* !CONFIG_SPL_BUILD || CONFIG_OMAP_I2C_DEBUG */

#define I2C_BUS_PRINT(i2c, fmt, args...) \
	do {printf("I2C%d: " fmt, i2c->bus, ##args);} while (0)
#define I2C_PRINT(fmt, args...) \
	do {printf("I2C: " fmt, ##args);} while (0)

#if 0
#define I2C_TIMEOUT	1000 /* ms */
#else
#define I2C_BUS_BUSY_TIMEOUT	350 /* ms */
#define I2C_BUS_CLEAR_TIMEOUT	150 /* ms */
#define I2C_XFER_TIMEOUT	350 /* ms */
#define I2C_TIMEOUT	350 /* ms */
#endif

/* I2C controller revisions */
#define OMAP_I2C_REV_2				0x20

/* I2C controller revisions present on specific hardware */
#define OMAP_I2C_REV_ON_2430		0x36
#define OMAP_I2C_REV_ON_3430		0x3C
#define OMAP_I2C_REV_ON_4430		0x40

/* Errata definitions */
#define I2C_OMAP_ERRATA_I207	(1 << 0)
#define I2C_OMAP3_1P153			(1 << 1)

struct i2c_msg {
	u16 addr;	/* slave address			*/
	u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
#define I2C_M_PROBE		0x0008	/* this is a ten bit chip address */
	u16 len;		/* msg length				*/
	u8 *buf;		/* pointer to msg data			*/
};

struct omap_i2c_dev {
	unsigned int		bus;
	unsigned int		speed;
	const struct i2c	*i2c_base;
	unsigned int		bus_initialized;
	int					use_count;

#if defined(CONFIG_OMAP34XX)
	/* OMAP3 clock registers */
	const unsigned int	*fclk_addr;
	const unsigned int	*iclk_addr;
	const unsigned int	*idlest_addr;
#elif defined(CONFIG_OMAP44XX)
	/* OMAP4 clock register */
	const unsigned int	*clkctrl_addr;
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif

	u16			cmd_err;
	u8			*buf;
	size_t		buf_len;

	u8			fifo_size;	/* use as flag and value
						 * fifo_size==0 implies no fifo
						 * if set, should be trsh+1
						 */
	u8			rev;
	unsigned	b_hw:1;		/* bad h/w fixes */
	unsigned	idle:1;
	u16			iestate;	/* Saved interrupt register */
	u16			pscstate;
	u16			scllstate;
	u16			sclhstate;
	u16			bufstate;
	u16			syscstate;
	u16			westate;
	u16			errata;
};

/* Because the first i2c_init() call is before clearing BSS, we have to move current_i2c_bus into DATA section. */
static unsigned int current_i2c_bus __attribute__ ((section (".data"))) = 0;

static struct omap_i2c_dev omap_i2c_devices[I2C_BUS_MAX] __attribute__((section (".data"))) = {
	[0] = {
		.bus		= 0,
		.speed		= CONFIG_SYS_I2C_SPEED,
		.i2c_base	= (const struct i2c *)I2C_BASE1,
		.bus_initialized	= 0,
		.use_count	= 0,
#if defined(CONFIG_OMAP34XX)
		/* OMAP3 clock registers */
		.fclk_addr		= (const unsigned int*)0x48004A00, /* CM_FCLKEN1_CORE */
		.iclk_addr		= (const unsigned int*)0x48004A10, /* CM_ICLKEN1_CORE */
		.idlest_addr	= (const unsigned int*)0x48004A20, /* CM_IDLEST1_CORE */
#elif defined(CONFIG_OMAP44XX)
		/* OMAP4 clock register */
		.clkctrl_addr	= (const unsigned int*)0x4A0094A0, /* CM_L4PER_I2C1_CLKCTRL */
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif
	},
#if I2C_BUS_MAX>=2
	[1] = {
		.bus		= 1,
		.speed		= CONFIG_SYS_I2C2_SPEED,
		.i2c_base	= (const struct i2c *)I2C_BASE2,
		.bus_initialized	= 0,
		.use_count	= 0,
#if defined(CONFIG_OMAP34XX)
		/* OMAP3 clock registers */
		.fclk_addr		= (const unsigned int*)0x48004A00, /* CM_FCLKEN1_CORE */
		.iclk_addr		= (const unsigned int*)0x48004A10, /* CM_ICLKEN1_CORE */
		.idlest_addr	= (const unsigned int*)0x48004A20, /* CM_IDLEST1_CORE */
#elif defined(CONFIG_OMAP44XX)
		/* OMAP4 clock register */
		.clkctrl_addr	= (const unsigned int*)0x4A0094A8, /* CM_L4PER_I2C2_CLKCTRL */
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif
	},
#endif /* I2C_BUS_MAX>=2 */
#if I2C_BUS_MAX>=3
	[2] = {
		.bus		= 2,
		.speed		= CONFIG_SYS_I2C3_SPEED,
		.i2c_base	= (const struct i2c *)I2C_BASE3,
		.bus_initialized	= 0,
		.use_count	= 0,
#if defined(CONFIG_OMAP34XX)
		/* OMAP3 clock registers */
		.fclk_addr		= (const unsigned int*)0x48004A00, /* CM_FCLKEN1_CORE */
		.iclk_addr		= (const unsigned int*)0x48004A10, /* CM_ICLKEN1_CORE */
		.idlest_addr	= (const unsigned int*)0x48004A20, /* CM_IDLEST1_CORE */
#elif defined(CONFIG_OMAP44XX)
		/* OMAP4 clock register */
		.clkctrl_addr	= (const unsigned int*)0x4A0094B0, /* CM_L4PER_I2C3_CLKCTRL */
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif
	},
#endif /* I2C_BUS_MAX>=3 */
#if I2C_BUS_MAX>=4
	[3] = {
		.bus		= 3,
		.speed		= CONFIG_SYS_I2C4_SPEED,
		.i2c_base	= (const struct i2c *)I2C_BASE4,
		.bus_initialized	= 0,
		.use_count	= 0,
#if defined(CONFIG_OMAP34XX)
#error "Not Supported!\n"
#elif defined(CONFIG_OMAP44XX)
		/* OMAP4 clock register */
		.clkctrl_addr	= (const unsigned int*)0x4A0094B8, /* CM_L4PER_I2C4_CLKCTRL */
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif
	},
#endif /* I2C_BUS_MAX>=4 */
};

static struct i2c_msg omap_i2c_msgs[2];

static void omap_i2c_clock_enable(struct omap_i2c_dev *dev)
{
#if defined(CONFIG_OMAP34XX)
	if (dev->fclk_addr)
		omap_clk_enable((u32 *)dev->fclk_addr, (u32 *)dev->iclk_addr,
				(u32 *)dev->idlest_addr, 15 + dev->bus);
#elif defined(CONFIG_OMAP44XX)
	if (dev->clkctrl_addr)
		omap_clk_enable((u32 *)dev->clkctrl_addr,
				MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif
}

static void omap_i2c_clock_disable(struct omap_i2c_dev *dev)
{
#if defined(CONFIG_OMAP34XX)
	if (dev->fclk_addr)
		omap_clk_disable((u32 *)dev->fclk_addr, (u32 *)dev->iclk_addr,
				15 + dev->bus);
#elif defined(CONFIG_OMAP44XX)
	if (dev->clkctrl_addr)
		omap_clk_disable((u32 *)dev->clkctrl_addr);
#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented!\n"
#else
#error "Not Implemented!\n"
#endif
}

static void omap_i2c_unidle(struct omap_i2c_dev *dev)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;

	if (dev->use_count == 0) {
		/* Enable I2C fclk & iclk */
		omap_i2c_clock_enable(dev);

#if defined(CONFIG_OMAP34XX) || defined(CONFIG_OMAP44XX) \
		|| defined(CONFIG_OMAP54XX)
		if (dev->idle) {
			ulong start;

			writew(0, &i2c_base->con);
			writew(dev->pscstate, &i2c_base->psc);
			writew(dev->scllstate, &i2c_base->scll);
			writew(dev->sclhstate, &i2c_base->sclh);
			writew(dev->bufstate, &i2c_base->buf);
			writew(dev->syscstate, &i2c_base->sysc);
			writew(dev->westate, &i2c_base->we);
			writew(I2C_CON_EN, &i2c_base->con);

			/* Wait till functional part of the I2C controller completely becomes valid */
			start = get_timer(0);
			while (!(readw(&i2c_base->syss) & I2C_SYSS_RDONE)) {
				if (get_timer(start) > I2C_TIMEOUT) {
					I2C_BUS_ERRPRINT(dev, "timeout in unidle (0x%04X)\n", readw(&i2c_base->syss));
					break;
				}
			}

			if (dev->rev >= OMAP_I2C_REV_ON_4430) {
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX)
				writew(0x6FFF, &i2c_base->irqenable_clr);
#endif /* CONFIG_OMAP44XX, CONFIG_OMAP54XX */
			}
			/*
			 * Don't write to this register if the IE state is 0 as it can
			 * cause deadlock.
			 */
			if (dev->iestate)
				writew(dev->iestate, &i2c_base->ie);
		}
#endif

		dev->idle = 0;
		dev->use_count = 1;
		I2C_BUS_VPRINT(dev, "unidled\n");
	} else {
		dev->use_count++;
		I2C_BUS_VPRINT(dev, "already unidled (%d)\n", dev->use_count);
	}
}

static void omap_i2c_idle(struct omap_i2c_dev *dev)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;

	if (dev->use_count == 1) {
		if (dev->rev >= OMAP_I2C_REV_ON_4430) {
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX)
			writew(0x6FFF, &i2c_base->irqenable_clr);
#endif /* CONFIG_OMAP44XX, CONFIG_OMAP54XX*/
		} else
			writew(0, &i2c_base->ie);

		if (dev->rev >= OMAP_I2C_REV_2) {
			writew(dev->iestate, &i2c_base->stat);

			/* Flush posted write before the dev->idle store occurs */
			readw(&i2c_base->stat);
		}
		dev->idle = 1;

		/* Disable I2C fclk & iclk */
		omap_i2c_clock_disable(dev);

		dev->use_count = 0;
		I2C_BUS_VPRINT(dev, "idled\n");
	} else if (dev->use_count > 1) {
		I2C_BUS_VPRINT(dev, "not idled (%d)\n", dev->use_count);
		dev->use_count--;
	} else {
		I2C_BUS_ERRPRINT(dev, "unbalanced idle (%d)\n", dev->use_count);
	}
}

static int omap_i2c_reset(struct omap_i2c_dev *dev)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	ulong start;
	int ret = 0;

	/* Disable I2C controller before soft reset */
	writew(readw(&i2c_base->con) & ~(I2C_CON_EN), &i2c_base->con);

	/* Set the I2Ci.I2C_SYSC[1] SRST bit to 1 */
	writew(I2C_SYSC_SOFTRESET_MASK, &i2c_base->sysc);
	/* For some reason we need to set the EN bit before the reset done bit gets set. */
	writew(I2C_CON_EN, &i2c_base->con);

	start = get_timer(0);
	while (!(readw(&i2c_base->syss) & I2C_SYSS_RDONE)) {
		if (get_timer(start) > I2C_TIMEOUT) {
			I2C_BUS_ERRPRINT(dev, "timeout in reset (0x%04X)\n", readw(&i2c_base->syss));
			return -ETIMEDOUT;
		}
	}

	return ret;
}

void i2c_init(int speed, int slaveadd)
{
	struct omap_i2c_dev *dev = &(omap_i2c_devices[current_i2c_bus]);
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	s16 psc = 0;
	u16 scll = 0, sclh = 0, buf = 0;
	u16 fsscll = 0, fssclh = 0, hsscll = 0, hssclh = 0;
	unsigned long internal_clk = 0;
	u16 s;

	if (current_i2c_bus >= I2C_BUS_MAX) {
		I2C_ERRPRINT("i2c_init: bad bus %u\n", current_i2c_bus);
		return;
	}

	dev->bus = current_i2c_bus;
	if ((speed <= 0) || (speed > OMAP_I2C_HIGH_SPEED)) {
		I2C_BUS_ERRPRINT(dev, "invalid speed %d\n", speed);
		speed = OMAP_I2C_STANDARD;
	}
	dev->speed = (unsigned int)speed;

	I2C_BUS_DPRINT(dev, "init: %d (0x%08X)\n", speed, (u32)i2c_base);

	dev->idle = 0;
	dev->errata = 0;

	omap_i2c_unidle(dev);

	dev->rev = (u8)(readw(&i2c_base->rev) & 0xFF);
	I2C_BUS_VPRINT(dev, "revision 0x%02X\n", dev->rev);

	if (dev->rev <= OMAP_I2C_REV_ON_3430) {
		dev->errata |= I2C_OMAP3_1P153;
	}
#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX)
	dev->errata |= I2C_OMAP_ERRATA_I207;
#endif

	/* Set up the fifo size - Get total size */
	s = (readw(&i2c_base->bufstat) >> 14) & 0x3;
	dev->fifo_size = 0x8 << s;

	/*
	 * Set up notification threshold as half the total available
	 * size. This is to ensure that we can handle the status on int
	 * call back latencies.
	 */
	dev->fifo_size = (dev->fifo_size / 2);
	I2C_BUS_VPRINT(dev, "fifo size: %d\n", dev->fifo_size);

	if (dev->rev >= OMAP_I2C_REV_ON_4430)
		dev->b_hw = 0; /* Disable hardware fixes */
	else
		dev->b_hw = 1; /* Enable hardware fixes */

	if (dev->rev >= OMAP_I2C_REV_2) {
		/* Execute Soft-reset sequence for I2C controller */
		if (omap_i2c_reset(dev))
			goto out;

		/* SYSC register is cleared by the reset; rewrite it */
		if (dev->rev == OMAP_I2C_REV_ON_2430) {

			writew(I2C_SYSC_AUTOIDLE_MASK, &i2c_base->sysc);

		} else if (dev->rev >= OMAP_I2C_REV_ON_3430) {
			dev->syscstate = I2C_SYSC_AUTOIDLE_MASK;
			dev->syscstate |= I2C_SYSC_ENAWAKEUP_MASK;
			dev->syscstate |= (I2C_SYSC_IDLEMODE_SMART << I2C_SYSC_IDLEMODE_SHIFT);
			dev->syscstate |= (I2C_SYSC_CLOCKACTIVITY_FCLK << I2C_SYSC_CLOCKACTIVITY_SHIFT);

			writew(dev->syscstate, &i2c_base->sysc);
			/*
			 * Enabling all wakup sources to stop I2C freezing on
			 * WFI instruction.
			 * REVISIT: Some wkup sources might not be needed.
			 */
			dev->westate = I2C_WE_ALL;
			writew(dev->westate, &i2c_base->we);
		}
	}
	writew(0, &i2c_base->con);

	/*
	 * HSI2C controller internal clk rate should be 96 Mhz for
	 * HS mode on 4430 ES2.x and 19.2 Mhz for HS mode on 2430 and
	 * For all modes on 34xx we can use lower rate to get longer
	 * filter period for better noise suppression.
	 * The filter is iclk (fclk for HS) period.
	 */
#if defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX)
	if (dev->speed > 1000000)
		internal_clk = 96000000;
	else if (dev->speed > OMAP_I2C_FAST_MODE)
		internal_clk = 19200000;
	else if (dev->speed > OMAP_I2C_STANDARD)
		internal_clk = 9600000;
	else /* standard */
		internal_clk = 4000000;
#elif defined(CONFIG_OMAP34XX)
	if (dev->speed > OMAP_I2C_FAST_MODE)
		internal_clk = 19200000;
	else if (dev->speed > OMAP_I2C_STANDARD)
		internal_clk = 9600000;
	else /* standard */
		internal_clk = 4000000;
#elif defined(CONFIG_OMAP243X)
	internal_clk = 19200000;
#else
#error "Not Implemented\n"
#endif

	psc = (s16)(I2C_IP_CLK / internal_clk) - 1;
	if (psc < I2C_PSC_MIN || psc > I2C_PSC_MAX) {
		I2C_BUS_ERRPRINT(dev, "invalid psc %d\n", psc);
		goto out;
	}

	/* If configured for High Speed */
	if (dev->speed > OMAP_I2C_FAST_MODE) {
		unsigned long scl;

		/* For first phase of HS mode */
		scl = internal_clk / OMAP_I2C_FAST_MODE;
		fsscll = scl - (scl / 3) - 7;
		fssclh = (scl / 3) - 5;

		/* For second phase of HS mode */
		scl = I2C_IP_CLK / dev->speed;
		hsscll = scl - (scl / 3) - 7;
		hssclh = (scl / 3) - 5;
	} else if (dev->speed > OMAP_I2C_STANDARD) {
		unsigned long scl;

		/* Fast mode */
		scl = internal_clk / dev->speed;
		fsscll = scl - (scl / 3) - 7;
		fssclh = (scl / 3) - 5;
 	} else {
		/* Standard mode */
		fsscll = internal_clk / (dev->speed * 2) - 7;
		fssclh = internal_clk / (dev->speed * 2) - 5;
	}
	scll = (hsscll << I2C_SCLL_HSSCLL) | fsscll;
	sclh = (hssclh << I2C_SCLH_HSSCLH) | fssclh;

	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	writew(psc, &i2c_base->psc);

	/* SCL low and high time values */
	writew(scll, &i2c_base->scll);
	writew(sclh, &i2c_base->sclh);

	if (dev->fifo_size) {
		/* Note: setup required fifo size - 1. RTRSH and XTRSH */
		buf = (dev->fifo_size - 1) << 8 | I2C_BUF_RXFIF_CLR |
			(dev->fifo_size - 1) | I2C_BUF_TXFIF_CLR;
		writew(buf, &i2c_base->buf);
	}

	/* Own address */
	writew (slaveadd, &i2c_base->oa);

	/* Take the I2C module out of reset: */
	writew(I2C_CON_EN, &i2c_base->con);

	/* Decide what interrupts are needed */
	dev->iestate = (I2C_IE_XRDY_IE | I2C_IE_RRDY_IE |
			I2C_IE_ARDY_IE | I2C_IE_NACK_IE |
			I2C_IE_AL_IE)  | ((dev->fifo_size) ?
				(I2C_IE_RDR_IE | I2C_IE_XDR_IE) : 0);

#if defined(CONFIG_OMAP34XX) || defined(CONFIG_OMAP44XX) \
		|| defined(CONFIG_OMAP54XX)
	dev->pscstate = psc;
	dev->scllstate = scll;
	dev->sclhstate = sclh;
	dev->bufstate = buf;
#endif

	I2C_BUS_VPRINT(dev, "FCLK %u\n", I2C_IP_CLK);
	I2C_BUS_VPRINT(dev, "internal_clk %lu\n", internal_clk);
	I2C_BUS_VPRINT(dev, "speed %d\n", dev->speed);
	I2C_BUS_VPRINT(dev, "psc %d, scll 0x%04X, sclh 0x%04X\n", dev->pscstate, dev->scllstate, dev->sclhstate);
	I2C_BUS_VPRINT(dev, "fifo_size %d\n", dev->fifo_size);
	I2C_BUS_VPRINT(dev, "buf 0x%04X\n", dev->bufstate);
	I2C_BUS_VPRINT(dev, "syscstate 0x%04X\n", dev->syscstate);
	I2C_BUS_VPRINT(dev, "iestate 0x%04X\n", dev->iestate);

	dev->bus_initialized = 1;

out:
	omap_i2c_idle(dev);
}

static int omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	ulong start;

	start = get_timer(0);
	while ((readw(&i2c_base->stat) & I2C_STAT_BB)) {
		if (get_timer(start) > I2C_BUS_BUSY_TIMEOUT) {
			I2C_BUS_PRINT(dev, "BB err (%04X)\n", readw(&i2c_base->stat));
			return -ETIMEDOUT;
		}
		udelay(1000);
	}

	return 0;
}

/*
 * Bus Clear
 */
static int omap_i2c_bus_clear(struct omap_i2c_dev *dev, int re_init, int re_wait_for_bb)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	u16 w;

	/* Per the I2C specification, if we are stuck in a bus busy state
	 * we can attempt a bus clear to try and recover the bus by sending
	 * at least 9 clock pulses on SCL. Put the I2C in a test mode so it
	 * will output a continuous clock on SCL.
	 */
	w = readw(&i2c_base->systest);
	I2C_BUS_DPRINT(dev, "bus clear: status 0x%04x\n", w);
	writew(I2C_CON_EN, &i2c_base->con);
	writew((I2C_SYSTEST_ST_EN | I2C_SYSTEST_TMODE_TEST), &i2c_base->systest);
	udelay(1000);

	/* Simulate the NACK */
	/* SCL low; SDA high */
	writew(I2C_SYSTEST_ST_EN | I2C_SYSTEST_SDA_O, &i2c_base->systest);
	udelay(50);
	/* SCL high; SDA high */
	writew(I2C_SYSTEST_ST_EN | I2C_SYSTEST_SCL_O | I2C_SYSTEST_SDA_O, &i2c_base->systest);
	udelay(50);
	/* SCL low; SDA high */
	writew(I2C_SYSTEST_ST_EN | I2C_SYSTEST_SDA_O, &i2c_base->systest);;
	udelay(50);

	/* Simulate the STOP signal */
	/* SCL low; SDA low */
	writew(I2C_SYSTEST_ST_EN, &i2c_base->systest);;
	udelay(50);
	/* SCL high; SDA low */
	writew(I2C_SYSTEST_ST_EN | I2C_SYSTEST_SCL_O, &i2c_base->systest);;
	udelay(50);
	/* SCL high; SDA high */
	writew(I2C_SYSTEST_ST_EN | I2C_SYSTEST_SCL_O | I2C_SYSTEST_SDA_O, &i2c_base->systest);;
	udelay(50);

	writew(w, &i2c_base->systest);

{
	ulong start;

	I2C_BUS_VPRINT(dev, "bus clear: status 0x%04x\n", readw(&i2c_base->systest));
	start = get_timer(0);
	while (((w = readw(&i2c_base->systest)) & (I2C_SYSTEST_SCL_I_FUNC | I2C_SYSTEST_SDA_I_FUNC))
			!= (I2C_SYSTEST_SCL_I_FUNC | I2C_SYSTEST_SDA_I_FUNC)) {
		if (get_timer(start) > I2C_BUS_CLEAR_TIMEOUT) {
			I2C_BUS_ERRPRINT(dev, "bus clear failed: SDA=%d SCL=%d (0x%04x)\n",
				(w & I2C_SYSTEST_SDA_I_FUNC) >> 6, (w & I2C_SYSTEST_SCL_I_FUNC) >> 8, w);
			break;
		}
		udelay(1000);
	}
	I2C_BUS_VPRINT(dev, "bus clear: status 0x%04x\n", readw(&i2c_base->systest));
}

	writew(0, &i2c_base->con);

	if (re_init)
		i2c_init(dev->speed, CONFIG_SYS_I2C_SLAVE);

	if (re_wait_for_bb)
		return omap_i2c_wait_for_bb(dev);

	return 0;
}

static inline void omap_i2c_ack_stat(struct omap_i2c_dev *dev, u16 stat)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;

	writew (stat, &i2c_base->stat);
	/* Flush the post write */
	readw(&i2c_base->stat);
}

/*
 * OMAP3430 Errata 1.153: When an XRDY/XDR is hit, wait for XUDF before writing
 * data to DATA_REG. Otherwise some data bytes can be lost while transferring
 * them from the memory to the I2C interface.
 */
static int errata_omap3_1p153(struct omap_i2c_dev *dev, u16 *stat, int *err)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	unsigned long timeout = 10000;

	while (--timeout && !(*stat & I2C_STAT_XUDF)) {
		if (*stat & (I2C_STAT_NACK | I2C_STAT_AL)) {
			omap_i2c_ack_stat(dev, *stat & (I2C_STAT_XRDY | I2C_STAT_XDR));
			*err |= I2C_STAT_XUDF;
			I2C_BUS_ERRPRINT(dev, "1.153: timeout (%04X)\n", *stat);
			return -ETIMEDOUT;
		}

		__iormb();
		*stat = readw(&i2c_base->stat);
	}

	if (!timeout) {
		I2C_BUS_ERRPRINT(dev, "1.153: XUDF timeout (%04X)\n", *stat);
		return 0;
	}

	return 0;
}

static inline void i2c_omap_errata_i207(struct omap_i2c_dev *dev, u16 stat)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;

	/*
	 * I2C Errata(Errata Nos. OMAP2: 1.67, OMAP3: 1.8)
	 * Not applicable for OMAP4.
	 * Under certain rare conditions, RDR could be set again
	 * when the bus is busy, then ignore the interrupt and
	 * clear the interrupt.
	 */
	if (stat & I2C_STAT_RDR) {
		/* Step 1: If RDR is set, clear it */
		omap_i2c_ack_stat(dev, I2C_STAT_RDR);

		/* Step 2: */
		if (!(readw(&i2c_base->stat) & I2C_STAT_BB)) {
			/* Step 3: */
			if (readw(&i2c_base->stat) & I2C_STAT_RDR) {
				omap_i2c_ack_stat(dev, I2C_STAT_RDR);
				I2C_BUS_ERRPRINT(dev, "i207 RDR when bus is busy\n");
			}
		}
	}
}

static int omap_i2c_isr(struct omap_i2c_dev *dev)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	ulong start;
	u16 stat = 0, w;
	int err, count = 0;;

	if (dev->idle)
		return -EINVAL;

	start = get_timer(0);
	while (1) {
		if (get_timer(start) > I2C_XFER_TIMEOUT) {
			I2C_BUS_DPRINT(dev, "ISR timeout\n");
			return -ETIMEDOUT;
		}
		__iormb();
		stat = readw(&i2c_base->stat);
		if (stat & dev->iestate) {
			if (count++ == 100) {
				/* Too much works */
				WATCHDOG_RESET();
				count = 0;
			}

			err = 0;
complete:
			/*
			 * Ack the stat in one go, but [R/X]DR and [R/X]RDY should be
			 * acked after the data operation is complete.
			 * Ref: TRM SWPU114Q Figure 18-31
			 */
			writew(stat & ~(I2C_STAT_RRDY | I2C_STAT_RDR | I2C_STAT_XRDY | I2C_STAT_XDR), &i2c_base->stat);

			if (stat & I2C_STAT_NACK) {
				err |= I2C_STAT_NACK;
				/* NACK without STP */
#if 0
				writew(I2C_CON_STP, &i2c_base->con);
#endif
			}
			if (stat & I2C_STAT_AL) {
				I2C_BUS_PRINT(dev, "arbitration lost\n");
				err |= I2C_STAT_AL;
				omap_i2c_ack_stat(dev, I2C_STAT_AL);
				dev->cmd_err |= err;
				return 0;
			}
			/*
			 * ProDB0017052: Clear ARDY bit twice
			 */
			if (stat & I2C_STAT_ARDY)
				omap_i2c_ack_stat(dev, I2C_STAT_ARDY);

			if (stat & (I2C_STAT_ARDY | I2C_STAT_NACK | I2C_STAT_AL)) {
				omap_i2c_ack_stat(dev, (I2C_STAT_RRDY | I2C_STAT_RDR |
						I2C_STAT_XRDY | I2C_STAT_XDR | I2C_STAT_ARDY));
				dev->cmd_err |= err;
				return 0;
			}
			if (stat & (I2C_STAT_RRDY | I2C_STAT_RDR)) {
				u8 num_bytes = 1;

				if (dev->errata & I2C_OMAP_ERRATA_I207)
					i2c_omap_errata_i207(dev, stat);

				if (dev->fifo_size) {
					if (stat & I2C_STAT_RRDY)
						num_bytes = dev->fifo_size;
					else    /* read RXSTAT on RDR interrupt */
						num_bytes = (readw(&i2c_base->bufstat) >> 8) & 0x3F;
				}
				while (num_bytes) {
					num_bytes--;
					w = readw(&i2c_base->data);
					if (dev->buf_len) {
						*dev->buf++ = w;
						dev->buf_len--;
#if 0
						/*
						 * Data reg in 2430, omap3 and
						 * omap4 is 8 bit wide
						 */
#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) \
		|| defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX)
#else
						if (dev->buf_len) {
							*dev->buf++ = w >> 8;
							dev->buf_len--;
						}
#endif
#endif
					} else {
						if (stat & I2C_STAT_RRDY) {
							I2C_BUS_ERRPRINT(dev, "RRDY IRQ (no data)\n");
						}
						if (stat & I2C_STAT_RDR) {
							I2C_BUS_ERRPRINT(dev, "RDR IRQ (no data)\n");
						}
						break;
					}
				}
				omap_i2c_ack_stat(dev, stat & (I2C_STAT_RRDY | I2C_STAT_RDR));
				continue;
			}
			if (stat & (I2C_STAT_XRDY | I2C_STAT_XDR)) {
				u8 num_bytes = 1;
				if (dev->fifo_size) {
					if (stat & I2C_STAT_XRDY)
						num_bytes = dev->fifo_size;
					else    /* read TXSTAT on XDR interrupt */
						num_bytes = readw(&i2c_base->bufstat) & 0x3F;
				}
				while (num_bytes) {
					num_bytes--;
					w = 0;
					if (dev->buf_len) {
						w = *dev->buf++;
						dev->buf_len--;
#if 0
						/*
						 * Data reg in 2430, omap3 and
						 * omap4 is 8 bit wide
						 */
#if defined(CONFIG_OMAP243X) || defined(CONFIG_OMAP34XX) \
		|| defined(CONFIG_OMAP44XX) || defined(CONFIG_OMAP54XX)
#else
						if (dev->buf_len) {
							w |= *dev->buf++ << 8;
							dev->buf_len--;
						}
#endif
#endif
					} else {
						if (stat & I2C_STAT_XRDY) {
							I2C_BUS_ERRPRINT(dev, "XRDY IRQ (no data)\n");
						}
						if (stat & I2C_STAT_XDR) {
							I2C_BUS_ERRPRINT(dev, "XDR IRQ (no data)\n");
						}
						break;
					}

					if ((dev->errata & I2C_OMAP3_1P153) && errata_omap3_1p153(dev, &stat, &err))
						goto complete;

					writew(w, &i2c_base->data);
				}
				omap_i2c_ack_stat(dev, stat & (I2C_STAT_XRDY | I2C_STAT_XDR));
				continue;
			}
			if (stat & I2C_STAT_ROVR) {
				I2C_BUS_PRINT(dev, "receive overrun\n");
				err |= I2C_STAT_ROVR;
				omap_i2c_ack_stat(dev, I2C_STAT_ROVR);
				dev->cmd_err |= err;
			}
			if (stat & I2C_STAT_XUDF) {
				I2C_BUS_PRINT(dev, "transmit underflow\n");
				err |= I2C_STAT_XUDF;
				omap_i2c_ack_stat(dev, I2C_STAT_XUDF);
				dev->cmd_err |= err;
			}
		}
	}

	if (stat) {
		/* Flush the post write */
		writew(stat, &i2c_base->stat);
		stat = readw(&i2c_base->stat);
	}

	return 0;
}

/*
 * Low level master read/write transaction.
 */
static int omap_i2c_xfer_msg(struct omap_i2c_dev *dev, struct i2c_msg *msg, int stop)
{
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	int ret = 0;
	u16 w;

	if (msg->len == 0 && !(msg->flags & I2C_M_PROBE))
		return -EINVAL;

	/* Set slave address */
	writew (msg->addr, &i2c_base->sa);

	dev->buf = msg->buf;
	dev->buf_len = msg->len;

	writew((u16)dev->buf_len, &i2c_base->cnt);

	/* Clear the FIFO Buffers */
	w = readw(&i2c_base->buf);
	w |= I2C_BUF_RXFIF_CLR | I2C_BUF_TXFIF_CLR;
	writew(w, &i2c_base->buf);

	dev->cmd_err = 0;

	w = I2C_CON_EN | I2C_CON_MST | I2C_CON_STT;
	/* High speed configuration */
	if (dev->speed > OMAP_I2C_FAST_MODE)
		w |= I2C_CON_OPMODE_HS;
	if (msg->flags & I2C_M_TEN)
		w |= I2C_CON_XA;
	if (!(msg->flags & I2C_M_RD))
		w |= I2C_CON_TRX;
	if (!dev->b_hw && stop)
		w |= I2C_CON_STP;
	writew(w, &i2c_base->con);

	/*
	 * Don't write stt and stp together on some hardware.
	 */
	if (dev->b_hw && stop) {
		ulong start;
		u16 con;

		start = get_timer(0);
		con = readw(&i2c_base->con);
		while (con & I2C_CON_STT) {
			if (get_timer(start) > I2C_TIMEOUT) {
				I2C_BUS_ERRPRINT(dev, "[0x%02x] timeout in STT\n", msg->addr);
				i2c_init(dev->speed, CONFIG_SYS_I2C_SLAVE);
				return -ETIMEDOUT;
			}
			con = readw(&i2c_base->con);
		}

		w |= I2C_CON_STP;
		w &= ~I2C_CON_STT;
		writew(w, &i2c_base->con);
	}

	ret = omap_i2c_isr(dev);
	dev->buf_len = 0;
	if (ret == -ETIMEDOUT && !(msg->flags & I2C_M_PROBE)) {
		I2C_BUS_ERRPRINT(dev, "[0x%02x] xfer timed out\n", msg->addr);
#if 0
		i2c_init(dev->speed, CONFIG_SYS_I2C_SLAVE);
#else
		omap_i2c_bus_clear(dev, 1, 1);
#endif
		return -ETIMEDOUT;
	}

	if (!dev->cmd_err)
		return 0;

	/* We have an error */
	if (dev->cmd_err & (I2C_STAT_AL | I2C_STAT_ROVR |
				I2C_STAT_XUDF)) {
#if 0
		i2c_init(dev->speed, CONFIG_SYS_I2C_SLAVE);
#else
		omap_i2c_bus_clear(dev, 1, 1);
#endif
		return -EAGAIN;
	}

	if (dev->cmd_err & I2C_STAT_NACK) {
		if (stop) {
			w = readw(&i2c_base->con);
			w |= I2C_CON_STP;
			writew(w, &i2c_base->con);
		}
		if (msg->flags & I2C_M_IGNORE_NAK)
			return 0;

		if (!(msg->flags & I2C_M_PROBE)) {
			I2C_BUS_DPRINT(dev, "[0x%02x] NACK err\n", msg->addr);
			i2c_init(dev->speed, CONFIG_SYS_I2C_SLAVE);
		}
		return -EREMOTEIO;
	}

	I2C_BUS_ERRPRINT(dev, "[0x%02x] 0x%04X err\n", msg->addr, dev->cmd_err);
	//i2c_init(dev->speed, CONFIG_SYS_I2C_SLAVE);
	return -EIO;
}

#if defined(CONFIG_OMAP_I2C_VERBOSE_DEBUG)
static int i2c_read_byte(struct omap_i2c_dev *dev, u8 devaddr, u16 regoffset, u8 alen, u8 *value)
{
	int ret = 0, i;
	u8 tmpbuf[2];

	/* [MSG1] fill the register address data */
	omap_i2c_msgs[0].addr = devaddr;
	omap_i2c_msgs[0].flags = 0;	/* Write the register address */
	omap_i2c_msgs[0].len = alen;
	if (alen > 1)
		tmpbuf[0] = (u8)((regoffset & 0xff00) >> 8);
	else
		tmpbuf[0] = (u8)(regoffset & 0xff);
	tmpbuf[1] = (u8)(regoffset & 0xff);
	omap_i2c_msgs[0].buf = tmpbuf;

	/* [MSG2] fill the data rx buffer */
	omap_i2c_msgs[1].addr = devaddr;
	omap_i2c_msgs[1].flags = I2C_M_RD;	/* Read the register value */
	omap_i2c_msgs[1].len = 1;	/* only 1 bytes */
	omap_i2c_msgs[1].buf = value;

	for (i = 0 ; i < 2 ; i++) {
		ret = omap_i2c_xfer_msg(dev, &(omap_i2c_msgs[i]), 1);
		if (ret)
			break;
	}

	return ret;
}
#endif /* CONFIG_OMAP_I2C_VERBOSE_DEBUG */

static int i2c_read_bytes(struct omap_i2c_dev *dev, u8 devaddr, u16 regoffset, u8 alen, u8 *values, int len)
{
	int ret = 0, i;
	u8 tmpbuf[2];

	/* [MSG1] fill the register address data */
	omap_i2c_msgs[0].addr = devaddr;
	omap_i2c_msgs[0].flags = 0;	/* Write the register address */
	omap_i2c_msgs[0].len = alen;
	if (alen == 1)
		tmpbuf[0] = regoffset & 0xff;
	else
		tmpbuf[0] = (regoffset & 0xff00) >> 8;
	tmpbuf[1] = regoffset & 0xff;
	omap_i2c_msgs[0].buf = tmpbuf;

	/* [MSG2] fill the data rx buffer */
	omap_i2c_msgs[1].addr = devaddr;
	omap_i2c_msgs[1].flags = I2C_M_RD;	/* Read the register values */
	omap_i2c_msgs[1].len = (u16)len;
	omap_i2c_msgs[1].buf = values;

	for (i = 0 ; i < 2 ; i++) {
		ret = omap_i2c_xfer_msg(dev, &(omap_i2c_msgs[i]), (i == 1 ? 1 : 0));
		if (ret)
			break;
	}

	return ret;
}

static int i2c_write_byte(struct omap_i2c_dev *dev, u8 devaddr, u16 regoffset, u8 alen, u8 value)
{
	u8 buffers[3];

	/* over write the first byte of buffer with the register address */
	if (alen == 1) {
		buffers[0] = regoffset & 0xff;
		buffers[1] = value;
		buffers[2] = 0;
		omap_i2c_msgs[0].len = 2;
	} else {
		buffers[0] = (regoffset & 0xff00) >> 8;
		buffers[1] = regoffset & 0xff;
		buffers[2] = value;
		omap_i2c_msgs[0].len = 3;
	}

	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	omap_i2c_msgs[0].addr = devaddr;
	omap_i2c_msgs[0].flags = 0;
	omap_i2c_msgs[0].buf = buffers;

	return omap_i2c_xfer_msg(dev, &(omap_i2c_msgs[0]), 1);
}

int i2c_probe(uchar chip)
{
#if defined(CONFIG_CMD_I2C)
	struct omap_i2c_dev *dev = &(omap_i2c_devices[current_i2c_bus]);
	struct i2c *i2c_base = (struct i2c *)dev->i2c_base;
	u8 value = 0;
	int res = 1; /* default = fail */
	int ret;

	omap_i2c_unidle(dev);

	if (chip == readw(&i2c_base->oa)) {
		goto out;
	}

	/* wait until bus not busy */
	ret = omap_i2c_wait_for_bb(dev);
	/* If timeout, try to again check after soft reset of I2C block */
	if (ret == -ETIMEDOUT)
		ret = omap_i2c_bus_clear(dev, 1, 1);
	if (ret)
		goto out;

	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	omap_i2c_msgs[0].addr = chip;
	omap_i2c_msgs[0].flags = I2C_M_PROBE;	/* Apply the probe functionality */
	omap_i2c_msgs[0].len = 0;
	omap_i2c_msgs[0].buf = &value;
	if (omap_i2c_xfer_msg(dev, &(omap_i2c_msgs[0]), 1) == 0)
		res = 0;

out:
	omap_i2c_idle(dev);
	return res;
#else
	return 0;
#endif
}

int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	struct omap_i2c_dev *dev = &(omap_i2c_devices[current_i2c_bus]);
	int ret = 0;
#ifdef CONFIG_OMAP_I2C_VERBOSE_DEBUG
	int i;
#endif /* CONFIG_OMAP_I2C_VERBOSE_DEBUG */

	if (alen > 2) {
		I2C_BUS_ERRPRINT(dev, "read: invalid addr len %d\n", alen);
		return -EINVAL;
	}

	if ((addr + len) > (1 << 16)) {
		I2C_BUS_ERRPRINT(dev, "read: address out of range (0x%x-0x%x)\n", addr, addr + len);
		return -EINVAL;
	}

	if (!dev->bus_initialized) {
		I2C_BUS_ERRPRINT(dev, "read: not initialized\n");
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		if (!dev->bus_initialized) {
			I2C_BUS_ERRPRINT(dev, "fail init\n");
			return -EPERM;
		}
	}

	omap_i2c_unidle(dev);

	/* wait until bus not busy */
	ret = omap_i2c_wait_for_bb(dev);
	/* If timeout, try to again check after soft reset of I2C block */
	if (ret == -ETIMEDOUT)
		ret = omap_i2c_bus_clear(dev, 1, 1);
	if (ret)
		goto out;

	ret = i2c_read_bytes(dev, chip, addr, alen, buffer, len);
	if (ret) {
		I2C_BUS_PRINT(dev, "read(0x%02X): [0x%02X:%d] I/O err (%d)\n", chip, addr, len, ret);
	} else {
#ifdef CONFIG_OMAP_I2C_VERBOSE_DEBUG
		for (i = 0; i < len; i++) {
			I2C_BUS_PRINT(dev, "read(0x%02X): [0x%02X] = 0x%02X\n", chip, addr + i, buffer[i]);
		}
#endif /* CONFIG_OMAP_I2C_VERBOSE_DEBUG */
	}

out:
	omap_i2c_idle(dev);
	return ret;
}

int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	struct omap_i2c_dev *dev = &(omap_i2c_devices[current_i2c_bus]);
	int ret = 0, i;

	if (alen > 2) {
		I2C_BUS_ERRPRINT(dev, "write: invalid addr len %d\n", alen);
		return 1;
	}

	if ((addr + len) > (1 << 16)) {
		I2C_BUS_ERRPRINT(dev, "write: address out of range (0x%x-0x%x)\n", addr, addr + len);
		return 1;
	}

	if (!dev->bus_initialized) {
		I2C_BUS_ERRPRINT(dev, "write: not initialized\n");
		i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
		if (!dev->bus_initialized) {
			I2C_BUS_ERRPRINT(dev, "fail init\n");
			return -EPERM;
		}
	}

	omap_i2c_unidle(dev);

	/* wait until bus not busy */
	ret = omap_i2c_wait_for_bb(dev);
	/* If timeout, try to again check after soft reset of I2C block */
	if (ret == -ETIMEDOUT)
		ret = omap_i2c_bus_clear(dev, 1, 1);
	if (ret)
		goto out;

	for (i = 0; i < len; i++) {
		ret = i2c_write_byte(dev, chip, addr + i, alen, buffer[i]);
		if (ret) {
			I2C_BUS_PRINT(dev, "write(0x%02X): [0x%02X] I/O err (%d)\n", chip, addr + i, ret);
			break;
		}
	}

#if defined(CONFIG_OMAP_I2C_VERBOSE_DEBUG)
	if (ret == 0) {
		uchar value = 0;

		/* wait until bus not busy */
		ret = omap_i2c_wait_for_bb(dev);
		/* If timeout, try to again check after soft reset of I2C block */
		if (ret == -ETIMEDOUT)
			ret = omap_i2c_bus_clear(dev, 1, 1);
		if (ret)
			goto out;

		for (i = 0; i < len; i++) {
			value = 0;
			ret = i2c_read_byte(dev, chip, addr + i, alen, &value);
			if (ret) {
				I2C_BUS_PRINT(dev, "read(0x%02X) for verify: [0x%02X] I/O err (%d)\n", chip, addr + i, ret);
				break;
			} else if (value != buffer[i]) {
				I2C_BUS_PRINT(dev, "write(0x%02X): [0x%02X] 0x%02X != 0x%02X (ERR?)\n", chip, addr + i, value, buffer[i]);
			} else {
				I2C_BUS_PRINT(dev, "write(0x%02X): [0x%02X] 0x%02X == 0x%02X\n", chip, addr + i, value, buffer[i]);
			}
		}
	}
#endif /* CONFIG_OMAP_I2C_VERBOSE_DEBUG */

out:
	omap_i2c_idle(dev);
	return ret;
}

#if defined(CONFIG_I2C_MULTI_BUS)
int i2c_set_bus_num(unsigned int bus)
{
	unsigned int speed;

	if ((bus < 0) || (bus >= I2C_BUS_MAX)) {
		I2C_ERRPRINT("bad bus %d\n", bus);
		return -1;
	}

	if (bus == 0) {
		speed = CONFIG_SYS_I2C_SPEED;
	}
#if I2C_BUS_MAX>=2
	else if (bus == 1) {
		speed = CONFIG_SYS_I2C2_SPEED;
	}
#endif
#if I2C_BUS_MAX>=3
	else if (bus == 2) {
		speed = CONFIG_SYS_I2C3_SPEED;
	}
#endif
#if I2C_BUS_MAX>=4
	else if (bus == 3) {
		speed = CONFIG_SYS_I2C4_SPEED;
	}
#endif

	if (omap_i2c_devices[bus].bus_initialized) {
		I2C_VPRINT("skip to init bus %u\n", bus);
		current_i2c_bus = bus;
	} else {
		unsigned int old_i2c_bus = current_i2c_bus;

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_INIT
		I2C_VPRINT("board init for bus %u\n", bus);
		/* Call board specific i2c bus initial routine before accessing the
		 * environment, which might be in a chip on that bus. For details
		 * about this problem see doc/I2C_Edge_Conditions.
		 */
		i2c_omap_board_init(bus);	/* current_i2c_bus not changed */
#endif

		current_i2c_bus = bus;

		i2c_init(speed, CONFIG_SYS_I2C_SLAVE);	/* current_i2c_bus is NEW (changed) */
		if (!omap_i2c_devices[current_i2c_bus].bus_initialized) {
			I2C_ERRPRINT("failed to init bus %u\n", bus);
			current_i2c_bus = old_i2c_bus;	/* current_i2c_bus is OLD (restored) */
#ifdef CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN
			I2C_VPRINT("board shutdown for bus %u\n", bus);
			i2c_omap_board_shutdown(bus);	/* current_i2c_bus is OLD */
#endif
			return -1;
		}

{
		struct omap_i2c_dev *dev = &(omap_i2c_devices[current_i2c_bus]);
		struct i2c *i2c_base = (struct i2c *)dev->i2c_base;

		omap_i2c_unidle(dev);
		if ((readw(&i2c_base->systest) & (I2C_SYSTEST_SCL_I_FUNC | I2C_SYSTEST_SDA_I_FUNC))
				!= (I2C_SYSTEST_SCL_I_FUNC | I2C_SYSTEST_SDA_I_FUNC)) {
			I2C_BUS_ERRPRINT(dev, "enforce to clear bus\n");
			omap_i2c_bus_clear(dev, 0, 0);
		}
		omap_i2c_idle(dev);
}

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_LATE_INIT
		I2C_VPRINT("board late init for bus %u\n", bus);
		/* Call board specific i2c bus initial routine before accessing the
		 * environment, which might be in a chip on that bus. For details
		 * about this problem see doc/I2C_Edge_Conditions.
		 */
		i2c_omap_board_late_init(bus);	/* current_i2c_bus is NEW */
#endif
	}


	return 0;
}

int i2c_get_bus_num(void)
{
	return (int)current_i2c_bus;
}

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN
void i2c_omap_shutdown(void)
{
	unsigned int bus;

	/* skip I2C1 (PMIC) */
	for (bus = 1 ; bus < I2C_BUS_MAX ; bus++) {
		if (omap_i2c_devices[bus].bus_initialized) {
			i2c_omap_board_shutdown(bus);
			omap_i2c_devices[bus].bus_initialized = 0;
		}
	}
}
#endif /* CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN */

#endif /* CONFIG_I2C_MULTI_BUS */

#if defined(CONFIG_CMD_I2C)
int i2c_set_bus_speed(unsigned int speed)
{
	if ((speed == 0) || (speed > OMAP_I2C_HIGH_SPEED)) {
#ifdef CONFIG_OMAP_I2C_DEBUG
		struct omap_i2c_dev *dev = &(omap_i2c_devices[current_i2c_bus]);
		I2C_BUS_ERRPRINT(dev, "invalid speed %d\n", speed);
#endif /* CONFIG_OMAP_I2C_DEBUG */
		return -1;
	}

	/* sets as side effect i2c_bus_speed[i2c_bus_num] */
	i2c_init(speed, CONFIG_SYS_I2C_SLAVE);

	return 0;
}

unsigned int i2c_get_bus_speed(void)
{
	return omap_i2c_devices[current_i2c_bus].speed;
}
#endif /* CONFIG_CMD_I2C */

#endif
