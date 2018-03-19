/*
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <exports.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>

#include <twl6030.h>

#include <icom/keypad.h>

/*-------------------------------------------------------------------------*/

/*#define CONFIG_KEYPAD_DEBUG*/
/*#define CONFIG_KEYPAD_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_KEYPAD_DEBUG
#define CONFIG_KEYPAD_DEBUG
#endif /* CONFIG_KEYPAD_DEBUG */
#ifndef CONFIG_KEYPAD_VERBOSE_DEBUG
#define CONFIG_KEYPAD_VERBOSE_DEBUG
#endif /* CONFIG_KEYPAD_VERBOSE_DEBUG */
#endif

#ifdef CONFIG_KEYPAD_DEBUG
#define KEYPAD_DPRINT(fmt, args...) \
	do {printf("[keypad] " fmt, ##args);} while (0)
#define KEYPAD_DPUTS(fmt) \
	do {puts("[keypad] " fmt);} while (0)
#else /* !CONFIG_KEYPAD_DEBUG */
#define KEYPAD_DPRINT(fmt, args...) \
	do {} while (0)
#define KEYPAD_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_KEYPAD_DEBUG */

#ifdef CONFIG_KEYPAD_VERBOSE_DEBUG
#define KEYPAD_VPRINT(fmt, args...) \
	do {printf("[keypad] " fmt, ##args);} while (0)
#define KEYPAD_VPUTS(fmt) \
	do {puts("[keypad] " fmt);} while (0)
#else /* !CONFIG_KEYPAD_VERBOSE_DEBUG */
#define KEYPAD_VPRINT(fmt, args...) \
	do {} while (0)
#define KEYPAD_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_KEYPAD_VERBOSE_DEBUG */

#define KEYPAD_PRINT(fmt, args...) \
	do {printf("Keypad: " fmt, ##args);} while (0)
#define KEYPAD_PUTS(fmt) \
	do {puts("Keypad: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define KEYPAD_ERR(fmt, args...) \
	do {printf("Keypad: " fmt, ##args);} while (0)

/*-------------------------------------------------------------------------*/

#define OMAP4_MAX_ROWS		8	/* OMAP4 hard limit is 9 */
#define OMAP4_MAX_COLS		8	/* OMAP4 hard limit is 9 */

/*-------------------------------------------------------------------------*/

/* OMAP4 registers */
#define OMAP4_KBD_REVISION			(KBD_BASE + 0x00)
#define OMAP4_KBD_SYSCONFIG			(KBD_BASE + 0x10)
#define OMAP4_KBD_SYSSTATUS			(KBD_BASE + 0x14)
#define OMAP4_KBD_IRQSTATUS			(KBD_BASE + 0x18)
#define OMAP4_KBD_IRQENABLE			(KBD_BASE + 0x1C)
#define		KBD_IRQ_EVENT				(1 << 0)
#define OMAP4_KBD_WAKEUPENABLE		(KBD_BASE + 0x20)
#define OMAP4_KBD_PENDING			(KBD_BASE + 0x24)
#define OMAP4_KBD_CTRL				(KBD_BASE + 0x28)
#define		KBD_CTRL_PTV_SHIFT			2
#define		KBD_CTRL_PTV				0x6	/* arbitrary prescaler value 0..7 */
#define		KBD_CTRL_NSOFTWARE_MODE		(1 << 1)
#define OMAP4_KBD_DEBOUNCINGTIME	(KBD_BASE + 0x2C)
#define		KEY_TIME_VALUE(us, prescale)	(((us) / (31 << (prescale + 1))) - 1)
#define OMAP4_KBD_LONGKEYTIME		(KBD_BASE + 0x30)
#define OMAP4_KBD_TIMEOUT			(KBD_BASE + 0x34)
#define OMAP4_KBD_STATEMACHINE		(KBD_BASE + 0x38)
#define OMAP4_KBD_ROWINPUTS			(KBD_BASE + 0x3C)
#define OMAP4_KBD_COLUMNOUTPUTS		(KBD_BASE + 0x40)
#define OMAP4_KBD_FULLCODE31_0		(KBD_BASE + 0x44)
#define OMAP4_KBD_FULLCODE63_32		(KBD_BASE + 0x48)
#define OMAP4_KBD_FULLCODE17_0		(KBD_BASE + 0x4C)
#define OMAP4_KBD_FULLCODE35_18		(KBD_BASE + 0x50)
#define OMAP4_KBD_FULLCODE53_36		(KBD_BASE + 0x54)
#define OMAP4_KBD_FULLCODE71_54		(KBD_BASE + 0x58)
#define OMAP4_KBD_FULLCODE80_72		(KBD_BASE + 0x5C)

/*-------------------------------------------------------------------------*/

/* TWL6032 registers */
#define STS_HW_CONDITIONS		0x21
#define		STS_PWRON			(1 << 0)	/* Level status of PWRON button */
#define KEY_PRESS_DURATION_CFG	0x2D
#define		KPD_STS				(1 << 7)	/* Key press duration status */
#define		LPK_TIME_8S			(1 << 6)	/* Long press key time before switch off/restart the device */
#define		KPD_DURATION(ms)	((((ms) + 49) / 50) & 0x1F)	/* Key press duration value, from 50ms~1.55secs, by 50-ms step */

/*-------------------------------------------------------------------------*/

/* DebouncingPeriodTime = (DebouncingValue + 1) * (1 << (PTV + 1)) * 31.25 us */
#define	OMAP4_KEYPAD_DEBOUNCE_TIME			12 /* ms */
#define OMAP4_KEYPAD_WAIT_DEBOUNCE_TIME		(OMAP4_KEYPAD_DEBOUNCE_TIME + 1)

/*-------------------------------------------------------------------------*/

static u32 keypad_n_rows __attribute__ ((section (".data"))) = 0;
static u32 keypad_n_cols __attribute__ ((section (".data"))) = 0;

/*-------------------------------------------------------------------------*/

void keypad_poll(u32* row_bits, u32* col_bits)
{
	u32 state;
	u32 row = 0, col = 0;
#ifdef CONFIG_TWL6030_POWER
	u8 hw_state = 0, kpd = 0;
#endif /* CONFIG_TWL6030_POWER */

	state = __raw_readl(OMAP4_KBD_STATEMACHINE);
#if 0
	KEYPAD_VPRINT("state machine: 0x%02X irqstatus: 0x%02X\n", state, __raw_readl(OMAP4_KBD_IRQSTATUS));
#endif
	if (state) {
		u32 r;
		u8 col_maskbits = (1 << keypad_n_cols) - 1;
		u8 key_state[8];
		u32 *new_state = (u32 *)key_state;

		*new_state = __raw_readl(OMAP4_KBD_FULLCODE31_0);
		*(new_state + 1) = __raw_readl(OMAP4_KBD_FULLCODE63_32);

		for (r = 0; r < keypad_n_rows; r++) {
			if (key_state[r])
				row |= (1 << r);
			col |= (key_state[r] & col_maskbits);
			KEYPAD_VPRINT("key[ROW%u]: 0x%02X, row: 0x%02X, col: 0x%02X\n",
					r, key_state[r], row, col);
		}
	}

#ifdef CONFIG_TWL6030_POWER
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);
	if (!(hw_state & STS_PWRON)) {
		row |= CONFIG_KEYPAD_PWRBUTTON_ROW;
		col |= CONFIG_KEYPAD_PWRBUTTON_COL;
	}
	/* Clear KPD_STS bit on read */
	twl_i2c_read_u8(TWL6030_MODULE_ID0, &kpd, KEY_PRESS_DURATION_CFG);
#endif /* CONFIG_TWL6030_POWER */

	if (row_bits)	*row_bits = row;
	if (col_bits)	*col_bits = col;

#ifdef CONFIG_KEYPAD_DEBUG
	if (row || col) {
#ifdef CONFIG_TWL6030_POWER
		KEYPAD_PRINT("power button: %s (0x%02X), row_bits: 0x%03X, col_bits: 0x%03X\n",
				(hw_state & STS_PWRON) ? "NO" : "YES", hw_state, row, col);
#else
		KEYPAD_PRINT("power button: X, row_bits: 0x%03X, col_bits: 0x%03X\n", row, col);
#endif /* CONFIG_TWL6030_POWER */
	}
#endif /* CONFIG_KEYPAD_DEBUG */

	/* Clear pending interrupts */
	__raw_writel(__raw_readl(OMAP4_KBD_IRQSTATUS), OMAP4_KBD_IRQSTATUS);
}

void keypad_init(u32 n_rows, u32 n_cols)
{
	if (n_rows > OMAP4_MAX_ROWS) {
		KEYPAD_ERR("invalid rows");
		n_rows = OMAP4_MAX_ROWS;
	}
	if (n_cols > OMAP4_MAX_COLS) {
		KEYPAD_ERR("invalid columns");
		n_cols = OMAP4_MAX_COLS;
	}

	/* Enable "kbd_fck" clock */
	omap_clk_enable(&prcm->cm_wkup_keyboard_clkctrl,
			MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	keypad_n_rows = n_rows;
	keypad_n_cols = n_cols;

#if (defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_BOARD_INIT) && \
		defined(CONFIG_SPL_INPUT_SUPPORT)) || \
	(!defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SPL_BOARD_INIT) && \
		!defined(CONFIG_SPL_INPUT_SUPPORT))
#ifdef CONFIG_TWL6030_POWER
	{
		u8 kpd = 0;

		/* Clear KPD_STS bit on read */
		twl_i2c_read_u8(TWL6030_MODULE_ID0, &kpd, KEY_PRESS_DURATION_CFG);

		/* LPK_TIME 8secs & 50ms key press duration time */
		twl_i2c_write_u8(TWL6030_MODULE_ID0, LPK_TIME_8S | KPD_DURATION(50),
				KEY_PRESS_DURATION_CFG);
	}
#endif /* CONFIG_TWL6030_POWER */

	/* Keypad controller initialization */
	__raw_writel(KBD_CTRL_NSOFTWARE_MODE |
			(KBD_CTRL_PTV << KBD_CTRL_PTV_SHIFT),
			OMAP4_KBD_CTRL);

	__raw_writel(KEY_TIME_VALUE(OMAP4_KEYPAD_DEBOUNCE_TIME * 1000, KBD_CTRL_PTV),
			OMAP4_KBD_DEBOUNCINGTIME);

	/* Disable interrupts */
	__raw_writel(0, OMAP4_KBD_IRQENABLE);

	/* Clear pending interrupts */
	__raw_writel(__raw_readl(OMAP4_KBD_IRQSTATUS), OMAP4_KBD_IRQSTATUS);

	/* We don't allow keypad to wakeup the device */
	__raw_writel(0, OMAP4_KBD_WAKEUPENABLE);

	/* Wait for the debounce time */
	mdelay(OMAP4_KEYPAD_WAIT_DEBOUNCE_TIME);
#endif
}
