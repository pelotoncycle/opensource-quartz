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
#include <asm/arch/sys_proto.h>

#include <twl4030.h>

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

/*
 * The TWL5030 family chips include a keypad controller that supports
 * up to an 8x8 switch matrix.  The controller can issue system wakeup
 * events, since it uses only the always-on 32KiHz oscillator, and has
 * an internal state machine that decodes pressed keys, including
 * multi-key combinations.
 */
#define TWL5030_MAX_ROWS	8	/* TWL5030 hard limit */
#define TWL5030_MAX_COLS	8

/*-------------------------------------------------------------------------*/

/* Keypad registers */
#define TWL_KEYPAD_KEYP_CTRL_REG	0x00
#define		KEYP_CTRL_KBD_ON		(1 << 6)
#define		KEYP_CTRL_RP_EN			(1 << 5)
#define		KEYP_CTRL_TOLE_EN		(1 << 4)
#define		KEYP_CTRL_TOE_EN		(1 << 3)
#define		KEYP_CTRL_LK_EN			(1 << 2)
#define		KEYP_CTRL_SOFTMODEN		(1 << 1)
#define		KEYP_CTRL_SOFT_NRST		(1 << 0)
#define TWL_KEYPAD_KEY_DEB_REG		0x01
#define		KEY_TIME_VALUE(us, prescale)	(((us) / (31 << (prescale + 1))) - 1)
#define TWL_KEYPAD_LONG_KEY_REG1	0x02
#define TWL_KEYPAD_LK_PTV_REG		0x03
#define		KEYP_LK_PTV_PTV_SHIFT	5
#define		PTV_PRESCALER			4	/* arbitrary prescaler value 0..7 */
#define TWL_KEYPAD_TIME_OUT_REG1	0x04	/* LSB value */
#define TWL_KEYPAD_TIME_OUT_REG2	0x05	/* MSB value */
#define TWL_KEYPAD_KBC_REG			0x06
#define TWL_KEYPAD_KBR_REG			0x07
#define TWL_KEYPAD_KEYP_SMS			0x08
#define		KEYP_STMSTS_MASK		0x0F
#define		KEYP_STMSTS_IDLE		0x0
#define TWL_KEYPAD_FULL_CODE_7_0	0x09	/* row 0 column status */
#define TWL_KEYPAD_FULL_CODE_15_8	0x0A	/* row 1 ...           */
#define TWL_KEYPAD_FULL_CODE_23_16	0x0B	/* row 2 ...           */
#define TWL_KEYPAD_FULL_CODE_31_24	0x0C	/* row 3 ...           */
#define TWL_KEYPAD_FULL_CODE_39_32	0x0D	/* row 4 ...           */
#define TWL_KEYPAD_FULL_CODE_47_40	0x0E	/* row 5 ...           */
#define TWL_KEYPAD_FULL_CODE_55_48	0x0F	/* row 6 ...           */
#define TWL_KEYPAD_FULL_CODE_63_56	0x10	/* row 7 ...           */
#define TWL_KEYPAD_KEYP_ISR1		0x11
#define		KEYP_INT_MIS			(1 << 3)	/* miss event interrupt for KEYP_{IMR,ISR,SIR}  */
#define		KEYP_INT_TO				(1 << 2)	/* time-out interrupt for KEYP_{IMR,ISR,SIR}    */
#define		KEYP_INT_LK				(1 << 1)	/* long key interrupt for TWL_KEYPAD_KEYP_{IMR,ISR,SIR}    */
#define		KEYP_INT_KP				(1 << 0)	/* key pressed interrupt for TWL_KEYPAD_KEYP_{IMR,ISR,SIR} */
#define TWL_KEYPAD_KEYP_IMR1		0x12
#define TWL_KEYPAD_KEYP_ISR2		0x13
#define TWL_KEYPAD_KEYP_IMR2		0x14
#define TWL_KEYPAD_KEYP_SIR			0x15
#define TWL_KEYPAD_KEYP_EDR			0x16	/* edge triggers */
#define		KEYP_EDR_KP_FALLING		0x01
#define		KEYP_EDR_KP_RISING		0x02
#define		KEYP_EDR_KP_BOTH		0x03
#define		KEYP_EDR_LK_FALLING		0x04
#define		KEYP_EDR_LK_RISING		0x08
#define		KEYP_EDR_TO_FALLING		0x10
#define		KEYP_EDR_TO_RISING		0x20
#define		KEYP_EDR_MIS_FALLING	0x40
#define		KEYP_EDR_MIS_RISING		0x80
#define TWL_KEYPAD_KEYP_SIH_CTRL	0x17

#define STS_HW_CONDITIONS			0x0F
#define		STS_PWON				(1 << 0)

/*-------------------------------------------------------------------------*/

/* DebouncingPeriodTime = (DebouncingValue + 1) * (1 << (PTV + 1)) * 31.25 us */
#define TWL_KEYPAD_DEBOUNCE_TIME		12 /* ms */
#define TWL_KEYPAD_WAIT_DEBOUNCE_TIME	(TWL_KEYPAD_DEBOUNCE_TIME + 1)

/*-------------------------------------------------------------------------*/

static u32 keypad_n_rows __attribute__ ((section (".data"))) = 0;
static u32 keypad_n_cols __attribute__ ((section (".data"))) = 0;

/*-------------------------------------------------------------------------*/

static int twl5030_kpread(u8 *data, u32 reg, u8 num_bytes)
{
	int ret = twl_i2c_read_u8(TWL4030_MODULE_KEYPAD, data, reg, num_bytes);
	if (ret)
		KEYPAD_DPRINT("readb[0x%x,0x%x] err %d\n", TWL4030_MODULE_KEYPAD, reg, ret);
	return ret;
}

static int twl5030_kpwrite_u8(u8 data, u8 reg)
{
	int ret = twl_i2c_write_u8(TWL4030_MODULE_KEYPAD, data, reg);
	if (ret)
		KEYPAD_DPRINT("write[0x%x,0x%x] err %d\n", TWL4030_MODULE_KEYPAD, reg, ret);
	return ret;
}

/*-------------------------------------------------------------------------*/

void keypad_poll(u32* row_bits, u32* col_bits)
{
	int ret;
	u32 row = 0, col = 0;
#ifdef CONFIG_TWL5030_POWER
	u8 hw_state = 0;
#endif /* CONFIG_TWL5030_POWER */
	u8 key_state[TWL5030_MAX_ROWS];

	memset(key_state, 0, sizeof(key_state));
	ret = twl5030_kpread(key_state, TWL_KEYPAD_FULL_CODE_7_0, (u8)keypad_n_rows);
	if (!ret) {
		int r;
		u8 col_maskbits = (1 << keypad_n_cols) - 1;

		for (r = 0; r < keypad_n_rows; r++) {
			if (key_state[r])
				row |= (1 << r);
			col |= (key_state[r] & col_maskbits);
			KEYPAD_VPRINT("key[ROW%u]: 0x%02X, row: 0x%02X, col: 0x%02X\n",
					r, key_state[r], row, col);
		}
	}

#ifdef CONFIG_TWL5030_POWER
	twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &hw_state, STS_HW_CONDITIONS);
	if (hw_state & STS_PWON) {
		row |= CONFIG_KEYPAD_PWRBUTTON_ROW;
		col |= CONFIG_KEYPAD_PWRBUTTON_COL;
	}
#endif /* CONFIG_TWL5030_POWER */

	if (row_bits)	*row_bits = row;
	if (col_bits)	*col_bits = col;

#ifdef CONFIG_KEYPAD_DEBUG
	if (row || col) {
		KEYPAD_PRINT("power button: %s (0x%02X), row_bits: 0x%03X, col_bits: 0x%03X\n",
				(hw_state & STS_PWON) ? "YES" : "NO", hw_state, row, col);
	}
#endif /* CONFIG_KEYPAD_DEBUG */
}

void keypad_init(u32 n_rows, u32 n_cols)
{
	if (n_rows > TWL5030_MAX_ROWS) {
		KEYPAD_ERR("invalid rows");
		n_rows = TWL5030_MAX_ROWS;
	}
	if (n_cols > TWL5030_MAX_COLS) {
		KEYPAD_ERR("invalid columns");
		n_cols = TWL5030_MAX_COLS;
	}

	keypad_n_rows = n_rows;
	keypad_n_cols = n_cols;

#if (defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_BOARD_INIT) && \
		defined(CONFIG_SPL_INPUT_SUPPORT)) || \
	(!defined(CONFIG_SPL_BUILD) && !defined(CONFIG_SPL_BOARD_INIT) && \
		!defined(CONFIG_SPL_INPUT_SUPPORT))
	/* Keypad controller initialization */
	/* Enable controller, with hardware decoding but not autorepeat */
	value = KEYP_CTRL_SOFT_NRST | KEYP_CTRL_SOFTMODEN | KEYP_CTRL_KBD_ON;
	if (twl5030_kpwrite_u8(value, TWL_KEYPAD_KEYP_CTRL_REG))
		return;

	/* Set PTV prescaler Field */
	value = (PTV_PRESCALER << KEYP_LK_PTV_PTV_SHIFT);
	if (twl5030_kpwrite_u8(value, TWL_KEYPAD_LK_PTV_REG))
		return;

	/* Set key debounce time (us) */
	value = (u8)KEY_TIME_VALUE(TWL_KEYPAD_DEBOUNCE_TIME * 1000, PTV_PRESCALER);
	if (twl5030_kpwrite_u8(value, TWL_KEYPAD_KEY_DEB_REG))
		return;

	/* Wait for the debounce time */
	mdelay(TWL_KEYPAD_WAIT_DEBOUNCE_TIME);
#endif
}
