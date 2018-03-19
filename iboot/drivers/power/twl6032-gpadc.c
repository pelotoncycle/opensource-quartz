/*
 * (C) Copyright 2012
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*----------------------------------------------------------------------*/

#include <common.h>
#include <exports.h>
#include <errno.h>
#include <watchdog.h>
#include <linux/usb/otg.h>
#include <twl6030.h>
#include <linux/compiler.h> 

/*----------------------------------------------------------------------*/

/*#define CONFIG_TWL6032_GPADC_DEBUG*/
/*#define CONFIG_TWL6032_GPADC_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_TWL6032_GPADC_DEBUG
#define CONFIG_TWL6032_GPADC_DEBUG
#endif
#ifndef CONFIG_TWL6032_GPADC_VERBOSE_DEBUG
#define CONFIG_TWL6032_GPADC_VERBOSE_DEBUG
#endif
#endif /* DEBUG */

#ifdef CONFIG_TWL6032_GPADC_DEBUG
#define GPADC_DPRINT(fmt, args...) \
	do {printf("[gpadc] " fmt, ##args);} while (0)
#define GPADC_DPUTS(fmt) \
	do {puts("[gpadc] " fmt);} while (0)
#else /* CONFIG_TWL6032_GPADC_DEBUG */
#define GPADC_DPRINT(fmt, args...) \
	do {} while (0)
#define GPADC_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6032_GPADC_DEBUG */

#ifdef CONFIG_TWL6032_GPADC_VERBOSE_DEBUG
#define GPADC_VPRINT(fmt, args...) \
	do {printf("[gpadc] " fmt, ##args);} while (0)
#define GPADC_VPUTS(fmt) \
	do {puts("[gpadc] " fmt);} while (0)
#else /* CONFIG_TWL6032_GPADC_VERBOSE_DEBUG */
#define GPADC_VPRINT(fmt, args...) \
	do {} while (0)
#define GPADC_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_TWL6032_GPADC_VERBOSE_DEBUG */

#define GPADC_PRINT(fmt, args...) \
	do {printf("gpadc: " fmt, ##args);} while (0)
#define GPADC_PUTS(fmt) \
	do {puts("gpadc: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt);} while (0)
#define GPADC_ERR(fmt, args...) \
	do {printf("GPADC: " fmt, ##args);} while (0)

/*----------------------------------------------------------------------*/

#define TWL6030_GPADC_CTRL			0x00    /* 0x2e */
#define TWL6032_GPADC_CTRL2			0x01    /* 0x2F */
#define TWL6032_GPADC_RTSELECT_LSB	0x04    /* 0x32 */
#define TWL6032_GPADC_RTSELECT_ISB	0x05
#define TWL6032_GPADC_RTSELECT_MSB	0x06
#define TWL6032_GPADC_GPSELECT_ISB	0x07
#define TWL6032_GPADC_CTRL_P1		0x08
#define		TWL6030_GPADC_CTRL_P1_SP1		(1 << 3)	/* Start Process #1 */
#define		TWL6030_GPADC_CTRL_P1_EOCRT		(1 << 2)	/* End Of Conversion Real Time */
#define		TWL6030_GPADC_CTRL_P1_EOCP1		(1 << 1)	/* End Of Conversion Process #1 */
#define		TWL6030_GPADC_CTRL_P1_BUSY		(1 << 0)	/* the GPADC is running conversions */
#define		TWL6030_GPADC_EOC_SW		(1 << 1)
#define		TWL6030_GPADC_BUSY			(1 << 0)
#define TWL6032_RTCH0_LSB			0x09
#define TWL6032_RTCH0_MSB			0x0a
#define TWL6032_RTCH1_LSB			0x0b
#define TWL6032_RTCH1_MSB			0x0c
#define TWL6032_GPCH0_LSB			0x0d
#define TWL6032_GPCH0_MSB			0x0e

#define REG_TOGGLE1				0x90
#define		FGDITHS					(1 << 7)	/* FG DITH driver set signal (fuel gauge DITH digital enabled) */
#define		FGDITHR					(1 << 6)	/* FG DITH driver reset signal (fuel gauge DITH digital disabled) */
#define		FGS						(1 << 5)	/* FG driver set signal (fuel gauge digital enabled) */
#define		FGR						(1 << 4)	/* FG driver reset signal (fuel gauge digital disabled) */
#define		GPADC_SAMP_WINDOW_3US	(0 << 2)	/* 0: 3-μs sampling time window (default) */
#define		GPADC_SAMP_WINDOW_450US	(1 << 2)	/* 1: 450-μs sampling time window */
#define		GPADCS					(1 << 1)	/* GPADC set signal (GPADC enabled) */
#define		GPADCR					(1 << 0)	/* GPADC reset signal (GPADC disabled) */

/*----------------------------------------------------------------------*/

struct twl6030_gpadc_conversion_method {
	u8 sel;
	u8 rbase;
	u8 ctrl;
	u8 enable;
};

#define TWL6030_GPADC_MAX_CHANNELS 17
#define TWL6032_GPADC_MAX_CHANNELS 19
/* Define this as the biggest of all chips using this driver */
#define GPADC_MAX_CHANNELS TWL6032_GPADC_MAX_CHANNELS

/*----------------------------------------------------------------------*/

#define SCALE			(1 << 15)

/*----------------------------------------------------------------------*/

struct twl6032_chnl_calib {
	s32 gain;
	s32 gain_error;
	s32 offset_error;
};

struct twl6032_ideal_code {
	s16 code1;
	s16 code2;
	s16 v1;
	s16 v2;
};


#define TWL6032_GPADC_TRIM1	0xCD
#define TWL6032_GPADC_TRIM2	0xCE
#define TWL6032_GPADC_TRIM3	0xCF
#define TWL6032_GPADC_TRIM4	0xD0
#define TWL6032_GPADC_TRIM5	0xD1
#define TWL6032_GPADC_TRIM6	0xD2
#define TWL6032_GPADC_TRIM7	0xD3
#define TWL6032_GPADC_TRIM8	0xD4
#define TWL6032_GPADC_TRIM9	0xD5
#define TWL6032_GPADC_TRIM10	0xD6
#define TWL6032_GPADC_TRIM11	0xD7
#define TWL6032_GPADC_TRIM12	0xD8
#define TWL6032_GPADC_TRIM13	0xD9
#define TWL6032_GPADC_TRIM14	0xDA
#define TWL6032_GPADC_TRIM15	0xDB
#define TWL6032_GPADC_TRIM16	0xDC
#define TWL6032_GPADC_TRIM17	0xDD
#define		RATIO_TLO_MASK	0x07
#define TWL6032_GPADC_TRIM18	0xDE
#define		RATIO_THI_MASK	0x07
#define TWL6032_GPADC_TRIM19	0xFD

#define TWL6030_GPADC_RTCH0_LSB		(0x07)
#define TWL6030_GPADC_GPCH0_LSB		(0x29)

/*----------------------------------------------------------------------*/


/*----------------------------------------------------------------------*/

/* PhoenixLite has a different calibration sysem to the Phoenix */
static const struct twl6032_ideal_code
			twl6032_ideal[TWL6032_GPADC_MAX_CHANNELS] = {
	{	/* CHANNEL 0 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 1 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 2 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 660,
		.v2 = 1500,
	},
	{	/* CHANNEL 3 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 4 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 5 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 6 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 7 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 2200,
		.v2 = 5000,
	},
	{	/* CHANNEL 8 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 2200,
		.v2 = 5000,
	},
	{	/* CHANNEL 9 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 3960,
		.v2 = 9000,
	},
	{	/* CHANNEL 10 */
		.code1 = 150,
		.code2 = 751,
		.v1 = 1000,
		.v2 = 5000,
	},
	{	/* CHANNEL 11 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 660,
		.v2 = 1500,
	},
	{	/* CHANNEL 12 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 13 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 440,
		.v2 = 1000,
	},
	{	/* CHANNEL 14 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 2420,
		.v2 = 5500,
	},
	{},	/* CHANNEL 15 - UNUSED */
	{},	/* CHANNEL 16 - UNUSED */
	{},	/* CHANNEL 17 - UNUSED */
	{	/* CHANNEL 18 */
		.code1 = 1441,
		.code2 = 3276,
		.v1 = 2200,
		.v2 = 5000,
	},
};

static const
struct twl6030_gpadc_conversion_method twl6032_conversion_methods_table = {
	/* We only support TWL6032_GPADC_SW2 */
	.sel	= TWL6032_GPADC_GPSELECT_ISB,
	.rbase	= TWL6032_GPCH0_LSB,
	.ctrl	= TWL6032_GPADC_CTRL_P1,
	.enable = TWL6030_GPADC_CTRL_P1_SP1,
#if 0
	.mask	= TWL6032_GPADC_SW_EOC_MASK,
#endif
};

/*----------------------------------------------------------------------*/

static struct twl6032_chnl_calib twl6032_cal_tbl[TWL6032_GPADC_MAX_CHANNELS] __attribute__ ((section (".data")));

/*----------------------------------------------------------------------*/

static u8 twl6030_gpadc_read(u8 reg)
{
	u8 data = 0;
#ifdef CONFIG_TWL6032_GPADC_DEBUG
	int ret = twl_i2c_read_u8(TWL_MODULE_MADC, &data, reg);
	if (ret)
		GPADC_PRINT("readb[MADC,0x%x] err %d\n", reg, ret);
#else
	twl_i2c_read_u8(TWL_MODULE_MADC, &data, reg);
#endif /* CONFIG_TWL6032_GPADC_DEBUG */
	return data;
}

static void twl6030_gpadc_write(u8 reg, u8 val)
{
#ifdef CONFIG_TWL6032_GPADC_DEBUG
	int ret = twl_i2c_write_u8(TWL_MODULE_MADC, val, reg);
	if (ret)
		GPADC_PRINT("write[MADC,0x%x] err %d\n", reg, ret);
#else
	twl_i2c_write_u8(TWL_MODULE_MADC, val, reg);
#endif /* CONFIG_TWL6032_GPADC_DEBUG */
}

/*----------------------------------------------------------------------*/

static int twl6030_gpadc_channel_raw_read(u8 reg)
{
	u8 msb, lsb;

	/* For each ADC channel, we have MSB and LSB register pair.
	 * MSB address is always LSB address+1. reg parameter is the
	 * addr of LSB register
	 */
	msb = twl6030_gpadc_read(reg + 1);
	lsb = twl6030_gpadc_read(reg);
	return (int)((msb << 8) | lsb);
}

static void twl6030_gpadc_read_channel(u8 reg_base, u8 channel, struct twl6030_gpadc_request *req)
{
	s32 raw_code;
	s32 corrected_code;
	s32 raw_channel_value;

	GPADC_VPRINT("GPADC chn: %d\n", channel);
	raw_code = twl6030_gpadc_channel_raw_read(reg_base);
	GPADC_VPRINT("GPADC raw: %d\n", raw_code);
	req->buf.raw_code = raw_code;

	/* No correction for channels 15-17 */
	if ((channel >= 15) && (channel <= 17)) {
		raw_channel_value = raw_code;
		req->buf.code = raw_code;
		req->rbuf = raw_code;
	} else {
		raw_channel_value = (raw_code * twl6032_cal_tbl[channel].gain);

		/* Shift back into mV range */
		raw_channel_value /= 1000;

		req->buf.code = corrected_code =
			((raw_code * 1000) -
				twl6032_cal_tbl[channel].offset_error) /
			twl6032_cal_tbl[channel].gain_error;

		GPADC_VPRINT("GPADC cor: %d\n", corrected_code);

		req->rbuf = corrected_code * twl6032_cal_tbl[channel].gain;

		/* Shift back into mV range */
		req->rbuf /= 1000;
	}
	req->buf.raw_channel_value = raw_channel_value;
	GPADC_VPRINT("GPADC val: %d\n", req->rbuf);
}

/*----------------------------------------------------------------------*/

static inline void
twl6030_gpadc_start_conversion(void)
{
	const struct twl6030_gpadc_conversion_method *method = &twl6032_conversion_methods_table;

#if 0
	/* Enable GPADC */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, GPADCS, REG_TOGGLE1);
#endif

	/* We only support TWL6032_GPADC_SW2 */
	twl6030_gpadc_write(method->ctrl, method->enable);
}

static int twl6030_gpadc_is_conversion_ready(u8 status_reg)
{
	u8 reg = twl6030_gpadc_read(status_reg);
	return !(reg & TWL6030_GPADC_BUSY) && (reg & TWL6030_GPADC_EOC_SW);
}

static int twl6030_gpadc_wait_conversion_ready(ulong timeout_ms, u8 status_reg)
{
	ulong start;

	start = get_timer(0);
	do {
		if (twl6030_gpadc_is_conversion_ready(status_reg))
			return 0;
		mdelay(1);
		barrier();
	} while ((get_timer(0) - start) <= timeout_ms);

	/* one more checking against scheduler-caused timeout */
	if (twl6030_gpadc_is_conversion_ready(status_reg))
		return 0;
	else
		return -EAGAIN;
}

/*----------------------------------------------------------------------*/

int twl6030_gpadc_conversion(struct twl6030_gpadc_request *req)
{
	const struct twl6030_gpadc_conversion_method *method = &twl6032_conversion_methods_table;
	int ret;

	if (req->channel >= TWL6032_GPADC_MAX_CHANNELS) {
		GPADC_DPRINT("invalid channel %d\n", req->channel);
		return -EINVAL;
	}

	/* select the ADC channel to be read */
	twl6030_gpadc_write(method->sel, req->channel);

	twl6030_gpadc_start_conversion();

	/* Wait until conversion is ready (ctrl register is EOC) */
	ret = twl6030_gpadc_wait_conversion_ready(10, method->ctrl);
	if (ret) {
		GPADC_ERR("timeout!\n");
		goto out;
	}

	twl6030_gpadc_read_channel(method->rbase, req->channel, req);
	ret = 1;

out:
	return ret;
}

/*----------------------------------------------------------------------*/

static int twl6032_calibration(void)
{
	int chn, d1 = 0, d2 = 0, b, k, gain, x1, x2, temp;
	u8 trim_regs[17];
	int ret;

	ret = twl_i2c_read(TWL6030_MODULE_ID2, trim_regs + 1,
			TWL6032_GPADC_TRIM1, 16);
	if (ret < 0)
		return ret;

	/* Loop to calculate the value needed for returning voltages from
	 * GPADC not values.
	 *
	 * gain is calculated to 3 decimal places fixed point.
	 */
	for (chn = 0; chn < TWL6032_GPADC_MAX_CHANNELS; chn++) {

		switch (chn) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 11:
		case 12:
		case 13:
		case 14:
			/* D1 */
			d1 = (trim_regs[3] & 0x1F) << 2;
			d1 |= (trim_regs[1] & 0x06) >> 1;
			if (trim_regs[1] & 0x01)
				d1 = -d1;

			/* D2 */
			d2 = (trim_regs[4] & 0x3F) << 2;
			d2 |= (trim_regs[2] & 0x06) >> 1;
			if (trim_regs[2] & 0x01)
				d2 = -d2;
			break;
		case 8:
			/* D1 */
			temp = (trim_regs[3] & 0x1F) << 2;
			temp |= (trim_regs[1] & 0x06) >> 1;
			if (trim_regs[1] & 0x01)
				temp = -temp;

			d1 = (trim_regs[8] & 0x18) << 1;
			d1 |= (trim_regs[7] & 0x1E) >> 1;
			if (trim_regs[7] & 0x01)
				d1 = -d1;

			d1 += temp;

			/* D2 */
			temp = (trim_regs[4] & 0x3F) << 2;
			temp |= (trim_regs[2] & 0x06) >> 1;
			if (trim_regs[2] & 0x01)
				temp = -temp;

			d2 = (trim_regs[10] & 0x1F) << 2;
			d2 |= (trim_regs[8] & 0x06) >> 1;
			if (trim_regs[8] & 0x01)
				d2 = -d2;

			d2 += temp;
			break;
		case 9:
			/* D1 */
			temp = (trim_regs[3] & 0x1F) << 2;
			temp |= (trim_regs[1] & 0x06) >> 1;
			if (trim_regs[1] & 0x01)
				temp = -temp;

			d1 = (trim_regs[14] & 0x18) << 1;
			d1 |= (trim_regs[12] & 0x1E) >> 1;
			if (trim_regs[12] & 0x01)
				d1 = -d1;

			d1 += temp;

			/* D2 */
			temp = (trim_regs[4] & 0x3F) << 2;
			temp |= (trim_regs[2] & 0x06) >> 1;
			if (trim_regs[2] & 0x01)
				temp = -temp;

			d2 = (trim_regs[16] & 0x1F) << 2;
			d2 |= (trim_regs[14] & 0x06) >> 1;
			if (trim_regs[14] & 0x01)
				d2 = -d2;

			d2 += temp;
			break;
		case 10:
			/* D1 */
			d1 = (trim_regs[11] & 0x0F) << 3;
			d1 |= (trim_regs[9] & 0x0E) >> 1;
			if (trim_regs[9] & 0x01)
				d1 = -d1;

			/* D2 */
			d2 = (trim_regs[15] & 0x0F) << 3;
			d2 |= (trim_regs[13] & 0x0E) >> 1;
			if (trim_regs[13] & 0x01)
				d2 = -d2;
			break;
		case 7:
		case 18:
			/* D1 */
			temp = (trim_regs[3] & 0x1F) << 2;
			temp |= (trim_regs[1] & 0x06) >> 1;
			if (trim_regs[1] & 0x01)
				temp = -temp;

			d1 = (trim_regs[5] & 0x7E) >> 1;
			if (trim_regs[5] & 0x01)
				d1 = -d1;

			d1 += temp;

			/* D2 */
			temp = (trim_regs[4] & 0x3F) << 2;
			temp |= (trim_regs[2] & 0x06) >> 1;
			if (trim_regs[2] & 0x01)
				temp = -temp;

			d2 = (trim_regs[6] & 0xFE) >> 1;
			if (trim_regs[6] & 0x01)
				d2 = -d2;

			d2 += temp;
			break;
		default:
			/* No data for other channels */
			continue;
		}

		GPADC_VPRINT("GPADC d1   for Chn: %d = %d\n", chn, d1);
		GPADC_VPRINT("GPADC d2   for Chn: %d = %d\n", chn, d2);

		/* Gain */
		gain = ((twl6032_ideal[chn].v2 -
			twl6032_ideal[chn].v1) * 1000) /
			((twl6032_ideal[chn].code2 -
			twl6032_ideal[chn].code1));

		x1 = twl6032_ideal[chn].code1;
		x2 = twl6032_ideal[chn].code2;

		/* k */
		k = 1000 + (((d2 - d1) * 1000) / (x2 - x1));

		/* b */
		b = (d1 * 1000) - (k - 1000) * x1;

		twl6032_cal_tbl[chn].gain = gain;
		twl6032_cal_tbl[chn].gain_error = k;
		twl6032_cal_tbl[chn].offset_error = b;

		GPADC_VPRINT("GPADC x1   for Chn: %d = %d\n", chn, x1);
		GPADC_VPRINT("GPADC x2   for Chn: %d = %d\n", chn, x2);
		GPADC_VPRINT("GPADC Gain for Chn: %d = %d\n", chn, gain);
		GPADC_VPRINT("GPADC k    for Chn: %d = %d\n", chn, k);
		GPADC_VPRINT("GPADC b    for Chn: %d = %d\n", chn, b);

	}

	return 0;
}

/*----------------------------------------------------------------------*/

void twl6032_gpadc_init(void)
{
#ifdef CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
	u8 value;
#endif /* CONFIG_POWER_SUPPLY_VERBOSE_DEBUG */
	int ret;

	/* Enable GPADC */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, GPADCS, REG_TOGGLE1);

	memset(twl6032_cal_tbl, 0, sizeof(twl6032_cal_tbl));

#ifdef CONFIG_POWER_SUPPLY_VERBOSE_DEBUG
	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_ID2, &value, TWL6032_GPADC_TRIM17);
	value &= RATIO_TLO_MASK;
	GPADC_PRINT("low-temp(%d): 0.%dxVREF\n", value, 2 + value);

	value = 0;
	twl_i2c_read_u8(TWL6030_MODULE_ID2, &value, TWL6032_GPADC_TRIM18);
	value &= RATIO_THI_MASK;
	GPADC_PRINT("high-temp(%d): 0.%dxVREF\n", value, 1 + value);
#endif /* CONFIG_POWER_SUPPLY_VERBOSE_DEBUG */

	ret = twl6032_calibration();
	if (ret < 0) {
		GPADC_ERR("read calibration err %d\n", ret);
	}
}

void twl6032_gpadc_shutdown(void)
{
	/* Disable GPADC */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, GPADCR, REG_TOGGLE1);
}
