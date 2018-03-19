/**
 * bma2x2.c -This file contains all function implementations for the BMA2X2 in linux
 *
 * Date: 2011/12/2 17:00:00
 * Revision: 1.0
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "../../../arch/arm/mach-omap2/board-44xx-zeus.h"

#define DEVICE_NAME 		ZEUS_GSENSOR_DRIVER_NAME

/**
 * Debug printk
 */
#define ERR -1
//#define DEBUG
#ifdef DEBUG
#define dprintk(fmt, arg...) \
	do { printk(KERN_INFO "[%s] " fmt, DEVICE_NAME, ##arg); } while (false)
#else
#define dprintk(fmt, arg...) \
	do { } while (false)
#endif

#define ABSMIN				-512
#define ABSMAX				512
#define SLOPE_THRESHOLD_VALUE 		32
#define SLOPE_DURATION_VALUE 		1
#define INTERRUPT_LATCH_MODE 		13
#define INTERRUPT_ENABLE 		1
#define INTERRUPT_DISABLE 		0
#define MAP_SLOPE_INTERRUPT 	2
#define SLOPE_X_INDEX 			5
#define SLOPE_Y_INDEX 			6
#define SLOPE_Z_INDEX 			7
#define BMA2X2_MAX_DELAY		200
#define BMA2X2_CHIP_ID			0xfa
#define BMA2X2_RANGE_SET		3  /* +/- 2G */
#define BMA2X2_BW_SET			12 /* 125HZ  */

#define LOW_G_INTERRUPT				REL_Z
#define HIGH_G_INTERRUPT 			REL_HWHEEL
#define SLOP_INTERRUPT 				REL_DIAL
#define DOUBLE_TAP_INTERRUPT 			REL_WHEEL
#define SINGLE_TAP_INTERRUPT 			REL_MISC
#define ORIENT_INTERRUPT 			ABS_PRESSURE
#define FLAT_INTERRUPT 				ABS_DISTANCE


#define HIGH_G_INTERRUPT_X_HAPPENED			1
#define HIGH_G_INTERRUPT_Y_HAPPENED 			2
#define HIGH_G_INTERRUPT_Z_HAPPENED 			3
#define HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED 		4
#define HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED		5
#define HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED 		6
#define SLOPE_INTERRUPT_X_HAPPENED 			7
#define SLOPE_INTERRUPT_Y_HAPPENED 			8
#define SLOPE_INTERRUPT_Z_HAPPENED 			9
#define SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED 		10
#define SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED 		11
#define SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED 		12
#define DOUBLE_TAP_INTERRUPT_HAPPENED 			13
#define SINGLE_TAP_INTERRUPT_HAPPENED 			14
#define UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED 		15
#define UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED	 	16
#define UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED 	17
#define UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED	18
#define DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED 	19
#define DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED 	20
#define DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED 	21
#define DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED 	22
#define FLAT_INTERRUPT_TURE_HAPPENED			23
#define FLAT_INTERRUPT_FALSE_HAPPENED			24
#define LOW_G_INTERRUPT_HAPPENED			25

#define PAD_LOWG					0
#define PAD_HIGHG					1
#define PAD_SLOP					2
#define PAD_DOUBLE_TAP					3
#define PAD_SINGLE_TAP					4
#define PAD_ORIENT					5
#define PAD_FLAT					6


#define BMA2X2_EEP_OFFSET                       0x16
#define BMA2X2_IMAGE_BASE                       0x38
#define BMA2X2_IMAGE_LEN                        22


#define BMA2X2_CHIP_ID_REG                      0x00
#define BMA2X2_VERSION_REG                      0x01
#define BMA2X2_X_AXIS_LSB_REG                   0x02
#define BMA2X2_X_AXIS_MSB_REG                   0x03
#define BMA2X2_Y_AXIS_LSB_REG                   0x04
#define BMA2X2_Y_AXIS_MSB_REG                   0x05
#define BMA2X2_Z_AXIS_LSB_REG                   0x06
#define BMA2X2_Z_AXIS_MSB_REG                   0x07
#define BMA2X2_TEMP_RD_REG                      0x08
#define BMA2X2_STATUS1_REG                      0x09
#define BMA2X2_STATUS2_REG                      0x0A
#define BMA2X2_STATUS_TAP_SLOPE_REG             0x0B
#define BMA2X2_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA2X2_STATUS_FIFO_REG                  0x0E
#define BMA2X2_RANGE_SEL_REG                    0x0F
#define BMA2X2_BW_SEL_REG                       0x10
#define BMA2X2_MODE_CTRL_REG                    0x11
#define BMA2X2_LOW_NOISE_CTRL_REG               0x12
#define BMA2X2_DATA_CTRL_REG                    0x13
#define BMA2X2_RESET_REG                        0x14
#define BMA2X2_INT_ENABLE1_REG                  0x16
#define BMA2X2_INT_ENABLE2_REG                  0x17
#define BMA2X2_INT_SLO_NO_MOT_REG               0x18
#define BMA2X2_INT1_PAD_SEL_REG                 0x19
#define BMA2X2_INT_DATA_SEL_REG                 0x1A
#define BMA2X2_INT2_PAD_SEL_REG                 0x1B
#define BMA2X2_INT_SRC_REG                      0x1E
#define BMA2X2_INT_SET_REG                      0x20
#define BMA2X2_INT_CTRL_REG                     0x21
#define BMA2X2_LOW_DURN_REG                     0x22
#define BMA2X2_LOW_THRES_REG                    0x23
#define BMA2X2_LOW_HIGH_HYST_REG                0x24
#define BMA2X2_HIGH_DURN_REG                    0x25
#define BMA2X2_HIGH_THRES_REG                   0x26
#define BMA2X2_SLOPE_DURN_REG                   0x27
#define BMA2X2_SLOPE_THRES_REG                  0x28
#define BMA2X2_SLO_NO_MOT_THRES_REG             0x29
#define BMA2X2_TAP_PARAM_REG                    0x2A
#define BMA2X2_TAP_THRES_REG                    0x2B
#define BMA2X2_ORIENT_PARAM_REG                 0x2C
#define BMA2X2_THETA_BLOCK_REG                  0x2D
#define BMA2X2_THETA_FLAT_REG                   0x2E
#define BMA2X2_FLAT_HOLD_TIME_REG               0x2F
#define BMA2X2_FIFO_WML_TRIG                    0x30
#define BMA2X2_SELF_TEST_REG                    0x32
#define BMA2X2_EEPROM_CTRL_REG                  0x33
#define BMA2X2_SERIAL_CTRL_REG                  0x34
#define BMA2X2_EXTMODE_CTRL_REG                 0x35
#define BMA2X2_OFFSET_CTRL_REG                  0x36
#define BMA2X2_OFFSET_PARAMS_REG                0x37
#define BMA2X2_OFFSET_X_AXIS_REG                0x38
#define BMA2X2_OFFSET_Y_AXIS_REG                0x39
#define BMA2X2_OFFSET_Z_AXIS_REG                0x3A
#define BMA2X2_GP0_REG                          0x3B
#define BMA2X2_GP1_REG                          0x3C
#define BMA2X2_FIFO_MODE_REG                    0x3E
#define BMA2X2_FIFO_DATA_OUTPUT_REG             0x3F




#define BMA2X2_CHIP_ID__POS             0
#define BMA2X2_CHIP_ID__MSK             0xFF
#define BMA2X2_CHIP_ID__LEN             8
#define BMA2X2_CHIP_ID__REG             BMA2X2_CHIP_ID_REG

#define BMA2X2_VERSION__POS          0
#define BMA2X2_VERSION__LEN          8
#define BMA2X2_VERSION__MSK          0xFF
#define BMA2X2_VERSION__REG          BMA2X2_VERSION_REG



#define BMA2X2_NEW_DATA_X__POS          0
#define BMA2X2_NEW_DATA_X__LEN          1
#define BMA2X2_NEW_DATA_X__MSK          0x01
#define BMA2X2_NEW_DATA_X__REG          BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X14_LSB__POS           2
#define BMA2X2_ACC_X14_LSB__LEN           6
#define BMA2X2_ACC_X14_LSB__MSK           0xFC
#define BMA2X2_ACC_X14_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X12_LSB__POS           4
#define BMA2X2_ACC_X12_LSB__LEN           4
#define BMA2X2_ACC_X12_LSB__MSK           0xF0
#define BMA2X2_ACC_X12_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X10_LSB__POS           6
#define BMA2X2_ACC_X10_LSB__LEN           2
#define BMA2X2_ACC_X10_LSB__MSK           0xC0
#define BMA2X2_ACC_X10_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X8_LSB__POS           0
#define BMA2X2_ACC_X8_LSB__LEN           0
#define BMA2X2_ACC_X8_LSB__MSK           0x00
#define BMA2X2_ACC_X8_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X_MSB__POS           0
#define BMA2X2_ACC_X_MSB__LEN           8
#define BMA2X2_ACC_X_MSB__MSK           0xFF
#define BMA2X2_ACC_X_MSB__REG           BMA2X2_X_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Y__POS          0
#define BMA2X2_NEW_DATA_Y__LEN          1
#define BMA2X2_NEW_DATA_Y__MSK          0x01
#define BMA2X2_NEW_DATA_Y__REG          BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y14_LSB__POS           2
#define BMA2X2_ACC_Y14_LSB__LEN           6
#define BMA2X2_ACC_Y14_LSB__MSK           0xFC
#define BMA2X2_ACC_Y14_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y12_LSB__POS           4
#define BMA2X2_ACC_Y12_LSB__LEN           4
#define BMA2X2_ACC_Y12_LSB__MSK           0xF0
#define BMA2X2_ACC_Y12_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y10_LSB__POS           6
#define BMA2X2_ACC_Y10_LSB__LEN           2
#define BMA2X2_ACC_Y10_LSB__MSK           0xC0
#define BMA2X2_ACC_Y10_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y8_LSB__POS           0
#define BMA2X2_ACC_Y8_LSB__LEN           0
#define BMA2X2_ACC_Y8_LSB__MSK           0x00
#define BMA2X2_ACC_Y8_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y_MSB__POS           0
#define BMA2X2_ACC_Y_MSB__LEN           8
#define BMA2X2_ACC_Y_MSB__MSK           0xFF
#define BMA2X2_ACC_Y_MSB__REG           BMA2X2_Y_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Z__POS          0
#define BMA2X2_NEW_DATA_Z__LEN          1
#define BMA2X2_NEW_DATA_Z__MSK          0x01
#define BMA2X2_NEW_DATA_Z__REG          BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z14_LSB__POS           2
#define BMA2X2_ACC_Z14_LSB__LEN           6
#define BMA2X2_ACC_Z14_LSB__MSK           0xFC
#define BMA2X2_ACC_Z14_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z12_LSB__POS           4
#define BMA2X2_ACC_Z12_LSB__LEN           4
#define BMA2X2_ACC_Z12_LSB__MSK           0xF0
#define BMA2X2_ACC_Z12_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z10_LSB__POS           6
#define BMA2X2_ACC_Z10_LSB__LEN           2
#define BMA2X2_ACC_Z10_LSB__MSK           0xC0
#define BMA2X2_ACC_Z10_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z8_LSB__POS           0
#define BMA2X2_ACC_Z8_LSB__LEN           0
#define BMA2X2_ACC_Z8_LSB__MSK           0x00
#define BMA2X2_ACC_Z8_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z_MSB__POS           0
#define BMA2X2_ACC_Z_MSB__LEN           8
#define BMA2X2_ACC_Z_MSB__MSK           0xFF
#define BMA2X2_ACC_Z_MSB__REG           BMA2X2_Z_AXIS_MSB_REG

#define BMA2X2_TEMPERATURE__POS         0
#define BMA2X2_TEMPERATURE__LEN         8
#define BMA2X2_TEMPERATURE__MSK         0xFF
#define BMA2X2_TEMPERATURE__REG         BMA2X2_TEMP_RD_REG

#define BMA2X2_LOWG_INT_S__POS          0
#define BMA2X2_LOWG_INT_S__LEN          1
#define BMA2X2_LOWG_INT_S__MSK          0x01
#define BMA2X2_LOWG_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_HIGHG_INT_S__POS          1
#define BMA2X2_HIGHG_INT_S__LEN          1
#define BMA2X2_HIGHG_INT_S__MSK          0x02
#define BMA2X2_HIGHG_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_SLOPE_INT_S__POS          2
#define BMA2X2_SLOPE_INT_S__LEN          1
#define BMA2X2_SLOPE_INT_S__MSK          0x04
#define BMA2X2_SLOPE_INT_S__REG          BMA2X2_STATUS1_REG


#define BMA2X2_SLO_NO_MOT_INT_S__POS          3
#define BMA2X2_SLO_NO_MOT_INT_S__LEN          1
#define BMA2X2_SLO_NO_MOT_INT_S__MSK          0x08
#define BMA2X2_SLO_NO_MOT_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_DOUBLE_TAP_INT_S__POS     4
#define BMA2X2_DOUBLE_TAP_INT_S__LEN     1
#define BMA2X2_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA2X2_DOUBLE_TAP_INT_S__REG     BMA2X2_STATUS1_REG

#define BMA2X2_SINGLE_TAP_INT_S__POS     5
#define BMA2X2_SINGLE_TAP_INT_S__LEN     1
#define BMA2X2_SINGLE_TAP_INT_S__MSK     0x20
#define BMA2X2_SINGLE_TAP_INT_S__REG     BMA2X2_STATUS1_REG

#define BMA2X2_ORIENT_INT_S__POS         6
#define BMA2X2_ORIENT_INT_S__LEN         1
#define BMA2X2_ORIENT_INT_S__MSK         0x40
#define BMA2X2_ORIENT_INT_S__REG         BMA2X2_STATUS1_REG

#define BMA2X2_FLAT_INT_S__POS           7
#define BMA2X2_FLAT_INT_S__LEN           1
#define BMA2X2_FLAT_INT_S__MSK           0x80
#define BMA2X2_FLAT_INT_S__REG           BMA2X2_STATUS1_REG

#define BMA2X2_FIFO_FULL_INT_S__POS           5
#define BMA2X2_FIFO_FULL_INT_S__LEN           1
#define BMA2X2_FIFO_FULL_INT_S__MSK           0x20
#define BMA2X2_FIFO_FULL_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_FIFO_WM_INT_S__POS           6
#define BMA2X2_FIFO_WM_INT_S__LEN           1
#define BMA2X2_FIFO_WM_INT_S__MSK           0x40
#define BMA2X2_FIFO_WM_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_DATA_INT_S__POS           7
#define BMA2X2_DATA_INT_S__LEN           1
#define BMA2X2_DATA_INT_S__MSK           0x80
#define BMA2X2_DATA_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_SLOPE_FIRST_X__POS        0
#define BMA2X2_SLOPE_FIRST_X__LEN        1
#define BMA2X2_SLOPE_FIRST_X__MSK        0x01
#define BMA2X2_SLOPE_FIRST_X__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_FIRST_Y__POS        1
#define BMA2X2_SLOPE_FIRST_Y__LEN        1
#define BMA2X2_SLOPE_FIRST_Y__MSK        0x02
#define BMA2X2_SLOPE_FIRST_Y__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_FIRST_Z__POS        2
#define BMA2X2_SLOPE_FIRST_Z__LEN        1
#define BMA2X2_SLOPE_FIRST_Z__MSK        0x04
#define BMA2X2_SLOPE_FIRST_Z__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_SIGN_S__POS         3
#define BMA2X2_SLOPE_SIGN_S__LEN         1
#define BMA2X2_SLOPE_SIGN_S__MSK         0x08
#define BMA2X2_SLOPE_SIGN_S__REG         BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_X__POS        4
#define BMA2X2_TAP_FIRST_X__LEN        1
#define BMA2X2_TAP_FIRST_X__MSK        0x10
#define BMA2X2_TAP_FIRST_X__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_Y__POS        5
#define BMA2X2_TAP_FIRST_Y__LEN        1
#define BMA2X2_TAP_FIRST_Y__MSK        0x20
#define BMA2X2_TAP_FIRST_Y__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_Z__POS        6
#define BMA2X2_TAP_FIRST_Z__LEN        1
#define BMA2X2_TAP_FIRST_Z__MSK        0x40
#define BMA2X2_TAP_FIRST_Z__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_SIGN_S__POS         7
#define BMA2X2_TAP_SIGN_S__LEN         1
#define BMA2X2_TAP_SIGN_S__MSK         0x80
#define BMA2X2_TAP_SIGN_S__REG         BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_HIGHG_FIRST_X__POS        0
#define BMA2X2_HIGHG_FIRST_X__LEN        1
#define BMA2X2_HIGHG_FIRST_X__MSK        0x01
#define BMA2X2_HIGHG_FIRST_X__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_FIRST_Y__POS        1
#define BMA2X2_HIGHG_FIRST_Y__LEN        1
#define BMA2X2_HIGHG_FIRST_Y__MSK        0x02
#define BMA2X2_HIGHG_FIRST_Y__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_FIRST_Z__POS        2
#define BMA2X2_HIGHG_FIRST_Z__LEN        1
#define BMA2X2_HIGHG_FIRST_Z__MSK        0x04
#define BMA2X2_HIGHG_FIRST_Z__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_SIGN_S__POS         3
#define BMA2X2_HIGHG_SIGN_S__LEN         1
#define BMA2X2_HIGHG_SIGN_S__MSK         0x08
#define BMA2X2_HIGHG_SIGN_S__REG         BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_ORIENT_S__POS             4
#define BMA2X2_ORIENT_S__LEN             3
#define BMA2X2_ORIENT_S__MSK             0x70
#define BMA2X2_ORIENT_S__REG             BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_FLAT_S__POS               7
#define BMA2X2_FLAT_S__LEN               1
#define BMA2X2_FLAT_S__MSK               0x80
#define BMA2X2_FLAT_S__REG               BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_FIFO_FRAME_COUNTER_S__POS             0
#define BMA2X2_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA2X2_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA2X2_FIFO_FRAME_COUNTER_S__REG             BMA2X2_STATUS_FIFO_REG

#define BMA2X2_FIFO_OVERRUN_S__POS             7
#define BMA2X2_FIFO_OVERRUN_S__LEN             1
#define BMA2X2_FIFO_OVERRUN_S__MSK             0x80
#define BMA2X2_FIFO_OVERRUN_S__REG             BMA2X2_STATUS_FIFO_REG

#define BMA2X2_RANGE_SEL__POS             0
#define BMA2X2_RANGE_SEL__LEN             4
#define BMA2X2_RANGE_SEL__MSK             0x0F
#define BMA2X2_RANGE_SEL__REG             BMA2X2_RANGE_SEL_REG

#define BMA2X2_BANDWIDTH__POS             0
#define BMA2X2_BANDWIDTH__LEN             5
#define BMA2X2_BANDWIDTH__MSK             0x1F
#define BMA2X2_BANDWIDTH__REG             BMA2X2_BW_SEL_REG

#define BMA2X2_SLEEP_DUR__POS             1
#define BMA2X2_SLEEP_DUR__LEN             4
#define BMA2X2_SLEEP_DUR__MSK             0x1E
#define BMA2X2_SLEEP_DUR__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_MODE_CTRL__POS             5
#define BMA2X2_MODE_CTRL__LEN             3
#define BMA2X2_MODE_CTRL__MSK             0xE0
#define BMA2X2_MODE_CTRL__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_DEEP_SUSPEND__POS          5
#define BMA2X2_DEEP_SUSPEND__LEN          1
#define BMA2X2_DEEP_SUSPEND__MSK          0x20
#define BMA2X2_DEEP_SUSPEND__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_LOW_POWER__POS          6
#define BMA2X2_EN_LOW_POWER__LEN          1
#define BMA2X2_EN_LOW_POWER__MSK          0x40
#define BMA2X2_EN_LOW_POWER__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_SUSPEND__POS            7
#define BMA2X2_EN_SUSPEND__LEN            1
#define BMA2X2_EN_SUSPEND__MSK            0x80
#define BMA2X2_EN_SUSPEND__REG            BMA2X2_MODE_CTRL_REG

#define BMA2X2_SLEEP_TIMER__POS          5
#define BMA2X2_SLEEP_TIMER__LEN          1
#define BMA2X2_SLEEP_TIMER__MSK          0x20
#define BMA2X2_SLEEP_TIMER__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_LOW_POWER_MODE__POS          6
#define BMA2X2_LOW_POWER_MODE__LEN          1
#define BMA2X2_LOW_POWER_MODE__MSK          0x40
#define BMA2X2_LOW_POWER_MODE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_EN_LOW_NOISE__POS          7
#define BMA2X2_EN_LOW_NOISE__LEN          1
#define BMA2X2_EN_LOW_NOISE__MSK          0x80
#define BMA2X2_EN_LOW_NOISE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_DIS_SHADOW_PROC__POS       6
#define BMA2X2_DIS_SHADOW_PROC__LEN       1
#define BMA2X2_DIS_SHADOW_PROC__MSK       0x40
#define BMA2X2_DIS_SHADOW_PROC__REG       BMA2X2_DATA_CTRL_REG

#define BMA2X2_EN_DATA_HIGH_BW__POS         7
#define BMA2X2_EN_DATA_HIGH_BW__LEN         1
#define BMA2X2_EN_DATA_HIGH_BW__MSK         0x80
#define BMA2X2_EN_DATA_HIGH_BW__REG         BMA2X2_DATA_CTRL_REG

#define BMA2X2_EN_SOFT_RESET__POS         0
#define BMA2X2_EN_SOFT_RESET__LEN         8
#define BMA2X2_EN_SOFT_RESET__MSK         0xFF
#define BMA2X2_EN_SOFT_RESET__REG         BMA2X2_RESET_REG

#define BMA2X2_EN_SOFT_RESET_VALUE        0xB6

#define BMA2X2_EN_SLOPE_X_INT__POS         0
#define BMA2X2_EN_SLOPE_X_INT__LEN         1
#define BMA2X2_EN_SLOPE_X_INT__MSK         0x01
#define BMA2X2_EN_SLOPE_X_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SLOPE_Y_INT__POS         1
#define BMA2X2_EN_SLOPE_Y_INT__LEN         1
#define BMA2X2_EN_SLOPE_Y_INT__MSK         0x02
#define BMA2X2_EN_SLOPE_Y_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SLOPE_Z_INT__POS         2
#define BMA2X2_EN_SLOPE_Z_INT__LEN         1
#define BMA2X2_EN_SLOPE_Z_INT__MSK         0x04
#define BMA2X2_EN_SLOPE_Z_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_DOUBLE_TAP_INT__POS      4
#define BMA2X2_EN_DOUBLE_TAP_INT__LEN      1
#define BMA2X2_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA2X2_EN_DOUBLE_TAP_INT__REG      BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SINGLE_TAP_INT__POS      5
#define BMA2X2_EN_SINGLE_TAP_INT__LEN      1
#define BMA2X2_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA2X2_EN_SINGLE_TAP_INT__REG      BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_ORIENT_INT__POS          6
#define BMA2X2_EN_ORIENT_INT__LEN          1
#define BMA2X2_EN_ORIENT_INT__MSK          0x40
#define BMA2X2_EN_ORIENT_INT__REG          BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_FLAT_INT__POS            7
#define BMA2X2_EN_FLAT_INT__LEN            1
#define BMA2X2_EN_FLAT_INT__MSK            0x80
#define BMA2X2_EN_FLAT_INT__REG            BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_HIGHG_X_INT__POS         0
#define BMA2X2_EN_HIGHG_X_INT__LEN         1
#define BMA2X2_EN_HIGHG_X_INT__MSK         0x01
#define BMA2X2_EN_HIGHG_X_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_HIGHG_Y_INT__POS         1
#define BMA2X2_EN_HIGHG_Y_INT__LEN         1
#define BMA2X2_EN_HIGHG_Y_INT__MSK         0x02
#define BMA2X2_EN_HIGHG_Y_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_HIGHG_Z_INT__POS         2
#define BMA2X2_EN_HIGHG_Z_INT__LEN         1
#define BMA2X2_EN_HIGHG_Z_INT__MSK         0x04
#define BMA2X2_EN_HIGHG_Z_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_LOWG_INT__POS            3
#define BMA2X2_EN_LOWG_INT__LEN            1
#define BMA2X2_EN_LOWG_INT__MSK            0x08
#define BMA2X2_EN_LOWG_INT__REG            BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_NEW_DATA_INT__POS        4
#define BMA2X2_EN_NEW_DATA_INT__LEN        1
#define BMA2X2_EN_NEW_DATA_INT__MSK        0x10
#define BMA2X2_EN_NEW_DATA_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_FFULL_EN_INT__POS        5
#define BMA2X2_INT_FFULL_EN_INT__LEN        1
#define BMA2X2_INT_FFULL_EN_INT__MSK        0x20
#define BMA2X2_INT_FFULL_EN_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_FWM_EN_INT__POS        6
#define BMA2X2_INT_FWM_EN_INT__LEN        1
#define BMA2X2_INT_FWM_EN_INT__MSK        0x40
#define BMA2X2_INT_FWM_EN_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__POS        0
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__MSK        0x01
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__POS        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__MSK        0x02
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__POS        2
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__MSK        0x04
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__POS        3
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__MSK        0x08
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_EN_INT1_PAD_LOWG__POS        0
#define BMA2X2_EN_INT1_PAD_LOWG__LEN        1
#define BMA2X2_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA2X2_EN_INT1_PAD_LOWG__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_HIGHG__POS       1
#define BMA2X2_EN_INT1_PAD_HIGHG__LEN       1
#define BMA2X2_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA2X2_EN_INT1_PAD_HIGHG__REG       BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SLOPE__POS       2
#define BMA2X2_EN_INT1_PAD_SLOPE__LEN       1
#define BMA2X2_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA2X2_EN_INT1_PAD_SLOPE__REG       BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__POS        3
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__LEN        1
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_DB_TAP__POS      4
#define BMA2X2_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA2X2_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA2X2_EN_INT1_PAD_DB_TAP__REG      BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA2X2_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA2X2_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA2X2_EN_INT1_PAD_SNG_TAP__REG     BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_ORIENT__POS      6
#define BMA2X2_EN_INT1_PAD_ORIENT__LEN      1
#define BMA2X2_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA2X2_EN_INT1_PAD_ORIENT__REG      BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_FLAT__POS        7
#define BMA2X2_EN_INT1_PAD_FLAT__LEN        1
#define BMA2X2_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA2X2_EN_INT1_PAD_FLAT__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_LOWG__POS        0
#define BMA2X2_EN_INT2_PAD_LOWG__LEN        1
#define BMA2X2_EN_INT2_PAD_LOWG__MSK        0x01
#define BMA2X2_EN_INT2_PAD_LOWG__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_HIGHG__POS       1
#define BMA2X2_EN_INT2_PAD_HIGHG__LEN       1
#define BMA2X2_EN_INT2_PAD_HIGHG__MSK       0x02
#define BMA2X2_EN_INT2_PAD_HIGHG__REG       BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SLOPE__POS       2
#define BMA2X2_EN_INT2_PAD_SLOPE__LEN       1
#define BMA2X2_EN_INT2_PAD_SLOPE__MSK       0x04
#define BMA2X2_EN_INT2_PAD_SLOPE__REG       BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__POS        3
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__LEN        1
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_DB_TAP__POS      4
#define BMA2X2_EN_INT2_PAD_DB_TAP__LEN      1
#define BMA2X2_EN_INT2_PAD_DB_TAP__MSK      0x10
#define BMA2X2_EN_INT2_PAD_DB_TAP__REG      BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SNG_TAP__POS     5
#define BMA2X2_EN_INT2_PAD_SNG_TAP__LEN     1
#define BMA2X2_EN_INT2_PAD_SNG_TAP__MSK     0x20
#define BMA2X2_EN_INT2_PAD_SNG_TAP__REG     BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_ORIENT__POS      6
#define BMA2X2_EN_INT2_PAD_ORIENT__LEN      1
#define BMA2X2_EN_INT2_PAD_ORIENT__MSK      0x40
#define BMA2X2_EN_INT2_PAD_ORIENT__REG      BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_FLAT__POS        7
#define BMA2X2_EN_INT2_PAD_FLAT__LEN        1
#define BMA2X2_EN_INT2_PAD_FLAT__MSK        0x80
#define BMA2X2_EN_INT2_PAD_FLAT__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_NEWDATA__POS     0
#define BMA2X2_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA2X2_EN_INT1_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT1_PAD_FWM__POS     1
#define BMA2X2_EN_INT1_PAD_FWM__LEN     1
#define BMA2X2_EN_INT1_PAD_FWM__MSK     0x02
#define BMA2X2_EN_INT1_PAD_FWM__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT1_PAD_FFULL__POS     2
#define BMA2X2_EN_INT1_PAD_FFULL__LEN     1
#define BMA2X2_EN_INT1_PAD_FFULL__MSK     0x04
#define BMA2X2_EN_INT1_PAD_FFULL__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_FFULL__POS     5
#define BMA2X2_EN_INT2_PAD_FFULL__LEN     1
#define BMA2X2_EN_INT2_PAD_FFULL__MSK     0x20
#define BMA2X2_EN_INT2_PAD_FFULL__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_FWM__POS     6
#define BMA2X2_EN_INT2_PAD_FWM__LEN     1
#define BMA2X2_EN_INT2_PAD_FWM__MSK     0x40
#define BMA2X2_EN_INT2_PAD_FWM__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_NEWDATA__POS     7
#define BMA2X2_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA2X2_EN_INT2_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_UNFILT_INT_SRC_LOWG__POS        0
#define BMA2X2_UNFILT_INT_SRC_LOWG__LEN        1
#define BMA2X2_UNFILT_INT_SRC_LOWG__MSK        0x01
#define BMA2X2_UNFILT_INT_SRC_LOWG__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_HIGHG__POS       1
#define BMA2X2_UNFILT_INT_SRC_HIGHG__LEN       1
#define BMA2X2_UNFILT_INT_SRC_HIGHG__MSK       0x02
#define BMA2X2_UNFILT_INT_SRC_HIGHG__REG       BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_SLOPE__POS       2
#define BMA2X2_UNFILT_INT_SRC_SLOPE__LEN       1
#define BMA2X2_UNFILT_INT_SRC_SLOPE__MSK       0x04
#define BMA2X2_UNFILT_INT_SRC_SLOPE__REG       BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__POS        3
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__LEN        1
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__MSK        0x08
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_TAP__POS         4
#define BMA2X2_UNFILT_INT_SRC_TAP__LEN         1
#define BMA2X2_UNFILT_INT_SRC_TAP__MSK         0x10
#define BMA2X2_UNFILT_INT_SRC_TAP__REG         BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_DATA__POS        5
#define BMA2X2_UNFILT_INT_SRC_DATA__LEN        1
#define BMA2X2_UNFILT_INT_SRC_DATA__MSK        0x20
#define BMA2X2_UNFILT_INT_SRC_DATA__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__POS       0
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__REG       BMA2X2_INT_SET_REG

#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__POS       2
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__REG       BMA2X2_INT_SET_REG

#define BMA2X2_INT1_PAD_OUTPUT_TYPE__POS        1
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__LEN        1
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__REG        BMA2X2_INT_SET_REG

#define BMA2X2_INT2_PAD_OUTPUT_TYPE__POS        3
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__LEN        1
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__REG        BMA2X2_INT_SET_REG

#define BMA2X2_LATCH_INT__POS                0
#define BMA2X2_LATCH_INT__LEN                4
#define BMA2X2_LATCH_INT__MSK                0x0F
#define BMA2X2_LATCH_INT__REG                BMA2X2_INT_CTRL_REG

#define BMA2X2_RESET_INT__POS           7
#define BMA2X2_RESET_INT__LEN           1
#define BMA2X2_RESET_INT__MSK           0x80
#define BMA2X2_RESET_INT__REG           BMA2X2_INT_CTRL_REG

#define BMA2X2_LOWG_DUR__POS                    0
#define BMA2X2_LOWG_DUR__LEN                    8
#define BMA2X2_LOWG_DUR__MSK                    0xFF
#define BMA2X2_LOWG_DUR__REG                    BMA2X2_LOW_DURN_REG

#define BMA2X2_LOWG_THRES__POS                  0
#define BMA2X2_LOWG_THRES__LEN                  8
#define BMA2X2_LOWG_THRES__MSK                  0xFF
#define BMA2X2_LOWG_THRES__REG                  BMA2X2_LOW_THRES_REG

#define BMA2X2_LOWG_HYST__POS                   0
#define BMA2X2_LOWG_HYST__LEN                   2
#define BMA2X2_LOWG_HYST__MSK                   0x03
#define BMA2X2_LOWG_HYST__REG                   BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_LOWG_INT_MODE__POS               2
#define BMA2X2_LOWG_INT_MODE__LEN               1
#define BMA2X2_LOWG_INT_MODE__MSK               0x04
#define BMA2X2_LOWG_INT_MODE__REG               BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_HIGHG_DUR__POS                    0
#define BMA2X2_HIGHG_DUR__LEN                    8
#define BMA2X2_HIGHG_DUR__MSK                    0xFF
#define BMA2X2_HIGHG_DUR__REG                    BMA2X2_HIGH_DURN_REG

#define BMA2X2_HIGHG_THRES__POS                  0
#define BMA2X2_HIGHG_THRES__LEN                  8
#define BMA2X2_HIGHG_THRES__MSK                  0xFF
#define BMA2X2_HIGHG_THRES__REG                  BMA2X2_HIGH_THRES_REG

#define BMA2X2_HIGHG_HYST__POS                  6
#define BMA2X2_HIGHG_HYST__LEN                  2
#define BMA2X2_HIGHG_HYST__MSK                  0xC0
#define BMA2X2_HIGHG_HYST__REG                  BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_SLOPE_DUR__POS                    0
#define BMA2X2_SLOPE_DUR__LEN                    2
#define BMA2X2_SLOPE_DUR__MSK                    0x03
#define BMA2X2_SLOPE_DUR__REG                    BMA2X2_SLOPE_DURN_REG

#define BMA2X2_SLO_NO_MOT_DUR__POS                    2
#define BMA2X2_SLO_NO_MOT_DUR__LEN                    6
#define BMA2X2_SLO_NO_MOT_DUR__MSK                    0xFC
#define BMA2X2_SLO_NO_MOT_DUR__REG                    BMA2X2_SLOPE_DURN_REG

#define BMA2X2_SLOPE_THRES__POS                  0
#define BMA2X2_SLOPE_THRES__LEN                  8
#define BMA2X2_SLOPE_THRES__MSK                  0xFF
#define BMA2X2_SLOPE_THRES__REG                  BMA2X2_SLOPE_THRES_REG

#define BMA2X2_SLO_NO_MOT_THRES__POS                  0
#define BMA2X2_SLO_NO_MOT_THRES__LEN                  8
#define BMA2X2_SLO_NO_MOT_THRES__MSK                  0xFF
#define BMA2X2_SLO_NO_MOT_THRES__REG  		BMA2X2_SLO_NO_MOT_THRES_REG

#define BMA2X2_TAP_DUR__POS                    0
#define BMA2X2_TAP_DUR__LEN                    3
#define BMA2X2_TAP_DUR__MSK                    0x07
#define BMA2X2_TAP_DUR__REG                    BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_SHOCK_DURN__POS             6
#define BMA2X2_TAP_SHOCK_DURN__LEN             1
#define BMA2X2_TAP_SHOCK_DURN__MSK             0x40
#define BMA2X2_TAP_SHOCK_DURN__REG             BMA2X2_TAP_PARAM_REG

#define BMA2X2_ADV_TAP_INT__POS                5
#define BMA2X2_ADV_TAP_INT__LEN                1
#define BMA2X2_ADV_TAP_INT__MSK                0x20
#define BMA2X2_ADV_TAP_INT__REG                BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_QUIET_DURN__POS             7
#define BMA2X2_TAP_QUIET_DURN__LEN             1
#define BMA2X2_TAP_QUIET_DURN__MSK             0x80
#define BMA2X2_TAP_QUIET_DURN__REG             BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_THRES__POS                  0
#define BMA2X2_TAP_THRES__LEN                  5
#define BMA2X2_TAP_THRES__MSK                  0x1F
#define BMA2X2_TAP_THRES__REG                  BMA2X2_TAP_THRES_REG

#define BMA2X2_TAP_SAMPLES__POS                6
#define BMA2X2_TAP_SAMPLES__LEN                2
#define BMA2X2_TAP_SAMPLES__MSK                0xC0
#define BMA2X2_TAP_SAMPLES__REG                BMA2X2_TAP_THRES_REG

#define BMA2X2_ORIENT_MODE__POS                  0
#define BMA2X2_ORIENT_MODE__LEN                  2
#define BMA2X2_ORIENT_MODE__MSK                  0x03
#define BMA2X2_ORIENT_MODE__REG                  BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_BLOCK__POS                 2
#define BMA2X2_ORIENT_BLOCK__LEN                 2
#define BMA2X2_ORIENT_BLOCK__MSK                 0x0C
#define BMA2X2_ORIENT_BLOCK__REG                 BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_HYST__POS                  4
#define BMA2X2_ORIENT_HYST__LEN                  3
#define BMA2X2_ORIENT_HYST__MSK                  0x70
#define BMA2X2_ORIENT_HYST__REG                  BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_AXIS__POS                  7
#define BMA2X2_ORIENT_AXIS__LEN                  1
#define BMA2X2_ORIENT_AXIS__MSK                  0x80
#define BMA2X2_ORIENT_AXIS__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_ORIENT_UD_EN__POS                  6
#define BMA2X2_ORIENT_UD_EN__LEN                  1
#define BMA2X2_ORIENT_UD_EN__MSK                  0x40
#define BMA2X2_ORIENT_UD_EN__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_THETA_BLOCK__POS                  0
#define BMA2X2_THETA_BLOCK__LEN                  6
#define BMA2X2_THETA_BLOCK__MSK                  0x3F
#define BMA2X2_THETA_BLOCK__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_THETA_FLAT__POS                  0
#define BMA2X2_THETA_FLAT__LEN                  6
#define BMA2X2_THETA_FLAT__MSK                  0x3F
#define BMA2X2_THETA_FLAT__REG                  BMA2X2_THETA_FLAT_REG

#define BMA2X2_FLAT_HOLD_TIME__POS              4
#define BMA2X2_FLAT_HOLD_TIME__LEN              2
#define BMA2X2_FLAT_HOLD_TIME__MSK              0x30
#define BMA2X2_FLAT_HOLD_TIME__REG              BMA2X2_FLAT_HOLD_TIME_REG

#define BMA2X2_FLAT_HYS__POS                   0
#define BMA2X2_FLAT_HYS__LEN                   3
#define BMA2X2_FLAT_HYS__MSK                   0x07
#define BMA2X2_FLAT_HYS__REG                   BMA2X2_FLAT_HOLD_TIME_REG

#define BMA2X2_FIFO_WML_TRIG_RETAIN__POS                   0
#define BMA2X2_FIFO_WML_TRIG_RETAIN__LEN                   6
#define BMA2X2_FIFO_WML_TRIG_RETAIN__MSK                   0x3F
#define BMA2X2_FIFO_WML_TRIG_RETAIN__REG                   BMA2X2_FIFO_WML_TRIG

#define BMA2X2_EN_SELF_TEST__POS                0
#define BMA2X2_EN_SELF_TEST__LEN                2
#define BMA2X2_EN_SELF_TEST__MSK                0x03
#define BMA2X2_EN_SELF_TEST__REG                BMA2X2_SELF_TEST_REG

#define BMA2X2_NEG_SELF_TEST__POS               2
#define BMA2X2_NEG_SELF_TEST__LEN               1
#define BMA2X2_NEG_SELF_TEST__MSK               0x04
#define BMA2X2_NEG_SELF_TEST__REG               BMA2X2_SELF_TEST_REG

#define BMA2X2_SELF_TEST_AMP__POS               4
#define BMA2X2_SELF_TEST_AMP__LEN               3
#define BMA2X2_SELF_TEST_AMP__MSK               0x70
#define BMA2X2_SELF_TEST_AMP__REG               BMA2X2_SELF_TEST_REG

#define BMA2X2_UNLOCK_EE_PROG_MODE__POS     0
#define BMA2X2_UNLOCK_EE_PROG_MODE__LEN     1
#define BMA2X2_UNLOCK_EE_PROG_MODE__MSK     0x01
#define BMA2X2_UNLOCK_EE_PROG_MODE__REG     BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_START_EE_PROG_TRIG__POS      1
#define BMA2X2_START_EE_PROG_TRIG__LEN      1
#define BMA2X2_START_EE_PROG_TRIG__MSK      0x02
#define BMA2X2_START_EE_PROG_TRIG__REG      BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EE_PROG_READY__POS          2
#define BMA2X2_EE_PROG_READY__LEN          1
#define BMA2X2_EE_PROG_READY__MSK          0x04
#define BMA2X2_EE_PROG_READY__REG          BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_UPDATE_IMAGE__POS                3
#define BMA2X2_UPDATE_IMAGE__LEN                1
#define BMA2X2_UPDATE_IMAGE__MSK                0x08
#define BMA2X2_UPDATE_IMAGE__REG                BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EE_REMAIN__POS                4
#define BMA2X2_EE_REMAIN__LEN                4
#define BMA2X2_EE_REMAIN__MSK                0xF0
#define BMA2X2_EE_REMAIN__REG                BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EN_SPI_MODE_3__POS              0
#define BMA2X2_EN_SPI_MODE_3__LEN              1
#define BMA2X2_EN_SPI_MODE_3__MSK              0x01
#define BMA2X2_EN_SPI_MODE_3__REG              BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_I2C_WATCHDOG_PERIOD__POS        1
#define BMA2X2_I2C_WATCHDOG_PERIOD__LEN        1
#define BMA2X2_I2C_WATCHDOG_PERIOD__MSK        0x02
#define BMA2X2_I2C_WATCHDOG_PERIOD__REG        BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_EN_I2C_WATCHDOG__POS            2
#define BMA2X2_EN_I2C_WATCHDOG__LEN            1
#define BMA2X2_EN_I2C_WATCHDOG__MSK            0x04
#define BMA2X2_EN_I2C_WATCHDOG__REG            BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_EXT_MODE__POS              7
#define BMA2X2_EXT_MODE__LEN              1
#define BMA2X2_EXT_MODE__MSK              0x80
#define BMA2X2_EXT_MODE__REG              BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_ALLOW_UPPER__POS        6
#define BMA2X2_ALLOW_UPPER__LEN        1
#define BMA2X2_ALLOW_UPPER__MSK        0x40
#define BMA2X2_ALLOW_UPPER__REG        BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_MAP_2_LOWER__POS            5
#define BMA2X2_MAP_2_LOWER__LEN            1
#define BMA2X2_MAP_2_LOWER__MSK            0x20
#define BMA2X2_MAP_2_LOWER__REG            BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_MAGIC_NUMBER__POS            0
#define BMA2X2_MAGIC_NUMBER__LEN            5
#define BMA2X2_MAGIC_NUMBER__MSK            0x1F
#define BMA2X2_MAGIC_NUMBER__REG            BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_UNLOCK_EE_WRITE_TRIM__POS        4
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__LEN        4
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__MSK        0xF0
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__REG        BMA2X2_CTRL_UNLOCK_REG

#define BMA2X2_EN_SLOW_COMP_X__POS              0
#define BMA2X2_EN_SLOW_COMP_X__LEN              1
#define BMA2X2_EN_SLOW_COMP_X__MSK              0x01
#define BMA2X2_EN_SLOW_COMP_X__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_EN_SLOW_COMP_Y__POS              1
#define BMA2X2_EN_SLOW_COMP_Y__LEN              1
#define BMA2X2_EN_SLOW_COMP_Y__MSK              0x02
#define BMA2X2_EN_SLOW_COMP_Y__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_EN_SLOW_COMP_Z__POS              2
#define BMA2X2_EN_SLOW_COMP_Z__LEN              1
#define BMA2X2_EN_SLOW_COMP_Z__MSK              0x04
#define BMA2X2_EN_SLOW_COMP_Z__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_FAST_CAL_RDY_S__POS             4
#define BMA2X2_FAST_CAL_RDY_S__LEN             1
#define BMA2X2_FAST_CAL_RDY_S__MSK             0x10
#define BMA2X2_FAST_CAL_RDY_S__REG             BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_CAL_TRIGGER__POS                5
#define BMA2X2_CAL_TRIGGER__LEN                2
#define BMA2X2_CAL_TRIGGER__MSK                0x60
#define BMA2X2_CAL_TRIGGER__REG                BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_RESET_OFFSET_REGS__POS           7
#define BMA2X2_RESET_OFFSET_REGS__LEN           1
#define BMA2X2_RESET_OFFSET_REGS__MSK           0x80
#define BMA2X2_RESET_OFFSET_REGS__REG           BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_COMP_CUTOFF__POS                 0
#define BMA2X2_COMP_CUTOFF__LEN                 1
#define BMA2X2_COMP_CUTOFF__MSK                 0x01
#define BMA2X2_COMP_CUTOFF__REG                 BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_X__POS        1
#define BMA2X2_COMP_TARGET_OFFSET_X__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA2X2_COMP_TARGET_OFFSET_X__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Y__POS        3
#define BMA2X2_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA2X2_COMP_TARGET_OFFSET_Y__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Z__POS        5
#define BMA2X2_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA2X2_COMP_TARGET_OFFSET_Z__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_FIFO_DATA_SELECT__POS                 0
#define BMA2X2_FIFO_DATA_SELECT__LEN                 2
#define BMA2X2_FIFO_DATA_SELECT__MSK                 0x03
#define BMA2X2_FIFO_DATA_SELECT__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_TRIGGER_SOURCE__POS                 2
#define BMA2X2_FIFO_TRIGGER_SOURCE__LEN                 2
#define BMA2X2_FIFO_TRIGGER_SOURCE__MSK                 0x0C
#define BMA2X2_FIFO_TRIGGER_SOURCE__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_TRIGGER_ACTION__POS                 4
#define BMA2X2_FIFO_TRIGGER_ACTION__LEN                 2
#define BMA2X2_FIFO_TRIGGER_ACTION__MSK                 0x30
#define BMA2X2_FIFO_TRIGGER_ACTION__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_MODE__POS                 6
#define BMA2X2_FIFO_MODE__LEN                 2
#define BMA2X2_FIFO_MODE__MSK                 0xC0
#define BMA2X2_FIFO_MODE__REG                 BMA2X2_FIFO_MODE_REG


#define BMA2X2_STATUS1                             0
#define BMA2X2_STATUS2                             1
#define BMA2X2_STATUS3                             2
#define BMA2X2_STATUS4                             3
#define BMA2X2_STATUS5                             4


#define BMA2X2_RANGE_2G                 3
#define BMA2X2_RANGE_4G                 5
#define BMA2X2_RANGE_8G                 8
#define BMA2X2_RANGE_16G                12


#define BMA2X2_BW_7_81HZ        0x08
#define BMA2X2_BW_15_63HZ       0x09
#define BMA2X2_BW_31_25HZ       0x0A
#define BMA2X2_BW_62_50HZ       0x0B
#define BMA2X2_BW_125HZ         0x0C
#define BMA2X2_BW_250HZ         0x0D
#define BMA2X2_BW_500HZ         0x0E
#define BMA2X2_BW_1000HZ        0x0F

#define BMA2X2_SLEEP_DUR_0_5MS        0x05
#define BMA2X2_SLEEP_DUR_1MS          0x06
#define BMA2X2_SLEEP_DUR_2MS          0x07
#define BMA2X2_SLEEP_DUR_4MS          0x08
#define BMA2X2_SLEEP_DUR_6MS          0x09
#define BMA2X2_SLEEP_DUR_10MS         0x0A
#define BMA2X2_SLEEP_DUR_25MS         0x0B
#define BMA2X2_SLEEP_DUR_50MS         0x0C
#define BMA2X2_SLEEP_DUR_100MS        0x0D
#define BMA2X2_SLEEP_DUR_500MS        0x0E
#define BMA2X2_SLEEP_DUR_1S           0x0F

#define BMA2X2_LATCH_DUR_NON_LATCH    0x00
#define BMA2X2_LATCH_DUR_250MS        0x01
#define BMA2X2_LATCH_DUR_500MS        0x02
#define BMA2X2_LATCH_DUR_1S           0x03
#define BMA2X2_LATCH_DUR_2S           0x04
#define BMA2X2_LATCH_DUR_4S           0x05
#define BMA2X2_LATCH_DUR_8S           0x06
#define BMA2X2_LATCH_DUR_LATCH        0x07
#define BMA2X2_LATCH_DUR_NON_LATCH1   0x08
#define BMA2X2_LATCH_DUR_250US        0x09
#define BMA2X2_LATCH_DUR_500US        0x0A
#define BMA2X2_LATCH_DUR_1MS          0x0B
#define BMA2X2_LATCH_DUR_12_5MS       0x0C
#define BMA2X2_LATCH_DUR_25MS         0x0D
#define BMA2X2_LATCH_DUR_50MS         0x0E
#define BMA2X2_LATCH_DUR_LATCH1       0x0F

#define BMA2X2_MODE_NORMAL             0
#define BMA2X2_MODE_LOWPOWER1          1
#define BMA2X2_MODE_SUSPEND            2
#define BMA2X2_MODE_DEEP_SUSPEND       3
#define BMA2X2_MODE_LOWPOWER2          4
#define BMA2X2_MODE_STANDBY            5

#define BMA2X2_X_AXIS           0
#define BMA2X2_Y_AXIS           1
#define BMA2X2_Z_AXIS           2

#define BMA2X2_Low_G_Interrupt       0
#define BMA2X2_High_G_X_Interrupt    1
#define BMA2X2_High_G_Y_Interrupt    2
#define BMA2X2_High_G_Z_Interrupt    3
#define BMA2X2_DATA_EN               4
#define BMA2X2_Slope_X_Interrupt     5
#define BMA2X2_Slope_Y_Interrupt     6
#define BMA2X2_Slope_Z_Interrupt     7
#define BMA2X2_Single_Tap_Interrupt  8
#define BMA2X2_Double_Tap_Interrupt  9
#define BMA2X2_Orient_Interrupt      10
#define BMA2X2_Flat_Interrupt        11
#define BMA2X2_FFULL_INTERRUPT       12
#define BMA2X2_FWM_INTERRUPT         13

#define BMA2X2_INT1_LOWG         0
#define BMA2X2_INT2_LOWG         1
#define BMA2X2_INT1_HIGHG        0
#define BMA2X2_INT2_HIGHG        1
#define BMA2X2_INT1_SLOPE        0
#define BMA2X2_INT2_SLOPE        1
#define BMA2X2_INT1_SLO_NO_MOT   0
#define BMA2X2_INT2_SLO_NO_MOT   1
#define BMA2X2_INT1_DTAP         0
#define BMA2X2_INT2_DTAP         1
#define BMA2X2_INT1_STAP         0
#define BMA2X2_INT2_STAP         1
#define BMA2X2_INT1_ORIENT       0
#define BMA2X2_INT2_ORIENT       1
#define BMA2X2_INT1_FLAT         0
#define BMA2X2_INT2_FLAT         1
#define BMA2X2_INT1_NDATA        0
#define BMA2X2_INT2_NDATA        1
#define BMA2X2_INT1_FWM          0
#define BMA2X2_INT2_FWM          1
#define BMA2X2_INT1_FFULL        0
#define BMA2X2_INT2_FFULL        1

#define BMA2X2_SRC_LOWG         0
#define BMA2X2_SRC_HIGHG        1
#define BMA2X2_SRC_SLOPE        2
#define BMA2X2_SRC_SLO_NO_MOT   3
#define BMA2X2_SRC_TAP          4
#define BMA2X2_SRC_DATA         5

#define BMA2X2_INT1_OUTPUT      0
#define BMA2X2_INT2_OUTPUT      1
#define BMA2X2_INT1_LEVEL       0
#define BMA2X2_INT2_LEVEL       1

#define BMA2X2_LOW_DURATION            0
#define BMA2X2_HIGH_DURATION           1
#define BMA2X2_SLOPE_DURATION          2
#define BMA2X2_SLO_NO_MOT_DURATION     3

#define BMA2X2_LOW_THRESHOLD            0
#define BMA2X2_HIGH_THRESHOLD           1
#define BMA2X2_SLOPE_THRESHOLD          2
#define BMA2X2_SLO_NO_MOT_THRESHOLD     3


#define BMA2X2_LOWG_HYST                0
#define BMA2X2_HIGHG_HYST               1

#define BMA2X2_ORIENT_THETA             0
#define BMA2X2_FLAT_THETA               1

#define BMA2X2_I2C_SELECT               0
#define BMA2X2_I2C_EN                   1

#define BMA2X2_SLOW_COMP_X              0
#define BMA2X2_SLOW_COMP_Y              1
#define BMA2X2_SLOW_COMP_Z              2

#define BMA2X2_CUT_OFF                  0
#define BMA2X2_OFFSET_TRIGGER_X         1
#define BMA2X2_OFFSET_TRIGGER_Y         2
#define BMA2X2_OFFSET_TRIGGER_Z         3

#define BMA2X2_GP0                      0
#define BMA2X2_GP1                      1

#define BMA2X2_SLO_NO_MOT_EN_X          0
#define BMA2X2_SLO_NO_MOT_EN_Y          1
#define BMA2X2_SLO_NO_MOT_EN_Z          2
#define BMA2X2_SLO_NO_MOT_EN_SEL        3

#define BMA2X2_WAKE_UP_DUR_20MS         0
#define BMA2X2_WAKE_UP_DUR_80MS         1
#define BMA2X2_WAKE_UP_DUR_320MS                2
#define BMA2X2_WAKE_UP_DUR_2560MS               3

#define BMA2X2_SELF_TEST0_ON            1
#define BMA2X2_SELF_TEST1_ON            2

#define BMA2X2_EE_W_OFF                 0
#define BMA2X2_EE_W_ON                  1

#define BMA2X2_LOW_TH_IN_G(gthres, range)           ((256 * gthres) / range)


#define BMA2X2_HIGH_TH_IN_G(gthres, range)          ((256 * gthres) / range)


#define BMA2X2_LOW_HY_IN_G(ghyst, range)            ((32 * ghyst) / range)


#define BMA2X2_HIGH_HY_IN_G(ghyst, range)           ((32 * ghyst) / range)


#define BMA2X2_SLOPE_TH_IN_G(gthres, range)    ((128 * gthres) / range)


#define BMA2X2_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA2X2_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


#define BMA255_CHIP_ID 0XFA
#define BMA250E_CHIP_ID 0XF9
#define BMA2x2E_CHIP_ID 0XF8
#define BMA280_CHIP_ID 0XFB
#define BMA250_CHIP_ID 0X03

#define BMA255_TYPE 0
#define BMA250E_TYPE 1
#define BMA2x2E_TYPE 2
#define BMA280_TYPE 3
#define BMA250_TYPE 4

unsigned char *sensor_name[] = { "BMA255", "BMA250E", "BMA2x2E", "BMA280", "BMA250" };

/* Driver data for BMA2X2
 * 
 */
struct value_convert_pair {
	unsigned char value;
	unsigned char converted;
};

unsigned char *mode_table[] = {
		"NORMAL",
		"LOWPOWER1",
		"SUSPEND",
		"DEEP_SUSPEND",
		"LOWPOWER2",
		"STANDBY"
};

extern struct kset *devices_kset;
static const struct value_convert_pair bw_table[] =
{
		{1,   BMA2X2_BW_1000HZ},
		{2,   BMA2X2_BW_500HZ},
		{4,   BMA2X2_BW_250HZ},
		{8,   BMA2X2_BW_125HZ},
		{16,  BMA2X2_BW_62_50HZ},
		{32,  BMA2X2_BW_31_25HZ},
		{64,  BMA2X2_BW_15_63HZ},
		{128, BMA2X2_BW_7_81HZ},
};

static const struct value_convert_pair range_table[] =
{
		{2,   BMA2X2_RANGE_2G},
		{4,   BMA2X2_RANGE_4G},
		{8,   BMA2X2_RANGE_8G},
		{16,  BMA2X2_RANGE_16G},
};

static const struct value_convert_pair sensitivity_table[] =
{
		{BMA255_TYPE,	4},
		{BMA250E_TYPE,	16},
		{BMA2x2E_TYPE,  4},
		{BMA280_TYPE,	1},
		{BMA250_TYPE,	4},
};

struct bma2x2acc{
	s16	x,
		y,
		z;
} ;

struct driver_data {
	struct i2c_client *client;
	struct sensors_pm_platform_data *pdata;
	int hw_initialized;
	int on_before_suspend;
	
	//atomic_t delay;
	atomic_t enabled;
	unsigned int chip_id;
	unsigned char mode;
	signed char sensor_type;
	
	struct input_dev *input_dev;
	struct workqueue_struct *input_wq;
	struct delayed_work input_work;
	//struct input_polled_dev *input_polled;
	
	struct mutex data_lock;
	struct mutex enable_lock;
};


static int bma2x2_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma2x2_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma2x2_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma2x2_read_accel_xyz(struct i2c_client *client,
		signed char sensor_type, struct bma2x2acc *acc)
{
	int comres;
	unsigned char data[6];

	switch (sensor_type) {
	case BMA255_TYPE:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X12_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X12_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X12_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y12_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y12_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z12_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z12_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case BMA250E_TYPE:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X10_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X10_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y10_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y10_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z10_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z10_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case BMA2x2E_TYPE:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X8_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X8_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X8_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y8_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y8_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z8_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z8_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case BMA280_TYPE:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X14_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X14_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X14_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y14_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y14_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z14_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z14_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case BMA250_TYPE:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X10_LSB__REG, data, 6);
		acc->x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X10_LSB__LEN));
		acc->x = acc->x << (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		acc->y = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y10_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y10_LSB__LEN
									));
		acc->y = acc->y << (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		acc->z = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z10_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z10_LSB__LEN));
		acc->z = acc->z << (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	default:
		break;
	}

	return comres;
}

static int bma2x2_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0 ;
	unsigned char data1, data2;
	dprintk("set mode %d", Mode);
	if (Mode < 6) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG,
				&data1);
		comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_NOISE_CTRL_REG,
				&data2);
		switch (Mode) {
		case BMA2X2_MODE_NORMAL:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_MODE_CTRL, 0);
			data2  = BMA2X2_SET_BITSLICE(data2,
					BMA2X2_LOW_POWER_MODE, 0);
			break;
		case BMA2X2_MODE_LOWPOWER1:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_MODE_CTRL, 2);
			data2  = BMA2X2_SET_BITSLICE(data2,
					BMA2X2_LOW_POWER_MODE, 0);
			break;
		case BMA2X2_MODE_SUSPEND:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_MODE_CTRL, 4);
			data2  = BMA2X2_SET_BITSLICE(data2,
					BMA2X2_LOW_POWER_MODE, 0);
			break;
		case BMA2X2_MODE_DEEP_SUSPEND:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_MODE_CTRL, 1);
			data2  = BMA2X2_SET_BITSLICE(data2,
					BMA2X2_LOW_POWER_MODE, 1);
			break;
		case BMA2X2_MODE_LOWPOWER2:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_MODE_CTRL, 2);
			data2  = BMA2X2_SET_BITSLICE(data2,
					BMA2X2_LOW_POWER_MODE, 1);
			break;
		case BMA2X2_MODE_STANDBY:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_MODE_CTRL, 4);
			data2  = BMA2X2_SET_BITSLICE(data2,
					BMA2X2_LOW_POWER_MODE, 1);
			break;
		default:
			break;
		}
		mdelay(2);
		comres += bma2x2_smbus_write_byte(client, BMA2X2_MODE_CTRL_REG,
				&data1);
		comres += bma2x2_smbus_write_byte(client,
				BMA2X2_LOW_NOISE_CTRL_REG,
				&data2);
	} else {
		comres = -1 ;
	}
	return comres;
}

static int bma2x2_get_mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;
	unsigned char data1, data2;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG, 
			&data1);
	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_NOISE_CTRL_REG,
			&data2);

	data1  = (data1 & 0xE0) >> 5;
	data2  = (data2 & 0x40) >> 6;


	if ((data1 == 0x00) && (data2 == 0x00)) {
		*Mode  = BMA2X2_MODE_NORMAL;
	} else {
		if ((data1 == 0x02) && (data2 == 0x00)) {
			*Mode  = BMA2X2_MODE_LOWPOWER1;
		} else {
			if ((data1 == 0x04 || data1 == 0x06) &&
						(data2 == 0x00)) {
				*Mode  = BMA2X2_MODE_SUSPEND;
			} else {
				if (((data1 & 0x01) == 0x01)) {
					*Mode  = BMA2X2_MODE_DEEP_SUSPEND;
				} else {
					if ((data1 == 0x02) &&
							(data2 == 0x01)) {
						*Mode  = BMA2X2_MODE_LOWPOWER2;
					} else {
						if ((data1 == 0x04) && (data2 ==
									0x01)) {
							*Mode  =
							BMA2X2_MODE_STANDBY;
						} else {
							*Mode =
						BMA2X2_MODE_DEEP_SUSPEND;
						}
					}
				}
			}
		}
	}

	return comres;
}

static int bma2x2_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0 ;
	unsigned char data1;

	dprintk("set range %d", Range);

	if ((Range == 3) || (Range == 5) || (Range == 8) || (Range == 12)) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL_REG,
				&data1);
		switch (Range) {
		case BMA2X2_RANGE_2G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 3);
			break;
		case BMA2X2_RANGE_4G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 5);
			break;
		case BMA2X2_RANGE_8G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 8);
			break;
		case BMA2X2_RANGE_16G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 12);
			break;
		default:
			break;
		}
		comres += bma2x2_smbus_write_byte(client, BMA2X2_RANGE_SEL_REG,
				&data1);
	} else {
		comres = -1 ;
	}
	return comres;
}

static int bma2x2_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL__REG, 
			&data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_RANGE_SEL);
	*Range = data;

	return comres;
}


static int bma2x2_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data;
	int Bandwidth = 0;

	dprintk("set bandwidth %d", BW);

	if (BW > 7 && BW < 16) {
		switch (BW) {
		case BMA2X2_BW_7_81HZ:
			Bandwidth = BMA2X2_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
		case BMA2X2_BW_15_63HZ:
			Bandwidth = BMA2X2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
		case BMA2X2_BW_31_25HZ:
			Bandwidth = BMA2X2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
		case BMA2X2_BW_62_50HZ:
			Bandwidth = BMA2X2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
		case BMA2X2_BW_125HZ:
			Bandwidth = BMA2X2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
		case BMA2X2_BW_250HZ:
			Bandwidth = BMA2X2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
		case BMA2X2_BW_500HZ:
			Bandwidth = BMA2X2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
		case BMA2X2_BW_1000HZ:
			Bandwidth = BMA2X2_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
		default:
			break;
		}
		comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_BANDWIDTH, Bandwidth);
		comres += bma2x2_smbus_write_byte(client, BMA2X2_BANDWIDTH__REG,
				&data);
	} else {
		comres = -1 ;
	}
	return comres;
}

static int bma2x2_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_BANDWIDTH);
	*BW = data ;

	return comres;
}

static unsigned char bma2x2_convert_bw_value(unsigned char value)
{
	int i;
	
	for (i = ARRAY_SIZE(bw_table) - 1; i >= 0; i--) {
		if (bw_table[i].value <= value)
		break;
 	}
 	if (i < 0) i = 0;
	return bw_table[i].converted;
}

static unsigned char bma2x2_unconvert_bw_value(unsigned char converted)
{
	int i;
	
	for (i = ARRAY_SIZE(bw_table) - 1; i >= 0; i--) {
		if (bw_table[i].converted == converted)
		break;
	}
 	if (i < 0) i = 0;
	return bw_table[i].value;
}

static unsigned char bma2x2_convert_range_value(unsigned char value)
{
	int i;
	
	for (i = ARRAY_SIZE(range_table) - 1; i >= 0; i--) {
		if (range_table[i].value <= value)
		break;
	}
 	if (i < 0) i = 0;
	return range_table[i].converted;
}

static unsigned char bma2x2_unconvert_range_value(unsigned char converted)
{
	int i;
	
	for (i = ARRAY_SIZE(range_table) - 1; i >= 0; i--) {
		if (range_table[i].converted == converted)
		break;
	}
 	if (i < 0) i = 0;
	return range_table[i].value;
}

/* Call platform power suspend to disable device
 *
 */
static int bma2x2_disable(struct driver_data *ddata)
{
	
	struct sensors_pm_platform_data *pdata = ddata->pdata;

	mutex_lock(&ddata->enable_lock);
	if (atomic_cmpxchg(&ddata->enabled, 1, 0)) {
		
		cancel_delayed_work_sync(&ddata->input_work);
		bma2x2_set_mode(ddata->client, BMA2X2_MODE_DEEP_SUSPEND);
		dprintk("enter deep suspend mode");

		if (pdata->suspend_platform_hw(pdata) < 0) {
			goto disable_failed;
		}

		if(ddata->hw_initialized) {
			ddata->hw_initialized = 0;
		}
	}
	mutex_unlock(&ddata->enable_lock);
	dprintk("DISABLED");

	return 0;

disable_failed:
	atomic_set(&ddata->enabled, 1);
	dprintk("DISABLE FAILED");
	return ERR;
}


/* Call platform power resume to enable device
 *
 */
static int bma2x2_enable(struct driver_data *ddata)
{
	struct sensors_pm_platform_data *pdata = ddata->pdata;
	
	mutex_lock(&ddata->enable_lock);
	if (!atomic_cmpxchg(&ddata->enabled, 0, 1)) {
		if(pdata->resume_platform_hw(pdata) < 0)
			goto enable_failed;

		bma2x2_set_mode(ddata->client, BMA2X2_MODE_NORMAL);
		dprintk("enter normal mode");
		
		if (!ddata->hw_initialized) {
			ddata->hw_initialized = 1;
		}
		queue_delayed_work(ddata->input_wq, &ddata->input_work, msecs_to_jiffies(
			ddata->pdata->poll_interval));
	}
	mutex_unlock(&ddata->enable_lock);
	dprintk("ENABLED");
	
	return 0;
	
enable_failed:
	dprintk("ENABLE FAILED");
	bma2x2_disable(ddata);\
	return ERR;
}
#if 0
/* Input polled device functions: 
 * device_open,  device_close, device_suspend, device_resume and device_poll.
 * 
 * device_poll is the key function that used to polling data by user programe.
 */
static void bma2x2_input_polled_open(struct input_polled_dev *input_polled)
{
	struct driver_data *ddata = input_polled->private;
	bma2x2_enable(ddata);
}

static void bma2x2_input_polled_close(struct input_polled_dev *input_polled)
{
	struct driver_data *ddata = input_polled->private;
	bma2x2_disable(ddata);
}

static int bma2x2_input_polled_suspend(struct input_polled_dev *input_polled)
{
	struct driver_data *ddata = input_polled->private;

	ddata->on_before_suspend = atomic_read(&ddata->enable);
	return bma2x2_disable(ddata);
}

static int bma2x2_input_polled_resume(struct input_polled_dev *input_polled)
{
	struct driver_data *ddata = input_polled->private;

	if (ddata->on_before_suspend)
		return bma2x2_enable(ddata);
	
	return 0;
}

static int bma2x2_input_polled_set_options(struct input_polled_dev *input_polled,
				const char *buf,
				size_t count)
{
	struct driver_data *ddata = input_polled->private;
	char *temp;
	char *option;
	unsigned long value;
	unsigned char converted;

	temp = kmalloc(sizeof(*buf),GFP_KERNEL);
	strcpy(temp, buf);

	option = strsep(&temp, " ");
	temp   = strsep(&temp, " ");
	if (strict_strtoul(temp, 10, &value))
		return -EINVAL;
		
	if (strcmp(option, "poll") == 0) {
			if (ddata->sensor_type == BMA280_TYPE)
				if ((unsigned char) value > 14)
					return -EINVAL;
				
			converted = bma2x2_convert_bw_value(value);
			if (bma2x2_set_bandwidth(ddata->client, converted) < 0)
				return -EINVAL;
	} else if (strcmp(option, "range") == 0) {	
			
			converted = bma2x2_convert_range_value(value);
			if (bma2x2_set_range(ddata->client, converted) < 0)
				return -EINVAL;
	} else {
		return -EINVAL;
	}
	
	dprintk("Set %s: value:%ld, converted as:0x%x", option, value, converted);
	//kfree(temp);
	return 0;
}

static int bma2x2_input_polled_get_options(struct input_polled_dev *input_polled,
				char *buf)
{
	struct driver_data *ddata = input_polled->private;
	char *mode;
	unsigned char converted;
	unsigned char bandwidth;
	unsigned char range;

	if (bma2x2_get_mode(ddata->client, &converted) < 0)
		return sprintf(buf, "Read error\n");
	mode = mode_table[converted];
	
	if (bma2x2_get_bandwidth(ddata->client, &converted) < 0)
		return sprintf(buf, "Read error\n");

	bandwidth = bma2x2_unconvert_bw_value(converted);
	
	if (bma2x2_get_range(ddata->client, &converted) < 0)
		return sprintf(buf, "Read error\n");
	
	range = bma2x2_unconvert_range_value(converted);

	return sprintf(buf, "Mode:%s\nPoll:%dms\nRnage:%dG\n", mode, bandwidth, range);
}
#endif
static void bma2x2_work_func (struct work_struct *work)
{
	struct driver_data *ddata = container_of((struct delayed_work *)work,
			struct driver_data, input_work);
	static struct bma2x2acc acc;
	s16 xyz[3];

	bma2x2_read_accel_xyz(ddata->client, ddata->sensor_type,
								 &acc);
	dprintk("x=%d, y=%d, z=%d", acc.x, acc.y, acc.z);
	/*
	input_report_abs(ddata->input_dev, ABS_X, acc.x);
	input_report_abs(ddata->input_dev, ABS_Y, acc.y);
	input_report_abs(ddata->input_dev, ABS_Z, acc.z);
	
	*/
	xyz[ddata->pdata->axis_map_x] = acc.x * sensitivity_table[ddata->sensor_type].converted;
	xyz[ddata->pdata->axis_map_y] = acc.y * sensitivity_table[ddata->sensor_type].converted;
	xyz[ddata->pdata->axis_map_z] = acc.z * sensitivity_table[ddata->sensor_type].converted;

	if (ddata->pdata->negate_x)
		xyz[0] = -xyz[0];
	if (ddata->pdata->negate_y)
		xyz[1] = -xyz[1];
	if (ddata->pdata->negate_z)
		xyz[2] = -xyz[2];
	
	dprintk("s:%d converted: x=%d, y=%d, z=%d",
		sensitivity_table[ddata->sensor_type].converted,
		xyz[0], xyz[1], xyz[2]);
	input_report_abs(ddata->input_dev, ABS_X, xyz[0]);
	input_report_abs(ddata->input_dev, ABS_Y, xyz[1]);
	input_report_abs(ddata->input_dev, ABS_Z, xyz[2]);
	input_sync(ddata->input_dev);
		
	if (atomic_read(&ddata->enabled)) {		
		queue_delayed_work(ddata->input_wq, &ddata->input_work, msecs_to_jiffies(
			ddata->pdata->poll_interval));
	}
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct driver_data *ddata = dev_get_drvdata(dev);
	mutex_lock(&ddata->data_lock);
	val = ddata->pdata->poll_interval;
	mutex_unlock(&ddata->data_lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct driver_data *ddata = dev_get_drvdata(dev);
	unsigned long interval_ms;
	unsigned char converted;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	
	if (!interval_ms)
		return -EINVAL;
	
	interval_ms = max(interval_ms,(unsigned long ) ddata->pdata->min_interval);

	mutex_lock(&ddata->data_lock);
	if (ddata->sensor_type == BMA280_TYPE)
		if ((unsigned char) interval_ms > 14)
			return -EINVAL;
		
	converted = bma2x2_convert_bw_value(interval_ms);
	if (bma2x2_set_bandwidth(ddata->client, converted) < 0)
		return -EINVAL;

	ddata->pdata->poll_interval = bma2x2_unconvert_bw_value(converted);
	mutex_unlock(&ddata->data_lock);
	
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct driver_data *ddata = dev_get_drvdata(dev);
	int val = atomic_read(&ddata->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct driver_data *ddata = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		bma2x2_enable(ddata);
	else
		bma2x2_disable(ddata);

	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(poll, 0664, attr_polling_rate_show, attr_polling_rate_store),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

/* Init device
 *
 */
static int bma2x2_init_ddata(struct driver_data *ddata)
{
	struct i2c_client *client = ddata->client;
	struct input_dev *input;
	struct sensors_pm_platform_data *pdata;
	unsigned char tempvalue;

	/* Init platform hardware */
	pdata = ddata->pdata;
	if (pdata->init_platform_hw(pdata)) {
		dprintk("init platform failed!");
		return ERR;
	}
	
	mutex_init(&ddata->data_lock);
	mutex_init(&ddata->enable_lock);
	
	/* read chip id */
	tempvalue = i2c_smbus_read_byte_data(ddata->client, BMA2X2_CHIP_ID_REG);
	switch (tempvalue) {
	case BMA255_CHIP_ID:
		ddata->sensor_type = BMA255_TYPE;
		break;
	case BMA250E_CHIP_ID:
		ddata->sensor_type = BMA250E_TYPE;
		break;
	case BMA2x2E_CHIP_ID:
		ddata->sensor_type = BMA2x2E_TYPE;
		break;
	case BMA280_CHIP_ID:
		ddata->sensor_type = BMA280_TYPE;
		break;
	case BMA250_CHIP_ID:
		ddata->sensor_type = BMA250_TYPE;
		break;
	default:
		ddata->sensor_type = -1;
	}
	
	if (ddata->sensor_type != -1) {
		ddata->chip_id = tempvalue;
		dprintk("Bosch Sensortec Device detected!\n");
		dprintk("%s registered I2C driver!\n",
						sensor_name[ddata->sensor_type]);
	} else{
		dprintk("Bosch Sensortec Device not found\n"
				"i2c error %d \n", tempvalue);
	}

	pdata->poll_interval = bma2x2_unconvert_bw_value(
		bma2x2_convert_bw_value(pdata->poll_interval));

	pdata->range = bma2x2_unconvert_range_value(
		bma2x2_convert_range_value(pdata->range));

	/* Init hardware */
	if ((bma2x2_set_mode(client, BMA2X2_MODE_NORMAL) < 0) || 
		(bma2x2_set_bandwidth(client, bma2x2_convert_bw_value(pdata->poll_interval)) < 0) ||
		(bma2x2_set_range(client, bma2x2_convert_range_value(pdata->range)) < 0)){
			return ERR;
	}	
#if 0
	/* Init input polled device */
	ddata->input_polled = input_allocate_polled_device();
	input_polled = ddata->input_polled;
	if (!input_polled) {
		dprintk("Failed to allocate memory for input polled");;
		return ERR;
	}
	
	input_polled->private = ddata;

	input_polled->open 		 = bma2x2_input_polled_open;
	input_polled->close 	 = bma2x2_input_polled_close;
	//input_polled->suspend  	 = bma2x2_input_polled_suspend;
	//input_polled->resume 	 = bma2x2_input_polled_resume;
	//input_polled->get_options= bma2x2_input_polled_get_options;
	//input_polled->set_options= bma2x2_input_polled_set_options;
	input_polled->poll 		 = bma2x2_work_func;
#endif
	/* Init input */
	ddata->input_wq = create_singlethread_workqueue("input_wq");
	INIT_DELAYED_WORK(&ddata->input_work, bma2x2_work_func);

	ddata->input_dev = input_allocate_device();
	if (!ddata->input_dev) {
		dev_err(&ddata->client->dev, "input device allocation failed\n");
		return ERR;
	}
	input = ddata->input_dev;			
	input->name = DEVICE_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &ddata->client->dev;

	set_bit(EV_ABS, input->evbit);
	//input_set_abs_params(input, ABS_X, -FS_MAX, FS_MAX, FUZZ, FLAT);
	//input_set_abs_params(input, ABS_Y, -FS_MAX, FS_MAX, FUZZ, FLAT);
	//input_set_abs_params(input, ABS_Z, -FS_MAX, FS_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(input, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(input, ABS_Z, ABSMIN, ABSMAX, 0, 0);

	/* Init sysfs */
#if 0
	if (input_register_polled_device(input_polled)) {
		dprintk("Failed to register input polled device");
		input_free_polled_device(input_polled);
		return ERR;
	}
#endif
	if (input_register_device(input)) {
		dprintk("failed to register input device");
		input_free_device(input);
		return ERR;
	}

	if (create_sysfs_interfaces(&ddata->client->dev)) {
		dprintk("create sysfs interfaces failed");
		return ERR;
	}
	
	if (sysfs_create_link(&devices_kset->kobj, &client->dev.kobj, "accelerometer")) {
		dprintk("create symbolic link to device failed");
	}
	

		
	return 0;
}

/* Validate platform data
 * 
 */
static int bma2x2_init_pdata(struct sensors_pm_platform_data *pdata)
{
	if (!pdata->init_platform_hw    ||
		!pdata->suspend_platform_hw ||
		!pdata->resume_platform_hw  ||
		!pdata->exit_platform_hw) {

		dprintk("functions of pdata missing");
		return ERR;
	}
	
	if (pdata->axis_map_x > 2 ||
	    pdata->axis_map_y > 2 ||
	    pdata->axis_map_z > 2) {
	    
		dprintk("invalid axis maps");
		return ERR;
	}

	if (pdata->negate_x > 1 ||
	    pdata->negate_y > 1 ||
	    pdata->negate_z > 1) {
	    
		dprintk("invalid negate values");
		return ERR;
	}
	
	pdata->poll_interval = max(pdata->poll_interval, pdata->min_interval);
	if (pdata->poll_interval < pdata->min_interval) {
		dprintk("minimum poll interval violated");
		return ERR;
	}
	
	return 0;
}


/* I2C functions:
 * 	probe, remove, resume and suspend
 * 
 */
static int bma2x2_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct driver_data *ddata;
	
	
	if (!client->dev.platform_data) {
		dprintk("platform data is NULL");
		goto probe_failed;
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dprintk("I2C is not capable");
		goto probe_failed;
	}

	if (bma2x2_init_pdata(client->dev.platform_data) < 0) {
		dprintk("validate pdata failed");
		goto probe_failed;
	}
	
	ddata = kzalloc(sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL) {
		dprintk("failed to allocate memory for driver data");
		goto probe_failed;
	}
	
	i2c_set_clientdata(client, ddata);
	ddata->client = client;
	ddata->pdata = client->dev.platform_data;
	ddata->pdata->i2c_client_dev = &client->dev;
	
	if (bma2x2_init_ddata(ddata) < 0) {
		dprintk("failed to init driver");
		goto init_failed;
	}
	
	dprintk("BMA2X2 Dirver Loaded");
	return 0;

init_failed:
	ddata->pdata->exit_platform_hw(ddata->pdata);
	//kfree(ddata->pdata); /*This will cause kernel panic when init_failed*/
	kfree(ddata);
probe_failed:
	return -ENODEV;
}

static int bma2x2_remove(struct i2c_client *client)
{
	struct driver_data *ddata = i2c_get_clientdata(client);

	dprintk("REMOVE");
	//sysfs_interfaces_remove(&client->dev);
#if 0
	input_unregister_polled_device(ddata->input_polled);
	input_free_polled_device(ddata->input_polled);
#endif
	remove_sysfs_interfaces(&client->dev);
	input_unregister_device(ddata->input_dev);
	input_free_device(ddata->input_dev);

	ddata->pdata->exit_platform_hw(ddata->pdata);
	kfree(ddata->pdata);
	kfree(ddata);
	
	return 0;
}

static int bma2x2_resume(struct device *dev)
{
	struct driver_data *ddata = i2c_get_clientdata(to_i2c_client(dev));

	dprintk("RESUME");
	if (ddata->on_before_suspend)
		return bma2x2_enable(ddata);
		
	return 0;
}

static int bma2x2_suspend(struct device *dev)
{
	struct driver_data *ddata = i2c_get_clientdata(to_i2c_client(dev));
	
	dprintk("SUSPEND");
	ddata->on_before_suspend = atomic_read(&ddata->enabled);
	return bma2x2_disable(ddata);
}

static const struct i2c_device_id bma2x2_id[] = {
	{ DEVICE_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bma2x2_id);

static struct dev_pm_ops bma2x2_pm = {
	.suspend = bma2x2_suspend,
	.resume  = bma2x2_resume,
};

static struct i2c_driver bma2x2_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name  = DEVICE_NAME,
			.pm    = &bma2x2_pm,
	},
	.probe    = bma2x2_probe,
	.remove   = __devexit_p(bma2x2_remove),
	.id_table = bma2x2_id,
};

/*Module init and exit
 * 
 */
static int __init BMA2X2_init(void)
{
	return i2c_add_driver(&bma2x2_driver);
}

static void __exit BMA2X2_exit(void)
{
	i2c_del_driver(&bma2x2_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA2X2 accelerometer sensor driver");
MODULE_LICENSE("GPL");

module_init(BMA2X2_init);
module_exit(BMA2X2_exit);
