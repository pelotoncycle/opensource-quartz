#ifndef __ELAN_TP_H__
#define __ELAN_TP_H__

#include <linux/input.h>

#define ELAN_8232_I2C_NAME "elan-touch"
#define ELAN_FS_8713 "elan-forcesensor" 
#define ELAN_EKTF2000_NAME "elan-ektf-touchpad"
#define ELAN_OFN_8713 "elan-ofn-sensor"
#define ELAN_TP_1058 "elan-ekt1058-touchpad"
#define ELAN_TP_2154 "elan-ekt2154-touchpad"
#define ELAN_USB_FS "elan-usb-fs"

/*for EPL6802 */
#define ELAN_LS_EPL6802 "elan-EPL6802"
#define ELAN_EPL6802_IOCTL_GET_PFLAG _IOR(ELAN_IOCTL_MAGIC, 1, int *)
#define ELAN_EPL6802_IOCTL_GET_LFLAG _IOR(ELAN_IOCTL_MAGIC, 2, int *)
#define ELAN_EPL6802_IOCTL_ENABLE_PFLAG _IOW(ELAN_IOCTL_MAGIC, 3, int *)
#define ELAN_EPL6802_IOCTL_ENABLE_LFLAG _IOW(ELAN_IOCTL_MAGIC, 4, int *)
#define ELAN_EPL6802_IOCTL_GETDATA _IOR(ELAN_IOCTL_MAGIC, 5, int *)

#define INT_TIME_16		0x00
#define INT_TIME_24		0x10
#define INT_TIME_32		0x20
#define INT_TIME_48		0x30
#define INT_TIME_80		0x40
#define INT_TIME_144		0x50
#define INT_TIME_272		0x60
#define INT_TIME_384		0x70
#define INT_TIME_DEFAULT	INT_TIME_24

/*for EPL6802 and EPL6804 command */
#define REG_0			0X00
#define REG_1			0X01
#define REG_2			0X02
#define REG_3			0X03
#define REG_4			0X04
#define REG_5			0X05
#define REG_6			0X06
#define REG_7			0X07
#define REG_8			0X08
#define REG_9			0X09
#define REG_10			0X0A
#define REG_11			0X0B
#define REG_12			0X0C
#define REG_13			0X0D
#define REG_14			0X0E
#define REG_15			0X0F
#define REG_16			0X10
#define REG_17			0X11
#define REG_18			0X12
#define REG_19			0X13
#define REG_20			0X14
#define REG_21			0X15

#define W_SINGLE_BYTE		0X00
#define W_TWO_BYTE		0X01
#define W_THREE_BYTE		0X02
#define W_FOUR_BYTE		0X03
#define W_FIVE_BYTE		0X04
#define W_SIX_BYTE		0X05
#define W_SEVEN_BYTE		0X06
#define W_EIGHT_BYTE		0X07

#define R_SINGLE_BYTE		0X00
#define R_TWO_BYTE		0X01
#define R_THREE_BYTE		0X02
#define R_FOUR_BYTE		0X03
#define R_FIVE_BYTE		0X04
#define R_SIX_BYTE		0X05
#define R_SEVEN_BYTE		0X06
#define R_EIGHT_BYTE		0X07

#define EPL_SENSING_1_TIME	(0 << 5) 
#define EPL_SENSING_2_TIME	(1 << 5)
#define EPL_SENSING_4_TIME	(2 << 5)
#define EPL_SENSING_8_TIME	(3 << 5)
#define EPL_SENSING_16_TIME	(4 << 5)
#define EPL_SENSING_32_TIME	(5 << 5)
#define EPL_SENSING_64_TIME	(6 << 5)
#define EPL_SENSING_128_TIME	(7 << 5)
#define EPL_C_SENSING_MODE	(0 << 4)
#define EPL_S_SENSING_MODE	(1 << 4)
#define EPL_ALS_MODE		(0 << 2)
#define EPL_PS_MODE		(1 << 2)
#define EPL_TEMP_MODE 		(2 << 2)
#define EPL_H_GAIN		(0)
#define EPL_M_GAIN		(1)
#define EPL_L_GAIN		(2)
#define EPL_AUTO_GAIN		(3)


#define EPL_8BIT_ADC		0
#define EPL_10BIT_ADC		1
#define EPL_12BIT_ADC		2
#define EPL_14BIT_ADC		3


#define EPL_C_RESET		(0 << 2)
#define EPL_C_START_RUN		(1 << 2)
#define EPL_C_P_UP		(0 << 1)
#define EPL_C_P_DOWN		(1 << 1)
#define EPL_DATA_LOCK		1
#define EPL_DATA_UNLOCK		0

#define EPL_INT_BINARY		0
#define EPL_INT_DISABLE		2
#define EPL_INT_ACTIVE_LOW	3

#endif
