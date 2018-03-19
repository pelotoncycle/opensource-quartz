/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: l3g4200d.h
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1.3 sysfs
* Date			: 2011/Sep/24
* Description		: L3G4200D digital output gyroscope sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0		| 2010/Aug/19	| Carmine Iascone	| First Release
* 1.1.0		| 2011/02/28	| Matteo Dameno		| Self Test Added
* 1.1.1		| 2011/05/25	| Matteo Dameno		| Corrects Polling Bug
* 1.1.2		| 2011/05/30	| Matteo Dameno		| Corrects ODR Bug
* 1.1.3		| 2011/06/24	| Matteo Dameno		| Corrects ODR Bug
* 1.1.4		| 2011/09/24	| Matteo Dameno		| forces BDU setting
*******************************************************************************/

#ifndef __L3G4200D_H__
#define __L3G4200D_H__

#define L3G4200D_MIN_POLL_PERIOD_MS	2

#define SAD0L				0x00
#define SAD0H				0x01
#define L3G4200D_GYR_I2C_SADROOT	0x34
#define L3G4200D_GYR_I2C_SAD_L		((L3G4200D_GYR_I2C_SADROOT<<1)|SAD0L)
#define L3G4200D_GYR_I2C_SAD_H		((L3G4200D_GYR_I2C_SADROOT<<1)|SAD0H)

#define L3G4200D_GYR_DEV_NAME		"l3g4200d"

#define L3G4200D_GYR_FS_250DPS	0x00
#define L3G4200D_GYR_FS_500DPS	0x10
#define L3G4200D_GYR_FS_2000DPS	0x30

#define L3G4200D_GYR_ENABLED	1
#define L3G4200D_GYR_DISABLED	0

/** Maximum polled-device-reported rot speed value value in dps*/
#define FS_MAX			32768

/* l3g4200d gyroscope registers */
#define WHO_AM_I        0x0F

#define CTRL_REG1       0x20    /* CTRL REG1 */
#define CTRL_REG2       0x21    /* CTRL REG2 */
#define CTRL_REG3       0x22    /* CTRL_REG3 */
#define CTRL_REG4       0x23    /* CTRL_REG4 */
#define CTRL_REG5       0x24    /* CTRL_REG5 */

/* CTRL_REG1 */
#define ALL_ZEROES	0x00
#define PM_OFF		0x00
#define PM_NORMAL	0x08
#define ENABLE_ALL_AXES	0x07
#define BW00		0x00
#define BW01		0x10
#define BW10		0x20
#define BW11		0x30
#define ODR100		0x00  /* ODR = 100Hz */
#define ODR200		0x40  /* ODR = 200Hz */
#define ODR400		0x80  /* ODR = 400Hz */
#define ODR800		0xC0  /* ODR = 800Hz */

/* CTRL_REG4 bits */
#define	FS_MASK				0x30
#define	BDU_ENABLE			0x80

#define	SELFTEST_MASK			0x06
#define L3G4200D_SELFTEST_DIS		0x00
#define L3G4200D_SELFTEST_EN_POS	0x02
#define L3G4200D_SELFTEST_EN_NEG	0x04

#define AXISDATA_REG			0x28

#define FUZZ			0
#define FLAT			0
#define AUTO_INCREMENT		0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RESUME_ENTRIES		5

/** Registers Contents */
#define WHOAMI_L3G4200D		0x00D3	/* Expected content for WAI register*/

#endif  /* __L3G4200D_H__ */
