/*
 * linux/drivers/power/bq27541.h
 *
 * OMAP4: bq27520 gas gauge driver for Linux
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Copyright (C) 2011 Innocomm Mobile Technology Corp.
 *
 * Code re-written by:
 * Innocomm Mobile Technology Corp.
 *
 * Initial Code:
 * Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef BQ_H_
#define BQ_H_

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#if 1
#define BQ27541 "bq27541"
#define BQ27541_ADDR 0x55
#else
#define BQADDR 0x55
#endif

#define ADAPTER_NR 2



/*
 * In order to write to the BQ gauge, we must write an additional 8 bit subaddress to read from.
 * Therefore, we cannot use the read() function, and must use the IOCTL_RDWR ioctl.  This requires
 * the following structure:
 */
typedef struct i2c_rdwr_ioctl_data_s {
	struct i2c_msg *msgs;	/* pointers to i2c_msgs */
	__u32 nmsgs;			/* number of i2c_msgs */
} i2c_rdwr_ioctl_data;

/*
 * bq_dataram structure: this structure contains all of the data from the gauge's dataram and should
 * suffice for most purposes.  Note that the structure is in the same order as the addresses on the gauge.
 * This means that we can copy directly from the gauge to the structure without re-ordering the bytes.
 */
typedef struct bq_dataram_s {
	unsigned short Control;
	short AtRate;
	unsigned short UnfilteredSOC;
	short Temp;
	unsigned short Voltage;
	unsigned short Flags;
	unsigned short NomAvailCap;
	unsigned short FullAvailCap;
	unsigned short RemCap;
	unsigned short FullChgCap;
	short AvgCurr;
	unsigned short TTE;
	unsigned short FilteredFCC;
	unsigned short StbyCurr;
	unsigned short UnfilterFCC;
	unsigned short MaxLoadCurr;
	unsigned short UnfilteredRM;
	unsigned short FilteredRM;
	unsigned short AvgPow;
	unsigned short Reverse;
	unsigned short InternalTemp;
	unsigned short CycleCnt;
	unsigned short StateOfChg;
	unsigned short StateOfHealth;
} bq_dataram;

typedef struct bq_ver_s {
	int Fw_version;
	int Df_version;
	u16 Control_status;
} bq_ver;

/*
 * seti2caddr: Sets the I2C address to a custom value
 * Should NOT be neccessary in normal operation.  Only needed if you enter ROM mode to reflash device.
 * Arguments: unsigned char addr: 8 bit address value (lower 7 bits are the address)
 * Returns: ioctl failure code on failure: 0 otherwise
 */
//int seti2caddr(unsigned char addr);
#endif /*BQ_H_*/
