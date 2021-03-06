
/*
 * linux/drivers/power/bq27541.c
 *
 * OMAP4: bq27541 gas gauge driver for Linux
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



#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#ifdef CONFIG_TWL6032_BCI_BATTERY
#include <linux/i2c/twl6032_bci_battery.h>
#else // for CONFIG_TWL603x_BCI_BATTERY and others... */
#include <linux/i2c/twl6030_bci_battery.h>
#endif
#include <linux/power_supply.h>
#include <linux/i2c/bq27541.h>
#include "bq_dffs_g3.h"

#define REV_BQ27541 27541

//cmd
#define BQ27541_CMD_CTRL 0x00
#define BQ27541_CMD_AR 0x02		//atRate
#define BQ27541_CMD_UFSOC 0x04		//predicted remaining battery capacity
#define BQ27541_CMD_TEMP 0x06		//temperature
#define BQ27541_CMD_VOLT 0x08		//voltage
#define BQ27541_CMD_FLAGS 0x0a		
#define BQ27541_CMD_NAC 0x0c		//nominal availabe capacity
#define BQ27541_CMD_FAC 0x0e		//full available capacity
#define BQ27541_CMD_RM 0x10		//remaining capacity
#define BQ27541_CMD_FCC 0x12		//full charge capacity
#define BQ27541_CMD_AI 0x14			//avg current
#define BQ27541_CMD_TTE 0x16		//time to empty
#define BQ27541_CMD_FFCC 0x18		//filtered compensated capacity
#define BQ27541_CMD_SI 0x1a			//standby current
#define BQ27541_CMD_UFFCC 0x1c		//compensated capacity of the battery
#define BQ27541_CMD_MLI 0x1e		//max load current
#define BQ27541_CMD_UFRM 0x20		//compensated battery capacity remaining
#define BQ27541_CMD_FRM 0x22		//filtered compensated battery capacity remaining
#define BQ27541_CMD_AP 0x24		//avg power 
#define BQ27541_CMD_INTTEMP 0x28		//internal temperature
#define BQ27541_CMD_CC 0x2a		//cycle count
#define BQ27541_CMD_SOC 0x2c		//state of charge
#define BQ27541_CMD_SOH 0x2e		//state of health
#define BQ27541_CMD_PCHG 0x34		//data log buffer jss
#define BQ27541_CMD_DOD0 0x36		//depth of discharge
#define BQ27541_CMD_SDSG 0x38		//battery self discharge current

#define ENABLE_LOW_BAT_INT (1<<5)

//sub
#define CTRL_SUB_DEVICE_TYPE 0x0001
#define CTRL_SUB_FW_REV 0x0002
//#define CTRL_SUB_DF_VER 0x001f
#define CTRL_SUB_DF_VER 0x000c
#define CTRL_SUB_CONTROL_STATUS 0x0000
// #define CTRL_SUB_FACTORY_RESTORE 0x0015
#define CTRL_SUB_RESET 0x0041

//romupdate
#define CTRL_ROM_ENTER 0x0f00
#define CTRL_ROM_EXIT_1 0x0f
#define CTRL_ROM_EXIT_2 0x000f

#define BUFFERSIZE             32           // # of bytes for Tx & Rx buffers
unsigned char TxData[BUFFERSIZE+1];           // Stores data bytes to be TX'd
unsigned char RxData[BUFFERSIZE+1];           // Stores data bytes that are RX'd

struct wake_lock wakelock;
bool mNotupdatedata = false;
bool scaled = false;

//bq_dataram dr;
struct bq27541_device_info {
	struct device		*dev;
	struct i2c_client	*client;
	struct notifier_block	nb;
	unsigned short		bqchip_version;

	int			active;
	int			eoc_enabled;
	unsigned int	debug;
	unsigned short pre_soc;	
	
};

/*
 * writebq: write a string of bytes to the BQ gauge
 * Arguments:	unsigned char cmd: subaddress value: this is the address on the gauge that you are writing to
 * 	                    unsigned char * buf: pointer to string of bytes that you will write
 *                     int cnt: number of bytes you will write
 * Returns: write failure code on failure, otherwise number of bytes written 
 */
int writebq(struct bq27541_device_info *di, u8 cmd, u8* buf, unsigned cnt);

/*
 * readbq: reads bytes from BQ gauge
 * Arguments:	unsigned char cmd: subaddress: this is the address on the gauge that you will read from
 *                     unsigned char* buf: buffer that you will read into
 *                     int cnt: number of bytes to read into buffer
 */
int readbq(struct bq27541_device_info *di, u8 cmd,u8* buf, unsigned cnt);

/*
 * readbqdataram: Reads entirity of BQ gauge dataram into a bq_dataram structure (defined in bq.h)
 * Arguments:	bq_dataram *dr: pointer to structure
 * Returns: ioctl failure code on fail, otherwise 0
 */
int readbqdataram(struct bq27541_device_info *di, bq_dataram *dr);
static int bq27541_read_byte(struct bq27541_device_info *di, u8 *value, u8 reg);
static int bq27541_read_block(struct bq27541_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes, bool is_rom_mode);
static int bq27541_write_byte(struct bq27541_device_info *di, u8 value, u8 reg, bool is_rom_mode);
static int bq27541_write_two_byte(struct bq27541_device_info *di, u16 value, u8 reg, bool is_rom_mode);
static int bq27541_write_block(struct bq27541_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes, bool is_rom_mode);


unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb)
{
	unsigned int tmp;
  
	tmp = ((msb << 8) & 0xFF00);
	return ((unsigned int)(tmp + lsb) & 0x0000FFFF);  
}

unsigned int readBQ27541Block(struct bq27541_device_info *di, bq_dataram *dr){
	bq_dataram newbqd;
	int ret;
	memset(&newbqd,0,sizeof(bq_dataram));
	ret = readbqdataram(di, &newbqd);
	if (ret != 2) {
		printk("!!!!!!!!!!!!!!!!!!!!!!! i2c_transfer has problem !!!!!!!!!!!!!!!!!!!!!!!!\n");
		return ret;
	}
	if ((newbqd.Temp != 0 ) && (newbqd.Voltage != 0)) {
		if(newbqd.StateOfChg==0 && di->pre_soc!=0) {
			printk("!!!!!!!!!!!!!!!!!!!!!!! keep soc as pre_soc !!!!!!!!!!!!!!!!!!!!!!!!\n");
			di->pre_soc = 0;
		}else{
			di->pre_soc = newbqd.StateOfChg;
			memcpy(dr, &newbqd, sizeof(bq_dataram));
		}
	} else {
			printk("!!!!!!!!!!!!!!!!!!!!!!! Temp or Voltage empty, keep soc as pre_soc !!!!!!!!!!!!!!!!!!!!!!!!\n");
	}

#if 0
	//print the results
if(di->debug){
	printk("AtRate: %d\n",dr.AtRate);
	printk("AtRateTTE: %d\n",dr.AtRateTTE);
	printk("AvailEnergy: %d\n",dr.AvailEnergy);
	printk("AvgCurr: %d\n",dr.AvgCurr);
	printk("AvgPow: %d\n",dr.AvgPow);
	printk("Control: %d\n",dr.Control);
	printk("CycleCnt: %d\n",dr.CycleCnt);
	printk("Flags: %d\n",dr.Flags);
	printk("FullAvailCap: %d\n",dr.FullAvailCap);
	printk("FullChgCap: %d\n",dr.FullChgCap);
	printk("MaxLoadCurr: %d\n",dr.MaxLoadCurr);
	printk("MaxLoadTTE: %d\n",dr.MaxLoadTTE);
	printk("NomAvailCap: %d\n",dr.NomAvailCap);
	printk("RemCap: %d\n",dr.RemCap);
	printk("StateOfChg: %d\n",dr.StateOfChg);
	printk("StbyCurr: %d\n",dr.StbyCurr);
	printk("StbyTTE: %d\n",dr.StbyTTE);
	printk("Temp: %d\n",dr.Temp);
	printk("TTE: %d\n",dr.TTE);
	printk("TTEAtConstPow: %d\n",dr.TTEAtConstPow);
	printk("TTF: %d\n",dr.TTF);
	printk("Voltage: %d\n",dr.Voltage);
}
#endif

	return 0;
}
void updateBQ27541CheckSum(struct bq27541_device_info *di){
	unsigned char new_checksum = 0;
	int i = 0;
	unsigned int sum = 0;
	
	memset(RxData,0,sizeof(RxData));
	bq27541_read_block(di, RxData, 0x40,  BUFFERSIZE, false);
	for (i = 0; i < BUFFERSIZE; i++)          
	{
		sum += RxData[i];                       
	}
	new_checksum = (0xFF - (sum & 0x00FF));       
	printk("new_checksum = 0x%02x \nwrite checksum to 0x60==>", new_checksum);
	bq27541_write_byte(di, new_checksum , 0x60, false);
	msleep(200);
		
}
unsigned int updateBQ27541Reg(struct bq27541_device_info *di, unsigned int new_val, u8 reg){
	unsigned int val = 0;
	
// 1. read original val
	memset(RxData,0,sizeof(RxData));
  	bq27541_read_block(di, RxData, reg,  2, false);
  	val = transBytes2UnsignedInt(RxData[0], RxData[1]);
	printk("original val = 0x%02x = %d\n",  val, val);

	if(val!=new_val){	
		unsigned char msb, lsb;		
		
// 2. write new val
		printk("ori val = %d  --> prepare to write new val as %d\n", val, new_val);
		msb = ((new_val >> 8) & 0x00FF);
		lsb = (new_val & 0x00FF);
		bq27541_write_byte(di, msb, reg, false);
		bq27541_write_byte(di, lsb, reg+1, false);

// 3. write checksum
		updateBQ27541CheckSum(di);

// 4. check new val
		memset(RxData,0,sizeof(RxData));
  		bq27541_read_block(di, RxData, reg,  2, false);
	  	val = transBytes2UnsignedInt(RxData[0], RxData[1]);
		printk("new val = 0x%02x = %d\n", val, val);	
	}else{
		printk("original val == new val => %d\n", val);
	}      
	
	
	return 0;
}
unsigned int writeBQ27541TermVolt(struct bq27541_device_info *di, unsigned int new_val){
	u8 reg = 0x43;
	if (!(scaled)) {
		printk("%s() - val = %d\n", __func__, new_val);
		bq27541_write_byte(di, 0x00, 0x61, false);
		bq27541_write_byte(di, 0x50, 0x3e, false);
		bq27541_write_byte(di, 0x02, 0x3f, false);	
		msleep(50);
		updateBQ27541Reg(di, new_val, reg);
	} else {
		printk("%s() - scaled, nothing to do\n", __func__);
	}

	return 0;
}
unsigned int writeBQ27541SysDnVoltThre(struct bq27541_device_info *di, unsigned int new_val){
#if 0
	u8 reg = 0x45;
	printk("%s() - val = %d \n", __func__, new_val);
	bq27541_write_byte(di, 0x00, 0x61, false);
	bq27541_write_byte(di, 0x31, 0x3e, false);
	bq27541_write_byte(di, 0x00, 0x3f, false);	
	msleep(50);
	updateBQ27541Reg(di, new_val, reg);
#else
	printk("%s() - not support\n", __func__);
#endif
	return 0;
}
unsigned int writeBQ27541SOC1(struct bq27541_device_info *di, unsigned int new_val){
	u8 reg = 0x40;
	u8 read_reg = 0;

	if (!(scaled)) {
		printk("%s() - val = %d \n", __func__, new_val);
		bq27541_write_byte(di, 0x00, 0x61, false);
		bq27541_write_byte(di, 0x31, 0x3e, false);
		bq27541_write_byte(di, 0x00, 0x3f, false);	
		msleep(50);

		bq27541_read_byte(di, &read_reg, reg);		
		printk("soc1 = [%d]\n", read_reg);

		bq27541_write_byte(di, new_val, reg, false);
		updateBQ27541CheckSum(di);
	} else {
		printk("%s() - scaled, nothing to do\n", __func__);
	}

	return 0;
}
unsigned int enableBQ27541LowBatInt(struct bq27541_device_info *di){
#if 0
	u8 reg = 0x4B;
	u8 read_reg = 0;
	
	printk("%s() \n", __func__);
	bq27541_write_byte(di, 0x00, 0x61, false);
	bq27541_write_byte(di, 0x40, 0x3e, false);
	bq27541_write_byte(di, 0x00, 0x3f, false);	
	msleep(50);
	bq27541_read_byte(di, &read_reg, reg);	
	printk("operation configuration b = [%08x]\n", read_reg);

	if(((ENABLE_LOW_BAT_INT)&read_reg)==0){
		bq27541_write_byte(di, ENABLE_LOW_BAT_INT|read_reg, reg, false);
		printk("write new operation configuration b as [%08x] \n", ENABLE_LOW_BAT_INT|read_reg);
		updateBQ27541CheckSum(di);
	}
#else
	printk("%s() - not support\n", __func__);
#endif
	return 0;
}

int asciitohex(int start, int end, u8 *value, char *rawdata)
{
    int index = 0;
    bool next = false;

    memset(value,0,98);

    while (start <= end)
    {
        switch (rawdata[start])
        {
            case '0': case '1': case '2': case '3': case '4': 
            case '5': case '6': case '7': case '8': case '9': 
                value[index] = (value[index] << 4) | (rawdata[start] & 0xf);
                next = true;
                break;
            case 'a': case 'b': case 'c': case 'd': case 'e': case 'f': 
            case 'A': case 'B': case 'C': case 'D': case 'E': case 'F': 
                value[index] = (value[index] << 4) | ( 9 + (rawdata[start] & 0xf) );
                next = true;
                break;
            default:
                if (next)
                {
                    index++;
                    next = false;
                }
        }
        start++;
    }
    return index;
}

int asciitodec(int start, int end, char *rawdata)
{
    int value=0;
    bool canreturn = false;

    while (start<=end)
    {
        switch (rawdata[start])
        {
            case '0': case '1': case '2': case '3': case '4': 
            case '5': case '6': case '7': case '8': case '9': 
                value = value*10 + (rawdata[start] & 0xf);
                canreturn = true;
                break;
            default:
                if (canreturn)
                {
                    return value;
                }
        }
        start++;
    }
    return value;
}

void unlockbq(struct bq27541_device_info *di)
{
    unsigned char msb, lsb;
    int bytes;
//    int i;

    // 0. unlock 
    memset(TxData,0,sizeof(TxData));
    msb = ((0x0414 >> 8) & 0x00FF);
    lsb = (0x0414 & 0x00FF);
    bytes = 0;
    TxData[bytes++] = lsb;
    TxData[bytes++] = msb;
    bq27541_write_block(di, TxData, 0x00, 2, false);

    memset(TxData,0,sizeof(TxData));
    msb = ((0x3672 >> 8) & 0x00FF);
    lsb = (0x3672 & 0x00FF);
    bytes = 0;
    TxData[bytes++] = lsb;
    TxData[bytes++] = msb;
    bq27541_write_block(di, TxData, 0x00, 2, false);
}

int enterrommode(struct bq27541_device_info *di)
{
    return bq27541_write_two_byte(di, CTRL_ROM_ENTER, BQ27541_CMD_CTRL, false);
}

void exitrommode(struct bq27541_device_info *di)
{
    printk("exit 1 [%d]\n", bq27541_write_byte(di, CTRL_ROM_EXIT_1, BQ27541_CMD_CTRL, true));
    printk("exit 2 [%d]\n", bq27541_write_two_byte(di, CTRL_ROM_EXIT_2, 0x64, true));
    msleep(5000);
}

void scaletest(struct bq27541_device_info *di)
{
    int ret;
    u8 read_reg = 0;

    printk("Write 0x00 to reg 0x00 : %d\n", bq27541_write_byte(di, 0x00, 0x00, false));
    printk("Write 0x00 to reg 0x01 : %d\n", bq27541_write_byte(di, 0x00, 0x01, false));
    ret = bq27541_read_byte(di, &read_reg, 0x01);
    printk("read reg 0x01: %d [%02x]\n", ret, read_reg);
    scaled = read_reg & (1 << 5);
}

unsigned int readdffsversion(struct bq27541_device_info *di)
{
#if 0
    bq27541_write_byte(di, 0x00, 0x61, false);  //BlockdataControl()
    bq27541_write_byte(di, 0x30, 0x3e, false);  //DataFlashClass()
    bq27541_write_byte(di, 0x00, 0x3f, false);  //DataFlashBlock()

    msleep(50);

    memset(RxData,0,sizeof(RxData));
    bq27541_read_block(di, RxData, 0x58, 2, false);

    return transBytes2UnsignedInt(RxData[1], RxData[0]);
#else
    return 0;
#endif
}

void writedffsversion(struct bq27541_device_info *di)
{
#if 0
    bq27541_write_byte(di, 0x00, 0x61, false);  //BlockdataControl()
    bq27541_write_byte(di, 0x30, 0x3e, false);  //DataFlashClass()
    bq27541_write_byte(di, 0x00, 0x3f, false);  //DataFlashBlock()
    msleep(50);

// 3. write new data flash version
    bq27541_write_two_byte(di, DFFS_VERSION, 0x58, false);

    msleep(50);

// 4. write checksum
    updateBQ27541CheckSum(di);

// 5. check new data flash version
     memset(RxData,0,sizeof(RxData));
     bq27541_read_block(di, RxData, 0x58, 2, false);
     printk("new Version = [%02x] [%02x]\n", RxData[0], RxData[1]); 
#endif
}

unsigned int readDeviceType(struct bq27541_device_info *di){
    int ret;

    ret = bq27541_write_two_byte(di, CTRL_SUB_DEVICE_TYPE, BQ27541_CMD_CTRL, false);
    if (ret != 0)
    {
        printk("Write CTRL_SUB_DEVICE_TYPE fail\n");
        return 0;
    }
    msleep(20);

    memset(RxData,0,sizeof(RxData));
    ret = bq27541_read_block(di, RxData,BQ27541_CMD_CTRL,  2, false);
    if (ret != 0)
    {
        printk("Read BQ27541_CMD_CTRL fail\n");
        return 0;
    }    
    return transBytes2UnsignedInt(RxData[1], RxData[0]);
}

unsigned int readDFVersion(struct bq27541_device_info *di){
    int ret;

    ret = bq27541_write_two_byte(di, CTRL_SUB_DF_VER, BQ27541_CMD_CTRL, false);
    if (ret != 0)
    {
        printk("Write CTRL_SUB_DF_VER fail\n");
        return 0;
    }
    msleep(20);

    memset(RxData,0,sizeof(RxData));
    ret = bq27541_read_block(di, RxData,BQ27541_CMD_CTRL,  2, false);
    if (ret != 0)
    {
        printk("Read BQ27541_CMD_CTRL fail\n");
        return 0;
    }    
    return transBytes2UnsignedInt(RxData[1], RxData[0]);
}

#if 0
unsigned int readFWVersion(struct bq27541_device_info *di){
    int ret;

    ret = bq27541_write_two_byte(di, CTRL_SUB_FW_REV, BQ27541_CMD_CTRL, false);
    if (ret != 0)
    {
        printk("Write CTRL_SUB_FW_REV fail\n");
        return 0;
    }
    msleep(20);

    memset(RxData,0,sizeof(RxData));
    ret = bq27541_read_block(di, RxData,BQ27541_CMD_CTRL,  2, false);
    if (ret != 0)
    {
        printk("Read BQ27541_CMD_CTRL fail\n");
        return 0;
    }    
    return transBytes2UnsignedInt(RxData[1], RxData[0]);
}

bool upgradedata(struct bq27541_device_info *di, char *writedata, int sizeofdata)
{
    int parsepos = 0;
    int timeout = 3;

    while ((parsepos < sizeofdata) && (timeout > 0))
    {
        bool comparefail = false;
        u8 ldffsdata[98];
        int lineend = parsepos;
        while(lineend < sizeofdata)
        {
            if (writedata[lineend] == 0x0d)
            {
                lineend+=2;
                break;
            }
            lineend++;
        }

//        printk("start:%d    end:%d\n", parsepos, lineend);

        if (lineend > parsepos+4)
        {
            int x,y;
            switch(writedata[parsepos]) {
                case 0x57:
                    y = asciitohex(parsepos+1, lineend, ldffsdata, writedata);
                    if (y > 1)
                    {
#if 1
                        printk("W:");
                        for (x=0;x<y;x++) {
                           printk(" %02x", ldffsdata[x]);
                        }
                        printk("\n");
#endif
                        bq27541_write_block(di, ldffsdata+1,  ldffsdata[1], y-2, true);
                    }
                    break;
                case 0x52:
                    y =asciitohex(parsepos+1, lineend, ldffsdata, writedata);
                    printk("R:");
                    for (x=0;x<y;x++) {
                        printk(" %02x", ldffsdata[x]);
                    }
                    printk("\n");
                    break;
                case 0x43:
                    y = asciitohex(parsepos+1, lineend, ldffsdata, writedata);
                    if (y>1)
                    {
#if 1
                        printk("C:");
                        for (x=0;x<y;x++) {
                            printk(" %02x", ldffsdata[x]);
                        }
#endif
                        memset(RxData,0,sizeof(RxData));
                        bq27541_read_block(di, RxData, ldffsdata[1],  y-2, true);
#if 1
                        printk("           read:");
                        for (x=0;x<(y-2);x++) {
                            printk(" %02x", RxData[x]);
                        }
                        for (x=0;x<(y-2);x++) {
                            if (RxData[x] != ldffsdata[x+2]) {
                                comparefail = true;
                            }
                        }
                        printk("\n");
#endif
                    }
                    break;
                case 0x58:
                    printk("X: %d\n", asciitodec(parsepos+1, lineend, writedata));
                    msleep(asciitodec(parsepos+1, lineend, writedata));
                    break;
                default:
                    printk("error happen\n");
                    parsepos++;
            }
            
        }
        if (comparefail) {
            timeout--;
            parsepos = 0;
            printk("Compare Fail\n");
            msleep(100);
        } else {
            parsepos = lineend;
        }
    }

    if (timeout > 0) return true;
    else return false;
}

void romupdatetest(struct bq27541_device_info *di, bool forceupdate)
{
    unsigned int  fwversion;
    bool upgragefw = false;

    printk("***************************** Begin update bq27541 ***********************************\n");
    printk("sizeof[%d][%d]\n",sizeof(dffsdata),sizeof(bqfsdata));

    fwversion = readFWVersion(di);

    if (fwversion != BQFS_FW_VERSION) {
        printk("BQFS_FW_VERSION[%04x] [%04x] are not same, no upgrade any data. \n", BQFS_FW_VERSION, fwversion);
        upgragefw = true;
//        return;
    } else if (fwversion == DFFS_FW_VERSION) {
        printk("DFFS_FW_VERSION[%04x] [%04x] are same, check dffs \n", DFFS_FW_VERSION, fwversion);
        if ((readdffsversion(di) == DFFS_VERSION) && !forceupdate)
        {
            printk("Data Flash version are same. No need upgrade\n");
            return;
        } else {
            upgragefw = false;
        }
    } else {
        printk("Not find any upgrade data\n");
        return;
    }
    
//    scaletest(di);
//    msleep(1000);
    if (enterrommode(di) < 0)
    {
        printk("Enter ROM mode fail\n");
    }
    msleep(1000);
//    printk("size: %d\n", sizeof(dffsdata));

//**************************************************************
    if (upgragefw) {
        if (!(upgradedata(di, bqfsdata, sizeof(bqfsdata)))) {
            printk("Update fail\n");
            exitrommode(di);
        }
    } else {
        if (upgradedata(di, dffsdata, sizeof(dffsdata))) {
            if (WRITE_DFFS_VERSION) {
                writedffsversion(di);
            }
        } else {
            printk("Update fail\n");
            exitrommode(di);
        }
    }
    printk("***************************** End update bq27541 ***********************************\n");
}
#endif

unsigned int readBQ27541Temp(struct bq27541_device_info *di){
	
	memset(RxData,0,sizeof(RxData));
	
	// Read Temperature (units = 0.1K)
  	bq27541_read_block(di, RxData, BQ27541_CMD_TEMP,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}
unsigned int readBQ27541Volt(struct bq27541_device_info *di){
	memset(RxData,0,sizeof(RxData));

  	bq27541_read_block(di, RxData,BQ27541_CMD_VOLT,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}
unsigned int readBQ27541Curr(struct bq27541_device_info *di){
	memset(RxData,0,sizeof(RxData));

  	bq27541_read_block(di, RxData,BQ27541_CMD_AI,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}

unsigned int readBQ27541TTE(struct bq27541_device_info *di){
	memset(RxData,0,sizeof(RxData));

  	bq27541_read_block(di, RxData,BQ27541_CMD_TTE,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}

/*
unsigned int readBQ27541TTF(struct bq27541_device_info *di){
	memset(RxData,0,sizeof(RxData));

  	bq27541_read_block(di, RxData,BQ27541_CMD_TTF,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}
*/

/*
unsigned int readBQ27541SOH(struct bq27541_device_info *di){
	memset(RxData,0,sizeof(RxData));

  	bq27541_read_block(di, RxData,BQ27541_CMD_SOH,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}
*/

unsigned int readBQ27541SOC(struct bq27541_device_info *di){
	memset(RxData,0,sizeof(RxData));

  	bq27541_read_block(di, RxData,BQ27541_CMD_SOC,  2, false);
  	return transBytes2UnsignedInt(RxData[1], RxData[0]);
}

unsigned int readBQ27541Version(struct bq27541_device_info *di, bq_ver *ver){
    int ret;
    memset(ver,0,sizeof(bq_ver));

// FW version
    ret = bq27541_write_two_byte(di, CTRL_SUB_FW_REV, BQ27541_CMD_CTRL, false);
    if (ret != 0)
    {
        printk("Write CTRL_SUB_FW_REV fail\n");
        return 0;
    }
    msleep(20);

    memset(RxData,0,sizeof(RxData));
    ret = bq27541_read_block(di, RxData,BQ27541_CMD_CTRL,  2, false);
    if (ret != 0)
    {
        printk("Read BQ27541_CMD_CTRL fail\n");
        return 0;
    }    
    ver->Fw_version = transBytes2UnsignedInt(RxData[1], RxData[0]);

// Data Flash version
    ret = bq27541_write_two_byte(di, CTRL_SUB_DF_VER, BQ27541_CMD_CTRL, false);
    if (ret != 0)
    {
        printk("Write CTRL_SUB_DF_VER fail\n");
        return 0;
    }
    msleep(20);

    memset(RxData,0,sizeof(RxData));
    ret = bq27541_read_block(di, RxData,BQ27541_CMD_CTRL,  2, false);
    if (ret != 0)
    {
        printk("Read BQ27541_CMD_CTRL fail\n");
        return 0;
    }
    ver->Df_version = transBytes2UnsignedInt(RxData[0], RxData[1]);

// Control Status
    ret = bq27541_write_two_byte(di, CTRL_SUB_CONTROL_STATUS, BQ27541_CMD_CTRL, false);
    if (ret != 0)
    {
        printk("Write CTRL_SUB_CONTROL_STATUS fail\n");
        return 1;
    }
    msleep(20);

    memset(RxData,0,sizeof(RxData));
    ret = bq27541_read_block(di, RxData,BQ27541_CMD_CTRL,  2, false);
    if (ret != 0)
    {
        printk("Read BQ27541_CMD_CTRL fail\n");
    }
    else
    {
        ver->Control_status= transBytes2UnsignedInt(RxData[0], RxData[1]);
    }

    return 1;
}

/*
 * readbqdataram: Reads entirity of BQ gauge dataram into a bq_dataram structure (defined in bq.h)
 * Arguments:	bq_dataram *dr: pointer to structure
 * Returns: ioctl failure code on fail, otherwise 0
 */
int readbqdataram(struct bq27541_device_info *di, bq_dataram *dr)
{
	int ret;
	struct i2c_msg msg[2];
	//u8 buf;
	char cmd = 0x00;

	memset(dr,0,sizeof(bq_dataram));

	msg[0].addr	= di->client->addr;
	msg[0].flags	= 0;
	msg[0].buf	= &cmd;
	msg[0].len	= 1;

	msg[1].addr	= di->client->addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= (char*)dr;
	msg[1].len	= sizeof(bq_dataram);

	ret = i2c_transfer(di->client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages,ret = %d\n",ret);
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {

	}
	return ret;
}

/*************************************************/
static int bq27541_write_byte(struct bq27541_device_info *di, u8 value, u8 reg, bool is_rom_mode)
{
	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };

	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return bq27541_write_block(di, temp_buffer, reg, 1, is_rom_mode);
}

static int bq27541_write_two_byte(struct bq27541_device_info *di, u16 value, u8 reg, bool is_rom_mode)
{
//    int ret;
    u8 temp_buffer[3] = { 0 };

    temp_buffer[2] = value >> 8 & 0xff;
    temp_buffer[1] = value & 0xff;
    return bq27541_write_block(di, temp_buffer,  reg, 2, false);
}

static int bq27541_write_block(struct bq27541_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes, bool is_rom_mode)
{
	struct i2c_msg msg[1];
	int ret;

	*value		= reg;
	//printk("write block addr = %02x \n", di->client->addr);
	if (is_rom_mode) {
		msg[0].addr	= 0x0b;
	} else {
		msg[0].addr	= di->client->addr;
	}

	msg[0].flags	= 0;
	msg[0].buf	= value;
	msg[0].len	= num_bytes + 1;

	ret = i2c_transfer(di->client->adapter, msg, 1);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {

	}
	return 0;
}
static int bq27541_read_byte(struct bq27541_device_info *di, u8 *value, u8 reg)
{
	return bq27541_read_block(di, value, reg, 1, false);
}

static int bq27541_read_block(struct bq27541_device_info *di, u8 *value,
						u8 reg, unsigned num_bytes, bool is_rom_mode)
{
	struct i2c_msg msg[2];
	u8 buf;
	int ret;

	buf		= reg;

	//printk("read block addr = %02x \n", di->client->addr);
	if (is_rom_mode) {
		msg[0].addr	= 0x0b;
	} else {
		msg[0].addr	= di->client->addr;
	}
	msg[0].flags	= 0;
	msg[0].buf	= &buf;
	msg[0].len	= 1;

	if (is_rom_mode) {
		msg[1].addr	= 0x0b;
	} else {
		msg[1].addr	= di->client->addr;
	}
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= value;
	msg[1].len	= num_bytes;

	ret = i2c_transfer(di->client->adapter, msg, 2);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		dev_err(di->dev,
			"i2c_write failed to transfer all messages\n");
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {

	}
	return 0;
}

static int bq27541_charger_event(struct notifier_block *nb, unsigned long event,
				void *_data)
{
#ifdef CONFIG_TWL6032_BCI_BATTERY

	struct bq27541_device_info *di;
	int ret = NOTIFY_DONE;
	int cmd = (event & (~BCI_TYPE_MASK));

	if (!(event & BCI_EXTERNAL_FUELGAUGE))
		return ret;
	
	di = container_of (nb, struct bq27541_device_info, nb);
	switch (cmd)
	{
		case BCI_CONFIG:
		{
			struct bci_config_fuel_gauge *f = _data; //Laker: include twl6032_bci_battery.h?

			printk("%s: BCI_CONFIG - set termination voltage = %d\n", __FUNCTION__, f->battery_cutoff_voltage);
			ret = writeBQ27541TermVolt(di, f->battery_cutoff_voltage);

			break;
		}

		case BCI_FUELGAUGE_STATUS_REQUEST:
		{
			struct bci_fuel_gauge_status_request *fg_req = _data; //Laker: include twl6032_bci_battery.h?
			int cap;
			bq_dataram dr;
			if (!mNotupdatedata)
			{         
				if (readBQ27541Block(di, &dr) < 0) {
					printk("[GG][Error] bq27541 happen error\n");
					ret	= NOTIFY_BAD;
					break;
				}
				fg_req->data_valid = 1;
			}			
			cap = dr.StateOfChg;
			fg_req->capacity_changed = (fg_req->capacity != cap);

			// assign gg data
			fg_req->capacity = dr.StateOfChg;
			fg_req->current_uA = dr.AvgCurr * 1000;
			fg_req->voltage = dr.Voltage;
			fg_req->temperature = dr.Temp / 10;
			ret	= NOTIFY_OK;

			break;
		}

		default:
			break;
	}

	return ret;

#else /* !CONFIG_TWL6032_BCI_BATTERY, for CONFIG_TWL603x_BCI_BATTERY and others... */

	struct bq27541_device_info *di;
	int ret = 0;
	
	di = container_of(nb, struct bq27541_device_info, nb);
	switch(event){
		case BCI_EXT_FG_SET_TERM_VOLT:
		{
			unsigned int *val=_data;	
			printk("set terminate voltage = %d\n",*val);
			ret = writeBQ27541TermVolt(di, *val);
			
		}			
			break;
		case BCI_EXT_FG_SET_SOC1:
		{
			unsigned int *val=_data;				
			printk("set soc1:%d - enable bat low interrupt\n", *val);
			ret = enableBQ27541LowBatInt(di);		
			writeBQ27541SOC1(di, *val);
			
		}
			break;			
		case BCI_EXT_FG_SET_SYSDN_VOLT:
		{
			unsigned int *val=_data;	
			printk("set sysdn voltage = %d\n",*val);
			ret = writeBQ27541SysDnVoltThre(di, *val);			
		}
			break;
		case BCI_EXT_FG_CAP:
		{
			struct bq_read *status=_data;
			status->value= readBQ27541SOC(di);
		}
			break;
		case BCI_EXT_FG_VOLT:
		{
			struct bq_read *status=_data;
			status->value = readBQ27541Volt(di);
		}			
			break;
		case BCI_EXT_FG_CURR:
		{
			struct bq_read *status=_data;
			status->value = readBQ27541Curr(di);
		}			
			break;
		case BCI_EXT_FG_TEMP:
		{
			struct bq_read *status=_data;
			status->value = readBQ27541Temp(di);
		}				
			break;

		case BCI_EXT_FG_FIRMWARE_READ:
		{
			bq_ver* _ver;
			_ver = _data;
			readBQ27541Version(di, _ver);
		}				
			break;
		case BCI_EXT_FG_FIRMWARE_WRITE:
		{
			long *val=_data;
			if (*val == 1)
			{
				if (!wake_lock_active(&wakelock))
					wake_lock(&wakelock);

				mNotupdatedata = true;
// jss				romupdatetest(di, true);
				mNotupdatedata = false;

				if (wake_lock_active(&wakelock))
					wake_unlock(&wakelock);
			}
			if (*val == 2)
			{
//				ret = bq27541_write_two_byte(di, CTRL_SUB_FACTORY_RESTORE, BQ27541_CMD_CTRL, false);
				if (ret != 0)
				{
					printk("Write CTRL_SUB_FACTORY_RESTORE fail\n");
				}
			}
			if (*val == 3)
			{
				ret = bq27541_write_two_byte(di, CTRL_SUB_RESET, BQ27541_CMD_CTRL, false);
				if (ret != 0)
				{
					printk("Write CTRL_SUB_RESET fail\n");
				}
			}
		}				
			break;
		case BCI_EXT_FG_BLOCK:
		{
			bq_dataram* _dr;
			_dr = _data;

			if (!mNotupdatedata)
			{         
				readBQ27541Block(di, _dr);
			}
		}
			break;
		default:
			break;
	}

	return ret;

#endif /* CONFIG_TWL6032_BCI_BATTERY */
}

static int __devinit bq27541_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27541_device_info *di;
	bq_ver bqver;
    unsigned int  devicetype;

	int ret;
	u8 read_reg = 0;  
	printk("-------======= %s() =======------- \n", __func__);

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	memset(di,0,sizeof(*di));

	di->dev = &client->dev;
	di->client = client;

	i2c_set_clientdata(client, di);

	wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "bq27541_wakelock");

devicetype = readDeviceType(di);

    printk("device type is [%04x]    DFVer[%04x]\n", readDeviceType(di), readDFVersion(di));

    scaletest(di);
// jss	romupdatetest(di, false);
// jss	unlockbq(di);

	ret = readBQ27541Version(di, &bqver);
	if (ret == 0) {
		dev_err(&client->dev, "chip not present at address %x\n", client->addr);
		ret = -EINVAL;
		goto err_kfree;
	}
	printk("read_reg = 0x%04x\n", bqver.Fw_version);
	if (client->addr == 0x55)
		di->bqchip_version = REV_BQ27541;

	printk("bq27541 data flash version = 0x%04x\n", bqver.Df_version);

	if (di->bqchip_version == 0) {
		dev_dbg(&client->dev, "unknown bq chip\n");
		dev_dbg(&client->dev, "Chip address %x", client->addr);
		dev_dbg(&client->dev, "bq chip version reg value %x", read_reg);
		ret = -EINVAL;
		goto err_kfree;
	}

	di->nb.notifier_call = bq27541_charger_event;
	di->active = 1;
	di->debug = 0;
	di->pre_soc = 0;
	
	if (ret == 0)
		dev_dbg(&client->dev, "could not create sysfs files\n");

#if defined (CONFIG_TWL6032_BCI_BATTERY)
	twl6030_bci_function_register(BQ27541, &di->nb,
		BCI_CONFIG | BCI_FUELGAUGE_STATUS_REQUEST,
		BCI_EXTERNAL_FUELGAUGE,
		di);
#elif defined (CONFIG_TWL603x_BCI_BATTERY)
	twl6030_register_bci_notifier(&di->nb, 0x2FFF, NOTIFIER_REG_EXT_FUELGAUGE);
#else
	twl6030_register_notifier(&di->nb, 0x2FFF);
#endif

	printk("-------======= bq27541 charger register successfully !! =======------- \n");
	return 0;

err_kfree:
	wake_lock_destroy(&wakelock);
	kfree(di);

	return ret;
}
static int __devexit bq27541_charger_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

#if defined (CONFIG_TWL6032_BCI_BATTERY)
	twl6030_bci_function_unregister(BQ27541);
#elif defined (CONFIG_TWL603x_BCI_BATTERY)
	twl6030_unregister_bci_notifier(&di->nb, 1, NOTIFIER_REG_EXT_FUELGAUGE);
#else
	twl6030_unregister_notifier(&di->nb, 1);
#endif

	wake_lock_destroy(&wakelock);

	kfree(di);

	return 0;
}
static void  bq27541_charger_shutdown(struct i2c_client *client) {
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	printk("bq27541_charger_shutdown()\n");

#if defined (CONFIG_TWL6032_BCI_BATTERY)
	twl6030_bci_function_unregister(BQ27541);
#elif defined (CONFIG_TWL603x_BCI_BATTERY)
	twl6030_unregister_bci_notifier(&di->nb, 1, NOTIFIER_REG_EXT_FUELGAUGE);
	twl6030_bci_notifier_shutdown();
#else
	twl6030_unregister_notifier(&di->nb, 1);
#endif
	wake_lock_destroy(&wakelock);
	kfree(di);	
}

static const struct i2c_device_id bq27541_id[] = {
	{ BQ27541, 0 },
	{},
};


static struct i2c_driver bq27541_charger_driver = {
	.probe		= bq27541_charger_probe,
	.remove		= __devexit_p(bq27541_charger_remove),
	
	.shutdown =	bq27541_charger_shutdown,	
	
	.id_table	= bq27541_id,
	.driver		= {
		.name	= BQ27541,
	},
};

static int __init bq27541_charger_init(void)
{
	return i2c_add_driver(&bq27541_charger_driver);
}
module_init(bq27541_charger_init);
  
static void __exit bq27541_charger_exit(void)
{
	i2c_del_driver(&bq27541_charger_driver);
}
module_exit(bq27541_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
