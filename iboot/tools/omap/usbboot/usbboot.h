/*
* Copyright (C) 2012 Texas Instruments, Inc.
* All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _USBBOOT_H_
#define _USBBOOT_H_

#define USE_TOC 0
#define USER_RQ_MASK	0x00FF0000

#define OFF_CHIP	0x04
#define OFF_ROM_REV	0x07
#define OFF_ID		0x0F
#define OFF_MPKH	0x26

extern unsigned char aboot_data[];
extern unsigned aboot_size;

extern unsigned char iboot_gp_data[];
extern unsigned iboot_gp_size;

#ifdef EMBED_IBOOT_HS
#if defined(CONFIG_IS_OMAP4)
extern unsigned char iboot_hs_4430_ES2_data[];
extern unsigned iboot_hs_4430_ES2_size;
extern unsigned char iboot_hs_4460_ES1_data[];
extern unsigned iboot_hs_4460_ES1_size;
extern unsigned char iboot_hs_4470_ES1_data[];
extern unsigned iboot_hs_4470_ES1_size;
#elif defined(CONFIG_IS_OMAP5)
extern unsigned char iboot_hs_5430_ES1_data[];
extern unsigned iboot_hs_5430_ES1_size;
extern unsigned char iboot_hs_5430_ES2_data[];
extern unsigned iboot_hs_5430_ES2_size;
#endif
#endif

extern unsigned char sboot_data[];
extern unsigned sboot_size;

typedef struct tocentry {
	unsigned offset;
	unsigned length;
	unsigned flags;
	unsigned align;
	unsigned spare;
	char name[12];
} tocentry;

/*
 * See TRM for OMAP4, OMAP5 ASIC ID Structure
 */
struct chip_info {
	uint16_t chip;
	char rom_rev;
	char IDEN[20];
	char MPKH[32];
	uint32_t crc0;
	uint32_t crc1;
	char proc_type[8];
};

#endif
