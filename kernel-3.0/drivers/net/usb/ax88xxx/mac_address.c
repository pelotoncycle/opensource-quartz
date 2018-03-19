/*
 *
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
 * Author: Neko Chang <neko.chang@innocomm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/etherdevice.h>
#include <linux/android_boot.h>

const u8 DEFAULT_NODE_ID[ETH_ALEN] = {0x00, 0x0E, 0xC6, 0x87, 0x72, 0x01};	// Reference section 4.1.3 Node ID (04~06h), page 28, AX88772B_USB 2.0 to 10&100M Ethernet Controller_ASIX.pdf 08/10/2011
const u8 NUM_MAC_EXTERNAL_PARAM_LEN = 12;
const u8 NUM_CHAR_BIT_LEN = 4;

static u8 MAC_DERIVES_FROM_BOOTLOADER = 0;


static int charToByte(char c)
{
       if (c >= '0' && c <= '9') return (c - '0');
       if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
       if (c >= 'a' && c <= 'f') return (c - 'a' + 10);
       return -1;
}


/**
  * mac_address_provider - Provide Ethernet address(MAC).
  * @mac_buf: Pointer to a six-byte array containing the Ethernet address.
  *           (External mounted EEPROM or on-chip SROM MAC address)
  *
  * 1.Inspect Ethernet address (MAC) from Bootloader, if OK, provide it to caller.
  * 2.Check caller provided @mac_buf, If it is default node ID, reject it.
  * 3.If all of provided MAC fail, provide random number.
  *
  * Reference: Section 4.1.3 Node ID (04~06h), page 28, AX88772B_USB 2.0 to 10&100M Ethernet Controller_ASIX.pdf 08/10/2011
  */
void mac_address_provider(void *mac_buf)
{
	char *MSG, tmpbuf[ETH_ALEN];
	unsigned int i, result;
	const char *mac_ext_src = NULL;
	
	MAC_DERIVES_FROM_BOOTLOADER = 0;
	
	do {
		// 1st priority: Bootloader
		do {
			mac_ext_src = android_boot_get_ethaddr();

			// Slient quit and enter next priority if bootloader doesn't provide MAC address.
			if (!mac_ext_src || (0 == memcmp(mac_ext_src, "\0", 1)))
				break;

			if (NUM_MAC_EXTERNAL_PARAM_LEN != strlen(mac_ext_src)) {
				printk(KERN_WARNING "WARNING: Bootloader provided MAC address=%s, length dismatch. Require=%d, Actual=%d, discard it.\n", mac_ext_src, NUM_MAC_EXTERNAL_PARAM_LEN, strlen(mac_ext_src));
				break;
			}
	
			for (i = 0; NUM_MAC_EXTERNAL_PARAM_LEN > i; i++) {
				// Inspect/Convert buffer content.
				if (-1 == (result = charToByte(mac_ext_src[i])))
					break;

				// Combind buffer content to temp buffer.   i=0~NUM_MAC_EXTERNAL_PARAM_LEN-1 -> tmpbuf=0~ETH_ALEN-1
				if ((i & 1) == 0)	// Even
					tmpbuf[i >> 1] = result;
				else	//Odd
					tmpbuf[i >> 1] = (tmpbuf[i >> 1] << NUM_CHAR_BIT_LEN) + result;
			}

			// Any one of buffer content corrupt, fail.
			if (-1 == result)
				printk(KERN_WARNING "WARNING: Inspect bootloader provided MAC address=%s, detect first illegal character=0x%02x at position=%d, discard it!", mac_ext_src, mac_ext_src[i], i + 1);
			else if (!is_valid_ether_addr(tmpbuf))
				printk(KERN_WARNING "WARNING: Bootloader provided MAC address=%pM is invalid, discard it.\n", tmpbuf);
			else {	// All of inspect successful.
				MAC_DERIVES_FROM_BOOTLOADER = 1;
				memcpy(mac_buf, tmpbuf, ETH_ALEN);
				MSG = "Bootloader";
			}
		}  while(0);

		if (MAC_DERIVES_FROM_BOOTLOADER)
			break;

		// 2nd priority: External mounted EEPROM.
		// Note: On-chip SROM default MAC is invalid, because ALL OF CHIP SAME.
		if (!compare_ether_addr(mac_buf, DEFAULT_NODE_ID))
			printk(KERN_DEBUG "DEBUG: Ethernet external mounted EEPROM doesn't exist and can't allow on-chip SROM default MAC address also, discard it.\n");
		else if (!is_valid_ether_addr(mac_buf))
			printk(KERN_WARNING "WARNING: Ethernet external mounted EEPROM exist, but MAC address content EMPTY or INVALID , discard it.\n");
		else {	// All of inspect successful.
			MSG = "external mounted EEPROM";
			break;
		}

		// 3rd priority: Random number.
		random_ether_addr(mac_buf);
		MSG = "random number";
	} while(0);
	
	printk(KERN_INFO "INFO: Ethernet working MAC address=%pM and derive from %s.\n", mac_buf, MSG);
}


/**
  * is_MAC_derives_from_bootloader - Tell using Ethernet address from Bootloader ir not.
  *
  * Return true if using Ethernet address from Bootloader.
  */
int is_MAC_derives_from_bootloader(void)
{
	return MAC_DERIVES_FROM_BOOTLOADER;
}
