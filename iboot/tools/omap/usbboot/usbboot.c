/*
 * Copyright (C) 2010 The Android Open Source Project
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <arpa/inet.h>
#include "usb.h"
#include "usbboot.h"

static struct chip_info chip;
static char *secondstage_filename;
static char *image_filename;

static char *usb_boot_read_chip_info(usb_handle *usb)
{
	static char proc_type[8];
	uint32_t msg_getid = 0xF0030003;
	uint8_t id[81];
	uint8_t *crc1;
	uint8_t gp_device_crc1[4] = {0, 0, 0, 0};
	int i;

	memset(id, 0xee, 81);
	fprintf(stderr,"reading ASIC ID\n");
	usb_write(usb, &msg_getid, sizeof(msg_getid));
	usb_read(usb, id, sizeof(id));

	memcpy(&chip.chip, &id[OFF_CHIP+0], 2);
	chip.chip = ntohs(chip.chip);
	chip.rom_rev = id[OFF_ROM_REV];
	memcpy(chip.IDEN, &id[OFF_ID], 20);
	memcpy(chip.MPKH, &id[OFF_MPKH], 32);
	chip.crc0 = ntohl(*(uint32_t *)&id[73]);
	chip.crc1 = ntohl(*(uint32_t *)&id[77]);

	fprintf(stderr,"CHIP: %02x%02x\n", id[OFF_CHIP+0], id[OFF_CHIP+1]);
	fprintf(stderr, "rom minor version: %02X\n", id[OFF_ROM_REV]);
	fprintf(stderr,"IDEN: ");
	for (i = 0; i < 20; i++)
		fprintf(stderr,"%02x", id[OFF_ID+i]);
	fprintf(stderr,"\nMPKH: ");
	for (i = 0; i < 32; i++)
		fprintf(stderr,"%02x", id[OFF_MPKH+i]);
	fprintf(stderr,"\nCRC0: %02x%02x%02x%02x\n",
		id[73], id[74], id[75], id[76]);
	fprintf(stderr,"CRC1: %02x%02x%02x%02x\n",
		id[77], id[78], id[79], id[80]);

	crc1 = &id[77];
	if (memcmp(crc1, &gp_device_crc1, 4 * sizeof(uint8_t))) {
		fprintf(stderr, "device is ED/HD (EMU/HS)\n");
		strcpy(proc_type, "EMU");
	} else {
		fprintf(stderr, "device is GP\n");
		strcpy(proc_type, "GP");
	}

	fprintf(stderr, "\n");

	strcpy(chip.proc_type, proc_type);
	return proc_type;
}

static int usb_boot(usb_handle *usb, unsigned boot_param,
	     void *data, unsigned sz,
	     void *data2, unsigned sz2)
{
	int ret;
	uint32_t msg_boot = 0xF0030002;
	uint32_t msg_size = sz;
	uint32_t param = 0;

	fprintf(stderr, "sending %s to target...\n"
				"size (%d-B/%d-KB)\n\n", secondstage_filename,
				msg_size, msg_size/1024);
	ret = usb_write(usb, &msg_boot, sizeof(msg_boot));
	if (ret != sizeof(msg_boot))
		fprintf(stderr, "usb_write(OMAPBOOT) err %d\n", ret);
	ret = usb_write(usb, &msg_size, sizeof(msg_size));
	if (ret != sizeof(msg_size))
		fprintf(stderr, "usb_write(2ndstage data size) err %d\n", ret);
	ret = usb_write(usb, data, sz);
	if (ret != (int)sz)
		fprintf(stderr, "usb_write(2ndstage data) err %d\n", ret);

	if (data2) {
		fprintf(stderr,"\nwaiting for 2ndstage response...\n\n");
		ret = usb_read(usb, &msg_size, sizeof(msg_size));
		if (ret != sizeof(msg_size))
			fprintf(stderr, "usb_read(2ndstage MAGIC) err %d\n", ret);
		if (msg_size != 0xaabbccdd) {
			fprintf(stderr, "unexpected 2ndstage response (0x%08x)\n", msg_size);
			return -1;
		}
#ifdef TRACE_USB
		fprintf(stderr, "received 2ndstage first response: 0x%08x\n", msg_size);
#endif

		param = boot_param;
		ret = usb_write(usb, &param, sizeof(param));
		if (ret != sizeof(param))
			fprintf(stderr, "usb_write(parameter) err %d\n", ret);

		msg_size = sz2;
		ret = usb_write(usb, &msg_size, sizeof(msg_size));
		if (ret != sizeof(msg_size))
			fprintf(stderr, "usb_write(image data size) err %d\n", ret);
		usb_read(usb, &param, sizeof(param));
		if (ret != sizeof(param))
			fprintf(stderr, "usb_read(2ndstage MAGIC) err %d\n", ret);
		if (param != 0xaabbccdd) {
			fprintf(stderr, "unexpected image size response (0x%08x)\n", param);
			return -1;
		}
#ifdef TRACE_USB
		fprintf(stderr, "received image size response: 0x%08x\n", param);
#endif

		fprintf(stderr, "sending '%s' image to target...\n"
				"size (%d-B/%d-KB)\n\n", image_filename,
				msg_size, msg_size/1024);

		ret = usb_write(usb, data2, sz2);
		if (ret != (int)sz2)
			fprintf(stderr, "usb_write(image data) err %d\n", ret);
	}
	
	return 0;
}

static int match_omap_bootloader(usb_ifc_info *ifc)
{
	if (ifc->dev_vendor != 0x0451)
		return -1;
	if ((ifc->dev_product != 0xd010) && (ifc->dev_product != 0xd00f) &&
		(ifc->dev_product != 0xd011) &&  (ifc->dev_product != 0xd012) &&
		(ifc->dev_product != 0xd013))
		return -1;
	return 0;
}

static void *load_file(const char *file, unsigned *sz)
{
	void *data;
	struct stat s;
	int fd;
	
	fd = open(file, O_RDONLY);
	if (fd < 0)
		return 0;
	
	if (fstat(fd, &s))
		goto fail;
	
	data = malloc(s.st_size);
	if (!data)
		goto fail;
	
	if (read(fd, data, s.st_size) != s.st_size) {
		free(data);
		goto fail;
	}
	
	close(fd);
	*sz = s.st_size;
	return data;
	
fail:
	fprintf(stderr, "failed loading file\n");
	close(fd);
	return 0;
}

static int usage(void)
{
	fprintf(stderr, "\nusbboot syntax and options:\n\n");
	fprintf(stderr, "usbboot <2ndstage> <image>\n\n");
	fprintf(stderr, "------------------------------------------------------\n");
	fprintf(stderr, "example: ./usbboot iboot_MLO.bin iboot.img\n");
	fprintf(stderr, "         ./usbboot spl/u-boot-spl.bin iboot.img\n\n");

	return 0;
}

int main(int argc, char **argv)
{
	void *data = NULL, *data2 = NULL;
	unsigned sz, sz2;
	usb_handle *usb = NULL;
	int once = 1;

	if (argc != 3) {
		usage();
		return 0;
	} else {
		secondstage_filename = argv[1];
		image_filename = argv[2];

		data = load_file(argv[1], &sz);
		if (data == 0) {
			fprintf(stderr, "cannot load '%s'\n", argv[1]);
			usage();
			return -1;
		}
		argc--;
		argv++;
	}

	data2 = load_file(argv[1], &sz2);
	if (data2 == 0) {
		fprintf(stderr, "cannot load '%s'\n", argv[1]);
		usage();

		/* free up memory */
		if (data)
			free(data);

		return -1;
	}

	for (;;) {
		if (usb == NULL)
			usb = usb_open(match_omap_bootloader);

		if (usb) {
			usb_boot_read_chip_info(usb);

			return usb_boot(usb, 0, data, sz, data2, sz2);
		}

		if (once) {
			once = 0;
			fprintf(stderr, "\nwaiting for device...\n\n");
		}
		usleep(250);
	}
	
	return -1;    
}
