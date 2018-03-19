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

#include <common.h>
#include <asm/omap_common.h>
#include <asm/arch/omap_rom.h>

static unsigned MSG = 0xaabbccdd;
static struct usb usb;

void spl_usb_load_image(void)
{
	unsigned len = 0, param = 0;
	int ret;
	struct image_header *header;
	const unsigned char *data;

	puts("## Loading from USB...\n");

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
			sizeof(struct image_header));

	spl_enable_irqs();

	ret = usb_open(&usb);
	if (ret) {
		printf("open err %d\n", ret);
		goto _disable_irqs_hang;
	}

	usb_queue_read(&usb, &param, 4);

	ret = usb_write(&usb, &MSG, 4);
	if (ret) {
		printf("write1 err %d\n", ret);
		goto _close_usb_hang;
	}

	ret = usb_wait_read(&usb);
	if (ret) {
		printf("read err %d\n", ret);
		goto _close_usb_hang;
	}

	ret = usb_read(&usb, &len, 4);
	if (ret) {
		printf("read size err %d\n", ret);
		goto _close_usb_hang;
	}

	ret = usb_write(&usb, &MSG, 4);
	if (ret) {
		printf("write2 err %d\n", ret);
		goto _close_usb_hang;
	}

	ret = usb_read(&usb, (void*)header, len);
	if (ret) {
		printf("read data err %d\n", ret);
		goto _close_usb_hang;
	}

	usb_close(&usb);

	spl_disable_irqs();

	if (param)
		printf("Got parameter 0x%x\n", param);
	printf("Loaded %u bytes\n", len);

	if (spl_parse_image_header(header))
		hang();

	header = (struct image_header *)spl_image.load_addr;
	data = ((const unsigned char *)header) + sizeof(image_header_t);
	len  = spl_image.size - sizeof(image_header_t);

	if (spl_verify_image_data(header, data, len))
		hang();

	return;

_close_usb_hang:
	usb_close(&usb);
_disable_irqs_hang:
	spl_disable_irqs();
	hang();
}
