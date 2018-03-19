/*
 * (C) Copyright 2011-2014
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <exports.h>
#include <command.h>
#include <malloc.h>
#include <errno.h>
#include <version.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <asm/byteorder.h>
#include <mmc.h>
#include <linux/usb/ch9.h>
#include <usbdescriptors.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/compiler.h>
#include <watchdog.h>

#include <video.h>

#include <asm/gpio.h>
#include <asm/arch/sys_proto.h>
#include <asm/sizes.h>

/* U-Boot image format */
#include <image.h>

#include <icom/fastboot.h>
#include <icom/aboot.h>
/* Android mkbootimg format */
#include <icom/bootimg.h>
/* InnoComm mkmultimg format */
#include <icom/multimg.h>
/* Sparse EXT4 format */
#include <icom/sparse_format.h>
#include <icom/power_supply.h>

#if defined(CONFIG_USB_GADGET_FASTBOOT)

/*-------------------------------------------------------------------------*/

#include <icom/sha.h>
#include <icom/rsa.h>

static SHA_CTX sha_ctx;

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_CFB_CONSOLE)
#define VIDEO_CLEAR()	video_clear()
#else
#define VIDEO_CLEAR()	do{}while(0)
#endif /* CONFIG_CFB_CONSOLE */

#if defined(CONFIG_CFB_CONSOLE) && defined(CONFIG_VIDEO_NOT_CONSOLE_DEVICE) \
		&& defined(CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG) && (CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG > 0)
#define xvideo_printf(level, format, args...) do { \
	if (CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG >= (level)) { \
		video_printf(format, ## args); \
	} } while (0)
#define xvideo_puts(level, format) do { \
	if (CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG >= (level)) { \
		video_puts(format); \
	} } while (0)
#define VIDEO_PRINT(level, fmt, args...) \
	xvideo_printf(level, fmt, ## args)
#define VIDEO_PUTS(level, fmt) \
	xvideo_puts(level, fmt)
#else
#define VIDEO_PRINT(level, fmt, args...)	do{}while(0)
#define VIDEO_PUTS(level, fmt)	do{}while(0)
#endif /* CONFIG_CFB_CONSOLE && CONFIG_VIDEO_NOT_CONSOLE_DEVICE && CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG */

/*-------------------------------------------------------------------------*/

/*#define CONFIG_FASTBOOT_DEBUG*/
/*#define CONFIG_FASTBOOT_VERBOSE_DEBUG*/

#ifdef DEBUG
#ifndef CONFIG_FASTBOOT_DEBUG
#define CONFIG_FASTBOOT_DEBUG
#endif /* CONFIG_FASTBOOT_DEBUG */
#ifndef CONFIG_FASTBOOT_VERBOSE_DEBUG
#define CONFIG_FASTBOOT_VERBOSE_DEBUG
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
#endif

#ifdef CONFIG_FASTBOOT_DEBUG
#define FASTBOOT_DPRINT(fmt, args...) \
	do {printf("[fboot] " fmt, ##args);} while (0)
#define FASTBOOT_DPUTS(fmt) \
	do {puts("[fboot] " fmt);} while (0)
#else /* !CONFIG_FASTBOOT_DEBUG */
#define FASTBOOT_DPRINT(fmt, args...) \
	do {} while (0)
#define FASTBOOT_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_FASTBOOT_DEBUG */

#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
#define FASTBOOT_VPRINT(fmt, args...) \
	do {printf("[fboot] " fmt, ##args);} while (0)
#define FASTBOOT_VPUTS(fmt) \
	do {puts("[fboot] " fmt);} while (0)
#else /* !CONFIG_FASTBOOT_VERBOSE_DEBUG */
#define FASTBOOT_VPRINT(fmt, args...) \
	do {} while (0)
#define FASTBOOT_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */

#define FASTBOOT_PRINT(fmt, args...) \
	do {printf("fboot: " fmt, ##args); VIDEO_PRINT(10, "Fastboot: " fmt, ##args);} while (0)
#define FASTBOOT_PUTS(fmt) \
	do {puts("fboot: " fmt); VIDEO_PUTS(10, "Fastboot: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define PUTC(fmt) \
	do {putc(fmt);} while (0)
#define ERROR(fmt) \
	do {puts(fmt); VIDEO_PUTS(5, fmt);} while (0)
#define FASTBOOT_ERR(fmt, args...) \
	do {printf("fboot: " fmt, ##args); VIDEO_PRINT(5, fmt, ##args);} while (0)

/*-------------------------------------------------------------------------*/

#define EP0_BUFFER_SIZE					512
#define EP_BUFFER_SIZE					4096
#define FASTBOOT_INTERFACE_CLASS		0xff
#define FASTBOOT_INTERFACE_SUB_CLASS	0x42
#define FASTBOOT_INTERFACE_PROTOCOL		0x03
#define FASTBOOT_VERSION				"0.5"

/*-------------------------------------------------------------------------*/

#define CONFIGURATION_NORMAL	1
#define BULK_ENDPOINT			1
#define RX_ENDPOINT_MAXIMUM_PACKET_SIZE_2_0		(0x0200)
#define RX_ENDPOINT_MAXIMUM_PACKET_SIZE_1_1		(0x0040)
#define TX_ENDPOINT_MAXIMUM_PACKET_SIZE_2_0		(0x0200)
#define TX_ENDPOINT_MAXIMUM_PACKET_SIZE_1_1		(0x0040)

/*-------------------------------------------------------------------------*/

#define FASTBOOT_MAX_WORKS		20

struct fastboot_work {
	unsigned		in_used;

	struct usb_ep	*ep;
	void			*data;

	fb_work_func_t	func;

	struct list_head	list;
};

/*-------------------------------------------------------------------------*/

DECLARE_GLOBAL_DATA_PTR;

/*-------------------------------------------------------------------------*/

static int fastboot_exit_reason = 0;

static unsigned long download_addr = CONFIG_FASTBOOT_BUFFER;
static unsigned int download_size = 0;
static int download_has_error;

static boot_img_hdr aboot_header;

/*-------------------------------------------------------------------------*/

static struct list_head		fb_work_list;
static struct fastboot_work fb_works[FASTBOOT_MAX_WORKS];

/*-------------------------------------------------------------------------*/

static u8 *ep0_buffer, *rx_buffer, *tx_buffer;

static unsigned long rx_addr;
static unsigned int rx_length;
static unsigned int rx_bytes;

static int current_config;

static struct usb_gadget *g;
static struct usb_request *ep0_req;

struct usb_ep *ep_out, *ep_in;
struct usb_request *req_rx, *req_tx;

/*-------------------------------------------------------------------------*/

static void rx_cmd_complete(struct usb_ep *ep, struct usb_request *req);
static void rx_data_complete(struct usb_ep *ep, struct usb_request *req);

/*-------------------------------------------------------------------------*/

/* e1 */
static struct usb_endpoint_descriptor fs_ep_in = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN, /* IN */
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= TX_ENDPOINT_MAXIMUM_PACKET_SIZE_1_1,
	.bInterval			= 0x00,
};

static struct usb_endpoint_descriptor hs_ep_in = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN, /* IN */
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= TX_ENDPOINT_MAXIMUM_PACKET_SIZE_2_0,
	.bInterval			= 0x00,
};

/* e2 */
static struct usb_endpoint_descriptor fs_ep_out = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT, /* OUT */
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= RX_ENDPOINT_MAXIMUM_PACKET_SIZE_1_1,
	.bInterval			= 0x00,
};

static struct usb_endpoint_descriptor hs_ep_out = {
	.bLength			= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT, /* OUT */
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= RX_ENDPOINT_MAXIMUM_PACKET_SIZE_2_0,
	.bInterval			= 0x00,
};

/*-------------------------------------------------------------------------*/

/* This is the TI USB vendor id a product ID from TI's internal tree */
#define DEVICE_VENDOR_ID	0x0451
#define DEVICE_PRODUCT_ID	0xd022
#define DEVICE_BCD			0x0100

static struct usb_device_descriptor fb_descriptor = {
	.bLength			= sizeof(fb_descriptor),
	.bDescriptorType	= USB_DT_DEVICE,
	.bcdUSB				= 0x200,
	.bMaxPacketSize0	= 0x40,
	.idVendor			= DEVICE_VENDOR_ID,
	.idProduct			= DEVICE_PRODUCT_ID,
	.bcdDevice			= DEVICE_BCD,
	.iManufacturer		= FASTBOOT_STR_MANUFACTURER_IDX,
	.iProduct			= FASTBOOT_STR_PRODUCT_IDX,
	.iSerialNumber		= FASTBOOT_STR_SERIAL_IDX,
	.bNumConfigurations	= 1,
};

#define TOT_CFG_DESC_LEN	(USB_DT_CONFIG_SIZE + USB_DT_INTERFACE_SIZE + \
		USB_DT_ENDPOINT_SIZE + USB_DT_ENDPOINT_SIZE)

static struct usb_config_descriptor config_desc = {
	.bLength		= USB_DT_CONFIG_SIZE,
	.bDescriptorType	= USB_DT_CONFIG,
	.wTotalLength		= cpu_to_le16(TOT_CFG_DESC_LEN),
	.bNumInterfaces		= 1,
	.bConfigurationValue	= CONFIGURATION_NORMAL,
	.iConfiguration		= FASTBOOT_STR_CONFIG_IDX,
	.bmAttributes		= 0xc0,
	.bMaxPower	= 0xFA, /* 250x2 = 500mA */
};

static struct usb_interface_descriptor interface_desc = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= 0x00,
	.bAlternateSetting	= 0x00,
	.bNumEndpoints		= 0x02,
	.bInterfaceClass	= FASTBOOT_INTERFACE_CLASS,
	.bInterfaceSubClass	= FASTBOOT_INTERFACE_SUB_CLASS,
	.bInterfaceProtocol	= FASTBOOT_INTERFACE_PROTOCOL,
	.iInterface		= FASTBOOT_STR_INTERFACE_IDX,
};

static struct usb_qualifier_descriptor qual_desc = {
	.bLength		= sizeof(qual_desc),
	.bDescriptorType	= USB_DT_DEVICE_QUALIFIER,
	.bcdUSB			= 0x200,
	.bMaxPacketSize0	= 0x40,
	.bNumConfigurations	= 1,
};

/*-------------------------------------------------------------------------*/

static struct usb_string device_usb_fb_strings[FASTBOOT_STR_MAX_IDX+1] = {
	[0]								= { 0, "" },
	[FASTBOOT_STR_PRODUCT_IDX]		= { FASTBOOT_STR_PRODUCT_IDX, "PROD." },
	[FASTBOOT_STR_SERIAL_IDX]		= { FASTBOOT_STR_SERIAL_IDX, "1234" },
	[FASTBOOT_STR_CONFIG_IDX]		= { FASTBOOT_STR_CONFIG_IDX, "Android Fastboot" },
	[FASTBOOT_STR_INTERFACE_IDX]	= { FASTBOOT_STR_INTERFACE_IDX, "Android Fastboot" },
	[FASTBOOT_STR_MANUFACTURER_IDX]	= { FASTBOOT_STR_MANUFACTURER_IDX, "MANU." },
	[FASTBOOT_STR_PROC_REV_IDX]		= { FASTBOOT_STR_PROC_REV_IDX, "1.0" },
	[FASTBOOT_STR_PROC_TYPE_IDX]	= { FASTBOOT_STR_PROC_TYPE_IDX, "GP" },
	[FASTBOOT_STR_PROC_VERSION_IDX]	= { FASTBOOT_STR_PROC_VERSION_IDX, "OMAP" },
	[FASTBOOT_STR_MAX_IDX]			= { FASTBOOT_STR_MAX_IDX, NULL }
};

static struct usb_gadget_strings fb_strings = {
	.language	= 0x0409, /* en-us */
	.strings	= device_usb_fb_strings,
};

/*-------------------------------------------------------------------------*/

int fastboot_queue_work(fb_work_func_t func, struct usb_ep *ep, void *data)
{
	struct fastboot_work *fb_work = fb_works;
	int i;

	for (i = 0 ; i < FASTBOOT_MAX_WORKS; i++) {
		if (!fb_work->in_used)
			goto _found;
		fb_work++;
	}

	return -EMFILE;

_found:
	memset(fb_work, 0x0, sizeof(*fb_work));

	fb_work->in_used = 1;
	fb_work->ep = ep;
	fb_work->data = data;
	fb_work->func = func;

	/* queue work to the list */
	list_add_tail(&fb_work->list, &fb_work_list);

	return 0;
}

static void fastboot_run_works(void)
{
	struct fastboot_work *fb_work;

	while (!list_empty(&fb_work_list)) {
		fb_work = list_first_entry(&fb_work_list, struct fastboot_work, list);

		if (fb_work->func)
			fb_work->func(fb_work->ep, fb_work->data);

		fb_work->in_used = 0;

		list_del(&fb_work->list);
	}
}

/*-------------------------------------------------------------------------*/

static void fastboot_ep_set_halt(struct usb_ep *ep, void *data)
{
	if (ep)
		usb_ep_set_halt(ep);

	VIDEO_PUTS(0, "Fastboot: error occurred\n");
	VIDEO_PUTS(0, "Fastboot: remove USB cable and re-insert it\n");
}

#if 0
static void fastboot_ep_clear_halt(struct usb_ep *ep, void *data)
{
	if (ep)
		usb_ep_clear_halt(ep);
}
#endif

/*-------------------------------------------------------------------------*/

static void tx_status_completed(struct usb_ep *ep, struct usb_request *req)
{
	if (req_tx && req->status)
		FASTBOOT_DPRINT("tx status: %d trans: %u\n",
				req->status, req->actual);
}

static int tx_status_complete(const char *status,
		void (*complete)(struct usb_ep *ep, struct usb_request *req))
{
	int rc;
	struct usb_request *req = req_tx;
	int len = strlen(status);

	if (!req)
		return -EPERM;

	FASTBOOT_DPRINT("tx_status: %s\n", status);

	if (complete)
		req->complete = complete;
	else
		req->complete = tx_status_completed;

	memcpy(req->buf, status, len);
	req->length = len;
	rc = usb_ep_queue(ep_in, req, 0);
	if (rc != 0 && rc != -ESHUTDOWN) {
		FASTBOOT_ERR("queue tx status err %d\n", rc);
		return rc;
	}
	return 0;
}

static int tx_status(const char *status)
{
	return tx_status_complete(status, tx_status_completed);
}

static int tx_status_err(const char *err_status)
{
	FASTBOOT_ERR("err: %s\n", err_status);
	return tx_status_complete(err_status, tx_status_completed);
}

/*-------------------------------------------------------------------------*/

static int rx_cmd(void)
{
	struct usb_request *req = req_rx;
	int rc;

	if (!req)
		return -EPERM;

	req->actual = 0;
	req->buf = rx_buffer;
	req->length = EP_BUFFER_SIZE;
	req->complete = rx_cmd_complete;
	rc = usb_ep_queue(ep_out, req, 0);
	if (rc != 0 && rc != -ESHUTDOWN)
		FASTBOOT_ERR("queue rx cmd err %d\n", rc);

	return rc;
}

#define RX_DATA_BYTES_PER_DOT	(0x00040000 - 1)	/* 262144-1 */
#define RX_DATA_BYTES_PER_LINE	(0x01000000 - 1)	/* 16777216-1 */

static int rx_data(void)
{
	struct usb_request *req = req_rx;
	int rc;

	if (!req)
		return -EPERM;

	req->actual = 0;
	req->buf = (void*) rx_addr;

#if defined(CONFIG_USB_DMA_MODE)
	req->length = rx_length;
#else
	req->length = min(rx_length, (RX_DATA_BYTES_PER_DOT + 1));
#endif /* CONFIG_USB_DMA_MODE */

	req->complete = rx_data_complete;
	rc = usb_ep_queue(ep_out, req, 0);
	if (rc) {
		download_has_error = 1;
		download_size = 0;
		rx_addr = 0;
		rx_bytes = 0;
		rx_length = 0;

		if (rc != -ESHUTDOWN &&
			!(rc == -EOPNOTSUPP && req->length == 0)) {
			FASTBOOT_ERR("queue rx data (0x%08lX,%u) err %d\n", rx_addr, req->length, rc);
		}
	}

	return rc;
}

static void rx_data_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (!req_rx)
		return;

	if (req->status != 0) {
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);

		download_has_error = 1;
		download_size = 0;
		rx_addr = 0;
		rx_bytes = 0;
		rx_length = 0;

		if (req->status != -ESHUTDOWN) {
			(void)tx_status_err("FAILdownload failure");
			rx_cmd();
		}

		return;
	}

	if (req->actual > rx_length)
		req->actual = rx_length;

	rx_bytes += req->actual;

#if !defined(CONFIG_USB_DMA_MODE)
	if (rx_bytes && (rx_bytes & RX_DATA_BYTES_PER_DOT) == 0) {
		PUTC('.');
		if ((rx_bytes & RX_DATA_BYTES_PER_LINE) == 0)
			PUTC('\n');
	}
#endif /* !CONFIG_USB_DMA_MODE */

	if (rx_bytes >= download_size) {
#if defined(CONFIG_USB_DMA_MODE)
		FASTBOOT_PRINT("DMA download of %u bytes finished\n", rx_bytes);
#else
		if (rx_bytes & RX_DATA_BYTES_PER_DOT)
			PUTC('*');
		if (rx_bytes & RX_DATA_BYTES_PER_LINE)
			PUTC('\n');
		FASTBOOT_PRINT("download of %u bytes finished\n", rx_bytes);
#endif /* !CONFIG_USB_DMA_MODE */

		rx_addr = 0;
		rx_bytes = 0;
		rx_length = 0;

		if (download_has_error)
			(void)tx_status_err("FAILdownload failure");
		else
			(void)tx_status("OKAY");
		rx_cmd();
	} else if (req->actual != req->length) {
		FASTBOOT_ERR("invalid data transmission (%u<->%u)\n",
				req->actual, req->length);
		download_has_error = 1;
		download_size = 0;
		rx_addr = 0;
		rx_bytes = 0;
		rx_length = 0;

		fastboot_queue_work(fastboot_ep_set_halt, ep_out, NULL);
		fastboot_queue_work(fastboot_ep_set_halt, ep_in, NULL);
	} else {
		rx_addr += req->actual;
		rx_length -= req->actual;

		if (rx_addr & (ARCH_DMA_MINALIGN - 1)) {
			FASTBOOT_ERR("rx_addr 0x%08lX (%u,%u) not aligned (%u)\n",
					rx_addr, rx_bytes, download_size, req->actual);
			download_has_error = 1;
			download_size = 0;
			rx_addr = 0;
			rx_bytes = 0;
			rx_length = 0;

			fastboot_queue_work(fastboot_ep_set_halt, ep_out, NULL);
			fastboot_queue_work(fastboot_ep_set_halt, ep_in, NULL);
		} else {
			if (rx_data())
				rx_cmd();
		}
	}
}

/*-------------------------------------------------------------------------*/

static void cmd_download(const char* cmdbuf)
{
	int rc;
	char status[16];

	rx_addr = download_addr;
	rx_length = simple_strtoul(cmdbuf, NULL, 16);
	rx_bytes = 0;

	FASTBOOT_VPRINT("%s: 0x%08X\n", __func__, rx_length);

#if defined(CONFIG_USB_DMA_MODE)
	FASTBOOT_PRINT("starting DMA download of %u bytes at 0x%08lX ...\n",
			rx_length, rx_addr);
#else
	FASTBOOT_PRINT("starting download of %u bytes at 0x%08lX\n",
			rx_length, rx_addr);
#endif /* CONFIG_USB_DMA_MODE */

	download_has_error = 1;

	if (0 == rx_length) {
		(void)tx_status_err("FAILinvalid data size");
		rx_cmd();
		return;
	} else if (rx_length > CONFIG_FASTBOOT_BUFFER_SIZE) {
		(void)tx_status_err("FAILdata too large");
		rx_cmd();
		return;
	}

	download_has_error = 0;
	download_size = rx_length;

	rc = rx_data();
	if (rc) {
		(void)tx_status_err("FAILdownload failure");
		rx_cmd();
	} else {
		sprintf(status, "DATA%08x", rx_length);
		(void)tx_status(status);
	}
}

/*-------------------------------------------------------------------------*/

/**
 * ABOOT_PART_FLAGS_TYPE_RAW verify
 */
static int pte_raw_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	u32 blk_cnt = ALIGN(size, pte->blk_size) / pte->blk_size;
	u32 repeat = ABOOT_PART_GET_REPEAT(pte->flags);

	if (repeat && blk_cnt > pte->part_blk_num) {
		FASTBOOT_ERR("%s: size too large (%u<->%u)\n", name, blk_cnt, pte->part_blk_num);
		goto _image_too_large;
	}

	if (blk_cnt > pte->part_blk_size) {
		FASTBOOT_ERR("%s: size too large (%u<->%u)\n", name, blk_cnt, pte->part_blk_size);
		goto _image_too_large;
	}

	return 0;

_image_too_large:
	(void)tx_status_err("FAILsize too large");
	return -ERANGE;
}

/*-------------------------------------------------------------------------*/

/**
 * ABOOT_PART_FLAGS_TYPE_RADIO verify
 */
static int pte_radio_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	return pte_raw_verify(name, pte, buf, size);
}

/**
 * ABOOT_PART_FLAGS_TYPE_RADIO write
 */
static int pte_radio_write(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	FASTBOOT_ERR("%s: not supported\n", name);
	(void)tx_status_err("FAILnot supported");
	return -ENODEV;
}

/*-------------------------------------------------------------------------*/

/* TI OMAP3/4 HS/GP header */
struct ch_toc {
	uint32_t section_offset;
	uint32_t section_size;
	uint8_t unused[12];
	uint8_t section_name[12];
};

struct ch_settings {
	uint32_t section_key;
	uint8_t valid;
	uint8_t version;
	uint16_t reserved;
	uint32_t flags;
};

struct gp_header {
	uint32_t size;
	uint32_t load_addr;
};

#define KEY_CHSETTINGS 0xC0C0C0C1

/* Header size is CH header rounded up to 512 bytes plus GP header */
#define OMAP_CH_HDR_SIZE 512
#define OMAP_GP_HDR_SIZE (sizeof(struct gp_header))
#define OMAP_FILE_HDR_SIZE (OMAP_CH_HDR_SIZE + OMAP_GP_HDR_SIZE)

#ifdef CONFIG_ANDROID_TRUST_BOOT
#define OMAP_FILE_PIGGYBACK_SIZE (IH_SHA_LENGTH + IH_RSA_LENGTH)
#else
#define OMAP_FILE_PIGGYBACK_SIZE (IH_SHA_LENGTH)
#define OMAP_FILE_PIGGYBACK_SIZE_TRUST_BOOT (OMAP_FILE_PIGGYBACK_SIZE + IH_RSA_LENGTH)
#endif /* CONFIG_ANDROID_TRUST_BOOT */

static int iboot_mlo_verify_gp_data(const char* name, const u8* buf, unsigned int size)
{
	const uint8_t *sha1;
	uint8_t *ptr_sha = (uint8_t *)(buf + size - OMAP_FILE_PIGGYBACK_SIZE);
#ifdef CONFIG_ANDROID_TRUST_BOOT
	const RSAPublicKey *key;
	uint8_t *ptr_rsa = (uint8_t *)(((void *)ptr_sha) + IH_SHA_LENGTH);
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx, buf, size - OMAP_FILE_PIGGYBACK_SIZE);
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(sha1, ptr_sha, IH_SHA_LENGTH)) {
		FASTBOOT_ERR("%s: corrupted data\n", name);
		return -EBADMSG;
	}

#ifdef CONFIG_ANDROID_TRUST_BOOT
	/* RSA signature */
	key = board_get_trust_boot_key();
	if (!key || RSA_verify(key, ptr_rsa, IH_RSA_LENGTH, sha1) == 0) {
		FASTBOOT_ERR("%s: corrupted data against key!\n", name);
		return -EBADMSG;
	}
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	return 0;
}

static int iboot_mlo_verify_gp(const char* name, const u8* buf, unsigned int size)
{
	struct ch_toc *toc = (struct ch_toc *)buf;
	struct gp_header *gph = (struct gp_header *)(buf + OMAP_CH_HDR_SIZE);
	uint32_t s_offset, s_size;
	int found = 0;

	while (toc->section_offset != 0xffffffff && toc->section_size != 0xffffffff) {
		s_offset = toc->section_offset;
		s_size = toc->section_size;
		if (!s_offset || !s_size) {
			FASTBOOT_ERR("%s: invalid section\n", name);
			return -EBADMSG;
		}
		if (s_offset >= OMAP_CH_HDR_SIZE || (s_offset + s_size) >= OMAP_CH_HDR_SIZE) {
			FASTBOOT_ERR("%s: incorrect section\n", name);
			return -EBADMSG;
		}
		FASTBOOT_DPRINT("%s: offset %u, size %u, name '%s'\n", name, s_offset, s_size, toc->section_name);
		if (memcmp(toc->section_name, "CHSETTINGS", 11) == 0)
			found = 1;
		toc++;
		if ((void*)toc > ((void*)buf + OMAP_CH_HDR_SIZE)) {
			FASTBOOT_ERR("%s: bad toc\n", name);
			return -EBADMSG;
		}
	}

	if (found)
		goto _found;

	FASTBOOT_ERR("%s: section name not found\n", name);
	return -EBADMSG;

_found:
	FASTBOOT_DPRINT("%s: addr 0x%08x\n", name, gph->load_addr);
	if (!gph->load_addr) {
		FASTBOOT_ERR("%s: invalid addr\n", name);
		return -EBADMSG;
	} else if (gph->load_addr != CONFIG_SPL_TEXT_BASE) {
		FASTBOOT_ERR("%s: incorrect addr 0x%08x\n", name, gph->load_addr);
		return -EBADMSG;
	}

	FASTBOOT_DPRINT("%s: size %u\n", name, gph->size);
	if (!gph->size) {
		FASTBOOT_ERR("%s: invalid size\n", name);
		return -EBADMSG;
	} else if (size <= (OMAP_FILE_HDR_SIZE + OMAP_FILE_PIGGYBACK_SIZE)) {
		FASTBOOT_ERR("%s: too small size %u (%u)\n", name, gph->size, size);
		return -EBADMSG;
	} else if (gph->size != (size - OMAP_FILE_HDR_SIZE - OMAP_FILE_PIGGYBACK_SIZE)) {
#ifdef CONFIG_ANDROID_TRUST_BOOT
		FASTBOOT_ERR("%s: not trust boot enabled size %u (%u-%u-%u)\n", name, gph->size, size,
				OMAP_FILE_HDR_SIZE, OMAP_FILE_PIGGYBACK_SIZE);
		return -EBADMSG;
#else
		if (gph->size == (size - OMAP_FILE_HDR_SIZE - OMAP_FILE_PIGGYBACK_SIZE_TRUST_BOOT)) {
			FASTBOOT_PRINT("%s: trust boot enabled size %u (%u)\n", name, gph->size, size);
			/* verify SHA checksum */
			if (iboot_mlo_verify_gp_data(name, buf, size - (OMAP_FILE_PIGGYBACK_SIZE_TRUST_BOOT-OMAP_FILE_PIGGYBACK_SIZE)))
				return -EBADMSG;
		} else if (gph->size == (size - OMAP_FILE_HDR_SIZE)) {
			FASTBOOT_PRINT("%s: pure GP size %u (%u) (skip checksum)\n", name, gph->size, size);
		} else {
			FASTBOOT_ERR("%s: incorrect size %u (%u-%u-%u/%u)\n", name, gph->size, size,
					OMAP_FILE_HDR_SIZE, OMAP_FILE_PIGGYBACK_SIZE, OMAP_FILE_PIGGYBACK_SIZE_TRUST_BOOT);
			return -EBADMSG;
		}
#endif /* CONFIG_ANDROID_TRUST_BOOT */
	} else {
		if (iboot_mlo_verify_gp_data(name, buf, size))
			return -EBADMSG;
	}

	return 0;
}

static int iboot_mlo_verify_hs(const char* name, const u8* buf, unsigned int size)
{
	struct ch_toc *toc = (struct ch_toc *)buf;
	uint32_t s_offset, s_size;
	int found = 0;

	while (toc->section_offset != 0xffffffff && toc->section_size != 0xffffffff) {
		s_offset = toc->section_offset;
		s_size = toc->section_size;
		if (!s_offset || !s_size) {
			FASTBOOT_ERR("%s: invalid section\n", name);
			return -EBADMSG;
		}
		FASTBOOT_DPRINT("%s: offset %u, size %u, name '%s'\n", name, s_offset, s_size, toc->section_name);
		if (memcmp(toc->section_name, "MLO", 4) == 0)
			found = 1;
		toc++;
		if ((void*)toc > ((void*)buf + OMAP_CH_HDR_SIZE)) {
			FASTBOOT_ERR("%s: bad toc\n", name);
			return -EBADMSG;
		}
	}

	if (found)
		goto _found;

	FASTBOOT_ERR("%s: section name not found\n", name);
	return -EBADMSG;

_found:
	if (size < (40 * 1024)) /* 40KB */
		return -EBADMSG;

	/* James Wu @TODO@
	 * 1. We still have no idea to implement this functionality.
	 * 2. Need TI's helps.
	 */
#if 0
	void *data = (void*)buf;
	void *sign = (void*)(data + size - 280);
	u32 ret;

	if ((size < 281) || (size > (32*1024*1024))) {
		FASTBOOT_ERR("%s: incorrect size %u\n", name, size);
		return -EBADMSG;
	}

	size -= 280;

	ret = omap4_secure_dispatcher(12, 0, 4, (u32)data, (u32)size, (u32)sign, (u32)2);
	if (ret != 0) {
		FASTBOOT_ERR("%s: corrupted data against key! (%d)\n", name, ret);
		return -EBADMSG;
	}
	FASTBOOT_ERR("%s: data passed!\n", name);
	return -EBADMSG;
#endif

	return 0;
}

/**
 * ABOOT_PART_FLAGS_TYPE_OMAP_MLO verify
 */
static int pte_mlo_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	u32 device_type = get_device_type();

	if (device_type == GP_DEVICE) {
		/* OMAP GP */
		if (iboot_mlo_verify_gp(name, buf, size)) {
#ifdef CONFIG_SYS_ALLOW_UBOOT_BIN
			if (size > (42 * 1024)) /* 42KB */
#endif /* CONFIG_SYS_ALLOW_UBOOT_BIN */
				goto _incorrect_mlo_img;
		}
	} else {
		/* OMAP HS/EMU */
		if (iboot_mlo_verify_hs(name, buf, size)) {
#ifdef CONFIG_SYS_ALLOW_UBOOT_BIN
			if (size < (36 * 1024)) /* 36KB */
#endif /* CONFIG_SYS_ALLOW_UBOOT_BIN */
				goto _incorrect_mlo_img;
		}
	}

	return pte_raw_verify(name, pte, buf, size);

_incorrect_mlo_img:
	(void)tx_status_err("FAILincorrect image");
	return -EPERM;
}

/*-------------------------------------------------------------------------*/

/**
 * ABOOT_PART_FLAGS_TYPE_IBOOT verify
 */

#if 0

static int pte_iboot_verify_data(const char* name,
		struct image_header *header,
		const unsigned char *data, uint32_t len)
{
	uint32_t checksum;

	checksum = crc32(0, data, len);
	if (checksum != be32_to_cpu(header->ih_dcrc)) {
		FASTBOOT_ERR("%s: corrupted data\n", name);
		FASTBOOT_DPRINT("data checksum 0x%08X <-> 0x%08X\n", checksum, be32_to_cpu(header->ih_dcrc));
		return -EBADMSG;
	}

	return 0;
}

#else

static int pte_iboot_verify_data(const char* name, struct image_header *header,
		const unsigned char *data, uint32_t len)
{
	const uint8_t* sha1;
#ifdef CONFIG_ANDROID_TRUST_BOOT
	const RSAPublicKey *key;
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	/* SHA */
	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx, data, len);
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(sha1, header->ih_sha, IH_SHA_LENGTH)) {
		FASTBOOT_ERR("%s: corrupted data\n", name);
		return -EBADMSG;
	}

#ifdef CONFIG_ANDROID_TRUST_BOOT
	/* RSA signature */
	key = board_get_trust_boot_key();
	if (!key || RSA_verify(key, header->ih_rsa, IH_RSA_LENGTH, sha1) == 0) {
		FASTBOOT_ERR("%s: corrupted data against key!\n", name);
		return -EBADMSG;
	}
#endif /* CONFIG_ANDROID_TRUST_BOOT */

	return 0;
}

#endif

static int pte_iboot_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	struct image_header header;
	struct image_header *hdr = &header;
	const unsigned char *data;
	uint32_t len;
	uint32_t checksum;

	/*
	 * create copy of header so that we can blank out the
	 * checksum field for checking.
	 */
	memcpy(hdr, buf, sizeof(image_header_t));

	/* Header magic */
	if (__be32_to_cpu(hdr->ih_magic) != IH_MAGIC) {
		FASTBOOT_ERR("%s: bad magic 0x%08X\n", name, hdr->ih_magic);
#ifdef CONFIG_SYS_ALLOW_UBOOT_BIN
		FASTBOOT_ERR("%s: assuming u-boot.bin ...\n", name);
		goto _allow_uboot_bin;
#else
		goto _not_iboot_img;
#endif /* CONFIG_SYS_ALLOW_UBOOT_BIN */
	}

	/* Header magic 2 */
	if (__be32_to_cpu(hdr->ih_magic2) != IH_MAGIC2) {
		FASTBOOT_ERR("%s: bad magic2 0x%08X\n", name, hdr->ih_magic2);
		goto _not_iboot_img;
	}

	/* Header checksum */
#if 0
	checksum = hdr->ih_hcrc;

	hdr->ih_hcrc = 0;	/* clear for re-calculation */
	hdr->ih_hcrc = cpu_to_be32(
			crc32(0, (const unsigned char *)hdr, (uint32_t)sizeof(image_header_t)));
	if (checksum != hdr->ih_hcrc) {
		FASTBOOT_ERR("%s: bad image header\n", name);
		FASTBOOT_DPRINT("header checksum 0x%08X <-> 0x%08X\n", checksum, hdr->ih_hcrc);
		goto _not_iboot_img;
	}
#else
	if (hdr->ih_full_hcrc) {
		/* check the full image header */
		checksum = hdr->ih_full_hcrc;

		hdr->ih_hcrc = 0;	/* clear for re-calculation */
		hdr->ih_full_hcrc = 0;	/* clear for re-calculation */
		hdr->ih_full_hcrc = cpu_to_be32(
				crc32(0, (const unsigned char *)hdr, (uint32_t)sizeof(image_header_t)));
		if (checksum != hdr->ih_full_hcrc) {
			FASTBOOT_ERR("%s: bad image header\n", name);
			FASTBOOT_DPRINT("header checksum 0x%08X <-> 0x%08X\n", checksum, hdr->ih_full_hcrc);
			goto _not_iboot_img;
		}
	} else {
		checksum = hdr->ih_hcrc;

		hdr->ih_hcrc = 0;	/* clear for re-calculation */
		hdr->ih_hcrc = cpu_to_be32(
				crc32(0, (const unsigned char *)hdr, (uint32_t)IH_HCRC_LENGTH));
		if (checksum != hdr->ih_hcrc) {
			FASTBOOT_ERR("%s: bad image header\n", name);
			FASTBOOT_DPRINT("header checksum 0x%08X <-> 0x%08X\n", checksum, hdr->ih_hcrc);
			goto _not_iboot_img;
		}
	}
#endif

	/* Validate the board name including the NULL terminator */
	if (memcmp(hdr->ih_name, CONFIG_SYS_BOARD_NAME, sizeof(CONFIG_SYS_BOARD_NAME))) {
		FASTBOOT_ERR("%s: incorrect board '%.32s'\n", name, hdr->ih_name);
		FASTBOOT_DPRINT("board name '%s' <-> '%.32s'\n", CONFIG_SYS_BOARD_NAME, hdr->ih_name);
#ifdef CONFIG_SYS_ALLOW_INCORRECT_BOARD_NAME
#else
		goto _bad_board_name;
#endif /* CONFIG_SYS_ALLOW_INCORRECT_BOARD_NAME */
	}

	/* Data Validation */
	data = ((const unsigned char *)buf) + sizeof(image_header_t);
	len = __be32_to_cpu(hdr->ih_size);
	if (pte_iboot_verify_data(name, (struct image_header *)buf, data, len))
		goto _not_iboot_img;

#ifdef CONFIG_SYS_ALLOW_UBOOT_BIN
_allow_uboot_bin:
#endif /* CONFIG_SYS_ALLOW_UBOOT_BIN */
	return pte_raw_verify(name, pte, buf, size);

_not_iboot_img:
	(void)tx_status_err("FAILnot a bootloader image");
	return -EPERM;
#ifdef CONFIG_SYS_ALLOW_INCORRECT_BOARD_NAME
#else
_bad_board_name:
	(void)tx_status_err("FAILincorrect device");
	return -EPERM;
#endif /* CONFIG_SYS_ALLOW_INCORRECT_BOARD_NAME */
}

/*-------------------------------------------------------------------------*/

/**
 * ABOOT_PART_FLAGS_TYPE_ICOM_MULTIMG verify
 */
static int pte_icom_multimg_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	multi_img_hdr *hdr = (multi_img_hdr*)buf;
	multi_img_entry *image;
	unsigned checksum;
	const uint8_t *sha1;
	unsigned i;

_retry:
	/* Header Validation */
	if (memcmp(hdr->magic, MULTI_IMAGE_MAGIC, MULTI_IMAGE_MAGIC_SIZE)) {
		FASTBOOT_ERR("%s: bad multimg magic\n", name);
		goto _not_multimg;
	}
	if ((hdr->page_size != 2048) && (hdr->page_size != 4096)) {
		FASTBOOT_ERR("%s: unsupported multimg page size %u\n", name, hdr->page_size);
		goto _not_multimg;
	}
	/* Header checksum */
	checksum = hdr->header_checksum;
	hdr->header_checksum = 0;
	hdr->header_checksum = (unsigned)crc32(0, (uchar*)hdr, sizeof(multi_img_hdr));
	if (checksum != hdr->header_checksum) {
		FASTBOOT_ERR("%s: bad multimg header\n", name);
		FASTBOOT_DPRINT("header checksum 0x%08X <-> 0x%08X\n", checksum, hdr->header_checksum);
		goto _not_multimg;
	}
	if (memcmp(hdr->name, pte->name, strlen(pte->name))) {
		FASTBOOT_ERR("%s: bad multimg name: %s<->%s\n", name, hdr->name, pte->name);
		goto _bad_multimg_name;
	}

	/* Data Validation */
	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	for (i = 0; i < hdr->num_of_images; i++) {
		image = hdr->images + i;
		SHA_update(&sha_ctx, (void*)(((unsigned)buf) + image->image_offset), image->image_size);
	}
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(hdr->checksum, sha1,
			SHA_DIGEST_SIZE > sizeof(hdr->checksum) ? sizeof(hdr->checksum) : SHA_DIGEST_SIZE)) {
		FASTBOOT_ERR("%s: corrupted multimg data\n", name);
		goto _corrupted_multimg;
	}

	if (hdr->next_header) {
		hdr = (multi_img_hdr*)(((unsigned)buf) + hdr->next_header);
		goto _retry;
	}

	return pte_raw_verify(name, pte, buf, size);

_not_multimg:
	(void)tx_status_err("FAILnot a multimg image");
	return -EPERM;
_bad_multimg_name:
	(void)tx_status_err("FAILinvalid multimg name");
	return -EPERM;
_corrupted_multimg:
	(void)tx_status_err("FAILcorrupted multimg image");
	return -EPERM;
}

/*-------------------------------------------------------------------------*/

/**
 * ABOOT_PART_FLAGS_TYPE_ABOOT verify
 */
static int pte_aboot_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	boot_img_hdr *hdr = (boot_img_hdr*)buf;
	unsigned kernel_addr, ramdisk_addr, second_addr;	/* physical load addr */
	const uint8_t *sha1;

	/* Header Validation */
	if (memcmp(hdr->magic, BOOT_MAGIC, BOOT_MAGIC_SIZE)) {
		FASTBOOT_ERR("%s: bad image magic\n", name);
		goto _not_aboot_img;
	}
	if ((hdr->page_size != 2048) && (hdr->page_size != 4096)) {
		FASTBOOT_ERR("%s: unsupported page size %u\n", name, hdr->page_size);
		goto _not_aboot_img;
	}
	if (hdr->unused[0] != 0) {
		/* Header checksum */
		unsigned checksum = hdr->unused[0];
		hdr->unused[0] = 0;
		hdr->unused[0] = (unsigned)crc32(0, (uchar*)hdr, sizeof(*hdr));
		if (checksum != hdr->unused[0]) {
			FASTBOOT_ERR("%s: bad image header\n", name);
			FASTBOOT_DPRINT("header checksum 0x%08X <-> 0x%08X\n", checksum, hdr->unused[0]);
			goto _not_aboot_img;
		}
	}
	if (hdr->kernel_size == 0 || hdr->kernel_size > (15 * 1024 * 1024)/*15MB*/) {
		FASTBOOT_ERR("%s: bad kernel size %u\n", name, hdr->kernel_size);
		goto _not_aboot_img;
	}
	if (hdr->ramdisk_size == 0 || hdr->ramdisk_size > (20 * 1024 * 1024)/*20MB*/) {
		FASTBOOT_ERR("%s: bad ramdisk size %u\n", name, hdr->ramdisk_size);
		goto _not_aboot_img;
	}
	if (hdr->second_size > (20 * 1024 * 1024)/*20MB*/) {
		FASTBOOT_ERR("%s: bad second size %u\n", name, hdr->second_size);
		goto _not_aboot_img;
	}

	/* Data Validation */
	kernel_addr = ((unsigned)buf) + ALIGN(sizeof(hdr), hdr->page_size);
	ramdisk_addr = kernel_addr + ALIGN(hdr->kernel_size, hdr->page_size);
	second_addr = ramdisk_addr + ALIGN(hdr->ramdisk_size, hdr->page_size);

	memset(&sha_ctx, 0, sizeof(SHA_CTX));
	SHA_init(&sha_ctx);
	SHA_update(&sha_ctx, (void*)kernel_addr, hdr->kernel_size);
	SHA_update(&sha_ctx, &hdr->kernel_size, sizeof(hdr->kernel_size));
	SHA_update(&sha_ctx, (void*)ramdisk_addr, hdr->ramdisk_size);
	SHA_update(&sha_ctx, &hdr->ramdisk_size, sizeof(hdr->ramdisk_size));
	SHA_update(&sha_ctx, (void*)second_addr, hdr->second_size);
	SHA_update(&sha_ctx, &hdr->second_size, sizeof(hdr->second_size));
	sha1 = SHA_final(&sha_ctx);
	if (memcmp(hdr->id, sha1,
			SHA_DIGEST_SIZE > sizeof(hdr->id) ? sizeof(hdr->id) : SHA_DIGEST_SIZE)) {
		FASTBOOT_ERR("%s: corrupted data\n", name);
		goto _corrupted_aboot_img;
	}

	return pte_raw_verify(name, pte, buf, size);

_not_aboot_img:
	(void)tx_status_err("FAILnot a boot image");
	return -EPERM;
_corrupted_aboot_img:
	(void)tx_status_err("FAILcorrupted boot image");
	return -EPERM;
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO

static struct mmc* get_mmc_device(int dev)
{
	int ret;
	struct mmc *mmc;

	mmc = find_mmc_device(dev);
	if (!mmc) {
		FASTBOOT_ERR("invalid mmc device: %d\n", dev);
		goto _exit;
	}

	ret = mmc_init(mmc);
	if (ret) {
		FASTBOOT_ERR("mmc init err %d\n", ret);
		goto _exit;
	}
	if (IS_SD(mmc)) {
		FASTBOOT_ERR("not eMMC device\n");
		goto _exit;
	}

	return mmc;

_exit:
	(void)tx_status_err("FAILfailed to find mmc device");
	return NULL;
}

/*-------------------------------------------------------------------------*/

/**
 * Generic MMC write
 */
static int pte_generic_mmc_write(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	unsigned long blk_start, blk_cnt, n;
	u32 num, repeat;

	mmc = get_mmc_device(CONFIG_ABOOT_EMMC_DEVICE_NO);
	if (!mmc)
		return -ENODEV;
	dev_desc = &(mmc->block_dev);

	blk_cnt = ALIGN(size, dev_desc->blksz) / dev_desc->blksz;

	repeat = ABOOT_PART_GET_REPEAT(pte->flags);
	if (repeat) {
		int err_num = 0;
		for (num = 0 ; num < repeat ; num++) {
			blk_start = num * pte->part_blk_num + pte->part_blk_start;
			FASTBOOT_DPRINT("writing '%s' (%lu blks) at 0x%08lX\n", pte->name, blk_cnt, blk_start);
			n = dev_desc->block_write(CONFIG_ABOOT_EMMC_DEVICE_NO, blk_start, blk_cnt, (void*)buf);
			if (n != blk_cnt) {
				FASTBOOT_ERR("write err %lu (0x%08lX,%lu)\n", n, blk_start, blk_cnt);
				err_num++;
				/*
				 * Because we have to make sure the writes of repeated images are successful,
				 * We don't return the error immediately.
				 */
			}
		}
		if (err_num == repeat)
			goto _write_err;
	} else {
		blk_start = pte->part_blk_start;
		FASTBOOT_DPRINT("writing '%s' (%lu blks) at 0x%08lX\n", pte->name, blk_cnt, blk_start);
		n = dev_desc->block_write(CONFIG_ABOOT_EMMC_DEVICE_NO, blk_start, blk_cnt, (void*)buf);
		if (n != blk_cnt) {
			FASTBOOT_ERR("write err %lu (0x%08lX,%lu)\n", n, blk_start, blk_cnt);
			goto _write_err;
		}
	}

	return 0;

_write_err:
	(void)tx_status_err("FAILflash write failure");
	return -EIO;
}

/*-------------------------------------------------------------------------*/

#define SPARSE_HEADER_MAJOR_VER 1

/**
 * ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4 verify
 */
static int pte_sparse_ext4_verify(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
	u64 part_size, total_size;
	sparse_header_t *sparse_header = (sparse_header_t*)buf;

	if (sparse_header->magic != SPARSE_HEADER_MAGIC) {
		FASTBOOT_ERR("%s: bad image magic\n", name);
		goto _not_sparse_ext4_img;
	}

	if (sparse_header->major_version != SPARSE_HEADER_MAJOR_VER) {
		FASTBOOT_ERR("%s: unknown major version: %d\n", name, sparse_header->major_version);
		goto _not_sparse_ext4_img;
	}

	if ((sparse_header->file_hdr_sz != sizeof(sparse_header_t)) ||
			(sparse_header->chunk_hdr_sz != sizeof(chunk_header_t))) {
		FASTBOOT_ERR("%s: incompatible format\n", name);
		goto _not_sparse_ext4_img;
	}

	if (sparse_header->blk_sz < pte->blk_size ||
			(sparse_header->blk_sz % pte->blk_size) != 0) {
		FASTBOOT_ERR("%s: incompatible block size (%u<->%u)\n",
				name, sparse_header->blk_sz, pte->blk_size);
		goto _not_sparse_ext4_img;
	}

	part_size = ((u64)pte->part_blk_size) * pte->blk_size;
	total_size = ((u64)sparse_header->total_blks) * sparse_header->blk_sz;
	if (total_size > part_size) {
		FASTBOOT_ERR("%s: size too large (%llu<->%llu)\n", name, total_size, part_size);
		goto _image_total_too_large;
	}

	return pte_raw_verify(name, pte, buf, size);

_not_sparse_ext4_img:
	(void)tx_status_err("FAILnot a sparse ext4 image");
	return -EPERM;

_image_total_too_large:
	(void)tx_status_err("FAILext4 size too large");
	return -ERANGE;
}

/**
 * ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4 write
 */
static int pte_sparse_ext4_mmc_write(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size)
{
#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
	unsigned long uart_flush_timeout = 0, uart_flush_ticks = 0;
#endif
	chunk_header_t *chunk;
	unsigned long blk_start, blk_cnt, blk_total, n;
	u8* ptr;
	u32 i, len;
	sparse_header_t *sparse_header;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;

	mmc = get_mmc_device(CONFIG_ABOOT_EMMC_DEVICE_NO);
	if (!mmc)
		return -ENODEV;
	dev_desc = &(mmc->block_dev);
	blk_start = pte->part_blk_start;
	blk_total = 0;

	/* Skip the header */
	sparse_header = (sparse_header_t*)buf;
	ptr = (u8*)(buf + sparse_header->file_hdr_sz);

	for (i = 0; i < sparse_header->total_chunks; i++) {
		chunk = (chunk_header_t*)ptr;
		/* move to next chunk */
		ptr += sizeof(chunk_header_t);

		switch (chunk->chunk_type) {
		case CHUNK_TYPE_RAW:
			len = chunk->chunk_sz * sparse_header->blk_sz;
			if (chunk->total_sz != (len + sizeof(chunk_header_t))) {
				FASTBOOT_ERR("%s: bad chunk for RAW %d\n", name, i);
				goto _bad_sparse_ext4_img;
			}

			blk_cnt = ALIGN(len, dev_desc->blksz) / dev_desc->blksz;
			blk_total += blk_cnt;
			if (blk_total > pte->part_blk_size) {
				FASTBOOT_ERR("%s: size too large (%lu<->%u)\n", name, blk_total, pte->part_blk_size);
				goto _image_size_too_large;
			}

#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
			FASTBOOT_VPRINT("writing '%s' RAW (%lu blks) at 0x%08lX\n", pte->name, blk_cnt, blk_start);
#elif defined(CONFIG_FASTBOOT_DEBUG)
			PUTC('.');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
			n = dev_desc->block_write(CONFIG_ABOOT_EMMC_DEVICE_NO, blk_start, blk_cnt, (void*)ptr);
			if (n != blk_cnt) {
				FASTBOOT_ERR("write err %lu (0x%08lX,%lu)\n", n, blk_start, blk_cnt);
				goto _write_err;
			}

			blk_start += blk_cnt;
			ptr += len;
			break;

		case CHUNK_TYPE_FILL: {
			u32 fill_value;
			u32 *fill_buf;
			u32 i = 0;

			len = 4;
			if (chunk->total_sz != (len + sizeof(chunk_header_t))) {
				FASTBOOT_ERR("%s: bad chunk for FILL %d\n", name, i);
				goto _bad_sparse_ext4_img;
			}
			fill_value = *((u32 *)ptr);
			ptr += len;

			len = chunk->chunk_sz * sparse_header->blk_sz;
			blk_cnt = ALIGN(len, dev_desc->blksz) / dev_desc->blksz;
			blk_total += blk_cnt;
			if (blk_total > pte->part_blk_size) {
				FASTBOOT_ERR("%s: size too large (%lu<->%u)\n", name, blk_total, pte->part_blk_size);
				goto _image_size_too_large;
			}

			fill_buf = memalign(ARCH_DMA_MINALIGN, dev_desc->blksz);
			if (fill_buf == NULL) {
				FASTBOOT_ERR("%s: out of memory\n", name);
				goto _write_err;
			}
			for (i = 0; i < dev_desc->blksz/sizeof(u32); i++) {
				fill_buf[i] = fill_value;
			}
#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
			FASTBOOT_VPRINT("'%s' FILL 0x%08X (%lu blks) at 0x%08lX\n", pte->name, fill_value, blk_cnt, blk_start);
#elif defined(CONFIG_FASTBOOT_DEBUG)
			PUTC('F');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
			while (blk_cnt) {
				n = dev_desc->block_write(CONFIG_ABOOT_EMMC_DEVICE_NO, blk_start, 1, (void*)fill_buf);
				if (n != 1) {
					FASTBOOT_ERR("write err %lu (0x%08lX,1)\n", n, blk_start);
					free(fill_buf);
					goto _write_err;
				}
				blk_start++;
				blk_cnt--;
			}
			free(fill_buf);

			} break;

		case CHUNK_TYPE_DONT_CARE:
			if (chunk->total_sz != sizeof(chunk_header_t)) {
				FASTBOOT_ERR("%s: bad chunk for DONT CARE %d\n", name, i);
				goto _bad_sparse_ext4_img;
			}
			len = chunk->chunk_sz * sparse_header->blk_sz;
			blk_cnt = ALIGN(len, dev_desc->blksz) / dev_desc->blksz;
			blk_total += blk_cnt;
			if (blk_total > pte->part_blk_size) {
				FASTBOOT_ERR("%s: size too large (%lu<->%u)\n", name, blk_total, pte->part_blk_size);
				goto _image_size_too_large;
			}
#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
			FASTBOOT_VPRINT("skipping '%s' DONT CARE (%lu blks) at 0x%08lX\n", pte->name, blk_cnt, blk_start);
#elif defined(CONFIG_FASTBOOT_DEBUG)
			PUTC('D');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
			blk_start += blk_cnt;
			break;

		case CHUNK_TYPE_CRC32:
			len = 4;
			if (chunk->total_sz != (len + sizeof(chunk_header_t))) {
				FASTBOOT_ERR("%s: bad chunk for CRC32 %d\n", name, i);
				goto _bad_sparse_ext4_img;
			}
			ptr += len;
#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
			FASTBOOT_VPRINT("skipping '%s' CRC32 (%lu blks) at 0x%08lX\n", pte->name, blk_cnt, blk_start);
#elif defined(CONFIG_FASTBOOT_DEBUG)
			PUTC('C');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
			break;

		default:
#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
			FASTBOOT_ERR("%s: unknown sparse chunk %04x\n", name, chunk->chunk_type);
#elif defined(CONFIG_FASTBOOT_DEBUG)
			PRINT("X[%04x]", chunk->chunk_type);
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
			break;
		}

#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
		if (timeout_check(&uart_flush_timeout, &uart_flush_ticks)) {
			UART_FIFO_FLUSH_ONCE();
			uart_flush_timeout = timeout_init(30000/*ms*/, &uart_flush_ticks);
		}
#endif

#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
#elif defined(CONFIG_FASTBOOT_DEBUG)
		if (i && (i & 0x3F) == 0)
			PUTC('\n');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
	}

#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
#elif defined(CONFIG_FASTBOOT_DEBUG)
	if (i & 0x3F)
		PUTC('\n');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */

	return 0;

_image_size_too_large:
	(void)tx_status_err("FAILext4 size too large");
	return -ERANGE;
_write_err:
	(void)tx_status_err("FAILflash write failure");
	return -EIO;
_bad_sparse_ext4_img:
	(void)tx_status_err("FAILbad sparse ext4 image");
	return -EPERM;
}

/*-------------------------------------------------------------------------*/

/**
 * Generic MMC erase (write by filling 0xFF)
 */
static int pte_generic_mmc_erase(const char* name, aboot_ptentry *pte)
{
#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
	unsigned long uart_flush_timeout = 0, uart_flush_ticks = 0;
#endif
	unsigned long blk_start, blk_cnt, blk_erase, blk_r, n;
	struct mmc *mmc;
	block_dev_desc_t *dev_desc;
	u8 *buf = (u8*)download_addr;
	u32 i;

	if ((pte->erase_size & (pte->blk_size - 1)) != 0) {
		FASTBOOT_ERR("%s: not 0x%X aligned erase size 0x%08X\n", pte->name,
				pte->blk_size, pte->erase_size);
		goto _not_aligned_part;
	}
	blk_erase = pte->erase_size / pte->blk_size;

	mmc = get_mmc_device(CONFIG_ABOOT_EMMC_DEVICE_NO);
	if (!mmc)
		return -ENODEV;
	dev_desc = &(mmc->block_dev);

	memset(buf, 0xFF, pte->erase_size);

	blk_start = pte->part_blk_start;
	if ((pte->flags & ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4) == ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4) {
		blk_cnt = min(pte->part_blk_size, 10 * blk_erase);
	} else
		blk_cnt = pte->part_blk_size;

	i = 0;
	while (blk_cnt > 0) {
		blk_r = min(blk_cnt, blk_erase);
#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
		FASTBOOT_VPRINT("erasing '%s' (%lu blks) at 0x%08lX\n", pte->name, blk_r, blk_start);
#elif defined(CONFIG_FASTBOOT_DEBUG)
		PUTC('.');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
		n = dev_desc->block_write(CONFIG_ABOOT_EMMC_DEVICE_NO, blk_start, blk_r, (void*)buf);
		if (n != blk_r) {
			FASTBOOT_ERR("erase err %lu (0x%08lX,%lu)\n", n, blk_start, blk_r);
			goto _erase_err;
		}
		blk_start += blk_r;
		blk_cnt -= blk_r;
		i++;

#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
		if (timeout_check(&uart_flush_timeout, &uart_flush_ticks)) {
			UART_FIFO_FLUSH_ONCE();
			uart_flush_timeout = timeout_init(30000/*ms*/, &uart_flush_ticks);
		}
#endif

#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
#elif defined(CONFIG_FASTBOOT_DEBUG)
		if ((i & 0x3F) == 0)
			PUTC('\n');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */
	}

#ifdef CONFIG_FASTBOOT_VERBOSE_DEBUG
#elif defined(CONFIG_FASTBOOT_DEBUG)
	if (i & 0x3F)
		PUTC('\n');
#endif /* CONFIG_FASTBOOT_VERBOSE_DEBUG */

	return 0;

_erase_err:
	(void)tx_status_err("FAILfailed to erase partition");
	return -EIO;
_not_aligned_part:
	(void)tx_status_err("FAILcan not erase partition");
	return -ERANGE;
}

#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */

/*-------------------------------------------------------------------------*/

struct pte_handler_info {
	char *name;
	int (*verify)(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size);
	int (*write)(const char* name, aboot_ptentry *pte, const u8* buf, unsigned int size);
	int (*erase)(const char* name, aboot_ptentry *pte);
};

static struct pte_handler_info pte_handlers[ABOOT_PART_FLAGS_DEVICE_MAX][ABOOT_PART_FLAGS_TYPE_MAX] = {
	[ABOOT_PART_FLAGS_DEVICE_EMMC >> ABOOT_PART_FLAGS_DEVICE_SHIFT] = {
#ifdef CONFIG_ABOOT_EMMC_DEVICE_NO
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_RAW)] = {
			.name = "eMMCraw",
			.verify	= pte_raw_verify,
			.write	= pte_generic_mmc_write,
			.erase	= pte_generic_mmc_erase,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_RADIO)] = {
			.name = "eMMCradio",
			.verify	= pte_radio_verify,
			.write	= pte_radio_write,
			.erase	= NULL,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_ICOM_MULTIMG)] = {
			.name = "eMMCmultimg",
			.verify	= pte_icom_multimg_verify,
			.write	= pte_generic_mmc_write,
			.erase	= pte_generic_mmc_erase,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_OMAP_MLO)] = {
			.name = "eMMCmlo",
			.verify	= pte_mlo_verify,
			.write	= pte_generic_mmc_write,
			.erase	= pte_generic_mmc_erase,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_IBOOT)] = {
			.name = "eMMCiboot",
			.verify	= pte_iboot_verify,
			.write	= pte_generic_mmc_write,
			.erase	= pte_generic_mmc_erase,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_ABOOT)] = {
			.name = "eMMCaboot",
			.verify	= pte_aboot_verify,
			.write	= pte_generic_mmc_write,
			.erase	= pte_generic_mmc_erase,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_SPARSE_EXT4)] = {
			.name = "eMMCext4",
			.verify	= pte_sparse_ext4_verify,
			.write	= pte_sparse_ext4_mmc_write,
			.erase	= pte_generic_mmc_erase,
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_YAFFS2)] = {
			/* Not Support */
			.name = "eMMCyaffs2",
		},
		[ABOOT_PART_GET_TYPE_INDEX(ABOOT_PART_FLAGS_TYPE_VFAT)] = {
			/* Not Support */
			.name = "eMMCfat32",
		},
#endif /* CONFIG_ABOOT_EMMC_DEVICE_NO */
	},
#ifdef CONFIG_CMD_NAND
	[ABOOT_PART_FLAGS_DEVICE_NAND >> ABOOT_PART_FLAGS_DEVICE_SHIFT] = {
#error "Not implemented yet!\n"
	},
#endif /* CONFIG_CMD_NAND */
};

/*-------------------------------------------------------------------------*/

static void cmd_flash(const char* cmdbuf)
{
	int ret;
	struct pte_handler_info *handler;
	aboot_ptentry *pte;
	unsigned int dev_idx, type_idx;
	const u8* buf;
	unsigned int size;

	FASTBOOT_VPRINT("%s\n", __func__);

	if (download_has_error || download_size == 0 || download_addr == 0) {
		(void)tx_status_err("FAILno image downloaded");
		goto _exit;
	}

	pte = aboot_get_part_by_name(cmdbuf);
	if (!pte || (pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_HIDDEN))) {
		FASTBOOT_ERR("%.20s: part not found\n", cmdbuf);
		(void)tx_status_err("FAILpartition does not exist");
		goto _exit;
	}

	if (pte->flags & (ABOOT_PART_FLAGS_LOCKED) && !aboot_board_is_unlocked()) {
		FASTBOOT_ERR("%.20s: part is locked\n", cmdbuf);
		(void)tx_status_err("FAILBootloader Locked");
		goto _exit;
	}

	dev_idx = ABOOT_PART_GET_DEVICE(pte->flags) >> ABOOT_PART_FLAGS_DEVICE_SHIFT;
	type_idx = ABOOT_PART_GET_TYPE(pte->flags) >> ABOOT_PART_FLAGS_TYPE_SHIFT;
	if (dev_idx >= ABOOT_PART_FLAGS_DEVICE_MAX || type_idx >= ABOOT_PART_FLAGS_TYPE_MAX) {
		(void)tx_status_err("FAILinternal error");
		goto _exit;
	}

	handler = &(pte_handlers[dev_idx][type_idx]);
	buf = (const u8*)download_addr;
	size = download_size;

	/* James Wu @TODO@: check the locked partition */

	if (handler->verify) {
		UART_FIFO_FLUSH_ONCE();

		FASTBOOT_PRINT("verifying for '%s' ...\n", pte->name);
		ret = handler->verify(handler->name, pte, buf, size);
		if (ret) {
			FASTBOOT_ERR("%s: verify err %d\n", handler->name, ret);
			goto _exit;
		}
	}

	if (handler->write) {
		UART_FIFO_FLUSH_ONCE();

#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
		FASTBOOT_PRINT("writing '%s' (%u bytes) ...\n", pte->name, size);
#else
		FASTBOOT_PRINT("writing '%s' (%u bytes) ...", pte->name, size);
#endif
		ret = handler->write(handler->name, pte, buf, size);
		if (ret) {
#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
			FASTBOOT_ERR("%s: write err %d\n", handler->name, ret);
#else
			PRINT("'%s' err %d (%s)\n", pte->name, ret, handler->name);
#endif
			goto _exit;
		}
#if defined(CONFIG_FASTBOOT_DEBUG) || defined(CONFIG_FASTBOOT_VERBOSE_DEBUG)
		FASTBOOT_PRINT("writing '%s' finished\n", pte->name, size);
#else
		PRINT("'%s' done\n\n", pte->name);
#endif

		UART_FIFO_FLUSH_ONCE();
	} else {
		FASTBOOT_ERR("%s: not supported\n", handler->name);
		(void)tx_status_err("FAILnot supported");
		goto _exit;
	}

	(void)tx_status("OKAY");

_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static void cmd_erase(const char* cmdbuf)
{
	int ret;
	struct pte_handler_info *handler;
	aboot_ptentry *pte;
	u32 flags;
	unsigned int dev_idx, type_idx;

	FASTBOOT_VPRINT("%s\n", __func__);

	pte = aboot_get_part_by_name(cmdbuf);
	if (!pte || (pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_HIDDEN))) {
		FASTBOOT_ERR("%.20s: part not found\n", cmdbuf);
		(void)tx_status_err("FAILpartition does not exist");
		goto _exit;
	}

	flags = ABOOT_PART_GET_OTHER(pte->flags);
	if (!(flags & ABOOT_PART_FLAGS_ERASABLE))
		goto _not_erasable;

	if (pte->flags & (ABOOT_PART_FLAGS_LOCKED) && !aboot_board_is_unlocked()) {
		FASTBOOT_ERR("%.20s: part is locked\n", cmdbuf);
		(void)tx_status_err("FAILBootloader Locked");
		goto _exit;
	}

	dev_idx = ABOOT_PART_GET_DEVICE(pte->flags) >> ABOOT_PART_FLAGS_DEVICE_SHIFT;
	type_idx = ABOOT_PART_GET_TYPE(pte->flags) >> ABOOT_PART_FLAGS_TYPE_SHIFT;
	if (dev_idx >= ABOOT_PART_FLAGS_DEVICE_MAX || type_idx >= ABOOT_PART_FLAGS_TYPE_MAX) {
		(void)tx_status_err("FAILinternal error");
		goto _exit;
	}

	handler = &(pte_handlers[dev_idx][type_idx]);
	if (handler->erase) {
		UART_FIFO_FLUSH_ONCE();

		FASTBOOT_PRINT("erasing '%s' ...\n", pte->name);
		ret = handler->erase(handler->name, pte);
		if (ret) {
			FASTBOOT_ERR("%s: erase err %d\n", handler->name, ret);
			goto _exit;
		}

		UART_FIFO_FLUSH_ONCE();
	} else {
		FASTBOOT_ERR("%s: not supported\n", handler->name);
		goto _not_erasable;
	}

	(void)tx_status("OKAY");
	goto _exit;

_not_erasable:
	(void)tx_status_err("FAILcan not erase partition");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static void cmd_complete_boot(struct usb_ep *ep, struct usb_request *req)
{
	UART_FIFO_FLUSH_ONCE();

	if (req_tx && req->status != 0)
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);

	if (aboot_header.kernel_addr && aboot_header.ramdisk_addr)
		do_aboot_linux((char*)aboot_header.cmdline, aboot_header.kernel_addr,
				aboot_header.ramdisk_addr, aboot_header.ramdisk_size);

	VIDEO_PUTS(0, "Fastboot: booting error occurred\n");

	memset(&aboot_header, 0, sizeof(aboot_header));

	rx_cmd();
}

static void cmd_boot(const char* cmdbuf)
{
	int ret;
	struct pte_handler_info *handler;
	aboot_ptentry *pte;
	unsigned int dev_idx, type_idx;
	unsigned kernel_addr, ramdisk_addr;	/* physical load addr */

	FASTBOOT_VPRINT("%s\n", __func__);

	if (download_has_error || download_size == 0 || download_addr == 0) {
		(void)tx_status_err("FAILno image downloaded");
		goto _exit;
	}

	pte = aboot_get_part_by_name("boot");
	if (!pte || (pte->flags & (ABOOT_PART_FLAGS_RESERVED | ABOOT_PART_FLAGS_HIDDEN))) {
		FASTBOOT_ERR("%.20s: part not found\n", cmdbuf);
		goto _internal_err;
	}

	if (pte->flags & (ABOOT_PART_FLAGS_LOCKED) && !aboot_board_is_unlocked()) {
		FASTBOOT_ERR("%.20s: part is locked\n", cmdbuf);
		goto _locked;
	}

	dev_idx = ABOOT_PART_GET_DEVICE(pte->flags) >> ABOOT_PART_FLAGS_DEVICE_SHIFT;
	type_idx = ABOOT_PART_GET_TYPE(pte->flags) >> ABOOT_PART_FLAGS_TYPE_SHIFT;
	if (dev_idx >= ABOOT_PART_FLAGS_DEVICE_MAX || type_idx >= ABOOT_PART_FLAGS_TYPE_MAX) {
		goto _internal_err;
	}

	handler = &(pte_handlers[dev_idx][type_idx]);
	if (handler->verify) {
		UART_FIFO_FLUSH_ONCE();

		FASTBOOT_PRINT("verifying for '%s' ...\n", pte->name);
		ret = handler->verify(handler->name, pte, (const u8*)download_addr, download_size);
		if (ret) {
			FASTBOOT_ERR("%s: verify err %d\n", handler->name, ret);
			goto _exit;
		}
	}

	memcpy(&aboot_header, (const void*)download_addr, sizeof(aboot_header));

	if (aboot_header.kernel_addr < (unsigned)download_addr &&
			aboot_header.kernel_addr >= (unsigned)(download_addr + download_size))
		goto _invalid_addr;

	if (aboot_header.ramdisk_addr < (unsigned)download_addr &&
			aboot_header.ramdisk_addr >= (unsigned)(download_addr + download_size))
		goto _invalid_addr;

	VIDEO_CLEAR();

	kernel_addr = ((unsigned)download_addr) + ALIGN(sizeof(aboot_header), aboot_header.page_size);
	ramdisk_addr = kernel_addr + ALIGN(aboot_header.kernel_size, aboot_header.page_size);

	PRINT("Moving Kernel@0x%08X -> 0x%08X (%u) ...\n",
			kernel_addr, aboot_header.kernel_addr, aboot_header.kernel_size);
	memmove((void*)aboot_header.kernel_addr, (void*)kernel_addr, aboot_header.kernel_size);

	PRINT("Moving Ramdisk@0x%08X -> 0x%08X (%u) ...\n", ramdisk_addr, aboot_header.ramdisk_addr, aboot_header.ramdisk_size);
	memmove((void*)aboot_header.ramdisk_addr, (void*)ramdisk_addr, aboot_header.ramdisk_size);

	(void)tx_status_complete("OKAY", cmd_complete_boot);

	goto _exit;

_invalid_addr:
	(void)tx_status_err("FAILincorrect address");
	goto _exit;
_locked:
	(void)tx_status_err("FAILBootloader Locked");
	goto _exit;
_internal_err:
	(void)tx_status_err("FAILinternal error");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

/*#define DEBUG_OEM_ARGS*/
#define MAX_OEM_ARGS	CONFIG_SYS_MAXARGS

/*-------------------------------------------------------------------------*/

struct cmd_oem_dispatch_info {
	char *cmd;
	unsigned int cmd_len;
	void (*func)(int argc, char* const argv[]);
};

static void cmd_complete_oem(struct usb_ep *ep, struct usb_request *req)
{
	if (!req_tx)
		return;

	if (req->status != 0)
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);
	else
		(void)tx_status("OKAY");

	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_SYS_MEMORY_POST
#include "icom/memory_post.h"

static int memory_post_test_databus(unsigned long start, unsigned long size)
{
	int ret = 0;
	unsigned long end, s;

	end = start + size - sizeof(unsigned long);

	for (s = 0 ; s < size ; s += SZ_4M) {
		start += s;
		if (start > end)
			break;

		ret = memory_post_databus((unsigned long *)start);
		if (ret)
			return ret;
	}

	if (start != end) {
		start = end;
		ret = memory_post_databus((unsigned long *)start);
	}

	return ret;
}

static int memory_post_test_dataline(unsigned long start, unsigned long size)
{
	int ret = 0;
	unsigned long end, s;

	end = start + size - 2 * sizeof(unsigned long long);

	for (s = 0 ; s < size ; s += SZ_4M) {
		start += s;
		if (start > end)
			break;

		ret = memory_post_dataline((unsigned long long *)start);
		if (ret)
			return ret;
	}

	if (start != end) {
		start = end;
		ret = memory_post_dataline((unsigned long long *)start);
	}

	return ret;
}

static int memory_post_test_addrline1(unsigned long start, unsigned long size)
{
	return memory_post_addrline((ulong *)start, (ulong *)start, size);
}

static int memory_post_test_addrline2(unsigned long start, unsigned long size)
{
	return memory_post_addrline((ulong *)(start+size-8), (ulong *)start, size);
}

static int memory_post_test1_pattern1(unsigned long start, unsigned long size)
{
	return memory_post_test1(start, size, 0x00000000);
}

static int memory_post_test1_pattern2(unsigned long start, unsigned long size)
{
	return memory_post_test1(start, size, 0xffffffff);
}

static int memory_post_test1_pattern3(unsigned long start, unsigned long size)
{
	return memory_post_test1(start, size, 0x55555555);
}

static int memory_post_test1_pattern4(unsigned long start, unsigned long size)
{
	return memory_post_test1(start, size, 0xaaaaaaaa);
}

/*-------------------------------------------------------------------------*/

static int memory_test_test1_pattern1(unsigned long start, unsigned long size)
{
	unsigned long i;
	int ret = 0;

	for (i = 0; i < (size >> 20) && (!ret); i++) {
		if (!ret)
			ret = memory_post_test1(start + (i << 20), 0x800, 0x00000000);
		WATCHDOG_RESET();
		if (!ret)
			ret = memory_post_test1(start + (i << 20), 0x800, 0xffffffff);
		WATCHDOG_RESET();
		if (!ret)
			ret = memory_post_test1(start + (i << 20) + 0xff800, 0x800, 0x00000000);
		WATCHDOG_RESET();
		if (!ret)
			ret = memory_post_test1(start + (i << 20) + 0xff800, 0x800, 0xffffffff);
		WATCHDOG_RESET();
	}

	return ret;
}

static int memory_test_test1_pattern2(unsigned long start, unsigned long size)
{
	unsigned long i;
	int ret = 0;

	for (i = 0; i < (size >> 20) && (!ret); i++) {
		if (!ret)
			ret = memory_post_test1(start + (i << 20), 0x800, 0x55555555);
		WATCHDOG_RESET();
		if (!ret)
			ret = memory_post_test1(start + (i << 20), 0x800, 0xaaaaaaaa);
		WATCHDOG_RESET();
		if (!ret)
			ret = memory_post_test1(start + (i << 20) + 0xff800, 0x800, 0x55555555);
		WATCHDOG_RESET();
		if (!ret)
			ret = memory_post_test1(start + (i << 20) + 0xff800, 0x800, 0xaaaaaaaa);
		WATCHDOG_RESET();
	}

	return ret;
}

static int memory_test_test2(unsigned long start, unsigned long size)
{
	unsigned long i;
	int ret = 0;

	for (i = 0; i < (size >> 20) && (!ret); i++) {
		if (!ret)
			ret = memory_post_test2(start + (i << 20), 0x800);
		if (!ret)
			ret = memory_post_test2(start + (i << 20) + 0xff800, 0x800);
	}

	return ret;
}

static int memory_test_test3(unsigned long start, unsigned long size)
{
	unsigned long i;
	int ret = 0;

	for (i = 0; i < (size >> 20) && (!ret); i++) {
		if (!ret)
			ret = memory_post_test3(start + (i << 20), 0x800);
		if (!ret)
			ret = memory_post_test3(start + (i << 20) + 0xff800, 0x800);
	}

	return ret;
}

static int memory_test_test4(unsigned long start, unsigned long size)
{
	unsigned long i;
	int ret = 0;

	for (i = 0; i < (size >> 20) && (!ret); i++) {
		if (!ret)
			ret = memory_post_test4(start + (i << 20), 0x800);
		if (!ret)
			ret = memory_post_test4(start + (i << 20) + 0xff800, 0x800);
	}

	return ret;
}

/*-------------------------------------------------------------------------*/

typedef int (*mem_test_func_t)(unsigned long start, unsigned long size);

struct memory_test {
	const char*	name;
	mem_test_func_t	test;
};

/*-------------------------------------------------------------------------*/

static u32 memory_addr_start;
static u32 memory_addr_size;
static struct memory_test *memory_test_case;
static struct memory_test *memory_curr_test;
static unsigned int memory_iterations;
static unsigned int memory_iteration_limit;
static unsigned int memory_errors;

static struct memory_test memory_quickie_tests[] = {
	{ "databus",	memory_post_test_databus, },
	{ "dataline",	memory_post_test_dataline, },
	{ "addrline1",	memory_post_test_addrline1, },
	{ "addrline2",	memory_post_test_addrline2, },
	{ "addrline3",	memory_post_addrline2, },
	{ "test1_pattern1",	memory_test_test1_pattern1, },
	{ "test1_pattern2",	memory_test_test1_pattern2, },
	{ "test2",	memory_test_test2, },
	{ "test3",	memory_test_test3, },
	{ "test4",	memory_test_test4, },
	{ NULL,	NULL },
};

static struct memory_test memory_full_tests[] = {
	{ "databus",	memory_post_test_databus, },
	{ "dataline",	memory_post_test_dataline, },
	{ "addrline1",	memory_post_test_addrline1, },
	{ "addrline2",	memory_post_test_addrline2, },
	{ "addrline3",	memory_post_addrline2, },
	{ "test1_pattern1",	memory_post_test1_pattern1, },
	{ "test1_pattern2",	memory_post_test1_pattern2, },
	{ "test1_pattern3",	memory_post_test1_pattern3, },
	{ "test1_pattern4",	memory_post_test1_pattern4, },
	{ "test2",	memory_post_test2, },
	{ "test3",	memory_post_test3, },
	{ "test4",	memory_post_test4, },
	{ NULL,	NULL },
};

/*-------------------------------------------------------------------------*/

static void cmd_complete_memory_post(struct usb_ep *ep, struct usb_request *req);

static void __fastboot_memory_test_prepare(u32 *start, u32 *size)
{
	*start = CONFIG_FASTBOOT_BUFFER;
	*size = CONFIG_FASTBOOT_BUFFER_SIZE;
}

void fastboot_memory_test_prepare(u32 *start, u32 *size)
	__attribute__((weak, alias("__fastboot_memory_test_prepare")));

/*
 * print sizes as "xxx KiB", "xxx.y KiB", "xxx MiB", "xxx.y MiB",
 * xxx GiB, xxx.y GiB, etc as needed; allow for optional trailing string
 * (like "\n")
 */
static void snprint_size(char *buf, size_t s, unsigned long long size)
{
	unsigned long m = 0, n;
	unsigned long long f;
	static const char names[] = {'E', 'P', 'T', 'G', 'M', 'K'};
	unsigned long d = 10 * ARRAY_SIZE(names);
	char c = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(names); i++, d -= 10) {
		if (size >> d) {
			c = names[i];
			break;
		}
	}

	if (!c) {
		snprintf(buf, s, "%llu Bytes", size);
		return;
	}

	n = size >> d;
	f = size & ((1ULL << d) - 1);

	/* If there's a remainder, deal with it */
	if (f) {
		m = (10ULL * f + (1ULL << (d - 1))) >> d;

		if (m >= 10) {
			m -= 10;
			n += 1;
		}
	}

	if (m)
		snprintf(buf, s, "%lu.%ld %cB", n, m, c);
	else
		snprintf(buf, s, "%lu %cB", n, c);
}

static void memory_post_log(const char* fmt, ...)
{
	va_list args;
	char printbuffer[CONFIG_SYS_PBSIZE];

	va_start(args, fmt);

	/* For this to work, printbuffer must be larger than
	 * anything we ever want to print.
	 */
	(void)vscnprintf(printbuffer, sizeof(printbuffer), fmt, args);
	va_end(args);

	/* Print the string */
	puts(printbuffer);
	VIDEO_PUTS(0, printbuffer);
}

static void memory_test_done(int result)
{
	if (result == 0)
		memory_post_log("\nMemory Test Result: PASS\n\n");
	else if (result > 0)
		memory_post_log("\nMemory Test Result: FAIL\n\n");
	else
		memory_post_log("\nMemory Test Not Completed!\n\n");

	memory_post_exit();
}

static void memory_perform_next_test(void)
{
	char response[65];
	int ret = 0;

	if (!memory_curr_test)
		goto _internal_err;

	if (!memory_curr_test->name || !memory_curr_test->test) {
		if (memory_iterations < memory_iteration_limit) {
			memory_curr_test = memory_test_case;
		} else {
			if (memory_iteration_limit) {
				memory_post_log("\nTested %u iteration(s) with %u testcase errors.!\n",
					memory_iterations, memory_errors);
			}

			if (memory_errors != 0) {
				(void)tx_status_err("FAILmemory error!");
				memory_test_done(1);
			} else {
				(void)tx_status("OKAY");
				memory_test_done(0);
			}
			goto _exit;
		}
	}

	if (memory_curr_test == memory_test_case) {
		memory_iterations++;
		if (memory_iteration_limit) {
			if (memory_errors)
				memory_post_log("Test Iteration: %u / %u (%u testcase errors)\n",
					memory_iterations, memory_iteration_limit, memory_errors);
			else
				memory_post_log("Test Iteration: %u / %u\n",
					memory_iterations, memory_iteration_limit);
		}
	}

	snprintf(response, 64, "INFOtest case: %s (0x%08x-0x%08x)",
		memory_curr_test->name, memory_addr_start, memory_addr_start + (memory_addr_size - 1));

	ret = tx_status_complete(response, cmd_complete_memory_post);
	if (ret)
		goto _internal_err;

	return;

_internal_err:
	(void)tx_status_err("FAILinternal error");
	memory_test_done(-1);
	goto _exit;
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_complete_memory_start(struct usb_ep *ep, struct usb_request *req)
{
	if (!req_tx)
		return;

	if (req->status != 0) {
		FASTBOOT_ERR("%s: status %d\n", __func__, req->status);
		memory_test_done(-1);
		goto _exit;
	}

	if (!memory_curr_test)
		goto _internal_err;

	memory_perform_next_test();

	return;

_internal_err:
	(void)tx_status_err("FAILinternal error");
	memory_test_done(-1);
	goto _exit;
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void memory_test_start(int quickie)
{
	char response[65];
	char buffer[16];
	int ret = 0;

	FASTBOOT_VPRINT("%s\n", __func__);

	snprint_size(buffer, 16, gd->ram_size);
	if (quickie) {
		memory_post_log("\nQuickie Memory Testing (DRAM: %s) ...\n", buffer);
		snprintf(response, 64, "INFOQuickie Memory Testing: %s", buffer);
		memory_test_case = memory_quickie_tests;
	} else {
		memory_post_log("\nFull Memory Testing (DRAM: %s) ...\n", buffer);
		snprintf(response, 64, "INFOFull Memory Testing: %s", buffer);
		memory_test_case = memory_full_tests;
	}
	memory_curr_test = memory_test_case;

	memory_post_init(memory_post_log);

	ret = tx_status_complete(response, cmd_complete_memory_start);
	if (ret)
		goto _internal_err;

	return;

_internal_err:
	(void)tx_status_err("FAILinternal error");
	memory_test_done(-1);
	goto _exit;
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_complete_memory_post(struct usb_ep *ep, struct usb_request *req)
{
	int ret = 0;

	if (!req_tx)
		return;

	if (req->status != 0) {
		FASTBOOT_ERR("%s: status %d\n", __func__, req->status);
		memory_test_done(-1);
		goto _exit;
	}

	if (!memory_curr_test)
		goto _internal_err;

	memory_post_log("Memory Test Case: %s (0x%08x-0x%08x)\n",
		memory_curr_test->name, memory_addr_start, memory_addr_start + (memory_addr_size - 1));
	ret = memory_curr_test->test(memory_addr_start, memory_addr_size);
	if (ret) {
		memory_errors++;
		if (memory_iteration_limit == 0) {
			(void)tx_status_err("FAILmemory error!");
			memory_test_done(1);
			goto _exit;
		}
	}

	memory_curr_test++;
	memory_perform_next_test();

	return;

_internal_err:
	(void)tx_status_err("FAILinternal error");
	memory_test_done(-1);
	goto _exit;
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_mempost(int argc, char* const argv[])
{
	u32 iterations_limit = (~0U); /* UINT_MAX */

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = mempost
	 * argv[1] = <iterations_limit>
	 */
	if (argc > 2)
		goto _invalid_arguments;

	if (argc == 2) {
		iterations_limit = (u32)simple_strtoul(argv[1], NULL, 10);
		if (iterations_limit == 0)
			goto _invalid_arguments;
	}

	fastboot_memory_test_prepare(&memory_addr_start, &memory_addr_size);

	memory_iterations = 0;
	memory_iteration_limit = iterations_limit;
	memory_errors = 0;

	memory_test_start(0);

	return;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_memtest(int argc, char* const argv[])
{
	u32 iterations_limit = 0;

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = memtest
	 * argv[1] = <iterations_limit>
	 */
	if (argc > 2)
		goto _invalid_arguments;

	if (argc == 2) {
		iterations_limit = (u32)simple_strtoul(argv[1], NULL, 10);
		if (iterations_limit == 0)
			goto _invalid_arguments;
	}

	fastboot_memory_test_prepare(&memory_addr_start, &memory_addr_size);

	memory_iterations = 0;
	memory_iteration_limit = iterations_limit;
	memory_errors = 0;

	memory_test_start(1);

	return;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}
#endif /* CONFIG_SYS_MEMORY_POST */

/*-------------------------------------------------------------------------*/

/* oem gpio XXX */

static void cmd_oem_gpio_clear(int argc, char* const argv[])
{
	/* oem gpio clear */
	unsigned gpio;

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = gpio
	 * argv[1] = clear
	 */
	if (argc != 3)
		goto _invalid_arguments;

	gpio = simple_strtoul(argv[2], NULL, 10);
	FASTBOOT_PRINT("gpio clear: %u\n", gpio);
#if 0
	if (gpio_request(gpio, NULL))
		goto _invalid_arguments;
#endif
	if (gpio_direction_output(gpio, 0))
		goto _invalid_arguments;
#if 0
	gpio_free(gpio);
#endif

	(void)tx_status("OKAY");
	goto _exit;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_gpio_set(int argc, char* const argv[])
{
	/* oem gpio set */
	unsigned gpio;

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = gpio
	 * argv[1] = set
	 */
	if (argc != 3)
		goto _invalid_arguments;

	gpio = simple_strtoul(argv[2], NULL, 10);
	FASTBOOT_PRINT("gpio set: %u\n", gpio);
#if 0
	if (gpio_request(gpio, NULL))
		goto _invalid_arguments;
#endif
	if (gpio_direction_output(gpio, 1))
		goto _invalid_arguments;
#if 0
	gpio_free(gpio);
#endif

	(void)tx_status("OKAY");
	goto _exit;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_gpio_input(int argc, char* const argv[])
{
	/* oem gpio input */
	unsigned gpio;
	int val;
	char response[16];

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = gpio
	 * argv[1] = input
	 */
	if (argc != 3)
		goto _invalid_arguments;

	gpio = simple_strtoul(argv[2], NULL, 10);
	FASTBOOT_PRINT("gpio input: %u\n", gpio);
#if 0
	if (gpio_request(gpio, NULL))
		goto _invalid_arguments;
#endif
	if (gpio_direction_input(gpio))
		goto _invalid_arguments;

	val = gpio_get_value(gpio);
	if (val < 0)
		goto _invalid_arguments;
#if 0
	gpio_free(gpio);
#endif

	sprintf(response, "INFO%u", val);
	(void)tx_status_complete(response, cmd_complete_oem);
	return;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
//_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static struct cmd_oem_dispatch_info cmd_oem_gpio_dispatch_info[] = {
	{
		.cmd		= "input",
		.cmd_len	= 6, /* including NULL terminator */
		.func		= cmd_oem_gpio_input,
	},
	{
		.cmd		= "set",
		.cmd_len	= 4, /* including NULL terminator */
		.func		= cmd_oem_gpio_set,
	},
	{
		.cmd		= "clear",
		.cmd_len	= 6, /* including NULL terminator */
		.func		= cmd_oem_gpio_clear,
	},
};

static void cmd_oem_gpio(int argc, char* const argv[])
{
	/* oem gpio */
	int i;
	void (*func)(int argc, char* const argv[]) = NULL;

	FASTBOOT_VPRINT("%s\n", __func__);

	if (!board_is_factory_mode())
		goto _invalid_oem_cmd;

	/**
	 * argv[0] = gpio
	 */
	if (argc < 3)
		goto _invalid_arguments;

	for (i = 0; i < ARRAY_SIZE(cmd_oem_gpio_dispatch_info); i++) {
		if (!memcmp(argv[1], cmd_oem_gpio_dispatch_info[i].cmd, cmd_oem_gpio_dispatch_info[i].cmd_len)) {
			func = cmd_oem_gpio_dispatch_info[i].func;
			break;
		}
	}

	if (func) {
		func(argc, argv);
		return;
	}

	FASTBOOT_VPRINT("%s: invalid oem command: \"%.32s\"\n", __func__, argv[0]);
	goto _invalid_oem_cmd;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
	goto _exit;
_invalid_oem_cmd:
	(void)tx_status_err("FAILinvalid oem command");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

/* oem reg XXX */

static void cmd_oem_reg_readb(int argc, char* const argv[])
{
	/* oem reg readb */
	u32 reg;
	u8 val;
	char response[16];

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = reg
	 * argv[1] = readb
	 */
	if (argc != 3)
		goto _invalid_arguments;

	reg = simple_strtoul(argv[2], NULL, 16);
	FASTBOOT_PRINT("reg readb: reg=0x%08x\n", reg);
	if (!reg)
		goto _invalid_arguments;

	val = __raw_readb(reg);
	sprintf(response, "INFO0x%02x", val);
	(void)tx_status_complete(response, cmd_complete_oem);
	return;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
//_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_reg_readw(int argc, char* const argv[])
{
	/* oem reg readw */
	u32 reg;
	u16 val;
	char response[16];

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = reg
	 * argv[1] = readw
	 */
	if (argc != 3)
		goto _invalid_arguments;

	reg = simple_strtoul(argv[2], NULL, 16);
	FASTBOOT_PRINT("reg readw: reg=0x%08x\n", reg);
	if (!reg)
		goto _invalid_arguments;

	val = __raw_readw(reg);
	sprintf(response, "INFO0x%04x", val);
	(void)tx_status_complete(response, cmd_complete_oem);
	return;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
//_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_reg_readl(int argc, char* const argv[])
{
	/* oem reg readl */
	u32 reg;
	u32 val;
	char response[16];

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = reg
	 * argv[1] = readl
	 */
	if (argc != 3)
		goto _invalid_arguments;

	reg = simple_strtoul(argv[2], NULL, 16);
	FASTBOOT_PRINT("reg readl: reg=0x%08x\n", reg);
	if (!reg)
		goto _invalid_arguments;

	val = __raw_readl(reg);
	sprintf(response, "INFO0x%08x", val);
	(void)tx_status_complete(response, cmd_complete_oem);
	return;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
//_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_reg_writeb(int argc, char* const argv[])
{
	/* oem reg writeb */
	u32 reg, val;
	u8 value;

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = reg
	 * argv[1] = writeb
	 */
	if (argc != 4)
		goto _invalid_arguments;

	reg = simple_strtoul(argv[2], NULL, 16);
	val = simple_strtoul(argv[3], NULL, 16);
	value = (u8)val;
	FASTBOOT_PRINT("reg writeb: reg=0x%08x, value=0x%02x(0x%08x)\n", reg, value, val);
	if (!reg || val > 0x0FF)
		goto _invalid_arguments;

	__raw_writeb(value, reg);

	(void)tx_status("OKAY");
	goto _exit;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_reg_writew(int argc, char* const argv[])
{
	/* oem reg writew */
	u32 reg, val;
	u16 value;

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = reg
	 * argv[1] = writew
	 */
	if (argc != 4)
		goto _invalid_arguments;

	reg = simple_strtoul(argv[2], NULL, 16);
	val = simple_strtoul(argv[3], NULL, 16);
	value = (u16)val;
	FASTBOOT_PRINT("reg writew: reg=0x%08x, value=0x%04x(0x%08x)\n", reg, value, val);
	if (!reg || val > 0x0FFFF)
		goto _invalid_arguments;

	__raw_writew(value, reg);

	(void)tx_status("OKAY");
	goto _exit;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_reg_writel(int argc, char* const argv[])
{
	/* oem reg writel */
	u32 reg, val;

	FASTBOOT_VPRINT("%s\n", __func__);

	/**
	 * argv[0] = reg
	 * argv[1] = writel
	 */
	if (argc != 4)
		goto _invalid_arguments;

	reg = simple_strtoul(argv[2], NULL, 16);
	val = simple_strtoul(argv[3], NULL, 16);
	FASTBOOT_PRINT("reg writel: reg=0x%08x, value=0x%08x\n", reg, val);
	if (!reg)
		goto _invalid_arguments;

	__raw_writel(val, reg);

	(void)tx_status("OKAY");
	goto _exit;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static struct cmd_oem_dispatch_info cmd_oem_reg_dispatch_info[] = {
	{
		.cmd		= "writel",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_oem_reg_writel,
	},
	{
		.cmd		= "readl",
		.cmd_len	= 6, /* including NULL terminator */
		.func		= cmd_oem_reg_readl,
	},
	{
		.cmd		= "writew",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_oem_reg_writew,
	},
	{
		.cmd		= "readw",
		.cmd_len	= 6, /* including NULL terminator */
		.func		= cmd_oem_reg_readw,
	},
	{
		.cmd		= "writeb",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_oem_reg_writeb,
	},
	{
		.cmd		= "readb",
		.cmd_len	= 6, /* including NULL terminator */
		.func		= cmd_oem_reg_readb,
	},
};

static void cmd_oem_reg(int argc, char* const argv[])
{
	/* oem reg */
	int i;
	void (*func)(int argc, char* const argv[]) = NULL;

	FASTBOOT_VPRINT("%s\n", __func__);

	if (!board_is_factory_mode())
		goto _invalid_oem_cmd;

	/**
	 * argv[0] = reg
	 */
	if (argc < 3)
		goto _invalid_arguments;

	for (i = 0; i < ARRAY_SIZE(cmd_oem_reg_dispatch_info); i++) {
		if (!memcmp(argv[1], cmd_oem_reg_dispatch_info[i].cmd, cmd_oem_reg_dispatch_info[i].cmd_len)) {
			func = cmd_oem_reg_dispatch_info[i].func;
			break;
		}
	}

	if (func) {
		func(argc, argv);
		return;
	}

	FASTBOOT_VPRINT("%s: invalid oem command: \"%.32s\"\n", __func__, argv[0]);
	goto _invalid_oem_cmd;

_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
	goto _exit;
_invalid_oem_cmd:
	(void)tx_status_err("FAILinvalid oem command");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE)
extern int _do_env_set(int flag, int argc, char * const argv[]);

static void cmd_oem_setenv(int argc, char* const argv[])
{
	int ret;

	FASTBOOT_VPRINT("%s\n", __func__);

#ifndef CONFIG_FASTBOOT_SETVAR_NO_PERMISSION_CHECK
	if (!board_is_factory_mode())
		goto _invalid_oem_cmd;
#endif

	if (argc < 2 || argc > 3)
		goto _invalid_arguments;

	FASTBOOT_PRINT("perform setvar: %s=%s\n", argv[1],
			(argc == 2) ? "" : (argv[2] ? argv[2] : ""));

	ret = _do_env_set(0, argc, argv);
	if (ret)
		goto _internal_err;

	(void)tx_status("OKAY");
	goto _exit;

_internal_err:
	(void)tx_status_err("FAILinternal error");
	goto _exit;
_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
	goto _exit;
#ifndef CONFIG_FASTBOOT_SETVAR_NO_PERMISSION_CHECK
_invalid_oem_cmd:
	(void)tx_status_err("FAILinvalid oem command");
#endif
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}
#endif /* defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE) */

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE)
static void cmd_oem_saveenv(int argc, char* const argv[])
{
	int ret;

	FASTBOOT_VPRINT("%s\n", __func__);

#ifndef CONFIG_FASTBOOT_SETVAR_NO_PERMISSION_CHECK
	if (!board_is_factory_mode())
		goto _invalid_oem_cmd;
#endif

	if (argc > 1)
		goto _invalid_arguments;

	FASTBOOT_PRINT("perform savevar\n");
	ret = saveenv();
	if (ret)
		goto _internal_err;

	(void)tx_status("OKAY");
	goto _exit;

_internal_err:
	(void)tx_status_err("FAILinternal error");
	goto _exit;
_invalid_arguments:
	(void)tx_status_err("FAILinvalid arguments");
	goto _exit;
#ifndef CONFIG_FASTBOOT_SETVAR_NO_PERMISSION_CHECK
_invalid_oem_cmd:
	(void)tx_status_err("FAILinvalid oem command");
#endif
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}
#endif /* defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE) */

/*-------------------------------------------------------------------------*/

static void cmd_oem_format(int argc, char* const argv[])
{
	/* Dummy "oem format" command */

	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status("OKAY");
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_complete_oem_powerdown(struct usb_ep *ep, struct usb_request *req)
{
	if (req_tx && req->status != 0)
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);

	fastboot_exit_reason = FASTBOOT_EXIT_SHUTDOWN;
}

static void cmd_oem_powerdown(int argc, char* const argv[])
{
	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status_complete("OKAY", cmd_complete_oem_powerdown);
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_unlock(int argc, char* const argv[])
{
	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status_err("FAIL");
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

static void cmd_oem_reset_bat_capacity(int argc, char* const argv[])
{
#ifdef CONFIG_OMAP44XX
	extern int fastboot_reset_battery_capacity(void);
	int ret;

	FASTBOOT_VPRINT("%s\n", __func__);

	ret = fastboot_reset_battery_capacity();
	if (ret) {
		FASTBOOT_VPRINT("%s: return %d\n", __func__, ret);
		(void)tx_status_err("FAILinvalid oem command");
	} else
		(void)tx_status("OKAY");
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
#else
	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status_err("FAIL");
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
#endif
}

/*-------------------------------------------------------------------------*/

static struct cmd_oem_dispatch_info cmd_oem_dispatch_info[] = {
	{
		.cmd		= "format",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_oem_format,
	},
	{
		.cmd		= "powerdown",
		.cmd_len	= 10, /* including NULL terminator */
		.func		= cmd_oem_powerdown,
	},
	{
		.cmd		= "unlock",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_oem_unlock,
	},
#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE)
	{
		.cmd		= "setvar",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_oem_setenv,
	},
	{
		.cmd		= "savevar",
		.cmd_len	= 8, /* including NULL terminator */
		.func		= cmd_oem_saveenv,
	},
#endif /* defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE) */
	{
		.cmd		= "reg",
		.cmd_len	= 4, /* including NULL terminator */
		.func		= cmd_oem_reg,
	},
	{
		.cmd		= "gpio",
		.cmd_len	= 5, /* including NULL terminator */
		.func		= cmd_oem_gpio,
	},
#ifdef CONFIG_SYS_MEMORY_POST
	{
		.cmd		= "memtest", /* quickie memory test */
		.cmd_len	= 8, /* including NULL terminator */
		.func		= cmd_oem_memtest,
	},
	{
		.cmd		= "mempost", /* full memory test */
		.cmd_len	= 8, /* including NULL terminator */
		.func		= cmd_oem_mempost,
	},
#endif /* CONFIG_SYS_MEMORY_POST */
	{
		.cmd		= "reset_bat_capacity", /* reset the battery capacity */
		.cmd_len	= 19, /* including NULL terminator */
		.func		= cmd_oem_reset_bat_capacity,
	},
};

/*-------------------------------------------------------------------------*/

static void cmd_oem(const char* cmdbuf)
{
	char *argv[CONFIG_SYS_MAXARGS + 1];	/* NULL terminated	*/
	int argc, i;
	void (*func)(int argc, char* const argv[]) = NULL;

	FASTBOOT_VPRINT("%s\n", __func__);

	/* Extract arguments */
	if ((argc = parse_line ((char*)cmdbuf, argv)) == 0)
		goto _internal_err;

	if (argc > MAX_OEM_ARGS)
		goto _too_many_arguments;

#ifdef DEBUG_OEM_ARGS
	FASTBOOT_PRINT("argc %d\n", argc);
	for (i = 0; i < argc; i++)
		FASTBOOT_PRINT("argv[%d] = %s\n", i, argv[i]);
#endif /* DEBUG_OEM_ARGS */

	for (i = 0; i < ARRAY_SIZE(cmd_oem_dispatch_info); i++) {
		if (!memcmp(argv[0], cmd_oem_dispatch_info[i].cmd, cmd_oem_dispatch_info[i].cmd_len)) {
			func = cmd_oem_dispatch_info[i].func;
			break;
		}
	}

	if (func) {
		func(argc, argv);
		return;
	}

	FASTBOOT_VPRINT("%s: invalid oem command: \"%.32s\"\n", __func__, argv[0]);
	(void)tx_status_err("FAILinvalid oem command");
	goto _exit;

_too_many_arguments:
	(void)tx_status_err("FAILtoo many arguments");
	goto _exit;
_internal_err:
	(void)tx_status_err("FAILinternal error");
_exit:
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static void cmd_getvar_version(char *response)
{
	sprintf(response, "%s", FASTBOOT_VERSION);
}

static void cmd_getvar_version_bootloader(char *response)
{
	sprintf(response, "%s", CONFIG_SYS_IBOOT_VERSION);
}

static void cmd_getvar_version_iboot(char *response)
{
	sprintf(response, "%s", PLAIN_VERSION);
}

static void cmd_getvar_product(char *response)
{
	sprintf(response, "%s", CONFIG_SYS_BOARD_NAME);
}

static void cmd_getvar_serialno(char *response)
{
	sprintf(response, "%s", device_usb_fb_strings[FASTBOOT_STR_SERIAL_IDX].s);
}

static void cmd_getvar_cpurev(char *response)
{
	sprintf(response, "%s", device_usb_fb_strings[FASTBOOT_STR_PROC_REV_IDX].s);
}

static void cmd_getvar_secure(char *response)
{
	sprintf(response, "%s", device_usb_fb_strings[FASTBOOT_STR_PROC_TYPE_IDX].s);
}

static void cmd_getvar_cpu(char *response)
{
	sprintf(response, "%s", device_usb_fb_strings[FASTBOOT_STR_PROC_VERSION_IDX].s);
}

static void cmd_getvar_userdata_size(char *response)
{
	u64 part_size = 0;
	aboot_ptentry *pte;

	pte = aboot_get_part_by_name("userdata");
	if (pte) {
		part_size = ((u64)pte->part_blk_size) * pte->blk_size;
		part_size /= 1024; /* KB */
	} else {
		FASTBOOT_ERR("userdata part not found\n");
	}

	sprintf(response, "%llu KB", part_size);
}

#ifdef CONFIG_SYS_POWER_SUPPLY
static void cmd_getvar_backup_vbat(char *response)
{
	sprintf(response, "%d mV", power_supply_get_backup_battery_voltage());
}

static void cmd_getvar_vbat(char *response)
{
	sprintf(response, "%d mV", power_supply_get_battery_voltage());
}

static void cmd_getvar_vsys(char *response)
{
	sprintf(response, "%d mV", power_supply_get_sys_voltage());
}

static void cmd_getvar_vbus(char *response)
{
	sprintf(response, "%d mV", power_supply_get_usb_voltage());
}

static void cmd_getvar_vac(char *response)
{
	sprintf(response, "%d mV", power_supply_get_ac_voltage());
}

static void cmd_getvar_bat_current(char *response)
{
	int currentmA = power_supply_get_battery_current();
	if (currentmA != CONFIG_POWER_SUPPLY_INVALID_CURRENT)
		sprintf(response, "%d mA", currentmA);
	else
		sprintf(response, "N/A");
}

static void cmd_getvar_bat_capacity(char *response)
{
	sprintf(response, "%d %%", power_supply_get_battery_capacity());
}

static void cmd_getvar_bat_status(char *response)
{
	if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
		int charging = power_supply_is_charging();
		if (power_supply_is_full_charged()) {
			if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING)
				sprintf(response, "Full Charged (Charging)");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_EOC)
				sprintf(response, "Full Charged (END of Charging)");
			else
				sprintf(response, "Full Charged (NOT Charging)");
		} else {
			if (charging == POWER_SUPPLY_CHARGER_STATE_CHARGING)
				sprintf(response, "Charging");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_EOC)
				sprintf(response, "END of Charging");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_NO_BATTERY)
				sprintf(response, "No Battery");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_TEMP_FAULT)
				sprintf(response, "Battery Temp Fault");
			else if (charging == POWER_SUPPLY_CHARGER_STATE_BATTERY_OVP)
				sprintf(response, "Battery OVP");
			else
				sprintf(response, "NOT Charging");
		}
	} else
		sprintf(response, "Discharge");
}
#endif /* CONFIG_SYS_POWER_SUPPLY */

/*-------------------------------------------------------------------------*/

struct cmd_getvar_dispatch_info {
	char *cmd;
	unsigned int cmd_len;
	void (*func)(char *response);
};

static struct cmd_getvar_dispatch_info cmd_getvar_dispatch_info[] = {
	{
		.cmd		= "version-bootloader",
		.cmd_len	= 19, /* including NULL terminator */
		.func		= cmd_getvar_version_bootloader,
	},
	{
		.cmd		= "version-iboot",
		.cmd_len	= 14, /* including NULL terminator */
		.func		= cmd_getvar_version_iboot,
	},
	{
		.cmd		= "version",
		.cmd_len	= 8, /* including NULL terminator */
		.func		= cmd_getvar_version,
	},
	{
		.cmd		= "product",
		.cmd_len	= 8, /* including NULL terminator */
		.func		= cmd_getvar_product,
	},
	{
		.cmd		= "serialno",
		.cmd_len	= 9, /* including NULL terminator */
		.func		= cmd_getvar_serialno,
	},
	{
		.cmd		= "cpurev",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_getvar_cpurev,
	},
	{
		.cmd		= "secure",
		.cmd_len	= 7, /* including NULL terminator */
		.func		= cmd_getvar_secure,
	},
	{
		.cmd		= "cpu",
		.cmd_len	= 4, /* including NULL terminator */
		.func		= cmd_getvar_cpu,
	},
	{
		.cmd		= "userdata_size",
		.cmd_len	= 14, /* including NULL terminator */
		.func		= cmd_getvar_userdata_size,
	},
#ifdef CONFIG_SYS_POWER_SUPPLY
	{
		.cmd		= "vbat",
		.cmd_len	= 5, /* including NULL terminator */
		.func		= cmd_getvar_vbat,
	},
	{
		.cmd		= "vsys",
		.cmd_len	= 5, /* including NULL terminator */
		.func		= cmd_getvar_vsys,
	},
	{
		.cmd		= "vbus",
		.cmd_len	= 5, /* including NULL terminator */
		.func		= cmd_getvar_vbus,
	},
	{
		.cmd		= "vac",
		.cmd_len	= 4, /* including NULL terminator */
		.func		= cmd_getvar_vac,
	},
	{
		.cmd		= "bat-current",
		.cmd_len	= 12, /* including NULL terminator */
		.func		= cmd_getvar_bat_current,
	},
	{
		.cmd		= "bat-capacity",
		.cmd_len	= 13, /* including NULL terminator */
		.func		= cmd_getvar_bat_capacity
	},
	{
		.cmd		= "bat-status",
		.cmd_len	= 11, /* including NULL terminator */
		.func		= cmd_getvar_bat_status,
	},
	{
		.cmd		= "backup-vbat",
		.cmd_len	= 12, /* including NULL terminator */
		.func		= cmd_getvar_backup_vbat,
	},
#endif /* CONFIG_SYS_POWER_SUPPLY */
};

/*-------------------------------------------------------------------------*/

static void cmd_getvar(const char* cmdbuf)
{
	int i;
	char response[513];
	void (*func)(char *response) = NULL;

	FASTBOOT_VPRINT("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(cmd_getvar_dispatch_info); i++) {
		if (!memcmp(cmdbuf, cmd_getvar_dispatch_info[i].cmd, cmd_getvar_dispatch_info[i].cmd_len)) {
			func = cmd_getvar_dispatch_info[i].func;
			break;
		}
	}

	strcpy(response, "OKAY");

	if (func) {
		func(response + strlen(response));
	} else {
#if defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE)
		char *value = getenv(cmdbuf);
		if (value)
			sprintf((response + strlen(response)), "%s", value);
#endif /* defined(CONFIG_CMD_SAVEENV) && !defined(CONFIG_ENV_IS_NOWHERE) */
	}

	(void)tx_status(response);
	UART_FIFO_FLUSH_ONCE();
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static void cmd_complete_reboot_bootloader(struct usb_ep *ep, struct usb_request *req)
{
	if (req_tx && req->status != 0)
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);

	fastboot_exit_reason = FASTBOOT_EXIT_REBOOT_BOOTLOADER;
}

static void cmd_reboot_bootloader(const char* cmdbuf)
{
	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status_complete("OKAY", cmd_complete_reboot_bootloader);
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static void cmd_complete_reboot(struct usb_ep *ep, struct usb_request *req)
{
	if (req_tx && req->status != 0)
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);

	fastboot_exit_reason = FASTBOOT_EXIT_REBOOT;
}

static void cmd_reboot(const char* cmdbuf)
{
	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status_complete("OKAY", cmd_complete_reboot);
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static void cmd_complete_continue(struct usb_ep *ep, struct usb_request *req)
{
	if (req_tx && req->status != 0)
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);

	fastboot_exit_reason = FASTBOOT_EXIT_CONTINUE;
}

static void cmd_continue(const char* cmdbuf)
{
	FASTBOOT_VPRINT("%s\n", __func__);

	(void)tx_status_complete("OKAY", cmd_complete_continue);
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

struct cmd_dispatch_info {
	char *cmd;
	unsigned int cmd_len;
	void (*func)(const char* cmdbuf);
};

static struct cmd_dispatch_info cmd_dispatch_info[] = {
	{
		.cmd		= "getvar:",
		.cmd_len	= 7,
		.func		= cmd_getvar,
	},
	{
		.cmd		= "download:",
		.cmd_len	= 9,
		.func		= cmd_download,
	},
	{
		.cmd		= "flash:",
		.cmd_len	= 6,
		.func		= cmd_flash,
	},
	{
		.cmd		= "erase:",
		.cmd_len	= 6,
		.func		= cmd_erase,
	},
	{
		.cmd		= "boot",
		.cmd_len	= 4,
		.func		= cmd_boot,
	},
	{
		.cmd		= "continue",
		.cmd_len	= 8,
		.func		= cmd_continue,
	},
	{
		.cmd		= "reboot-bootloader",
		.cmd_len	= 17,
		.func		= cmd_reboot_bootloader,
	},
	{
		.cmd		= "reboot",
		.cmd_len	= 6,
		.func		= cmd_reboot,
	},
	{
		.cmd		= "oem",
		.cmd_len	= 3,
		.func		= cmd_oem,
	},
};

static void rx_cmd_complete(struct usb_ep *ep, struct usb_request *req)
{
	char *cmdbuf = req->buf;
	void (*func)(const char* cmdbuf) = NULL;
	int i;

	if (!req_rx)
		return;

	if (req->status != 0) {
		FASTBOOT_DPRINT("%s: status %d\n", __func__, req->status);
		return;
	}

	if (req->actual > (EP_BUFFER_SIZE - 1))
		req->actual = EP_BUFFER_SIZE - 1;
	cmdbuf[req->actual] = 0;

	for (i = 0; i < ARRAY_SIZE(cmd_dispatch_info); i++) {
		if (!memcmp(cmdbuf, cmd_dispatch_info[i].cmd, cmd_dispatch_info[i].cmd_len)) {
			func = cmd_dispatch_info[i].func;
			break;
		}
	}

	if (func) {
		FASTBOOT_VPRINT("%s: \"%s\" >>>\n", __func__, cmdbuf);
		func(cmdbuf + cmd_dispatch_info[i].cmd_len);
		FASTBOOT_VPRINT("%s: \"%s\" <<<\n", __func__, cmdbuf);
		return;
	}

	FASTBOOT_VPRINT("%s: invalid command: \"%.32s\"\n", __func__, cmdbuf);
	(void)tx_status_err("FAILinvalid command");
	rx_cmd();
}

/*-------------------------------------------------------------------------*/

static int fastboot_ep0_queue(struct usb_gadget *gadget)
{
	int	rc;

	if (!ep0_req)
		return -EPERM;

	rc = usb_ep_queue(gadget->ep0, ep0_req, 0);
	if (rc != 0 && rc != -ESHUTDOWN)
		FASTBOOT_ERR("queue %s ep0 err %d\n", gadget->ep0->name, rc);

	return rc;
}

static void fastboot_ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status)
		FASTBOOT_DPRINT("ep0 status: %d\n", req->status);
}

static int fastboot_bind(struct usb_gadget *gadget)
{
	int ret = 0;

	g = gadget;
	ep0_req = usb_ep_alloc_request(g->ep0, 0);
	if (!ep0_req) {
		ret = -ENOMEM;
		goto err;
	}
	ep0_req->buf = ep0_buffer;
	ep0_req->complete = fastboot_ep0_complete;

	ep_in = usb_ep_autoconfig(gadget, &fs_ep_in);
	if (!ep_in) {
		ret = -ENODEV;
		goto err;
	}
	ep_in->driver_data = ep_in;

	ep_out = usb_ep_autoconfig(gadget, &fs_ep_out);
	if (!ep_out) {
		ret = -ENODEV;
		goto err;
	}
	ep_out->driver_data = ep_out;

	hs_ep_in.bEndpointAddress = fs_ep_in.bEndpointAddress;
	hs_ep_out.bEndpointAddress = fs_ep_out.bEndpointAddress;

	return 0;

err:
	if (ep0_req) {
		usb_ep_free_request(g->ep0, ep0_req);
		ep0_req = NULL;
	}

	if (ep_in)
		ep_in->driver_data = NULL;
	if (ep_out)
		ep_out->driver_data = NULL;

	return ret;
}

static void fastboot_unbind(struct usb_gadget *gadget)
{
	if (ep0_req) {
		usb_ep_free_request(g->ep0, ep0_req);
		ep0_req = NULL;
	}

	if (ep_in)
		ep_in->driver_data = NULL;
	if (ep_out)
		ep_out->driver_data = NULL;
}

static int fastboot_disable_ep(struct usb_gadget *gadget)
{
	if (req_rx && req_tx) {
		PUTS("Fastboot: USB gadget disabled\n");
		VIDEO_PUTS(2, "Fastboot: USB disabled\n");
	}

	if (ep_in) {
		usb_ep_disable(ep_in);
		if (req_tx) {
			usb_ep_free_request(ep_in, req_tx);
			req_tx = NULL;
		}
	}

	if (ep_out) {
		usb_ep_disable(ep_out);
		if (req_rx) {
			usb_ep_free_request(ep_out, req_rx);
			req_rx = NULL;
		}
	}

	return 0;
}

static int fastboot_enable_ep(struct usb_gadget *gadget)
{
	int ret = 0;
	unsigned power;

	/* Make sure buffers are released */
	if (ep_in && req_tx) {
		usb_ep_free_request(ep_in, req_tx);
		req_tx = NULL;
	}
	if (ep_out && req_rx) {
		usb_ep_free_request(ep_out, req_rx);
		req_rx = NULL;
	}

	/* make sure we don't enable the ep twice */
	if (gadget->speed == USB_SPEED_HIGH)
		ret = usb_ep_enable(ep_out, &hs_ep_out);
	else
		ret = usb_ep_enable(ep_out, &fs_ep_out);
	if (ret) {
		FASTBOOT_ERR("enable OUT ep err %d\n", ret);
		goto err;
	}

	req_rx = usb_ep_alloc_request(ep_out, 0);
	if (!req_rx) {
		ret = -ENOMEM;
		FASTBOOT_ERR("alloc rx req err\n");
		goto err;
	}

	if (gadget->speed == USB_SPEED_HIGH)
		ret = usb_ep_enable(ep_in, &hs_ep_in);
	else
		ret = usb_ep_enable(ep_in, &fs_ep_in);
	if (ret) {
		FASTBOOT_ERR("enable IN ep err %d\n", ret);
		goto err;
	}

	req_tx = usb_ep_alloc_request(ep_in, 0);
	if (!req_tx) {
		ret = -ENOMEM;
		FASTBOOT_ERR("alloc tx req err\n");
		goto err;
	}
	req_tx->buf = tx_buffer;
	req_tx->length = EP_BUFFER_SIZE;

	ret = rx_cmd();
	if (ret)
		goto err;

	power = config_desc.bMaxPower ? (2 * config_desc.bMaxPower) : CONFIG_USB_GADGET_VBUS_DRAW;

	PRINT("Fastboot: %s speed USB gadget enabled (%umA)\n",
			(gadget->speed == USB_SPEED_LOW) ? "low" :
			((gadget->speed == USB_SPEED_FULL) ? "full" :
			((gadget->speed == USB_SPEED_HIGH) ? "high" : "?")),
			power);

	usb_gadget_vbus_draw(gadget, power);

	VIDEO_PRINT(2, "Fastboot: %s speed USB enabled\n",
			(gadget->speed == USB_SPEED_LOW) ? "low" :
			((gadget->speed == USB_SPEED_FULL) ? "full" :
			((gadget->speed == USB_SPEED_HIGH) ? "high" : "?")));

	return 0;

err:
	fastboot_disable_ep(gadget);
	return ret;
}

static int fastboot_setup_get_descr(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl)
{
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	u16 val;
	int ret = 0;
	u32 bytes_remaining;
	u32 bytes_total;
	u32 this_inc;

	val = w_value >> 8;

	switch (val) {
	case USB_DT_DEVICE:
		memcpy(ep0_buffer, &fb_descriptor, sizeof(fb_descriptor));
		ep0_req->length = min(w_length, sizeof(fb_descriptor));
		ret = fastboot_ep0_queue(gadget);
		break;

	case USB_DT_CONFIG:
		bytes_remaining = min(w_length, EP0_BUFFER_SIZE);
		bytes_total = 0;

		/* config */
		this_inc = min(bytes_remaining, USB_DT_CONFIG_SIZE);
		bytes_remaining -= this_inc;
		memcpy(ep0_buffer + bytes_total, &config_desc, this_inc);
		bytes_total += this_inc;

		/* interface */
		this_inc = min(bytes_remaining, USB_DT_INTERFACE_SIZE);
		bytes_remaining -= this_inc;
		memcpy(ep0_buffer + bytes_total, &interface_desc, this_inc);
		bytes_total += this_inc;

		/* ep in */
		this_inc = min(bytes_remaining, USB_DT_ENDPOINT_SIZE);
		bytes_remaining -= this_inc;
		if (gadget->speed == USB_SPEED_HIGH)
			memcpy(ep0_buffer + bytes_total, &hs_ep_in,
					this_inc);
		else
			memcpy(ep0_buffer + bytes_total, &fs_ep_in,
					this_inc);
		bytes_total += this_inc;

		/* ep out */
		this_inc = min(bytes_remaining, USB_DT_ENDPOINT_SIZE);
		/*bytes_remaining -= this_inc;*/
		if (gadget->speed == USB_SPEED_HIGH)
			memcpy(ep0_buffer + bytes_total, &hs_ep_out,
					this_inc);
		else
			memcpy(ep0_buffer + bytes_total, &fs_ep_out,
					this_inc);
		bytes_total += this_inc;

		ep0_req->length = bytes_total;
		ret = fastboot_ep0_queue(gadget);
		break;

	case USB_DT_STRING:
		if ((w_value & 0xff) >= FASTBOOT_STR_MAX_IDX) {
			/* Bypass the invalid string index. */
			FASTBOOT_DPRINT("string: bypass %d\n", (w_value & 0xff));
			ret = 0;
		} else {
			ret = usb_gadget_get_string(&fb_strings, w_value & 0xff, ep0_buffer);
			if (ret < 0) {
				FASTBOOT_DPRINT("string: %d err %d\n", (w_value & 0xff), ret);
				break;
			}
		}
		ep0_req->length = ret;
		ret = fastboot_ep0_queue(gadget);
		break;

	case USB_DT_DEVICE_QUALIFIER:
		memcpy(ep0_buffer, &qual_desc, sizeof(qual_desc));
		ep0_req->length = min(w_length, sizeof(qual_desc));
		ret = fastboot_ep0_queue(gadget);
		break;

	default:
		FASTBOOT_DPRINT("%s: unknown desc %d\n", __func__, val);
		ret = -EPERM;
		break;
	}
	return ret;
}

static int fastboot_setup_get_conf(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *ctrl)
{
	u16 w_length = le16_to_cpu(ctrl->wLength);

	if (w_length == 0)
		return -EINVAL;

	ep0_buffer[0] = current_config;
	ep0_req->length = 1;
	return usb_ep_queue(gadget->ep0, ep0_req, 0);
}

static int fastboot_set_interface(struct usb_gadget *gadget, u32 enable)
{
	if (enable && req_rx) {
		FASTBOOT_DPRINT("%s: ep was enabled\n", __func__);
		return 0;
	}
	if (!enable && !req_rx) {
		FASTBOOT_DPRINT("%s: ep was not enabled\n", __func__);
		return 0;
	}

	if (enable)
		return fastboot_enable_ep(gadget);
	else
		return fastboot_disable_ep(gadget);
}

static int fastboot_setup_out_req(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *req)
{
	int ret = -EPERM;

	switch (req->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		switch (req->bRequest) {
		case USB_REQ_SET_CONFIGURATION:
			ep0_req->length = 0;
			if (req->wValue == CONFIGURATION_NORMAL) {
				current_config = CONFIGURATION_NORMAL;
				ret = fastboot_set_interface(gadget, 1);
				if (!ret)
					ret = fastboot_ep0_queue(gadget);
				return ret;
			}
			if (req->wValue == 0) {
				current_config = 0;
				ret = fastboot_set_interface(gadget, 0);
				if (!ret)
					ret = fastboot_ep0_queue(gadget);
				return ret;
			}
			return ret;
			break;
		default:
			FASTBOOT_DPRINT("%s: unknown request %d\n", __func__, req->bRequest);
			return ret;
		};

	case USB_RECIP_INTERFACE:
		switch (req->bRequest) {
		case USB_REQ_SET_INTERFACE:
			ep0_req->length = 0;
			ret = fastboot_set_interface(gadget, 1);
			if (!ret)
				ret = fastboot_ep0_queue(gadget);
			return ret;
			break;
		default:
			FASTBOOT_DPRINT("%s: unknown request %d\n", __func__, req->bRequest);
			return ret;
		}

	case USB_RECIP_ENDPOINT:
		switch (req->bRequest) {
		case USB_REQ_CLEAR_FEATURE:
			return fastboot_ep0_queue(gadget);
			break;
		default:
			FASTBOOT_DPRINT("%s: unknown request %d\n", __func__, req->bRequest);
			return ret;
		}

	default:
		FASTBOOT_DPRINT("%s: unknown request type %d\n", __func__,
				(req->bRequestType & USB_RECIP_MASK));
		break;
	}
	return ret;
}

static int fastboot_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *req)
{
	if ((req->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD) {
		FASTBOOT_DPRINT("%s invalid request type %d\n", __func__,
				(req->bRequestType & USB_TYPE_MASK) >> 5);
		return -EPERM;
	}

	if ((req->bRequestType & USB_DIR_IN) == 0)
		/* host-to-device */
		return fastboot_setup_out_req(gadget, req);

	/* device-to-host */
	if ((req->bRequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		switch (req->bRequest) {
		case USB_REQ_GET_DESCRIPTOR:
			return fastboot_setup_get_descr(gadget, req);
			break;

		case USB_REQ_GET_CONFIGURATION:
			return fastboot_setup_get_conf(gadget, req);
			break;

		default:
			FASTBOOT_DPRINT("%s: unknown request %d\n", __func__, req->bRequest);
			return -EPERM;
		}
	}
	return -EPERM;
}

static void fastboot_disconnect(struct usb_gadget *gadget)
{
	fastboot_disable_ep(gadget);
}

static void fastboot_suspend(struct usb_gadget *gadget)
{
#if 0
	usb_gadget_vbus_draw(gadget, 2);
#endif

	if (req_rx && req_tx) {
		PUTS("Fastboot: USB gadget suspended\n");
	}
}

static void fastboot_resume(struct usb_gadget *gadget)
{
	unsigned power = config_desc.bMaxPower ? (2 * config_desc.bMaxPower) : CONFIG_USB_GADGET_VBUS_DRAW;

	usb_gadget_vbus_draw(gadget, power);

	if (req_rx && req_tx) {
#if 0
		PRINT("Fastboot: %s speed USB gadget resumed (%umA)\n",
				(gadget->speed == USB_SPEED_LOW) ? "low" :
				((gadget->speed == USB_SPEED_FULL) ? "full" :
				((gadget->speed == USB_SPEED_HIGH) ? "high" : "?")),
				power);
#else
		PUTS("Fastboot: USB gadget resumed\n");
#endif
	}
}

/*-------------------------------------------------------------------------*/

static struct usb_gadget_driver fastboot_gadget = {
	.speed		= USB_SPEED_HIGH,
	.bind		= fastboot_bind,
	.unbind		= fastboot_unbind,
	.setup		= fastboot_setup,
	.disconnect	= fastboot_disconnect,
	.suspend	= fastboot_suspend,
	.resume		= fastboot_resume,
};

/*-------------------------------------------------------------------------*/

#define FASTBOOT_UART_FLUSH_COUNT	0x00DF0000
static unsigned int uart_flush_count;

int fastboot_init(void)
{
	int ret = -ENOMEM;

	/* Make sure buffers are released */
	if (tx_buffer) {
		free(tx_buffer);
		tx_buffer = NULL;
	}
	if (rx_buffer) {
		free(rx_buffer);
		rx_buffer = NULL;
	}
	if (ep0_buffer) {
		free(ep0_buffer);
		ep0_buffer = NULL;
	}
	req_tx = NULL;
	req_rx = NULL;
	ep_in = NULL;
	ep_out = NULL;

	fastboot_exit_reason = 0;
	download_addr = CONFIG_FASTBOOT_BUFFER;
	download_size = 0;
	download_has_error = 1;

	/* Allocate buffers */
	ep0_buffer = memalign(ARCH_DMA_MINALIGN, EP0_BUFFER_SIZE);
	if (!ep0_buffer)
		goto _exit0;
	rx_buffer = memalign(ARCH_DMA_MINALIGN, EP_BUFFER_SIZE);
	if (!rx_buffer)
		goto _exit0;
	tx_buffer = memalign(ARCH_DMA_MINALIGN, EP_BUFFER_SIZE);
	if (!tx_buffer)
		goto _exit0;

	INIT_LIST_HEAD(&fb_work_list);
	memset(fb_works, 0x0, sizeof(struct fastboot_work) * FASTBOOT_MAX_WORKS);

	fastboot_board_init(device_usb_fb_strings, &fb_descriptor);

	ret = usb_gadget_register_driver(&fastboot_gadget);
	if (ret) {
		FASTBOOT_ERR("add gadget err %d\n", ret);
		goto _exit0;
	}

	PUTS("Fastboot: waiting for USB connection\n");

#if defined(CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG) && (CONFIG_FASTBOOT_VIDEO_CONSOLE_DEBUG == 1)
	VIDEO_PUTS(1, "Fastboot\n");
#else
	VIDEO_PUTS(2, "Fastboot: waiting for USB\n");
#endif

	UART_FIFO_FLUSH_ONCE();
	uart_flush_count = FASTBOOT_UART_FLUSH_COUNT;
	UART_FIFO_FLUSH_ONCE();

	return 0;

_exit0:
	/* Free buffers */
	if (tx_buffer) {
		free(tx_buffer);
		tx_buffer = NULL;
	}
	if (rx_buffer) {
		free(rx_buffer);
		rx_buffer = NULL;
	}
	if (ep0_buffer) {
		free(ep0_buffer);
		ep0_buffer = NULL;
	}

	PRINT("Fastboot: init err %d\n", ret);

	return ret;
}

void fastboot_shutdown(void)
{
	usb_gadget_unregister_driver(&fastboot_gadget);

	/* Free buffers */
	if (tx_buffer) {
		free(tx_buffer);
		tx_buffer = NULL;
	}
	if (rx_buffer) {
		free(rx_buffer);
		rx_buffer = NULL;
	}
	if (ep0_buffer) {
		free(ep0_buffer);
		ep0_buffer = NULL;
	}

	PRINT("Fastboot: exit 0x%08x\n", fastboot_exit_reason);
	if (fastboot_exit_reason == FASTBOOT_EXIT_CONTINUE)
		VIDEO_CLEAR();
}

int fastboot_poll(void)
{
	if (--uart_flush_count == 0) {
		UART_FIFO_FLUSH_ONCE();
		uart_flush_count = FASTBOOT_UART_FLUSH_COUNT;
		UART_FIFO_FLUSH_ONCE();
	}

	usb_gadget_handle_interrupts();
	otg_handle_interrupts(0);
	usb_gadget_handle_interrupts();

	fastboot_run_works();

	return fastboot_exit_reason;
}

#endif /* CONFIG_USB_GADGET_FASTBOOT */
