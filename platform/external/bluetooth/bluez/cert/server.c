/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2011  Nokia Corporation
 *  Copyright (C) 2011  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <glib.h>
#include <gdbus.h>
#include <errno.h>
#include <bluetooth/uuid.h>
#include <stdlib.h>
#include <dbus-common.h>
#include <dbus/dbus.h>

#include "att.h"
#include "adapter.h"
#include "error.h"
#include "gattrib.h"
#include "attrib-server.h"
#include "gatt-service.h"
#include "log.h"
#include "server.h"
#include "storage.h"
#include "attio.h"
#include "manager.h"
#include "device.h"

#define SERVICE_A_SVC_UUID	0xA00A
#define SERVICE_B_SVC_UUID	0xA00B
#define SERVICE_C_SVC_UUID_128	"0000A00C-0000-0000-0123-456789ABCDEF"
#define SERVICE_D_SVC_UUID	0xA00D
#define SERVICE_E_SVC_UUID	0xA00E

#define V01_UUID		0xB001
#define V02_UUID		0xB002
#define V03_UUID		0xB003
#define V04_UUID		0xB004
#define V05_UUID		0xB005
#define V06_UUID		0xB006
#define V07_UUID		0xB007
#define V08_UUID		0xB008
#define V09_UUID_128		"0000B009-0000-0000-0123-456789ABCDEF"
#define V10_UUID		0xB00A
#define V11_UUID_128		"0000B00B-0000-0000-0123-456789ABCDEF"
#define V12_UUID		0xB00C
#define V13_UUID		0xB00D

#define DESC_V5D4_UUID_128	"0000D5D4-0000-0000-0123-456789ABCDEF"
#define DESC_V9D2_UUID_128	"0000D9D2-0000-0000-0123-456789ABCDEF"
#define DESC_V9D3_UUID_128	"0000D9D3-0000-0000-0123-456789ABCDEF"

#define SVC_A_VAL_02_INIT	"11111222223333344444555556666677777888889999900000"
#define GAP_DEVICE_NAME_INIT	"Test Database"
#define SVC_B1_FUTURE_CHAR_INIT	"1111122222333334444455555666667777788888999"
#define SVC_B1_USER_DESC_INIT	"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
#define SVC_C1_VAL_02A_INIT	"111112222233333444445"
#define SVC_C1_VAL_02B_INIT	"2222233333444445555566"
#define SVC_C1_VAL_02C_INIT	"33333444445555566666777"
#define SVC_C1_VAL_02D_INIT	"1111122222333334444455555666667777788888999"
#define SVC_C1_VAL_02E_INIT	"22222333334444455555666667777788888999990000"
#define SVC_C1_VAL_02F_INIT	"333334444455555666667777788888999990000011111"

#define CERT_INTERFACE		"org.bluez.Cert"

const uint128_t v9_uuid_btorder = {
	.data = { 0x00, 0x00, 0xF2, 0x18, 0x90, 0x2C, 0x45, 0x0B,
		  0xB6, 0xC4, 0x62, 0x89, 0x1E, 0x8C, 0x25, 0xE9 } };

struct adapter_ccc {
	struct btd_adapter *adapter;
	uint16_t handle;
};

struct notify_callback {
	struct btd_device *device;
	guint id;
	uint8_t pdu[ATT_MAX_MTU];
	size_t len;
};

struct cert_adapter {
	struct btd_adapter *adapter;
};

enum {
	QUIRK_B3_V06_AUTH_READ		= 1 << 0,
	QUIRK_A_V02_AUTH_READ		= 1 << 1,
	QUIRK_C1_V9D3_AUTH_READ		= 1 << 2,
	QUIRK_C1_CHR_AUTH_READ		= 1 << 3,
	QUIRK_B3_V06_AUTH_WRITE		= 1 << 4,
	QUIRK_C1_V9D2_AUTH_WRITE	= 1 << 5,
	QUIRK_SEC_AUT_BV12_AUTH_MITM	= 1 << 6,
};

static uint32_t quirks;

static uint16_t notif_handle;
static uint16_t ind_handle;
static uint16_t ccc_handle;
static uint16_t scc_handle;

static DBusConnection *connection;

static GSList *cert_adapters;

static GSList *devices_notify;
static GSList *devices_indicate;

static void filter_devices_indicate(char *key, char *value, void *user_data)
{
	struct adapter_ccc *ccc = user_data;
	struct btd_adapter *adapter = ccc->adapter;
	struct btd_device *device;
	char addr[18];
	uint16_t handle, ccc_val;

	sscanf(key, "%17s#%04hX", addr, &handle);

	DBG("addr %s handle %#x", addr, handle);

	if (ccc->handle != handle)
		return;

	ccc_val = strtol(value, NULL, 16);
	if (!(ccc_val & 0x0002))
		return;

	device = adapter_find_device(adapter, addr);
	if (device == NULL)
		return;

	if (g_slist_find(devices_indicate, device))
		return;

	devices_indicate = g_slist_append(devices_indicate, device);
}

static GSList *devices_to_indicate(struct btd_adapter *adapter, uint16_t ccc_hnd)
{
	struct adapter_ccc ccc_list = {adapter, ccc_hnd};
	char filename[PATH_MAX + 1];
	char srcaddr[18];
	bdaddr_t src;

	adapter_get_address(adapter, &src);
	ba2str(&src, srcaddr);

	DBG("srcaddr=%s, ccc_hnd=0x%04x", srcaddr, ccc_hnd);

	create_name(filename, PATH_MAX, STORAGEDIR, srcaddr, "ccc");

	textfile_foreach(filename, filter_devices_indicate, &ccc_list);

	return devices_indicate;
}

static void ind_result_cb(guint8 status, struct btd_adapter *adapter,
			  struct btd_device *device, gpointer user_data)
{
	int ret;

	DBG("%d", status);

	ret = g_dbus_emit_signal(connection, "/", CERT_INTERFACE, "IndicationResult",
			         DBUS_TYPE_BYTE, &status,
			         DBUS_TYPE_INVALID);
	DBG("%d", ret);
}

static void register_service_gap(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(GENERIC_ACCESS_PROFILE_ID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(GATT_CHARAC_DEVICE_NAME, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, GATT_CHARAC_DEVICE_NAME);
	memcpy(atval, GAP_DEVICE_NAME_INIT, sizeof(GAP_DEVICE_NAME_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, sizeof(GAP_DEVICE_NAME_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(GATT_CHARAC_APPEARANCE, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, GATT_CHARAC_APPEARANCE);
	att_put_u16(17, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(GATT_CHARAC_PERIPHERAL_PREF_CONN, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, GATT_CHARAC_PERIPHERAL_PREF_CONN);
	att_put_u16(100, &atval[0]);
	att_put_u16(200, &atval[2]);
	att_put_u16(0, &atval[4]);
	att_put_u16(2000, &atval[6]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 8);
}

static void register_service_ap(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(GENERIC_ATTRIB_PROFILE_ID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_INDICATE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(GATT_CHARAC_SERVICE_CHANGED, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, GATT_CHARAC_SERVICE_CHANGED);
	att_put_u16(0x0001, &atval[0]);
	att_put_u16(0xffff, &atval[2]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NOT_PERMITTED,
		      ATT_NOT_PERMITTED, atval, 4);
}

static void register_service_a(struct cert_adapter *ca, uint16_t start,
			       uint16_t d_start, uint16_t d_end,
			       uint16_t c1_start, uint16_t c1_end)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];
	int read_reqs;

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_A_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_INCLUDE_UUID);
	att_put_u16(d_start, &atval[0]);
	att_put_u16(d_end, &atval[2]);
	att_put_u16(SERVICE_D_SVC_UUID, &atval[4]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 6);

	bt_uuid16_create(&uuid, GATT_INCLUDE_UUID);
	att_put_u16(c1_start, &atval[0]);
	att_put_u16(c1_end, &atval[2]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 4);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V01_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V01_UUID);
	att_put_u8(0x01, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	if (quirks & QUIRK_A_V02_AUTH_READ)
		read_reqs = ATT_AUTHENTICATION;
	else
		read_reqs = ATT_NONE;

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_A_VAL_02_INIT, sizeof(SVC_A_VAL_02_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, read_reqs, ATT_NONE,
		      atval, sizeof(SVC_A_VAL_02_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V03_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V03_UUID);
	att_put_u8(0x03, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NOT_PERMITTED, ATT_NONE,
		      atval, 1);
}

static void register_service_b1(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_B_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V04_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V04_UUID);
	att_put_u8(0x04, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 1);

	ccc_handle = h;
	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	att_put_u16(0x0000, &atval[1]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 5);

	scc_handle = h;
	bt_uuid16_create(&uuid, GATT_SERVER_CHARAC_CFG_UUID);
	att_put_u16(0x0000, &atval[1]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 5);
}

static void register_service_b2(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_B_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE |
		   ATT_CHAR_PROPER_EXT_PROPER;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V05_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V05_UUID);
	att_put_u16(0x05, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_EXT_PROPER_UUID);
	att_put_u16(0x0003, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_USER_DESC_UUID);
	memcpy(atval, SVC_B1_USER_DESC_INIT, sizeof(SVC_B1_USER_DESC_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE, atval,
		      sizeof(SVC_B1_USER_DESC_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_FMT_UUID);
	att_put_u8(0x04, &atval[0]);
	att_put_u8(0x00, &atval[1]);
	att_put_u16(0x3001, &atval[2]);
	att_put_u8(0x00, &atval[4]);
	att_put_u16(0x3111, &atval[5]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 7);

	bt_string_to_uuid(&uuid, DESC_V5D4_UUID_128);
	att_put_u8(0x44, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 1);
}

static void register_service_b5(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_B_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V08_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V08_UUID);
	att_put_u8(0x08, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 1);
}

static void register_service_b3(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];
	int read_reqs, write_reqs;

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_D_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	notif_handle = h;
	ind_handle = h;
	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE_WITHOUT_RESP |
		   ATT_CHAR_PROPER_WRITE | ATT_CHAR_PROPER_NOTIFY |
		   ATT_CHAR_PROPER_INDICATE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V06_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	if (quirks & QUIRK_B3_V06_AUTH_READ)
		read_reqs = ATT_AUTHENTICATION;
	else
		read_reqs = ATT_NONE;

	if (quirks & QUIRK_B3_V06_AUTH_WRITE)
		write_reqs = ATT_AUTHENTICATION;
	else
		write_reqs = ATT_NONE;

	bt_uuid16_create(&uuid, V06_UUID);
	att_put_u8(0x06, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, read_reqs, write_reqs, atval, 1);
}

static void register_service_b4(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_A_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V07_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V07_UUID);
	att_put_u8(0x07, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NOT_PERMITTED, ATT_NONE,
		      atval, 1);
}

gboolean check_authenticated_cb(struct attribute *a, gpointer user_data,
				gpointer device)
{
	if (device_is_seclevel_sufficient(device, 3))
		return 0x0;

	return ATT_ECODE_AUTHENTICATION;
}

static void register_service_c1(struct cert_adapter *ca, uint16_t start,
				uint16_t d_start, uint16_t d_end)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];
	int read_reqs, write_reqs;
	struct attribute *attr;

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	bt_string_to_uuid(&u, SERVICE_C_SVC_UUID_128);
	att_put_uuid128(u, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 16);

	bt_uuid16_create(&uuid, GATT_INCLUDE_UUID);
	att_put_u16(d_start, &atval[0]);
	att_put_u16(d_end, &atval[2]);
	att_put_u16(SERVICE_D_SVC_UUID, &atval[4]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 6);

	if (quirks & QUIRK_C1_CHR_AUTH_READ)
		read_reqs = ATT_AUTHENTICATION;
	else
		read_reqs = ATT_NONE;

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE |
		   ATT_CHAR_PROPER_EXT_PROPER;
	att_put_u16(h + 1, &atval[1]);
	bt_string_to_uuid(&u, V09_UUID_128);
	att_put_uuid128(u, &atval[3]);
	attr = attrib_db_add(ca->adapter, h++, &uuid, read_reqs, ATT_NOT_PERMITTED,
		      atval, 19);

	if (quirks & QUIRK_SEC_AUT_BV12_AUTH_MITM) {
		attr->read_cb = check_authenticated_cb;
		attr->cb_user_data = NULL;
	}

	att_put_u8(0x09, &atval[0]);
	attrib_db_add(ca->adapter, h++, &u, ATT_NONE, ATT_NONE,
		      atval, 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_EXT_PROPER_UUID);
	att_put_u16(0x0001, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	if (quirks & QUIRK_C1_V9D2_AUTH_WRITE)
		write_reqs = ATT_AUTHENTICATION;
	else
		write_reqs = ATT_NONE;

	bt_string_to_uuid(&uuid, DESC_V9D2_UUID_128);
	att_put_u8(0x22, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, write_reqs,
		      atval, 1);

	if (quirks & QUIRK_C1_V9D3_AUTH_READ)
		read_reqs = ATT_AUTHENTICATION;
	else
		read_reqs = ATT_NOT_PERMITTED;

	bt_string_to_uuid(&uuid, DESC_V9D3_UUID_128);
	att_put_u8(0x33, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, read_reqs, ATT_NONE,
		      atval, 1);
}

static void register_service_c2(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	bt_string_to_uuid(&u, SERVICE_C_SVC_UUID_128);
	att_put_uuid128(u, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 16);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V10_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V10_UUID);
	att_put_u8(0x0A, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_C1_VAL_02A_INIT, sizeof(SVC_C1_VAL_02A_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, sizeof(SVC_C1_VAL_02A_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_C1_VAL_02B_INIT, sizeof(SVC_C1_VAL_02B_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, sizeof(SVC_C1_VAL_02B_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_C1_VAL_02C_INIT, sizeof(SVC_C1_VAL_02C_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, sizeof(SVC_C1_VAL_02C_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_C1_VAL_02D_INIT, sizeof(SVC_C1_VAL_02D_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, sizeof(SVC_C1_VAL_02D_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_C1_VAL_02E_INIT, sizeof(SVC_C1_VAL_02E_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, sizeof(SVC_C1_VAL_02E_INIT) - 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V02_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V02_UUID);
	memcpy(atval, SVC_C1_VAL_02F_INIT, sizeof(SVC_C1_VAL_02F_INIT) - 1);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, sizeof(SVC_C1_VAL_02F_INIT) - 1);
}

static void register_service_d(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];

	h = start;

	bt_uuid16_create(&uuid, GATT_SND_SVC_UUID);
	att_put_u16(SERVICE_D_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V12_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);

	bt_uuid16_create(&uuid, V12_UUID);
	att_put_u8(0x0C, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 1);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ;
	att_put_u16(h + 1, &atval[1]);
	bt_string_to_uuid(&u, V11_UUID_128);
	att_put_uuid128(u, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 19);

	att_put_u8(0x0B, &atval[0]);
	attrib_db_add(ca->adapter, h++, &u, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 1);
}


uint8_t service_brle_read_cb(struct attribute *a, gpointer user_data,
				gpointer device)
{
	gboolean bredr_only = (user_data == NULL);
	bdaddr_t dba;
	addr_type_t type;

	device_get_address(device, &dba,&type);

	if (bredr_only) {
		if (type == ADDR_TYPE_BREDR)
			return 0x0;
		else
			return ATT_ECODE_IO;
	} else {
		if (type != ADDR_TYPE_BREDR)
			return 0x0;
		else
			return ATT_ECODE_IO;
	}
}

static void register_service_brle(struct cert_adapter *ca, uint16_t start)
{
	bt_uuid_t uuid, u;
	uint16_t h;
	uint8_t atval[256];
	struct attribute *attr;

	h = start;

	bt_uuid16_create(&uuid, GATT_PRIM_SVC_UUID);
	att_put_u16(SERVICE_B_SVC_UUID, &atval[0]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 2);

	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V04_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);


	DBG(" BR/EDR only handle=0x%04x",h);

	bt_uuid16_create(&uuid, V04_UUID);
	att_put_u8(0x04, &atval[0]);
	attr = attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 1);


	ccc_handle = h;
	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	att_put_u16(0x0000, &atval[1]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 5);

	attr->read_cb = service_brle_read_cb;
	attr->cb_user_data = NULL;


	bt_uuid16_create(&uuid, GATT_CHARAC_UUID);
	atval[0] = ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE;
	att_put_u16(h + 1, &atval[1]);
	att_put_u16(V04_UUID, &atval[3]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NOT_PERMITTED,
		      atval, 5);


	DBG("LE only handle=0x%04x",h);

	bt_uuid16_create(&uuid, V04_UUID);
	att_put_u8(0x04, &atval[0]);
	attr = attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 1);

	ccc_handle = h;
	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	att_put_u16(0x0000, &atval[1]);
	attrib_db_add(ca->adapter, h++, &uuid, ATT_NONE, ATT_NONE,
		      atval, 5);

	attr->read_cb = service_brle_read_cb;
	attr->cb_user_data = (gpointer)1;

}

static void register_db1_database(gpointer data, gpointer user_data)
{
	struct cert_adapter *ca = data;

	DBG("");

	btd_adapter_gatt_server_stop(ca->adapter);
	btd_adapter_gatt_server_start(ca->adapter, TRUE);

	register_service_gap(ca, 0x01);
	register_service_ap(ca, 0x10);
	register_service_brle(ca, 0x20);
}

static void register_small_database(gpointer data, gpointer user_data)
{
	struct cert_adapter *ca = data;

	DBG("");

	btd_adapter_gatt_server_stop(ca->adapter);
	btd_adapter_gatt_server_start(ca->adapter, TRUE);

	register_service_gap(ca, 0x01);
	register_service_ap(ca, 0x10);
	register_service_b1(ca, 0x20);
}


static void register_large_database_1(gpointer data, gpointer user_data)
{
	struct cert_adapter *ca = data;

	DBG("");

	btd_adapter_gatt_server_stop(ca->adapter);
	btd_adapter_gatt_server_start(ca->adapter, TRUE);

	register_service_d(ca, 0x01);
	register_service_b3(ca, 0x10);
	register_service_a(ca, 0x20, 0x0001, 0x0005, 0x0090, 0x0096);
	register_service_b4(ca, 0x30);
	register_service_gap(ca, 0x40);
	register_service_ap(ca, 0x50);
	register_service_b1(ca, 0x60);
	register_service_b2(ca, 0x70);
	register_service_b5(ca, 0x80);
	register_service_c1(ca, 0x90, 0x0001, 0x0005);
	register_service_c2(ca, 0xa0);
}

static DBusMessage *start_cert(DBusConnection *conn, DBusMessage *msg,
			       void *data)
{
	const char *db_name;
	uint32_t q;

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_STRING, &db_name,
				   DBUS_TYPE_UINT32, &q,
				   DBUS_TYPE_INVALID)) {
		error("Invalid arguments");
		return NULL;
	}

	DBG(" db_name=%s, quirks=%d",db_name,q);

	if (!strcasecmp(db_name, "small")) {
		quirks = q;
		g_slist_foreach(cert_adapters, register_small_database, NULL);
	} else if (!strcasecmp(db_name, "large1")) {
		quirks = q;
		g_slist_foreach(cert_adapters, register_large_database_1, NULL);
	} else if (!strcasecmp(db_name, "db1")) {
		quirks = q;
		g_slist_foreach(cert_adapters, register_db1_database, NULL);
	} else {
		error("Invalid DB name");
		return btd_error_invalid_args(msg);
	}

	return dbus_message_new_method_return(msg);
}

static void __stop_cert(gpointer data, gpointer user_data)
{
	struct cert_adapter *ca = data;

	DBG("");

	btd_adapter_gatt_server_stop(ca->adapter);
	btd_adapter_gatt_server_start(ca->adapter, FALSE);
}

static DBusMessage *stop_cert(DBusConnection *conn, DBusMessage *msg,
			       void *data)
{
	g_slist_foreach(cert_adapters, __stop_cert, NULL);

	return dbus_message_new_method_return(msg);
}

static void __test_notification(gpointer data, gpointer user_data)
{
	struct cert_adapter *ca = data;
	uint8_t tmp = 0;

	DBG("");

	queue_notification(ca->adapter, BDADDR_ALL, notif_handle, ccc_handle,
			   &tmp, sizeof(tmp));
}

static DBusMessage *test_notification(DBusConnection *conn, DBusMessage *msg,
			       void *data)
{
	g_slist_foreach(cert_adapters, __test_notification, NULL);

	return dbus_message_new_method_return(msg);
}

static void __test_indication(gpointer data, gpointer user_data)
{
	struct cert_adapter *ca = data;
	uint8_t tmp = 0;

	DBG("");

	queue_indication(ca->adapter, BDADDR_ALL, ind_handle, ccc_handle, &tmp,
			 sizeof(tmp), ind_result_cb, ca);
}

static DBusMessage *test_indication(DBusConnection *conn, DBusMessage *msg,
			       void *data)
{
	g_slist_foreach(cert_adapters, __test_indication, NULL);

	return dbus_message_new_method_return(msg);
}

static int cert_server_probe(struct btd_adapter *adapter)
{
	struct cert_adapter *ca;

	ca = g_new0(struct cert_adapter, 1);
	ca->adapter = adapter;
	cert_adapters = g_slist_append(cert_adapters, ca);

	return 0;
}

static int aa_cmp(gconstpointer a, gconstpointer b)
{
	const struct cert_adapter *ca = a;
	const struct btd_adapter *adapter = b;

	if (ca->adapter == adapter)
		return 0;

	return -1;
}

static void cert_server_remove(struct btd_adapter *adapter)
{
	struct cert_adapter *ca;
	GSList *el = g_slist_find_custom(cert_adapters, adapter, aa_cmp);
	if (!el)
		return;

	ca = el->data;
	cert_adapters = g_slist_remove(cert_adapters, ca);
	g_free(ca);
}

struct btd_adapter_driver cert_server_driver = {
	.name = "gatt-cert-server",
	.probe = cert_server_probe,
	.remove = cert_server_remove,
};

static GDBusMethodTable cert_methods[] = {
	{"StartCertification", "su", "", start_cert},
	{"StopCertification", "", "", stop_cert},
	{"TestNotification", "", "", test_notification},
	{"TestIndication", "", "", test_indication},
	{}
};

static GDBusSignalTable cert_signals[] = {
	{"IndicationResult", "y"},
	{}
};

int cert_server_init(void)
{
	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	if (!g_dbus_register_interface(connection, "/", CERT_INTERFACE,
				       cert_methods, cert_signals, NULL, NULL,
				       NULL)) {
		error("D-Bus failed to register %s interface", CERT_INTERFACE);
		dbus_connection_unref(connection);
		connection = NULL;
		return -1;
	}

	DBG("Registered interface %s on", CERT_INTERFACE);
	btd_register_adapter_driver(&cert_server_driver);

	return 0;
}

void cert_server_exit(void)
{
	btd_unregister_adapter_driver(&cert_server_driver);
}
