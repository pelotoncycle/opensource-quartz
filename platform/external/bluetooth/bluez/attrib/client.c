/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2010  Nokia Corporation
 *  Copyright (C) 2010  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2011  Texas Instruments, Inc.
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

#include <errno.h>
#include <stdlib.h>
#include <glib.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>

#include "glib-compat.h"
#include "adapter.h"
#include "device.h"
#include "log.h"
#include "gdbus.h"
#include "error.h"
#include "dbus-common.h"
#include "btio.h"
#include "storage.h"

#include "att.h"
#include "gattrib.h"
#include "attio.h"
#include "gatt.h"
#include "client.h"

#define CHAR_INTERFACE "org.bluez.Characteristic"

#define GATT_CHAR_PROPS_BROADCAST 			0x01
#define GATT_CHAR_PROPS_READ				0x02
#define GATT_CHAR_PROPS_WRITE_NO_RSP		0x04
#define GATT_CHAR_PROPS_WRITE				0x08
#define GATT_CHAR_PROPS_NOTIFY				0x10
#define GATT_CHAR_PROPS_INDICATE			0x20
#define GATT_CHAR_PROPS_WRITE_AUTHSIGN		0x40

#define GATT_CHAR_EXT_PROPS_WRITE_RELIABLE	0x0001
#define GATT_CHAR_EXT_PROPS_AUX_WRITABLE	0x0002

#define GATT_CHAR_CLIENTCONF_NOTIFY			0x0001
#define GATT_CHAR_CLIENTCONF_INDICATE		0x0002

#define GATT_CHAR_SERVERCONF_BROADCAST		0x0001

struct format {
	guint8 format;
	guint8 exponent;
	guint16 unit;
	guint8 namespace;
	guint16 desc;
} __attribute__ ((packed));

struct query {
	DBusMessage *msg;
	guint attioid;
	GSList *list;
};

struct gatt_service {
	struct btd_device *dev;
	struct gatt_primary *prim;
	DBusConnection *conn;
	GAttrib *attrib;
	guint attioid;
	int psm;
	char *path;
	GSList *chars;
	GSList *offline_chars;
	GSList *watchers;
	struct query *query;
};

struct characteristic_descriptor {
	uint16_t handle;
	bt_uuid_t uuid;
};

struct characteristic {
	struct gatt_service *gatt;
	char *path;
	uint16_t handle;
	uint16_t end;
	uint8_t perm;
	uint16_t ext_properties;
	char type[MAX_LEN_UUID_STR + 1];
	char *name;
	char *desc;
	struct format *format;
	uint16_t client_config;
	uint16_t server_config;

	uint16_t ccc_handle;
	uint16_t scc_handle;
	uint16_t extprops_handle;

	uint8_t *value; /* Deprecated */
	size_t vlen;	/* Deprecated */

	GSList *descriptors;
	uint16_t descriptor_count;
};

struct query_data {
	struct gatt_service *gatt;
	struct characteristic *chr;
	uint16_t handle;
};

struct readwrite_op {
	struct characteristic *chr;
	DBusMessage *msg;
	gboolean wait_for_response;
};

struct write_val_op {
	DBusMessage *msg;
	DBusConnection *conn;
	struct characteristic *chr;
};

struct write_resp_op {
	struct characteristic *chr;
	uint8_t status;
};

struct read_resp_op {
	struct characteristic *chr;
	uint8_t status;
	guint8* data;
	uint16_t len;
};

struct watcher {
	guint id;
	char *name;
	char *path;
	struct gatt_service *gatt;
};


static GSList *gatt_services = NULL;

static void characteristic_free(void *user_data)
{
	struct characteristic *chr = user_data;

	g_free(chr->path);
	g_free(chr->desc);
	g_free(chr->format);
	g_free(chr->value);
	g_free(chr->name);
	g_slist_free_full(chr->descriptors, free);
	g_free(chr);
}

static void watcher_free(void *user_data)
{
	struct watcher *watcher = user_data;

	g_free(watcher->path);
	g_free(watcher->name);
	g_free(watcher);
}

static gboolean att_device_connected(struct gatt_service *gatt)
{
	GSList *l, *left;
	if (gatt != NULL && gatt->dev != NULL && device_is_connected(gatt->dev))
			return true;

	return false;
}

static void gatt_service_free(struct gatt_service *gatt)
{
	g_slist_free_full(gatt->watchers, watcher_free);
	g_slist_free_full(gatt->chars, characteristic_free);
	g_slist_free(gatt->offline_chars);
	g_free(gatt->path);
	btd_device_unref(gatt->dev);
	dbus_connection_unref(gatt->conn);
	g_free(gatt);
}

static void gatt_get_address(struct gatt_service *gatt,
				bdaddr_t *sba, bdaddr_t *dba)
{
	struct btd_device *device = gatt->dev;
	struct btd_adapter *adapter;

	adapter = device_get_adapter(device);
	adapter_get_address(adapter, sba);
	device_get_address(device, dba, NULL);
}

static int characteristic_handle_cmp(gconstpointer a, gconstpointer b)
{
	const struct characteristic *chr = a;
	uint16_t handle = GPOINTER_TO_UINT(b);

	return chr->handle - handle;
}

static int watcher_cmp(gconstpointer a, gconstpointer b)
{
	const struct watcher *watcher = a;
	const struct watcher *match = b;
	int ret;

	ret = g_strcmp0(watcher->name, match->name);
	if (ret != 0)
		return ret;

	return g_strcmp0(watcher->path, match->path);
}

struct CharacteristicNames {
    char uuid[MAX_LEN_UUID_STR + 1];
    char name[50];
};

static struct CharacteristicNames charNames[] = {
		{"00002a43-0000-1000-8000-00805f9b34fb","Alert Category ID" },
		{"00002a42-0000-1000-8000-00805f9b34fb","Alert Category ID Bit Mask" },
		{"00002a06-0000-1000-8000-00805f9b34fb","Alert Level" },
		{"00002a44-0000-1000-8000-00805f9b34fb","Alert Notification Control Point" },
		{"00002a3f-0000-1000-8000-00805f9b34fb","Alert Status" },
		{"00002a01-0000-1000-8000-00805f9b34fb","Appearance" },
		{"00002a49-0000-1000-8000-00805f9b34fb","Blood Pressure Feature" },
		{"00002a35-0000-1000-8000-00805f9b34fb","Blood Pressure Measurement" },
		{"00002a38-0000-1000-8000-00805f9b34fb","Body Sensor Location" },
		{"00002a2b-0000-1000-8000-00805f9b34fb","Current Time" },
		{"00002a08-0000-1000-8000-00805f9b34fb","Date Time" },
		{"00002a0a-0000-1000-8000-00805f9b34fb","Day Date Time" },
		{"00002a09-0000-1000-8000-00805f9b34fb","Day of Week" },
		{"00002a00-0000-1000-8000-00805f9b34fb","Device Name" },
		{"00002a0d-0000-1000-8000-00805f9b34fb","DST Offset" },
		{"00002a0c-0000-1000-8000-00805f9b34fb","Exact Time 256" },
		{"00002a26-0000-1000-8000-00805f9b34fb","Firmware Revision String" },
		{"00002a27-0000-1000-8000-00805f9b34fb","Hardware Revision String" },
		{"00002a39-0000-1000-8000-00805f9b34fb","Heart Rate Control Point" },
		{"00002a37-0000-1000-8000-00805f9b34fb","Heart Rate Measurement" },
		{"00002a2a-0000-1000-8000-00805f9b34fb","IEEE 11073-20601 Regulatory" },
		{"00002a36-0000-1000-8000-00805f9b34fb","Intermediate Cuff Pressure" },
		{"00002a1e-0000-1000-8000-00805f9b34fb","Intermediate Temperature" },
		{"00002a0f-0000-1000-8000-00805f9b34fb","Local Time Information" },
		{"00002a29-0000-1000-8000-00805f9b34fb","Manufacturer Name String" },
		{"00002a21-0000-1000-8000-00805f9b34fb","Measurement Interval" },
		{"00002a24-0000-1000-8000-00805f9b34fb","Model Number String" },
		{"00002a46-0000-1000-8000-00805f9b34fb","New Alert" },
		{"00002a04-0000-1000-8000-00805f9b34fb","Peripheral Preferred Connection Parameters" },
		{"00002a02-0000-1000-8000-00805f9b34fb","Peripheral Privacy Flag" },
		{"00002a03-0000-1000-8000-00805f9b34fb","Reconnection Address" },
		{"00002a14-0000-1000-8000-00805f9b34fb","Reference Time Information" },
		{"00002a40-0000-1000-8000-00805f9b34fb","Ringer Control Point" },
		{"00002a41-0000-1000-8000-00805f9b34fb","Ringer Setting" },
		{"00002a25-0000-1000-8000-00805f9b34fb","Serial Number String" },
		{"00002a05-0000-1000-8000-00805f9b34fb","Service Changed" },
		{"00002a28-0000-1000-8000-00805f9b34fb","Software Revision String" },
		{"00002a47-0000-1000-8000-00805f9b34fb","Supported New Alert Category" },
		{"00002a48-0000-1000-8000-00805f9b34fb","Supported Unread Alert Category" },
		{"00002a23-0000-1000-8000-00805f9b34fb","System ID" },
		{"00002a1c-0000-1000-8000-00805f9b34fb","Temperature Measurement" },
		{"00002a1d-0000-1000-8000-00805f9b34fb","Temperature Type" },
		{"00002a12-0000-1000-8000-00805f9b34fb","Time Accuracy" },
		{"00002a13-0000-1000-8000-00805f9b34fb","Time Source" },
		{"00002a16-0000-1000-8000-00805f9b34fb","Time Update Control Point" },
		{"00002a17-0000-1000-8000-00805f9b34fb","Time Update State" },
		{"00002a11-0000-1000-8000-00805f9b34fb","Time with DST" },
		{"00002a0e-0000-1000-8000-00805f9b34fb","Time Zone" },
		{"00002a07-0000-1000-8000-00805f9b34fb","Tx Power Level" },
		{"00002a45-0000-1000-8000-00805f9b34fb","Unread Alert Status" },
		{"00009025-0000-1000-8000-00805f9b34fb","TIBLUEZ_TEST1" },
};

static const char* get_char_name(const char* uuid)
{
	uint8_t i = 0;
	uint8_t count = sizeof(charNames) / sizeof(struct CharacteristicNames);

	for (i = 0; i < count; i++) {
		if(g_strcmp0(charNames[i].uuid, uuid) == 0) {
			return charNames[i].name;
		}
	}
	return NULL;
}

static void append_char_dict(DBusMessageIter *iter, struct characteristic *chr)
{
	DBusMessageIter dict;
	const char *name = "";
	char *uuid;
	gboolean broadcast;
	gboolean indicate;
	gboolean notify;
	gboolean readable;
	char **write_methods;
	uint8_t write_methods_count = 0;

	dbus_message_iter_open_container(iter, DBUS_TYPE_ARRAY,
			DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
			DBUS_TYPE_STRING_AS_STRING DBUS_TYPE_VARIANT_AS_STRING
			DBUS_DICT_ENTRY_END_CHAR_AS_STRING, &dict);

	uuid = g_strdup(chr->type);
	dict_append_entry(&dict, "UUID", DBUS_TYPE_STRING, &uuid);
	g_free(uuid);

	name = get_char_name(chr->type);

	/* FIXME: Translate UUID to name. */
	if (name != NULL)
		dict_append_entry(&dict, "Name", DBUS_TYPE_STRING, &name);

	if (chr->desc)
		dict_append_entry(&dict, "Description", DBUS_TYPE_STRING,
								&chr->desc);

	/* Broadcast */
	broadcast = ((chr->perm & GATT_CHAR_PROPS_BROADCAST) != 0) &&
				((chr->server_config & GATT_CHAR_SERVERCONF_BROADCAST) != 0);
	dict_append_entry(&dict, "Broadcast", DBUS_TYPE_BOOLEAN,
			&broadcast);

	/* Indicate */
	indicate = ((chr->perm & GATT_CHAR_PROPS_INDICATE) != 0) &&
			((chr->client_config & GATT_CHAR_CLIENTCONF_INDICATE) != 0);
	dict_append_entry(&dict, "Indicate", DBUS_TYPE_BOOLEAN,
			&indicate);

	/* Notify */
	notify =  ((chr->perm & GATT_CHAR_PROPS_NOTIFY) != 0) &&
			((chr->client_config & GATT_CHAR_CLIENTCONF_NOTIFY) != 0);
	dict_append_entry(&dict, "Notify", DBUS_TYPE_BOOLEAN,
			&notify);

	/* Readable */
	readable = ((chr->perm & GATT_CHAR_PROPS_READ) != 0) ;
	dict_append_entry(&dict, "Readable", DBUS_TYPE_BOOLEAN,
			&readable);

	write_methods = g_new0(char *, 5);

	/* WritableNoRsp */
	if ((chr->perm & GATT_CHAR_PROPS_WRITE_NO_RSP) != 0)
		write_methods[write_methods_count++] = "WriteWithoutResponse";

	/* WritableRsp */
	if ((chr->perm & GATT_CHAR_PROPS_WRITE) != 0)
		write_methods[write_methods_count++] = "Write";

	/* WritableAuth */
	if ((chr->perm & GATT_CHAR_PROPS_WRITE_AUTHSIGN) != 0)
		write_methods[write_methods_count++] = "AuthenticatedSignedWrite";

	if ((chr->ext_properties & GATT_CHAR_EXT_PROPS_WRITE_RELIABLE) != 0)
		write_methods[write_methods_count++] = "ReliableWrite";

	/* TODO : add "WritableAuxiliaries" */
	/* TODO : add CustomCharacteristicsDesc  */

	dict_append_array(&dict, "WriteMethods", DBUS_TYPE_STRING,
									&write_methods, write_methods_count);

	if (chr->value)
		dict_append_array(&dict, "Value", DBUS_TYPE_BYTE, &chr->value,
	                                                       chr->vlen);

	g_free(write_methods);


	/* FIXME: Missing Format, Value and Representation */

	dbus_message_iter_close_container(iter, &dict);
}

static void watcher_exit(DBusConnection *conn, void *user_data)
{
	struct watcher *watcher = user_data;
	struct gatt_service *gatt = watcher->gatt;

	INFO("%s watcher %s exited", gatt->path, watcher->name);

	gatt->watchers = g_slist_remove(gatt->watchers, watcher);
}

static int characteristic_set_value(struct characteristic *chr,
					const uint8_t *value, size_t vlen)
{
	chr->value = g_try_realloc(chr->value, vlen);
	if (chr->value == NULL)
		return -ENOMEM;

	memcpy(chr->value, value, vlen);
	chr->vlen = vlen;

	return 0;
}

static void update_watchers_valuechange(gpointer data, gpointer user_data)
{
	struct watcher *w = data;
	struct characteristic *chr = user_data;
	DBusConnection *conn = w->gatt->conn;
	DBusMessage *msg;

	msg = dbus_message_new_method_call(w->name, w->path,
				"org.bluez.Watcher", "ValueChanged");
	if (msg == NULL)
		return;

	dbus_message_append_args(msg, DBUS_TYPE_OBJECT_PATH, &chr->path,
			DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE,
			&chr->value, chr->vlen, DBUS_TYPE_INVALID);

	dbus_message_set_no_reply(msg, TRUE);
	g_dbus_send_message(conn, msg);
}

static void update_watchers_read_complete(gpointer data, gpointer user_data)
{
	struct watcher *w = data;
	struct read_resp_op *op = user_data;
	struct characteristic *chr = op->chr;
	DBusConnection *conn = w->gatt->conn;
	DBusMessage *msg;
	uint8_t status = op->status;
	INFO("read_resp_watchers chr:%s data[0]:%02X len:%d",chr->path,op->data[0],op->len);

	msg = dbus_message_new_method_call(w->name, w->path,
				"org.bluez.Watcher", "ReadResponse");

	if (msg == NULL)
		return;

	dbus_message_append_args(msg, DBUS_TYPE_OBJECT_PATH, &chr->path,
			DBUS_TYPE_BYTE, &status,
			DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE,
			&op->data, op->len, DBUS_TYPE_INVALID);

	dbus_message_set_no_reply(msg, TRUE);
	g_dbus_send_message(conn, msg);
}

static void update_watchers_write_complete(gpointer data, gpointer user_data)
{
	struct watcher *w = data;
	struct write_resp_op *op = user_data;
	struct characteristic *chr = op->chr;
	uint8_t status = op->status;
	DBusConnection *conn = w->gatt->conn;
	DBusMessage *msg;

	INFO("write_resp_watcher watcher:%s char:%s status:%d",w->path, chr->path, op->status);

	msg = dbus_message_new_method_call(w->name, w->path,
				"org.bluez.Watcher", "WriteResponse");
	if (msg == NULL)
		return;

	dbus_message_append_args(msg, DBUS_TYPE_OBJECT_PATH, &chr->path,
			DBUS_TYPE_BYTE,&status,
			DBUS_TYPE_INVALID);

	dbus_message_set_no_reply(msg, TRUE);
	g_dbus_send_message(conn, msg);
}

static void check_updated_characteristics(struct gatt_service *gatt)
{
	DBusMessage *reply;
	DBusMessageIter iter, array_iter;
	GSList *l, *d;
	uint16_t count = 0;

	if (!att_device_connected(gatt))
		return;

	INFO("%s",gatt->path);

	for (l = gatt->chars; l; l = l->next) {
		struct characteristic *chr = l->data;
		for (d = chr->descriptors; d; d = d->next, count++);
		if (count < chr->descriptor_count) {
			INFO("Not all chars are updated (%s)",chr->path);
			return;
		}
	}

	/* DEBUG ONLY */
	for (l = gatt->chars; l; l = l->next) {
		struct characteristic *chr = l->data;
		for (d = chr->descriptors; d; d = d->next) {
			struct characteristic_descriptor *desc = d->data;
			char str[100];
			bt_uuid_to_string(&desc->uuid,str,100);
			INFO("%s desc:%s handle:%02X",chr->path,str,desc->handle);
		}
	}
	/* END OF DEBUG ONLY */

	reply = dbus_message_new_method_return(gatt->query->msg);

	dbus_message_iter_init_append(reply, &iter);

	dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
				DBUS_TYPE_OBJECT_PATH_AS_STRING, &array_iter);

	for (l = gatt->chars; l; l = l->next) {
		struct characteristic *chr = l->data;

		dbus_message_iter_append_basic(&array_iter,
					DBUS_TYPE_OBJECT_PATH, &chr->path);
	}

	dbus_message_iter_close_container(&iter, &array_iter);

	g_dbus_send_message(gatt->conn, reply);
	INFO("dbus sent");
}

static void add_characteristic_descriptor(struct query_data* user_data,
											uint16_t uuid)
{
	struct characteristic *chr = user_data->chr;
	struct gatt_service *gatt = user_data->gatt;
	struct characteristic_descriptor *desc;
	uint16_t count = 0;
	GSList *d;

	desc = malloc(sizeof(struct characteristic_descriptor));

	desc->handle = user_data->handle;
	bt_uuid16_create(&desc->uuid, uuid);

	chr->descriptors = g_slist_append(chr->descriptors,desc);

	for (d = chr->descriptors; d; d = d->next, count++);
	INFO("%s descriptors count=%d done=%d",chr->path, chr->descriptor_count,
			count);

	check_updated_characteristics(gatt);
}

static void events_handler(const uint8_t *pdu, uint16_t len,
							gpointer user_data)
{
	struct gatt_service *gatt = user_data;
	struct characteristic *chr;
	GSList *l;
	uint8_t opdu[ATT_MAX_MTU];
	guint handle;
	uint16_t olen;

	if (len < 3) {
		DBG("Malformed notification/indication packet (opcode 0x%02x)",
									pdu[0]);
		return;
	}

	handle = att_get_u16(&pdu[1]);

	l = g_slist_find_custom(gatt->chars, GUINT_TO_POINTER(handle),
						characteristic_handle_cmp);

	if (!l)
		return;

	chr = l->data;

	if (chr == NULL) {
		DBG("Attribute handle 0x%02x not found", handle);
		return;
	}

	switch (pdu[0]) {
	case ATT_OP_HANDLE_IND:
		olen = enc_confirmation(opdu, sizeof(opdu));
		g_attrib_send(gatt->attrib, 0, opdu[0], opdu, olen,
						NULL, NULL, NULL);

	case ATT_OP_HANDLE_NOTIFY:
		if (characteristic_set_value(chr, &pdu[3], len - 3) < 0)
			DBG("Can't change Characteristic 0x%02x", handle);

		g_slist_foreach(gatt->watchers, update_watchers_valuechange, chr);
		break;
	}
}

static void offline_char_written(gpointer user_data)
{
	struct characteristic *chr = user_data;
	struct gatt_service *gatt = chr->gatt;

	gatt->offline_chars = g_slist_remove(gatt->offline_chars, chr);

	if (gatt->offline_chars || gatt->watchers)
		return;

	btd_device_remove_attio_callback(gatt->dev, gatt->attioid);
	gatt->attioid = 0;
}

static void offline_char_write(gpointer data, gpointer user_data)
{
	struct characteristic *chr = data;
	GAttrib *attrib = user_data;

	gatt_write_cmd(attrib, chr->handle, chr->value, chr->vlen,
						offline_char_written, chr);
}

static void attio_connected(GAttrib *attrib, gpointer user_data)
{
	struct gatt_service *gatt = user_data;

	gatt->attrib = g_attrib_ref(attrib);

	g_attrib_register(gatt->attrib, ATT_OP_HANDLE_NOTIFY,
					events_handler, gatt, NULL);
	g_attrib_register(gatt->attrib, ATT_OP_HANDLE_IND,
					events_handler, gatt, NULL);

	/*
	 * TODO : we do not support offline writing
	 */
	/*g_slist_foreach(gatt->offline_chars, offline_char_write, attrib);*/

	INFO("%s connected", gatt->path);
}

static void attio_disconnected(gpointer user_data)
{
	struct gatt_service *gatt = user_data;

	if (gatt->query && gatt->query->msg) {
		DBusMessage *reply;

		reply = btd_error_failed(gatt->query->msg,
					"ATT IO channel was disconnected");
		g_dbus_send_message(gatt->conn, reply);
		dbus_message_unref(gatt->query->msg);
		gatt->query->msg = NULL;
	}

	if (gatt->query) {
		g_slist_free_full(gatt->query->list, g_free);
		gatt->query = NULL;
	}

	if (gatt->attrib) {
		g_attrib_cancel_all(gatt->attrib);
		g_attrib_unref(gatt->attrib);
		gatt->attrib = NULL;
	}
}

static DBusMessage *register_watcher(DBusConnection *conn,
						DBusMessage *msg, void *data)
{
	const char *sender = dbus_message_get_sender(msg);
	struct gatt_service *gatt = data;
	struct watcher *watcher;
	char *path;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_OBJECT_PATH, &path,
							DBUS_TYPE_INVALID))
		return btd_error_invalid_args(msg);

	watcher = g_new0(struct watcher, 1);
	watcher->name = g_strdup(sender);
	watcher->gatt = gatt;
	watcher->path = g_strdup(path);
	watcher->id = g_dbus_add_disconnect_watch(conn, sender, watcher_exit,
							watcher, watcher_free);

	INFO("registering watcher on %s",watcher->path);
	if (gatt->attioid == 0) {
		gatt->attioid = btd_device_add_attio_callback(gatt->dev,
							attio_connected,
							attio_disconnected,
							gatt);
	}
	gatt->watchers = g_slist_append(gatt->watchers, watcher);

	return dbus_message_new_method_return(msg);
}

static DBusMessage *unregister_watcher(DBusConnection *conn,
						DBusMessage *msg, void *data)
{
	const char *sender = dbus_message_get_sender(msg);
	struct gatt_service *gatt = data;
	struct watcher *watcher, *match;
	GSList *l;
	char *path;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_OBJECT_PATH, &path,
							DBUS_TYPE_INVALID))
		return btd_error_invalid_args(msg);

	match = g_new0(struct watcher, 1);
	match->name = g_strdup(sender);
	match->path = g_strdup(path);
	l = g_slist_find_custom(gatt->watchers, match, watcher_cmp);
	INFO("unregistering watcher %s",match->path);
	watcher_free(match);
	if (!l)
		return btd_error_not_authorized(msg);


	watcher = l->data;
	g_dbus_remove_watch(conn, watcher->id);
	gatt->watchers = g_slist_remove(gatt->watchers, watcher);
	watcher_free(watcher);

	if (gatt->watchers == NULL && gatt->attioid) {
		btd_device_remove_attio_callback(gatt->dev, gatt->attioid);
		gatt->attioid = 0;
	}

	return dbus_message_new_method_return(msg);
}

static DBusMessage *set_value(DBusConnection *conn, DBusMessage *msg,
			DBusMessageIter *iter, struct characteristic *chr)
{
	struct gatt_service *gatt = chr->gatt;
	DBusMessageIter sub;
	uint8_t *value;
	int len;

	if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_ARRAY ||
			dbus_message_iter_get_element_type(iter) != DBUS_TYPE_BYTE)
		return btd_error_invalid_args(msg);

	dbus_message_iter_recurse(iter, &sub);

	dbus_message_iter_get_fixed_array(&sub, &value, &len);

	characteristic_set_value(chr, value, len);

	if (gatt->attioid == 0)
		gatt->attioid = btd_device_add_attio_callback(gatt->dev,
							attio_connected,
							attio_disconnected,
							gatt);

	if (gatt->attrib)
		gatt_write_cmd(gatt->attrib, chr->handle, value, len,
								NULL, NULL);
	else
		gatt->offline_chars = g_slist_append(gatt->offline_chars, chr);

	return dbus_message_new_method_return(msg);
}

static void set_notifying_cb(gpointer user_data)
{
	struct write_val_op *op = user_data;
	DBusMessage *reply;
	dbus_bool_t notifying;

	notifying = (op->chr->client_config & GATT_CHAR_CLIENTCONF_NOTIFY);
	INFO("chr:%s notifying:%d", op->chr->path,notifying);

	emit_property_changed(op->conn, op->chr->path,
				CHAR_INTERFACE, "Notify",
				DBUS_TYPE_BOOLEAN, &notifying);

	g_free(op);
}

static DBusMessage *set_notifying(DBusConnection *conn, DBusMessage *msg,
			DBusMessageIter *iter, struct characteristic *chr)
{
	struct gatt_service *gatt = chr->gatt;
	DBusMessageIter sub;
	dbus_bool_t notifying;
	uint8_t value[2];
	int len = 2;
	struct write_val_op *op;
	INFO("set_notifying");

	if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_BOOLEAN)
		return btd_error_invalid_args(msg);

	dbus_message_iter_get_basic(iter, &notifying);

	if (chr->ccc_handle == 0) /* Error if char does not have a ccc */
		return btd_error_not_supported(msg);

	if ((chr->perm & GATT_CHAR_PROPS_NOTIFY) == 0) /* Error if char permissions do not allow notifying */
		return btd_error_not_supported(msg);

	if (notifying) {/* we do not allow notification + indicating at the same time */
		chr->client_config = chr->client_config | GATT_CHAR_CLIENTCONF_NOTIFY;
		chr->client_config = chr->client_config & ~GATT_CHAR_CLIENTCONF_INDICATE;
	} else {
		chr->client_config = chr->client_config & ~GATT_CHAR_CLIENTCONF_NOTIFY;
	}

	value[0] = chr->client_config & 0xFF;
	value[1] = 0;

	op = g_new0(struct write_val_op, 1);
	op->msg = 0;
	op->conn = conn;
	op->chr = chr;
	INFO("value[0]=%02X",chr->client_config);

	gatt_write_cmd(chr->gatt->attrib, chr->ccc_handle, (uint8_t*)value, len, set_notifying_cb, op);

	return dbus_message_new_method_return(msg);
}

static void set_indicating_cb(gpointer user_data)
{
	struct write_val_op *op = user_data;
	DBusMessage *reply;
	dbus_bool_t indicating;

	indicating = (op->chr->client_config & GATT_CHAR_CLIENTCONF_INDICATE);
	INFO("chr:%s indicating:%d", op->chr->path,indicating);

	emit_property_changed(op->conn, op->chr->path,
				CHAR_INTERFACE, "Indicate",
				DBUS_TYPE_BOOLEAN, &indicating);

	g_free(op);
}

static DBusMessage *set_indicating(DBusConnection *conn, DBusMessage *msg,
			DBusMessageIter *iter, struct characteristic *chr)
{
	struct gatt_service *gatt = chr->gatt;
	DBusMessageIter sub;
	dbus_bool_t indicating;
	uint8_t value[2];
	int len = 2;
	struct write_val_op *op;

	INFO("set_indicating");

	if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_BOOLEAN)
		return btd_error_invalid_args(msg);

	dbus_message_iter_get_basic(iter, &indicating);

	if (chr->ccc_handle == 0) /* Error if char does not have a ccc */
		return btd_error_not_supported(msg);

	if ((chr->perm & GATT_CHAR_PROPS_INDICATE) == 0) /* Error if char permissions do not allow indicating */
		return btd_error_not_supported(msg);

	if (indicating) { /* we do not allow notification + indicating at the same time */
		chr->client_config = chr->client_config | GATT_CHAR_CLIENTCONF_INDICATE;
		chr->client_config = chr->client_config & ~GATT_CHAR_CLIENTCONF_NOTIFY;
	} else {
		chr->client_config = chr->client_config & ~GATT_CHAR_CLIENTCONF_INDICATE;
	}

	value[0] = chr->client_config & 0xFF;
	value[1] = 0;
	INFO("value[0]=%02X",chr->client_config);

	op = g_new0(struct write_val_op, 1);
	op->msg = 0;
	op->conn = conn;
	op->chr = chr;

	gatt_write_cmd(chr->gatt->attrib, chr->ccc_handle, (uint8_t*)value, len, set_indicating_cb, op);

	return dbus_message_new_method_return(msg);
}

static void set_broadcasting_cb(gpointer user_data)
{
	struct write_val_op *op = user_data;
	DBusMessage *reply;
	dbus_bool_t broadcasting;

	broadcasting = (op->chr->server_config & GATT_CHAR_SERVERCONF_BROADCAST);
	INFO("chr:%s broadcasting:%d", op->chr->path,broadcasting);

	emit_property_changed(op->conn, op->chr->path,
				CHAR_INTERFACE, "Broadcast",
				DBUS_TYPE_BOOLEAN, &broadcasting);

	g_free(op);
}

static DBusMessage *set_broadcasting(DBusConnection *conn, DBusMessage *msg,
			DBusMessageIter *iter, struct characteristic *chr)
{
	struct gatt_service *gatt = chr->gatt;
	DBusMessageIter sub;
	dbus_bool_t broadcasting;
	uint8_t value[2];
	int len = 2;
	struct write_val_op *op;

	INFO("set_broadcasting");

	if (dbus_message_iter_get_arg_type(iter) != DBUS_TYPE_BOOLEAN)
		return btd_error_invalid_args(msg);

	dbus_message_iter_get_basic(iter, &broadcasting);

	if (chr->scc_handle == 0) /* Error if char does not have a ccc */
		return btd_error_not_supported(msg);

	if ((chr->perm & GATT_CHAR_PROPS_BROADCAST) == 0) /* Error if char permissions do not allow broadcasting */
		return btd_error_not_supported(msg);

	if (broadcasting) {
		chr->server_config = chr->server_config | GATT_CHAR_SERVERCONF_BROADCAST;
	} else {
		chr->server_config = chr->server_config & ~GATT_CHAR_SERVERCONF_BROADCAST;
	}

	value[0] = chr->server_config & 0xFF;
	value[1] = 0;

	op = g_new0(struct write_val_op, 1);
	op->msg = 0;
	op->conn = conn;
	op->chr = chr;
	INFO("value[0]=%02X",chr->server_config);


	gatt_write_cmd(chr->gatt->attrib, chr->scc_handle, (uint8_t*)value, len, set_broadcasting_cb, op);

	return dbus_message_new_method_return(msg);
}

static DBusMessage *get_properties(DBusConnection *conn, DBusMessage *msg,
								void *data)
{
	struct characteristic *chr = data;
	DBusMessage *reply;
	DBusMessageIter iter;

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	dbus_message_iter_init_append(reply, &iter);

	append_char_dict(&iter, chr);

	return reply;
}

static DBusMessage *set_property(DBusConnection *conn,
					DBusMessage *msg, void *data)
{
	struct characteristic *chr = data;
	DBusMessageIter iter;
	DBusMessageIter sub;
	const char *property;

	if (!dbus_message_iter_init(msg, &iter))
		return btd_error_invalid_args(msg);

	if (dbus_message_iter_get_arg_type(&iter) != DBUS_TYPE_STRING)
		return btd_error_invalid_args(msg);

	dbus_message_iter_get_basic(&iter, &property);
	dbus_message_iter_next(&iter);

	if (dbus_message_iter_get_arg_type(&iter) != DBUS_TYPE_VARIANT)
		return btd_error_invalid_args(msg);

	dbus_message_iter_recurse(&iter, &sub);

	if (g_str_equal("Value", property))
		return set_value(conn, msg, &sub, chr);

	if (g_str_equal("Notify", property))
		return set_notifying(conn, msg, &sub, chr);

	if (g_str_equal("Indicate", property))
		return set_indicating(conn, msg, &sub, chr);

	if (g_str_equal("Broadcast", property))
		return set_broadcasting(conn, msg, &sub, chr);

	return btd_error_invalid_args(msg);
}

static void read_val_cb(guint8 status, const guint8 *pdu,
					guint16 len, gpointer user_data)
{
	struct characteristic *chr = user_data;
	struct read_resp_op *op;
	const char *error_str = att_ecode2str(status);

	INFO("read_value_cb chr:%s status:%s(%d) pdu[0]:%02X len:%d", chr->path, error_str,status,pdu[0],len);

	if (pdu[0] != ATT_OP_READ_RESP && pdu[0] != ATT_OP_READ_BLOB_RESP) {
		ERROR("Read char value protocol error");
		status = ATT_ECODE_IO;
	}

    op = g_new0(struct read_resp_op,1);
    op->chr = chr;
    op->status = status;
    op->data = (guint8*)pdu + 1;
    op->len = len - 1;
	g_slist_foreach(chr->gatt->watchers, update_watchers_read_complete, op);

	g_free(op);
}

static DBusMessage *read_val(DBusConnection *conn,
					DBusMessage *msg, void *data)
{
	struct characteristic *chr = data;
	DBusMessage *reply = NULL;
	DBusMessageIter iter;
	uint16_t offset = 0;
	uint16_t timeout = 0;
	guint retvalue = 0;

	if (!dbus_message_get_args(msg, NULL,
			DBUS_TYPE_UINT16, &offset,
			DBUS_TYPE_UINT16, &timeout,
			DBUS_TYPE_INVALID))
		return btd_error_invalid_args(msg);

	DBG("ReadValue chr=%s offset=%d timeout=%d",chr->path, offset, timeout);

	retvalue = gatt_read_char(chr->gatt->attrib,chr->handle,offset, read_val_cb,chr);
	if (retvalue == 0)
		return btd_error_failed(msg,"gatt_read_char failed");

	return dbus_message_new_method_return(msg);
}

static void write_req_cb(guint8 status, const guint8 *pdu,
					guint16 len, gpointer user_data)
{
	DBusMessage *reply;
	struct write_val_op *write_op = user_data;
	struct characteristic *chr = write_op->chr;
	struct write_resp_op *resp_op;
	const char *error_str = att_ecode2str(status);

	INFO("write_req_cb chr:%s status:%s(%d)", chr->path, error_str,status);

	reply = dbus_message_new_method_return(write_op->msg);
	g_dbus_send_message(write_op->conn, reply);
	g_free(write_op);

	resp_op = g_new0(struct write_resp_op,1);
	resp_op->chr = chr;
	resp_op->status = status;
	g_slist_foreach(chr->gatt->watchers, update_watchers_write_complete, resp_op);

	g_free(resp_op);
}

static void write_cmd_cb(gpointer user_data)
{
	struct write_val_op *op = user_data;
	DBusMessage *reply;

	INFO("write_cmd_cb chr:%s", op->chr->path);

	reply = dbus_message_new_method_return(op->msg);
	g_dbus_send_message(op->conn, reply);
	g_free(op);
}

static DBusMessage *write_val(DBusConnection *conn,
					DBusMessage *msg, void *data)
{
	DBusMessage *reply;
	struct characteristic *chr = data;
	struct gatt_service *gatt = chr->gatt;
	char* value;
	char* write_method;
	uint16_t len = 0;
	uint16_t offset = 0;
	uint16_t timeout = 0;
	guint retvalue = 0;
	struct write_val_op *op;

	if (!dbus_message_get_args(msg, NULL,
			DBUS_TYPE_ARRAY,DBUS_TYPE_BYTE, &value, &len,
			DBUS_TYPE_STRING, &write_method,
			DBUS_TYPE_UINT16, &offset,
			DBUS_TYPE_UINT16, &timeout,
			DBUS_TYPE_INVALID))
		return btd_error_invalid_args(msg);

	DBG("WriteValue");// chr=%s value[0]=%d len=%d write_method=%s offset=%d timeout=%d",chr->path, value[0], len, write_method, offset, timeout);

	op = g_new0(struct write_val_op, 1);
	op->msg = dbus_message_ref(msg);
	op->conn = conn;
	op->chr = chr;

	if (g_str_equal("WriteWithResponse", write_method)) {
		INFO("write with response");
		retvalue = gatt_write_char(chr->gatt->attrib, chr->handle, (uint8_t*)value, len, write_req_cb, op);
	} else if (g_str_equal("WriteWithoutResponse", write_method)) {
		INFO("write without response");
		retvalue = gatt_write_cmd(chr->gatt->attrib, chr->handle, (uint8_t*)value, len, write_cmd_cb, op);
	} else {
		ERROR("write_method %s is not supported at the moment.",write_method);
		return btd_error_not_supported(msg);
	}
	return NULL;
}

static GDBusMethodTable char_methods[] = {
	{ "GetProperties",	"",	"a{sv}", get_properties },
	{ "SetProperty",	"sv",	"",	set_property},
	{ "WriteValue", "aysqq", "" , write_val,
					G_DBUS_METHOD_FLAG_ASYNC	},
	{ "ReadValue",   "qq",  "" , read_val},
	{ }
};

static GDBusSignalTable char_signals[] = {
	{ "PropertyChanged",		"sv"	},
	{ }
};


static char *characteristic_list_to_string(GSList *chars)
{
	GString *characteristics;
	GSList *l;

	characteristics = g_string_new(NULL);

	for (l = chars; l; l = l->next) {
		struct characteristic *chr = l->data;
		char chr_str[64];

		memset(chr_str, 0, sizeof(chr_str));

		snprintf(chr_str, sizeof(chr_str), "%04X#%02X#%04X#%s ",
				chr->handle, chr->perm, chr->end, chr->type);

		characteristics = g_string_append(characteristics, chr_str);
	}

	return g_string_free(characteristics, FALSE);
}

static void store_characteristics(const bdaddr_t *sba, const bdaddr_t *dba,
						uint16_t start, GSList *chars)
{
	char *characteristics;

	characteristics = characteristic_list_to_string(chars);

	write_device_characteristics(sba, dba, start, characteristics);

	g_free(characteristics);
}

static void register_characteristic(gpointer data, gpointer user_data)
{
	struct characteristic *chr = data;
	DBusConnection *conn = chr->gatt->conn;
	const char *gatt_path = user_data;

	chr->path = g_strdup_printf("%s/characteristic%04x", gatt_path,
								chr->handle);

	g_dbus_register_interface(conn, chr->path, CHAR_INTERFACE,
					char_methods, char_signals, NULL, chr, NULL);

	DBG("Registered: %s", chr->path);
}

static GSList *string_to_characteristic_list(struct gatt_service *gatt,
							const char *str)
{
	GSList *l = NULL;
	char **chars;
	int i;

	if (str == NULL)
		return NULL;

	chars = g_strsplit(str, " ", 0);
	if (chars == NULL)
		return NULL;

	for (i = 0; chars[i]; i++) {
		struct characteristic *chr;
		int ret;

		chr = g_new0(struct characteristic, 1);

		ret = sscanf(chars[i], "%04hX#%02hhX#%04hX#%s", &chr->handle,
				&chr->perm, &chr->end, chr->type);
		if (ret < 4) {
			g_free(chr);
			continue;
		}

		chr->gatt = gatt;
		l = g_slist_append(l, chr);
	}

	g_strfreev(chars);

	return l;
}

static GSList *load_characteristics(struct gatt_service *gatt, uint16_t start)
{
	GSList *chrs_list;
	bdaddr_t sba, dba;
	char *str;

	gatt_get_address(gatt, &sba, &dba);

	str = read_device_characteristics(&sba, &dba, start);
	if (str == NULL)
		return NULL;

	chrs_list = string_to_characteristic_list(gatt, str);

	free(str);

	return chrs_list;
}

static void store_attribute(struct gatt_service *gatt, uint16_t handle,
				uint16_t type, uint8_t *value, gsize len)
{
	bdaddr_t sba, dba;
	bt_uuid_t uuid;
	char *str, *tmp;
	guint i;

	str = g_malloc0(MAX_LEN_UUID_STR + len * 2 + 1);

	bt_uuid16_create(&uuid, type);
	bt_uuid_to_string(&uuid, str, MAX_LEN_UUID_STR);

	str[MAX_LEN_UUID_STR - 1] = '#';

	for (i = 0, tmp = str + MAX_LEN_UUID_STR; i < len; i++, tmp += 2)
		sprintf(tmp, "%02X", value[i]);

	gatt_get_address(gatt, &sba, &dba);

	write_device_attribute(&sba, &dba, handle, str);

	g_free(str);
}

static void query_list_append(struct gatt_service *gatt, struct query_data *data)
{
	struct query *query = gatt->query;

	query->list = g_slist_append(query->list, data);
}

static void query_list_remove(struct gatt_service *gatt, struct query_data *data)
{
	struct query *query = gatt->query;

	query->list = g_slist_remove(query->list, data);
	if (query->list != NULL)
		return;

	btd_device_remove_attio_callback(gatt->dev, query->attioid);
	g_free(query);

	gatt->query = NULL;
}

static void update_char_desc(guint8 status, const guint8 *pdu, guint16 len,
							gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;
	INFO("%s",chr->path);

	if (status == 0) {

		g_free(chr->desc);

		chr->desc = g_malloc(len);
		memcpy(chr->desc, pdu + 1, len - 1);
		chr->desc[len - 1] = '\0';

		store_attribute(gatt, current->handle,
				GATT_CHARAC_USER_DESC_UUID,
				(void *) chr->desc, len);
	} else if (status == ATT_ECODE_INSUFF_ENC) {
		GIOChannel *io = g_attrib_get_channel(gatt->attrib);
		BtIOSecLevel level = BT_IO_SEC_HIGH;

		bt_io_get(io, BT_IO_L2CAP, NULL,
				BT_IO_OPT_SEC_LEVEL, &level,
				BT_IO_OPT_INVALID);

		if (level < BT_IO_SEC_HIGH)
			level++;

		if (bt_io_set(io, BT_IO_L2CAP, NULL,
				BT_IO_OPT_SEC_LEVEL, level,
				BT_IO_OPT_INVALID)) {
			gatt_read_char(gatt->attrib, current->handle, 0,
					update_char_desc, current);
			return;
		}
	}

	add_characteristic_descriptor(current, GATT_CHARAC_USER_DESC_UUID);
	query_list_remove(gatt, current);
	g_free(current);
}

static void update_char_format(guint8 status, const guint8 *pdu, guint16 len,
								gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;

	INFO("%s",chr->path);

	if (status != 0)
		goto done;

	if (len < 8)
		goto done;

	g_free(chr->format);

	chr->format = g_new0(struct format, 1);
	memcpy(chr->format, pdu + 1, 7);

	store_attribute(gatt, current->handle, GATT_CHARAC_FMT_UUID,
				(void *) chr->format, sizeof(*chr->format));

done:
	add_characteristic_descriptor(current, GATT_CHARAC_FMT_UUID);
	query_list_remove(gatt, current);
	g_free(current);
}

static void update_char_client_conf(guint8 status, const guint8 *pdu, guint16 len,
								gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;

	INFO("%s",chr->path);

	if (status != 0)
		goto done;

	chr->client_config = pdu[1];
	chr->ccc_handle = current->handle;

	store_attribute(gatt, current->handle, GATT_CLIENT_CHARAC_CFG_UUID,
				(void*)&chr->client_config, sizeof(chr->client_config));

done:
	add_characteristic_descriptor(current, GATT_CLIENT_CHARAC_CFG_UUID);
	query_list_remove(gatt, current);
	g_free(current);
}

static void update_char_server_conf(guint8 status, const guint8 *pdu, guint16 len,
								gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;

	INFO("%s",chr->path);

	if (status != 0)
		goto done;

	chr->server_config = att_get_u16(pdu + 1);
	chr->scc_handle = current->handle;

	store_attribute(gatt, current->handle, GATT_SERVER_CHARAC_CFG_UUID,
				(void*)&chr->server_config, sizeof(chr->server_config));

done:
	add_characteristic_descriptor(current, GATT_SERVER_CHARAC_CFG_UUID);
	query_list_remove(gatt, current);
	g_free(current);
}

static void update_char_ext_props(guint8 status, const guint8 *pdu, guint16 len,
								gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;

	INFO("%s",chr->path);

	if (status != 0)
		goto done;

	chr->ext_properties = att_get_u16(pdu + 1);
	chr->extprops_handle = current->handle;

	store_attribute(gatt, current->handle, GATT_CHARAC_EXT_PROPER_UUID,
				(void*)&chr->ext_properties, sizeof(chr->ext_properties));

done:
	add_characteristic_descriptor(current, GATT_CHARAC_EXT_PROPER_UUID);
	query_list_remove(gatt, current);
	g_free(current);
}


static void update_char_value(guint8 status, const guint8 *pdu,
					guint16 len, gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;

	if (status == 0)
		characteristic_set_value(chr, pdu + 1, len - 1);
	else if (status == ATT_ECODE_INSUFF_ENC) {
		GIOChannel *io = g_attrib_get_channel(gatt->attrib);

		if (bt_io_set(io, BT_IO_L2CAP, NULL,
				BT_IO_OPT_SEC_LEVEL, BT_IO_SEC_HIGH,
				BT_IO_OPT_INVALID)) {
			gatt_read_char(gatt->attrib, chr->handle, 0,
					update_char_value, current);
			return;
		}
	}

	query_list_remove(gatt, current);
	g_free(current);
}

static int uuid_desc16_cmp(bt_uuid_t *uuid, guint16 desc)
{
	bt_uuid_t u16;

	bt_uuid16_create(&u16, desc);

	return bt_uuid_cmp(uuid, &u16);
}

static void descriptor_cb(guint8 status, const guint8 *pdu, guint16 plen,
							gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	struct characteristic *chr = current->chr;
	struct att_data_list *list;
	guint8 format;
	int i;

	INFO("%s status=%d",chr->path,status);

	if (status != 0) {
		chr->descriptor_count = 0;
		if (att_device_connected(gatt))
			goto update;
		else
			goto done;
	}

	list = dec_find_info_resp(pdu, plen, &format);
	if (list == NULL) {
		INFO("%s no descriptors",chr->path);
		chr->descriptor_count = 0;
		goto update;
	}

	INFO("%s %d descriptors",chr->path,list->num);

	chr->descriptor_count = list->num;
	for (i = 0; i < list->num; i++) {
		guint16 handle;
		bt_uuid_t uuid;
		uint8_t *dinfo = list->data[i];
		struct query_data *qfmt;
		guint ret = 0;

		handle = att_get_u16(dinfo);

		if (format == 0x01) {
			uuid = att_get_uuid16(&dinfo[2]);
		} else {
			/* Currently, only "user description" and "presentation
			 * format" descriptors are used, and both have 16-bit
			 * UUIDs. Therefore there is no need to support format
			 * 0x02 yet. */
			INFO("unkown format");
			add_characteristic_descriptor(current,0x0000);
			continue;
		}
		qfmt = g_new0(struct query_data, 1);
		qfmt->gatt = current->gatt;
		qfmt->chr = current->chr;
		qfmt->handle = handle;

		if (uuid_desc16_cmp(&uuid, GATT_CHARAC_USER_DESC_UUID) == 0) {
			query_list_append(gatt, qfmt);
			ret = gatt_read_char(gatt->attrib, handle, 0, update_char_desc,
									qfmt);
		} else if (uuid_desc16_cmp(&uuid, GATT_CHARAC_FMT_UUID) == 0) {
			query_list_append(gatt, qfmt);
			ret = gatt_read_char(gatt->attrib, handle, 0,
						update_char_format, qfmt);
		} else if (uuid_desc16_cmp(&uuid, GATT_CLIENT_CHARAC_CFG_UUID) == 0) {
			query_list_append(gatt, qfmt);
			ret = gatt_read_char(gatt->attrib, handle, 0,
						update_char_client_conf, qfmt);
		} else if (uuid_desc16_cmp(&uuid, GATT_SERVER_CHARAC_CFG_UUID) == 0) {
			query_list_append(gatt, qfmt);
			ret = gatt_read_char(gatt->attrib, handle, 0,
						update_char_server_conf, qfmt);
		} else if (uuid_desc16_cmp(&uuid, GATT_CHARAC_EXT_PROPER_UUID) == 0) {
			query_list_append(gatt, qfmt);
			ret = gatt_read_char(gatt->attrib, handle, 0,
						update_char_ext_props, qfmt);
		} else
			g_free(qfmt);

		if (ret == 0)
			add_characteristic_descriptor(current,0x0000);
	}

	att_data_list_free(list);
update:
	check_updated_characteristics(gatt);
done:
	query_list_remove(gatt, current);
	g_free(current);
}

static void update_all_chars(gpointer data, gpointer user_data)
{
	struct query_data *qdesc, *qvalue;
	struct characteristic *chr = data;
	struct gatt_service *gatt = user_data;

	INFO("%s",chr->path);

	qdesc = g_new0(struct query_data, 1);
	qdesc->gatt = gatt;
	qdesc->chr = chr;

	query_list_append(gatt, qdesc);

	gatt_find_info(gatt->attrib, chr->handle + 1, chr->end, descriptor_cb,
									qdesc);
}

static void char_discovered_cb(GSList *characteristics, guint8 status,
							gpointer user_data)
{
	struct query_data *current = user_data;
	struct gatt_service *gatt = current->gatt;
	uint16_t *previous_end = NULL;
	GSList *l;
	bdaddr_t sba, dba;

	INFO("char_discovered_cb status=%02X",status);

	if (status != 0) {
		const char *str = att_ecode2str(status);

		DBG("failed: %s", str);


		goto fail;
	}

	for (l = characteristics; l; l = l->next) {
		struct gatt_char *current_chr = l->data;
		struct characteristic *chr;
		guint handle = current_chr->value_handle;
		GSList *lchr;

		lchr = g_slist_find_custom(gatt->chars,
			GUINT_TO_POINTER(handle), characteristic_handle_cmp);
		if (lchr)
			continue;

		chr = g_new0(struct characteristic, 1);
		chr->gatt = gatt;
		chr->perm = current_chr->properties;
		chr->handle = current_chr->value_handle;
		chr->descriptor_count = 0;
		strncpy(chr->type, current_chr->uuid, sizeof(chr->type));

		if (previous_end)
			*previous_end = current_chr->handle;

		previous_end = &chr->end;

		gatt->chars = g_slist_append(gatt->chars, chr);

		DBG("uuid:%s val_handle:%02X props:%02X", chr->type, chr->handle, chr->perm);
	}

	if (previous_end)
		*previous_end = gatt->prim->range.end;

	gatt_get_address(gatt, &sba, &dba);

	store_characteristics(&sba, &dba, gatt->prim->range.start, gatt->chars);
	g_slist_foreach(gatt->chars, register_characteristic, gatt->path);

	g_slist_foreach(gatt->chars, update_all_chars, gatt);

fail:
	query_list_remove(gatt, current);

	g_free(current);
}

static void send_discover(GAttrib *attrib, gpointer user_data)
{
	struct query_data *qchr = user_data;
	struct gatt_service *gatt = qchr->gatt;
	struct gatt_primary *prim = gatt->prim;

	gatt->attrib = g_attrib_ref(attrib);

	gatt_discover_char(gatt->attrib, prim->range.start, prim->range.end, NULL,
						char_discovered_cb, qchr);
}

static void cancel_discover(gpointer user_data)
{
	struct query_data *qchr = user_data;
	struct gatt_service *gatt = qchr->gatt;

	g_attrib_unref(gatt->attrib);
	gatt->attrib = NULL;
}

static DBusMessage *discover_char(DBusConnection *conn, DBusMessage *msg,
								void *data)
{
	struct gatt_service *gatt = data;
    struct query *query;
	struct query_data *qchr;
	INFO("discovering characteristics on %s",gatt->path);

	query = g_new0(struct query, 1);

	qchr = g_new0(struct query_data, 1);
	qchr->gatt = gatt;
	query->msg = dbus_message_ref(msg);

	gatt->query = query;

	query_list_append(gatt, qchr);

	if (gatt->attrib == 0) {
		INFO("trying to connect...");
		query->attioid = btd_device_add_attio_callback(gatt->dev,
								send_discover,
								cancel_discover,
								qchr);
	} else {
		INFO("using existing connection");
		gatt_discover_char(gatt->attrib, gatt->prim->range.start, gatt->prim->range.end, NULL,
							char_discovered_cb, qchr);

	}

	return NULL;
}

static DBusMessage *prim_get_properties(DBusConnection *conn, DBusMessage *msg,
								void *data)
{
	struct gatt_service *gatt = data;
	DBusMessage *reply;
	DBusMessageIter iter;
	DBusMessageIter dict;
	GSList *l;
	char **chars;
	const char *uuid;
	int i;

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	dbus_message_iter_init_append(reply, &iter);

	dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
			DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
			DBUS_TYPE_STRING_AS_STRING DBUS_TYPE_VARIANT_AS_STRING
			DBUS_DICT_ENTRY_END_CHAR_AS_STRING, &dict);

	chars = g_new0(char *, g_slist_length(gatt->chars) + 1);

	for (i = 0, l = gatt->chars; l; l = l->next, i++) {
		struct characteristic *chr = l->data;
		chars[i] = chr->path;
	}

	dict_append_array(&dict, "Characteristics", DBUS_TYPE_OBJECT_PATH,
								&chars, i);
	uuid = gatt->prim->uuid;
	dict_append_entry(&dict, "UUID", DBUS_TYPE_STRING, &uuid);

	g_free(chars);

	dbus_message_iter_close_container(&iter, &dict);

	return reply;
}

static GDBusMethodTable prim_methods[] = {
	{ "DiscoverCharacteristics",	"",	"ao",	discover_char,
					G_DBUS_METHOD_FLAG_ASYNC	},
	{ "RegisterCharacteristicsWatcher",	"o", "",
						register_watcher	},
	{ "UnregisterCharacteristicsWatcher",	"o", "",
						unregister_watcher	},
	{ "GetProperties",	"",	"a{sv}",prim_get_properties	},
	{ }
};

static struct gatt_service *primary_register(DBusConnection *conn,
						struct btd_device *device,
						struct gatt_primary *prim,
						int psm)
{
	struct gatt_service *gatt;
	const char *device_path;

	device_path = device_get_path(device);

	gatt = g_new0(struct gatt_service, 1);
	gatt->dev = btd_device_ref(device);
	gatt->prim = prim;
	gatt->psm = psm;
	gatt->conn = dbus_connection_ref(conn);
	gatt->path = g_strdup_printf("%s/service%04x", device_path,
								prim->range.start);

	g_dbus_register_interface(gatt->conn, gatt->path,
					CHAR_INTERFACE, prim_methods,
					NULL, NULL, gatt, NULL);
	gatt->chars = load_characteristics(gatt, prim->range.start);
	g_slist_foreach(gatt->chars, register_characteristic, gatt->path);

	return gatt;
}

GSList *attrib_client_register(DBusConnection *connection,
					struct btd_device *device, int psm,
					GAttrib *attrib, GSList *primaries)
{
	GSList *l, *services;

	for (l = primaries, services = NULL; l; l = l->next) {
		struct gatt_primary *prim = l->data;
		struct gatt_service *gatt;

		gatt = primary_register(connection, device, prim, psm);

		DBG("Registered: %s", gatt->path);

		services = g_slist_append(services, g_strdup(gatt->path));
		gatt_services = g_slist_append(gatt_services, gatt);

	}

	return services;
}

static void primary_unregister(struct gatt_service *gatt)
{
	GSList *l;

	if (gatt->attioid)
		btd_device_remove_attio_callback(gatt->dev, gatt->attioid);

	for (l = gatt->chars; l; l = l->next) {
		struct characteristic *chr = l->data;
		g_dbus_unregister_interface(gatt->conn, chr->path,
							CHAR_INTERFACE);
	}

	g_dbus_unregister_interface(gatt->conn, gatt->path, CHAR_INTERFACE);
}

static int path_cmp(gconstpointer data, gconstpointer user_data)
{
	const char *path = data;
	const char *gatt_path = user_data;

	return g_strcmp0(path, gatt_path);
}

void attrib_client_unregister(GSList *services)
{
	GSList *l, *left;

	for (l = gatt_services, left = NULL; l; l = l->next) {
		struct gatt_service *gatt = l->data;

		if (!g_slist_find_custom(services, gatt->path, path_cmp)) {
			left = g_slist_append(left, gatt);
			continue;
		}

		primary_unregister(gatt);
		gatt_service_free(gatt);
	}

	g_slist_free(gatt_services);
	gatt_services = left;
}
