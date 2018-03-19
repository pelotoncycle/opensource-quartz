/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2012 Texas Instruments Corporation
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

#include <gdbus.h>
#include <glib.h>
#include <errno.h>
#include <stdlib.h>
#include <bluetooth/uuid.h>
#include <adapter.h>

#include "manager.h"
#include "att.h"
#include "gattrib.h"
#include "attio.h"
#include "textfile.h"
#include "storage.h"
#include "attrib-server.h"
#include "gatt-service.h"
#include "log.h"
#include "server.h"
#include "error.h"
#include "device.h"

#define BLUEZ_INTERFACE "org.bluez"
#define GGSP_INTERFACE "org.bluez.ggsp"
#define GGSP_AGENT_INTERFACE "org.bluez.ggsp_agent"

#define CALLBACK_TIMEOUT 5000

#define ggsp_err(fmt, arg...) error("[%s:%s()] " fmt, __FILE__, __func__, ##arg)

struct ggsp_adapter {
	struct btd_adapter *adapter;
	DBusConnection *conn;
	gchar *service_dest;
	gchar *service_path;
	guint watch;
};

struct indication_data {
	struct ggsp_adapter *ga;
	uint32_t cookie;
};

static bool initialized;

static GSList *ggsp_adapters;

static void ind_result_cb(guint8 status, struct btd_adapter *adapter,
			  struct btd_device *device, gpointer user_data)
{
	struct indication_data *ind_data = user_data;
	struct ggsp_adapter *ga = ind_data->ga;
	const char *device_path;
	struct DBusMessage *msg;
	int ret;

	DBG("%d", status);

	device_path = device_get_path(device);
	if (!device_path) {
		ggsp_err("Invalid device path");
		goto exit;
	}

	msg = dbus_message_new_method_call(ga->service_dest, ga->service_path,
					   GGSP_AGENT_INTERFACE, "IndicationResult");
	if (!msg) {
		ggsp_err("dbus_message_new_method_call failed");
		goto exit;
	}

	if (!dbus_message_append_args(msg,
				      DBUS_TYPE_OBJECT_PATH, &device_path,
				      DBUS_TYPE_UINT32, &ind_data->cookie,
				      DBUS_TYPE_BYTE, &status,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		goto free_msg;
	}

	dbus_connection_send(ga->conn, msg, NULL);

free_msg:
	if (msg)
		dbus_message_unref(msg);

exit:
	g_free(ind_data);
}

static uint8_t ggsp_attrib_readcb(struct attribute *a, gpointer user_data,
				gpointer data)
{
	struct ggsp_adapter *ga = user_data;
	struct btd_device *device = data;
	struct DBusMessage *msg, *reply;
	struct DBusError err;
	uint8_t *val;
	uint32_t len;
	uint8_t ret = ATT_ECODE_IO, cb_ret;
	const char *device_path;

	dbus_error_init(&err);

	msg = dbus_message_new_method_call(ga->service_dest, ga->service_path,
					   GGSP_AGENT_INTERFACE, "ReadCB");
	if (!msg) {
		ggsp_err("dbus_message_new_method_call failed");
		goto free_err;
	}

	device_path = device_get_path(device);
	if (!device_path) {
		ggsp_err("Invalid device path");
		goto free_msg;
	}

	if (!dbus_message_append_args(msg,
				      DBUS_TYPE_OBJECT_PATH, &device_path,
				      DBUS_TYPE_UINT16, &a->handle,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		goto free_msg;
	}

	reply = dbus_connection_send_with_reply_and_block(ga->conn, msg,
							  CALLBACK_TIMEOUT,
							  &err);
	if (dbus_error_is_set(&err)) {
		ggsp_err("Service read callback failed: %s", err.message);
		goto free_msg;
	}

	if (!dbus_message_get_args(reply, NULL,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE, &val, &len,
				   DBUS_TYPE_BYTE, &cb_ret,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		goto free_reply;
	}

	if (attrib_db_update(ga->adapter, a->handle, NULL, (uint8_t *)val, len,
			     NULL)) {
		ggsp_err("Failed updating db");
		goto free_reply;
	}

	ret = cb_ret;

free_reply:
	if (reply)
		dbus_message_unref(reply);

free_msg:
	if (msg)
		dbus_message_unref(msg);

free_err:
	dbus_error_free(&err);
	return ret;
}

static uint8_t ggsp_attrib_writecb(struct attribute *a, gpointer user_data,
				   gpointer data)
{
	struct ggsp_adapter *ga = user_data;
	struct btd_device *device = data;
	struct DBusMessage *msg, *reply;
	struct DBusError err;
	uint8_t ret = ATT_ECODE_IO, cb_ret;
	const char *device_path;

	dbus_error_init(&err);

	msg = dbus_message_new_method_call(ga->service_dest, ga->service_path,
					   GGSP_AGENT_INTERFACE, "WriteCB");
	if (!msg) {
		ggsp_err("dbus_message_new_method_call failed");
		goto free_err;
	}

	device_path = device_get_path(device);
	if (!device_path) {
		ggsp_err("Invalid device path");
		goto free_msg;
	}

	if (!dbus_message_append_args(msg,
				      DBUS_TYPE_OBJECT_PATH, &device_path,
				      DBUS_TYPE_UINT16, &a->handle,
				      DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE,
				      &a->data, a->len,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		goto free_msg;
	}

	reply = dbus_connection_send_with_reply_and_block(ga->conn, msg,
							  CALLBACK_TIMEOUT,
							  &err);
	if (dbus_error_is_set(&err)) {
		ggsp_err("Service read callback failed: %s", err.message);
		goto free_msg;
	}

	if (!dbus_message_get_args(reply, NULL,
				   DBUS_TYPE_BYTE, &cb_ret,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		goto free_reply;
	}

	ret = cb_ret;

free_reply:
	if (reply)
		dbus_message_unref(reply);

free_msg:
	if (msg)
		dbus_message_unref(msg);

free_err:
	dbus_error_free(&err);
	return ret;
}

static DBusMessage *ggsp_register(DBusConnection *conn,
				  DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	uint32_t nitems;
	uint8_t *uuid_bytes;
	uint32_t uuid_len;
	bt_uuid_t uuid;
	uint16_t handle;
	const gchar *path;

	if (initialized) {
		ggsp_err("Already initialized");
		return btd_error_already_exists(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_OBJECT_PATH, &path,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	g_free(ga->service_path);
	g_free(ga->service_dest);
	ga->service_path = g_strdup(path);
	ga->service_dest = g_strdup(dbus_message_get_sender(msg));
	if (!ga->service_dest || !ga->service_path) {
		ga->service_path = NULL;
		ga->service_dest = NULL;
		return btd_error_failed(msg, "g_strdup");
	}

	DBG("Registered dest=%s, path=%s", ga->service_dest, ga->service_path);
	initialized = true;

	return dbus_message_new_method_return(msg);
}

static DBusMessage *ggsp_find_available(DBusConnection *conn,
					     DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	uint32_t nitems;
	uint8_t *uuid_bytes;
	uint32_t uuid_len;
	bt_uuid_t uuid;
	uint16_t handle;
	DBusMessage *reply;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE,
				   &uuid_bytes, &uuid_len,
				   DBUS_TYPE_UINT16, &nitems,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	if (!nitems) {
		ggsp_err("Invalid nitems: %d", nitems);
		return btd_error_invalid_args(msg);
	}

	switch (uuid_len) {
	case 2:
		bt_uuid16_create(&uuid, att_get_u16(uuid_bytes));
		break;
	case 16:
		bt_uuid128_create(&uuid, att_get_u128(uuid_bytes));
		break;
	default:
		ggsp_err("Invalid uuid size: %d", uuid_len);
		return btd_error_invalid_args(msg);
	};

	handle = attrib_db_find_avail(ga->adapter, &uuid, nitems);

	DBG("Found handle 0x%X for %d items", handle, nitems);

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	if (!dbus_message_append_args(reply, DBUS_TYPE_UINT16, &handle,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		return NULL;
	}

	return reply;
}

static DBusMessage *ggsp_add_attribute(DBusConnection *conn,
				       DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	uint16_t handle;
	uint8_t *uuid_bytes;
	uint32_t uuid_len;
	bt_uuid_t uuid;
	uint32_t read_reqs, write_reqs;
	uint8_t *val;
	uint32_t len;
	dbus_bool_t register_callbacks;
	struct attribute *attr;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_UINT16, &handle,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE,
				   &uuid_bytes, &uuid_len,
				   DBUS_TYPE_UINT32, &read_reqs,
				   DBUS_TYPE_UINT32, &write_reqs,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE, &val, &len,
				   DBUS_TYPE_BOOLEAN, &register_callbacks,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	if (!handle || (read_reqs > ATT_NOT_PERMITTED) ||
	    (write_reqs > ATT_NOT_PERMITTED)) {
		ggsp_err("Invalid arguments");
		return btd_error_invalid_args(msg);
	}

	switch (uuid_len) {
	case 2:
		bt_uuid16_create(&uuid, att_get_u16(uuid_bytes));
		break;
	case 16:
		bt_uuid128_create(&uuid, att_get_u128(uuid_bytes));
		break;
	default:
		ggsp_err("Invalid uuid size: %d", uuid_len);
		return btd_error_invalid_args(msg);
	};

	attr = attrib_db_add(ga->adapter, handle, &uuid, read_reqs, write_reqs,
			     val, len);
	if (!attr) {
		ggsp_err("attrib_db_add failed");
		return btd_error_failed(msg, "attrib_db_add");
	}

	DBG("Added attribute %p at handle 0x%X", attr, handle);

	if (register_callbacks) {
		DBG("Registering callbacks");
		attr->read_cb = ggsp_attrib_readcb;
		attr->write_cb = ggsp_attrib_writecb;
		attr->cb_user_data = ga;
	}

	return dbus_message_new_method_return(msg);
}

static DBusMessage *ggsp_remove_attribute(DBusConnection *conn,
					DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	uint16_t handle;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_UINT16, &handle,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	DBG("removing attribute %d", handle);
	if (attrib_db_del(ga->adapter, handle))
		ggsp_err("failed deleting attribute handle %d", handle);

	return dbus_message_new_method_return(msg);
}

static DBusMessage *ggsp_send_notification(DBusConnection *conn,
					   DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	const gchar *addr_str;
	bdaddr_t addr;
	uint16_t handle, handle_ccc;
	uint8_t *val;
	uint32_t len;
	struct attribute *attr;
	uint32_t sent;
	DBusMessage *reply;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_STRING, &addr_str,
				   DBUS_TYPE_UINT16, &handle,
				   DBUS_TYPE_UINT16, &handle_ccc,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE, &val, &len,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	if (str2ba(addr_str, &addr)) {
		ggsp_err("Invalid bdaddr");
		return btd_error_invalid_args(msg);
	}

	if (!handle || !handle_ccc || !len) {
		ggsp_err("Invalid arguments");
		return btd_error_invalid_args(msg);
	}

	sent = queue_notification(ga->adapter, &addr, handle, handle_ccc, val,
				  len);

	DBG("Notification: handle=0x%x, handle_ccc=0x%X, sent = %d", handle,
	    handle_ccc, sent);

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	if (!dbus_message_append_args(reply, DBUS_TYPE_UINT32, &sent,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		return NULL;
	}

	return reply;
}

static DBusMessage *ggsp_send_indication(DBusConnection *conn,
					 DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	const gchar *addr_str;
	bdaddr_t addr;
	uint16_t handle, handle_ccc;
	uint8_t *val;
	uint32_t len;
	uint32_t cookie;
	struct attribute *attr;
	struct indication_data *ind_data;
	uint32_t sent;
	DBusMessage *reply;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_STRING, &addr_str,
				   DBUS_TYPE_UINT16, &handle,
				   DBUS_TYPE_UINT16, &handle_ccc,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE, &val, &len,
				   DBUS_TYPE_UINT32, &cookie,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	if (str2ba(addr_str, &addr)) {
		ggsp_err("Invalid bdaddr");
		return btd_error_invalid_args(msg);
	}

	if (!handle || !handle_ccc || !len) {
		ggsp_err("Invalid arguments");
		return btd_error_invalid_args(msg);
	}

	ind_data = g_new0(struct indication_data, 1);
	ind_data->ga = ga;
	ind_data->cookie = cookie;

	sent = queue_indication(ga->adapter, &addr, handle, handle_ccc, val,
				len, ind_result_cb, ind_data);

	DBG("Indication: handle=0x%x, handle_ccc=0x%X, sent = %d", handle,
	    handle_ccc, sent);

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	if (!dbus_message_append_args(reply, DBUS_TYPE_UINT32, &sent,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		return NULL;
	}

	return reply;
}

static DBusMessage *ggsp_invalidate_cache(DBusConnection *conn,
					  DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;

	DBG("");

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	attrib_invalidate_attributes(ga->adapter);

	return dbus_message_new_method_return(msg);
}

static DBusMessage *ggsp_register_sdp(DBusConnection *conn,
				      DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	uint16_t handle;
	uint32_t sdp_handle;
	const gchar *name;
	DBusMessage *reply;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_UINT16, &handle,
				   DBUS_TYPE_STRING, &name,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	if (!handle || !strlen(name)) {
		ggsp_err("Invalid arguments");
		return btd_error_invalid_args(msg);
	}

	sdp_handle = attrib_create_sdp(ga->adapter, handle, name);
	if (!sdp_handle)
		ggsp_err("attrib_create_sdp failed");

	DBG("ATT SDP record (0x%X): Handle 0x%X, name %s", sdp_handle, handle,
	    name);

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	if (!dbus_message_append_args(reply, DBUS_TYPE_UINT32, &sdp_handle,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		return NULL;
	}

	return reply;
}

static DBusMessage *ggsp_unregister_sdp(DBusConnection *conn,
					DBusMessage *msg, void *data)
{
	struct ggsp_adapter *ga = data;
	uint32_t sdp_handle;

	if (!initialized) {
		ggsp_err("Not registered");
		return btd_error_not_ready(msg);
	}

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_UINT32, &sdp_handle,
				   DBUS_TYPE_INVALID)) {
		ggsp_err("Invalid arguments");
		return NULL;
	}

	DBG("free SDP handle %d", sdp_handle);
	attrib_free_sdp(sdp_handle);
	return dbus_message_new_method_return(msg);
}

static void tx_power_read_cb(struct btd_adapter *adapter,
			     struct btd_device *dev, int8_t level,
			     gpointer user_data)
{
	struct ggsp_adapter *ga = user_data;
	const char *device_path;
	struct DBusMessage *msg;
	int ret;

	DBG("Updating tx power level signal of dev %p to %d", dev, level);

	if (!initialized) {
		ggsp_err("Not registered");
		return;
	}

	device_path = device_get_path(dev);
	if (!device_path) {
		ggsp_err("Invalid device path");
		return;
	}

	msg = dbus_message_new_method_call(ga->service_dest, ga->service_path,
					   GGSP_AGENT_INTERFACE, "TxPower");
	if (!msg) {
		ggsp_err("dbus_message_new_method_call failed");
		return;
	}

	if (!dbus_message_append_args(msg,
				      DBUS_TYPE_OBJECT_PATH, &device_path,
				      DBUS_TYPE_BYTE, &level,
				      DBUS_TYPE_INVALID)) {
		ggsp_err("dbus_message_append_args failed");
		goto free_msg;
	}

	dbus_connection_send(ga->conn, msg, NULL);

free_msg:
	if (msg)
		dbus_message_unref(msg);
}

static gboolean handle_property_change(DBusConnection *conn, DBusMessage *msg,
				       void *data)
{
	struct ggsp_adapter *ga = data;
	DBusMessageIter iter, sub;
	const char *property;
	dbus_bool_t conn_state;
	struct btd_device *device;
	const char *obj_path = dbus_message_get_path(msg);

	DBG("path %s", obj_path);

	dbus_message_iter_init(msg, &iter);

	if (dbus_message_iter_get_arg_type(&iter) != DBUS_TYPE_STRING) {
		error("Unexpected signature in device PropertyChanged signal");
		return TRUE;
	}

	dbus_message_iter_get_basic(&iter, &property);
	DBG("property %s", property);

	dbus_message_iter_next(&iter);
	dbus_message_iter_recurse(&iter, &sub);
	if (!g_str_equal(property, "Connected"))
		return TRUE;

	device = adapter_get_device_by_path(ga->adapter, obj_path);
	if (!device)
		return TRUE;

	dbus_message_iter_get_basic(&sub, &conn_state);
	DBG("Connected %d", conn_state);
	if (conn_state) {
		/* we only care about LE devices */
		if (device_is_bredr(device))
			return TRUE;

		/*
		 * Get the Tx power to this device. It cannot change during
		 * an LE connection.
		 */
		adapter_read_tx_power(ga->adapter, device,
				      TX_POWER_CURRENT_POWER,
				      tx_power_read_cb, ga);
	}

	return TRUE;
}

static GDBusMethodTable ggsp_methods[] = {
	{"Register", "o", "", ggsp_register},
	{"FindAvailable", "ayq", "q", ggsp_find_available},
	{"AddAttribute", "qayuuayb", "", ggsp_add_attribute},
	{"RemoveAttribute", "q", "", ggsp_remove_attribute},
	{"SendNotification", "sqqay", "u", ggsp_send_notification},
	{"SendIndication", "sqqayu", "u", ggsp_send_indication},
	{"InvalidateCache", "", "", ggsp_invalidate_cache},
	{"RegisterSDPRecord", "qs", "u", ggsp_register_sdp},
	{"UnregisterSDPRecord", "u", "", ggsp_unregister_sdp},
	{}
};

static int ggsp_server_probe(struct btd_adapter *adapter)
{
	struct ggsp_adapter *ga;
	const gchar *path;

	path = adapter_get_path(adapter);
	if (!path)
		goto exit;

	ga = g_new0(struct ggsp_adapter, 1);
	ga->adapter = adapter;
	ga->conn = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (ga->conn == NULL)
		goto free_ga;

	if (!g_dbus_register_interface(ga->conn, path, GGSP_INTERFACE,
				       ggsp_methods, NULL, NULL, ga, NULL)) {
		ggsp_err("D-Bus failed to register %s:%s", GGSP_INTERFACE,
			 path);
		goto unref_connection;
	}

	DBG("Registered interface on %s:%s", GGSP_INTERFACE, path);

	ggsp_adapters = g_slist_append(ggsp_adapters, ga);

	/* watch for connecting/disconnecting devices */
	ga->watch = g_dbus_add_signal_watch(ga->conn, BLUEZ_INTERFACE, NULL,
					    DEVICE_INTERFACE, "PropertyChanged",
					    handle_property_change, ga, NULL);

	return 0;

unref_connection:
	dbus_connection_unref(ga->conn);

free_ga:
	g_free(ga);

exit:
	return -1;
}

static int ga_cmp(gconstpointer a, gconstpointer b)
{
	const struct ggsp_adapter *ga = a;
	const struct btd_adapter *adapter = b;

	if (ga->adapter == adapter)
		return 0;

	return -1;
}

static void ggsp_server_remove(struct btd_adapter *adapter)
{
	struct ggsp_adapter *ga;
	const gchar *path;

	path = adapter_get_path(adapter);
	if (!path)
		return;

	GSList *el = g_slist_find_custom(ggsp_adapters, adapter, ga_cmp);
	if (!el)
		return;

	ga = el->data;

	g_dbus_remove_watch(ga->conn, ga->watch);

	ggsp_adapters = g_slist_remove(ggsp_adapters, ga);

	g_dbus_unregister_interface(ga->conn, path, GGSP_INTERFACE);
	dbus_connection_unref(ga->conn);
	g_free(ga->service_path);
	g_free(ga);
}

struct btd_adapter_driver ggsp_driver = {
	.name = "gatt-ggsp",
	.probe = ggsp_server_probe,
	.remove = ggsp_server_remove,
};

int ggsp_server_init()
{
	btd_register_adapter_driver(&ggsp_driver);

	return 0;
}

void ggsp_server_exit()
{
	btd_unregister_adapter_driver(&ggsp_driver);
}
