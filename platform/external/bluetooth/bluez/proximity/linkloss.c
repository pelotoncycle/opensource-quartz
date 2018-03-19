/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2012  Texas Instruments Corporation
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
#include <bluetooth/uuid.h>
#include <adapter.h>

#include <dbus/dbus.h>
#include <gdbus.h>

#include "log.h"
#include "att.h"
#include "gattrib.h"
#include "gatt-service.h"
#include "attrib-server.h"
#include "device.h"
#include "attio.h"
#include "dbus-common.h"
#include "reporter.h"
#include "linkloss.h"

#define BLUEZ_SERVICE "org.bluez"

struct link_loss_adapter {
	struct btd_adapter *adapter;
	uint16_t alert_lvl_value_handle;
	DBusConnection *conn;
	GSList *connected_devices;
	guint watch;
};

struct connected_device {
	struct btd_device *device;
	struct link_loss_adapter *adapter;
	uint8_t alert_level;
	guint callback_id;
};

static GSList *link_loss_adapters;

static int lldevice_cmp(gconstpointer a, gconstpointer b)
{
	const struct connected_device *llcondev = a;
	const struct btd_device *device = b;

	if (llcondev->device == device)
		return 0;

	return -1;
}

static struct connected_device *
find_connected_device(struct link_loss_adapter *la, struct btd_device *device)
{
	GSList *l = g_slist_find_custom(la->connected_devices, device,
					lldevice_cmp);
	if (!l)
		return NULL;

	return l->data;
}

static void link_loss_emit_alert_signal(struct connected_device *condev)
{
	bdaddr_t addr;
	char addr_str[18];
	char *addr_str_ptr = addr_str;
	struct link_loss_adapter *adapter = condev->adapter;

	if (!condev->device)
		return;

	device_get_address(condev->device, &addr, NULL);
	ba2str(&addr, addr_str);

	DBG("emit signal alert %d remote %s", condev->alert_level,
								addr_str);

	g_dbus_emit_signal(adapter->conn, adapter_get_path(adapter->adapter),
			PROXIMITY_REPORTER_INTERFACE, "LinkLossAlertLevel",
			DBUS_TYPE_BYTE, &condev->alert_level,
			DBUS_TYPE_STRING, &addr_str_ptr,
			DBUS_TYPE_INVALID);
}

/* condev might be NULL here */
static void link_loss_change_alert_level(struct link_loss_adapter *la,
					 struct connected_device *condev,
					 uint8_t level)
{
	attrib_db_update(la->adapter, la->alert_lvl_value_handle,
			 NULL, &level, sizeof(level), NULL);
	if (condev)
		condev->alert_level = level;
}

static uint8_t link_loss_alert_lvl_read(struct attribute *a, gpointer user_data,
					struct btd_device *device)
{
	struct link_loss_adapter *adapter = user_data;
	struct connected_device *condev;
	GSList *l = NULL;
	uint8_t alert_level = NO_ALERT;

	if (device)
		l = g_slist_find_custom(adapter->connected_devices, device,
					lldevice_cmp);
	if (l) {
		condev = l->data;
		alert_level = condev->alert_level;
	}

	DBG("return alert level %d for dev %p", alert_level, device);

	/* update the alert level according to the requesting device */
	attrib_db_update(adapter->adapter, a->handle, NULL, &alert_level,
			 sizeof(alert_level), NULL);

	return 0;
}

static void link_loss_purge_attio_cb(struct connected_device *condev)
{
	struct link_loss_adapter *la;

	if (!condev)
		return;

	la = condev->adapter;

	if (condev->callback_id && condev->device)
		btd_device_remove_attio_callback(condev->device, condev->callback_id);

	if (condev->device)
		btd_device_unref(condev->device);

	la->connected_devices = g_slist_remove(la->connected_devices, condev);
	g_free(condev);
}

static void link_loss_disc_cb(gpointer user_data)
{
	struct connected_device *condev = user_data;

	DBG("alert loss disconnect device %p", condev->device);

	/* if an alert-level is set, emit a signal */
	if (condev->alert_level != NO_ALERT)
		link_loss_emit_alert_signal(condev);

	/* change to NO_ALERT */
	link_loss_change_alert_level(condev->adapter, condev, NO_ALERT);

	/* we are open for more changes now */
	link_loss_purge_attio_cb(condev);
}

static uint8_t link_loss_alert_lvl_write(struct attribute *a,
		gpointer user_data, struct btd_device *device)
{
	uint8_t value;
	struct link_loss_adapter *adapter = user_data;
	GSList *l = NULL;
	struct connected_device *condev = NULL;

	if (device) {
		l = g_slist_find_custom(adapter->connected_devices, device,
					lldevice_cmp);
		if (l)
			condev = l->data;
	}

	if (a->len == 0) {
		DBG("Illegal alert level len");
		goto set_error;
	}

	value = a->data[0];
	if (value != NO_ALERT && value != MILD_ALERT && value != HIGH_ALERT) {
		DBG("Illegal alert value");
		goto set_error;
	}

	/* Register a disconnect cb if the alert level is non-zero */
	if (value != NO_ALERT && device && !condev) {
		condev = g_new0(struct connected_device, 1);
		condev->device = btd_device_ref(device);
		condev->adapter = adapter;
		condev->callback_id = btd_device_add_attio_callback(device,
					NULL, link_loss_disc_cb, condev);
		adapter->connected_devices = g_slist_append(
					adapter->connected_devices, condev);
	} else if (value == NO_ALERT && condev) {
		link_loss_purge_attio_cb(condev);
		condev = NULL;
	}

	DBG("alert level set to %d by device %p", value, device);

	/* note condev might be NULL if we stay with NO_ALERT */
	link_loss_change_alert_level(adapter, condev, value);
	return 0;

set_error:
	link_loss_change_alert_level(adapter, condev, NO_ALERT);
	link_loss_purge_attio_cb(condev);
	return ATT_ECODE_IO;
}

static int lladapter_cmp(gconstpointer a, gconstpointer b)
{
	const struct link_loss_adapter *lladapter = a;
	const struct btd_adapter *adapter = b;

	if (lladapter->adapter == adapter)
		return 0;

	return -1;
}

static gboolean handle_local_disconnect(DBusConnection *conn,
					DBusMessage *msg, void *data)
{
	struct link_loss_adapter *la = data;
	struct connected_device *condev;
	struct btd_device *device;
	const char *path;

	path = dbus_message_get_path(msg);

	device = adapter_get_device_by_path(la->adapter, path);
	if (!device)
		return TRUE;

	condev = find_connected_device(la, device);
	if (!condev)
		return TRUE;

	/* no need to alert on this device - we requested disconnection */
	link_loss_purge_attio_cb(condev);

	DBG("alert level zeroed for locally disconnecting dev %p", device);
	return TRUE;
}

void register_link_loss(struct btd_adapter *adapter, DBusConnection *conn)
{
	gboolean svc_added;
	uint8_t initial_value = NO_ALERT;
	bt_uuid_t uuid;
	struct link_loss_adapter *lladapter;

	bt_uuid16_create(&uuid, LINK_LOSS_SVC_UUID);

	lladapter = g_new0(struct link_loss_adapter, 1);
	lladapter->adapter = adapter;
	lladapter->conn = dbus_connection_ref(conn);

	link_loss_adapters = g_slist_append(link_loss_adapters, lladapter);

	/* Link Loss Service */
	svc_added = gatt_service_add(adapter,
		GATT_PRIM_SVC_UUID, &uuid,
		/* Alert level characteristic */
		GATT_OPT_CHR_UUID, ALERT_LEVEL_CHR_UUID,
		GATT_OPT_CHR_PROPS,
			ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_WRITE,
		GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
			link_loss_alert_lvl_read, lladapter,
		GATT_OPT_CHR_VALUE_CB, ATTRIB_WRITE,
			link_loss_alert_lvl_write, lladapter,
		GATT_OPT_CHR_VALUE_GET_HANDLE,
			&lladapter->alert_lvl_value_handle,
		GATT_OPT_INVALID);

	if (!svc_added) {
		unregister_link_loss(adapter);
		return;
	}

	lladapter->watch = g_dbus_add_signal_watch(lladapter->conn,
				BLUEZ_SERVICE, NULL, DEVICE_INTERFACE,
				"DisconnectRequested", handle_local_disconnect,
				lladapter, NULL);

	/* set initial value */
	link_loss_change_alert_level(lladapter, NULL, NO_ALERT);
	DBG("Link Loss service added");
}

static void purge_all_attio_cb(gpointer data, gpointer user_data)
{
	struct connected_device *condev = data;

	link_loss_purge_attio_cb(condev);
}

void unregister_link_loss(struct btd_adapter *adapter)
{
	struct link_loss_adapter *lladapter;
	GSList *l = g_slist_find_custom(link_loss_adapters, adapter,
								lladapter_cmp);
	if (!l)
		return;

	lladapter = l->data;

	g_dbus_remove_watch(lladapter->conn, lladapter->watch);

	g_slist_foreach(lladapter->connected_devices, purge_all_attio_cb,
			NULL);
	dbus_connection_unref(lladapter->conn);

	link_loss_adapters = g_slist_remove(link_loss_adapters, lladapter);
	g_free(lladapter);
}
