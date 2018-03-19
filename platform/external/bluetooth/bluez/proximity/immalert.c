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
#include "immalert.h"

struct imm_alert_adapter {
	struct btd_adapter *adapter;
	DBusConnection *conn;
	GSList *connected_devices;
};

struct connected_device {
	struct btd_device *device;
	struct imm_alert_adapter *adapter;
	guint callback_id;
};

static GSList *imm_alert_adapters;

static int imdevice_cmp(gconstpointer a, gconstpointer b)
{
	const struct connected_device *condev = a;
	const struct btd_device *device = b;

	if (condev->device == device)
		return 0;

	return -1;
}

static void imm_alert_emit_alert_signal(struct connected_device *condev,
					uint8_t alert_level)
{
	struct imm_alert_adapter *adapter;
	bdaddr_t addr;
	char addr_str[18];
	char *addr_str_ptr = addr_str;

	if (!condev)
		return;

	adapter = condev->adapter;

	device_get_address(condev->device, &addr, NULL);
	ba2str(&addr, addr_str);

	DBG("emit signal alert %d remote %s", alert_level, addr_str);

	g_dbus_emit_signal(adapter->conn, adapter_get_path(adapter->adapter),
			PROXIMITY_REPORTER_INTERFACE, "ImmediateAlertLevel",
			DBUS_TYPE_BYTE, &alert_level,
			DBUS_TYPE_STRING, &addr_str_ptr,
			DBUS_TYPE_INVALID);
}

static void imm_alert_purge_attio_cb(struct connected_device *condev)
{
	struct imm_alert_adapter *ia;

	if (!condev)
		return;

	ia = condev->adapter;

	if (condev->callback_id && condev->device)
		btd_device_remove_attio_callback(condev->device, condev->callback_id);

	if (condev->device)
		btd_device_unref(condev->device);

	ia->connected_devices = g_slist_remove(ia->connected_devices, condev);
	g_free(condev);
}

static void imm_alert_disc_cb(gpointer user_data)
{
	struct connected_device *condev = user_data;
	struct imm_alert_adapter *adapter;

	if (!condev)
		return;

	adapter = condev->adapter;

	DBG("immediate alert remove device %p num dev %d", condev->device,
		g_slist_length(adapter->connected_devices));

	/* change to NO_ALERT and emit if this was the last device */
	if (g_slist_length(adapter->connected_devices) == 1) {
		DBG("last device removed - emitting NO_ALERT");
		imm_alert_emit_alert_signal(condev, NO_ALERT);
	}

	imm_alert_purge_attio_cb(condev);
}

static uint8_t imm_alert_alert_lvl_write(struct attribute *a,
			gpointer user_data, struct btd_device *device)
{
	uint8_t value;
	struct imm_alert_adapter *adapter = user_data;
	struct connected_device *condev = NULL;
	GSList *l = NULL;

	if (!device)
		goto set_error;

	l = g_slist_find_custom(adapter->connected_devices, device,
				imdevice_cmp);
	if (l)
		condev = l->data;

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
	if (value != NO_ALERT && !condev) {
		condev = g_new0(struct connected_device, 1);
		condev->device = btd_device_ref(device);
		condev->adapter = adapter;
		condev->callback_id = btd_device_add_attio_callback(device,
					NULL, imm_alert_disc_cb, condev);
		adapter->connected_devices = g_slist_append(
					adapter->connected_devices, condev);
		DBG("added connected dev %p now %d devices", device,
			g_slist_length(adapter->connected_devices));
	}

	if (value != NO_ALERT)
		imm_alert_emit_alert_signal(condev, value);

	/* only emit NO_ALERT if it's the final device changing the value */
	if (value == NO_ALERT && condev)
		imm_alert_disc_cb(condev);

	DBG("alert level set to %d by device %p", value, device);
	return 0;

set_error:
	DBG("error");
	/* remove alerts by erroneous devices */
	imm_alert_disc_cb(condev);
	return ATT_ECODE_IO;
}

static int imadapter_cmp(gconstpointer a, gconstpointer b)
{
	const struct imm_alert_adapter *imadapter = a;
	const struct btd_adapter *adapter = b;

	if (imadapter->adapter == adapter)
		return 0;

	return -1;
}

void register_imm_alert(struct btd_adapter *adapter, DBusConnection *conn)
{
	gboolean svc_added;
	bt_uuid_t uuid;
	struct imm_alert_adapter *imadapter;

	bt_uuid16_create(&uuid, IMMEDIATE_ALERT_SVC_UUID);

	imadapter = g_new0(struct imm_alert_adapter, 1);
	imadapter->adapter = adapter;
	imadapter->conn = dbus_connection_ref(conn);

	imm_alert_adapters = g_slist_append(imm_alert_adapters, imadapter);

	/* Immediate Alert Service */
	svc_added = gatt_service_add(adapter,
		GATT_PRIM_SVC_UUID, &uuid,
		/* Alert level characteristic */
		GATT_OPT_CHR_UUID, ALERT_LEVEL_CHR_UUID,
		GATT_OPT_CHR_PROPS,
			ATT_CHAR_PROPER_WRITE_WITHOUT_RESP,
		GATT_OPT_CHR_VALUE_CB, ATTRIB_WRITE,
			imm_alert_alert_lvl_write, imadapter,
		GATT_OPT_INVALID);

	if (!svc_added) {
		unregister_imm_alert(adapter);
		return;
	}

	DBG("Immediate Alert service added");
}

static void purge_all_attio_cb(gpointer data, gpointer user_data)
{
	struct connected_device *condev = data;

	imm_alert_purge_attio_cb(condev);
}

void unregister_imm_alert(struct btd_adapter *adapter)
{
	struct imm_alert_adapter *imadapter;
	GSList *l = g_slist_find_custom(imm_alert_adapters, adapter,
								imadapter_cmp);
	if (!l)
		return;

	imadapter = l->data;

	g_slist_foreach(imadapter->connected_devices, purge_all_attio_cb,
			NULL);
	dbus_connection_unref(imadapter->conn);

	imm_alert_adapters = g_slist_remove(imm_alert_adapters, imadapter);
	g_free(imadapter);
}


