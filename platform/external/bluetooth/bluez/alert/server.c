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

#define PHONE_ALERT_STATUS_SVC_UUID	0x180E
#define ALERT_NOTIF_SVC_UUID		0x1811

#define ALERT_STATUS_CHR_UUID		0x2A3F
#define RINGER_CP_CHR_UUID		0x2A40
#define RINGER_SETTING_CHR_UUID		0x2A41

#define ALERT_NOTIF_CP_CHR_UUID		0x2A44
#define UNREAD_ALERT_CHR_UUID		0x2A45
#define NEW_ALERT_CHR_UUID		0x2A46
#define SUPP_NEW_ALERT_CAT_CHR_UUID	0x2A47
#define SUPP_UNREAD_ALERT_CAT_CHR_UUID	0x2A48

#define ALERT_INTERFACE			"org.bluez.Alert"

enum {
	ALERT_RINGER_STATE	= 1 << 0,
	ALERT_VIBRATOR_STATE	= 1 << 1,
	ALERT_DISPLAY_STATE	= 1 << 2,
};

enum {
	SET_SILENT_MODE = 1,
	MUTE_ONCE,
	CANCEL_SILENT_MODE,
};

enum {
	RINGER_SILENT = 0,
	RINGER_NORMAL = 1,
};

enum {
	ENABLE_NEW_INCOMING,
	ENABLE_UNREAD_CAT,
	DISABLE_NEW_INCOMING,
	DISABLE_UNREAD_CAT,
	NOTIFY_NEW_INCOMING,
	NOTIFY_UNREAD_CAT,
};

enum {
	ALERT_CAT_SIMPLE_ALERT,
	ALERT_CAT_EMAIL,
	ALERT_CAT_NEWS,
	ALERT_CAT_CALL,
	ALERT_CAT_MISSED_CALL,
	ALERT_CAT_SMS_MMS,
	ALERT_CAT_VOICE_MAIL,
	ALERT_CAT_SCHEDULE,
	NUM_OF_ALERT_CATEGORIES
};

#define TEXT_SIZE 18
struct new_alert {
	uint8_t category_id;
	uint8_t num_new_alerts;
	uint8_t text[TEXT_SIZE];
} __attribute__((packed));

struct unread_alert {
	uint8_t category_id;
	uint8_t unread_count;
} __attribute__((packed));

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

struct alert_adapter {
	struct btd_adapter *adapter;

	uint8_t en_new_alerts;
	uint8_t en_unread_alerts;

	uint16_t h_new_alerts;
	uint16_t h_new_alerts_ccc;
	uint16_t h_unread_alerts;
	uint16_t h_unread_alerts_ccc;

	uint16_t h_ringer_setting;
	uint16_t h_ringer_setting_ccc;
	uint16_t h_alert_status;
	uint16_t h_alert_status_ccc;

	/* Disconnection callback for remote device */
	struct btd_device *device;
	guint id;
};

static bool initialized;

static uint8_t ringer_setting;
static uint8_t alert_status;

uint8_t supp_new_alerts;
uint8_t supp_unread_alerts;
static struct new_alert new_alerts[NUM_OF_ALERT_CATEGORIES];
static struct unread_alert unread_alerts[NUM_OF_ALERT_CATEGORIES];

static DBusConnection *connection;

static GSList *alert_adapters;

static GSList *devices_notify;

static bool override_notifications;

static void set_silent_mode(struct alert_adapter *aa)
{
	DBG("Not supported");
}

static void cancel_silent_mode(struct alert_adapter *aa)
{
	DBG("Not supported");
}

static void mute_once(struct alert_adapter *aa)
{
	DBG("Emitting MuteOnce signal");

	g_dbus_emit_signal(connection, "/",
			   ALERT_INTERFACE, "MuteOnce",
			   DBUS_TYPE_INVALID);
}

static uint8_t ringer_cp_write(struct attribute *a, gpointer user_data,
			       struct btd_device *device)
{
	struct alert_adapter *aa = user_data;

	if (!initialized) {
		error("Uninitialized");
		return ATT_ECODE_IO;
	}

	DBG("Command = %d", a->data[0]);

	switch (a->data[0]) {
	case SET_SILENT_MODE:
		set_silent_mode(aa);
		break;
	case MUTE_ONCE:
		mute_once(aa);
		break;
	case CANCEL_SILENT_MODE:
		cancel_silent_mode(aa);
		break;
	default:
		DBG("Unknown mode");
	}

	return 0;
}

static uint8_t alert_status_read(struct attribute *a, gpointer user_data,
				 struct btd_device *device)
{
	struct alert_adapter *aa = user_data;

	if (!initialized) {
		error("Uninitialized");
		return ATT_ECODE_IO;
	}

	DBG("Alert state = 0x%X", alert_status);

	if (a->data == NULL || a->data[0] != alert_status)
		attrib_db_update(aa->adapter, a->handle, NULL, &alert_status,
				 sizeof(alert_status), NULL);

	return 0;
}

static uint8_t ringer_setting_read(struct attribute *a, gpointer user_data,
				   struct btd_device *device)
{
	struct alert_adapter *aa = user_data;

	if (!initialized) {
		error("Uninitialized");
		return ATT_ECODE_IO;
	}

	DBG("Ringer state = 0x%X", ringer_setting);

	if (a->data == NULL || a->data[0] != ringer_setting)
		attrib_db_update(aa->adapter, a->handle, NULL, &ringer_setting,
				 sizeof(ringer_setting), NULL);

	return 0;
}

static uint8_t supp_new_alert_cat_read(struct attribute *a, gpointer user_data,
				       struct btd_device *device)
{
	struct alert_adapter *aa = user_data;

	if (!initialized) {
		error("Uninitialized");
		return ATT_ECODE_IO;
	}

	DBG("Supported new alert = 0x%X", supp_new_alerts);

	if (a->data == NULL || a->data[0] != supp_new_alerts)
		attrib_db_update(aa->adapter, a->handle, NULL, &supp_new_alerts,
				 sizeof(supp_new_alerts), NULL);

	return 0;
}

static uint8_t supp_unread_alert_cat_read(struct attribute *a,
					  gpointer user_data,
					  struct btd_device *device)
{
	struct alert_adapter *aa = user_data;

	if (!initialized) {
		error("Uninitialized");
		return ATT_ECODE_IO;
	}

	DBG("Supported unread alert = 0x%X", supp_unread_alerts);

	if (a->data == NULL || a->data[0] != supp_unread_alerts)
		attrib_db_update(aa->adapter, a->handle, NULL,
				 &supp_unread_alerts,
				 sizeof(supp_unread_alerts), NULL);

	return 0;
}

static void devstate_cleanup_cb(gpointer user_data)
{
	struct alert_adapter *aa = user_data;

	DBG("Device disconnected, reinitializing state");

	aa->en_new_alerts = 0;
	aa->en_unread_alerts = 0;

	btd_device_remove_attio_callback(aa->device, aa->id);
	btd_device_unref(aa->device);
	aa->device = NULL;
	aa->id = 0;
}

static void register_devstate_cleanup_cb(struct alert_adapter *aa,
					 struct btd_device *device)
{
	if (!aa->device) {
		aa->device = btd_device_ref(device);
		aa->id = btd_device_add_attio_callback(aa->device, NULL,
						       devstate_cleanup_cb, aa);
	}
}

static uint8_t alert_notif_cp_write(struct attribute *a, gpointer user_data,
				    struct btd_device *device)
{
	struct alert_adapter *aa = user_data;
	bdaddr_t addr;
	int i;

	if (!initialized) {
		error("Uninitialized");
		return ATT_ECODE_IO;
	}

	device_get_address(device, &addr, NULL);

	DBG("Command = %d; Data = 0x%X", a->data[0], a->data[1]);

	switch (a->data[0]) {
	case ENABLE_NEW_INCOMING:
		aa->en_new_alerts |= a->data[1];
		register_devstate_cleanup_cb(aa, device);
		break;
	case ENABLE_UNREAD_CAT:
		aa->en_unread_alerts |= a->data[1];
		register_devstate_cleanup_cb(aa, device);
		break;
	case DISABLE_NEW_INCOMING:
		aa->en_new_alerts &= ~a->data[1];
		break;
	case DISABLE_UNREAD_CAT:
		aa->en_unread_alerts &= ~a->data[1];
		break;
	case NOTIFY_NEW_INCOMING:
		for (i = 0; i < NUM_OF_ALERT_CATEGORIES; i++) {
			if ((1 << i) & aa->en_new_alerts &&
			    new_alerts[i].num_new_alerts)
				queue_notification(aa->adapter,
						   &addr,
						   aa->h_new_alerts,
						   aa->h_new_alerts_ccc,
						   &new_alerts[i],
						   sizeof(new_alerts[0]));
		}
		break;
	case NOTIFY_UNREAD_CAT:
		for (i = 0; i < NUM_OF_ALERT_CATEGORIES; i++) {
			if ((1 << i) & aa->en_unread_alerts &&
			    unread_alerts[i].unread_count)
				queue_notification(aa->adapter,
						   &addr,
						   aa->h_unread_alerts,
						   aa->h_unread_alerts_ccc,
						   &unread_alerts[i],
						   sizeof(unread_alerts[0]));
		}
		break;
	default:
		DBG("Unknown command");
	}

	DBG("Enabled new alert = 0x%X", aa->en_new_alerts);
	DBG("Enabled unread alert = 0x%X", aa->en_unread_alerts);

	return 0;
}

static DBusMessage *setup_alert(DBusConnection *conn, DBusMessage *msg,
				void *data)
{
	if (initialized) {
		error("Already set up");
		return btd_error_busy(msg);
	}
	initialized = true;

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_BYTE, &alert_status,
				   DBUS_TYPE_BYTE, &ringer_setting,
				   DBUS_TYPE_BYTE, &supp_new_alerts,
				   DBUS_TYPE_BYTE, &supp_unread_alerts,
				   DBUS_TYPE_INVALID))
		return NULL;

	DBG("Alert status = 0x%X", alert_status);
	DBG("Ringer setting = %d", ringer_setting);
	DBG("Supported new alerts = 0x%X", supp_new_alerts);
	DBG("Supported unread alerts = 0x%X", supp_unread_alerts);

	return dbus_message_new_method_return(msg);
}

static void __notify_alert_status(gpointer data, gpointer user_data)
{
	struct alert_adapter *aa = data;
	uint8_t status = (uint32_t)user_data & 0xff;

	if (alert_status != status) {
		alert_status = status;
		queue_notification(aa->adapter, BDADDR_ALL,
				   aa->h_alert_status, aa->h_alert_status_ccc,
				   &alert_status, sizeof(alert_status));
		attrib_db_update(aa->adapter, aa->h_alert_status, NULL,
				 &alert_status, sizeof(alert_status), NULL);
	}
}

static DBusMessage *notify_alert_status(DBusConnection *conn, DBusMessage *msg,
					void *data)
{
	uint32_t status = 0;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_BYTE, &status,
				   DBUS_TYPE_INVALID)) {
		error("Invalid arguments");
		return NULL;
	}

	DBG("Alert status: 0x%X -> 0x%X", alert_status, status);

	g_slist_foreach(alert_adapters, __notify_alert_status,
			(gpointer)status);

	return dbus_message_new_method_return(msg);
}

static void __notify_ringer_setting(gpointer data, gpointer user_data)
{
	struct alert_adapter *aa = data;
	uint8_t setting = (uint32_t)user_data & 0xff;

	if (ringer_setting != setting) {
		ringer_setting = setting;
		queue_notification(aa->adapter, BDADDR_ALL,
				   aa->h_ringer_setting,
				   aa->h_ringer_setting_ccc,
				   &ringer_setting, sizeof(ringer_setting));
		attrib_db_update(aa->adapter, aa->h_ringer_setting, NULL,
				 &ringer_setting, sizeof(ringer_setting), NULL);
	}
}

static DBusMessage *notify_ringer_setting(DBusConnection *conn,
					  DBusMessage *msg,
					  void *data)
{
	uint32_t setting = 0;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_BYTE, &setting,
				   DBUS_TYPE_INVALID)) {
		error("Invalid arguments");
		return NULL;
	}

	DBG("Ringer setting: 0x%X -> 0x%X", ringer_setting, setting);

	g_slist_foreach(alert_adapters, __notify_ringer_setting,
			(gpointer)setting);

	return dbus_message_new_method_return(msg);
}

static void __notify_new_alert(gpointer data, gpointer user_data)
{
	struct alert_adapter *aa = data;
	uint8_t category_id = (uint32_t)user_data & 0xff;
	uint8_t category_id_bit = 1 << category_id;

	if ((aa->en_new_alerts & category_id_bit) | override_notifications)
		queue_notification(aa->adapter, BDADDR_ALL, aa->h_new_alerts,
				   aa->h_new_alerts_ccc,
				   &new_alerts[category_id],
				   sizeof(new_alerts[0]));
}

static DBusMessage *notify_new_alert(DBusConnection *conn, DBusMessage *msg,
				     void *data)
{
	uint32_t category_id = 0;
	uint8_t category_id_bit;
	uint8_t num_new_alerts;
	uint8_t *text;
	uint32_t cnt;

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_BYTE, &category_id,
				   DBUS_TYPE_BYTE, &num_new_alerts,
				   DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE, &text, &cnt,
				   DBUS_TYPE_INVALID)) {
		error("Invalid arguments");
		return NULL;
	}

	if (cnt > TEXT_SIZE) {
		error("Invalid text argument size: %d", cnt);
		return btd_error_invalid_args(msg);
	}

	category_id_bit = 1 << category_id;
	if (!(supp_new_alerts & category_id_bit)) {
		error("Unsupported catergory ID 0x%X", category_id);
		return btd_error_invalid_args(msg);
	}

	new_alerts[category_id].category_id = category_id;
	new_alerts[category_id].num_new_alerts = num_new_alerts;
	memcpy(new_alerts[category_id].text, text, cnt);
	memset(&new_alerts[category_id].text[cnt], ' ', TEXT_SIZE - cnt);

	DBG("Updated category %d", category_id);

	g_slist_foreach(alert_adapters, __notify_new_alert,
			(gpointer)category_id);

	return dbus_message_new_method_return(msg);
}

static void __notify_unread_alert(gpointer data, gpointer user_data)
{
	struct alert_adapter *aa = data;
	uint8_t category_id = (uint32_t)user_data & 0xff;
	uint8_t category_id_bit = 1 << category_id;

	if ((aa->en_unread_alerts & category_id_bit) | override_notifications)
		queue_notification(aa->adapter, BDADDR_ALL, aa->h_unread_alerts,
				   aa->h_unread_alerts_ccc,
				   &unread_alerts[category_id],
				   sizeof(unread_alerts[0]));
}

static DBusMessage *notify_unread_alert(DBusConnection *conn, DBusMessage *msg,
					void *data)
{
	uint32_t category_id = 0;
	uint8_t category_id_bit;
	uint8_t unread_count;

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_BYTE, &category_id,
				   DBUS_TYPE_BYTE, &unread_count,
				   DBUS_TYPE_INVALID)) {
		error("Invalid arguments");
		return NULL;
	}

	category_id_bit = 1 << category_id;
	if (!(supp_unread_alerts & category_id_bit)) {
		error("Unsupported catergory ID");
		return btd_error_invalid_args(msg);
	}

	unread_alerts[category_id].category_id = category_id;
	unread_alerts[category_id].unread_count = unread_count;

	DBG("Updated category %d", category_id);

	g_slist_foreach(alert_adapters, __notify_new_alert,
			(gpointer)category_id);

	return dbus_message_new_method_return(msg);
}

static DBusMessage *cert_enable_all_notifications(DBusConnection *conn,
						  DBusMessage *msg, void *data)
{
	DBG("Forcing all notifications for certification");

	override_notifications = true;

	return dbus_message_new_method_return(msg);
}

static DBusMessage *cert_disable_all_notifications(DBusConnection *conn,
						   DBusMessage *msg, void *data)
{
	DBG("Initializing all notifications");

	override_notifications = false;

	return dbus_message_new_method_return(msg);
}

static void register_phone_alert_service(struct alert_adapter *aa)
{
	bt_uuid_t uuid;

	bt_uuid16_create(&uuid, PHONE_ALERT_STATUS_SVC_UUID);

	/* Phone Alert Status Service */
	gatt_service_add(aa->adapter, GATT_PRIM_SVC_UUID, &uuid,
			/* Alert Status characteristic */
			GATT_OPT_CHR_UUID, ALERT_STATUS_CHR_UUID,
			GATT_OPT_CHR_PROPS,
				ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_NOTIFY,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				alert_status_read, aa,
			GATT_OPT_CCC_GET_HANDLE, &aa->h_alert_status_ccc,
			GATT_OPT_CHR_VALUE_GET_HANDLE, &aa->h_alert_status,

			/* Ringer Setting characteristic */
			GATT_OPT_CHR_UUID, RINGER_SETTING_CHR_UUID,
			GATT_OPT_CHR_PROPS,
				ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_NOTIFY,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				ringer_setting_read, aa,
			GATT_OPT_CCC_GET_HANDLE, &aa->h_ringer_setting_ccc,
			GATT_OPT_CHR_VALUE_GET_HANDLE, &aa->h_ringer_setting,

			/* Ringer Control Point characteristic */
			GATT_OPT_CHR_UUID, RINGER_CP_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_WRITE_WITHOUT_RESP,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_WRITE,
				ringer_cp_write, aa,

			/* End of service */
			GATT_OPT_INVALID);
}

static void register_alert_notif_service(struct alert_adapter *aa)
{
	bt_uuid_t uuid;

	bt_uuid16_create(&uuid, ALERT_NOTIF_SVC_UUID);

	/* Alert Notification Service */
	gatt_service_add(aa->adapter, GATT_PRIM_SVC_UUID, &uuid,
			/* Supported New Alert Category */
			GATT_OPT_CHR_UUID, SUPP_NEW_ALERT_CAT_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_READ,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				supp_new_alert_cat_read, aa,

			/* New Alert */
			GATT_OPT_CHR_UUID, NEW_ALERT_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_NOTIFY,
			GATT_OPT_CCC_GET_HANDLE, &aa->h_new_alerts_ccc,
			GATT_OPT_CHR_VALUE_GET_HANDLE, &aa->h_new_alerts,

			/* Supported Unread Alert Category */
			GATT_OPT_CHR_UUID, SUPP_UNREAD_ALERT_CAT_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_READ,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				supp_unread_alert_cat_read, aa,

			/* Unread Alert Status */
			GATT_OPT_CHR_UUID, UNREAD_ALERT_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_NOTIFY,
			GATT_OPT_CCC_GET_HANDLE, &aa->h_unread_alerts_ccc,
			GATT_OPT_CHR_VALUE_GET_HANDLE, &aa->h_unread_alerts,

			/* Alert Notification Control Point */
			GATT_OPT_CHR_UUID, ALERT_NOTIF_CP_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_WRITE,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_WRITE,
				alert_notif_cp_write, aa,

			GATT_OPT_INVALID);
}

static int alert_server_probe(struct btd_adapter *adapter)
{
	struct alert_adapter *aa;

	aa = g_new0(struct alert_adapter, 1);
	aa->adapter = adapter;
	alert_adapters = g_slist_append(alert_adapters, aa);

	/* TODO: Handle failure cases of the following: */
	register_phone_alert_service(aa);
	register_alert_notif_service(aa);

	return 0;
}

static int aa_cmp(gconstpointer a, gconstpointer b)
{
	const struct alert_adapter *aa = a;
	const struct btd_adapter *adapter = b;

	if (aa->adapter == adapter)
		return 0;

	return -1;
}

static void alert_server_remove(struct btd_adapter *adapter)
{
	struct alert_adapter *aa;
	GSList *el = g_slist_find_custom(alert_adapters, adapter, aa_cmp);
	if (!el)
		return;

	aa = el->data;
	alert_adapters = g_slist_remove(alert_adapters, aa);
	g_free(aa);
}

struct btd_adapter_driver alert_server_driver = {
	.name = "gatt-alert-server",
	.probe = alert_server_probe,
	.remove = alert_server_remove,
};

static GDBusMethodTable alert_methods[] = {
	/* PASS + ANS */
	{"Setup", "yyyy", "", setup_alert},

	/* PASS */
	{"NotifyAlertStatus", "y", "", notify_alert_status},
	{"NotifyRingerSetting", "y", "", notify_ringer_setting},

	/* ANS */
	{"NotifyNewAlert", "yyay", "", notify_new_alert},
	{"NotifyUnreadAlert", "yy", "", notify_unread_alert},

	/* ANS certification only */
	{"CertEnableAllNotifications", "", "", cert_enable_all_notifications},
	{"CertDisableAllNotifications", "", "", cert_disable_all_notifications},
	{}
};

static GDBusSignalTable alert_signals[] = {
	/* PASS */
	{"MuteOnce", ""},
	{}
};

int alert_server_init(void)
{
	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	if (!g_dbus_register_interface(connection, "/", ALERT_INTERFACE,
				       alert_methods, alert_signals, NULL, NULL,
				       NULL)) {
		error("D-Bus failed to register %s interface", ALERT_INTERFACE);
		dbus_connection_unref(connection);
		connection = NULL;
		return -1;
	}

	DBG("Registered interface %s", ALERT_INTERFACE);

	btd_register_adapter_driver(&alert_server_driver);

	return 0;
}

void alert_server_exit(void)
{
	btd_unregister_adapter_driver(&alert_server_driver);
	g_dbus_unregister_interface(connection, "/", ALERT_INTERFACE);
	dbus_connection_unref(connection);
	connection = NULL;
}
