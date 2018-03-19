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

#include <gdbus.h>
#include <glib.h>
#include <time.h>
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

#define CURRENT_TIME_SVC_UUID		0x1805
#define NEXT_DST_CHANGE_SVC_UUID	0x1807

#define LOCAL_TIME_INFO_CHR_UUID	0x2A0F
#define TIME_WITH_DATE_CHR_UUID		0x2A11
#define CT_TIME_CHR_UUID		0x2A2B

#define TIME_INTERFACE			"org.bluez.Time"

/* org.bluetooth.characteristic.date_time, 0x2A08 */
struct date_time {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} __attribute__((packed));

/* org.bluetooth.characteristic.day_of_week, 0x2A09 */
struct day_of_week {
	uint8_t day;
} __attribute__((packed));

/* org.bluetooth.characteristic.day_date_time, 0x2A0A */
struct day_date_time {
	struct date_time date_time;
	struct day_of_week day_of_week;
} __attribute__((packed));

/* org.bluetooth.characteristic.exact_time_256, 0x2A0C */
struct exact_time_256 {
	struct day_date_time day_date_time;
	uint8_t fractions256;
} __attribute__((packed));

/* org.bluetooth.characteristic.current_time, 0x2A2B */
struct current_time {
	struct exact_time_256 exact_time_256;
	uint8_t adjust_reason;
} __attribute__((packed));

/* org.bluetooth.characteristic.time_zone, 0x2A0E */
struct time_zone {
	int8_t time_zone;
} __attribute__((packed));

/* org.bluetooth.characteristic.dst_offset, 0x2A0D */
enum {
	UNKNOWN_DST = 255,
};

struct dst_offset {
	uint8_t dst_offset;
} __attribute__((packed));

/* org.bluetooth.characteristic.local_time_information, 0x2A0F */
struct local_time_information {
	struct time_zone time_zone;
	struct dst_offset dst_offset;
} __attribute__((packed));

/* org.bluetooth.characteristic.time_with_dst, 0x2A11 */
struct time_with_dst {
	struct date_time date_time;
	struct dst_offset dst_offset;
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

struct time_adapter {
	struct btd_adapter *adapter;

	uint16_t h_curr_time_value;
	uint16_t h_curr_time_ccc;
};

static DBusConnection *connection;

static GSList *devices_notify;

static GSList *time_adapters;

static uint8_t adjust_reason;
static struct dst_offset dst_offset;
static struct time_with_dst next_dst;

static int encode_date_time(time_t time, struct date_time *date_time,
			    struct day_of_week *day_of_week)
{
	struct tm tm;

	if (localtime_r(&time, &tm) == NULL) {
		error("localtime_r() failed");
		/* localtime_r() does not set errno */
		return -EINVAL;
	}

	att_put_u16(1900 + tm.tm_year, &date_time->year);
	date_time->month = tm.tm_mon + 1;
	date_time->day = tm.tm_mday;
	date_time->hours = tm.tm_hour;
	date_time->minutes = tm.tm_min;
	date_time->seconds = tm.tm_sec;

	if (day_of_week)
		day_of_week->day = tm.tm_wday == 0 ? 7 : tm.tm_wday;

	return 0;
}

static int get_current_time(struct current_time *cur_time)
{
	struct timespec tp;
	struct tm tm;
	struct day_date_time *day_date_time;
	int ret;

	if (clock_gettime(CLOCK_REALTIME, &tp) == -1) {
		int err = -errno;

		error("clock_gettime: %s", strerror(-err));
		return err;
	}

	day_date_time = &cur_time->exact_time_256.day_date_time;
	ret = encode_date_time(tp.tv_sec, &day_date_time->date_time,
			       &day_date_time->day_of_week);
	if (ret)
		return ret;

	cur_time->exact_time_256.fractions256 = tp.tv_nsec * 256 / 1000000000L;
	cur_time->adjust_reason = adjust_reason;

	return 0;
}

static uint8_t current_time_read(struct attribute *a, gpointer user_data,
				 struct btd_device *device)
{
	struct current_time curtime;
	struct time_adapter *ta = user_data;

	if (get_current_time(&curtime) < 0)
		return ATT_ECODE_IO;

	attrib_db_update(ta->adapter, a->handle, NULL, (uint8_t *)&curtime,
			 sizeof(curtime), NULL);

	return 0;
}

static uint8_t local_time_info_read(struct attribute *a, gpointer user_data,
				    struct btd_device *device)
{
	struct local_time_information local_time;
	struct time_adapter *ta = user_data;

	tzset();

	local_time.dst_offset.dst_offset = dst_offset.dst_offset;

	/* Convert POSIX "timezone" (seconds West of GMT) to Time Profile
	 * format (offset from UTC in number of 15 minutes increments). */
	local_time.time_zone.time_zone = (uint8_t)(-1 * timezone / (60 * 15));

	DBG("dst_offset = %d, time_zone = %d",
	    local_time.dst_offset.dst_offset,
	    local_time.time_zone.time_zone);

	attrib_db_update(ta->adapter, a->handle, NULL, (uint8_t*)&local_time,
			 sizeof(local_time), NULL);

	return 0;
}

static uint8_t time_with_dst_read(struct attribute *a, gpointer user_data,
				    struct btd_device *device)
{
	struct time_with_dst next_time;
	struct time_adapter *ta = user_data;

	DBG("");

	attrib_db_update(ta->adapter, a->handle, NULL, (uint8_t*)&next_dst,
			 sizeof(next_dst), NULL);

	return 0;
}

static void register_services(struct time_adapter *ta)
{
	bt_uuid_t curr_time_uuid;
	bt_uuid_t next_dst_uuid;

	bt_uuid16_create(&curr_time_uuid, CURRENT_TIME_SVC_UUID);

	/* Current Time Service */
	gatt_service_add(ta->adapter, GATT_PRIM_SVC_UUID, &curr_time_uuid,
			/* CT Time characteristic */
			GATT_OPT_CHR_UUID, CT_TIME_CHR_UUID,
			GATT_OPT_CHR_PROPS,
				ATT_CHAR_PROPER_READ | ATT_CHAR_PROPER_NOTIFY,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				current_time_read, ta,
			GATT_OPT_CCC_GET_HANDLE, &ta->h_curr_time_ccc,
			GATT_OPT_CHR_VALUE_GET_HANDLE, &ta->h_curr_time_value,

			/* Local Time Information characteristic */
			GATT_OPT_CHR_UUID, LOCAL_TIME_INFO_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_READ,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				local_time_info_read, ta,

			GATT_OPT_INVALID);

	bt_uuid16_create(&curr_time_uuid, NEXT_DST_CHANGE_SVC_UUID);

	/* Next DST Change Service */
	gatt_service_add(ta->adapter, GATT_PRIM_SVC_UUID, &curr_time_uuid,
			/* Time with DST characteristic */
			GATT_OPT_CHR_UUID, TIME_WITH_DATE_CHR_UUID,
			GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_READ,
			GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
				time_with_dst_read, ta,

			GATT_OPT_INVALID);
}

static void notify_curr_time(gpointer data, gpointer user_data)
{
	struct time_adapter *ta = data;
	struct current_time *curtime = user_data;

	DBG("");

	queue_notification(ta->adapter, BDADDR_ALL,
			   ta->h_curr_time_value, ta->h_curr_time_ccc,
			   curtime, sizeof(*curtime));
}

static DBusMessage *time_updated(DBusConnection *conn, DBusMessage *msg,
				 void *data)
{
	time_t next_dst_time = 0;
	uint8_t next_dst_offset;
	struct current_time curtime;
	uint8_t reason;
	uint8_t offset;

	if (!dbus_message_get_args(msg, NULL,
				   DBUS_TYPE_BYTE, &reason,
				   DBUS_TYPE_BYTE, &offset,
				   DBUS_TYPE_UINT32, &next_dst_time,
				   DBUS_TYPE_BYTE, &next_dst_offset,
				   DBUS_TYPE_INVALID)) {
		error("Invalid arguments");
		return NULL;
	}

	if (reason & ~0xf) {
		error("Invalid adjust reason");
		return btd_error_invalid_args(msg);
	}

	adjust_reason = reason;
	dst_offset.dst_offset = offset;
	encode_date_time(next_dst_time, &next_dst.date_time, NULL);
	next_dst.dst_offset.dst_offset = next_dst_offset;

	DBG("adjust_reason = %d, dst_offset = %d, "
	    "next_dst_time = %u, next_dst_offset = %d",
	    adjust_reason, dst_offset.dst_offset,
	    (uint32_t)next_dst_time, next_dst_offset);

	if (get_current_time(&curtime) < 0) {
		error("Could not get current time");
		return btd_error_not_available(msg);
	}

	if (reason)
		g_slist_foreach(time_adapters, notify_curr_time, &curtime);

	return dbus_message_new_method_return(msg);
}

static int time_server_probe(struct btd_adapter *adapter)
{
	struct time_adapter *ta;

	ta = g_new0(struct time_adapter, 1);
	ta->adapter = adapter;

	time_adapters = g_slist_append(time_adapters, ta);

	register_services(ta);

	return 0;
}

static int ta_cmp(gconstpointer a, gconstpointer b)
{
	const struct time_adapter *ta = a;
	const struct btd_adapter *adapter = b;

	if (ta->adapter == adapter)
		return 0;

	return -1;
}

static void time_server_remove(struct btd_adapter *adapter)
{
	struct time_adapter *ta;
	GSList *el = g_slist_find_custom(time_adapters, adapter, ta_cmp);
	if (!el)
		return;

	ta = el->data;
	time_adapters = g_slist_remove(time_adapters, ta);
	g_free(ta);
}

struct btd_adapter_driver alert_server_driver = {
	.name = "gatt-alert-server",
	.probe = time_server_probe,
	.remove = time_server_remove,
};

static GDBusMethodTable time_methods[] = {
	{"TimeUpdated", "yyuy", "", time_updated},
	{}
};

int time_server_init()
{
	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	if (!g_dbus_register_interface(connection, "/", TIME_INTERFACE,
				       time_methods, NULL, NULL, NULL,
				       NULL)) {
		error("D-Bus failed to register %s interface", TIME_INTERFACE);
		dbus_connection_unref(connection);
		connection = NULL;
		return -1;
	}

	DBG("Registered interface %s on", TIME_INTERFACE);

	dst_offset.dst_offset = UNKNOWN_DST;
	next_dst.dst_offset.dst_offset = UNKNOWN_DST;

	btd_register_adapter_driver(&alert_server_driver);

	return 0;
}

void time_server_exit()
{
	btd_unregister_adapter_driver(&alert_server_driver);

	g_dbus_unregister_interface(connection, "/", TIME_INTERFACE);
	dbus_connection_unref(connection);
	connection = NULL;
}
