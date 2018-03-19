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
#include <bluetooth/uuid.h>
#include <adapter.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <dbus/dbus.h>
#include <gdbus.h>

#include "log.h"

#include "hcid.h"
#include "att.h"
#include "gattrib.h"
#include "attrib-server.h"
#include "reporter.h"
#include "gatt-service.h"
#include "device.h"
#include "attio.h"
#include "dbus-common.h"
#include "linkloss.h"
#include "immalert.h"

#define MIN_TX_POWER_LEVEL (-100)

static DBusConnection *connection;

const char *get_alert_level_string(uint8_t level)
{
	switch (level) {
	case NO_ALERT:
		return "none";
	case MILD_ALERT:
		return "mild";
	case HIGH_ALERT:
		return "high";
	}

	return "unknown";
}

static int read_tx_power_level(struct btd_adapter *adapter, bdaddr_t *addr,
			       int8_t *level)
{
	struct hci_conn_list_req *cl;
	struct hci_conn_info *ci;
	int i, dd, sk;
	int err = -1;
	int dev_id;

	dev_id = adapter_get_dev_id(adapter);

	sk = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
	if (sk < 0)
		return -1;

	if (!(cl = malloc(10 * sizeof(*ci) + sizeof(*cl)))) {
		error("Can't allocate memory");
		goto sk_free;
	}

	cl->dev_id = dev_id;
	cl->conn_num = 10;
	ci = cl->conn_info;

	if (ioctl(sk, HCIGETCONNLIST, (void *) cl)) {
		error("Can't get connection list");
		goto mem_free;
	}

	dd = hci_open_dev(dev_id);
	if (dd < 0) {
		error("cannot open dev %d", dev_id);
		goto mem_free;
	}

	for (i = 0; i < cl->conn_num; i++, ci++) {
		if (bacmp(&ci->bdaddr, addr) == 0)
			break;
	}

	if (i == cl->conn_num) {
		error("could not find connection");
		goto dev_free;
	}

	if (hci_read_transmit_power_level(dd, htobs(ci->handle), 0, level,
					  1000)) {
		error("could not read level");
		goto dev_free;
	}

	err = 0;

dev_free:
	hci_close_dev(dd);
mem_free:
	free(cl);
sk_free:
	close(sk);

	return err;
}

static uint8_t tx_power_read(struct attribute *a, gpointer user_data,
			     struct btd_device *device)
{
	struct btd_adapter *adapter = user_data;
	bdaddr_t addr;
	int8_t value;

	if (!device)
		goto done;

	device_get_address(device, &addr, NULL);

	if (read_tx_power_level(adapter, &addr, &value) != 0)
		value = MIN_TX_POWER_LEVEL;

done:
	attrib_db_update(adapter, a->handle, NULL, (uint8_t *)&value,
			 sizeof(value), NULL);
	DBG("Tx power level: %d", value);
	return 0;
}

static void register_tx_power(struct btd_adapter *adapter)
{
	gboolean svc_added;
	bt_uuid_t uuid;

	bt_uuid16_create(&uuid, TX_POWER_SVC_UUID);

	/* Tx Power Service */
	svc_added = gatt_service_add(adapter,
		GATT_PRIM_SVC_UUID, &uuid,
		/* Power level characteristic */
		GATT_OPT_CHR_UUID, POWER_LEVEL_CHR_UUID,
		GATT_OPT_CHR_PROPS, ATT_CHAR_PROPER_READ,
		GATT_OPT_CHR_VALUE_CB, ATTRIB_READ,
			tx_power_read, adapter,
		GATT_OPT_INVALID);

	if (!svc_added)
		return;

	DBG("Tx Power service added");
}

static GDBusSignalTable reporter_signals[] = {
	{ "LinkLossAlertLevel",		"ys"	},
	{ "ImmediateAlertLevel",	"ys"	},
	{ }
};

int reporter_init(struct btd_adapter *adapter)
{
	if (!main_opts.gatt_enabled) {
		DBG("GATT is disabled");
		return -ENOTSUP;
	}

	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	DBG("Proximity Reporter for adapter %p", adapter);

	register_link_loss(adapter, connection);
	register_tx_power(adapter);
	register_imm_alert(adapter, connection);

	g_dbus_register_interface(connection, adapter_get_path(adapter),
						PROXIMITY_REPORTER_INTERFACE,
						NULL, reporter_signals, NULL,
						NULL, NULL);
	return 0;
}

void reporter_exit(struct btd_adapter *adapter)
{
	g_dbus_unregister_interface(connection, adapter_get_path(adapter),
						PROXIMITY_REPORTER_INTERFACE);
	unregister_link_loss(adapter);
	unregister_imm_alert(adapter);
	dbus_connection_unref(connection);
}
