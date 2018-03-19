/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2010 GSyC/LibreSoft, Universidad Rey Juan Carlos.
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

#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>

#include <btio.h>
#include <adapter.h>
#include <device.h>

#include "mcap_types.h"

#include "log.h"
#include "mcap_manager.h"
#include "../mcap.h"
#include "mcap_app.h"

#include "glib-helper.h"

#define DBG(fmt, arg...)  printf("AMIZ: %s: " fmt "\n" , __FUNCTION__ , ## arg)
#define BASE_BT_UUID  "00000000-0000-1000-8000-00805F9B34FB" 

static DBusConnection *connection = NULL;

static int mcap_adapter_probe(struct btd_adapter *adapter)
{
	DBG("");

	return mcap_adapter_register(connection, adapter);
}

static void mcap_adapter_remove(struct btd_adapter *adapter)
{
	mcap_adapter_unregister(adapter);
}

static struct btd_adapter_driver mcap_adapter_driver = {
	.name	= "mcap-adapter-driver",
	.probe	= mcap_adapter_probe,
	.remove	= mcap_adapter_remove,
};

static int mcap_driver_probe(struct btd_device *device, GSList *uuids)
{
	DBG("");
	return mcap_device_register(connection, device);
}

static void mcap_driver_remove(struct btd_device *device)
{
	mcap_device_unregister(device);
}

static struct btd_device_driver mcap_device_driver = {
	.name	= "mcap-device-driver",
	.uuids	= BTD_UUIDS(MCAP_UUID, MCAP_SOURCE_UUID, MCAP_SINK_UUID),

	.probe	= mcap_driver_probe,
	.remove	= mcap_driver_remove
};

int mcap_manager_init(DBusConnection *conn)
{
	DBG("mcap_manager_init");
	if (mcap_manager_start(conn))
		return -1;

	connection = dbus_connection_ref(conn);
	btd_register_adapter_driver(&mcap_adapter_driver);
	btd_register_device_driver(&mcap_device_driver);

	return 0;
}

void mcap_manager_exit(void)
{
	btd_unregister_device_driver(&mcap_device_driver);
	btd_unregister_adapter_driver(&mcap_adapter_driver);
	mcap_manager_stop();

	dbus_connection_unref(connection);
	connection = NULL;
}
