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
#include <stdio.h>
#include <errno.h>

#include <gdbus.h>

#include "plugin.h"
#include "log.h"
#include "mcap_manager.h"
#define DBG(fmt, arg...)  printf("AMIZ: %s: " fmt "\n" , __FUNCTION__ , ## arg)

static DBusConnection *connection = NULL;

int mcap_init(void)
{
	DBG("mcap_init");

	connection = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
	if (connection == NULL)
		return -EIO;

	if (mcap_manager_init(connection) < 0) {
		dbus_connection_unref(connection);
		return -EIO;
	}

	return 0;
}

void mcap_exit(void)
{
	mcap_manager_exit();

	dbus_connection_unref(connection);
	connection = NULL;
}

BLUETOOTH_PLUGIN_DEFINE(mcaptest, VERSION,
                        BLUETOOTH_PLUGIN_PRIORITY_DEFAULT, mcap_init, mcap_exit)

