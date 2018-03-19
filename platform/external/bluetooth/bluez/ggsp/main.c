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

#include <stdint.h>
#include <glib.h>

#include "plugin.h"
#include "hcid.h"
#include "log.h"
#include "server.h"

static int ggsp_init(void)
{
	if (!main_opts.gatt_enabled) {
		DBG("Attribute server is disabled");
		return -1;
	}

	return ggsp_server_init();
}

static void ggsp_exit(void)
{
	if (!main_opts.gatt_enabled)
		return;

	ggsp_server_exit();
}

BLUETOOTH_PLUGIN_DEFINE(ggsp, VERSION,
			BLUETOOTH_PLUGIN_PRIORITY_DEFAULT,
			ggsp_init, ggsp_exit)
