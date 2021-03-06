/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2000-2001  Qualcomm Incorporated
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2002-2010  Marcel Holtmann <marcel@holtmann.org>
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

struct main_opts {
	char		host_name[40];
	unsigned long	flags;
	char		*name;
	uint32_t	class;
	uint16_t	pageto;
	uint16_t	autoto;
	uint32_t	discovto;
	uint32_t	pairto;
	uint16_t	link_mode;
	uint16_t	link_policy;
	gboolean	remember_powered;
	gboolean	reverse_sdp;
	gboolean	name_resolv;
	gboolean	debug_keys;
	gboolean	gatt_enabled;
	gboolean	wide_band_speech;

	uint8_t		mode;
	uint8_t		discov_interval;
	char		deviceid[15]; /* FIXME: */
	uint32_t	le_reconnect_timeout;
	uint16_t	le_bk_scan_interval;
	uint16_t	le_bk_scan_window;
};

enum {
	HCID_SET_PAGETO,
};

extern struct main_opts main_opts;

void btd_start_exit_timer(void);
void btd_stop_exit_timer(void);

gboolean plugin_init(GKeyFile *config, const char *enable,
							const char *disable);
void plugin_cleanup(void);

void rfkill_init(void);
void rfkill_exit(void);
