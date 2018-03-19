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

#ifndef __MCAP_UTIL_H__
#define __MCAP_UTIL_H__



typedef void (*mcap_continue_mdep_f)(uint8_t mdep, gpointer user_data,
								GError *err);
typedef void (*mcap_continue_dcpsm_f)(uint16_t dcpsm, gpointer user_data,
								GError *err);
typedef void (*mcap_continue_proc_f)(gpointer user_data, GError *err);

struct conn_mcl_data {
	int			refs;
	gpointer		data;
	mcap_continue_proc_f func;
	GDestroyNotify	destroy;
	DBusConnection	*conn;	/* For name listener handling */
	DBusMessage *msg;
	struct mcap_device	*dev;
};

struct mcap_application *mcap_get_app_config(DBusMessageIter *iter, GError **err);
gboolean mcap_update_sdp_record(struct mcap_adapter *adapter, GSList *app_list);
/*gboolean mcap_get_mdep(struct mcap_device *device, struct mcap_application *app,
				mcap_continue_mdep_f func,
				gpointer data, GDestroyNotify destroy,
				GError **err);*/

gboolean mcap_establish_mcl(struct mcap_device *device,
						mcap_continue_proc_f func,
						gpointer data,
						GDestroyNotify destroy,
						GError **err);

/*gboolean mcap_get_dcpsm(struct mcap_device *device, mcap_continue_dcpsm_f func,
							gpointer data,
							GDestroyNotify destroy,
							GError **err);

*/
struct mcap_application *mcap_application_ref(struct mcap_application *app);
void mcap_application_unref(struct mcap_application *app);

struct mcap_device *mcap_device_ref(struct mcap_device *mcap_dev);
void mcap_device_unref(struct mcap_device *mcap_dev);
/*
void con_mcl_data_unref(struct conn_mcl_data *conn_data);

void mcap_channel_unref(struct mcap_channel *chan);

struct mcap_channel *mcap_channel_ref(struct mcap_channel *chan);

void device_unref_mcl(struct mcap_device *mcap_device);
*/
#endif /* __MCAP_UTIL_H__ */
