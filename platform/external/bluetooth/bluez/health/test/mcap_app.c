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

#include <gdbus.h>

#include "log.h"
#include "error.h"
#include <stdlib.h>
#include <stdint.h>
#include "mcap_types.h"
#include "mcap_utils.h"
#include <adapter.h>
#include <device.h>
#include "../mcap.h"
#include <btio.h>
#include "../mcap_lib.h"
#include <bluetooth/l2cap.h>
#include <sdpd.h>
#include "../src/dbus-common.h"
#include <unistd.h>

#ifndef DBUS_TYPE_UNIX_FD
	#define DBUS_TYPE_UNIX_FD -1
#endif
#define DBG(fmt, arg...)  printf("MCAP APP: %s: " fmt "\n" , __FUNCTION__ , ## arg)

#define ECHO_TIMEOUT	1 /* second */
#define MCAP_ECHO_LEN	15

static DBusConnection *connection = NULL;

static GSList *applications = NULL;
static GSList *devices = NULL;
static uint8_t next_app_id = MCAP_MDEP_INITIAL;

static GSList *adapters;
static struct mcap_application *application = NULL;

static gboolean update_adapter(struct mcap_adapter *adapter);
static struct mcap_device *create_mcap_device(DBusConnection *conn,
						struct btd_device *device);
static void free_echo_data(struct mcap_echo_data *edata);

struct mcap_create_dc {
	DBusConnection			*conn;
	DBusMessage			*msg;
	struct mcap_application		*app;
	struct mcap_device		*dev;
	uint8_t				config;
	uint8_t				mdep;
	guint				ref;
	mcap_mdl_operation_cb		cb;
};

struct mcap_create_mcl_data {
	DBusConnection			*conn;
	DBusMessage			*msg;
	struct mcap_application		*app;
	struct mcap_device		*dev;
	uint8_t				config;
	uint8_t				mdep;
	guint				ref;
	mcap_mdl_operation_cb		cb;
};


struct mcap_tmp_dc_data {
	DBusConnection			*conn;
	DBusMessage			*msg;
	struct mcap_channel		*mcap_chann;
	guint				ref;
	mcap_mdl_operation_cb		cb;
};

struct mcap_echo_data {
	gboolean		echo_done;	/* Is a echo was already done */
	gpointer		buf;		/* echo packet sent */
	uint			tid;		/* echo timeout */
};
void mcap_device_unregister(struct btd_device *device);


static struct mcap_channel *mcap_channel_ref(struct mcap_channel *chan)
{
DBG("");
	if (!chan)
		return NULL;

	chan->ref++;

	DBG("mcap_channel_ref(%p): ref=%d", chan, chan->ref);
	return chan;
}

static void free_mcap_channel(struct mcap_channel *chan)
{
DBG("");
		if (chan->mdep == MCAP_MDEP_ECHO) {
		free_echo_data(chan->edata);
		chan->edata = NULL;
	}

	mcap_mdl_unref(chan->mdl);
	mcap_application_unref(chan->app);
	mcap_device_unref(chan->dev);
	g_free(chan->path);
	g_free(chan);
}

static void mcap_channel_unref(struct mcap_channel *chan)
{
DBG("");
		if (!chan)
		return;

	chan->ref --;
	DBG("mcap_channel_unref(%p): ref=%d", chan, chan->ref);

	if (chan->ref > 0)
		return;

	free_mcap_channel(chan);
}

static void free_mcap_create_dc(struct mcap_create_dc *dc_data)
{
DBG("");
		dbus_message_unref(dc_data->msg);
	dbus_connection_unref(dc_data->conn);
	mcap_application_unref(dc_data->app);
	mcap_device_unref(dc_data->dev);

	g_free(dc_data);
}

static struct mcap_create_dc *mcap_create_data_ref(struct mcap_create_dc *dc_data)
{
DBG("");
		dc_data->ref++;

	DBG("mcap_create_data_ref(%p): ref=%d", dc_data, dc_data->ref);

	return dc_data;
}

static void mcap_create_data_unref(struct mcap_create_dc *dc_data)
{
DBG("");
		dc_data->ref--;

	DBG("mcap_create_data_unref(%p): ref=%d", dc_data, dc_data->ref);

	if (dc_data->ref > 0)
		return;

	free_mcap_create_dc(dc_data);
}

static void free_mcap_conn_dc(struct mcap_tmp_dc_data *data)
{
DBG("");
		dbus_message_unref(data->msg);
	dbus_connection_unref(data->conn);
	mcap_channel_unref(data->mcap_chann);

	g_free(data);
}

static struct mcap_tmp_dc_data *mcap_tmp_dc_data_ref(struct mcap_tmp_dc_data *data)
{
DBG("");
		data->ref++;

	DBG("mcap_conn_data_ref(%p): ref=%d", data, data->ref);

	return data;
}

static void mcap_tmp_dc_data_unref(struct mcap_tmp_dc_data *data)
{
DBG("");
		data->ref--;

	DBG("mcap_conn_data_unref(%p): ref=%d", data, data->ref);

	if (data->ref > 0)
		return;

	free_mcap_conn_dc(data);
}

static int cmp_app_id(gconstpointer a, gconstpointer b)
{
DBG("");
		const struct mcap_application *app = a;
	const uint8_t *id = b;

	return app->id - *id;
}

static int cmp_adapter(gconstpointer a, gconstpointer b)
{
DBG("");
		const struct mcap_adapter *mcap_adapter = a;
	const struct btd_adapter *adapter = b;

	if (mcap_adapter->btd_adapter == adapter)
		return 0;

	return -1;
}

static int cmp_device(gconstpointer a, gconstpointer b)
{
DBG("");
		const struct mcap_device *mcap_device = a;
	const struct btd_device *device = b;

	if (mcap_device->dev == device)
		return 0;

	return -1;
}

static gint cmp_dev_addr(gconstpointer a, gconstpointer dst)
{
DBG("");
		const struct mcap_device *device = a;
	bdaddr_t addr;

	device_get_address(device->dev, &addr, NULL);
	return bacmp(&addr, dst);
}

static gint cmp_dev_mcl(gconstpointer a, gconstpointer mcl)
{
DBG("");
		const struct mcap_device *device = a;

	if (mcl == device->mcl)
		return 0;
	return -1;
}

static gint cmp_chan_mdlid(gconstpointer a, gconstpointer b)
{
DBG("");
		const struct mcap_channel *chan = a;
	const uint16_t *mdlid = b;

	return chan->mdlid - *mdlid;
}

static gint cmp_chan_path(gconstpointer a, gconstpointer b)
{ DBG("");
	const struct mcap_channel *chan = a;
	const char *path = b;

	return g_ascii_strcasecmp(chan->path, path);
}

static gint cmp_chan_mdl(gconstpointer a, gconstpointer mdl)
{ DBG("");
	const struct mcap_channel *chan = a;

	if (chan->mdl == mdl)
		return 0;
	return -1;
}

static uint8_t get_app_id(void)
{ DBG("");
	uint8_t id = next_app_id;

	do { DBG("");
		GSList *l = g_slist_find_custom(applications, &id, cmp_app_id);

		if (!l) {
			next_app_id = (id % MCAP_MDEP_FINAL) + 1;
			return id;
		} else
			id = (id % MCAP_MDEP_FINAL) + 1;
	} while (id != next_app_id);

	/* No more ids available */
	return 0;
}

static int cmp_app(gconstpointer a, gconstpointer b)
{ DBG("");
	const struct mcap_application *app = a;

	return g_strcmp0(app->path, b);
}

static gboolean set_app_path(struct mcap_application *app)
{ DBG("");
	app->id =0x10;
	
	if (!app->id)
		return FALSE;
	app->path = g_strdup_printf(MANAGER_PATH "/mcap_app_%d", app->id);

	return TRUE;
};

static void device_unref_mcl(struct mcap_device *mcap_device)
{ DBG("");
	if (!mcap_device->mcl)
		return;

	mcap_close_mcl(mcap_device->mcl, FALSE);
	mcap_mcl_unref(mcap_device->mcl);
	mcap_device->mcl = NULL;
	mcap_device->mcl_conn = FALSE;
}

static void free_mcap_device(struct mcap_device *device)
{ DBG("");
	if (device->conn) {
		dbus_connection_unref(device->conn);
		device->conn = NULL;
	}

	if (device->dev) {
		btd_device_unref(device->dev);
		device->dev = NULL;
	}

	device_unref_mcl(device);

	g_free(device);
}

static void remove_application(struct mcap_application *app)
{ DBG("");
	DBG("Application %s deleted", app->path);
	mcap_application_unref(app);
	DBG("Application unrefed");

	g_slist_foreach(adapters, (GFunc) update_adapter, NULL);
	DBG("adapter updated");
}

static void client_disconnected(DBusConnection *conn, void *user_data)
{ DBG("");
	struct mcap_application *app = user_data;

	DBG("Client disconnected from the bus, deleting mcap application");
	applications = g_slist_remove(applications, app);

	app->dbus_watcher = 0; /* Watcher shouldn't be freed in this case */
	remove_application(app);
}

static DBusMessage *manager_create_application(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	struct mcap_application *app;
	const char *name;
	DBusMessageIter iter;
	GError *err = NULL;

	if (application != NULL)
		remove_application(application);
	application = NULL;
	
	dbus_message_iter_init(msg, &iter);
	app = mcap_get_app_config(&iter, &err);
	
	if (err) {
		g_error_free(err);
		return btd_error_invalid_args(msg);
	}
	DBG("app %d %s %d %d", app->role, app->description, app->chan_type, app->data_type);
	name = dbus_message_get_sender(msg);
	if (!name) {
		mcap_application_unref(app);
		return g_dbus_create_error(msg,
					ERROR_INTERFACE ".HealthError",
					"Can't get sender name");
	}
	
	if (!set_app_path(app)) {
		mcap_application_unref(app);
		return g_dbus_create_error(msg,
				ERROR_INTERFACE ".HealthError",
				"Can't get a valid id for the application");
	}

	app->oname = g_strdup(name);
	app->conn = dbus_connection_ref(conn);
	
	applications = g_slist_prepend(applications, app);
	application = app;
	app->dbus_watcher = g_dbus_add_disconnect_watch(conn, name,
						client_disconnected, app, NULL);
	g_slist_foreach(adapters, (GFunc) update_adapter, NULL);

	DBG("Mcap application created with id %s", app->path);

	return g_dbus_create_reply(msg, DBUS_TYPE_OBJECT_PATH, &app->path,
							DBUS_TYPE_INVALID);
}

static DBusMessage *manager_destroy_application(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	const char *path;
	struct mcap_application *app;
	GSList *l;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_OBJECT_PATH, &path,
						DBUS_TYPE_INVALID))
		return btd_error_invalid_args(msg);

	l = g_slist_find_custom(applications, path, cmp_app);

	if (!l)
		return g_dbus_create_error(msg,
					ERROR_INTERFACE ".InvalidArguments",
					"Invalid arguments in method call, "
					"no such application");

	app = l->data;
	applications = g_slist_remove(applications, app);

	remove_application(app);

	return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
}

static void manager_path_unregister(gpointer data)
{ DBG("");
	g_slist_foreach(applications, (GFunc) mcap_application_unref, NULL);

	g_slist_free(applications);
	applications = NULL;

	g_slist_foreach(adapters, (GFunc) update_adapter, NULL);
}

static DBusMessage *manager_test(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	DBG("Mcap test succsess");
	return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
}

static GDBusMethodTable mcap_manager_methods[] = {
	{"CreateApplication", "", "o", manager_create_application},
	{"DestroyApplication", "o", "", manager_destroy_application},
	{"Test", "", "", manager_test},
	{ NULL }
};

static DBusMessage *channel_get_properties(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	struct mcap_channel *chan = user_data;
	DBusMessageIter iter, dict;
	DBusMessage *reply;
	const char *path;
	char *type;

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	dbus_message_iter_init_append(reply, &iter);

	dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
			DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
			DBUS_TYPE_STRING_AS_STRING DBUS_TYPE_VARIANT_AS_STRING
			DBUS_DICT_ENTRY_END_CHAR_AS_STRING, &dict);

	path = device_get_path(chan->dev->dev);
	dict_append_entry(&dict, "Device", DBUS_TYPE_OBJECT_PATH, &path);

	path = chan->app->path;
	dict_append_entry(&dict, "Application", DBUS_TYPE_OBJECT_PATH, &path);

	if (chan->config == MCAP_RELIABLE_DC)
		type = g_strdup("Reliable");
	else
		type = g_strdup("Streaming");

	dict_append_entry(&dict, "Type", DBUS_TYPE_STRING, &type);

	g_free(type);

	dbus_message_iter_close_container(&iter, &dict);

	return reply;
}

static void mcap_tmp_dc_data_destroy(gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *mcap_conn = data;

	mcap_tmp_dc_data_unref(mcap_conn);
}

static void abort_mdl_cb(GError *err, gpointer data)
{ DBG("");
	if (err)
		error("Aborting error: %s", err->message);
}

static void mcap_mdl_reconn_cb(struct mcap_mdl *mdl, GError *err, gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *dc_data = data;
	DBusMessage *reply;
	int fd;

	if (err) {
		struct mcap_channel *chan = dc_data->mcap_chann;
		GError *gerr = NULL;

		error("%s", err->message);
		reply = g_dbus_create_error(dc_data->msg,
					ERROR_INTERFACE ".HealthError",
					"Cannot reconnect: %s", err->message);
		g_dbus_send_message(dc_data->conn, reply);

		/* Send abort request because remote side */
		/* is now in PENDING state */
		if (!mcap_mdl_abort(chan->mdl, abort_mdl_cb, NULL, NULL,
								&gerr)) {
			error("%s", gerr->message);
			g_error_free(gerr);
		}
		return;
	}

	fd = mcap_mdl_get_fd(dc_data->mcap_chann->mdl);
	if (fd < 0) {
		reply = g_dbus_create_error(dc_data->msg,
						ERROR_INTERFACE ".HealthError",
						"Cannot get file descriptor");
		g_dbus_send_message(dc_data->conn, reply);
		return;
	}

	reply = g_dbus_create_reply(dc_data->msg, DBUS_TYPE_UNIX_FD,
							&fd, DBUS_TYPE_INVALID);
	g_dbus_send_message(dc_data->conn, reply);

	g_dbus_emit_signal(dc_data->conn,
			device_get_path(dc_data->mcap_chann->dev->dev),
			MCAP_DEVICE, "ChannelConnected",
			DBUS_TYPE_OBJECT_PATH, &dc_data->mcap_chann->path,
			DBUS_TYPE_INVALID);
}

static void mcap_get_dcpsm_cb(uint16_t dcpsm, gpointer user_data, GError *err)
{ DBG("");
	struct mcap_tmp_dc_data *mcap_conn = user_data;
	struct mcap_channel *mcap_chann = mcap_conn->mcap_chann;
	GError *gerr = NULL;
	uint8_t mode;

	if (err) {
		mcap_conn->cb(mcap_chann->mdl, err, mcap_conn);
		return;
	}

	if (mcap_chann->config == MCAP_RELIABLE_DC)
		mode = L2CAP_MODE_ERTM;
	else
		mode = L2CAP_MODE_STREAMING;

	if (mcap_connect_mdl(mcap_chann->mdl, mode, dcpsm, mcap_conn->cb,
					mcap_tmp_dc_data_ref(mcap_conn),
					mcap_tmp_dc_data_destroy, &gerr))
		return;

	mcap_tmp_dc_data_unref(mcap_conn);
	mcap_conn->cb(mcap_chann->mdl, err, mcap_conn);
	g_error_free(gerr);
	gerr = NULL;
}

static void device_reconnect_mdl_cb(struct mcap_mdl *mdl, GError *err,
								gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *dc_data = data;
	GError *gerr = NULL;
	DBusMessage *reply;

	if (err) {
		reply = g_dbus_create_error(dc_data->msg,
					ERROR_INTERFACE ".HealthError",
					"Cannot reconnect: %s", err->message);
		g_dbus_send_message(dc_data->conn, reply);
		return;
	}

	dc_data->cb = mcap_mdl_reconn_cb;

	if (mcap_get_dcpsm(dc_data->mcap_chann->dev, mcap_get_dcpsm_cb,
					mcap_tmp_dc_data_ref(dc_data),
					mcap_tmp_dc_data_destroy, &gerr))
		return;

	error("%s", gerr->message);

	reply = g_dbus_create_error(dc_data->msg,
					ERROR_INTERFACE ".HealthError",
					"Cannot reconnect: %s", gerr->message);
	g_dbus_send_message(dc_data->conn, reply);
	mcap_tmp_dc_data_unref(dc_data);
	g_error_free(gerr);

	/* Send abort request because remote side is now in PENDING state */
	if (!mcap_mdl_abort(mdl, abort_mdl_cb, NULL, NULL, &gerr)) {
		error("%s", gerr->message);
		g_error_free(gerr);
	}
}

static DBusMessage *channel_acquire_continue(struct mcap_tmp_dc_data *data,
								GError *err)
{ DBG("");
	DBusMessage *reply;
	GError *gerr = NULL;
	int fd;

	if (err) {
		return g_dbus_create_error(data->msg,
						ERROR_INTERFACE ".HealthError",
						"%s", err->message);
	}

	fd = mcap_mdl_get_fd(data->mcap_chann->mdl);
	DBG("fd = %d\n", fd);
	if (fd >= 0)
		return g_dbus_create_reply(data->msg, DBUS_TYPE_UNIX_FD, &fd,
							DBUS_TYPE_INVALID);

	mcap_tmp_dc_data_ref(data);
	if (mcap_reconnect_mdl(data->mcap_chann->mdl, device_reconnect_mdl_cb,
					data, mcap_tmp_dc_data_destroy, &gerr))
		return NULL;

	mcap_tmp_dc_data_unref(data);
	reply = g_dbus_create_error(data->msg, ERROR_INTERFACE ".HealthError",
					"Cannot reconnect: %s", gerr->message);
	g_error_free(gerr);

	return reply;
}

static void channel_acquire_cb(gpointer data, GError *err)
{
	struct mcap_tmp_dc_data *dc_data = data;
	DBusMessage *reply;

	reply = channel_acquire_continue(data, err);

	if (reply)
		g_dbus_send_message(dc_data->conn, reply);
}

static DBusMessage *channel_acquire(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	struct mcap_channel *chan = user_data;
	struct mcap_tmp_dc_data *dc_data;
	GError *gerr = NULL;
	DBusMessage *reply;

	dc_data = g_new0(struct mcap_tmp_dc_data, 1);
	dc_data->conn = dbus_connection_ref(conn);
	dc_data->msg = dbus_message_ref(msg);
	dc_data->mcap_chann = mcap_channel_ref(chan);

	if (chan->dev->mcl_conn) {
		reply = channel_acquire_continue(mcap_tmp_dc_data_ref(dc_data),
									NULL);
		mcap_tmp_dc_data_unref(dc_data);
		DBG("and return reply");
		return reply;
	}

	if (mcap_establish_mcl(chan->dev, channel_acquire_cb,
						mcap_tmp_dc_data_ref(dc_data),
						mcap_tmp_dc_data_destroy, &gerr))
		return NULL;

	reply = g_dbus_create_error(msg, ERROR_INTERFACE ".HealthError",
					"%s", gerr->message);
	mcap_tmp_dc_data_unref(dc_data);
	g_error_free(gerr);

	return reply;
}

static void close_mdl(struct mcap_channel *mcap_chann)
{ DBG("");
	int fd;

	fd = mcap_mdl_get_fd(mcap_chann->mdl);
	if (fd < 0)
		return;

	close(fd);
}

static DBusMessage *channel_release(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	struct mcap_channel *mcap_chann = user_data;

	close_mdl(mcap_chann);

	return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
}

static void free_echo_data(struct mcap_echo_data *edata)
{ DBG("");
	if (!edata)
		return;

	if (edata->tid)
		g_source_remove(edata->tid);

	if (edata->buf)
		g_free(edata->buf);


	g_free(edata);
}

static void mcap_channel_destroy(void *data)
{ DBG("");
	struct mcap_channel *mcap_chan = data;
	struct mcap_device *dev = mcap_chan->dev;

	DBG("Destroy mcap Channel %s", mcap_chan->path);
	if (!g_slist_find(dev->channels, mcap_chan))
		goto end;

	dev->channels = g_slist_remove(dev->channels, mcap_chan);

	if (mcap_chan->mdep != MCAP_MDEP_ECHO)
		g_dbus_emit_signal(dev->conn, device_get_path(dev->dev),
					MCAP_DEVICE, "ChannelDeleted",
					DBUS_TYPE_OBJECT_PATH, &mcap_chan->path,
					DBUS_TYPE_INVALID);

	if (mcap_chan == dev->fr) {
		char *empty_path;

		mcap_channel_unref(dev->fr);
		dev->fr = NULL;
		empty_path = "/";
		emit_property_changed(dev->conn, device_get_path(dev->dev),
					MCAP_DEVICE, "MainChannel",
					DBUS_TYPE_OBJECT_PATH, &empty_path);
	}

end:
	mcap_channel_unref(mcap_chan);
}

static GDBusMethodTable mcap_channels_methods[] = {
	{"GetProperties","",	"a{sv}",	channel_get_properties },
	{"Acquire",	"",	"h",		channel_acquire,
						G_DBUS_METHOD_FLAG_ASYNC },
	{"Release",	"",	"",		channel_release },
	{ NULL }
};

static struct mcap_channel *create_channel(struct mcap_device *dev,
						uint8_t config,
						struct mcap_mdl *mdl,
						uint16_t mdlid,
						struct mcap_application *app,
						GError **err)
{ DBG("");
	struct mcap_channel *mcap_chann;

	if (!dev)
		return NULL;

	mcap_chann = g_new0(struct mcap_channel, 1);
	mcap_chann->config = config;
	mcap_chann->dev = mcap_device_ref(dev);
	mcap_chann->mdlid = mdlid;

	if (mdl)
		mcap_chann->mdl = mcap_mdl_ref(mdl);

	if (app) { DBG("");
		mcap_chann->mdep = app->id;
		mcap_chann->app = mcap_application_ref(app);
	} else
		mcap_chann->edata = g_new0(struct mcap_echo_data, 1);

	mcap_chann->path = g_strdup_printf("%s/chan%d",
					device_get_path(mcap_chann->dev->dev),
					mcap_chann->mdlid);

	dev->channels = g_slist_append(dev->channels,
						mcap_channel_ref(mcap_chann));

	if (mcap_chann->mdep == MCAP_MDEP_ECHO)
		return mcap_channel_ref(mcap_chann);

	if (!g_dbus_register_interface(dev->conn, mcap_chann->path,
					MCAP_CHANNEL,
					mcap_channels_methods, NULL, NULL,
					mcap_chann, mcap_channel_destroy)) {
		g_set_error(err, MCAP_ERROR, MCAP_UNSPECIFIED_ERROR,
					"Can't register the channel interface");
		mcap_channel_destroy(mcap_chann);
		return NULL;
	}

	return mcap_channel_ref(mcap_chann);
}

static void remove_channels(struct mcap_device *dev)
{ DBG("");
	struct mcap_channel *chan;
	char *path;

	while (dev->channels) {
		chan = dev->channels->data;

		path = g_strdup(chan->path);
		if (!g_dbus_unregister_interface(dev->conn, path,
								MCAP_CHANNEL))
			mcap_channel_destroy(chan);
		g_free(path);
	}
}

static void close_device_con(struct mcap_device *dev, gboolean cache)
{ DBG("");
	if (!dev->mcl)
		return;

	mcap_close_mcl(dev->mcl, cache);
	dev->mcl_conn = FALSE;

	if (cache)
		return;

	device_unref_mcl(dev);
	remove_channels(dev);

	if (!dev->sdp_present) {
		const char *path;

		path = device_get_path(dev->dev);
		g_dbus_unregister_interface(dev->conn, path, MCAP_DEVICE);
	}
}
/*
static int send_echo_data(int sock, const void *buf, uint32_t size)
{ DBG("");
	const uint8_t *buf_b = buf;
	uint32_t sent = 0;

	while (sent < size) {
		int n = write(sock, buf_b + sent, size - sent);
		if (n < 0)
			return -1;
		sent += n;
	}

	return 0;
}

static gboolean serve_echo(GIOChannel *io_chan, GIOCondition cond,
								gpointer data)
{ DBG("");
	struct mcap_channel *chan = data;
	uint8_t buf[MCAP_DC_MTU];
	int fd, len;

	if (cond & (G_IO_ERR | G_IO_HUP | G_IO_NVAL)) {
		mcap_channel_unref(chan);
		return FALSE;
	}

	if (chan->edata->echo_done)
		goto fail;

	chan->edata->echo_done = TRUE;

	fd = g_io_channel_unix_get_fd(io_chan);
	len = read(fd, buf, sizeof(buf));

	if (send_echo_data(fd, buf, len)  >= 0)
		return TRUE;

fail:
	close_device_con(chan->dev, FALSE);
	mcap_channel_unref(chan);
	return FALSE;
}
*/
static gboolean check_channel_conf(struct mcap_channel *chan)
{ DBG("");
	GError *err = NULL;
	GIOChannel *io;
	uint8_t mode;
	uint16_t imtu, omtu;
	int fd;

	fd = mcap_mdl_get_fd(chan->mdl);
	if (fd < 0)
		return FALSE;
	io = g_io_channel_unix_new(fd);

	if (!bt_io_get(io, BT_IO_L2CAP, &err,
			BT_IO_OPT_MODE, &mode,
			BT_IO_OPT_IMTU, &imtu,
			BT_IO_OPT_OMTU, &omtu,
			BT_IO_OPT_INVALID)) {
		error("Error: %s", err->message);
		g_io_channel_unref(io);
		g_error_free(err);
		return FALSE;
	}

	g_io_channel_unref(io);

	switch (chan->config) {
	case MCAP_RELIABLE_DC:
		if (mode != L2CAP_MODE_ERTM)
			return FALSE;
		break;
	case MCAP_STREAMING_DC:
		if (mode != L2CAP_MODE_STREAMING)
			return FALSE;
		break;
	default:
		error("Error: Connected with unknown configuration");
		return FALSE;
	}

	DBG("MDL imtu %d omtu %d Channel imtu %d omtu %d", imtu, omtu,
						chan->imtu, chan->omtu);

	if (!chan->imtu)
		chan->imtu = imtu;
	if (!chan->omtu)
		chan->omtu = omtu;

	if (chan->imtu != imtu || chan->omtu != omtu)
		return FALSE;

	return TRUE;
}

static void mcap_mcap_mdl_connected_cb(struct mcap_mdl *mdl, void *data)
{ DBG("");
	struct mcap_device *dev = data;
	struct mcap_channel *chan;

	DBG("mcap_mcap_mdl_connected_cb");
	if (!dev->ndc)
		return;

	chan = dev->ndc;
	if (!chan->mdl)
		chan->mdl = mcap_mdl_ref(mdl);

	if (!g_slist_find(dev->channels, chan))
		dev->channels = g_slist_prepend(dev->channels,
							mcap_channel_ref(chan));

	if (!check_channel_conf(chan)) {
		close_mdl(chan);
		goto end;
	}

	DBG("signal path = %s %s", device_get_path(dev->dev), MCAP_DEVICE);
	g_dbus_emit_signal(dev->conn, device_get_path(dev->dev), MCAP_DEVICE,
					"ChannelConnected",
					DBUS_TYPE_OBJECT_PATH, &chan->path,
					DBUS_TYPE_INVALID);

	if (dev->fr)
		goto end;

	dev->fr = mcap_channel_ref(chan);

	emit_property_changed(dev->conn, device_get_path(dev->dev),
					MCAP_DEVICE, "MainChannel",
					DBUS_TYPE_OBJECT_PATH, &dev->fr->path);

end:
	mcap_channel_unref(dev->ndc);
	dev->ndc = NULL;
}

static void mcap_mcap_mdl_closed_cb(struct mcap_mdl *mdl, void *data)
{ DBG("");
	/* struct mcap_device *dev = data; */

	DBG("mcap_mcap_mdl_closed_cb");

	/* Nothing to do */
}

static void mcap_mcap_mdl_deleted_cb(struct mcap_mdl *mdl, void *data)
{ DBG("");
	struct mcap_device *dev = data;
	struct mcap_channel *chan;
	char *path;
	GSList *l;

	DBG("mcap_mcap_mdl_deleted_cb");
	l = g_slist_find_custom(dev->channels, mdl, cmp_chan_mdl);
	if (!l)
		return;

	chan = l->data;

	path = g_strdup(chan->path);
	if (!g_dbus_unregister_interface(dev->conn, path, MCAP_CHANNEL))
		mcap_channel_destroy(chan);
	g_free(path);
}

static void mcap_mcap_mdl_aborted_cb(struct mcap_mdl *mdl, void *data)
{ DBG("");
	struct mcap_device *dev = data;

	DBG("mcap_mcap_mdl_aborted_cb");
	if (!dev->ndc)
		return;

	dev->ndc->mdl = mcap_mdl_ref(mdl);

	if (!g_slist_find(dev->channels, dev->ndc))
		dev->channels = g_slist_prepend(dev->channels,
						mcap_channel_ref(dev->ndc));

	if (dev->ndc->mdep != MCAP_MDEP_ECHO)
		g_dbus_emit_signal(dev->conn, device_get_path(dev->dev),
					MCAP_DEVICE, "ChannelConnected",
					DBUS_TYPE_OBJECT_PATH, &dev->ndc->path,
					DBUS_TYPE_INVALID);

	mcap_channel_unref(dev->ndc);
	dev->ndc = NULL;
}

static uint8_t mcap2l2cap_mode(uint8_t mcap_mode)
{ DBG("");
	return mcap_mode == MCAP_STREAMING_DC ? L2CAP_MODE_STREAMING :
								L2CAP_MODE_ERTM;
}

static uint8_t mcap_mcap_mdl_conn_req_cb(struct mcap_mcl *mcl, uint8_t mdepid,
				uint16_t mdlid, uint8_t *conf, void *data)
{ DBG("");
	struct mcap_device *dev = data;
	struct mcap_application *app;
	GError *err = NULL;
	GSList *l;

	DBG("Data channel request for %d", mdepid);
    ///// doronk for PTS test TC_MCAP_ERR_BI_17_C
	if (dev->unavailable) {
		DBG("dev->unavailable = %d , send MCAP_RESOURCE_UNAVAILABLE", dev->unavailable);
		return MCAP_RESOURCE_UNAVAILABLE;
	}
	///// doronk for PTS test TC_MCAP_ERR_BI_07_C
	if(0 == mdepid){
		DBG("Change mdepid to 16 for test TC_MCAPP_ERR_BI_07_C " );
		mdepid = 16;
	}
    /////////////////////////////////////////////////
	l = g_slist_find_custom(applications, &mdepid, cmp_app_id);
	if (!l)
		return MCAP_INVALID_MDEP;
	DBG("conf = %d", *conf);

	if (dev->unavailable) {
		return MCAP_RESOURCE_UNAVAILABLE;
	}
	
	app = l->data;

	/* Check if is the first dc if so,
	* only reliable configuration is allowed */
	
	switch (*conf) {
	case MCAP_NO_PREFERENCE_DC:
	
			*conf = MCAP_RELIABLE_DC;
		break;
	case MCAP_STREAMING_DC:
		if (!dev->fr || app->role == MCAP_SOURCE)
			return MCAP_CONFIGURATION_REJECTED;
	case MCAP_RELIABLE_DC:
		if (app->role == MCAP_SOURCE)
			return MCAP_CONFIGURATION_REJECTED;
		break;
	default:
		/* Special case defined in MCAP spec 3.4. When an invalid
		* configuration is received we shall close the MCL when
		* we are still processing the callback. */
		close_device_con(dev, FALSE);
		return MCAP_CONFIGURATION_REJECTED; /* not processed */
	}

	l = g_slist_find_custom(dev->channels, &mdlid, cmp_chan_mdlid);
	if (l) {
		struct mcap_channel *chan = l->data;
		char *path;

		path = g_strdup(chan->path);
		g_dbus_unregister_interface(dev->conn, path, MCAP_CHANNEL);
		g_free(path);
	}

	if (!mcap_set_data_chan_mode(dev->mcap_adapter->mi,
						mcap2l2cap_mode(*conf), &err)) {
		error("Error: %s", err->message);
		g_error_free(err);
		return MCAP_MDL_BUSY;
	}

	dev->ndc = create_channel(dev, *conf, NULL, mdlid, app, NULL);
	if (!dev->ndc)
		return MCAP_MDL_BUSY;

	return MCAP_SUCCESS;
}

static uint8_t mcap_mcap_mdl_reconn_req_cb(struct mcap_mdl *mdl, void *data)
{ DBG("");
	struct mcap_device *dev = data;
	struct mcap_channel *chan;
	GError *err = NULL;
	GSList *l;

	l = g_slist_find_custom(dev->channels, mdl, cmp_chan_mdl);
	if (!l)
		return MCAP_INVALID_MDL;

	chan = l->data;

	if (!dev->fr && (chan->config != MCAP_RELIABLE_DC) &&
						(chan->mdep != MCAP_MDEP_ECHO))
		return MCAP_UNSPECIFIED_ERROR;

	if (!mcap_set_data_chan_mode(dev->mcap_adapter->mi,
					mcap2l2cap_mode(chan->config), &err)) {
		error("Error: %s", err->message);
		g_error_free(err);
		return MCAP_MDL_BUSY;
	}

	dev->ndc = mcap_channel_ref(chan);

	return MCAP_SUCCESS;
}

gboolean mcap_set_mcl_cb(struct mcap_device *device, GError **err)
{ DBG("");
	gboolean ret;

	if (!device->mcl)
		return FALSE;

	ret = mcap_mcl_set_cb(device->mcl, device, err,
		MCAP_MDL_CB_CONNECTED, mcap_mcap_mdl_connected_cb,
		MCAP_MDL_CB_CLOSED, mcap_mcap_mdl_closed_cb,
		MCAP_MDL_CB_DELETED, mcap_mcap_mdl_deleted_cb,
		MCAP_MDL_CB_ABORTED, mcap_mcap_mdl_aborted_cb,
		MCAP_MDL_CB_REMOTE_CONN_REQ, mcap_mcap_mdl_conn_req_cb,
		MCAP_MDL_CB_REMOTE_RECONN_REQ, mcap_mcap_mdl_reconn_req_cb,
		MCAP_MDL_CB_INVALID);

	if (ret)
		return TRUE;

	error("Can't set mcl callbacks, closing mcl");
	close_device_con(device, TRUE);

	return FALSE;
}

static void mcl_connected(struct mcap_mcl *mcl, gpointer data)
{ DBG("");
	struct mcap_device *mcap_device;
	bdaddr_t addr;
	GSList *l;

	mcap_mcl_get_addr(mcl, &addr);
	l = g_slist_find_custom(devices, &addr, cmp_dev_addr);
	if (!l) {
		struct mcap_adapter *mcap_adapter = data;
		struct btd_device *device;
		char str[18];

		ba2str(&addr, str);
		device = adapter_get_device(connection,
					mcap_adapter->btd_adapter, str);
		if (!device)
			return;
		mcap_device = create_mcap_device(connection, device);
		if (!mcap_device)
			return;
		devices = g_slist_append(devices, mcap_device);
	} else
		mcap_device = l->data;

	mcap_device->mcl = mcap_mcl_ref(mcl);
	mcap_device->mcl_conn = TRUE;

DBG(" set temporary");
	device_set_temporary(mcap_device->dev, FALSE);

	DBG("New mcl connected from  %s", device_get_path(mcap_device->dev));


	mcap_set_mcl_cb(mcap_device, NULL);
}

static void mcl_reconnected(struct mcap_mcl *mcl, gpointer data)
{ DBG("");
	struct mcap_device *mcap_device;
	GSList *l;

	l = g_slist_find_custom(devices, mcl, cmp_dev_mcl);
	if (!l)
		return;

	mcap_device = l->data;
	mcap_device->mcl_conn = TRUE;

	DBG("MCL reconnected %s", device_get_path(mcap_device->dev));
	device_set_temporary(mcap_device->dev, FALSE);
	mcap_set_mcl_cb(mcap_device, NULL);
}

static void mcl_disconnected(struct mcap_mcl *mcl, gpointer data)
{ DBG("");
	struct mcap_device *mcap_device;
	GSList *l;

	l = g_slist_find_custom(devices, mcl, cmp_dev_mcl);
	if (!l)
		return;

	mcap_device = l->data;
	mcap_device->mcl_conn = FALSE;

	DBG("Mcl disconnected %s", device_get_path(mcap_device->dev));
	//mcap_device_unregister(mcap_device->dev);
}

static void mcl_uncached(struct mcap_mcl *mcl, gpointer data)
{
	struct mcap_device *mcap_device;
	const char *path;
	GSList *l;

	l = g_slist_find_custom(devices, mcl, cmp_dev_mcl);
	if (!l)
		return;

	mcap_device = l->data;
	device_unref_mcl(mcap_device);

	if (mcap_device->sdp_present)
		return;

	/* Because remote device hasn't announced an MCAP record */
	/* the Bluetooth daemon won't notify when the device shall */
	/* be removed. Then we have to remove the HealthDevice */
	/* interface manually */
	path = device_get_path(mcap_device->dev);
	g_dbus_unregister_interface(mcap_device->conn, path, MCAP_DEVICE);
	DBG("Mcl uncached %s", path);
}

static void check_devices_mcl(void)
{ DBG("");
	struct mcap_device *dev;
	GSList *l, *to_delete = NULL;

	for (l = devices; l; l = l->next) {
		dev = l->data;
		device_unref_mcl(dev);

		if (!dev->sdp_present)
			to_delete = g_slist_append(to_delete, dev);
		else
			remove_channels(dev);
	}

	for (l = to_delete; l; l = l->next) {
		const char *path;

		path = device_get_path(dev->dev);
		g_dbus_unregister_interface(dev->conn, path, MCAP_DEVICE);
	}

	g_slist_free(to_delete);
}

static void release_adapter_instance(struct mcap_adapter *mcap_adapter)
{ DBG("");
	if (!mcap_adapter->mi)
		return;

	check_devices_mcl();
	mcap_release_instance(mcap_adapter->mi);
	mcap_instance_unref(mcap_adapter->mi);
	mcap_adapter->mi = NULL;
}

static gboolean update_adapter(struct mcap_adapter *mcap_adapter)
{ DBG("");
	GError *err = NULL;
	bdaddr_t addr;
	printf("update adapter called");
	if (!applications) {
		release_adapter_instance(mcap_adapter);
		goto update;
	}

	if (mcap_adapter->mi)
		goto update;

	adapter_get_address(mcap_adapter->btd_adapter, &addr);
	mcap_adapter->mi = mcap_create_instance(&addr, BT_IO_SEC_MEDIUM, 0, 0,
					mcl_connected, mcl_reconnected,
					mcl_disconnected, mcl_uncached,
					NULL, /* CSP is not used by now */
					mcap_adapter, &err);

	if (!mcap_adapter->mi) {
		error("Error creating the MCAP instance: %s", err->message);
		g_error_free(err);
		return FALSE;
	}

	mcap_adapter->ccpsm = mcap_get_ctrl_psm(mcap_adapter->mi, &err);
	if (err) {
		error("Error getting MCAP control PSM: %s", err->message);
		goto fail;
	}

	mcap_adapter->dcpsm = mcap_get_data_psm(mcap_adapter->mi, &err);
	if (err) {
		error("Error getting MCAP data PSM: %s", err->message);
		goto fail;
	}
		DBG("mcap_adapter->ccpsm = %d mcap_adapter->dcpsm = %d\n", mcap_adapter->ccpsm, mcap_adapter->dcpsm);
update:
	if (mcap_update_sdp_record(mcap_adapter, applications))
		return TRUE;
	error("Error updating the SDP record");

fail:
	release_adapter_instance(mcap_adapter);
	if (err)
		g_error_free(err);
	return FALSE;
}

int mcap_adapter_register(DBusConnection *conn, struct btd_adapter *adapter)
{
	struct mcap_adapter *mcap_adapter;
	printf("mcap_adapter_register\n");
	mcap_adapter = g_new0(struct mcap_adapter, 1);
	mcap_adapter->btd_adapter = btd_adapter_ref(adapter);

	if(!update_adapter(mcap_adapter))
		goto fail;

	adapters = g_slist_append(adapters, mcap_adapter);

	return 0;

fail:
	btd_adapter_unref(mcap_adapter->btd_adapter);
	g_free(mcap_adapter);
	return -1;
}

void mcap_adapter_unregister(struct btd_adapter *adapter)
{ DBG("");
	struct mcap_adapter *mcap_adapter;
	GSList *l;

	l = g_slist_find_custom(adapters, adapter, cmp_adapter);

	if (!l)
		return;

	mcap_adapter = l->data;
	adapters = g_slist_remove(adapters, mcap_adapter);
	if (mcap_adapter->sdp_handler)
		remove_record_from_server(mcap_adapter->sdp_handler);
	release_adapter_instance(mcap_adapter);
	btd_adapter_unref(mcap_adapter->btd_adapter);
	g_free(mcap_adapter);
}
 

static void destroy_create_dc_data(gpointer data)
{ DBG("");
	struct mcap_create_dc *dc_data = data;

	mcap_create_data_unref(dc_data);
}
/*
static void *generate_echo_packet(void)
{ DBG("");
	uint8_t *buf;
	int i;

	buf = g_malloc(MCAP_ECHO_LEN);
	srand(time(NULL));

	for(i = 0; i < MCAP_ECHO_LEN; i++)
		buf[i] = rand() % UINT8_MAX;

	return buf;
}

static gboolean check_echo(GIOChannel *io_chan, GIOCondition cond,
								gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *mcap_conn =  data;
	struct mcap_echo_data *edata = mcap_conn->mcap_chann->edata;
	struct mcap_channel *chan = mcap_conn->mcap_chann;
	uint8_t buf[MCAP_DC_MTU];
	DBusMessage *reply;
	gboolean value;
	int fd, len;

	if (cond & (G_IO_ERR | G_IO_HUP | G_IO_NVAL)) {
		value = FALSE;
		goto end;
	}

	fd = g_io_channel_unix_get_fd(io_chan);
	len = read(fd, buf, sizeof(buf));

	if (len != MCAP_ECHO_LEN) {
		value = FALSE;
		goto end;
	}

	value = (memcmp(buf, edata->buf, len) == 0);

end:
	reply = g_dbus_create_reply(mcap_conn->msg, DBUS_TYPE_BOOLEAN, &value,
							DBUS_TYPE_INVALID);
	g_dbus_send_message(mcap_conn->conn, reply);
	g_source_remove(edata->tid);
	edata->tid = 0;
	g_free(edata->buf);
	edata->buf = NULL;

	if (!value)
		close_device_con(chan->dev, FALSE);
	else
		delete_echo_channel(chan);
	mcap_tmp_dc_data_unref(mcap_conn);

	return FALSE;
}

static gboolean echo_timeout(gpointer data)
{ DBG("");
	struct mcap_channel *chan = data;
	GIOChannel *io;
	int fd;

	error("Error: Echo request timeout");
	chan->edata->tid = 0;

	fd = mcap_mdl_get_fd(chan->mdl);
	if (fd < 0)
		return FALSE;

	io = g_io_channel_unix_new(fd);
	g_io_channel_shutdown(io, TRUE, NULL);

	return FALSE;
}

static void mcap_echo_connect_cb(struct mcap_mdl *mdl, GError *err,
								gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *mcap_conn =  data;
	struct mcap_echo_data *edata;
	GError *gerr = NULL;
	DBusMessage *reply;
	GIOChannel *io;
	int fd;

	if (err) {
		reply = g_dbus_create_error(mcap_conn->msg,
						ERROR_INTERFACE ".HealthError",
						"%s", err->message);
		g_dbus_send_message(mcap_conn->conn, reply);

	
		if (!mcap_mdl_abort(mcap_conn->mcap_chann->mdl,
					abort_echo_channel_cb,
					mcap_channel_ref(mcap_conn->mcap_chann),
					(GDestroyNotify) mcap_channel_unref,
					&gerr)) {
			error("%s", gerr->message);
			g_error_free(gerr);
			mcap_channel_unref(mcap_conn->mcap_chann);
		}
		return;
	}

	fd = mcap_mdl_get_fd(mcap_conn->mcap_chann->mdl);
	if (fd < 0) {
		reply = g_dbus_create_error(mcap_conn->msg,
						ERROR_INTERFACE ".HealthError",
						"Can't write in echo channel");
		g_dbus_send_message(mcap_conn->conn, reply);
		delete_echo_channel(mcap_conn->mcap_chann);
		return;
	}

	edata = mcap_conn->mcap_chann->edata;
	edata->buf = generate_echo_packet();
	send_echo_data(fd, edata->buf, MCAP_ECHO_LEN);

	io = g_io_channel_unix_new(fd);
	g_io_add_watch(io, G_IO_ERR | G_IO_HUP | G_IO_NVAL | G_IO_IN,
			check_echo, mcap_tmp_dc_data_ref(mcap_conn));

	edata->tid  = g_timeout_add_seconds_full(G_PRIORITY_DEFAULT,
					ECHO_TIMEOUT, echo_timeout,
					mcap_channel_ref(mcap_conn->mcap_chann),
					(GDestroyNotify) mcap_channel_unref);

	g_io_channel_unref(io);
}
*/
static void delete_mdl_cb(GError *err, gpointer data)
{ DBG("");
	if (err)
		error("Deleting error: %s", err->message);
}

static void abort_and_del_mdl_cb(GError *err, gpointer data)
{ DBG("");
	struct mcap_mdl *mdl = data;
	GError *gerr = NULL;

	if (err) {
		error("%s", err->message);
		if (err->code == MCAP_INVALID_MDL) {
			/* MDL is removed from MCAP so we don't */
			/* need to delete it. */
			return;
		}
	}

	if (!mcap_delete_mdl(mdl, delete_mdl_cb, NULL, NULL, &gerr)) {
		error("%s", gerr->message);
		g_error_free(gerr);
	}
}

static void abort_mdl_connection_cb(GError *err, gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *mcap_conn = data;
	struct mcap_channel *mcap_chann = mcap_conn->mcap_chann;

	if (err != NULL)
		error("Aborting error: %s", err->message);

	/* Connection operation has failed but we have to */
	/* notify the channel created at MCAP level */
	if (mcap_chann->mdep != MCAP_MDEP_ECHO)
		g_dbus_emit_signal(mcap_conn->conn,
					device_get_path(mcap_chann->dev->dev),
					MCAP_DEVICE,
					"ChannelConnected",
					DBUS_TYPE_OBJECT_PATH, &mcap_chann->path,
					DBUS_TYPE_INVALID);
}

static void mcap_mdl_conn_cb(struct mcap_mdl *mdl, GError *err, gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *mcap_conn =  data;
	struct mcap_channel *mcap_chann = mcap_conn->mcap_chann;
	struct mcap_device *dev = mcap_chann->dev;
	DBusMessage *reply;
	GError *gerr = NULL;

	if (err) {
		error("%s", err->message);
		reply = g_dbus_create_reply(mcap_conn->msg,
					DBUS_TYPE_OBJECT_PATH, &mcap_chann->path,
					DBUS_TYPE_INVALID);
		g_dbus_send_message(mcap_conn->conn, reply);

		/* Send abort request because remote side */
		/* is now in PENDING state */
		if (!mcap_mdl_abort(mcap_chann->mdl, abort_mdl_connection_cb,
					mcap_tmp_dc_data_ref(mcap_conn),
					mcap_tmp_dc_data_destroy, &gerr)) {
			mcap_tmp_dc_data_unref(mcap_conn);
			error("%s", gerr->message);
			g_error_free(gerr);
		}
		return;
	}

	reply = g_dbus_create_reply(mcap_conn->msg,
					DBUS_TYPE_OBJECT_PATH, &mcap_chann->path,
					DBUS_TYPE_INVALID);
	g_dbus_send_message(mcap_conn->conn, reply);

	DBG("signal path = %s %s", device_get_path(mcap_chann->dev->dev), MCAP_DEVICE);
	g_dbus_emit_signal(mcap_conn->conn,
					device_get_path(mcap_chann->dev->dev),
					MCAP_DEVICE,
					"ChannelConnected",
					DBUS_TYPE_OBJECT_PATH, &mcap_chann->path,
					DBUS_TYPE_INVALID);

	if (!check_channel_conf(mcap_chann)) {
		close_mdl(mcap_chann);
		return;
	}

	if (dev->fr)
		return;

	dev->fr = mcap_channel_ref(mcap_chann);

	emit_property_changed(dev->conn, device_get_path(dev->dev),
					MCAP_DEVICE, "MainChannel",
					DBUS_TYPE_OBJECT_PATH, &dev->fr->path);
}


static void device_mcl_created_cb(gpointer user_data, GError *err)
{ DBG("");
	struct mcap_create_dc *data = user_data;
	DBusMessage *reply;
	GError *gerr = NULL;
	gboolean ret = TRUE;
	
	reply = g_dbus_create_reply(data->msg,
					DBUS_TYPE_BOOLEAN, &ret,
					DBUS_TYPE_INVALID);
	g_dbus_send_message(data->conn, reply);
	DBG("out");

	device_set_temporary(data->dev->dev, FALSE);

}




void create_mcl_cb(struct mcap_mcl *mcl, GError *err, gpointer data);

void destroy_con_mcl_data(gpointer data);


#define MCAP_CCSPM 0x1001

void device_create_mcl_cb(struct mcap_mcl *mcl, GError *err, gpointer data)
{
		DBG("");
	struct mcap_create_mcl_data *mcl_data  = data;
	struct mcap_device *device = mcl_data->dev;
	GError *gerr = NULL;
	DBusMessage *reply;
	gboolean ret = TRUE;
	
	if (err) {
		reply = g_dbus_create_error(mcl_data->msg, ERROR_INTERFACE ".HealthError",
							"%s", err->message);
		g_error_free(err);
		return;
	} else {
		if (!device->mcl_conn)
			device->mcl = mcap_mcl_ref(mcl);
		device->mcl_conn = TRUE;
		mcap_set_mcl_cb(device, &gerr);
		reply = g_dbus_create_reply(mcl_data->msg,
					DBUS_TYPE_BOOLEAN, &ret,
					DBUS_TYPE_INVALID);
		}
	g_dbus_send_message(mcl_data->conn, reply);
	g_free(mcl_data);
	if (gerr)
		g_error_free(gerr);
	

	device_set_temporary(device->dev, FALSE);
}


static DBusMessage *device_create_mcl_channel(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ 
	DBG("");
	struct mcap_device *device = user_data;
	struct mcap_application *app;
	struct mcap_create_mcl_data *data;
	char *app_path, *conf;
	DBusMessage *reply;
	GError *err = NULL;
	uint8_t config;
	GSList *l;
	bdaddr_t dst, src;
	
	device_get_address(device->dev, &dst,NULL);
	adapter_get_address(device_get_adapter(device->dev), &src);

	struct get_mdep_data *mdep_data;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_STRING, &app_path,
							DBUS_TYPE_INVALID)){
		DBG("app_path = :%s:", app_path);
		return btd_error_invalid_args(msg);
	}
	DBG("fff");
	l = g_slist_find_custom(applications, app_path, cmp_app);
	if (!l)
		return btd_error_invalid_args(msg);

	app = l->data;


	config = MCAP_NO_PREFERENCE_DC;

	data = g_new0(struct mcap_create_mcl_data, 1);
	data->dev = mcap_device_ref(device);
	
	data->config = config;
	data->app = mcap_application_ref(app);
	data->msg = dbus_message_ref(msg);
	data->conn = dbus_connection_ref(conn);
	data->cb = NULL;
	data->mdep = 0x10;

	if (mcap_create_mcl(device->mcap_adapter->mi, &dst, MCAP_CCSPM,
							device_create_mcl_cb, data,
							destroy_con_mcl_data, &err)) {
		return NULL;
	}
			g_free(data);
			DBG("Call mcap_create_mcl ERROR");
		

	/*if (mcap_app_establish_mcl(data->dev, device_mcl_created_cb,
					data, destroy_create_dc_data, &err))
		return NULL;
	*/
	DBG("fter mcap_get_mdep");
	
	reply = g_dbus_create_error(msg, ERROR_INTERFACE ".HealthError",
							"%s", err->message);
	g_error_free(err);
	//mcap_create_data_unref(data);
	return reply;
}

static void device_create_mdl_cb(struct mcap_mdl *mdl, uint8_t conf,
						GError *err, gpointer data)
{ DBG("");
	struct mcap_create_dc *user_data = data;
	struct mcap_tmp_dc_data *mcap_conn;
	struct mcap_channel *mcap_chan;
	GError *gerr = NULL;
	DBusMessage *reply;

	if (err) {
		reply = g_dbus_create_error(user_data->msg,
					ERROR_INTERFACE ".HealthError",
					"%s", err->message);
		g_dbus_send_message(user_data->conn, reply);
		return;
	}


	mcap_chan = create_channel(user_data->dev, conf, mdl,
							mcap_mdl_get_mdlid(mdl),
							user_data->app, &gerr);
	if (!mcap_chan)
		goto fail;

	mcap_conn = g_new0(struct mcap_tmp_dc_data, 1);
	mcap_conn->msg = dbus_message_ref(user_data->msg);
	mcap_conn->conn = dbus_connection_ref(user_data->conn);
	mcap_conn->mcap_chann = mcap_chan;
	mcap_conn->cb = user_data->cb;
	mcap_chan->mdep = user_data->mdep;
		uint8_t mode;
	
		if (mcap_chan->config == MCAP_RELIABLE_DC)
			mode = L2CAP_MODE_ERTM;
		else
			mode = L2CAP_MODE_STREAMING;
	
		if (mcap_connect_mdl(mcap_chan->mdl, mode, 0x1003, mcap_conn->cb,
						mcap_tmp_dc_data_ref(mcap_conn),
						mcap_tmp_dc_data_destroy, &gerr))
			return;
	
		mcap_tmp_dc_data_unref(mcap_conn);
		mcap_conn->cb(mcap_chan->mdl, err, mcap_conn);
		g_error_free(gerr);
		gerr = NULL;
	return;

fail:
	reply = g_dbus_create_error(user_data->msg,
						ERROR_INTERFACE ".HealthError",
						"%s", gerr->message);
	g_dbus_send_message(user_data->conn, reply);
	g_error_free(gerr);

	/* Send abort request because remote side is now in PENDING */
	/* state. Then we have to delete it because we couldn't */
	/* register the HealthChannel interface */
	if (!mcap_mdl_abort(mdl, abort_and_del_mdl_cb, mcap_mdl_ref(mdl), NULL,
								&gerr)) {
		error("%s", gerr->message);
		g_error_free(gerr);
		mcap_mdl_unref(mdl);
	}
	
	
}

static DBusMessage *device_create_mdl_channel(DBusConnection *conn,
						DBusMessage *msg, void *user_data)
{
	DBG("");
	struct mcap_device *device = user_data;
	DBusMessage *reply;
	char *app_path;
	GError *err = NULL;

	struct mcap_application *app;
	struct mcap_create_dc *data;

	GSList *l;
	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_OBJECT_PATH, &app_path,
								DBUS_TYPE_INVALID))
			return btd_error_invalid_args(msg);
	DBG("app_path=  %s\n", app_path);
	l = g_slist_find_custom(applications, app_path, cmp_app);
	if (!l)
		return btd_error_invalid_args(msg);

	app = l->data;
	
	data = g_new0(struct mcap_create_dc, 1);
		data->dev = mcap_device_ref(device);
		data->config = MCAP_RELIABLE_DC;
		data->app = mcap_application_ref(app);
		data->msg = dbus_message_ref(msg);
		data->conn = dbus_connection_ref(conn);
		data->cb = mcap_mdl_conn_cb;

	if (!device->mcl_conn){
		reply = g_dbus_create_error(msg,
						ERROR_INTERFACE ".HealthError",
						"mcl not connected %s", err->message);
		return reply;
	}
	
	if (mcap_create_mdl(data->dev->mcl, 0x10, MCAP_RELIABLE_DC,
						device_create_mdl_cb, data,
						destroy_create_dc_data, &err)){
		return NULL;
		}
		
	reply = g_dbus_create_error(msg, ERROR_INTERFACE ".HealthError",
							"%s", err->message);
	g_error_free(err);
	mcap_create_data_unref(data);
	return reply;				
}


static void mcap_mdl_delete_cb(GError *err, gpointer data)
{ DBG("");
	struct mcap_tmp_dc_data *del_data = data;
	DBusMessage *reply;
	char *path;

	if (err && err->code != MCAP_INVALID_MDL) { DBG("");
		reply = g_dbus_create_error(del_data->msg,
						ERROR_INTERFACE ".HealthError",
						"%s", err->message);
		g_dbus_send_message(del_data->conn, reply);
		return;
	}

	path = g_strdup(del_data->mcap_chann->path);
	g_dbus_unregister_interface(del_data->conn, path, MCAP_CHANNEL);
	g_free(path);

	reply = g_dbus_create_reply(del_data->msg, DBUS_TYPE_INVALID);
	g_dbus_send_message(del_data->conn, reply);
}

static void mcap_continue_del_cb(gpointer user_data, GError *err)
{ DBG("");
	struct mcap_tmp_dc_data *del_data = user_data;
	GError *gerr = NULL;
	DBusMessage *reply;

	if (err) {
		reply = g_dbus_create_error(del_data->msg,
					ERROR_INTERFACE ".HealthError",
					"%s", err->message);
		g_dbus_send_message(del_data->conn, reply);
		return;
	}

	if (mcap_delete_mdl(del_data->mcap_chann->mdl, mcap_mdl_delete_cb,
						mcap_tmp_dc_data_ref(del_data),
						mcap_tmp_dc_data_destroy, &gerr))
			return;

	reply = g_dbus_create_error(del_data->msg,
						ERROR_INTERFACE ".HealthError",
						"%s", gerr->message);
	mcap_tmp_dc_data_unref(del_data);
	g_error_free(gerr);
	g_dbus_send_message(del_data->conn, reply);
}

static DBusMessage *device_destroy_channel(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	struct mcap_device *device = user_data;
	struct mcap_tmp_dc_data *del_data;
	struct mcap_channel *mcap_chan;
	DBusMessage *reply;
	GError *err = NULL;
	char *path;
	GSList *l;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_OBJECT_PATH, &path,
							DBUS_TYPE_INVALID)){
		return btd_error_invalid_args(msg);
	}

	l = g_slist_find_custom(device->channels, path, cmp_chan_path);
	if (!l)
		return btd_error_invalid_args(msg);

	mcap_chan = l->data;
	del_data = g_new0(struct mcap_tmp_dc_data, 1);
	del_data->msg = dbus_message_ref(msg);
	del_data->conn = dbus_connection_ref(conn);
	del_data->mcap_chann = mcap_channel_ref(mcap_chan);

	if (device->mcl_conn) {
		if (mcap_delete_mdl(mcap_chan->mdl, mcap_mdl_delete_cb,
						mcap_tmp_dc_data_ref(del_data),
						mcap_tmp_dc_data_destroy, &err))
			return NULL;
		goto fail;
	}

	if (mcap_establish_mcl(device, mcap_continue_del_cb,
						mcap_tmp_dc_data_ref(del_data),
						mcap_tmp_dc_data_destroy, &err))
		return NULL;

fail:
	reply = g_dbus_create_error(msg, ERROR_INTERFACE ".HealthError",
							"%s", err->message);
	mcap_tmp_dc_data_unref(del_data);
	g_error_free(err);
	return reply;
}

// Call to shut-down the MDL channel as required by PTS MCAP test-TC_MCAP_CM_DIS_BV_04_C
static DBusMessage *shut_down_mdl_channel(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{
	DBG("");
	struct mcap_device *device = user_data;
	struct mcap_tmp_dc_data *del_data;
	struct mcap_channel *mcap_chan;
	DBusMessage *reply;
	GError *err = NULL;
	char *path;
	GSList *l;

	if (!dbus_message_get_args(msg, NULL, DBUS_TYPE_OBJECT_PATH, &path,
							DBUS_TYPE_INVALID)){
		return btd_error_invalid_args(msg);
	}

	l = g_slist_find_custom(device->channels, path, cmp_chan_path);
	if (!l)
		return btd_error_invalid_args(msg);

	mcap_chan = l->data;
	del_data = g_new0(struct mcap_tmp_dc_data, 1);
	del_data->msg = dbus_message_ref(msg);
	del_data->conn = dbus_connection_ref(conn);
	del_data->mcap_chann = mcap_channel_ref(mcap_chan);

	if (device->mcl_conn) {
		if ( mcap_mdl_shutdown_for_pts_blueti(mcap_chan->mdl) )
			return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
		goto fail;
	}

fail:
	reply = g_dbus_create_error(msg, ERROR_INTERFACE ".HealthError",
							"%s", err->message);
	mcap_tmp_dc_data_unref(del_data);
	g_error_free(err);
	return reply;
}

static DBusMessage *device_get_properties(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{ DBG("");
	struct mcap_device *device = user_data;
	DBusMessageIter iter, dict;
	DBusMessage *reply;
	char *path;

	reply = dbus_message_new_method_return(msg);
	if (!reply)
		return NULL;

	dbus_message_iter_init_append(reply, &iter);

	dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
			DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
			DBUS_TYPE_STRING_AS_STRING DBUS_TYPE_VARIANT_AS_STRING
			DBUS_DICT_ENTRY_END_CHAR_AS_STRING, &dict);

	if (device->fr)
		path = g_strdup(device->fr->path);
	else
		path = g_strdup("");
	dict_append_entry(&dict, "MainChannel", DBUS_TYPE_OBJECT_PATH, &path);
	g_free(path);
	dbus_message_iter_close_container(&iter, &dict);

	return reply;
}

static void mcap_device_destroy(void *data)
{ DBG("");
	struct mcap_device *device = data;

	DBG("Unregistered interface %s on path %s", MCAP_DEVICE,
						device_get_path(device->dev));

	remove_channels(device);
	if (device->ndc) {
		mcap_channel_unref(device->ndc);
		device->ndc = NULL;
	}

	devices = g_slist_remove(devices, device);
	mcap_device_unref(device);
}

	

static DBusMessage *device_test(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{
	DBG("device test success");
	return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
}


static DBusMessage *device_unavailable(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{
	DBG("edvice test success");
	struct mcap_device *device = user_data;
	
	device->unavailable = !device->unavailable;
	return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
}


static DBusMessage *device_destroy_mcl_channel(DBusConnection *conn,
					DBusMessage *msg, void *user_data)
{
	struct mcap_device *device = user_data;

	DBG("destroy  mcl");

	close_device_con(device, TRUE);
	return g_dbus_create_reply(msg, DBUS_TYPE_INVALID);
}



static GDBusMethodTable mcap_device_methods[] = {
	{"CreateMclChannel",	"s",	"b",	device_create_mcl_channel,
						G_DBUS_METHOD_FLAG_ASYNC },
	{"DestroyChannel",	"o",	"",	device_destroy_channel,
							G_DBUS_METHOD_FLAG_ASYNC },
	{"DestroyMclChannel",	"",	"",	device_destroy_mcl_channel,
							G_DBUS_METHOD_FLAG_ASYNC },							
	{"CreateMdlChannel",	"o",	"o",	device_create_mdl_channel,
						G_DBUS_METHOD_FLAG_ASYNC },
	{"GetProperties",	"",	"a{sv}", device_get_properties},
	{"Test", "",	"",	device_test,
						G_DBUS_METHOD_FLAG_ASYNC },
	{"Unavailable", "",	"",	device_unavailable,
						G_DBUS_METHOD_FLAG_ASYNC },
	{"ShutDownMdl", "o",	"",	shut_down_mdl_channel,
						G_DBUS_METHOD_FLAG_ASYNC },
	{ NULL }
};

static GDBusSignalTable mcap_device_signals[] = {
	{"ChannelConnected",		"o"		},
	{"ChannelDeleted",		"o"		},
	{"PropertyChanged",		"sv"		},
	{ NULL }
};

static struct mcap_device *create_mcap_device(DBusConnection *conn,
						struct btd_device *device)
{ DBG("");
	struct btd_adapter *adapter = device_get_adapter(device);
	const gchar *path = device_get_path(device);
	struct mcap_device *dev;
	GSList *l;

	if (!device)
		return NULL;

	dev = g_new0(struct mcap_device, 1);
	dev->conn = dbus_connection_ref(conn);
	dev->dev = btd_device_ref(device);
	dev->unavailable = FALSE;
	
	mcap_device_ref(dev);

	l = g_slist_find_custom(adapters, adapter, cmp_adapter);
	if (!l)
		goto fail;

	dev->mcap_adapter = l->data;

	if (!g_dbus_register_interface(conn, path,
					MCAP_DEVICE,
					mcap_device_methods,
					mcap_device_signals, NULL,
					dev, mcap_device_destroy)) {
		error("D-Bus failed to register %s interface", MCAP_DEVICE);
		goto fail;
	}
	
	DBG("Registered interface %s on path %s", MCAP_DEVICE, path);
	return dev;

fail:
	mcap_device_unref(dev);
	return NULL;
}

int mcap_device_register(DBusConnection *conn, struct btd_device *device)
{ DBG("");
	struct mcap_device *hdev;
	GSList *l;
	DBG("");
	l = g_slist_find_custom(devices, device, cmp_device);
	if (l) {
		hdev = l->data;
		hdev->sdp_present = TRUE;
		return 0;
	}

	hdev = create_mcap_device(conn, device);
	if (!hdev)
		return -1;

	hdev->sdp_present = TRUE;

	devices = g_slist_prepend(devices, hdev);
	return 0;
}

void mcap_device_unregister(struct btd_device *device)
{ DBG("");
	struct mcap_device *mcap_dev;
	const char *path;
	GSList *l;

	l = g_slist_find_custom(devices, device, cmp_device);
	if (!l)
		return;

	mcap_dev = l->data;
	path = device_get_path(mcap_dev->dev);
	g_dbus_unregister_interface(mcap_dev->conn, path, MCAP_DEVICE);
}

int mcap_manager_start(DBusConnection *conn)
{ DBG("");
	DBG("Starting MCAP manager");

	if (!g_dbus_register_interface(conn, MANAGER_PATH,
					MCAP_MANAGER,
					mcap_manager_methods, NULL, NULL,
					NULL, manager_path_unregister)) {
		error("D-Bus failed to register %s interface", MCAP_MANAGER);
		return -1;
	}

	connection = dbus_connection_ref(conn);
	DBG("all and all");
	return 0;
}

void mcap_manager_stop(void)
{ DBG("");
	g_dbus_unregister_interface(connection, MANAGER_PATH, MCAP_MANAGER);

	dbus_connection_unref(connection);
	DBG("Stopped Health manager");
}

struct mcap_device *mcap_device_ref(struct mcap_device *mcap_dev)
{ DBG("");
	mcap_dev->ref++;

	DBG("mcap_device_ref(%p): ref=%d", mcap_dev, mcap_dev->ref);

	return mcap_dev;
}

void mcap_device_unref(struct mcap_device *mcap_dev)
{ DBG("");
	mcap_dev->ref--;

	DBG("mcap_device_unref(%p): ref=%d", mcap_dev, mcap_dev->ref);

	if (mcap_dev->ref > 0)
		return;

	free_mcap_device(mcap_dev);
}

