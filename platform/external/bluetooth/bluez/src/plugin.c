/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2004-2010  Marcel Holtmann <marcel@holtmann.org>
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

#include <errno.h>
#include <dlfcn.h>
#include <string.h>
#include <sys/stat.h>

#include <bluetooth/bluetooth.h>

#include <glib.h>

#include "plugin.h"
#include "log.h"
#include "hcid.h"
#include "btio.h"

static GSList *plugins = NULL;

struct bluetooth_plugin {
	void *handle;
	gboolean active;
	struct bluetooth_plugin_desc *desc;
};

static gint compare_priority(gconstpointer a, gconstpointer b)
{
	const struct bluetooth_plugin *plugin1 = a;
	const struct bluetooth_plugin *plugin2 = b;

	return plugin2->desc->priority - plugin1->desc->priority;
}

static gboolean add_plugin(void *handle, struct bluetooth_plugin_desc *desc)
{
	struct bluetooth_plugin *plugin;

	if (desc->init == NULL)
		return FALSE;

	if (g_str_equal(desc->version, VERSION) == FALSE) {
		error("Version mismatch for %s", desc->name);
		return FALSE;
	}

	DBG("Loading %s plugin", desc->name);

	plugin = g_try_new0(struct bluetooth_plugin, 1);
	if (plugin == NULL)
		return FALSE;

	plugin->handle = handle;
	plugin->active = FALSE;
	plugin->desc = desc;


	plugins = g_slist_insert_sorted(plugins, plugin, compare_priority);

	return TRUE;
}

static gboolean enable_plugin(const char *name, char **conf_disable,
					char **cli_enable, char **cli_disable)
{
	if (conf_disable) {
		for (; *conf_disable; conf_disable++)
			if (g_pattern_match_simple(*conf_disable, name))
				break;
		if (*conf_disable) {
			INFO("Excluding (conf) %s", name);
			return FALSE;
		}
	}

	if (cli_disable) {
		for (; *cli_disable; cli_disable++)
			if (g_pattern_match_simple(*cli_disable, name))
				break;
		if (*cli_disable) {
			INFO("Excluding (cli) %s", name);
			return FALSE;
		}
	}

	if (cli_enable) {
		for (; *cli_enable; cli_enable++)
			if (g_pattern_match_simple(*cli_enable, name))
				break;
		if (!*cli_enable) {
			INFO("Ignoring (cli) %s", name);
			return FALSE;
		}
	}

	return TRUE;
}

#include "builtin.h"

gboolean plugin_init(GKeyFile *config, const char *enable, const char *disable)
{
	GSList *list;
	GDir *dir;
	const gchar *file;
	char **conf_disabled, **cli_disabled, **cli_enabled;
	unsigned int i;

	/* Make a call to BtIO API so its symbols got resolved before the
	 * plugins are loaded. */
	bt_io_error_quark();

	if (config)
		conf_disabled = g_key_file_get_string_list(config, "General",
							"DisablePlugins",
							NULL, NULL);
	else
		conf_disabled = NULL;

	if (enable)
		cli_enabled = g_strsplit_set(enable, ", ", -1);
	else
		cli_enabled = NULL;

	if (disable)
		cli_disabled = g_strsplit_set(disable, ", ", -1);
	else
		cli_disabled = NULL;

	DBG("Loading builtin plugins");

	for (i = 0; __bluetooth_builtin[i]; i++) {
		if (!enable_plugin(__bluetooth_builtin[i]->name, conf_disabled,
						cli_enabled, cli_disabled))
			continue;

		add_plugin(NULL,  __bluetooth_builtin[i]);
	}

	if (strlen(PLUGINDIR) == 0)
		goto start;

	DBG("Loading plugins %s", PLUGINDIR);

	dir = g_dir_open(PLUGINDIR, 0, NULL);
	if (!dir)
		goto start;

	while ((file = g_dir_read_name(dir)) != NULL) {
		struct bluetooth_plugin_desc *desc;
		void *handle;
		gchar *filename;

		if (g_str_has_prefix(file, "lib") == TRUE ||
				g_str_has_suffix(file, ".so") == FALSE)
			continue;

		filename = g_build_filename(PLUGINDIR, file, NULL);

		handle = dlopen(filename, RTLD_NOW);
		if (handle == NULL) {
			error("Can't load plugin %s: %s", filename,
								dlerror());
			g_free(filename);
			continue;
		}

		g_free(filename);

		desc = dlsym(handle, "bluetooth_plugin_desc");
		if (desc == NULL) {
			error("Can't load plugin description: %s", dlerror());
			dlclose(handle);
			continue;
		}

		if (!enable_plugin(desc->name, conf_disabled,
						cli_enabled, cli_disabled)) {
			dlclose(handle);
			continue;
		}

		if (add_plugin(handle, desc) == FALSE)
			dlclose(handle);
	}

	g_dir_close(dir);

start:
	for (list = plugins; list; list = list->next) {
		struct bluetooth_plugin *plugin = list->data;

		if (plugin->desc->init() < 0) {
			error("Failed to init %s plugin", plugin->desc->name);
			continue;
		}

		plugin->active = TRUE;
	}

	g_strfreev(conf_disabled);
	g_strfreev(cli_enabled);
	g_strfreev(cli_disabled);

	return TRUE;
}

void plugin_cleanup(void)
{
	GSList *list;

	DBG("Cleanup plugins");

	for (list = plugins; list; list = list->next) {
		struct bluetooth_plugin *plugin = list->data;

		if (plugin->active == TRUE && plugin->desc->exit)
			plugin->desc->exit();

		if (plugin->handle != NULL)
			dlclose(plugin->handle);

		g_free(plugin);
	}

	g_slist_free(plugins);
}
