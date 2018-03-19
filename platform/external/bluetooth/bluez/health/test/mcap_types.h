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

#ifndef __MCAP_TYPES_H__
#define __MCAP_TYPES_H__

#define MCAP_UUID		"00001400-0000-1000-8000-00805F9B34FB"
#define MCAP_SOURCE_UUID		"00001401-0000-1000-8000-00805F9B34FB"
#define MCAP_SINK_UUID		"00001402-0000-1000-8000-00805F9B34FB"

#define MANAGER_PATH		"/org/bluez"

#define MCAP_MANAGER		"org.bluez.McapManager"
#define MCAP_DEVICE		"org.bluez.McapDevice"
#define MCAP_CHANNEL		"org.bluez.McapChannel"

#define MCAP_VERSION		0x0100

#define MCAP_SERVICE_NAME	"Bluez MCAP"
#define MCAP_SERVICE_DSC		"A Bluez health device profile implementation"
#define MCAP_SERVICE_PROVIDER	"Bluez"

#define MCAP_MDEP_ECHO		0x00
#define MCAP_MDEP_INITIAL	0x10
#define MCAP_MDEP_FINAL		0x7F

#define MCAP_ERROR		g_quark_from_static_string("mcap-error-quark")

#define MCAP_NO_PREFERENCE_DC	0x00
#define MCAP_RELIABLE_DC		0x01
#define MCAP_STREAMING_DC	0x02

#define MCAP_SINK_ROLE_AS_STRING		"sink"
#define MCAP_SOURCE_ROLE_AS_STRING	"source"

typedef enum {
	MCAP_SOURCE = 0x00,
	MCAP_SINK = 0x01
} HdpRole;

typedef enum {
	MCAP_DIC_PARSE_ERROR,
	MCAP_DIC_ENTRY_PARSE_ERROR,
	MCAP_CONNECTION_ERROR,
	MCAP_UNSPECIFIED_ERROR,
	MCAP_UNKNOWN_ERROR
} HdpError;

enum data_specs {
	DATA_EXCHANGE_SPEC_11073 = 0x01
};

struct mcap_application {
	DBusConnection		*conn;		/* For dbus watcher */
	char			*path;		/* The path of the application */
	uint16_t		data_type;	/* Data type handled for this application */
	gboolean		data_type_set;	/* Flag for dictionary parsing */
	uint8_t			role;		/* Role of this application */
	gboolean		role_set;	/* Flag for dictionary parsing */
	uint8_t			chan_type;	/* QoS preferred by source applications */
	gboolean		chan_type_set;	/* Flag for dictionary parsing */
	char			*description;	/* Options description for SDP record */
	uint8_t			id;		/* The identification is also the mdepid */
	char			*oname;		/* Name of the owner application */
	int			dbus_watcher;	/* Watch for clients disconnection */
	gint			ref;		/* Reference counter */
};

struct mcap_adapter {
	struct btd_adapter	*btd_adapter;	/* Bluetooth adapter */
	struct mcap_instance	*mi;		/* Mcap instance in */
	uint16_t		ccpsm;		/* Control channel psm */
	uint16_t		dcpsm;		/* Data channel psm */
	uint32_t		sdp_handler;	/* SDP record handler */
	uint32_t		record_state;	/* Service record state */
};

struct mcap_device {
	DBusConnection		*conn;		/* For name listener handling */
	struct btd_device	*dev;		/* Device reference */
	struct mcap_adapter	*mcap_adapter;	/* mcap_adapater */
	struct mcap_mcl		*mcl;		/* The mcap control channel */
	gboolean		mcl_conn;	/* Mcl status */
	gboolean		sdp_present;	/* Has an sdp record */
	GSList			*channels;	/* Data Channel list */
	struct mcap_channel	*ndc;		/* Data channel being negotiated */
	struct mcap_channel	*fr;		/* First reliable data channel */
	gint			ref;		/* Reference counting */
	gboolean unavailable;
};

struct mcap_echo_data;

struct mcap_channel {
	struct mcap_device	*dev;		/* Device where this channel belongs */
	struct mcap_application	*app;		/* Application */
	struct mcap_mdl		*mdl;		/* The data channel reference */
	char			*path;		/* The path of the channel */
	uint8_t			config;		/* Channel configuration */
	uint8_t			mdep;		/* Remote MDEP */
	uint16_t		mdlid;		/* Data channel Id */
	uint16_t		imtu;		/* Channel incoming MTU */
	uint16_t		omtu;		/* Channel outgoing MTU */
	struct mcap_echo_data	*edata;		/* private data used by echo channels */
	gint			ref;		/* Reference counter */
};

#endif /* __MCAP_TYPES_H__ */
