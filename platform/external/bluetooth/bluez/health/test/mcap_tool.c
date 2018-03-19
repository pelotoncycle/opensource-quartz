/* This File is ment to test the mcap with the PTS.
   it is a TI tool and is ment for that purpose only */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <pthread.h>

#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <syslog.h>
#include <signal.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>
#include <gdbus.h>
#include <stdint.h>
#include <adapter.h>
#include <device.h>
#include "../src/dbus-common.h"
#include "../src/manager.h"


#define DBG(fmt, arg...)  printf("MCAP_TOOL: %s: " fmt "\n" , __FUNCTION__ , ## arg)
#define ERR(fmt, arg...)  fprintf(stderr, "MCAP_TOOL: ERROR: %s: " fmt "\n" , __FUNCTION__ , ## arg)

#define DEFAULT_BDADDR "00:80:98:E7:2F:74"
#define DEFAULT_MSG "Hello Wrold"

#define DBUS_ORG_BLUEZ "org.bluez"
#define DBUS_MANAGER DBUS_ORG_BLUEZ".Manager"
#define DBUS_MCAP_MANAGER DBUS_ORG_BLUEZ".McapManager"
#define DBUS_MCAP_DEVICE DBUS_ORG_BLUEZ".McapDevice"
#define DBUS_MCAP_CHANNEL DBUS_ORG_BLUEZ".McapChannel"

struct mcap_test_app {
    pthread_t signal_th;
    char *app_path;
    char *adapter_path;
    char *dev_addr;
    char *dev_path;
    DBusConnection *conn;
    char *chan_path;
    bool sig_listen;
    bdaddr_t dst;
};


struct mcap_test_app *app = NULL;

static DBusConnection *init_dbus() {
    DBusConnection *conn = dbus_bus_get(DBUS_BUS_SYSTEM, NULL);
    if (!conn) {
	ERR("Can't get on system bus\n");
	return NULL;
    }

    DBG("dbus Connection established\n");
    return conn;
}


static bool get_default_adapter() {
    DBusMessage *msg, *reply;
    DBusError err;
    const char *reply_path;
    char *path;

    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, "/",
	    DBUS_MANAGER, "DefaultAdapter");

    if (!msg) {
	printf("Can't allocate new method call\n");
	return FALSE;
    }

    dbus_error_init(&err);
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);
    dbus_message_unref(msg);

    if (!reply) {
	ERR("Can't get default adapter\n");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }

    if (!dbus_message_get_args(reply, &err,
		DBUS_TYPE_OBJECT_PATH, &reply_path,
		DBUS_TYPE_INVALID)) { 
	ERR("Missing adapter path\n");
	return FALSE;
    }

    DBG("default adapter path = %s\n", reply_path);
    dbus_message_unref(reply);
    app->adapter_path = strdup(reply_path);
    dbus_connection_flush(app->conn);
    return TRUE;
}


/* Create a app in Bluez memory */
static bool create_app() {
    DBusMessage *msg, *reply;
    DBusError err;
    const char *reply_path;
    char *path;
    dbus_error_init(&err);

    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, "/org/bluez",
	    DBUS_MCAP_MANAGER, "CreateApplication");

    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);
    dbus_message_unref(msg);
    if (!reply) {
	ERR("Cant create new Application\n");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }
    if (!dbus_message_get_args(reply, &err,
		DBUS_TYPE_OBJECT_PATH, &reply_path,
		DBUS_TYPE_INVALID)) { 
	ERR("Missing app path");
	return FALSE;
    }

    app->app_path = strdup(reply_path);
    dbus_message_unref(reply);
    DBG("MCAP app path = %s\n", reply_path);

    dbus_connection_flush(app->conn);
    return TRUE;

}


static volatile sig_atomic_t __io_finished = 0;

static void callback(uint8_t type, uint16_t status,
	uint8_t *rsp, size_t size, void *udata)
{
    DBG("\t sdp search notify complete\n");
    __io_finished = 1;
}


void sdp_check() {

    uuid_t uuid;
    bdaddr_t src;

    __io_finished = 0;


    sdp_session_t *session;
    sdp_list_t *search, *attrids;
    uint32_t range = 0x0000ffff;



    bacpy(&src, BDADDR_ANY);
    session = sdp_connect(&src, &app->dst, 0);
    if (!session) {
	perror("Can't connect to SDP service");
	return;
    }

    sdp_set_notify(session, callback, NULL);

    sdp_uuid16_create(&uuid, PUBLIC_BROWSE_GROUP);

    search = sdp_list_append(NULL, &uuid);

    attrids = sdp_list_append(NULL, &range);

    //sdp_service_search_attr_async(session, search,
    //				SDP_ATTR_REQ_RANGE, attrids);

    sdp_service_search_async(session, search, 0xffff);

    sdp_list_free(attrids, NULL);

    sdp_list_free(search, NULL);

    while (!__io_finished)
	sdp_process(session);

    sdp_close(session);

}


static bool toggle_unavailable() {
    DBG("");
    DBusMessage *msg, *reply;
    DBusError err;
    const char *reply_path;
    char path[255] = {};

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "Unavailable");
    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);

    if (!reply) {
	ERR("Toggle rejected");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }

    dbus_message_unref(reply);

    dbus_connection_flush(app->conn);
    return TRUE;
}


bool test_device() {
    DBG("");
    DBusMessage *msg, *reply;
    DBusError err;
    const char *reply_path;


    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "Test");
    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);

    if (!reply) {
	ERR("Can't create channel\n");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }
    DBG("test success");
    dbus_message_unref(reply);

    dbus_connection_flush(app->conn);
    return TRUE;
}


static bool create_mcl() {
    DBusMessage *msg, *reply;
    DBusError err;
    const char *reply_path;

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "CreateMclChannel");
    dbus_message_append_args(msg, DBUS_TYPE_STRING, &app->app_path, 
	    DBUS_TYPE_INVALID);

    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);

    if (!reply) {
	ERR("Can't create channel");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }

    dbus_message_unref(reply);

    dbus_connection_flush(app->conn);
    return TRUE;
}

static void set_channel_path(char *path) {
    if (app->chan_path != NULL)
	g_free(app->chan_path);

    app->chan_path = strdup(path);
    DBG("new MDL channel path = %s\n", app->chan_path);

}

static bool create_mdl() {
    DBusMessage *msg, *reply;
    DBusError err;
    char *chan_path;
    char path[255] = {};

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "CreateMdlChannel");
    dbus_message_append_args (msg, DBUS_TYPE_OBJECT_PATH, &app->app_path, 
	    DBUS_TYPE_INVALID);

    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);

    if (!reply) {
	ERR(
		"Can't create MDL channel\n");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }
    if (!dbus_message_get_args(reply, &err,
		DBUS_TYPE_OBJECT_PATH, &chan_path,
		DBUS_TYPE_INVALID)) { 
	ERR("Missing Channel Path");
	return FALSE;
    }
    set_channel_path(chan_path);
    dbus_message_unref(reply);

    dbus_connection_flush(app->conn);
    return TRUE;
}

static bool disconnect_mcl() {

    DBusMessage *msg, *reply;
    DBusError err;

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "DestroyMclChannel");

    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }
    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);

    if (!reply) {
	ERR(
		"Can't destroy channel\n");
	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }

    dbus_message_unref(reply);

    dbus_connection_flush(app->conn);
    return TRUE;
}


static bool disconnect_mdl() {
    DBusMessage *msg, *reply;
    DBusError err;

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "DestroyChannel");

    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }

    dbus_message_append_args (msg,
	    DBUS_TYPE_OBJECT_PATH, &app->chan_path, DBUS_TYPE_INVALID);

    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);
    if (!reply) {
		ERR("Can't write on channel\n");
		if (dbus_error_is_set(&err)) {
		    ERR("%s\n", err.message);
		    dbus_error_free(&err);
		}
		return FALSE;
    }
    return TRUE;
}

static bool graceful_disconnect_mdl() {
	DBusMessage *msg, *reply;
    DBusError err;
    DBG("");
    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->dev_path,
	    DBUS_MCAP_DEVICE, "ShutDownMdl");

    if (!msg) {
		ERR("Can't allocate new method call\n");
		return FALSE;
    }

    dbus_message_append_args (msg,
	    DBUS_TYPE_OBJECT_PATH, &app->chan_path, DBUS_TYPE_INVALID);

    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);

    dbus_message_unref(msg);
    if (!reply) {
		ERR("Can't write on channel\n");
		if (dbus_error_is_set(&err)) {
			ERR("%s\n", err.message);
			dbus_error_free(&err);
		}
		return FALSE;
    }
	DBG("Dbus, ShutDownMdl, sent successfuly");
    return TRUE;
}

bool send_data_on_mdl() {
    int fd = -1;
    DBusMessage *msg, *reply;
    DBusError err;
    sdp_check();

    dbus_error_init(&err);
    msg = dbus_message_new_method_call(DBUS_ORG_BLUEZ, app->chan_path,
	    DBUS_MCAP_CHANNEL, "Acquire");

    if (!msg) {
	ERR("Can't allocate new method call\n");
	return FALSE;
    }


    reply = dbus_connection_send_with_reply_and_block(app->conn, msg, -1, &err);
    DBG("Reply");
    dbus_message_unref(msg);
    if (!reply) {
	ERR("Can't write on  channel\n");

	if (dbus_error_is_set(&err)) {
	    ERR("%s\n", err.message);
	    dbus_error_free(&err);
	}
	return FALSE;
    }
    DBG("Parse");
    if (!dbus_message_get_args(reply, &err,
		DBUS_TYPE_UNIX_FD, &fd,
		DBUS_TYPE_INVALID)) { 
	ERR("Cant extract fd");
	return FALSE;
    }

    write(fd, DEFAULT_MSG, strlen(DEFAULT_MSG));
    return TRUE;
}



void *get_signal(void *arg)
{
    DBusError err;
    DBusMessageIter iter;
    DBusMessage *msg, *reply;
    char *path;
    char *chan_path;

    char watcharg[255] ;

    sprintf(watcharg, "type='signal',interface='org.bluez.McapDevice',path=%s", app->dev_path);
    app->sig_listen = TRUE;

    dbus_bus_add_match(app->conn, watcharg, &err); 
    dbus_connection_flush(app->conn);

    if (dbus_error_is_set(&err)) { 
	DBG("Match Error (%s)\n", err.message);

    }
    while (app->sig_listen) {
	// non blocking read of the next available message
	dbus_connection_read_write(app->conn, 0);
	msg = dbus_connection_pop_message(app->conn);
	if (NULL == msg) { 
	    sleep(1);
	    continue;
	}

	// check if the message is a signal from the correct interface and with the correct name
	if (!dbus_message_is_signal(msg, DBUS_MCAP_DEVICE, "ChannelConnected")) 
	    continue;
	if ((!dbus_message_iter_init(msg, &iter)) || 
		(DBUS_TYPE_OBJECT_PATH != dbus_message_iter_get_arg_type(&iter))){
	    ERR( "signal has bad arguments!\n"); 
	    ERR("signal rejected");
	    continue;
	}

	dbus_message_iter_get_basic(&iter, &chan_path);	
	dbus_message_unref(msg);
	set_channel_path(chan_path);


    }
    return NULL;
}

static void bring_up_signal_thread(){

    pthread_create(&app->signal_th, NULL, get_signal, NULL);

}

static void usage(void)
{
    printf("mcap_tool - MCAP protocol testing tool\n"
	    "Usage:\n");
    printf("\tmcap_tool [bd_addr]\n");
}



static void set_dev_path(char *dev_bdaddr){
    int i, j;

    if (dev_bdaddr == NULL || (strlen(dev_bdaddr) != 17)) {
	ERR("Illegal BD Address given for remote PTS\n"); 
	DBG("Using default addr = %s\n",  DEFAULT_BDADDR);
	dev_bdaddr = DEFAULT_BDADDR;
    }

    str2ba(dev_bdaddr, &app->dst);
    app->dev_path = (char *)g_new0(char *, 255);

    sprintf(app->dev_path, "%s/dev", app->adapter_path); 
    j = 0;
    for (i = 0; i < 6; i++)
    {
	sprintf(app->dev_path, "%s_%c%c", app->dev_path, dev_bdaddr[j], dev_bdaddr[j+1]);
	j+=3;
    }
    DBG("dev_path = %s\n", app->dev_path);	

}

static bool init_test_app(char *dev_bdaddr){
    app = g_new0(struct mcap_test_app, 1);
    app->conn = init_dbus();

    if (app->conn == NULL)
	return FALSE;
    if (!get_default_adapter())
	return FALSE;
    set_dev_path(dev_bdaddr);

    return create_app();
}

static void  print_notes() {
    DBG("PTS NOTES");
    DBG("\t*  PIXIT selected values ");
	DBG("\t   TSPX_L2CAP_psm_control 1001");
	DBG("\t   TSPX_L2CAP_psm_data    1003");
	DBG("\t   TSPX_DC_max        	 1");
	DBG("\t   TSPX_MDEP_ID	         10");
	DBG("\t*  You should ethier be paired with the PTS,");
    DBG("\t   or start with a test that connectes to the IUT (like CE_BV_02_C)");
    DBG("\t   to insure the device driver for the PTS is loaded.");
	DBG("");
	DBG("\t* In order to pass the PTS test, TC_MCAP_CM_REC_BV_04_C - ");
    DBG("\t   When prompt to place the IUT out of range - type in the adb shell -");
    DBG("\t   hcitool cmd 3f 0x030a 01 01");
    DBG("\t   When prompt to bring the IUT back in range - type in the adb shell -");
    DBG("\t   hcitool cmd 3f 0x030a 00 01");
	
}

static void print_options() {
    DBG("OPTIONS:");
    DBG("\tc: create mcl");
    DBG("\td: create mdl");
    DBG("\tX: destroy mcl");
    DBG("\tx: destroy mdl");
    DBG("\ts: send data on mdl");
    DBG("\tu: toggle resource");
    DBG("\tr: run sdp test");
    DBG("\tq: quit");
    DBG("\th: print this message");
    DBG("\tn: print notes");
    DBG("\tg: graceful MDL disconnect");
    DBG("\n");
}

int main(int argc ,char *argv[])
{	
    int opt, sk;
    char *dev_bdaddr = NULL;

    if (argc == 1) {
	DBG("No BD Address given for remote PTS\n\tyou will not be able to initiate a connection"); 
    } else {
	dev_bdaddr = argv[1];
    }

    if (!init_test_app(dev_bdaddr)) {
	ERR("intialization failed");
	exit(0);
    }

    char op ='1';
    bring_up_signal_thread();
    print_options();

    while (op != 'q') {
	scanf("%c", &op);
	switch(op) {
	    case 't':
		test_device();
		break;
	    case 'c':
		create_mcl();
		break;
	    case 'd':
		create_mdl();
		break;
	    case 's':
		send_data_on_mdl();
		break;
	    case 'x':
		disconnect_mdl();
		break;
	    case 'X':
		disconnect_mcl();
		break;
	    case 'u':
		toggle_unavailable();
		break;
	    case 'r':
		sdp_check();
		break;

	    case 'h':
		print_options();
		break;
	    case 'n':
		print_notes();
		break;
		case 'g':
		graceful_disconnect_mdl();
		break;


	    default:
		break;
	}
    }

    app->sig_listen = FALSE;
    DBG("all done\n");
    return 0;
}
