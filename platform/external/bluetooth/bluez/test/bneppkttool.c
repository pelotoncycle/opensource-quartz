#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <syslog.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <string.h>


typedef struct {
        unsigned char b[6];
} bdaddr_t;

static	bdaddr_t src_bdaddr;
static	bdaddr_t dst_bdaddr;
char buf[2880] = {};

char filename[100];

int str2ba(const char *str, bdaddr_t *ba)
{
	int i;

	for (i = 0; i < 6; i++, str += 3)
		ba->b[i] = strtol(str, NULL, 16);
	
	return 0;
}

static int fill_test_tx_type_0_bv_20_c() {
	int i;
	int j;
	char *buf1 = buf;

	*buf1++ = 0x00;

	memcpy(buf1, &dst_bdaddr, sizeof(bdaddr_t));
	buf1+= sizeof(bdaddr_t);
	memcpy(buf1, &src_bdaddr, sizeof(bdaddr_t));
	buf1+= sizeof(bdaddr_t);
	*buf1++ = 0x08;	
	*buf1++ = 0x00;

	/* Ignored Data */	
	for (i = 0; i < 60; i++) {
		*buf1++ =   0xfe;
	}	
	/* Data*/	
	for (j = 0; j < 5; j++) {
		for (i = 0; i <= 0xff; i++) {
			*buf1++ = i;
		}
	}
	for (i = 0; i <= 0x9f; i++) {
		*buf1++ = i;
	}
	
	int size = (0x5eb);

	int fd = creat(filename, 777);
	if (fd < 0) {
		printf( "Open failed: %s %s (%d)\n", filename,	strerror(errno), errno);
		exit(1);
	}
	size = write(fd, buf, size);
	close(fd);
	printf("saved %s\n", filename);
	return size;	
}

static const char *main_help =
	"Bluetooth BNEP PTS PacketMaker\n"
	"Usage:\n"
	"\tbnep src dst packet_file_name\n";


int main(int argc, char *argv[]) {
	printf("bnep test packet generator\n");


	if (argc < 4) {
		printf("%s", main_help);
		return -1;
	}
	str2ba(argv[1], &src_bdaddr);
	str2ba(argv[2], &dst_bdaddr);
	sprintf(filename, "%s", argv[3]);
	printf("filename = %s\n", filename);
	fill_test_tx_type_0_bv_20_c();

	return 0;
}

