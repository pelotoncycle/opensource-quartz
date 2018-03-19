/*
 * (C) Copyright 2011
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * fastboot commands
 */
#include <common.h>
#include <command.h>

#include <icom/fastboot.h>
#include <icom/aboot.h>

#if defined(CONFIG_CMD_FASTBOOT)

DECLARE_GLOBAL_DATA_PTR;

int do_fastboot (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#ifdef CONFIG_USB_GADGET_FASTBOOT
	int ret;

	ret = fastboot_init();
	if (!ret) {
		aboot_set_state(ABOOT_STATE_BOOT_FASTBOOT);
		while (1) {
			ret = fastboot_poll();
			if (ret)
				break;
		}
		fastboot_shutdown();

		switch (ret) {
		case FASTBOOT_EXIT_REBOOT_BOOTLOADER:
			puts("<FASTBOOT> Rebooting to Bootloader ...\n");
			board_reboot("bootloader");
			break;
		case FASTBOOT_EXIT_REBOOT:
			puts("<FASTBOOT> Rebooting ...\n");
			board_reboot(NULL);
			break;
		case FASTBOOT_EXIT_SHUTDOWN:
			puts("<FASTBOOT> Shutting down ...\n");
			board_poweroff();
			break;
		case FASTBOOT_EXIT_CONTINUE:
		default:
			puts("<FASTBOOT> Continue the booting ...\n");
			break;
		}
	}

	return ret;
#else
	return 0;
#endif /* CONFIG_USB_GADGET_FASTBOOT */
}

U_BOOT_CMD(
	fastboot,	1,	0,	do_fastboot,
	"fastboot",
	"Android Fastboot Protocol"
);

#endif /* CONFIG_CMD_FASTBOOT */
