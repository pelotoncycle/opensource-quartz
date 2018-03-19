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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _BOARD_INNOCOMM_COMMON_H
#define _BOARD_INNOCOMM_COMMON_H

#include <icom/fastboot.h>

u32 innocomm_board_identify(void);
void innocomm_board_early_init_f(void);
void innocomm_board_init(void);
void innocomm_board_misc_init(void);
void innocomm_board_late_init(void);
void innocomm_board_preboot_os(void);
size_t innocomm_board_setup_linux_cmdline(char *cmdline);

void innocomm_board_detect_factory_tool(void);
int innocomm_board_is_factory_mode(void);

void innocomm_fastboot_board_init(struct usb_string *strings, struct usb_device_descriptor *desc);

#if defined(CONFIG_PANEL_DPI2LVDS) && defined(CONFIG_PANEL_DPI2LVDS_SETUP_CMDLINE)
size_t omap4_dpi2lvds_setup_linux_cmdline(char *cmdline);
#endif

#endif /* _BOARD_INNOCOMM_COMMON_H */
