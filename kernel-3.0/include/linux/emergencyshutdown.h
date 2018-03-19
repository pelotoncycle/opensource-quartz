/*
 * Copyright (C) 2013 InnoComm Mobile Technology Corp.
 * Author: James Wu <james.wu@innocomm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _INCLUDE_LINUX_EMERGENCY_SHUTDOWN_H_
#define _INCLUDE_LINUX_EMERGENCY_SHUTDOWN_H_

#define EMERGENCY_SHUTDOWN_DRV_NAME	"emergency_shutdown"

struct emergency_shutdown_platform_data {
	int (*platform_init)(int *irq_no, unsigned long *irq_flags);
	void (*platform_shutdown)(void);
};

#endif /* _INCLUDE_LINUX_EMERGENCY_SHUTDOWN_H_ */
