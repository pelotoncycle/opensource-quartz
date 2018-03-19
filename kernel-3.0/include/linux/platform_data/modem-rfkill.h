/*
 * include/linux/platform_data/modem-rfkill.h
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


#ifndef __MDM_RFKILL_H__
#define __MDM_RFKILL_H__

#include <linux/types.h>
#include <linux/rfkill.h>

/**
 * struct modem_rfkill_platform_data - platform data for rfkill modem device.
 * @reset_pmu_req_gpio:           RESET_PMU line GPIO (negative if not N/A)
 * @baseband_reset_gpio:          BASEBAND_RESET line GPIO (negative if N/A)
 * @on_off_gpio:                  ON_OFF line GPIO (negative if N/A)
 * @reset2n_gpio:                 RESET2_N line GPIO (negative if N/A)
 * @pwrstate_gpio:                PWRSTATE line GPIO (negative if N/A)
 * @flashless:                    Modem is flashless (1/0)
 * @c2c:                          AP/CP unified memory over Chip-To_Chip (1/0)
 * @ipc:                          AP/CP IPC bus: "hsi", "usb", "hsic"
 * @ipc_init:                     Function pointer for IPC initialization
 * @io_pads_enable:               IO pads enabling function (NULL if N/A)
 * @io_pads_disable:              IO pads disabling function (NULL if N/A)
 * @io_wakeup_enable:             Enable IO wakeup capability (NULL if N/A)
 * @power_on_reset2_timeout_ms:   Timeout for reset2n assert on power on
 *							(negative if N/A)
 * @power_off_reset2_timeout_ms:  Timeout for reset2n assert on power off
 *							(negative if N/A)
 * @power_on_pwrstate_timeout_ms: Timeout for pwrstate assert on power on
 *							(negative if N/A)
 */

struct modem_rfkill_platform_data {
	int  reset_pmu_req_gpio;
	int  baseband_reset_gpio;
	int  on_off_gpio;
	int  reset2n_gpio;
	int  pwrstate_gpio;
	int  pwrstate_irq;
	int  active_gpio;
	int  wakeup_gpio;
	int  wakeup_irq;
	int  flashless;
	int  c2c;
	char *ipc;
	int  (*ipc_init) (void);
	int  (*io_pads_enable) (void);
	int  (*io_pads_disable)(void);
	int  (*io_wakeup_enable)(void);
	int  power_on_reset2_timeout_ms;
	int  power_off_reset2_timeout_ms;
	int  power_on_pwrstate_timeout_ms;
};

#endif /* __MDM_RFKILL_H__ */
