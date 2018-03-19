/*
 * (C) Copyright 2014
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 * Sinjhe Yu <sinjhe.yu@innocomm.com>
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

#ifndef __OMAP_PANEL_ICOM_DPI2LVDS_H
#define __OMAP_PANEL_ICOM_DPI2LVDS_H

#include <icom/omapdss.h>

/**
 * Power-on Sequence:
 *   1. Enable DPI mux configurations
 *   2. Power on LCM voltages
 *   3. RESET (if reset_gpio >= 0)
 *      3.a. RESET pin is INACTIVE
 *      3.b. Wait <power_on_reset_delay> ms
 *      3.c. RESET pin is ACTIVE
 *      3.d. Wait ~1500 us
 *      3.e. RESET pin is INACTIVE
 *   4. (OPTIONAL) Wait <power_on_dpi_delay> ms
 *   5. Enable DPI signals
 *   6. Wait <power_on_delay> ms
 *
 * Power-off Sequence:
 *   1. Disable DPI signals
 *   2. Disable DPI mux configurations
 *   3. (OPTIONAL) Wait <power_off_dpi_delay> ms
 *   4. RESET pin is low (if reset_gpio >= 0)
 *   5. Power off LCM voltages
 *   6. Wait <power_off_delay> ms
 */

/**
 * struct panel_icom_dpi2lvds_data - panel driver configuration data
 */
struct panel_icom_dpi2lvds_data {
	u16 power_on_reset_delay;	/* see above Power-on Sequence */
	u16 power_on_dpi_delay;		/* (OPTIONAL) see above Power-on Sequence */
	u16 power_on_delay;			/* see above Power-on Sequence */

	u16 power_off_dpi_delay;	/* (OPTIONAL) see above Power-off Sequence */
	u16 power_off_delay;		/* see above Power-off Sequence */

	int reset_high;				/* Polarity of LCM reset GPIO: 1 = Active HIGH, 0 = Active LOW */
	int (*is_regulator_enabled)(struct omap_dss_device *dssdev);

	void (*dpi_mux_pads)(bool enable);
};

int panel_dpi2lvds_init(void);
struct omap_dss_device *omap4_dpi2lvds_display_setup(
		const char *panel_name, int reset_gpio, int reset_high);

#endif /* __OMAP_PANEL_ICOM_DPI2LVDS_H */
