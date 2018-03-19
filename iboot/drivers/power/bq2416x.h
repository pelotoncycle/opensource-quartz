/*
 * (C) Copyright 2013
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

#ifndef _BQ2416X_H
#define _BQ2416X_H

/*----------------------------------------------------------------------*/

struct bq2416x_charger_profile {
	int				battery_voltage;
	unsigned int	input_current;
	unsigned int	charge_current;
	int				use_half_charge_current;
	int				extra_increase_voltage;
	int				extra_decrease_voltage;
};

/*----------------------------------------------------------------------*/

void bq2416x_init(void);
void bq2416x_shutdown(void);

/*----------------------------------------------------------------------*/

#endif /* _BQ2416X_H */
