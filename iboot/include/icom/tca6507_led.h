/*
 * (C) Copyright 2012
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

#ifndef _TCA6507_LED_H
#define _TCA6507_LED_H

/*----------------------------------------------------------------------*/

#define TCA6507			"tca6507"
#define TCA6507_ADDR	0x45

/*----------------------------------------------------------------------*/

struct tca6507_platform_data {
	void (*platform_init)(void);
	void (*platform_enable)(void);
	void (*platform_disable)(void);
};

/*----------------------------------------------------------------------*/

/**
 * Inputs:
 *   brightness
 *     - The brightness of the specific LED. Do your best here.
 *     - If you can only do on or off, 0 is off, 255 is on.
 *   flashOnMS
 *     - the number of milliseconds to turn the light on.
 *   flashOffMS
 *     - the number of milliseconds to turn the light off.
 */
void tca6507_set_light(int port, int brightness, int flashOnMS, int flashOffMS);

int tca6507_enable_lights(void);

/*----------------------------------------------------------------------*/

void tca6507_led_init(struct tca6507_platform_data *pdata);
void tca6507_led_shutdown(void);

#endif /* _TCA6507_LED_H */
