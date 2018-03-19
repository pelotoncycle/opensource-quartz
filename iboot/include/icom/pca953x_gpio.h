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

#ifndef __PCA953X_GPIO_H_
#define __PCA953X_GPIO_H_

/*----------------------------------------------------------------------*/

#define PCA6416_ADDR1	0x20
#define PCA6416_ADDR2	0x21

/*----------------------------------------------------------------------*/

#define PCA953X_TYPE	0x1000
#define PCA957X_TYPE	0x2000

/*----------------------------------------------------------------------*/

struct pca953x_gpio_chip {
	uint8_t	addr;
	uint8_t	ngpio;
	int	chip_type;

	/* initial polarity inversion setting */
	uint16_t	invert;

#if 0
	void	*context;	/* param to setup/teardown */
	int		(*setup)(unsigned gpio, unsigned ngpio, void *context);
	int		(*teardown)(unsigned gpio, unsigned ngpio, void *context);
#endif

	/* Internal */
	uint8_t	chip_valid;
	uint16_t reg_output;
	uint16_t reg_direction;
};

/*----------------------------------------------------------------------*/

int pca953x_gpio_direction_input(uint8_t chip_addr, unsigned gpio_offset);
int pca953x_gpio_direction_output(uint8_t chip_addr, unsigned gpio_offset, int val);
int pca953x_gpio_get_value(uint8_t chip_addr, unsigned gpio_offset);
int pca953x_gpio_set_value(uint8_t chip_addr, unsigned gpio_offset, int val);

/*----------------------------------------------------------------------*/

void pca953x_gpio_init(struct pca953x_gpio_chip *gpios, int num);
void pca953x_gpio_shutdown(void);

#endif /* __PCA953X_GPIO_H_ */
