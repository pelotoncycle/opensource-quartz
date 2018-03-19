/*
 * NXP's 4, 8 and 16 bit I2C gpio expanders
 *
 * (C) Copyright 2013
 * InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
 *
 * Derived from u-boot/drivers/gpio/pca954x.c and linux/drivers/gpio/pca953x.c.
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

#include <common.h>
#include <exports.h>
#include <errno.h>
#include <i2c.h>
#include <asm/gpio.h>

#include <icom/pca953x_gpio.h>

/*----------------------------------------------------------------------*/

#ifdef CONFIG_PCA953X_GPIO_DEBUG
#define PCA953X_DPRINT(fmt, args...) \
	do {printf("[pca953x] " fmt, ##args);} while (0)
#define PCA953X_DPUTS(fmt) \
	do {puts("[pca953x] " fmt);} while (0)
#else /* !CONFIG_PCA953X_GPIO_DEBUG */
#define PCA953X_DPRINT(fmt, args...) \
	do {} while (0)
#define PCA953X_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_PCA953X_GPIO_DEBUG */

#ifdef CONFIG_PCA953X_GPIO_VERBOSE_DEBUG
#define PCA953X_VPRINT(fmt, args...) \
	do {printf("[pca953x] " fmt, ##args);} while (0)
#define PCA953X_VPUTS(fmt) \
	do {puts("[pca953x] " fmt);} while (0)
#else /* !CONFIG_PCA953X_GPIO_VERBOSE_DEBUG */
#define PCA953X_VPRINT(fmt, args...) \
	do {} while (0)
#define PCA953X_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_PCA953X_GPIO_VERBOSE_DEBUG */

#define PCA953X_PRINT(fmt, args...) \
	do {printf("pca953x: " fmt, ##args);} while (0)
#define PCA953X_PUTS(fmt) \
	do {puts("pca953x: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)

/*----------------------------------------------------------------------*/

#define PCA953X_INPUT		0
#define PCA953X_OUTPUT		1
#define PCA953X_INVERT		2
#define PCA953X_DIRECTION	3

#define PCA957X_IN		0
#define PCA957X_INVRT		1
#define PCA957X_BKEN		2
#define PCA957X_PUPD		3
#define PCA957X_CFG		4
#define PCA957X_OUT		5
#define PCA957X_MSK		6
#define PCA957X_INTS		7

/*----------------------------------------------------------------------*/

#if 0
static const struct i2c_device_id pca953x_id[] = {
	{ "pca9534", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9535", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9536", 4  | PCA953X_TYPE, },
	{ "pca9537", 4  | PCA953X_TYPE | PCA_INT, },
	{ "pca9538", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9539", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9554", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca9555", 16 | PCA953X_TYPE | PCA_INT, },
	{ "pca9556", 8  | PCA953X_TYPE, },
	{ "pca9557", 8  | PCA953X_TYPE, },
	{ "pca9574", 8  | PCA957X_TYPE | PCA_INT, },
	{ "pca9575", 16 | PCA957X_TYPE | PCA_INT, },

	{ "max7310", 8  | PCA953X_TYPE, },
	{ "max7312", 16 | PCA953X_TYPE | PCA_INT, },
	{ "max7313", 16 | PCA953X_TYPE | PCA_INT, },
	{ "max7315", 8  | PCA953X_TYPE | PCA_INT, },
	{ "pca6107", 8  | PCA953X_TYPE | PCA_INT, },
	{ "tca6408", 8  | PCA953X_TYPE | PCA_INT, },
	{ "tca6416", 16 | PCA953X_TYPE | PCA_INT, },
	/* NYET:  { "tca6424", 24, }, */
	{ }
};
#endif

/*----------------------------------------------------------------------*/

static struct pca953x_gpio_chip *pca953x_chips __attribute__ ((section (".data"))) = NULL;
static int num_pca953x_chips __attribute__ ((section (".data"))) = 0;

/*----------------------------------------------------------------------*/

#ifdef CONFIG_I2C_MULTI_BUS
static unsigned int saved_i2c_bus __attribute__ ((section (".data"))) = 0;

/* NOTE: These two functions MUST be always_inline to avoid code growth! */
static inline void pca953x_activate_i2c_bus(void) __attribute__((always_inline));
static inline void pca953x_activate_i2c_bus(void)
{
	saved_i2c_bus = i2c_get_bus_num();
	if (saved_i2c_bus != CONFIG_PCA953X_GPIO_I2C_BUS)
		i2c_set_bus_num(CONFIG_PCA953X_GPIO_I2C_BUS);
}

static inline void pca953x_inactivate_i2c_bus(void) __attribute__((always_inline));
static inline void pca953x_inactivate_i2c_bus(void)
{
	if (saved_i2c_bus != CONFIG_PCA953X_GPIO_I2C_BUS)
		i2c_set_bus_num(saved_i2c_bus);
}
#else
#define pca953x_activate_i2c_bus()
#define pca953x_inactivate_i2c_bus()
#endif /* CONFIG_I2C_MULTI_BUS */

/*----------------------------------------------------------------------*/

static struct pca953x_gpio_chip *get_chip(uint8_t addr, uint8_t	gpio)
{
	if (pca953x_chips) {
		struct pca953x_gpio_chip *chips = pca953x_chips;
		struct pca953x_gpio_chip *chip;
		int i;

		for (i = 0 ; i < num_pca953x_chips ; i++) {
			chip = chips++;
			if (chip->chip_valid && chip->addr == addr) {
				if (gpio < chip->ngpio) {
					return chip;
				} else {
					PCA953X_PRINT("0x%02x: bad gpio %d\n", addr, gpio);
					return NULL;
				}
			}
		}
	}

	return NULL;
}

/*----------------------------------------------------------------------*/

static int pca953x_write_reg(struct pca953x_gpio_chip *chip, uint reg, uint16_t val)
{
	int ret = 0;
    u8 buf[2];
	int retries = 3;

_retry:
	buf[0] = val & 0xff;
	buf[1] = (val & 0xff00) >> 8;

	if (chip->ngpio <= 8) {
		PCA953X_VPRINT("write(0x%02x): [0x%02x]=0x%02x\n", chip->addr, reg, buf[0]);
		ret = i2c_write(chip->addr, reg, 1, buf, 1);
		if (ret) {
			if (--retries > 0) {
				PCA953X_DPRINT("%d: write(0x%02x): [0x%02x]=0x%02x err %d\n", retries, chip->addr, reg, buf[0], ret);
				mdelay(10);
				goto _retry;
			} else {
				PCA953X_PRINT("write(0x%02x): [0x%02x]=0x%02x err %d\n", chip->addr, reg, buf[0], ret);
			}
		}
	} else {
		switch (chip->chip_type) {
		case PCA953X_TYPE:
			PCA953X_VPRINT("write(0x%02x): [0x%02x]=0x%04x\n", chip->addr, reg << 1, val);
			ret = i2c_write(chip->addr, reg << 1, 1, buf, 2);
			if (ret) {
				if (--retries > 0) {
					PCA953X_DPRINT("%d: write(0x%02x): [0x%02x]=0x%04x err %d\n", retries, chip->addr, reg << 1, val, ret);
					mdelay(10);
					goto _retry;
				} else {
					PCA953X_PRINT("write(0x%02x): [0x%02x]=0x%04x err %d\n", chip->addr, reg << 1, val, ret);
				}
			}
			break;
		case PCA957X_TYPE:
			PCA953X_VPRINT("write(0x%02x): [0x%02x]=0x%02x\n", chip->addr, reg << 1, buf[0]);
			ret = i2c_write(chip->addr, reg << 1, 1, buf, 1);
			if (ret) {
				if (--retries > 0) {
					PCA953X_DPRINT("%d: write(0x%02x): [0x%02x]=0x%02x err %d\n", retries, chip->addr, reg << 1, buf[0], ret);
					mdelay(10);
					goto _retry;
				} else {
					PCA953X_PRINT("write(0x%02x): [0x%02x]=0x%02x err %d\n", chip->addr, reg << 1, buf[0], ret);
					break;
				}
			}
			PCA953X_VPRINT("write(0x%02x): [0x%02x]=0x%02x\n", chip->addr, (reg << 1) + 1, buf[1]);
			ret = i2c_write(chip->addr, (reg << 1) + 1, 1, (buf + 1), 1);
			if (ret) {
				if (--retries > 0) {
					PCA953X_DPRINT("%d: write(0x%02x): [0x%02x]=0x%02x err %d\n", retries, chip->addr, (reg << 1) + 1, buf[1], ret);
					mdelay(10);
					goto _retry;
				} else {
					PCA953X_PRINT("write(0x%02x): [0x%02x]=0x%02x err %d\n", chip->addr, (reg << 1) + 1, buf[1], ret);
				}
			}
			break;
		}
	}

	return ret;
}

static int pca953x_read_reg(struct pca953x_gpio_chip *chip, uint reg, uint16_t *val)
{
	int ret;
	u8 buf[2];
	int retries = 3;

_retry:
	buf[0] = buf[1] = 0;

	if (chip->ngpio <= 8) {
		ret = i2c_read(chip->addr, reg, 1, buf, 1);
		if (ret) {
			if (--retries > 0) {
				PCA953X_DPRINT("%d: read(0x%02x): [0x%02x] err %d\n", retries, chip->addr, reg, ret);
				mdelay(10);
				goto _retry;
			} else {
				PCA953X_DPRINT("read(0x%02x): [0x%02x] err %d\n", chip->addr, reg, ret);
				*val = 0;
			}
		} else {
			*val = buf[0];
			PCA953X_VPRINT("read(0x%02x): [0x%02x]=0x%02x\n", chip->addr, reg, buf[0]);
		}
	} else {
		ret = i2c_read(chip->addr, reg << 1, 1, buf, 2);
		if (ret) {
			if (--retries > 0) {
				PCA953X_DPRINT("%d: read(0x%02x): [0x%02x] err %d\n", retries, chip->addr, reg << 1, ret);
				mdelay(10);
				goto _retry;
			} else {
				PCA953X_DPRINT("read(0x%02x): [0x%02x] err %d\n", chip->addr, reg << 1, ret);
				*val = 0;
			}
		} else {
			*val = ((uint16_t)(buf[1] << 8)) | buf[0];
			PCA953X_VPRINT("read(0x%02x): [0x%02x]=0x%04x\n", chip->addr, reg << 1, *val);
		}
	}

	return ret;
}

/*----------------------------------------------------------------------*/

int pca953x_gpio_direction_input(uint8_t chip_addr, unsigned gpio_offset)
{
	struct pca953x_gpio_chip *chip;
	uint16_t reg_val;
	uint offset = 0;
	int ret;

	chip = get_chip(chip_addr, gpio_offset);
	if (!chip) {
		PCA953X_PRINT("0x%02x not found\n", chip_addr);
		return -EINVAL;
	}

	pca953x_activate_i2c_bus();

	reg_val = chip->reg_direction | (1u << gpio_offset);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;

exit:
	pca953x_inactivate_i2c_bus();
	return ret;
}

int pca953x_gpio_direction_output(uint8_t chip_addr, unsigned gpio_offset, int val)
{
	struct pca953x_gpio_chip *chip;
	uint16_t reg_val;
	uint offset = 0;
	int ret;

	chip = get_chip(chip_addr, gpio_offset);
	if (!chip) {
		PCA953X_PRINT("0x%02x not found\n", chip_addr);
		return -EINVAL;
	}

	pca953x_activate_i2c_bus();

	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << gpio_offset);
	else
		reg_val = chip->reg_output & ~(1u << gpio_offset);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << gpio_offset);
	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;

exit:
	pca953x_inactivate_i2c_bus();
	return ret;
}

int pca953x_gpio_get_value(uint8_t chip_addr, unsigned gpio_offset)
{
	struct pca953x_gpio_chip *chip;
	uint16_t reg_val;
	uint offset = 0;
	int ret;

	chip = get_chip(chip_addr, gpio_offset);
	if (!chip) {
		PCA953X_PRINT("0x%02x not found\n", chip_addr);
		return -EINVAL;
	}

	pca953x_activate_i2c_bus();

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_INPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_IN;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &reg_val);
	pca953x_inactivate_i2c_bus();
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << gpio_offset)) ? 1 : 0;
}

int pca953x_gpio_set_value(uint8_t chip_addr, unsigned gpio_offset, int val)
{
	struct pca953x_gpio_chip *chip;
	uint16_t reg_val;
	uint offset = 0;
	int ret;

	chip = get_chip(chip_addr, gpio_offset);
	if (!chip) {
		PCA953X_PRINT("0x%02x not found\n", chip_addr);
		return -EINVAL;
	}

	pca953x_activate_i2c_bus();

	if (val)
		reg_val = chip->reg_output | (1u << gpio_offset);
	else
		reg_val = chip->reg_output & ~(1u << gpio_offset);

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;
	ret = 0;

exit:
	pca953x_inactivate_i2c_bus();
	return ret;
}

/*----------------------------------------------------------------------*/

static int device_pca953x_init_state(struct pca953x_gpio_chip *chip)
{
	int ret;

	/* set initial state (Input) */
	ret = pca953x_write_reg(chip, PCA953X_DIRECTION, 0xffff);
	if (ret)
		return ret;

	/* set initial state (Low) */
	ret = pca953x_write_reg(chip, PCA953X_OUTPUT, 0);
	return ret;
}

static int device_pca953x_init(struct pca953x_gpio_chip *chip, int invert)
{
	int ret;

	/* set initial state */
	ret = device_pca953x_init_state(chip);
	if (ret)
		goto out;

	ret = pca953x_read_reg(chip, PCA953X_OUTPUT, &chip->reg_output);
	if (ret)
		goto out;

	ret = pca953x_read_reg(chip, PCA953X_DIRECTION,
			       &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	ret = pca953x_write_reg(chip, PCA953X_INVERT, invert);
	if (ret)
		goto out;

	return 0;

out:
	return ret;
}

static int device_pca957x_init_state(struct pca953x_gpio_chip *chip)
{
	int ret;

	/* Let every port in proper state, that could save power */
	ret = pca953x_write_reg(chip, PCA957X_PUPD, 0x0);
	if (ret)
		return ret;

	ret = pca953x_write_reg(chip, PCA957X_CFG, 0xffff);
	if (ret)
		return ret;

	ret = pca953x_write_reg(chip, PCA957X_OUT, 0x0);
	return ret;
}

static int device_pca957x_init(struct pca953x_gpio_chip *chip, int invert)
{
	int ret;
	uint16_t val = 0;

	/* Let every port in proper state, that could save power */
	ret = device_pca957x_init_state(chip);
	if (ret)
		goto out;

	ret = pca953x_read_reg(chip, PCA957X_IN, &val);
	if (ret)
		goto out;
	ret = pca953x_read_reg(chip, PCA957X_OUT, &chip->reg_output);
	if (ret)
		goto out;
	ret = pca953x_read_reg(chip, PCA957X_CFG, &chip->reg_direction);
	if (ret)
		goto out;

	/* set platform specific polarity inversion */
	ret = pca953x_write_reg(chip, PCA957X_INVRT, invert);
	if (ret)
		goto out;

	/* To enable register 6, 7 to controll pull up and pull down */
	ret = pca953x_write_reg(chip, PCA957X_BKEN, 0x202);
	if (ret)
		goto out;

	return 0;

out:
	return ret;
}

/*----------------------------------------------------------------------*/

void pca953x_gpio_init(struct pca953x_gpio_chip *gpios, int num)
{
	if (gpios) {
		struct pca953x_gpio_chip *chip;
		int i;
		int ret;

		pca953x_activate_i2c_bus();

		pca953x_chips = gpios;
		num_pca953x_chips = num;

		for (i = 0 ; i < num ; i++) {
			chip = gpios++;
			if (chip) {
				chip->chip_type = chip->chip_type & (PCA953X_TYPE | PCA957X_TYPE);
				if (chip->chip_type == PCA953X_TYPE)
					ret = device_pca953x_init(chip, chip->invert);
				else if (chip->chip_type == PCA957X_TYPE)
					ret = device_pca957x_init(chip, chip->invert);
				else {
					PCA953X_PRINT("invalid chip type: 0x%x \n", chip->chip_type);
					ret = -EINVAL;
					continue;
				}

				if (ret) {
					chip->chip_valid = 0;
					PCA953X_PRINT("0x%02x not found\n", chip->addr);
				} else {
					chip->chip_valid = 1;
					PCA953X_PRINT("found 0x%02x chip\n", chip->addr);
#ifdef CONFIG_PCA953X_GPIO_DEBUG
					PCA953X_PRINT("reg_output: 0x%04x \n", chip->reg_output);
					PCA953X_PRINT("reg_direction: 0x%04x \n", chip->reg_direction);
#endif
				}
			}
		}

		pca953x_inactivate_i2c_bus();

#ifdef CONFIG_PCA953X_GPIO_VERBOSE_DEBUG
		PCA953X_PUTS("initialized\n");
#endif
	}
}

void pca953x_gpio_shutdown(void)
{
	if (pca953x_chips) {
		struct pca953x_gpio_chip *chip;
		int i;
		int ret;

		pca953x_activate_i2c_bus();

		for (i = 0 ; i < num_pca953x_chips; i++) {
			chip = pca953x_chips++;
			if (chip && chip->chip_valid) {
				if (chip->chip_type == PCA953X_TYPE)
					ret = device_pca953x_init_state(chip);
				else if (chip->chip_type == PCA957X_TYPE)
					ret = device_pca957x_init_state(chip);
				else {
					ret = -EINVAL;
					continue;
				}
				if (ret)
					PCA953X_PRINT("shutdown state err %d\n", ret);
			}
		}

		pca953x_inactivate_i2c_bus();

#ifdef CONFIG_PCA953X_GPIO_VERBOSE_DEBUG
		PCA953X_PUTS("shutted down\n");
#endif
	}
}

/*----------------------------------------------------------------------*/

#ifdef CONFIG_CMD_PCA953X_GPIO
#ifdef CONFIG_CMD_PCA953X_GPIO_INFO
/*
 * Display pca953x gpio information
 */
static int pca953x_gpio_info(uint8_t chip_addr)
{
	struct pca953x_gpio_chip *chip;
	int i;
	uint16_t data;
	uint offset = 0;
	int ret;
	int nr_gpio;
	int msb;

	chip = get_chip(chip_addr, 0);
	if (!chip) {
		printf("0x%02x not found\n", chip_addr);
		return -EINVAL;
	}

	pca953x_activate_i2c_bus();

	nr_gpio = chip->ngpio;
	msb = nr_gpio - 1;

	printf("pca953x gpio @ 0x%x (%d pins):\n\n", chip_addr, nr_gpio);
	printf("gpio pins: ");
	for (i = msb; i >= 0; i--)
		printf("%x", i);
	printf("\n");
	for (i = 11 + nr_gpio; i > 0; i--)
		printf("-");
	printf("\n");

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_DIRECTION;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_CFG;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &data);
	if (ret)
		data = 0;
	printf("direction: ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? 'i' : 'o');
	printf("\n");

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_INVERT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_INVRT;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &data);
	printf("invert:    ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? '1' : '0');
	printf("\n");

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_INPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_IN;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &data);
	if (ret)
		data = 0;
	printf("input:     ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? '1' : '0');
	printf("\n");

	switch (chip->chip_type) {
	case PCA953X_TYPE:
		offset = PCA953X_OUTPUT;
		break;
	case PCA957X_TYPE:
		offset = PCA957X_OUT;
		break;
	}
	ret = pca953x_read_reg(chip, offset, &data);
	if (ret)
		data = 0;
	printf("output:    ");
	for (i = msb; i >= 0; i--)
		printf("%c", data & (1 << i) ? '1' : '0');
	printf("\n");

	pca953x_inactivate_i2c_bus();

	return 0;
}
#endif /* CONFIG_CMD_PCA953X_INFO */

enum {
	PCA953X_GPIO_CMD_INFO,
	PCA953X_GPIO_CMD_DEVICE,
	PCA953X_GPIO_CMD_OUTPUT,
	PCA953X_GPIO_CMD_INPUT,
};

cmd_tbl_t cmd_pca953x[] = {
	U_BOOT_CMD_MKENT(device, 3, 0, (void *)PCA953X_GPIO_CMD_DEVICE, "", ""),
	U_BOOT_CMD_MKENT(output, 4, 0, (void *)PCA953X_GPIO_CMD_OUTPUT, "", ""),
	U_BOOT_CMD_MKENT(input, 3, 0, (void *)PCA953X_GPIO_CMD_INPUT, "", ""),
#ifdef CONFIG_CMD_PCA953X_GPIO_INFO
	U_BOOT_CMD_MKENT(info, 2, 0, (void *)PCA953X_GPIO_CMD_INFO, "", ""),
#endif
};

int do_pca953x_gpio(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	static uint8_t chip = CONFIG_SYS_I2C_PCA953X_GPIO_ADDR;
	int val;
	ulong ul_arg2 = 0;
	ulong ul_arg3 = 0;
	cmd_tbl_t *c;

	c = find_cmd_tbl(argv[1], cmd_pca953x, ARRAY_SIZE(cmd_pca953x));

	/* All commands but "device" require 'maxargs' arguments */
	if (!c || !((argc == (c->maxargs)) ||
		(((int)c->cmd == PCA953X_GPIO_CMD_DEVICE) &&
		 (argc == (c->maxargs - 1))))) {
		return cmd_usage(cmdtp);
	}

	/* arg2 used as chip number or pin number */
	if (argc > 2)
		ul_arg2 = simple_strtoul(argv[2], NULL, 16);

	/* arg3 used as pin or invert value */
	if (argc > 3)
		ul_arg3 = simple_strtoul(argv[3], NULL, 16) & 0x1;

	switch ((int)c->cmd) {
#ifdef CONFIG_CMD_PCA953X_GPIO_INFO
	case PCA953X_GPIO_CMD_INFO:
		return pca953x_gpio_info(chip);
#endif
	case PCA953X_GPIO_CMD_DEVICE:
		if (argc == 3)
			chip = (uint8_t)ul_arg2;
		printf("Current device address: 0x%x\n", chip);
		return 0;
	case PCA953X_GPIO_CMD_INPUT:
		val = pca953x_gpio_direction_input(chip, ul_arg2);
		if (val) {
			printf("chip 0x%02x, pin 0x%lx err %d\n", chip, ul_arg2, val);
			return val;
		}
		val = pca953x_gpio_get_value(chip, ul_arg2);
		printf("chip 0x%02x, pin 0x%lx = %d\n", chip, ul_arg2, val);
		return val;
	case PCA953X_GPIO_CMD_OUTPUT:
		return pca953x_gpio_direction_output(chip, ul_arg2, ul_arg3);
	default:
		/* We should never get here */
		return 1;
	}
}

U_BOOT_CMD(
	pca953x_gpio,	5,	1,	do_pca953x_gpio,
	"pca953x gpio access",
	"device [dev]\n"
	"	- show or set current device address\n"
#ifdef CONFIG_CMD_PCA953X_GPIO_INFO
	"pca953x_gpio info\n"
	"	- display info for current chip\n"
#endif
	"pca953x_gpio output pin 0|1\n"
	"	- set pin as output and drive low or high\n"
	"pca953x_gpio invert pin 0|1\n"
	"	- disable/enable polarity inversion for reads\n"
);

#endif /* CONFIG_CMD_PCA953X_GPIO */
