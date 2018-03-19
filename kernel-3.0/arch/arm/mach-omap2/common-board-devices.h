#ifndef __OMAP_COMMON_BOARD_DEVICES__
#define __OMAP_COMMON_BOARD_DEVICES__

#define NAND_BLOCK_SIZE	SZ_128K

struct twl4030_platform_data;
struct mtd_partition;

void omap_pmic_init(int bus, u32 clkrate, const char *pmic_type, int pmic_irq,
		    struct twl4030_platform_data *pmic_data);

static inline void omap2_pmic_init(const char *pmic_type,
				   struct twl4030_platform_data *pmic_data)
{
	omap_pmic_init(2, 2600, pmic_type, INT_24XX_SYS_NIRQ, pmic_data);
}

static inline void omap3_pmic_init(const char *pmic_type,
				   struct twl4030_platform_data *pmic_data)
{
	omap_pmic_init(1, 2600, pmic_type, INT_34XX_SYS_NIRQ, pmic_data);
}

#if 0
static inline void omap4_pmic_init(const char *pmic_type,
				   struct twl4030_platform_data *pmic_data)
{
	/* Phoenix Audio IC needs I2C1 to start with 400 KHz or less */
	omap_pmic_init(1, 400, pmic_type, OMAP44XX_IRQ_SYS_1N, pmic_data);
}
#else
static u32 pmic_speed = 400;

static inline void omap4_pmic_init(const char *pmic_type,
				   struct twl4030_platform_data *pmic_data)
{
	omap_pmic_init(1, pmic_speed, pmic_type, OMAP44XX_IRQ_SYS_1N, pmic_data);
}

/*
 * You can override this with pmic= cmdline option.
 */
static int __init omap4_pmic_speed_setup(char *str)
{
	pmic_speed = simple_strtoul(str, NULL, 10);

	if (pmic_speed == 0)
		pmic_speed = 400;
	else if (pmic_speed > 3400)
		pmic_speed = 3400;

	return 1;
}
__setup("pmic=", omap4_pmic_speed_setup);
#endif

struct ads7846_platform_data;

void omap_ads7846_init(int bus_num, int gpio_pendown, int gpio_debounce,
		       struct ads7846_platform_data *board_pdata);
void omap_nand_flash_init(int opts, struct mtd_partition *parts, int n_parts);

#endif /* __OMAP_COMMON_BOARD_DEVICES__ */
