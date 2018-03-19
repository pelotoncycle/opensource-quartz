/*
 * linux/drivers/staging/android/android_boot.c
 *
 * Copyright (C) 2012 InnoComm Mobile Technology Corp.
 * James Wu <james.wu@innocomm.com>
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/proc_fs.h>

#include <mach/emif-44xx.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/lpddr2-jedec.h>

/*----------------------------------------------------------------------*/

#define LPDDR2_MANUFACTURER_SAMSUNG	1
#define LPDDR2_MANUFACTURER_QIMONDA	2
#define LPDDR2_MANUFACTURER_ELPIDA	3
#define LPDDR2_MANUFACTURER_ETRON	4
#define LPDDR2_MANUFACTURER_NANYA	5
#define LPDDR2_MANUFACTURER_HYNIX	6
#define LPDDR2_MANUFACTURER_MOSEL	7
#define LPDDR2_MANUFACTURER_WINBOND	8
#define LPDDR2_MANUFACTURER_ESMT	9
#define LPDDR2_MANUFACTURER_SPANSION 11
#define LPDDR2_MANUFACTURER_SST		12
#define LPDDR2_MANUFACTURER_ZMOS	13
#define LPDDR2_MANUFACTURER_INTEL	14
#define LPDDR2_MANUFACTURER_NUMONYX	254
#define LPDDR2_MANUFACTURER_MICRON	255

/* MR8 register fields */
#define MR8_TYPE_SHIFT		0x0
#define MR8_TYPE_MASK		0x3
#define MR8_DENSITY_SHIFT	0x2
#define MR8_DENSITY_MASK	(0xF << 0x2)
#define MR8_IO_WIDTH_SHIFT	0x6
#define MR8_IO_WIDTH_MASK	(0x3 << 0x6)

/*----------------------------------------------------------------------*/

extern const struct lpddr2_timings lpddr2_elpida_timings_466_mhz;

/*----------------------------------------------------------------------*/

static struct lpddr2_device_info lpddr2_S4_emif1_cs0_dev = {
	.device_timings = {
		&lpddr2_elpida_timings_200_mhz,
		&lpddr2_elpida_timings_333_mhz,
		&lpddr2_elpida_timings_400_mhz,
		&lpddr2_elpida_timings_466_mhz,
	},
	.min_tck	= &lpddr2_elpida_min_tck,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144,
};

static struct lpddr2_device_info lpddr2_S4_emif1_cs1_dev = {
	.device_timings = {
		&lpddr2_elpida_timings_200_mhz,
		&lpddr2_elpida_timings_333_mhz,
		&lpddr2_elpida_timings_400_mhz,
		&lpddr2_elpida_timings_466_mhz,
	},
	.min_tck	= &lpddr2_elpida_min_tck,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144,
};

static struct lpddr2_device_info lpddr2_S4_emif2_cs0_dev = {
	.device_timings = {
		&lpddr2_elpida_timings_200_mhz,
		&lpddr2_elpida_timings_333_mhz,
		&lpddr2_elpida_timings_400_mhz,
		&lpddr2_elpida_timings_466_mhz,
	},
	.min_tck	= &lpddr2_elpida_min_tck,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144,
};

static struct lpddr2_device_info lpddr2_S4_emif2_cs1_dev = {
	.device_timings = {
		&lpddr2_elpida_timings_200_mhz,
		&lpddr2_elpida_timings_333_mhz,
		&lpddr2_elpida_timings_400_mhz,
		&lpddr2_elpida_timings_466_mhz,
	},
	.min_tck	= &lpddr2_elpida_min_tck,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144,
};

static __initdata struct emif_device_details emif_devices_emif1_1cs = {
	.cs0_device = &lpddr2_S4_emif1_cs0_dev,
	.cs1_device = NULL,
};

static __initdata struct emif_device_details emif_devices_emif1_2cs = {
	.cs0_device = &lpddr2_S4_emif1_cs0_dev,
	.cs1_device = &lpddr2_S4_emif1_cs1_dev,
};

static __initdata struct emif_device_details emif_devices_emif2_1cs = {
	.cs0_device = &lpddr2_S4_emif2_cs0_dev,
	.cs1_device = NULL,
};

static __initdata struct emif_device_details emif_devices_emif2_2cs = {
	.cs0_device = &lpddr2_S4_emif2_cs0_dev,
	.cs1_device = &lpddr2_S4_emif2_cs1_dev,
};

/*----------------------------------------------------------------------*/

static u32 lpddr2_manufacturer;
static u32 emif1_cs0_density, emif1_cs1_density;
static u32 emif2_cs0_density, emif2_cs1_density;

/*----------------------------------------------------------------------*/

#define OMAP_EMIF_MR_ERROR	0xFFFFFFFF

static u32 omap_emif_get_mr(u32 emif_nr, u32 cs, u32 mr_addr)
{
	void __iomem *base = (emif_nr == EMIF1) ?
			(void __iomem *)OMAP44XX_EMIF1_VIRT : (void __iomem *)OMAP44XX_EMIF2_VIRT;
	u32 mr;

	if (emif_nr > EMIF2)
		return OMAP_EMIF_MR_ERROR;

	if (cs > 1)
		return OMAP_EMIF_MR_ERROR;
	else if (cs == 1) {
		u32 temp = __raw_readl(base + OMAP44XX_EMIF_SDRAM_CONFIG);
		if ((temp & OMAP44XX_REG_EBANK_MASK) != (EBANK_CS1_EN << OMAP44XX_REG_EBANK_SHIFT))
			return OMAP_EMIF_MR_ERROR;
	}

	mr_addr |= cs << OMAP44XX_REG_CS_SHIFT;
	__raw_writel(mr_addr, base + OMAP44XX_EMIF_LPDDR2_MODE_REG_CFG);
	mr =  __raw_readb(base  +  OMAP44XX_EMIF_LPDDR2_MODE_REG_DATA);

	if (((mr & 0x0000ff00) >>  8) == (mr & 0xff) &&
	    ((mr & 0x00ff0000) >> 16) == (mr & 0xff) &&
	    ((mr & 0xff000000) >> 24) == (mr & 0xff))
		return mr & 0xff;
	else
		return mr;
}

/*----------------------------------------------------------------------*/

/*
 * lpddr2_omap4_emif_get_manufacturer - identify ddr manufacturer
 * Identify DDR vendor ID for selecting correct timing parameter
 * for dynamic ddr detection.
 */
static u32 lpddr2_omap4_emif_get_manufacturer(void)
{
	return omap_emif_get_mr(EMIF1, 0, LPDDR2_MR5);
}

/*
 * lpddr2_omap4_emif_get_density - identify ddr density
 * Identify DDR density for selecting correct timing parameter
 * for dynamic ddr detection.
 */
static u32 lpddr2_omap4_emif_get_density(u32 emif_nr, u32 cs)
{
	u32 mr;

	mr = omap_emif_get_mr(emif_nr, cs, LPDDR2_MR8);
	if (mr >= 0xFF)
		return mr;

	return (mr & MR8_DENSITY_MASK) >> MR8_DENSITY_SHIFT;
}

/*----------------------------------------------------------------------*/

static const char *get_lpddr2_manufacturer(u32 manufacturer)
{
	switch (manufacturer) {
	case LPDDR2_MANUFACTURER_SAMSUNG:
		return "Samsung";
	case LPDDR2_MANUFACTURER_QIMONDA:
		return "Qimonda";
	case LPDDR2_MANUFACTURER_ELPIDA:
		return "Elpida";
	case LPDDR2_MANUFACTURER_ETRON:
		return "Etron";
	case LPDDR2_MANUFACTURER_NANYA:
		return "Nanya";
	case LPDDR2_MANUFACTURER_HYNIX:
		return "Hynix";
	case LPDDR2_MANUFACTURER_MOSEL:
		return "Mosel";
	case LPDDR2_MANUFACTURER_WINBOND:
		return "Winbond";
	case LPDDR2_MANUFACTURER_ESMT:
		return "ESMT";
	case LPDDR2_MANUFACTURER_SPANSION:
		return "Spansion";
	case LPDDR2_MANUFACTURER_SST:
		return "SST";
	case LPDDR2_MANUFACTURER_ZMOS:
		return "ZMOS";
	case LPDDR2_MANUFACTURER_INTEL:
		return "Intel";
	case LPDDR2_MANUFACTURER_NUMONYX:
		return "Numonyx";
	case LPDDR2_MANUFACTURER_MICRON:
		return "Micron";
	default:
		return "Unknown";
	}
}

static const u32 lpddr2_density_2_size_in_mbytes[] = {
	8,			/* 64Mb */
	16,			/* 128Mb */
	32,			/* 256Mb */
	64,			/* 512Mb */
	128,			/* 1Gb   */
	256,			/* 2Gb   */
	512,			/* 4Gb   */
	1024,			/* 8Gb   */
	2048,			/* 16Gb  */
	4096			/* 32Gb  */
};

static const u32 get_lpddr2_density_in_mbytes(u32 density)
{
	if (density <= LPDDR2_DENSITY_32Gb) {
		return lpddr2_density_2_size_in_mbytes[density];
	} else
		return 0;
}

/*----------------------------------------------------------------------*/

//static struct emif_device_details *lpddr2_omap4_emif_get_device(
static struct emif_device_details * __init lpddr2_omap4_emif_get_device(
		u32 emif_nr, u32 *emif_cs0_density, u32 *emif_cs1_density)
{
	struct emif_device_details *emif_device = (emif_nr == EMIF1) ?
			&emif_devices_emif1_1cs : &emif_devices_emif2_1cs;
	struct lpddr2_device_info *emif_cs0_dev = NULL, *emif_cs1_dev = NULL;
	u32 cs0_density, cs1_density;

	cs0_density = lpddr2_omap4_emif_get_density(emif_nr, 0);
	cs1_density = lpddr2_omap4_emif_get_density(emif_nr, 1);

	if (cs0_density > LPDDR2_DENSITY_32Gb) {
		WARN(1, "LPDDR2 emif%u cs0 density: 0x%x (unsupported)\n", emif_nr+1, cs0_density);
		goto _exit;
	}

	if (cs1_density != OMAP_EMIF_MR_ERROR) {
		/* found cs1 density */
		if (cs1_density > LPDDR2_DENSITY_32Gb)
			WARN(1, "LPDDR2 emif%u cs1 density: 0x%x (unsupported)\n", emif_nr+1, cs1_density);
		else
			emif_device = (emif_nr == EMIF1) ?
					&emif_devices_emif1_2cs : &emif_devices_emif2_2cs;
	}
	emif_cs0_dev = (struct lpddr2_device_info *)emif_device->cs0_device;
	emif_cs1_dev = (struct lpddr2_device_info *)emif_device->cs1_device;

	if (emif_cs1_dev)
		emif_cs1_dev->density = cs1_density;

	if (emif_cs0_dev) {
		emif_cs0_dev->density = cs0_density;

		if (emif_cs1_dev)
			pr_info("LPDDR2 emif%u density: %uMB,%uMB\n", emif_nr+1,
					get_lpddr2_density_in_mbytes(emif_cs0_dev->density),
					get_lpddr2_density_in_mbytes(emif_cs1_dev->density));
		else
			pr_info("LPDDR2 emif%u density: %uMB\n", emif_nr+1,
					get_lpddr2_density_in_mbytes(emif_cs0_dev->density));
	}

_exit:
	if (emif_cs0_density)
		*emif_cs0_density = cs0_density;
	if (emif_cs1_density)
		*emif_cs1_density = cs1_density;

	return emif_device;
}

void lpddr2_omap4_emif_setup(void)
{
	struct emif_device_details *emif1_device, *emif2_device;

	lpddr2_manufacturer = lpddr2_omap4_emif_get_manufacturer();
	pr_info("LPDDR2 manufacturer: %s (%u)\n", get_lpddr2_manufacturer(lpddr2_manufacturer), lpddr2_manufacturer);

	emif1_device = lpddr2_omap4_emif_get_device(EMIF1, &emif1_cs0_density, &emif1_cs1_density);
	emif2_device = lpddr2_omap4_emif_get_device(EMIF2, &emif2_cs0_density, &emif2_cs1_density);
	pr_info("LPDDR2: emif1 0x%p, emif2 0x%p\n", emif1_device, emif2_device);

	omap_emif_setup_device_details(emif1_device, emif2_device);
}

/*----------------------------------------------------------------------*/

/**
 * /proc/lpddr2info
 */

static int dump_lpddr2info(char *buffer)
{
	int n;
	int count = 0;

	*buffer = 0;

	/* manufacturer */
	n = sprintf(buffer, "LPDDR2 manufacturer: %s (%u)\n",
			get_lpddr2_manufacturer(lpddr2_manufacturer), lpddr2_manufacturer);

	if (n < 0)
		return n;

	count += n;
	buffer += n;

	/* emif1 density */
	n = -EINVAL;
	if (emif1_cs0_density > LPDDR2_DENSITY_32Gb)
		n = sprintf(buffer, "LPDDR2 emif1 cs0: 0x%x (unsupported)\n", emif1_cs0_density);
	else if (emif1_cs1_density != OMAP_EMIF_MR_ERROR) {
		/* found cs1 density */
		if (emif1_cs1_density > LPDDR2_DENSITY_32Gb)
			n = sprintf(buffer, "LPDDR2 emif1 cs1: 0x%x (unsupported)\n", emif1_cs0_density);
		else
			n = sprintf(buffer, "LPDDR2 emif1: %uMB,%uMB\n",
					get_lpddr2_density_in_mbytes(emif1_cs0_density),
					get_lpddr2_density_in_mbytes(emif1_cs1_density));
	} else
		n = sprintf(buffer, "LPDDR2 emif1: %uMB\n",
				get_lpddr2_density_in_mbytes(emif1_cs0_density));

	if (n < 0)
		return n;

	count += n;
	buffer += n;

	/* emif2 density */
	n = -EINVAL;
	if (emif2_cs0_density > LPDDR2_DENSITY_32Gb)
		n = sprintf(buffer, "LPDDR2 emif2 cs0: 0x%x (unsupported)\n", emif2_cs0_density);
	else if (emif2_cs1_density != OMAP_EMIF_MR_ERROR) {
		/* found cs1 density */
		if (emif2_cs1_density > LPDDR2_DENSITY_32Gb)
			n = sprintf(buffer, "LPDDR2 emif2 cs1: 0x%x (unsupported)\n", emif2_cs0_density);
		else
			n = sprintf(buffer, "LPDDR2 emif2: %uMB,%uMB\n",
					get_lpddr2_density_in_mbytes(emif2_cs0_density),
					get_lpddr2_density_in_mbytes(emif2_cs1_density));
	} else
		n = sprintf(buffer, "LPDDR2 emif2: %uMB\n",
				get_lpddr2_density_in_mbytes(emif2_cs0_density));

	if (n < 0)
		return count;

	count += n;
	buffer += n;

	return count;
}

static int proc_lpddr2info_read(char *page, char **start,
			off_t off, int count, int *eof, void *data)
{
	char *buffer = page;
	int code = 0;

	count -= off;
	count -= 1;		/* for NUL at end */
	if (count < 0)
		return -EINVAL;

	code = dump_lpddr2info(buffer);
	if (code > 0) {
		buffer += code;
		count -= code;
	}

	*eof = 1;
	return (buffer - page) - off;
}

static int __init lpddr2_omap4_emif_procfs_init(void)
{
	struct proc_dir_entry *proc_lpddr2info = create_proc_entry("lpddr2info", S_IRUGO, NULL);

	if (proc_lpddr2info) {
		proc_lpddr2info->data = NULL;
		proc_lpddr2info->read_proc = proc_lpddr2info_read;
		proc_lpddr2info->write_proc = NULL;
		proc_lpddr2info->size = 0;
	}

	return 0;
}
late_initcall(lpddr2_omap4_emif_procfs_init);
