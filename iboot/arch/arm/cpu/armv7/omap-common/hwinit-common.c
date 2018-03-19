/*
 *
 * Common functions for OMAP4/5 based boards
 *
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Aneesh V	<aneesh@ti.com>
 *	Steve Sakoman	<steve@sakoman.com>
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
#include <asm/arch/sys_proto.h>
#include <asm/sizes.h>
#include <asm/emif.h>
#include <asm/omap_common.h>
#include <asm/arch/usb.h>
#include <errno.h>
#include <i2c.h>

#include <icom/keypad.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_BOARD_INIT)
void spl_board_init(void)
{
#ifdef CONFIG_SPL_I2C_SUPPORT
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif /* CONFIG_SPL_I2C_SUPPORT */
#ifdef CONFIG_SPL_INPUT_SUPPORT
	keypad_init(CONFIG_SYS_KEYPAD_ROWS, CONFIG_SYS_KEYPAD_COLS);
#endif /* CONFIG_SPL_INPUT_SUPPORT */
}
#endif /* CONFIG_SPL_BUILD && CONFIG_SPL_BOARD_INIT */

void do_set_mux(u32 base, struct pad_conf_entry const *array, int size)
{
	int i;
	struct pad_conf_entry *pad = (struct pad_conf_entry *) array;

	for (i = 0; i < size; i++, pad++)
		writew(pad->val, base + pad->offset);
}

static void set_mux_conf_regs(void)
{
	switch (omap_hw_init_context()) {
	case OMAP_INIT_CONTEXT_SPL:
		set_muxconf_regs_essential();
		break;
	case OMAP_INIT_CONTEXT_UBOOT_AFTER_SPL:
#if defined(CONFIG_OMAP54XX)
#ifdef CONFIG_SYS_ENABLE_PADS_ALL
		set_muxconf_regs_non_essential();
#endif
#else
		set_muxconf_regs_non_essential();
#endif
		break;
	case OMAP_INIT_CONTEXT_UBOOT_FROM_NOR:
	case OMAP_INIT_CONTEXT_UBOOT_AFTER_CH:
		set_muxconf_regs_essential();
#if defined(CONFIG_OMAP54XX)
#ifdef CONFIG_SYS_ENABLE_PADS_ALL
		set_muxconf_regs_non_essential();
#endif
#else
		set_muxconf_regs_non_essential();
#endif
		break;
	}
}

u32 cortex_rev(void)
{

	unsigned int rev;

	/* Read Main ID Register (MIDR) */
	asm ("mrc p15, 0, %0, c0, c0, 0" : "=r" (rev));

	return rev;
}

void omap_rev_string(void)
{
	u32 omap_rev = omap_revision();
	u32 omap_variant = (omap_rev & 0xFFFF0000) >> 16;
	u32 major_rev = (omap_rev & 0x00000F00) >> 8;
	u32 minor_rev = (omap_rev & 0x000000F0) >> 4;

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_DISPLAY_CPUINFO)
	u32 device_type = get_device_type();
	u32 silicon_type = (readl(STD_FUSE_PROD_ID_1) & SILICON_TYPE_MASK) >> SILICON_TYPE_SHIFT;
	u32 cpu_mhz = 0;

#if defined(CONFIG_OMAP44XX)
	if (omap_rev < OMAP4460_ES1_0) {
		/* OMAP4430 */
		if (silicon_type == SILICON_TYPE_HIGH_PERF)
			cpu_mhz = 1200;
		else if (silicon_type == SILICON_TYPE_STD_PERF)
			cpu_mhz = 1000;
		else if (silicon_type == SILICON_TYPE_LOW_PERF)
			cpu_mhz = 800;
	} else if (omap_rev < OMAP4470_ES1_0) {
		/* OMAP4460 */
		if (silicon_type == SILICON_TYPE_HIGH_PERF)
			cpu_mhz = 1500;
		else if (silicon_type == SILICON_TYPE_STD_PERF)
			cpu_mhz = 1200;
		else if (silicon_type == SILICON_TYPE_LOW_PERF)
			cpu_mhz = 920;
	} else {
		/* OMAP4470 */
		if (silicon_type == SILICON_TYPE_HIGH_PERF)
			cpu_mhz = 1500;
		else if (silicon_type == SILICON_TYPE_STD_PERF)
			cpu_mhz = 1300;
		else if (silicon_type == SILICON_TYPE_LOW_PERF)
			cpu_mhz = 1100;
	}
#endif /* CONFIG_OMAP44XX */

	printf(CONFIG_SYS_BOARD_NAME " <%s> OMAP%x ES%x.%x %s %uMHz\n",
			warm_reset() ? "WARM" : "COLD",
			omap_variant, major_rev, minor_rev,
			(device_type == GP_DEVICE) ? "GP" :
				(device_type == HS_DEVICE) ? "HS" : "EMU",
			cpu_mhz);

#else /* !(CONFIG_SPL_BUILD && CONFIG_DISPLAY_CPUINFO) */
	printf(CONFIG_SYS_BOARD_NAME ": OMAP%x ES%x.%x\n",
			omap_variant, major_rev, minor_rev);
#endif /* CONFIG_SPL_BUILD && CONFIG_DISPLAY_CPUINFO */
}

#ifdef CONFIG_SPL_BUILD
#if 0
static void init_boot_params(void)
{
	boot_params_ptr = (u32 *) &boot_params;
}
#endif
#else
static void init_boot_params(void)
{
	u32 boot_params_ptr_addr = (u32)boot_params.boot_message;
	/* See if the boot_params passed from SPL is valid */
	if (boot_params_ptr_addr > NON_SECURE_SRAM_START &&
			boot_params_ptr_addr < NON_SECURE_SRAM_END)
		boot_params_ptr = (struct omap_boot_parameters*)boot_params.boot_message;
	else
		boot_params_ptr = NULL;
}
#endif /* CONFIG_SPL_BUILD */

int ocp_softreset(u32 sysc, u32 syss, u32 sysc_value)
{
#if defined(CONFIG_OMAP44XX)

#define	SYSC_SOFTRESET			(1 << 1)
#define	SYSS_RESETDONE			(1 << 0)

	u32 start = 1 + (LDELAY << 1);

	__raw_writel(SYSC_SOFTRESET, sysc);
	while (!(__raw_readl(syss) & SYSS_RESETDONE)) {
		if (--start == 0) {
			debug("SYSS(0x%08x)=0x%08x\n", syss, __raw_readl(syss));
			debug("softreset: 0x%08x timedout\n", sysc);
			__raw_writel(sysc_value, sysc);
			return -ETIMEDOUT;
		}
	}
	__raw_writel(sysc_value, sysc);

	debug("SYSC(0x%08x)=0x%08X\n", sysc, __raw_readl(sysc));

	return 0;
#else
#error "ocp_softreset() not implemented\n"
#endif
}

int ocp_softreset_type2(u32 sysc, u32 sysc_value)
{
#if defined(CONFIG_OMAP44XX)

#define	TYPE2_SYSC_SOFTRESET	(1 << 0)

	u32 start = 1 + (LDELAY << 1);

	__raw_writel(TYPE2_SYSC_SOFTRESET, sysc);
	while ((__raw_readl(sysc) & TYPE2_SYSC_SOFTRESET)) {
		if (--start == 0) {
			debug("TYPE2 SYSC(0x%08x)=0x%08x\n", sysc, __raw_readl(sysc));
			debug("softreset2: 0x%08x timedout\n", sysc);
			__raw_writel(sysc_value, sysc);
			return -ETIMEDOUT;
		}
	}
	__raw_writel(sysc_value, sysc);

	debug("SYSC(0x%08x)=0x%08X\n", sysc, __raw_readl(sysc));

	return 0;
#else
#error "ocp_softreset_type2() not implemented\n"
#endif
}

/*
 * Routine: s_init
 * Description: Does early system init of watchdog, muxing,  andclocks
 * Watchdog disable is done always. For the rest what gets done
 * depends on the boot mode in which this function is executed
 *   1. s_init of SPL running from SRAM
 *   2. s_init of U-Boot running from FLASH
 *   3. s_init of U-Boot loaded to SDRAM by SPL
 *   4. s_init of U-Boot loaded to SDRAM by ROM code using the
 *	Configuration Header feature
 * Please have a look at the respective functions to see what gets
 * done in each of these cases
 * This function is called with SRAM stack.
 */
void s_init(void)
{
#ifdef CONFIG_SPL_BUILD
#if defined(CONFIG_OMAP44XX)
	/* Force EMIF self refresh earlier on OMAP4 */
	if (warm_reset())
		force_emif_self_refresh();
#endif
#else
	init_boot_params();
#endif /* CONFIG_SPL_BUILD */
	init_omap_revision();
#if defined(CONFIG_SPL_BUILD) && !defined(CONFIG_OMAP44XX)
	if (warm_reset() && (omap_revision() <= OMAP5430_ES1_0))
		force_emif_self_refresh();
#endif
	watchdog_init();
	set_mux_conf_regs();
#ifdef CONFIG_SPL_BUILD
	setup_clocks_for_console();
	preloader_console_init();
	do_io_settings();
#endif
/* James Wu: only apply prcm_init() once from SPL */
#ifdef CONFIG_SPL_BUILD
	prcm_init();
#endif
#ifdef CONFIG_SPL_BUILD
	timer_init();

	/* For regular u-boot sdram_init() is called from dram_init() */
	sdram_init();
#if 0
	init_boot_params();
#endif

	omap_gpio_init();
#endif /* CONFIG_SPL_BUILD */
}

/*
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 */
void wait_for_command_complete(struct watchdog *wd_base)
{
	int pending = 1;
	do {
		pending = readl(&wd_base->wwps);
	} while (pending);
}

/*
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 */
void watchdog_init(void)
{
	struct watchdog *wd2_base = (struct watchdog *)WDT2_BASE;

	writel(WD_UNLOCK1, &wd2_base->wspr);
	wait_for_command_complete(wd2_base);
	writel(WD_UNLOCK2, &wd2_base->wspr);
}


/*
 * This function finds the SDRAM size available in the system
 * based on DMM section configurations
 * This is needed because the size of memory installed may be
 * different on different versions of the board
 */
u32 omap_sdram_size(void)
{
	u32 section, i, valid;
	u64 sdram_start = 0, sdram_end = 0, addr,
	    size, total_size = 0, trap_size = 0;

	for (i = 0; i < 4; i++) {
		section	= __raw_readl(DMM_BASE + i*4);
		valid = (section & EMIF_SDRC_ADDRSPC_MASK) >>
			(EMIF_SDRC_ADDRSPC_SHIFT);
		addr = section & EMIF_SYS_ADDR_MASK;

		/* See if the address is valid */
		if ((addr >= DRAM_ADDR_SPACE_START) &&
		    (addr < DRAM_ADDR_SPACE_END)) {
			size = ((section & EMIF_SYS_SIZE_MASK) >>
				   EMIF_SYS_SIZE_SHIFT);
			size = 1 << size;
			size *= SZ_16M;

			if (valid != DMM_SDRC_ADDR_SPC_INVALID) {
				if (!sdram_start || (addr < sdram_start))
					sdram_start = addr;
				if (!sdram_end || ((addr + size) > sdram_end))
					sdram_end = addr + size;
			} else {
				trap_size = size;
			}

		}

	}
	total_size = (sdram_end - sdram_start) - (trap_size);

	return total_size;
}

#ifndef CONFIG_SPL_BUILD

/*
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 */
int dram_init(void)
{
	sdram_init();
	gd->ram_size = omap_sdram_size();
	return 0;
}

#if 0
/*
 * Print board information
 */
int checkboard(void)
{
	puts(sysinfo.board_string);
	return 0;
}
#endif


/* OMAP control module register for UTMI PHY */
#define CONTROL_DEV_CONF	0x4A002300
#define PHY_PD				0x1

/*
* This function is called by start_armboot. You can reliably use static
* data. Any boot-time function that require static data should be
* called from here
*/
int arch_cpu_init(void)
{
#if defined(CONFIG_OMAP44XX)
	/* Power down the internal USB phy */
	__raw_writel(PHY_PD, CONTROL_DEV_CONF);

#ifndef CONFIG_SYS_L2CACHE_OFF
	omap4_l2_cache_init();
#endif /* !CONFIG_SYS_L2CACHE_OFF */

#elif defined(CONFIG_OMAP54XX)
#error "Not Implemented\n"
#else
#error "Not Implemented\n"
#endif
	return 0;
}

void arch_preboot_os(void)
{
	/* James Wu @TODO@ */
}

#define ARCH_CMDLINE_OMAP4_PMIC_SPEED		" pmic="
#define ARCH_CMDLINE_ANDROID_BOOTLOADER		" androidboot.bootloader="
#define	ARCH_CMDLINE_DSS_DEBUG				" omapdss.debug=y"

size_t arch_setup_linux_cmdline(char *cmdline)
{
	char* cmdline_start = cmdline;

#ifdef CONFIG_OMAP4_PMIC_SPEED
{
	/* James Wu: the maximum speed of I2C-1 bus frequency depends on the system clock:
	 * 2.2 MHz (if 19.2 MHz), 2.4 MHz (26 MHz) or 2.9 MHz (38.4 MHz).
	 */
	char buff[10];
	size_t length;

	/* pmic= */
	strcpy(cmdline, ARCH_CMDLINE_OMAP4_PMIC_SPEED);
	cmdline += sizeof(ARCH_CMDLINE_OMAP4_PMIC_SPEED) - 1;
	length = sprintf(buff, "%u", CONFIG_OMAP4_PMIC_SPEED);
	strncpy(cmdline, buff, length);
	cmdline += length;
}
#endif /* CONFIG_OMAP4_PMIC_SPEED */

	/* androidboot.bootloader */
#ifdef CONFIG_SYS_IBOOT_VERSION
	strcpy(cmdline, ARCH_CMDLINE_ANDROID_BOOTLOADER);
	cmdline += sizeof(ARCH_CMDLINE_ANDROID_BOOTLOADER) - 1;
	strcpy(cmdline, CONFIG_SYS_IBOOT_VERSION);
	cmdline += sizeof(CONFIG_SYS_IBOOT_VERSION) - 1;
#endif /* CONFIG_SYS_IBOOT_VERSION */

#if defined(CONFIG_OMAP2_DSS) && defined(CONFIG_OMAP2_DSS_KERNEL_DEBUG_SUPPORT)
	strcpy(cmdline, ARCH_CMDLINE_DSS_DEBUG);
	cmdline += sizeof(ARCH_CMDLINE_DSS_DEBUG) - 1;
#endif /* CONFIG_OMAP2_DSS && CONFIG_OMAP2_DSS_KERNEL_DEBUG_SUPPORT */

	return (size_t)(cmdline - cmdline_start);
}

#endif /* !CONFIG_SPL_BUILD */

/*
 *  get_device_type(): tell if GP/HS/EMU/TST
 */
u32 get_device_type(void)
{
	struct omap_sys_ctrl_regs *ctrl =
		      (struct omap_sys_ctrl_regs *) SYSCTRL_GENERAL_CORE_BASE;

	return (readl(&ctrl->control_status) &
				      (DEVICE_TYPE_MASK)) >> DEVICE_TYPE_SHIFT;
}

#ifndef CONFIG_SPL_BUILD

/*
 * Print CPU information
 */
#if 0
int print_cpuinfo(void)
{
	puts("CPU  : ");
	omap_rev_string();

	return 0;
}
#else
#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{
	u32 omap4_rev = omap_revision();

	if (!boot_params_ptr)
		puts("BAD boot params\n");

	printf("CPU: OMAP%x ES%x.%x\n",
			(omap4_rev & 0xFFFF0000) >> 16,
			(omap4_rev & 0x00000F00) >> 8,
			(omap4_rev & 0x000000F0) >> 4);

	return 0;
}
#endif /* CONFIG_DISPLAY_CPUINFO */
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#ifdef CONFIG_SYS_MEMORY_POST
#include <asm/armv7.h>
#include <icom/memory_post.h>

#if 0
#ifndef CONFIG_FASTBOOT_DEBUG
#define CONFIG_FASTBOOT_DEBUG
#endif
#endif

extern u32 omap_vram_start_address(void);

void fastboot_memory_test_prepare(u32 *start, u32 *size)
{
#if defined(CONFIG_OMAP2_DSS) && defined(CONFIG_OMAP2_VRAM)
	u32 addr_limit_end = omap_vram_start_address();
#else
	u32 addr_limit_end = ((u32)gd->start_addr_sp) & ~(SZ_2M - 1);
#endif /* CONFIG_OMAP2_DSS && CONFIG_OMAP2_VRAM */

	if ((CONFIG_FASTBOOT_BUFFER + CONFIG_FASTBOOT_BUFFER_SIZE) > addr_limit_end) {
		printf("%s: bad Fastboot Buffer: 0x%08x-0x%08lx (Size 0x%08lx) (END 0x%08x)\n", __func__,
				CONFIG_FASTBOOT_BUFFER, CONFIG_FASTBOOT_BUFFER + CONFIG_FASTBOOT_BUFFER_SIZE,
				CONFIG_FASTBOOT_BUFFER_SIZE, addr_limit_end);
	} else {
#ifdef CONFIG_FASTBOOT_DEBUG
		printf("%s: Fastboot Buffer: 0x%08x-0x%08lx (Size 0x%08lx) (END 0x%08x)\n", __func__,
				CONFIG_FASTBOOT_BUFFER, CONFIG_FASTBOOT_BUFFER + CONFIG_FASTBOOT_BUFFER_SIZE,
				CONFIG_FASTBOOT_BUFFER_SIZE, addr_limit_end);
#endif
	}

#if defined(CONFIG_OMAP2_DSS) && defined(CONFIG_OMAP2_VRAM)
	if (addr_limit_end > (u32)gd->start_addr_sp)
		printf("%s: bad VRAM: 0x%08x (addr_sp 0x%08x)\n", __func__, addr_limit_end, (u32)gd->start_addr_sp);
#endif

	/* Limit area */
	*start = CONFIG_SYS_SDRAM_BASE;
	*size = addr_limit_end - *start;

#ifdef CONFIG_FASTBOOT_DEBUG
	printf("%s: 0x%08x-0x%08x (bd 0x%08x)\n",
		__func__, *start, *start + (*size - 1), (u32)bd);
#endif
}

void arch_memory_post_prepare(memory_post_log_t log)
{
	/*
	 * this function is called just before we call linux
	 * it prepares the processor for linux
	 *
	 * we turn off caches etc ...
	 */
	disable_interrupts();

	/*
	 * Turn off I-cache and invalidate it
	 */
	icache_disable();
	invalidate_icache_all();

	/*
	 * turn off D-cache
	 * dcache_disable() in turn flushes the d-cache and disables MMU
	 */
	dcache_disable();
	v7_outer_cache_disable();

	/*
	 * After D-cache is flushed and before it is disabled there may
	 * be some new valid entries brought into the cache. We are sure
	 * that these lines are not dirty and will not affect our execution.
	 * (because unwinding the call-stack and setting a bit in CP15 SCTRL
	 * is all we did during this. We have not pushed anything on to the
	 * stack. Neither have we affected any static data)
	 * So just invalidate the entire d-cache again to avoid coherency
	 * problems
	 */
	invalidate_dcache_all();
}

void arch_memory_post_cleanup(memory_post_log_t log)
{
	/*
	 * Turn on I-cache
	 */
	icache_enable();

/* @FIXME@ @TODO@ Hang if we enable DCACHE again */
#if 0
	/*
	 * turn on D-cache
	 * dcache_enable() in turn flushes the d-cache and enables MMU and the outer cache
	 */
	dcache_enable();
#endif

	/* enable exceptions */
	enable_interrupts();

	invalidate_dcache_all();

	log("Data Cache was disabled!\nPlease REBOOT the device!\n\n");
}
#endif /* CONFIG_SYS_MEMORY_POST */

#endif /* !CONFIG_SPL_BUILD */
