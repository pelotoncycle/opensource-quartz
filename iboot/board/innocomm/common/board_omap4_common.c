/*
 * (C) Copyright 2011
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
#include <common.h>
#include <exports.h>
#include <version.h>
#include <twl6030.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/usb.h>
#include <asm/arch/i2c.h>
#include <asm/gpio.h>
#include <linux/usb/musb.h>

#include <icom/aboot.h>
#include <icom/keypad.h>
#include <icom/power_supply.h>
#include <icom/omapdss.h>

#include "board_omap4_common.h"

/*-------------------------------------------------------------------------*/

#if !defined(CONFIG_ABOOT_FASTBOOT_ROWBITS) || (CONFIG_ABOOT_FASTBOOT_ROWBITS <= 0)
#warning "Please define CONFIG_ABOOT_FASTBOOT_ROWBITS"
#define CONFIG_ABOOT_FASTBOOT_ROWBITS	(CONFIG_KEYPAD_PWRBUTTON_ROW | CONFIG_SYS_KEYPAD_ROW_MASKS)
#endif
#if !defined(CONFIG_ABOOT_FASTBOOT_COLBITS) || (CONFIG_ABOOT_FASTBOOT_COLBITS <= 0)
#warning "Please define CONFIG_ABOOT_FASTBOOT_COLBITS"
#define CONFIG_ABOOT_FASTBOOT_COLBITS	(CONFIG_KEYPAD_PWRBUTTON_COL | CONFIG_SYS_KEYPAD_COL_MASKS)
#endif

#if !defined(CONFIG_ABOOT_RECOVERY_ROWBITS) || (CONFIG_ABOOT_RECOVERY_ROWBITS <= 0)
#warning "Please define CONFIG_ABOOT_RECOVERY_ROWBITS"
#define CONFIG_ABOOT_RECOVERY_ROWBITS	(CONFIG_KEYPAD_PWRBUTTON_ROW | CONFIG_SYS_KEYPAD_ROW_MASKS)
#endif
#if !defined(CONFIG_ABOOT_RECOVERY_COLBITS) || (CONFIG_ABOOT_RECOVERY_COLBITS <= 0)
#warning "Please define CONFIG_ABOOT_RECOVERY_COLBITS"
#define CONFIG_ABOOT_RECOVERY_COLBITS	(CONFIG_KEYPAD_PWRBUTTON_COL | CONFIG_SYS_KEYPAD_COL_MASKS)
#endif

/*-------------------------------------------------------------------------*/

#ifdef DEBUG
#ifndef CONFIG_BOARD_DEBUG
#define CONFIG_BOARD_DEBUG
#endif /* CONFIG_BOARD_DEBUG */
#ifndef CONFIG_BOARD_VERBOSE_DEBUG
#define CONFIG_BOARD_VERBOSE_DEBUG
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */
#endif

#ifdef CONFIG_BOARD_DEBUG
#define BOARD_DPRINT(fmt, args...) \
	do {printf("[board] " fmt, ##args);} while (0)
#define BOARD_DPUTS(fmt) \
	do {puts("[board] " fmt);} while (0)
#else /* !CONFIG_BOARD_DEBUG */
#define BOARD_DPRINT(fmt, args...) \
	do {} while (0)
#define BOARD_DPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_BOARD_DEBUG */

#ifdef CONFIG_BOARD_VERBOSE_DEBUG
#define BOARD_VPRINT(fmt, args...) \
	do {printf("[board] " fmt, ##args);} while (0)
#define BOARD_VPUTS(fmt) \
	do {puts("[board] " fmt);} while (0)
#else /* !CONFIG_BOARD_VERBOSE_DEBUG */
#define BOARD_VPRINT(fmt, args...) \
	do {} while (0)
#define BOARD_VPUTS(fmt) \
	do {} while (0)
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */

#define BOARD_PRINT(fmt, args...) \
	do {printf("board: " fmt, ##args);} while (0)
#define BOARD_PUTS(fmt) \
	do {puts("board: " fmt);} while (0)
#define PRINT(fmt, args...) \
	do {printf(fmt, ##args);} while (0)
#define PUTS(fmt) \
	do {puts(fmt);} while (0)
#define BOARD_ERR(fmt, args...) \
	do {printf("BOARD: " fmt, ##args);} while (0)

/*-------------------------------------------------------------------------*/

/* TWL6032 registers */
#define	VALIDITY0				0x17	/* iCOM: Battery capacity */
#define	VALIDITY1				0x18	/* iCOM: Battery insertion/removal */
#define		ICOM_VALIDITY1_MASK		0x07
#define		ICOM_DEVOFF_BCK			(1 << 2)	/* DEVOFF_BCK in PHOENIX_LAST_TURNOFF_STS */
#define		ICOM_RESTART_BB			(1 << 1)	/* RESTART_BB in PHOENIX_START_CONDITION */
#define		ICOM_FIRST_SYS_INS		(1 << 0)	/* FIRST_SYS_INS in PHOENIX_START_CONDITION */
#define	VALIDITY2				0x19
#define	VALIDITY3				0x1A
#define	VALIDITY4				0x1B
#define	VALIDITY5				0x1C
#define	VALIDITY6				0x1D
#define	VALIDITY7				0x1E
#define PHOENIX_START_CONDITION	0x1F
#define		RESTART_BB			(1 << 6)	/* a system supply bounce or battery re-insertion (with coin cell) */
#define		FIRST_SYS_INS		(1 << 5)	/* a battery insertion (or system supply rise) */
#define		STRT_ON_RTC			(1 << 4)	/* RTC event */
#define		STRT_ON_PLUG_DET	(1 << 3)	/* PLUG_DET (AC or USB R/W 0 charger) event */
#define		STRT_ON_USB_ID		(1 << 2)	/* USB_ID event */
#define		STRT_ON_RPWRON		(1 << 1)	/* RPWRON event */
#define		STRT_ON_PWRON		(1 << 0)	/* PWRON event */
#define STS_HW_CONDITIONS		0x21
#define		STS_PREQ3			(1 << 7)	/* Level status of PREQ3 */
#define		STS_PREQ2			(1 << 6)	/* Level status of PREQ2 */
#define		STS_PREQ1			(1 << 5)	/* Level status of PREQ1 */
#define		STS_VSYSMIN_HI		(1 << 4)	/* Level status of VSYSMIN_HI comparator */
#define		STS_PLUG_DET		(1 << 3)	/* Level status of an AC or USB charger plug detection */
#define		STS_USB_ID			(1 << 2)	/* Level status of USB ID */
#define		STS_RPWRON			(1 << 1)	/* Level status of RPWRON button */
#define		STS_PWRON			(1 << 0)	/* Level status of PWRON button */
#define PHOENIX_LAST_TURNOFF_STS	0x22
#define		FALLBACK			(1 << 7)	/* a system supply drop below VSYSMIN_LO when ACTIVE and a charger ID plug */
#define		DEVOFF_RPWRON		(1 << 6)	/* RPWRON */
#define		DEVOFF_SHORT		(1 << 5)	/* a shorted power resource (LDO or SMPS) */
#define		DEVOFF_WDT			(1 << 4)	/* a primary watchdog expiration */
#define		DEVOFF_TSHUT		(1 << 3)	/* a thermal shutdown event */
#define		DEVOFF_BCK			(1 << 2)	/* a battery removal */
#define		DEVOFF_LPK			(1 << 1)	/* a long key press event */
#define		OSC_RC32K			(1 << 0)
#define RTC_STATUS_REG			0x11
#define		RTC_STATUS_POWER_UP	0x80	/* Indicates that a reset occurred */
#define		RTC_STATUS_ALARM	0x40	/* Indicates that an alarm interrupt has been generated */
#define		RTC_STATUS_1D_EVENT	0x20	/* One day has occurred */
#define		RTC_STATUS_1H_EVENT	0x10	/* One hour has occurred */
#define		RTC_STATUS_1M_EVENT	0x08	/* One minute has occurred */
#define		RTC_STATUS_1S_EVENT	0x04	/* One second has occurred */
#define		RTC_STATUS_RUN		0x02	/* RTC is running */
#define	RTC_STATUS_TIMER_EVENTS \
		(RTC_STATUS_1D_EVENT | RTC_STATUS_1H_EVENT | RTC_STATUS_1M_EVENT | RTC_STATUS_1S_EVENT)
#define RTC_INTERRUPTS_REG		0x12
#define		RTC_IT_ALARM		0x08	/* Interrupt enabled */
#define		RTC_IT_TIMER		0x04	/* Interrupt enabled */
#define		RTC_IT_EVERY_DAY	0x03	/* Every day */
#define		RTC_IT_EVERY_HOUR	0x02	/* Every hour */
#define		RTC_IT_EVERY_MIN	0x01	/* Every minute */
#define		RTC_IT_EVERY_SEC	0x00	/* Every second */
#define CONTROLLER_STAT1		0x03
#define		VAC_DET				(1 << 3)	/* VAC is present */
#define		VBUS_DET			(1 << 2)	/* VBUS is present */
#define		BAT_REMOVED			(1 << 1)	/* Battery has been removed */

/*-------------------------------------------------------------------------*/

DECLARE_GLOBAL_DATA_PTR;

static int board_factory_tool __attribute__ ((section (".data"))) = -1;

/*-------------------------------------------------------------------------*/

void innocomm_board_detect_factory_tool(void)
{
	if (board_factory_tool == -1) {
#define BOARD_FACTORY_TOOL_DE_GPIO		35 /* JIG_DET */

		/* Pull pin up */
		writew((IEN | PTU | M3), CONTROL_PADCONF_CORE + GPMC_AD11);	/* gpio_35: JIG_DET active low */
		sdelay(1200); /* 1us (Enough loops assuming a maximum of 1.1GHz) */

		board_factory_tool = 0;
		if (!gpio_request(BOARD_FACTORY_TOOL_DE_GPIO, NULL)) {
			gpio_direction_input(BOARD_FACTORY_TOOL_DE_GPIO);
			if (gpio_get_value(BOARD_FACTORY_TOOL_DE_GPIO) == 0)
				board_factory_tool = 1;
			gpio_free(BOARD_FACTORY_TOOL_DE_GPIO);
		}

		/* Pull pin down and switch to the safe mode */
		writew((IEN | PTD | M7), CONTROL_PADCONF_CORE + GPMC_AD11);
	}
}

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_ABOOT_PCBA_WAIT_FOR_FACTORY_TOOL_TIME)
static void innocomm_board_wait_for_detect_factory_tool(int count)
{
	if (board_factory_tool != 1) {
		ulong start;

		/* Pull pin up */
		writew((IEN | PTU | M3), CONTROL_PADCONF_CORE + GPMC_AD11);	/* gpio_35: JIG_DET active low */
		sdelay(1200); /* 1us (Enough loops assuming a maximum of 1.1GHz) */

		gpio_request(BOARD_FACTORY_TOOL_DE_GPIO, NULL);
		gpio_direction_input(BOARD_FACTORY_TOOL_DE_GPIO);

		board_factory_tool = 0;

		puts("Wait for the Factory Tool...");

		start = get_timer(0);
		while (count > 0) {
			--count;

			if (gpio_get_value(BOARD_FACTORY_TOOL_DE_GPIO) == 0) {
				board_factory_tool = 1;
				break;
			}

			mdelay(50);
		}
		printf("%lums\n", get_timer(start));

		gpio_free(BOARD_FACTORY_TOOL_DE_GPIO);
		/* Pull pin down and switch to the safe mode */
		writew((IEN | PTD | M7), CONTROL_PADCONF_CORE + GPMC_AD11);
	}
}
#endif /* !CONFIG_SPL_BUILD && CONFIG_ABOOT_PCBA_WAIT_FOR_FACTORY_TOOL_TIME */

int innocomm_board_is_factory_mode(void)
{
	return (board_factory_tool == 1) ? 1 : 0;
}

#if !defined(CONFIG_SPL_BUILD)

static struct sar_public_free *const sar_public = (struct sar_public_free *)PUBLIC_SAR_RAM_1_FREE;
static u32 board_sys_serial_low = 0xFFFFFFFF;
static u32 board_sys_serial_high = 0xFFFFFFFF;

/* Make sure board_serialno is in DATA section */
static char board_serialno[32] __attribute__ ((section (".data")));

static u32 omap_reset_status __attribute__ ((section (".data"))) = 0;
static u8 pmic_last_turnoff __attribute__ ((section (".data"))) = 0;
static u8 pmic_start_condition __attribute__ ((section (".data"))) = 0;

/*-------------------------------------------------------------------------*/

static int twl6030_writeb(u8 module, u8 data, u8 address)
{
	int ret = twl_i2c_write_u8(module, data, address);
#ifdef CONFIG_BOARD_VERBOSE_DEBUG
	if (ret)
		BOARD_PRINT("write[0x%x,0x%x] err %d\n", module, address, ret);
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */
	return ret;
}

static u8 twl6030_readb(u8 module, u8 address)
{
	u8 data = 0;
#ifdef CONFIG_BOARD_VERBOSE_DEBUG
	int ret = twl_i2c_read_u8(module, &data, address);
	if (ret)
		BOARD_PRINT("readb[0x%x,0x%x] err %d\n", module, address, ret);
#else
	twl_i2c_read_u8(module, &data, address);
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */
	return data;
}

static void twl6030_secure_mode(void)
{
	/*
	 * Drive MSECURE low to enter the TWL6030 secure mode
	 */
	gpio_direction_output(CONFIG_TWL6030_MSECURE_GPIO, 0);
}

static void twl6030_unsecure_mode(void)
{
#if (CONFIG_TWL6030_MSECURE_GPIO == 6)
	writew((IDIS | M3), CONTROL_PADCONF_WKUP + PAD0_FREF_CLK0_OUT);	/* gpio_wk6: MSECURE high active */
#else
#error "CONFIG_TWL6030_MSECURE_GPIO not implemented"
#endif /* CONFIG_TWL6030_MSECURE_GPIO */

	/*
	 * Drive MSECURE high for TWL6030 write access.
	 */
	gpio_request(CONFIG_TWL6030_MSECURE_GPIO, NULL);
	gpio_direction_output(CONFIG_TWL6030_MSECURE_GPIO, 1);
}

/*-------------------------------------------------------------------------*/

static inline u32 board_get_cpu_reset(void)
{
	u32 prm_reset_status = readl(&prcm->prm_rstst);

	if (prm_reset_status) {
		if (prm_reset_status & GLOBAL_COLD_RST)
			PUTS("OMAP rst: GLOBAL_COLD_RST\n");
		if (prm_reset_status & GLOBAL_WARM_SW_RST)
			PUTS("OMAP rst: GLOBAL_WARM_SW_RST\n");
		if (prm_reset_status & MPU_WDT_RST)
			PUTS("OMAP rst: MPU_WDT_RST\n");
		if (prm_reset_status & EXTERNAL_WARM_RST)
			PUTS("OMAP rst: EXTERNAL_WARM_RST\n");
		if (prm_reset_status & VDD_MPU_VOLT_MGR_RST)
			PUTS("OMAP rst: VDD_MPU_VOLT_MGR_RST\n");
		if (prm_reset_status & VDD_IVA_VOLT_MGR_RST)
			PUTS("OMAP rst: VDD_IVA_VOLT_MGR_RST\n");
		if (prm_reset_status & VDD_CORE_VOLT_MGR_RST)
			PUTS("OMAP rst: VDD_CORE_VOLT_MGR_RST\n");
		if (prm_reset_status & ICEPICK_RST)
			PUTS("OMAP rst: ICEPICK_RST\n");
		if (prm_reset_status & C2C_RST)
			PUTS("OMAP rst: C2C_RST\n");
	}

	return prm_reset_status;
}

static inline u8 board_get_pmic_start_condition(void)
{
	u8 start_condition = twl6030_readb(TWL6030_MODULE_ID0, PHOENIX_START_CONDITION);
	/* PHOENIX_START_CONDITION: Cleared on a write access */
	twl6030_writeb(TWL6030_MODULE_ID0, start_condition, PHOENIX_START_CONDITION);

	if (start_condition) {
		if (start_condition & STRT_ON_PWRON)
			PUTS("PMIC: STRT_ON_PWRON\n");
		if (start_condition & STRT_ON_RPWRON)
			PUTS("PMIC: STRT_ON_RPWRON\n");
		if (start_condition & STRT_ON_USB_ID)
			PUTS("PMIC: STRT_ON_USB_ID\n");
		if (start_condition & STRT_ON_PLUG_DET)
			PUTS("PMIC: STRT_ON_PLUG_DET\n");
		if (start_condition & STRT_ON_RTC)
			PUTS("PMIC: STRT_ON_RTC\n");
		if (start_condition & FIRST_SYS_INS)
			PUTS("PMIC: FIRST_SYS_INS\n");
		if (start_condition & RESTART_BB)
			PUTS("PMIC: RESTART_BB\n");
	}

	return start_condition;
}

static inline u8 board_get_pmic_last_off(void)
{
	u8 last_turnoff_state = twl6030_readb(TWL6030_MODULE_ID0, PHOENIX_LAST_TURNOFF_STS);
	/* PHOENIX_LAST_TURNOFF_STS: Cleared on a write access */
	twl6030_writeb(TWL6030_MODULE_ID0, last_turnoff_state, PHOENIX_LAST_TURNOFF_STS);

	if (last_turnoff_state) {
		if (last_turnoff_state & DEVOFF_LPK)
			PUTS("PMIC: last off: DEVOFF_LPK\n");
		if (last_turnoff_state & DEVOFF_BCK)
			PUTS("PMIC: last off: DEVOFF_BCK\n");
		if (last_turnoff_state & DEVOFF_TSHUT)
			PUTS("PMIC: last off: DEVOFF_TSHUT\n");
		if (last_turnoff_state & DEVOFF_WDT)
			PUTS("PMIC: last off: DEVOFF_WDT\n");
		if (last_turnoff_state & DEVOFF_SHORT)
			PUTS("PMIC: last off: DEVOFF_SHORT\n");
		if (last_turnoff_state & DEVOFF_RPWRON)
			PUTS("PMIC: last off: DEVOFF_RPWRON\n");
		if (last_turnoff_state & FALLBACK)
			PUTS("PMIC: last off: FALLBACK\n");
	}

	return last_turnoff_state;
}

static int board_has_rtc_poweron_alarm(void)
{
	int has_poweron_alarm = 0;
#ifdef CONFIG_TWL6030_POWER
	u8 rtc_status, rtc_int;

	rtc_status = twl6030_readb(TWL_MODULE_RTC, RTC_STATUS_REG);
	rtc_int = twl6030_readb(TWL_MODULE_RTC, RTC_INTERRUPTS_REG);
	if (rtc_status || rtc_int) {
		int clear_rtc_int;
		int clear_rtc_status;

#ifdef CONFIG_BOARD_VERBOSE_DEBUG
		if (rtc_status & RTC_STATUS_POWER_UP)
			BOARD_DPUTS("RTC_STATUS: POWER UP\n");
		if (rtc_status & RTC_STATUS_ALARM)
			BOARD_DPUTS("RTC_STATUS: ALARM\n");
		if (rtc_status & RTC_STATUS_1D_EVENT)
			BOARD_DPUTS("RTC_STATUS: 1D EVENT\n");
		if (rtc_status & RTC_STATUS_1H_EVENT)
			BOARD_DPUTS("RTC_STATUS: 1H EVENT\n");
		if (rtc_status & RTC_STATUS_1M_EVENT)
			BOARD_DPUTS("RTC_STATUS: 1M EVENT\n");
		if (rtc_status & RTC_STATUS_1S_EVENT)
			BOARD_DPUTS("RTC_STATUS: 1S EVENT\n");
		if (rtc_status & RTC_STATUS_RUN)
			BOARD_DPUTS("RTC_STATUS: RUN\n");
		if (rtc_int & RTC_IT_ALARM)
			BOARD_DPUTS("RTC_INT: ALARM\n");
		if (rtc_int & RTC_IT_TIMER)
			BOARD_DPRINT("RTC_INT: TIMER (%d)\n", (rtc_int & 0x03));
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */

		/* RTC timer interrupt must be clear */
		clear_rtc_int = (rtc_int & RTC_IT_TIMER);
		/* RTC timer events must be clear */
		clear_rtc_status = (rtc_status & RTC_STATUS_TIMER_EVENTS);
		/* Clear RTC power-on reset */
		clear_rtc_status |= (rtc_status & RTC_STATUS_POWER_UP);

		/* James Wu @TODO@ Check the RTC power-on alarm */
#ifdef CONFIG_ABOOT_SUPPORT_POWERON_ALARM
#endif /* CONFIG_ABOOT_SUPPORT_POWERON_ALARM */

		/* The power-on alarm is not set so clear it */
		clear_rtc_int |= (rtc_int & RTC_IT_ALARM);
		clear_rtc_status |= (rtc_status & RTC_STATUS_ALARM);

		/* Clear RTC status */
		if (clear_rtc_status) {
			BOARD_VPRINT("Clear RTC Status (0x%02X)\n", clear_rtc_status);
			twl6030_writeb(TWL_MODULE_RTC, clear_rtc_status, RTC_STATUS_REG);
		}
		/* Mask RTC timer & alarm interrupts */
		if (clear_rtc_int) {
			BOARD_VPRINT("Clear RTC INTs (0x%02X)\n", clear_rtc_int);
			twl_clear_n_set(TWL_MODULE_RTC,
					/* clear */ clear_rtc_int,
					/* set   */ 0,
					RTC_INTERRUPTS_REG);
		}
	}
#endif /* CONFIG_TWL6030_POWER */

	return has_poweron_alarm;
}

static void board_update_battery_change_status(u8 start_condition, u8 last_turnoff_state)
{
	u8 validity1 = 0;

	if (start_condition & FIRST_SYS_INS)
		validity1 |= ICOM_FIRST_SYS_INS;
	if (start_condition & RESTART_BB)
		validity1 |= ICOM_RESTART_BB;
	if (last_turnoff_state & DEVOFF_BCK)
		validity1 |= ICOM_DEVOFF_BCK;

	if (validity1) {
		/* iCOM: Battery insertion/removal */
		u8 old_validity1 = twl6030_readb(TWL6030_MODULE_ID0, VALIDITY1) & ICOM_VALIDITY1_MASK;
		validity1 |= old_validity1;
		if (validity1 != old_validity1) {
			/**
			 * 1. Bootloader must store the current battery change status in TWL6032 VALIDITY1 register.
			 * 2. InnoComm BCI drivers in Kernel will read the battery change status and then clear TWL6032 VALIDITY1 register.
			 */
			twl6030_writeb(TWL6030_MODULE_ID0, validity1, VALIDITY1);
		}
	}
}

static u32 board_get_lpk_reason(u8 charger_stat)
{
	u32 boot_reason = ABOOT_REASON_NORMAL;

#ifdef CONFIG_ABOOT_NO_DEVOFF_LPK_POWERON
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

	PUTS("<LPK> <NO Main Battery>\nPlease press POWER button again to boot!\n\n");
	boot_reason = ABOOT_REASON_POWEROFF;

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

	if ((charger_stat & VAC_DET) && power_supply_has_ac_feature() &&
			power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS) {
		PUTS("<LPK> <AC> < CHARGER >\n");
		boot_reason = ABOOT_REASON_CHARGER;
	} else if (charger_stat & VBUS_DET) {
		if (power_supply_has_usb_feature()) {
			if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) {
				PUTS("<LPK> <NOT USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER)
#if defined(CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON)
				PUTS("\n<NOT ALLOWED> Power-off charging only supports AC charger!\n");
				PUTS("\nPlease press POWER button to boot!\n\n");
				boot_reason = ABOOT_REASON_POWEROFF;
#else
				PUTS("<No PWRBTN> < NORMAL >\n");
				boot_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON */
#else
				boot_reason = ABOOT_REASON_PC_CHARGER;
#endif /* CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER */
			} else if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
				PUTS("<LPK> <USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_POWEROFF_USB_AC_CHARGER)
#if defined(CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON)
				PUTS("\n<NOT ALLOWED> Power-off charging only supports External AC charger!\n");
				PUTS("\nPlease press POWER button to boot!\n\n");
				boot_reason = ABOOT_REASON_POWEROFF;
#else
				PUTS("<No PWRBTN> < NORMAL >\n");
				boot_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */
#else
				boot_reason = ABOOT_REASON_CHARGER;
#endif /* CONFIG_ABOOT_NO_POWEROFF_USB_AC_CHARGER */
			} else {
				PUTS("<LPK> <USB> < No CHARGER > <NOT ALLOWED> Please press POWER button to boot!\n\n");
				boot_reason = ABOOT_REASON_POWEROFF;
			}
		} else {
			if (power_supply_get_usb_type() == POWER_SUPPLY_TYPE_USB) {
				PUTS("<LPK> <NOT USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON)
				PUTS("\n<NOT ALLOWED> Power-off charging only supports AC charger!\n");
				PUTS("\nPlease press POWER button to boot!\n\n");
				boot_reason = ABOOT_REASON_POWEROFF;
#else
				PUTS("<No PWRBTN> < NORMAL >\n");
				boot_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON */
			} else if (power_supply_get_usb_type() != POWER_SUPPLY_TYPE_UNKNOWN) {
				PUTS("<LPK> <USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON)
				PUTS("\n<NOT ALLOWED> Power-off charging only supports External AC charger!\n");
				PUTS("\nPlease press POWER button to boot!\n\n");
				boot_reason = ABOOT_REASON_POWEROFF;
#else
				PUTS("<No PWRBTN> < NORMAL >\n");
				boot_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */
			} else {
				PUTS("<LPK> <USB> < No CHARGER > <NOT ALLOWED> Please press POWER button to boot!\n\n");
				boot_reason = ABOOT_REASON_POWEROFF;
			}
		}
	} else {
		PUTS("<LPK> Please press POWER button again to boot!\n\n");
		boot_reason = ABOOT_REASON_POWEROFF;
	}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
#else
	PUTS("<LPK> < NORMAL >\n");
#endif /* CONFIG_ABOOT_NO_DEVOFF_LPK_POWERON */

	return boot_reason;
}

static u32 board_get_coldreset_reason(u8 start_condition)
{
	u32 boot_dev = omap_boot_device();
	u32 coldreset_reason = ABOOT_REASON_NORMAL;
	u32 row_bits = 0, col_bits = 0;
	u8 last_turnoff_state = board_get_pmic_last_off();
	u8 hw_state;
	u8 charger_stat;

	/* Update the Battery change status */
	pmic_last_turnoff = last_turnoff_state;
	board_update_battery_change_status(start_condition, last_turnoff_state);

	keypad_poll(&row_bits, &col_bits);
	PRINT("<<COLD>> PWRBTN: %s, ROW: 0x%03X, COL: 0x%03X\n",
			(row_bits & CONFIG_KEYPAD_PWRBUTTON_ROW) ? "Yes" : "No",
			row_bits, col_bits);

	hw_state = twl6030_readb(TWL6030_MODULE_ID0, STS_HW_CONDITIONS);
#ifdef CONFIG_BOARD_VERBOSE_DEBUG
	if (hw_state) {
		PUTS("\n");
		if (hw_state & STS_PWRON)
			BOARD_DPUTS("PMIC: STS_PWRON\n");
		if (hw_state & STS_RPWRON)
			BOARD_DPUTS("PMIC: STS_RPWRON\n");
		if (hw_state & STS_USB_ID)
			BOARD_DPUTS("PMIC: STS_USB_ID\n");
		if (hw_state & STS_PLUG_DET)
			BOARD_DPUTS("PMIC: STS_PLUG_DET\n");
		if (hw_state & STS_VSYSMIN_HI)
			BOARD_DPUTS("PMIC: STS_VSYSMIN_HI\n");
		if (hw_state & STS_PREQ1)
			BOARD_DPUTS("PMIC: STS_PREQ1\n");
		if (hw_state & STS_PREQ2)
			BOARD_DPUTS("PMIC: STS_PREQ2\n");
		if (hw_state & STS_PREQ3)
			BOARD_DPUTS("PMIC: STS_PREQ3\n");
		PUTS("\n");
	}
#endif

	charger_stat = twl6030_readb(TWL6030_MODULE_CHARGER, CONTROLLER_STAT1);
#ifdef CONFIG_BOARD_VERBOSE_DEBUG
	if (charger_stat) {
		if (charger_stat & VAC_DET)
			BOARD_DPUTS("PMIC: VAC_DET\n");
		if (charger_stat & VBUS_DET)
			BOARD_DPUTS("PMIC: VBUS_DET\n");
	}
#endif

	if (board_is_factory_mode()) {
		/* Factory mode */

		if (boot_dev == BOOT_DEVICE_MMC1) {
			PUTS("<FACTORY> <MMC1 boot> < PCBA >\n");
			coldreset_reason = ABOOT_REASON_PCBA;
		} else if (boot_dev == BOOT_DEVICE_USB) {
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
			PUTS("<FACTORY> <USB boot> < PCBA >\n");
			coldreset_reason = ABOOT_REASON_PCBA;
#else
			PUTS("<FACTORY> <USB boot> < FASTBOOT >\n");
			coldreset_reason = ABOOT_REASON_FASTBOOT;
#endif
		} else if ((row_bits & CONFIG_SYS_KEYPAD_ROW_MASKS) == (CONFIG_ABOOT_FASTBOOT_ROWBITS & CONFIG_SYS_KEYPAD_ROW_MASKS) &&
				(col_bits & CONFIG_SYS_KEYPAD_COL_MASKS) == (CONFIG_ABOOT_FASTBOOT_COLBITS & CONFIG_SYS_KEYPAD_COL_MASKS)) {
			/* Fastboot: don't check the Power Button */
			/* Make sure we got the Fastboot keys */
			PUTS("<FACTORY> < FASTBOOT >\n");
			coldreset_reason = ABOOT_REASON_FASTBOOT;
		} else if ((row_bits & CONFIG_SYS_KEYPAD_ROW_MASKS) == (CONFIG_ABOOT_RECOVERY_ROWBITS & CONFIG_SYS_KEYPAD_ROW_MASKS) &&
				(col_bits & CONFIG_SYS_KEYPAD_COL_MASKS) == (CONFIG_ABOOT_RECOVERY_COLBITS & CONFIG_SYS_KEYPAD_COL_MASKS)) {
			/* Recovery: don't check the Power Button */
			/* Make sure we got the Recovery keys */
			PUTS("<FACTORY> < RECOVERY >\n");
			coldreset_reason = ABOOT_REASON_RECOVERY;
		} else if (last_turnoff_state & DEVOFF_LPK) {
			PUTS("<FACTORY> <LPK> < NORMAL >\n");
			coldreset_reason = ABOOT_REASON_NORMAL;
		} else {
			PUTS("<FACTORY> < NORMAL >\n");
			coldreset_reason = ABOOT_REASON_NORMAL;
		}

	} else if (boot_dev == BOOT_DEVICE_USB) {
		/* USB booting */

		PUTS("<USB boot> < FASTBOOT >\n");
		coldreset_reason = ABOOT_REASON_FASTBOOT;

	} else if (row_bits & CONFIG_KEYPAD_PWRBUTTON_ROW) {
		/* Power Button is pressed */

		if (row_bits == CONFIG_ABOOT_FASTBOOT_ROWBITS && col_bits == CONFIG_ABOOT_FASTBOOT_COLBITS) {
			/* Fastboot */
			if (last_turnoff_state & DEVOFF_LPK)
				PUTS("<LPK> < FASTBOOT >\n");
			else
				PUTS("<PWRBTN> < FASTBOOT >\n");
			coldreset_reason = ABOOT_REASON_FASTBOOT;
		} else if (row_bits == CONFIG_ABOOT_RECOVERY_ROWBITS && col_bits == CONFIG_ABOOT_RECOVERY_COLBITS) {
			/* Recovery */
			if (last_turnoff_state & DEVOFF_LPK)
				PUTS("<LPK> < RECOVERY >\n");
			else
				PUTS("<PWRBTN> < RECOVERY >\n");
			coldreset_reason = ABOOT_REASON_RECOVERY;
		} else if (last_turnoff_state & DEVOFF_LPK) {
			coldreset_reason = board_get_lpk_reason(charger_stat);
		} else {
			PUTS("<PWRBTN> < NORMAL >\n");
			coldreset_reason = ABOOT_REASON_NORMAL;
		}

	} else {
		/* Power Button is NOT pressed */

		if (
#ifndef CONFIG_ABOOT_NO_PWRBTN_FASTBOOT_NO_VAC_VBUS_CHECK
			(charger_stat & (VBUS_DET | VAC_DET)) &&
#endif
				row_bits == (CONFIG_ABOOT_FASTBOOT_ROWBITS & CONFIG_SYS_KEYPAD_ROW_MASKS) &&
				col_bits == (CONFIG_ABOOT_FASTBOOT_COLBITS & CONFIG_SYS_KEYPAD_COL_MASKS)) {
			/* Fastboot: don't check the Power Button */
			/* Make sure we got the AC/USB charger & Fastboot keys */
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
			if (last_turnoff_state & DEVOFF_LPK)
				PUTS("<LPK> <NO Main Battery> < FASTBOOT >\n");
			else
				PUTS("<No PWRBTN> <NO Main Battery> < FASTBOOT >\n");
#else
			if (last_turnoff_state & DEVOFF_LPK)
				PUTS("<LPK> <AC/USB> <No PWRBTN> < FASTBOOT >\n");
			else
				PUTS("<AC/USB> <No PWRBTN> < FASTBOOT >\n");
#endif
			coldreset_reason = ABOOT_REASON_FASTBOOT;
		} else if (
#ifndef CONFIG_ABOOT_NO_PWRBTN_RECOVERY_NO_VAC_VBUS_CHECK
			(charger_stat & (VBUS_DET | VAC_DET)) &&
#endif
				row_bits == (CONFIG_ABOOT_RECOVERY_ROWBITS & CONFIG_SYS_KEYPAD_ROW_MASKS) &&
				col_bits == (CONFIG_ABOOT_RECOVERY_COLBITS & CONFIG_SYS_KEYPAD_COL_MASKS)) {
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
			if (last_turnoff_state & DEVOFF_LPK)
				PUTS("<LPK> <NO Main Battery> < RECOVERY >\n");
			else
				PUTS("<No PWRBTN> <NO Main Battery> < RECOVERY >\n");
#else
			if (last_turnoff_state & DEVOFF_LPK)
				PUTS("<LPK> <AC/USB> <No PWRBTN> < RECOVERY >\n");
			else
				PUTS("<AC/USB> <No PWRBTN> < RECOVERY >\n");
#endif
			coldreset_reason = ABOOT_REASON_RECOVERY;
		} else if (last_turnoff_state & DEVOFF_LPK) {
			coldreset_reason = board_get_lpk_reason(charger_stat);

#ifdef CONFIG_ABOOT_SUPPORT_POWERON_ALARM
		} else if (start_condition & STRT_ON_RTC) {
			PUTS("<RTC ALARM>\n");
			coldreset_reason = ABOOT_REASON_ALARM;
#endif

#if defined(CONFIG_ABOOT_NO_USB_OTG_POWERON)
		} else if (start_condition & STRT_ON_USB_ID) {
			PUTS("<USB OTG> <NOT ALLOWED>\nPlease press POWER button to boot!\n\n");
			coldreset_reason = ABOOT_REASON_POWEROFF;
#endif

		} else if (start_condition & STRT_ON_PLUG_DET) {
#ifdef CONFIG_ABOOT_NO_CHARGER_POWERON

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY
			if (charger_stat & VAC_DET)
				PUTS("<VAC POWER>");
			if (charger_stat & VBUS_DET)
				PUTS("<USB CHARGER>");
			PUTS(" <No PWRBTN> <NOT ALLOWED> <NO Main Battery>\nPlease press POWER button to boot!\n\n");
#else
			if (charger_stat & VAC_DET)
				PUTS("<AC CHARGER>");
			if (charger_stat & VBUS_DET)
				PUTS("<USB CHARGER>");
			PUTS(" <No PWRBTN> <NOT ALLOWED>\nPlease press POWER button to boot!\n\n");
#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

			coldreset_reason = ABOOT_REASON_POWEROFF;

#else /* !CONFIG_ABOOT_NO_CHARGER_POWERON */

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

#ifdef CONFIG_ABOOT_NO_MAIN_BATTERY_VAC_POWERON
			if (charger_stat & VAC_DET) {
				PUTS("<VAC POWER> <No PWRBTN> <NO Main Battery> < NORMAL >\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
			} else
#endif
			{
#if defined(CONFIG_TWL6030_PWRBUTTON_TURNON)

#if defined(CONFIG_TWL6030_PWRBUTTON_STRT_ON_PWRON)
				if (start_condition & STRT_ON_PWRON) {
					PUTS("<CHARGER> <STRT_ON_PWRON> <NO Main Battery> < NORMAL >\n");
					coldreset_reason = ABOOT_REASON_NORMAL;
				} else
#endif
				{
					PUTS("<CHARGER> <NOT ALLOWED> <NO Main Battery>\nPlease press POWER button to boot!\n\n");
					coldreset_reason = ABOOT_REASON_POWEROFF;
				}

#else /* !CONFIG_TWL6030_PWRBUTTON_TURNON */
				PUTS("<CHARGER> <No PWRBTN> <NO Main Battery> < NORMAL >\n\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_TWL6030_PWRBUTTON_TURNON */
			}

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

			if (charger_stat & (VAC_DET | VBUS_DET)) {
				/* AC/USB charger */
				if ((charger_stat & VAC_DET) && power_supply_has_ac_feature() &&
						power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS) {
					PUTS("<AC> <CHARGER>\n");
					coldreset_reason = ABOOT_REASON_CHARGER;
				} else if (charger_stat & VBUS_DET) {
					if (power_supply_has_usb_feature()) {
						if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) {
							PUTS("<NOT USB AC> <CHARGER>\n");
#if defined(CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER)
#if defined(CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON)
							PUTS("\n<NOT ALLOWED> Power-off charging only supports AC charger!\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							coldreset_reason = ABOOT_REASON_POWEROFF;
#else
							PUTS("<No PWRBTN> < NORMAL >\n");
							coldreset_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON */
#else
							coldreset_reason = ABOOT_REASON_PC_CHARGER;
#endif /* CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER */
						} else if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
							PUTS("<USB AC> <CHARGER>\n");
#if defined(CONFIG_ABOOT_NO_POWEROFF_USB_AC_CHARGER)
#if defined(CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON)
							PUTS("\n<NOT ALLOWED> Power-off charging only supports External AC charger!\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							coldreset_reason = ABOOT_REASON_POWEROFF;
#else
							PUTS("<No PWRBTN> < NORMAL >\n");
							coldreset_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */
#else
							coldreset_reason = ABOOT_REASON_CHARGER;
#endif /* CONFIG_ABOOT_NO_POWEROFF_USB_AC_CHARGER */
						} else {
							PUTS("<USB> <No CHARGER> <NOT ALLOWED> Please press POWER button to boot!\n\n");
							coldreset_reason = ABOOT_REASON_POWEROFF;
						}
					} else {
						if (power_supply_get_usb_type() == POWER_SUPPLY_TYPE_USB) {
							PUTS("<NOT USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON)
							PUTS("\n<NOT ALLOWED> Power-off charging only supports AC charger!\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							coldreset_reason = ABOOT_REASON_POWEROFF;
#else
							PUTS("<No PWRBTN> < NORMAL >\n");
							coldreset_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_PC_CHARGER_POWERON */
						} else if (power_supply_get_usb_type() != POWER_SUPPLY_TYPE_UNKNOWN) {
							PUTS("<USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON)
							PUTS("\n<NOT ALLOWED> Power-off charging only supports External AC charger!\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							coldreset_reason = ABOOT_REASON_POWEROFF;
#else
							PUTS("<No PWRBTN> < NORMAL >\n");
							coldreset_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_ABOOT_NO_USB_AC_CHARGER_POWERON */
						} else {
							PUTS("<USB> < No CHARGER > <NOT ALLOWED> Please press POWER button to boot!\n\n");
							coldreset_reason = ABOOT_REASON_POWEROFF;
						}
					}
				} else {
					PUTS("<AC> <CHARGER> <NOT ALLOWED> Please press POWER button to boot!\n\n");
					coldreset_reason = ABOOT_REASON_POWEROFF;
				}
			} else {
#if defined(CONFIG_TWL6030_PWRBUTTON_TURNON)

#if defined(CONFIG_TWL6030_PWRBUTTON_STRT_ON_PWRON)
				if (start_condition & STRT_ON_PWRON) {
					PUTS("<No CHARGER> <STRT_ON_PWRON> < NORMAL >\n");
					coldreset_reason = ABOOT_REASON_NORMAL;
				} else
#endif
				{
					PUTS("\n<No CHARGER> Please press POWER button to boot!\n\n");
					coldreset_reason = ABOOT_REASON_POWEROFF;
				}

#else /* !CONFIG_TWL6030_PWRBUTTON_TURNON */
				PUTS("<No CHARGER> <No PWRBTN> < NORMAL >\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
#endif /* CONFIG_TWL6030_PWRBUTTON_TURNON */
			}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

#endif /* CONFIG_ABOOT_NO_CHARGER_POWERON */

		} else {
			if (start_condition & STRT_ON_RTC)
				PUTS("<RTC ALARM>\n");
			if (start_condition & STRT_ON_USB_ID)
				PUTS("<USB OTG>\n");

#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

#if defined(CONFIG_TWL6030_PWRBUTTON_TURNON)

#if defined(CONFIG_TWL6030_PWRBUTTON_STRT_ON_PWRON)
			if (start_condition & STRT_ON_PWRON) {
				PUTS("<STRT_ON_PWRON> <NO Main Battery> < NORMAL >\n\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
			} else
#endif
			{
				PUTS("<NO Main Battery>\nPlease press POWER button to boot!\n\n");
				coldreset_reason = ABOOT_REASON_POWEROFF;
			}

#else /* !CONFIG_TWL6030_PWRBUTTON_TURNON */

#if defined(CONFIG_ABOOT_NO_USB_OTG_POWERON)
			if (start_condition & STRT_ON_USB_ID) {
				PUTS("<USB OTG> <NOT ALLOWED>\nPlease press POWER button to boot!\n\n");
				coldreset_reason = ABOOT_REASON_POWEROFF;
			} else
#endif
			{
				PUTS("<No PWRBTN> <NO Main Battery> < NORMAL >\n\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
			}

#endif /* CONFIG_TWL6030_PWRBUTTON_TURNON */

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

#if defined(CONFIG_TWL6030_PWRBUTTON_TURNON)

#if defined(CONFIG_TWL6030_PWRBUTTON_STRT_ON_PWRON)
			if (start_condition & STRT_ON_PWRON) {
				PUTS("<STRT_ON_PWRON> < NORMAL >\n\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
			} else
#endif
			{
				PUTS("\nPlease press POWER button to boot!\n\n");
				coldreset_reason = ABOOT_REASON_POWEROFF;
			}

#else /* !CONFIG_TWL6030_PWRBUTTON_TURNON */

#if defined(CONFIG_ABOOT_NO_USB_OTG_POWERON)
			if (start_condition & STRT_ON_USB_ID) {
				PUTS("<USB OTG> <NOT ALLOWED>\nPlease press POWER button to boot!\n\n");
				coldreset_reason = ABOOT_REASON_POWEROFF;
			} else
#endif
			{
				PUTS("<No PWRBTN> < NORMAL >\n");
				coldreset_reason = ABOOT_REASON_NORMAL;
			}

#endif /* CONFIG_TWL6030_PWRBUTTON_TURNON */

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
		}
	}

#ifdef CONFIG_ABOOT_POWERON_HAS_VAC_CHECK
	if (board_is_factory_mode()) {
		PUTS("<FACTORY> <ALLOWED> skip to check VAC_DET\n");
	} else if (charger_stat & VAC_DET) {
		PUTS("<VAC_DET> <ALLOWED>\n");
	} else {
		if (charger_stat & VBUS_DET)
			PUTS("\n<VBUS_DET> <no VAC_DET>");
		else
			PUTS("\n<no VAC_DET>");
		PUTS(" <NOT ALLOWED> Please plugin External AC charger!\n\n");
		coldreset_reason = ABOOT_REASON_POWEROFF;
	}
#endif /* CONFIG_ABOOT_POWERON_HAS_VAC_CHECK */

	return coldreset_reason;
}

static u32 board_get_warmreset_reason(u32 prm_reset_status, u8 start_condition)
{
	u32 warmreset_reason = ABOOT_REASON_NORMAL;
	u8 charger_stat;
	u32 row_bits = 0, col_bits = 0;

#ifdef CONFIG_ABOOT_NO_CPU_WATCHDOG_RESET_POWERON
	if (prm_reset_status & MPU_WDT_RST == MPU_WDT_RST) {
		PUTS("\nOMAP Watchdog Timer Reset!\nPlease press POWER button to boot!\n\n");
		return ABOOT_REASON_POWEROFF;
	}
#endif /* CONFIG_ABOOT_NO_CPU_WATCHDOG_RESET_POWERON */

	charger_stat = twl6030_readb(TWL6030_MODULE_CHARGER, CONTROLLER_STAT1);
#ifdef CONFIG_BOARD_VERBOSE_DEBUG
	if (charger_stat) {
		if (charger_stat & VAC_DET)
			BOARD_DPUTS("PMIC: VAC_DET\n");
		if (charger_stat & VBUS_DET)
			BOARD_DPUTS("PMIC: VBUS_DET\n");
	}
#endif /* CONFIG_BOARD_VERBOSE_DEBUG */

	if (charger_stat & (VBUS_DET | VAC_DET)) {
		keypad_poll(&row_bits, &col_bits);
		PRINT("<<WARM>> <AC/USB> PWRBTN: %s, ROW: 0x%03X, COL: 0x%03X\n",
				(row_bits & CONFIG_KEYPAD_PWRBUTTON_ROW) ? "Yes" : "No",
				row_bits, col_bits);

		/* Fastboot: don't check the Power Button */
		if ((row_bits & CONFIG_SYS_KEYPAD_ROW_MASKS) == (CONFIG_ABOOT_FASTBOOT_ROWBITS & CONFIG_SYS_KEYPAD_ROW_MASKS) &&
				(col_bits & CONFIG_SYS_KEYPAD_COL_MASKS) == (CONFIG_ABOOT_FASTBOOT_COLBITS & CONFIG_SYS_KEYPAD_COL_MASKS)) {
			if (board_is_factory_mode() && omap_boot_device() == BOOT_DEVICE_MMC1) {
				PUTS("<FACTORY> <MMC1 boot> <AC/USB> < PCBA >\n");
				return ABOOT_REASON_PCBA;
			} else {
				/* Make sure we got the USB charger & Fastboot keys */
				PUTS("<AC/USB> < FASTBOOT >\n");
				return ABOOT_REASON_FASTBOOT;
			}
		}
	}

	sar_public->reboot_reason[OMAP_REBOOT_REASON_SIZE] = '\0';
	PRINT("<<WARM>> REASON '%s'\n", (char*)&sar_public->reboot_reason);

	if (!memcmp(&sar_public->reboot_reason, "bootloader", 10)) {
		PUTS("< FASTBOOT >\n");
		warmreset_reason = ABOOT_REASON_FASTBOOT;
	} else if (!memcmp(&sar_public->reboot_reason, "pcba", 4) && board_is_factory_mode()) {
		PUTS("< PCBA >\n");
		warmreset_reason = ABOOT_REASON_PCBA;
	} else if (!memcmp(&sar_public->reboot_reason, "recovery", 8)) {
		PUTS("< RECOVERY >\n");
		warmreset_reason = ABOOT_REASON_RECOVERY;
	} else if (!memcmp(&sar_public->reboot_reason, "charger", 7)) {
		if (!(charger_stat & (VBUS_DET | VAC_DET))) {
			keypad_poll(&row_bits, &col_bits);
			PRINT("<<WARM>> PWRBTN: %s, ROW: 0x%03X, COL: 0x%03X\n",
					(row_bits & CONFIG_KEYPAD_PWRBUTTON_ROW) ? "Yes" : "No",
					row_bits, col_bits);
		}

		if (row_bits & CONFIG_KEYPAD_PWRBUTTON_ROW) {
			/* Power Button is pressed */
			PUTS("<PWRBTN> < NORMAL >\n");
#ifdef CONFIG_ABOOT_SUPPORT_POWERON_ALARM
		} else if (start_condition & STRT_ON_RTC) {
			PUTS("< ALARM >\n");
			warmreset_reason = ABOOT_REASON_ALARM;
#endif
		} else {
			if (charger_stat & (VAC_DET | VBUS_DET)) {
#ifdef CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY

				PUTS("< CHARGER > <NOT ALLOWED> <NO Main Battery>\nPlease press POWER button to boot!\n\n");
				warmreset_reason = ABOOT_REASON_POWEROFF;

#else /* !CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */

				if ((charger_stat & VAC_DET) && power_supply_has_ac_feature() &&
						power_supply_get_charger_source() == POWER_SUPPLY_TYPE_MAINS) {
					PUTS("<AC> < CHARGER >\n");
					warmreset_reason = ABOOT_REASON_CHARGER;
				} else if (charger_stat & VBUS_DET) {
					if (power_supply_has_usb_feature()) {
						if (power_supply_get_charger_source() == POWER_SUPPLY_TYPE_USB) {
							PUTS("<NOT USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_HAS_WARM_RESET_POWEROFF_USB_PC_CHARGER) || !defined(CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER)
							warmreset_reason = ABOOT_REASON_PC_CHARGER;
#else
							PUTS("\n<NOT ALLOWED> Power-off charging only supports AC charger!\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							warmreset_reason = ABOOT_REASON_POWEROFF;
#endif /* CONFIG_ABOOT_HAS_WARM_RESET_POWEROFF_USB_PC_CHARGER || !CONFIG_ABOOT_NO_POWEROFF_USB_PC_CHARGER */
						} else if (power_supply_get_charger_source() != POWER_SUPPLY_TYPE_UNKNOWN) {
							PUTS("<USB AC> < CHARGER >\n");
#if defined(CONFIG_ABOOT_NO_POWEROFF_USB_AC_CHARGER)
							PUTS("\n<NOT ALLOWED> Power-off charging only supports External AC charger!\n");
							PUTS("\nPlease press POWER button to boot!\n\n");
							warmreset_reason = ABOOT_REASON_POWEROFF;
#else
							warmreset_reason = ABOOT_REASON_CHARGER;
#endif /* CONFIG_ABOOT_NO_POWEROFF_USB_AC_CHARGER */
						} else {
							PUTS("<USB> < No CHARGER > <NOT ALLOWED> Please press POWER button to boot!\n\n");
							warmreset_reason = ABOOT_REASON_POWEROFF;
						}
					} else {
						PUTS("<USB> < CHARGER >\n");
						PUTS("\n<NOT ALLOWED> Power-off charging only supports External AC charger!\n");
						PUTS("\nPlease press POWER button to boot!\n\n");
						warmreset_reason = ABOOT_REASON_POWEROFF;
					}
				} else {
					PUTS("<AC> < CHARGER > <NOT ALLOWED> Please press POWER button to boot!\n\n");
					warmreset_reason = ABOOT_REASON_POWEROFF;
				}

#endif /* CONFIG_POWER_SUPPLY_HAS_NO_MAIN_BATTERY */
			} else if (board_is_factory_mode()) {
				PUTS("<No CHARGER> <FACTORY> < NORMAL >\n");
			} else {
				PUTS("\nCharger was unplugged!\nPlease press POWER button to boot!\n\n");
				warmreset_reason = ABOOT_REASON_POWEROFF;
			}
		}
#ifdef CONFIG_ABOOT_SUPPORT_POWERON_ALARM
	} else if (!memcmp(&sar_public->reboot_reason, "alarm", 5)) {
		PUTS("< ALARM >\n");
		warmreset_reason = ABOOT_REASON_ALARM;
#endif
	} else {
		PUTS("< NORMAL >\n");
	}

#ifdef CONFIG_ABOOT_POWERON_HAS_VAC_CHECK
	if (board_is_factory_mode()) {
		PUTS("<FACTORY> <ALLOWED> skip to check VAC_DET\n");
	} else if (charger_stat & VAC_DET) {
		PUTS("<VAC_DET> <ALLOWED>\n");
	} else {
		if (charger_stat & VBUS_DET)
			PUTS("\n<VBUS_DET> <no VAC_DET>");
		else
			PUTS("\n<no VAC_DET>");
		PUTS(" <NOT ALLOWED> Please plugin External AC charger!\n\n");
		warmreset_reason = ABOOT_REASON_POWEROFF;
	}
#endif /* CONFIG_ABOOT_POWERON_HAS_VAC_CHECK */

	return warmreset_reason;
}

/*-------------------------------------------------------------------------*/

u32 aboot_board_get_poweron_reason(int *is_cold_reset)
{
	u32 bootreason;
	u32 prm_reset_status;
	u8 start_condition;

	UART_FIFO_FLUSH_ONCE();

	/* Enter the TWL6030 unsecure mode */
	twl6030_unsecure_mode();

	omap_reset_status = board_get_cpu_reset();
	prm_reset_status = omap_reset_status & RPM_WARMRESET_MASK;
	pmic_start_condition = board_get_pmic_start_condition();

	if (!board_has_rtc_poweron_alarm())
		start_condition = pmic_start_condition & ~STRT_ON_RTC;
	else
		start_condition = pmic_start_condition;

	if (prm_reset_status) {
		*is_cold_reset = 0;
		bootreason = board_get_warmreset_reason(prm_reset_status, start_condition);
	} else {
		*is_cold_reset = 1;
		bootreason = board_get_coldreset_reason(start_condition);
	}

	/* Enter the TWL6030 secure mode */
	twl6030_secure_mode();

#if !defined(CONFIG_SPL) && defined(CONFIG_ABOOT_ALWAYS_ENTER_FASTBOOT)
	/* Always enter Fastboot mode */
#ifdef CONFIG_ABOOT_PCBA_USE_FASTBOOT
	PUTS("<< Force PCBA >>\n");
	bootreason = ABOOT_REASON_PCBA;
#else
	PUTS("<< Force FASTBOOT >>\n");
	bootreason = ABOOT_REASON_FASTBOOT;
#endif
#endif /* !CONFIG_SPL && CONFIG_ABOOT_ALWAYS_ENTER_FASTBOOT */

	UART_FIFO_FLUSH_ONCE();

	return bootreason;
}

/*-------------------------------------------------------------------------*/

/**
 * Return the Bootloader unlocked status of the device
 */
int aboot_board_is_unlocked(void)
{
#ifdef CONFIG_BOARD_IS_ALWAYS_UNLOCKED
	return 1;
#else
	return board_is_factory_mode();
#endif
}

/*-------------------------------------------------------------------------*/

/**
 * Return the serial number of the device
 */
const char *aboot_board_get_serialno(void)
{
	/* Board serial number */
	if (board_serialno[0] == '\0') {
		char* env_serialno = NULL;
		struct omap_sys_ctrl_regs *const ctrl =
				(struct omap_sys_ctrl_regs *)SYSCTRL_GENERAL_CORE_BASE;

		board_sys_serial_low = readl(&ctrl->control_std_fuse_die_id_0);
		board_sys_serial_high = readl(&ctrl->control_std_fuse_die_id_1);

		env_serialno = getenv("serialno");
		if (env_serialno && env_serialno[0] != '\0') {
			strncpy(board_serialno, env_serialno, 31);
		} else {
			if (board_is_factory_mode()) {
				/* Apply the fixed serial number when the factory tool is detected */
				board_sys_serial_low	= 0x89ABCDEF;
				board_sys_serial_high	= 0x01234567;
				BOARD_PUTS("use hard-coded number as the serial#\n");
			} else {
				BOARD_PUTS("use die# as the serial#\n");
			}
			sprintf(board_serialno, "%08X%08X",
					board_sys_serial_high, board_sys_serial_low);
		}
	}

	return board_serialno;
}

#ifdef CONFIG_SERIAL_TAG
void get_board_serial(struct tag_serialnr *serialnr)
{
	serialnr->low = board_sys_serial_low;
	serialnr->high = board_sys_serial_high;
}
#endif /* CONFIG_SERIAL_TAG */

/*-------------------------------------------------------------------------*/

static inline void __board_shutdown_drivers(void) {}
void board_shutdown_drivers(void) __attribute__((weak, alias("__board_shutdown_drivers")));

/**
 * @brief board_poweroff
 */
void board_poweroff(void)
{
	UART_FIFO_FLUSH_ONCE();

	aboot_set_state(ABOOT_STATE_POWEROFF);

#ifdef CONFIG_TWL6030_PWM
	twl6030_pwm_shutdown();
#endif /* CONFIG_TWL6030_PWM */

#ifdef CONFIG_OMAP2_DSS
	/* Shutting down the display after the LCD backlight is shutted down. */
	omap_display_shutdown();
#endif /* CONFIG_OMAP2_DSS */

	board_shutdown_drivers();

	/* Shutdown MUSB */
#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
	musb_shutdown();
#endif /* CONFIG_USB_GADGET_MUSB_HDRC */

	/* Shutdown Platform Dependent Power */
	power_shutdown();

	UART_FIFO_FLUSH_ONCE();

	/* Clear PRM reset sources */
	writel(0x0FFF, &prcm->prm_rstst);

	disable_interrupts();

	/* power supply shutdown */
	power_supply_shutdown();

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN
	i2c_omap_shutdown();
#endif /* CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN */

	PUTS("\nPower down.\n\n");

	udelay(50000);				/* wait 50 ms */

#ifdef CONFIG_TWL6030_POWER
	twl6030_poweroff();
#else /* CONFIG_TWL6030_POWER */
	puts("Please define CONFIG_TWL6030_POWER\n\n");
	for (;;);
#endif /* CONFIG_TWL6030_POWER */
}

/**
 * @brief board_reboot
 */
void board_reboot(char *reason)
{
	UART_FIFO_FLUSH_ONCE();

	aboot_set_state(ABOOT_STATE_REBOOT);

#ifdef CONFIG_TWL6030_PWM
	twl6030_pwm_shutdown();
#endif /* CONFIG_TWL6030_PWM */

#ifdef CONFIG_OMAP2_DSS
	/* Shutting down the display after the LCD backlight is shutted down. */
	omap_display_shutdown();
#endif /* CONFIG_OMAP2_DSS */

	board_shutdown_drivers();

	/* Shutdown MUSB */
#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
	musb_shutdown();
#endif /* CONFIG_USB_GADGET_MUSB_HDRC */

	/* Shutdown Platform Dependent Power */
	power_shutdown();

#ifdef CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN
	i2c_omap_shutdown();
#endif /* CONFIG_SYS_I2C_OMAP_BOARD_SHUTDOWN */

	UART_FIFO_FLUSH_ONCE();

	disable_interrupts();

	if (reason && *reason != '\0') {
		PRINT("\nRestarting system with reason '%s'.\n\n", reason);
		strncpy((char*)&sar_public->reboot_reason, reason, OMAP_REBOOT_REASON_SIZE);
		sar_public->reboot_reason[OMAP_REBOOT_REASON_SIZE] = '\0';
	} else {
		PUTS("\nRestarting system.\n\n");
		sar_public->reboot_reason[0] = '\0';
	}

	udelay(50000);				/* wait 50 ms */

	reset_cpu(0);
}

/*-------------------------------------------------------------------------*/

u32 innocomm_board_identify(void)
{
	u32 revision = 0;
	int value;

#define BOARD_REV_GPIO_1		101 /* HW_VER_S1 */
#define BOARD_REV_GPIO_1_SHIFT	0
#define BOARD_REV_GPIO_2		102 /* HW_VER_S2 */
#define BOARD_REV_GPIO_2_SHIFT	1
#define BOARD_REV_GPIO_3		103 /* HW_VER_S3 */
#define BOARD_REV_GPIO_3_SHIFT	2

	/* Pull pins up */
	writew((IEN | PTU | M3), CONTROL_PADCONF_CORE + C2C_DATA12/*GPMC_NCS4*/);
	writew((IEN | PTU | M3), CONTROL_PADCONF_CORE + C2C_DATA13/*GPMC_NCS5*/);
	writew((IEN | PTU | M3), CONTROL_PADCONF_CORE + C2C_DATA14/*GPMC_NCS6*/);
	sdelay(1200); /* 1us (Enough loops assuming a maximum of 1.1GHz) */

	/* HW_VER_S1 */
	if (!gpio_request(BOARD_REV_GPIO_1, NULL)) {
		gpio_direction_input(BOARD_REV_GPIO_1);
		value = gpio_get_value(BOARD_REV_GPIO_1);
		gpio_free(BOARD_REV_GPIO_1);
		revision |= (value << BOARD_REV_GPIO_1_SHIFT);
	}

	/* HW_VER_S2 */
	if (!gpio_request(BOARD_REV_GPIO_2, NULL)) {
		gpio_direction_input(BOARD_REV_GPIO_2);
		value = gpio_get_value(BOARD_REV_GPIO_2);
		gpio_free(BOARD_REV_GPIO_2);
		revision |= (value << BOARD_REV_GPIO_2_SHIFT);
	}

	/* HW_VER_S3 */
	if (!gpio_request(BOARD_REV_GPIO_3, NULL)) {
		gpio_direction_input(BOARD_REV_GPIO_3);
		value = gpio_get_value(BOARD_REV_GPIO_3);
		gpio_free(BOARD_REV_GPIO_3);
		revision |= (value << BOARD_REV_GPIO_3_SHIFT);
	}

	/* Pull pins down and switch to the safe mode */
	writew((IEN | PTD | M7), CONTROL_PADCONF_CORE + C2C_DATA12/*GPMC_NCS4*/);
	writew((IEN | PTD | M7), CONTROL_PADCONF_CORE + C2C_DATA13/*GPMC_NCS5*/);
	writew((IEN | PTD | M7), CONTROL_PADCONF_CORE + C2C_DATA14/*GPMC_NCS6*/);

	return revision;
}

/**
 * @brief innocomm_board_early_init_f
 *
 * James Wu: triggered by board_init_f()
 *  1.	BSS is not clear
 *  2.	UART console is NOT ready
 *  3.	timer is NOT ready
 */
void innocomm_board_early_init_f(void)
{
}

/**
 * @brief innocomm_board_init - Common platform dependent initialisations
 *
 * James Wu:
 *  1.	UART console is ready
 *  2.	timer is ready
 *  3.  before mem_malloc_init()
 *  4.  before interrupt_init()
 */
void innocomm_board_init(void)
{
#ifndef CONFIG_DISPLAY_BOARDINFO
	(void)checkboard();
#endif /* !CONFIG_DISPLAY_BOARDINFO */

	if (board_is_factory_mode())
		aboot_set_mode(ABOOT_MODE_FACTORY);
	else
		aboot_set_mode(ABOOT_MODE_NORMAL);

	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* Linux ATAGs address */

	gpmc_init();

	/* Platform Dependent Power Initialisations */
	power_init();

#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
{
	int ret = musb_init();
	if (ret)
		BOARD_ERR("init musb err %d\n", ret);
}
#endif /* CONFIG_USB_GADGET_MUSB_HDRC */
}

/**
 * @brief misc_init_r - Common misc platform dependent initialisations
 *
 * James Wu:
 *  1.  after mem_malloc_init()
 *  2.	after env_relocate()
 *  3.  before interrupt_init()
 *  4.	before board_late_init()
 */
void innocomm_board_misc_init(void)
{
	const char* serialno = aboot_board_get_serialno();

	PRINT(CONFIG_SYS_BOARD_NAME " serial# %s\n\n", serialno);
}

/**
 * @brief innocomm_board_late_init - Common late platform dependent initialisations
 *
 * James Wu:
 *  1.  after mem_malloc_init()
 *  2.  after interrupt_init()
 */
void innocomm_board_late_init(void)
{
	aboot_init();
}

/**
 * @brief innocomm_board_preboot_os - Common board-specific preboot before Linux
 */
void innocomm_board_preboot_os(void)
{
	/* Shutdown MUSB */
#if defined(CONFIG_USB_GADGET_MUSB_HDRC)
	musb_shutdown();
#endif /* CONFIG_USB_GADGET_MUSB_HDRC */

	/* Shutdown Platform Dependent Power */
	power_shutdown();
}

#define BOARD_CMDLINE_ANDROID_SERIALNO			" androidboot.serialno="
#define BOARD_CMDLINE_ANDROID_CARRIER			" androidboot.carrier="
#define BOARD_CMDLINE_ANDROID_CARRIER_WIFI_ONLY	" androidboot.carrier=wifi-only"
#define BOARD_CMDLINE_ANDROID_WIFIMAC			" androidboot.wifimac="
#define BOARD_CMDLINE_ANDROID_ETHADDR			" androidboot.ethaddr="
#define BOARD_CMDLINE_PMIC_LAST_OFF				" last_off="
#define BOARD_CMDLINE_PMIC_START_CONDITION		" start_con="
#define BOARD_CMDLINE_OMAP_RESET_STATUS			" omap_rst="
#define BOARD_CMDLINE_IBOOT_VERSION				" iboot="

/**
 * @brief innocomm_board_setup_linux_cmdline - Common board-specific Linux commands
 */
size_t innocomm_board_setup_linux_cmdline(char *cmdline)
{
	char* cmdline_start = cmdline;
	size_t length;
	const char* serialno = aboot_board_get_serialno();
	char* env_str;

	/* Common Commands */
#if defined(CONFIG_PANEL_DPI2LVDS) && defined(CONFIG_PANEL_DPI2LVDS_SETUP_CMDLINE)
	/* panel */
	cmdline += omap4_dpi2lvds_setup_linux_cmdline(cmdline);
#endif

	/* androidboot.serialno */
	strcpy(cmdline, BOARD_CMDLINE_ANDROID_SERIALNO);
	cmdline += sizeof(BOARD_CMDLINE_ANDROID_SERIALNO) - 1;
	length = strlen(serialno);
	strncpy(cmdline, serialno, length);
	cmdline += length;

#ifdef CONFIG_BOARD_HAS_CMDLINE_ANDROID_CARRIER
	env_str = getenv("carrier");
	if (env_str && env_str[0] !='\0') {
		/* androidboot.carrier */
		strcpy(cmdline, BOARD_CMDLINE_ANDROID_CARRIER);
		cmdline += sizeof(BOARD_CMDLINE_ANDROID_CARRIER) - 1;
		length = strlen(env_str);
		strncpy(cmdline, env_str, length);
		cmdline += length;
	} else {
		/* androidboot.carrier=wifi-only */
		strcpy(cmdline, BOARD_CMDLINE_ANDROID_CARRIER_WIFI_ONLY);
		cmdline += sizeof(BOARD_CMDLINE_ANDROID_CARRIER_WIFI_ONLY) - 1;
	}
#endif /* CONFIG_BOARD_HAS_CMDLINE_ANDROID_CARRIER */

	env_str = getenv("wifimac");
	if (env_str && env_str[0] != '\0') {
		/* androidboot.wifimac */
		strcpy(cmdline, BOARD_CMDLINE_ANDROID_WIFIMAC);
		cmdline += sizeof(BOARD_CMDLINE_ANDROID_WIFIMAC) - 1;
		length = min(12, strlen(env_str));
		strncpy(cmdline, env_str, length);
		cmdline += length;
	} else {
		BOARD_VPRINT("Use default wifimac\n");
	}

#ifdef CONFIG_BOARD_HAS_CMDLINE_ANDROID_ETHADDR
	env_str = getenv("ethaddr");
	if (env_str && env_str[0] !='\0') {
		/* androidboot.ethaddr */
		strcpy(cmdline, BOARD_CMDLINE_ANDROID_ETHADDR);
		cmdline += sizeof(BOARD_CMDLINE_ANDROID_ETHADDR) - 1;
		length = min(12, strlen(env_str));
		strncpy(cmdline, env_str, length);
		cmdline += length;
	} else {
		BOARD_VPRINT("Use default ethaddr\n");
	}
#endif /* CONFIG_BOARD_HAS_CMDLINE_ANDROID_ETHADDR */

#ifdef CONFIG_BOARD_DEBUG
{
	char buff[10];

	/* last_off */
	strcpy(cmdline, BOARD_CMDLINE_PMIC_LAST_OFF);
	cmdline += sizeof(BOARD_CMDLINE_PMIC_LAST_OFF) - 1;
	length = sprintf(buff, "0x%02X", pmic_last_turnoff);
	strncpy(cmdline, buff, length);
	cmdline += length;

	/* start_con */
	strcpy(cmdline, BOARD_CMDLINE_PMIC_START_CONDITION);
	cmdline += sizeof(BOARD_CMDLINE_PMIC_START_CONDITION) - 1;
	length = sprintf(buff, "0x%02X", pmic_start_condition);
	strncpy(cmdline, buff, length);
	cmdline += length;

	/* omap_rst */
	strcpy(cmdline, BOARD_CMDLINE_OMAP_RESET_STATUS);
	cmdline += sizeof(BOARD_CMDLINE_OMAP_RESET_STATUS) - 1;
	length = sprintf(buff, "0x%02X", omap_reset_status);
	strncpy(cmdline, buff, length);
	cmdline += length;
}
#endif /* CONFIG_BOARD_DEBUG */

	/* "iboot=" must be the last command. */
	strcpy(cmdline, BOARD_CMDLINE_IBOOT_VERSION);
	cmdline += sizeof(BOARD_CMDLINE_IBOOT_VERSION) - 1;
	strcpy(cmdline, PLAIN_VERSION);
	cmdline += sizeof(PLAIN_VERSION) - 1;

	return (size_t)(cmdline - cmdline_start);
}

int checkboard(void)
{
	u32 boot_dev = omap_boot_device();

#if defined(CONFIG_ABOOT_PCBA_WAIT_FOR_FACTORY_TOOL_TIME)
	if (boot_dev == BOOT_DEVICE_MMC1 || boot_dev == BOOT_DEVICE_USB)
		innocomm_board_wait_for_detect_factory_tool(CONFIG_ABOOT_PCBA_WAIT_FOR_FACTORY_TOOL_TIME);
#endif /* CONFIG_ABOOT_PCBA_WAIT_FOR_FACTORY_TOOL_TIME */

	PRINT(CONFIG_SYS_BOARD_NAME " rev: %u%s\n", get_board_rev(),
			(board_is_factory_mode() ? " (JIG_DET)" : ""));

	switch (boot_dev) {
	case BOOT_DEVICE_MMC1:
		PRINT("<Loaded from MMC1>\n");
		break;
	case BOOT_DEVICE_MMC2:
		PRINT("<Loaded from MMC2>\n");
		break;
	case BOOT_DEVICE_USB:
		PRINT("<Loaded from USB>\n");
		break;
	default:
		PRINT("<Loaded from 0x%02X>\n", boot_dev);
		break;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static char proc_version[12];
static char proc_rev[8];
static char proc_type[8];

void innocomm_fastboot_board_init(struct usb_string *strings, struct usb_device_descriptor *desc)
{
	u32 device_type = get_device_type();
	u32 omap_rev = omap_revision();
	u32 omap_variant = (omap_rev & 0xFFFF0000) >> 16;
	u32 major_rev = (omap_rev & 0x00000F00) >> 8;
	u32 minor_rev = (omap_rev & 0x000000F0) >> 4;

	sprintf(proc_version, "OMAP%x", omap_variant);
	sprintf(proc_rev, "ES%x.%x", major_rev, minor_rev);
	sprintf(proc_type, "%s", (device_type == GP_DEVICE) ? "GP" : (device_type == HS_DEVICE) ? "HS" : "EMU");

	strings[FASTBOOT_STR_MANUFACTURER_IDX].s	= "InnoComm";
	strings[FASTBOOT_STR_PRODUCT_IDX].s			= "Android Device";
	strings[FASTBOOT_STR_SERIAL_IDX].s			= aboot_board_get_serialno();
	strings[FASTBOOT_STR_PROC_REV_IDX].s		= proc_rev;
	strings[FASTBOOT_STR_PROC_TYPE_IDX].s		= proc_type;
	strings[FASTBOOT_STR_PROC_VERSION_IDX].s	= proc_version;
}

/*-------------------------------------------------------------------------*/

int fastboot_reset_battery_capacity(void)
{
#if defined(CONFIG_POWER_SUPPLY_HAS_EXTERNAL_FUELGAUGE)
	return -EPERM;
#else
	board_update_battery_change_status(FIRST_SYS_INS, 0);
	return 0;
#endif
}

/*-------------------------------------------------------------------------*/

/* shutdown */
static int do_poweroff(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	board_poweroff();

	/*NOTREACHED*/
	return 0;
}

U_BOOT_CMD(
	shutdown, 1, 0,	do_poweroff,
	"Shutdown the device",
	""
);

#endif /* CONFIG_SPL_BUILD */
