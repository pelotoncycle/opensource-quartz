/*
 * (C) Copyright 2012
 * InnoComm Mobile Technology Corp.
 * Jiahe Jou <jiahe.jou@innocomm.com>
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
#ifndef _BOARD_MUX_DATA_H
#define _BOARD_MUX_DATA_H

#include <asm/arch/mux_omap4.h>

/*-------------------------------------------------------------------------*/
#if defined(CONFIG_SPL_BUILD)

const struct pad_conf_entry core_padconf_array_essential[] = {
	/* eMMC */
	{GPMC_AD0, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat0 */
	{GPMC_AD1, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat1 */
	{GPMC_AD2, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat2 */
	{GPMC_AD3, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat3 */
	{GPMC_AD4, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat4 */
	{GPMC_AD5, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat5 */
	{GPMC_AD6, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat6 */
	{GPMC_AD7, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat7 */
	{GPMC_NOE, (PTU | IEN | OFF_OUT_PD | M1)},				/* sdmmc2_clk */
	{GPMC_NWE, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_cmd */
	/* External SD */
	{SDMMC1_CLK, (PTU | IEN | OFF_OUT_PD | M0)},				/* sdmmc1_clk */
	{SDMMC1_CMD, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_cmd */
	{SDMMC1_DAT0, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat0 */
	{SDMMC1_DAT1, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat1 */
	{SDMMC1_DAT2, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat2 */
	{SDMMC1_DAT3, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat3 */
	{SDMMC1_DAT4, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat4: NC: SafeMode */
	{SDMMC1_DAT5, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat5: NC: SafeMode */
	{SDMMC1_DAT6, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat6: NC: SafeMode */
	{SDMMC1_DAT7, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat7: NC: SafeMode */
	/* I2C1~4 */
	{I2C1_SCL, (PTU | IEN | M0)},						/* i2c1_scl */
	{I2C1_SDA, (PTU | IEN | M0)},						/* i2c1_sda */
	{I2C2_SCL, (PTU | IEN | M0)},						/* i2c2_scl */
	{I2C2_SDA, (PTU | IEN | M0)},						/* i2c2_sda */
	{I2C3_SCL, (PTU | IEN | M0)},						/* i2c3_scl */
	{I2C3_SDA, (PTU | IEN | M0)},						/* i2c3_sda */
	{I2C4_SCL, (PTU | IEN | M0)},						/* i2c4_scl */
	{I2C4_SDA, (PTU | IEN | M0)},						/* i2c4_sda */
#if (CONFIG_CONS_INDEX == 1)
#error "UART1 console is not implemented yet!\n"
#elif (CONFIG_CONS_INDEX == 2)
#error "UART2 console is not implemented yet!\n"
#elif (CONFIG_CONS_INDEX == 3)
	/* OMAP low-level debug UART */
	/* Pull up UART RX pin to avoid the undefined state */
	{UART3_RX_IRRX, (PTU | IEN | M0)},					/* uart3_rx: LOG_UART3_RX */
	{UART3_TX_IRTX, (M0)},							/* uart3_tx: LOG_UART3_TX */
#elif (CONFIG_CONS_INDEX == 4)
#error "UART4 console is not implemented yet!\n"
#endif
	/* Keypad */
	{UNIPRO_TX0/*KPD_COL3*/, (M1)},						/* kpd_col0 */
	{UNIPRO_TY0/*KPD_COL4*/, (M1)},						/* kpd_col1 */
	{UNIPRO_RX0/*KPD_ROW3*/, (PTU | IEN | M1)},				/* kpd_row0 */
	/* SYS_BOOTx */
	{SYS_BOOT0, (IEN | PTD | M0)},						/* SYS_BOOT0: Input */
	{SYS_BOOT1, (IEN | PTU | M0)},						/* SYS_BOOT1: Input */
	{SYS_BOOT2, (IEN | PTU | M0)},						/* SYS_BOOT2: Input */
	{SYS_BOOT3, (IEN | PTD | M0)},						/* SYS_BOOT3: Input */
	{SYS_BOOT4, (IEN | PTU | M0)},						/* SYS_BOOT4: Input */
	{SYS_BOOT5, (IEN | PTU | M0)},						/* SYS_BOOT5: Input */
};

const struct pad_conf_entry wkup_padconf_array_essential[] = {
	/* SmartReflex I2C */
	{PAD1_SR_SCL, (PTU | IEN | M0)},					/* sr_scl */
	{PAD0_SR_SDA, (PTU | IEN | M0)},					/* sr_sda */
	/* SYS_32K */
	{PAD1_SYS_32K, (IEN | M0)},						/* sys_32k */
};

/*-------------------------------------------------------------------------*/

#else /* !CONFIG_SPL_BUILD */

const struct pad_conf_entry core_padconf_array_non_essential[] = {
	{GPMC_AD0, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat0: eMMC_AD0 */
	{GPMC_AD1, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat1: eMMC_AD1 */
	{GPMC_AD2, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat2: eMMC_AD2 */
	{GPMC_AD3, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat3: eMMC_AD3 */
	{GPMC_AD4, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat4: eMMC_AD4 */
	{GPMC_AD5, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat5: eMMC_AD5 */
	{GPMC_AD6, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat6: eMMC_AD6 */
	{GPMC_AD7, (PTU | IEN | OFF_IN_PD | M1)},				/* sdmmc2_dat7: eMMC_AD7 */
	{GPMC_AD8, (PTD | IEN | OFF_IN_PD | M7)},				/* gpmc_ad8: NC: SafeMode */
	{GPMC_AD9, (PTU | IEN | OFF_IN_PU | M3)},				/* gpio_33: Reset high: ETHERNET_EN: with reverse */
	{GPMC_AD10, (PTU | IEN | OFF_IN_PU | M7)},				/* gpio_33: Reset high: ETHERNET_PME */
	{GPMC_AD11, (PTD | IEN | M3)},						/* gpio_35: Reset high: JIG_DET active low */
	{GPMC_AD12, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_36: ETHERNET_RST: active low --> RESET_N*/
	{GPMC_AD13, (PTU | IDIS | OFF_OUT_PU | M3)},				/* gpio_37: VTP_EN_N: active low with reverse */
	{GPMC_AD14, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_38: MAIN_CAM_nSTANDBY active high */
	{GPMC_AD15, (PTD | IEN | M3)},						/* gpio_39: TEMP_IRQ active ? */
	{GPMC_A16, (PTD | IEN | M3)},							/* gpio_40: M_USBB1_RST active low */
	{GPMC_A17, (PTD | IDIS | OFF_OUT_PD | M3)},				/* gpio_41: V5V_EN active high */
	{GPMC_A18, (PTD | IDIS | OFF_OUT_PD | M3)},				/* gpio_42: CAM_2V8_EN active high */
	{GPMC_A19, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_43: LVDS_RST */
	{GPMC_A20, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_44: LVDS_1V2_EN: NM */
	{GPMC_A21, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_45: LVDS_1V8_EN: NM */
	{GPMC_A22, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_46: HW_VER_P0: NM */
	{GPMC_A23, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_47: HW_VER_P1: NM  */
	{GPMC_A24, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_48: HW_VER_T0: NM */
	{GPMC_A25, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_49: HW_VER_T1: NM */
	{GPMC_NCS0, (PTU | IEN | OFF_IN_PU | M7)},				/* gpio_50: Reset high: HUB_RST_N with reverse */
	{GPMC_NCS1, (PTU | IEN | OFF_IN_PU | M3)},				/* gpio_51: Reset high: LCM_5V_EN_N active low with reverse */
	{GPMC_NCS2, (PTU | IEN | OFF_IN_PU | M7)},				/* gpio_52: Reset high: RS232_DET_N with ? external pull up */
	{GPMC_NCS3, (PTU | IEN | M3)},						/* gpio_53: WLAN_IRQ#: active low */
	{GPMC_NWP, (PTD | IDIS | OFF_OUT_PD | M3)},				/* gpio_54: WL_EN active high */
	{GPMC_CLK, (PTD | IDIS | OFF_OUT_PD | M3)},				/* gpio_55: BT_EN active high */
	{GPMC_NADV_ALE, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_56: RS232_SHDN_N with ? external pull down*/
	{GPMC_NOE, (PTU | IEN | OFF_OUT_PD | M1)},	 			/* sdmmc2_clk: eMMC_CLK */
	{GPMC_NWE, (PTU | IEN | OFF_IN_PD | M1)}, 				/* sdmmc2_cmd: eMMC_CMD */
	{GPMC_NBE0_CLE, (PTD | IDIS | OFF_OUT_PD | M3)},			/* gpio_59: RS232_EN */
	{GPMC_NBE1, (PTD | IDIS | OFF_OUT_PD | M3)},				/* gpio_60: MODEM_CLK_EN */
	{GPMC_WAIT0, (PTU | IDIS | OFF_OUT_PU | M3)},				/* gpio_61: Reset high: BL_EN_N active low with reverse */
	{GPMC_WAIT1, (PTU | IEN | OFF_IN_PU | M7)},				/* gpio_62: Reset high: AUDIO_AMP_SD with reverse */
	{C2C_DATA11/*GPMC_WAIT2*/, (PTD | IEN | OFF_IN_PD | M7)},		/* gpio_100: NC: SafeMode */
	{C2C_DATA12/*GPMC_NCS4*/, (PTD | IEN | M3)},				/* gpio_101: HW_VER_S1 */
	{C2C_DATA13/*GPMC_NCS5*/, (PTD | IEN | M3)},				/* gpio_102: HW_VER_S2 */
	{C2C_DATA14/*GPMC_NCS6*/, (PTD | IEN | M3)},				/* gpio_103: HW_VER_S3 */
	{C2C_DATA15/*GPMC_NCS7*/, (PTD | IEN | OFF_IN_PD | M7)},		/* gpio_104: NC: SafeMode */
	{HDMI_HPD, (PTD | IEN | OFF_IN_PD | M7)},				/* hdmi_hpd: NC: SafeMode */
	{HDMI_CEC, (PTD | IEN | OFF_IN_PD | M7)},				/* hdmi_cec: NC: SafeMode */
	{HDMI_DDC_SCL, (PTD | IEN | OFF_IN_PD | M7)},				/* hdmi_ddc_scl: NC: SafeMode */
	{HDMI_DDC_SDA, (PTD | IEN | OFF_IN_PD | M7)},				/* hdmi_ddc_sda: NC: SafeMode */

	/* Camera */
	{CSI21_DX0, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dx0: MAIN_CAM */
	{CSI21_DY0, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dy0: MAIN_CAM */
	{CSI21_DX1, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dx1: MAIN_CAM */
	{CSI21_DY1, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dy1: MAIN_CAM */
	{CSI21_DX2, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dx2: NC: SafeMode */
	{CSI21_DY2, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dy2: NC: SafeMode */
	{CSI21_DX3, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dx3: NC: SafeMode */
	{CSI21_DY3, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dy3: NC: SafeMode */
	{CSI21_DX4, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dx4: NC: SafeMode */
	{CSI21_DY4, (PTD | IEN | OFF_IN_PD | M7)},				/* csi21_dy4: NC: SafeMode */
	{CSI22_DX0, (PTD | IEN | OFF_IN_PD | M7)},				/* csi22_dx0: SUB_CAM */
	{CSI22_DY0, (PTD | IEN | OFF_IN_PD | M7)},				/* csi22_dy0: SUB_CAM */
	{CSI22_DX1, (PTD | IEN | OFF_IN_PD | M7)},				/* csi22_dx1: SUB_CAM */
	{CSI22_DY1, (PTD | IEN | OFF_IN_PD | M7)},				/* csi22_dy1: SUB_CAM */
	{CSI22_DX2, (PTD | IEN | OFF_IN_PD | M7)},				/* csi22_dx2: NC: SafeMode */
	{CSI22_DY2, (PTD | IEN | OFF_IN_PD | M7)},				/* csi22_dy2: NC: SafeMode */
	{CAM_SHUTTER, (PTD | IEN | OFF_IN_PD | M7)},				/* cam_shutter: NC: SafeMode */
	{CAM_STROBE, (PTD | IEN | OFF_IN_PD | M7)},				/* cam_strobe: NC: SafeMode */
	{CAM_GLOBALRESET, (PTD | OFF_OUT_PD | M3)},				/* cam_nreset: JMT-FP24AB D8 */

	/* Modem USB or HSI (not used in Bootloader) */
	{USBB1_ULPITLL_CLK, (PTD | IDIS | M7)},					/* SafeMode: hsi1_cawake */
	{USBB1_ULPITLL_STP, (PTU | IDIS | M7)},					/* SafeMode: hsi1_cadata */
	{USBB1_ULPITLL_DIR, (PTD | IDIS | M7)},					/* SafeMode: hsi1_caflag */
	{USBB1_ULPITLL_NXT, (PTD | IDIS | M7)},					/* SafeMode: hsi1_acready */
	{USBB1_ULPITLL_DAT0, (PTD | IDIS | M7)},				/* SafeMode: hsi1_acwake */
	{USBB1_ULPITLL_DAT1, (PTD | IDIS | M7)},				/* SafeMode: hsi1_acdata */
	{USBB1_ULPITLL_DAT2, (PTD | IDIS | M7)},				/* SafeMode: hsi1_acflag */
	{USBB1_ULPITLL_DAT3, (PTD | IDIS | M7)},				/* SafeMode: hsi1_caready */
	{USBB1_ULPITLL_DAT4, (PTD | IDIS | M7)},				/* SafeMode: usbb1_ulpiphy_dat4 */
	{USBB1_ULPITLL_DAT5, (PTD | IDIS | M7)},				/* SafeMode: usbb1_ulpiphy_dat5 */
	{USBB1_ULPITLL_DAT6, (PTD | IDIS | M7)},				/* SafeMode: usbb1_ulpiphy_dat6 */
	{USBB1_ULPITLL_DAT7, (PTD | IDIS | M7)},				/* SafeMode: usbb1_ulpiphy_dat7 */

	{USBB1_HSIC_DATA, (PTD | IEN | OFF_IN_PD | M7)},			/* usbb1_hsic_data NC: SafeMode */
	{USBB1_HSIC_STROBE, (PTD | IEN | OFF_IN_PD | M7)},			/* usbb1_hsic_strobe NC: SafeMode */

	{USBC1_ICUSB_DP /*GPIO_98*/, (PTD | IEN | OFF_IN_PD | M7)},		/* GPIO_98 NC: SafeMode */
	{USBC1_ICUSB_DM /*GPIO_99*/, (PTD | IEN | OFF_IN_PD | M7)},		/* GPIO_99 NC: SafeMode */

	/* External SD */
	{SDMMC1_CLK, (PTU | IEN | OFF_OUT_PD | M0)},				/* sdmmc1_clk */
	{SDMMC1_CMD, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_cmd */
	{SDMMC1_DAT0, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat0 */
	{SDMMC1_DAT1, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat1 */
	{SDMMC1_DAT2, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat2 */
	{SDMMC1_DAT3, (PTU | IEN | OFF_IN_PD | M0)},				/* sdmmc1_dat3 */

	{SDMMC1_DAT4, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat4: NC: SafeMode */
	{SDMMC1_DAT5, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat5: NC: SafeMode */
	{SDMMC1_DAT6, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat6: NC: SafeMode */
	{SDMMC1_DAT7, (PTD | IEN | OFF_IN_PD | M7)},				/* sdmmc1_dat7: NC: SafeMode */

	{ABE_MCBSP2_CLKX, (PTD | IEN | OFF_IN_PD | M7)},			/* abe_mcbsp2_clkx: NC: SafeMode */
	{ABE_MCBSP2_DR, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_mcbsp2_dr: NC: SafeMode */
	{ABE_MCBSP2_DX, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_mcbsp2_dx: NC: SafeMode */
	{ABE_MCBSP2_FSX, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_mcbsp2_fsx: NC: SafeMode */

	/* BT_PCM */
	{ABE_MCBSP1_CLKX, (PTD | IEN | OFF_IN_PD | M7)},			/* abe_mcbsp1_clkx: BT_PCM_CLK */
	{ABE_MCBSP1_DR, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_mcbsp1_dr: BT_PCM_DR */
	{ABE_MCBSP1_DX, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_mcbsp1_dx: BT_PCM_DX */
	{ABE_MCBSP1_FSX, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_mcbsp1_fsx: BT_PCM_FS */

	/* Audio */
	{ABE_PDM_UL_DATA, (PTD | IEN | OFF_IN_PD | M7)},			/* abe_pdm_ul_data: h_PDM_UL_DATA */
	{ABE_PDM_DL_DATA, (PTD | IEN | OFF_IN_PD | M7)},			/* abe_pdm_dl_data: h_PDM_DL_DATA */
	{ABE_PDM_FRAME, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_pdm_frame: h_PDM_FRAME */
	{ABE_PDM_LB_CLK, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_pdm_lb_clk: h_PDM_CLK */
	{ABE_CLKS, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_clks: h_ABE_CLK */

	/* Digital MIC */
	{ABE_DMIC_CLK1, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_dmic_clk1: h_DMIC_CLK */
	{ABE_DMIC_DIN1, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_dmic_din1: h_DMIC_DIN1 */

	{ABE_DMIC_DIN2, (PTD | IEN | OFF_IN_PD | M7)},				/* abe_dmic_din2: NC: SafeMode */
	{ABE_DMIC_DIN3, (PTU | IEN | OFF_IN_PD | M7)},				/* abe_dmic_din3: NC: SafeMode */

	/* BT UART */
	{UART2_CTS, (PTU | IEN | M0)},						/* uart2_cts: BT_UART2_CTS */
	{UART2_RTS, (M0)},							/* uart2_rts: BT_UART2_RTS */
	{UART2_RX, (PTU | IEN | M0)},						/* uart2_rx: BT_UART2_RX */
	{UART2_TX, (M0)},							/* uart2_tx: BT_UART2_TX */

	{HDQ_SIO, (PTD | IDIS | OFF_OUT_PD | M3)},				/* gpio_127: AUD_PWRON (TWL6041) */

	/* I2C1~4 */
	{I2C1_SCL, (PTU | IEN | M0)},						/* i2c1_scl */
	{I2C1_SDA, (PTU | IEN | M0)},						/* i2c1_sda */
	{I2C2_SCL, (PTU | IEN | M0)},						/* i2c2_scl */
	{I2C2_SDA, (PTU | IEN | M0)},						/* i2c2_sda */
	{I2C3_SCL, (PTU | IEN | M0)},						/* i2c3_scl */
	{I2C3_SDA, (PTU | IEN | M0)},						/* i2c3_sda */
	{I2C4_SCL, (PTU | IEN | M0)},						/* i2c4_scl */
	{I2C4_SDA, (PTU | IEN | M0)},						/* i2c4_sda */

	{MCSPI1_CLK, (PTD | IEN | OFF_IN_PD | M7)},				/* mcspi1_clk: NC: SafeMode */
	{MCSPI1_SOMI, (PTD | IEN | OFF_IN_PD | M7)},				/* mcspi1_somi: NC: SafeMode */
	{MCSPI1_SIMO, (PTD | IEN | OFF_IN_PD | M7)},				/* mcspi1_simo: NC: SafeMode */
	{MCSPI1_CS0, (PTD | IEN | OFF_IN_PD | M7)},				/* mcspi1_cs0: NC: SafeMode */
	{MCSPI1_CS1, (PTD | IEN | OFF_IN_PD | M7)},				/* uart1_rx: RS232_UART1_RX */
	{MCSPI1_CS2, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_139: NC: SafeMode */
	{MCSPI1_CS3, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_140: NC: SafeMode */
	{UART3_CTS_RCTX, (PTD | IEN | OFF_IN_PD | M7)},				/* uart1_tx: RS232_UART1_TX */
	{UART3_RTS_SD, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_142: NC: SafeMode */

	/* OMAP low-level debug UART */
	/* Pull up UART RX pin to avoid the undefined state */
	{UART3_RX_IRRX, (PTU | IEN | M0)},					/* uart3_rx: LOG_UART3_RX */
	{UART3_TX_IRTX, (M0)},							/* uart3_tx: LOG_UART3_TX */

	/* WIFI SDIO (not used in Bootloader) */
	{SDMMC5_CLK, (PTD | IEN | OFF_IN_PD | M7)},				/* SafeMode: sdmmc5_clk: WIFI_SDIO_CLK */
	{SDMMC5_CMD, (PTD | IEN | OFF_IN_PD | M7)},				/* SafeMode: sdmmc5_cmd: WIFI_SDIO_CMD */
	{SDMMC5_DAT0, (PTD | IEN | OFF_IN_PD | M7)},				/* SafeMode: sdmmc5_dat0: WIFI_SDIO_DAT0 */
	{SDMMC5_DAT1, (PTD | IEN | OFF_IN_PD | M7)},				/* SafeMode: sdmmc5_dat1: WIFI_SDIO_DAT1 */
	{SDMMC5_DAT2, (PTD | IEN | OFF_IN_PD | M7)},				/* SafeMode: sdmmc5_dat2: WIFI_SDIO_DAT2 */
	{SDMMC5_DAT3, (PTD | IEN | OFF_IN_PD | M7)},				/* SafeMode: sdmmc5_dat3: WIFI_SDIO_DAT3 */

	/* Modem SPI */
	{MCSPI4_CLK, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_151: NC: SafeMode */
	{MCSPI4_SIMO, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_152: NC: SafeMode */
	{MCSPI4_SOMI, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_153: NC: SafeMode */
	{MCSPI4_CS0, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_154: NC: SafeMode */
	{UART4_RX, (PTD | IEN | OFF_IN_PD | M7)},				/* uart4_rx: @TODO@NC: SafeMode */
	{UART4_TX, (PTD | IEN | OFF_IN_PD | M7)},				/* uart4_tx: @TODO@NC: SafeMode */
	{USBB2_ULPITLL_CLK, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_157: NC: SafeMode */
	{USBB2_ULPITLL_STP, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data23: NC: SafeMode */
	{USBB2_ULPITLL_DIR, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data22: NC: SafeMode */
	{USBB2_ULPITLL_NXT, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data21: NC: SafeMode */
	{USBB2_ULPITLL_DAT0, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data20: MIC_BIAS_EN NM*/
	{USBB2_ULPITLL_DAT1, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data19: NC: SafeMode */
	{USBB2_ULPITLL_DAT2, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data18: NC: SafeMode */
	{USBB2_ULPITLL_DAT3, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data15: NC: SafeMode */
	{USBB2_ULPITLL_DAT4, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data14: NC: SafeMode */
	{USBB2_ULPITLL_DAT5, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data13: NC: SafeMode */
	{USBB2_ULPITLL_DAT6, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data12: NC: SafeMode */
	{USBB2_ULPITLL_DAT7, (PTD | IEN | OFF_IN_PD | M7)},			/* dispc2_data11: NC: SafeMode */
	{USBB2_HSIC_DATA, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_169: NC: SafeMode */
	{USBB2_HSIC_STROBE, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_170: NC: SafeMode */
	{UNIPRO_TX0/*KPD_COL3*/, (M1)},						/* kpd_col0 */
	{UNIPRO_TY0/*KPD_COL4*/, (M1)},						/* kpd_col1 */
	{UNIPRO_TX1/*KPD_COL5*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_173: NC: SafeMode */
	{UNIPRO_TY1/*KPD_COL0*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_174: NC: SafeMode */
	{UNIPRO_TX2/*KPD_COL1*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_0: NC: SafeMode */
	{UNIPRO_TY2/*KPD_COL2*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_1: NC: SafeMode */
	{UNIPRO_RX0/*KPD_ROW3*/, (PTU | IEN | M1)},				/* kpd_row0 */
	{UNIPRO_RY0/*KPD_ROW4*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_176: NC: SafeMode */
	{UNIPRO_RX1/*KPD_ROW5*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_177: NC: SafeMode */
	{UNIPRO_RY1/*KPD_ROW0*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_178: NC: SafeMode */
	{UNIPRO_RX2/*KPD_ROW1*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_2: NC: SafeMode */
	{UNIPRO_RY2/*KPD_ROW2*/, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_3: NC: SafeMode */
	{USBA0_OTG_CE, (PTD | OFF_OUT_PD | M0)},				/* usba0_otg_ce: USB_CHGEN */
	{USBA0_OTG_DP, (IEN | OFF_IN_PD | M0)},					/* usba0_otg_dp: OTG_USB1_DP */
	{USBA0_OTG_DM, (IEN | OFF_IN_PD | M0)},					/* usba0_otg_dm: OTG_USB1_DM */
	{FREF_CLK1_OUT, (PTD | IEN | OFF_IN_PD | M7)},				/* fref_clk1_out: MAIN_CAM_CLK (gpio_181) */
	{FREF_CLK2_OUT, (PTD | IEN | OFF_IN_PD | M7)},				/* fref_clk2_out: NC: SafeMode */
	{SYS_NIRQ1, (PTU | IEN | M0)},						/* sys_nirq1: PMICINT active ? */
	{SYS_NIRQ2, (PTU | IEN | M0)},						/* sys_nirq2: AUDINT active ? */

	/* SYS_BOOTx */
	{SYS_BOOT0, (IEN | PTD | M0)},						/* SYS_BOOT0: Input */
	{SYS_BOOT1, (IEN | PTU | M0)},						/* SYS_BOOT1: Input */
	{SYS_BOOT2, (IEN | PTU | M0)},						/* SYS_BOOT2: Input */
	{SYS_BOOT3, (IEN | PTD | M0)},						/* SYS_BOOT3: Input */
	{SYS_BOOT4, (IEN | PTU | M0)},						/* SYS_BOOT4: Input */
	{SYS_BOOT5, (IEN | PTU | M0)},						/* SYS_BOOT5: Input */

	{DPM_EMU0, (IEN | M0)},							/* dpm_emu0 */
	{DPM_EMU1, (IEN | M0)},							/* dpm_emu1 */
	{DPM_EMU2, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu2: NC: SafeMode */
	{DPM_EMU3, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu3: NC: SafeMode */
	{DPM_EMU4, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu4: NC: SafeMode */
	{DPM_EMU5, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu5: NC: SafeMode */
	{DPM_EMU6, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu6: NC: SafeMode */
	{DPM_EMU7, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu7: NC: SafeMode */
	{DPM_EMU8, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu8: NC: SafeMode */
	{DPM_EMU9, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu9: NC: SafeMode */
	{DPM_EMU10, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu10: NC: SafeMode */
	{DPM_EMU11, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu11: NC: SafeMode */
	{DPM_EMU12, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu12: NC: SafeMode */
	{DPM_EMU13, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu13: NC: SafeMode */
	{DPM_EMU14, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu14: NC: SafeMode */
	{DPM_EMU15, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu15: NC: SafeMode */
	{DPM_EMU16, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu16: NC: SafeMode */
	{DPM_EMU17, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu17: NC: SafeMode */
	{DPM_EMU18, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu18: NC: SafeMode */
	{DPM_EMU19, (PTD | IEN | OFF_IN_PD | M7)},				/* dpm_emu19: NC: SafeMode */
};

const struct pad_conf_entry wkup_padconf_array_non_essential[] = {
	{PAD0_SIM_IO, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_wk0: NC: SafeMode */
	{PAD1_SIM_CLK, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_wk1: NC: SafeMode */
	{PAD0_SIM_RESET, (PTD | IEN | OFF_IN_PD | M7)},				/* gpio_wk2: NC: SafeMode */
	{PAD1_SIM_CD, (PTU | IEN | OFF_IN_PD | M7)},				/* SafeMode: gpio_wk3: SIM_DET active low */
	{PAD0_SIM_PWRCTRL, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_wk4: NC: SafeMode */
	{PAD1_SR_SCL, (PTU | IEN | M0)},					/* sr_scl */
	{PAD0_SR_SDA, (PTU | IEN | M0)},					/* sr_sda */
	{PAD1_FREF_XTAL_IN, (PTD | IEN | OFF_IN_PD | M0)},			/* fref_xtal_in: NC */
	{PAD0_FREF_SLICER_IN, (M0)},						/* fref_slicer_in */
	{PAD1_FREF_CLK_IOREQ, (M0)},						/* fref_clk_ioreq */
	{PAD0_FREF_CLK0_OUT, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_wk6: h_SYS_DRM_MSEC */
	{PAD1_FREF_CLK3_REQ, (PTD | IEN | OFF_IN_PD | M7)},			/* gpio_wk30: NC: SafeMode */
	{PAD0_FREF_CLK3_OUT, (M0)},						/* fref_clk3_out: LVDS_CLK1 */
	{PAD1_FREF_CLK4_REQ, (PTU | IEN | M3)},					/* gpio_wk7: VAC_INT_N with external pull up*/
	{PAD0_FREF_CLK4_OUT, (M0)},						/* fref_clk4_out: LVDS_CLK2 */
	{PAD1_SYS_32K, (IEN | M0)},						/* sys_32k */
	{PAD0_SYS_NRESPWRON, (M0)},						/* sys_nrespwron */
	{PAD1_SYS_NRESWARM, (M0)},						/* sys_nreswarm */
	{PAD0_SYS_PWR_REQ, (PTU | M0)},						/* sys_pwr_req */
	{PAD1_SYS_PWRON_RESET, (M3)},						/* sys_pwron_reset: gpio_wk29 @TODO@ */
	{PAD0_SYS_BOOT6, (IEN | PTD | M0)},					/* SYSBOOT6: Input */
	{PAD1_SYS_BOOT7, (IEN | PTU | M0)},					/* SYSBOOT7: Input */
};
#endif /* CONFIG_SPL_BUILD */

#endif /* _BOARD_MUX_DATA_H */
