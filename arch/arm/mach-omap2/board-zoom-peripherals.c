/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/i2c/twl4030.h>
#include <linux/regulator/machine.h>
#include <linux/interrupt.h>
#include <linux/i2c/synaptics_i2c_rmi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mux.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <plat/display.h>

#include "mmc-twl4030.h"
#include "twl4030-script.h"

#define OMAP_SYNAPTICS_GPIO		163
#define LCD_PANEL_ENABLE_GPIO           (7 + OMAP_MAX_GPIO_LINES)
#define LCD_PANEL_RESET_GPIO            55
#define LCD_PANEL_QVGA_GPIO             56
#define TV_PANEL_ENABLE_GPIO            95
#define ENABLE_VAUX2_DEDICATED          0x09
#define ENABLE_VAUX2_DEV_GRP            0x20
#define ENABLE_VAUX3_DEDICATED          0x03
#define ENABLE_VAUX3_DEV_GRP            0x20
#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED         0x36
#define ENABLE_VDAC_DEDICATED		0x03
#define ENABLE_VDAC_DEV_GRP		0x20
#define DISABLE_VDAC_DEDICATED		0x00
#define DISABLE_VDAC_DEV_GRP		0x00

static void zoom_lcd_tv_panel_init(void)
{
	unsigned char lcd_panel_reset_gpio;

	if (omap_rev() > OMAP3430_REV_ES3_0) {
		/* Production Zoom2 Board:
		*  GPIO-96 is the LCD_RESET_GPIO
		*/
		lcd_panel_reset_gpio = 96;
	} else {
		/* Pilot Zoom2 board
		*  GPIO-55 is the LCD_RESET_GPIO
		*/
		lcd_panel_reset_gpio = 55;
	}

	gpio_request(lcd_panel_reset_gpio, "lcd reset");
	gpio_direction_output(lcd_panel_reset_gpio, 1);
	gpio_request(LCD_PANEL_QVGA_GPIO, "lcd qvga");
	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 1);
	gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd panel");
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	gpio_request(TV_PANEL_ENABLE_GPIO, "tv panel");
	gpio_direction_output(TV_PANEL_ENABLE_GPIO, 0);
}

static int zoom_panel_power_enable(int enable)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEDICATED : 0,
				TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				(enable) ? ENABLE_VPLL2_DEV_GRP : 0,
				TWL4030_VPLL2_DEV_GRP);
	return 0;
}

static int zoom_panel_enable_lcd(struct omap_dss_device *dssdev)
{

	zoom_panel_power_enable(1);

	gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd panel");
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 1);

	return 0;
}

static void zoom_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	zoom_panel_power_enable(0);

        gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd panel");
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);

}

static int zoom_panel_enable_tv(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VDAC_DEDICATED, TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void zoom_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				DISABLE_VDAC_DEDICATED,
				TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				DISABLE_VDAC_DEV_GRP,
				TWL4030_VDAC_DEV_GRP);
}

static struct omap_dss_device zoom_lcd_device = {
	.name = "lcd",
	.driver_name = "NEC_8048_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = zoom_panel_enable_lcd,
	.platform_disable = zoom_panel_disable_lcd,
};

static struct omap_dss_device zoom_tv_device = {
	.name                   = "tv",
	.driver_name            = "venc",
	.type                   = OMAP_DISPLAY_TYPE_VENC,
#ifdef CONFIG_MACH_OMAP_3630SDP
	.phy.venc.type          = OMAP_DSS_VENC_TYPE_SVIDEO,
#else
	.phy.venc.type          = OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable        = zoom_panel_enable_tv,
	.platform_disable       = zoom_panel_disable_tv,
};

static struct omap_dss_device *zoom_dss_devices[] = {
	&zoom_lcd_device,
	&zoom_tv_device,
};

static struct omap_dss_board_info zoom_dss_data = {
	.num_devices = ARRAY_SIZE(zoom_dss_devices),
	.devices = zoom_dss_devices,
	.default_device = &zoom_lcd_device,
};

static struct platform_device zoom_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &zoom_dss_data,
	},
};

static struct regulator_consumer_supply zoom_vdda_dac_supply = {
	.supply         = "vdda_dac",
	.dev            = &zoom_dss_device.dev,
};

static struct omap2_mcspi_device_config dss_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info nec_8048_spi_board_info[] __initdata = {
	[0] = {
		.modalias               = "nec_8048_spi",
		.bus_num                = 1,
		.chip_select            = 2,
		.max_speed_hz           = 375000,
		.controller_data        = &dss_lcd_mcspi_config,
	},
};

#include <media/v4l2-int-device.h>

#if (defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)) && \
    defined(CONFIG_VIDEO_OMAP3)
#include <media/imx046.h>
extern struct imx046_platform_data zoom_imx046_platform_data;
#endif

#ifdef CONFIG_VIDEO_OMAP3
extern void zoom_cam_init(void);
#else
#define zoom_cam_init()	NULL
#endif

#if defined(CONFIG_VIDEO_LV8093) && defined(CONFIG_VIDEO_OMAP3)
#include <media/lv8093.h>
extern struct imx046_platform_data zoom_lv8093_platform_data;
#endif

/* Zoom2 has Qwerty keyboard*/
static int board_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(0, 1, KEY_R),
	KEY(0, 2, KEY_T),
	KEY(0, 3, KEY_HOME),
	KEY(0, 6, KEY_I),
	KEY(0, 7, KEY_LEFTSHIFT),
	KEY(1, 0, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(1, 2, KEY_G),
	KEY(1, 3, KEY_SEND),
	KEY(1, 6, KEY_K),
	KEY(1, 7, KEY_ENTER),
	KEY(2, 0, KEY_X),
	KEY(2, 1, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(2, 3, KEY_END),
	KEY(2, 6, KEY_DOT),
	KEY(2, 7, KEY_CAPSLOCK),
	KEY(3, 0, KEY_Z),
	KEY(3, 1, KEY_KPPLUS),
	KEY(3, 2, KEY_B),
	KEY(3, 3, KEY_F1),
	KEY(3, 6, KEY_O),
	KEY(3, 7, KEY_SPACE),
	KEY(4, 0, KEY_W),
	KEY(4, 1, KEY_Y),
	KEY(4, 2, KEY_U),
	KEY(4, 3, KEY_F2),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(4, 6, KEY_L),
	KEY(4, 7, KEY_LEFT),
	KEY(5, 0, KEY_S),
	KEY(5, 1, KEY_H),
	KEY(5, 2, KEY_J),
	KEY(5, 3, KEY_F3),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_M),
	KEY(5, 7, KEY_ENTER),
	KEY(6, 0, KEY_Q),
	KEY(6, 1, KEY_A),
	KEY(6, 2, KEY_N),
	KEY(6, 3, KEY_BACKSPACE),
	KEY(6, 6, KEY_P),
	KEY(6, 7, KEY_SELECT),
	KEY(7, 0, KEY_PROG1),	/*MACRO 1 <User defined> */
	KEY(7, 1, KEY_PROG2),	/*MACRO 2 <User defined> */
	KEY(7, 2, KEY_PROG3),	/*MACRO 3 <User defined> */
	KEY(7, 3, KEY_PROG4),	/*MACRO 4 <User defined> */
	KEY(7, 5, KEY_RIGHT),
	KEY(7, 6, KEY_UP),
	KEY(7, 7, KEY_DOWN)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data zoom_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 8,
	.cols		= 8,
	.rep		= 1,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1, .type = 3,
		.type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA1, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VINTDIG, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1,
		.type = 4, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1,
		.type = 3, .type2 = 1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 2,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 0,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 3,
		.type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 6,
		.type2 = 1, .remap_sleep = RES_STATE_SLEEP },
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3,
		.type = 0, .type2 = 2, .remap_sleep = RES_STATE_SLEEP },
	{ 0, 0},
};

static struct twl4030_power_data zoom_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

static struct platform_device zoom_cam_device = {
	.name		= "zoom_cam",
	.id		= -1,
};

static struct regulator_consumer_supply zoom_vaux2_supplies[] = {
	{
		.supply		= "vaux2_1",
		.dev		= &zoom_cam_device.dev,
	},
};

static struct regulator_consumer_supply zoom_vaux4_supplies[] = {
	{
		.supply		= "vaux4_1",
		.dev		= &zoom_cam_device.dev,
	},
};

static struct regulator_consumer_supply zoom_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply zoom_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply zoom_vmmc2_supply = {
	.supply		= "vmmc",
};

/* VAUX2 for camera module */
static struct regulator_init_data zoom_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(zoom_vaux2_supplies),
	.consumer_supplies	= zoom_vaux2_supplies,
};

/* VAUX4 for OMAP VDD_CSI2 (camera) */
static struct regulator_init_data zoom_vaux4 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(zoom_vaux4_supplies),
	.consumer_supplies	= zoom_vaux4_supplies,
};

/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data zoom_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &zoom_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data zoom_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &zoom_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data zoom_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &zoom_vsim_supply,
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.wires		= 4,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.wires		= 8,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{}      /* Terminator */
};

static struct regulator_init_data zoom_vdac = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &zoom_vdda_dac_supply,
};

static int zoom_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	zoom_vmmc1_supply.dev = mmc[0].dev;
	zoom_vsim_supply.dev = mmc[0].dev;
	zoom_vmmc2_supply.dev = mmc[1].dev;

	return 0;
}

#define ENABLE_VDAC_DEDICATED		0x03
#define ENABLE_VDAC_DEV_GRP		0x20
#define DISABLE_VDAC_DEDICATED		0x00
#define DISABLE_VDAC_DEV_GRP		0x00

static int zoom_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,  9280,  8950,  8620,  8310,
8020,  7730,  7460,  7200,  6950,  6710,  6470,  6250,  6040,  5830,
5640,  5450,  5260,  5090,  4920,  4760,  4600,  4450,  4310,  4170,
4040,  3910,  3790,  3670,  3550
};

static struct twl4030_bci_platform_data zoom_bci_data = {
	.battery_tmp_tbl	= zoom_batt_table,
	.tblsize		= ARRAY_SIZE(zoom_batt_table),
};

static struct twl4030_usb_data zoom_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data zoom_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= zoom_twl_gpio_setup,
};

static struct twl4030_madc_platform_data zoom_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data zoom_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data zoom_codec_data = {
	.audio_mclk = 26000000,
	.audio = &zoom_audio_data,
};

static struct twl4030_platform_data zoom_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &zoom_bci_data,
	.madc		= &zoom_madc_data,
	.usb		= &zoom_usb_data,
	.gpio		= &zoom_gpio_data,
	.keypad		= &zoom_kp_twl4030_data,
	.power		= &zoom_t2scripts_data,
	.codec		= &zoom_codec_data,
	.vaux2		= &zoom_vaux2,
	.vaux4		= &zoom_vaux4,
	.vmmc1          = &zoom_vmmc1,
	.vmmc2          = &zoom_vmmc2,
	.vsim           = &zoom_vsim,
	.vdac		= &zoom_vdac,

};

static void synaptics_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	omap_cfg_reg(H18_34XX_GPIO163);

	if (gpio_request(OMAP_SYNAPTICS_GPIO, "touch") < 0) {
		printk(KERN_ERR "can't get synaptics pen down GPIO\n");
		return;
	}
	gpio_direction_input(OMAP_SYNAPTICS_GPIO);
	omap_set_gpio_debounce(OMAP_SYNAPTICS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP_SYNAPTICS_GPIO, 0xa);
}

static int synaptics_power(int power_state)
{
	/* TODO: synaptics is powered by vbatt */
	return 0;
}

static struct synaptics_i2c_rmi_platform_data synaptics_platform_data[] = {
	{
		.version        = 0x0,
		.power          = &synaptics_power,
		.flags          = SYNAPTICS_SWAP_XY,
		.irqflags       = IRQF_TRIGGER_LOW,
	}
};

static struct i2c_board_info __initdata zoom_i2c_boardinfo2[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME,  0x20),
		.platform_data = &synaptics_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_SYNAPTICS_GPIO),
	},
#if (defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)) && \
    defined(CONFIG_VIDEO_OMAP3)
	{
		I2C_BOARD_INFO("imx046", IMX046_I2C_ADDR),
		.platform_data = &zoom_imx046_platform_data,
	},
#endif
#if defined(CONFIG_VIDEO_LV8093) && defined(CONFIG_VIDEO_OMAP3)
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &zoom_lv8093_platform_data,
	},
#endif
};

static struct i2c_board_info __initdata zoom_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl5030", 0x48),
		.flags		= I2C_CLIENT_WAKE,
		.irq		= INT_34XX_SYS_NIRQ,
		.platform_data	= &zoom_twldata,
	},
};

static int __init omap_i2c_init(void)
{
/* Disable OMAP 3630 internal pull-ups for I2Ci */
	if (cpu_is_omap3630()) {

		u32 prog_io;
		prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
		/* Program (bit 19)=1 to disable internal pull-up on I2C1 */
		prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
		/* Program (bit 0)=1 to disable internal pull-up on I2C2 */
		prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
		/* Program (bit 7)=1 to disable internal pull-up on I2C3 */
		prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);

		prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
		/* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
		prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
		omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
	}

	omap_register_i2c_bus(1, 2400, zoom_i2c_boardinfo,
			ARRAY_SIZE(zoom_i2c_boardinfo));
	omap_register_i2c_bus(2, 100, zoom_i2c_boardinfo2,
			ARRAY_SIZE(zoom_i2c_boardinfo2));
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void enable_board_wakeup_source(void)
{
	omap_cfg_reg(AF26_34XX_SYS_NIRQ); /* T2 interrupt line */
}

static struct platform_device *zoom_devices[] __initdata = {
	&zoom_dss_device,
	&zoom_cam_device,
};

void __init zoom_peripherals_init(void)
{
	omap_i2c_init();
	platform_add_devices(zoom_devices, ARRAY_SIZE(zoom_devices));
	spi_register_board_info(nec_8048_spi_board_info,
				ARRAY_SIZE(nec_8048_spi_board_info));
	zoom_lcd_tv_panel_init();
	synaptics_dev_init();
	omap_serial_init();
	usb_musb_init();
	enable_board_wakeup_source();
	zoom_cam_init();
}
