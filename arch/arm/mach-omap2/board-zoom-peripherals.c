/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-ldp.c
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
#include <linux/synaptics_i2c_rmi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mux.h>

#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <plat/display.h>
#include "mmc-twl4030.h"
#include <mach/board-zoom.h>

#define OMAP_SYNAPTICS_GPIO		163

#include <media/v4l2-int-device.h>

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
#include <media/imx046.h>
extern struct imx046_platform_data zoom_imx046_platform_data;
#endif

extern void zoom_cam_init(void);

#ifdef CONFIG_VIDEO_LV8093
#include <media/lv8093.h>
extern struct imx046_platform_data zoom_lv8093_platform_data;
#endif
#define LCD_PANEL_BACKLIGHT_GPIO        (15 + OMAP_MAX_GPIO_LINES)
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

/*#define SIL9022_RESET_GPIO              97*/


static void zoom_lcd_tv_panel_init(void)
{
	unsigned char lcd_panel_reset_gpio;

	if (omap_rev() > OMAP3430_REV_ES3_0) {
		/* Production Zoom2 Board:
 * 		 * GPIO-96 is the LCD_RESET_GPIO
 * 		 		 */
		omap_cfg_reg(C25_34XX_GPIO96);
		lcd_panel_reset_gpio = 96;
	} else {
	/* Pilot Zoom2 board
 * 	 * GPIO-55 is the LCD_RESET_GPIO
 * 	 		 */
		omap_cfg_reg(T8_34XX_GPIO55);
		lcd_panel_reset_gpio = 55;
	}

	gpio_request(lcd_panel_reset_gpio, "lcd reset");
	gpio_request(LCD_PANEL_QVGA_GPIO, "lcd qvga");
	gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd panel");
	gpio_request(LCD_PANEL_BACKLIGHT_GPIO, "lcd backlight");

	gpio_request(TV_PANEL_ENABLE_GPIO, "tv panel");

	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 0);
	gpio_direction_output(lcd_panel_reset_gpio, 0);
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);
	gpio_direction_output(TV_PANEL_ENABLE_GPIO, 0);

	gpio_direction_output(LCD_PANEL_QVGA_GPIO, 1);
	gpio_direction_output(lcd_panel_reset_gpio, 1);
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
	gpio_request(LCD_PANEL_BACKLIGHT_GPIO, "lcd backlight");
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 1);

	return 0;
}

static void zoom_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	zoom_panel_power_enable(0);

	gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd panel");
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	gpio_request(LCD_PANEL_BACKLIGHT_GPIO, "lcd backlight");
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);

}

static struct omap_dss_device zoom_lcd_device = {
	.name = "lcd",
	.driver_name = "NEC_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = zoom_panel_enable_lcd,
	.platform_disable = zoom_panel_disable_lcd,
 };

static struct omap_dss_device *zoom_dss_devices[] = {
	&zoom_lcd_device,
/*        &zoom_tv_device,
 *        #ifdef CONFIG_SIL9022
 *        	&zoom_hdmi_device,
 *        	#endif
 *        	*/
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


#ifdef CONFIG_FB_OMAP2
static struct resource zoom_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};
#else
static struct resource zoom_vout_resource[2] = {
};
#endif


static struct omap2_mcspi_device_config zoom_lcd_mcspi_config = {
	.turbo_mode             = 0,
	.single_channel         = 1,  /* 0: slave, 1: master */
};

static struct spi_board_info zoom_spi_board_info[] __initdata = {
	[0] = {
		.modalias               = "zoom_disp_spi",
		.bus_num                = 1,
		.chip_select            = 2,
		.max_speed_hz           = 375000,
		.controller_data        = &zoom_lcd_mcspi_config,
	},
};

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

static struct twl4030_madc_platform_data zoom_madc_data = {
	.irq_line		= 1,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.name		= "external",
		.mmc		= 1,
		.wires		= 4,
		.cover_only	= true,
		.gpio_cd	= -EINVAL,
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
	{}	/* Terminator */
};

static struct regulator_consumer_supply zoom_vmmc1_supply = {
	.supply		= "vmmc",
};

static struct regulator_consumer_supply zoom_vmmc2_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply zoom_vsim_supply = {
	.supply		= "vmmc_aux",
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
		.name			= "VMMC2_30",
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &zoom_vmmc2_supply,
};

/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data zoom_vsim = {
	.constraints = {
		.name			= "VMMC2_IO_18",
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies      = &zoom_vsim_supply,
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
	/* gpio + 0 is "mmc0_cd" (input/IRQ),
	 * gpio + 1 is "mmc1_cd" (input/IRQ)
	 */
	mmc[0].gpio_cd = gpio + 0;
	mmc[1].gpio_cd = gpio + 1;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	zoom_vmmc1_supply.dev = mmc[0].dev;
	zoom_vmmc2_supply.dev = mmc[1].dev;
	zoom_vsim_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_gpio_platform_data zoom_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= zoom_twl_gpio_setup,
};

static struct twl4030_usb_data zoom_usb_data = {
	.usb_mode		= T2_USB_MODE_ULPI,
};
static void enable_board_wakeup_source(void)
{
	omap_cfg_reg(AF26_34XX_SYS_NIRQ);
}

static struct twl4030_ins __initdata sleep_on_seq[] = {
	/* Broadcast message to put res to sleep */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R1, RES_TYPE2_R0,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

static struct twl4030_ins wakeup_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R1, RES_TYPE2_R0,
			RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_script __initdata = {
	.script = wakeup_seq,
	.size   = ARRAY_SIZE(wakeup_seq),
	.flags  = TWL4030_WAKEUP_SCRIPT,
};

static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset VDD1 regulator.
 * Reset VDD2 regulator.
 * Reset VPLL1 regulator.
 * Enable sysclk output.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_P1, 0xf, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x10, RES_STATE_WRST), 15},
	{MSG_SINGULAR(DEV_GRP_P1, 0x7, RES_STATE_WRST), 0x60},
	{MSG_SINGULAR(DEV_GRP_P1, 0x19, RES_STATE_ACTIVE), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, 0x1b, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_script,
	&wrst_script,
};

static struct twl4030_resconfig twl4030_rconfig[] = {
	{ .resource = RES_HFCLKOUT, .devgroup = DEV_GRP_P3, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_VDD1, .devgroup = DEV_GRP_P1, .type = 1,
		.type2 = -1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VDD2, .devgroup = DEV_GRP_P1, .type = 1,
		.type2 = -1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VPLL1, .devgroup = DEV_GRP_P1, .type = 1,
		.type2 = -1, .remap_sleep = RES_STATE_OFF },
	{ .resource = RES_VINTANA2, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_VIO, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_REGEN, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_NRES_PWRON, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_CLKEN, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_SYSEN, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_32KCLKOUT, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_RESET, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ .resource = RES_Main_Ref, .devgroup = DEV_GRP_ALL, .type = 1,
		.type2 = -1, .remap_sleep = -1 },
	{ 0, 0 },
};

static struct twl4030_power_data zoom_t2scripts_data __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
	.resource_config = twl4030_rconfig,
};

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
	.vmmc1          = &zoom_vmmc1,
	.vmmc2          = &zoom_vmmc2,
	/* .vaux1		= &zoom_vaux1,*/
	.vaux2		= &zoom_vaux2,
	.vaux4		= &zoom_vaux4,
	.vsim           = &zoom_vsim,
	.vdac		= &zoom_vdac,
};

static struct i2c_board_info __initdata zoom_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl5030", 0x48),
		.flags          = I2C_CLIENT_WAKE,
		.irq            = INT_34XX_SYS_NIRQ,
		.platform_data  = &zoom_twldata,
	},
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
#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
	{
		I2C_BOARD_INFO("imx046", IMX046_I2C_ADDR),
		.platform_data = &zoom_imx046_platform_data,
	},
#endif
#ifdef CONFIG_VIDEO_LV8093
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &zoom_lv8093_platform_data,
	},
#endif
};

static int __init zoom_i2c_init(void)
{
	omap_register_i2c_bus(1, 2200, zoom_i2c_boardinfo,
			ARRAY_SIZE(zoom_i2c_boardinfo));

	/* TODO: I2C2 on zoom2/3:
	 * Add Synaptic RMI controller
	 * Add Camera sensor IMX046
	 * Add Camera sensor LV8093
	 */
	omap_register_i2c_bus(2, 100, zoom_i2c_boardinfo2,
			ARRAY_SIZE(zoom_i2c_boardinfo2));

	/* TODO: I2C3 on Zoom2/3:
	 * Add: SIL9022 HDMI
	 */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

/*
#ifdef CONFIG_PM
struct vout_platform_data zoom_vout_data = {
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.set_max_mpu_wakeup_lat =  omap_pm_set_max_mpu_wakeup_lat,
	.set_cpu_freq = omap_pm_cpu_set_freq,
};
#endif
*/

static struct platform_device zoom_vout_device = {
	.name           = "omap_vout",
	.num_resources  = ARRAY_SIZE(zoom_vout_resource),
	.resource       = &zoom_vout_resource[0],
	.id             = -1,
/*
#ifdef CONFIG_PM
	.dev            = {
		.platform_data = &zoom_vout_data,
	}
#else
*/
	.dev            = {
		.platform_data = NULL,
	}
/*#endif*/
};


static struct platform_device *zoom_devices[] __initdata = {
	&zoom_cam_device,
	&zoom_dss_device,
};

void __init zoom_peripherals_init(void)
{
	zoom_i2c_init();
	platform_add_devices(zoom_devices, ARRAY_SIZE(zoom_devices));
	synaptics_dev_init();
	spi_register_board_info(zoom_spi_board_info,
				ARRAY_SIZE(zoom_spi_board_info));
	omap_serial_init();
	usb_musb_init();
	zoom_flash_init();
	zoom_lcd_tv_panel_init();
	zoom_cam_init();
}
