/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom2.c
 * Support for WLAN-device reset by:
 * 	Ohad Ben-Cohen		<ohad@bencohen.org>
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
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/interrupt.h>
#include <linux/i2c/synaptics_i2c_rmi.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>

#include "mux.h"
#include "mmc-twl4030.h"
#include "twl4030-script.h"
#include "pm.h"

#ifdef CONFIG_MMC_EMBEDDED_SDIO
#include <plat/wifi_tiwlan.h>
#endif

/* FIXME: These are not the optimal setup values */
static struct prm_setup_vc omap3_setuptime_table = {
	/* CLK SETUPTIME for RET & OFF */
	.clksetup_ret = 0xff,
	.clksetup_off = 0xff,
	/* VOLT SETUPTIME for RET & OFF */
	.voltsetup_time1_ret = 0xfff,
	.voltsetup_time2_ret = 0xfff,
	.voltsetup_time1_off = 0xfff,
	.voltsetup_time2_off = 0xfff,
	.voltoffset = 0xff,
	.voltsetup2 = 0xff,
	/* VC COMMAND VALUES for VDD1/VDD2 */
	.vdd0_on = 0x30,
	.vdd0_onlp = 0x20,
	.vdd0_ret = 0x1e,
	.vdd0_off = 0x00,
	.vdd1_on = 0x2c,
	.vdd1_onlp = 0x20,
	.vdd1_ret = 0x1e,
	.vdd1_off = 0x00,
};

#define OMAP_SYNAPTICS_GPIO		163

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
	KEY(5, 4, KEY_UNKNOWN),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(5, 6, KEY_M),
	KEY(5, 7, KEY_RIGHT),
	KEY(6, 0, KEY_Q),
	KEY(6, 1, KEY_A),
	KEY(6, 2, KEY_N),
	KEY(6, 3, KEY_BACKSPACE),
	KEY(6, 6, KEY_P),
	KEY(6, 7, KEY_UP),
	KEY(7, 0, KEY_PROG1),	/*MACRO 1 <User defined> */
	KEY(7, 1, KEY_PROG2),	/*MACRO 2 <User defined> */
	KEY(7, 2, KEY_PROG3),	/*MACRO 3 <User defined> */
	KEY(7, 3, KEY_PROG4),	/*MACRO 4 <User defined> */
	KEY(7, 6, KEY_SELECT),
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
	.resource_config = twl4030_rconfig,
};

#ifdef CONFIG_TWL4030_POWER
static void use_generic_twl4030_script(void)
{
	omap3_setuptime_table.voltsetup_time1_ret =
				twl4030_voltsetup_time.voltsetup_time1_ret;
	omap3_setuptime_table.voltsetup_time2_ret =
				twl4030_voltsetup_time.voltsetup_time2_ret;
	omap3_setuptime_table.voltsetup_time1_off =
				twl4030_voltsetup_time.voltsetup_time1_off;
	omap3_setuptime_table.voltsetup_time2_off =
				twl4030_voltsetup_time.voltsetup_time1_off;

	omap3_setuptime_table.voltoffset = twl4030_voltsetup_time.voltoffset;
	omap3_setuptime_table.voltsetup2 = twl4030_voltsetup_time.voltsetup2;

	zoom_t2scripts_data.scripts = twl4030_generic_script.scripts;
	zoom_t2scripts_data.num = twl4030_generic_script.num;
}
#endif

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
	{
		.mmc		= 3,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static struct regulator_consumer_supply zoom_vpll2_supply = {
	.supply         = "vpll2",
};

static struct regulator_consumer_supply zoom_vdda_dac_supply = {
	.supply         = "vdda_dac",
};

static struct regulator_init_data zoom_vpll2 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &zoom_vpll2_supply,
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
	.vpll2		= &zoom_vpll2,
	.vdac		= &zoom_vdac,

};

static void synaptics_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	omap_mux_init_gpio(OMAP_SYNAPTICS_GPIO, OMAP_PIN_INPUT_PULLUP);

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

#ifdef CONFIG_WLAN_1273
void wlan_1273_reset()
{
	if (gpio_request(OMAP_ZOOM3_WIFI_PMENA_GPIO, NULL) != 0)
		pr_err("GPIO 101 request failed\n");
	gpio_direction_output(OMAP_ZOOM3_WIFI_PMENA_GPIO, 0);
	return;
}
#else
#define wlan_1273_reset() do { } while (0)
#endif

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

/* GPIOS need to be in order of BT, FM and GPS
 * provide -1 is Not applicable for chip */
static int gpios[] = {109, 161, -1};
static struct platform_device zoom_btfmgps_device = {
	.name           = "kim", /* named after init manager for ST */
	.id             = -1,
	.dev.platform_data = &gpios,
};

static struct platform_device *zoom_devices[] __initdata = {
       &zoom_btfmgps_device,
	&zoom_cam_device,
};

void __init zoom_peripherals_init(void)
{
#ifdef CONFIG_TWL4030_POWER
	use_generic_twl4030_script();
#endif
	omap_i2c_init();
	synaptics_dev_init();

	wlan_1273_reset();

	omap_serial_init();
	usb_musb_init();
	omap3_pm_init_vc(&omap3_setuptime_table);
	zoom_cam_init();
	platform_add_devices(zoom_devices, ARRAY_SIZE(zoom_devices));
}
