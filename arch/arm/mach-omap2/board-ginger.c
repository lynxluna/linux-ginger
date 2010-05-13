/*
 * board-ginger - TimLL Devkit8000-based Ginger Console
 *
 * Copyright (C) 2009 Kim Botherway
 * Copyright (C) 2010 Thomas Weber
 *
 * Modified from mach-omap2/board-omap3beagle.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/timer-gp.h>
#include <plat/display.h>

#include <plat/mcspi.h>
#include <linux/input/matrix_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/usb/otg.h>
#include <linux/dm9000.h>
#include <linux/interrupt.h>

#include "sdram-micron-mt46h32m32lf-6.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"

#include <plat/mux.h>
#include <plat/mmc.h>

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

#define OMAP_DM9000_GPIO_IRQ	25
#define OMAP3_EVM_TS_GPIO	27

static struct mtd_partition ginger_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data ginger_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= ginger_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ginger_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource ginger_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ginger_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &ginger_nand_data,
	},
	.num_resources	= 1,
	.resource	= &ginger_nand_resource,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_wp	= 29,
	},
	{}	/* Terminator */
};
static struct omap_board_config_kernel ginger_config[] __initdata = {
};

// static int ginger_panel_enable_lcd(struct omap_dss_device *dssdev)
// {
// 	twl_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80, REG_GPIODATADIR1);
// 	twl_i2c_write_u8(TWL4030_MODULE_LED, 0x0, 0x0);
// 
// 	return 0;
// }
// 
// static void ginger_panel_disable_lcd(struct omap_dss_device *dssdev)
// {
// }
static int ginger_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80, REG_GPIODATAOUT1);
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x0, 0x0);
	return 0;
}
static void ginger_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80, REG_GPIODATAOUT1);
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x0, 0x0);
}
static int ginger_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	return 0;
}

static void ginger_panel_disable_dvi(struct omap_dss_device *dssdev)
{
}

static int ginger_panel_enable_tv(struct omap_dss_device *dssdev)
{

	return 0;
}

static void ginger_panel_disable_tv(struct omap_dss_device *dssdev)
{
}


static struct regulator_consumer_supply ginger_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply ginger_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct regulator_consumer_supply ginger_vio_supplies[] = {
	REGULATOR_SUPPLY("vcc", "spi2.0"),
}; 


static struct omap_dss_device ginger_lcd_device = {
	.name                   = "lcd",
	.driver_name            = "ginger_panel",
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines     = 24,
	.platform_enable        = ginger_panel_enable_lcd,
	.platform_disable       = ginger_panel_disable_lcd,
};
static struct omap_dss_device ginger_dvi_device = {
	.name                   = "dvi",
	.driver_name            = "generic_panel",
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines     = 24,
	.platform_enable        = ginger_panel_enable_dvi,
	.platform_disable       = ginger_panel_disable_dvi,
};

static struct omap_dss_device ginger_tv_device = {
	.name                   = "tv",
	.driver_name            = "venc",
	.type                   = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type          = OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable        = ginger_panel_enable_tv,
	.platform_disable       = ginger_panel_disable_tv,
};


static struct omap_dss_device *ginger_dss_devices[] = {
	&ginger_lcd_device,
	&ginger_dvi_device,
	&ginger_tv_device,
};

static struct omap_dss_board_info ginger_dss_data = {
	.num_devices = ARRAY_SIZE(ginger_dss_devices),
	.devices = ginger_dss_devices,
	.default_device = &ginger_lcd_device,
};

static struct platform_device ginger_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &ginger_dss_data,
	},
};

static struct regulator_consumer_supply ginger_vdda_dac_supply = {
	.supply = "vdda_dac",
	.dev	= &ginger_dss_device.dev,
};

static int board_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(1, 0, KEY_2),
	KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(2, 1, KEY_6),
	KEY(3, 1, KEY_F5),
	KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8),
	KEY(2, 2, KEY_9),
	KEY(3, 2, KEY_F6),
	KEY(0, 3, KEY_F7),
	KEY(1, 3, KEY_0),
	KEY(2, 3, KEY_F8),
	PERSISTENT_KEY(4, 5),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(5, 5, KEY_VOLUMEDOWN),
	0
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size	= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data ginger_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 6,
	.cols		= 6,
	.rep		= 1,
};

static struct gpio_led gpio_leds[];

static int ginger_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	omap_cfg_reg(AH8_34XX_GPIO29);
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	ginger_vmmc1_supply.dev = mmc[0].dev;
	ginger_vsim_supply.dev = mmc[0].dev;

	/* REVISIT: need ehci-omap hooks for external VBUS
	 * power switch and overcurrent detect
	 */

	gpio_request(gpio + 1, "EHCI_nOC");
	gpio_direction_input(gpio + 1);

	/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
	gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
	gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);

	/* TWL4030_GPIO_MAX + 1 == ledB, PMU_STAT (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data ginger_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= ginger_twl_gpio_setup,
};

static struct regulator_consumer_supply ginger_vpll1_supplies[] = {
	{
	.supply		= "vdvi",
	.dev		= &ginger_lcd_device.dev,
	},
	{
	.supply		= "vdss_dsi",
	.dev		= &ginger_dss_device.dev,
	}
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data ginger_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ginger_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data ginger_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ginger_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data ginger_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ginger_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data ginger_vpll1 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(ginger_vpll1_supplies),
	.consumer_supplies	= ginger_vpll1_supplies,
};

/* VAUX4 for ads7846 and nubs */
static struct regulator_init_data ginger_vio = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(ginger_vio_supplies),
	.consumer_supplies      = ginger_vio_supplies,
};


static struct twl4030_usb_data ginger_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_platform_data ginger_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &ginger_usb_data,
	.gpio		= &ginger_gpio_data,
	.vmmc1		= &ginger_vmmc1,
	.vsim		= &ginger_vsim,
	.vdac		= &ginger_vdac,
	.vpll1		= &ginger_vpll1,
	.keypad		= &ginger_kp_data,
};

static struct i2c_board_info __initdata ginger_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &ginger_twldata,
	},
};

static int __init ginger_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, ginger_i2c_boardinfo,
			ARRAY_SIZE(ginger_i2c_boardinfo));
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "led1",
		.default_trigger	= "heartbeat",
		.gpio			= 186,
		.active_low		= true,
	},
	{
		.name			= "led2",
		.default_trigger	= "mmc0",
		.gpio			= 163,
		.active_low		= true,
	},
	{
		.name			= "ledB",
		.default_trigger	= "none",
		.gpio			= 153,
		.active_low             = true,
	},
	{
		.name			= "led3",
		.default_trigger	= "none",
		.gpio			= 164,
		.active_low             = true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 26,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};



#define OMAP_DM9000_BASE	0x2c000000

static struct resource omap_dm9000_resources[] = {
	[0] = {
		.start		= OMAP_DM9000_BASE,
		.end		= (OMAP_DM9000_BASE + 0x4 - 1),
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= (OMAP_DM9000_BASE + 0x400),
		.end		= (OMAP_DM9000_BASE + 0x400 + 0x4 - 1),
		.flags		= IORESOURCE_MEM,
	},
	[2] = {
		.start		= OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),
		.flags		= IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
	},
};

static struct dm9000_plat_data omap_dm9000_platdata = {
	.flags = DM9000_PLATF_16BITONLY,
};

static struct platform_device omap_dm9000_dev = {
	.name = "dm9000",
	.id = -1,
	.num_resources	= ARRAY_SIZE(omap_dm9000_resources),
	.resource	= omap_dm9000_resources,
	.dev = {
		.platform_data = &omap_dm9000_platdata,
	},
};

static void __init omap_dm9000_init(void)
{
	if (gpio_request(OMAP_DM9000_GPIO_IRQ, "dm9000 irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for dm9000 IRQ\n",
			OMAP_DM9000_GPIO_IRQ);
		return;
		}

	gpio_direction_input(OMAP_DM9000_GPIO_IRQ);
}

static void __init ginger_init_irq(void)
{
	omap_board_config = ginger_config;
	omap_board_config_size = ARRAY_SIZE(ginger_config);
	
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params, mt46h32m32lf6_sdrc_params,
	omap3_mpu_rate_table,omap3_dsp_rate_table, omap3_l3_rate_table);
	
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
	omap_dm9000_init();
}

static struct platform_device *ginger_devices[] __initdata = {
	&ginger_dss_device,
	&leds_gpio,
	&keys_gpio,
	&omap_dm9000_dev,
};

static void __init ginger_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		ginger_nand_data.cs = nandcs;
		ginger_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		ginger_nand_data.gpmc_baseaddr = (void *)
			(gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&ginger_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 147,
	.reset_gpio_port[2]  = -EINVAL
};

////// TOUCH
static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_EVM_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(OMAP3_EVM_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_EVM_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EVM_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
//	.x_plate_ohms		= 180,
//	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 5,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 2,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};


static void __init ginger_init(void)
{
	ginger_i2c_init();
	platform_add_devices(ginger_devices,
			ARRAY_SIZE(ginger_devices));
	omap_board_config = ginger_config;
	omap_board_config_size = ARRAY_SIZE(ginger_config);
	
	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));
	omap_serial_init();
	

	
	omap_cfg_reg(J25_34XX_GPIO170);
	gpio_request(170, "DVI_nPD");
	/* REVISIT leave DVI powered down until it's needed ... */
	gpio_direction_output(170, true);

	usb_musb_init();
	usb_ehci_init(&ehci_pdata);
	
	ads7846_dev_init();
	ginger_flash_init();
	
	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_cfg_reg(H16_34XX_SDRC_CKE0);
	omap_cfg_reg(H17_34XX_SDRC_CKE1);
}

static void __init ginger_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(GINGER, "GDi Ginger Console")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= ginger_map_io,
	.init_irq	= ginger_init_irq,
	.init_machine	= ginger_init,
	.timer		= &omap_timer,
MACHINE_END
