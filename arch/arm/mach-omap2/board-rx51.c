/*
 * linux/arch/arm/mach-omap2/board-rx51.c
 *
 * Copyright (C) 2007, 2008 Nokia
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
#include <linux/gpio.h>
#include <linux/leds.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/dma.h>
#include <plat/gpmc.h>
#include <plat/usb.h>

#include "omap3-opp.h"

#define RX51_GPIO_SLEEP_IND 162

struct omap_sdrc_params *rx51_get_sdram_timings(void);

static struct gpio_led gpio_leds[] = {
	{
		.name	= "sleep_ind",
		.gpio	= RX51_GPIO_SLEEP_IND,
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

static struct omap_lcd_config rx51_lcd_config = {
	.ctrl_name	= "internal",
};

static struct omap_fbmem_config rx51_fbmem0_config = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config rx51_fbmem1_config = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config rx51_fbmem2_config = {
	.size = 752 * 1024,
};

static struct omap_board_config_kernel rx51_config[] = {
	{ OMAP_TAG_FBMEM,	&rx51_fbmem0_config },
	{ OMAP_TAG_FBMEM,	&rx51_fbmem1_config },
	{ OMAP_TAG_FBMEM,	&rx51_fbmem2_config },
	{ OMAP_TAG_LCD,		&rx51_lcd_config },
};

static void __init rx51_init_irq(void)
{
	omap_board_config = rx51_config;
	omap_board_config_size = ARRAY_SIZE(rx51_config);
	omap2_init_common_hw(rx51_get_sdram_timings(),
			     rx51_get_sdram_timings(), omap3_mpu_rate_table,
			     omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
}

extern void __init rx51_peripherals_init(void);

static void __init rx51_init(void)
{
	omap_serial_init();
	usb_musb_init();
	rx51_peripherals_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_cfg_reg(H16_34XX_SDRC_CKE0);
	omap_cfg_reg(H17_34XX_SDRC_CKE1);

	platform_device_register(&leds_gpio);
}

static void __init rx51_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(NOKIA_RX51, "Nokia RX-51 board")
	/* Maintainer: Lauri Leukkunen <lauri.leukkunen@nokia.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= rx51_map_io,
	.init_irq	= rx51_init_irq,
	.init_machine	= rx51_init,
	.timer		= &omap_timer,
MACHINE_END
