/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/board-zoom.h>

#include <plat/common.h>
#include <plat/board.h>

#include "mux.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"
#include "pm.h"

/* Update with the optimal setup values to be used on Zoom3 */
static struct prm_setup_vc omap3_setuptime_table = {
	.clksetup = 0x14A,
	.voltsetup_time1 = 0x00B3,
	.voltsetup_time2 = 0x00A0,
	.voltoffset = 0x118,
	.voltsetup2 = 0x32,
	.vdd0_on = 0x28,	/* 1.1v */
	.vdd0_onlp = 0x20,	/* 1.0v */
	.vdd0_ret = 0x12,	/* 0.83v */
	.vdd0_off = 0x00,	/* 0.6v */
	.vdd1_on = 0x2B,	/* 1.1375v */
	.vdd1_onlp = 0x20,	/* 1.0v */
	.vdd1_ret = 0x14,	/* 0.85v */
	.vdd1_off = 0x00,	/* 0.6v */
};

static void __init omap_zoom_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

static struct omap_board_config_kernel zoom_config[] __initdata = {
};

static void __init omap_zoom_init_irq(void)
{
	omap_board_config = zoom_config;
	omap_board_config_size = ARRAY_SIZE(zoom_config);
	omap3_pm_init_vc(&omap3_setuptime_table);
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params,
			     NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init omap_zoom_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	zoom_peripherals_init();
	zoom_debugboard_init();
}

MACHINE_START(OMAP_ZOOM3, "OMAP Zoom3 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_zoom_map_io,
	.init_irq	= omap_zoom_init_irq,
	.init_machine	= omap_zoom_init,
	.timer		= &omap_timer,
MACHINE_END
