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

#include <plat/common.h>
#include <plat/board.h>

#include <mach/board-zoom.h>

#include "sdram-hynix-h8mbx00u0mer-0em.h"
#include "omap3-opp.h"
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
			     omap36xx_mpu_rate_table,
			     omap36xx_dsp_rate_table,
			     omap36xx_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
}

extern int __init omap_zoom2_debugboard_init(void);
extern void __init zoom_peripherals_init(void);

static struct mtd_partition zoom_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),	/* 512KB, 0x80000 */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),	/* 1.25MB, 0x140000 */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",
		.offset		= MTDPART_OFS_APPEND,   /* Offset = 0x1c0000 */
		.size		= 2 * (64 * 2048),	/* 256KB, 0x40000 */
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x0200000*/
		.size		= 240 * (64 * 2048),	/* 30M, 0x1E00000 */
	},
	{
		.name		= "system",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2000000 */
		.size           = 1280 * (64 * 2048),	/* 160M, 0xA000000 */
	},
	{
		.name		= "userdata",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xC000000 */
		.size		= 256 * (64 * 2048),	/* 32M, 0x2000000 */
	},
	{
		.name		= "cache",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xE000000 */
		.size		= 256 * (64 * 2048),	/* 32M, 0x2000000 */
	},
};

static struct flash_partitions zoom_flash_partitions[] = {
	{
		.parts = zoom_nand_partitions,
		.nr_parts = ARRAY_SIZE(zoom_nand_partitions),
	},
};

static void __init omap_zoom_init(void)
{
	zoom_peripherals_init();
	zoom_flash_init(zoom_flash_partitions, ZOOM_NAND_CS);
	omap_zoom2_debugboard_init();
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
