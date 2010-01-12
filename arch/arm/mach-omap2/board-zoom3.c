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
#include "omap3-opp.h"

static void __init omap_zoom_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

static struct omap_board_config_kernel zoom_config[] __initdata = {
};

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

static void __init omap_zoom_init_irq(void)
{
	omap_board_config = zoom_config;
	omap_board_config_size = ARRAY_SIZE(zoom_config);
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params,
			     omap36xx_mpu_rate_table,
			     omap36xx_dsp_rate_table,
			     omap36xx_l3_rate_table);
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
	zoom_flash_init(zoom_flash_partitions, ZOOM_NAND_CS);
	zoom_debugboard_init();
	zoom_display_init(OMAP_DSS_VENC_TYPE_COMPOSITE);
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
