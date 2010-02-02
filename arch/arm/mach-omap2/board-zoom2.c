/*
 * Copyright (C) 2009 Texas Instruments Inc.
 * Mikkel Christensen <mlc@ti.com>
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
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>

#include <mach/board-zoom.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "omap3-opp.h"
#include "pm.h"

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

static void __init omap_zoom2_init_irq(void)
{
	omap3_pm_init_opp_table();
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
				 mt46h32m32lf6_sdrc_params);
	omap_init_irq();
	omap_gpio_init();
}

/* REVISIT: These audio entries can be removed once MFD code is merged */
#if 0

static struct twl4030_madc_platform_data zoom2_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data zoom2_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data zoom2_codec_data = {
	.audio_mclk = 26000000,
	.audio = &zoom2_audio_data,
};

static struct twl4030_platform_data zoom2_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &zoom2_bci_data,
	.madc		= &zoom2_madc_data,
	.usb		= &zoom2_usb_data,
	.gpio		= &zoom2_gpio_data,
	.keypad		= &zoom2_kp_twl4030_data,
	.codec		= &zoom2_codec_data,
	.vmmc1          = &zoom2_vmmc1,
	.vmmc2          = &zoom2_vmmc2,
	.vsim           = &zoom2_vsim,

};

#endif

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

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

static void __init omap_zoom2_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	zoom_peripherals_init(&omap3_setuptime_table);
	zoom_flash_init(zoom_flash_partitions, ZOOM_NAND_CS);
	zoom_debugboard_init();
}

static void __init omap_zoom2_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_ZOOM2, "OMAP Zoom2 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_zoom2_map_io,
	.init_irq	= omap_zoom2_init_irq,
	.init_machine	= omap_zoom2_init,
	.timer		= &omap_timer,
MACHINE_END
