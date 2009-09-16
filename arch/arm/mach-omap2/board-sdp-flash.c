/*
 * arch/arm/mach-omap2/board-sdp-flash.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp-flash.c
 * Author: Vimal Singh <vimalsingh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <plat/onenand.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#define REG_FPGA_REV			0x10
#define REG_FPGA_DIP_SWITCH_INPUT2	0x60
#define MAX_SUPPORTED_GPMC_CONFIG	3

/* various memory sizes */
#define FLASH_SIZE_SDPV1	SZ_64M
#define FLASH_SIZE_SDPV2	SZ_128M

#define FLASH_BASE_SDPV1	0x04000000 /* NOR flash (64 Meg aligned) */
#define FLASH_BASE_SDPV2	0x10000000 /* NOR flash (256 Meg aligned) */

#define DEBUG_BASE		0x08000000 /* debug board */

#define PDC_NOR		1
#define PDC_NAND	2
#define PDC_ONENAND	3
#define DBG_MPDB	4

/* REVISIT: Does for some x, chip_sel_sdp[x] maches for 2430 SDP ? */

/* SDP3430 V2 Board CS organization
 * Different from SDP3430 V1. Now 4 switches used to specify CS
 */
static const unsigned char chip_sel_sdp[][GPMC_CS_NUM] = {
/* GPMC CS Indices (ON=0, OFF=1)*/
/* S8-1 2 3 4 IDX   CS0,       CS1,      CS2 ..                    CS7  */
/*ON ON ON ON*/{PDC_NOR, PDC_NAND, PDC_ONENAND, DBG_MPDB, 0, 0, 0, 0},
/*ON ON ON OFF*/{PDC_ONENAND, PDC_NAND, PDC_NOR, DBG_MPDB, 0, 0, 0, 0},
/*ON ON OFF ON */{PDC_NAND, PDC_ONENAND, PDC_NOR, DBG_MPDB, 0, 0, 0, 0},
};

static struct mtd_partition sdp_nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "Bootloader-NOR",
		.offset		= 0,
		.size		= SZ_256K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "Params-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "Kernel-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "Filesystem-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

static struct flash_platform_data sdp_nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= sdp_nor_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_nor_partitions),
};

static struct resource sdp_nor_resource = {
	.start		= 0,
	.end		= 0,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device sdp_nor_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
			.platform_data = &sdp_nor_data,
	},
	.num_resources	= 1,
	.resource	= &sdp_nor_resource,
};

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
		defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

static struct mtd_partition board_onenand_partitions[] = {
	{
		.name		= "X-Loader-OneNAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE  /* force read-only */
	},
	{
		.name		= "U-Boot-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE  /* force read-only */
	},
	{
		.name		= "U-Boot Environment-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 1 * (64 * 2048),
	},
	{
		.name		= "Kernel-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 16 * (64 * 2048),
	},
	{
		.name		= "File System-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_onenand_platform_data board_onenand_data = {
	.parts		= board_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(board_onenand_partitions),
	.dma_channel	= -1,   /* disable DMA in OMAP OneNAND driver */
};


static void __init board_onenand_init(void)
{
	gpmc_onenand_init(&board_onenand_data);
}

#else

static inline void board_onenand_init(void)
{
}
#endif /* CONFIG_MTD_ONENAND_OMAP2 || CONFIG_MTD_ONENAND_OMAP2_MODULE */

static struct mtd_partition sdp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",

		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1c0000 */
		.size		= 6 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 40 * (64 * 2048),
	},
	{
		.name		= "File System - NAND",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x780000 */
	},
};

static struct omap_nand_platform_data sdp_nand_data = {
	.parts		= sdp_nand_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_nand_partitions),
	.nand_setup	= NULL,
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	= NULL,
};

static struct resource sdp_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device sdp_nand_device = {
	.name			= "omap2-nand",
	.id			= 0,
	.dev			= {
		.platform_data	= &sdp_nand_data,
	},
	.num_resources	= 1,
	.resource		= &sdp_nand_resource,
};

/**
 * get_gpmc0_type - Reads the FPGA DIP_SWITCH_INPUT_REGISTER2 to get
 * the various cs values.
 */
static u8 get_gpmc0_type(void)
{
	u8 cs;
	void __iomem *fpga_map_addr;
	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	/* if ioremap does not return a valid pointer, exit */
	if (!fpga_map_addr)
		return -ENOMEM;

	if (!(__raw_readw(fpga_map_addr + REG_FPGA_REV))) {
		/* we dont have an DEBUG FPGA??? */
		/* Depend on #defines!! default to strata boot return param */
		return 0x0;
	}
	/* S8-DIP-OFF = 1, S8-DIP-ON = 0 */
	cs = (u8) (__raw_readw(fpga_map_addr + REG_FPGA_DIP_SWITCH_INPUT2)
			& 0xF);
	/* ES2.0 SDP's onwards 4 dip switches are provided for CS */
	if (omap_rev() >= OMAP3430_REV_ES1_0)
		/* change (S8-1:4=DS-2:0) to (S8-4:1=DS-2:0) */
		cs = ((cs & 8) >> 3) | ((cs & 4) >> 1) |
			((cs & 2) << 1) | ((cs & 1) << 3);
	else
		/* change (S8-1:3=DS-2:0) to (S8-3:1=DS-2:0) */
		cs = ((cs & 4) >> 2) | (cs & 2) | ((cs & 1) << 2);
	iounmap(fpga_map_addr);
	return cs;
}

/**
 * sdp3430_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init sdp_flash_init(void)
{
	u8		cs = 0;
	u8		nandcs = GPMC_CS_NUM + 1;
	u8		onenandcs = GPMC_CS_NUM + 1;
	u8		idx;
	unsigned char	*config_sel = NULL;

	/* REVISIT: Is this return correct idx for 2430 SDP?
	 * for which cs configuration matches for 2430 SDP?
	 */
	idx = get_gpmc0_type();
	if (idx >= MAX_SUPPORTED_GPMC_CONFIG) {
		printk(KERN_ERR "Invalid Chip select Selection..\
				sdp3430_flash_init returned with error\n");
		return;
	}
	config_sel = (unsigned char *)(chip_sel_sdp[idx]);

	/* Configure start address and size of NOR device */
	if (omap_rev() >= OMAP3430_REV_ES1_0) {
		sdp_nor_resource.start	= FLASH_BASE_SDPV2;
		sdp_nor_resource.end	= FLASH_BASE_SDPV2
						+ FLASH_SIZE_SDPV2 - 1;
	} else {
		sdp_nor_resource.start	= FLASH_BASE_SDPV1;
		sdp_nor_resource.end	= FLASH_BASE_SDPV1
						+ FLASH_SIZE_SDPV1 - 1;
	}

	if (platform_device_register(&sdp_nor_device) < 0)
		printk(KERN_ERR "Unable to register NOR device\n");

	while (cs < GPMC_CS_NUM) {
		switch (config_sel[cs]) {
		case PDC_NAND:
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
			break;
		case PDC_ONENAND:
			if (onenandcs > GPMC_CS_NUM)
				onenandcs = cs;
			break;
		};
		cs++;
	}

	if (onenandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "OneNAND: Unable to find configuration "
				" in GPMC\n ");
	} else {
#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
		defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

		board_onenand_data.cs = onenandcs;
#endif /* CONFIG_MTD_ONENAND_OMAP2 || CONFIG_MTD_ONENAND_OMAP2_MODULE */
		board_onenand_init();
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				" in GPMC\n ");
	} else {
		sdp_nand_data.cs = nandcs;
		sdp_nand_data.gpmc_cs_baseaddr = (void *)(OMAP34XX_GPMC_VIRT +
					GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		sdp_nand_data.gpmc_baseaddr = (void *) (OMAP34XX_GPMC_VIRT);

		if (platform_device_register(&sdp_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");

	}
}
