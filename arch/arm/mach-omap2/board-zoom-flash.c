/*
 * arch/arm/mach-omap2/board-zoom-flash.c
 *
 * Copyright (C) 2008-09 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-2430sdp-flash.c
 * Author(s): Rohit Choraria <rohitkc@ti.com>
 *            Vimal Singh <vimalsingh@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include <mach/board-zoom.h>

#define NAND_CMD_UNLOCK1	0x23
#define NAND_CMD_UNLOCK2	0x24
/**
 * @brief platform specific unlock function
 *
 * @param mtd - mtd info
 * @param ofs - offset to start unlock from
 * @param len - length to unlock
 *
 * @return - unlock status
 */
static int omap_ldp_nand_unlock(struct mtd_info *mtd, loff_t ofs, size_t len)
{
	int ret = 0;
	int chipnr;
	int status;
	unsigned long page;
	struct nand_chip *this = mtd->priv;
	printk(KERN_INFO "nand_unlock: start: %08x, length: %d!\n",
			(int)ofs, (int)len);

	/* select the NAND device */
	chipnr = (int)(ofs >> this->chip_shift);
	this->select_chip(mtd, chipnr);
	/* check the WP bit */
	this->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	if ((this->read_byte(mtd) & 0x80) == 0) {
		printk(KERN_ERR "nand_unlock: Device is write protected!\n");
		ret = -EINVAL;
		goto out;
	}

	if ((ofs & (mtd->writesize - 1)) != 0) {
		printk(KERN_ERR "nand_unlock: Start address must be"
				"beginning of nand page!\n");
		ret = -EINVAL;
		goto out;
	}

	if (len == 0 || (len & (mtd->writesize - 1)) != 0) {
		printk(KERN_ERR "nand_unlock: Length must be a multiple of "
				"nand page size!\n");
		ret = -EINVAL;
		goto out;
	}

	/* submit address of first page to unlock */
	page = (unsigned long)(ofs >> this->page_shift);
	this->cmdfunc(mtd, NAND_CMD_UNLOCK1, -1, page & this->pagemask);

	/* submit ADDRESS of LAST page to unlock */
	page += (unsigned long)((ofs + len) >> this->page_shift) ;
	this->cmdfunc(mtd, NAND_CMD_UNLOCK2, -1, page & this->pagemask);

	/* call wait ready function */
	status = this->waitfunc(mtd, this);
	udelay(1000);
	/* see if device thinks it succeeded */
	if (status & 0x01) {
		/* there was an error */
		printk(KERN_ERR "nand_unlock: error status =0x%08x ", status);
		ret = -EIO;
		goto out;
	}

 out:
	/* de-select the NAND device */
	this->select_chip(mtd, -1);
	return ret;
}

static struct mtd_partition ldp_nand_partitions[] = {
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
#ifdef CONFIG_MACH_OMAP_ZOOM2
	{
		.name		= "system",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2000000 */
		.size		= 1280 * (64 * 2048),   /* 160M, 0xA000000 */
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
#else
	{
		.name		= "File System - NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2000000 */
		.size		= MTDPART_SIZ_FULL,	/* 96MB, 0x6000000 */
	},
#endif
};

/* NAND chip access: 16 bit */
static struct omap_nand_platform_data ldp_nand_data = {
	.parts		= ldp_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ldp_nand_partitions),
	.nand_setup	= NULL,
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	= NULL,
	.unlock		= omap_ldp_nand_unlock,
};

static struct resource ldp_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ldp_nand_device = {
	.name		= "omap2-nand",
	.id		= 0,
	.dev		= {
	.platform_data	= &ldp_nand_data,
	},
	.num_resources	= 1,
	.resource	= &ldp_nand_resource,
};

/**
 * ldp430_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init zoom_flash_init(void)
{
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* pop nand part */
	nandcs = LDP3430_NAND_CS;

	ldp_nand_data.cs = nandcs;
	ldp_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
	ldp_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

	if (platform_device_register(&ldp_nand_device) < 0)
		printk(KERN_ERR "Unable to register NAND device\n");
}

