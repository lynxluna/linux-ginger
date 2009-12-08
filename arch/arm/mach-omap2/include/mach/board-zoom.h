/*
 * Defines for zoom boards
 */
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

struct flash_partitions {
	struct mtd_partition *parts;
	int nr_parts;
};

extern void __init zoom_flash_init(struct flash_partitions [], int);

#define ZOOM_NAND_CS	0
