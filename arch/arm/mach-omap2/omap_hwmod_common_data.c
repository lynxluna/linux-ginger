/*
 * omap_hwmod common data structures
 *
 * Copyright (C) 2010 Nokia Corporation
 * Paul Walmsley
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * Created in collaboration with (alphabetical order): Benoit Cousson,
 * Kevin Hilman, Tony Lindgren, Rajendra Nayak, Vikram Pandita, Sakari
 * Poussa, Anand Sawant, Santosh Shilimkar, Richard Woodruff
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This data/structures are to be used while defining OMAP on-chip module
 * data and their integration with other OMAP modules and Linux.
 */

#include <linux/list.h>

#include <plat/omap_hwmod.h>

/**
 * struct omap_hwmod_sysc_legacy - OMAP3 legacy scheme for old IPs
 *
 * To be used by hwmod structure to specify the sysconfig offsets
 * if the device ip follows the Legacy scheme.
 */
struct omap_hwmod_sysc_fields omap_hwmod_sysc_legacy = {
	.midle_shift        = SYSC_LGCY_MIDLEMODE_SHIFT,
	.clkact_shift       = SYSC_LGCY_CLOCKACTIVITY_SHIFT,
	.sidle_shift        = SYSC_LGCY_SIDLEMODE_SHIFT,
	.enwkup_shift       = SYSC_LGCY_ENAWAKEUP_SHIFT,
	.srst_shift         = SYSC_LGCY_SOFTRESET_SHIFT,
	.autoidle_shift     = SYSC_LGCY_AUTOIDLE_SHIFT,
};

/**
 * struct omap_hwmod_sysc_fields - OMAP4 new scheme for Highlander compliant IPs
 *
 * To be used by hwmod structure to specify the sysconfig offsets if the
 * device ip follows the highlander scheme
 */
struct omap_hwmod_sysc_fields omap_hwmod_sysc_highlander = {
	.midle_shift        = SYSC_HIGH_MIDLEMODE_SHIFT,
	.sidle_shift        = SYSC_HIGH_SIDLEMODE_SHIFT,
	.srst_shift         = SYSC_HIGH_SOFTRESET_SHIFT,
};
