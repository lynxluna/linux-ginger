#ifndef __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H
#define __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H
/*
 * linux/arch/arm/mach-omap2/smartreflex.h
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/platform_device.h>

/* SR Modules */
#define SR1		1
#define SR2		2

#define GAIN_MAXLIMIT	16
#define R_MAXLIMIT	256

/* SMART REFLEX REG ADDRESS OFFSET */
#define SRCONFIG		0x00
#define SRSTATUS		0x04
#define SENVAL			0x08
#define SENMIN			0x0C
#define SENMAX			0x10
#define SENAVG			0x14
#define AVGWEIGHT		0x18
#define NVALUERECIPROCAL	0x1C
#define SENERROR		0x20
#define ERRCONFIG		0x24
#define IRQ_EOI			0x20
#define IRQSTATUS		0x28
#define IRQENABLE_SET		0x2C
#define IRQENABLE_CLR		0x30
#define SENERROR_36XX		0x34
#define ERRCONFIG_36XX		0x38

/* Bit/Shift Positions */

/* SRCONFIG */
#define SRCONFIG_ACCUMDATA_SHIFT	22
#define SRCONFIG_SRCLKLENGTH_SHIFT	12
#define SRCONFIG_SENNENABLE_SHIFT	5
#define SRCONFIG_SENPENABLE_SHIFT	3
#define SRCONFIG_SENNENABLE_SHIFT_36XX	1
#define SRCONFIG_SENPENABLE_SHIFT_36XX	0
#define SRCONFIG_CLKCTRL_SHIFT		0

#define SRCONFIG_ACCUMDATA_MASK		(0x3FF << 22)

#define SRCONFIG_SRENABLE		BIT(11)
#define SRCONFIG_SENENABLE		BIT(10)
#define SRCONFIG_ERRGEN_EN		BIT(9)
#define SRCONFIG_MINMAXAVG_EN		BIT(8)
#define SRCONFIG_DELAYCTRL		BIT(2)

/* AVGWEIGHT */
#define AVGWEIGHT_SENPAVGWEIGHT_SHIFT	2
#define AVGWEIGHT_SENNAVGWEIGHT_SHIFT	0

/* NVALUERECIPROCAL */
#define NVALUERECIPROCAL_SENPGAIN_SHIFT	20
#define NVALUERECIPROCAL_SENNGAIN_SHIFT	16
#define NVALUERECIPROCAL_RNSENP_SHIFT	8
#define NVALUERECIPROCAL_RNSENN_SHIFT	0

/* ERRCONFIG */
#define ERRCONFIG_ERRWEIGHT_SHIFT	16
#define ERRCONFIG_ERRMAXLIMIT_SHIFT	8
#define ERRCONFIG_ERRMiNLIMIT_SHIFT	0
#define SR_ERRWEIGHT_MASK		(0x07 << 16)
#define SR_ERRMAXLIMIT_MASK		(0xFF << 8)
#define SR_ERRMINLIMIT_MASK		(0xFF << 0)

/* IDLEMODE SETTINGS for OMAP3630 */
#define SR_IDLEMODE_MASK		(0x3 << 24)
#define SR_FORCE_IDLE			0x0
#define SR_NO_IDLE			0x1
#define SR_SMART_IDLE			0x2
#define SR_SMART_IDLE_WKUP		0x3

#define ERRCONFIG_VPBOUNDINTEN		BIT(31)
#define ERRCONFIG_VPBOUNDINTST		BIT(30)
#define ERRCONFIG_VPBOUNDINTST_36XX	BIT(23)
#define ERRCONFIG_VPBOUNDINTEN_36XX	BIT(22)
#define	ERRCONFIG_MCUACCUMINTEN		BIT(29)
#define ERRCONFIG_MCUACCUMINTST		BIT(28)
#define	ERRCONFIG_MCUVALIDINTEN		BIT(27)
#define ERRCONFIG_MCUVALIDINTST		BIT(26)
#define ERRCONFIG_MCUBOUNDINTEN		BIT(25)
#define	ERRCONFIG_MCUBOUNDINTST		BIT(24)
#define	ERRCONFIG_MCUDISACKINTEN	BIT(23)
#define ERRCONFIG_MCUDISACKINTST	BIT(22)

#define ERRCONFIG_STATUS_MASK		(ERRCONFIG_VPBOUNDINTST | \
					ERRCONFIG_MCUACCUMINTST | \
					ERRCONFIG_MCUVALIDINTST | \
					ERRCONFIG_MCUBOUNDINTST | \
					ERRCONFIG_MCUDISACKINTST)
/* IRQSTATUS */
#define IRQSTATUS_MCUACCUMINTST		BIT(3)
#define IRQSTATUS_MCUVALIDINTST		BIT(2)
#define IRQSTATUS_MCUBOUNDINTST		BIT(1)
#define IRQSTATUS_MCUDISACKINTST	BIT(0)

/* IRQSET IRQCLEAR */
#define IRQ_MCUACCUMINTENA		BIT(3)
#define IRQ_MCUVALIDINTENA		BIT(2)
#define IRQ_MCUBOUNDINTENA		BIT(1)
#define IRQ_MCUDISACKINTENA		BIT(0)

/* Common Bit values */
#define SRCLKLENGTH_12MHZ_SYSCLK	0x3C
#define SRCLKLENGTH_13MHZ_SYSCLK	0x41
#define SRCLKLENGTH_19MHZ_SYSCLK	0x60
#define SRCLKLENGTH_26MHZ_SYSCLK	0x82
#define SRCLKLENGTH_38MHZ_SYSCLK	0xC0

/**
* 3430 specific values. Maybe these should be passed from board file or
* pmic structures.
*/
#define OMAP3430_SR_ACCUMDATA		0x1F4

#define OMAP3430_SR1_SENPAVGWEIGHT	0x03
#define OMAP3430_SR1_SENNAVGWEIGHT	0x03

#define OMAP3430_SR2_SENPAVGWEIGHT	0x01
#define OMAP3430_SR2_SENNAVGWEIGHT	0x01

#define OMAP3430_SR_ERRWEIGHT		0x04
#define OMAP3430_SR_ERRMAXLIMIT		0x02
#define OMAP3430_SR_ERRMINLIMIT_HIGHOPP	0xF9
#define OMAP3430_SR_ERRMINLIMIT_LOWOPP	0xF4
/*TODO:3630/OMAP4 values if it has to come from this file*/

/* Info for enabling SR in T2/gaia. ToDo: Move it to twl4030_power.c */
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)
#define R_DCDC_GLOBAL_CFG	PHY_TO_OFF_PM_RECIEVER(0x61)
/* R_DCDC_GLOBAL_CFG register, SMARTREFLEX_ENABLE values */
#define DCDC_GLOBAL_CFG_ENABLE_SRFLX	0x08


#ifdef CONFIG_OMAP_SMARTREFLEX_TESTING
#define SR_TESTING_NVALUES 	1
#else
#define SR_TESTING_NVALUES 	0
#endif

extern struct dentry *pm_dbg_main_dir;
#ifdef CONFIG_OMAP_SMARTREFLEX
/**
 * The smart reflex driver supports both CLASS2 and CLASS3 SR.
 * The smartreflex class driver should pass the class type.
 * Should be used to populate the class_type field of the
 * omap_smartreflex_class_data structure.
 */
#define SR_CLASS2	0x1
#define SR_CLASS3	0x2

/**
 * CLASS2 SR can use either the MINMAXAVG module or the ERROR module
 * of the Smartreflex. Should be used to populate the mod_use field
 * of omap_smartreflex_class_data structure is class_type is chosen
 * as SR_CLASS2.
 */
#define SR_USE_MINMAXAVG_MOD	0x1
#define SR_USE_ERROR_MOD	0x2

/**
 * omap_smartreflex_class_data : Structure to be populated by
 * Smartreflex class driver with corresponding class enable disable API's
 *
 * @enable - API to enable a particular class smaartreflex.
 * @disable - API to disable a particular class smartreflex.
 * @notify - API to notify the class driver about an event in SR. Not needed
 *		for class3.
 * @notify_flags - specify the events to be notified to the class driver
 * @class_type - specify which smartreflex class. Can be used by the SR driver
 * 		to tkae any class based decisions.
 * @mod_use - specify whether to use the error module or minmaxavg module for
 *		smartreflex caliberations in case of class2 SR. In case of
 *		class 3 SR only error module is used.
 */
struct omap_smartreflex_class_data {
	int (*enable)(int sr_id);
	int (*disable)(int sr_id);
	int (*notify)(int sr_id, u32 status);
	u8 notify_flags;
	u8 class_type;
	u8 mod_use;
};

/**
 * omap_smartreflex_data - Smartreflex platform data
 *
 * @senp_mod	: SENPENABLE value for the sr
 * @senn_mod	: SENNENABLE value for sr
 * @sr_nvalue	: array of n target values for sr
 * @no_opp	: number of opp's for this SR
 * init_enable	: whether this sr module needs to enabled at boot up or not
 */
struct omap_smartreflex_data {
	u32		senp_mod;
	u32		senn_mod;
	u32		*sr_nvalue;
	int		no_opp;
	bool		init_enable;
	/* omap_device function pointers */
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown)(struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
};

/*
 * Smartreflex module enable/disable interface.
 * NOTE: if smartreflex is not enabled from sysfs, these functions will not
 * do anything.
 */
void omap_smartreflex_enable(int srid);
void omap_smartreflex_disable(int srid);

/**
 * Smartreflex driver hooks to be called from Smartreflex class driver
 */
int sr_enable(int srid, u32 target_opp_no);
void sr_disable(int srid);

/**
 * API to register the smartreflex class driver with the smartreflex driver
 */
void omap_sr_register_class(struct omap_smartreflex_class_data *class_data);

/**
 * API that board file can use if needed to calculate the N target values
 */
u32 cal_test_nvalue(u32 sennval, u32 senpval);
#else
static inline void omap_smartreflex_enable(int srid) {}
static inline void omap_smartreflex_disable(int srid) {}
#endif

#endif
