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

#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)

/* SMART REFLEX REG ADDRESS OFFSET */
#define SRCONFIG	0x00
#define SRSTATUS	0x04
#define SENVAL		0x08
#define SENMIN		0x0C
#define SENMAX		0x10
#define SENAVG		0x14
#define AVGWEIGHT	0x18
#define NVALUERECIPROCAL	0x1C
#define SENERROR	0x20
#define ERRCONFIG	0x24
#define SENERROR_36XX	0x34
#define ERRCONFIG_36XX	0x38

/* SR Modules */
#define SR1		1
#define SR2		2

#define GAIN_MAXLIMIT	16
#define R_MAXLIMIT	256

/* SRCONFIG */
#define SR1_SRCONFIG_ACCUMDATA		(0x1F4 << 22)
#define SR2_SRCONFIG_ACCUMDATA		(0x1F4 << 22)

#define SRCLKLENGTH_12MHZ_SYSCLK	0x3C
#define SRCLKLENGTH_13MHZ_SYSCLK	0x41
#define SRCLKLENGTH_19MHZ_SYSCLK	0x60
#define SRCLKLENGTH_26MHZ_SYSCLK	0x82
#define SRCLKLENGTH_38MHZ_SYSCLK	0xC0

#define SRCONFIG_SRCLKLENGTH_SHIFT	12
#define SRCONFIG_SENNENABLE_SHIFT	5
#define SRCONFIG_SENPENABLE_SHIFT	3
#define SRCONFIG_SENNENABLE_SHIFT_36XX	1
#define SRCONFIG_SENPENABLE_SHIFT_36XX	0

#define SRCONFIG_SRENABLE		BIT(11)
#define SRCONFIG_SENENABLE		BIT(10)
#define SRCONFIG_ERRGEN_EN		BIT(9)
#define SRCONFIG_MINMAXAVG_EN		BIT(8)

#define SRCONFIG_DELAYCTRL		BIT(2)
#define SRCONFIG_CLKCTRL		(0x00 << 0)

/* AVGWEIGHT */
#define SR1_AVGWEIGHT_SENPAVGWEIGHT	(0x03 << 2)
#define SR1_AVGWEIGHT_SENNAVGWEIGHT	(0x03 << 0)

#define SR2_AVGWEIGHT_SENPAVGWEIGHT	BIT(2)
#define SR2_AVGWEIGHT_SENNAVGWEIGHT	BIT(0)

/* NVALUERECIPROCAL */
#define NVALUERECIPROCAL_SENPGAIN_SHIFT	20
#define NVALUERECIPROCAL_SENNGAIN_SHIFT	16
#define NVALUERECIPROCAL_RNSENP_SHIFT	8
#define NVALUERECIPROCAL_RNSENN_SHIFT	0

/* ERRCONFIG */
#define SR_CLKACTIVITY_MASK		(0x03 << 20)
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
#define ERRCONFIG_VPBOUNDINTEN_36XX	BIT(23)
#define ERRCONFIG_VPBOUNDINTST_36XX	BIT(22)

#define SR1_ERRWEIGHT			(0x07 << 16)
#define SR1_ERRMAXLIMIT			(0x02 << 8)
#define SR1_ERRMINLIMIT			(0xFA << 0)

#define SR2_ERRWEIGHT			(0x07 << 16)
#define SR2_ERRMAXLIMIT			(0x02 << 8)
#define SR2_ERRMINLIMIT			(0xF9 << 0)


/* Vmode control */
#define R_DCDC_GLOBAL_CFG	PHY_TO_OFF_PM_RECIEVER(0x61)

/* R_DCDC_GLOBAL_CFG register, SMARTREFLEX_ENABLE values */
#define DCDC_GLOBAL_CFG_ENABLE_SRFLX	0x08


#ifdef CONFIG_OMAP_SMARTREFLEX_TESTING
#define SR_TESTING_NVALUES 	1
#else
#define SR_TESTING_NVALUES 	0
#endif

#ifdef CONFIG_OMAP_SMARTREFLEX
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
 */
struct omap_smartreflex_class_data {
	int (*enable)(int sr_id);
	int (*disable)(int sr_id);
	int (*notify)(int sr_id, u32 status);
	u8 notify_flags;
	u8 class_type;
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
