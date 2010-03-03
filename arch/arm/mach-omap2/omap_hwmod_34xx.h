/*
 * omap_hwmod_34xx.h - hardware modules present on the OMAP34xx chips
 *
 * Copyright (C) 2009 Nokia Corporation
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __ARCH_ARM_PLAT_OMAP_INCLUDE_MACH_OMAP_HWMOD34XX_H
#define __ARCH_ARM_PLAT_OMAP_INCLUDE_MACH_OMAP_HWMOD34XX_H

#ifdef CONFIG_ARCH_OMAP34XX

#include <plat/omap_hwmod.h>
#include <mach/irqs.h>
#include <plat/cpu.h>
#include <plat/dma.h>

#include "prm-regbits-34xx.h"

static struct omap_hwmod omap34xx_mpu_hwmod;
static struct omap_hwmod omap34xx_l3_hwmod;
static struct omap_hwmod omap34xx_l4_core_hwmod;
static struct omap_hwmod omap34xx_l4_per_hwmod;
static struct omap_hwmod omap34xx_sr1_hwmod;
static struct omap_hwmod omap34xx_sr2_hwmod;

/* L3 -> L4_CORE interface */
static struct omap_hwmod_ocp_if omap34xx_l3__l4_core = {
	.master	= &omap34xx_l3_hwmod,
	.slave	= &omap34xx_l4_core_hwmod,
	.user	= OCP_USER_MPU | OCP_USER_SDMA,
};

/* L3 -> L4_PER interface */
static struct omap_hwmod_ocp_if omap34xx_l3__l4_per = {
	.master = &omap34xx_l3_hwmod,
	.slave	= &omap34xx_l4_per_hwmod,
	.user	= OCP_USER_MPU | OCP_USER_SDMA,
};

/* MPU -> L3 interface */
static struct omap_hwmod_ocp_if omap34xx_mpu__l3 = {
	.master = &omap34xx_mpu_hwmod,
	.slave	= &omap34xx_l3_hwmod,
	.user	= OCP_USER_MPU,
};

/* Slave interfaces on the L3 interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l3_slaves[] = {
	&omap34xx_mpu__l3,
};

/* Master interfaces on the L3 interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l3_masters[] = {
	&omap34xx_l3__l4_core,
	&omap34xx_l3__l4_per,
};

/* L3 */
static struct omap_hwmod omap34xx_l3_hwmod = {
	.name		= "l3_hwmod",
	.masters	= omap34xx_l3_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l3_masters),
	.slaves		= omap34xx_l3_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l3_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

static struct omap_hwmod omap34xx_l4_wkup_hwmod;

/* L4_CORE -> L4_WKUP interface */
static struct omap_hwmod_ocp_if omap34xx_l4_core__l4_wkup = {
	.master	= &omap34xx_l4_core_hwmod,
	.slave	= &omap34xx_l4_wkup_hwmod,
	.user	= OCP_USER_MPU | OCP_USER_SDMA,
};

/* L4 CORE -> SR1 interface */
static struct omap_hwmod_addr_space omap34xx_sr1_addr_space[] = {
	{
		.pa_start	= OMAP34XX_SR1_BASE,
		.pa_end		= OMAP34XX_SR1_BASE + SZ_1K - 1,
		.flags		= ADDR_MAP_ON_INIT | ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap3_l4_core__sr1 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_sr1_hwmod,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id  = NULL,
	.addr		= omap34xx_sr1_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_sr1_addr_space),
	.user		= OCP_USER_MPU,
};

/* L4 CORE -> SR1 interface */
static struct omap_hwmod_addr_space omap34xx_sr2_addr_space[] = {
	{
		.pa_start	= OMAP34XX_SR2_BASE,
		.pa_end		= OMAP34XX_SR2_BASE + SZ_1K - 1,
		.flags		= ADDR_MAP_ON_INIT | ADDR_TYPE_RT,
	},
};

static struct omap_hwmod_ocp_if omap3_l4_core__sr2 = {
	.master		= &omap34xx_l4_core_hwmod,
	.slave		= &omap34xx_sr2_hwmod,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id  = NULL,
	.addr		= omap34xx_sr2_addr_space,
	.addr_cnt	= ARRAY_SIZE(omap34xx_sr2_addr_space),
	.user		= OCP_USER_MPU,
};

/* Slave interfaces on the L4_CORE interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_core_slaves[] = {
	&omap34xx_l3__l4_core,
};

/* Master interfaces on the L4_CORE interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_core_masters[] = {
	&omap34xx_l4_core__l4_wkup,
	&omap3_l4_core__sr1,
	&omap3_l4_core__sr2,
};

/* L4 CORE */
static struct omap_hwmod omap34xx_l4_core_hwmod = {
	.name		= "l4_core_hwmod",
	.masters	= omap34xx_l4_core_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l4_core_masters),
	.slaves		= omap34xx_l4_core_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l4_core_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

/* Slave interfaces on the L4_PER interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_per_slaves[] = {
	&omap34xx_l3__l4_per,
};

/* Master interfaces on the L4_PER interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_per_masters[] = {
};

/* L4 PER */
static struct omap_hwmod omap34xx_l4_per_hwmod = {
	.name		= "l4_per_hwmod",
	.masters	= omap34xx_l4_per_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l4_per_masters),
	.slaves		= omap34xx_l4_per_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l4_per_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

/* Slave interfaces on the L4_WKUP interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_wkup_slaves[] = {
	&omap34xx_l4_core__l4_wkup,
};

/* Master interfaces on the L4_WKUP interconnect */
static struct omap_hwmod_ocp_if *omap34xx_l4_wkup_masters[] = {
};

/* L4 WKUP */
static struct omap_hwmod omap34xx_l4_wkup_hwmod = {
	.name		= "l4_wkup_hwmod",
	.masters	= omap34xx_l4_wkup_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_l4_wkup_masters),
	.slaves		= omap34xx_l4_wkup_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_l4_wkup_slaves),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430)
};

/* Master interfaces on the MPU device */
static struct omap_hwmod_ocp_if *omap34xx_mpu_masters[] = {
	&omap34xx_mpu__l3,
};

/* MPU */
static struct omap_hwmod omap34xx_mpu_hwmod = {
	.name		= "mpu_hwmod",
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "arm_fck",
	.masters	= omap34xx_mpu_masters,
	.masters_cnt	= ARRAY_SIZE(omap34xx_mpu_masters),
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
};

/* SR common */
static struct omap_hwmod_sysc_fields omap3430_sr_sysc_fields = {
	.clkact_shift	= 20,
};

static struct omap_hwmod_sysc_fields omap3630_sr_sysc_fields = {
	.sidle_shift	= 24,
};

static struct omap_hwmod_sysconfig omap3430_sr_if_ctrl = {
	.sysc_offs	= 0x24,
	.sysc_flags	= (SYSC_HAS_CLOCKACTIVITY | SYSC_NO_CACHE),
	.clockact	= CLOCKACT_TEST_ICLK,
	.sysc_fields	= &omap3430_sr_sysc_fields,
};

static struct omap_hwmod_sysconfig omap3630_sr_if_ctrl = {
	.sysc_offs	= 0x38,
	.sysc_flags	= (SYSC_HAS_SIDLEMODE | SYSC_NO_CACHE),
	.clockact	= CLOCKACT_TEST_ICLK,
	.sysc_fields	= &omap3630_sr_sysc_fields,
};

/* SR1 */
static struct omap_hwmod_ocp_if *omap34xx_sr1_slaves[] = {
	&omap3_l4_core__sr1,
};

static struct omap_hwmod omap34xx_sr1_hwmod = {
	.name		= "sr1_hwmod",
	.mpu_irqs	= NULL,
	.sdma_chs	= NULL,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "sr1_fck",
	.slaves		= omap34xx_sr1_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_sr1_slaves),
	.sysconfig	= &omap3430_sr_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
	.flags		= HWMOD_SET_DEFAULT_CLOCKACT,
};

static struct omap_hwmod omap36xx_sr1_hwmod = {
	.name		= "sr1_hwmod",
	.mpu_irqs	= NULL,
	.sdma_chs	= NULL,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "sr1_fck",
	.slaves		= omap34xx_sr1_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_sr1_slaves),
	.sysconfig	= &omap3630_sr_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3630ES1),
};

/* SR2 */
static struct omap_hwmod_ocp_if *omap34xx_sr2_slaves[] = {
	&omap3_l4_core__sr2,
};

static struct omap_hwmod omap34xx_sr2_hwmod = {
	.name		= "sr2_hwmod",
	.mpu_irqs	= NULL,
	.sdma_chs	= NULL,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "sr2_fck",
	.slaves		= omap34xx_sr2_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_sr2_slaves),
	.sysconfig	= &omap3430_sr_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3430),
	.flags		= HWMOD_SET_DEFAULT_CLOCKACT,
};

static struct omap_hwmod omap36xx_sr2_hwmod = {
	.name		= "sr2_hwmod",
	.mpu_irqs	= NULL,
	.sdma_chs	= NULL,
	.clkdev_dev_id	= NULL,
	.clkdev_con_id	= "sr2_fck",
	.slaves		= omap34xx_sr2_slaves,
	.slaves_cnt	= ARRAY_SIZE(omap34xx_sr2_slaves),
	.sysconfig	= &omap3630_sr_if_ctrl,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP3630ES1),
};

static __initdata struct omap_hwmod *omap34xx_hwmods[] = {
	&omap34xx_l3_hwmod,
	&omap34xx_l4_core_hwmod,
	&omap34xx_l4_per_hwmod,
	&omap34xx_l4_wkup_hwmod,
	&omap34xx_mpu_hwmod,
	&omap34xx_sr1_hwmod,
	&omap34xx_sr2_hwmod,
	NULL,
};

static __initdata struct omap_hwmod *omap36xx_hwmods[] = {
	&omap34xx_l3_hwmod,
	&omap34xx_l4_core_hwmod,
	&omap34xx_l4_per_hwmod,
	&omap34xx_l4_wkup_hwmod,
	&omap34xx_mpu_hwmod,
	&omap36xx_sr1_hwmod,
	&omap36xx_sr2_hwmod,
	NULL,
};

#else
# define omap34xx_hwmods		0
#endif

#endif


