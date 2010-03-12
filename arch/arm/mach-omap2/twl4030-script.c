/*
 * linux/arch/arm/mach-omap2/twl4030-script.c
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_TWL4030_POWER

#include "twl4030-script.h"

struct prm_setup_vc twl4030_voltsetup_time = {
	/* VOLT SETUPTIME for RET & OFF */
	.voltsetup_time1_ret = 0x005B,
	.voltsetup_time2_ret = 0x0055,
	.voltsetup_time1_off = 0x00B3,
	.voltsetup_time2_off = 0x00A0,
	.voltoffset = 0x118,
	.voltsetup2 = 0x32,
};

/*
 * Sequence to controll the TRITON Power resources,
 * when the system goes into sleep.
 * Executed upon P1_P2/P3 transition for sleep.
 */
static struct twl4030_ins __initdata sleep_on_seq[] = {
	/* Broadcast message to put res to sleep */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script __initdata = {
	.script	= sleep_on_seq,
	.size	= ARRAY_SIZE(sleep_on_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/*
 * Sequence to controll the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P1_P2 transition for wakeup.
 */
static struct twl4030_ins wakeup_p12_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p12_script __initdata = {
	.script	= wakeup_p12_seq,
	.size	= ARRAY_SIZE(wakeup_p12_seq),
	.flags	= TWL4030_WAKEUP12_SCRIPT,
};

/*
 * Sequence to controll the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P3 transition for wakeup.
 */
static struct twl4030_ins wakeup_p3_seq[] __initdata = {
	/* Broadcast message to put res to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_p3_script __initdata = {
	.script = wakeup_p3_seq,
	.size   = ARRAY_SIZE(wakeup_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

#ifdef CONFIG_TWL5030_GLITCH_FIX
struct prm_setup_vc twl4030_voltsetup_time_glitchfix = {
	/* VOLT SETUPTIME for RET & OFF */
	.voltsetup_time1_ret = 0x005B,
	.voltsetup_time2_ret = 0x0055,
	.voltsetup_time1_off = 0x00B3,
	.voltsetup_time2_off = 0x00A0,
	.voltoffset = 0x10,
	.voltsetup2 = 0x16B,
};

/*
 * Sequence to controll the TRITON Power resources,
 * when the system goes into sleep.
 * Executed upon P1_P2/P3 transition for sleep.
 */
static struct twl4030_ins __initdata sleep_on_seq_glitchfix[] = {
	/* Singular message to disable HCLKOUT */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
	/* Broadcast message to put res to sleep */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_SLEEP), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_SLEEP), 2},
};

static struct twl4030_script sleep_on_script_glitchfix __initdata = {
	.script	= sleep_on_seq_glitchfix,
	.size	= ARRAY_SIZE(sleep_on_seq_glitchfix),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/*
 * Sequence to controll the TRITON Power resources,
 * when the system wakeup from sleep.
 * Executed upon P1/P2/P3 transition for wakeup.
 */
static struct twl4030_ins wakeup_seq_glitchfix[] __initdata = {
	/* Broadcast message to put res(TYPE2 =1) to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 55},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 55},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 54},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_ACTIVE), 1},
	/* Singular message to enable HCLKOUT */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_ACTIVE), 1},
	/* Broadcast message to put res(TYPE2 =1) to active */
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wakeup_script_glitchfix __initdata = {
	.script	= wakeup_seq_glitchfix,
	.size	= ARRAY_SIZE(wakeup_seq_glitchfix),
	.flags	= TWL4030_WAKEUP12_SCRIPT | TWL4030_WAKEUP3_SCRIPT,
};

#endif

/*
 * Sequence to reset the TRITON Power resources,
 * when the system gets warm reset.
 * Executed upon warm reset signal.
 */
static struct twl4030_ins wrst_seq[] __initdata = {
/*
 * Reset twl4030.
 * Reset Main_Ref.
 * Reset All type2_group2.
 * Reset VUSB_3v1.
 * Reset All type2_group1.
 * Reset RC.
 * Reenable twl4030.
 */
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_Main_Ref, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1,
							RES_STATE_WRST), 2},
	{MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0,
							RES_STATE_WRST), 2},
	{MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 2},
};

static struct twl4030_script wrst_script __initdata = {
	.script = wrst_seq,
	.size   = ARRAY_SIZE(wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

/* TRITON script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts[] __initdata = {
	&sleep_on_script,
	&wakeup_p12_script,
	&wakeup_p3_script,
	&wrst_script,
};

struct twl4030_power_data twl4030_generic_script __initdata = {
	.scripts	= twl4030_scripts,
	.num		= ARRAY_SIZE(twl4030_scripts),
};

void use_generic_twl4030_script(
		struct twl4030_power_data *t2scripts_data,
		struct prm_setup_vc *setup_vc)
{
	setup_vc->voltsetup_time1_ret =
			twl4030_voltsetup_time.voltsetup_time1_ret;
	setup_vc->voltsetup_time2_ret =
			twl4030_voltsetup_time.voltsetup_time2_ret;
	setup_vc->voltsetup_time1_off =
			twl4030_voltsetup_time.voltsetup_time1_off;
	setup_vc->voltsetup_time2_off =
			twl4030_voltsetup_time.voltsetup_time1_off;

	setup_vc->voltoffset = twl4030_voltsetup_time.voltoffset;
	setup_vc->voltsetup2 = twl4030_voltsetup_time.voltsetup2;

	t2scripts_data->scripts = twl4030_generic_script.scripts;
	t2scripts_data->num = twl4030_generic_script.num;
}

#ifdef CONFIG_TWL5030_GLITCH_FIX
/* TRITON script for sleep, wakeup & warm_reset */
static struct twl4030_script *twl4030_scripts_glitchfix[] __initdata = {
	&sleep_on_script_glitchfix,
	&wakeup_script_glitchfix,
	&wrst_script,
};

struct twl4030_power_data twl4030_script_glitchfix __initdata = {
	.scripts	= twl4030_scripts_glitchfix,
	.num		= ARRAY_SIZE(twl4030_scripts_glitchfix),
};

void use_twl4030_script_glitchfix(
		struct twl4030_power_data *t2scripts_data)
{
	t2scripts_data->scripts = twl4030_script_glitchfix.scripts;
	t2scripts_data->num = twl4030_script_glitchfix.num;
}
#endif

#endif
