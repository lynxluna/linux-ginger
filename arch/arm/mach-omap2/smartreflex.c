/*
 * linux/arch/arm/mach-omap3/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
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


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/i2c/twl.h>
#include <linux/io.h>
#include <linux/list.h>

#include <plat/omap34xx.h>
#include <plat/control.h>
#include <plat/clock.h>
#include <plat/opp.h>
#include <plat/opp_twl_tps.h>
#include <plat/omap_device.h>

#include "prm.h"
#include "smartreflex.h"
#include "prm-regbits-34xx.h"

#define MAX_TRIES 100

struct omap_sr {
	int			srid;
	int			is_sr_reset;
	int			is_autocomp_active;
	struct clk		*vdd_opp_clk;
	u32			clk_length;
	u32			req_opp_no;
	u32			senp_mod, senn_mod;
	void __iomem		*srbase_addr;
	unsigned int    	irq;
	struct platform_device 	*pdev;
	struct list_head 	node;
};

/* sr_list contains all the instances of smartreflex module */
static LIST_HEAD(sr_list);

#define SR_REGADDR(offs)	(sr->srbase_addr + offset)

static inline void sr_write_reg(struct omap_sr *sr, unsigned offset, u32 value)
{
	__raw_writel(value, SR_REGADDR(offset));
}

static inline void sr_modify_reg(struct omap_sr *sr, unsigned offset, u32 mask,
					u32 value)
{
	u32 reg_val;

	reg_val = __raw_readl(SR_REGADDR(offset));
	reg_val &= ~mask;
	reg_val |= value;

	__raw_writel(reg_val, SR_REGADDR(offset));
}

static inline u32 sr_read_reg(struct omap_sr *sr, unsigned offset)
{
	return __raw_readl(SR_REGADDR(offset));
}

static struct omap_sr *_sr_lookup(int srid)
{
	struct omap_sr *sr_info, *temp_sr_info;

	sr_info = NULL;

	list_for_each_entry(temp_sr_info, &sr_list, node) {
		if (srid == temp_sr_info->srid) {
			sr_info = temp_sr_info;
			break;
		}
	}

	return sr_info;
}

static int sr_clk_enable(struct omap_sr *sr)
{
	struct omap_smartreflex_data *pdata = sr->pdev->dev.platform_data;

	if (pdata->device_enable)
		pdata->device_enable(sr->pdev);

	return 0;
}

static void sr_clk_disable(struct omap_sr *sr)
{
	struct omap_smartreflex_data *pdata = sr->pdev->dev.platform_data;

	if (pdata->device_idle)
		pdata->device_idle(sr->pdev);

	sr->is_sr_reset = 1;
}

static void cal_reciprocal(u32 sensor, u32 *sengain, u32 *rnsen)
{
	u32 gn, rn, mul;

	for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
		mul = 1 << (gn + 8);
		rn = mul / sensor;
		if (rn < R_MAXLIMIT) {
			*sengain = gn;
			*rnsen = rn;
		}
	}
}

u32 cal_test_nvalue(u32 sennval, u32 senpval)
{
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;

	/* Calculating the gain and reciprocal of the SenN and SenP values */
	cal_reciprocal(senpval, &senpgain, &rnsenp);
	cal_reciprocal(sennval, &senngain, &rnsenn);

	return (senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT);
}

static u8 get_vdd1_opp(void)
{
	struct omap_opp *opp;
	unsigned long freq;
	struct omap_sr *sr_info = _sr_lookup(SR1);

	if (!sr_info) {
		pr_warning("omap_sr struct corresponding to SR1 not found\n");
		return 0;
	}

	if (sr_info->vdd_opp_clk == NULL || IS_ERR(sr_info->vdd_opp_clk))
		return 0;

	freq = sr_info->vdd_opp_clk->rate;
	opp = opp_find_freq_ceil(OPP_MPU, &freq);
	if (IS_ERR(opp))
		return 0;
	/*
	 * Use higher freq voltage even if an exact match is not available
	 * we are probably masking a clock framework bug, so warn
	 */
	if (unlikely(freq != sr_info->vdd_opp_clk->rate))
		pr_warning("%s: Available freq %ld != dpll freq %ld.\n",
			   __func__, freq, sr_info->vdd_opp_clk->rate);

	return opp_get_opp_id(opp);
}

static u8 get_vdd2_opp(void)
{
	struct omap_opp *opp;
	unsigned long freq;
	struct omap_sr *sr_info = _sr_lookup(SR2);

	if (!sr_info) {
		pr_warning("omap_sr struct corresponding to SR2 not found\n");
		return 0;
	}

	if (sr_info->vdd_opp_clk == NULL || IS_ERR(sr_info->vdd_opp_clk))
		return 0;

	freq = sr_info->vdd_opp_clk->rate;
	opp = opp_find_freq_ceil(OPP_L3, &freq);
	if (IS_ERR(opp))
		return 0;

	/*
	 * Use higher freq voltage even if an exact match is not available
	 * we are probably masking a clock framework bug, so warn
	 */
	if (unlikely(freq != sr_info->vdd_opp_clk->rate))
		pr_warning("%s: Available freq %ld != dpll freq %ld.\n",
			   __func__, freq, sr_info->vdd_opp_clk->rate);
	return opp_get_opp_id(opp);
}


static void sr_set_clk_length(struct omap_sr *sr)
{
	struct clk *sys_ck;
	u32 sys_clk_speed;

	sys_ck = clk_get(NULL, "sys_ck");
	sys_clk_speed = clk_get_rate(sys_ck);
	clk_put(sys_ck);

	switch (sys_clk_speed) {
	case 12000000:
		sr->clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000000:
		sr->clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200000:
		sr->clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000000:
		sr->clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400000:
		sr->clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default:
		pr_err("Invalid sysclk value: %d\n", sys_clk_speed);
		break;
	}
}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;
	u32 vsel;
	int uvdc;
	u32 target_opp_no;
	struct omap_opp *opp;

	if (srid == SR1) {
		target_opp_no = get_vdd1_opp();
		if (!target_opp_no)
			target_opp_no = VDD1_OPP3;

		opp = opp_find_by_opp_id(OPP_MPU, target_opp_no);
		BUG_ON(!opp); /* XXX ugh */

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);

		vpconfig = PRM_VP1_CONFIG_ERROROFFSET |
			PRM_VP1_CONFIG_ERRORGAIN |
			PRM_VP1_CONFIG_TIMEOUTEN |
			vsel << OMAP3430_INITVOLTAGE_SHIFT;

		prm_write_mod_reg(vpconfig, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);
		prm_write_mod_reg(PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
					PRM_VP1_VSTEPMIN_VSTEPMIN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VSTEPMIN_OFFSET);

		prm_write_mod_reg(PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
					PRM_VP1_VSTEPMAX_VSTEPMAX,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VSTEPMAX_OFFSET);

		prm_write_mod_reg(PRM_VP1_VLIMITTO_VDDMAX |
					PRM_VP1_VLIMITTO_VDDMIN |
					PRM_VP1_VLIMITTO_TIMEOUT,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_VLIMITTO_OFFSET);

		/* Trigger initVDD value copy to voltage processor */
		prm_set_mod_reg_bits(PRM_VP1_CONFIG_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);

		/* Clear initVDD copy trigger bit */
		prm_clear_mod_reg_bits(PRM_VP1_CONFIG_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP1_CONFIG_OFFSET);

		/* Force update of voltage */
		prm_set_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* Clear force bit */
		prm_clear_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP1_CONFIG_OFFSET);

	} else if (srid == SR2) {
		target_opp_no = get_vdd2_opp();
		if (!target_opp_no)
			target_opp_no = VDD2_OPP3;

		opp = opp_find_by_opp_id(OPP_L3, target_opp_no);
		BUG_ON(!opp); /* XXX ugh */

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);

		vpconfig = PRM_VP2_CONFIG_ERROROFFSET |
			PRM_VP2_CONFIG_ERRORGAIN |
			PRM_VP2_CONFIG_TIMEOUTEN |
			vsel << OMAP3430_INITVOLTAGE_SHIFT;

		prm_write_mod_reg(vpconfig, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);
		prm_write_mod_reg(PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
					PRM_VP2_VSTEPMIN_VSTEPMIN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VSTEPMIN_OFFSET);

		prm_write_mod_reg(PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
					PRM_VP2_VSTEPMAX_VSTEPMAX,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VSTEPMAX_OFFSET);

		prm_write_mod_reg(PRM_VP2_VLIMITTO_VDDMAX |
					PRM_VP2_VLIMITTO_VDDMIN |
					PRM_VP2_VLIMITTO_TIMEOUT,
					OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_VLIMITTO_OFFSET);

		/* Trigger initVDD value copy to voltage processor */
		prm_set_mod_reg_bits(PRM_VP1_CONFIG_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);

		/* Clear initVDD copy trigger bit */
		prm_clear_mod_reg_bits(PRM_VP1_CONFIG_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP2_CONFIG_OFFSET);

		/* Force update of voltage */
		prm_set_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* Clear force bit */
		prm_clear_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP2_CONFIG_OFFSET);

	}
}

static void sr_configure(struct omap_sr *sr)
{
	u32 sr_config;
	u32 senp_en , senn_en;
	struct omap_smartreflex_data *pdata = sr->pdev->dev.platform_data;

	if (sr->clk_length == 0)
		sr_set_clk_length(sr);

	senp_en = pdata->senp_mod;
	senn_en = pdata->senn_mod;

	if (sr->srid == SR1) {
		sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);
		sr_write_reg(sr, AVGWEIGHT, SR1_AVGWEIGHT_SENPAVGWEIGHT |
					SR1_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT | SR1_ERRMINLIMIT));

	} else if (sr->srid == SR2) {
		sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);
		sr_write_reg(sr, AVGWEIGHT, SR2_AVGWEIGHT_SENPAVGWEIGHT |
					SR2_AVGWEIGHT_SENNAVGWEIGHT);
		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT | SR2_ERRMINLIMIT));

	}
	sr->is_sr_reset = 0;
}

static int sr_reset_voltage(int srid)
{
	struct omap_opp *opp;
	unsigned long uvdc;
	u32 target_opp_no, vsel = 0;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 vc_bypass_value;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;
	u32 prm_vp1_voltage, prm_vp2_voltage;

	if (srid == SR1) {
		target_opp_no = get_vdd1_opp();
		if (!target_opp_no) {
			pr_info("Current OPP unknown: Cannot reset voltage\n");
			return 1;
		}

		opp = opp_find_by_opp_id(OPP_MPU, target_opp_no);
		if (!opp)
			return 1;

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);

		reg_addr = R_VDD1_SR_CONTROL;
		prm_vp1_voltage = prm_read_mod_reg(OMAP3430_GR_MOD,
						OMAP3_PRM_VP1_VOLTAGE_OFFSET);
		t2_smps_steps = abs(vsel - prm_vp1_voltage);
	} else if (srid == SR2) {
		target_opp_no = get_vdd2_opp();
		if (!target_opp_no) {
			pr_info("Current OPP unknown: Cannot reset voltage\n");
			return 1;
		}

		opp = opp_find_by_opp_id(OPP_L3, target_opp_no);
		if (!opp)
			return 1;

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);

		reg_addr = R_VDD2_SR_CONTROL;
		prm_vp2_voltage = prm_read_mod_reg(OMAP3430_GR_MOD,
						OMAP3_PRM_VP2_VOLTAGE_OFFSET);
		t2_smps_steps = abs(vsel - prm_vp2_voltage);
	}

	vc_bypass_value = (vsel << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			pr_info("Loop count exceeded in check SR I2C"
								"write\n");
			return 1;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	/*
	 *  T2 SMPS slew rate (min) 4mV/uS, step size 12.5mV,
	 *  2us added as buffer.
	 */
	t2_smps_delay = ((t2_smps_steps * 125) / 40) + 2;
	udelay(t2_smps_delay);

	return 0;
}

static int sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal, v;
	struct omap_opp *opp;
	int uvdc;
	char vsel;
	struct omap_smartreflex_data *pdata = sr->pdev->dev.platform_data;

	if (target_opp_no > pdata->no_opp) {
		pr_notice("Wrong target opp\n");
		return false;
	}

	if (sr->srid == SR1) {
		opp = opp_find_by_opp_id(OPP_MPU, target_opp_no);
		if (!opp)
			return false;
	} else {
		opp = opp_find_by_opp_id(OPP_L3, target_opp_no);
		if (!opp)
			return false;
	}

	nvalue_reciprocal = pdata->sr_nvalue[target_opp_no - 1];
	if (nvalue_reciprocal == 0) {
		pr_notice("OPP%d doesn't support SmartReflex\n",
								target_opp_no);
		return false;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST));

	uvdc = opp_get_voltage(opp);
	vsel = omap_twl_uv_to_vsel(uvdc);

	if (sr->srid == SR1) {
		/* set/latch init voltage */
		v = prm_read_mod_reg(OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
		v &= ~(OMAP3430_INITVOLTAGE_MASK | OMAP3430_INITVDD);

		v |= vsel << OMAP3430_INITVOLTAGE_SHIFT;
		prm_write_mod_reg(v, OMAP3430_GR_MOD,
				  OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* write1 to latch */
		prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* write2 clear */
		prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP1_CONFIG_OFFSET);
		/* Enable VP1 */
		prm_set_mod_reg_bits(PRM_VP1_CONFIG_VPENABLE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP1_CONFIG_OFFSET);
	} else if (sr->srid == SR2) {
		/* set/latch init voltage */
		v = prm_read_mod_reg(OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
		v &= ~(OMAP3430_INITVOLTAGE_MASK | OMAP3430_INITVDD);
		v |= vsel << OMAP3430_INITVOLTAGE_SHIFT;
		prm_write_mod_reg(v, OMAP3430_GR_MOD,
				  OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* write1 to latch */
		prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* write2 clear */
		prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
				       OMAP3_PRM_VP2_CONFIG_OFFSET);
		/* Enable VP2 */
		prm_set_mod_reg_bits(PRM_VP2_CONFIG_VPENABLE, OMAP3430_GR_MOD,
				     OMAP3_PRM_VP2_CONFIG_OFFSET);
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);
	return true;
}

static void sr_disable(struct omap_sr *sr)
{
	u32 i = 0;

	sr->is_sr_reset = 1;

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, ~SRCONFIG_SRENABLE);

	if (sr->srid == SR1) {
		/* Wait for VP idle before disabling VP */
		while ((!prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_STATUS_OFFSET))
					&& i++ < MAX_TRIES)
			udelay(1);

		if (i >= MAX_TRIES)
			pr_warning("VP1 not idle, still going ahead with \
							VP1 disable\n");

		/* Disable VP1 */
		prm_clear_mod_reg_bits(PRM_VP1_CONFIG_VPENABLE, OMAP3430_GR_MOD,
					OMAP3_PRM_VP1_CONFIG_OFFSET);

	} else if (sr->srid == SR2) {
		/* Wait for VP idle before disabling VP */
		while ((!prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_STATUS_OFFSET))
					&& i++ < MAX_TRIES)
			udelay(1);

		if (i >= MAX_TRIES)
			pr_warning("VP2 not idle, still going ahead with \
							 VP2 disable\n");

		/* Disable VP2 */
		prm_clear_mod_reg_bits(PRM_VP2_CONFIG_VPENABLE, OMAP3430_GR_MOD,
					OMAP3_PRM_VP2_CONFIG_OFFSET);
	}
}


void sr_start_vddautocomap(int srid, u32 target_opp_no)
{
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return;
	}
	if (sr->is_sr_reset == 1) {
		sr_clk_enable(sr);
		sr_configure(sr);
	}

	sr->is_autocomp_active = 1;
	if (!sr_enable(sr, target_opp_no)) {
		sr->is_autocomp_active = 0;
		if (sr->is_sr_reset == 1)
			sr_clk_disable(sr);
	}
}
EXPORT_SYMBOL(sr_start_vddautocomap);

int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return false;
	}

	if (sr->is_autocomp_active == 1) {
		sr_disable(sr);
		sr_clk_disable(sr);
		sr->is_autocomp_active = 0;
		/* Reset the volatage for current OPP */
		sr_reset_voltage(srid);
		return true;
	} else
		return false;

}
EXPORT_SYMBOL(sr_stop_vddautocomap);

void enable_smartreflex(int srid)
{
	u32 target_opp_no = 0;
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return;
	}
	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			/* Enable SR clks */
			sr_clk_enable(sr);

			if (srid == SR1)
				target_opp_no = get_vdd1_opp();
			else if (srid == SR2)
				target_opp_no = get_vdd2_opp();

			if (!target_opp_no) {
				pr_info("Current OPP unknown \
						 Cannot configure SR\n");
			}

			sr_configure(sr);

			if (!sr_enable(sr, target_opp_no))
				sr_clk_disable(sr);
		}
	}
}

void disable_smartreflex(int srid)
{
	u32 i = 0;

	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return;
	}
	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 0) {

			sr->is_sr_reset = 1;
			/* SRCONFIG - disable SR */
			sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
							~SRCONFIG_SRENABLE);

			/* Disable SR clk */
			sr_clk_disable(sr);
			if (sr->srid == SR1) {
				/* Wait for VP idle before disabling VP */
				while ((!prm_read_mod_reg(OMAP3430_GR_MOD,
						OMAP3_PRM_VP1_STATUS_OFFSET))
						&& i++ < MAX_TRIES)
					udelay(1);

				if (i >= MAX_TRIES)
					pr_warning("VP1 not idle, still going \
						ahead with VP1 disable\n");

				/* Disable VP1 */
				prm_clear_mod_reg_bits(PRM_VP1_CONFIG_VPENABLE,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VP1_CONFIG_OFFSET);
			} else if (sr->srid == SR2) {
				/* Wait for VP idle before disabling VP */
				while ((!prm_read_mod_reg(OMAP3430_GR_MOD,
						OMAP3_PRM_VP2_STATUS_OFFSET))
						&& i++ < MAX_TRIES)
					udelay(1);

				if (i >= MAX_TRIES)
					pr_warning("VP2 not idle, still going \
						 ahead with VP2 disable\n");

				/* Disable VP2 */
				prm_clear_mod_reg_bits(PRM_VP2_CONFIG_VPENABLE,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VP2_CONFIG_OFFSET);
			}
			/* Reset the volatage for current OPP */
			sr_reset_voltage(srid);
		}
	}
}

/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale_vcbypass(u32 target_opp, u32 current_opp,
					u8 target_vsel, u8 current_vsel)
{
	int sr_status = 0;
	u32 vdd, target_opp_no, current_opp_no;
	u32 vc_bypass_value;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);
	current_opp_no = get_opp_no(current_opp);

	if (vdd == VDD1_OPP) {
		sr_status = sr_stop_vddautocomap(SR1);
		t2_smps_steps = abs(target_vsel - current_vsel);

		prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
				(target_vsel << OMAP3430_VC_CMD_ON_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_0_OFFSET);
		reg_addr = R_VDD1_SR_CONTROL;

	} else if (vdd == VDD2_OPP) {
		sr_status = sr_stop_vddautocomap(SR2);
		t2_smps_steps =  abs(target_vsel - current_vsel);

		prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
				(target_vsel << OMAP3430_VC_CMD_ON_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CMD_VAL_1_OFFSET);
		reg_addr = R_VDD2_SR_CONTROL;
	}

	vc_bypass_value = (target_vsel << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			pr_info("Loop count exceeded in check SR I2C"
								"write\n");
			return 1;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	/*
	 *  T2 SMPS slew rate (min) 4mV/uS, step size 12.5mV,
	 *  2us added as buffer.
	 */
	t2_smps_delay = ((t2_smps_steps * 125) / 40) + 2;
	udelay(t2_smps_delay);

	if (sr_status) {
		if (vdd == VDD1_OPP)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == VDD2_OPP)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return 0;
}

/* Sysfs interface to select SR VDD1 auto compensation */
static ssize_t omap_sr_vdd1_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct omap_sr *sr_info = _sr_lookup(SR1);

	if (!sr_info) {
		pr_warning("omap_sr struct corresponding to SR1 not found\n");
		return 0;
	}
	return sprintf(buf, "%d\n", sr_info->is_autocomp_active);
}

static ssize_t omap_sr_vdd1_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		pr_err("sr_vdd1_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (value == 0) {
		sr_stop_vddautocomap(SR1);
	} else {
		u32 current_vdd1opp_no = get_vdd1_opp();
		if (!current_vdd1opp_no) {
			pr_err("sr_vdd1_autocomp: Current VDD1 opp unknown\n");
			return -EINVAL;
		}
		sr_start_vddautocomap(SR1, current_vdd1opp_no);
	}
	return n;
}

static struct kobj_attribute sr_vdd1_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd1_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd1_autocomp_show,
	.store = omap_sr_vdd1_autocomp_store,
};

/* Sysfs interface to select SR VDD2 auto compensation */
static ssize_t omap_sr_vdd2_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct omap_sr *sr_info = _sr_lookup(SR2);

	if (!sr_info) {
		pr_warning("omap_sr struct corresponding to SR2 not found\n");
		return 0;
	}
	return sprintf(buf, "%d\n", sr_info->is_autocomp_active);
}

static ssize_t omap_sr_vdd2_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		pr_err("sr_vdd2_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (value == 0) {
		sr_stop_vddautocomap(SR2);
	} else {
		u32 current_vdd2opp_no = get_vdd2_opp();
		if (!current_vdd2opp_no) {
			pr_err("sr_vdd2_autocomp: Current VDD2 opp unknown\n");
			return -EINVAL;
		}
		sr_start_vddautocomap(SR2, current_vdd2opp_no);
	}
	return n;
}

static struct kobj_attribute sr_vdd2_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd2_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd2_autocomp_show,
	.store = omap_sr_vdd2_autocomp_store,
};

static int __devinit smartreflex_probe(struct platform_device *pdev)
{
	struct omap_sr *sr_info = kzalloc(sizeof(struct omap_sr), GFP_KERNEL);
	struct omap_device *odev = to_omap_device(pdev);
	int ret = 0;

	sr_info->pdev = pdev;
	sr_info->srid = pdev->id + 1;
	sr_info->is_sr_reset = 1,
	sr_info->is_autocomp_active = 0;
	sr_info->clk_length = 0;
	sr_info->srbase_addr = odev->hwmods[0]->_rt_va;
	if (odev->hwmods[0]->mpu_irqs)
		sr_info->irq = odev->hwmods[0]->mpu_irqs[0].irq;
	sr_set_clk_length(sr_info);

	if (sr_info->srid == SR1) {
		sr_info->vdd_opp_clk = clk_get(NULL, "dpll1_ck");
		ret = sysfs_create_file(power_kobj, &sr_vdd1_autocomp.attr);
		if (ret)
			pr_err("sysfs_create_file failed: %d\n", ret);
	} else {
		sr_info->vdd_opp_clk = clk_get(NULL, "l3_ick");
		ret = sysfs_create_file(power_kobj, &sr_vdd2_autocomp.attr);
		if (ret)
			pr_err("sysfs_create_file failed: %d\n", ret);
	}

	/* Call the VPConfig */
	sr_configure_vp(sr_info->srid);
	odev->hwmods[0]->dev_attr = sr_info;
	list_add(&sr_info->node, &sr_list);
	pr_info("SmartReflex driver initialized\n");

	return ret;
}

static int __devexit smartreflex_remove(struct platform_device *pdev)
{
	struct omap_device *odev = to_omap_device(pdev);
	struct omap_sr *sr_info = odev->hwmods[0]->dev_attr;

	/* Disable Autocompensation if enabled before removing the module */
	if (sr_info->is_autocomp_active == 1)
		sr_stop_vddautocomap(sr_info->srid);
	list_del(&sr_info->node);
	return 0;
}

static struct platform_driver smartreflex_driver = {
	.probe          = smartreflex_probe,
	.remove         = smartreflex_remove,
	.driver		= {
		.name	= "smartreflex",
	},
};

static int __init sr_init(void)
{
	int ret = 0;
	u8 RdReg;

	/* TODO . Find an appropriate place for this */
	/* Enable SR on T2 */
	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &RdReg,
			      R_DCDC_GLOBAL_CFG);

	RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, RdReg,
				R_DCDC_GLOBAL_CFG);

	ret = platform_driver_register(&smartreflex_driver);

	if (ret)
		pr_err("platform driver register failed for smartreflex");
	return ret;
}

void __exit sr_exit(void)
{
	platform_driver_unregister(&smartreflex_driver);
}
late_initcall(sr_init);
module_exit(sr_exit);

MODULE_DESCRIPTION("OMAP SMARTREFLEX DRIVER");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
