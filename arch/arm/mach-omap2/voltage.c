/*
 * OMAP3/OMAP4 Voltage Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <plat/omap-pm.h>
#include <plat/omap34xx.h>
#include <plat/powerdomain.h>
#include <plat/opp_twl_tps.h>

#include "prm-regbits-34xx.h"
#include "voltage.h"
#include "pm.h"

#define VP_IDLE_TIMEOUT 	200
#define VP_TRANXDONE_TIMEOUT	300

/**
 * OMAP3 Voltage controller SR parameters. TODO: Pass this info as part of
 * board data or PMIC data
 */
#define R_SRI2C_SLAVE_ADDR              0x12
#define R_VDD1_SR_CONTROL               0x00
#define R_VDD2_SR_CONTROL		0x01

/*
 * Voltage processor register offsets and bit field values.
 */
struct vp_reg_info {
	void __iomem *vp_vonfig_reg;
	void __iomem *vp_vstepmin_reg;
	void __iomem *vp_vstepmax_reg;
	void __iomem *vp_vlimitto_reg;
	void __iomem *vp_status_reg;
	void __iomem *vp_voltage_reg;
	void __iomem *prm_irqstatus_reg;
	/* actual values for various fields */
	u32 vp_erroroffset;
	u32 vp_errorgain;
	u32 vp_smpswaittimemin;
	u32 vp_smpswaittimemax;
	u32 vp_stepmax;
	u32 vp_stepmin;
	u32 vp_vddmin;
	u32 vp_vddmax;
	u32 vp_timeout;
	u32 vp_tranxdone_status;
};

static struct vp_reg_info vp_reg[NO_SCALABLE_VDD + 1];

/**
 * Voltage controller register offsets
 */
struct vc_reg_info {
	void __iomem *vc_cmdval0_reg;
	void __iomem *vc_cmdval1_reg;
	void __iomem *vc_bypass_val_reg;
} vc_reg;

/**
 * Default voltage controller settings for OMAP3
 */
static struct prm_setup_vc vc_config = {
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
	.vdd0_on = 0x30,        /* 1.2v */
	.vdd0_onlp = 0x20,      /* 1.0v */
	.vdd0_ret = 0x1e,       /* 0.975v */
	.vdd0_off = 0x00,       /* 0.6v */
	.vdd1_on = 0x2c,        /* 1.15v */
	.vdd1_onlp = 0x20,      /* 1.0v */
	.vdd1_ret = 0x1e,       /* .975v */
	.vdd1_off = 0x00,       /* 0.6v */
};

static inline u32 voltage_read_reg(void __iomem *offset)
{
	return __raw_readl(offset);
}

static inline void voltage_write_reg(void __iomem *offset, u32 value)
{
	__raw_writel(value, offset);
}

static inline void voltage_modify_reg(void __iomem *offset, u32 clear_mask,
					u32 set_mask)
{
	u32 reg_val;

	reg_val = __raw_readl(offset);
	reg_val &= ~clear_mask;
	reg_val |= set_mask;
	__raw_writel(reg_val, offset);
}
/**
 * voltagecontroller_init - initializes the voltage controller.
 *
 * Intializes the voltage controller registers with the PMIC and board
 * specific parameters and voltage setup times. If the board file does not
 * populate the voltage controller parameters through omap3_pm_init_vc,
 * default values specified in vc_config is used.
 */
static void __init init_voltagecontroller(void)
{

	void __iomem *vc_ch_conf_reg, *vc_i2c_cfg_reg, *vc_smps_sa_reg;
	void __iomem *vc_smps_vol_ra_reg;
	void __iomem *prm_clksetup_reg, *prm_voltsetup1_reg;
	void __iomem *prm_voltsetup2_reg, *prm_voltoffset_reg;

	if (cpu_is_omap34xx()) {
		vc_reg.vc_cmdval0_reg = OMAP3430_PRM_VC_CMD_VAL_0;
		vc_reg.vc_cmdval1_reg = OMAP3430_PRM_VC_CMD_VAL_1;
		vc_reg.vc_bypass_val_reg = OMAP3430_PRM_VC_BYPASS_VAL;
		vc_ch_conf_reg = OMAP3430_PRM_VC_CH_CONF;
		vc_i2c_cfg_reg = OMAP3430_PRM_VC_I2C_CFG;
		vc_smps_sa_reg = OMAP3430_PRM_VC_SMPS_SA;
		vc_smps_vol_ra_reg = OMAP3430_PRM_VC_SMPS_VOL_RA;
		prm_clksetup_reg = OMAP3430_PRM_CLKSETUP;
		prm_voltoffset_reg = OMAP3430_PRM_VOLTOFFSET;
		prm_voltsetup1_reg = OMAP3430_PRM_VOLTSETUP1;
		prm_voltsetup2_reg = OMAP3430_PRM_VOLTSETUP2;
	} else {
		pr_warning("support for voltage controller not added\n");
		return;
	}
	voltage_write_reg(vc_smps_sa_reg, (R_SRI2C_SLAVE_ADDR <<
			VC_SMPS_SA1_SHIFT) | (R_SRI2C_SLAVE_ADDR <<
			VC_SMPS_SA0_SHIFT));

	voltage_write_reg(vc_smps_vol_ra_reg, (R_VDD2_SR_CONTROL <<
			VC_VOLRA1_SHIFT) | (R_VDD1_SR_CONTROL <<
			VC_VOLRA0_SHIFT));

	voltage_write_reg(vc_reg.vc_cmdval0_reg,
			(vc_config.vdd0_on << VC_CMD_ON_SHIFT) |
			(vc_config.vdd0_onlp << VC_CMD_ONLP_SHIFT) |
			(vc_config.vdd0_ret << VC_CMD_RET_SHIFT) |
			(vc_config.vdd0_off << VC_CMD_OFF_SHIFT));

	voltage_write_reg(vc_reg.vc_cmdval1_reg,
			(vc_config.vdd1_on << VC_CMD_ON_SHIFT) |
			(vc_config.vdd1_onlp << VC_CMD_ONLP_SHIFT) |
			(vc_config.vdd1_ret << VC_CMD_RET_SHIFT) |
			(vc_config.vdd1_off << VC_CMD_OFF_SHIFT));

	voltage_write_reg(vc_ch_conf_reg, VC_CMD1 | VC_RAV1);

	voltage_write_reg(vc_i2c_cfg_reg, VC_MCODE_SHIFT | VC_HSEN);

	/* Write setup times */
	voltage_write_reg(prm_clksetup_reg, vc_config.clksetup_ret);
	voltage_write_reg(prm_voltsetup1_reg,
		(vc_config.voltsetup_time2_ret << VC_SETUP_TIME2_SHIFT) |
		(vc_config.voltsetup_time1_ret << VC_SETUP_TIME1_SHIFT));
	voltage_write_reg(prm_voltoffset_reg, vc_config.voltoffset);
	voltage_write_reg(prm_voltsetup2_reg, vc_config.voltsetup2);

	pm_dbg_regset_init(1);
	pm_dbg_regset_init(2);
}

static void vp_latch_vsel(int vp_id)
{
	u32 vsel, vpconfig;
	u32 target_opp_no;
	int uvdc;
	struct omap_opp *opp;

	/* Should remove this once OPP framework is fixed */
	if (vp_id == VP1) {
		target_opp_no = get_vdd1_opp();
		if (!target_opp_no)
			/* Assume Nominal OPP as current OPP unknown */
			target_opp_no = VDD1_OPP3;

		opp = opp_find_by_opp_id(OPP_MPU, target_opp_no);
		BUG_ON(!opp); /* XXX ugh */

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);

	} else if (vp_id == VP2) {
		target_opp_no = get_vdd2_opp();
		if (!target_opp_no)
			/* Assume Nominal OPP */
			target_opp_no = VDD2_OPP2;

		opp = opp_find_by_opp_id(OPP_L3, target_opp_no);
		BUG_ON(!opp); /* XXX ugh */

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);
	} else {
		pr_warning("Voltage processor%d does not exisit", vp_id);
		return;
	}
	vpconfig = voltage_read_reg(vp_reg[vp_id].vp_vonfig_reg);
	vpconfig &= ~(VP_INITVOLTAGE_MASK | VP_CONFIG_INITVDD);
	vpconfig |= vsel << VP_INITVOLTAGE_SHIFT;

	voltage_write_reg(vp_reg[vp_id].vp_vonfig_reg, vpconfig);

	/* Trigger initVDD value copy to voltage processor */
	voltage_modify_reg(vp_reg[vp_id].vp_vonfig_reg, VP_CONFIG_INITVDD,
			VP_CONFIG_INITVDD);

	/* Clear initVDD copy trigger bit */
	voltage_modify_reg(vp_reg[vp_id].vp_vonfig_reg, VP_CONFIG_INITVDD, 0x0);
}

static void __init vp_configure(int vp_id)
{
	u32 vpconfig;

	vpconfig = vp_reg[vp_id].vp_erroroffset | vp_reg[vp_id].vp_errorgain |
			VP_CONFIG_TIMEOUTEN;

	voltage_write_reg(vp_reg[vp_id].vp_vonfig_reg, vpconfig);

	voltage_write_reg(vp_reg[vp_id].vp_vstepmin_reg,
			(vp_reg[vp_id].vp_smpswaittimemin |
			vp_reg[vp_id].vp_stepmin));

	voltage_write_reg(vp_reg[vp_id].vp_vstepmax_reg,
			(vp_reg[vp_id].vp_smpswaittimemax |
			vp_reg[vp_id].vp_stepmax));

	voltage_write_reg(vp_reg[vp_id].vp_vlimitto_reg,
			(vp_reg[vp_id].vp_vddmax | vp_reg[vp_id].vp_vddmin |
			vp_reg[vp_id].vp_timeout));

	/* Set the init voltage */

	vp_latch_vsel(vp_id);
	/* Force update of voltage */
	voltage_modify_reg(vp_reg[vp_id].vp_vonfig_reg, VP_FORCEUPDATE,
			VP_FORCEUPDATE);
	/* Clear force bit */
	voltage_modify_reg(vp_reg[vp_id].vp_vonfig_reg, VP_FORCEUPDATE, 0x0);
}

static void __init vp_reg_offs_configure(int vp_id)
{
	struct clk *sys_ck;
	u32 sys_clk_speed, timeout_val;

	if (cpu_is_omap34xx()) {
		if (vp_id == VP1) {
			vp_reg[vp_id].vp_vonfig_reg =
					OMAP3430_PRM_VP1_CONFIG;
			vp_reg[vp_id].vp_vstepmin_reg =
					OMAP3430_PRM_VP1_VSTEPMIN;
			vp_reg[vp_id].vp_vstepmax_reg =
					OMAP3430_PRM_VP1_VSTEPMAX;
			vp_reg[vp_id].vp_vlimitto_reg =
					OMAP3430_PRM_VP1_VLIMITTO;
			vp_reg[vp_id].vp_status_reg =
					OMAP3430_PRM_VP1_STATUS;
			vp_reg[vp_id].vp_voltage_reg =
					OMAP3430_PRM_VP1_VOLTAGE;
			/* OMAP3430 has error gain varying btw higher and
			 * lower opp's
			 */
			vp_reg[vp_id].vp_errorgain = (((get_vdd1_opp() > 2) ?
					(OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP) :
					(OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP)) <<
					OMAP3430_ERRORGAIN_SHIFT);
			vp_reg[vp_id].vp_vddmin = (OMAP3_VP1_VLIMITTO_VDDMIN <<
					OMAP3430_VDDMIN_SHIFT);
			vp_reg[vp_id].vp_vddmax = (OMAP3_VP1_VLIMITTO_VDDMAX <<
					OMAP3430_VDDMAX_SHIFT);
			vp_reg[vp_id].vp_tranxdone_status =
					OMAP3430_VP1_TRANXDONE_ST;
		} else if (vp_id == VP2) {
			vp_reg[vp_id].vp_vonfig_reg =
					OMAP3430_PRM_VP2_CONFIG;
			vp_reg[vp_id].vp_vstepmin_reg =
					OMAP3430_PRM_VP2_VSTEPMIN;
			vp_reg[vp_id].vp_vstepmax_reg =
					OMAP3430_PRM_VP2_VSTEPMAX;
			vp_reg[vp_id].vp_vlimitto_reg =
					OMAP3430_PRM_VP2_VLIMITTO;
			vp_reg[vp_id].vp_status_reg =
					OMAP3430_PRM_VP2_STATUS;
			vp_reg[vp_id].vp_voltage_reg =
					OMAP3430_PRM_VP2_VOLTAGE;
			/* OMAP3430 has error gain varying btw higher and
			 * lower opp's
			 */
			vp_reg[vp_id].vp_errorgain = (((get_vdd2_opp() > 2) ?
					(OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP) :
					(OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP)) <<
					OMAP3430_ERRORGAIN_SHIFT);
			vp_reg[vp_id].vp_vddmin = (OMAP3_VP2_VLIMITTO_VDDMIN <<
					OMAP3430_VDDMIN_SHIFT);
			vp_reg[vp_id].vp_vddmax = (OMAP3_VP2_VLIMITTO_VDDMAX <<
					OMAP3430_VDDMAX_SHIFT);
			vp_reg[vp_id].vp_tranxdone_status =
					OMAP3430_VP2_TRANXDONE_ST;
		} else {
			pr_warning("Voltage processor%d does not exisit\
					in OMAP3 \n", vp_id);
			return;
		}

		vp_reg[vp_id].prm_irqstatus_reg	= OMAP3430_PRM_IRQSTATUS_MPU;
		vp_reg[vp_id].vp_erroroffset = (OMAP3_VP_CONFIG_ERROROFFSET <<
					OMAP3430_INITVOLTAGE_SHIFT);
		vp_reg[vp_id].vp_smpswaittimemin =
					(OMAP3_VP_VSTEPMIN_SMPSWAITTIMEMIN <<
					OMAP3430_SMPSWAITTIMEMIN_SHIFT);
		vp_reg[vp_id].vp_smpswaittimemax =
					(OMAP3_VP_VSTEPMAX_SMPSWAITTIMEMAX <<
					OMAP3430_SMPSWAITTIMEMAX_SHIFT);
		vp_reg[vp_id].vp_stepmin = (OMAP3_VP_VSTEPMIN_VSTEPMIN <<
					OMAP3430_VSTEPMIN_SHIFT);
		vp_reg[vp_id].vp_stepmax = (OMAP3_VP_VSTEPMAX_VSTEPMAX <<
					OMAP3430_VSTEPMAX_SHIFT);

		/* Use sys clk speed to convet the VP timeout in us to no of
		 * clock cycles
		 */
		sys_ck = clk_get(NULL, "sys_ck");
		sys_clk_speed = clk_get_rate(sys_ck);
		clk_put(sys_ck);

		/* Divide to avoid overflow */
		sys_clk_speed /= 1000;
		timeout_val = (sys_clk_speed * OMAP3_VP_VLIMITTO_TIMEOUT_US) /
					1000;

		vp_reg[vp_id].vp_timeout = (timeout_val <<
					OMAP3430_TIMEOUT_SHIFT);
	}
	/* TODO Extend this for OMAP4 ?? Or need a separate file  */
}

#ifdef CONFIG_OMAP_VOLT_VPFORCEUPDATE
/* VP force update method of voltage scaling */
static int vp_forceupdate_scale_voltage(u32 vdd, u8 target_vsel,
				u8 current_vsel)
{
	u32 smps_steps = 0, smps_delay = 0;
	int timeout = 0;

	if (!((vdd == VDD1_OPP) || (vdd == VDD2_OPP))) {
		pr_warning("Wrong vdd id passed to vp forceupdate\n");
		return false;
	}

	smps_steps = abs(target_vsel - current_vsel);

	/* OMAP3430 has errorgain varying btw higher and lower opp's */
	if (cpu_is_omap34xx()) {
		if (vdd == VDD1_OPP) {
			vp_reg[vdd].vp_errorgain = (((get_vdd1_opp() > 2) ?
					(OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP) :
					(OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP)) <<
					OMAP3430_ERRORGAIN_SHIFT);
			voltage_modify_reg(vc_reg.vc_cmdval0_reg,
					VC_CMD_ON_MASK,
					(target_vsel << VC_CMD_ON_SHIFT));
		} else if (vdd == VDD2_OPP) {
			voltage_modify_reg(vc_reg.vc_cmdval1_reg,
					VC_CMD_ON_MASK,
					(target_vsel << VC_CMD_ON_SHIFT));
			vp_reg[vdd].vp_errorgain = (((get_vdd2_opp() > 2) ?
					(OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP) :
					(OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP)) <<
					OMAP3430_ERRORGAIN_SHIFT);
		}
	}

	/* Clear all pending TransactionDone interrupt/status. Typical latency
	 * is <3us
	 */
	while (timeout++ < VP_TRANXDONE_TIMEOUT) {
		voltage_modify_reg(vp_reg[vdd].prm_irqstatus_reg,
				vp_reg[vdd].vp_tranxdone_status,
				vp_reg[vdd].vp_tranxdone_status);
		if (!(voltage_read_reg(vp_reg[vdd].prm_irqstatus_reg) &
				vp_reg[vdd].vp_tranxdone_status))
				break;
		udelay(1);
	}

	if (timeout >= VP_TRANXDONE_TIMEOUT) {
		pr_warning("VP%d TRANXDONE timeout exceeded. Voltage change \
				aborted", vdd);
		return false;
	}

	/* Configure for VP-Force Update */
	voltage_modify_reg(vp_reg[vdd].vp_vonfig_reg, (VP_CONFIG_INITVDD |
			VP_FORCEUPDATE | VP_INITVOLTAGE_MASK |
			VP_ERRORGAIN_MASK), ((target_vsel <<
			VP_INITVOLTAGE_SHIFT) | vp_reg[vdd].vp_errorgain));

	/* Trigger initVDD value copy to voltage processor */
	voltage_modify_reg(vp_reg[vdd].vp_vonfig_reg, VP_CONFIG_INITVDD,
			VP_CONFIG_INITVDD);

	/* Force update of voltage */
	voltage_modify_reg(vp_reg[vdd].vp_vonfig_reg, VP_FORCEUPDATE,
			VP_FORCEUPDATE);

	timeout = 0;
	/* Wait for TransactionDone. Typical latency is <200us.
	 * Depends on SMPSWAITTIMEMIN/MAX and voltage change
	 */
	while ((timeout++ < VP_TRANXDONE_TIMEOUT) &&
			(!(voltage_read_reg(vp_reg[vdd].prm_irqstatus_reg) &
			vp_reg[vdd].vp_tranxdone_status)))
		udelay(1);

	if (timeout >= VP_TRANXDONE_TIMEOUT)
		pr_warning("VP%d TRANXDONE timeout exceeded. TRANXDONE never \
			got set after the voltage update.Serious error!!!!\n",
			vdd);

	/* Wait for voltage to settle with SW wait-loop */
	smps_delay = ((smps_steps * 125) / 40) + 2;
	udelay(smps_delay);

	/* Disable TransactionDone interrupt , clear all status, clear
	 * control registers
	 */
	timeout = 0;
	while (timeout++ < VP_TRANXDONE_TIMEOUT) {
		voltage_modify_reg(vp_reg[vdd].prm_irqstatus_reg,
				vp_reg[vdd].vp_tranxdone_status,
				vp_reg[vdd].vp_tranxdone_status);
		if (!(voltage_read_reg(vp_reg[vdd].prm_irqstatus_reg) &
				vp_reg[vdd].vp_tranxdone_status))
				break;
		udelay(1);
	}
	if (timeout >= VP_TRANXDONE_TIMEOUT)
		pr_warning("VP%d TRANXDONE timeout exceeded while trying to \
			clear the TRANXDONE status\n", vdd);

	/* Clear initVDD copy trigger bit */
	voltage_modify_reg(vp_reg[vdd].vp_vonfig_reg, VP_CONFIG_INITVDD, 0x0);
	/* Clear force bit */
	voltage_modify_reg(vp_reg[vdd].vp_vonfig_reg, VP_FORCEUPDATE, 0x0);

	return true;
}
#endif

#ifdef CONFIG_OMAP_VOLT_VCBYPASS
/**
 * vc_bypass_scale_voltage - VC bypass method of voltage scaling
 */
static int vc_bypass_scale_voltage(u32 vdd, u8 target_vsel, u8 current_vsel)
{
	u32 vc_bypass_value;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 smps_steps = 0;
	u32 smps_delay = 0;

	smps_steps = abs(target_vsel - current_vsel);

	if (vdd == VDD1_OPP) {
		voltage_modify_reg(vc_reg.vc_cmdval0_reg, VC_CMD_ON_MASK,
				(target_vsel << VC_CMD_ON_SHIFT));
		reg_addr = R_VDD1_SR_CONTROL;
		/* OMAP3430 has errorgain varying btw higher and lower opp's */
		if (cpu_is_omap34xx())
			vp_reg[vdd].vp_errorgain = (((get_vdd1_opp() > 2) ?
					(OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP) :
					(OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP)) <<
					OMAP3430_ERRORGAIN_SHIFT);
	} else if (vdd == VDD2_OPP) {
		voltage_modify_reg(vc_reg.vc_cmdval1_reg, VC_CMD_ON_MASK,
				(target_vsel << VC_CMD_ON_SHIFT));
		reg_addr = R_VDD2_SR_CONTROL;
		/* OMAP3430 has errorgain varying btw higher and lower opp's */
		if (cpu_is_omap34xx())
			vp_reg[vdd].vp_errorgain = (((get_vdd2_opp() > 2) ?
					(OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP) :
					(OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP)) <<
					OMAP3430_ERRORGAIN_SHIFT);
	} else {
		pr_warning("Wrong VDD passed in vc_bypass_scale_voltage %d\n",
				vdd);
		return false;
	}

	/* OMAP3430 has errorgain varying btw higher and lower opp's */
	if (cpu_is_omap34xx())
		voltage_modify_reg(vp_reg[vdd].vp_vonfig_reg,
			VP_ERRORGAIN_MASK, vp_reg[vdd].vp_errorgain);

	vc_bypass_value = (target_vsel << VC_DATA_SHIFT) |
			(reg_addr << VC_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << VC_SLAVEADDR_SHIFT);

	voltage_write_reg(vc_reg.vc_bypass_val_reg, vc_bypass_value);

	voltage_modify_reg(vc_reg.vc_bypass_val_reg, VC_VALID, VC_VALID);
	vc_bypass_value = voltage_read_reg(vc_reg.vc_bypass_val_reg);

	while ((vc_bypass_value & VC_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			pr_warning("Loop count exceeded in check SR I2C write\
						during voltgae scaling\n");
			return false;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = voltage_read_reg(vc_reg.vc_bypass_val_reg);
	}

	/*
	 *  T2 SMPS slew rate (min) 4mV/uS, step size 12.5mV,
	 *  2us added as buffer.
	 */
	smps_delay = ((smps_steps * 125) / 40) + 2;
	udelay(smps_delay);
	return true;
}
#endif

static void __init init_voltageprocessors(void)
{
	int i;

	for (i = 1; i < NO_SCALABLE_VDD + 1; i++) {
		vp_reg_offs_configure(i);
		vp_configure(i);
	}
}


/* Public functions */

/**
 * omap_voltageprocessor_enable : API to enable a particular VP
 * @vp_id : The id of the VP to be enable.
 *
 * This API enables a particular voltage processor. Needed by the smartreflex
 * class drivers.
 */
void omap_voltageprocessor_enable(int vp_id)
{
	/* If VP is already enabled, do nothing. Return */
	if (voltage_read_reg(vp_reg[vp_id].vp_vonfig_reg) &
				VP_CONFIG_VPENABLE)
		return;

#ifdef CONFIG_OMAP_VOLT_VCBYPASS
	/* This latching is required only if VC bypass method is used for
	 * voltage scaling during dvfs.
	 */
	vp_latch_vsel(vp_id);
#endif
	/* Enable VP */
	voltage_modify_reg(vp_reg[vp_id].vp_vonfig_reg, VP_CONFIG_VPENABLE,
				VP_CONFIG_VPENABLE);
}

/**
 * omap_voltageprocessor_disable : API to disable a particular VP
 * @vp_id : The id of the VP to be disable.
 *
 * This API disables a particular voltage processor. Needed by the smartreflex
 * class drivers.
 */
void omap_voltageprocessor_disable(int vp_id)
{
	int timeout = 0;

	/* If VP is already disabled, do nothing. Return */
	if (!(voltage_read_reg(vp_reg[vp_id].vp_vonfig_reg) &
				VP_CONFIG_VPENABLE))
		return;

	/* Disable VP */
	voltage_modify_reg(vp_reg[vp_id].vp_vonfig_reg, VP_CONFIG_VPENABLE,
				0x0);

	/* Wait for VP idle Typical latency is <2us. Maximum latency is ~100us
	 */
	while ((timeout++ < VP_IDLE_TIMEOUT) &&
			(!(voltage_read_reg(vp_reg[vp_id].vp_status_reg))))
		udelay(1);

	if (timeout >= VP_IDLE_TIMEOUT)
		pr_warning("VP%d idle timedout\n", vp_id);
	return;
}

/**
 * omap_voltage_scale : API to scale voltage of a particular voltage domain.
 * @vdd : the voltage domain whose voltage is to be scaled
 * @target_vsel : The target voltage of the voltage domain
 * @current_vsel : the current voltage of the voltage domain.
 *
 * This API should be called by the kernel to do the voltage scaling
 * for a particular voltage domain during dvfs or any other situation.
 */
int omap_voltage_scale(int vdd, u8 target_vsel, u8 current_vsel)
{
#ifdef CONFIG_OMAP_VOLT_VCBYPASS
	return vc_bypass_scale_voltage(vdd, target_vsel, current_vsel);
#elif CONFIG_OMAP_VOLT_VPFORCEUPDATE
	return vp_forceupdate_scale_voltage(vdd, target_vsel, current_vsel);
#endif
}

/**
 * omap_reset_voltage : Resets the voltage of a particular voltage domain
 * to that of the current OPP.
 * @vdd : the voltage domain whose voltage is to be reset.
 *
 * This API finds out the correct voltage the voltage domain is supposed
 * to be at and resets the voltage to that level. Should be used expecially
 * while disabling any voltage compensation modules.
 */
void omap_reset_voltage(int vdd)
{
	u32 vsel, target_opp_no;
	int uvdc;
	struct omap_opp *opp;

	/* Should remove this once OPP framework is fixed */
	if (vdd == VP1) {
		target_opp_no = get_vdd1_opp();
		if (!target_opp_no)
			/* Assume Nominal OPP as current OPP unknown */
			target_opp_no = VDD1_OPP3;

		opp = opp_find_by_opp_id(OPP_MPU, target_opp_no);
		BUG_ON(!opp); /* XXX ugh */

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);

	} else if (vdd == VP2) {
		target_opp_no = get_vdd2_opp();
		if (!target_opp_no)
			/* Assume Nominal OPP */
			target_opp_no = VDD2_OPP2;

		opp = opp_find_by_opp_id(OPP_L3, target_opp_no);
		BUG_ON(!opp); /* XXX ugh */

		uvdc = opp_get_voltage(opp);
		vsel = omap_twl_uv_to_vsel(uvdc);
	} else {
		pr_warning("Wrong VDD passed in omap_reset_voltage %d\n", vdd);
		return;
	}
	omap_voltage_scale(vdd, vsel, voltage_read_reg(
				vp_reg[vdd].vp_voltage_reg));
}

/**
 * omap3_pm_init_vc - polpulates vc_config with values specified in board file
 * @setup_vc - the structure with various vc parameters
 *
 * Updates vc_config with the voltage setup times and other parameters as
 * specified in setup_vc. vc_config is later used in init_voltagecontroller
 * to initialize the voltage controller registers. Board files should call
 * this function with the correct volatge settings corresponding
 * the particular PMIC and chip.
 */
void __init omap_voltage_init_vc(struct prm_setup_vc *setup_vc)
{
	if (!setup_vc)
		return;

	/* CLK SETUPTIME for RET & OFF */
	vc_config.clksetup_ret = setup_vc->clksetup_ret;
	vc_config.clksetup_off = setup_vc->clksetup_off;
	/* VOLT SETUPTIME for RET & OFF */
	vc_config.voltsetup_time1_ret = setup_vc->voltsetup_time1_ret;
	vc_config.voltsetup_time2_ret = setup_vc->voltsetup_time2_ret;
	vc_config.voltsetup_time1_off = setup_vc->voltsetup_time1_off;
	vc_config.voltsetup_time2_off = setup_vc->voltsetup_time2_off;
	vc_config.voltoffset = setup_vc->voltoffset;
	vc_config.voltsetup2 = setup_vc->voltsetup2;

	/* VC COMMAND VALUES for VDD1/VDD2 */
	vc_config.vdd0_on = setup_vc->vdd0_on;
	vc_config.vdd0_onlp = setup_vc->vdd0_onlp;
	vc_config.vdd0_ret = setup_vc->vdd0_ret;
	vc_config.vdd0_off = setup_vc->vdd0_off;
	vc_config.vdd1_on = setup_vc->vdd1_on;
	vc_config.vdd1_onlp = setup_vc->vdd1_onlp;
	vc_config.vdd1_ret = setup_vc->vdd1_ret;
	vc_config.vdd1_off = setup_vc->vdd1_off;
}

void update_voltsetup_time(int core_next_state)
{
	/* update voltsetup time */
	if (core_next_state == PWRDM_POWER_OFF) {
		prm_write_mod_reg(vc_config.clksetup_off, OMAP3430_GR_MOD,
				OMAP3_PRM_CLKSETUP_OFFSET);
		prm_write_mod_reg((vc_config.voltsetup_time2_off <<
				OMAP3430_SETUP_TIME2_SHIFT) |
				(vc_config.voltsetup_time1_off <<
				OMAP3430_SETUP_TIME1_SHIFT),
				OMAP3430_GR_MOD, OMAP3_PRM_VOLTSETUP1_OFFSET);

		if (voltage_off_while_idle)
			prm_write_mod_reg(vc_config.voltsetup2, OMAP3430_GR_MOD,
					OMAP3_PRM_VOLTSETUP2_OFFSET);

	} else if (core_next_state == PWRDM_POWER_RET) {
		prm_write_mod_reg(vc_config.clksetup_ret, OMAP3430_GR_MOD,
				OMAP3_PRM_CLKSETUP_OFFSET);
		prm_write_mod_reg((vc_config.voltsetup_time2_ret <<
				OMAP3430_SETUP_TIME2_SHIFT) |
				(vc_config.voltsetup_time1_ret <<
				OMAP3430_SETUP_TIME1_SHIFT),
				OMAP3430_GR_MOD, OMAP3_PRM_VOLTSETUP1_OFFSET);

		/* clear voltsetup2_reg if sys_off not enabled */
		prm_write_mod_reg(0x0, OMAP3430_GR_MOD,
				OMAP3_PRM_VOLTSETUP2_OFFSET);
	}
}

/**
 * omap_voltage_init : Volatage init API which does VP and VC init.
 */
void __init omap_voltage_init(void)
{
	init_voltagecontroller();
	init_voltageprocessors();
}
