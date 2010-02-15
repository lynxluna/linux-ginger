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

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/kobject.h>
#include <linux/i2c/twl.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#include <plat/opp.h>
#include <plat/opp_twl_tps.h>
#include <plat/omap_device.h>

#include "smartreflex.h"

#define SMARTREFLEX_NAME_LEN	16
#define SR_DISABLE_TIMEOUT	200

struct omap_sr {
	int			srid;
	int			is_sr_reset;
	int			is_autocomp_active;
	u32			clk_length;
	u32			err_weight;
	u32			err_minlimit;
	u32			err_maxlimit;
	u32			accum_data;
	u32			senn_avgweight;
	u32			senp_avgweight;
	void __iomem		*srbase_addr;
	unsigned int    	irq;
	struct platform_device 	*pdev;
	struct list_head 	node;
};

/* sr_list contains all the instances of smartreflex module */
static LIST_HEAD(sr_list);

static struct omap_smartreflex_class_data *sr_class;

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
	/* Smartreflex error config register is special as it contains
	 * certain status bits which if written a 1 into means a clear
	 * of those bits. So in order to make sure no accidental write of
	 * 1 happens to those status bits, do a clear of them in the read
	 * value. Now if there is an actual reguest to write to these bits
	 * they will be set in the nex step.
	 */
	if (cpu_is_omap3430()) {
		if (offset == ERRCONFIG)
			reg_val &= ~ERRCONFIG_STATUS_MASK;
	} else if (cpu_is_omap3630()) {
		if (offset == ERRCONFIG_36XX)
			reg_val &= ~ERRCONFIG_VPBOUNDINTST_36XX;
	}

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

	sr->is_sr_reset = 1;
	if (pdata->device_idle)
		pdata->device_idle(sr->pdev);
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

static irqreturn_t sr_omap_isr(int irq, void *data)
{
	struct omap_sr *sr_info = (struct omap_sr *)data;
	u32 status = 0;

	if (cpu_is_omap3430()) {
		/*Read the status bits*/
		status = sr_read_reg(sr_info, ERRCONFIG);

		/*Clear them by writing back*/
		sr_write_reg(sr_info, ERRCONFIG, status);
	} else if (cpu_is_omap3630()) {
		/*Read the status bits*/
		status = sr_read_reg(sr_info, ERRCONFIG_36XX);

		/*Clear them by writing back*/
		sr_write_reg(sr_info, ERRCONFIG_36XX, status);
	}

	/* Call the class driver notify function if registered*/
	if (sr_class->class_type == SR_CLASS2 && sr_class->notify)
		sr_class->notify(sr_info->srid, status);

	return IRQ_HANDLED;
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

static void sr_set_regfields(struct omap_sr *sr)
{
	/**
	* For time being these values are defined in smartreflex.h
	* and populated during init. May be they can be moved to board
	* file or pmic specific data structure. In that case these structure
	* fields will have to be populated using the pdata or pmic structure.
	*/
	if (cpu_is_omap343x() || cpu_is_omap3630()) {
		sr->err_weight = OMAP3430_SR_ERRWEIGHT;
		sr->err_maxlimit = OMAP3430_SR_ERRMAXLIMIT;
		sr->accum_data = OMAP3430_SR_ACCUMDATA;
		if (sr->srid == SR1) {
			sr->senn_avgweight = OMAP3430_SR1_SENNAVGWEIGHT;
			sr->senp_avgweight = OMAP3430_SR1_SENPAVGWEIGHT;
		} else {
			sr->senn_avgweight = OMAP3430_SR2_SENNAVGWEIGHT;
			sr->senp_avgweight = OMAP3430_SR2_SENPAVGWEIGHT;
		}
	}
	/*TODO: 3630 and Omap4 specific bit field values */
}

static void sr_configure(struct omap_sr *sr)
{
	u32 sr_config;
	u32 senp_en , senn_en;
	struct omap_smartreflex_data *pdata = sr->pdev->dev.platform_data;

	/* Common settings for SR Class3 and SR Class2 */
	if (sr->clk_length == 0)
		sr_set_clk_length(sr);

	senp_en = pdata->senp_mod;
	senn_en = pdata->senn_mod;
	if (cpu_is_omap3430()) {
		sr_config = (sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;
	} else if (cpu_is_omap3630()) {
		sr_config = (sr->clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT_36XX) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT_36XX);
	}

	sr_write_reg(sr, SRCONFIG, sr_config);

	if ((sr_class->class_type == SR_CLASS3) || (sr_class->class_type ==
		SR_CLASS2 && sr_class->mod_use == SR_USE_ERROR_MOD)) {
		/**
		* SR settings if using the ERROR module inside Smartreflex.
		* SR CLASS 3 by default uses only the ERROR module where as
		* SR CLASS 2 can choose between ERROR module and MINMAXAVG
		* module.
		*/
		u32 sr_errconfig;

		sr_modify_reg(sr, SRCONFIG, SRCONFIG_ERRGEN_EN,
			SRCONFIG_ERRGEN_EN);
		sr_errconfig = (sr->err_weight << ERRCONFIG_ERRWEIGHT_SHIFT) |
			(sr->err_maxlimit << ERRCONFIG_ERRMAXLIMIT_SHIFT) |
			(sr->err_minlimit <<  ERRCONFIG_ERRMiNLIMIT_SHIFT);
		if (cpu_is_omap3430()) {
			sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
				SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
				sr_errconfig);
			/* Enabling the interrupts if the ERROR module is used
			*/
			sr_modify_reg(sr, ERRCONFIG,
				(ERRCONFIG_VPBOUNDINTEN),
				(ERRCONFIG_VPBOUNDINTEN |
				ERRCONFIG_VPBOUNDINTST));
		} else if (cpu_is_omap3630()) {
			sr_modify_reg(sr, ERRCONFIG_36XX, (SR_ERRWEIGHT_MASK |
				SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
				sr_errconfig);
			/* Enabling the interrupts if the ERROR module is used
			*/
			sr_modify_reg(sr, ERRCONFIG_36XX,
				(ERRCONFIG_VPBOUNDINTEN_36XX),
				(ERRCONFIG_VPBOUNDINTEN_36XX |
				ERRCONFIG_VPBOUNDINTST_36XX));
		}

	} else if ((sr_class->class_type == SR_CLASS2) &&
			(sr_class->mod_use == SR_USE_ERROR_MOD)) {
		/**
		* SR settings if using the MINMAXAVG module inside
		* Smartreflex. SR CLASS 3 does not use this module where as
		* SR CLASS 2 can choose between ERROR module and MINMAXAVG
		* module.
		*/
		u32 avgwt;

		sr_modify_reg(sr, SRCONFIG, SRCONFIG_ACCUMDATA_MASK,
			sr->accum_data << SRCONFIG_ACCUMDATA_SHIFT);
		avgwt = (sr->senp_avgweight << AVGWEIGHT_SENPAVGWEIGHT_SHIFT) |
			(sr->senn_avgweight << AVGWEIGHT_SENNAVGWEIGHT_SHIFT);
		sr_write_reg(sr, AVGWEIGHT, avgwt);
		/* Enabling the interrupts if MINMAXAVG module is used.
		* TODO: check if all the interrupts are mandatory
		*/
		if (cpu_is_omap3430()) {
			sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_MCUACCUMINTEN | ERRCONFIG_MCUVALIDINTEN |
			ERRCONFIG_MCUBOUNDINTEN),
			(ERRCONFIG_MCUACCUMINTEN | ERRCONFIG_MCUACCUMINTST |
			ERRCONFIG_MCUVALIDINTEN | ERRCONFIG_MCUVALIDINTST |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST));
		} else if (cpu_is_omap3630()) {
			sr_modify_reg(sr, ERRCONFIG_36XX,
			(ERRCONFIG_MCUACCUMINTEN | ERRCONFIG_MCUVALIDINTEN |
			ERRCONFIG_MCUBOUNDINTEN),
			(ERRCONFIG_MCUACCUMINTEN | ERRCONFIG_MCUACCUMINTST |
			ERRCONFIG_MCUVALIDINTEN | ERRCONFIG_MCUVALIDINTST |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST));
		}
	}

	sr->is_sr_reset = 0;
}

static void sr_start_vddautocomap(int srid)
{
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return;
	}
	if (!sr_class || !(sr_class->enable)) {
		pr_warning("smartreflex class driver not registered\n");
		return;
	}

	if (sr->is_autocomp_active == 1)
		return;

	sr->is_autocomp_active = 1;
	if (!sr_class->enable(srid)) {
		sr->is_autocomp_active = 0;
		if (sr->is_sr_reset == 1)
			sr_clk_disable(sr);
	}
}

static int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return false;
	}

	if (!sr_class || !(sr_class->disable)) {
		pr_warning("smartreflex class driver not registered\n");
		return false;
	}

	if (sr->is_autocomp_active == 1) {
		sr_class->disable(srid);
		sr_clk_disable(sr);
		sr->is_autocomp_active = 0;
		return true;
	} else
		return false;

}

/* Public Functions */

/**
* cal_test_nvalue - Calculates the ntarget values
*/
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

/**
* sr_enable : Enables the smartreflex module.
* @srid - The id of the sr module to be enabled.
* @target_opp_no - The OPP at which the Voltage domain associated with
* the smartreflex module is operating at. This is required only to program
* the correct Ntarget value.
*
* This API is to be called from the smartreflex class driver to
* enable a smartreflex module. Returns true on success.Returns false if the
* target opp id passed is wrong or if ntarget value is wrong.
*/
int sr_enable(int srid, u32 target_opp_no)
{
	u32 nvalue_reciprocal;
	struct omap_sr *sr = _sr_lookup(srid);
	struct omap_smartreflex_data *pdata = sr->pdev->dev.platform_data;

	if (target_opp_no > pdata->no_opp) {
		pr_notice("Wrong target opp\n");
		return false;
	}

	/* For OMAP3430 errminlimit is dependent on opp. So choose
	* it appropriately
	*/
	if (cpu_is_omap343x())
		sr->err_minlimit = (target_opp_no > 2) ?
			OMAP3430_SR_ERRMINLIMIT_HIGHOPP :
			OMAP3430_SR_ERRMINLIMIT_LOWOPP;

	/*Enable the clocks and configure SR*/
	sr_clk_enable(sr);
	sr_configure(sr);

	nvalue_reciprocal = pdata->sr_nvalue[target_opp_no - 1];
	if (nvalue_reciprocal == 0) {
		sr->is_sr_reset = 1;
		pr_notice("OPP%d doesn't support SmartReflex\n",
								target_opp_no);
		return false;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);
	return true;
}

/**
* sr_disable : Disables the smartreflex module.
* @srid - The id of the sr module to be disabled.
*
* This API is to be called from the smartreflex class driver to
* disable a smartreflex module.
*/
void sr_disable(int srid)
{
	struct omap_sr *sr = _sr_lookup(srid);
	int timeout = 0;

	/* Check if SR Ris already disabled. If yes do nothing */
	if (!(sr_read_reg(sr, SRCONFIG) & SRCONFIG_SRENABLE))
		return;
	if (cpu_is_omap3430()) {
		/* Enable MCUDisableAcknowledge interrupt */
		sr_modify_reg(sr, ERRCONFIG,
			ERRCONFIG_MCUDISACKINTEN, ERRCONFIG_MCUDISACKINTEN);

		/* SRCONFIG - disable SR */
		sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0x0);

		/* Disable all other SR interrupts and clear the status */
		sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_MCUACCUMINTEN | ERRCONFIG_MCUVALIDINTEN |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_VPBOUNDINTEN),
			(ERRCONFIG_MCUACCUMINTST | ERRCONFIG_MCUVALIDINTST |
			ERRCONFIG_MCUBOUNDINTST | ERRCONFIG_VPBOUNDINTST));

		/* Wait for SR to be disabled.
		 * wait until ERRCONFIG.MCUDISACKINTST = 1.
		 * Typical latency is 1us.
		 */
		while ((timeout < SR_DISABLE_TIMEOUT) &&
			(!(sr_read_reg(sr, ERRCONFIG) &
			ERRCONFIG_MCUDISACKINTST))) {
			udelay(1);
			timeout++;
		}

		if (timeout == SR_DISABLE_TIMEOUT)
			pr_warning("SR%d disable timedout\n", srid);

		/* Disable MCUDisableAcknowledge interrupt &
		 * clear pending interrupt
		 */
		sr_modify_reg(sr, ERRCONFIG, ERRCONFIG_MCUDISACKINTEN,
			ERRCONFIG_MCUDISACKINTST);
	} else if (cpu_is_omap3630()) {
		/* Enable MCUDisableAcknowledge interrupt */
		sr_write_reg(sr, IRQENABLE_SET, IRQ_MCUDISACKINTENA);

		/* SRCONFIG - disable SR */
		sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0x0);

		/* Disable all other SR interrupts and clear the status */
		 sr_modify_reg(sr, ERRCONFIG_36XX,
				ERRCONFIG_VPBOUNDINTEN_36XX,
				ERRCONFIG_VPBOUNDINTST_36XX);
		sr_write_reg(sr, IRQENABLE_CLR, IRQ_MCUACCUMINTENA |
				IRQ_MCUVALIDINTENA | IRQ_MCUBOUNDINTENA);
		sr_write_reg(sr, IRQSTATUS, IRQSTATUS_MCUACCUMINTST |
				IRQSTATUS_MCUVALIDINTST |
				IRQSTATUS_MCUBOUNDINTST);
		/* Wait for SR to be disabled.
		 * wait until ERRCONFIG.MCUDISACKINTST = 1.
		 * Typical latency is 1us.
		 */
		while ((timeout < SR_DISABLE_TIMEOUT) &&
				(!(sr_read_reg(sr, IRQSTATUS) &
				IRQSTATUS_MCUDISACKINTST))) {
			udelay(1);
			timeout++;
		}

		if (timeout == SR_DISABLE_TIMEOUT)
			pr_warning("SR%d disable timedout\n", srid);

		/* Disable MCUDisableAcknowledge interrupt
		 * & clear pending interrupt
		 */
		sr_write_reg(sr, IRQENABLE_CLR, IRQ_MCUDISACKINTENA);
		sr_write_reg(sr, IRQSTATUS, IRQSTATUS_MCUDISACKINTST);
	}
}

/**
* omap_smartreflex_enable : API to enable SR clocks and to call into the
* registered smartreflex class enable API.
* @srid - The id of the sr module to be enabled.
*
* This API is to be called from the kernel in order to enable
* a particular smartreflex module. This API will do the initial
* configurations to turn on the smartreflex module and in turn call
* into the registered smartreflex class enable API.
*/
void omap_smartreflex_enable(int srid)
{
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return;
	}

	if (!sr_class || !(sr_class->enable)) {
		pr_warning("smartreflex class driver not registered\n");
		return;
	}
	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			if (!sr_class->enable(srid))
				sr_clk_disable(sr);
		}
	}
}

/**
 * omap_smartreflex_enable : API to disable SR clocks and to call into the
 * registered smartreflex class disable API.
 * @srid - The id of the sr module to be disabled.
 *
 * This API is to be called from the kernel in order to disable
 * a particular smartreflex module. This API will in turn call
 * into the registered smartreflex class disable API.
 */
void omap_smartreflex_disable(int srid)
{
	struct omap_sr *sr = _sr_lookup(srid);

	if (!sr) {
		pr_warning("omap_sr struct corresponding to SR%d not found\n",
								srid);
		return;
	}

	if (!sr_class || !(sr_class->disable)) {
		pr_warning("smartreflex class driver not registered\n");
		return;
	}

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 0) {
			sr_class->disable(srid);
			/* Disable SR clk */
			sr_clk_disable(sr);
		}
	}
}

/**
 * omap_sr_register_class : API to register a smartreflex class parameters.
 * @class_data - The structure containing various sr class specific data.
 *
 * This API is to be called by the smartreflex class driver to register
 * itself with the smartreflex driver during init.
 */
void omap_sr_register_class(struct omap_smartreflex_class_data *class_data)
{
	struct omap_sr *sr_info;

	if (!class_data) {
		pr_warning("Smartreflex class data passed is NULL\n");
		return;
	}

	if (sr_class) {
		pr_warning("Smartreflex class driver already registered\n");
		return;
	}

	if ((class_data->class_type != SR_CLASS2) &&
			(class_data->class_type != SR_CLASS3)) {
		pr_warning("SR Class type passed is invalid. So cannot \
				register the class structure\n");
		return;
	}

	if ((class_data->class_type == SR_CLASS2) &&
			!((class_data->mod_use == SR_USE_MINMAXAVG_MOD) ||
			(class_data->mod_use == SR_USE_ERROR_MOD))) {
		pr_warning("SR Class 2 specified but whether to use error \
				module or minmaxavg module not specified\n");
		return;
	}

	sr_class = class_data;

	/* Check if any SR module needs to be enabled as part of init.
	 * In case the probe for the SR module is not yet called the enable
	 * will not be done here but will be done in the probe whenever
	 * it gets called. Also register the interrupt handler in case
	 * requested by the class driver.
	 */
	list_for_each_entry(sr_info, &sr_list, node) {
		struct omap_smartreflex_data *pdata =
				sr_info->pdev->dev.platform_data;
		if (sr_class->class_type == SR_CLASS2 &&
				sr_class->notify_flags && sr_info->irq) {
			char name[SMARTREFLEX_NAME_LEN];
			int ret;

			sprintf(name, "sr%d", sr_info->srid);
			ret = request_irq(sr_info->irq, sr_omap_isr,
					IRQF_DISABLED, name, (void *)sr_info);
			if (ret < 0) {
				pr_warning("ERROR in registering interrupt \
					handler for SR%d Smartreflex will \
					not function as desired\n",
					sr_info->srid);
				return;
			}
		}
		if (pdata->init_enable)
			sr_start_vddautocomap(sr_info->srid);
	}
}

/* PM Debug Fs enteries to enable disable smartreflex. */
static int omap_sr_autocomp_show(void *data, u64 *val)
{
	struct omap_sr *sr_info = (struct omap_sr *) data;
	*val = sr_info->is_autocomp_active;
	return 0;
}

static int omap_sr_autocomp_store(void *data, u64 val)
{
	struct omap_sr *sr_info = (struct omap_sr *) data;
	if (val == 0)
		sr_stop_vddautocomap(sr_info->srid);
	else
		sr_start_vddautocomap(sr_info->srid);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_sr_fops, omap_sr_autocomp_show,
		omap_sr_autocomp_store, "%llu\n");

static int __devinit omap_smartreflex_probe(struct platform_device *pdev)
{
	struct omap_sr *sr_info = kzalloc(sizeof(struct omap_sr), GFP_KERNEL);
	struct omap_device *odev = to_omap_device(pdev);
	struct omap_smartreflex_data *pdata = pdev->dev.platform_data;
	int ret = 0;
	char name[SMARTREFLEX_NAME_LEN];

	sr_info->pdev = pdev;
	sr_info->srid = pdev->id + 1;
	sr_info->is_sr_reset = 1,
	sr_info->is_autocomp_active = 0;
	sr_info->clk_length = 0;
	sr_info->srbase_addr = odev->hwmods[0]->_rt_va;
	if (odev->hwmods[0]->mpu_irqs)
		sr_info->irq = odev->hwmods[0]->mpu_irqs[0].irq;
	sr_set_clk_length(sr_info);
	sr_set_regfields(sr_info);

	/* Create the debug fs enteries */
	sprintf(name, "sr%d_autocomp", sr_info->srid);
	(void) debugfs_create_file(name, S_IRUGO | S_IWUGO, pm_dbg_main_dir,
				(void *)sr_info, &pm_sr_fops);

	odev->hwmods[0]->dev_attr = sr_info;
	list_add(&sr_info->node, &sr_list);

	/* Enable the smartreflex module if init_enable flag is set and
	 * if the class driver is registered. During the registration
	 * of class driver once again we will check if enabling of the
	 * sr module is needed or not. Also register the interrupt handler
	 * if requested by the class driver. Again this will be attempted
	 * in the class driver register if it does not happen here.
	 */
	if (sr_class) {
		if (sr_class->class_type == SR_CLASS2 &&
				sr_class->notify_flags && sr_info->irq) {
			sprintf(name, "sr%d", sr_info->srid);
			ret = request_irq(sr_info->irq, sr_omap_isr,
					IRQF_DISABLED, name, (void *)sr_info);
			if (ret < 0) {
				pr_warning("ERROR in registering interrupt \
					handler for SR%d. Smartreflex will \
					not function as desired\n",
					sr_info->srid);
				return ret;
			}
		}
		if (pdata->init_enable)
			sr_start_vddautocomap(sr_info->srid);
	}

	pr_info("SmartReflex driver initialized\n");

	return ret;
}

static int __devexit omap_smartreflex_remove(struct platform_device *pdev)
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
	.probe          = omap_smartreflex_probe,
	.remove         = omap_smartreflex_remove,
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
