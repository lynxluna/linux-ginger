/*
 * Smart reflex Class 3 specific implementations
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "smartreflex.h"
#include "voltage.h"

int sr_class3_enable(int id)
{
	int target_opp_no = 0;
	if (id == SR1)
		target_opp_no = get_vdd1_opp();
	else if (id == SR2)
		target_opp_no = get_vdd2_opp();
	if (!target_opp_no) {
		pr_warning("Targetopp not known.Cannot enable SR%d\n", id);
		return false;
	}
	omap_voltageprocessor_enable(id);
	return sr_enable(id, target_opp_no);
}

int sr_class3_disable(int id)
{
	int target_opp_no = 0;
	if (id == SR1)
		target_opp_no = get_vdd1_opp();
	else if (id == SR2)
		target_opp_no = get_vdd2_opp();
	omap_voltageprocessor_disable(id);
	sr_disable(id);
	omap_reset_voltage(id);
	return true;
}

struct omap_smartreflex_class_data class3_data = {
	.enable = sr_class3_enable,
	.disable = sr_class3_disable,
};

static int __init sr_class3_init(void)
{
	omap_sr_register_class(&class3_data);
	return 0;
}

late_initcall(sr_class3_init);
