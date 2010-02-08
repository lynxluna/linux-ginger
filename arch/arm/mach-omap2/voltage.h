/*
 * OMAP3 Voltage Management Routines
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "pm.h"

/* SMART REFLEX I2C slave address */
#define VP1	1
#define VP2	2

/* Number of scalable voltage domains that has an independent Voltage processor
 * Can change for OMAP4
 */
#define	NO_SCALABLE_VDD		2

extern int get_vdd1_opp(void);
extern int get_vdd2_opp(void);

/* Generic VP definitions. Need to be redefined for OMAP4 */
#define VP_CONFIG_TIMEOUTEN	OMAP3430_TIMEOUTEN
#define VP_CONFIG_INITVDD	OMAP3430_INITVDD
#define VP_FORCEUPDATE		OMAP3430_FORCEUPDATE
#define VP_CONFIG_VPENABLE	OMAP3430_VPENABLE
#define VP_ERRORGAIN_MASK	OMAP3430_ERRORGAIN_MASK
#define VP_INITVOLTAGE_MASK	OMAP3430_INITVOLTAGE_MASK
#define VP_INITVOLTAGE_SHIFT	OMAP3430_INITVOLTAGE_SHIFT

/* Generic VC definitions. Need to be redefined for OMAP4 */
#define VC_SMPS_SA1_SHIFT	OMAP3430_SMPS_SA1_SHIFT
#define VC_SMPS_SA0_SHIFT	OMAP3430_SMPS_SA0_SHIFT
#define VC_VOLRA1_SHIFT		OMAP3430_VOLRA1_SHIFT
#define VC_VOLRA0_SHIFT		OMAP3430_VOLRA0_SHIFT
#define VC_CMD_ON_SHIFT		OMAP3430_VC_CMD_ON_SHIFT
#define VC_CMD_ONLP_SHIFT	OMAP3430_VC_CMD_ONLP_SHIFT
#define VC_CMD_RET_SHIFT	OMAP3430_VC_CMD_RET_SHIFT
#define VC_CMD_OFF_SHIFT	OMAP3430_VC_CMD_OFF_SHIFT
#define VC_SETUP_TIME2_SHIFT	OMAP3430_SETUP_TIME2_SHIFT
#define VC_SETUP_TIME1_SHIFT	OMAP3430_SETUP_TIME1_SHIFT
#define VC_DATA_SHIFT		OMAP3430_DATA_SHIFT
#define VC_REGADDR_SHIFT	OMAP3430_REGADDR_SHIFT
#define VC_SLAVEADDR_SHIFT	OMAP3430_SLAVEADDR_SHIFT
#define VC_CMD_ON_MASK		OMAP3430_VC_CMD_ON_MASK
#define VC_CMD1			OMAP3430_CMD1
#define VC_RAV1			OMAP3430_RAV1
#define VC_MCODE_SHIFT		OMAP3430_MCODE_SHIFT
#define VC_HSEN			OMAP3430_HSEN
#define VC_VALID		OMAP3430_VALID

/* Omap 3430 VP registerspecific values. Maybe these need to come from
 * board file or PMIC data structure
 */
#define OMAP3_VP_CONFIG_ERROROFFSET		0x00
#define OMAP3_VP_CONFIG_ERRORGAIN_LOWOPP	0x0C
#define OMAP3_VP_CONFIG_ERRORGAIN_HIGHOPP	0x18
#define	OMAP3_VP_VSTEPMIN_SMPSWAITTIMEMIN	0x3C
#define OMAP3_VP_VSTEPMIN_VSTEPMIN		0x1
#define OMAP3_VP_VSTEPMAX_SMPSWAITTIMEMAX	0x3C
#define OMAP3_VP_VSTEPMAX_VSTEPMAX		0x04
#define OMAP3_VP1_VLIMITTO_VDDMIN		0x12
#define OMAP3_VP1_VLIMITTO_VDDMAX		0x3C
#define OMAP3_VP2_VLIMITTO_VDDMAX		0x30
#define OMAP3_VP2_VLIMITTO_VDDMIN		0x12
#define OMAP3_VP_VLIMITTO_TIMEOUT_US		200

#define VOLTAGE_MOD	OMAP3430_GR_MOD
/* TODO OMAP4 VP register values if the same file is used for OMAP4*/

void omap_voltageprocessor_enable(int vp_id);
void omap_voltageprocessor_disable(int vp_id);
void omap_voltage_init_vc(struct prm_setup_vc *setup_vc);
void update_voltsetup_time(int core_next_state);
void omap_voltage_init(void);
int omap_voltage_scale(int vdd, u8 target_vsel, u8 current_vsel);
void omap_reset_voltage(int vdd);
