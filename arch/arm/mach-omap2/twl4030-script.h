#ifndef __ARCH_ARM_MACH_OMAP3_TWL4030_SCRIPT_H
#define __ARCH_ARM_MACH_OMAP3_TWL4030_SCRIPT_H

#include <linux/i2c/twl.h>
#include "pm.h"

#ifdef CONFIG_TWL4030_POWER
extern void use_generic_twl4030_script(
		struct twl4030_power_data *t2scripts_data,
		struct prm_setup_vc *setup_vc);
void use_twl4030_script_glitchfix(
		struct twl4030_power_data *t2scripts_data);
#else
static inline void use_generic_twl4030_script(
		struct twl4030_power_data *t2scripts_data,
		struct prm_setup_vc *setup_vc) {}
void use_twl4030_script_glitchfix(
		struct twl4030_power_data *t2scripts_data) {}
#endif

#endif
