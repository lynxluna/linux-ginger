#ifndef __ARCH_ARM_MACH_OMAP3_TWL4030_SCRIPT_H
#define __ARCH_ARM_MACH_OMAP3_TWL4030_SCRIPT_H

#include <linux/i2c/twl.h>
#include "pm.h"

#ifdef CONFIG_TWL4030_POWER
extern struct prm_setup_vc twl4030_voltsetup_time;
extern struct twl4030_power_data twl4030_generic_script;
#endif

#endif
