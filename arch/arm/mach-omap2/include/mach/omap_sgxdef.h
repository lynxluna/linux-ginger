#ifndef OMAP_SGXDEF_H
#define OMAP_SGXDEF_H

#include <plat/omap-pm.h>

#ifdef CONFIG_PM
struct sgx_platform_data {
	void(*set_min_bus_tput)(struct device *dev, u8 agent_id,
							unsigned long r);
};

#endif
#endif
