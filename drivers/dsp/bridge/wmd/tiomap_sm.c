/*
 * tiomap_sm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>

#include <dspbridge/dbg.h>

#include "_tiomap.h"
#include "_tiomap_pwr.h"



DSP_STATUS CHNLSM_InterruptDSP2(struct WMD_DEV_CONTEXT *pDevContext,
				u16 wMbVal)
{
#ifdef CONFIG_BRIDGE_DVFS
	struct dspbridge_platform_data *pdata =
		omap_dspbridge_dev->dev.platform_data;
	u32 opplevel = 0;
#endif
	struct CFG_HOSTRES resources;
	DSP_STATUS status = DSP_SOK;
	u32 temp;

	if (!pDevContext->mbox)
		return DSP_SOK;

	status = CFG_GetHostResources((struct CFG_DEVNODE *)
			DRV_GetFirstDevExtension(), &resources);
	if (DSP_FAILED(status))
		return DSP_EFAIL;

	if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
#ifdef CONFIG_BRIDGE_DVFS
		if (pdata->dsp_get_opp)
			opplevel = (*pdata->dsp_get_opp)();
		if (opplevel == VDD1_OPP1) {
			if (pdata->dsp_set_min_opp)
				(*pdata->dsp_set_min_opp)(VDD1_OPP2);
		}
#endif
		/* Restart the peripheral clocks */
		DSP_PeripheralClocks_Enable(pDevContext, NULL);

		/*
		 * 2:0 AUTO_IVA2_DPLL - Enabling IVA2 DPLL auto control
		 *     in CM_AUTOIDLE_PLL_IVA2 register
		 */
		*(REG_UWORD32 *)(resources.dwCmBase + 0x34) = 0x1;

		/*
		 * 7:4 IVA2_DPLL_FREQSEL - IVA2 internal frq set to
		 *     0.75 MHz - 1.0 MHz
		 * 2:0 EN_IVA2_DPLL - Enable IVA2 DPLL in lock mode
		 */
		temp = *(REG_UWORD32 *)(resources.dwCmBase + 0x4);
		temp = (temp & 0xFFFFFF08) | 0x37;
		*(REG_UWORD32 *)(resources.dwCmBase + 0x4) = temp;

		/* Restore mailbox settings */
		omap_mbox_restore_ctx(pDevContext->mbox);

		/* Access MMU SYS CONFIG register to generate a short wakeup */
		temp = *(REG_UWORD32 *)(resources.dwDmmuBase + 0x10);

		pDevContext->dwBrdState = BRD_RUNNING;
	} else if (pDevContext->dwBrdState == BRD_RETENTION) {
		/* Restart the peripheral clocks */
		DSP_PeripheralClocks_Enable(pDevContext, NULL);
	}

	status = omap_mbox_msg_send(pDevContext->mbox, wMbVal);

	if (status) {
		pr_err("omap_mbox_msg_send Fail and status = %d\n", status);
		status = DSP_EFAIL;
	}

	DBG_Trace(DBG_LEVEL3, "writing %x to Mailbox\n", wMbVal);
	return DSP_SOK;
}

