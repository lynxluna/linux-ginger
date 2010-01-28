/*
 * linux/arch/arm/mach-omap2/devices.c
 *
 * OMAP2 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <plat/control.h>
#include <plat/tc.h>
#include <plat/board.h>
#include <plat/mux.h>
#include <mach/gpio.h>
#include <plat/mmc.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <mach/omap_sgxdef.h>
#include "mux.h"
#include "smartreflex.h"

#define MAX_HWMOD_NAME_LEN	16

#if defined(CONFIG_VIDEO_OMAP2) || defined(CONFIG_VIDEO_OMAP2_MODULE)

static struct resource cam_resources[] = {
	{
		.start		= OMAP24XX_CAMERA_BASE,
		.end		= OMAP24XX_CAMERA_BASE + 0xfff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_CAM_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap_cam_device = {
	.name		= "omap24xxcam",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cam_resources),
	.resource	= cam_resources,
};

static inline void omap_init_camera(void)
{
	platform_device_register(&omap_cam_device);
}

#elif defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)

static struct resource omap3isp_resources[] = {
	{
		.start		= OMAP3430_ISP_BASE,
		.end		= OMAP3430_ISP_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CBUFF_BASE,
		.end		= OMAP3430_ISP_CBUFF_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCP2_BASE,
		.end		= OMAP3430_ISP_CCP2_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CCDC_BASE,
		.end		= OMAP3430_ISP_CCDC_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_HIST_BASE,
		.end		= OMAP3430_ISP_HIST_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_H3A_BASE,
		.end		= OMAP3430_ISP_H3A_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_PREV_BASE,
		.end		= OMAP3430_ISP_PREV_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_RESZ_BASE,
		.end		= OMAP3430_ISP_RESZ_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_SBL_BASE,
		.end		= OMAP3430_ISP_SBL_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSI2A_BASE,
		.end		= OMAP3430_ISP_CSI2A_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3430_ISP_CSI2PHY_BASE,
		.end		= OMAP3430_ISP_CSI2PHY_END,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_34XX_CAM_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device omap3isp_device = {
	.name		= "omap3isp",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3isp_resources),
	.resource	= omap3isp_resources,
};

static inline void omap_init_camera(void)
{
	platform_device_register(&omap3isp_device);
}
#else
static inline void omap_init_camera(void)
{
}
#endif

#if defined(CONFIG_OMAP_MBOX_FWK) || defined(CONFIG_OMAP_MBOX_FWK_MODULE)

#define MBOX_REG_SIZE   0x120

#ifdef CONFIG_ARCH_OMAP2
static struct resource omap_mbox_resources[] = {
	{
		.start		= OMAP24XX_MAILBOX_BASE,
		.end		= OMAP24XX_MAILBOX_BASE + MBOX_REG_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MAIL_U0_MPU,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= INT_24XX_MAIL_U3_MPU,
		.flags		= IORESOURCE_IRQ,
	},
};
#endif

#ifdef CONFIG_ARCH_OMAP3
static struct resource omap_mbox_resources[] = {
	{
		.start		= OMAP34XX_MAILBOX_BASE,
		.end		= OMAP34XX_MAILBOX_BASE + MBOX_REG_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MAIL_U0_MPU,
		.flags		= IORESOURCE_IRQ,
	},
};
#endif

#ifdef CONFIG_ARCH_OMAP4

#define OMAP4_MBOX_REG_SIZE	0x130
static struct resource omap_mbox_resources[] = {
	{
		.start          = OMAP44XX_MAILBOX_BASE,
		.end            = OMAP44XX_MAILBOX_BASE +
					OMAP4_MBOX_REG_SIZE - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = INT_44XX_MAIL_U0_MPU,
		.flags          = IORESOURCE_IRQ,
	},
};
#endif

static struct platform_device mbox_device = {
	.name		= "omap2-mailbox",
	.id		= -1,
};

static inline void omap_init_mbox(void)
{
	if (cpu_is_omap2420() || cpu_is_omap3430() || cpu_is_omap3630() ||
							cpu_is_omap44xx()) {
		mbox_device.num_resources = ARRAY_SIZE(omap_mbox_resources);
		mbox_device.resource = omap_mbox_resources;
	} else {
		pr_err("%s: platform not supported\n", __func__);
		return;
	}
	platform_device_register(&mbox_device);
}
#else
static inline void omap_init_mbox(void) { }
#endif /* CONFIG_OMAP_MBOX_FWK */

#if defined(CONFIG_OMAP_STI)

#if defined(CONFIG_ARCH_OMAP2)

#define OMAP2_STI_BASE		0x48068000
#define OMAP2_STI_CHANNEL_BASE	0x54000000
#define OMAP2_STI_IRQ		4

static struct resource sti_resources[] = {
	{
		.start		= OMAP2_STI_BASE,
		.end		= OMAP2_STI_BASE + 0x7ff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_STI_CHANNEL_BASE,
		.end		= OMAP2_STI_CHANNEL_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_STI_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};
#elif defined(CONFIG_ARCH_OMAP3)

#define OMAP3_SDTI_BASE		0x54500000
#define OMAP3_SDTI_CHANNEL_BASE	0x54600000

static struct resource sti_resources[] = {
	{
		.start		= OMAP3_SDTI_BASE,
		.end		= OMAP3_SDTI_BASE + 0xFFF,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP3_SDTI_CHANNEL_BASE,
		.end		= OMAP3_SDTI_CHANNEL_BASE + SZ_1M - 1,
		.flags		= IORESOURCE_MEM,
	}
};

#endif

static struct platform_device sti_device = {
	.name		= "sti",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sti_resources),
	.resource	= sti_resources,
};

static inline void omap_init_sti(void)
{
	platform_device_register(&sti_device);
}
#else
static inline void omap_init_sti(void) {}
#endif

#if defined(CONFIG_SPI_OMAP24XX) || defined(CONFIG_SPI_OMAP24XX_MODULE)

#include <plat/mcspi.h>

#define OMAP2_MCSPI1_BASE		0x48098000
#define OMAP2_MCSPI2_BASE		0x4809a000
#define OMAP2_MCSPI3_BASE		0x480b8000
#define OMAP2_MCSPI4_BASE		0x480ba000

#define OMAP4_MCSPI1_BASE		0x48098100
#define OMAP4_MCSPI2_BASE		0x4809a100
#define OMAP4_MCSPI3_BASE		0x480b8100
#define OMAP4_MCSPI4_BASE		0x480ba100

static struct omap2_mcspi_platform_config omap2_mcspi1_config = {
	.num_cs		= 4,
	.force_cs_mode	= 1,
};

static struct resource omap2_mcspi1_resources[] = {
	{
		.start		= OMAP2_MCSPI1_BASE,
		.end		= OMAP2_MCSPI1_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi1 = {
	.name		= "omap2_mcspi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(omap2_mcspi1_resources),
	.resource	= omap2_mcspi1_resources,
	.dev		= {
		.platform_data = &omap2_mcspi1_config,
	},
};

static struct omap2_mcspi_platform_config omap2_mcspi2_config = {
	.num_cs		= 2,
	.mode		= OMAP2_MCSPI_MASTER,
	.dma_mode	= 0,
	.force_cs_mode	= 0,
	.fifo_depth	= 0,
};

static struct resource omap2_mcspi2_resources[] = {
	{
		.start		= OMAP2_MCSPI2_BASE,
		.end		= OMAP2_MCSPI2_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi2 = {
	.name		= "omap2_mcspi",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(omap2_mcspi2_resources),
	.resource	= omap2_mcspi2_resources,
	.dev		= {
		.platform_data = &omap2_mcspi2_config,
	},
};

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3) || \
	defined(CONFIG_ARCH_OMAP4)
static struct omap2_mcspi_platform_config omap2_mcspi3_config = {
	.num_cs		= 2,
};

static struct resource omap2_mcspi3_resources[] = {
	{
	.start		= OMAP2_MCSPI3_BASE,
	.end		= OMAP2_MCSPI3_BASE + 0xff,
	.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi3 = {
	.name		= "omap2_mcspi",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(omap2_mcspi3_resources),
	.resource	= omap2_mcspi3_resources,
	.dev		= {
		.platform_data = &omap2_mcspi3_config,
	},
};
#endif

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
static struct omap2_mcspi_platform_config omap2_mcspi4_config = {
	.num_cs		= 1,
};

static struct resource omap2_mcspi4_resources[] = {
	{
		.start		= OMAP2_MCSPI4_BASE,
		.end		= OMAP2_MCSPI4_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_mcspi4 = {
	.name		= "omap2_mcspi",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(omap2_mcspi4_resources),
	.resource	= omap2_mcspi4_resources,
	.dev		= {
		.platform_data = &omap2_mcspi4_config,
	},
};
#endif

#ifdef CONFIG_ARCH_OMAP4
static inline void omap4_mcspi_fixup(void)
{
	omap2_mcspi1_resources[0].start	= OMAP4_MCSPI1_BASE;
	omap2_mcspi1_resources[0].end	= OMAP4_MCSPI1_BASE + 0xff;
	omap2_mcspi2_resources[0].start	= OMAP4_MCSPI2_BASE;
	omap2_mcspi2_resources[0].end	= OMAP4_MCSPI2_BASE + 0xff;
	omap2_mcspi3_resources[0].start	= OMAP4_MCSPI3_BASE;
	omap2_mcspi3_resources[0].end	= OMAP4_MCSPI3_BASE + 0xff;
	omap2_mcspi4_resources[0].start	= OMAP4_MCSPI4_BASE;
	omap2_mcspi4_resources[0].end	= OMAP4_MCSPI4_BASE + 0xff;
}
#else
static inline void omap4_mcspi_fixup(void)
{
}
#endif

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3) || \
	defined(CONFIG_ARCH_OMAP4)
static inline void omap2_mcspi3_init(void)
{
	platform_device_register(&omap2_mcspi3);
}
#else
static inline void omap2_mcspi3_init(void)
{
}
#endif

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
static inline void omap2_mcspi4_init(void)
{
	platform_device_register(&omap2_mcspi4);
}
#else
static inline void omap2_mcspi4_init(void)
{
}
#endif

static void omap_init_mcspi(void)
{
	if (cpu_is_omap44xx())
		omap4_mcspi_fixup();

	platform_device_register(&omap2_mcspi1);
	platform_device_register(&omap2_mcspi2);

	if (cpu_is_omap2430() || cpu_is_omap343x() || cpu_is_omap44xx())
		omap2_mcspi3_init();

	if (cpu_is_omap343x() || cpu_is_omap44xx())
		omap2_mcspi4_init();
}

#else
static inline void omap_init_mcspi(void) {}
#endif

#ifdef CONFIG_OMAP_SHA1_MD5
static struct resource sha1_md5_resources[] = {
	{
		.start	= OMAP24XX_SEC_SHA1MD5_BASE,
		.end	= OMAP24XX_SEC_SHA1MD5_BASE + 0x64,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_24XX_SHA1MD5,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct platform_device sha1_md5_device = {
	.name		= "OMAP SHA1/MD5",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sha1_md5_resources),
	.resource	= sha1_md5_resources,
};

static void omap_init_sha1_md5(void)
{
	platform_device_register(&sha1_md5_device);
}
#else
static inline void omap_init_sha1_md5(void) { }
#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)

#define MMCHS_SYSCONFIG			0x0010
#define MMCHS_SYSCONFIG_SWRESET		(1 << 1)
#define MMCHS_SYSSTATUS			0x0014
#define MMCHS_SYSSTATUS_RESETDONE	(1 << 0)

static struct platform_device dummy_pdev = {
	.dev = {
		.bus = &platform_bus_type,
	},
};

/**
 * omap_hsmmc_reset() - Full reset of each HS-MMC controller
 *
 * Ensure that each MMC controller is fully reset.  Controllers
 * left in an unknown state (by bootloader) may prevent retention
 * or OFF-mode.  This is especially important in cases where the
 * MMC driver is not enabled, _or_ built as a module.
 *
 * In order for reset to work, interface, functional and debounce
 * clocks must be enabled.  The debounce clock comes from func_32k_clk
 * and is not under SW control, so we only enable i- and f-clocks.
 **/
static void __init omap_hsmmc_reset(void)
{
	u32 i, nr_controllers = cpu_is_omap44xx() ? OMAP44XX_NR_MMC :
		(cpu_is_omap34xx() ? OMAP34XX_NR_MMC : OMAP24XX_NR_MMC);

	for (i = 0; i < nr_controllers; i++) {
		u32 v, base = 0;
		struct clk *iclk, *fclk;
		struct device *dev = &dummy_pdev.dev;

		switch (i) {
		case 0:
			base = OMAP2_MMC1_BASE;
			break;
		case 1:
			base = OMAP2_MMC2_BASE;
			break;
		case 2:
			base = OMAP3_MMC3_BASE;
			break;
		case 3:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC4_BASE;
			break;
		case 4:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC5_BASE;
			break;
		}

		if (cpu_is_omap44xx())
			base += OMAP4_MMC_REG_OFFSET;

		dummy_pdev.id = i;
		dev_set_name(&dummy_pdev.dev, "mmci-omap-hs.%d", i);
		iclk = clk_get(dev, "ick");
		if (iclk && clk_enable(iclk))
			iclk = NULL;

		fclk = clk_get(dev, "fck");
		if (fclk && clk_enable(fclk))
			fclk = NULL;

		if (!iclk || !fclk) {
			printk(KERN_WARNING
			       "%s: Unable to enable clocks for MMC%d, "
			       "cannot reset.\n",  __func__, i);
			break;
		}

		omap_writel(MMCHS_SYSCONFIG_SWRESET, base + MMCHS_SYSCONFIG);
		v = omap_readl(base + MMCHS_SYSSTATUS);
		while (!(omap_readl(base + MMCHS_SYSSTATUS) &
			 MMCHS_SYSSTATUS_RESETDONE))
			cpu_relax();

		if (fclk) {
			clk_disable(fclk);
			clk_put(fclk);
		}
		if (iclk) {
			clk_disable(iclk);
			clk_put(iclk);
		}
	}
}
#else
static inline void omap_hsmmc_reset(void) {}
#endif

#if defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

static inline void omap2_mmc_mux(struct omap_mmc_platform_data *mmc_controller,
			int controller_nr)
{
	if (cpu_is_omap2420() && controller_nr == 0) {
		omap_cfg_reg(H18_24XX_MMC_CMD);
		omap_cfg_reg(H15_24XX_MMC_CLKI);
		omap_cfg_reg(G19_24XX_MMC_CLKO);
		omap_cfg_reg(F20_24XX_MMC_DAT0);
		omap_cfg_reg(F19_24XX_MMC_DAT_DIR0);
		omap_cfg_reg(G18_24XX_MMC_CMD_DIR);
		if (mmc_controller->slots[0].wires == 4) {
			omap_cfg_reg(H14_24XX_MMC_DAT1);
			omap_cfg_reg(E19_24XX_MMC_DAT2);
			omap_cfg_reg(D19_24XX_MMC_DAT3);
			omap_cfg_reg(E20_24XX_MMC_DAT_DIR1);
			omap_cfg_reg(F18_24XX_MMC_DAT_DIR2);
			omap_cfg_reg(E18_24XX_MMC_DAT_DIR3);
		}

		/*
		 * Use internal loop-back in MMC/SDIO Module Input Clock
		 * selection
		 */
		if (mmc_controller->slots[0].internal_clock) {
			u32 v = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
			v |= (1 << 24);
			omap_ctrl_writel(v, OMAP2_CONTROL_DEVCONF0);
		}
	}

	if (cpu_is_omap34xx()) {
		if (controller_nr == 0) {
			omap_mux_init_signal("sdmmc1_clk",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc1_cmd",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc1_dat0",
				OMAP_PIN_INPUT_PULLUP);
			if (mmc_controller->slots[0].wires == 4 ||
				mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc1_dat1",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat2",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat3",
					OMAP_PIN_INPUT_PULLUP);
			}
			if (mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc1_dat4",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat5",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat6",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc1_dat7",
					OMAP_PIN_INPUT_PULLUP);
			}
		}
		if (controller_nr == 1) {
			/* MMC2 */
			omap_mux_init_signal("sdmmc2_clk",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc2_cmd",
				OMAP_PIN_INPUT_PULLUP);
			omap_mux_init_signal("sdmmc2_dat0",
				OMAP_PIN_INPUT_PULLUP);

			/*
			 * For 8 wire configurations, Lines DAT4, 5, 6 and 7 need to be muxed
			 * in the board-*.c files
			 */
			if (mmc_controller->slots[0].wires == 4 ||
				mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc2_dat1",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat2",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat3",
					OMAP_PIN_INPUT_PULLUP);
			}
			if (mmc_controller->slots[0].wires == 8) {
				omap_mux_init_signal("sdmmc2_dat4.sdmmc2_dat4",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat5.sdmmc2_dat5",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat6.sdmmc2_dat6",
					OMAP_PIN_INPUT_PULLUP);
				omap_mux_init_signal("sdmmc2_dat7.sdmmc2_dat7",
					OMAP_PIN_INPUT_PULLUP);
			}
		}

		/*
		 * For MMC3 the pins need to be muxed in the board-*.c files
		 */
	}
}

void __init omap2_init_mmc(struct omap_mmc_platform_data **mmc_data,
			int nr_controllers)
{
	int i;
	char *name;

	for (i = 0; i < nr_controllers; i++) {
		unsigned long base, size;
		unsigned int irq = 0;

		if (!mmc_data[i])
			continue;

		omap2_mmc_mux(mmc_data[i], i);

		switch (i) {
		case 0:
			base = OMAP2_MMC1_BASE;
			irq = INT_24XX_MMC_IRQ;
			break;
		case 1:
			base = OMAP2_MMC2_BASE;
			irq = INT_24XX_MMC2_IRQ;
			break;
		case 2:
			if (!cpu_is_omap44xx() && !cpu_is_omap34xx())
				return;
			base = OMAP3_MMC3_BASE;
			irq = INT_34XX_MMC3_IRQ;
			break;
		case 3:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC4_BASE + OMAP4_MMC_REG_OFFSET;
			irq = INT_44XX_MMC4_IRQ;
			break;
		case 4:
			if (!cpu_is_omap44xx())
				return;
			base = OMAP4_MMC5_BASE + OMAP4_MMC_REG_OFFSET;
			irq = INT_44XX_MMC5_IRQ;
			break;
		default:
			continue;
		}

		if (cpu_is_omap2420()) {
			size = OMAP2420_MMC_SIZE;
			name = "mmci-omap";
		} else if (cpu_is_omap44xx()) {
			if (i < 3) {
				base += OMAP4_MMC_REG_OFFSET;
				irq += IRQ_GIC_START;
			}
			size = OMAP4_HSMMC_SIZE;
			name = "mmci-omap-hs";
		} else {
			size = OMAP3_HSMMC_SIZE;
			name = "mmci-omap-hs";
		}
		omap_mmc_add(name, i, base, size, irq, mmc_data[i]);
	};
}

#endif

#ifdef CONFIG_OMAP_SMARTREFLEX
struct omap_device_pm_latency omap_sr_latency[] = {
	{
		.deactivate_lat	 = 1,
		.deactivate_func = omap_device_idle_hwmods,
		.activate_lat	 = 1,
		.activate_func	 = omap_device_enable_hwmods,
		.flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST
	},
};

/* Read EFUSE values from control registers */
static void __init sr_read_efuse(struct omap_smartreflex_data *sr_data,
						int sr_id)
{
	if (cpu_is_omap343x()) {
		if (sr_id == SR1) {
			/* TODO: When opp framework come into picture use
			 * appropriate API's to find out number of opp's.
			 */
			sr_data->no_opp = 5;
			sr_data->sr_nvalue =
				kzalloc(sizeof(sr_data->sr_nvalue) *
					sr_data->no_opp , GFP_KERNEL);
			sr_data->senn_mod =
				(omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR1_SENNENABLE_MASK) >>
					OMAP343X_SR1_SENNENABLE_SHIFT;
			sr_data->senp_mod =
				(omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR1_SENPENABLE_MASK) >>
					OMAP343X_SR1_SENPENABLE_SHIFT;
			sr_data->sr_nvalue[4] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP5_VDD1);
			sr_data->sr_nvalue[3] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP4_VDD1);
			sr_data->sr_nvalue[2] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP3_VDD1);
			sr_data->sr_nvalue[1] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP2_VDD1);
			sr_data->sr_nvalue[0] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP1_VDD1);
		} else if (sr_id == SR2) {
			/* TODO: When opp framework come into picture use
			 * appropriate API's to find out number of opp's.
			 */
			sr_data->no_opp = 3;
			sr_data->sr_nvalue =
				kzalloc(sizeof(sr_data->sr_nvalue) *
				sr_data->no_opp , GFP_KERNEL);

			sr_data->senn_mod =
				(omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR2_SENNENABLE_MASK) >>
					OMAP343X_SR2_SENNENABLE_SHIFT;
			sr_data->senp_mod =
				(omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
					OMAP343X_SR2_SENPENABLE_MASK) >>
					OMAP343X_SR2_SENPENABLE_SHIFT;
			sr_data->sr_nvalue[2] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP3_VDD2);
			sr_data->sr_nvalue[1] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP2_VDD2);
			sr_data->sr_nvalue[0] = omap_ctrl_readl(
					OMAP343X_CONTROL_FUSE_OPP1_VDD2);
		}
	}
	/*TODO: Add 3630/OMAP4 support */

}

/* Hard coded nvalues for testing purposes, may cause device to hang! */
static void __init omap34xx_sr_testing_nvalues(
			struct omap_smartreflex_data *sr_data, int srid)
{
	if (srid == SR1) {
		/* TODO: When opp framework come into picture use appropriate
		 * API's to find out number of opp's.
		 */
		sr_data->no_opp = 5;
		sr_data->sr_nvalue = kzalloc(sizeof(sr_data->sr_nvalue) *
				sr_data->no_opp , GFP_KERNEL);

		sr_data->senp_mod = 0x03;	/* SenN-M5 enabled */
		sr_data->senn_mod = 0x03;
		/* calculate nvalues for each opp */
		sr_data->sr_nvalue[4] = cal_test_nvalue(0xacd + 0x330,
					0x848 + 0x330);
		sr_data->sr_nvalue[3] = cal_test_nvalue(0x964 + 0x2a0,
					0x727 + 0x2a0);
		sr_data->sr_nvalue[2] = cal_test_nvalue(0x85b + 0x200,
					0x655 + 0x200);
		sr_data->sr_nvalue[1] = cal_test_nvalue(0x506 + 0x1a0,
					0x3be + 0x1a0);
		sr_data->sr_nvalue[0] = cal_test_nvalue(0x373 + 0x100,
					0x28c + 0x100);
	} else if (srid == SR2) {
		/* TODO: When opp framework come into picture use appropriate
		 * API's to find out number of opp's.
		 */
		sr_data->no_opp = 3;
		sr_data->sr_nvalue = kzalloc(sizeof(sr_data->sr_nvalue) *
					sr_data->no_opp , GFP_KERNEL);

		sr_data->senp_mod = 0x03;	/* SenN-M5 enabled */
		sr_data->senn_mod = 0x03;
		sr_data->sr_nvalue[2] = cal_test_nvalue(0x76f + 0x200,
					0x579 + 0x200);
		sr_data->sr_nvalue[1] = cal_test_nvalue(0x4f5 + 0x1c0,
					0x390 + 0x1c0);
		sr_data->sr_nvalue[0] = cal_test_nvalue(0x359, 0x25d);
	}

}
static void __init omap36xx_sr_testing_nvalues(
			struct omap_smartreflex_data *sr_data, int srid)
{
	if (srid == SR1) {
		sr_data->no_opp = 4;
		sr_data->sr_nvalue = kzalloc(sizeof(sr_data->sr_nvalue) *
						sr_data->no_opp , GFP_KERNEL);
		sr_data->senp_mod = 0x1;
		sr_data->senn_mod = 0x1;

		/* OMAP3630 nvalues for each VDD1 opp */
		sr_data->sr_nvalue[3] = 0xaab197;
		sr_data->sr_nvalue[2] = 0xaac5a8;
		sr_data->sr_nvalue[1] = 0x999b83;
		sr_data->sr_nvalue[0] = 0x898beb;


	} else if (srid == SR2) {
		sr_data->no_opp = 2;
		sr_data->sr_nvalue = kzalloc(sizeof(sr_data->sr_nvalue) *
						sr_data->no_opp , GFP_KERNEL);
		sr_data->senp_mod = 0x1;
		sr_data->senn_mod = 0x1;

		/* OMAP3630 nvalues for each VDD2 opp */
		sr_data->sr_nvalue[1] = 0x9a8cee;
		sr_data->sr_nvalue[0] = 0x898beb;
	}

}

static void __init sr_set_nvalues(struct omap_smartreflex_data *sr_data,
						int srid)
{
	if (SR_TESTING_NVALUES) {
		if (cpu_is_omap3430())
			omap34xx_sr_testing_nvalues(sr_data, srid);
		else if (cpu_is_omap3630())
			omap36xx_sr_testing_nvalues(sr_data, srid);
	} else
		sr_read_efuse(sr_data, srid);
}

static void __init omap_init_smartreflex(void)
{
	int i = 0;
	char *name = "smartreflex";

	do {
		struct omap_smartreflex_data *sr_data;
		struct omap_device *od;
		struct omap_hwmod *oh;
		char oh_name[MAX_HWMOD_NAME_LEN];

		snprintf(oh_name, MAX_HWMOD_NAME_LEN, "sr%d_hwmod", i + 1);
		oh = omap_hwmod_lookup(oh_name);
		if (!oh)
			break;

		sr_data = kzalloc(sizeof(struct omap_smartreflex_data),
								GFP_KERNEL);
		if (!sr_data)
			return;

		/* Enable the SR module by default if it is a OMAP3430
		 * ES3.1 chip
		 */
		if (omap_rev() == OMAP3430_REV_ES3_1)
			sr_data->init_enable = true;
		else
			sr_data->init_enable = false;
		sr_data->device_enable = omap_device_enable;
		sr_data->device_shutdown = omap_device_shutdown;
		sr_data->device_idle = omap_device_idle;
		sr_set_nvalues(sr_data, i + 1);

		od = omap_device_build(name, i, oh, sr_data, sizeof(*sr_data),
				       omap_sr_latency,
				       ARRAY_SIZE(omap_sr_latency));
		WARN(IS_ERR(od), "Could not build omap_device for %s: %s.\n",
		     name, oh->name);
		i++;
	} while (1);
}
#else
static inline void omap_init_smartreflex(void) { }
#endif

/*-------------------------------------------------------------------------*/

#if defined(CONFIG_HDQ_MASTER_OMAP) || defined(CONFIG_HDQ_MASTER_OMAP_MODULE)
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
#define OMAP_HDQ_BASE	0x480B2000
#endif
static struct resource omap_hdq_resources[] = {
	{
		.start		= OMAP_HDQ_BASE,
		.end		= OMAP_HDQ_BASE + 0x1C,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_HDQ_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device omap_hdq_dev = {
	.name = "omap_hdq",
	.id = 0,
	.dev = {
		.platform_data = NULL,
	},
	.num_resources	= ARRAY_SIZE(omap_hdq_resources),
	.resource	= omap_hdq_resources,
};
static inline void omap_hdq_init(void)
{
	(void) platform_device_register(&omap_hdq_dev);
}
#else
static inline void omap_hdq_init(void) {}
#endif

struct sgx_platform_data omap_sgx_data = {
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
};

static struct platform_device powervr_device = {
	.name            = "pvrsrvkm",
	.id              = -1,
#ifdef CONFIG_PM
	.dev            = {
	.platform_data = &omap_sgx_data,
       }
#else
	.dev            = {
	.platform_data = NULL,}
#endif
};
static void omap_init_sgx(void)
{
       (void) platform_device_register(&powervr_device);
}


/*-------------------------------------------------------------------------*/

static int __init omap2_init_devices(void)
{
	/* please keep these calls, and their implementations above,
	 * in alphabetical order so they're easier to sort through.
	 */
	omap_hsmmc_reset();
	omap_init_camera();
	omap_init_mbox();
	omap_init_mcspi();
	omap_hdq_init();
	omap_init_sti();
	omap_init_sha1_md5();
	omap_init_smartreflex();
	omap_init_sgx();
	return 0;
}
arch_initcall(omap2_init_devices);
