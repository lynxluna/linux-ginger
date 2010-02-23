/*
 * linux/arch/arm/mach-omap2/board-zoom-camera.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef CONFIG_VIDEO_OMAP3

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/regulator/consumer.h>

#include <asm/io.h>

#include <mach/gpio.h>
#ifdef CONFIG_OMAP_PM_SRF
#include <plat/omap-pm.h>
#endif

static int cam_inited;

static struct device *zoomcam_dev;

#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#define DEBUG_BASE		0x08000000

#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

#define CAMZOOM_USE_XCLKB  	1

#define ISP_IMX046_MCLK		216000000

/* Sensor specific GPIO signals */
#define IMX046_RESET_GPIO  	98
#define IMX046_STANDBY_GPIO	58
#define LV8093_PS_GPIO		7

static struct regulator *zoom_imx046_reg1;
static struct regulator *zoom_imx046_reg2;

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
#include <media/imx046.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#define IMX046_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define IMX046_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define IMX046_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define IMX046_CSI2_PHY_THS_TERM	2
#define IMX046_CSI2_PHY_THS_SETTLE	23
#define IMX046_CSI2_PHY_TCLK_TERM	0
#define IMX046_CSI2_PHY_TCLK_MISS	1
#define IMX046_CSI2_PHY_TCLK_SETTLE	14
#define IMX046_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(3280 * 2464 * 2)
#endif

#ifdef CONFIG_VIDEO_LV8093
#include <media/lv8093.h>
/* GPIO7 is connected to lens PS pin through inverter */
#define LV8093_PWR_OFF			1
#define LV8093_PWR_ON			(!LV8093_PWR_OFF)
#endif


#ifdef CONFIG_VIDEO_LV8093
static int lv8093_lens_power_set(enum v4l2_power power)
{
	static enum v4l2_power previous_pwr = V4L2_POWER_OFF;

	if (!cam_inited) {
		printk(KERN_ERR "MT9P012: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	switch (power) {
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "lv8093_lens_power_set(ON)\n");
		if (previous_pwr == V4L2_POWER_OFF)
			gpio_set_value(LV8093_PS_GPIO, LV8093_PWR_OFF);
		gpio_set_value(LV8093_PS_GPIO, LV8093_PWR_ON);
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "lv8093_lens_power_set(OFF)\n");
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "lv8093_lens_power_set(STANDBY)\n");
		gpio_set_value(LV8093_PS_GPIO, LV8093_PWR_OFF);
		break;
	}
	previous_pwr = power;
	return 0;
}

static int lv8093_lens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 2;
	hwc->dev_minor = 5;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;
	return 0;
}

struct lv8093_platform_data zoom_lv8093_platform_data = {
	.power_set      = lv8093_lens_power_set,
	.priv_data_set  = lv8093_lens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)

static struct omap34xxcam_sensor_config imx046_hwc = {
	.sensor_isp  = 0,
	.capture_mem = IMX046_BIGGEST_FRAME_BYTE_SIZE * 2,
	.ival_default	= { 1, 10 },
	.isp_if = ISP_CSIA,
};

static int imx046_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor		= imx046_hwc;
	hwc->dev_index		= 2;
	hwc->dev_minor		= 5;
	hwc->dev_type		= OMAP34XXCAM_SLAVE_SENSOR;

	return 0;
}

static struct isp_interface_config imx046_if_config = {
	.ccdc_par_ser 		= ISP_CSIA,
	.dataline_shift 	= 0x0,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 0,
	.cam_mclk		= ISP_IMX046_MCLK,
	.u.csi.crc 		= 0x0,
	.u.csi.mode 		= 0x0,
	.u.csi.edge 		= 0x0,
	.u.csi.signalling 	= 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge 		= 0x0,
	.u.csi.channel 		= 0x0,
	.u.csi.vpclk 		= 0x2,
	.u.csi.data_start 	= 0x0,
	.u.csi.data_size 	= 0x0,
	.u.csi.format 		= V4L2_PIX_FMT_SGRBG10,
};


static int imx046_sensor_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	int err = 0;

	if (!cam_inited) {
		printk(KERN_ERR "MT9P012: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	/*
	 * Plug regulator consumer to respective VAUX supply
	 * if not done before.
	 */
	if (!zoom_imx046_reg1 && !zoom_imx046_reg2) {
		zoom_imx046_reg1 = regulator_get(zoomcam_dev, "vaux2_1");
		if (IS_ERR(zoom_imx046_reg1)) {
			dev_err(zoomcam_dev, "vaux2_1 regulator missing\n");
			return PTR_ERR(zoom_imx046_reg1);
		}
		zoom_imx046_reg2 = regulator_get(zoomcam_dev, "vaux4_1");
		if (IS_ERR(zoom_imx046_reg2)) {
			dev_err(zoomcam_dev, "vaux4_1 regulator missing\n");
			regulator_put(zoom_imx046_reg1);
			return PTR_ERR(zoom_imx046_reg2);
		}
	}

	switch (power) {
	case V4L2_POWER_ON:
		/* Power Up Sequence */
		printk(KERN_DEBUG "imx046_sensor_power_set(ON)\n");

		/* Through-put requirement:
		 * 3280 x 2464 x 2Bpp x 7.5fps x 3 memory ops = 355163 KByte/s
		 */
#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 450000);
#endif

		isp_csi2_reset(&isp->isp_csi2);

		lanecfg.clk.pol = IMX046_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = IMX046_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = IMX046_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = IMX046_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = IMX046_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = IMX046_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&isp->isp_csi2, &lanecfg);
		isp_csi2_complexio_lanes_update(&isp->isp_csi2, true);

		isp_csi2_ctrl_config_ecc_enable(&isp->isp_csi2, true);

		phyconfig.ths_term = IMX046_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = IMX046_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = IMX046_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = IMX046_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = IMX046_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&isp->isp_csi2, &phyconfig);
		isp_csi2_phy_update(&isp->isp_csi2, true);

		isp_configure_interface(vdev->cam->isp, &imx046_if_config);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(IMX046_RESET_GPIO, 1);

		/* turn on analog power */
		regulator_enable(zoom_imx046_reg1);
		regulator_enable(zoom_imx046_reg2);
		udelay(100);

		/* have to put sensor to reset to guarantee detection */
		gpio_set_value(IMX046_RESET_GPIO, 0);
		udelay(1500);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(IMX046_RESET_GPIO, 1);
		udelay(300);
		break;
	case V4L2_POWER_OFF:
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "imx046_sensor_power_set(OFF/STANDBY)\n");
		/* Power Down Sequence */
		isp_csi2_complexio_power(&isp->isp_csi2, ISP_CSI2_POWER_OFF);

		if (regulator_is_enabled(zoom_imx046_reg1))
			regulator_disable(zoom_imx046_reg1);
		if (regulator_is_enabled(zoom_imx046_reg2))
			regulator_disable(zoom_imx046_reg2);

#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		if (previous_power == V4L2_POWER_ON)
			isp_disable_mclk(isp);
		break;
	}


	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return err;
}

static u32 imx046_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	return isp_set_xclk(vdev->cam->isp, xclkfreq, CAMZOOM_USE_XCLKB);
}

static int imx046_csi2_lane_count(struct v4l2_int_device *s, int count)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	return isp_csi2_complexio_lanes_count(&isp->isp_csi2, count);
}

static int imx046_csi2_cfg_vp_out_ctrl(struct v4l2_int_device *s,
				       u8 vp_out_ctrl)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	return isp_csi2_ctrl_config_vp_out_ctrl(&isp->isp_csi2, vp_out_ctrl);
}

static int imx046_csi2_ctrl_update(struct v4l2_int_device *s, bool force_update)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	return isp_csi2_ctrl_update(&isp->isp_csi2, force_update);
}

static int imx046_csi2_cfg_virtual_id(struct v4l2_int_device *s, u8 ctx, u8 id)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	return isp_csi2_ctx_config_virtual_id(&isp->isp_csi2, ctx, id);
}

static int imx046_csi2_ctx_update(struct v4l2_int_device *s, u8 ctx,
				  bool force_update)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	return isp_csi2_ctx_update(&isp->isp_csi2, ctx, force_update);
}

static int imx046_csi2_calc_phy_cfg0(struct v4l2_int_device *s,
				     u32 mipiclk, u32 lbound_hs_settle,
				     u32 ubound_hs_settle)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	return isp_csi2_calc_phy_cfg0(&isp->isp_csi2, mipiclk,
				      lbound_hs_settle, ubound_hs_settle);
}

struct imx046_platform_data zoom_imx046_platform_data = {
	.power_set            = imx046_sensor_power_set,
	.priv_data_set        = imx046_sensor_set_prv_data,
	.set_xclk             = imx046_sensor_set_xclk,
	.csi2_lane_count      = imx046_csi2_lane_count,
	.csi2_cfg_vp_out_ctrl = imx046_csi2_cfg_vp_out_ctrl,
	.csi2_ctrl_update     = imx046_csi2_ctrl_update,
	.csi2_cfg_virtual_id  = imx046_csi2_cfg_virtual_id,
	.csi2_ctx_update      = imx046_csi2_ctx_update,
	.csi2_calc_phy_cfg0   = imx046_csi2_calc_phy_cfg0,
};
#endif

static int zoom_cam_probe(struct platform_device *pdev)
{
	int ret = 0;

	/* Request and configure gpio pins */
	if (gpio_request(IMX046_STANDBY_GPIO, "imx046_standby_gpio") != 0) {
		dev_err(&pdev->dev, "Could not request GPIO %d",
			IMX046_STANDBY_GPIO);
		ret = -ENODEV;
		goto err;
	}

	if (gpio_request(IMX046_RESET_GPIO, "imx046_rst") != 0) {
		dev_err(&pdev->dev, "Could not request GPIO %d",
			IMX046_RESET_GPIO);
		ret = -ENODEV;
		goto err_freegpio1;
	}

	if (gpio_request(LV8093_PS_GPIO, "lv8093_ps") != 0) {
		dev_err(&pdev->dev, "Could not request GPIO %d",
			LV8093_PS_GPIO);
		ret = -ENODEV;
		goto err_freegpio2;
	}

	/* set to output mode */
	gpio_direction_output(IMX046_STANDBY_GPIO, true);
	gpio_direction_output(IMX046_RESET_GPIO, true);
	gpio_direction_output(LV8093_PS_GPIO, true);

	cam_inited = 1;
	zoomcam_dev = &pdev->dev;
	return 0;

err_freegpio2:
	gpio_free(IMX046_RESET_GPIO);
err_freegpio1:
	gpio_free(IMX046_STANDBY_GPIO);
err:
	cam_inited = 0;
	return ret;
}

static int zoom_cam_remove(struct platform_device *pdev)
{
	if (regulator_is_enabled(zoom_imx046_reg1))
		regulator_disable(zoom_imx046_reg1);
	regulator_put(zoom_imx046_reg1);
	if (regulator_is_enabled(zoom_imx046_reg2))
		regulator_disable(zoom_imx046_reg2);
	regulator_put(zoom_imx046_reg2);

	gpio_free(IMX046_STANDBY_GPIO);
	gpio_free(IMX046_RESET_GPIO);
	gpio_free(LV8093_PS_GPIO);
	return 0;
}

static int zoom_cam_suspend(struct device *dev)
{
	return 0;
}

static int zoom_cam_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops zoom_cam_pm_ops = {
	.suspend = zoom_cam_suspend,
	.resume  = zoom_cam_resume,
};

static struct platform_driver zoom_cam_driver = {
	.probe		= zoom_cam_probe,
	.remove		= zoom_cam_remove,
	.driver		= {
		.name	= "zoom_cam",
		.pm	= &zoom_cam_pm_ops,
	},
};

void __init zoom_cam_init(void)
{
	cam_inited = 0;
	platform_driver_register(&zoom_cam_driver);
}

#endif /* CONFIG_VIDEO_OMAP3 */
