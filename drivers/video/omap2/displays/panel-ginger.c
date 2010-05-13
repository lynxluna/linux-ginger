/*
 * LCD panel driver for Ginger Console
 * Author: LynxLuna <lynxluna@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>

static char _lcd_type [10] = "Unknown\0";

static struct omap_video_timings ginger_lcd_timing = {
	.x_res          = 480,
	.y_res          = 272,
	.hsw            = 41,           /* hsync_len (4) - 1 */
	.hfp            = 2,            /* right_margin (4) - 1 */
	.hbp            = 2,            /* left_margin (40) - 1 */
	.vsw            = 10,           /* vsync_len (2) - 1 */
	.vfp            = 2,            /* lower_margin */
	.vbp            = 2,            /* upper_margin (8) - 1 */
	.pixel_clock	= 9600,
};

static int ginger_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	printk(KERN_INFO "Probing Ginger LCD... [%s]", _lcd_type );
	dssdev->panel.timings = ginger_lcd_timing;
	return 0;
}

static void ginger_panel_remove(struct omap_dss_device *dssdev)
{
}

static int ginger_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	return r;
}

static void ginger_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int ginger_panel_suspend(struct omap_dss_device *dssdev)
{
	ginger_panel_disable(dssdev);
	return 0;
}

static int ginger_panel_resume(struct omap_dss_device *dssdev)
{
	return ginger_panel_enable(dssdev);
}

static struct omap_dss_driver ginger_driver = {
	.probe		= ginger_panel_probe,
	.remove		= ginger_panel_remove,

	.enable		= ginger_panel_enable,
	.disable	= ginger_panel_disable,
	.suspend	= ginger_panel_suspend,
	.resume		= ginger_panel_resume,

	.driver         = {
		.name   = "ginger_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init ginger_panel_drv_init(void)
{
	printk ("Registering Ginger LCD Driver....");
	return omap_dss_register_driver(&ginger_driver);
}

static void __exit ginger_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&ginger_driver);
}

module_param_string(lcd,_lcd_type, 10, 0);

module_init(ginger_panel_drv_init);
module_exit(ginger_panel_drv_exit);
MODULE_LICENSE("GPL");



