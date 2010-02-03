/* mach-omap2/board-zoom3-wifi.c
 *
 * Driver for zoom3 1273 WLAN.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Authors:
 *	Ohad Ben-Cohen		<ohad@bencohen.org>
 *	BVijay			<bvijay@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <linux/gpio.h>

#include <plat/wifi_tiwlan.h>

static int omap_zoom3_wifi_cd;	/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb) (int card_present, void *dev_id);
static void *wifi_status_cb_devid;

int omap_wifi_status_register(void (*callback) (int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;

	wifi_status_cb_devid = dev_id;
	return 0;
}

int omap_wifi_status(int irq)
{
	return omap_zoom3_wifi_cd;
}

int omap_zoom3_wifi_set_carddetect(int val)
{
	omap_zoom3_wifi_cd = val;

	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);

	return 0;
}

static int omap_zoom3_wifi_power_state;

int omap_zoom3_wifi_power(int on)
{
	gpio_set_value(OMAP_ZOOM3_WIFI_PMENA_GPIO, on);
	omap_zoom3_wifi_power_state = on;
	return 0;
}

static int omap_zoom3_wifi_reset_state;
int omap_zoom3_wifi_reset(int on)
{
	omap_zoom3_wifi_reset_state = on;
	return 0;
}

struct wifi_platform_data omap_zoom3_wifi_control = {
	.set_power = omap_zoom3_wifi_power,
	.set_reset = omap_zoom3_wifi_reset,
	.set_carddetect = omap_zoom3_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource omap_zoom3_wifi_resources[] = {
	[0] = {
	       .name = "device_wifi_irq",
	       .start = OMAP_GPIO_IRQ(OMAP_ZOOM3_WIFI_IRQ_GPIO),
	       .end = OMAP_GPIO_IRQ(OMAP_ZOOM3_WIFI_IRQ_GPIO),
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	       },
};

static struct platform_device omap_zoom3_wifi_device = {
	.name = "device_wifi",
	.id = 1,
	.num_resources = ARRAY_SIZE(omap_zoom3_wifi_resources),
	.resource = omap_zoom3_wifi_resources,
	.dev = {
		.platform_data = &omap_zoom3_wifi_control,
	},
};
#endif

static int __init omap_zoom3_wifi_init(void)
{
	int ret;

	ret = gpio_request(OMAP_ZOOM3_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
		       OMAP_ZOOM3_WIFI_IRQ_GPIO);
		goto out;
	}
	gpio_direction_input(OMAP_ZOOM3_WIFI_IRQ_GPIO);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&omap_zoom3_wifi_device);
#endif
out:
	return ret;
}

device_initcall(omap_zoom3_wifi_init);
