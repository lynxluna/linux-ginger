/* plat/wifi_tiwlan.h
 *
 * This file contains the WLAN CHIP specific data.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _LINUX_WIFI_TIWLAN_H_
#define _LINUX_WIFI_TIWLAN_H_

#define WMPA_NUMBER_OF_SECTIONS	3
#define WMPA_NUMBER_OF_BUFFERS	160
#define WMPA_SECTION_HEADER	24
#define WMPA_SECTION_SIZE_0	(WMPA_NUMBER_OF_BUFFERS * 64)
#define WMPA_SECTION_SIZE_1	(WMPA_NUMBER_OF_BUFFERS * 256)
#define WMPA_SECTION_SIZE_2	(WMPA_NUMBER_OF_BUFFERS * 2048)

#define OMAP_ZOOM3_WIFI_PMENA_GPIO	 101
#define OMAP_ZOOM3_WIFI_IRQ_GPIO	 162

struct wifi_platform_data {
	int (*set_power) (int val);
	int (*set_reset) (int val);
	int (*set_carddetect) (int val);
	void *(*mem_prealloc) (int section, unsigned long size);
};

#endif
