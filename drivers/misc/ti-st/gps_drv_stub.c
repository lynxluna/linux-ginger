/*
 *   GPS driver to test the functionalities of the Shared transport.
 *   Copyright (C) 2009 Texas Instruments
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>

#include <linux/uaccess.h>
#include <linux/tty.h>
#include <linux/sched.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>

#include "st.h"
#include "gps_drv_stub.h"

#ifdef DEBUG
#define ST_GPS_DRV_DBG(fmt, arg...) \
	printk(KERN_INFO "(st gps):"fmt"\n", ## arg)
#else
#define ST_GPS_DRV_DBG(fmt, arg...)
#endif

/* Initialization TaskLet for performing GPS Write */
DECLARE_TASKLET(gps_stub_tsklet, gps_chrdev_tsklet_stub, 0);

/*
 * The delayed work structure for this task
 */
static void intrpt_routine(struct work_struct *irrelevant);
static DECLARE_DELAYED_WORK(Task, intrpt_routine);

static void intrpt_routine(struct work_struct *irrelevant)
{
	static unsigned char rx_data[6] = { 0x09, 0x02, 0x02,
					    0x00, 0x01, 0x02 };
	int index = 0;
	ST_GPS_DRV_DBG(" Inside %s", __func__);

	if (rx_data[1] == 0x02) {
		for (index = 0; index < 6; index++)
			ST_GPS_DRV_DBG("****rx_data[%d] = %x",
				       index, rx_data[index]);
		st_int_recv(rx_data, 6);	/* Sending Read data */
		rx_data[1] = 0x03;
		rx_data[2] = 0x01;
		schedule_delayed_work(&Task, msecs_to_jiffies(2000));
	} else {
		for (index = 0; index < 6; index++)
			ST_GPS_DRV_DBG("****rx_data[%d] = %x", index,
				       rx_data[index]);
		st_int_recv(rx_data, 5);	/* Sending delayed ACK */
		rx_data[1] = 0x02;
		rx_data[2] = 0x02;
	}
}

void gps_chrdrv_stub_init()
{
	ST_GPS_DRV_DBG(" Inside %s", __func__);
	tasklet_enable(&gps_stub_tsklet);
}

int gps_chrdrv_stub_write(const unsigned char *data, int count)
{
	ST_GPS_DRV_DBG(" Inside %s", __func__);
	/* TODO skb->data[5]    is considered as the opcode
	 * 0x1 - considered as Write with instant ACK
	 * 0x2 - Considered as Read with instant Data reply
	 * 0x3 - Considered as Write with delayed ACK
	 * 0x4 - considered as Read with delayed Data reply
	 */
	if (0x01 == data[4]) {
		/* Update the ACK parameters */
		ST_GPS_DRV_DBG("********* invoked tasklet *****\n");
		tasklet_schedule(&gps_stub_tsklet);
	} else if (0x00 == data[4]) {
		/* Update the SKB with some data */
		ST_GPS_DRV_DBG("********* invoked work queue *****\n");
		schedule_delayed_work(&Task, msecs_to_jiffies(5000));
	}

	return count;
}

void gps_chrdev_tsklet_stub(unsigned long irrelevant)
{
	unsigned char data[5] = { 0x09, 0x03, 0x01, 0x00, 0x01 };
	ST_GPS_DRV_DBG("\n Inside %s \n", __func__);
	st_int_recv(data, 5);
}
