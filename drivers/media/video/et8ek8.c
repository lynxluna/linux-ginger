/*
 * drivers/media/video/et8ek8.c
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
 *          Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *
 * Based on code from Toni Leinonen <toni.leinonen@offcode.fi>.
 *
 * This driver is based on the Micron MT9T012 camera imager driver
 * (C) Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/kernel.h>

#include <media/smiaregs.h>
#include <media/v4l2-int-device.h>

#include "et8ek8.h"

#define ET8EK8_XCLK_HZ		9600000

#define ET8EK8_MAG_MIN		0x18
#define ET8EK8_MAG_MAX		0xFF
#define ET8EK8_MES_MIN		1
#define ET8EK8_MES_MAX		2100

#define DEFAULT_GAIN		0
#define DEFAULT_EXPOSURE	ET8EK8_MES_MIN

enum et8ek8_versions {
	ET8EK8_REV_1 = 0x0001,
	ET8EK8_REV_2,
};

/*
 * This table describes what should be written to the sensor register
 * for each gain value. The gain(index in the table) is in terms of
 * 0.1EV, i.e. 10 indexes in the table give 2 time more gain [0] in
 * the *analog gain, [1] in the digital gain
 *
 * Analog gain [dB] = 20*log10(regvalue/32); 0x20..0x100
 */
static struct et8ek8_gain {
	u16 analog;
	u16 digital;
} const et8ek8_gain_table[] = {
	{ 32,    0},  /* x1 */
	{ 34,    0},
	{ 37,    0},
	{ 39,    0},
	{ 42,    0},
	{ 45,    0},
	{ 49,    0},
	{ 52,    0},
	{ 56,    0},
	{ 60,    0},
	{ 64,    0},  /* x2 */
	{ 69,    0},
	{ 74,    0},
	{ 79,    0},
	{ 84,    0},
	{ 91,    0},
	{ 97,    0},
	{104,    0},
	{111,    0},
	{119,    0},
	{128,    0},  /* x4 */
	{137,    0},
	{147,    0},
	{158,    0},
	{169,    0},
	{181,    0},
	{194,    0},
	{208,    0},
	{223,    0},
	{239,    0},
	{256,    0},  /* x8 */
	{256,   73},
	{256,  152},
	{256,  236},
	{256,  327},
	{256,  424},
	{256,  528},
	{256,  639},
	{256,  758},
	{256,  886},
	{256, 1023},  /* x16 */
};

/* Register definitions */
#define REG_REVISION_NUMBER_L	0x1200
#define REG_REVISION_NUMBER_H	0x1201

#define PRIV_MEM_START_REG	0x0008
#define PRIV_MEM_WIN_SIZE	8

#define ET8EK8_I2C_DELAY	3	/* msec delay b/w accesses */

#define USE_CRC			1

static int et8ek8_apply_gain(struct et8ek8_sensor *sensor)
{
	int rval;
	struct et8ek8_gain new =
		et8ek8_gain_table[sensor->current_gain];

	/* FIXME: optimise I2C writes! */
	rval = smia_i2c_write_reg(sensor->i2c_client, SMIA_REG_8BIT,
				  0x124a, new.analog >> 8);
	if (rval)
		return rval;
	rval = smia_i2c_write_reg(sensor->i2c_client, SMIA_REG_8BIT,
				  0x1249, new.analog & 0xff);
	if (rval)
		return rval;

	rval = smia_i2c_write_reg(sensor->i2c_client, SMIA_REG_8BIT,
				  0x124d, new.digital >> 8);
	if (rval)
		return rval;
	rval = smia_i2c_write_reg(sensor->i2c_client, SMIA_REG_8BIT,
				  0x124c, new.digital & 0xff);

	return rval;
}

static int et8ek8_apply_exposure(struct et8ek8_sensor *sensor)
{
	return smia_i2c_write_reg(sensor->i2c_client, SMIA_REG_16BIT,
				  0x1243, swab16(sensor->current_exposure));
}

static int et8ek8_configure(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval, val;

#ifdef USE_CRC
	rval = smia_i2c_read_reg(sensor->i2c_client,
				 SMIA_REG_8BIT, 0x1263, &val);
	if (rval)
		goto fail;
#if USE_CRC
	val |= (1<<4);
#else
	val &= ~(1<<4);
#endif
	rval = smia_i2c_write_reg(sensor->i2c_client,
				  SMIA_REG_8BIT, 0x1263, val);
	if (rval)
		goto fail;
#endif

	rval = et8ek8_apply_gain(sensor);
	if (rval)
		goto fail;

	rval = et8ek8_apply_exposure(sensor);
	if (rval)
		goto fail;

	rval = smia_i2c_write_regs(sensor->i2c_client,
				   sensor->current_reglist->regs);
	if (rval)
		goto fail;

	return rval;

fail:
	dev_err(&sensor->i2c_client->dev, "sensor configuration failed\n");
	return rval;

}

static int et8ek8_setup_if(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval;

	if (sensor->current_reglist) {
		rval = sensor->platform_data->configure_interface(
			s,
			sensor->current_reglist->mode.window_width,
			sensor->current_reglist->mode.window_height,
			sensor->current_reglist->mode.pixel_format);
		if (rval)
			return rval;
	}

	sensor->platform_data->set_xclk(s, ET8EK8_XCLK_HZ);
	msleep(5000*1000/ET8EK8_XCLK_HZ+1);		/* Wait 5000 cycles */

	return 0;
}

static int et8ek8_resume(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval;

	rval = et8ek8_setup_if(s);
	if (rval)
		return rval;

	if (!sensor->current_reglist)
		return 0;

	rval = smia_i2c_reglist_find_write(sensor->i2c_client,
					   sensor->meta_reglist,
					   SMIA_REGLIST_POWERON);
	if (rval)
		return rval;

	return et8ek8_configure(s);
}

static int et8ek8_standby(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval;

	rval = smia_i2c_reglist_find_write(sensor->i2c_client,
					   sensor->meta_reglist,
					   SMIA_REGLIST_STANDBY);
	if (rval)
		return rval;

	sensor->platform_data->set_xclk(s, 0);

	return 0;
}

static int et8ek8_power_on(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval;

	rval = sensor->platform_data->power_on(s);
	if (rval)
		goto out;

	/* 1 ms delay before enabling EXTCLK */
	msleep(1);

	rval = et8ek8_resume(s);

out:
	if (rval)
		sensor->platform_data->power_off(s);

	return rval;
}

static int et8ek8_s_power(struct v4l2_int_device *s, enum v4l2_power state)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval = 0;
	enum v4l2_power old_state;

	old_state = sensor->power;
	sensor->power = state;

	switch (state) {
	case V4L2_POWER_OFF:
		rval = sensor->platform_data->power_off(s);
		break;
	case V4L2_POWER_ON:
		rval = et8ek8_power_on(s);
		break;
/* 	case V4L2_POWER_STANDBY: */
/* 		rval = et8ek8_standby(s); */
/* 		break; */
	default:
		return -EINVAL;
	}

	if (rval)
		sensor->power = old_state;

	return rval;
}

static int et8ek8_set_gain(struct et8ek8_sensor *sensor, int gain)
{
	int r = 0;

	sensor->current_gain =
		clamp(gain, 0, (int)(ARRAY_SIZE(et8ek8_gain_table) - 1));
	if (sensor->power == V4L2_POWER_ON)
		r = et8ek8_apply_gain(sensor);

	return r;
}

static int et8ek8_set_exposure(struct et8ek8_sensor *sensor, int mode,
			       int exp_time)
{
	int r = 0;
	sensor->current_exposure =
		clamp(exp_time, ET8EK8_MES_MIN,
		      (int)sensor->current_reglist->mode.max_exp);
	if (sensor->power == V4L2_POWER_ON)
		et8ek8_apply_exposure(sensor);

	return r;
}

static struct v4l2_queryctrl et8ek8_ctrls[] = {
	{
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain [0.1 EV]",
		.minimum	= 0,
		.maximum	= ARRAY_SIZE(et8ek8_gain_table) - 1,
		.step		= 1,
		.default_value	= DEFAULT_GAIN,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure time [rows]",
		.minimum	= ET8EK8_MES_MIN,
		.maximum	= ET8EK8_MES_MAX,
		.step		= 1,
		.default_value	= DEFAULT_EXPOSURE,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
};

static int et8ek8_ioctl_queryctrl(struct v4l2_int_device *s,
				  struct v4l2_queryctrl *a)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval;

	rval = smia_ctrl_query(et8ek8_ctrls, ARRAY_SIZE(et8ek8_ctrls), a);

	if (!rval && a->id == V4L2_CID_EXPOSURE)
		a->maximum = sensor->current_reglist->mode.max_exp;

	return rval;
}

static int et8ek8_ioctl_g_ctrl(struct v4l2_int_device *s,
			       struct v4l2_control *vc)
{
	struct et8ek8_sensor *sensor = s->priv;

	switch (vc->id) {
	case V4L2_CID_GAIN:
		vc->value = sensor->current_gain;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = sensor->current_exposure;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int et8ek8_ioctl_s_ctrl(struct v4l2_int_device *s,
			struct v4l2_control *vc)
{
	struct et8ek8_sensor *sensor = s->priv;
	int r = 0;

	switch (vc->id) {
	case V4L2_CID_GAIN:
		r = et8ek8_set_gain(sensor, vc->value);
		break;
	case V4L2_CID_EXPOSURE:
		r = et8ek8_set_exposure(sensor, 0, vc->value);
		break;
	default:
		return -EINVAL;
	}
	return r;
}

static int et8ek8_ioctl_s_power(struct v4l2_int_device *s,
				enum v4l2_power state);

static int et8ek8_ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				     struct v4l2_fmtdesc *fmt)
{
	struct et8ek8_sensor *sensor = s->priv;

	return smia_reglist_enum_fmt(sensor->meta_reglist, fmt);
}


static int et8ek8_ioctl_g_fmt_cap(struct v4l2_int_device *s,
				  struct v4l2_format *f)
{
	struct et8ek8_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = sensor->current_reglist->mode.window_width;
	pix->height = sensor->current_reglist->mode.window_height;
	pix->pixelformat = sensor->current_reglist->mode.pixel_format;

	return 0;
}

static int et8ek8_ioctl_s_fmt_cap(struct v4l2_int_device *s,
				  struct v4l2_format *f)
{
	struct et8ek8_sensor *sensor = s->priv;
	struct smia_reglist *reglist;

	reglist = smia_reglist_find_mode_fmt(sensor->meta_reglist,
					     sensor->current_reglist, f);

	if (!reglist)
		return -EINVAL;

	sensor->current_reglist = reglist;

	return 0;
}

static int et8ek8_ioctl_g_parm(struct v4l2_int_device *s,
			       struct v4l2_streamparm *a)
{
	struct et8ek8_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->current_reglist->mode.timeperframe;

	return 0;
}

static int et8ek8_ioctl_s_parm(struct v4l2_int_device *s,
			       struct v4l2_streamparm *a)
{
	struct et8ek8_sensor *sensor = s->priv;
	struct smia_reglist *reglist;

	reglist = smia_reglist_find_mode_streamparm(sensor->meta_reglist,
						    sensor->current_reglist, a);

	if (!reglist)
		return -EINVAL;

	sensor->current_reglist = reglist;

	return 0;
}

static int et8ek8_g_priv_mem(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	unsigned int length = ET8EK8_PRIV_MEM_SIZE;
	unsigned int offset = 0;
	u8 *ptr  = sensor->priv_mem;
	int rval = 0;

	/* Read the EEPROM window-by-window, each window 8 bytes */
	do {
		u8 buffer[PRIV_MEM_WIN_SIZE];
		struct i2c_msg msg;
		int bytes, i;
		int ofs;

		/* Set the current window */
		rval = smia_i2c_write_reg(sensor->i2c_client,
					  SMIA_REG_8BIT,
					  0x0001,
					  0xe0 | (offset >> 3));
		if (rval < 0)
			goto out;

		/* Wait for status bit */
		i = 1000;
		do {
			u32 status;
			rval = smia_i2c_read_reg(sensor->i2c_client,
						 SMIA_REG_8BIT,
						 0x0003,
						 &status);
			if (rval < 0)
				goto out;
			if ((status & 0x08) == 0)
				break;
			if (--i == 0) {
				rval = -EIO;
				goto out;
			}
			msleep(1);
		} while (1);

		/* Read window, 8 bytes at once, and copy to user space */
		ofs = offset & 0x07;	/* Offset within this window */
		bytes = length + ofs > 8 ? 8-ofs : length;
		msg.addr = client->addr;
		msg.flags = 0;
		msg.len = 2;
		msg.buf = buffer;
		ofs += PRIV_MEM_START_REG;
		buffer[0] = (u8)(ofs >> 8);
		buffer[1] = (u8)(ofs & 0xFF);
		rval = i2c_transfer(client->adapter, &msg, 1);
		if (rval < 0)
			goto out;
		mdelay(ET8EK8_I2C_DELAY);
		msg.addr = client->addr;
		msg.len = bytes;
		msg.flags = I2C_M_RD;
		msg.buf = buffer;
		memset(buffer, 0, sizeof(buffer));
		rval = i2c_transfer(client->adapter, &msg, 1);
		if (rval < 0)
			goto out;
		rval = 0;
		memcpy(ptr, buffer, bytes);

		length -= bytes;
		offset += bytes;
		ptr    += bytes;
	} while (length > 0);

out:
	return rval;
}

static int et8ek8_dev_init(struct v4l2_int_device *s)
{
	struct et8ek8_sensor *sensor = s->priv;
	char name[FIRMWARE_NAME_MAX];
	int rval, rev_l, rev_h;

	rval = et8ek8_s_power(s, V4L2_POWER_ON);
	if (rval)
		return -ENODEV;

	if (smia_i2c_read_reg(sensor->i2c_client, SMIA_REG_8BIT,
			      REG_REVISION_NUMBER_L, &rev_l) != 0
	    || smia_i2c_read_reg(sensor->i2c_client, SMIA_REG_8BIT,
				 REG_REVISION_NUMBER_H, &rev_h) != 0) {
		dev_err(&sensor->i2c_client->dev,
			"no et8ek8 sensor detected\n");
		return -ENODEV;
	}

	sensor->version = (rev_h << 8) + rev_l;

	if (sensor->version != ET8EK8_REV_1
	    && sensor->version != ET8EK8_REV_2)
		dev_info(&sensor->i2c_client->dev,
			 "unknown version 0x%x detected, "
			 "continuing anyway\n", sensor->version);

	snprintf(name, FIRMWARE_NAME_MAX, "%s-%4.4x.bin", ET8EK8_NAME,
		 sensor->version);

	if (request_firmware(&sensor->fw, name,
			     &sensor->i2c_client->dev)) {
		dev_err(&sensor->i2c_client->dev,
			"can't load firmware %s\n", name);
		return -ENODEV;
	}

	sensor->meta_reglist =
		(struct smia_meta_reglist *)sensor->fw->data;

	rval = smia_reglist_import(sensor->meta_reglist);
	if (rval) {
		dev_err(&sensor->i2c_client->dev,
			"invalid register list %s, import failed\n",
			name);
		goto out_release;
	}

	sensor->current_reglist =
		smia_reglist_find_type(sensor->meta_reglist,
				       SMIA_REGLIST_MODE);
	if (!sensor->current_reglist) {
		dev_err(&sensor->i2c_client->dev,
			"invalid register list %s, no mode found\n",
			name);
		rval = -ENODEV;
		goto out_release;
	}

	rval = et8ek8_resume(s);
	if (rval)
		goto out_release;

	sensor->dev_init_done = true;

	return 0;

out_release:
	release_firmware(sensor->fw);
	sensor->meta_reglist = NULL;
	sensor->fw = NULL;

	return rval;
}

static int et8ek8_ioctl_s_power(struct v4l2_int_device *s,
				enum v4l2_power state)
{
	struct et8ek8_sensor *sensor = s->priv;
	int rval = 0;

	if (state == V4L2_POWER_ON)
		state = V4L2_POWER_ON;
	else
		state = V4L2_POWER_OFF;

	switch (state) {
	case V4L2_POWER_OFF:
		goto out;
	default:
		break;
	}

	if (!sensor->dev_init_done) {
		rval = et8ek8_dev_init(s);
		if (rval)
			goto out;
		rval = et8ek8_g_priv_mem(s);
	} else {
		rval = et8ek8_s_power(s, state);
	}

	if (rval)
		goto out;

	return 0;

out:
	et8ek8_s_power(s, V4L2_POWER_OFF);

	return rval;
}

static int et8ek8_ioctl_g_priv(struct v4l2_int_device *s, void *priv)
{
	struct et8ek8_sensor *sensor = s->priv;

	return sensor->platform_data->g_priv(s, priv);
}

static int et8ek8_ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frm)
{
	struct et8ek8_sensor *sensor = s->priv;

	return smia_reglist_enum_framesizes(sensor->meta_reglist, frm);
}

static int et8ek8_ioctl_enum_frameintervals(struct v4l2_int_device *s,
					    struct v4l2_frmivalenum *frm)
{
	struct et8ek8_sensor *sensor = s->priv;

	return smia_reglist_enum_frameintervals(sensor->meta_reglist, frm);
}

static int et8ek8_ioctl_enum_slaves(struct v4l2_int_device *s,
				    struct v4l2_slave_info *si)
{
	struct et8ek8_sensor *sensor = s->priv;

	strlcpy(si->driver, ET8EK8_NAME, sizeof(si->driver));
	strlcpy(si->bus_info, "ccp2", sizeof(si->bus_info));
	snprintf(si->version, sizeof(si->version), "%x", sensor->version);

	return 0;
}

static int et8ek8_ioctl_g_smia_mode(struct v4l2_int_device *s,
			     struct v4l2_smia_mode *mode)
{
	struct et8ek8_sensor *sensor = s->priv;
	struct smia_mode *sm = &sensor->current_reglist->mode;

	mode->width		= sm->width;
	mode->height		= sm->height;
	mode->window_origin_x	= sm->window_origin_x;
	mode->window_origin_y	= sm->window_origin_y;
	mode->window_width	= sm->window_width;
	mode->window_height	= sm->window_height;
	mode->pixel_clock	= sm->pixel_clock;

	return 0;
}

static ssize_t
et8ek8_priv_mem_read(struct device *dev, struct device_attribute *attr,
		     char *buf)
{
	struct et8ek8_sensor *sensor = dev_get_drvdata(dev);

#if PAGE_SIZE < ET8EK8_PRIV_MEM_SIZE
#error PAGE_SIZE too small!
#endif
	if (!sensor->dev_init_done)
		return -EBUSY;

	memcpy(buf, sensor->priv_mem, ET8EK8_PRIV_MEM_SIZE);

	return ET8EK8_PRIV_MEM_SIZE;
}
static DEVICE_ATTR(priv_mem, S_IRUGO, et8ek8_priv_mem_read, NULL);

static struct v4l2_int_ioctl_desc et8ek8_ioctl_desc[] = {
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_enum_fmt_cap },
	{ vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_g_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_g_fmt_cap },
	{ vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_s_fmt_cap },
	{ vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_queryctrl },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_s_ctrl },
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_s_parm },
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_s_power },
	{ vidioc_int_g_priv_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_g_priv },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_enum_framesizes },
	{ vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_enum_frameintervals },
	{ vidioc_int_enum_slaves_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_enum_slaves },
	{ vidioc_int_g_smia_mode_num,
	  (v4l2_int_ioctl_func *)et8ek8_ioctl_g_smia_mode },
};

static struct v4l2_int_slave et8ek8_slave = {
	.ioctls = et8ek8_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(et8ek8_ioctl_desc),
};

static struct et8ek8_sensor et8ek8;

static struct v4l2_int_device et8ek8_int_device = {
	.module = THIS_MODULE,
	.name = ET8EK8_NAME,
	.priv = &et8ek8,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &et8ek8_slave,
	},
};

static int et8ek8_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	struct et8ek8_sensor *sensor = &et8ek8;
	int rval;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->platform_data = client->dev.platform_data;

	if (sensor->platform_data == NULL)
		return -ENODEV;

	if (device_create_file(&client->dev, &dev_attr_priv_mem) != 0) {
		dev_err(&client->dev, "could not register sysfs entry\n");
		return -EBUSY;
	}

	sensor->v4l2_int_device = &et8ek8_int_device;
	sensor->current_gain = DEFAULT_GAIN;
	BUG_ON(sensor->current_gain >= ARRAY_SIZE(et8ek8_gain_table));
	sensor->current_exposure = DEFAULT_EXPOSURE;

	sensor->i2c_client = client;
	i2c_set_clientdata(client, sensor);
	dev_set_drvdata(&client->dev, sensor);

	rval = v4l2_int_device_register(sensor->v4l2_int_device);
	if (rval) {
		device_remove_file(&client->dev, &dev_attr_priv_mem);
		i2c_set_clientdata(client, NULL);
		dev_set_drvdata(&client->dev, NULL);
	}

	return rval;
}

static int __exit et8ek8_remove(struct i2c_client *client)
{
	struct et8ek8_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	device_remove_file(&client->dev, &dev_attr_priv_mem);
	release_firmware(sensor->fw);

	return 0;
}

static const struct i2c_device_id et8ek8_id_table[] = {
	{ ET8EK8_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, et8ek8_id_table);

static struct i2c_driver et8ek8_i2c_driver = {
	.driver		= {
		.name	= ET8EK8_NAME,
	},
	.probe		= et8ek8_probe,
	.remove		= __exit_p(et8ek8_remove),
	.id_table	= et8ek8_id_table,
};

static int __init et8ek8_init(void)
{
	int rval;

	rval = i2c_add_driver(&et8ek8_i2c_driver);
	if (rval)
		printk(KERN_ALERT "%s: failed at i2c_add_driver\n", __func__);

	return rval;
}

static void __exit et8ek8_exit(void)
{
	i2c_del_driver(&et8ek8_i2c_driver);
}

/*
 * FIXME: Menelaus isn't ready (?) at module_init stage, so use
 * late_initcall for now.
 */
late_initcall(et8ek8_init);
module_exit(et8ek8_exit);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@nokia.com>");
MODULE_DESCRIPTION("Toshiba ET8EK8 camera sensor driver");
MODULE_LICENSE("GPL");
