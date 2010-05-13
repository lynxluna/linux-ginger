/*
 * ispccdc.c
 *
 * Driver Library for CCDC module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Pallavi Kulkarni <p-kulkarni@ti.com>
 *	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"

#define LSC_TABLE_INIT_SIZE	50052

/* Structure for saving/restoring CCDC module registers*/
static struct isp_reg ispccdc_reg_list[] = {
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HD_VD_WID, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PIX_LINES, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HORZ_INFO, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_START, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_LINES, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CULLING, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_DCSUB, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_COLPTN, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_BLKCMP, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC_ADDR, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VDINT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_REC656IF, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_HORZ, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_VERT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR2, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR3, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR4, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR5, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR6, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR7, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VP_OUT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_INITIAL, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_OFFSET, 0},
	{0, ISP_TOK_TERM, 0}
};

/**
 * omap34xx_isp_ccdc_config - Set CCDC configuration from userspace
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @userspace_add: Structure containing CCDC configuration sent from userspace.
 *
 * Returns 0 if successful, -EINVAL if the pointer to the configuration
 * structure is null, or the copy_from_user function fails to copy user space
 * memory to kernel space memory.
 **/
int omap34xx_isp_ccdc_config(struct isp_ccdc_device *isp_ccdc,
			     void *userspace_add)
{
	struct isp_device *isp =
		container_of(isp_ccdc, struct isp_device, isp_ccdc);
	struct ispccdc_bclamp bclamp_t;
	struct ispccdc_blcomp blcomp_t;
	struct ispccdc_fpc fpc_t;
	struct ispccdc_culling cull_t;
	struct ispccdc_update_config *ccdc_struct;

	if (userspace_add == NULL)
		return -EINVAL;

	ccdc_struct = userspace_add;

	if (ISP_ABS_CCDC_ALAW & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_ALAW & ccdc_struct->update)
			ispccdc_config_alaw(isp_ccdc, ccdc_struct->alawip);
		ispccdc_enable_alaw(isp_ccdc, 1);
	} else if (ISP_ABS_CCDC_ALAW & ccdc_struct->update)
		ispccdc_enable_alaw(isp_ccdc, 0);

	if (ISP_ABS_CCDC_LPF & ccdc_struct->flag)
		ispccdc_enable_lpf(isp_ccdc, 1);
	else
		ispccdc_enable_lpf(isp_ccdc, 0);

	if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) {
			if (copy_from_user(&bclamp_t, (struct ispccdc_bclamp *)
					   ccdc_struct->bclamp,
					   sizeof(struct ispccdc_bclamp)))
				goto copy_from_user_err;

			ispccdc_enable_black_clamp(isp_ccdc, 1);
			ispccdc_config_black_clamp(isp_ccdc, bclamp_t);
		} else
			ispccdc_enable_black_clamp(isp_ccdc, 1);
	} else {
		if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) {
			if (copy_from_user(&bclamp_t, (struct ispccdc_bclamp *)
					   ccdc_struct->bclamp,
					   sizeof(struct ispccdc_bclamp)))
				goto copy_from_user_err;

			ispccdc_enable_black_clamp(isp_ccdc, 0);
			ispccdc_config_black_clamp(isp_ccdc, bclamp_t);
		}
	}

	if (ISP_ABS_CCDC_BCOMP & ccdc_struct->update) {
		if (copy_from_user(&blcomp_t, (struct ispccdc_blcomp *)
				   ccdc_struct->blcomp,
				   sizeof(blcomp_t)))
			goto copy_from_user_err;

		ispccdc_config_black_comp(isp_ccdc, blcomp_t);
	}

	if (ISP_ABS_CCDC_FPC & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_FPC & ccdc_struct->update) {
			if (copy_from_user(&fpc_t, (struct ispccdc_fpc *)
					   ccdc_struct->fpc,
					   sizeof(fpc_t)))
				goto copy_from_user_err;
			isp_ccdc->fpc_table_add = kmalloc(64 + fpc_t.fpnum * 4,
						GFP_KERNEL | GFP_DMA);
			if (!isp_ccdc->fpc_table_add) {
				dev_err(isp_ccdc->dev,
					"ccdc: Cannot allocate memory for"
					" FPC table");
				return -ENOMEM;
			}
			while (((unsigned long)isp_ccdc->fpc_table_add
				& 0xFFFFFFC0)
			       != (unsigned long)isp_ccdc->fpc_table_add)
				isp_ccdc->fpc_table_add++;

			isp_ccdc->fpc_table_add_m = iommu_kmap(
				isp->iommu,
				0,
				virt_to_phys(isp_ccdc->fpc_table_add),
				fpc_t.fpnum * 4,
				IOMMU_FLAG);
			/* FIXME: Correct unwinding */
			BUG_ON(IS_ERR_VALUE(isp_ccdc->fpc_table_add_m));

			if (copy_from_user(isp_ccdc->fpc_table_add,
					   (u32 *)fpc_t.fpcaddr,
					   fpc_t.fpnum * 4))
				goto copy_from_user_err;

			fpc_t.fpcaddr = isp_ccdc->fpc_table_add_m;
			ispccdc_config_fpc(isp_ccdc, fpc_t);
		}
		ispccdc_enable_fpc(isp_ccdc, 1);
	} else if (ISP_ABS_CCDC_FPC & ccdc_struct->update)
		ispccdc_enable_fpc(isp_ccdc, 0);

	if (ISP_ABS_CCDC_CULL & ccdc_struct->update) {
		if (copy_from_user(&cull_t, (struct ispccdc_culling *)
				   ccdc_struct->cull,
				   sizeof(cull_t)))
			goto copy_from_user_err;
		ispccdc_config_culling(isp_ccdc, cull_t);
	}

	if (is_isplsc_activated()) {
		if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->flag) {
			if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->update) {
				if (copy_from_user(
					    &isp_ccdc->lsc_config,
					    (struct ispccdc_lsc_config *)
					    ccdc_struct->lsc_cfg,
					    sizeof(struct ispccdc_lsc_config)))
					goto copy_from_user_err;
				ispccdc_config_lsc(isp_ccdc,
						   &isp_ccdc->lsc_config);
			}
			ispccdc_enable_lsc(isp_ccdc, 1);
		} else if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->update) {
			ispccdc_enable_lsc(isp_ccdc, 0);
		}
		if (ISP_ABS_TBL_LSC & ccdc_struct->update) {
			if (copy_from_user(isp_ccdc->lsc_gain_table,
					   ccdc_struct->lsc,
					   isp_ccdc->lsc_config.size))
				goto copy_from_user_err;
			ispccdc_load_lsc(isp_ccdc, isp_ccdc->lsc_gain_table,
					 isp_ccdc->lsc_config.size);
		}
	}

	if (ISP_ABS_CCDC_COLPTN & ccdc_struct->update)
		ispccdc_config_imgattr(isp_ccdc, ccdc_struct->colptn);

	return 0;

copy_from_user_err:
	dev_err(isp_ccdc->dev, "ccdc: Config: copy from user error");
	return -EINVAL ;
}
EXPORT_SYMBOL(omap34xx_isp_ccdc_config);

/**
 * ispccdc_set_wenlog - Set the CCDC Write Enable valid region.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @wenlog: Write enable logic to apply against valid area. 0 - AND, 1 - OR.
 */
void ispccdc_set_wenlog(struct isp_ccdc_device *isp_ccdc, u32 wenlog)
{
	isp_ccdc->wenlog = wenlog;
}
EXPORT_SYMBOL(ispccdc_set_wenlog);

/**
 * ispccdc_request - Reserve the CCDC module.
 * @isp_ccdc: Pointer to ISP CCDC device.
 *
 * Returns 0 if successful, or -EBUSY if CCDC module is busy.
 **/
int ispccdc_request(struct isp_ccdc_device *isp_ccdc)
{
	mutex_lock(&isp_ccdc->mutexlock);
	if (isp_ccdc->ccdc_inuse) {
		mutex_unlock(&isp_ccdc->mutexlock);
		DPRINTK_ISPCCDC("ISP_ERR : CCDC Module Busy\n");
		return -EBUSY;
	}

	isp_ccdc->ccdc_inuse = 1;
	mutex_unlock(&isp_ccdc->mutexlock);
	isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
		   ISPCTRL_CCDC_RAM_EN | ISPCTRL_CCDC_CLK_EN |
		   ISPCTRL_SBL_WR1_RAM_EN);
	isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
		   ISPCCDC_CFG_VDLC);
	return 0;
}
EXPORT_SYMBOL(ispccdc_request);

/**
 * ispccdc_free - Free the CCDC module.
 * @isp_ccdc: Pointer to ISP CCDC device.
 *
 * Frees the CCDC module so it can be used by another process.
 *
 * Returns 0 if successful, or -EINVAL if module has been already freed.
 **/
int ispccdc_free(struct isp_ccdc_device *isp_ccdc)
{
	mutex_lock(&isp_ccdc->mutexlock);
	if (!isp_ccdc->ccdc_inuse) {
		mutex_unlock(&isp_ccdc->mutexlock);
		DPRINTK_ISPCCDC("ISP_ERR: CCDC Module already freed\n");
		return -EINVAL;
	}

	isp_ccdc->ccdc_inuse = 0;
	mutex_unlock(&isp_ccdc->mutexlock);
	isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
		    ~(ISPCTRL_CCDC_CLK_EN |
		      ISPCTRL_CCDC_RAM_EN |
		      ISPCTRL_SBL_WR1_RAM_EN));
	return 0;
}
EXPORT_SYMBOL(ispccdc_free);

/**
 * ispccdc_free_lsc - Frees Lens Shading Compensation table
 * @isp_ccdc: Pointer to ISP CCDC device.
 *
 * Always returns 0.
 **/
static int ispccdc_free_lsc(struct isp_ccdc_device *isp_ccdc)
{
	struct isp_device *isp =
		container_of(isp_ccdc, struct isp_device, isp_ccdc);

	if (!isp_ccdc->lsc_ispmmu_addr)
		return 0;

	ispccdc_enable_lsc(isp_ccdc, 0);
	isp_ccdc->lsc_initialized = 0;
	isp_reg_writel(isp_ccdc->dev, 0, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_TABLE_BASE);
	iommu_kunmap(isp->iommu, isp_ccdc->lsc_ispmmu_addr);
	kfree(isp_ccdc->lsc_gain_table);
	return 0;
}

/**
 * ispccdc_allocate_lsc - Allocate space for Lens Shading Compensation table
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @table_size: LSC gain table size.
 *
 * Returns 0 if successful, -ENOMEM of its no memory available, or -EINVAL if
 * table_size is zero.
 **/
static int ispccdc_allocate_lsc(struct isp_ccdc_device *isp_ccdc,
				u32 table_size)
{
	struct isp_device *isp =
		container_of(isp_ccdc, struct isp_device, isp_ccdc);

	if (table_size == 0)
		return -EINVAL;

	if ((isp_ccdc->lsc_config.size >= table_size)
	    && isp_ccdc->lsc_gain_table)
		return 0;

	ispccdc_free_lsc(isp_ccdc);

	isp_ccdc->lsc_gain_table = kmalloc(table_size, GFP_KERNEL | GFP_DMA);

	if (!isp_ccdc->lsc_gain_table) {
		dev_err(isp_ccdc->dev,
			"ccdc: Cannot allocate memory for gain tables\n");
		return -ENOMEM;
	}

	isp_ccdc->lsc_ispmmu_addr =
		iommu_kmap(isp->iommu,
			   0,
			   virt_to_phys(isp_ccdc->lsc_gain_table),
			   table_size,
			   IOMMU_FLAG);
	if (IS_ERR_VALUE(isp_ccdc->lsc_ispmmu_addr)) {
		dev_err(isp_ccdc->dev,
			"ccdc: Cannot map memory for gain tables\n");
		kfree(isp_ccdc->lsc_gain_table);
		return -ENOMEM;
	}

	return 0;
}

/**
 * ispccdc_program_lsc - Program Lens Shading Compensation table.
 * @isp_ccdc: Pointer to ISP CCDC device.
 *
 * Returns 0 if successful, or -EINVAL if there's no mapped address for the
 * table yet.
 **/
static int ispccdc_program_lsc(struct isp_ccdc_device *isp_ccdc)
{
	if (!isp_ccdc->lsc_ispmmu_addr)
		return -EINVAL;

	if (isp_ccdc->lsc_initialized)
		return 0;

	isp_reg_writel(isp_ccdc->dev, isp_ccdc->lsc_ispmmu_addr,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE);
	isp_ccdc->lsc_initialized = 1;
	return 0;
}

/**
 * ispccdc_load_lsc - Load Lens Shading Compensation table.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @table_addr: MMU Mapped address to LSC gain table.
 * @table_size: LSC gain table size.
 *
 * Returns 0 if successful, -ENOMEM of its no memory available, or -EINVAL if
 * table_size is zero.
 **/
int ispccdc_load_lsc(struct isp_ccdc_device *isp_ccdc, u8 *table_addr,
		     u32 table_size)
{
	int ret;

	if (!is_isplsc_activated())
		return 0;

	if (!table_addr)
		return -EINVAL;

	ret = ispccdc_allocate_lsc(isp_ccdc, table_size);
	if (ret)
		return ret;

	if (table_addr != isp_ccdc->lsc_gain_table)
		memcpy(isp_ccdc->lsc_gain_table, table_addr, table_size);
	ret = ispccdc_program_lsc(isp_ccdc);
	if (ret)
		return ret;
	return 0;
}
EXPORT_SYMBOL(ispccdc_load_lsc);

/**
 * ispccdc_config_lsc - Configures the lens shading compensation module
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @lsc_cfg: Pointer to LSC configuration structure
 **/
void ispccdc_config_lsc(struct isp_ccdc_device *isp_ccdc,
			struct ispccdc_lsc_config *lsc_cfg)
{
	int reg;

	if (!is_isplsc_activated())
		return;

	ispccdc_enable_lsc(isp_ccdc, 0);
	isp_reg_writel(isp_ccdc->dev, lsc_cfg->offset, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_TABLE_OFFSET);

	reg = 0;
	reg |= lsc_cfg->gain_mode_n << ISPCCDC_LSC_GAIN_MODE_N_SHIFT;
	reg |= lsc_cfg->gain_mode_m << ISPCCDC_LSC_GAIN_MODE_M_SHIFT;
	reg |= lsc_cfg->gain_format << ISPCCDC_LSC_GAIN_FORMAT_SHIFT;
	isp_reg_writel(isp_ccdc->dev, reg, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_CONFIG);

	reg = 0;
	reg &= ~ISPCCDC_LSC_INITIAL_X_MASK;
	reg |= lsc_cfg->initial_x << ISPCCDC_LSC_INITIAL_X_SHIFT;
	reg &= ~ISPCCDC_LSC_INITIAL_Y_MASK;
	reg |= lsc_cfg->initial_y << ISPCCDC_LSC_INITIAL_Y_SHIFT;
	isp_reg_writel(isp_ccdc->dev, reg, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_INITIAL);
}
EXPORT_SYMBOL(ispccdc_config_lsc);

/**
 * ispccdc_enable_lsc - Enables/Disables the Lens Shading Compensation module.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LSC, 1 Enables LSC.
 **/
void ispccdc_enable_lsc(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	if (!is_isplsc_activated())
		return;

	if (enable) {
		if (!ispccdc_busy(isp_ccdc)) {
			isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN,
				   ISP_CTRL, ISPCTRL_SBL_SHARED_RPORTB
				   | ISPCTRL_SBL_RD_RAM_EN);

			isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				   ISPCCDC_LSC_CONFIG, 0x1);

			isp_ccdc->lsc_state = 1;
		} else {
			/* Postpone enabling LSC */
			isp_ccdc->lsc_enable = 1;
		}
	} else {
		isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			    ISPCCDC_LSC_CONFIG, 0xFFFE);
		isp_ccdc->lsc_state = 0;
		isp_ccdc->lsc_enable = 0;
	}
}
EXPORT_SYMBOL(ispccdc_enable_lsc);

/**
 * ispccdc_lsc_error_handler - Handle LSC prefetch error scenario.
 * @isp_ccdc: Pointer to ISP CCDC device.
 *
 * Disables LSC, and defers enablement to shadow registers update time.
 **/
void ispccdc_lsc_error_handler(struct isp_ccdc_device *isp_ccdc)
{
	int lsc_enable = isp_ccdc->lsc_state;

	ispccdc_enable_lsc(isp_ccdc, 0);

	isp_ccdc->lsc_enable = lsc_enable;
}

/**
 * ispccdc_config_crop - Configures crop parameters for the ISP CCDC.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @left: Left offset of the crop area.
 * @top: Top offset of the crop area.
 * @height: Height of the crop area.
 * @width: Width of the crop area.
 *
 * The following restrictions are applied for the crop settings. If incoming
 * values do not follow these restrictions then we map the settings to the
 * closest acceptable crop value.
 * 1) Left offset is always odd. This can be avoided if we enable byte swap
 *    option for incoming data into CCDC.
 * 2) Top offset is always even.
 * 3) Crop height is always even.
 * 4) Crop width is always a multiple of 16 pixels
 **/
void ispccdc_config_crop(struct isp_ccdc_device *isp_ccdc, u32 left, u32 top,
			 u32 height, u32 width)
{
	isp_ccdc->ccdcin_woffset = left + (left % 2);
	isp_ccdc->ccdcin_hoffset = top + (top % 2);

	isp_ccdc->crop_w = width - (width % 16);
	isp_ccdc->crop_h = height + (height % 2);

	DPRINTK_ISPCCDC("\n\tOffsets L %d T %d W %d H %d\n",
			isp_ccdc->ccdcin_woffset,
			isp_ccdc->ccdcin_hoffset,
			isp_ccdc->crop_w,
			isp_ccdc->crop_h);
}

/**
 * ispccdc_config_datapath - Specify the input and output modules for CCDC.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @pipe: Pointer to ISP pipeline structure to base on for config.
 *
 * Configures the default configuration for the CCDC to work with.
 *
 * The valid values for the input are CCDC_RAW (0), CCDC_YUV_SYNC (1),
 * CCDC_YUV_BT (2), and CCDC_OTHERS (3).
 *
 * The valid values for the output are CCDC_YUV_RSZ (0), CCDC_YUV_MEM_RSZ (1),
 * CCDC_OTHERS_VP (2), CCDC_OTHERS_MEM (3), CCDC_OTHERS_VP_MEM (4).
 *
 * Returns 0 if successful, or -EINVAL if wrong I/O combination or wrong input
 * or output values.
 **/
static int ispccdc_config_datapath(struct isp_ccdc_device *isp_ccdc,
				   struct isp_pipeline *pipe)
{
	u32 syn_mode = 0;
	struct ispccdc_vp vpcfg;
	struct ispccdc_syncif syncif;
	struct ispccdc_bclamp blkcfg;

	u32 colptn = ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC0_SHIFT |
		ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP0PLC1_SHIFT |
		ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC2_SHIFT |
		ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP0PLC3_SHIFT |
		ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP1PLC0_SHIFT |
		ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP1PLC1_SHIFT |
		ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP1PLC2_SHIFT |
		ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP1PLC3_SHIFT |
		ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC0_SHIFT |
		ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP2PLC1_SHIFT |
		ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC2_SHIFT |
		ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP2PLC3_SHIFT |
		ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP3PLC0_SHIFT |
		ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP3PLC1_SHIFT |
		ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP3PLC2_SHIFT |
		ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

	syn_mode = isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				 ISPCCDC_SYN_MODE);

	switch (pipe->ccdc_out) {
	case CCDC_YUV_RSZ:
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		break;

	case CCDC_YUV_MEM_RSZ:
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		isp_ccdc->wen = 1;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		break;

	case CCDC_OTHERS_VP:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(isp_ccdc, vpcfg);
		ispccdc_enable_vp(isp_ccdc, 1);
		break;

	case CCDC_OTHERS_MEM:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		if (pipe->ccdc_in == CCDC_YUV_BT) {
			syn_mode &= ~ISPCCDC_SYN_MODE_EXWEN;
			isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_CFG, ~ISPCCDC_CFG_WENLOG);
		} else {
			syn_mode |= ISPCCDC_SYN_MODE_EXWEN;
			isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_CFG, ISPCCDC_CFG_WENLOG);
		}
		vpcfg.bitshift_sel = BIT11_2;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(isp_ccdc, vpcfg);
		ispccdc_enable_vp(isp_ccdc, 0);
		break;

	case CCDC_OTHERS_VP_MEM:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		syn_mode &= ~ISPCCDC_SYN_MODE_EXWEN;

		isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_CFG, ~ISPCCDC_CFG_WENLOG,
			       isp_ccdc->wenlog);
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(isp_ccdc, vpcfg);
		ispccdc_enable_vp(isp_ccdc, 1);
		break;
	default:
		DPRINTK_ISPCCDC("ISP_ERR: Wrong CCDC Output\n");
		return -EINVAL;
	};

	isp_reg_writel(isp_ccdc->dev, syn_mode, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_SYN_MODE);

	switch (pipe->ccdc_in) {
	case CCDC_RAW_GRBG:
	case CCDC_RAW_RGGB:
	case CCDC_RAW_BGGR:
	case CCDC_RAW_GBRG:
		syncif.ccdc_mastermode = 0;
		syncif.datapol = 0;
		syncif.datsz = DAT10;
		syncif.fldmode = 0;
		syncif.fldout = 0;
		syncif.fldpol = 0;
		syncif.fldstat = 0;
		syncif.hdpol = 0;
		syncif.ipmod = RAW;
		syncif.vdpol = 0;
		syncif.bt_r656_en = 0;
		ispccdc_config_sync_if(isp_ccdc, syncif);
		ispccdc_config_imgattr(isp_ccdc, colptn);
		blkcfg.dcsubval = 64;
		ispccdc_config_black_clamp(isp_ccdc, blkcfg);
		if (is_isplsc_activated()) {
			ispccdc_config_lsc(isp_ccdc, &isp_ccdc->lsc_config);
			ispccdc_load_lsc(isp_ccdc, isp_ccdc->lsc_gain_table_tmp,
					 LSC_TABLE_INIT_SIZE);
		}

		break;
	case CCDC_YUV_SYNC:
		syncif.ccdc_mastermode = 0;
		syncif.datapol = 0;
		syncif.datsz = DAT8;
		syncif.fldmode = 0;
		syncif.fldout = 0;
		syncif.fldpol = 0;
		syncif.fldstat = 0;
		syncif.hdpol = 0;
		syncif.ipmod = YUV16;
		syncif.vdpol = 1;
		syncif.bt_r656_en = 0;
		ispccdc_config_imgattr(isp_ccdc, 0);
		ispccdc_config_sync_if(isp_ccdc, syncif);
		blkcfg.dcsubval = 0;
		ispccdc_config_black_clamp(isp_ccdc, blkcfg);
		break;
	case CCDC_YUV_BT:
		syncif.ccdc_mastermode = 0;
		syncif.datapol = 0;
		syncif.datsz = DAT8;
		syncif.fldmode = 1;
		syncif.fldout = 0;
		syncif.fldpol = 0;
		syncif.fldstat = 0;
		syncif.hdpol = 0;
		syncif.ipmod = YUV8;
		syncif.vdpol = 1;
		syncif.bt_r656_en = 1;
		ispccdc_config_imgattr(isp_ccdc, 0);
		ispccdc_config_sync_if(isp_ccdc, syncif);
		blkcfg.dcsubval = 0;
		ispccdc_config_black_clamp(isp_ccdc, blkcfg);
		break;
	case CCDC_OTHERS:
		break;
	default:
		DPRINTK_ISPCCDC("ISP_ERR: Wrong CCDC Input\n");
		return -EINVAL;
	}

	ispccdc_print_status(isp_ccdc, pipe);
	isp_print_status(isp_ccdc->dev);
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_datapath);

/**
 * ispccdc_config_sync_if - Set CCDC sync interface params between sensor and CCDC.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @syncif: Structure containing the sync parameters like field state, CCDC in
 *          master/slave mode, raw/yuv data, polarity of data, field, hs, vs
 *          signals.
 **/
void ispccdc_config_sync_if(struct isp_ccdc_device *isp_ccdc,
			    struct ispccdc_syncif syncif)
{
	u32 syn_mode = isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				     ISPCCDC_SYN_MODE);

	syn_mode |= ISPCCDC_SYN_MODE_VDHDEN;

	if (syncif.fldstat)
		syn_mode |= ISPCCDC_SYN_MODE_FLDSTAT;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDSTAT;

	syn_mode &= ISPCCDC_SYN_MODE_INPMOD_MASK;
	isp_ccdc->syncif_ipmod = syncif.ipmod;

	switch (syncif.ipmod) {
	case RAW:
		break;
	case YUV16:
		syn_mode |= ISPCCDC_SYN_MODE_INPMOD_YCBCR16;
		break;
	case YUV8:
		syn_mode |= ISPCCDC_SYN_MODE_INPMOD_YCBCR8;
		if (syncif.bt_r656_en)
			syn_mode |= ISPCCDC_SYN_MODE_PACK8;
		break;
	};

	syn_mode &= ISPCCDC_SYN_MODE_DATSIZ_MASK;
	switch (syncif.datsz) {
	case DAT8:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_8;
		break;
	case DAT10:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_10;
		break;
	case DAT11:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_11;
		break;
	case DAT12:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_12;
		break;
	};

	if (syncif.fldmode)
		syn_mode |= ISPCCDC_SYN_MODE_FLDMODE;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDMODE;

	if (syncif.datapol)
		syn_mode |= ISPCCDC_SYN_MODE_DATAPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_DATAPOL;

	if (syncif.fldpol)
		syn_mode |= ISPCCDC_SYN_MODE_FLDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDPOL;

	if (syncif.hdpol)
		syn_mode |= ISPCCDC_SYN_MODE_HDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_HDPOL;

	if (syncif.vdpol)
		syn_mode |= ISPCCDC_SYN_MODE_VDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_VDPOL;

	if (syncif.ccdc_mastermode) {
		syn_mode |= ISPCCDC_SYN_MODE_FLDOUT | ISPCCDC_SYN_MODE_VDHDOUT;
		isp_reg_writel(isp_ccdc->dev,
			       syncif.hs_width << ISPCCDC_HD_VD_WID_HDW_SHIFT
			       | syncif.vs_width << ISPCCDC_HD_VD_WID_VDW_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_HD_VD_WID);

		isp_reg_writel(isp_ccdc->dev,
			       syncif.ppln << ISPCCDC_PIX_LINES_PPLN_SHIFT
			       | syncif.hlprf << ISPCCDC_PIX_LINES_HLPRF_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_PIX_LINES);
	} else
		syn_mode &= ~(ISPCCDC_SYN_MODE_FLDOUT |
			      ISPCCDC_SYN_MODE_VDHDOUT);

	isp_reg_writel(isp_ccdc->dev, syn_mode, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_SYN_MODE);

	if (!(syncif.bt_r656_en)) {
		isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			    ISPCCDC_REC656IF, ~ISPCCDC_REC656IF_R656ON);
	} else {
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			ISPCCDC_REC656IF, ISPCCDC_REC656IF_R656ON |
			ISPCCDC_REC656IF_ECCFVH);
	}
}
EXPORT_SYMBOL(ispccdc_config_sync_if);

/**
 * ispccdc_config_black_clamp - Configures the clamp parameters in CCDC.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @bclamp: Structure containing the optical black average gain, optical black
 *          sample length, sample lines, and the start pixel position of the
 *          samples w.r.t the HS pulse.
 *
 * Configures the clamp parameters in CCDC. Either if its being used the
 * optical black clamp, or the digital clamp. If its a digital clamp, then
 * assures to put a valid DC substraction level.
 *
 * Returns always 0 when completed.
 **/
int ispccdc_config_black_clamp(struct isp_ccdc_device *isp_ccdc,
			       struct ispccdc_bclamp bclamp)
{
	u32 bclamp_val = 0;

	if (isp_ccdc->obclamp_en) {
		bclamp_val |= bclamp.obgain << ISPCCDC_CLAMP_OBGAIN_SHIFT;
		bclamp_val |= bclamp.oblen << ISPCCDC_CLAMP_OBSLEN_SHIFT;
		bclamp_val |= bclamp.oblines << ISPCCDC_CLAMP_OBSLN_SHIFT;
		bclamp_val |= bclamp.obstpixel << ISPCCDC_CLAMP_OBST_SHIFT;
		isp_reg_writel(isp_ccdc->dev, bclamp_val,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP);
	} else {
		if (omap_rev() < OMAP3430_REV_ES2_0)
			if (isp_ccdc->syncif_ipmod == YUV16 ||
			    isp_ccdc->syncif_ipmod == YUV8 ||
			    isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
					  ISPCCDC_REC656IF) &
			    ISPCCDC_REC656IF_R656ON)
				bclamp.dcsubval = 0;
		isp_reg_writel(isp_ccdc->dev, bclamp.dcsubval,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_DCSUB);
	}
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_black_clamp);

/**
 * ispccdc_enable_black_clamp - Enables/Disables the optical black clamp.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables optical black clamp, 1 Enables optical black clamp.
 *
 * Enables or disables the optical black clamp. When disabled, the digital
 * clamp operates.
 **/
void ispccdc_enable_black_clamp(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP,
		       ~ISPCCDC_CLAMP_CLAMPEN,
		       enable ? ISPCCDC_CLAMP_CLAMPEN : 0);
	isp_ccdc->obclamp_en = enable;
}
EXPORT_SYMBOL(ispccdc_enable_black_clamp);

/**
 * ispccdc_config_fpc - Configures the Faulty Pixel Correction parameters.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @fpc: Structure containing the number of faulty pixels corrected in the
 *       frame, address of the FPC table.
 *
 * Returns 0 if successful, or -EINVAL if FPC Address is not on the 64 byte
 * boundary.
 **/
int ispccdc_config_fpc(struct isp_ccdc_device *isp_ccdc, struct ispccdc_fpc fpc)
{
	u32 fpc_val = 0;

	fpc_val = isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				ISPCCDC_FPC);

	if ((fpc.fpcaddr & 0xFFFFFFC0) == fpc.fpcaddr) {
		isp_reg_writel(isp_ccdc->dev, fpc_val & (~ISPCCDC_FPC_FPCEN),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);
		isp_reg_writel(isp_ccdc->dev, fpc.fpcaddr,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC_ADDR);
	} else {
		DPRINTK_ISPCCDC("FPC Address should be on 64byte boundary\n");
		return -EINVAL;
	}
	isp_reg_writel(isp_ccdc->dev, fpc_val |
		       (fpc.fpnum << ISPCCDC_FPC_FPNUM_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_fpc);

/**
 * ispccdc_enable_fpc - Enable Faulty Pixel Correction.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables FPC, 1 Enables FPC.
 **/
void ispccdc_enable_fpc(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC,
		       ~ISPCCDC_FPC_FPCEN,
		       enable ? ISPCCDC_FPC_FPCEN : 0);
}
EXPORT_SYMBOL(ispccdc_enable_fpc);

/**
 * ispccdc_config_black_comp - Configure Black Level Compensation.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @blcomp: Structure containing the black level compensation value for RGrGbB
 *          pixels. in 2's complement.
 **/
void ispccdc_config_black_comp(struct isp_ccdc_device *isp_ccdc,
			       struct ispccdc_blcomp blcomp)
{
	u32 blcomp_val = 0;

	blcomp_val |= blcomp.b_mg << ISPCCDC_BLKCMP_B_MG_SHIFT;
	blcomp_val |= blcomp.gb_g << ISPCCDC_BLKCMP_GB_G_SHIFT;
	blcomp_val |= blcomp.gr_cy << ISPCCDC_BLKCMP_GR_CY_SHIFT;
	blcomp_val |= blcomp.r_ye << ISPCCDC_BLKCMP_R_YE_SHIFT;

	isp_reg_writel(isp_ccdc->dev, blcomp_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_BLKCMP);
}
EXPORT_SYMBOL(ispccdc_config_black_comp);

/**
 * ispccdc_config_vp - Configure the Video Port.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @vpcfg: Structure containing the Video Port input frequency, and the 10 bit
 *         format.
 **/
void ispccdc_config_vp(struct isp_ccdc_device *isp_ccdc,
		       struct ispccdc_vp vpcfg)
{
	u32 fmtcfg_vp = isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_FMTCFG);

	fmtcfg_vp &= ISPCCDC_FMTCFG_VPIN_MASK & ISPCCDC_FMTCFG_VPIF_FRQ_MASK;

	switch (vpcfg.bitshift_sel) {
	case BIT9_0:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_9_0;
		break;
	case BIT10_1:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_10_1;
		break;
	case BIT11_2:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_11_2;
		break;
	case BIT12_3:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_12_3;
		break;
	};
	switch (vpcfg.freq_sel) {
	case PIXCLKBY2:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIF_FRQ_BY2;
		break;
	case PIXCLKBY3_5:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIF_FRQ_BY3;
		break;
	case PIXCLKBY4_5:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIF_FRQ_BY4;
		break;
	case PIXCLKBY5_5:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIF_FRQ_BY5;
		break;
	case PIXCLKBY6_5:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIF_FRQ_BY6;
		break;
	};
	isp_reg_writel(isp_ccdc->dev, fmtcfg_vp, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_FMTCFG);
}
EXPORT_SYMBOL(ispccdc_config_vp);

/**
 * ispccdc_enable_vp - Enable Video Port.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables VP, 1 Enables VP
 *
 * This is needed for outputting image to Preview, H3A and HIST ISP submodules.
 **/
void ispccdc_enable_vp(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG,
		       ~ISPCCDC_FMTCFG_VPEN,
		       enable ? ISPCCDC_FMTCFG_VPEN : 0);
}
EXPORT_SYMBOL(ispccdc_enable_vp);

/**
 * ispccdc_config_reformatter - Configure Data Reformatter.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @refmt: Structure containing the memory address to format and the bit fields
 *         for the reformatter registers.
 *
 * Configures the Reformatter register values if line alternating is disabled.
 * Else, just enabling line alternating is enough.
 **/
void ispccdc_config_reformatter(struct isp_ccdc_device *isp_ccdc,
				struct ispccdc_refmt refmt)
{
	u32 fmtcfg_val = 0;

	fmtcfg_val = isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				   ISPCCDC_FMTCFG);

	if (refmt.lnalt)
		fmtcfg_val |= ISPCCDC_FMTCFG_LNALT;
	else {
		fmtcfg_val &= ~ISPCCDC_FMTCFG_LNALT;
		fmtcfg_val &= 0xFFFFF003;
		fmtcfg_val |= refmt.lnum << ISPCCDC_FMTCFG_LNUM_SHIFT;
		fmtcfg_val |= refmt.plen_even <<
			ISPCCDC_FMTCFG_PLEN_EVEN_SHIFT;
		fmtcfg_val |= refmt.plen_odd << ISPCCDC_FMTCFG_PLEN_ODD_SHIFT;

		isp_reg_writel(isp_ccdc->dev, refmt.prgeven0,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN0);
		isp_reg_writel(isp_ccdc->dev, refmt.prgeven1,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN1);
		isp_reg_writel(isp_ccdc->dev, refmt.prgodd0,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD0);
		isp_reg_writel(isp_ccdc->dev, refmt.prgodd1,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD1);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr0,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR0);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr1,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR1);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr2,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR2);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr3,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR3);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr4,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR4);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr5,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR5);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr6,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR6);
		isp_reg_writel(isp_ccdc->dev, refmt.fmtaddr7,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR7);
	}
	isp_reg_writel(isp_ccdc->dev, fmtcfg_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_FMTCFG);
}
EXPORT_SYMBOL(ispccdc_config_reformatter);

/**
 * ispccdc_enable_reformatter - Enable Data Reformatter.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables Reformatter, 1- Enables Data Reformatter
 **/
void ispccdc_enable_reformatter(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG,
		       ~ISPCCDC_FMTCFG_FMTEN,
		       enable ? ISPCCDC_FMTCFG_FMTEN : 0);
	isp_ccdc->refmt_en = enable;
}
EXPORT_SYMBOL(ispccdc_enable_reformatter);

/**
 * ispccdc_config_culling - Configure culling parameters.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @cull: Structure containing the vertical culling pattern, and horizontal
 *        culling pattern for odd and even lines.
 **/
void ispccdc_config_culling(struct isp_ccdc_device *isp_ccdc,
			    struct ispccdc_culling cull)
{
	u32 culling_val = 0;

	culling_val |= cull.v_pattern << ISPCCDC_CULLING_CULV_SHIFT;
	culling_val |= cull.h_even << ISPCCDC_CULLING_CULHEVN_SHIFT;
	culling_val |= cull.h_odd << ISPCCDC_CULLING_CULHODD_SHIFT;

	isp_reg_writel(isp_ccdc->dev, culling_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_CULLING);
}
EXPORT_SYMBOL(ispccdc_config_culling);

/**
 * ispccdc_enable_lpf - Enable Low-Pass Filter (LPF).
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LPF, 1 Enables LPF
 **/
void ispccdc_enable_lpf(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE,
		       ~ISPCCDC_SYN_MODE_LPF,
		       enable ? ISPCCDC_SYN_MODE_LPF : 0);
}
EXPORT_SYMBOL(ispccdc_enable_lpf);

/**
 * ispccdc_config_alaw - Configure the input width for A-law compression.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @ipwidth: Input width for A-law
 **/
void ispccdc_config_alaw(struct isp_ccdc_device *isp_ccdc,
			 enum alaw_ipwidth ipwidth)
{
	isp_reg_writel(isp_ccdc->dev, ipwidth << ISPCCDC_ALAW_GWDI_SHIFT,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW);
}
EXPORT_SYMBOL(ispccdc_config_alaw);

/**
 * ispccdc_enable_alaw - Enable A-law compression.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 - Disables A-law, 1 - Enables A-law
 **/
void ispccdc_enable_alaw(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW,
		       ~ISPCCDC_ALAW_CCDTBL,
		       enable ? ISPCCDC_ALAW_CCDTBL : 0);
}
EXPORT_SYMBOL(ispccdc_enable_alaw);

/**
 * ispccdc_config_imgattr - Configure sensor image specific attributes.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @colptn: Color pattern of the sensor.
 **/
void ispccdc_config_imgattr(struct isp_ccdc_device *isp_ccdc, u32 colptn)
{
	isp_reg_writel(isp_ccdc->dev, colptn, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_COLPTN);
}
EXPORT_SYMBOL(ispccdc_config_imgattr);

/**
 * ispccdc_config_shadow_registers - Configure CCDC during interframe time.
 * @isp_ccdc: Pointer to ISP CCDC device.
 *
 * Executes LSC deferred enablement before next frame starts.
 **/
void ispccdc_config_shadow_registers(struct isp_ccdc_device *isp_ccdc)
{
	if (isp_ccdc->lsc_enable) {
		ispccdc_enable_lsc(isp_ccdc, 1);
		isp_ccdc->lsc_enable = 0;
	}
}

/**
 * ispccdc_try_pipeline - Checks if requested Input/output dimensions are valid
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @pipe: Pointer to ISP pipeline structure to fill back.
 *
 * Calculates the number of pixels cropped if the reformater is disabled,
 * Fills up the output width and height variables in the isp_ccdc structure.
 *
 * Returns 0 if successful, or -EINVAL if the input width is less than 2 pixels
 **/
int ispccdc_try_pipeline(struct isp_ccdc_device *isp_ccdc,
			 struct isp_pipeline *pipe)
{
	struct isp_device *isp =
		container_of(isp_ccdc, struct isp_device, isp_ccdc);

	if (pipe->ccdc_in_w < 32 || pipe->ccdc_in_h < 32) {
		DPRINTK_ISPCCDC("ISP_ERR: CCDC cannot handle input width less"
				" than 32 pixels or height less than 32\n");
		return -EINVAL;
	}

	/* CCDC does not convert the image format */
	if ((pipe->ccdc_in == CCDC_RAW_GRBG ||
	     pipe->ccdc_in == CCDC_RAW_RGGB ||
	     pipe->ccdc_in == CCDC_RAW_BGGR ||
	     pipe->ccdc_in == CCDC_RAW_GBRG ||
	     pipe->ccdc_in == CCDC_OTHERS) &&
	    pipe->ccdc_out == CCDC_YUV_RSZ) {
		dev_info(isp->dev, "wrong CCDC I/O Combination\n");
		return -EINVAL;
	}

	pipe->ccdc_in_h_st = 0;
	pipe->ccdc_in_v_st = 0;
	pipe->ccdc_out_w = pipe->ccdc_in_w;
	pipe->ccdc_out_h = pipe->ccdc_in_h;

	if (!isp_ccdc->refmt_en
	    && pipe->ccdc_out != CCDC_OTHERS_MEM
	    && pipe->ccdc_out != CCDC_OTHERS_VP_MEM)
		pipe->ccdc_out_h -= 1;

	if (pipe->ccdc_out == CCDC_OTHERS_VP) {
		switch (pipe->ccdc_in) {
		case CCDC_RAW_GRBG:
			pipe->ccdc_in_h_st = 1;
			pipe->ccdc_in_v_st = 0;
			break;
		case CCDC_RAW_BGGR:
			pipe->ccdc_in_h_st = 1;
			pipe->ccdc_in_v_st = 1;
			break;
		case CCDC_RAW_RGGB:
			pipe->ccdc_in_h_st = 0;
			pipe->ccdc_in_v_st = 0;
			break;
		case CCDC_RAW_GBRG:
			pipe->ccdc_in_h_st = 0;
			pipe->ccdc_in_v_st = 1;
			break;
		default:
			break;
		}
		pipe->ccdc_out_h -= pipe->ccdc_in_v_st;
		pipe->ccdc_out_w -= pipe->ccdc_in_h_st;
		pipe->ccdc_out_h -= (pipe->ccdc_out_h % 2);
		pipe->ccdc_out_w -= (pipe->ccdc_out_w % 2);
	}

	pipe->ccdc_out_w_img = pipe->ccdc_out_w;
	/* Round up to nearest 16 pixels. */
	pipe->ccdc_out_w = ALIGN(pipe->ccdc_out_w, 0x10);

	return 0;
}
EXPORT_SYMBOL(ispccdc_try_pipeline);

/**
 * ispccdc_s_pipeline - Configure the CCDC based on overall ISP pipeline.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @pipe: Pointer to ISP pipeline structure to configure.
 *
 * Configures the appropriate values stored in the isp_ccdc structure to
 * HORZ/VERT_INFO registers and the VP_OUT depending on whether the image
 * is stored in memory or given to the another module in the ISP pipeline.
 *
 * Returns 0 if successful, or -EINVAL if try_size was not called before to
 * validate the requested dimensions.
 **/
int ispccdc_s_pipeline(struct isp_ccdc_device *isp_ccdc,
		       struct isp_pipeline *pipe)
{
	int rval;

	rval = ispccdc_config_datapath(isp_ccdc, pipe);
	if (rval)
		return rval;

	if (pipe->ccdc_out == CCDC_OTHERS_VP) {
		isp_reg_writel(isp_ccdc->dev, (pipe->ccdc_in_h_st <<
				ISPCCDC_FMT_HORZ_FMTSPH_SHIFT) |
			       ((pipe->ccdc_in_w - pipe->ccdc_in_h_st) <<
				ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_FMT_HORZ);
		isp_reg_writel(isp_ccdc->dev, (pipe->ccdc_in_v_st <<
				ISPCCDC_FMT_VERT_FMTSLV_SHIFT) |
			       ((pipe->ccdc_in_h - pipe->ccdc_in_v_st) <<
				ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_FMT_VERT);
		isp_reg_writel(isp_ccdc->dev, (pipe->ccdc_out_w <<
				ISPCCDC_VP_OUT_HORZ_NUM_SHIFT) |
			       (pipe->ccdc_out_h - 1) <<
			       ISPCCDC_VP_OUT_VERT_NUM_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VP_OUT);
		isp_reg_writel(isp_ccdc->dev, (((pipe->ccdc_out_h - 25) &
				 ISPCCDC_VDINT_0_MASK) <<
				ISPCCDC_VDINT_0_SHIFT) |
			       ((50 & ISPCCDC_VDINT_1_MASK) <<
				ISPCCDC_VDINT_1_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VDINT);

	} else if (pipe->ccdc_out == CCDC_OTHERS_MEM) {
		isp_reg_writel(isp_ccdc->dev, 0, OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VP_OUT);
		if (pipe->ccdc_in == CCDC_RAW_GRBG ||
		    pipe->ccdc_in == CCDC_RAW_RGGB ||
		    pipe->ccdc_in == CCDC_RAW_BGGR ||
		    pipe->ccdc_in == CCDC_RAW_GBRG) {
			isp_reg_writel(isp_ccdc->dev,
				       pipe->ccdc_in_h_st << ISPCCDC_HORZ_INFO_SPH_SHIFT
				       | ((pipe->ccdc_out_w - 1)
					  << ISPCCDC_HORZ_INFO_NPH_SHIFT),
				       OMAP3_ISP_IOMEM_CCDC,
				       ISPCCDC_HORZ_INFO);
		} else if (pipe->ccdc_in == CCDC_YUV_BT) {
			isp_reg_writel(isp_ccdc->dev,
					0 << ISPCCDC_HORZ_INFO_SPH_SHIFT |
					(((pipe->ccdc_out_w << 1) - 1) <<
					 ISPCCDC_HORZ_INFO_NPH_SHIFT),
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_HORZ_INFO);
			isp_reg_writel(isp_ccdc->dev,
					2 << ISPCCDC_VERT_START_SLV0_SHIFT |
					2 << ISPCCDC_VERT_START_SLV1_SHIFT,
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_VERT_START);
			isp_reg_writel(isp_ccdc->dev,
					((pipe->ccdc_out_h >> 1) - 1) <<
					ISPCCDC_VERT_LINES_NLV_SHIFT,
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_VERT_LINES);
		} else {
			isp_reg_writel(isp_ccdc->dev,
				       0 << ISPCCDC_HORZ_INFO_SPH_SHIFT
				       | ((pipe->ccdc_out_w - 1)
					  << ISPCCDC_HORZ_INFO_NPH_SHIFT),
				       OMAP3_ISP_IOMEM_CCDC,
				       ISPCCDC_HORZ_INFO);
		}
		ispccdc_config_outlineoffset(isp_ccdc, pipe->ccdc_out_w * 2,
					     0, 0);
		if (pipe->ccdc_in != CCDC_YUV_BT) {
			isp_reg_writel(isp_ccdc->dev,
					0 << ISPCCDC_VERT_START_SLV0_SHIFT,
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_VERT_START);
			isp_reg_writel(isp_ccdc->dev, (pipe->ccdc_out_h - 1) <<
					ISPCCDC_VERT_LINES_NLV_SHIFT,
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_VERT_LINES);
			isp_reg_writel(isp_ccdc->dev, (((pipe->ccdc_out_h - 2) &
					ISPCCDC_VDINT_0_MASK) <<
					ISPCCDC_VDINT_0_SHIFT) |
					((100 & ISPCCDC_VDINT_1_MASK) <<
					 ISPCCDC_VDINT_1_SHIFT),
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_VDINT);
		} else {
			ispccdc_config_outlineoffset(isp_ccdc,
					pipe->ccdc_out_w * 2, EVENEVEN, 1);
			ispccdc_config_outlineoffset(isp_ccdc,
					pipe->ccdc_out_w * 2, ODDEVEN, 1);
			ispccdc_config_outlineoffset(isp_ccdc,
					pipe->ccdc_out_w * 2, EVENODD, 1);
			ispccdc_config_outlineoffset(isp_ccdc,
					pipe->ccdc_out_w * 2, ODDODD, 1);
			isp_reg_writel(isp_ccdc->dev,
					((((pipe->ccdc_out_h >> 1) - 1) &
					ISPCCDC_VDINT_0_MASK) <<
					ISPCCDC_VDINT_0_SHIFT) |
					((50 & ISPCCDC_VDINT_1_MASK) <<
					 ISPCCDC_VDINT_1_SHIFT),
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_VDINT);
		}

	} else if (pipe->ccdc_out == CCDC_OTHERS_VP_MEM) {
		isp_reg_writel(isp_ccdc->dev,
			       (pipe->ccdc_in_h_st << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT) |
			       ((pipe->ccdc_in_w - pipe->ccdc_in_h_st) <<
				ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_FMT_HORZ);
		isp_reg_writel(isp_ccdc->dev,
			       (pipe->ccdc_in_v_st << ISPCCDC_FMT_VERT_FMTSLV_SHIFT) |
			       ((pipe->ccdc_in_h - pipe->ccdc_in_v_st) <<
				ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_FMT_VERT);
		isp_reg_writel(isp_ccdc->dev, (pipe->ccdc_out_w
				<< ISPCCDC_VP_OUT_HORZ_NUM_SHIFT) |
			       ((pipe->ccdc_out_h - 1) <<
				ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VP_OUT);
		isp_reg_writel(isp_ccdc->dev,
			       pipe->ccdc_in_h_st << ISPCCDC_HORZ_INFO_SPH_SHIFT |
			       ((pipe->ccdc_out_w - 1) <<
				ISPCCDC_HORZ_INFO_NPH_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_HORZ_INFO);
		isp_reg_writel(isp_ccdc->dev,
			       pipe->ccdc_in_v_st << ISPCCDC_VERT_START_SLV0_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VERT_START);
		isp_reg_writel(isp_ccdc->dev, (pipe->ccdc_out_h - 1) <<
			       ISPCCDC_VERT_LINES_NLV_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VERT_LINES);
		ispccdc_config_outlineoffset(isp_ccdc, pipe->ccdc_out_w * 2,
					     0, 0);
		isp_reg_writel(isp_ccdc->dev, (((pipe->ccdc_out_h - 2) &
				 ISPCCDC_VDINT_0_MASK) <<
				ISPCCDC_VDINT_0_SHIFT) |
			       ((100 & ISPCCDC_VDINT_1_MASK) <<
				ISPCCDC_VDINT_1_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_VDINT);
	}

	if (is_isplsc_activated()) {
		if (pipe->ccdc_in == CCDC_RAW_GRBG ||
		    pipe->ccdc_in == CCDC_RAW_RGGB ||
		    pipe->ccdc_in == CCDC_RAW_BGGR ||
		    pipe->ccdc_in == CCDC_RAW_GBRG) {
			ispccdc_config_lsc(isp_ccdc, &isp_ccdc->lsc_config);
			ispccdc_load_lsc(isp_ccdc, isp_ccdc->lsc_gain_table,
					 isp_ccdc->lsc_config.size);
		}
	}

	return 0;
}
EXPORT_SYMBOL(ispccdc_s_pipeline);

/**
 * ispccdc_config_outlineoffset - Configure memory saving output line offset
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @offset: Address offset to start a new line. Must be twice the
 *          Output width and aligned on 32 byte boundary
 * @oddeven: Specifies the odd/even line pattern to be chosen to store the
 *           output.
 * @numlines: Set the value 0-3 for +1-4lines, 4-7 for -1-4lines.
 *
 * - Configures the output line offset when stored in memory
 * - Sets the odd/even line pattern to store the output
 *    (EVENEVEN (1), ODDEVEN (2), EVENODD (3), ODDODD (4))
 * - Configures the number of even and odd line fields in case of rearranging
 * the lines.
 *
 * Returns 0 if successful, or -EINVAL if the offset is not in 32 byte
 * boundary.
 **/
int ispccdc_config_outlineoffset(struct isp_ccdc_device *isp_ccdc, u32 offset,
				 u8 oddeven, u8 numlines)
{
	if ((offset & ISP_32B_BOUNDARY_OFFSET) == offset) {
		isp_reg_writel(isp_ccdc->dev, (offset & 0xFFFF),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF);
	} else {
		DPRINTK_ISPCCDC("ISP_ERR : Offset should be in 32 byte"
				" boundary\n");
		return -EINVAL;
	}

	isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
		    ~ISPCCDC_SDOFST_FINV);

	isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
		    ~ISPCCDC_SDOFST_FOFST_4L);

	switch (oddeven) {
	case EVENEVEN:
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			   ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST0_SHIFT);
		break;
	case ODDEVEN:
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			   ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST1_SHIFT);
		break;
	case EVENODD:
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			   ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST2_SHIFT);
		break;
	case ODDODD:
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			   ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST3_SHIFT);
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_outlineoffset);

/**
 * ispccdc_set_outaddr - Set memory address to save output image
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @addr: ISP MMU Mapped 32-bit memory address aligned on 32 byte boundary.
 *
 * Returns 0 if successful, or -EINVAL if the address is not in the 32 byte
 * boundary.
 **/
int ispccdc_set_outaddr(struct isp_ccdc_device *isp_ccdc, u32 addr)
{
	if ((addr & ISP_32B_BOUNDARY_BUF) == addr) {
		isp_reg_writel(isp_ccdc->dev, addr, OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_SDR_ADDR);
		return 0;
	} else {
		DPRINTK_ISPCCDC("ISP_ERR : Address should be in 32 byte"
				" boundary\n");
		return -EINVAL;
	}

}
EXPORT_SYMBOL(ispccdc_set_outaddr);

/**
 * ispccdc_enable - Enable the CCDC module.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables CCDC, 1 Enables CCDC
 *
 * Client should configure all the sub modules in CCDC before this.
 **/
void ispccdc_enable(struct isp_ccdc_device *isp_ccdc, u8 enable)
{
	struct isp_device *isp =
		container_of(isp_ccdc, struct isp_device, isp_ccdc);

	if (enable) {
		if (isp_ccdc->lsc_enable &&
		    ((isp->pipeline.ccdc_in == CCDC_RAW_GRBG) ||
		     (isp->pipeline.ccdc_in == CCDC_RAW_RGGB) ||
		     (isp->pipeline.ccdc_in == CCDC_RAW_BGGR) ||
		     (isp->pipeline.ccdc_in == CCDC_RAW_GBRG)))
			ispccdc_enable_lsc(isp_ccdc, 1);

	} else {
		int lsc_enable = isp_ccdc->lsc_state;

		ispccdc_enable_lsc(isp_ccdc, 0);
		isp_ccdc->lsc_enable = lsc_enable;
	}

	isp_reg_and_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR,
		       ~ISPCCDC_PCR_EN, enable ? ISPCCDC_PCR_EN : 0);
}
EXPORT_SYMBOL(ispccdc_enable);

/**
 * ispccdc_sbl_busy - Poll idle state of CCDC and related SBL memory write bits
 * @_isp_ccdc: Pointer to ISP CCDC device.
 *
 * Returns zero if the CCDC is idle and the image has been written to
 * memory, too.
 **/
int ispccdc_sbl_busy(void *_isp_ccdc)
{
	struct isp_ccdc_device *isp_ccdc = _isp_ccdc;

	return ispccdc_busy(isp_ccdc)
		| (isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_SBL,
				 ISPSBL_CCDC_WR_0) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_SBL,
				 ISPSBL_CCDC_WR_1) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_SBL,
				 ISPSBL_CCDC_WR_2) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_SBL,
				 ISPSBL_CCDC_WR_3) &
		   ISPSBL_CCDC_WR_0_DATA_READY);
}
EXPORT_SYMBOL(ispccdc_sbl_busy);

/**
 * ispccdc_config_y8pos - Configures the location of Y color component
 * @mode: Y8POS_EVEN Y pixel in even position, otherwise Y pixel in odd
 *
 * Configures the location of Y color componenent for YCbCr 8-bit data
 */
void ispccdc_config_y8pos(struct isp_ccdc_device *isp_ccdc,
		enum y8pos_mode mode)
{
	if (mode == Y8POS_EVEN) {
		isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
							~ISPCCDC_CFG_Y8POS);
	} else {
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
							ISPCCDC_CFG_Y8POS);
	}
}
EXPORT_SYMBOL(ispccdc_config_y8pos);

/**
 * ispccdc_config_byteswap - Configures byte swap data stored in memory
 * @swap: 1 - swap bytes, 0 - normal
 *
 * Controls the order in which the Y and C pixels are stored in memory
 */
void ispccdc_config_byteswap(struct isp_ccdc_device *isp_ccdc, int swap)
{
	if (swap) {
		isp_reg_or(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
							ISPCCDC_CFG_BSWD);
	} else {
		isp_reg_and(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
							~ISPCCDC_CFG_BSWD);
	}
}
EXPORT_SYMBOL(ispccdc_config_byteswap);

/**
 * ispccdc_busy - Get busy state of the CCDC.
 * @isp_ccdc: Pointer to ISP CCDC device.
 **/
int ispccdc_busy(struct isp_ccdc_device *isp_ccdc)
{
	return isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
			     ISPCCDC_PCR) &
		ISPCCDC_PCR_BUSY;
}
EXPORT_SYMBOL(ispccdc_busy);

/**
 * ispccdc_save_context - Save values of the CCDC module registers
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void ispccdc_save_context(struct device *dev)
{
	DPRINTK_ISPCCDC("Saving context\n");
	isp_save_context(dev, ispccdc_reg_list);
}
EXPORT_SYMBOL(ispccdc_save_context);

/**
 * ispccdc_restore_context - Restore values of the CCDC module registers
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void ispccdc_restore_context(struct device *dev)
{
	DPRINTK_ISPCCDC("Restoring context\n");
	isp_restore_context(dev, ispccdc_reg_list);
}
EXPORT_SYMBOL(ispccdc_restore_context);

/**
 * ispccdc_print_status - Print current CCDC Module register values.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @pipe: Pointer to current ISP pipeline structure.
 *
 * Also prints other debug information stored in the CCDC module.
 **/
void ispccdc_print_status(struct isp_ccdc_device *isp_ccdc,
			  struct isp_pipeline *pipe)
{
	if (!is_ispccdc_debug_enabled())
		return;

	DPRINTK_ISPCCDC("Module in use =%d\n", isp_ccdc->ccdc_inuse);
	DPRINTK_ISPCCDC("Accepted CCDC Input (width = %d,Height = %d)\n",
			isp_ccdc->ccdcin_w,
			isp_ccdc->ccdcin_h);
	DPRINTK_ISPCCDC("Accepted CCDC Output (width = %d,Height = %d)\n",
			isp_ccdc->ccdcout_w,
			isp_ccdc->ccdcout_h);
	DPRINTK_ISPCCDC("###CCDC PCR=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_PCR));
	DPRINTK_ISPCCDC("ISP_CTRL =0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_CTRL));
	switch ((int)pipe->ccdc_in) {
	case CCDC_RAW_GRBG:
	case CCDC_RAW_RGGB:
	case CCDC_RAW_BGGR:
	case CCDC_RAW_GBRG:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_RAW\n");
		break;
	case CCDC_YUV_SYNC:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_YUV_SYNC\n");
		break;
	case CCDC_YUV_BT:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_YUV_BT\n");
		break;
	}

	switch ((int)pipe->ccdc_out) {
	case CCDC_OTHERS_VP:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_OTHERS_VP\n");
		break;
	case CCDC_OTHERS_MEM:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_OTHERS_MEM\n");
		break;
	case CCDC_YUV_RSZ:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_YUV_RSZ\n");
		break;
	}

	DPRINTK_ISPCCDC("###ISP_CTRL in ccdc =0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_CTRL));
	DPRINTK_ISPCCDC("###ISP_IRQ0ENABLE in ccdc =0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_IRQ0ENABLE));
	DPRINTK_ISPCCDC("###ISP_IRQ0STATUS in ccdc =0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_IRQ0STATUS));
	DPRINTK_ISPCCDC("###CCDC SYN_MODE=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_SYN_MODE));
	DPRINTK_ISPCCDC("###CCDC HORZ_INFO=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_HORZ_INFO));
	DPRINTK_ISPCCDC("###CCDC VERT_START=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_VERT_START));
	DPRINTK_ISPCCDC("###CCDC VERT_LINES=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_VERT_LINES));
	DPRINTK_ISPCCDC("###CCDC CULLING=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_CULLING));
	DPRINTK_ISPCCDC("###CCDC HSIZE_OFF=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_HSIZE_OFF));
	DPRINTK_ISPCCDC("###CCDC SDOFST=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_SDOFST));
	DPRINTK_ISPCCDC("###CCDC SDR_ADDR=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_SDR_ADDR));
	DPRINTK_ISPCCDC("###CCDC CLAMP=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_CLAMP));
	DPRINTK_ISPCCDC("###CCDC COLPTN=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_COLPTN));
	DPRINTK_ISPCCDC("###CCDC CFG=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_CFG));
	DPRINTK_ISPCCDC("###CCDC VP_OUT=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_VP_OUT));
	DPRINTK_ISPCCDC("###CCDC_SDR_ADDR= 0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_SDR_ADDR));
	DPRINTK_ISPCCDC("###CCDC FMTCFG=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_FMTCFG));
	DPRINTK_ISPCCDC("###CCDC FMT_HORZ=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_FMT_HORZ));
	DPRINTK_ISPCCDC("###CCDC FMT_VERT=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_FMT_VERT));
	DPRINTK_ISPCCDC("###CCDC LSC_CONFIG=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_LSC_CONFIG));
	DPRINTK_ISPCCDC("###CCDC LSC_INIT=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_LSC_INITIAL));
	DPRINTK_ISPCCDC("###CCDC LSC_TABLE BASE=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_LSC_TABLE_BASE));
	DPRINTK_ISPCCDC("###CCDC LSC TABLE OFFSET=0x%x\n",
			isp_reg_readl(isp_ccdc->dev, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_LSC_TABLE_OFFSET));
}
EXPORT_SYMBOL(ispccdc_print_status);

/**
 * isp_ccdc_init - CCDC module initialization.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Always returns 0
 **/
int __init isp_ccdc_init(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_ccdc_device *isp_ccdc = &isp->isp_ccdc;

	isp_ccdc->ccdc_inuse = 0;
	ispccdc_config_crop(isp_ccdc, 0, 0, 0, 0);
	mutex_init(&isp_ccdc->mutexlock);
	isp_ccdc->dev = dev;

	if (is_isplsc_activated()) {
		isp_ccdc->lsc_gain_table_tmp = kmalloc(LSC_TABLE_INIT_SIZE,
						       GFP_KERNEL | GFP_DMA);
		memset(isp_ccdc->lsc_gain_table_tmp, 0x40, LSC_TABLE_INIT_SIZE);
		isp_ccdc->lsc_config.initial_x = 0;
		isp_ccdc->lsc_config.initial_y = 0;
		isp_ccdc->lsc_config.gain_mode_n = 0x6;
		isp_ccdc->lsc_config.gain_mode_m = 0x6;
		isp_ccdc->lsc_config.gain_format = 0x4;
		isp_ccdc->lsc_config.offset = 0x60;
		isp_ccdc->lsc_config.size = LSC_TABLE_INIT_SIZE;
		isp_ccdc->lsc_enable = 1;
	}

	return 0;
}

/**
 * isp_ccdc_cleanup - CCDC module cleanup.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void isp_ccdc_cleanup(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_ccdc_device *isp_ccdc = &isp->isp_ccdc;

	if (is_isplsc_activated()) {
		ispccdc_free_lsc(isp_ccdc);
		kfree(isp_ccdc->lsc_gain_table_tmp);
	}

	if (isp_ccdc->fpc_table_add_m != 0) {
		iommu_kunmap(isp->iommu, isp_ccdc->fpc_table_add_m);
		kfree(isp_ccdc->fpc_table_add);
	}
}
