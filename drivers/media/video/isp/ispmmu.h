/*
 * omap iommu wrapper for TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Nokia.
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Thara Gopinath <thara@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_MMU_H
#define OMAP_ISP_MMU_H

#include <linux/err.h>
#include <linux/scatterlist.h>

#include <mach/iommu.h>
#include <mach/iovmm.h>

#define IOMMU_FLAG (IOVMF_ENDIAN_LITTLE | IOVMF_ELSZ_8)

extern struct iommu *isp_iommu;

static inline dma_addr_t ispmmu_vmalloc(size_t bytes)
{
	return (dma_addr_t)iommu_vmalloc(isp_iommu, NULL, bytes, IOMMU_FLAG);
}

static inline void ispmmu_vfree(const dma_addr_t da)
{
	iommu_vfree(isp_iommu, (void *)da);
}

static inline dma_addr_t ispmmu_kmap(u32 pa, int size)
{
	void *da;

	da = iommu_kmap(isp_iommu, NULL, pa, size, IOMMU_FLAG);
	if (IS_ERR(da))
		return PTR_ERR(da);

	return (dma_addr_t)da;
}

static inline void ispmmu_kunmap(dma_addr_t da)
{
	iommu_kunmap(isp_iommu, (void *)da);
}

static inline dma_addr_t ispmmu_vmap(const struct scatterlist *sglist,
				     int sglen)
{
	void *da;
	struct sg_table sgt;

	sgt.sgl = (struct scatterlist *)sglist;
	sgt.nents = sglen;

	da = iommu_vmap(isp_iommu, NULL, &sgt, IOMMU_FLAG);
	if (IS_ERR(da))
		return PTR_ERR(da);

	return (dma_addr_t)da;
}

static inline void ispmmu_vunmap(dma_addr_t da)
{
	iommu_vunmap(isp_iommu, (void *)da);
}

static inline void ispmmu_save_context(void)
{
	iommu_save_ctx(isp_iommu);
}

static inline void ispmmu_restore_context(void)
{
	iommu_restore_ctx(isp_iommu);
}

static inline int ispmmu_init(void)
{
	isp_get();
	isp_iommu = iommu_get("isp");
	isp_put();

	if (IS_ERR(isp_iommu))
		return PTR_ERR(isp_iommu);

	return 0;
}

static inline void ispmmu_cleanup(void)
{
	isp_get();
	iommu_put(isp_iommu);
	isp_put();
}

#endif /* OMAP_ISP_MMU_H */
