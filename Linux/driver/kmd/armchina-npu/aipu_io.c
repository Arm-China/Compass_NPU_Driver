// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#include <linux/ioport.h>
#include "aipu_io.h"

/**
 * @init_aipu_ioregion() - initialize struct io_region using physical base address
 * @region:    pointer to struct io_region to be initialized
 * @phys_base: base address of this region
 * @size:      size of this region
 *
 * Return: 0 on success and error code otherwise.
 */
int init_aipu_ioregion(struct io_region *region, u64 phys_base, u32 size)
{
	int ret = 0;

	if (!region || !size)
		return -EINVAL;

	if (!request_mem_region(phys_base, size, "aipu"))
		return -ENOMEM;

	region->kern = ioremap(phys_base, size);
	if (!region->kern) {
		release_mem_region(phys_base, size);
		return -ENOMEM;
	}

	region->phys = phys_base;
	region->size = size;
	return ret;
}

/**
 * @deinit_aipu_ioregion() - destroy an AIPU IO region
 * @region: pointer to struct io_region initialized in init_aipu_ioregion()
 */
void deinit_aipu_ioregion(struct io_region *region)
{
	if (region && region->kern) {
		iounmap(region->kern);
		release_mem_region(region->phys, region->size);
		region->kern = NULL;
		region->phys = 0;
		region->size = 0;
	}
}

/**
 * @aipu_read32() - read AIPU register in word
 * @region: pointer to struct io_region initialized in init_aipu_ioregion()
 * @offset: AIPU register offset
 * Return: u32 value in the register or error code
 */
int aipu_read32(struct io_region *region, int offset)
{
	u32 val = 0;

	if (region && region->kern && offset < region->size) {
		val = readl((void __iomem *)((unsigned long)(region->kern) + offset));
		pr_debug("[Read AIPU Register]: reg addr: 0x%llx + 0x%x, read back value: 0x%x\n",
			 region->phys, offset, val);
		return val;
	}
	return -EINVAL;
}

/**
 * @aipu_write32() - write AIPU register in word
 * @region: pointer to struct io_region initialized in init_aipu_ioregion()
 * @offset: AIPU register offset
 * @data:   u32 data to be writen
 */
void aipu_write32(struct io_region *region, int offset, unsigned int data)
{
	if (region && region->kern && offset < region->size)
		writel((u32)data, (void __iomem *)((unsigned long)(region->kern) + offset));
	pr_debug("[Write AIPU Register]: reg addr: 0x%llx + 0x%x, write value: 0x%x\n",
		 region->phys, offset, (u32)data);
}
