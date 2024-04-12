/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_IO_H__
#define __AIPU_IO_H__

#include <linux/io.h>
#include <asm/types.h>

/**
 * struct io_region - a general struct describe IO region
 * @phys: physical address base of an IO region
 * @kern: kernel virtual address base remapped from phys
 * @size: size of an IO region in byte
 */
struct io_region {
	u64  phys;
	void *kern;
	u32  size;
};

int init_aipu_ioregion(struct io_region *region, u64 phys_base, u32 size);
void deinit_aipu_ioregion(struct io_region *region);
int aipu_read32(struct io_region *region, int offset);
void aipu_write32(struct io_region *region, int offset, unsigned int data);

#endif /* __AIPU_IO_H__ */
