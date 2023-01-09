// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include "zhouyi.h"

int zhouyi_read_status_reg(struct io_region *io)
{
	return aipu_read32(io, ZHOUYI_STAT_REG_OFFSET);
}

void zhouyi_clear_qempty_interrupt(struct io_region *io)
{
	aipu_write32(io, ZHOUYI_STAT_REG_OFFSET, ZHOUYI_IRQ_QEMPTY);
}

void zhouyi_clear_done_interrupt(struct io_region *io)
{
	aipu_write32(io, ZHOUYI_STAT_REG_OFFSET, ZHOUYI_IRQ_DONE);
}

void zhouyi_clear_excep_interrupt(struct io_region *io)
{
	aipu_write32(io, ZHOUYI_STAT_REG_OFFSET, ZHOUYI_IRQ_EXCEP);
}

void zhouyi_io_rw(struct io_region *io, struct aipu_io_req *io_req)
{
	if (unlikely(!io || !io_req))
		return;

	if (io_req->rw == AIPU_IO_READ)
		io_req->value = aipu_read32(io, io_req->offset);
	else if (io_req->rw == AIPU_IO_WRITE)
		aipu_write32(io, io_req->offset, io_req->value);
}

int zhouyi_detect_aipu_version(struct platform_device *p_dev, int *version, int *config)
{
	struct resource *res = NULL;
	struct io_region io;

	if (!p_dev || !version || !config)
		return -EINVAL;

	res = platform_get_resource(p_dev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	if (init_aipu_ioregion(&io, res->start, res->end - res->start + 1))
		return -EINVAL;

	*version = zhouyi_get_hw_version_number(&io);
	if (*version > 0 && *version < ZHOUYI_X2_REVISION_ID)
		*config = zhouyi_get_hw_config_number(&io);
	else
		*config = 0;

	deinit_aipu_ioregion(&io);
	return 0;
}

#ifdef CONFIG_SYSFS
int zhouyi_print_reg_info(struct io_region *io, char *buf, const char *name, int offset)
{
	if (unlikely(!io || !buf || !name))
		return -EINVAL;

	return snprintf(buf, 1024, "0x%-*x%-*s0x%08x\n", 6, offset, 28, name,
	    aipu_read32(io, offset));
}
#endif

#ifdef CONFIG_SYSFS
int zhouyi_sysfs_show(struct io_region *io, char *buf)
{
	int ret = 0;
	char tmp[512];

	if (unlikely(!io || !buf))
		return -EINVAL;

	ret += zhouyi_print_reg_info(io, tmp, "Ctrl Reg", ZHOUYI_CTRL_REG_OFFSET);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(io, tmp, "Status Reg", ZHOUYI_STAT_REG_OFFSET);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(io, tmp, "Start PC Reg", ZHOUYI_START_PC_REG_OFFSET);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(io, tmp, "Intr PC Reg", ZHOUYI_INTR_PC_REG_OFFSET);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(io, tmp, "IPI Ctrl Reg", ZHOUYI_IPI_CTRL_REG_OFFSET);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(io, tmp, "Data Addr 0 Reg", ZHOUYI_DATA_ADDR_0_REG_OFFSET);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(io, tmp, "Data Addr 1 Reg", ZHOUYI_DATA_ADDR_1_REG_OFFSET);
	strcat(buf, tmp);
	return ret;
}
#endif

int zhouyi_get_hw_version_number(struct io_region *io)
{
	int revision_id = 0;

	if (!io)
		return 0;

	revision_id = aipu_read32(io, ZHOUYI_REVISION_ID_REG_OFFSET);

#if (defined BUILD_ZHOUYI_Z1 || defined BUILD_ZHOUYI_ALL)
	if (revision_id == ZHOUYI_Z1_REVISION_ID)
		return AIPU_ISA_VERSION_ZHOUYI_Z1;
#endif

#if (defined BUILD_ZHOUYI_Z2 || defined BUILD_ZHOUYI_Z3 || defined BUILD_ZHOUYI_X1 || defined BUILD_ZHOUYI_ALL)
	if (revision_id == ZHOUYI_Z2_REVISION_ID)
		return AIPU_ISA_VERSION_ZHOUYI_Z2;
	else if (revision_id == ZHOUYI_Z3_REVISION_ID)
		return AIPU_ISA_VERSION_ZHOUYI_Z3;
	else if (revision_id == ZHOUYI_X1_REVISION_ID)
		return AIPU_ISA_VERSION_ZHOUYI_X1;
#endif

#if (defined BUILD_ZHOUYI_X2 || defined BUILD_ZHOUYI_ALL)
	if (revision_id == ZHOUYI_X2_REVISION_ID)
		return AIPU_ISA_VERSION_ZHOUYI_X2;
#endif

	return 0;
}

int zhouyi_get_hw_config_number(struct io_region *io)
{
	int high = 0;
	int low = 0;
	int isa_version = 0;
	int aiff_feature = 0;
	int tpc_feature = 0;

	if (!io)
		return 0;

	isa_version = aipu_read32(io, ZHOUYI_ISA_VERSION_REG_OFFSET);
	aiff_feature = aipu_read32(io, ZHOUYI_HWA_FEATURE_REG_OFFSET);
	tpc_feature = aipu_read32(io, ZHOUYI_TPC_FEATURE_REG_OFFSET);

	if (isa_version == 0)
		high = (aiff_feature & 0xF) + 6;
	else if (isa_version == 1)
		high = (aiff_feature & 0xF) + 8;

	low = (tpc_feature) & 0x1F;

	return high * 100 + low;
}

int zhouyi_soft_reset(struct io_region *io, int offset, int delay_us)
{
	int ret = 0;

	aipu_write32(io, offset, ZHOUYI_LAUNCH_SOFT_RESET);
	udelay(delay_us);
	if (aipu_read32(io, offset) != ZHOUYI_SOFT_RESET_DONE)
		ret = -EIO;

	return ret;
}
