/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_ZHOUYI_H__
#define __AIPU_ZHOUYI_H__

#include <linux/platform_device.h>
#include <linux/device.h>
#include <armchina_aipu.h>
#include "aipu_io.h"
#include "config.h"

/*
 * Zhouyi AIPU Common Interrupts
 */
#define ZHOUYI_IRQ_NONE                       0x0
#define ZHOUYI_IRQ_QEMPTY                     0x1
#define ZHOUYI_IRQ_DONE                       0x2
#define ZHOUYI_IRQ_EXCEP                      0x4

#define ZHOUYI_IRQ  (ZHOUYI_IRQ_QEMPTY | ZHOUYI_IRQ_DONE | ZHOUYI_IRQ_EXCEP)

#define ZHOUYI_AIPU_IDLE_STATUS               0x70000

#define ZHOUYI_ASID_COUNT                     32

/*
 * Revision ID for V1 ~ V3_1
 */
#define ZHOUYI_V1_REVISION_ID                 0x0
#define ZHOUYI_V2_0_REVISION_ID               0x100
#define ZHOUYI_V2_1_REVISION_ID               0x200
#define ZHOUYI_V2_2_REVISION_ID               0x300
#define ZHOUYI_V3_REVISION_ID_R0P2            0x10000
#define ZHOUYI_V3_REVISION_ID_R0P3            0x10003
#define ZHOUYI_V3_1_REVISION_ID_R0P0          0x10100

/*
 * Soft Reset
 */
#define ZHOUYI_LAUNCH_SOFT_RESET              BIT(0)
#define ZHOUYI_SOFT_RESET_DONE                BIT(1)

/*
 * QoS
 */
#define TSM_QOS_SLOW                          0
#define TSM_QOS_FAST                          (2 << 8)

/*
 * Command Pool
 */
#define ZHOUYI_COMMAND_POOL_DEFAULT           0
#define ZHOUYI_COMMAND_POOL_PCP               0
#define ZHOUYI_V3_1_COMMAND_POOL_FULL         0xFFFF

#define PARTITION_MODE_NONE                   0

/*
 * Zhouyi AIPU Common Host Control Register Map
 */
#define ZHOUYI_CTRL_REG_OFFSET                0x0
#define ZHOUYI_STAT_REG_OFFSET                0x4
#define ZHOUYI_START_PC_REG_OFFSET            0x8
#define ZHOUYI_INTR_PC_REG_OFFSET             0xC
#define ZHOUYI_IPI_CTRL_REG_OFFSET            0x10
#define ZHOUYI_DATA_ADDR_0_REG_OFFSET         0x14
#define ZHOUYI_DATA_ADDR_1_REG_OFFSET         0x18
#define ZHOUYI_CLK_CTRL_REG_OFFSET            0x3C
#define ZHOUYI_ISA_VERSION_REG_OFFSET         0x40
#define ZHOUYI_TPC_FEATURE_REG_OFFSET         0x44
#define ZHOUYI_SPU_FEATURE_REG_OFFSET         0x48
#define ZHOUYI_HWA_FEATURE_REG_OFFSET         0x4C
#define ZHOUYI_REVISION_ID_REG_OFFSET         0x50
#define ZHOUYI_MEM_FEATURE_REG_OFFSET         0x54
#define ZHOUYI_INST_RAM_FEATURE_REG_OFFSET    0x58
#define ZHOUYI_LOCAL_SRAM_FEATURE_REG_OFFSET  0x5C
#define ZHOUYI_GLOBAL_SRAM_FEATURE_REG_OFFSET 0x60
#define ZHOUYI_INST_CACHE_FEATURE_REG_OFFSET  0x64
#define ZHOUYI_DATA_CACHE_FEATURE_REG_OFFSET  0x68

#define ZHOUYI_TRIGGER_TYPE_CREATE               0
#define ZHOUYI_TRIGGER_TYPE_UPDATE_DISPATCH      1
#define ZHOUYI_TRIGGER_TYPE_DISPATCH             2
#define ZHOUYI_TRIGGER_TYPE_DEBUG_DISPATCH       3



int zhouyi_read_status_reg(struct io_region *io);
void zhouyi_clear_qempty_interrupt(struct io_region *io);
void zhouyi_clear_done_interrupt(struct io_region *io);
void zhouyi_clear_excep_interrupt(struct io_region *io);
void zhouyi_io_rw(struct io_region *io, struct aipu_io_req *io_req);
int zhouyi_detect_aipu_version(struct platform_device *p_dev, int *version, int *config, int *rev);
#ifdef CONFIG_SYSFS
int zhouyi_print_reg_info(struct io_region *io, char *buf, const char *name, int offset);
int zhouyi_sysfs_show(struct io_region *io, char *buf);
#endif
int zhouyi_get_hw_version_number(struct io_region *io, int *rev);
int zhouyi_get_hw_config_number(struct io_region *io);
int zhouyi_soft_reset(struct io_region *io, int offset, int delay_us);
int get_qos(u32 exec_flag);

#endif /* __AIPU_ZHOUYI_H__ */
