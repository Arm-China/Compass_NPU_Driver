/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __X1_H__
#define __X1_H__

#include "zhouyi.h"

/*
 * Zhouyi z2/x1 AIPU Specific Interrupts/Configs
 */
#define ZHOUYI_IRQ_FAULT                    0x8

#define ZHOUYIV2_IRQ                        (ZHOUYI_IRQ | ZHOUYI_IRQ_FAULT)
#define ZHOUYIV2_IRQ_ENABLE_FLAG            (ZHOUYIV2_IRQ)
#define ZHOUYIV2_IRQ_DISABLE_FLAG           (ZHOUYI_IRQ_NONE)

#define ZHOUYI_X1_MAX_SCHED_JOB_NUM         1

#define ZHOUYI_X1_ASE_READ_ENABLE           BIT(31)
#define ZHOUYI_X1_ASE_WRITE_ENABLE          BIT(30)
#define ZHOUYI_X1_ASE_RW_ENABLE             (ZHOUYI_X1_ASE_READ_ENABLE | ZHOUYI_X1_ASE_WRITE_ENABLE)
#define ZHOUYI_X1_DTCM_ENABLE               BIT(16)
#define ZHOUYI_X1_DTCM_MAX_BYTES            (32 * SZ_1M) /* 32MB at maximum */

/*
 * Zhouyi z2/x1 AIPU Specific Host Control Register Map
 */
#define AIPU_DATA_ADDR_2_REG_OFFSET         0x1C
#define AIPU_DATA_ADDR_3_REG_OFFSET         0x20
#define AIPU_SECURE_CONFIG_REG_OFFSET       0x30
#define AIPU_POWER_CTRL_REG_OFFSET          0x38
#define AIPU_ADDR_EXT0_CTRL_REG_OFFSET      0xC0
#define AIPU_ADDR_EXT0_HIGH_BASE_REG_OFFSET 0xC4
#define AIPU_ADDR_EXT0_LOW_BASE_REG_OFFSET  0xC8
#define AIPU_ADDR_EXT1_CTRL_REG_OFFSET      0xCC
#define AIPU_ADDR_EXT1_HIGH_BASE_REG_OFFSET 0xD0
#define AIPU_ADDR_EXT1_LOW_BASE_REG_OFFSET  0xD4
#define AIPU_ADDR_EXT2_CTRL_REG_OFFSET      0xD8
#define AIPU_ADDR_EXT2_HIGH_BASE_REG_OFFSET 0xDC
#define AIPU_ADDR_EXT2_LOW_BASE_REG_OFFSET  0xE0
#define AIPU_ADDR_EXT3_CTRL_REG_OFFSET      0xE4
#define AIPU_ADDR_EXT3_HIGH_BASE_REG_OFFSET 0xE8
#define AIPU_ADDR_EXT3_LOW_BASE_REG_OFFSET  0xEC
#define ZHOUYI_V2_MAX_REG_OFFSET            (AIPU_ADDR_EXT3_LOW_BASE_REG_OFFSET)
#define ZHOUYI_X1_DTCM_CTRL                 0x180
#define ZHOUYI_X1_DTCM_HIGH_BASE_ADDR       0x184
#define ZHOUYI_X1_DTCM_LOW_BASE_ADDR        0x188
#define ZHOUYI_X1_SOFT_RESET_OFFSET         0x200
#define ZHOUYI_X1_MAX_REG_OFFSET            (ZHOUYI_X1_SOFT_RESET_OFFSET)

struct aipu_priv_operations *get_legacy_priv_ops(void);
struct aipu_operations *get_zhouyi_x1_ops(void);

#endif /* __X1_H__ */
