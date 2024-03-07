/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. */

#ifndef __V1_H__
#define __V1_H__

#include "zhouyi.h"

/*
 * Zhouyi V1 AIPU Interrupts
 */
#define ZHOUYI_V1_IRQ              (ZHOUYI_IRQ)
#define ZHOUYI_V1_IRQ_ENABLE_FLAG  (ZHOUYI_V1_IRQ)
#define ZHOUYI_V1_IRQ_DISABLE_FLAG (ZHOUYI_IRQ_NONE)

#define ZHOUYI_V1_MAX_SCHED_JOB_NUM  1

/*
 * Zhouyi V1 AIPU Specific Host Control Register Map
 */
#define ZHOUYI_INTR_CAUSE_REG_OFFSET          0x20
#define ZHOUYI_INTR_STAT_REG_OFFSET           0x24
#define ZHOUYI_INTR_BACKUP_STAT_REG_OFFSET    0x28
#define ZHOUYI_INTR_BACKUP_PC_REG_OFFSET      0x2C
#define ZHOUYI_DBG_ERR_CAUSE_REG_OFFSET       0x30
#define ZHOUYI_DBG_DATA_REG_0_OFFSET          0x34
#define ZHOUYI_DBG_DATA_REG_1_OFFSET          0x38
#define ZHOUYI_L2_CACHE_FEATURE_REG_OFFSET    0x6C
#define ZHOUYI_V1_MAX_REG_OFFSET              0x6C

struct aipu_priv_operations *get_v1v2_priv_ops(void);
struct aipu_operations *get_zhouyi_v1_ops(void);

#endif /* __V1_H__ */
