/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __V4_H__
#define __V4_H__

#include "v3.h"

/**************************************************************************************
 *                             Cluster [N] Registers
 *************************************************************************************/
/**
 * Cluster Configuration Register (Read-Only)
 *
 * the same as v3
 */

/**
 * Cluster [N] Control Register
 *
 * [31:24] core enable bits (27:24: core 3 - core 0; 31:28: reserved)
 * [13]  core partition mode
 * [3:0] number of TEC enabled in every core (RO)
 *
 * other fields are the same as v3
 */
#define CONFIG_CLUSTER_V4(partition, en_core, en_aiff, par_mode) \
	(CONFIG_CLUSTER(partition, en_core, en_aiff, 0) | (par_mode))

/**************************************************************************************
 *                             TSM Registers
 *************************************************************************************/
/**
 * TSM Command Schedule Control Handle Register
 *
 * [31:28] Write to select a cluster for direct dispatching
 * [27:24] Write to select a core for direct dispatching
 * [19]    Command pool type
 *         0: PCP
 *         1: SCP
 * [18:16] Write to select a command pool
 *         PCP: pool -> cluster
 *             0 -> 0, 1 -> 1, 2 -> 2, 3 -> 3
 *         SCP: pool -> cluster
 *             4 -> 0, 5 -> 1, 6 -> 2, 7 -> 3
 * [12]    0 to map one cluster to this command pool (RO)
 *         1) Only works for command pool creation
 *         2) R/O for single cluster TSM
 *
 * other fields are the same as v3
 */
#define TSM_DISPATCH_CMD_POOL_V4(pool, qos)          (_TSM_DISPATCH | (qos) | (pool))

/**
 * TSM Command Schedule Address High Register
 *
 * the same as v3
 */

/**
 * TSM Command Schedule Address Low Register
 *
 * the same as v3
 */

/**
 * TSM Configuration Register
 *
 * the same as v3
 */

/**
 * TSM Build Info Register (Read-Only)
 *
 * [25:24] Partition mode: 0
 * [19:16] Supported command pool number: 1/2
 *
 * other fields are the same as v3
 */
#define GET_V4_CMD_POOL_NUM(build_info_32)          ((((build_info_32) >> 16) & 0xF) + 1)

/**
 * TSM Status Register
 *
 * [23:16] TSM abort command pool done
 * [15:12] TSM SCP (QoS high) is full
 * [11:8]  TSM PCP (QoS high) is full
 * [7:4]   TSM SCP (QoS low) is full
 * [3:0]   TSM PCP (QoS low) is full
 */
#define PCP_ABORT_DONE_BIT                          16
#define SCP_ABORT_DONE_BIT                          20
#define IS_ABORT_DONE(status_32, done_bit)          (((status_32) >> (done_bit)) & 0x1)
#define CLEAR_ABORT(done_bit)                       BIT(done_bit)
#define IS_PCP_FULL(status, high)                   (((status) >> ((high) ? 8 : 0)) & 0x1)
#define IS_SCP_FULL(status, high)                   (((status) >> (4 + ((high) ? 8 : 0))) & 0x1)

/**
 * TSM Command Schedule TCB Number Register
 *
 * [31:0] TCB number of chain
 */
#define TSM_COMMAND_SCHEDULE_TCB_NUM_REG            0x1C

/**
 * A1.1.8 TSM Revision Register (Read-Only)
 *
 * the same as v3
 */

/**
 * Tick Counter Related Registers
 *
 * the same as v3
 */

/**
 * TSM Interrupt Status Register
 *
 * Bit 0 - 15: TSM interrupt 0 - 15
 */
#define TSM_CLEAR_INTERRUPT_V4(i)                   BIT(i)
#define GET_V4_IRQS_BITS(status)                    ((status) & 0xFFFF)

#define TSM_INTERRUPT_STATUS_REG                    0x80

/**************************************************************************************
 *                             TSM Command Pool (PCP) Registers
 *************************************************************************************/
#define _TSM_PCP_REGISTERS_BASE                     0x800

/**
 * TSM Command Pool [N] Status PCP Register
 *
 * [31:24]: cluster physical ID
 * [6]: idle (0: busy; 1: idle)
 * [5]: timeout (0: none; 1: MIF timeout)
 * [4]: error (0: none; 1: pool error)
 * [0]: done (0: none; 1: pool done)
 */
#define IS_POOL_DONE(val)                           ((val) & 0x1)
#define IS_POOL_ERROR(val)                          (((val) >> 4) & 0x1)
#define IS_POOL_TIMEOUT(val)                        (((val) >> 5) & 0x1)
#define IS_POOL_IDLE(val)                           (((val) >> 6) & 0x1)
#define IS_POOL_CID(val)                            ((val) >> 24)
#define CLEAR_POOL_DONE                             BIT(0)

#define COMMAND_POOL_PCP_STATUS_REG                 (_TSM_PCP_REGISTERS_BASE + 0x4)

/**
 * TSM Command Pool [N] Interrupt Control PCP Register
 *
 * [6]  timeout interrupt enable
 *
 * other fields are the same as v3
 */
#define EN_TIMEOUT_INTR                             BIT(6)  /* shared with SCP */
#define EN_ALL_TYPE_INTRS_V4                        (EN_ALL_TYPE_INTRS_V3 | EN_TIMEOUT_INTR)

#define COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG      (_TSM_PCP_REGISTERS_BASE + 0x8)

/**************************************************************************************
 *                             TSM Command Pool (SCP) Registers
 *************************************************************************************/
#define _TSM_SCP_REGISTERS_BASE                     0x900

/**
 * TSM Command Pool [N] Status SCP Register
 *
 * [31:24]: cluster physical ID
 * [6]: idle (0: busy; 1: idle)
 * [5]: timeout (0: none; 1: MIF timeout)
 * [4]: error (0: none; 1: pool error)
 * [0]: done (0: none; 1: pool done)
 */
#define COMMAND_POOL_SCP_STATUS_REG                 (_TSM_SCP_REGISTERS_BASE + 0x4)

/**
 * TSM Command Pool [N] Interrupt Control SCP Register
 *
 * [6]  timeout interrupt enable
 *
 * other fields are the same as v3
 */
#define COMMAND_POOL_SCP_INTERRUPT_CONTROL_REG      (_TSM_SCP_REGISTERS_BASE + 0x8)

/**************************************************************************************
 *                             Interrupt Registers (Per Interrupt)
 *************************************************************************************/
/**
 * Per interrupt registers
 *
 * 0xA00: interrupt 0
 * 0xA10: interrupt 1
 * 0xA20: interrupt 2
 * ...
 * 0xAf0: interrupt 15
 */
#define _GET_PER_INTERRUPT_REGISTER_OFFSET(id)      (0xA00 + 0x10 * (id))

/**
 * TSM Interrupt Type Info Register [N]
 *
 * [23:20] core physical ID
 * [19:16] TEC physical ID
 * [11]    pool interrupt status
 * [10]    cluster interrupt status
 * [9]     core interrupt status
 * [8]     TEC interrupt status
 * [6]     timeout bit
 * [5]     signal bit
 * [4]     pool error bit
 * [3]     fault bit
 * [2]     exception bit
 * [0]     done bit
 */
#define GET_INTERRUPT_CORE_ID(status_32)            (((status_32) >> 20) & 0xF)
#define GET_INTERRUPT_TEC_ID(status_32)             (((status_32) >> 16) & 0xF)
#define IS_TIMEOUT_INTR_V4(status_32)               (((status_32) >> 6) & 0x1)
#define IS_SIGNAL_INTR_V4(status_32)                (((status_32) >> 5) & 0x1)
#define IS_ERROR_INTR_V4(status_32)                 (((status_32) >> 4) & 0x1)
#define IS_FAULT_INTR_V4(status_32)                 (((status_32) >> 3) & 0x1)
#define IS_EXCEPTION_INTR_V4(status_32)             (((status_32) >> 2) & 0x1)
#define IS_DONE_INTR_V4(status_32)                  ((status_32) & 0x1)

#define INTERRUPT_TYPE_INFO_REG(id)                 (_GET_PER_INTERRUPT_REGISTER_OFFSET(id) + 0x0)

/**
 * TSM Interrupt TCB Pointer Register [N]
 *
 * [31:0] TCBP (RO)
 */
#define INTERRUPT_TCB_PTR_REG(id)                   (_GET_PER_INTERRUPT_REGISTER_OFFSET(id) + 0x4)

#define ZHOUYI_V4_MAX_REG_OFFSET                    0x322C /* to be updated */

u64 get_gm_size(u32 val);
struct aipu_operations *get_zhouyi_v4_ops(void);
struct aipu_priv_operations *get_v4_priv_ops(void);

#endif /* __V4_H__ */
