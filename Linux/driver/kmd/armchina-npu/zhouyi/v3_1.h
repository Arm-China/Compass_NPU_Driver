/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __V3_1_H__
#define __V3_1_H__

/**************************************************************************************
 *                             Cluster [N] Registers
 *************************************************************************************/
/**
 * Cluster Range Base Address
 *
 * 0xC00 ~ 0xC1F    Cluster 0
 * 0xC20 ~ 0xC3F    Cluster 1
 * 0xC40 ~ 0xC5F    Cluster 2
 * ...
 * 0xCE0 ~ 0xCFF    Cluster 7
 */
#define _GET_CLUSTER_OFFSET_V3_1(id)                     (0xC00 + 0x20 * (id))
#define MAX_CLUSTER_NUM_V3_1                             8

/**
 * A1.1.1 Cluster Configuration Register (Read-Only)
 * [31-13]  Reserved
 * [12]     set if cluster is present
 * [11:8]   number of available AIPU cores
 * [7:4]    number of available AIFF in every core
 * [3:0]    number of available TEC in every core
 */
#define IS_CLUSTER_PRESENT_V3_1(config_32)               (((config_32) >> 12) & 0x1)
#define GET_AIPU_CORE_NUM_V3_1(config_32)                (((config_32) >> 8) & 0xF)
#define GET_AIFF_NUM_V3_1(config_32)                     (((config_32) >> 4) & 0xF)
#define GET_TEC_NUM_V3_1(config_32)                      ((config_32) & 0xF)
#define GET_NUMS_V3_1(config_32)                         ((config_32) & 0xFFF)
#define CLUSTER_CONFIG_REG_V3_1(id)                      (_GET_CLUSTER_OFFSET_V3_1(id) + 0x0)

/*
 * A1.1.2 Cluster Control Register
 * [31:24] core enable bits (27:24: core 3 - core 0; )
 * [23:13] reserved (RO)
 * [12]    cluster enable (RO)
 * [11:8]  Core enable number (RO)
 * [7:4]   AIFF enable number (RO)
 * [3:0]   TEC enable number (RO)
 * [3:0] number of TEC enabled in every core (RO)
 */
#define _ENABLE_CORE_V3_1(core_en)                      ((core_en) << 24)
#define _ENABLE_CLUSTER_V3_1                             (0x1 << 12)
#define _DISABLE_CLUSTER_V3_1                            (0x0 << 12)
#define _EN_CORE_NUM_V3_1(num)                           (((num) & 0xF) << 8)
#define _EN_AIFF_NUM_V3_1(num)                           (((num) & 0xF) << 4)
#define _EN_TEC_NUM_V3_1(num)                            ((num) & 0xF)
#define EN_NUMS_V3_1(nums)                               ((nums) & 0xFFF)
#define ENABLE_CLUSTER_V3_1(partition, nums) \
	(EN_NUMS_V3_1(nums) | _ENABLE_CLUSTER_V3_1)
#define DISABLE_CLUSTER_V3_1                             _DISABLE_CLUSTER_V3_1
#define CONFIG_CLUSTER_V3_1(core_en, en_core_num, en_aiff, en_tec, par_mode) \
	(_ENABLE_CORE_V3_1(core_en) | \
	 _EN_CORE_NUM_V3_1(en_core_num) | \
	 _EN_AIFF_NUM_V3_1(en_aiff) | \
	 _EN_TEC_NUM_V3_1(en_tec) | \
	 _ENABLE_CLUSTER_V3_1 | (par_mode))

#define CLUSTER_CONTROL_REG_V3_1(id)                     (_GET_CLUSTER_OFFSET_V3_1(id) + 0x4)

/*****************************************************************************
 *                             TSM Registers
 *****************************************************************************/
/**
 * TSM Command Schedule Control Handle Register
 *
 * [31:28] Write to select a cluster for direct dispatching
 * [27:24] Write to select a core for direct dispatching
 * [23:20] Reserved (RO)
 * [19]    Command pool type
 *         0: PCP
 * [18:16] Write to select a command pool
 *         0 command pool 0 sellected
 * [15:13] Reserved (RO)
 * [12]    0 to map one cluster to this command pool (RO)
 *         1) Only works for command pool creation
 *         2) R/O for single cluster TSM
 * [11:10] Reserved (RO)
 * [9:8]   QOS 0 slow, 1 Media, 2 fast (reserved in LIN), 3 reserved
 * [7:4]   Reserved (RO)
 * [3:0]   command tpye:0 none, 1 create, 2 destroy, 3 Abortion,
 *                      4 Dispatch,5 direct dispatch, 6 Others.
 */
#define _TSM_DIRECT_CLUSTER_V3_1(cluser)                  ((cluster) << 28)
#define _TSM_DIRECT_CORE_V3_1(core)                       ((core) << 24)
#define _TSM_POOL_V3_1(pool)                              ((pool) << 16)
#define _TSM_MAP_V3_1(map)                                ((map) << 12)
#define _TSM_QOS_V3_1(qos)                                ((qos) << 8)
#define _TSM_CREATE_V3_1                                  0x1
#define _TSM_DESTROY_V3_1                                 0x2
#define _TSM_ABORT_V3_1                                   0x3
#define _TSM_DISPATCH_V3_1                                0x4
#define _TSM_DEBUG_DISPATCH_V3_1                          0x5

#define TSM_MAP_SINGLE_V3_1                              _TSM_MAP_V3_1(0x0)
#define TSM_MAP_ALL_V3_1                                 _TSM_MAP_V3_1(0x1)
#define TSM_CREATE_CMD_POOL_V3_1(pool, map)              (_TSM_CREATE_V3_1 | _TSM_POOL_V3_1(pool) | (map))
#define TSM_DESTROY_CMD_POOL_V3_1(pool)                  (_TSM_DESTROY_V3_1 | _TSM_POOL_V3_1(pool))
#define TSM_ABORT_CMD_POOL_V3_1(pool)                    (_TSM_ABORT_V3_1 | _TSM_POOL_V3_1(pool))
#define TSM_DBG_DISPATCH_CMD_POOL_V3_1(pool, qos, core)  (_TSM_DEBUG_DISPATCH_V3_1 | (qos) | \
						     _TSM_POOL_V3_1(pool) | _TSM_DIRECT_CORE_V3_1(core))

#define TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1             0x0
#define TSM_DISPATCH_CMD_POOL_V3_1(pool, qos)         (_TSM_DISPATCH_V3_1 | (qos) | (pool))

/**
 * A1.1.6 TSM Command Schedule Address High Register
 *
 * [31:0] Write to set the address[63:32] of head of a TCB chain to be dispatched
 */
#define TSM_CMD_SCHD_ADDR_HIGH_REG_V3_1               0x8

/**
 * A1.1.7 TSM Command Schedule Address Low Register
 *
 * [31:12] Write to set the address[31:12] of head of a TCB chain to
 *         be dispatched
 *         1) this address shall be 4k page aligned
 */
#define TSM_CMD_SCHD_ADDR_LOW_REG_V3_1                0xC

/**
 * A1.1.9 TSM Configuration Register
 * [31:4] Reserved (RO)
 * [3:0]  TSM scheduling policy
 */
#define TSM_CONFIG_REG_V3_1                           0x10

/**
 * TSM Build Info Register (Read-Only)
 *
 * [25:24] Partition mode: 0
 * [19:16] Supported command pool number: 1/2
 *
 * other fields are the same as v3
 */
#define TSM_BUILD_INFO_REG_V3_1                       0x14
#define GET_CMD_POOL_NUM_V3_1(build_info_32)          ((((build_info_32) >> 16) & 0xF) + 1)

/**
 * TSM Status Register
 * [31]    scheduling cmd fail
 * [30:24] Reserved (RO)
 * [23:16] TSM abort command pool done
 * [15:12] Reserved
 * [11:8]  TSM PCP (QoS high) is full
 * [7:4]   Reserved
 * [3:0]   TSM PCP (QoS low) is full
 */
#define PCP_ABORT_DONE_BIT                          16
#define IS_ABORT_DONE(status_32, done_bit)          (((status_32) >> (done_bit)) & 0x1)
#define CLEAR_ABORT(done_bit)                       ((~(1 << (done_bit))) & (0x7FFFFFFF))
#define IS_PCP_FULL(status, high)                   (((status) >> ((high) ? 8 : 0)) & 0x1)
#define IS_CMD_FAIL_V3_1(status_32)                   ((status_32) >> 31)
#define IS_CMD_POOL_FULL_V3_1(status_32)              ((status_32) & 0xFF)
#define CLEAR_CMD_FAIL_V3_1(status_32)                ((status_32) & 0x7FFFFFFF)
#define TSM_STATUS_REG_V3_1                           0x18

/**
 * A1.1.8 TSM Command Schedule TCB Number Register
 *
 * [31:0] TCB number of chain
 */
#define TSM_COMMAND_SCHEDULE_TCB_NUM_REG_V3_1         0x1C

/**************************************************************************************
 *                             Debug Link Access Registers
 *************************************************************************************/
/**
 * A1.1.28 AHB Internal CSR Selection Control Register
 *
 * [12]   debug enable
 * [11:4] cluster selection
 * [3:0]  core selection
 */
#define _ENABLE_DEBUG_V3_1                            BIT(12)
#define _SET_DBG_CLUSTER_V3_1(id)                     (((id) & 0xFF) << 4)
#define _SET_DBG_CORE_V3_1(id)                        ((id) & 0xF)
#define SELECT_DEBUG_CORE_V3_1(cluster, core)         (_ENABLE_DEBUG_V3_1 | \
			_SET_DBG_CLUSTER_V3_1(cluster) | _SET_DBG_CORE_V3_1(core))
#define DISABLE_DEBUG_V3_1                            0
#define AHB_INTERNAL_CSR_SELECTION_CTRL_REG         0x1F00
#define CLUSTER_MMR_WIN_BASE                        (0x2000)
#define CORE_MMR_WIN_BASE                           (0x3000)

/**
 * A1.1.8 TSM Revision Register (Read-Only)
 *
 * [27:24] implementation ID
 * [23:16] primary part number
 * [15:8]  secondary part number
 * [7:4]   major revision ID
 * [3:0]   minor number
 */
#define _IS_TSM_ARCH_V3_1(rev_32)                     (((rev_32) >> 16) & 0x1)
#define _IS_V3_1_ARCH(rev_32)                         ((((rev_32) >> 8) & 0x1))

#define TSM_REVISION_REG_V3_1                         0x50

/**
 * A1.1.22 Tick Counter Control Status Register
 *
 * [8] counter overflow flag
 * [1] clear
 * [0] enable
 */
#define IS_COUNTER_OVERFLOW_V3_1(status_32)           (((status_32) >> 8) & 0x1)
#define CLEAR_COUNTER_V3_1                            BIT(1)
#define ENABLE_COUNTER_V3_1                           0x1
#define DISABLE_COUNTER_V3_1                          ((CLEAR_COUNTER_V3_1))

#define TICK_COUNTER_CONTROL_STATUS_REG_V3_1          0x68

/**
 * TSM Interrupt Status Register
 *
 * Bit 0 - 15: TSM interrupt 0 - 15
 */
#define TSM_CLEAR_INTERRUPT_V3_1(i)                   BIT(i)
#define GET_V3_1_IRQS_BITS(status)                    ((status) & 0xFFFF)

#define TSM_INTERRUPT_STATUS_REG                    0x80
#define TSM_IRQ_MAX_NUM                             16

/**************************************************************************************
 *                             TSM Command Pool (PCP) Registers
 *************************************************************************************/
#define _TSM_PCP_REGISTERS_BASE                     0x800

/**************************************************************************************
 *                             HOST TSM PMU Register
 *************************************************************************************/
/**
 * A1.1.32 PMU TOP Soft Reset
 *
 * [1] soft reset status bit
 *     1) host write 0 to launch AIPU soft reset
 *     2) AIPU write 1 when a soft reset is done
 * [0] soft reset config bit
 *     1) host write 1 to launch AIPU soft reset
 *     2) AIPU write 0 when a soft reset is done
 */
#define PMU_TOP_SOFT_RESET_REG                      (0x20)

/**
 * A1.1.33 PMU Cluster Soft Reset
 *
 * [7:4] soft reset status bit
 *     1) host write 0 to clear it after reading
 *     2) AIPU write 1 when a soft reset is done
 * [3:0] cluster soft reset config bit
 *     1) host write 1 to launch AIPU soft reset
 *     2) AIPU write 0 when a soft reset is done
 */
#define CONFIG_PMU_CLUSTER_RESET(cluster)           (1 << ((cluster) & 0xF))
#define PMU_CLUSTER_RESET_DONE(cluster)			(1 << (((cluster) & 0xF) + 4))
#define PMU_CLUSTER_RESET_REG                       (0x24)

/**
 * A1.1.34 PMU Core Soft Reset
 *
 * [31:16] soft reset status bit
 *     1) host write 0 to clear it after reading
 *     2) AIPU write 1 when a soft reset is done
 * [15:0] cluster soft reset config bit
 *     1) host write 1 to launch AIPU soft reset
 *     2) AIPU write 0 when a soft reset is done
 */
#define CONFIG_PMU_CORE_RESET(cluster, core)	(1 << (((core) & 0xFF) + \
								((cluster) & 0xF) * 4))
#define PMU_CORE_RESET_DONE(cluster, core)		(1 << (((core) & 0xFF) + \
								(((cluster) & 0xF) * 4) + 16))
#define PMU_CORE_RESET_REG                          (0x28)

/**
 * A1.1.35 PMU Reset Timeout
 *
 * [31:16] core reset timeout flag bit
 *     1) host write 0 to clear it after reading
 *     2) AIPU write 1 when there is reset request.
 * [3:0] Cluster reset timeout flag
 *     1) host write 0 to clear it after reading
 *     2) AIPU write 1 when there is reset request.
 */
#define PMU_RESET_TIMEOUT_REG                       (0x2c)

/**
 * A1.1.36 PMU Top Clock and Power Control Register
 *
 * [31:3] reserved
 * [2]    secure/non-secure mode (0/1)
 * [1]    clock gating disabled/enabled (0/1)
 * [0]    PMU disabled/enabled (0/1)
 */
#define PMU_SECURE_MODE                             BIT(2)
#define PMU_NONSECURE_MODE                          0
#define PMU_ENABLE_CLOCK_GATING                     BIT(1)
#define PMU_DISABLE_CLOCK_GATING                    0
#define PMU_ENABLE                                  BIT(0)
#define PMU_DISABLE                                 0
#define PMU_TOP_CLOCK_POWER_CTRL_REG                0x30

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
#define IS_POOL_BUSY(val)                           (!(((val) >> 6) & 0x1))
#define IS_POOL_CID(val)                            ((val) >> 24)
#define CLEAR_POOL_DONE                             BIT(0)

#define COMMAND_POOL_PCP_STATUS_REG                 (_TSM_PCP_REGISTERS_BASE + 0x4)

/**
 * A1.1.20 TSM Command Pool [N] Interrupt Control PCP Register
 * [31:12] Reserved (RO)
 * [11] command pool interrupt enable
 * [10] cluster interrupt enable
 * [9]  core interrupt enable
 * [8]  tec interrupt enable
 * [6]  timeout interrupt enable
 * [5]  signal interrupt enable
 * [4]  error interrupt enable
 * [3]  fault interrupt enable
 * [2]  exception interrupt enable
 * [0]  done interrupt enable
 */
#define EN_CMD_POOL_INTR_V3_1                         BIT(11)
#define EN_CLUSTER_INTR_V3_1                          BIT(10)
#define EN_CORE_INTR_V3_1                             BIT(9)
#define EN_TEC_INTR_V3_1                              BIT(8)
#define EN_TIMEOUT_INTR_V3_1                          BIT(5)
#define EN_ERROR_INTR_V3_1                            BIT(4)
#define EN_FAULT_INTR_V3_1                            BIT(3)
#define EN_EXCEPTION_INTR_V3_1                        BIT(2)
#define EN_SIGNAL_INTR_V3_1                           BIT(1)
#define EN_DONE_INTR_V3_1                             BIT(0)

#define EN_ALL_TYPE_INTRS_V3_1 \
						(EN_SIGNAL_INTR_V3_1 | EN_ERROR_INTR_V3_1 | \
						EN_FAULT_INTR_V3_1 | EN_EXCEPTION_INTR_V3_1 | \
						EN_DONE_INTR_V3_1 | EN_TIMEOUT_INTR_V3_1)

#define EN_ALL_LEVEL_INTRS_V3_1 \
						(EN_CMD_POOL_INTR_V3_1 | EN_CLUSTER_INTR_V3_1 | \
						 EN_CORE_INTR_V3_1 | EN_TEC_INTR_V3_1)
#define EN_CMD_POOL_ALL_INTRS_V3_1  (EN_CMD_POOL_INTR_V3_1 | EN_ALL_TYPE_INTRS_V3_1)
#define EN_ALL_INTRS_V3_1           (EN_ALL_LEVEL_INTRS_V3_1 | EN_ALL_TYPE_INTRS_V3_1)
#define DISABLE_ALL_INTRS_V3_1                           0

#define COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG      (_TSM_PCP_REGISTERS_BASE + 0x8)

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
 * A1.1.14 TSM Interrupt Type Info Register [N]
 * [31:28] cmd pool ID
 * [27:24] Cluster physical ID
 * [23:20] core physical ID
 * [19:16] TEC physical ID
 * [11]    pool interrupt status
 * [10]    cluster interrupt status
 * [9]     core interrupt status
 * [8]     TEC interrupt status
 * [5]     timeout bit
 * [4]     pool error bit
 * [3]     fault bit
 * [2]     exception bit
 * [1]     irq_signal bit
 * [0]     done bit
 */
#define IS_RESET(status_32)                         (((status_32) >> 3) & 0x7)
#define GET_INTR_CLUSTER_ID_V3_1(status_32)           (((status_32) >> 24) & 0xFF)
#define GET_INTR_CORE_ID_V3_1(status_32)              (((status_32) >> 20) & 0xF)
#define GET_INTR_TEC_ID_V3_1(status_32)               (((status_32) >> 16) & 0xF)
#define IS_POOL_IRQ_V3_1(status_32)                   (((status_32) >> 11) & 0x1)
#define IS_CLUSTER_IRQ_V3_1(status_32)                (((status_32) >> 10) & 0x1)
#define IS_CORE_IRQ_V3_1(status_32)                   (((status_32) >> 9) & 0x1)
#define IS_TEC_IRQ_V3_1(status_32)                    (((status_32) >> 8) & 0x1)
#define IS_TIMEOUT_IRQ_V3_1(status_32)                (((status_32) >> 5) & 0x1)
#define IS_ERROR_IRQ_V3_1(status_32)                  (((status_32) >> 4) & 0x1)
#define IS_FAULT_IRQ_V3_1(status_32)                  (((status_32) >> 3) & 0x1)
#define IS_EXCEPTION_IRQ_V3_1(status_32)              (((status_32) >> 2) & 0x1)
#define IS_SIGNAL_IRQ_V3_1(status_32)                 (((status_32) >> 1) & 0x1)
#define IS_DONE_IRQ_V3_1(status_32)                   ((status_32) & 0x1)
#define IS_ABNORMAL_V3_1(status_32)                   (((status_32) >> 2) & 0xF)
#define IS_SERIOUS_ERR_V3_1(status_32)                (((status_32) >> 3) & 0x7)
#define GET_INTR_TYPE_V3_1(status_32)                 ((status_32) & 0x3F)
#define IS_IRQ_TO_HANDLE_V3_1(status_32)              ((status_32) & 0xF1F)

#define INTERRUPT_TYPE_INFO_REG(id)                 (_GET_PER_INTERRUPT_REGISTER_OFFSET(id) + 0x0)

/**
 * TSM Interrupt TCB Pointer Register [N]
 *
 * [31:0] TCBP (RO)
 */
#define INTERRUPT_TCB_PTR_REG(id)                   (_GET_PER_INTERRUPT_REGISTER_OFFSET(id) + 0x4)

/**
 * TSM Interrupt Signal Register [N]
 *
 * [31:15]   reserved
 * [14:0]    irq signal id
 */
#define INTERRUPT_SIGNAL_REG(id)                    (_GET_PER_INTERRUPT_REGISTER_OFFSET(id) + 0x8)

/**
 * TSM Interrupt Signal flag Register [N]
 *
 * [31:0]    irq signal flag
 */
#define IS_HW_EXCEPTION_SIGNAL_V3_1(flag)             ((flag) >> 31)
#define IS_SW_EXCEPTION_SIGNAL_V3_1(flag)             (((flag) >> 30) & 0x1)
#define IS_EXCEPTION_SIGNAL_V3_1(flag)                ((flag) >> 30)
#define IS_PRINTF_SIGNAL_V3_1(flag)                   (((flag) >> 29) & 0x1)
#define IS_PROFILER_SIGNAL_V3_1(flag)                 (((flag) >> 28) & 0x1)
#define IS_COREDUMP_SIGNAL_V3_1(flag)                 (((flag) >> 27) & 0x1)
#define GET_ERR_CODE_V3_1(flag)                       ((flag) & 0xFFFF)
#define GET_PRINF_SIZE_V3_1(flag)                     ((flag) & 0xFFFF)
#define GET_PROFILER_BUF_PA_V3_1(flag)                (((flag) & 0xFFFFF) << 12)

#define INTERRUPT_SIGNAL_FLAG_REG(id)               (_GET_PER_INTERRUPT_REGISTER_OFFSET(id) + 0xc)

#define ZHOUYI_V3_1_MAX_REG_OFFSET                    0x322C /* to be updated */

u64 get_gm_size(u32 val);
struct aipu_operations *get_zhouyi_v3_1_ops(void);
struct aipu_priv_operations *get_v3_1_priv_ops(void);

#endif /* __V3_1_H__ */
