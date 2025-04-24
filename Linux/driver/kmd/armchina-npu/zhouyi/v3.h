/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __V3_H__
#define __V3_H__

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
#define _GET_CLUSTER_OFFSET(id)                     (0xC00 + 0x20 * (id))
#define MAX_CLUSTER_NUM                             8

/**
 * A1.1.1 Cluster Configuration Register (Read-Only)
 *
 * [12]   set if cluster is present
 * [11:8] number of available AIPU cores
 * [7:4]  number of available AIFF in every core
 * [3:0]  number of available TEC in every core
 */
#define IS_CLUSTER_PRESENT(config_32)               (((config_32) >> 12) & 0x1)
#define GET_AIPU_CORE_NUM(config_32)                (((config_32) >> 8) & 0xF)
#define GET_AIFF_NUM(config_32)                     (((config_32) >> 4) & 0xF)
#define GET_TEC_NUM(config_32)                      ((config_32) & 0xF)
#define GET_NUMS(config_32)                         ((config_32) & 0xFFF)

#define CLUSTER_CONFIG_REG(id)                      (_GET_CLUSTER_OFFSET(id) + 0x0)

/**
 * A1.1.2 Cluster Control Register
 *
 * [17:16] partition ID
 * [12]    cluster enable
 * [11:8]  number of cores enabled (should be > 0)
 * [7:4]   number of AIFF enabled in every core (should be > 0)
 * [3:0]   number of TEC enabled in every core (should be > 0)
 */
#define _SET_PARTITION(partition)                   ((partition) << 16)
#define _ENABLE_CLUSTER                             (0x1 << 12)
#define _DISABLE_CLUSTER                            (0x0 << 12)
#define _EN_CORE_NUM(num)                           (((num) & 0xF) << 8)
#define _EN_AIFF_NUM(num)                           (((num) & 0xF) << 4)
#define _EN_TEC_NUM(num)                            ((num) & 0xF)
#define EN_NUMS(nums)                               ((nums) & 0xFFF)

#define ENABLE_CLUSTER(partition, nums) \
	(_SET_PARTITION(partition) | EN_NUMS(nums) | _ENABLE_CLUSTER)
#define CONFIG_CLUSTER(partition, en_core, en_aiff, en_tec) \
	(_SET_PARTITION(partition) | \
	 _EN_CORE_NUM(en_core)     | \
	 _EN_AIFF_NUM(en_aiff)     | \
	 _EN_TEC_NUM(en_tec)       | \
	 _ENABLE_CLUSTER)
#define DISABLE_CLUSTER                             _DISABLE_CLUSTER

#define CLUSTER_CONTROL_REG(id)                     (_GET_CLUSTER_OFFSET(id) + 0x4)

/**************************************************************************************
 *                             TSM Registers
 *************************************************************************************/
/**
 * A1.1.3 TSM Command Schedule Control Handle Register
 *
 * [27:24] Write to select a core for dispatching a command to be debugged
 * [19:16] Write to select a command pool
 * [12]    Write 0/1 to map one/all of the clusters to this command pool
 *         1) Only works for command pool creation
 *         2) R/O for single cluster TSM
 * [9:8]   Write 0/1/2 to configure the QoS of this pool as slow/fast
 * [3:0]   Write 0/1/2/3/4/5 to issue command type as:
 *         none/create/destroy/abort/dispatch/debug-dispatch
 */
#define _TSM_DEBUG_CORE(core)                        ((core) << 24)
#define _TSM_POOL(pool)                              ((pool) << 16)
#define _TSM_MAP(map)                                ((map) << 12)
#define _TSM_QOS(qos)                                ((qos) << 8)
#define _TSM_CREATE                                  0x1
#define _TSM_DESTROY                                 0x2
#define _TSM_ABORT                                   0x3
#define _TSM_DISPATCH                                0x4
#define _TSM_DEBUG_DISPATCH                          0x5

#define TSM_MAP_SINGLE                              _TSM_MAP(0x0)
#define TSM_MAP_ALL                                 _TSM_MAP(0x1)
#define TSM_CREATE_CMD_POOL(pool, map)              (_TSM_CREATE | _TSM_POOL(pool) | (map))
#define TSM_DESTROY_CMD_POOL(pool)                  (_TSM_DESTROY | _TSM_POOL(pool))
#define TSM_ABORT_CMD_POOL(pool)                    (_TSM_ABORT | _TSM_POOL(pool))
#define TSM_DISPATCH_CMD_POOL(pool, qos)            (_TSM_DISPATCH | (qos) | _TSM_POOL(pool))
#define TSM_DBG_DISPATCH_CMD_POOL(pool, qos, core)  (_TSM_DEBUG_DISPATCH | (qos) | \
						     _TSM_POOL(pool) | _TSM_DEBUG_CORE(core))

#define TSM_CMD_SCHD_CTRL_HANDLE_REG                0x0

/**
 * A1.1.4 TSM Command Schedule Control Info Register
 *
 * [15:0] Write to set a command tag ID for debug purpose
 *        1) Such tag ID will be attached to every command to TSM
 */
#define TSM_CMD_SCHD_CTRL_INFO_REG                  0x4

/**
 * A1.1.5 TSM Command Schedule Address High Register
 *
 * [31:0] Write to set the address[63:32] of head of a TCB chain to be dispatched
 */
#define TSM_CMD_SCHD_ADDR_HIGH_REG                  0x8

/**
 * A1.1.6 TSM Command Schedule Address Low Register
 *
 * [31:12] Write to set the address[31:12] of head of a TCB chain to be dispatched
 *         1) this address shall be 4k page aligned
 */
#define TSM_CMD_SCHD_ADDR_LOW_REG                   0xC

/**
 * A1.1.9 TSM Configuration Register
 *
 * [2:0] TSM scheduling policy (reserved)
 */
#define TSM_CONFIG_REG                              0x10

/**
 * A1.1.7 TSM Build Info Register (Read-Only)
 *
 * [25:24] TSM supported partition number: 1/2/3/4
 * [19:16] TSM supported command pool number
 */
#define GET_MAX_PARTITION_NUM(build_info_32)        ((((build_info_32) >> 24) & 0x3) + 1)
#define GET_MAX_CMD_POOL_NUM(build_info_32)         (((build_info_32) >> 16) & 0xF)

#define TSM_BUILD_INFO_REG                          0x14

/**
 * A1.1.10 TSM Status Register
 *
 * [31]  set if TSM failed in scheduling the last dispatched command
 *       1) shared by all possible command pools
 * [7:0] TSM command pool is full
 */
#define IS_CMD_FAIL(status_32)                      ((status_32) >> 31)
#define IS_CMD_POOL_FULL(status_32)                 ((status_32) & 0xFF)
#define CLEAR_CMD_FAIL(status_32)                   ((status_32) & 0x7FFFFFFF)

#define TSM_STATUS_REG                              0x18

/**
 * A1.1.25 TSM Soft Reset
 *
 * [1] soft reset status bit
 *     1) host write 0 to launch AIPU soft reset
 *     2) AIPU write 1 when a soft reset is done
 * [0] soft reset config bit
 *     1) host write 1 to launch AIPU soft reset
 *     2) AIPU write 0 when a soft reset is done
 */
#define TSM_SOFT_RESET_REG                          0x20

/**
 * A1.1.8 TSM Revision Register (Read-Only)
 *
 * [27:24] implementation ID
 * [23:16] primary part number
 * [15:8]  secondary part number
 * [7:4]   major revision ID
 * [3:0]   minor number
 */
#define _IS_TSM_ARCH(rev_32)                        (((rev_32) >> 16) & 0xFF)
#define _IS_V3_ARCH(rev_32)                         (!(((rev_32) >> 8) & 0xFF))

#define TSM_REVISION_REG                            0x50

/**
 * A1.1.22 Tick Counter Control Status Register
 *
 * [8] counter overflow flag
 * [1] clear
 * [0] enable
 */
#define IS_COUNTER_OVERFLOW(status_32)              (((status_32) >> 8) & 0x1)
#define CLEAR_COUNTER                               BIT(1)
#define ENABLE_COUNTER                              0x0
#define DISABLE_COUNTER                             ((CLEAR_COUNTER) | 0x1)

#define TICK_COUNTER_CONTROL_STATUS_REG             0x68

/**
 * A1.1.23 Tick Counter High Register
 *
 * [31:0] high of tick counter
 */
#define TICK_COUNTER_HIGH_REG                       0x64

/**
 * A1.1.24 Tick Counter Low Register
 *
 * [31:0] low of tick counter
 */
#define TICK_COUNTER_LOW_REG                        0x60

/**************************************************************************************
 *                             TSM Command Pool [N] Registers
 *************************************************************************************/
/**
 * TSM Command Pool Range Address
 *
 * 0x800 ~ 0xC3F    Command Pool 0
 * 0x840 ~ 0xC7F    Command Pool 1
 * 0x880 ~ 0xCBF    Command Pool 2
 * ...
 * 0x9C0 ~ 0x9FF    Command Pool 7
 */
#define _GET_CMD_POOL_OFFSET(id)                    (0x800 + 0x40 * (id))

/**
 * A1.1.11 TSM Command Pool [N] Configuration Register
 *
 * [3:0] write 1/2/3/4 to set current pool to partition 1/2/3/4
 */
#define CONFIG_COMMAND_POOL(p)                      ((p) & 0xF)

#define CMD_POOL_CONFIG_REG(id)                     (_GET_CMD_POOL_OFFSET(id) + 0x0)

/**
 * A1.1.12 TSM Command Pool [N] Status Register
 *
 * [31:24] cluster physical ID
 * [6]     idle bit
 * [5]     signal bit
 * [4]     error bit
 * [3]     fault bit
 * [2]     exception bit
 * [0]     done bit
 */
#define GET_LAST_FINISHED_CID(status_32)            ((status_32) >> 24)
#define IS_CMD_POOL_IDLE(status_32)                 (((status_32) >> 6) & 0x1)
#define IS_CMD_POOL_BUSY(status_32)                 (!IS_CMD_POOL_IDLE(status_32))
#define IS_CMD_POOL_SIGNAL(status_32)               (((status_32) >> 5) & 0x1)
#define IS_CMD_POOL_ERROR(status_32)                (((status_32) >> 4) & 0x1)
#define IS_CMD_POOL_FAULT(status_32)                (((status_32) >> 3) & 0x1)
#define IS_CMD_POOL_EXCEPTION(status_32)            (((status_32) >> 2) & 0x1)
#define IS_CMD_POOL_DONE(status_32)                 ((status_32) & 0x1)
#define CLEAR_CMD_POOL_IDLE                         BIT(6)
#define CLEAR_CMD_POOL_SIGNAL                       BIT(5)
#define CLEAR_CMD_POOL_ERROR                        BIT(4)
#define CLEAR_CMD_POOL_FAULT                        BIT(3)
#define CLEAR_CMD_POOL_EXCEPTION                    BIT(2)
#define CLEAR_CMD_POOL_DONE                         BIT(0)

#define CMD_POOL_STATUS_REG(id)                     (_GET_CMD_POOL_OFFSET(id) + 0x4)

/**
 * A1.1.13 TSM Command Pool [N] Interrupt Control Register
 *
 * [11] command pool interrupt enable
 * [10] cluster interrupt enable
 * [9]  core interrupt enable
 * [8]  tec interrupt enable
 * [5]  signal interrupt enable
 * [4]  error interrupt enable
 * [3]  fault interrupt enable
 * [2]  exception interrupt enable
 * [0]  done interrupt enable
 */
#define EN_CMD_POOL_INTR                            BIT(11)
#define EN_CLUSTER_INTR                             BIT(10)
#define EN_CORE_INTR                                BIT(9)
#define EN_TEC_INTR                                 BIT(8)
#define EN_SIGNAL_INTR                              BIT(5)
#define EN_ERROR_INTR                               BIT(4)
#define EN_FAULT_INTR                               BIT(3)
#define EN_EXCEPTION_INTR                           BIT(2)
#define EN_DONE_INTR                                BIT(0)
#define EN_ALL_LEVEL_INTRS \
	(EN_CMD_POOL_INTR | EN_CLUSTER_INTR | EN_CORE_INTR | EN_TEC_INTR)
#define EN_ALL_TYPE_INTRS_V3  \
	(EN_SIGNAL_INTR | EN_ERROR_INTR | EN_FAULT_INTR | EN_EXCEPTION_INTR | EN_DONE_INTR)
#define EN_CMD_POOL_ALL_INTRS                       (EN_CMD_POOL_INTR | EN_ALL_TYPE_INTRS)
#define EN_ALL_INTRS                                (EN_ALL_LEVEL_INTRS | EN_ALL_TYPE_INTRS)
#define DISABLE_ALL_INTRS                           0

#define CMD_POOL_INTR_CTRL_REG(id)                  (_GET_CMD_POOL_OFFSET(id) + 0x8)

/**
 * A1.1.14 TSM Command Pool [N] Interrupt Status Register (Read-Only)
 *
 * [31:24] cluster physical ID
 * [23:20] core dynamic ID
 * [19:16] TEC dynamic ID
 * [11]    pool interrupt status
 * [10]    cluster interrupt status
 * [9]     core interrupt status
 * [8]     TEC interrupt status
 * [4]     error interrupt
 * [3]     fault interrupt
 * [2]     exception interrupt
 * [0]     done interrupt
 */
#define GET_INTR_CLUSTER_ID(status_32)              (((status_32) >> 24) & 0xFF)
#define GET_INTR_CORE_ID(status_32)                 (((status_32) >> 20) & 0xF)
#define GET_INTR_TEC_ID(status_32)                  (((status_32) >> 16) & 0xF)
#define IS_CMD_POOL_IRQ(status_32)                  (((status_32) >> 11) & 0x1)
#define IS_CLUSTER_IRQ(status_32)                   (((status_32) >> 10) & 0x1)
#define IS_CORE_IRQ(status_32)                      (((status_32) >> 9) & 0x1)
#define IS_TEC_IRQ(status_32)                       (((status_32) >> 8) & 0x1)
#define IS_SIGNAL_IRQ(status_32)                    (((status_32) >> 5) & 0x1)
#define IS_ERROR_IRQ(status_32)                     (((status_32) >> 4) & 0x1)
#define IS_FAULT_IRQ(status_32)                     (((status_32) >> 3) & 0x1)
#define IS_EXCEPTION_IRQ(status_32)                 (((status_32) >> 2) & 0x1)
#define IS_DONE_IRQ(status_32)                      ((status_32) & 0x1)
#define IS_ABNORMAL(status_32)                      (((status_32) >> 2) & 0x7)
#define IS_SERIOUS_ERR(status_32)                   (((status_32) >> 3) & 0x3) /* error or fault */
#define GET_INTR_TYPE(status_32)                    ((status_32) & 0x3F)
#define IS_IRQ_TO_HANDLE(status_32)                 ((status_32) & 0xF1F)

#define CMD_POOL_INTR_STATUS_REG(id)                (_GET_CMD_POOL_OFFSET(id) + 0xC)

/**
 * A1.1.16 TSM Command Pool [N] IRQ Signal Register (Read-Only)
 *
 * [31:16] IRQ signal number
 * [14:0]  IRQ signal ID
 */
#define GET_IRQ_SIGNAL_NUM(sig_32)                  ((sig_32) >> 16)
#define GET_IRQ_SIGNAL_ID(sig_32)                   ((sig_32) & 0x7FFF)

#define CMD_POOL_IRQ_SIG_REG(id)                    (_GET_CMD_POOL_OFFSET(id) + 0x10)

/**
 * A1.1.18 TSM Command Pool [N] First Bad Command Info Register
 *
 * [31]   tag ID valid bit
 * [15:0] first bad tag ID
 */
#define IS_BAD_TAG_ID_VALID(val_32)                 ((val_32) >> 31)
#define GET_FIRST_BAD_TAG_ID(val_32)                ((val_32) & 0xFFFF)

#define CMD_POOL_FIRST_BAD_CMD_REG(id)              (_GET_CMD_POOL_OFFSET(id) + 0x14)

/**
 * A1.1.19 TSM Command Pool [N] Executing Command Info Register
 *
 * [15:0] the last issued executing command tag ID
 */
#define GET_LAST_ISSUED_TAG_ID(val_32)              ((val_32) & 0xFFFF)

#define CMD_POOL_LAST_EXECUTING_CMD_REG(id)         (_GET_CMD_POOL_OFFSET(id) + 0x18)

/**
 * A1.1.20 TSM Command Pool [N] Secure Configuration Register
 *
 * [0]  write 0/1 to configure secure/non-secure node
 */
#define SET_SECURE_MODE                             0x0
#define SET_NONSECURE_MODE                          0x1

#define CMD_POOL_SECURE_REG_REG(id)                 (_GET_CMD_POOL_OFFSET(id) + 0x1C)

/**
 * A1.1.15 TSM Command Pool [N] Interrupt TCB Pointer Register (Read-Only)
 *
 * [31:0] TCBP
 */
#define CMD_POOL_INTR_TCB_PTR_REG(id)                (_GET_CMD_POOL_OFFSET(id) + 0x24)

/**
 * A1.1.17 TSM Command Pool [N] IRQ Signal Flag Register (Read-Only)
 *
 * [31:0] flag
 */
#define IS_HW_EXCEPTION_SIGNAL(flag)                ((flag) >> 31)
#define IS_SW_EXCEPTION_SIGNAL(flag)                (((flag) >> 30) & 0x1)
#define IS_EXCEPTION_SIGNAL(flag)                   ((flag) >> 30)
#define IS_PRINTF_SIGNAL(flag)                      (((flag) >> 29) & 0x1)
#define IS_PROFILER_SIGNAL(flag)                    (((flag) >> 28) & 0x1)
#define IS_COREDUMP_SIGNAL(flag)                    (((flag) >> 27) & 0x1)
#define GET_ERR_CODE(flag)                          ((flag) & 0xFFFF)
#define GET_PRINF_SIZE(flag)                        ((flag) & 0xFFFF)
#define GET_PROFILER_BUF_PA(flag)                   (((flag) & 0xFFFFF) << 12)

#define CMD_POOL_IRQ_SIGNAL_FLAG_REG(id)            (_GET_CMD_POOL_OFFSET(id) + 0x20)

/**************************************************************************************
 *                             Debug Link Access Registers
 *************************************************************************************/
/**
 * A1.1.19 Debug Page Selection Control Register
 *
 * [12]   debug enable
 * [11:4] cluster selection
 * [3:0]  core selection
 */
#define _ENABLE_DEBUG                               BIT(12)
#define _SET_DBG_CLUSTER(id)                        (((id) & 0xFF) << 4)
#define _SET_DBG_CORE(id)                           ((id) & 0xF)
#define SELECT_DEBUG_CORE(cluster, core)            (_ENABLE_DEBUG | _SET_DBG_CLUSTER(cluster) | \
						     _SET_DBG_CORE(core))
#define DISABLE_DEBUG                               0
#define DEBUG_PAGE_SELECTION_REG                    0x1F00

/**
 * A1.2.13 Global Memory Control Register
 *
 * [19:16] GM size (0/512KB, 1/1MB, 2/2MB, 3/4M, 4/8MB, 5/16MB, 7/64MB)
 * [3:0] remap enable
 */
#define _GET_GM_SIZE(val_32)                        (((val_32) >> 16) & 0xF)

#define DEBUG_CLUSTER_GM_CONTROL                    0x2070

#define ZHOUYI_V3_MAX_REG_OFFSET                    0x322C

u64 get_gm_size(u32 val);
struct aipu_operations *get_zhouyi_v3_ops(void);
struct aipu_priv_operations *get_v3_priv_ops(void);

#endif /* __V3_H__ */
