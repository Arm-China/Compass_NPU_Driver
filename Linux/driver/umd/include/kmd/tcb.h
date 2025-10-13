// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef __TCB_H__
#define __TCB_H__

#include <stdint.h>

namespace aipudrv {
/**
 * #reserved[8:11], dtcm_en[7], rd_en[6], wr_en[5], size[0:4]
 */
#define ZY_ASID_WR (1 << 5)
#define ZY_ASID_RD (1 << 6)
#define ZY_DTCM_EN (1 << 7)

#if (defined ZHOUYI_V3)
enum class TCBType { TCB_INIT = 0, TCB_TASK, TCB_LOOP };

enum class TCBDepType { TCB_NO_DEP = 0, TCB_IMMIDIATE_DEP, TCB_PRE_ALL_DEP };

union addr64_t {
  uint64_t v64;
  struct {
    uint32_t lo;
    uint32_t hi;
  } v32;
};

struct smmu_conf_t {
  uint32_t ctrl;
  uint32_t remap;
  struct {
    uint32_t ctrl0;
    uint32_t ctrl1;
  } segs[4];
};

struct tcb_t {
  uint32_t flag;
  uint32_t next;
  union {
    struct {
      uint32_t loop_count;
      uint32_t spc;
      uint32_t interrupt_en;
      uint16_t group_id;
      uint16_t grid_id;
      uint16_t rsvd0;
      uint16_t task_id;
      uint16_t grid_dim_x;
      uint16_t grid_dim_y;
      uint16_t grid_dim_z;
      uint16_t group_dim_x;
      uint16_t group_dim_y;
      uint16_t group_dim_z;
      uint16_t group_id_x;
      uint16_t group_id_y;
      uint16_t group_id_z;
      uint16_t task_id_x;
      uint16_t task_id_y;
      uint16_t task_id_z;
      uint32_t sp;
      uint32_t pp;
      uint32_t dp;
      uint32_t cp;
      uint32_t pprint;
      uint32_t pprofiler;
      uint16_t core_id;
      uint16_t cluster_id;
      uint16_t rsvd1;
      uint16_t tec_id;
      uint32_t fmdp;
      uint32_t tap;
      uint32_t dap;
      uint32_t pap;
      uint32_t idp;
      uint32_t dsize;
      uint32_t tcbp;
      uint32_t global_param;
      uint32_t rsvd2[3];
    } task;
    union {
      struct {
        uint32_t rsvd0[2];
        uint32_t gm_ctrl;
        uint32_t grid_id;
        uint32_t gm_rgnx_ctrl[2];
        addr64_t gm_rgnx_addr[2];
        addr64_t asids[4];
        addr64_t dtcm_addr;
        uint32_t rsvd2[10];
      } grid;

      struct {
        uint32_t rsvd0[4];
        smmu_conf_t smmu;
        uint32_t rsvd1[6];
        smmu_conf_t next_core_smmu;
      } group;
    };
  };
};

#define TCB_FLAG_TASK_TYPE(flag) (flag & 0xF)
#define TCB_FLAG_TASK_TYPE_INIT 0
#define TCB_FLAG_TASK_TYPE_TASK 1
#define TCB_FLAG_TASK_TYPE_LOOP_TASK 2

#define TCB_FLAG_DEP_TYPE_NONE 0
#define TCB_FLAG_DEP_TYPE_IMMEDIATE (1 << 4)
#define TCB_FLAG_DEP_TYPE_PRE_ALL (2 << 4)

#define TCB_FLAG_END_TYPE_NOT_END 0
#define TCB_FLAG_END_TYPE_GROUP_END (1 << 6)
#define TCB_FLAG_END_TYPE_GRID_END (1 << 7)
#define TCB_FLAG_END_TYPE_END_WITH_DESTROY (1 << 8)

#define EN_INTERRUPT_DONE 1
#define EN_INTERRUPT_EXCEPTION (1 << 2)
#define EN_INTERRUPT_FAULT (1 << 3)
#define EN_INTERRUPT_ERROR (1 << 4)
#define EN_INTERRUPT_SIGNAL (1 << 5)
#define EN_INTERRUPT_ALL_TYPE                                                  \
  (EN_INTERRUPT_DONE | EN_INTERRUPT_EXCEPTION | EN_INTERRUPT_FAULT |           \
   EN_INTERRUPT_ERROR | EN_INTERRUPT_SIGNAL)
#define EN_INTERRUPT_TEC (1 << 8)
#define EN_INTERRUPT_CORE (1 << 9)
#define EN_INTERRUPT_CLUSTER (1 << 10)
#define EN_INTERRUPT_POOL (1 << 11)

/**
 * GM data sync direction
 * GM_REGION_CTRL_SYNC_TO_GM: DDR to GM region
 * GM_REGION_CTRL_SYNC_TO_DDR: GM region to DDR
 */
#define GM_CTRL_TSM_IGNORE_CFG (0xf)
#define GM_REGION_CTRL_ONLY_UPDATE_REG (0 << 30)
#define GM_REGION_CTRL_SYNC_TO_GM (1 << 30)
#define GM_REGION_CTRL_SYNC_TO_DDR (2UL << 30)
#define GM_REGION_CTRL_IGNORE_CFG (3UL << 30)

#define GM_CTRL_REMAP_BOTH_REGION_DEN (0x0)
#define GM_CTRL_REMAP_REGION0_EN (0x1)
#define GM_CTRL_REMAP_BOTH_REGION_EN (0x2)

#else

enum class TCBType { TCB_GRID_INIT = 0, TCB_GROUP_INIT, TCB_TASK };

enum class TCBDepType { TCB_NO_DEP = 0, TCB_GROUP_DEP, TCB_PRE_ALL_DEP };

union addr64_t {
  uint64_t v64;
  struct {
    uint32_t lo;
    uint32_t hi;
  } v32;
};

union config64_t {
  uint64_t v64;
  struct {
    uint32_t ctrl0;
    uint32_t ctrl1;
  } v32;
};

struct smmu_conf_t {
  uint32_t ctrl;
  uint32_t remap;
  struct {
    uint32_t ctrl0;
    uint32_t ctrl1;
  } segs[4];
};

struct tcb_t {
  uint32_t flag;
  union {
    struct {
      uint32_t next;
      uint32_t rsvd0[1];
      uint32_t spc;
      uint32_t interrupt_en;
      uint16_t group_id;
      uint16_t grid_id;
      uint16_t task_id;
      uint16_t warmup_len;
      uint16_t grid_dim_x;
      uint16_t grid_dim_y;
      uint16_t grid_dim_z;
      uint16_t group_dim_x;
      uint16_t group_dim_y;
      uint16_t group_dim_z;
      uint16_t group_id_x;
      uint16_t group_id_y;
      uint16_t group_id_z;
      uint16_t task_id_x;
      uint16_t task_id_y;
      uint16_t task_id_z;
      uint32_t sp;
      uint32_t pp;
      uint32_t dp;
      uint32_t cp;
      uint32_t pprint;
      uint32_t pprofiler;
      uint16_t core_id;
      uint16_t cluster_id;
      uint16_t rsvd1;
      uint16_t tec_id;
      uint32_t rsvd2[5];
      uint32_t dsize;
      uint32_t tcbp;
      uint32_t global_param;
      uint32_t rsvd3;
    } task;

    union {
      struct {
        uint32_t rsvd0;
        uint32_t group_num;
        uint32_t rsvd1;
        uint32_t interrupt_en;
        uint16_t group_id;
        uint16_t grid_id;
        uint32_t rsvd2[6];
        uint32_t gm_ctrl;
        uint32_t gm_sync;
        uint32_t gm_addr_low;
        uint32_t gm_addr_high;
        uint32_t rsvd3[14];
      } grid;

      struct {
        uint32_t segmmu_ctrl;
        uint32_t segmmu_remap_ctrl0;
        uint32_t segmmu_remap_ctrl1;
        uint32_t interrupt_en;
        uint16_t group_id;
        uint16_t grid_id;
        uint32_t segmmu_seg_ctrl[16];
        uint32_t asids[8];
      } group;
    };
  };
  uint16_t group_deps[4];
};

#define TCB_FLAG_TASK_TYPE(flag) (flag & 0xF)
#define TCB_FLAG_TASK_TYPE_GRID_INIT 0
#define TCB_FLAG_TASK_TYPE_GROUP_INIT 1
#define TCB_FLAG_TASK_TYPE_TASK 2

#define TCB_FLAG_DEP_TYPE(flag) (flag & 0x30)
#define TCB_FLAG_DEP_TYPE_NONE 0
#define TCB_FLAG_DEP_TYPE_GROUP (1 << 4)
#define TCB_FLAG_DEP_TYPE_PRE_ALL (2 << 4)

#define TCB_FLAG_END_TYPE_NOT_END 0
#define TCB_FLAG_END_TYPE_GROUP_END (1 << 6)
#define TCB_FLAG_END_TYPE_GRID_END (1 << 7)
#define TCB_FLAG_END_TYPE_POOL_END (1 << 8)

#define TCB_FLAG_CORE_NUM(n) ((n & 0xF) << 16)
#define TCB_FLAG_BROADCAST_START (1 << 20)
#define TCB_FLAG_GRID_INIT (1 << 21)
#define TCB_FLAG_L2D_FLUSH (1 << 22)

/* task tcb interrupt */
#define EN_INTERRUPT_TEC_DONE (1 << 0)
#define EN_INTERRUPT_TEC_SIGNAL (1 << 1)
#define EN_INTERRUPT_TEC_EXCEPTION (1 << 2)
#define EN_INTERRUPT_TEC_FAULT (1 << 3)
#define EN_INTERRUPT_TEC_ALL                                                   \
  (EN_INTERRUPT_TEC_DONE | EN_INTERRUPT_TEC_SIGNAL |                           \
   EN_INTERRUPT_TEC_EXCEPTION | EN_INTERRUPT_TEC_FAULT)

/* grid tcb interrupt */
#define EN_INTERRUPT_GRID_DONE (1 << 0)
#define EN_INTERRUPT_GRID_GM_FALUT (1 << 3)
#define EN_INTERRUPT_GRID_ALL                                                  \
  (EN_INTERRUPT_GRID_DONE | EN_INTERRUPT_GRID_GM_FALUT)

/* group tcb interrupt */
#define EN_INTERRUPT_GROUP_DONE (1 << 0)

#define GM_CTRL_REMAP_EN (0x1)
#define GM_CTRL_REMAP_MODE_RES_PRIOR (0x2)
#define GM_MAX_SIZE (8 << 20)

#define GM_SYNC_ONLY_UPDATE_REG (0 << 30)
#define GM_SYNC_DDR_TO_GM (1UL << 30)
#define GM_SYNC_IGNORE_CFG (3UL << 30)

#define EN_GROUP_DEPEND (1 << 15)

#endif
} // namespace aipudrv

#endif //!__TCB_H__
