// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef __TCB_H__
#define __TCB_H__

#include <stdint.h>

namespace aipudrv {
inline uint32_t tcb_task_type(uint32_t flag) { return flag & 0xF; }

inline uint32_t tcb_dep_type(uint32_t flag) { return flag & 0x30; }

namespace tcb_ctl {
constexpr uint32_t TCB_LEN = (128u);
/**
 * #reserved[8:11], dtcm_en[7], rd_en[6], wr_en[5], size[0:4]
 */
constexpr uint32_t ASID_WR = (1 << 5);
constexpr uint32_t ASID_RD = (1 << 6);

constexpr uint32_t FLAG_DEP_TYPE_NONE = 0;
constexpr uint32_t FLAG_DEP_TYPE_PRE_ALL = (2 << 4);

constexpr uint32_t FLAG_END_TYPE_GROUP_END = (1 << 6);
constexpr uint32_t FLAG_END_TYPE_GRID_END = (1 << 7);

constexpr uint32_t GM_REGION_CTRL_SYNC_TO_GM = (1 << 30);
constexpr uint32_t GM_REGION_CTRL_SYNC_TO_DDR = (2UL << 30);
constexpr uint32_t GM_REGION_CTRL_IGNORE_CFG = (3UL << 30);

/* v3 */
constexpr uint32_t FLAG_TASK_TYPE_INIT = 0;
constexpr uint32_t FLAG_TASK_TYPE_TASK_V3 = 1;

constexpr uint32_t FLAG_DEP_TYPE_IMMEDIATE = (1 << 4);

constexpr uint32_t FLAG_END_TYPE_END_WITH_DESTROY = (1 << 8);

constexpr uint32_t EN_INTERRUPT_DONE = 1;
constexpr uint32_t EN_INTERRUPT_EXCEPTION = (1 << 2);
constexpr uint32_t EN_INTERRUPT_FAULT = (1 << 3);
constexpr uint32_t EN_INTERRUPT_ERROR = (1 << 4);
constexpr uint32_t EN_INTERRUPT_SIGNAL = (1 << 5);
constexpr uint32_t EN_INTERRUPT_ALL_TYPE_V3 =
    (EN_INTERRUPT_DONE | EN_INTERRUPT_EXCEPTION | EN_INTERRUPT_FAULT |
     EN_INTERRUPT_ERROR | EN_INTERRUPT_SIGNAL);

constexpr uint32_t EN_INTERRUPT_TEC = (1 << 8);
constexpr uint32_t EN_INTERRUPT_CORE = (1 << 9);
constexpr uint32_t EN_INTERRUPT_CLUSTER = (1 << 10);

constexpr uint32_t GM_CTRL_REMAP_REGION0_EN = (0x1);
constexpr uint32_t GM_CTRL_REMAP_BOTH_REGION_EN = (0x2);

/* v3_2 */
constexpr uint32_t FLAG_TASK_TYPE_GRID_INIT = 0;
constexpr uint32_t FLAG_TASK_TYPE_GROUP_INIT = 1;
constexpr uint32_t FLAG_TASK_TYPE_TASK_V32 = 2;

constexpr uint32_t FLAG_DEP_TYPE_GROUP = (1 << 4);

constexpr uint32_t FLAG_END_TYPE_POOL_END = (1 << 8);

constexpr uint32_t FLAG_GRID_INIT = (1 << 21);
constexpr uint32_t FLAG_L2D_FLUSH = (1 << 22);

/* group tcb interrupt */
constexpr uint32_t EN_INTERRUPT_GROUP_DONE = (1 << 0);

constexpr uint32_t EN_GROUP_DEPEND = (1 << 15);

constexpr uint32_t EN_INTERRUPT_TEC_DONE = (1 << 0);
constexpr uint32_t EN_INTERRUPT_TEC_SIGNAL = (1 << 1);
constexpr uint32_t EN_INTERRUPT_TEC_EXCEPTION = (1 << 2);
constexpr uint32_t EN_INTERRUPT_TEC_FAULT = (1 << 3);
constexpr uint32_t EN_INTERRUPT_POOL_ERROR = (1 << 4);
constexpr uint32_t EN_INTERRUPT_TIMEOUT = (1 << 5);
/* default close tec done */
constexpr uint32_t EN_INTERRUPT_ALL_TYPE_V32 =
    (EN_INTERRUPT_TEC_SIGNAL | EN_INTERRUPT_TEC_EXCEPTION |
     EN_INTERRUPT_TEC_FAULT | EN_INTERRUPT_POOL_ERROR | EN_INTERRUPT_TIMEOUT);

/* grid tcb interrupt */
constexpr uint32_t EN_INTERRUPT_GRID_ALL = (1 << 0 | 1 << 3);

constexpr uint32_t GM_CTRL_REMAP_EN = (0x1);
}; // namespace tcb_ctl

namespace tcb_v3 {
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
}; /* namespace tcb_v3 */

namespace tcb_v3_2 {
union config64_t {
  uint64_t v64;
  struct {
    uint32_t ctrl0;
    uint32_t ctrl1;
  } v32;
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
}; /* namespace tcb_v3_2 */
}; // namespace aipudrv

#endif //!__TCB_H__
