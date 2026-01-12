/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_TCB_H__
#define __AIPU_TCB_H__

#include <linux/types.h>

/**
 * #reserved[8:11], dtcm_en[7], rd_en[6], wr_en[5], size[0:4]
 */
#define ASID_WR BIT(5)
#define ASID_RD BIT(6)
#define DTCM_EN BIT(7)

enum {
	TCB_INIT = 0,
	TCB_TASK,
	TCB_LOOP
};

enum {
	TCB_NO_DEP = 0,
	TCB_IMMIDIATE_DEP,
	TCB_PRE_ALL_DEP
};

union addr64_t {
	u64 v64;
	struct {
		u32 lo;
		u32 hi;
	} v32;
};

struct smmu_conf_t {
	u32 ctrl;
	u32 remap;
	struct {
		u32 ctrl0;
		u32 ctrl1;
	} segs[4];
};

struct aipu_tcb {
	u32 flag;
	volatile u32 next;
	union {
		struct {
			u32 loop_count;
			u32 spc;
			u32 interrupt_en;
			u16 group_id;
			u16 grid_id;
			u16 rsvd0;
			u16 task_id;
			u16 grid_dim_x;
			u16 grid_dim_y;
			u16 grid_dim_z;
			u16 group_dim_x;
			u16 group_dim_y;
			u16 group_dim_z;
			u16 group_id_x;
			u16 group_id_y;
			u16 group_id_z;
			u16 task_id_x;
			u16 task_id_y;
			u16 task_id_z;
			u32 sp;
			u32 pp;
			u32 dp;
			u32 cp;
			u32 pprint;
			u32 pprofiler;
			u16 core_id;
			u16 cluster_id;
			u16 rsvd1;
			u16 tec_id;
			u32 fmdp;
			u32 tap;
			u32 dap;
			u32 pap;
			u32 idp;
			u32 dsize;
			u32 tcbp;
			u32 global_param;
			u32 rsvd2[3];
		} task;
		union {
			struct {
				u32 rsvd0[2];
				u32 gm_ctrl;
				u16 grid_id;
				u16 rsvd1;
				u32 gm_rgnx_ctrl[2];
				union addr64_t gm_rgnx_addr[2];
				union addr64_t asids[4];
				union addr64_t dtcm_addr;
				u32 rsvd2[10];
			} grid;

			struct {
				u32 rsvd0[4];
				struct smmu_conf_t smmu;
				u32 rsvd1[6];
				struct smmu_conf_t next_core_smmu;
			} group;
		};
	};
};

#define TCB_FLAG_TASK_TYPE_INIT        0
#define TCB_FLAG_TASK_TYPE_TASK        1
#define TCB_FLAG_TASK_TYPE_LOOP_TASK   2
#define TCB_FLAG_END_TYPE_GRID_END     BIT(7)
#define TCB_FLAG_END_TYPE_GROUP_END    BIT(6)
#define INTERRUPT_TEC                  (1 << 8)

#define GET_SEGMMU_CONFIG_CNT(flag)     (((flag) >> 16) & 0x1f)
#define IS_INIT_TCB(flag)               (((flag) & 0x3) == TCB_FLAG_TASK_TYPE_INIT)
#define IS_TASK_TCB(flag)               (((flag) & 0x3) == TCB_FLAG_TASK_TYPE_TASK)
#define IS_GRID_END(flag)               ((flag) & TCB_FLAG_END_TYPE_GRID_END)

struct aipu_tcb_v32
{
    uint32_t flag;
    union {
        struct
        {
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
            struct
            {
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

            struct
            {
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

#endif /* __AIPU_TCB_H__ */
