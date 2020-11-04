/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __AIPU_TCB_H__
#define __AIPU_TCB_H__

enum
{
    TCB_INIT = 0,
    TCB_TASK,
    TCB_LOOP
};

enum
{
    TCB_NO_DEP = 0,
    TCB_IMMIDIATE_DEP,
    TCB_PRE_ALL_DEP
};

union addr64_t
{
    u64 v64;
    struct
    {
        u32 lo;
        u32 hi;
    } v32;
};

struct smmu_conf_t
{
    u32 ctrl;
    u32 remap;
    struct
    {
        u32 ctrl0;
        u32 ctrl1;
    } segs[4];
};

struct aipu_tcb
{
    u32 flag;
    u32 next;
    union
    {
        struct
        {
            u32 loop_count;
            u32 spc;
            u32 interrupt;
            u16 groupid;
            u16 gridid;
            u16 rsvd0;
            u16 taskid;
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
            u16 coreid;
            u16 clusterid;
            u16 rsvd1;
            u16 tecid;
            u32 fmdp;
            u32 tap;
            u32 dap;
            u32 pap;
            u32 idp;
            u32 dsize;
            u32 tcb_asid_offset;
            u32 rsvd2[4];
        } noninit;
        union
        {
            struct
            {
                u32 rsvd0[2];
                u32 gm_ctrl;
                u16 grid_id;
                u16 rsvd1;
                u32 gm_rgnx_ctrl[2];
                union addr64_t gm_rgnx_addr[2];
                union addr64_t asids[4];
                union addr64_t dtcm_addr;
                u32 rsvd2[10];
            } clst;

            struct
            {
                u32 rsvd0[4];
                struct smmu_conf_t smmu;
                u32 rsvd1[6];
                struct smmu_conf_t next_core_smmu;
            } core;
        } init;
    } __data;
};

#endif /* __AIPU_TCB_H__ */
