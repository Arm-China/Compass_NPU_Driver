// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  job_v3.cpp
 * @brief AIPU User Mode Driver (UMD) x2 job module implementation
 */

#include <cstring>
#include <assert.h>
#include "job_v3.h"
#include "graph_v3.h"
#include "parser_base.h"
#include "utils/helper.h"

#if defined(SIMULATION) && defined(ZHOUYI_V3)
#include "simulator/simulator_v3.h"
#endif
aipudrv::JobV3::JobV3(MainContext* ctx, const GraphBase& graph, DeviceBase* dev, aipu_create_job_cfg_t *config):
    JobBase(ctx, graph, dev), m_partition_id(config->partition_id), m_qos(config->qos_level)
{
    m_tcbs.reset();
    m_init_tcb.init(0);

#if defined(SIMULATION)
    /**
     * initialize an invalid cmdpool id, set it on scheduling to sim
     */
    m_bind_cmdpool_id = 0xffffffff;
#endif

    set_job_params(get_graph().m_subgraphs.size(), m_dev->tec_cnt_per_core(config->partition_id),
        get_graph().m_remap_flag);

    /**
     * allocate buffer for backup tcb chain, this is just for support run the same job
     * multiple times based on the same one Job struct created by aipu_create_job()
     */
    m_backup_tcb.reset(new char[m_tot_tcb_cnt * sizeof(tcb_t)]);

    m_segmmu_num = get_graph().m_segmmu_num;
    m_gm = new GM_V3(*this);
}

aipudrv::JobV3::~JobV3()
{
    delete m_gm;
}

void aipudrv::JobV3::set_job_params(uint32_t sg_cnt, uint32_t task_per_sg, uint32_t remap)
{
    m_sg_cnt = sg_cnt;
    m_task_per_sg = task_per_sg;
    m_remap_flag = remap;

    /**
     * currently: 1 init-tcb + n task-tcb + 1 gm_to_ddr sync-tcb
     *
     * - if output buffer exists in GM, the last gm_to_ddr sync-tcb is needed.
     * - if using SegMMU, m_tot_tcb_cnt will be extended accordingly.
     */
    m_tot_tcb_cnt = m_sg_cnt * m_task_per_sg + 1 + 1;
}

void aipudrv::JobV3::setup_gm_sync_from_ddr(tcb_t *tcb)
{
    uint32_t gm_region_idx = 0;

    if (!m_mem->is_gm_enable())
        return;

    if (!m_gm->gm_need_remap())
        return;

    if (m_qos == AIPU_JOB_QOS_SLOW)
        gm_region_idx = 0;
    else if (m_qos == AIPU_JOB_QOS_HIGH)
    {
        if (m_mem->is_both_gm_region_enable())
            gm_region_idx = 1;
        else
            return;
    }

    if (m_mem->is_both_gm_region_enable())
        tcb->gm_ctl = GM_CTRL_REMAP_BOTH_REGION_EN;
    else
        tcb->gm_ctl = GM_CTRL_REMAP_REGION0_EN;

    tcb->gm_rgnx_addr[0].v64 = 0;
    tcb->gm_rgnx_addr[1].v64 = 0;
    tcb->gm_rgnx_addr[gm_region_idx].v64 = m_gm->m_gm_base;

    tcb->gm_rgnx_ctrl[0] = GM_REGION_CTRL_IGNORE_CFG;
    tcb->gm_rgnx_ctrl[1] = GM_REGION_CTRL_IGNORE_CFG;
    if (m_gm->m_gm_base != 0)
        tcb->gm_rgnx_ctrl[gm_region_idx] = 0;

    if (m_gm->m_gm_buf_map_size[EM_GM_BUF_INPUT] != 0)
    {
        tcb->gm_rgnx_ctrl[gm_region_idx] = GM_REGION_CTRL_SYNC_TO_GM;
        tcb->gm_rgnx_ctrl[gm_region_idx] |= get_low_32(m_gm->m_gm_buf_base_pa[EM_GM_BUF_INPUT]
            - m_gm->m_gm_base) & 0xffff000;
        tcb->gm_rgnx_ctrl[gm_region_idx] |= (m_gm->m_gm_buf_map_size[EM_GM_BUF_INPUT] >> 12) & 0xfff;
    }
}

void aipudrv::JobV3::setup_gm_sync_to_ddr(tcb_t *tcb)
{
    uint32_t gm_region_idx = 0;
    tcb_t pre_tcb;

    if (!m_mem->is_gm_enable())
        return;

    if (!m_gm->gm_need_sync_out())
        return;

    if (m_qos == AIPU_JOB_QOS_SLOW)
        gm_region_idx = 0;
    else if (m_qos == AIPU_JOB_QOS_HIGH) {
        if (m_mem->is_both_gm_region_enable())
            gm_region_idx = 1;
        else
           return;
    }

    /* modify the last task tcb and link to GM sync tcb */
    m_mem->read(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 2),
        &pre_tcb, sizeof(tcb_t));
    pre_tcb.next = get_low_32(m_init_tcb.pa + sizeof(*tcb) * (m_tot_tcb_cnt - 1));
    pre_tcb.flag &= ~(TCB_FLAG_END_TYPE_GROUP_END
                    | TCB_FLAG_END_TYPE_GRID_END
                    | TCB_FLAG_END_TYPE_END_WITH_DESTROY);
    m_mem->write(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 2),
        (const char*)&pre_tcb, sizeof(tcb_t));

    /* config GM sync tcb: GM->DDR */
    tcb->next = 0;
    tcb->flag = TCB_FLAG_DEP_TYPE_PRE_ALL
                | TCB_FLAG_END_TYPE_GROUP_END
                | TCB_FLAG_END_TYPE_GRID_END;
    tcb->igrid_id = pre_tcb.gridid;
    tcb->igroup_id = pre_tcb.groupid;
    tcb->gm_rgnx_addr[0].v64 = 0;
    tcb->gm_rgnx_addr[1].v64 = 0;
    tcb->gm_rgnx_addr[gm_region_idx].v64 = m_gm->m_gm_base;
    tcb->gm_rgnx_ctrl[0] = GM_REGION_CTRL_IGNORE_CFG;
    tcb->gm_rgnx_ctrl[1] = GM_REGION_CTRL_IGNORE_CFG;
    tcb->gm_rgnx_ctrl[gm_region_idx] = GM_REGION_CTRL_SYNC_TO_DDR;
    tcb->gm_rgnx_ctrl[gm_region_idx] |= get_low_32(m_gm->m_gm_buf_base_pa[EM_GM_BUF_OUTPUT]
        - m_gm->m_gm_base) & 0xffff000;
    tcb->gm_rgnx_ctrl[gm_region_idx] |= (m_gm->m_gm_buf_map_size[EM_GM_BUF_OUTPUT] >> 12) & 0xfff;
    tcb->gm_ctl = GM_CTRL_TSM_IGNORE_CFG;

    m_mem->write(m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 1),
        (const char*)tcb, sizeof(tcb_t));
}

#define SEGMMU_MME_CTRL_EN (1 << 0)
#define SEGMMU_REMAP_SHARE_EN  (1 << 5)
#define SEGMMU_IN_ASID_WR (1 << 0)
#define SEGMMU_IN_ASID_RD (1 << 1)
inline uint32_t get_segmmu_and_reg_idx(uint32_t offset)
{
    uint32_t segmmu_idx = 0;

    // get segmmu index
    if (offset < 8)
        segmmu_idx = 0;
    else if (offset < 16)
        segmmu_idx = 1;
    else if (offset < 24)
        segmmu_idx = 2;
    else if (offset < 32)
        segmmu_idx = 3;
    else
        LOG(LOG_WARN, "SegMMU entry offset is invalid\n");

    return segmmu_idx;
}

aipu_status_t aipudrv::JobV3::setup_segmmu(SubGraphTask &sg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const std::vector<struct GraphParamMapLoadDesc>& param_map =
        get_graph().m_subgraphs[0].param_map;
    std::vector<BufferDesc>& reuse_buf  = sg.reuses;
    uint32_t *start = nullptr, *entry = nullptr;
    uint32_t total_segmmu_sz = 0;
    SegMMUConfig *segmmu = nullptr;

    if (m_segmmu_num == 0)
        goto out;

    m_segmmu_sec = get_graph().m_bsegmmu;
    start = (uint32_t *)m_segmmu_sec.va;
    segmmu = (SegMMUConfig *)start;
    total_segmmu_sz = sizeof(SegMMUConfig) * m_segmmu_num;

    for (uint32_t i = 0; i < param_map.size(); i++)
    {
        if (param_map[i].buf_type != SECTION_TYPE_SEGMMU)
            continue;

        uint32_t buf_idx = param_map[i].ref_section_iter;
        uint32_t sec_offset = param_map[i].sub_section_offset;
        uint32_t offset_in_map = param_map[i].offset_in_map;
        uint32_t segmmu_core_idx = offset_in_map / sizeof(SegMMUConfig);
        uint32_t segmmu_reg_idx = get_segmmu_and_reg_idx(offset_in_map % sizeof(SegMMUConfig));

        if (offset_in_map < total_segmmu_sz)
        {
            uint32_t seg_limit_sz = 0;

            entry = start + offset_in_map;
            seg_limit_sz = (*entry & 0x7f) << 14; // 16KB unit

            /* buffer size is orght to larger than segmmu_limit */
            if (reuse_buf[buf_idx].size > seg_limit_sz)
            {
                *entry &= ~0xffffc000;
                *entry |= (reuse_buf[buf_idx].pa + sec_offset) & 0xffffc000;
                segmmu[segmmu_core_idx].SegMMU_ctl |= SEGMMU_REMAP_SHARE_EN | SEGMMU_MME_CTRL_EN;

                /**
                 * config RW (seg0_in_ASID) for segmmu, actually this will
                 * be ignored due to SEGMMU_REMAP_SHARE_EN (set)
                 */
                segmmu[segmmu_core_idx].SegMMU_remap |= (SEGMMU_IN_ASID_RD | SEGMMU_IN_ASID_WR)
                    << (16 + 4 * segmmu_reg_idx);
            } else {
                LOG(LOG_ERR, "SegMMU: remap buffer size too small\n");
                ret = AIPU_STATUS_ERROR_INVALID_SEGMMU;
                goto out;
            }
        } else {
            LOG(LOG_ERR, "SegMMU: entry offset beyond scope\n");
            ret = AIPU_STATUS_ERROR_INVALID_SEGMMU;
            goto out;
        }
    }

out:
    return ret;
}

aipu_status_t aipudrv::JobV3::setup_rodata_sg(uint32_t sg_id,
    const std::vector<struct GraphParamMapLoadDesc>& param_map,
    std::vector<BufferDesc>& reuse_buf, std::vector<BufferDesc>& static_buf)
{
    BufferDesc rodata, dcr;
    // const std::vector<struct GraphParamMapLoadDesc>& param_map =
    //     get_graph().m_subgraphs[sg_id].param_map;
    // std::vector<BufferDesc>& reuse_buf  = m_sg_job[sg_id].reuses;
    // std::vector<BufferDesc>& static_buf = m_sg_job[sg_id].weights;

    rodata.init(0, m_rodata.pa, m_rodata.size, m_rodata.req_size);
    dcr.init(0, m_descriptor.pa, m_descriptor.size, m_descriptor.req_size);
    return setup_rodata(param_map, reuse_buf, static_buf, rodata, dcr);
}

aipu_status_t aipudrv::JobV3::alloc_subgraph_buffers()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    SubGraphTask sg;

    /* allocate subgraph buffers */
    for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++)
    {
        sg.reset(sg_idx);

        /* each subgraph has private buffer core-accessed */
        for (uint32_t k = 0; k < get_graph().m_subgraphs[sg_idx].private_buffers.size(); k++)
        {
            const GraphSectionDesc &section_desc = get_graph().m_subgraphs[sg_idx].private_buffers[k];
            BufferDesc buf;
            buf.reset();

            /* FIX ME: type */
            if ((section_desc.support_dma_buf == false) && (section_desc.size != 0))
            {
                std::string buf_name = "reuse_priv_" + std::to_string(k);

                ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &buf, buf_name.c_str());
                if (AIPU_STATUS_SUCCESS != ret)
                    goto out;
            }

            if (m_dump_reuse)
                m_mem->mem_bzero(buf.pa, buf.size);

            sg.reuse_priv_buffers.push_back(buf);
        }

        /* allocate reuse buffers, all subgraphs share one copy of reuse buffers */
        if (sg.id == 0)
        {
            for (uint32_t k = 0; k < get_graph().m_subgraphs[0].reuse_sections.size(); k++)
            {
                const GraphSectionDesc &section_desc = get_graph().m_subgraphs[0].reuse_sections[k];
                BufferDesc buf;
                buf.reset();

                /* FIX ME: type */
                if ((section_desc.support_dma_buf == false) && (section_desc.size != 0))
                {
                    std::string buf_name = "reuse_" + std::to_string(k);

                    /* handle buffer if allocated from GM */
                    if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE))
                    {
                        ret = m_gm->gm_malloc(sg_idx, k, GM_BUF_TYPE_REUSE, buf_name, buf);
                    } else {
                        ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &buf, buf_name.c_str());
                    }

                    if (AIPU_STATUS_SUCCESS != ret)
                        goto out;
                }

                if (m_dump_reuse)
                    m_mem->mem_bzero(buf.pa, buf.size);

                sg.reuses.push_back(buf);
            }

            /* init task weights address */
            for (uint32_t w = 0; w < get_graph().m_subgraphs[0].static_sections.size(); w++)
            {
                const GraphSectionDesc &section_desc = get_graph().m_subgraphs[0].static_sections[w];
                BufferDesc buf;

                if (m_gm->gm_is_gm_buffer(w, GM_BUF_TYPE_WEIGHT))
                {
                    std::string buf_name = "weight_" + std::to_string(w);
                    ret = m_gm->gm_malloc(sg_idx, w, GM_BUF_TYPE_WEIGHT, buf_name, buf);
                    if (AIPU_STATUS_SUCCESS != ret)
                        goto out;

                    if (buf.ram_region == AIPU_BUF_REGION_DEFAULT)
                    {
                        /**
                         * allocate buffer fail from GM, use weight's original DDR buffer
                         */
                        buf.init(get_graph().m_weight.asid_base, get_graph().m_weight.pa + section_desc.offset,
                            section_desc.size, section_desc.size);
                    } else {
                        /**
                         * copy weight from original buffer to GM DDR buffer,
                         * then sync GM DDR buffer to GM SRAM via TCB sync config.
                         */
                        m_mem->write(buf.pa, section_desc.load_src, section_desc.size);
                    }
                } else {
                    buf.init(get_graph().m_weight.asid_base, get_graph().m_weight.pa + section_desc.offset,
                        section_desc.size, section_desc.size);
                }
                sg.weights.push_back(buf);
            }
        }

        m_sg_job.push_back(sg);
    }

out:
    return ret;
}

aipu_status_t aipudrv::JobV3::init_per_task_data()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    for (uint32_t i = 0; i < m_sg_cnt; i++)
    {
        SubGraphTask &sg = m_sg_job[i];

        /* 1 init per-task data structs */
        for (uint32_t j = 0; j < m_task_per_sg; j++)
        {
            Task task;
            memset((void *)&task, 0, sizeof(task));

            /* 1.1. init task tcb */
            task.tcb.init(m_tcbs.pa + (i * m_task_per_sg + j + 1)* sizeof(tcb_t));

            /* 1.2. allocate task stack */
            ret = m_mem->malloc(get_graph().m_subgraphs[0].stack_size, get_graph().m_subgraphs[0].stack_align_in_page,
                &task.stack, "stack");
            if (AIPU_STATUS_SUCCESS != ret)
                goto out;

            /* 1.3. allocate and load task dp */
            if (get_graph().m_bdata.size != 0)
            {
                ret = m_mem->malloc(get_graph().m_bdata.size, 0, &task.dp_cc, "data_cc");
                if (AIPU_STATUS_SUCCESS != ret)
                    goto out;

                m_mem->write(task.dp_cc.pa, get_graph().m_bdata.va, get_graph().m_bdata.size);
            }
            sg.tasks.push_back(task);
        }
    }

out:
    return ret;
}

aipu_status_t aipudrv::JobV3::alloc_load_job_buffers()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    SubGraphTask sg;

    /* 1. allocate and load job rodata */
    if (get_graph().m_brodata.size != 0)
    {
        ret = m_mem->malloc(get_graph().m_brodata.size, 0, &m_rodata, "rodata");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        m_mem->write(m_rodata.pa, get_graph().m_brodata.va, get_graph().m_brodata.size);
    }

    /* 2. allocate and load job descriptor */
    if (get_graph().m_bdesc.size != 0)
    {
        ret = m_mem->malloc(get_graph().m_bdesc.size, 0, &m_descriptor, "dcr");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        m_mem->write(m_descriptor.pa, get_graph().m_bdesc.va, get_graph().m_bdesc.size);
    }

    /* 3. allocate and reset job TCBs */
    if (m_segmmu_num > 0)
        m_tot_tcb_cnt += (m_core_cnt + 1) / 2; // extend total tcb number if Segmmu needs

    ret = m_mem->malloc(m_tot_tcb_cnt * sizeof(tcb_t), 0, &m_tcbs, "tcbs");
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    m_mem->zeroize(m_tcbs.pa, m_tot_tcb_cnt * sizeof(tcb_t));
    m_init_tcb.init(m_tcbs.pa);

    /* 4. allocate subgraph buffers */
    ret = alloc_subgraph_buffers();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* 5. init each subgraph's task tcbs */
    ret = init_per_task_data();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* 6. setup rodata & dcr, update entry for all subgraphs in global RO/DCR section */
    ret = setup_rodata_sg(0, get_graph().m_subgraphs[0].param_map, m_sg_job[0].reuses, m_sg_job[0].weights);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* update subgraph private buffers PA in RO/DCR section */
    for (uint32_t sg = 0; sg < m_sg_cnt; sg++)
    {
        std::vector<BufferDesc> invalid_buf;

        LOG(LOG_INFO, "sg: %d\n", sg);
        ret = setup_rodata_sg(sg, get_graph().m_subgraphs[sg].private_buffers_map,
            m_sg_job[sg].reuse_priv_buffers, invalid_buf);
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;
    }

    /* 7. setup remap */
    setup_remap(m_rodata, m_descriptor);

    /* 8. get IO buffer address, all subgraphs share the same copy of reuse buffers */
    create_io_buffers(get_graph().m_subgraphs[0].io, m_sg_job[0].reuses);

    /* 9. parse SegMMU config */
    ret = setup_segmmu(m_sg_job[0]);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

finish:
    if (ret)
    {
        for (uint32_t i = 0; i < m_sg_job.size(); i++)
            free_sg_buffers(m_sg_job[i]);

        free_job_buffers();
    }
    return ret;
}

void aipudrv::JobV3::free_sg_buffers(const SubGraphTask& sg)
{
    for (uint32_t i = 0; i < sg.reuse_priv_buffers.size(); i++)
        m_mem->free(&sg.reuse_priv_buffers[i]);

    /**
     * just exist one copy of reuse buffers for all subgraphs,
     * free only once
     */
    if (sg.id == 0)
    {
        for (uint32_t i = 0; i < sg.reuses.size(); i++)
        {
            if (sg.reuses[i].size != 0)
                m_mem->free(&sg.reuses[i]);
        }
    }

    for (uint32_t i = 0; i < sg.tasks.size(); i++)
    {
        if (sg.tasks[i].stack.size != 0)
            m_mem->free(&sg.tasks[i].stack);

        if (sg.tasks[i].dp_cc.size != 0)
            m_mem->free(&sg.tasks[i].dp_cc);
    }
}

aipu_status_t aipudrv::JobV3::free_job_buffers()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (m_rodata.size != 0)
    {
        m_mem->free(&m_rodata);
        m_rodata.reset();
    }
    if (m_descriptor.size != 0)
    {
        m_mem->free(&m_descriptor);
        m_descriptor.reset();
    }
    if (m_tcbs.size != 0)
    {
        m_mem->free(&m_tcbs, "tcbs");
        m_tcbs.reset();
    }
    m_init_tcb.init(0);

    for (uint32_t i = 0; i < m_sg_job.size(); i++)
        free_sg_buffers(m_sg_job[i]);

    m_sg_job.clear();
    m_inputs.clear();
    m_outputs.clear();
    m_inter_dumps.clear();
    m_profiler.clear();
    m_printf.clear();
    m_layer_counter.clear();

    return ret;
}

aipu_status_t aipudrv::JobV3::setup_tcb_task(uint32_t sg_id, uint32_t grid_id, uint32_t core_id, uint32_t task_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    Task& task = m_sg_job[sg_id].tasks[task_id];
    tcb_t* tcb = new tcb_t;
    TCB* next_tcb = nullptr;

    if (task_id != (m_task_per_sg - 1))
        next_tcb = &m_sg_job[sg_id].tasks[task_id + 1].tcb;
    else if (sg_id != (m_sg_cnt - 1))
        next_tcb = &m_sg_job[sg_id + 1].tasks[0].tcb;
    else
        next_tcb = nullptr;

    memset(tcb, 0, sizeof(tcb_t));
    tcb->flag = TCB_FLAG_TASK_TYPE_TASK;

    if (task_id == (m_task_per_sg - 1))
        tcb->flag |= TCB_FLAG_END_TYPE_GROUP_END;

    if (next_tcb != nullptr)
    {
        tcb->next = get_low_32(next_tcb->pa);
    } else {
        tcb->next = 0;
        tcb->flag |= TCB_FLAG_END_TYPE_GRID_END;
    }

    /* It is assumed that subgraphs are topology sorted. */
    if (task_id == 0)
    {
        switch (get_graph().m_subgraphs[sg_id].precursor_cnt)
        {
            case 0:
                tcb->flag |= TCB_FLAG_DEP_TYPE_NONE;
                break;
            case 1:
                tcb->flag |= TCB_FLAG_DEP_TYPE_IMMEDIATE;
                break;
            case -1:
                tcb->flag |= TCB_FLAG_DEP_TYPE_PRE_ALL;
                break;
            default:
                printf("subgraph %u, precursor_cnt=%d\n", sg_id,
                    get_graph().m_subgraphs[sg_id].precursor_cnt);
                return AIPU_STATUS_ERROR_INVALID_GBIN;
        }
    }

    tcb->interrupt = EN_INTERRUPT_ALL_TYPE | EN_INTERRUPT_CLUSTER | EN_INTERRUPT_CORE;
    tcb->spc = get_low_32(get_graph().m_text.pa + get_graph().m_subgraphs[sg_id].text.offset);
    tcb->gridid = (uint16_t)grid_id;
    tcb->groupid = (uint16_t)core_id;
    tcb->taskid = (uint16_t)task_id;
    tcb->grid_dim_x = 1;
    tcb->grid_dim_y = 1;
    tcb->grid_dim_z = 1;
    tcb->group_dim_x = m_task_per_sg;
    tcb->group_dim_y = 1;
    tcb->group_dim_z = 1;
    tcb->group_id_x = 1;
    tcb->group_id_y = 0;
    tcb->group_id_z = 0;
    tcb->task_id_x = (uint16_t)task_id;
    tcb->task_id_y = 0;
    tcb->task_id_z = 0;
    tcb->tcbp = get_low_32(task.tcb.pa - m_tcbs.asid_base);
    tcb->sp = get_low_32(task.stack.pa);
    tcb->pp = get_low_32(m_rodata.pa + get_graph().m_subgraphs[sg_id].rodata.offset);

    /* const rodata */
    if (get_graph().m_crodata.size > 0)
        tcb->cp = get_low_32(get_graph().m_crodata.pa);

    /* the below arg, not used currenltly */
    tcb->dp = get_low_32(task.dp_cc.pa);

    /* update profile buffer offset according to subgraph index */
    if (m_profiler.size() > 0)
        tcb->pprofiler = get_low_32(m_profiler[0].pa +  get_graph().m_subgraphs[sg_id].profiler_buf_size);

    /* flush TCB to AIPU mem */
    m_mem->write(task.tcb.pa, (const char*)tcb, sizeof(*tcb));

    delete tcb;
    return ret;
}

aipu_status_t aipudrv::JobV3::setup_tcb_sg(uint32_t sg_id, uint32_t grid_id, uint32_t core_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    /* setup task TCBs */
    for (uint32_t t = 0; t < m_task_per_sg; t++)
    {
        ret = setup_tcb_task(sg_id, grid_id, core_id, t);
        if (AIPU_STATUS_SUCCESS != ret)
            return ret;
    }

    return ret;
}

aipu_status_t aipudrv::JobV3::setup_tcbs()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    tcb_t* tcb = new tcb_t;
    uint32_t core_id = 0;

    /* 1. setup init TCB 0 */
    memset(tcb, 0, sizeof(tcb_t));

    if (m_segmmu_num == 0)
        tcb->flag = TCB_FLAG_TASK_TYPE_INIT;
    else
        tcb->flag = ((m_core_cnt & 0x1f) << 16) | TCB_FLAG_TASK_TYPE_INIT;

    tcb->next = get_low_32(m_sg_job[0].tasks[0].tcb.pa);
    tcb->igrid_id = m_grid_id;

    /* 1.1 config GM if need */
    setup_gm_sync_from_ddr(tcb);

    /**
     * 1.2 config ASID
     * #reserved[8:11], dtcm_en[7], rd_en[6], wr_en[5], size[0:4]
     * the ASID ctrl bits locate at low 12 bits in lo address
     */
    for (uint32_t i = 0; i < 4; i++)
    {
        tcb->asids[i].v32.lo = get_low_32(m_mem->get_asid_base(i) | ASID_RD | ASID_WR);
        tcb->asids[i].v32.hi = get_high_32(m_mem->get_asid_base(i));
    }

    m_mem->write(m_init_tcb.pa, (const char*)tcb, sizeof(*tcb));

    /* 1.3 config SegMMU if need */
    if (m_segmmu_num > 0)
    {
        for (uint32_t i = 0; i < m_core_cnt; i++)
        {
            SegMMUConfig *segmmu = nullptr;

            if (m_segmmu_num == 1)
                segmmu = (SegMMUConfig *)m_segmmu_sec.va;
            else
                segmmu = (SegMMUConfig *)m_segmmu_sec.va + i;

            if (i % 2 == 0)
            {
                memset(tcb, 0, sizeof(tcb_t));
                tcb->smmu.ctrl = segmmu->SegMMU_ctl;
                tcb->smmu.remap = segmmu->SegMMU_remap;
                tcb->smmu.segs[0].ctrl0 = segmmu->seg0.control0;
                tcb->smmu.segs[0].ctrl1 = segmmu->seg0.control1;
                tcb->smmu.segs[1].ctrl0 = segmmu->seg1.control0;
                tcb->smmu.segs[1].ctrl1 = segmmu->seg1.control1;
                tcb->smmu.segs[2].ctrl0 = segmmu->seg2.control0;
                tcb->smmu.segs[2].ctrl1 = segmmu->seg2.control1;
                tcb->smmu.segs[3].ctrl0 = segmmu->seg3.control0;
                tcb->smmu.segs[3].ctrl1 = segmmu->seg3.control1;

                /* handle the last one segmmu config */
                if (i == m_core_cnt - 1)
                {
                    m_mem->write(m_init_tcb.pa + (1 + i/2) * sizeof(tcb_t),
                        (const char*)tcb, sizeof(*tcb));
                    break;
                }
            } else {
                tcb->next_core_smmu.ctrl = segmmu->SegMMU_ctl;
                tcb->next_core_smmu.remap = segmmu->SegMMU_remap;
                tcb->next_core_smmu.segs[0].ctrl0 = segmmu->seg0.control0;
                tcb->next_core_smmu.segs[0].ctrl1 = segmmu->seg0.control1;
                tcb->next_core_smmu.segs[1].ctrl0 = segmmu->seg1.control0;
                tcb->next_core_smmu.segs[1].ctrl1 = segmmu->seg1.control1;
                tcb->next_core_smmu.segs[2].ctrl0 = segmmu->seg2.control0;
                tcb->next_core_smmu.segs[2].ctrl1 = segmmu->seg2.control1;
                tcb->next_core_smmu.segs[3].ctrl0 = segmmu->seg3.control0;
                tcb->next_core_smmu.segs[3].ctrl1 = segmmu->seg3.control1;

                m_mem->write(m_init_tcb.pa + (1 + i/2) * sizeof(tcb_t),
                    (const char*)tcb, sizeof(*tcb));
            }
        }
    }

    /* 2. setup task TCBs */
    for (uint32_t i = 0; i < get_graph().m_subgraphs.size(); i++)
    {
        ret = setup_tcb_sg(get_graph().m_subgraphs[i].id, m_grid_id, core_id);
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        if (++core_id >= m_core_cnt)
            core_id = 0;
    }

    setup_gm_sync_to_ddr(tcb);
    m_status = AIPU_JOB_STATUS_INIT;

finish:
    delete tcb;
    return ret;
}

aipu_status_t aipudrv::JobV3::init(const aipu_global_config_simulation_t* cfg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    assert(cfg != nullptr);
    m_cfg = cfg;

    ret = m_dev->get_next_cluster_id(m_partition_id, m_grid_id);
    if (ret != AIPU_STATUS_SUCCESS)
    {
        LOG(LOG_ERR, "get cluster id fail\n");
        goto finish;
    }

    m_dev->get_core_count(m_partition_id, m_grid_id, &m_core_cnt);

    /* allocate and load job buffers */
    ret = alloc_load_job_buffers();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    ret = setup_tcbs();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    if (m_backup_tcb != nullptr)
        m_mem->read(m_init_tcb.pa, m_backup_tcb.get(), m_tot_tcb_cnt * sizeof(tcb_t));

finish:
    return ret;
}

aipu_status_t aipudrv::JobV3::schedule()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobDesc desc;

    ret = validate_schedule_status();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    if (m_err_code.size() == 1)
        m_mem->zeroize(m_err_code[0].pa, m_err_code[0].size);

    /* with the backup tcbchain if run the job again */
    if (m_backup_tcb != nullptr && m_backup_tcb_used == true)
        m_mem->write(m_init_tcb.pa, m_backup_tcb.get(), m_tot_tcb_cnt * sizeof(tcb_t));
    m_backup_tcb_used = true;

    dump_job_shared_buffers();
    dump_job_private_buffers(m_rodata, m_descriptor);
    dump_specific_buffers();
    ret = dump_for_emulation();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    memset(&desc.kdesc, 0, sizeof(desc.kdesc));

    /* for X2 simulation */
    desc.kdesc.job_id = m_id;
    desc.kdesc.version_compatible = !get_graph().m_do_vcheck;
    desc.kdesc.aipu_config = get_graph().m_hw_config;
    desc.jobbase = this;
    desc.tcb_head = m_init_tcb.pa;
    desc.tcb_tail = m_sg_job[m_sg_cnt-1].tasks[m_task_per_sg-1].tcb.pa;

    /* for X2 HW */
    desc.kdesc.exec_flag = (m_qos == AIPU_JOB_QOS_HIGH)
        ? AIPU_JOB_EXEC_FLAG_QOS_FAST : AIPU_JOB_EXEC_FLAG_QOS_SLOW;
    desc.kdesc.aipu_version = get_graph().m_hw_version;
    desc.kdesc.partition_id = m_partition_id;
    desc.kdesc.head_tcb_pa = m_init_tcb.pa;
    desc.kdesc.last_task_tcb_pa = m_sg_job[m_sg_cnt-1].tasks[m_task_per_sg-1].tcb.pa;
    desc.kdesc.tail_tcb_pa = m_sg_job[m_sg_cnt-1].tasks[m_task_per_sg-1].tcb.pa;

    /* for X2 HW and Simulation if need syncing from GM to DDR */
    if (m_gm->gm_need_sync_out())
    {
        desc.tcb_tail = m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 1);
        desc.kdesc.tail_tcb_pa = m_init_tcb.pa + sizeof(tcb_t) * (m_tot_tcb_cnt - 1);
    }

    /* for debugger */
    desc.kdesc.is_defer_run = m_is_defer_run;
    desc.kdesc.do_trigger = m_do_trigger;

    if (get_graph().m_text.size == 0)
        LOG(LOG_WARN, "Graph text size is 0\n");
    else
        ret = m_dev->schedule(desc);

    m_status = AIPU_JOB_STATUS_SCHED;

    return ret;
}

aipu_status_t aipudrv::JobV3::destroy()
{
    return free_job_buffers();
}

void aipudrv::JobV3::dump_specific_buffers()
{
    DEV_PA_64 dump_pa;
    uint32_t dump_size;

    if (m_dump_tcb)
    {
        dump_pa = m_tcbs.pa;
        dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
        if (dump_size != 0)
            dump_buffer(dump_pa, nullptr, dump_size, "TCBs");
    }
}

aipu_status_t aipudrv::JobV3::dump_for_emulation()
{
    DEV_PA_64 dump_pa;
    uint32_t dump_size;
    char dump_name[4096];
    int emu_input_cnt = 3 + m_inputs.size() + (get_graph().m_bweight.size != 0 ? 1 : 0) +
        (m_descriptor.size != 0 ? 1 : 0);
    int emu_output_cnt = m_outputs.size();
    int file_id = -1;
    tcb_t tcb;
    bool default_output_prefix = true;
    FileWrapper ofs(m_dump_dir + "/runtime.cfg", std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    FileWrapper ofsmt(m_dump_dir + "/metadata.txt", std::ios_base::in | std::ios_base::out | std::ios_base::trunc);

    if (m_dump_emu == false)
        return AIPU_STATUS_SUCCESS;

    ofs << "[COMMON]\n";

    /* runtime.cfg: config */
    ofs << "#configuration 1:X2_1204 2:X2_1204_MP3\n";
    ofs << "CONFIG=" << m_dev->get_config_code() << "\n";

    /* runtime.cfg: enable_avx */
    ofs << "#if ENABLE_AVX is true then using the intel SIMD instructions to speedup.\n";
    if (m_cfg->enable_avx)
        ofs << "ENABLE_AVX=true\n";
    else
        ofs << "ENABLE_AVX=false\n";

    /* runtime.cfg: log file path */
    ofs << "#Where log output to store is.\n";
    ofs << "LOG_FILEPATH=" << m_cfg->log_file_path << "\n";

    /* runtime.cfg: log_level */
    ofs << "#which level is your selected: 0:ERROR, 1: WARN, 2: INFO, 3: DEBUG\n";
    ofs << "LOG_LEVEL=" << m_cfg->log_level << "\n";

    /* runtime.cfg: verbose */
    ofs << "#if LOG_VERBOSE is true then print log to console. otherwise no\n";
    if (m_cfg->verbose)
        ofs << "LOG_VERBOSE=true\n";
    else
        ofs << "LOG_VERBOSE=false\n";

    /* runtime.cfg: enable_calloc */
    ofs << "##if ENABLE_CALLOC is true the allocation memory is set to zero.\n";
    if (m_cfg->enable_calloc)
        ofs << "ENABLE_CALLOC=true\n";
    else
        ofs << "ENABLE_CALLOC=false\n";

    /* runtime.cfg: en_eval */
    ofs << "## EN_EVAL\n";
    if (m_cfg->en_eval)
        ofs << "EN_EVAL=true\n";
    else
        ofs << "EN_EVAL=false\n";
    ofs << "\n";

    ofs.dump_to_string(m_dumpcfg_header);

    /* runtime.cfg: [INPUT] */
    ofs << "[INPUT]\n";
    ofs << "COUNT=" << emu_input_cnt << "\n";

    /* dump temp.text */
    dump_pa = get_graph().m_text.pa;
    dump_size = get_graph().m_btext.size;
    if (dump_size != 0)
    {
        snprintf(dump_name, 128, "%s/%s.text", m_dump_dir.c_str(), m_dump_prefix.c_str());
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".text\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
    }

    /* dump temp.weight */
    dump_pa = get_graph().m_weight.pa;
    dump_size = get_graph().m_bweight.size;
    if (dump_size != 0)
    {
        snprintf(dump_name, 128, "%s/%s.weight", m_dump_dir.c_str(), m_dump_prefix.c_str());
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".weight\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
    }

    /* dump temp.rodata */
    dump_pa = m_rodata.pa;
    dump_size = m_rodata.size;
    snprintf(dump_name, 128, "%s/%s.ro", m_dump_dir.c_str(), m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);
    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".ro\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});

    /* dump temp.dcr */
    if (m_descriptor.size != 0)
    {
        dump_pa = m_descriptor.pa;
        dump_size = m_descriptor.size;
        snprintf(dump_name, 128, "%s/%s.dcr", m_dump_dir.c_str(), m_dump_prefix.c_str());
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".dcr\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
    }

    /* dump temp.tcb */
    dump_pa = m_tcbs.pa;
    dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
    snprintf(dump_name, 128, "%s/%s.tcb", m_dump_dir.c_str(), m_dump_prefix.c_str());
    m_dump_tcb_info[0] = std::make_tuple(m_dump_dir + "/init.tcb", dump_pa, sizeof(tcb_t));
    m_dump_tcb_info[1] = std::make_tuple(m_dump_dir + "/task.tcb", dump_pa + sizeof(tcb_t), dump_size - sizeof(tcb_t));
    m_mem->dump_file(std::get<1>(m_dump_tcb_info[0]), std::get<0>(m_dump_tcb_info[0]).c_str(),
        std::get<2>(m_dump_tcb_info[0]));
    m_mem->dump_file(std::get<1>(m_dump_tcb_info[1]), std::get<0>(m_dump_tcb_info[1]).c_str(),
        std::get<2>(m_dump_tcb_info[1]));
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".tcb\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";

    /* dump temp.input[n] */
    for (uint32_t i = 0; i < m_inputs.size(); i++)
    {
        dump_pa   = m_inputs[i].pa;
        dump_size = m_inputs[i].size;
        snprintf(dump_name, 128, "%s/%s.input%d", m_dump_dir.c_str(), m_dump_prefix.c_str(), i);
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".input" << i << "\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
    }
    ofs << "\n";

    ofs << "[HOST]\n";
    ofs << "TCBP_HI=0x" << std::hex << get_high_32(m_init_tcb.pa) << "\n";
    ofs << "TCBP_LO=0x" << get_low_32(m_init_tcb.pa) << "\n";
    m_dumpcfg_host = {m_partition_id, get_high_32(m_init_tcb.pa), get_low_32(m_init_tcb.pa)};
    ofs << "\n";


    /* runtime.cfg: [OUTPUT] */
    ofs << "[OUTPUT]\n";
    ofs << "COUNT=" << std::dec << emu_output_cnt << "\n";

    /* dump output.bin[n] */
    if (strncmp(m_dump_output_prefix.c_str(), "temp", 4))
        default_output_prefix = false;

    for (uint32_t i = 0; i < m_outputs.size(); i++)
    {
        dump_pa   = m_outputs[i].pa;
        dump_size = m_outputs[i].size;

        if (default_output_prefix)
        {
            ofs << "FILE" << std::dec << i
                << "=" << m_dump_output_prefix << ".output" << i << "\n";
            snprintf(dump_name, 128, "%s/%s.output%d", m_dump_dir.c_str(), m_dump_prefix.c_str(), i);
            m_dumpcfg_output.push_back({dump_name, dump_pa, dump_size});
        } else {
            if (0 == i)
            {
                ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << "\n";
            } else {
                ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << i << "\n";
            }
        }

        ofs << "BASE" << std::dec << i << "=0x" << std::hex << dump_pa << "\n";
        ofs << "SIZE" << std::dec << i << "=0x" << std::hex << dump_size << "\n";
    }

    /* close runtime.cfg */
    ofs.close();

    /* dump metadata.txt */
    ofsmt << "Total TCBs Count: " << std::dec << m_tot_tcb_cnt << "\n";

    /* init TCB */
    m_mem->read(m_init_tcb.pa, &tcb, sizeof(tcb));
    ofsmt << "\n***INIT TCB***\n";
    ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
    ofsmt << "next: 0x" << tcb.next << "\n";

    ofsmt << "GM_CTRL: 0x" << tcb.gm_ctl << "\n";
    ofsmt << "group_id: " << std::dec << tcb.igroup_id << "\n";
    ofsmt << "grid_id: " << std::dec << tcb.igrid_id << "\n";
    ofsmt << "GM0_CTRL: 0x" << std::hex << tcb.gm_rgnx_ctrl[0] << "\n";
    ofsmt << "GM1_CTRL: 0x" << tcb.gm_rgnx_ctrl[1] << "\n";
    ofsmt << "GM0_LO: 0x" << tcb.gm_rgnx_addr[0].v32.lo << "\n";
    ofsmt << "GM0_HI: 0x" << tcb.gm_rgnx_addr[0].v32.hi << "\n";
    ofsmt << "GM1_LO: 0x" << tcb.gm_rgnx_addr[1].v32.lo << "\n";
    ofsmt << "GM1_HI: 0x" << tcb.gm_rgnx_addr[1].v32.hi << "\n";
    for (int i = 0; i < 4; i++)
    {
        ofsmt << "ASID" << std::dec << i << "_LO: 0x" << std::hex << tcb.asids[i].v32.lo << "\n";
        ofsmt << "ASID" << std::dec << i << "_HI: 0x" << std::hex << tcb.asids[i].v32.hi << "\n";
    }

    /* TCBs */
    for (uint32_t i = 1; i < m_tot_tcb_cnt - 1; i++)
    {
        m_mem->read(m_init_tcb.pa + sizeof(tcb) * i, &tcb, sizeof(tcb));
        ofsmt << "\n***TCB " << std::dec << i - 1 << "***\n";
        ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";

        ofsmt << "next: 0x" << tcb.next << "\n";
        ofsmt << "loop_count: " << std::dec << tcb.loop_count << "\n";

        ofsmt << "start_pc: 0x" << std::hex << tcb.spc << "\n";
        ofsmt << "interrupt: 0x" << std::hex <<tcb.interrupt << "\n";

        ofsmt << "group_id: " << std::dec << tcb.groupid << "\n";
        ofsmt << "grid_id: " << tcb.gridid << "\n";
        ofsmt << "task_id: " << tcb.taskid << "\n";

        ofsmt << "grid_dim_x: " << tcb.grid_dim_x << "\n";
        ofsmt << "grid_dim_y: " << tcb.grid_dim_y << "\n";
        ofsmt << "grid_dim_z: " << tcb.grid_dim_z << "\n";

        ofsmt << "group_dim_x: " << tcb.group_dim_x << "\n";
        ofsmt << "group_dim_y: " <<  tcb.group_dim_y << "\n";
        ofsmt << "group_dim_z: " << tcb.group_dim_z << "\n";

        ofsmt << "group_id_x: " << tcb.group_id_x << "\n";
        ofsmt << "group_id_y: " << tcb.group_id_y << "\n";
        ofsmt << "group_id_z: " << tcb.group_id_z << "\n";

        ofsmt << "task_id_x: " << tcb.task_id_x << "\n";
        ofsmt << "task_id_y: " << tcb.task_id_y << "\n";
        ofsmt << "task_id_z: " << tcb.task_id_z << "\n";

        ofsmt << "sp: 0x" << std::hex << tcb.sp << "\n";
        ofsmt << "pp: 0x" << tcb.pp << "\n";
        ofsmt << "dp: 0x" << tcb.dp << "\n";
        ofsmt << "cp: 0x" << tcb.cp << "\n";
        ofsmt << "pprint: 0x" << tcb.pprint << "\n";
        ofsmt << "pprofiler: 0x" << tcb.pprofiler << "\n";
        ofsmt << "fmdp: 0x"<< tcb.fmdp << "\n";
        ofsmt << "tap: 0x" << tcb.tap << "\n";
        ofsmt << "dap: 0x" << tcb.dap << "\n";
        ofsmt << "pap: 0x" << tcb.pap << "\n";
        ofsmt << "idp: 0x" << tcb.idp << "\n";
        ofsmt << "dsize: 0x" << tcb.dsize << "\n";
    }

    if (m_gm->m_gm_buf_map_size[EM_GM_BUF_OUTPUT] != 0 && m_mem->is_gm_enable())
    {
        m_mem->read(m_init_tcb.pa + sizeof(tcb) * (m_tot_tcb_cnt - 1), &tcb, sizeof(tcb));
        ofsmt << "\n***INIT Sync TCB***\n";
        ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
        ofsmt << "next: 0x" << tcb.next << "\n";

        ofsmt << "GM_CTRL: 0x" << tcb.gm_ctl << "\n";
        ofsmt << "group_id: " << std::dec << tcb.igroup_id << "\n";
        ofsmt << "grid_id: " << std::dec << tcb.igrid_id << "\n";
        ofsmt << "GM0_CTRL: 0x" << std::hex << tcb.gm_rgnx_ctrl[0] << "\n";
        ofsmt << "GM1_CTRL: 0x" << tcb.gm_rgnx_ctrl[1] << "\n";
        ofsmt << "GM0_LO: 0x" << tcb.gm_rgnx_addr[0].v32.lo << "\n";
        ofsmt << "GM0_HI: 0x" << tcb.gm_rgnx_addr[0].v32.hi << "\n";
        ofsmt << "GM1_LO: 0x" << tcb.gm_rgnx_addr[1].v32.lo << "\n";
        ofsmt << "GM1_HI: 0x" << tcb.gm_rgnx_addr[1].v32.hi << "\n";
        for (int i = 0; i < 4; i++)
        {
            ofsmt << "ASID" << std::dec << i << "_LO: 0x" << std::hex << tcb.asids[i].v32.lo << "\n";
            ofsmt << "ASID" << std::dec << i << "_HI: 0x" << std::hex << tcb.asids[i].v32.hi << "\n";
        }
    }

    ofsmt << "\n***IO Tensors***\n";

    for (uint32_t i = 0; i < m_inputs.size(); i++)
    {
        dump_pa   = m_inputs[i].pa;
        dump_size = m_inputs[i].size;

        ofsmt << "input" << std::dec << i << "_addr: 0x" << std::hex << dump_pa << "\n";
        ofsmt << "input" << std::dec << i << "_size: 0x" << std::hex << dump_size << "\n";
    }

    for (uint32_t i = 0; i < m_outputs.size(); i++)
    {
        dump_pa   = m_outputs[i].pa;
        dump_size = m_outputs[i].size;

        ofsmt << "output" << std::dec << i << "_addr: 0x" << std::hex << dump_pa << "\n";
        ofsmt << "output" << std::dec << i << "_size: 0x" << std::hex << dump_size << "\n";
    }

    ofsmt.dump_to_string(m_dumpcfg_meta);
    /* close metadata.txt */
    ofsmt.close();
    return AIPU_STATUS_SUCCESS;
}

#if defined(SIMULATION) && defined(ZHOUYI_V3)
void aipudrv::JobV3::dumpcfg_alljob()
{
    JobV3 *job = nullptr;
    std::vector<uint32_t> cluster_id[4];
    FileWrapper ofs("./runtime.cfg", std::ios::out);
    FileWrapper ofsmt("./metadata.txt", std::ios::out);
    uint32_t count = 0, cmdpool_mask = 0;
    MainContext *ctx = static_cast<MainContext *>(get_graph().m_ctx);
    GraphTable &graphs = ctx->get_graphtable();
    GraphV3 *graph = nullptr;
    std::ostringstream oss;
    SimulatorV3 *sim = static_cast<SimulatorV3 *>(m_dev);

    /**
     * set destroy flag for each tcbchain in all cmdpool, the simulator
     * can know when it's reasonable to set end/idle status for cmdpool.
     */
    sim->set_destroy_to_all_tcbchain();

    /* runtime.cfg: [COMMON] */
    ofs << m_dumpcfg_header << "\n";

    /* runtime.cfg: [INPUT] */
    count = 0;
    for (auto g : graphs)
    {
        graph = static_cast<GraphV3 *>(g.second);
        for (auto item: graph->m_jobs)
        {
            job = static_cast<JobV3 *>(item.second);
            for (uint32_t i = 0; i < job->m_dumpcfg_input.size(); i++)
            {
                oss << "FILE" << std::dec << count << "=" << job->m_dumpcfg_input.at(i).file << "\n";
                oss << "BASE" << count << "=0x" << std::hex << job->m_dumpcfg_input.at(i).base << "\n";
                count++;
            }

            /* dump finally updated init/task tcb for each job */
            for (int i = 0; i < 2; i++)
            {
                m_mem->dump_file(std::get<1>(job->m_dump_tcb_info[i]), std::get<0>(job->m_dump_tcb_info[i]).c_str(),
                    std::get<2>(job->m_dump_tcb_info[i]));
                oss << "FILE" << std::dec << count << "=" << std::get<0>(job->m_dump_tcb_info[i]) << "\n";
                oss << "BASE" << count << "=0x" << std::hex << std::get<1>(job->m_dump_tcb_info[i]) << "\n";
                count++;
            }
        }
    }
    ofs << "[INPUT]\n";
    ofs << "COUNT=" << count << "\n";
    ofs << oss.str();
    ofs << "\n";

    /* runtime.cfg: [HOST] */
    oss.str("");
    count = 0;
    cmdpool_mask = sim->get_cmdpool_bitmap();
    for (auto g : graphs)
    {
        graph = static_cast<GraphV3 *>(g.second);
        for (auto item: graph->m_jobs)
        {
            job = static_cast<JobV3 *>(item.second);
            if (cmdpool_mask & (1 << job->m_bind_cmdpool_id))
            {
                oss << "SET_PARTITION" << std::dec << count << "=" << job->m_dumpcfg_host.part_id << "\n";
                oss << "TCBP_HI" << std::dec << count << "=0x" << std::hex << job->m_dumpcfg_host.hi_addr << "\n";
                oss << "TCBP_LO" << std::dec << count << "=0x" << std::hex << job->m_dumpcfg_host.lo_addr << "\n";
                count++;
                cmdpool_mask &= ~(1 << job->m_bind_cmdpool_id);
                sim->clear_cmdpool_bitmap(job->m_bind_cmdpool_id);
            }
        }
    }
    ofs << "[HOST]\n";
    ofs << "COUNT=" << count << "\n";
    ofs << oss.str();
    ofs << "\n";

    /* runtime.cfg: [ALLOCATE_PARTITION] */
    count = 0;
    ofs << "[ALLOCATE_PARTITION]\n";
    for (int i = 0; i < 4; i++)
    {
        m_dev->get_cluster_id(i, cluster_id[i]);
        count += cluster_id[i].size();
    }
    ofs << "COUNT=" << count << "\n";
    for (int part_id = 0; part_id < 4; part_id++)
    {
        for (auto cluster_id : cluster_id[part_id])
            ofs << "CLUSTER" << std::dec << cluster_id << "=" << part_id << "\n";
    }
    ofs << "\n";

    /* runtime.cfg: [OUTPUT] */
    oss.str("");
    count = 0;
    for (auto g : graphs)
    {
        graph = static_cast<GraphV3 *>(g.second);
        for (auto item: graph->m_jobs)
        {
            job = static_cast<JobV3 *>(item.second);
            for (uint32_t i = 0; i < job->m_dumpcfg_output.size(); i++)
            {
                oss << "FILE" << std::dec << count << "=" << job->m_dumpcfg_output.at(i).file << "\n";
                oss << "BASE" << count << "=0x" << std::hex << job->m_dumpcfg_output.at(i).base << "\n";
                oss << "SIZE" << count << "=0x" << std::hex << job->m_dumpcfg_output.at(i).size << "\n";
                count++;
            }
        }
    }
    ofs << "[OUTPUT]\n";
    ofs << "COUNT=" << std::dec << count << "\n";
    ofs << oss.str();
    ofs << "\n";

    /* clost runtime.cfg */
    ofs.close();

    /* gen metadata.txt */
    for (auto g : graphs)
    {
        graph = static_cast<GraphV3 *>(g.second);
        for (auto item: graph->m_jobs)
        {
            job = static_cast<JobV3 *>(item.second);
            ofsmt << job->m_dumpcfg_meta;
            ofsmt << "\n";
        }
        ofsmt << "\n";
    }
    ofsmt.close();
}
#endif

aipu_status_t aipudrv::JobV3::import_buffers(aipu_tensor_type_t type, int* fds)
{
    if ((type != AIPU_TENSOR_TYPE_INPUT) && (type != AIPU_TENSOR_TYPE_OUTPUT))
        return AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE;

    if (m_support_dma_buf == false)
        return AIPU_STATUS_ERROR_INVALID_OP;

    /* TBD */
    return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
}

aipu_status_t aipudrv::JobV3::export_buffers(aipu_tensor_type_t type, int* fds)
{
    if ((type != AIPU_TENSOR_TYPE_INPUT) && (type != AIPU_TENSOR_TYPE_OUTPUT))
        return AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE;

    if (m_support_dma_buf == false)
        return AIPU_STATUS_ERROR_INVALID_OP;

    /* TBD */
    return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
}

aipu_status_t aipudrv::JobV3::bind_core(uint32_t partition_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t partition_cnt = 0;

    ret = m_dev->get_partition_count(&partition_cnt);
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    if (partition_id >= partition_cnt)
        return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;

    ret = validate_schedule_status();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    m_is_defer_run = true;
    m_do_trigger = false;
    m_partition_id = partition_id;
    ret = schedule();
    if (AIPU_STATUS_SUCCESS == ret)
        m_status = AIPU_JOB_STATUS_BIND;

    return ret;
}

aipu_status_t aipudrv::JobV3::debugger_run()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_job_status_t status;

    if (m_status != AIPU_JOB_STATUS_BIND)
        return AIPU_STATUS_ERROR_INVALID_OP;

    m_is_defer_run = true;
    m_do_trigger = true;
    ret = schedule();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    ret = get_status_blocking(&status, -1);
    if ((AIPU_STATUS_SUCCESS == ret) && (AIPU_JOB_STATUS_DONE != status))
        ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;

    return ret;
}
