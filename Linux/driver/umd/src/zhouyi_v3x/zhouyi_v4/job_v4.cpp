// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v4.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v4 job module implementation
 */

#include <cstring>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "job_v4.h"
#include "../common/graph_v3x.h"
#include "parser_base.h"
#include "utils/helper.h"

#if defined(SIMULATION)
#include "simulator/simulator_v4.h"
#endif
aipudrv::JobV4::JobV4(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
    aipu_create_job_cfg_t *config) : JobBase(ctx, graph, dev),
    m_partition_id(config->partition_id),
    m_qos(config->qos_level),
    m_fm_mem_region(config->fm_mem_region),
    m_dbg_dispatch(config->dbg_dispatch),
    m_core_id(config->dbg_core_id)
{
    m_init_tcb.init(0);
    m_sgt_allocated.clear();

#if defined(SIMULATION)
    /**
     * initialize an invalid cmdpool id, set it on scheduling to sim
     */
    m_bind_cmdpool_id = 0xffffffff;
#endif

    m_segmmu_num = get_graph().m_segmmu_num;
    m_gm = new GM_V4(*this);

    if (m_mem->get_asid_base(0) != m_mem->get_asid_base(1))
        m_same_asid = false;

    if (config->fm_idxes)
    {
        for (int i = 0; i < config->fm_idxes_cnt; i++)
            m_fm_idxes.insert(config->fm_idxes[i]);
    }
}

aipudrv::JobV4::~JobV4()
{
    delete m_gm;
}

void aipudrv::JobV4::set_job_params(uint32_t sg_cnt, uint32_t task_per_sg,
                                    uint32_t remap, uint32_t core_cnt)
{
    m_sg_cnt = sg_cnt;
    m_task_per_sg = task_per_sg;
    m_remap_flag = remap;

    /**
     * currently: 1 grid init-tcb + 1 group init-tcb + n task-tcb
     */
    m_segmmu_tcb_num = core_cnt;
    m_tot_tcb_cnt = m_sg_cnt * m_task_per_sg + 1 + 1;
    m_backup_tcb.reset(new char[m_tot_tcb_cnt * sizeof(tcb_t)]);
}

void aipudrv::JobV4::setup_gm_sync_from_ddr(tcb_t *tcb)
{
    if (!m_mem->is_gm_enable())
        return;

    if (!m_gm->gm_need_remap())
        return;

    tcb->gm_ctrl = GM_CTRL_REMAP_EN | GM_CTRL_REMAP_MODE_TIME_PRIOR;
    tcb->gm_addr_low = get_low_32(m_gm->m_gm_buf_base);
    tcb->gm_addr_high = get_high_32(m_gm->m_gm_buf_base);

    if (m_gm->m_gm_buf_sync_size != 0)
        tcb->gm_sync = GM_SYNC_DDR_TO_GM;
}

#define SEGMMU_MEM_CTRL_EN (1 << 0)
#define SEGMMU_REMAP_EN (1 << 4)
#define SEGMMU_REMAP_SHARE_EN (1 << 5)
#define SEGMMU_IN_ASID_WR (1 << 0)
#define SEGMMU_IN_ASID_RD (1 << 1)
aipu_status_t aipudrv::JobV4::setup_segmmu(SubGraphTask &sg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    SegMMUConfig *segmmu = nullptr;
    union segmmu_id
    {
        uint32_t id;
        struct
        {
            uint32_t segmmu_ctrl_idx : 8;
            uint32_t segmmu_idx : 8;
            uint32_t core_id_mask : 16;
        };
    };

    if (m_segmmu_num == 0)
        goto out;

    segmmu = (SegMMUConfig *)get_graph().m_bsegmmu.va;
    for (uint32_t i = 0; i < m_core_cnt; i++)
    {
        if (m_segmmu_num != 1)
            segmmu++;

        segmmu->SegMMU_ctl = SEGMMU_REMAP_SHARE_EN | SEGMMU_MEM_CTRL_EN;
        segmmu->SegMMU_remap = 0;

        m_segmmu_sec.push_back(*segmmu);
    }

    for (auto &iobuf : m_segmmus)
    {
        segmmu_id s_id = {.id = iobuf.id};

        if ((s_id.core_id_mask & ((1 << m_core_cnt) - 1)) == 0)
        {
            LOG(LOG_ERR, "Segmmu core idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, %d)\n",
                s_id.core_id_mask, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
            goto out;
        }

        for (uint32_t core_idx = 0; core_idx < m_core_cnt; core_idx++)
        {
            if (!(s_id.core_id_mask & (1 << core_idx)))
                continue;

            if (s_id.segmmu_idx >= 0 && s_id.segmmu_idx < 4)
            {
                if (s_id.segmmu_ctrl_idx >= 0 && s_id.segmmu_ctrl_idx <= 1)
                {
                    uint32_t ctrl = m_segmmu_sec[core_idx].seg[s_id.segmmu_idx].control[s_id.segmmu_ctrl_idx];
                    ctrl &= 0x3fff;
                    ctrl |= (iobuf.pa & (~0x3fff));
                    m_segmmu_sec[core_idx].seg[s_id.segmmu_idx].control[s_id.segmmu_ctrl_idx] = ctrl;
                } else {
                    LOG(LOG_ERR, "Segmmu ctrl idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, %d)\n",
                        core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
                    goto out;
                }
            }
            else
            {
                LOG(LOG_ERR, "Segmmu seg idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, %d)\n",
                    core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
                goto out;
            }
        }
    }

out:
    return ret;
}

aipu_status_t aipudrv::JobV4::setup_rodata_sg(uint32_t sg_id,
                                              const std::vector<struct GraphParamMapLoadDesc> &param_map,
                                              std::vector<BufferDesc *> &reuse_buf, std::vector<BufferDesc *> &static_buf,
                                              std::set<uint32_t> *dma_buf_idx)
{
    BufferDesc rodata, dcr;
    // const std::vector<struct GraphParamMapLoadDesc>& param_map =
    //     get_graph().m_subgraphs[sg_id].param_map;
    // std::vector<BufferDesc>& reuse_buf  = m_sg_job[sg_id].reuses;
    // std::vector<BufferDesc>& static_buf = m_sg_job[sg_id].weights;

    rodata.init(0, m_rodata->pa, m_rodata->size, m_rodata->req_size);
    if (m_descriptor != nullptr)
    {
        dcr.init(0, m_descriptor->pa, m_descriptor->size, m_descriptor->req_size);
        return setup_rodata(param_map, reuse_buf, static_buf, rodata, &dcr, dma_buf_idx);
    }
    else
    {
        return setup_rodata(param_map, reuse_buf, static_buf, rodata, nullptr, dma_buf_idx);
    }
}

aipu_status_t aipudrv::JobV4::alloc_subgraph_buffers()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    SubGraphTask sg = {0};

    /* allocate subgraph buffers */
    for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++)
    {
        sg.reset(sg_idx);

        /* each subgraph has private buffer core-accessed */
        for (uint32_t k = 0; k < get_graph().m_subgraphs[sg_idx].private_buffers.size(); k++)
        {
            const GraphSectionDesc &section_desc = get_graph().m_subgraphs[sg_idx].private_buffers[k];
            BufferDesc *buf = nullptr;

            if (section_desc.size != 0)
            {
                std::string buf_name = "priv_" + std::to_string(sg_idx) + "_" + std::to_string(k);

                ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &buf, buf_name.c_str());
                if (AIPU_STATUS_SUCCESS != ret)
                {
                    LOG(LOG_ERR, "alloc private buffer %d [fail]", k);
                    goto add_sg;
                }

                if (m_dump_reuse)
                    m_mem->mem_bzero(buf->pa, buf->size);

                sg.reuse_priv_buffers.push_back(buf);
            }
        }

        /* allocate reuse buffers, all subgraphs share one copy of reuse buffers */
        if (sg.id == 0)
        {
            for (uint32_t k = 0; k < get_graph().m_subgraphs[0].reuse_sections.size(); k++)
            {
                const GraphSectionDesc &section_desc = get_graph().m_subgraphs[0].reuse_sections[k];
                BufferDesc *bufferDesc = nullptr;

                if (section_desc.size != 0)
                {
                    std::string buf_name = "reuse_" + std::to_string(k);

                    /* handle buffer if allocated from GM_V4 */
                    if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE))
                    {
                        bufferDesc = new BufferDesc;
                        buf_name = "gm_" + buf_name;
                        ret = m_gm->gm_malloc(sg_idx, k, GM_BUF_TYPE_REUSE, buf_name, bufferDesc);
                    }
                    else
                    {
                        if ((m_fm_idxes.count(k) == 1) || (m_fm_mem_region != AIPU_MEM_REGION_DEFAULT))
                            ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &bufferDesc,
                                                buf_name.c_str(), m_fm_mem_region);
                        else
                            ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &bufferDesc,
                                                buf_name.c_str(), AIPU_MEM_REGION_DEFAULT);
                    }

                    if (AIPU_STATUS_SUCCESS != ret)
                    {
                        LOG(LOG_ERR, "alloc reuse buffer %d [fail]", k);
                        goto add_sg;
                    }

                    if (m_dump_reuse)
                        m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

                    sg.reuses.push_back(bufferDesc);
                }
                else
                {
                    LOG(LOG_WARN, "reuse %d: size == 0\n", k);
                }
            }

            /* init task weights address, share a common copy */
            sg.weights = &get_graph().m_weights;
        }

    add_sg:
        m_sg_job.push_back(sg);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
    }

    if (get_subgraph_cnt() > 0 && get_graph().m_subgraphs[0].printfifo_size > 0)
    {
        std::string buf_name = "printf";
        ret = m_mem->malloc(get_subgraph_cnt() * AIPU_PAGE_SIZE, 0, &m_pprint, buf_name.c_str());
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
    }

out:
    return ret;
}

int aipudrv::JobV4::alloc_subgraph_buffers_optimized()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    SubGraphTask sg = {0};
    uint32_t priv_buf_total_size = 0;
    uint32_t reuse_buf_total_size = 0;
    uint32_t priv_offset = 0, offset = 0;
    int retval = 0;

    if (m_fm_mem_region != AIPU_MEM_REGION_DEFAULT)
    {
        retval = -1;
        LOG(LOG_DEBUG, "don't try optimization if specify memory region\n");
        goto opt_alloc_fail;
    }

    /* caculate the total size of each buffer type */
    for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++)
    {
        for (uint32_t k = 0; k < get_graph().m_subgraphs[sg_idx].private_buffers.size(); k++)
        {
            const GraphSectionDesc &section_desc = get_graph().m_subgraphs[sg_idx].private_buffers[k];
            priv_buf_total_size += ALIGN_PAGE(section_desc.size);
        }

        if (sg_idx == 0)
        {
            for (uint32_t k = 0; k < get_graph().m_subgraphs[0].reuse_sections.size(); k++)
            {
                /**
                 * the below two types buffer can't pass
                 * centralized memory allocation flow.
                 */
                if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE))
                    continue;

                if (m_fm_idxes.count(k) == 1)
                    continue;

                const GraphSectionDesc &section_desc = get_graph().m_subgraphs[0].reuse_sections[k];
                reuse_buf_total_size += ALIGN_PAGE(section_desc.size);
                m_top_reuse_idx.insert(k);
            }
        }
    }

    /* allocate buffer only once for each type */
    if (priv_buf_total_size > 0)
    {
        ret = m_mem->malloc(priv_buf_total_size, 0, &m_top_priv_buf, "tot_priv");
        if (AIPU_STATUS_SUCCESS != ret)
        {
            retval = -1;
            LOG(LOG_DEBUG, "optmize alloc private buffer, size: 0x%x [fail], try scatter alloc\n",
                priv_buf_total_size);
            goto opt_alloc_fail;
        }
    }

    ret = m_mem->malloc(reuse_buf_total_size, 0, &m_top_reuse_buf, "tot_reuse");
    if (AIPU_STATUS_SUCCESS != ret)
    {
        retval = -1;
        LOG(LOG_DEBUG, "optmize alloc reuse buffer, size: 0x%x [fail], try scatter alloc\n",
            reuse_buf_total_size);
        goto opt_alloc_fail;
    }

    for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++)
    {
        sg.reset(sg_idx);

        /* each subgraph has private buffer core-accessed */
        for (uint32_t k = 0; k < get_graph().m_subgraphs[sg_idx].private_buffers.size(); k++)
        {
            const GraphSectionDesc &section_desc = get_graph().m_subgraphs[sg_idx].private_buffers[k];
            BufferDesc *bufferDesc = nullptr;

            if (section_desc.size != 0)
            {
                bufferDesc = new BufferDesc;
                bufferDesc->reset();
                bufferDesc->init(m_top_priv_buf->asid_base, m_top_priv_buf->pa + priv_offset,
                                 ALIGN_PAGE(section_desc.size), section_desc.size);
                priv_offset += ALIGN_PAGE(section_desc.size);

                if (m_dump_reuse)
                    m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

                sg.reuse_priv_buffers.push_back(bufferDesc);
            }
            else
            {
                LOG(LOG_WARN, "opt priv %d: size == 0\n", k);
            }
        }

        /* allocate reuse buffers, all subgraphs share one copy of reuse buffers */
        if (sg.id == 0)
        {
            for (uint32_t k = 0; k < get_graph().m_subgraphs[0].reuse_sections.size(); k++)
            {
                const GraphSectionDesc &section_desc = get_graph().m_subgraphs[0].reuse_sections[k];
                BufferDesc *bufferDesc = nullptr;

                if (section_desc.size != 0)
                {
                    std::string buf_name = "reuse_" + std::to_string(k);
                    bufferDesc = new BufferDesc;
                    bufferDesc->reset();

                    /* handle buffer if allocated from GM_V4 */
                    if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE))
                    {
                        buf_name = "gm_" + buf_name;
                        ret = m_gm->gm_malloc(sg_idx, k, GM_BUF_TYPE_REUSE, buf_name, bufferDesc);
                        if (AIPU_STATUS_SUCCESS != ret)
                        {
                            retval = -3;
                            LOG(LOG_ERR, "alloc GM_V4 reuse buffer %d [fail]", k);
                            goto add_sg;
                        }
                    }
                    else
                    {
                        if (m_fm_idxes.count(k) == 1)
                        {
                            ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &bufferDesc,
                                                buf_name.c_str(), m_fm_mem_region);
                            if (AIPU_STATUS_SUCCESS != ret)
                            {
                                retval = -4;
                                LOG(LOG_ERR, "alloc specified reuse buffer %d [fail]", k);
                                goto add_sg;
                            }
                        }
                        else
                        {
                            bufferDesc->init(m_top_reuse_buf->asid_base, m_top_reuse_buf->pa + offset,
                                             ALIGN_PAGE(section_desc.size), section_desc.size);
                            offset += ALIGN_PAGE(section_desc.size);
                        }
                    }

                    if (m_dump_reuse)
                        m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

                    sg.reuses.push_back(bufferDesc);
                }
                else
                {
                    LOG(LOG_WARN, "opt reuse %d: size == 0\n", k);
                }
            }

            /* init task weights address, share a common copy */
            sg.weights = &get_graph().m_weights;
        }

    add_sg:
        m_sg_job.push_back(sg);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
    }

    if (get_subgraph_cnt() > 0 && get_graph().m_subgraphs[0].printfifo_size > 0)
    {
        std::string buf_name = "printf";
        ret = m_mem->malloc(get_subgraph_cnt() * AIPU_PAGE_SIZE, 0, &m_pprint, buf_name.c_str());
        if (ret != AIPU_STATUS_SUCCESS)
        {
            retval = -6;
            goto out;
        }
    }

    m_optimized_reuse_alloc = true;
    return retval;

opt_alloc_fail:
    if (m_top_priv_buf != nullptr && m_top_priv_buf->size > 0)
        m_mem->free(&m_top_priv_buf);

    if (m_top_reuse_buf != nullptr && m_top_reuse_buf->size > 0)
        m_mem->free(&m_top_reuse_buf);

    m_top_reuse_idx.clear();

out:
    return retval;
}

aipu_status_t aipudrv::JobV4::init_per_task_data()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t sg_idx = 0;
    bool dep_all_flag = false;

    for (uint32_t i = 0; i < m_sg_cnt; i++)
    {
        SubGraphTask &sg = m_sg_job[i];

        if (i != 0)
        {
            if (get_graph().m_subgraphs[i].precursor_cnt == SUBG_DEPEND_PREALL)
            {
                sg_idx = 0;
                dep_all_flag = true;
            }

            if (dep_all_flag && sg_idx < m_sgt_allocated.size())
            {
                for (uint32_t j = 0; j < m_task_per_sg; j++)
                {
                    Task task;
                    memset((void *)&task, 0, sizeof(task));

                    task = m_sgt_allocated[sg_idx]->tasks[j];
                    task.tcb.init(m_tcbs->pa + (i * m_task_per_sg + j + 2) * sizeof(tcb_t));
                    sg.tasks.push_back(task);
                }

                sg_idx++;
                continue;
            }
            else
            {
                dep_all_flag = false;
            }
        }

        {
            /* 1 init per-task data structs */
            for (uint32_t j = 0; j < m_task_per_sg; j++)
            {
                Task task;
                memset((void *)&task, 0, sizeof(task));

                /* 1.1. init task tcb */
                task.tcb.init(m_tcbs->pa + (i * m_task_per_sg + j + 2) * sizeof(tcb_t));

                /* 1.2. allocate task stack */
                task.stack = nullptr;
                ret = m_mem->malloc(get_graph().m_subgraphs[0].stack_size,
                                    get_graph().m_subgraphs[0].stack_align_in_page,
                                    &task.stack, "stack");
                if (AIPU_STATUS_SUCCESS != ret)
                    goto out;

                /* 1.3. allocate and load task dp */
                if (get_graph().m_subgraphs[i].private_data_size != 0)
                {
                    task.private_data = nullptr;
                    ret = m_mem->malloc(get_graph().m_subgraphs[i].private_data_size, 0,
                                        &task.private_data, "dp_data");
                    if (AIPU_STATUS_SUCCESS != ret)
                        goto out;

                    m_mem->mem_bzero(task.private_data->pa, task.private_data->size);
                }
                sg.tasks.push_back(task);
            }
            m_sgt_allocated.push_back(&sg);
        }
    }

out:
    return ret;
}

aipu_status_t aipudrv::JobV4::alloc_load_job_buffers()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    SubGraphTask sg;
    int retval = 0;

    /* 1. allocate and load job rodata */
    if (get_graph().m_brodata.size != 0)
    {
        ret = m_mem->malloc(get_graph().m_brodata.size, 0, &m_rodata, "rodata");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        m_mem->write(m_rodata->pa, get_graph().m_brodata.va, get_graph().m_brodata.size);
    }

    /* 2. allocate and load job descriptor */
    if (get_graph().m_bdesc.size != 0)
    {
        ret = m_mem->malloc(get_graph().m_bdesc.size, 0, &m_descriptor, "dcr");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        m_mem->write(m_descriptor->pa, get_graph().m_bdesc.va, get_graph().m_bdesc.size);
    }

    /* 3. allocate and reset job TCBs */
    ret = m_mem->malloc(m_tot_tcb_cnt * sizeof(tcb_t), 0, &m_tcbs, "tcbs");
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    m_mem->zeroize(m_tcbs->pa, m_tot_tcb_cnt * sizeof(tcb_t));
    m_init_tcb.init(m_tcbs->pa);

    /* 4. allocate subgraph buffers */
    retval = alloc_subgraph_buffers_optimized();
    if (retval == -1)
    {
        ret = alloc_subgraph_buffers();
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;
    } else if (retval < -1) {
        ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
        goto finish;
    }

    /* 5. init each subgraph's task tcbs */
    ret = init_per_task_data();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* 6. get IO buffer address, all subgraphs share the same copy of reuse buffers */
    create_io_buffers(get_graph().m_subgraphs[0].io, m_sg_job[0].reuses);
    if (get_subgraph_cnt() == 0)
        goto finish;

    /* 7. setup rodata & dcr, update entry for all subgraphs in global RO/DCR section */
    ret = setup_rodata_sg(0, get_graph().m_subgraphs[0].param_map, m_sg_job[0].reuses, *m_sg_job[0].weights);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* update subgraph private buffers PA in RO/DCR section */
    for (uint32_t sg = 0; sg < m_sg_cnt; sg++)
    {
        std::vector<BufferDesc *> invalid_buf;

        LOG(LOG_INFO, "sg: %d\n", sg);
        ret = setup_rodata_sg(sg, get_graph().m_subgraphs[sg].private_buffers_map,
                              m_sg_job[sg].reuse_priv_buffers, invalid_buf);
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;
    }

    /* 8. setup remap */
    setup_remap(*m_rodata, m_descriptor);

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

aipu_status_t aipudrv::JobV4::specify_io_buffer(aipu_shared_tensor_info_t &tensor_info)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const std::vector<struct JobIOBuffer> *iobuffer_vec = nullptr;
    BufferDesc *bufferDesc = nullptr;
    const char *str = "free_input";
    uint32_t reuse_index = 0;
    uint32_t type = tensor_info.type;
    uint32_t index = tensor_info.tensor_idx;
    uint64_t offset = tensor_info.offset_in_dmabuf;
    int fd = tensor_info.dmabuf_fd;
    bool update_ro = true;
    int share_case_type = tensor_info.shared_case_type;
    uint64_t buffer_pa = tensor_info.pa;
    struct aipu_dma_buf dma_buf
    {
        fd, 0, 0
    };

    switch (type)
    {
    case AIPU_TENSOR_TYPE_INPUT:
        iobuffer_vec = &m_inputs;
        break;

    case AIPU_TENSOR_TYPE_OUTPUT:
        iobuffer_vec = &m_outputs;
        str = "free_output";
        break;

    default:
        ret = AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
        LOG(LOG_ERR, "tensor type: %d, index: %d [not exist]\n",
            type, index);
        goto out;
    }

    if (index > iobuffer_vec->size())
    {
        ret = AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
        goto out;
    }

    /**
     * first check whether the input and output buffers are share one buffer,
     * if such condition occurs, return an error code to up layer, and try the
     * original calling flow other than dma_buf scheme.
     */
    reuse_index = (*iobuffer_vec)[index].ref_section_iter;
    if (type == AIPU_TENSOR_TYPE_INPUT)
    {
        for (uint32_t i = 0; i < get_graph().get_subgraph(0).io.outputs.size(); i++)
        {
            auto &idtensor_desc = get_graph().get_subgraph(0).io.outputs[i];
            if (idtensor_desc.ref_section_iter == reuse_index)
                return AIPU_STATUS_ERROR_DMABUF_SHARED_IO;
        }
    } else {
        for (uint32_t i = 0; i < get_graph().get_subgraph(0).io.inputs.size(); i++)
        {
            auto &idtensor_desc = get_graph().get_subgraph(0).io.inputs[i];
            if (idtensor_desc.ref_section_iter == reuse_index)
                return AIPU_STATUS_ERROR_DMABUF_SHARED_IO;
        }
    }

    /* free io buffer allocated internally,replace it with new buffer */
    bufferDesc = m_sg_job[0].reuses[reuse_index];
    m_sg_job[0].dma_buf_idx.insert(reuse_index);
    if (!m_optimized_reuse_alloc)
    {
        ret = m_mem->free_phybuffer(bufferDesc, str);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
    }

    switch (share_case_type)
    {
    case AIPU_SHARE_BUF_IN_ONE_PROCESS:
        bufferDesc->init(m_mem->get_asid_base(0), buffer_pa,
                         bufferDesc->size, bufferDesc->req_size);
        break;
    case AIPU_SHARE_BUF_CUSTOMED:
        bufferDesc->init(m_mem->get_asid_base(0), buffer_pa,
                         bufferDesc->size, bufferDesc->req_size);
        (*iobuffer_vec)[index].set_dump_ignore_flag(true);
        break;
    case AIPU_SHARE_BUF_DMABUF:
        ret = convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_GET_DMA_BUF_INFO, &dma_buf));
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;

        buffer_pa = dma_buf.pa + offset;
        bufferDesc->init(m_mem->get_asid_base(0), buffer_pa,
                         bufferDesc->size, bufferDesc->req_size);
        (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, offset);
        break;
    case AIPU_SHARE_BUF_ATTACH_DMABUF:
        ret = convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ATTACH_DMABUF, &dma_buf));
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;

        buffer_pa = dma_buf.pa + offset;
        bufferDesc->init(m_mem->get_asid_base(0), buffer_pa,
                         bufferDesc->size, bufferDesc->req_size);
        (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, offset);
        break;
    default:
        ret = AIPU_STATUS_ERROR_INVALID_OP;
        goto out;
    }

    LOG(LOG_DEBUG, "specify_io_buffer: pa=%lx, size=%lx, share_case_type=%d\n",
        buffer_pa, bufferDesc->size, share_case_type);

    if (update_ro)
    {
        update_io_buffers(get_graph().m_subgraphs[0].io, m_sg_job[0].reuses);
        ret = setup_rodata_sg(0, get_graph().m_subgraphs[0].param_map,
                              m_sg_job[0].reuses, *m_sg_job[0].weights, &m_sg_job[0].dma_buf_idx);
        if (AIPU_STATUS_SUCCESS != ret)
            goto out;
    }

out:
    return ret;
}

void aipudrv::JobV4::free_sg_buffers(SubGraphTask &sg)
{
    if (m_top_priv_buf != nullptr && m_top_priv_buf->size > 0)
    {
        m_mem->free(&m_top_priv_buf, "tot_priv");
        m_top_priv_buf_freed = true;
    }

    if (m_top_priv_buf_freed)
    {
        for (uint32_t i = 0; i < sg.reuse_priv_buffers.size(); i++)
            m_mem->free_bufferdesc(&sg.reuse_priv_buffers[i]);
    }
    else
    {
        for (uint32_t i = 0; i < sg.reuse_priv_buffers.size(); i++)
            m_mem->free(&sg.reuse_priv_buffers[i]);
    }
    sg.reuse_priv_buffers.clear();

    /**
     * just exist one copy of reuse buffers for all subgraphs,
     * free only once
     */
    if (sg.id == 0)
    {
        if (m_top_reuse_buf != nullptr && m_top_reuse_buf->size > 0)
        {
            m_mem->free(&m_top_reuse_buf, "tot_reuse");
            for (uint32_t i = 0; i < sg.reuses.size(); i++)
            {
                if (m_gm->gm_is_gm_buffer(i, GM_BUF_TYPE_REUSE))
                    m_mem->free(&sg.reuses[i]);
                else
                    m_mem->free_bufferdesc(&sg.reuses[i]);
            }
            m_top_reuse_idx.clear();
        }
        else
        {
            for (uint32_t i = 0; i < sg.reuses.size(); i++)
            {
                if (sg.dma_buf_idx.count(i) == 1)
                {
                    m_mem->free_bufferdesc(&sg.reuses[i]);
                    continue;
                }

                m_mem->free(&sg.reuses[i]);
            }
        }

        sg.reuses.clear();
        sg.weights = nullptr;
    }

    for (uint32_t i = 0; i < m_sgt_allocated.size(); i++)
    {
        for (uint32_t j = 0; j < m_task_per_sg; j++)
        {
            Task *task;
            task = &m_sgt_allocated[i]->tasks[j];
            m_mem->free(&task->stack);
            m_mem->free(&task->private_data);
        }
    }
    m_sgt_allocated.clear();
}

aipu_status_t aipudrv::JobV4::free_job_buffers()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (m_rodata && m_rodata->size != 0)
        m_mem->free(&m_rodata, "rodata");

    if (m_descriptor && m_descriptor->size != 0)
        m_mem->free(&m_descriptor, "dcr");

    if (m_tcbs && m_tcbs->size != 0)
        m_mem->free(&m_tcbs, "tcbs");

    if (m_pprint && m_pprint->size != 0)
        m_mem->free(&m_pprint, "printf");

    m_init_tcb.init(0);

    for (uint32_t i = 0; i < m_sg_job.size(); i++)
    {
        free_sg_buffers(m_sg_job[i]);
        m_sg_job[i].reset(i);
    }

    m_sg_job.clear();
    m_inputs.clear();
    m_outputs.clear();
    m_inter_dumps.clear();
    m_profiler.clear();
    m_printf.clear();
    m_layer_counter.clear();

    m_dev->put_start_group_id(m_start_group_id, m_sg_cnt);

    return ret;
}

aipu_status_t aipudrv::JobV4::config_smmu_tcb(tcb_t *tcb)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (m_segmmu_num > 0)
    {
        if (m_segmmu_num == 1)
        {
            SegMMUConfig &segmmu = m_segmmu_sec[0];

            tcb->segmmu_ctrl = segmmu.SegMMU_ctl;
            tcb->segmmu_remap_ctrl0 = segmmu.SegMMU_remap;
            tcb->segmmu_remap_ctrl1 = segmmu.SegMMU_remap;

            for (int j = 0; j < 4; j++)
            {
                tcb->segmmu_seg_ctrl[2 * j] = segmmu.seg[j].control[0];
                tcb->segmmu_seg_ctrl[2 * j + 1] = segmmu.seg[j].control[1];
            }
        }
    }

    return ret;
}

aipu_status_t aipudrv::JobV4::setup_tcb_task(uint32_t sg_id, uint32_t grid_id,
    uint32_t core_id, uint32_t task_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphV3X &graph = get_graph();
    Task &task = m_sg_job[sg_id].tasks[task_id];
    tcb_t *tcb = new tcb_t;
    TCB* next_tcb = nullptr;

    if (task_id != (m_task_per_sg - 1))
        next_tcb = &m_sg_job[sg_id].tasks[task_id + 1].tcb;
    else if (sg_id != (m_sg_cnt - 1))
        next_tcb = &m_sg_job[sg_id + 1].tasks[0].tcb;
    else
        next_tcb = nullptr;

    memset(tcb, 0, sizeof(tcb_t));
    tcb->interrupt_en = EN_INTERRUPT_TEC_ALL;
    tcb->flag = TCB_FLAG_TASK_TYPE_TASK;

    if ((task_id == 0) && (graph.m_broadcast_info.count(sg_id) == 1))
    {
        tcb->flag |= TCB_FLAG_CORE_NUM(graph.m_broadcast_info[sg_id]);
        tcb->flag |= TCB_FLAG_BROADCAST_START;
    }

    if (next_tcb != nullptr)
    {
        tcb->_next = get_low_32(next_tcb->pa);
    } else {
        tcb->_next = 0;
    }

    if (task_id == (m_task_per_sg - 1))
        tcb->flag |= TCB_FLAG_END_TYPE_GROUP_END;

    if ((sg_id == (m_sg_cnt - 1)) && (task_id == (m_task_per_sg - 1)))
        tcb->flag |= TCB_FLAG_END_TYPE_GRID_END;

    /* It is assumed that subgraphs are topology sorted. */
    if (task_id == 0)
    {
        switch (graph.m_subgraphs[sg_id].precursor_cnt)
        {
            case SUBG_DEPEND_NONE:
                tcb->flag |= TCB_FLAG_DEP_TYPE_NONE;
                break;

            case 1 ... 4:
                {
                    uint16_t dep_group_id = 0;

                    tcb->flag |= TCB_FLAG_DEP_TYPE_GROUP;
                    for (int32_t i = 0; i < graph.m_subgraphs[sg_id].precursor_cnt; i++)
                    {
                        if (graph.m_subgraphs[sg_id].precursors[i] > 0x7fff)
                        {
                            LOG(LOG_ERR, "Depend group id(%d) is invalid\n",
                                graph.m_subgraphs[sg_id].precursors[i]);
                            delete tcb;
                            return AIPU_STATUS_ERROR_INVALID_GBIN;
                        }

                        dep_group_id = graph.m_subgraphs[sg_id].precursors[i] + m_start_group_id;
                        dep_group_id &= 0x7FFF; // 15 bits group id field
                        tcb->group_deps[i] = EN_GROUP_DEPEND | dep_group_id;
                    }
                }
                break;

            case SUBG_DEPEND_PREALL:
                tcb->flag |= TCB_FLAG_DEP_TYPE_PRE_ALL;
                break;

            default:
                LOG(LOG_ERR, "subgraph %u, precursor_cnt=%d\n", sg_id,
                    graph.m_subgraphs[sg_id].precursor_cnt);
                delete tcb;
                return AIPU_STATUS_ERROR_INVALID_GBIN;
        }
    }

    tcb->spc = get_low_32(graph.m_text->align_asid_pa + graph.m_subgraphs[sg_id].text.offset);
    tcb->groupid = (uint16_t)m_group_id_idx;
    tcb->gridid = (uint16_t)grid_id;
    tcb->taskid = (uint16_t)task_id;
    tcb->ica_warmup_len = graph.m_subgraphs[sg_id].warmup_len;
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
    tcb->tcbp = get_low_32(task.tcb.pa - m_tcbs->asid_base);
    tcb->sp = get_low_32(task.stack->align_asid_pa);
    tcb->pp = get_low_32(m_rodata->align_asid_pa + graph.m_subgraphs[sg_id].rodata.offset);
    tcb->dp = get_low_32(task.private_data->align_asid_pa);

    /* const rodata */
    if (graph.m_crodata != nullptr && graph.m_crodata->size > 0)
        tcb->cp = get_low_32(graph.m_crodata->align_asid_pa);

    /* update profile buffer offset according to subgraph index */
    if (m_profiler.size() > 0)
        tcb->pprofiler = get_low_32(m_profiler[0].align_asid_pa + graph.m_subgraphs[sg_id].profiler_buf_size);

    if (graph.m_subgraphs[sg_id].printfifo_size > 0)
    {
        uint32_t pa = m_pprint->align_asid_pa + AIPU_PAGE_SIZE * core_id + 1024 * task_id;
        tcb->pprint = get_low_32(pa);
        tcb->interrupt_en |= EN_INTERRUPT_TEC_SIGNAL;
    }

    /* flush TCB to AIPU mem */
    m_mem->write(task.tcb.pa, (const char *)tcb, sizeof(*tcb));

    delete tcb;
    return ret;
}

aipu_status_t aipudrv::JobV4::setup_tcb_sg(uint32_t sg_id, uint32_t grid_id, uint32_t core_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    /* setup task TCBs */
    for (uint32_t t = 0; t < m_task_per_sg; t++)
    {
        ret = setup_tcb_task(sg_id, grid_id, core_id, t);
        if (AIPU_STATUS_SUCCESS != ret)
            return ret;
    }

    /* increase group index for each group */
    m_group_id_idx++;

    return ret;
}

aipu_status_t aipudrv::JobV4::setup_tcbs()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    tcb_t *tcb = new tcb_t;
    uint32_t core_id = 0;

    /* 1. Grid init TCB 0 */
    memset(tcb, 0, sizeof(tcb_t));
    tcb->flag = TCB_FLAG_TASK_TYPE_GRID_INIT /*| TCB_FLAG_L2D_FLUSH*/;
    tcb->group_num = m_sg_cnt;
    tcb->grid_intrrupt_en = EN_INTERRUPT_GRID_ALL;
    tcb->grid_gridid = m_grid_id;
    tcb->grid_groupid = m_group_id_idx;

    setup_gm_sync_from_ddr(tcb);
    m_mem->write(m_init_tcb.pa, (const char *)tcb, sizeof(tcb_t));

    /* 2. Group init TCB 1 */
    memset(tcb, 0, sizeof(tcb_t));
    tcb->flag = TCB_FLAG_TASK_TYPE_GROUP_INIT | TCB_FLAG_GRID_INIT;
    // tcb->group_interrupt_en = EN_INTERRUPT_GROUP_DONE;
    tcb->group_gridid = m_grid_id;
    tcb->group_groupid = m_group_id_idx;

    // SegMMU
    // config_smmu_tcb(tcb);

    // ASID
    for (uint32_t i = 0; i < 4; i++)
    {
        tcb->asids[2 * i] = get_low_32(m_mem->get_asid_base(i) | ASID_RD | ASID_WR);
        tcb->asids[2 * i + 1] = get_high_32(m_mem->get_asid_base(i));
    }
    m_mem->write(m_init_tcb.pa + sizeof(*tcb), (const char *)tcb, sizeof(*tcb));

    /* 3. Task TCB */
    for (uint32_t i = 0; i < get_graph().m_subgraphs.size(); i++)
    {
        ret = setup_tcb_sg(get_graph().m_subgraphs[i].id, m_grid_id, core_id);
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        if (++core_id >= m_core_cnt)
            core_id = 0;
    }

    /**
     * store aligned TEXT and RO base at tail of text buffer for debugger
     */
    m_mem->write(get_graph().m_text->pa + get_graph().m_btext.size,
        &get_graph().m_text->align_asid_pa, 4);
    m_mem->write(get_graph().m_text->pa + get_graph().m_btext.size + 4,
        &m_rodata->align_asid_pa, 4);

    m_status = AIPU_JOB_STATUS_INIT;

finish:
    delete tcb;
    return ret;
}

aipu_status_t aipudrv::JobV4::init(const aipu_global_config_simulation_t *cfg,
                                   const aipu_global_config_hw_t *hw_cfg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    m_cfg = cfg;
    m_hw_cfg = hw_cfg;

    m_dev->get_core_count(m_partition_id, 0, &m_core_cnt);
    set_job_params(get_graph().m_subgraphs.size(), 4,
                   get_graph().m_remap_flag, m_core_cnt);

    if (m_dev->get_grid_id(m_grid_id) < 0)
    {
        ret = AIPU_STATUS_ERROR_ALLOC_GRIP_ID;
        goto finish;
    }

    if (m_dev->get_start_group_id(m_sg_cnt, m_start_group_id) < 0)
    {
        ret = AIPU_STATUS_ERROR_ALLOC_GROUP_ID;
        goto finish;
    }
    m_group_id_idx = m_start_group_id;

    /* allocate and load job buffers */
    ret = alloc_load_job_buffers();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /**
     * no need to create TCBs if there's no subgraphs,
     * just directly return.
     */
    if (get_subgraph_cnt() == 0)
    {
        m_status = AIPU_JOB_STATUS_INIT;
        goto finish;
    }

    ret = setup_tcbs();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    if (m_backup_tcb != nullptr)
        m_mem->read(m_init_tcb.pa, m_backup_tcb.get(), m_tot_tcb_cnt * sizeof(tcb_t));

finish:
    return ret;
}

aipu_status_t aipudrv::JobV4::schedule()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobDesc desc;

    ret = validate_schedule_status();
    if (ret != AIPU_STATUS_SUCCESS)
    {
        LOG(LOG_ERR, "Job state %d is invalid", m_status);
        return ret;
    }

    if (get_subgraph_cnt() == 0)
        return ret;

    if (m_err_code.size() == 1)
        m_mem->zeroize(m_err_code[0].pa, m_err_code[0].size);

    /* with the backup tcbchain if run the job again */
    if (m_backup_tcb != nullptr && m_backup_tcb_used == true)
        m_mem->write(m_init_tcb.pa, m_backup_tcb.get(), m_tot_tcb_cnt * sizeof(tcb_t));
    m_backup_tcb_used = true;

    dump_job_shared_buffers();
    dump_job_private_buffers(*m_rodata, m_descriptor);
    dump_specific_buffers();

    memset(&desc.kdesc, 0, sizeof(desc.kdesc));

    /* for simulation */
    desc.kdesc.job_id = m_id;
    desc.kdesc.version_compatible = !get_graph().m_do_vcheck;
    desc.kdesc.aipu_config = get_graph().m_hw_config;
    desc.jobbase = this;
    desc.tcb_number = m_tot_tcb_cnt;
    desc.tcb_head = m_init_tcb.pa;
    desc.tcb_tail = m_sg_job[m_sg_cnt - 1].tasks[m_task_per_sg - 1].tcb.pa;

    /* for HW */
    desc.kdesc.exec_flag = (m_qos == AIPU_JOB_QOS_HIGH)
                               ? AIPU_JOB_EXEC_FLAG_QOS_FAST
                               : AIPU_JOB_EXEC_FLAG_QOS_SLOW;

    if (m_dbg_dispatch)
    {
        desc.kdesc.exec_flag |= AIPU_JOB_EXEC_FLAG_DBG_DISPATCH;
        desc.kdesc.core_id = m_core_id;
    }
    else
    {
        desc.kdesc.core_id = 0;
    }

    desc.kdesc.enable_poll_opt = !m_hw_cfg->poll_in_commit_thread;
    desc.kdesc.aipu_version = get_graph().m_hw_version;
    desc.kdesc.partition_id = m_partition_id;
    desc.kdesc.head_tcb_pa = m_init_tcb.pa;
    desc.kdesc.tail_tcb_pa = m_sg_job[m_sg_cnt - 1].tasks[m_task_per_sg - 1].tcb.pa;

    /* for debugger */
    desc.kdesc.is_defer_run = m_is_defer_run;
    desc.kdesc.do_trigger = m_do_trigger;

    if (get_graph().m_text->size == 0)
        LOG(LOG_WARN, "Graph text size is 0\n");
    else
        ret = m_dev->schedule(desc);

#ifdef SIMULATION
    ret = dump_for_emulation();
#endif
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    m_status = AIPU_JOB_STATUS_SCHED;

    return ret;
}

aipu_status_t aipudrv::JobV4::destroy()
{
    return free_job_buffers();
}

void aipudrv::JobV4::dump_specific_buffers()
{
    DEV_PA_64 dump_pa;
    uint32_t dump_size;

    if (m_dump_tcb)
    {
        dump_pa = m_tcbs->pa;
        dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
        if (dump_size != 0)
            dump_buffer(dump_pa, nullptr, dump_size, "TCBs");
    }

    if (m_dump_profile && m_profiler.size() > 0)
    {
        std::string profile_file_name = m_dump_dir + "/" + m_dump_misc_prefix + "_PerfData.bin";
        m_profile_fd = open(profile_file_name.c_str(), O_RDWR | O_CREAT, 0644);
        if (m_profile_fd < 0)
            LOG(LOG_ALERT, "open: %s [fail], ret: %d\n", profile_file_name.c_str(), m_profile_fd);
        else
            chmod(profile_file_name.c_str(), 0644);

        convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ENABLE_TICK_COUNTER, nullptr));
    }
}

aipu_status_t aipudrv::JobV4::dump_for_emulation()
{
#define SINGLE_TCB_BIN
#ifdef SINGLE_TCB_BIN
#define INIT_NUM 3
#else
#define INIT_NUM 4
#endif

    DEV_PA_64 dump_pa = 0;
    uint32_t dump_size = 0;
    char dump_name[4096] = {0};
    int emu_input_cnt = INIT_NUM + m_inputs.size() + (m_descriptor != nullptr ? 1 : 0);
    int emu_output_cnt = m_outputs.size();
    int file_id = -1;
    tcb_t tcb = {0};
    bool default_output_prefix = true;
    std::string runtime_cfg = m_dump_dir + "/runtime.cfg";
    std::string metadata_txt = m_dump_dir + "/metadata.txt";
    std::map<uint32_t, std::string> gm_info = {
        {512 << 10, "512K"},
        {1 << 20, "1M"},
        {2 << 20, "2M"},
        {4 << 20, "4M"},
        {8 << 20, "8M"},
        {16 << 20, "16M"},
        {32 << 20, "32M"},
        {64 << 20, "64M"},
    };

    if (m_dump_emu == false)
        return AIPU_STATUS_SUCCESS;

    FileWrapper ofs(runtime_cfg, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    FileWrapper ofsmt(metadata_txt, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
    if (!ofs.is_open() || !ofsmt.is_open())
        return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

    ofs << "[COMMON]\n";

    /* runtime.cfg: config */
    ofs << "#configuration 1:tbd1 2:tbd2\n";
    if (m_dev->get_config_code() != nullptr)
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
    ofs << "#if ENABLE_CALLOC is true the allocation memory is set to zero.\n";
    if (m_cfg->enable_calloc)
        ofs << "ENABLE_CALLOC=true\n";
    else
        ofs << "ENABLE_CALLOC=false\n";

    /* runtime.cfg: gm_size */
    ofs << "#GM_V4 support: 512KiB,1MiB,2MiB,4MiB,8MiB,16MiB,32MiB,64MiB.\n";
    if (gm_info.count(m_cfg->gm_size) == 1)
        ofs << "GM_SIZE=" << gm_info[m_cfg->gm_size] << "\n";

    if (m_cfg->plugin_name != nullptr)
    {
        ofs << "#PLUGIN_FILENAME\n";
        ofs << "PLUGIN_FILENAME=" << m_cfg->plugin_name << "\n";
    }

    if (m_cfg->json_filename != nullptr)
    {
        ofs << "#JSON_FILENAME\n";
        ofs << "JSON_FILENAME=" << m_cfg->json_filename << "\n";
    }

    /* runtime.cfg: en_eval */
    ofs << "\n[PROFILE]\n";
    if (m_cfg->en_eval)
        ofs << "EN_EVAL=1\n";
    else
        ofs << "EN_EVAL=0\n";

    if (m_profiler.size() == 1)
    {
        ofs << "PROFILE_BUF_ADDR=0x" << std::hex << m_profiler[0].pa << "\n";
        ofs << "PROFILE_BUF_SIZE=0x" << std::hex << m_profiler[0].size << "\n";
    }
    ofs << "\n";

    ofs.dump_to_string(m_dumpcfg_header);

    /* runtime.cfg: [INPUT] */
    if (get_graph().m_weight != nullptr && get_graph().m_weight->size > 0)
    {
        emu_input_cnt += 1;
        if (get_graph().m_zerocpy_const != nullptr && get_graph().m_zerocpy_const->size != 0)
            emu_input_cnt += 1;
    }
    else
        emu_input_cnt += m_sg_job[0].weights->size();

    ofs << "[INPUT]\n";
    ofs << "COUNT=" << emu_input_cnt << "\n";

    /* dump temp.text */
    dump_pa = get_graph().m_text->pa;
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
    if (get_graph().m_weight != nullptr && get_graph().m_weight->req_size > 0)
    {
        dump_pa = get_graph().m_weight->pa;
        dump_size = get_graph().m_weight->req_size;
        if (dump_size != 0)
        {
            snprintf(dump_name, 128, "%s/%s.weight", m_dump_dir.c_str(), m_dump_prefix.c_str());
            m_mem->dump_file(dump_pa, dump_name, dump_size);

            ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".weight\n";
            ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
            m_dumpcfg_input.push_back({dump_name, dump_pa});

            if (get_graph().m_zerocpy_const != nullptr && get_graph().m_zerocpy_const->size > 0)
            {
                dump_pa = get_graph().m_zerocpy_const->pa;
                dump_size = get_graph().m_zerocpy_const->req_size;
                snprintf(dump_name, 128, "%s/%s.zerocpy_const", m_dump_dir.c_str(), m_dump_prefix.c_str());
                m_mem->dump_file(dump_pa, dump_name, dump_size);

                ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".zerocpy_const\n";
                ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
                m_dumpcfg_input.push_back({dump_name, dump_pa});
            }
        }
    }
    else
    {
        for (uint32_t i = 0; i < m_sg_job[0].weights->size(); i++)
        {
            dumpcfg_input_desc input_desc;
            std::string name;

            dump_pa = (*m_sg_job[0].weights)[i]->pa;
            dump_size = (*m_sg_job[0].weights)[i]->size;
            // snprintf(dump_name, 128, "%s/%s.weight", m_dump_dir.c_str(), m_dump_prefix.c_str());
            name = m_dump_dir + "/" + m_dump_prefix + ".weight" + std::to_string(i);
            m_mem->dump_file(dump_pa, name.c_str(), dump_size);

            ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".weight\n";
            ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
            input_desc.file = name;
            input_desc.base = dump_pa;
            m_dumpcfg_input.push_back(input_desc);
        }
    }

    /* dump temp.rodata */
    dump_pa = m_rodata->pa;
    dump_size = m_rodata->size;
    snprintf(dump_name, 128, "%s/%s.ro", m_dump_dir.c_str(), m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);
    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".ro\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});

    /* dump temp.dcr */
    if (m_descriptor != nullptr && m_descriptor->size != 0)
    {
        dump_pa = m_descriptor->pa;
        dump_size = m_descriptor->size;
        snprintf(dump_name, 128, "%s/%s.dcr", m_dump_dir.c_str(), m_dump_prefix.c_str());
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".dcr\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
    }

    /* dump temp.tcb */
    dump_pa = m_tcbs->pa;
    dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
    snprintf(dump_name, 128, "%s/%s.tcb", m_dump_dir.c_str(), m_dump_prefix.c_str());
    // m_dump_tcb_info[0] = std::make_tuple(m_dump_dir + "/init.tcb", dump_pa,
    //                                      (1 + (m_segmmu_tcb_num + 1) / 2) * sizeof(tcb_t));
    // m_dump_tcb_info[1] = std::make_tuple(m_dump_dir + "/task.tcb",
    //                                      dump_pa + std::get<2>(m_dump_tcb_info[0]), dump_size - std::get<2>(m_dump_tcb_info[0]));
    // m_mem->dump_file(std::get<1>(m_dump_tcb_info[0]), std::get<0>(m_dump_tcb_info[0]).c_str(),
    //                  std::get<2>(m_dump_tcb_info[0]));
    // m_mem->dump_file(std::get<1>(m_dump_tcb_info[1]), std::get<0>(m_dump_tcb_info[1]).c_str(),
    //                  std::get<2>(m_dump_tcb_info[1]));
    m_mem->dump_file(dump_pa, dump_name, dump_size);

#ifdef SINGLE_TCB_BIN
    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".tcb\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
#else
    ofs << "FILE" << std::dec << ++file_id << "="
        << "init.tcb\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << std::get<1>(m_dump_tcb_info[0]) << "\n";

    ofs << "FILE" << std::dec << ++file_id << "="
        << "task.tcb\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << std::get<1>(m_dump_tcb_info[1]) << "\n";
#endif
    /* dump temp.input[n] */
    for (uint32_t i = 0; i < m_inputs.size(); i++)
    {
        if (m_inputs[i].dump_ignore_flag)
            continue;

        dump_pa = m_inputs[i].pa;
        dump_size = m_inputs[i].size;
        snprintf(dump_name, 128, "%s/%s.input%u", m_dump_dir.c_str(), m_dump_prefix.c_str(), i);

        if (m_inputs[i].dmabuf_fd < 0)
            m_mem->dump_file(dump_pa, dump_name, dump_size);
        else
            dump_share_buffer(m_inputs[i], dump_name, true);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".input" << i << "\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});
    }
    ofs << "\n";

    ofs << "[HOST]\n";
    ofs << "TCBP_HI=0x" << std::hex << get_high_32(m_init_tcb.pa) << "\n";
    ofs << "TCBP_LO=0x" << get_low_32(m_init_tcb.pa) << "\n";
    ofs << "TCB_NUM=0x" << std::hex << m_tot_tcb_cnt << "\n";
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
        if (m_outputs[i].dump_ignore_flag)
            continue;

        dump_pa = m_outputs[i].pa;
        dump_size = m_outputs[i].size;

        if (default_output_prefix)
        {
            ofs << "FILE" << std::dec << i
                << "=" << m_dump_output_prefix << ".output" << i << "\n";
            snprintf(dump_name, 128, "%s/%s.output%u", m_dump_dir.c_str(), m_dump_prefix.c_str(), i);
            m_dumpcfg_output.push_back({dump_name, dump_pa, dump_size});
        }
        else
        {
            if (0 == i)
            {
                ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << "\n";
            }
            else
            {
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

    /* Grid/Group init TCB and Task TCB */
    for (uint32_t i = 0; i < m_tot_tcb_cnt; i++)
    {
        m_mem->read(m_init_tcb.pa + sizeof(tcb) * i, &tcb, sizeof(tcb));

        if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_GRID_INIT)
        {
            ofsmt << "\n***GRID INIT TCB " << std::dec << i << " ***\n";

            ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
            ofsmt << "group_num: " << std::dec << tcb.group_num << "\n";
            ofsmt << "grid_intrrupt_en: 0x" << std::hex << tcb.grid_intrrupt_en << "\n";
            ofsmt << "grid_groupid: " << std::dec << tcb.grid_groupid << "\n";
            ofsmt << "grid_gridid: " << tcb.grid_gridid << "\n";
            ofsmt << "gm_ctrl: 0x" << std::hex << tcb.gm_ctrl << "\n";
            ofsmt << "gm_sync: 0x" << tcb.gm_sync << "\n";
            ofsmt << "gm_addr_low: 0x" << tcb.gm_addr_low << "\n";
            ofsmt << "gm_addr_high: 0x" << tcb.gm_addr_high << "\n";
        } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_GROUP_INIT) {
            ofsmt << "\n***GROUP INIT TCB " << std::dec << i << " ***\n";

            ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
            ofsmt << "segmmu_ctrl: 0x" << tcb.segmmu_ctrl << "\n";
            ofsmt << "segmmu_remap_ctrl0: 0x" << tcb.segmmu_remap_ctrl0 << "\n";
            ofsmt << "segmmu_remap_ctrl1: 0x" << tcb.segmmu_remap_ctrl1 << "\n";
            ofsmt << "group_interrupt_en: " << std::hex << tcb.group_interrupt_en << "\n";
            ofsmt << "group_groupid: " << std::dec << tcb.group_groupid << "\n";
            ofsmt << "group_gridid: " << tcb.group_gridid << "\n";

            for (int j = 0; j < 8; j++)
            {
                ofsmt << "segmmu_seg" << std::dec << j << "_ctrl0: 0x"
                      << std::hex << tcb.segmmu_seg_ctrl[2 * i] << "\n";
                ofsmt << "segmmu_seg" << std::dec << j << "_ctrl1: 0x"
                      << std::hex << tcb.segmmu_seg_ctrl[2 * i + 1] << "\n";
            }

            for (int j = 0; j < 4; j++)
            {
                ofsmt << "ASID" << std::dec << j << "_LO: 0x"
                      << std::hex << tcb.asids[2 * i] << "\n";
                ofsmt << "ASID" << std::dec << j << "_HI: 0x"
                      << std::hex << tcb.asids[2 * i + 1] << "\n";
            }
        } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_TASK) {
            ofsmt << "\n***TASK TCB " << std::dec << i << " ***\n";

            ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
            ofsmt << "next: 0x" << std::hex << tcb._next << "\n";
            ofsmt << "start_pc: 0x" << std::hex << tcb.spc << "\n";
            ofsmt << "interrupt_en: 0x" << tcb.interrupt_en << "\n";

            ofsmt << "group_id: " << std::dec << tcb.groupid << "\n";
            ofsmt << "grid_id: " << tcb.gridid << "\n";
            ofsmt << "task_id: " << tcb.taskid << "\n";
            ofsmt << "warm_len: " << tcb.ica_warmup_len << "\n";

            ofsmt << "grid_dim_x: " << tcb.grid_dim_x << "\n";
            ofsmt << "grid_dim_y: " << tcb.grid_dim_y << "\n";
            ofsmt << "grid_dim_z: " << tcb.grid_dim_z << "\n";

            ofsmt << "group_dim_x: " << tcb.group_dim_x << "\n";
            ofsmt << "group_dim_y: " << tcb.group_dim_y << "\n";
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
            ofsmt << "dsize: 0x" << tcb.dsize << "\n";
            ofsmt << "tcbp: 0x" << tcb.tcbp << "\n";

            ofsmt << "group_deps[0]: " << tcb.group_deps[0] << "\n";
            ofsmt << "group_deps[1]: " << tcb.group_deps[1] << "\n";
            ofsmt << "group_deps[2]: " << tcb.group_deps[2] << "\n";
            ofsmt << "group_deps[3]: " << tcb.group_deps[3] << "\n";
        } else {
            LOG(LOG_ERR, "invalid TCB type\n");
        }
    }

    ofsmt << "\n***IO Tensors***\n";
    for (uint32_t i = 0; i < m_inputs.size(); i++)
    {
        dump_pa = m_inputs[i].pa;
        dump_size = m_inputs[i].size;

        ofsmt << "input" << std::dec << i << "_addr: 0x" << std::hex << dump_pa << "\n";
        ofsmt << "input" << std::dec << i << "_size: 0x" << std::hex << dump_size << "\n";
    }

    for (uint32_t i = 0; i < m_outputs.size(); i++)
    {
        dump_pa = m_outputs[i].pa;
        dump_size = m_outputs[i].size;

        ofsmt << "output" << std::dec << i << "_addr: 0x" << std::hex << dump_pa << "\n";
        ofsmt << "output" << std::dec << i << "_size: 0x" << std::hex << dump_size << "\n";
    }

    ofsmt.dump_to_string(m_dumpcfg_meta);
    /* close metadata.txt */
    ofsmt.close();
    return AIPU_STATUS_SUCCESS;
}

#if defined(SIMULATION)
void aipudrv::JobV4::dumpcfg_alljob()
{
    JobV4 *job = nullptr;
    std::vector<uint32_t> cluster_id[4];
    uint32_t count = 0, cmdpool_mask = 0;
    MainContext *ctx = static_cast<MainContext *>(get_graph().m_ctx);
    GraphTable &graphs = ctx->get_graphtable();
    GraphV3X *graph = nullptr;
    std::ostringstream oss;
    static bool dump_done = false;
    static std::mutex mtex;

    {
        std::lock_guard<std::mutex> _lock(mtex);
        if (dump_done)
            return;
        dump_done = true;
    }

    if (!m_dump_emu)
        return;

    FileWrapper ofs("./runtime.cfg", std::ios::out);
    FileWrapper ofsmt("./metadata.txt", std::ios::out);
    if (!ofs.is_open() || !ofsmt.is_open())
        return;

    /* runtime.cfg: [COMMON] */
    ofs << m_dumpcfg_header << "\n";

    /* runtime.cfg: [INPUT] */
    count = 0;
    auto graph_iter = graphs.begin();
    for (auto g : graphs)
    {
        graph_iter++;
        graph = static_cast<GraphV3X *>(g.second);
        auto job_iter = graph->m_jobs.begin();
        for (auto item : graph->m_jobs)
        {
            job_iter++;
            job = static_cast<JobV4 *>(item.second);
            for (uint32_t i = 0; i < job->m_dumpcfg_input.size(); i++)
            {
                oss << "FILE" << std::dec << count << "=" << job->m_dumpcfg_input.at(i).file << "\n";
                oss << "BASE" << count << "=0x" << std::hex << job->m_dumpcfg_input.at(i).base << "\n";
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
    cmdpool_mask = 1;
    for (auto g : graphs)
    {
        graph = static_cast<GraphV3X *>(g.second);
        for (auto item : graph->m_jobs)
        {
            job = static_cast<JobV4 *>(item.second);
            if (cmdpool_mask & (1 << job->m_bind_cmdpool_id))
            {
                oss << "SET_PARTITION" << std::dec << count << "=" << job->m_dumpcfg_host.part_id << "\n";
                oss << "TCBP_HI" << std::dec << count << "=0x" << std::hex << job->m_dumpcfg_host.hi_addr << "\n";
                oss << "TCBP_LO" << std::dec << count << "=0x" << std::hex << job->m_dumpcfg_host.lo_addr << "\n";
                count++;
                cmdpool_mask &= ~(1 << job->m_bind_cmdpool_id);
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
        graph = static_cast<GraphV3X *>(g.second);
        for (auto item : graph->m_jobs)
        {
            job = static_cast<JobV4 *>(item.second);
            for (uint32_t i = 0; i < job->m_dumpcfg_output.size(); i++)
            {
                oss << "FILE" << std::dec << count << "=" << job->m_dumpcfg_output.at(i).file << "\n";
                oss << "BASE" << std::dec << count << "=0x" << std::hex << job->m_dumpcfg_output.at(i).base << "\n";
                oss << "SIZE" << std::dec << count << "=0x" << std::hex << job->m_dumpcfg_output.at(i).size << "\n";
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
        graph = static_cast<GraphV3X *>(g.second);
        for (auto item : graph->m_jobs)
        {
            job = static_cast<JobV4 *>(item.second);
            ofsmt << job->m_dumpcfg_meta;
            ofsmt << "\n";
        }
        ofsmt << "\n";
    }
    ofsmt.close();
}
#endif

aipu_status_t aipudrv::JobV4::bind_core(uint32_t partition_id)
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

aipu_status_t aipudrv::JobV4::debugger_run()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;

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