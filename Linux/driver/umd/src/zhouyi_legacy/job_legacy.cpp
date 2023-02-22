// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  job_legacy.cpp
 * @brief AIPU User Mode Driver (UMD) legacy job module implementation
 */

#include <cstring>
#include <unistd.h>
#include "job_legacy.h"

aipudrv::JobLegacy::JobLegacy(MainContext* ctx, GraphBase& graph,
    DeviceBase* dev, aipu_create_job_cfg_t *config):
    JobBase(ctx, graph, dev)
{
    m_stack.reset();

    if (config != nullptr)
    {
        m_fm_mem_region = config->fm_mem_region;
        graph.set_weight_region(config->wt_mem_region);
    }
}

aipudrv::JobLegacy::~JobLegacy()
{
}

aipu_status_t aipudrv::JobLegacy::setup_rodata_legacy()
{
    const std::vector<struct GraphParamMapLoadDesc>& param_map =
        get_graph().m_param_map;

    return setup_rodata(param_map, m_reuses, m_weights, m_rodata, m_descriptor);
}

aipu_status_t aipudrv::JobLegacy::init(const aipu_global_config_simulation_t* cfg,
    const aipu_global_config_hw_t* hw_cfg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    m_cfg = cfg;
    m_hw_cfg = hw_cfg;

#if (defined SIMULATION)
    if (nullptr == cfg)
        return AIPU_STATUS_ERROR_INVALID_CONFIG;

    if (((get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V1) && (cfg->z1_simulator == nullptr)) ||
        ((get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V2) && (cfg->z2_simulator == nullptr)) ||
        ((get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V3) && (cfg->z3_simulator == nullptr)) ||
        ((get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_X1) && (cfg->x1_simulator == nullptr)))
    {
        return AIPU_STATUS_ERROR_INVALID_CONFIG;
    }

    if (cfg->z1_simulator != nullptr)
        m_z1_sim = cfg->z1_simulator;
    if (cfg->z2_simulator != nullptr)
        m_z2_sim = cfg->z2_simulator;
    if (cfg->z3_simulator != nullptr)
        m_z3_sim = cfg->z3_simulator;
    if (cfg->x1_simulator != nullptr)
        m_x1_sim = cfg->x1_simulator;

    m_log_level = cfg->log_level;
    m_en_eval = cfg->en_eval;

    if (cfg->log_file_path != nullptr)
        m_log_path = cfg->log_file_path;
#endif

    /* 1. allocate and load job rodata */
    ret = m_mem->malloc(get_graph().m_brodata.size, 0, &m_rodata, "rodata");
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    m_mem->write(m_rodata.pa, get_graph().m_brodata.va, get_graph().m_brodata.size);

    /* 2. allocate and load job descriptor */
    if (get_graph().m_bdesc.size != 0)
    {
        ret = m_mem->malloc(get_graph().m_bdesc.size, 0, &m_descriptor, "dcr");
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        m_mem->write(m_descriptor.pa, get_graph().m_bdesc.va, get_graph().m_bdesc.size);
    }

    /* 3. allocate task stack */
    ret = m_mem->malloc(get_graph().m_stack_size, get_graph().m_stack_align_in_page,
            &m_stack, "stack");
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* 4. allocate reuse buffers */
    for (uint32_t i = 0; i < get_graph().m_reuse_sections.size(); i++)
    {
        uint32_t size = get_graph().m_reuse_sections[i].size;
        uint32_t align_in_page = get_graph().m_reuse_sections[i].align_in_page;
        BufferDesc bufferDesc;

        if (get_graph().m_shared_tensor_map.count(i) == 1)
        {
            Buffer buffer;
            if(m_mem->get_shared_buffer(get_graph().m_shared_tensor_map[i], size, buffer) != 0)
            {
                ret = AIPU_STATUS_ERROR_SET_SHARED_TENSOR;
                goto finish;
            }
            bufferDesc = buffer.desc;
        } else {
            bufferDesc.reset();
            if (size != 0)
            {
                std::string str = "reuse_" + std::to_string(i);
                ret = m_mem->malloc(size, align_in_page, &bufferDesc, str.c_str(), m_fm_mem_region);
                if (AIPU_STATUS_SUCCESS != ret)
                    goto finish;
            }
        }

        if (m_dump_reuse)
            m_mem->mem_bzero(bufferDesc.pa, bufferDesc.size);

        m_reuses.push_back(bufferDesc);
    }

    /* 5. init weights address */
    if ((ret = get_graph().alloc_weight_buffer(get_graph().m_static_sections)) != AIPU_STATUS_SUCCESS)
        goto finish;

    m_weights.assign(get_graph().m_weights.begin(), get_graph().m_weights.end());

    /* 6. update rodata & dcr */
    ret = setup_rodata_legacy();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* 7. setup remap */
    setup_remap(m_rodata, m_descriptor);

    /* 8. get IO buffer address */
    create_io_buffers(get_graph().m_io, m_reuses);

    /* 9. initialize printf header */
    for (uint32_t i = 0; i < m_printf.size(); i++)
    {
        uint32_t header_len = 8;
        m_mem->zeroize(m_printf[i].pa, header_len);
    }

    /* 10. others */
    m_spc = get_graph().m_text.pa + get_graph().m_entry;
    m_intr_pc = get_graph().m_text.pa + 0x10;

    /* success */
    m_status = AIPU_JOB_STATUS_INIT;

finish:
    if (ret)
        free_job_buffers();

    return ret;
}

aipu_status_t aipudrv::JobLegacy::free_job_buffers()
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
    if (m_stack.size != 0)
    {
        m_mem->free(&m_stack);
        m_stack.reset();
    }

    for (uint32_t i = 0; i < m_reuses.size(); i++)
        m_mem->free(&m_reuses[i]);

    m_reuses.clear();
    m_weights.clear();
    m_inputs.clear();
    m_outputs.clear();
    m_inter_dumps.clear();
    m_profiler.clear();
    m_printf.clear();
    m_layer_counter.clear();

    return ret;
}

aipu_status_t aipudrv::JobLegacy::schedule()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JobDesc desc;

    ret = validate_schedule_status();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    dump_job_shared_buffers();
    dump_job_private_buffers(m_rodata, m_descriptor);

    /* initialize error code buffer */
    for (uint32_t i = 0; i < m_err_code.size(); i++)
        m_mem->zeroize(m_err_code[i].pa, m_err_code[i].size);

    memset(&desc.kdesc, 0, sizeof(desc.kdesc));
    desc.kdesc.job_id = m_id;
    desc.kdesc.is_defer_run = m_is_defer_run;
    desc.kdesc.do_trigger = m_do_trigger;
    desc.kdesc.core_id = m_bind_core_id;
    desc.kdesc.version_compatible = !get_graph().m_do_vcheck;
    desc.kdesc.aipu_arch = get_graph().m_arch;
    desc.kdesc.aipu_version = get_graph().m_hw_version;
    desc.kdesc.aipu_config = get_graph().m_hw_config;
    desc.aipu_revision = get_graph().m_hw_revision;
    desc.instruction_base_pa = get_graph().m_text.pa;

    /**
     * note: on simulation, it's true align_asid_pa == pa.
     */
    if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V1)
    {
        desc.kdesc.start_pc_addr = m_spc;
        desc.kdesc.intr_handler_addr = m_intr_pc;
        desc.kdesc.data_0_addr = m_rodata.pa;
        desc.kdesc.data_1_addr = m_stack.pa;
    } else {
        desc.kdesc.start_pc_addr = m_spc - get_graph().m_text.asid_base;
        desc.kdesc.intr_handler_addr = m_intr_pc - get_graph().m_text.asid_base;
        desc.kdesc.data_0_addr = m_rodata.align_asid_pa;
        desc.kdesc.data_1_addr = m_stack.align_asid_pa;
    }

    desc.kdesc.enable_prof = 0;
    desc.kdesc.exec_flag = AIPU_JOB_EXEC_FLAG_NONE;
    desc.kdesc.dtcm_size_kb = get_graph().m_dtcm_size;
    desc.text_size = get_graph().m_text.req_size;
    desc.weight_pa = get_graph().m_weight.pa;
    desc.weight_size = get_graph().m_weight.req_size;
    desc.rodata_size = m_rodata.req_size;
    desc.dcr_pa = m_descriptor.pa;
    desc.dcr_size = m_descriptor.req_size;
    desc.stack_size = m_stack.req_size;
    desc.reuses = m_reuses;
    desc.weights = m_weights;
    desc.dump_reuse = !!m_dump_reuse;
    desc.output_dir = m_data_dir;

    for (uint32_t i = 0; i < m_outputs.size(); i++)
    {
        BufferDesc buf;
        buf.init(0, m_outputs[i].pa, m_outputs[i].size, m_outputs[i].size);
        desc.outputs.push_back(buf);
    }

    if (m_err_code.size() > 0)
    {
        BufferDesc buf;
        buf.init(0, m_err_code[0].pa, m_err_code[0].size, m_err_code[0].size);
        desc.misc_outputs[desc.output_dir + "/error_code.bin"] = buf;
    }

    if (m_profiler.size() > 0)
    {
        BufferDesc buf;
        buf.init(0, m_profiler[0].pa, m_profiler[0].size, m_profiler[0].size);
        desc.profile.push_back(buf);
    }

    if (m_printf.size() > 0)
    {
        BufferDesc buf;
        buf.init(0, m_printf[0].pa, m_printf[0].size, m_printf[0].size);
        desc.misc_outputs[desc.output_dir + "/printf_data.bin"] = buf;
    }

    if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V1)
        desc.simulator = m_z1_sim;
    else if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V2)
        desc.simulator = m_z2_sim;
    else if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V3)
        desc.simulator = m_z3_sim;
    else if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_X1)
        desc.simulator = m_x1_sim;

    if (m_log_path.length() != 0)
        desc.log_path = m_log_path;
    else
        desc.log_path = desc.output_dir;

    desc.log_level = m_log_level;
    desc.en_eval = m_en_eval;

    if (get_graph().m_sram_flag)
        desc.kdesc.exec_flag |= AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;

    ret = m_dev->schedule(desc);
    if (AIPU_STATUS_SUCCESS == ret)
    {
#if (defined SIMULATION)
        m_status = AIPU_JOB_STATUS_DONE;
#else
        m_status = AIPU_JOB_STATUS_SCHED;
#endif
    } else {
        m_status = AIPU_JOB_STATUS_EXCEPTION;
    }

    return ret;
}

aipu_status_t aipudrv::JobLegacy::destroy()
{
    return free_job_buffers();
}

aipu_status_t aipudrv::JobLegacy::config_simulation(uint64_t types, const aipu_job_config_simulation_t* config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (nullptr == config)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if(access(config->data_dir, F_OK) != 0)
    {
        LOG(LOG_ERR, "%s [non-exist]", config->data_dir);
        return AIPU_STATUS_ERROR_INVALID_CONFIG;
    }

    m_data_dir = config->data_dir;

    return ret;
}

aipu_status_t aipudrv::JobLegacy::bind_core(uint32_t core_id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    ret = validate_schedule_status();
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    m_is_defer_run = true;
    m_do_trigger = false;
    m_bind_core_id = core_id;
    ret = schedule();
    if (AIPU_STATUS_SUCCESS == ret)
        m_status = AIPU_JOB_STATUS_BIND;

    return ret;
}

aipu_status_t aipudrv::JobLegacy::debugger_run()
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
