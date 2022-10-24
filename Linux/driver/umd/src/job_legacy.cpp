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

aipudrv::JobLegacy::JobLegacy(MainContext* ctx, const GraphBase& graph, DeviceBase* dev):
    JobBase(ctx, graph, dev)
{
    m_stack.reset();
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

aipu_status_t aipudrv::JobLegacy::init(const aipu_global_config_simulation_t* cfg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

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
        BufferDesc buf;

        buf.reset();
        if (size != 0)
        {
            std::string str = "reuse_" + std::to_string(i);
            ret = m_mem->malloc(size, align_in_page, &buf, str.c_str());
            if (AIPU_STATUS_SUCCESS != ret)
                goto finish;
        }

        if (m_dump_reuse)
            m_mem->mem_bzero(buf.pa, buf.size);

        m_reuses.push_back(buf);
    }

    /* 5. init weights address */
    for (uint32_t i = 0; i < get_graph().m_static_sections.size(); i++)
    {
        BufferDesc buf;

        #if 0
        /**
         * if enable this logic, please also enable the corresponding free logic.
         */
        buf.reset();
        std::string str = "static_" + std::to_string(i);
        const GraphSectionDesc &section_desc = get_graph().m_static_sections[i];
        ret = m_mem->malloc(section_desc.size, section_desc.align_in_page, &buf, str.c_str(), AIPU_BUF_REGION_DTCM);
        if (AIPU_STATUS_SUCCESS != ret)
            goto finish;

        m_mem->write(buf.pa, (char *)section_desc.load_src, section_desc.size);
        #else
        buf.init(get_graph().m_weight.asid_base, get_graph().m_weight.pa + get_graph().m_static_sections[i].offset,
            get_graph().m_static_sections[i].size,
            get_graph().m_static_sections[i].size);
        #endif
        m_weights.push_back(buf);
    }

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

    #if 0
    for (uint32_t i = 0; i < m_weights.size(); i++)
    {
        m_mem->free(&m_weights[i]);
    }
    m_weights.clear();
    #endif

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
    desc.dump_reuse = !!m_dump_reuse;
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
        desc.outputs.push_back(buf);
    }

    if (m_profiler.size() > 0)
    {
        BufferDesc buf;
        buf.init(0, m_profiler[0].pa, m_profiler[0].size, m_profiler[0].size);
        desc.outputs.push_back(buf);
    }

    if (m_printf.size() > 0)
    {
        BufferDesc buf;
        buf.init(0, m_printf[0].pa, m_printf[0].size, m_printf[0].size);
        desc.outputs.push_back(buf);
    }

    desc.output_dir = m_data_dir;
    if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V1)
        desc.simulator = m_z1_sim;
    else if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V2)
        desc.simulator = m_z2_sim;
    else if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_V3)
        desc.simulator = m_z3_sim;
    else if (get_graph().m_hw_version == AIPU_VERSION_ZHOUYI_X1)
        desc.simulator = m_x1_sim;

    desc.log_level = m_log_level;

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
        LOG(LOG_ERR, "%s [non-exit]", config->data_dir);
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
