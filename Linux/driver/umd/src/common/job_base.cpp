// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  job_base.cpp
 * @brief AIPU User Mode Driver (UMD) job base module implementation
 */

#include <cstring>
#include <unistd.h>
#include "job_base.h"
#include "utils/helper.h"

aipudrv::JobBase::JobBase(MainContext* ctx, GraphBase& graph, DeviceBase* dev):
    m_ctx(ctx), m_graph(graph), m_dev(dev)
{
    m_mem = m_dev->get_mem();
    m_rodata.reset();
    m_descriptor.reset();
}

aipudrv::JobBase::~JobBase()
{
}

aipu_status_t aipudrv::JobBase::get_status(aipu_job_status_t* status)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    std::vector<aipu_job_status_desc> jobs_status;
    uint32_t err = 0;

    if (get_subgraph_cnt() == 0)
    {
        aipu_job_status_desc desc = {0};
        desc.state = AIPU_JOB_STATE_DONE;
        jobs_status.push_back(desc);
    } else {
        ret = convert_ll_status(m_dev->get_status(jobs_status, 1, this));
        if (ret != AIPU_STATUS_SUCCESS)
            return ret;
    }

    if (jobs_status.size() != 0)
        m_status = jobs_status[0].state;

    if ((m_status == AIPU_JOB_STATUS_DONE) || (m_status == AIPU_JOB_STATUS_EXCEPTION))
    {
        *status = (aipu_job_status_t)m_status;
        dump_job_private_buffers_after_run(m_rodata, m_descriptor);
        dump_job_shared_buffers_after_run();
    } else {
        *status = AIPU_JOB_STATUS_NO_STATUS;
    }

    if (m_err_code.size() > 0)
        m_mem->read32(&err, m_err_code[0].pa);

    return m_ctx->log_rt_err_msg(err);
}

aipu_status_t aipudrv::JobBase::get_status_blocking(aipu_job_status_t* status, int32_t time_out)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    std::vector<aipu_job_status_desc> jobs_status;
    uint32_t err = 0;

    if (get_subgraph_cnt() == 0)
    {
        aipu_job_status_desc desc = {0};
        desc.state = AIPU_JOB_STATE_DONE;
        jobs_status.push_back(desc);
    } else {
        ret = convert_ll_status(m_dev->poll_status(jobs_status, 1, time_out,
            m_hw_cfg->poll_in_commit_thread, this));
        if (ret != AIPU_STATUS_SUCCESS)
            return ret;
    }

    if (jobs_status.size() != 0)
        m_status = jobs_status[0].state;

    if ((m_status == AIPU_JOB_STATUS_DONE) || (m_status == AIPU_JOB_STATUS_EXCEPTION))
    {
        *status = (aipu_job_status_t)m_status;
        dump_job_private_buffers_after_run(m_rodata, m_descriptor);
        dump_job_shared_buffers_after_run();
    } else {
        *status = AIPU_JOB_STATUS_NO_STATUS;
    }

    if (m_err_code.size() > 0)
        m_mem->read32(&err, m_err_code[0].pa);

    return m_ctx->log_rt_err_msg(err);
}

aipu_status_t aipudrv::JobBase::load_tensor(uint32_t tensor, const void* data)
{
    if (nullptr == data)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (tensor >= m_inputs.size())
        return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

    /* Applications cannot load tensors if a job is not in the to-be-scheduled status */
    if ((m_status != AIPU_JOB_STATUS_INIT) &&
        (m_status != AIPU_JOB_STATUS_DONE) &&
        (m_status != AIPU_JOB_STATUS_BIND))
        return AIPU_STATUS_ERROR_INVALID_OP;

    m_mem->write(m_inputs[tensor].pa, (const char*)data, m_inputs[tensor].size);
    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::JobBase::get_tensor(aipu_tensor_type_t type, uint32_t tensor, void* data)
{
    DEV_PA_64 pa = 0;
    uint64_t size = 0;
    uint32_t isa = AIPU_ISA_VERSION_ZHOUYI_Z1;
    std::vector<struct JobIOBuffer> *iobuffer_vec = nullptr;

    if (nullptr == data)
        return AIPU_STATUS_ERROR_NULL_PTR;

    /* Applications cannot get tensors if a job is not done status */
    if (m_status != AIPU_JOB_STATUS_DONE)
        return AIPU_STATUS_ERROR_INVALID_OP;

    isa = m_dev->get_npu_version();

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            iobuffer_vec = &m_inputs;
            break;

        case AIPU_TENSOR_TYPE_OUTPUT:
            iobuffer_vec = &m_outputs;
            break;

        case AIPU_TENSOR_TYPE_INTER_DUMP:
            iobuffer_vec = &m_inter_dumps;
            break;

        case AIPU_TENSOR_TYPE_PRINTF:
            if (isa == AIPU_ISA_VERSION_ZHOUYI_X2)
            {
                // Todo
            } else {
                iobuffer_vec = &m_printf;
            }
            break;

        case AIPU_TENSOR_TYPE_PROFILER:
            // if (isa == AIPU_ISA_VERSION_ZHOUYI_X2)
            // {
            //     std::string profile_file_name = m_dump_dir + "/" + m_dump_misc_prefix + "_PerfData.bin";
            //     LOG(LOG_ALERT, "check dump file: %s\n", profile_file_name.c_str());
            //     return AIPU_STATUS_SUCCESS;
            // } else {
                iobuffer_vec = &m_profiler;
            // }
            break;

        case AIPU_TENSOR_TYPE_LAYER_COUNTER:
            iobuffer_vec = &m_layer_counter;
            break;

        case AIPU_TENSOR_TYPE_ERROR_CODE:
            iobuffer_vec = &m_err_code;
            break;

        default:
            return AIPU_STATUS_ERROR_INVALID_OP;
    }

    if (tensor >= iobuffer_vec->size())
        return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

    pa = iobuffer_vec->at(tensor).pa;
    size = iobuffer_vec->at(tensor).size;
    m_mem->read(pa, (char*)data, size);

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::JobBase::mark_shared_tensor(aipu_tensor_type_t type, uint32_t tensor, uint64_t &pa_addr)
{
    DEV_PA_64 addr;
    uint32_t size;
    std::vector<struct JobIOBuffer> *iobuffer_vec = nullptr;

    switch (type)
    {
        case AIPU_TENSOR_TYPE_INPUT:
            iobuffer_vec = &m_inputs;
            break;

        case AIPU_TENSOR_TYPE_OUTPUT:
            iobuffer_vec = &m_outputs;
            break;

        default:
            return AIPU_STATUS_ERROR_INVALID_OP;
    }

    if (tensor >= iobuffer_vec->size())
        return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;

    addr = iobuffer_vec->at(tensor).pa;
    size = iobuffer_vec->at(tensor).size;

    if(m_mem->mark_shared_buffer(addr, size) != 0)
        return AIPU_STATUS_ERROR_MARK_SHARED_TENSOR;

    pa_addr = addr;

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::JobBase::setup_rodata(
    const std::vector<struct GraphParamMapLoadDesc>& param_map,
    const std::vector<BufferDesc>& reuse_buf,
    const std::vector<BufferDesc>& static_buf,
    BufferDesc rodata,
    BufferDesc dcr
)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    char* ro_va = nullptr;
    char* dcr_va = nullptr;

    m_mem->pa_to_va(rodata.pa, rodata.size, &ro_va);
    if (dcr.size != 0)
        m_mem->pa_to_va(dcr.pa, dcr.size, &dcr_va);

    for (uint32_t i = 0; i < param_map.size(); i++)
    {
        char* entry = nullptr;
        uint32_t init_val = 0;
        uint32_t finl_val = 0;
        uint32_t ref_iter = param_map[i].ref_section_iter;
        uint32_t sec_offset = param_map[i].sub_section_offset;
        uint32_t sub_sec_pa_32 = 0;
        uint32_t offset_in_map = param_map[i].offset_in_map;

        LOG(LOG_INFO, "%3u: type=%d: <%lx, %lx>, < %x, 0x%x>", i, param_map[i].load_type,
            rodata.req_size, dcr.req_size, offset_in_map,
            get_low_32(reuse_buf[ref_iter].align_asid_pa) + sec_offset);

        if (offset_in_map < rodata.req_size)
            entry = ro_va + offset_in_map;
        else
            entry = dcr_va + offset_in_map - rodata.req_size;

        if (param_map[i].load_type == PARAM_MAP_LOAD_TYPE_REUSE)
        {
            if (ref_iter >= reuse_buf.size())
            {
                ret = AIPU_STATUS_ERROR_INVALID_SIZE;
                goto finish;
            }
            sub_sec_pa_32 = get_low_32(reuse_buf[ref_iter].align_asid_pa) + sec_offset;
        }
        else if (param_map[i].load_type == PARAM_MAP_LOAD_TYPE_STATIC)
        {
            if (ref_iter >= static_buf.size())
            {
                ret = AIPU_STATUS_ERROR_INVALID_SIZE;
                goto finish;
            }
            sub_sec_pa_32 = get_low_32(static_buf[ref_iter].align_asid_pa + sec_offset);
        }

        memcpy(&init_val, entry, 4);
        finl_val = ((sub_sec_pa_32 & param_map[i].addr_mask) | (init_val & (~param_map[i].addr_mask)));
        memcpy(entry, &finl_val, 4);

        LOG(LOG_CLOSE, "param %u: write addr/final_val 0x%x/0x%x (%s section %u offset 0x%x) into %s",
            i,
            sub_sec_pa_32,
            finl_val,
            (param_map[i].load_type == PARAM_MAP_LOAD_TYPE_REUSE) ? "reuse" : "weight",
            ref_iter,
            sec_offset,
            (offset_in_map < rodata.req_size) ? "rodata" : "descriptor");
    }

finish:
    return ret;
}

aipudrv::DEV_PA_64 aipudrv::JobBase::get_base_pa(int sec_type, BufferDesc& rodata,
        BufferDesc& descriptor, bool align_asid)
{
    DEV_PA_64 pa = 0;
    DEV_PA_64 align_asid_pa = 0;

    if (sec_type == SECTION_TYPE_RODATA)
    {
        pa = rodata.pa;
        align_asid_pa = pa - rodata.asid_base;
    } else if (sec_type == SECTION_TYPE_DESCRIPTOR) {
        pa = descriptor.pa;
        align_asid_pa = pa - descriptor.asid_base;
    } else if (sec_type == SECTION_TYPE_TEXT) {
        pa = get_graph().m_text.pa;
        align_asid_pa = pa - get_graph().m_text.asid_base;
    }

    return (align_asid == false) ? pa : align_asid_pa;
}

void aipudrv::JobBase::setup_remap(BufferDesc& rodata, BufferDesc& descriptor)
{
    for (uint32_t i = 0; i < get_graph().m_remap.size(); i++)
    {
        DEV_PA_64 dest = get_base_pa(get_graph().m_remap[i].type, rodata, descriptor, false) +
            get_graph().m_remap[i].next_addr_entry_offset;
        DEV_PA_32 pa = get_base_pa(get_graph().m_remap[i].next_type, rodata, descriptor, true) +
            get_graph().m_remap[i].next_offset;
        m_mem->write32(dest, pa);
    }
}

void aipudrv::JobBase::create_io_buffers(std::vector<struct JobIOBuffer>& bufs,
        const std::vector<GraphIOTensorDesc>& desc,
        const std::vector<BufferDesc>& reuses)
{
    uint32_t cnt = desc.size();

    for (uint32_t i = 0; i < cnt; i++)
    {
        uint32_t sec_iter = desc[i].ref_section_iter;
        DEV_PA_64 pa = reuses[sec_iter].pa + desc[i].offset_in_section;
        JobIOBuffer iobuf;

        if (desc[i].support_dma_buf)
        {
            /* I/O buffer will be allocated later */
            iobuf.init(0, 0, AIPU_JOB_BUFFER_VOID, 0);
        } else {
            iobuf.init(desc[i].id, desc[i].size, AIPU_JOB_BUFFER_INTERNAL, pa);
        }
        bufs.push_back(iobuf);
    }
}

void aipudrv::JobBase::create_io_buffers(const struct GraphIOTensors& io,
    const std::vector<BufferDesc>& reuses)
{
    create_io_buffers(m_inputs, io.inputs, reuses);
    create_io_buffers(m_outputs, io.outputs, reuses);
    create_io_buffers(m_inter_dumps, io.inter_dumps, reuses);
    create_io_buffers(m_profiler, io.profiler, reuses);
    create_io_buffers(m_printf, io.printf, reuses);
    create_io_buffers(m_layer_counter, io.layer_counter, reuses);
    create_io_buffers(m_err_code, io.err_code, reuses);
    create_io_buffers(m_segmmus, io.segmmus, reuses);
}

void aipudrv::JobBase::dump_buffer(DEV_PA_64 pa, const char* bin_va, uint32_t size, const char* name)
{
    char file_name[4096];

    if (bin_va != nullptr)
    {
        snprintf(file_name, 4096, "%s/Graph_0x%lx_Job_0x%lx_%s_Dump_in_Binary_Size_0x%x.bin",
        m_dump_dir.c_str(), get_graph().m_id, m_id, name, size);
        umd_dump_file_helper(file_name, bin_va, size);
    }

    snprintf(file_name, 4096, "%s/Graph_0x%lx_Job_0x%lx_%s_Dump_in_DRAM_PA_0x%lx_Size_0x%x.bin",
        m_dump_dir.c_str(), get_graph().m_id, m_id, name, pa, size);
    m_mem->dump_file(pa, file_name, size);
}

void aipudrv::JobBase::dump_single_buffer(DEV_PA_64 pa, uint32_t size, const char* name)
{
    char file_name[2048] = {0};

    snprintf(file_name, 2048, "%s/Graph_0x%lx_Job_0x%lx_%s_Dump_in_DRAM_PA_0x%lx_Size_0x%x.bin",
        m_dump_dir.c_str(), get_graph().m_id, m_id, name, pa, size);
    m_mem->dump_file(pa, file_name, size);
}

aipu_status_t aipudrv::JobBase::config_mem_dump(uint64_t types, const aipu_job_config_dump_t* config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if ((nullptr != config) && (nullptr != config->dump_dir))
    {
        if(access(config->dump_dir, F_OK) != 0)
        {
            LOG(LOG_ERR, "%s [non-exist]", config->dump_dir);
            ret = AIPU_STATUS_ERROR_INVALID_CONFIG;
            goto finish;
        }
        m_dump_dir = config->dump_dir;
    }

    if ((nullptr != config) && (nullptr != config->prefix))
        m_dump_prefix = config->prefix;

    if ((nullptr != config) && (nullptr != config->output_prefix))
        m_dump_output_prefix = config->output_prefix;

     if ((nullptr != config) && (nullptr != config->misc_prefix))
        m_dump_misc_prefix = config->misc_prefix;

    m_dump_text = types & AIPU_JOB_CONFIG_TYPE_DUMP_TEXT;
    m_dump_weight = types & AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT;
    m_dump_rodata = types & AIPU_JOB_CONFIG_TYPE_DUMP_RODATA;
    m_dump_dcr = types & AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR;
    m_dump_input = types & AIPU_JOB_CONFIG_TYPE_DUMP_INPUT;
    m_dump_output = types & AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT;
    m_dump_reuse = types & AIPU_JOB_CONFIG_TYPE_DUMP_REUSE;
    m_dump_tcb = types & AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN;
    m_dump_emu = types & AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;
    m_dump_profile = types & AIPU_JOB_CONFIG_TYPE_DUMP_PROFILE;

finish:
    return ret;
}

void aipudrv::JobBase::dump_job_shared_buffers()
{
    DEV_PA_64 dump_pa;
    uint32_t dump_size;
    const char* bin_va = nullptr;

    if (m_dump_text)
    {
        dump_pa = get_graph().m_text.pa;
        bin_va = get_graph().m_btext.va;
        dump_size = get_graph().m_btext.size;
        dump_buffer(dump_pa, bin_va, dump_size, "Text_BeforeRun");
    }

    if (m_dump_weight && get_graph().m_weight.size > 0)
    {
        dump_pa = get_graph().m_weight.pa;
        bin_va = get_graph().m_bweight.va;
        dump_size = get_graph().m_bweight.size;
        if (dump_size != 0)
            dump_buffer(dump_pa, bin_va, dump_size, "Weight_BeforeRun");
    }
}

void aipudrv::JobBase::dump_job_private_buffers(BufferDesc& rodata, BufferDesc& descriptor)
{
    DEV_PA_64 dump_pa;
    uint32_t dump_size;
    const char* bin_va = nullptr;

    if (m_dump_rodata)
    {
        dump_pa = rodata.pa;
        bin_va = get_graph().m_brodata.va;
        dump_size = get_graph().m_brodata.size;
        if (dump_size != 0)
            dump_buffer(dump_pa, bin_va, dump_size, "Rodata_BeforeRun");
    }

    if (m_dump_dcr)
    {
        dump_pa = descriptor.pa;
        bin_va = get_graph().m_bdesc.va;
        dump_size = get_graph().m_bdesc.size;
        if (dump_size != 0)
            dump_buffer(dump_pa, bin_va, dump_size, "Descriptor_BeforeRun");
    }

    if (m_dump_input)
    {
        for (uint32_t i = 0; i < m_inputs.size(); i++)
        {
            char name[32];
            dump_pa   = m_inputs[i].pa;
            dump_size = m_inputs[i].size;
            snprintf(name, 32, "Input%u", m_inputs[i].id);
            if (dump_size != 0)
                dump_buffer(dump_pa, nullptr, dump_size, name);
        }
    }
}

void aipudrv::JobBase::dump_job_shared_buffers_after_run()
{
    DEV_PA_64 dump_pa = 0;
    uint32_t dump_size = 0;

    if (m_dump_text)
    {
        dump_pa = get_graph().m_text.pa;
        dump_size = get_graph().m_btext.size;
        dump_single_buffer(dump_pa, dump_size, "Text_AfterRun");
    }

    if (m_dump_weight && get_graph().m_weight.size > 0)
    {
        dump_pa = get_graph().m_weight.pa;
        dump_size = get_graph().m_bweight.size;
        if (dump_size != 0)
            dump_single_buffer(dump_pa, dump_size, "Weight_AfterRun");
    }
}

void aipudrv::JobBase::dump_job_private_buffers_after_run(BufferDesc& rodata, BufferDesc& descriptor)
{
    DEV_PA_64 dump_pa;
    uint32_t dump_size;

    if (m_dump_emu)
    {
        for (uint32_t i = 0; i < m_outputs.size(); i++)
        {
            std::string dump_name = m_dump_dir + "/" + m_dump_prefix + ".output" + std::to_string(i);

            dump_pa   = m_outputs[i].pa;
            dump_size = m_outputs[i].size;
            m_mem->dump_file(dump_pa, dump_name.c_str(), dump_size);
        }
    }

    if (m_dump_output)
    {
        for (uint32_t i = 0; i < m_outputs.size(); i++)
        {
            char name[32];
            dump_pa   = m_outputs[i].pa;
            dump_size = m_outputs[i].size;
            snprintf(name, 32, "Output%u", m_outputs[i].id);
            if (dump_size != 0)
                dump_buffer(dump_pa, nullptr, dump_size, name);
        }
    }

    if (m_dump_reuse)
    {
        if ((m_dev->get_dev_type() == DEV_TYPE_AIPU) || (m_dev->get_dev_type() == DEV_TYPE_SIMULATOR_V3))
        {
            std::vector<BufferDesc> m_job_reuses = get_reuse();
            for (uint32_t i = 0; i < m_job_reuses.size(); i++)
            {
                char name[32];
                dump_pa   = m_job_reuses[i].pa;
                dump_size = m_job_reuses[i].size;
                snprintf(name, 32, "AfRun_Reuse%u", i);
                if (dump_size != 0)
                    dump_buffer(dump_pa, nullptr, dump_size, name);
            }
        }
    }

    if (m_dump_rodata)
    {
        dump_pa = rodata.pa;
        dump_size = get_graph().m_brodata.size;
        if (dump_size != 0)
            dump_buffer(dump_pa, nullptr, dump_size, "Rodata_AfterRun");
    }

    if (m_dump_dcr)
    {
        dump_pa = descriptor.pa;
        dump_size = get_graph().m_bdesc.size;
        if (dump_size != 0)
            dump_buffer(dump_pa, nullptr, dump_size, "Descriptor_AfterRun");
    }

    if (m_dump_profile)
    {
        if (m_profile_fd > 0)
            close(m_profile_fd);
    }
}

aipu_status_t aipudrv::JobBase::validate_schedule_status()
{
    if ((m_status == AIPU_JOB_STATUS_INIT) ||
        (m_status == AIPU_JOB_STATUS_DONE) ||
        (m_status == AIPU_JOB_STATUS_BIND))
        return AIPU_STATUS_SUCCESS;

    return AIPU_STATUS_ERROR_INVALID_OP;
}