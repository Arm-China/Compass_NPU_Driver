// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  simulator.cpp
 * @brief AIPU User Mode Driver (UMD) zhouyi z1/2/3 simulator module implementation
 */

#include <cstring>
#include <iomanip>
#include <unistd.h>
#include "simulator.h"
#include "parser_base.h"
#include "utils/helper.h"
#include "kmd/armchina_aipu.h"

aipudrv::Simulator::Simulator()
{
    m_dev_type = DEV_TYPE_SIMULATOR_LEGACY;
    m_dram = UMemory::get_memory();
}

aipudrv::Simulator::~Simulator()
{
    m_dram = nullptr;
}

bool aipudrv::Simulator::has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev)
{
    aipu_partition_cap aipu_cap = {0};

    if ((arch != 0) || (version > AIPU_ISA_VERSION_ZHOUYI_X1))
        return false;

    aipu_cap.arch = arch;
    aipu_cap.version = version;
    aipu_cap.config = config;
    m_part_caps.push_back(aipu_cap);

    return true;
}

aipu_status_t aipudrv::Simulator::create_simulation_input_file(char* fname, const char* interfix,
    JOB_ID id, DEV_PA_64 pa, uint32_t size, const JobDesc& job)
{
    snprintf(fname, FNAME_LEN, "%s/Simulation_JOB0x%lx_%s_Base0x%lx_Size0x%x.bin",
        job.output_dir.c_str(), id, interfix, pa, size);
    return m_dram->dump_file(pa, fname, size);
}

aipu_status_t aipudrv::Simulator::update_simulation_rtcfg(const JobDesc& job, SimulationJobCtx& ctx)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    char fname[FNAME_LEN];
    std::string cfg_fname = job.output_dir + "/runtime.cfg";
    uint32_t input_data_cnt;
    uint32_t weight_cnt = 0, dcr_cnt = 0;
    uint32_t input_file_idx = 0;
    uint32_t output_idx = 0;
    std::vector<std::string> reuse_outputs;
    FileWrapper ofs(cfg_fname, std::ios::app);

    /* text */
    ret = create_simulation_input_file(fname, "Text", job.kdesc.job_id,
        job.instruction_base_pa, job.text_size, job);
    if (ret != AIPU_STATUS_SUCCESS)
        goto finish;

    ctx.text = fname;

    /* weight */
    if (job.weight_size != 0)
    {
        ret = create_simulation_input_file(fname, "Weight", job.kdesc.job_id,
            job.weight_pa, job.weight_size, job);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        ctx.weight = fname;
        weight_cnt = 1;
    } else {
        for (uint32_t i = 0; i < job.weights.size(); i++)
        {
            char inter_fix[32] = {0};
            snprintf(inter_fix, 32, "Weight%u", i);
            ret = create_simulation_input_file(fname, inter_fix, job.kdesc.job_id,
                job.weights[i].pa, job.weights[i].size, job);
            if (ret != AIPU_STATUS_SUCCESS)
                goto finish;

            ctx.weights.push_back(fname);
        }
        weight_cnt = job.weights.size();
    }

    /* rodata */
    ret = create_simulation_input_file(fname, "Rodata", job.kdesc.job_id,
        job.kdesc.data_0_addr, job.rodata_size, job);
    if (ret != AIPU_STATUS_SUCCESS)
        goto finish;

    ctx.rodata = fname;

    /* dcr */
    if (job.dcr_size != 0)
    {
        ret = create_simulation_input_file(fname, "Descriptor", job.kdesc.job_id,
            job.dcr_pa, job.dcr_size, job);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        ctx.dcr = fname;
        dcr_cnt = 1;
    }

    /* stack */
    ret = create_simulation_input_file(fname, "Stack", job.kdesc.job_id,
        job.kdesc.data_1_addr, job.stack_size, job);
    if (ret != AIPU_STATUS_SUCCESS)
        goto finish;

    ctx.stack = fname;

    /* reuse */
    for (uint32_t i = 0; i < job.reuses.size(); i++)
    {
        char inter_fix[32] = {0};
        snprintf(inter_fix, 32, "Reuse%u", i);
        ret = create_simulation_input_file(fname, inter_fix, job.kdesc.job_id,
            job.reuses[i].pa, job.reuses[i].size, job);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        ctx.reuses.push_back(fname);

        snprintf(inter_fix, 32, "AfRun_Reuse%u", i);
        ret = create_simulation_input_file(fname, inter_fix, job.kdesc.job_id,
            job.reuses[i].pa, job.reuses[i].size, job);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        reuse_outputs.push_back(fname);
    }

    /* output */
    for (uint32_t i = 0; i < job.outputs.size(); i++)
    {
        char inter_fix[32];
        snprintf(inter_fix, 32, "Output%u", i);
        ret = create_simulation_input_file(fname, inter_fix, job.kdesc.job_id,
            job.outputs[i].pa, job.outputs[i].size, job);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;

        ctx.outputs.push_back(fname);
    }

    /* misc output */
    for (auto item : job.misc_outputs)
    {
        auto &buf = item.second;
        ret = m_dram->dump_file(buf.pa, item.first.c_str(), buf.size);
        if (ret != AIPU_STATUS_SUCCESS)
            goto finish;
    }

    /* init config file */
    if (job.aipu_revision == 0)
    {
        bool is_x1 = (job.kdesc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_X1);
        if (is_x1)
            ofs << "CONFIG=X1-" << job.kdesc.aipu_config << "\n";
        else
            ofs << "CONFIG=Z" << job.kdesc.aipu_version
                << "-" << std::setw(4) << std::setfill('0') << job.kdesc.aipu_config << "\n";
    }
    else if (job.aipu_revision == AIPU_REVISION_P)
    {
        ofs << "CONFIG=Z" << job.kdesc.aipu_version
                << "-" << std::setw(4) << std::setfill('0') << job.kdesc.aipu_config << "p" << "\n";
    }

    ofs << "LOG_LEVEL=" << job.log_level << "\n";
    ofs << "LOG_FILE=" << job.log_path << "/log_default" << "\n";
    ofs << "FAST_FWD_INST=0" << "\n";

    if (job.en_eval)
        ofs << "EN_EVAL=1\n";
    else
        ofs << "EN_EVAL=0\n";
    ofs << "\n";

    /**
     * for aipu v2, set ASID 0/1;
     * if config UMD_ASID_ENABLE environment, set ASID 2/3 accordingly.
     */
    if (job.kdesc.aipu_version != AIPU_ISA_VERSION_ZHOUYI_Z1)
    {
        char *umd_asid_enable_env = getenv("UMD_ASID_ENABLE");
        std::map<std::string, bool> umd_asid_enable_map = {
            {"yes", true},
            {"y", true},
            {"YES", true},
            {"Y", true}
        };

        for (int i = 0; i < 2; i++)
        {
            ofs << "ASX_" << i <<  "_CTRL=0xC0000000" << "\n";
            ofs << "ASX_" << i <<  "_HIGH=0x0" << "\n";
            ofs << "ASX_" << i <<  "_LOW=0x0" << "\n";
        }

        if (umd_asid_enable_env != nullptr
            && umd_asid_enable_map.count(umd_asid_enable_env) > 0)
        {
            for (int i = 2; i < 4; i++)
            {
                ofs << "ASX_" << i <<  "_CTRL=0xC0000000" << "\n";
                ofs << "ASX_" << i <<  "_HIGH=0x0" << "\n";
                ofs << "ASX_" << i <<  "_LOW=0x0" << "\n";
            }
        }
    }

    ofs << "\n";
    ofs << "INPUT_INST_CNT=1\n";
    ofs << "INPUT_INST_FILE0=" << ctx.text << "\n";
    ofs << "INPUT_INST_BASE0=0x" << std::hex << job.instruction_base_pa << "\n";
    ofs << "INPUT_INST_STARTPC0=0x" << std::hex << job.kdesc.start_pc_addr << "\n";
    ofs << "INT_PC=0x" << std::hex << job.kdesc.intr_handler_addr << "\n";

    input_data_cnt = 2 + dcr_cnt + weight_cnt + job.reuses.size();

    ofs << "\n";
    ofs << "INPUT_DATA_CNT=" << std::dec <<  input_data_cnt << "\n";
    ofs << "INPUT_DATA_FILE0=" << ctx.rodata << "\n";
    ofs << "INPUT_DATA_BASE0=0x" << std::hex << job.kdesc.data_0_addr << "\n";

    ofs << "INPUT_DATA_FILE1=" << ctx.stack << "\n";
    ofs << "INPUT_DATA_BASE1=0x" << std::hex << job.kdesc.data_1_addr << "\n";

    input_file_idx = 2;
    if (dcr_cnt == 1)
    {
        ofs << "INPUT_DATA_FILE" << std::dec << input_file_idx <<  "=" << ctx.dcr << "\n";
        ofs << "INPUT_DATA_BASE" << std::dec << input_file_idx <<  "=0x" << std::hex <<job.dcr_pa << "\n";
        input_file_idx++;
    }

    if (weight_cnt == 1)
    {
        ofs << "INPUT_DATA_FILE" << std::dec << input_file_idx <<  "=" << ctx.weight << "\n";
        ofs << "INPUT_DATA_BASE" << std::dec << input_file_idx <<  "=0x" << std::hex << job.weight_pa << "\n";
        input_file_idx++;
    } else {
        for(uint32_t i = 0; i < job.weights.size(); i++)
        {
            ofs << "INPUT_DATA_FILE" << std::dec << input_file_idx + i <<  "=" << ctx.weights[i] << "\n";
            ofs << "INPUT_DATA_BASE" << std::dec << input_file_idx + i <<  "=0x" << std::hex << job.weights[i].pa << "\n";
        }
        input_file_idx += job.weights.size();
    }

    for(uint32_t i = 0; i < job.reuses.size(); i++)
    {
        ofs << "INPUT_DATA_FILE" << std::dec << input_file_idx + i <<  "=" << ctx.reuses[i] << "\n";
        ofs << "INPUT_DATA_BASE" << std::dec << input_file_idx + i <<  "=0x" << std::hex << job.reuses[i].pa << "\n";
    }

    ofs << "\n";
    if (job.dump_reuse)
    {
        ofs << "OUTPUT_DATA_CNT=" << std::dec << (job.outputs.size()
            + job.misc_outputs.size() + reuse_outputs.size()) << "\n";
    } else {
        ofs << "OUTPUT_DATA_CNT=" << std::dec << job.outputs.size()
            + job.misc_outputs.size() << "\n";
    }

    for (auto &buf : job.outputs)
    {
        ofs << "OUTPUT_DATA_FILE" << std::dec << output_idx << "=" << ctx.outputs[output_idx] << "\n";
        ofs << "OUTPUT_DATA_BASE" << std::dec << output_idx << "=0x" << std::hex << buf.pa  << "\n";
        ofs << "OUTPUT_DATA_SIZE" << std::dec << output_idx << "=0x" << std::hex << buf.size << "\n";
        output_idx++;
    }

    for (auto &item : job.misc_outputs)
    {
        auto &buf = item.second;
        ofs << "OUTPUT_DATA_FILE" << std::dec << output_idx << "=" << item.first << "\n";
        ofs << "OUTPUT_DATA_BASE" << std::dec << output_idx << "=0x" << std::hex << buf.pa << "\n";
        ofs << "OUTPUT_DATA_SIZE" << std::dec << output_idx << "=0x" << std::hex << buf.size << "\n";
        output_idx++;
    }

    if(job.profile.size() == 1)
    {
        ofs << "\nPROFILE_BUF_ADDR=0x" << std::hex << job.profile[0].pa << "\n";
        ofs << "PROFILE_BUF_SIZE=0x" << std::hex << job.profile[0].req_size << "\n";
    }

    if (job.dump_reuse)
    {
        for (uint32_t i = 0; i < reuse_outputs.size(); i++)
        {
            ofs << "OUTPUT_DATA_FILE" << std::dec << output_idx << "=" << reuse_outputs[i] << "\n";
            ofs << "OUTPUT_DATA_BASE" << std::dec << output_idx << "=0x" << std::hex << job.reuses[i].pa << "\n";
            ofs << "OUTPUT_DATA_SIZE" << std::dec << output_idx << "=0x" << std::hex << job.reuses[i].size << "\n";
            output_idx++;
        }
    }

    ofs << "RUN_DESCRIPTOR=BIN[0]" << "\n";

    snprintf(ctx.simulation_cmd, sizeof(ctx.simulation_cmd), "%s %s", job.simulator.c_str(), cfg_fname.c_str());
finish:
    return ret;
}

aipu_status_t aipudrv::Simulator::schedule(const JobDesc& job)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    int sys_ret = 0;
    SimulationJobCtx ctx;

    ret = update_simulation_rtcfg(job, ctx);
    if (ret != AIPU_STATUS_SUCCESS)
        goto error;

    LOG(LOG_DEFAULT, "[UMD SIMULATION] %s", ctx.simulation_cmd);
    sys_ret = system(ctx.simulation_cmd);
    if (sys_ret == -1)
    {
        LOG(LOG_ERR, "Simulation execution failed!");
        ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;
        goto error;
    }
    else if (WIFEXITED(sys_ret) && (WEXITSTATUS(sys_ret) != 0))
    {
        LOG(LOG_ERR, "Simulation execution failed! (simulator ret = %d)", WEXITSTATUS(sys_ret));
        ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;
        goto error;
    }
    else if (WIFSIGNALED(sys_ret))
    {
        LOG(LOG_ERR, "Simulation terminated by signal %d!", WTERMSIG(sys_ret));
        ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;
        goto error;
    }

    for (uint32_t i = 0; i < ctx.outputs.size(); i++)
    {
        ret = m_dram->load_file(job.outputs[i].pa, ctx.outputs[i].c_str(), job.outputs[i].size);
        if (ret != AIPU_STATUS_SUCCESS)
            goto error;
    }

    for (auto &item : job.misc_outputs)
    {
        const BufferDesc &buf = item.second;
        ret = m_dram->load_file(buf.pa, item.first.c_str(), buf.size);
        if (ret != AIPU_STATUS_SUCCESS)
            goto error;
    }

error:
    return ret;
}

aipu_status_t aipudrv::Simulator::set_sim_log_level(uint32_t level)
{
    return AIPU_STATUS_SUCCESS;
}
