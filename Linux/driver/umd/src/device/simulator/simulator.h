// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  simulator.h
 * @brief AIPU User Mode Driver (UMD) zhouyi z1/2/3 simulator module header
 */

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <map>
#include <vector>
#include <string>
#include "standard_api.h"
#include "device_base.h"
#include "umemory.h"
#include "type.h"
#include "utils/debug.h"

namespace aipudrv
{
#define FNAME_LEN 2048
#define OPT_LEN   2148
#define CMD_MEN   4096

struct SimulationData
{
    std::string fname;
    DEV_PA_64   pa;
    uint32_t    size;
};

struct SimulationJobCtx
{
    std::string text;
    std::string weight;
    std::string rodata;
    std::string dcr;
    std::string stack;
    std::vector<std::string> reuses;
    std::vector<std::string> weights;
    std::vector<std::string> outputs;
    char simulation_cmd[CMD_MEN];
};

class Simulator : public DeviceBase
{
private:
    aipu_status_t create_simulation_input_file(char* fname, const char* interfix,
        JOB_ID id, DEV_PA_64 pa, uint32_t size, const JobDesc& job);
    aipu_status_t update_simulation_rtcfg(const JobDesc& job, SimulationJobCtx& ctx);

public:
    bool has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev);
    aipu_status_t schedule(const JobDesc& job);
    aipu_status_t get_simulation_instance(void** simulator, void** memory)
    {
        return AIPU_STATUS_ERROR_INVALID_OP;
    }
    aipu_status_t set_sim_log_level(uint32_t level);

public:
    static Simulator* get_simulator()
    {
        static Simulator sim_instance;
        sim_instance.inc_ref_cnt();
        return &sim_instance;
    }
    virtual ~Simulator();
    Simulator(const Simulator& sim) = delete;
    Simulator& operator=(const Simulator& sim) = delete;

private:
    Simulator();
};
}

#endif /* _SIMULATOR_H_ */
