// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <unistd.h>
#include <cstring>
#include <iostream>
#include <sys/stat.h>
#include <tr1/tuple>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <string>
#include "doctest.h"
#include "graph_legacy.h"
#include "graph_v3.h"
#include "job_legacy.h"
#include "job_v3.h"
#include "standard_api.h"
#include "context.h"
#include "ctx_ref_map.h"
#ifdef SIMULATION
#include "simulator/simulator.h"
#include "simulator/simulator_v3.h"
#else
#include "aipu/aipu.h"
#endif

using namespace aipudrv;
using namespace std;

class JobTest
{
public:
    /* data */
    aipudrv::JobBase* p_job = nullptr;
    GraphBase* p_gobj = nullptr;
    aipudrv::MainContext* p_ctx = nullptr;
    DeviceBase* m_dev = nullptr;
    aipu_global_config_simulation_t m_sim_cfg = {0};
    aipu_job_config_simulation_t sim_job_config = {0};
    aipu_job_config_dump_t mem_dump_config;
    aipu_create_job_cfg create_job_cfg = {0};
    aipu_tensor_desc_t out_desc;
    char* input_dest = nullptr;
    uint32_t input_size = 0;
    string graph_file = "./benchmark/aipu.bin";
    string input_file = "./benchmark/input0.bin";
    std::ifstream gbin;
    uint64_t _id = 0;
    uint32_t g_version = 0;
    uint32_t fsize = 0;
    bool m_do_vcheck = true;
#if (defined ZHOUYI_V3)
    JobV3* job_v3 = nullptr;
#endif

    uint32_t get_graph_bin_version(std::istream& gbin)
    {
        #define EI_NIDENT 16
        uint32_t g_version = 0;
        const char e_ident[EI_NIDENT] = {0};

        gbin.read((char*)&e_ident, EI_NIDENT);
        gbin.seekg(0, gbin.beg);
        if (gbin.gcount() != EI_NIDENT)
        {
            goto finish;
        }

        if (!strncmp(e_ident, MAGIC, 8))
        {
            g_version = AIPU_LOADABLE_GRAPH_V0005;
        } else if (!strncmp(e_ident, "\177ELF", 4)) {
            g_version = AIPU_LOADABLE_GRAPH_ELF_V0;
        } else {
            goto finish;
        }

    finish:
        return g_version;
    }

    int load_file_helper(const char* fname, char** dest, uint32_t* size)
    {
        int ret = 0;
        int fd = 0;
        struct stat finfo;

        if ((nullptr == fname) || (nullptr == dest) || (nullptr == size))
        {
            return -1;
        }

        *dest = nullptr;

        if (stat(fname, &finfo) != 0)
        {
            fprintf(stderr, "open file failed: %s! (errno = %d)\n", fname, errno);
            return -1;
        }

        fd = open(fname, O_RDONLY);
        if (fd <= 0)
        {
            fprintf(stderr, "open file failed: %s! (errno = %d)\n", fname, errno);
            return -1;
        }

        *dest = new char[finfo.st_size];
        *size = finfo.st_size;

        if (read(fd, *dest, finfo.st_size) < 0)
        {
            fprintf(stderr, "load file failed: %s! (errno = %d)\n", fname, errno);
            ret = -1;
            goto finish;
        }

    finish:
        if (fd > 0)
        {
            close(fd);
        }
        if ((ret < 0) && (nullptr != dest) && (nullptr != *dest))
        {
            delete[] *dest;
            *dest = nullptr;
        }
        return ret;
    }

    aipu_status_t test_get_device(uint32_t graph_version, DeviceBase** dev,
    const aipu_global_config_simulation_t* cfg)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        assert(dev != nullptr);

        if ((AIPU_LOADABLE_GRAPH_V0005 != graph_version) &&
           (AIPU_LOADABLE_GRAPH_ELF_V0 != graph_version))
        {
           return AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;
        }

    #if (defined SIMULATION)
    #if (defined ZHOUYI_V12)
        if (AIPU_LOADABLE_GRAPH_V0005 == graph_version)
        {
            if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_LEGACY))
            {
                return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
            }
            else if (nullptr == *dev)
            {
                *dev = Simulator::get_simulator();
            }
        }
    #endif
    #if (defined ZHOUYI_V3)
        if (AIPU_LOADABLE_GRAPH_ELF_V0 == graph_version)
        {
            if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3))
            {
                return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
            }
            else if (nullptr == *dev)
            {
                *dev = SimulatorV3::get_v3_simulator(cfg);
            }
        }
    #endif
    #else /* !SIMULATION */
        ret = Aipu::get_aipu(dev);
    #endif

        if (nullptr == *dev)
        {
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        }
        return ret;
    }

    JobTest()
    {
        aipu_status_t ret;
        p_ctx = new aipudrv::MainContext();
        p_ctx->init();
        memset(&mem_dump_config, 0, sizeof(mem_dump_config));
        mem_dump_config.dump_dir = "./";
#if (defined SIMULATION)
        memset(&sim_job_config, 0, sizeof(sim_job_config));
        sim_job_config.data_dir = "./output";
#endif

#if (defined SIMULATION)
#if (defined ZHOUYI_V12)
        m_sim_cfg.x1_simulator = "./simulator/aipu_simulator_x1";
        m_sim_cfg.log_level = 3;
#endif
#if (defined ZHOUYI_V3)
        m_sim_cfg.log_level = 3;
        m_sim_cfg.x2_arch_desc = "X2_1204";
#endif
#endif

        ret = p_ctx->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &m_sim_cfg);
        gbin.open(graph_file.c_str(), std::ifstream::in | std::ifstream::binary);
        if (!gbin.is_open()) {
            printf("file open failed");
        }
        gbin.seekg (0, gbin.end);
        fsize = gbin.tellg();
        gbin.seekg (0, gbin.beg);

        _id = 1;
        g_version = ParserBase::get_graph_bin_version(gbin);
        ret = test_get_device(g_version, &m_dev, &m_sim_cfg);
#if (defined ZHOUYI_V12)
        if (AIPU_LOADABLE_GRAPH_V0005 == g_version) {
            p_gobj = new GraphLegacy(p_ctx, _id, m_dev);
            ret = p_gobj->load(gbin, fsize, m_do_vcheck);
            if (ret != AIPU_STATUS_SUCCESS) {
                printf("load graphj fail");
            }
            p_job = new JobLegacy(p_ctx, *p_gobj, m_dev);
        }
#endif
#if (defined ZHOUYI_V3)
        if (AIPU_LOADABLE_GRAPH_ELF_V0 == g_version) {
            p_gobj = new GraphV3(p_ctx, _id, m_dev);
            ret = p_gobj->load(gbin, fsize, m_do_vcheck);
            if (ret != AIPU_STATUS_SUCCESS) {
                printf("load graphj fail");
            }
            p_job = new JobV3(p_ctx, *p_gobj, m_dev, &create_job_cfg);
            job_v3 = static_cast<JobV3 *>(p_job);
        }
#endif
        load_file_helper(input_file.c_str(), &input_dest, &input_size);
        p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_OUTPUT, 0, &out_desc);
    }

    ~JobTest()
    {
        if(p_job != nullptr) {
            p_job = nullptr;
        }
        if(p_gobj != nullptr) {
            p_gobj = nullptr;
        }
        if(p_ctx != nullptr) {
            p_ctx = nullptr;
        }
    }
};
