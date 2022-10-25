// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

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
#include "parser_legacy.h"
#include "parser_elf.h"
#include "context.h"
#include "helper.h"
#ifdef SIMULATION
#include "simulator/simulator.h"
#include "simulator/simulator_v3.h"
#else
#include "aipu/aipu.h"
#endif

using namespace aipudrv;
using namespace std;

class ParserTest
{
public:
    /* data */
    ParserBase* m_parser = nullptr;
    GraphBase* p_gobj = nullptr;
    aipudrv::MainContext* p_ctx = nullptr;
    DeviceBase* m_dev = nullptr;
    aipu_global_config_simulation_t m_sim_cfg = {0};
    string graph_file = "./benchmark/aipu.bin";
    std::ifstream gbin;
    uint64_t _id = 0;
    uint32_t g_version = 0;
    uint32_t fsize = 0;
    bool m_do_vcheck = true;
    int failed_case = 0;
    bool res = false;

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

    ParserTest()
    {
        p_ctx = new aipudrv::MainContext();
        p_ctx->init();

        m_sim_cfg.z1_simulator = nullptr;
        m_sim_cfg.z2_simulator = nullptr;
        m_sim_cfg.z3_simulator = nullptr;
        m_sim_cfg.x1_simulator = nullptr;
        m_sim_cfg.x2_arch_desc = nullptr;
        m_sim_cfg.log_file_path = new char[1024];
        strcpy((char*)m_sim_cfg.log_file_path, "./");
        m_sim_cfg.log_level = 0;
        m_sim_cfg.verbose = false;
        m_sim_cfg.enable_avx = false;
        m_sim_cfg.enable_calloc = false;

        gbin.open(graph_file.c_str(), std::ifstream::in | std::ifstream::binary);
        if (!gbin.is_open()) {
            printf("file open failed");
        }
        gbin.seekg (0, gbin.end);
        fsize = gbin.tellg();
        gbin.seekg (0, gbin.beg);

        _id = 1;
        g_version = get_graph_bin_version(gbin);
        test_get_device(g_version, &m_dev, &m_sim_cfg);
#if (defined ZHOUYI_V12)
        if (AIPU_LOADABLE_GRAPH_V0005 == g_version) {
            p_gobj = new GraphLegacy(p_ctx, _id, m_dev);
            m_parser = new ParserLegacy();
        }
#endif
#if (defined ZHOUYI_V3)
        if (AIPU_LOADABLE_GRAPH_ELF_V0 == g_version) {
            p_gobj = new GraphV3(p_ctx, _id, m_dev);
            m_parser = new ParserELF();
        }
#endif
    }

    ~ParserTest()
    {
        if(m_parser != nullptr) {
            delete m_parser;
            m_parser = NULL;
        }
        if(p_gobj != nullptr) {
            delete p_gobj;
            p_gobj = NULL;
        }
        if(p_ctx != nullptr) {
            delete p_ctx;
            p_ctx = NULL;
        }
    }
};
