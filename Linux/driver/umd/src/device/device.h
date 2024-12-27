// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  device.h
 * @brief AIPU User Mode Driver (UMD) device module header
 */

#include <mutex>
#include "standard_api.h"
#include "device_base.h"
#include "parser_base.h"
#ifdef SIMULATION

#ifdef ZHOUYI_V12
#include "simulator/simulator.h"
#endif

#ifdef ZHOUYI_V3
#include "simulator/simulator_v3.h"
#endif

#ifdef ZHOUYI_V3_1
#include "simulator/simulator_v3_1.h"
#endif

#else
#include "aipu/aipu.h"
#endif

#ifndef _DEVICE_H_
#define _DEVICE_H_

namespace aipudrv
{

#define X2_CMDPOOL_EXCEPTION (1 << 2)
#define X2_CMDPOOL_IDLE      (1 << 6)

#define X1_DEV_EXCEPTION     (1 << 2)
#define X1_DEV_IDLE          (1 << 17)

std::mutex m_tex;
inline aipu_status_t test_get_device(uint32_t graph_version, DeviceBase** dev,
    const aipu_global_config_simulation_t* cfg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (dev == nullptr)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if ((graph_version != AIPU_LOADABLE_GRAPH_V0005) &&
        (graph_version != AIPU_LOADABLE_GRAPH_ELF_V0))
        return AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;

#if (defined SIMULATION)
    {
        std::lock_guard<std::mutex> lock_(m_tex);
#if (defined ZHOUYI_V12)
        if (graph_version == AIPU_LOADABLE_GRAPH_V0005)
        {
            if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V1V2))
                return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
            else if (*dev == nullptr)
                *dev = Simulator::get_simulator();
        }
#endif
#if (defined ZHOUYI_V3)
        if (graph_version == AIPU_LOADABLE_GRAPH_ELF_V0)
        {
            if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3))
                return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
            else if (*dev == nullptr)
                *dev = SimulatorV3::get_v3_simulator(cfg);
        }
#endif
#if (defined ZHOUYI_V3_1)
        if (graph_version == AIPU_LOADABLE_GRAPH_ELF_V0)
        {
            if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3_1))
                return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
            else if (*dev == nullptr)
                *dev = SimulatorV3_1::get_v3_1_simulator(cfg);
        }
#endif
    }
#endif

    if (*dev == nullptr)
        return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

    return ret;
}

inline aipu_status_t set_target(uint32_t graph_version, DeviceBase** dev,
    const aipu_global_config_simulation_t* cfg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (dev == nullptr)
        return AIPU_STATUS_ERROR_NULL_PTR;

#if (defined SIMULATION)
    std::lock_guard<std::mutex> lock_(m_tex);

#if (defined ZHOUYI_V3)
    if (graph_version == AIPU_LOADABLE_GRAPH_ELF_V0)
    {
        if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3))
        {
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        } else if (*dev == nullptr) {
            SimulatorV3 *v3_sim = nullptr;
            *dev = v3_sim = SimulatorV3::get_v3_simulator(cfg);
            v3_sim->has_target(AIPU_ARCH_ZHOUYI, AIPU_ISA_VERSION_ZHOUYI_V3, 1204, 0);
        }
    }
#elif (defined ZHOUYI_V3_1)
    if (graph_version == AIPU_LOADABLE_GRAPH_ELF_V0)
    {
        if ((*dev != nullptr) && ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3_1))
        {
            return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
        } else if (*dev == nullptr) {
            SimulatorV3_1 *v3_1_sim = nullptr;
            *dev = v3_1_sim = SimulatorV3_1::get_v3_1_simulator(cfg);
            v3_1_sim->has_target(AIPU_ARCH_ZHOUYI, AIPU_ISA_VERSION_ZHOUYI_V3_1, 1304, 0);
        }
    }
#endif

    if (*dev == nullptr)
        return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

#endif
    return ret;
}

inline aipu_status_t get_device(DeviceBase** dev)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

#ifndef SIMULATION
    if (dev == nullptr)
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = Aipu::get_aipu(dev);
#endif

    return ret;
}

inline bool put_device(DeviceBase* dev)
{
    if (dev == nullptr)
        return false;

    if (dev->dec_ref_cnt() == 0)
    {
#ifdef SIMULATION
        dev = nullptr;
#else
        Aipu::put_aipu(dev);
#endif
        return true;
    } else {
        return false;
    }
}
}

#endif /* _DEVICE_H_ */
