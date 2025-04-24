// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  device.h
 * @brief AIPU User Mode Driver (UMD) device module header
 */

#include <mutex>

#include "device_base.h"
#include "parser_base.h"

#ifdef SIMULATION

#ifdef ZHOUYI_V12
#include "device/simulator/simulator.h"
#endif

#ifdef ZHOUYI_V3
#include "device/simulator/simulator_v3.h"
#endif

#ifdef ZHOUYI_V3_1
#include "device/simulator/simulator_v3_1.h"
#endif

#else
#include "device/aipu/aipu.h"
#endif

#ifndef _DEVICE_H_
#define _DEVICE_H_

namespace aipudrv {
std::mutex m_tex;
inline aipu_status_t
set_device_cfg(uint32_t graph_version, DeviceBase **dev,
               const aipu_global_config_simulation_t *cfg) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (dev == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  if ((graph_version != AIPU_LOADABLE_GRAPH_V0005) &&
      (graph_version != AIPU_LOADABLE_GRAPH_ELF_V0))
    return AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;

#if (defined SIMULATION)
  std::lock_guard<std::mutex> lock_(m_tex);
#if (defined ZHOUYI_V12)
  if (graph_version == AIPU_LOADABLE_GRAPH_V0005) {
    /**
     * The current device maybe V3 because of the new APIs changes in
     * get_device, so here just assign the needed simulator instance pointer
     * without target check.
     * */
    *dev = Simulator::get_simulator();
  }
#endif
#if (defined ZHOUYI_V3)
  if (graph_version == AIPU_LOADABLE_GRAPH_ELF_V0) {
    if ((*dev != nullptr) &&
        ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3)) {
      return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    } else {
      *dev = SimulatorV3::get_v3_simulator(cfg);
      ret = reinterpret_cast<SimulatorV3 *>(*dev)->init();
    }
  }
#elif (defined ZHOUYI_V3_1)
  if (graph_version == AIPU_LOADABLE_GRAPH_ELF_V0) {
    if ((*dev != nullptr) &&
        ((*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3_1)) {
      return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    } else {
      *dev = SimulatorV3_1::get_v3_1_simulator(cfg);
      ret = reinterpret_cast<SimulatorV3_1 *>(*dev)->init();
    }
  }
#endif

  if (*dev == nullptr)
    return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

#endif
  return ret;
}

inline aipu_status_t get_device(DeviceBase **dev) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (dev == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

#if defined(SIMULATION)
    /**
     * The new APIs only support the device since V3, so here only initialize
     * these devices, for the devices before V3, just leave them empty pointer.
     */
#if defined(ZHOUYI_V3)
  /* only get UMD simulator object, doesn't initialize Simulator object */
  *dev = SimulatorV3::get_v3_simulator(nullptr);
#elif defined(ZHOUYI_V3_1)
  /* only get UMD simulator object, doesn't initialize Simulator object */
  *dev = SimulatorV3_1::get_v3_1_simulator(nullptr);
#endif
#else
  ret = Aipu::get_aipu(dev);
#endif

  return ret;
}

inline bool put_device(DeviceBase **dev) {
  if (dev == nullptr || *dev == nullptr)
    return false;

  bool ret = false;
#ifdef SIMULATION
  *dev = nullptr;
  ret = true;
#else
  ret = Aipu::put_aipu(dev);
#endif
  return ret;
}
} // namespace aipudrv

#endif /* _DEVICE_H_ */
