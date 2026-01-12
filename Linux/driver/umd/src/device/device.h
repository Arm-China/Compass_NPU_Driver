// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
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

#ifdef ZHOUYI_V3_2
#include "device/simulator/simulator_v3_2.h"
#endif

#else
#include "device/aipu/aipu.h"
#endif

#ifndef _DEVICE_H_
#define _DEVICE_H_

namespace aipudrv {
inline aipu_status_t get_device(const std::string &target, uint32_t hw_version,
                                DeviceBase **dev,
                                const aipu_global_config_simulation_t *cfg) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  std::mutex m_tex;

  if (dev == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

#if (defined SIMULATION)
  std::lock_guard<std::mutex> lock_(m_tex);
#if (defined ZHOUYI_V12)
  if (hw_version <= AIPU_ISA_VERSION_ZHOUYI_V2_2) {
    /**
     * The current device maybe V3 because of the new APIs changes in
     * get_device, so here just assign the needed simulator instance pointer
     * without target check.
     * */
    *dev = Simulator::get_simulator();
  }
#endif
#if (defined ZHOUYI_V3)
  if (hw_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
    if (*dev != nullptr && (*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3) {
      LOG(LOG_ERR, "aipu binary is x2, but simulator is initialized as %u",
          (*dev)->get_dev_type());
      return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    } else {
      *dev = SimulatorV3::get_v3_simulator();
      (*dev)->set_target(target);
      (*dev)->set_cfg(cfg);
      ret = convert_ll_status((*dev)->init());
    }
  }
#endif

#if (defined ZHOUYI_V3_2)
  if (hw_version >= AIPU_ISA_VERSION_ZHOUYI_V3_2_0) {
    if (*dev != nullptr && (*dev)->get_dev_type() != DEV_TYPE_SIMULATOR_V3_2) {
      LOG(LOG_ERR, "aipu binary is x3p, but simulator is initialized as %u",
          (*dev)->get_dev_type());
      return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
    } else {
      *dev = SimulatorV3_2::get_v3_2_simulator();
      (*dev)->set_target(target);
      (*dev)->set_cfg(cfg);
      ret = convert_ll_status((*dev)->init());
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

#ifndef SIMULATION
  ret = convert_ll_status(Aipu::get_aipu(dev));
#endif

  return ret;
}

inline bool put_device(DeviceBase **dev) {
  if (dev == nullptr || *dev == nullptr)
    return false;

  bool ret = false;
#ifndef SIMULATION
  ret = Aipu::put_aipu(dev);
#endif
  return ret;
}
} // namespace aipudrv

#endif /* _DEVICE_H_ */
