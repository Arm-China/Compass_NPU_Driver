// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  simulator.h
 * @brief AIPU User Mode Driver (UMD) zhouyi z1/2/3 simulator module header
 */

#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <string>
#include <vector>

#include "device/simulator/umemory.h"
#include "device_base.h"
#include "type.h"
#include "utils/debug.h"

namespace aipudrv {
#define FNAME_LEN 2048
#define OPT_LEN 2148
#define CMD_MEN 4096

struct SimulationData {
  std::string fname;
  DEV_PA_64 pa;
  uint32_t size;
};

struct SimulationJobCtx {
  std::string text;
  std::string weight;
  std::string zerocpy_const;
  std::string rodata;
  std::string dcr;
  std::string stack;
  std::vector<std::string> reuses;
  std::vector<std::string> weights;
  std::vector<std::string> outputs;
  char simulation_cmd[CMD_MEN];
};

class Simulator : public DeviceBase {
public:
  virtual ~Simulator();
  Simulator(const Simulator &sim) = delete;
  Simulator &operator=(const Simulator &sim) = delete;

  bool has_target(uint32_t arch, uint32_t version, uint32_t config,
                  uint32_t rev) override;
  aipu_ll_status_t schedule(const JobDesc &job);

  static Simulator *get_simulator() {
    static Simulator sim_instance;
    return &sim_instance;
  }

  aipu_ll_status_t get_simulation_instance(void **, void **) {
    LOG(LOG_ERR, "v1/v2 has no simulator instance");
    return AIPU_LL_STATUS_ERROR_INVALID_OP;
  }

private:
  Simulator();
};
} // namespace aipudrv

#endif /* _SIMULATOR_H_ */
