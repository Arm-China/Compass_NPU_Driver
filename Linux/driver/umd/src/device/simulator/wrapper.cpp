// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  wrapper.cpp
 * @brief AIPU User Mode Driver (UMD) simulator wrapper implementation
 */

#include "wrapper.h"

#include <unistd.h>

#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

#include "device/simulator/umemory.h"

namespace aipudrv {
namespace {
UMemory *dram = UMemory::get_memory();

int64_t read(uint64_t addr, void *dest, size_t size, void *) {
  return dram->read(addr, dest, size);
}

int64_t write(uint64_t addr, const void *src, size_t size, void *) {
  return dram->write(addr, src, size);
}

int64_t zeroize(uint64_t addr, size_t size, void *) {
  return dram->zeroize(addr, size);
}

size_t size(void *) { return dram->size(); }

bool invalid(uint64_t addr, void *) { return dram->invalid(addr); }

bool get_info(uint64_t addr, uint64_t *base, size_t *size, void *) {
  return dram->get_info(addr, *base, *reinterpret_cast<uint32_t *>(size));
}

} // namespace

SimWrapper::SimWrapper(const std::string &target,
                       const aipu_global_config_simulation_t *cfg) {
  m_target = target;
  m_config = to_sim_conf(cfg);
}

aipu_ll_status_t SimWrapper::init() {
  if (m_config.code == -1) {
    LOG(LOG_ERR, "target %s cannot generate correct code for simulator",
        m_target.c_str());
    return AIPU_LL_STATUS_ERROR_SIM_CONFIG_FAIL;
  }

  auto strs = split_string(m_target, "_");
  std::transform(strs[0].begin(), strs[0].end(), strs[0].begin(), ::tolower);
  m_so_name = std::string("libaipu_simulator_") + strs[0] + ".so";
  return load_simulator();
}

SimWrapper::~SimWrapper() {
  if (m_aipu_ops != nullptr) {
    m_aipu_ops->destroy(m_sim_aipu);
    m_aipu_ops = nullptr;
  }
  if (m_so_handler != nullptr) {
    int32_t err = dlclose(m_so_handler);
    if (err != 0)
      LOG(LOG_ERR, "dlclose %s fail", m_so_name.c_str());
    m_so_handler = nullptr;
  }
}

aipu_ll_status_t SimWrapper::load_simulator() {
  m_so_handler = dlopen(m_so_name.c_str(), RTLD_LAZY | RTLD_LOCAL);
  if (m_so_handler == nullptr) {
    LOG(LOG_ERR, "dlopen %s fail", m_so_name.c_str());
    return AIPU_LL_STATUS_ERROR_OPEN_FAIL;
  }

  fn_sim_aipu_api_init_t init_fn =
      load_fn<fn_sim_aipu_api_init_t>("sim_aipu_api_init");
  if (init_fn == nullptr) {
    LOG(LOG_ERR, "dlsym simulator 'fn_sim_aipu_api_init_t' failed");
    return AIPU_LL_STATUS_ERROR_SIM_CONFIG_FAIL;
  }

  m_aipu_ops = init_fn();

  m_mem.cxt = nullptr;
  m_mem.read = read;
  m_mem.write = write;
  m_mem.zeroize = zeroize;
  m_mem.size = size;
  m_mem.invalid = invalid;
  m_mem.get_info = get_info;
  m_sim_aipu = m_aipu_ops->create(&m_config, &m_mem);

  return AIPU_LL_STATUS_SUCCESS;
}

} // namespace aipudrv
