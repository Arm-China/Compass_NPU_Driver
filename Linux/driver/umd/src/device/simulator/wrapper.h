// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  wrapper.h
 * @brief AIPU User Mode Driver (UMD) simulator wrapper header
 */

#ifndef _SIM_WRAPPER_H_
#define _SIM_WRAPPER_H_

#include <dlfcn.h>
#include <string.h>

#include <algorithm>
#include <string>

#include "device_base.h"
#include "memory_base.h"
#include "simulator/aipu_c.h"
#include "standard_api.h"
#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
using event_handler_t = void (*)(uint32_t, uint64_t, void *);

struct arch_item_t {
  uint32_t config;
  uint32_t tec_num;
  uint32_t aiff_num;
  uint32_t cluster_num;
  int32_t sim_code;
};

class SimWrapper {
private:
  std::string m_target;
  std::string m_so_name;
  void *m_so_handler = nullptr;
  sim_aipu_t m_sim_aipu = nullptr;
  struct sim_aipu_api_t *m_aipu_ops = nullptr;
  mem_interface_t m_mem;
  config_t m_config;

  /* {config,tec_num,aiff_num,core_num,sim_code} */
  const std::map<std::string, arch_item_t> m_npu_arch_map = {
      {"X2_1204", {1204, 4, 1, 1, X2_1204}},
      {"X2_1204MP3", {1204, 4, 1, 1, X2_1204MP3}},
      {"X3P_1304", {1304, 4, 2, 1, -1}}, /*x3p generates sim code */
      {"X3P_1302", {1302, 2, 2, 1, -1}},
      {"X3P_1202", {1202, 2, 1, 1, -1}},
      {"X3P_1204", {1204, 4, 1, 1, -1}},
      {"X3S_1304", {1304, 4, 2, 1, -1}},
  };

private:
  SimWrapper() = delete;
  SimWrapper(const SimWrapper &) = delete;
  SimWrapper(SimWrapper &&) = delete;
  SimWrapper &operator=(const SimWrapper &) = delete;
  SimWrapper &operator=(SimWrapper &&) = delete;

  aipu_ll_status_t load_simulator();

  int target_to_code(const std::string &target) const {
    int code = -1;
    if (target.find("X2") != std::string::npos) {
      if (m_npu_arch_map.count(target) != 0)
        code = m_npu_arch_map.at(target).sim_code;
    } else if (target.find("X3") != std::string::npos) {
      uint32_t core_num = 1;
      size_t pos = 0;
      std::string bare_target = target;
      if ((pos = target.find("MP", pos)) != std::string::npos) {
        auto mp = target.substr(pos + 2, target.size() - pos - 2);
        core_num = std::stoi(mp);
        if (core_num == 0)
          core_num = 1;
        if (pos > 0 && target[pos - 1] == '_')
          pos = pos - 1;
        bare_target = target.substr(0, pos);
      }

      if (m_npu_arch_map.count(bare_target) != 0) {
        auto &item = m_npu_arch_map.at(bare_target);
        code = (item.tec_num & 0xF) + ((item.aiff_num & 0xF) << 4) +
               ((core_num & 0xF) << 8) + ((item.cluster_num & 0xF) << 12) +
               (1 << 31);
      }
    }
    return code;
  }

public:
  SimWrapper(const std::string &target,
             const aipu_global_config_simulation_t *cfg);
  ~SimWrapper();

  aipu_ll_status_t init();

  sim_aipu_t sim() { return m_sim_aipu; }

  mem_interface_t *mem() { return &m_mem; }

  const config_t &config() { return m_config; }

  int read_register(uint32_t addr, uint32_t &v) const {
    return m_aipu_ops->read_register(m_sim_aipu, addr, &v);
  }

  int write_register(uint32_t addr, uint32_t v) {
    return m_aipu_ops->write_register(m_sim_aipu, addr, v);
  }

  int version() { return m_aipu_ops->version(m_sim_aipu); }

  void enable_profiling(bool en) {
    if (en)
      m_config.perf.mode = m_target.find("X2") != std::string::npos
                               ? PERF_MODE_EVAL
                               : PERF_MODE_FAST;
    else
      m_config.perf.mode = PERF_MODE_NONE;

    return m_aipu_ops->enable_profiling(m_sim_aipu, en);
  }

  void dump_profiling() { return m_aipu_ops->dump_profiling(m_sim_aipu); }

  void set_event_handler(event_handler_t handler, void *context) {
    return m_aipu_ops->set_event_handler(m_sim_aipu, handler, context);
  }

  config_t to_sim_conf(const aipu_global_config_simulation_t *conf) {
    config_t sim_conf = {0};
    sim_conf.code = target_to_code(m_target);
    sim_conf.gm_size = 0x800000;
    sim_conf.log.level = 1;
    sim_conf.log.verbose = true;
    sim_conf.perf.sys_freq = 1000;
    sim_conf.perf.ddr.rd_latency = 0;
    sim_conf.perf.ddr.wr_latency = 0;
    sim_conf.perf.ddr.bandwidth = 256;
    sim_conf.fp_mode = false;
    sim_conf.print_subg_info = false;
    strcpy(sim_conf.perf.report_filename, "./perf.csv");
    strcpy(sim_conf.log.filepath, "./");

    if (conf != nullptr) {
      sim_conf.enable_avx = conf->enable_avx;
      sim_conf.enable_calloc = conf->enable_calloc;
      sim_conf.max_pkg_num = -1;
      sim_conf.gm_size = conf->gm_size;
      sim_conf.log.level = conf->log_level;
      sim_conf.log.verbose = conf->verbose;

      if (conf->log_file_path != nullptr)
        strcpy(sim_conf.log.filepath, conf->log_file_path);
      if (conf->plugin_name != nullptr)
        strcpy(sim_conf.plugin_filename, conf->plugin_name);
      if (conf->json_filename != nullptr)
        strcpy(sim_conf.graph_filename, conf->json_filename);

      sim_conf.fp_mode = conf->fp_mode;
      sim_conf.print_subg_info = conf->print_subg_info;
      sim_conf.perf.mode = PERF_MODE_NONE;
      if (m_target.find("X3") != std::string::npos) {
        if (conf->en_fast_perf)
          sim_conf.perf.mode = PERF_MODE_FAST;

        sim_conf.perf.sys_freq = conf->freq_mhz;
        sim_conf.perf.ddr.rd_latency = conf->ddr_latency_rd;
        sim_conf.perf.ddr.wr_latency = conf->ddr_latency_wr;
        sim_conf.perf.ddr.bandwidth = conf->ddr_bw;
        if (conf->perf_report != nullptr)
          strcpy(sim_conf.perf.report_filename, conf->perf_report);
      } else if (m_target.find("X2") != std::string::npos && conf->en_eval)
        sim_conf.perf.mode = PERF_MODE_EVAL;
    }

    auto mode_to_string = [](uint32_t mode) {
      const static std::map<uint32_t, std::string> table = {
          {PERF_MODE_NONE, "NONE"},   {PERF_MODE_FAST, "FAST"},
          {PERF_MODE_EVAL, "EVAL"},   {PERF_MODE_IDU, "IDU"},
          {PERF_MODE_PROBE, "PROBE"},
      };

      return table.at(mode);
    };

    LOG(LOG_DEBUG,
        "\n---------simulator configuration---------\n"
        "code = 0x%x\n"
        "enable_avx = %d\n"
        "enable_calloc = %d\n"
        "max_pkg_num = %ld\n"
        "gm_size = 0x%x\n"
        "log.filepath = %s\n"
        "log.level = %d\n"
        "log.verbose = %d\n"
        "plugin_filename = %s\n"
        "graph_filename = %s\n"
        "fp_mode = %d\n"
        "perf.mode = %s\n"
        "perf.report_filename = %s\n"
        "perf.sys_freq = %u\n"
        "perf.ddr.rd_latency = %u\n"
        "perf.ddr.wr_latency = %u\n"
        "perf.ddr.bandwidth = %u\n",
        sim_conf.code, sim_conf.enable_avx, sim_conf.enable_calloc,
        sim_conf.max_pkg_num, sim_conf.gm_size, sim_conf.log.filepath,
        sim_conf.log.level, sim_conf.log.verbose, sim_conf.plugin_filename,
        sim_conf.graph_filename, sim_conf.fp_mode,
        mode_to_string(sim_conf.perf.mode).c_str(),
        sim_conf.perf.report_filename, sim_conf.perf.sys_freq,
        sim_conf.perf.ddr.rd_latency, sim_conf.perf.ddr.wr_latency,
        sim_conf.perf.ddr.bandwidth);

    return sim_conf;
  }

  const std::string &code_to_string() const {
    static std::string code_str;

    if (m_target.find("X2") != std::string::npos) {
      code_str = m_target;
    } else if (m_target.find("X3") != std::string::npos) {
      code_str = std::string("X3P-K") +
                 std::to_string((m_config.code >> 12) & 0xF) + "C" +
                 std::to_string((m_config.code >> 8) & 0xF) + "A" +
                 std::to_string((m_config.code >> 4) & 0xF) + "T" +
                 std::to_string(m_config.code & 0xF);
    }
    return code_str;
  }

  template <typename Function> Function load_fn(const char *fn_name) {
    void *fn = dlsym(m_so_handler, fn_name);
    if (fn == nullptr)
      LOG(LOG_ERR, "dlsym funciton %s fail", fn_name);

    return reinterpret_cast<Function>(fn);
  }
};

} // namespace aipudrv

#endif /* _SIM_WRAPPER_H_ */
