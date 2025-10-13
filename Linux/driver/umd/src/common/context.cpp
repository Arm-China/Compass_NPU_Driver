// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  context.cpp
 * @brief AIPU User Mode Driver (UMD) context module implementation
 */

#include "context.h"

#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <queue>
#include <set>

#include "device/device.h"
#include "type.h"
#include "utils/debug.h"
#include "utils/helper.h"
#include "utils/log.h"

#if (defined ZHOUYI_V12)
#include "zhouyi_v1v2/graph_v1v2.h"
#endif

#if (defined ZHOUYI_V3) || (defined ZHOUYI_V3_2)
#include "zhouyi_v3x/common/graph_v3x.h"
#endif

#include "job_base.h"
#include "memory_base.h"
#include "parser_base.h"
#include "super_graph.h"

volatile int32_t UMD_LOG_LEVEL = LOG_WARN;
volatile char UMD_LOG_TIMESTAMP = 'n';

#ifndef MACRO_UMD_VERSION
#define MACRO_UMD_VERSION "0.0"
#endif

namespace aipudrv {
MainContext::MainContext() {
  const char *umd_log_level_env = getenv("UMD_LOG_LEVEL");
  const char *umd_log_timestamp_env = getenv("UMD_LOG_TIMESTAMP");

  m_dev = nullptr;
  m_dram = nullptr;
  pthread_rwlock_init(&m_glock, NULL);
  pthread_rwlock_init(&m_lock_share_graph, NULL);
  m_sim_cfg.simulator = nullptr;
  m_sim_cfg.npu_arch_desc = nullptr;
  m_sim_cfg.plugin_name = nullptr;
  m_sim_cfg.json_filename = nullptr;
  m_sim_cfg.log_file_path = new char[BUF_LEN];
  strcpy((char *)m_sim_cfg.log_file_path, "./");
  m_sim_cfg.log_level = 1;
  m_sim_cfg.verbose = true;
  m_sim_cfg.enable_avx = false;
  m_sim_cfg.enable_calloc = false;
  m_sim_cfg.en_eval = false;
  m_sim_cfg.en_l2d = false;
#if (defined ZHOUYI_V3)
  m_sim_cfg.gm_size = 4 * MB_SIZE;
#else
  m_sim_cfg.gm_size = 8 * MB_SIZE;
#endif

  m_sim_cfg.en_fast_perf = 0;
  m_sim_cfg.freq_mhz = 1000;
  m_sim_cfg.ddr_latency_rd = 0;
  m_sim_cfg.ddr_latency_wr = 0;
  m_sim_cfg.ddr_bw = 512;
  m_sim_cfg.ddr_bw_ratio = 1.0;
  m_sim_cfg.perf_report = nullptr;

  m_hw_cfg.poll_in_commit_thread = true;
  m_hw_cfg.enable_tec_done_irq = true;
  if (umd_log_level_env != nullptr) {
    int32_t log_level = umd_log_level_env[0] - '0';
    if (log_level > LOG_WARN && log_level <= LOG_CLOSE)
      UMD_LOG_LEVEL = log_level;
  }

  if (umd_log_timestamp_env != nullptr) {
    UMD_LOG_TIMESTAMP = umd_log_timestamp_env[0];
  }
}

MainContext::~MainContext() {
  pthread_rwlock_destroy(&m_glock);
  pthread_rwlock_destroy(&m_lock_share_graph);
  if (m_sim_cfg.simulator != nullptr) {
    delete[] m_sim_cfg.simulator;
    m_sim_cfg.simulator = nullptr;
  }

  if (m_sim_cfg.npu_arch_desc != nullptr) {
    delete[] m_sim_cfg.npu_arch_desc;
    m_sim_cfg.npu_arch_desc = nullptr;
  }

  if (m_sim_cfg.plugin_name != nullptr) {
    delete[] m_sim_cfg.plugin_name;
    m_sim_cfg.plugin_name = nullptr;
  }

  if (m_sim_cfg.json_filename != nullptr) {
    delete[] m_sim_cfg.json_filename;
    m_sim_cfg.json_filename = nullptr;
  }

  if (m_sim_cfg.perf_report != nullptr) {
    delete[] m_sim_cfg.perf_report;
    m_sim_cfg.perf_report = nullptr;
  }

  if (m_sim_cfg.log_file_path != nullptr) {
    delete[] m_sim_cfg.log_file_path;
    m_sim_cfg.log_file_path = nullptr;
  }
}

bool MainContext::is_deinit_ok() { return true; }

aipu_status_t MainContext::init() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  /* arm64 platform init m_dev here; simulation re-init m_dev later again. */
  ret = get_device(&m_dev);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /* init m_dram here as debugger may allocate buffer after contex initializing
   * done */
  if (m_dev != nullptr && m_dev->get_mem() != nullptr)
    m_dram = m_dev->get_mem();

  m_umd_version = MACRO_UMD_VERSION;

  return ret;
}

void MainContext::force_deinit() {
  pthread_rwlock_wrlock(&m_lock_share_graph);
  for (auto it = m_share_graphs.begin(); it != m_share_graphs.end();) {
    if (it->second != nullptr) {
      it->second->free_shared_weight(it->first);
      delete it->second;
      it->second = nullptr;
      it = m_share_graphs.erase(it);
    } else
      ++it;
  }
  m_share_graphs.clear();
  pthread_rwlock_unlock(&m_lock_share_graph);

  pthread_rwlock_wrlock(&m_glock);
  for (auto it = m_graphs.begin(); it != m_graphs.end(); ++it) {
    if (it->second != nullptr)
      it->second->unload();
  }
  m_graphs.clear();

  if (put_device(&m_dev))
    m_dram = nullptr;

  pthread_rwlock_unlock(&m_glock);
}

aipu_status_t MainContext::deinit() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (!is_deinit_ok()) {
    ret = AIPU_STATUS_ERROR_DEINIT_FAIL;
    goto finish;
  }

  force_deinit();

finish:
  return ret;
}

aipu_status_t MainContext::get_status_msg(aipu_status_t status,
                                          const char **msg) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (msg == nullptr) {
    ret = AIPU_STATUS_ERROR_NULL_PTR;
    goto finish;
  }

  if ((status > AIPU_STATUS_MAX) && (status == m_last_err))
    *msg = m_last_err_msg;
  else
    *msg = get_static_msg(status);

finish:
  return ret;
}

GRAPH_ID MainContext::create_unique_graph_id() {
  GRAPH_ID id_candidate = (1UL << 32);

  pthread_rwlock_wrlock(&m_glock);
  while (m_graphs.count(id_candidate) == 1)
    id_candidate += (1UL << 32);

  /* push nullptr into graphs to pin this graph ID */
  m_graphs[id_candidate] = nullptr;
  pthread_rwlock_unlock(&m_glock);
  return id_candidate;
}

void MainContext::erase_unique_graph_id(GRAPH_ID id) {
  pthread_rwlock_wrlock(&m_glock);
  if (m_graphs.count(id) != 0)
    m_graphs.erase(id);
  pthread_rwlock_unlock(&m_glock);
}

GraphBase *MainContext::get_graph_object(GRAPH_ID id) {
  GraphBase *p_gobj = nullptr;
  pthread_rwlock_rdlock(&m_glock);
  if (m_graphs.count(id) != 0)
    p_gobj = m_graphs[id];
  pthread_rwlock_unlock(&m_glock);

  return p_gobj;
}

JobBase *MainContext::get_job_object(JOB_ID id) {
  GraphBase *p_gobj = get_graph_object(job_id2graph_id(id));
  if (p_gobj == nullptr)
    return nullptr;

  return p_gobj->get_job(id);
}

aipu_status_t MainContext::destroy_graph_object(GraphBase **gobj) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if ((gobj == nullptr) || ((*gobj) == nullptr)) {
    ret = AIPU_STATUS_ERROR_NULL_PTR;
    goto finish;
  }

  ret = (*gobj)->unload();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* success */
  delete *gobj;
  *gobj = nullptr;

finish:
  return ret;
}

aipu_status_t MainContext::create_graph_object(std::istream &gbin,
                                               uint32_t size, GRAPH_ID *id,
                                               GraphBase **gobj) {
  static std::mutex mtex;

  *id = create_unique_graph_id();

  uint32_t g_version = ParserBase::get_graph_bin_version(gbin);
  aipu_status_t ret = set_device_cfg(g_version, &m_dev, &m_sim_cfg);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  {
    std::lock_guard<std::mutex> lock_(mtex);
    if (m_dram == nullptr)
      m_dram = m_dev->get_mem();
  }

  GraphBase *pobj = nullptr;
#if (defined ZHOUYI_V12)
  if (g_version == AIPU_LOADABLE_GRAPH_V0005)
    pobj = new GraphV12(this, *id, m_dev);
#endif
#if (defined ZHOUYI_V3) || (defined ZHOUYI_V3_2)
  if (g_version == AIPU_LOADABLE_GRAPH_ELF_V0)
    pobj = new GraphV3X(this, *id, m_dev);
#endif

  /* success: update graphs[id] */
  pthread_rwlock_wrlock(&m_glock);
  m_graphs[*id] = pobj;
  pthread_rwlock_unlock(&m_glock);

  *gobj = pobj;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t MainContext::load_graph(const char *graph_file, GRAPH_ID *id,
                                      aipu_load_graph_cfg_t *config) {
  if ((graph_file == nullptr) || (id == nullptr))
    return AIPU_STATUS_ERROR_NULL_PTR;

  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  GraphBase *p_gobj = nullptr;
  std::ifstream gbin;
  gbin.open(graph_file, std::ifstream::in | std::ifstream::binary);
  if (!gbin.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  gbin.seekg(0, gbin.end);
  uint32_t size = gbin.tellg();
  gbin.seekg(0, gbin.beg);

  ret = create_graph_object(gbin, size, id, &p_gobj);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  gbin.close();

  if (p_gobj == nullptr) {
    ret = AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;
    goto finish;
  }

  ret = p_gobj->load(graph_file, m_do_vcheck, config);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  ret = p_gobj->alloc_weight_buffer();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

finish:
  if (ret != AIPU_STATUS_SUCCESS) {
    erase_unique_graph_id(*id);
    if (p_gobj != nullptr)
      destroy_graph_object(&p_gobj);
  }

  return ret;
}

aipu_status_t MainContext::load_graph(const char *graph_buf,
                                      uint32_t graph_size, GRAPH_ID *id,
                                      aipu_load_graph_cfg_t *config) {
  if ((graph_buf == nullptr) || (id == nullptr))
    return AIPU_STATUS_ERROR_NULL_PTR;

  if ((graph_size <= 0) || (graph_size > UINT32_MAX))
    return AIPU_STATUS_ERROR_INVALID_SIZE;

  CustomMemBuf buf(const_cast<char *>(graph_buf), graph_size);
  std::istream gbin(&buf);
  if (gbin.fail())
    return AIPU_STATUS_ERROR_READ_FILE_FAIL;

  GraphBase *p_gobj = nullptr;
  aipu_status_t ret = create_graph_object(gbin, graph_size, id, &p_gobj);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (p_gobj == nullptr) {
    ret = AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;
    goto finish;
  }

  ret = p_gobj->load(gbin, graph_size, m_do_vcheck, config);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  ret = p_gobj->alloc_weight_buffer();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

finish:
  if (ret != AIPU_STATUS_SUCCESS) {
    erase_unique_graph_id(*id);
    if (p_gobj != nullptr)
      destroy_graph_object(&p_gobj);
  }
  return ret;
}

aipu_status_t
MainContext::load_share_weight_graph(const char *graph_file, GRAPH_ID **ids,
                                     uint32_t *id_cnt,
                                     aipu_load_graph_cfg_t *config) {
  if (graph_file == nullptr || ids == nullptr || id_cnt == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  aipudrv::SharedWeightMgr *sw_mgr =
      new aipudrv::SharedWeightMgr(this, graph_file);
  aipu_status_t ret = sw_mgr->parse();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  auto &elfs = sw_mgr->get_belfs();
  *id_cnt = elfs.size();
  if (*id_cnt < 2) {
    LOG(LOG_ERR, "there has less than 2 graphs in zip file, please use "
                 "'aipu_load_graph()' API");
    return AIPU_STATUS_ERROR_INVALID_GBIN;
  }

  std::vector<GRAPH_ID> graph_ids(*id_cnt, 0);
  void *elf = nullptr;
  uint32_t elf_size = 0;
  SharedGraphTable::iterator it;

  for (uint32_t i = 0; i < *id_cnt; ++i) {
    elf_size = elfs[i].size;
    ret = umd_mmap_file_helper(sw_mgr->file_path().c_str(), &elf, elf_size,
                               elfs[i].offset, true);
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;

    CustomMemBuf buf(reinterpret_cast<char *>(elf), elf_size);
    std::istream gbin(&buf);
    if (gbin.fail()) {
      ret = AIPU_STATUS_ERROR_READ_FILE_FAIL;
      goto finish;
    }

    GraphBase *p_gobj = nullptr;
    ret = create_graph_object(gbin, elf_size, &graph_ids[i], &p_gobj);
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;

    if (p_gobj == nullptr) {
      ret = AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;
      goto finish;
    }

    Graph *graph = reinterpret_cast<Graph *>(p_gobj);
    graph->set_shared_weight_mgr(sw_mgr);
    ret = graph->load(gbin, elf_size, m_do_vcheck, config);
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;

    munmap(elf, elf_size);
    elf = nullptr;
  }

  ret = sw_mgr->alloc_weight_buffer(graph_ids);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  {
    pthread_rwlock_wrlock(&m_lock_share_graph);
    auto it = m_share_graphs.insert({graph_ids, sw_mgr}).first;
    *ids = const_cast<GRAPH_ID *>(it->first.data());
    pthread_rwlock_unlock(&m_lock_share_graph);
  }

finish:
  if (ret != AIPU_STATUS_SUCCESS) {
    if (elf != nullptr)
      munmap(elf, elf_size);

    for (auto id : graph_ids) {
      GraphBase *p_gobj = get_graph_object(id);
      if (p_gobj != nullptr)
        destroy_graph_object(&p_gobj);
      erase_unique_graph_id(id);
    }

    sw_mgr->free_shared_weight(graph_ids);
    pthread_rwlock_wrlock(&m_lock_share_graph);
    m_share_graphs.erase(graph_ids);
    pthread_rwlock_unlock(&m_lock_share_graph);
    delete sw_mgr;
    sw_mgr = nullptr;
  }
  return ret;
}

aipu_status_t MainContext::unload_graph(GRAPH_ID id) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  GraphBase *p_gobj = nullptr;

  pthread_rwlock_wrlock(&m_glock);
  if (m_graphs.count(id) != 0)
    p_gobj = m_graphs[id];

  if (p_gobj == nullptr) {
    ret = AIPU_STATUS_ERROR_INVALID_GRAPH_ID;
    goto finish;
  }

  ret = destroy_graph_object(&p_gobj);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* success */
  m_graphs.erase(id);
  pthread_rwlock_unlock(&m_glock);

  pthread_rwlock_wrlock(&m_lock_share_graph);
  for (auto it = m_share_graphs.begin(); it != m_share_graphs.end(); ++it) {
    if (std::find(it->first.begin(), it->first.end(), id) != it->first.end() &&
        it->second != nullptr) {
      it->second->free_shared_weight({id});
      if (it->second->get_ref_cnt() == 0) {
        delete it->second;
        it->second = nullptr;
        m_share_graphs.erase(it);
      }
      break;
    }
  }
  pthread_rwlock_unlock(&m_lock_share_graph);

finish:
  return ret;
}

aipu_status_t MainContext::create_job(GRAPH_ID graph, JOB_ID *id,
                                      aipu_create_job_cfg_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  GraphBase *p_gobj = nullptr;

  if (id == nullptr) {
    ret = AIPU_STATUS_ERROR_NULL_PTR;
    goto finish;
  }

  p_gobj = get_graph_object(graph);
  if (p_gobj == nullptr) {
    ret = AIPU_STATUS_ERROR_INVALID_GRAPH_ID;
    goto finish;
  }

  ret = p_gobj->create_job(id, &m_sim_cfg, &m_hw_cfg, config);

finish:
  return ret;
}

aipu_status_t MainContext::get_simulation_instance(void **simulator,
                                                   void **memory) {
  return m_dev->get_simulation_instance(simulator, memory);
}

aipu_status_t MainContext::get_partition_count(uint32_t *cnt) {
  return m_dev->get_partition_count(cnt);
}

aipu_status_t MainContext::get_cluster_count(uint32_t partition_id,
                                             uint32_t *cnt) {
  return m_dev->get_cluster_count(partition_id, cnt);
}

aipu_status_t MainContext::get_core_count(uint32_t partition_id,
                                          uint32_t cluster, uint32_t *cnt) {
  return m_dev->get_core_count(partition_id, cluster, cnt);
}

aipu_status_t MainContext::get_core_info(uint32_t id, aipu_core_info_t *info) {
  return m_dev->get_core_info(id, info);
}

aipu_status_t
MainContext::debugger_get_job_info(uint64_t job,
                                   aipu_debugger_job_info_t *info) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  JobBase *p_job = get_job_object(job);
  if (p_job == nullptr) {
    LOG(LOG_ERR, "cannot find job id 0x%lx", job);
    ret = AIPU_STATUS_ERROR_INVALID_JOB_ID;
    goto finish;
  }

  if (info == nullptr) {
    LOG(LOG_ERR, "arguments have nullptr");
    ret = AIPU_STATUS_ERROR_NULL_PTR;
    goto finish;
  }

  ret = get_simulation_instance(&info->simulation_aipu,
                                &info->simulation_mem_engine);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  info->instr_base = p_job->debugger_get_instr_base();

finish:
  return ret;
}

aipu_status_t
MainContext::config_simulation(uint64_t types,
                               aipu_global_config_simulation_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  char *sim_npu_arch_env = getenv("SIM_NPU_ARCH");

  const std::set<uint32_t> gm_sz_cfg = {0,
                                        512 << 10, // 512KB
                                        1 << 20,   // 1MB
                                        2 << 20,   4 << 20,  8 << 20,
                                        16 << 20,  32 << 20, 64 << 20};

  if (config == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  if (config->simulator != nullptr) {
    if (m_sim_cfg.simulator == nullptr)
      m_sim_cfg.simulator = new char[BUF_LEN];

    strncpy((char *)m_sim_cfg.simulator, config->simulator, BUF_LEN);
  }

  if (config->plugin_name != nullptr) {
    if (m_sim_cfg.plugin_name == nullptr)
      m_sim_cfg.plugin_name = new char[BUF_LEN];

    strncpy((char *)m_sim_cfg.plugin_name, config->plugin_name, BUF_LEN);
  }

  if (config->json_filename != nullptr) {
    if (m_sim_cfg.json_filename == nullptr)
      m_sim_cfg.json_filename = new char[BUF_LEN];

    strncpy((char *)m_sim_cfg.json_filename, config->json_filename, BUF_LEN);
  }

  if (config->log_file_path != nullptr) {
    strncpy((char *)m_sim_cfg.log_file_path, config->log_file_path, BUF_LEN);
  }

  m_sim_cfg.log_level = config->log_level;
  m_sim_cfg.verbose = config->verbose;
  m_sim_cfg.enable_avx = config->enable_avx;
  m_sim_cfg.enable_calloc = config->enable_calloc;
  m_sim_cfg.en_eval = config->en_eval;
  m_sim_cfg.en_l2d = config->en_l2d;

  m_sim_cfg.gm_size = config->gm_size;
  if (gm_sz_cfg.count(m_sim_cfg.gm_size) != 1) {
#if (defined ZHOUYI_V3)
    m_sim_cfg.gm_size = 4 * MB_SIZE;
#else
    m_sim_cfg.gm_size = 8 * MB_SIZE;
#endif
  }

  m_sim_cfg.en_fast_perf = config->en_fast_perf;
  m_sim_cfg.freq_mhz = config->freq_mhz;
  m_sim_cfg.ddr_latency_rd = config->ddr_latency_rd;
  m_sim_cfg.ddr_latency_wr = config->ddr_latency_wr;
  m_sim_cfg.ddr_bw = config->ddr_bw;
  m_sim_cfg.ddr_bw_ratio = config->ddr_bw_ratio;
  if (config->perf_report != nullptr) {
    if (m_sim_cfg.perf_report == nullptr)
      m_sim_cfg.perf_report = new char[BUF_LEN];

    strncpy((char *)m_sim_cfg.perf_report, config->perf_report, BUF_LEN);
  }

  if ((config->npu_arch_desc != nullptr) || (sim_npu_arch_env != nullptr)) {
    if (m_sim_cfg.npu_arch_desc == nullptr)
      m_sim_cfg.npu_arch_desc = new char[64];

    if (config->npu_arch_desc != nullptr)
      strncpy((char *)m_sim_cfg.npu_arch_desc, config->npu_arch_desc, 64);
    else
      strncpy((char *)m_sim_cfg.npu_arch_desc, sim_npu_arch_env, 64);

    ret = set_device_cfg(AIPU_LOADABLE_GRAPH_ELF_V0, &m_dev, &m_sim_cfg);
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
  }

out:
  return ret;
}

aipu_status_t MainContext::config_hw(uint64_t types,
                                     aipu_global_config_hw_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (config == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  m_hw_cfg = *config;
  return ret;
}

aipu_status_t MainContext::debugger_malloc(uint32_t size, void **va) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  BufferDesc *buf = nullptr;
  char *alloc_va = nullptr;

  if ((va == nullptr) || (m_dram == nullptr))
    return AIPU_STATUS_ERROR_NULL_PTR;

  ret = m_dram->malloc(size, 1, &buf, "dbg");
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_dram->pa_to_va(buf->pa, buf->size, &alloc_va) != 0)
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

  /**
   * debugger for Jtag-only need data address 0/1 register (0x14/0x18) of V1/V2
   * or the TSM command schedule address hi/lo register (0x8/0xc) of V3x as
   * channel between server & client
   *
   * dreg0: buffer base address in device space
   * dreg1: magic number requested by debugger
   */
  if (m_dev->get_npu_version() >= AIPU_ISA_VERSION_ZHOUYI_V3) {
    m_dev->write_reg(0, 0x0c, buf->pa);
    m_dev->write_reg(0, 0x08, 0x1248FFA5);
  } else {
    m_dev->write_reg(0, 0x14, buf->pa);
    m_dev->write_reg(0, 0x18, 0x1248FFA5);
  }

  m_dbg_buffers[alloc_va] = buf;
  *va = alloc_va;
  return ret;
}

aipu_status_t MainContext::debugger_free(void *va) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (va == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  if (m_dbg_buffers.count(va) == 0)
    return AIPU_STATUS_ERROR_BUF_FREE_FAIL;

  ret = m_dram->free(&m_dbg_buffers[va], "dbg");
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  m_dbg_buffers.erase(va);
  return ret;
}

aipu_status_t MainContext::aipu_get_target(char *target) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t isa = AIPU_ISA_VERSION_ZHOUYI_V1;
  uint32_t config = 904;
  std::string isa_version, arch_cfg;
  std::stringstream config_ss;
  std::set<std::string> arch_set = {
      "Z1_0904", "Z1_1002", "Z1_0701", "Z2_1104", "Z2_1002",  "Z2_0901",
      "Z3_1104", "Z3_0901", "X1_1204", "X2_1204", "X3P_1304",
  };

#if SIMULATION
  const char *nul_ptr = "null";
  strcpy(target, nul_ptr);
  return ret;
#endif

  isa = m_dev->get_npu_version();
  config = m_dev->get_npu_config();

  switch (isa) {
  case AIPU_ISA_VERSION_ZHOUYI_V1:
    isa_version = "Z1_";
    break;
  case AIPU_ISA_VERSION_ZHOUYI_V2_0:
    isa_version = "Z2_";
    break;
  case AIPU_ISA_VERSION_ZHOUYI_V2_1:
    isa_version = "Z3_";
    break;
  case AIPU_ISA_VERSION_ZHOUYI_V2_2:
    isa_version = "X1_";
    break;
  case AIPU_ISA_VERSION_ZHOUYI_V3:
    isa_version = "X2_";
    config = 1204;
    break;
  case AIPU_ISA_VERSION_ZHOUYI_V3_2:
    isa_version = "X3P_";
    config = 1300 + m_dev->get_npu_tec_cnt();
    break;
  default:
    return AIPU_STATUS_ERROR_INVALID_CONFIG;
  }

  config_ss << std::setw(4) << std::setfill('0') << config;
  arch_cfg = isa_version + config_ss.str();
  if (arch_set.count(arch_cfg.c_str()) == 0)
    return AIPU_STATUS_ERROR_INVALID_CONFIG;

  uint32_t core_num = m_dev->get_npu_core_cnt();
  if (core_num > 1)
    arch_cfg = arch_cfg + "MP" + std::to_string(core_num);

  strncpy(target, arch_cfg.c_str(), arch_cfg.size());
  return ret;
}

aipu_status_t MainContext::aipu_get_device_status(device_status_t *status) {
  if (status == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

#ifdef SIMULATION
  *status = DEV_IDLE;
  return AIPU_STATUS_SUCCESS;
#endif
  return convert_ll_status(m_dev->get_device_status(status));
}

aipu_status_t MainContext::get_status(JobBase *job, aipu_job_status_t *status) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  const char *msg = nullptr;
  int timeout = 3600; // prompt per 1 hour

  while ((*status != AIPU_JOB_STATUS_DONE) &&
         (*status != AIPU_JOB_STATUS_EXCEPTION)) {
    ret = job->get_status_blocking(status, 1000);
    if (ret == AIPU_STATUS_ERROR_TIMEOUT) {
      if (--timeout == 0) {
        ret = AIPU_STATUS_ERROR_TIMEOUT;
        LOG(LOG_WARN, "job: %p polled over 1h", job);
      }
      continue;
    } else if (ret != AIPU_STATUS_SUCCESS) {
      get_status_msg(ret, &msg);
      LOG(LOG_ERR, "job status: %s", msg);
      goto out;
    }
  }

out:
  return ret;
}

aipu_status_t MainContext::run_batch(GraphBase &graph, uint32_t queue_id,
                                     aipu_create_job_cfg_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS, oldret = AIPU_STATUS_SUCCESS;
  JOB_ID job_id = 0;
  JobBase *job = nullptr;
  aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;
  typedef struct job_info {
    JOB_ID job_id;
    JobBase *job;
    batch_info_t *batch;
  } job_info_t;

  job_info_t job_info_item = {0};
  std::queue<job_info_t> job_queue;
  uint32_t types = 0;
  uint32_t batch_num = 0;
  uint32_t max_in_flight = 3;
  uint32_t batch_queue_size = 0;
  const char *umd_max_batch = getenv("UMD_MAX_BATCH");

  if (!graph.is_valid_batch_queue(queue_id))
    return AIPU_STATUS_ERROR_NO_BATCH_QUEUE;

  if (umd_max_batch != nullptr) {
    int max_batch = atoi(umd_max_batch);
    max_in_flight = (max_batch != 0) ? max_batch : max_in_flight;
  }

  batch_queue_size = graph.get_batch_queue_size(queue_id);
  types = graph.get_batch_dump_type(queue_id);
repeat:
  for (; batch_num < batch_queue_size; batch_num++) {
    if (job_queue.size() >= max_in_flight)
      break;

    batch_info_t &batch = graph.get_batch_queue_item(queue_id, batch_num);
    ret = graph.create_job(&job_id, &m_sim_cfg, &m_hw_cfg, config);
    if (ret == AIPU_STATUS_ERROR_BUF_ALLOC_FAIL) {
      break;
    } else if (ret != AIPU_STATUS_SUCCESS) {
      oldret = ret;
      goto poll_job_sts;
    }

    job = graph.get_job(job_id);
#ifdef SIMULATION
    if (types & AIPU_CONFIG_TYPE_SIMULATION) {
      aipu_job_config_simulation_t sim_config = {0};

      sim_config.data_dir = graph.get_batch_dump_path(queue_id);
      ret = job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_config);
      if (ret != AIPU_STATUS_SUCCESS) {
        oldret = ret;
        goto poll_job_sts;
      }
    }
#endif

    if (types & (~AIPU_CONFIG_TYPE_SIMULATION)) {
      aipu_job_config_dump_t dump_config = {0};

      dump_config.dump_dir = graph.get_batch_dump_path(queue_id);
      ret = job->config_mem_dump(types, &dump_config);
      if (ret != AIPU_STATUS_SUCCESS) {
        oldret = ret;
        goto poll_job_sts;
      }
    }

    for (uint32_t in_idx = 0; in_idx < batch.inputs.size(); in_idx++) {
      ret = job->load_tensor(in_idx, batch.inputs[in_idx]);
      if (ret != AIPU_STATUS_SUCCESS) {
        oldret = ret;
        goto poll_job_sts;
      }
    }

    ret = job->schedule();
    if (ret != AIPU_STATUS_SUCCESS) {
      oldret = ret;
      goto poll_job_sts;
    }

    job_info_item.job_id = job_id;
    job_info_item.job = job;
    job_info_item.batch = &batch;
    job_queue.push(job_info_item);
  }

poll_job_sts:
  while (job_queue.size() > 0) {
#ifdef SIMULATION
    uint32_t min_in_flight =
        (job_queue.size() < max_in_flight) ? job_queue.size() : max_in_flight;
    for (uint32_t i = 0; i < min_in_flight; i++) {
      job_info_item = job_queue.front();
      job_queue.pop();

      status = AIPU_JOB_STATUS_NO_STATUS;
      ret = get_status(job_info_item.job, &status);
      if (ret != AIPU_STATUS_SUCCESS)
        goto out;

      for (uint32_t out_idx = 0; out_idx < job_info_item.batch->outputs.size();
           out_idx++) {
        ret = job_info_item.job->get_tensor(
            AIPU_TENSOR_TYPE_OUTPUT, out_idx,
            job_info_item.batch->outputs[out_idx]);
        if (ret != AIPU_STATUS_SUCCESS)
          goto out;
      }

      ret = graph.destroy_job(job_info_item.job_id);
      if (ret != AIPU_STATUS_SUCCESS)
        goto out;
    }
#else
    job_info_t job_info_item = job_queue.front();
    job_queue.pop();
    status = AIPU_JOB_STATUS_NO_STATUS;
    ret = get_status(job_info_item.job, &status);
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;

    if (status == AIPU_JOB_STATUS_EXCEPTION) {
      LOG(LOG_ERR, "job exception, check HW status");
      goto out;
    }

    for (uint32_t out_idx = 0; out_idx < job_info_item.batch->outputs.size();
         out_idx++) {
      ret =
          job_info_item.job->get_tensor(AIPU_TENSOR_TYPE_OUTPUT, out_idx,
                                        job_info_item.batch->outputs[out_idx]);
      if (ret != AIPU_STATUS_SUCCESS)
        goto out;
    }

    ret = graph.destroy_job(job_info_item.job_id);
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
#endif

    if ((oldret == AIPU_STATUS_SUCCESS) &&
        (batch_num < graph.get_batch_queue_size(queue_id)))
      goto repeat;
  }

out:
  for (uint32_t i = 0; i < job_queue.size(); i++) {
    job_info_t job_info_item = job_queue.front();
    job_queue.pop();
    graph.destroy_job(job_info_item.job_id);
  }
  graph.clean_batches(queue_id);

  if (oldret != AIPU_STATUS_SUCCESS)
    return oldret;
  else
    return ret;
}

aipu_status_t MainContext::ioctl_cmd(uint32_t cmd, void *arg) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  GraphBase *p_gobj = nullptr;

  if (cmd != AIPU_IOCTL_ENABLE_TICKCOUNTER &&
      cmd != AIPU_IOCTL_DISABLE_TICKCOUNTER &&
      cmd != AIPU_IOCTL_ABORT_CMDPOOL) {
    if (arg == nullptr)
      return AIPU_STATUS_ERROR_NULL_PTR;
  }

  if (cmd < AIPU_IOCTL_SET_PROFILE || cmd >= AIPU_IOCTL_UMD_MAX)
    return AIPU_STATUS_ERROR_INVALID_OP;

  switch (cmd) {
  case AIPU_IOCTL_SET_PROFILE:
    m_dev->enable_profiling((*(int32_t *)arg) != 0);
    break;

  case AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION: {
    aipu_bin_buildversion_t *buildver = (aipu_bin_buildversion_t *)arg;

    if (!valid_graph_id(buildver->graph_id))
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_gobj = get_graph_object(buildver->graph_id);
    if (p_gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    buildver->aipubin_buildversion = p_gobj->get_buildversion();
  } break;

  case AIPU_IOCTL_ALLOC_SHARE_BUF: {
    aipu_share_buf_t *share_buf = (aipu_share_buf_t *)arg;
    BufferDesc *buf = nullptr;
    uint32_t pad_size = 0;
#if defined(ZHOUYI_V3) && !defined(SIMULATION)
    pad_size = 0x800;
#endif
    ret = m_dram->malloc(share_buf->size + pad_size, 1, &buf, "share",
                         share_buf->mem_type);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    if (m_dram->pa_to_va(buf->pa, buf->size, (char **)&share_buf->va) != 0)
      return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    share_buf->pa = buf->pa;
  } break;

  case AIPU_IOCTL_FREE_SHARE_BUF: {
    aipu_share_buf_t *share_buf = (aipu_share_buf_t *)arg;
    Buffer buffer;

    if (m_dram->get_shared_buffer(share_buf->pa, share_buf->size, buffer) != 0)
      return AIPU_STATUS_ERROR_SET_SHARED_TENSOR;

    ret = m_dram->free(&buffer.desc, "share");
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  } break;

  case AIPU_IOCTL_GET_DS_NUM: /* [[deprecate]]*/
  {
    aipu_dynshape_num_t *ds_num = (aipu_dynshape_num_t *)arg;

    if (!valid_graph_id(ds_num->graph_id))
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_gobj = get_graph_object(ds_num->graph_id);
    if (p_gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    if (ds_num->ds_num != nullptr) {
      *ds_num->ds_num = p_gobj->get_dynamic_shape_num();
    } else {
      LOG(LOG_ERR, "ds_num ptr is NULL");
      return AIPU_STATUS_ERROR_NULL_PTR;
    }
  } break;

  case AIPU_IOCTL_GET_DS_DIM_NUM: /* [[deprecate]]*/
  {
    aipu_dynshape_dim_num_t *ds_dim_num = (aipu_dynshape_dim_num_t *)arg;

    if (!valid_graph_id(ds_dim_num->graph_id))
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_gobj = get_graph_object(ds_dim_num->graph_id);
    if (p_gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    if (ds_dim_num->ds_dim_num != nullptr) {
      *ds_dim_num->ds_dim_num = p_gobj->get_dynamic_shape_dim_num(
          ds_dim_num->ds_idx, ds_dim_num->max_threshhold);
    } else {
      LOG(LOG_ERR, "ds_dim_num ptr is NULL");
      return AIPU_STATUS_ERROR_NULL_PTR;
    }
  } break;

  case AIPU_IOCTL_GET_DS_INFO: /* [[deprecate]]*/
  {
    aipu_dynshape_info_t *ds_info = (aipu_dynshape_info_t *)arg;

    if (!valid_graph_id(ds_info->graph_id))
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_gobj = get_graph_object(ds_info->graph_id);
    if (p_gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    bool success = p_gobj->get_dynamic_shape_data(
        ds_info->ds_idx, ds_info->max_threshhold, ds_info->ds_data);
    if (!success) {
      LOG(LOG_ERR, "get dynamic shape failed");
      return AIPU_STATUS_ERROR_GET_SHAPE_FAILED;
    }
  } break;

  case AIPU_IOCTL_GET_DS_RANK: {
    aipu_dynshape_rank_t *p_rank = (aipu_dynshape_rank_t *)arg;
    if (!valid_graph_id(p_rank->graph_id))
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_gobj = get_graph_object(p_rank->graph_id);
    if (p_gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_rank->rank = p_gobj->get_dynamic_shape_dim_num(p_rank->ds_idx, true);
  } break;

  case AIPU_IOCTL_GET_DS_DIM_CONSTRAINT: {
    aipu_dynshape_dimension_t *p_dims = (aipu_dynshape_dimension_t *)arg;

    if (!valid_graph_id(p_dims->graph_id))
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    p_gobj = get_graph_object(p_dims->graph_id);
    if (p_gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    bool success =
        p_gobj->get_dynamic_shape_data(p_dims->ds_idx, true, p_dims->max_dim);
    success = success && p_gobj->get_dynamic_shape_data(p_dims->ds_idx, false,
                                                        p_dims->min_dim);
    if (!success) {
      LOG(LOG_ERR, "get dynamic shape failed");
      return AIPU_STATUS_ERROR_GET_SHAPE_FAILED;
    }
  } break;

  case AIPU_IOCTL_SET_DYNAMIC_ASID1: /* only for v3 */
  {
    bool dynamic_asid1 = *(bool *)arg;
    m_dram->set_asid1(dynamic_asid1 ? 1 : 0);
  } break;

  case AIPU_IOCTL_RW_REGISTER: {
    aipu_reg_rw_t *reg_rw = (aipu_reg_rw_t *)arg;
    if (reg_rw->type == AIPU_IOCTL_READ_TSMREG)
      ret = convert_ll_status(m_dev->read_reg(0, reg_rw->offset, &reg_rw->value,
                                              RegType::AIPU_HOST_REG));
    else if (reg_rw->type == AIPU_IOCTL_WRITE_TSMREG)
      ret = convert_ll_status(m_dev->write_reg(0, reg_rw->offset, reg_rw->value,
                                               RegType::AIPU_HOST_REG));
    else if (reg_rw->type == AIPU_IOCTL_READ_DBGREG)
      ret = convert_ll_status(m_dev->read_reg(0, reg_rw->offset, &reg_rw->value,
                                              RegType::AIPU_DBG_REG));
    else if (reg_rw->type == AIPU_IOCTL_WRITE_DBGREG)
      ret = convert_ll_status(m_dev->write_reg(0, reg_rw->offset, reg_rw->value,
                                               RegType::AIPU_DBG_REG));
    else {
      LOG(LOG_ERR, "invalid register control type: %u", reg_rw->type);
      ret = AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
    }
  } break;

  case AIPU_IOCTL_RW_MEMORY: {
    aipu_mem_rw_t *mem_rw = (aipu_mem_rw_t *)arg;

    JobBase *job = get_job_object(mem_rw->job_id);
    if (job == nullptr)
      return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    DEV_PA_64 pa = job->get_asid0_base() + mem_rw->addr;
    char *va = nullptr;
    if (m_dram->pa_to_va(pa, mem_rw->size, &va) != 0)
      return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;

    if (mem_rw->type == AIPU_IOCTL_READ_MEM)
      memcpy(mem_rw->data, va, mem_rw->size);
    else if (mem_rw->type == AIPU_IOCTL_WRITE_MEM)
      memcpy(va, mem_rw->data, mem_rw->size);
    else {
      LOG(LOG_ERR, "invalid memory control type: %u", mem_rw->type);
      ret = AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
    }
  } break;

  default: {
#ifndef SIMULATION
    ret = convert_ll_status(m_dev->ioctl_cmd(cmd, arg));
#endif
    if (cmd == AIPU_IOCTL_GET_VERSION) {
      aipu_driver_version_t *drv_ver = (aipu_driver_version_t *)arg;
      strncpy(drv_ver->umd_version, m_umd_version.c_str(),
              m_umd_version.length());
    }
  }
  }
  return ret;
}

} // namespace aipudrv
