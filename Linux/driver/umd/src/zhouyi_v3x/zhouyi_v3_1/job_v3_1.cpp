// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3_1.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3_1 job module implementation
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <mutex>
#include <vector>

// clang-format off
#include "job_v3_1.h"
// clang-format on

#include "utils/helper.h"
#include "zhouyi_v3x/common/graph_v3x.h"
#include "zhouyi_v3x/zhouyi_v3_1/gm_v3_1.h"

#if defined(SIMULATION)
#include "device/simulator/simulator_v3_1.h"
#endif

namespace aipudrv {
JobV3_1::JobV3_1(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
                 aipu_create_job_cfg_t *config)
    : JobV3X(ctx, graph, dev, config) {
  m_gm = new GM_V3_1(*this);
}

JobV3_1::~JobV3_1() {
  if (m_dyn_shape != nullptr) {
    delete m_gm;
    m_gm = nullptr;
  }

  if (m_dyn_shape != nullptr) {
    delete m_dyn_shape;
    m_dyn_shape = nullptr;
  }
}

void JobV3_1::set_job_params(uint32_t sg_cnt, uint32_t task_per_sg,
                             uint32_t remap, uint32_t core_cnt) {
  m_sg_cnt = sg_cnt;
  m_task_per_sg = task_per_sg;
  m_remap_flag = remap;

  /**
   * tcb chain format
   * global group init tcb:
   *     1 grid init-tcb + 1 group init-tcb + n task-tcb grp
   *
   * local group init tcb:
   *     1 grid init-tcb + 1 group init-tcb + 1 task-tcb grp +...+ 1 group
   * init-tcb + 1 task-tcb grp
   */
  m_segmmu_tcb_num = core_cnt;
  m_tot_tcb_cnt = 1 + m_sg_cnt * (m_task_per_sg + 1);

  m_backup_tcb.reset(new char[m_tot_tcb_cnt * sizeof(tcb_t)]);
}

void JobV3_1::setup_gm_sync_from_ddr(tcb_t &tcb) {
  GM_V3_1 *gm_v3_1 = reinterpret_cast<GM_V3_1 *>(m_gm);
  if (!m_mem->is_gm_enable())
    return;

  if (!gm_v3_1->gm_need_remap())
    return;

  uint32_t remap_mode = 0; /* time priority */
  uint32_t remap_size = (gm_v3_1->m_gm_buf_map_size >> 18) - 1;
  tcb.gm_ctrl =
      (remap_size & 0xFF) << 8 | (remap_mode & 0x1) << 1 | GM_CTRL_REMAP_EN;
  tcb.gm_addr_low = get_low_32(gm_v3_1->m_gm_map_base);
  tcb.gm_addr_high = get_high_32(gm_v3_1->m_gm_map_base);

  if (gm_v3_1->m_gm_buf_sync_size != 0)
    tcb.gm_sync = GM_SYNC_DDR_TO_GM;
}

#define SEGMMU_MEM_CTRL_EN (1 << 0)
#define SEGMMU_REMAP_EN (1 << 4)
#define SEGMMU_REMAP_SHARE_EN (1 << 5)
#define SEGMMU_IN_ASID_WR (1 << 0)
#define SEGMMU_IN_ASID_RD (1 << 1)
aipu_status_t JobV3_1::setup_segmmu(SubGraphTask &sg_task) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  SegMMUConfig *segmmu = nullptr;
  union segmmu_id {
    uint32_t id;
    struct {
      uint32_t segmmu_ctrl_idx : 8;
      uint32_t segmmu_idx : 8;
      uint32_t core_id_mask : 16;
    };
  };

  if (m_segmmu_num == 0)
    goto out;

  segmmu = (SegMMUConfig *)get_graph().m_bsegmmu.va;
  for (uint32_t i = 0; i < m_core_cnt; i++) {
    if (m_segmmu_num != 1)
      segmmu++;

    segmmu->SegMMU_ctl = SEGMMU_REMAP_SHARE_EN | SEGMMU_MEM_CTRL_EN;
    segmmu->SegMMU_remap = 0;

    m_segmmu_sec.push_back(*segmmu);
  }

  for (auto &iobuf : m_segmmus) {
    segmmu_id s_id = {.id = iobuf.id};

    if ((s_id.core_id_mask & ((1 << m_core_cnt) - 1)) == 0) {
      LOG(LOG_ERR,
          "Segmmu core idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, %d)",
          s_id.core_id_mask, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
      goto out;
    }

    for (uint32_t core_idx = 0; core_idx < m_core_cnt; core_idx++) {
      if (!(s_id.core_id_mask & (1 << core_idx)))
        continue;

      if (s_id.segmmu_idx >= 0 && s_id.segmmu_idx < 4) {
        if (s_id.segmmu_ctrl_idx >= 0 && s_id.segmmu_ctrl_idx <= 1) {
          uint32_t ctrl = m_segmmu_sec[core_idx]
                              .seg[s_id.segmmu_idx]
                              .control[s_id.segmmu_ctrl_idx];
          ctrl &= 0x3fff;
          ctrl |= (iobuf.pa & (~0x3fff));
          m_segmmu_sec[core_idx]
              .seg[s_id.segmmu_idx]
              .control[s_id.segmmu_ctrl_idx] = ctrl;
        } else {
          LOG(LOG_ERR,
              "Segmmu ctrl idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, "
              "%d)",
              core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
          goto out;
        }
      } else {
        LOG(LOG_ERR,
            "Segmmu seg idx invalid, (core_id, seg_idx, ctrl_idx): (%x, %d, "
            "%d)",
            core_idx, s_id.segmmu_idx, s_id.segmmu_ctrl_idx);
        goto out;
      }
    }
  }

out:
  return ret;
}

void JobV3_1::get_tcb_head_cnt(uint32_t sg_idx, uint32_t &head_cnt) {
  head_cnt = 2 + sg_idx;
}

aipu_status_t JobV3_1::init_group_id(uint32_t sg_cnt) {
  if (m_dev->get_start_group_id(m_sg_cnt, m_start_group_id) < 0) {
    return AIPU_STATUS_ERROR_ALLOC_GROUP_ID;
  }
  m_group_id_idx = m_start_group_id;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_1::free_job_buffers() {
  aipu_status_t ret = JobV3X::free_job_buffers();
  m_dev->put_start_group_id(m_start_group_id, m_sg_cnt);
  return ret;
}

aipu_status_t JobV3_1::config_tcb_smmu(tcb_t &tcb) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (m_segmmu_num > 0) {
    if (m_segmmu_num == 1) {
      SegMMUConfig &segmmu = m_segmmu_sec[0];

      tcb.segmmu_ctrl = segmmu.SegMMU_ctl;
      tcb.segmmu_remap_ctrl0 = segmmu.SegMMU_remap;
      tcb.segmmu_remap_ctrl1 = segmmu.SegMMU_remap;

      for (int j = 0; j < 4; j++) {
        tcb.segmmu_seg_ctrl[2 * j] = segmmu.seg[j].control[0];
        tcb.segmmu_seg_ctrl[2 * j + 1] = segmmu.seg[j].control[1];
      }
    }
  }

  return ret;
}

aipu_status_t JobV3_1::config_tcb_deps(tcb_t &tcb, uint32_t sg_id) {
  GraphV3X &graph = get_graph();

  switch (graph.get_subgraph(sg_id).precursor_cnt) {
  case SUBG_DEPEND_NONE:
    tcb.flag |= TCB_FLAG_DEP_TYPE_NONE;
    break;

  case 1 ... 4: {
    uint16_t dep_group_id = 0;

    tcb.flag |= TCB_FLAG_DEP_TYPE_GROUP;
    for (int32_t i = 0; i < graph.get_subgraph(sg_id).precursor_cnt; i++) {
      if (graph.get_subgraph(sg_id).precursors[i] > 0x7fff) {
        LOG(LOG_ERR, "Depend group id(%d) is invalid",
            graph.get_subgraph(sg_id).precursors[i]);
        return AIPU_STATUS_ERROR_INVALID_GBIN;
      }

      dep_group_id = graph.get_subgraph(sg_id).precursors[i] + m_start_group_id;
      dep_group_id &= 0x7FFF; // 15 bits group id field
      tcb.group_deps[i] = EN_GROUP_DEPEND | dep_group_id;
    }
  } break;

  case SUBG_DEPEND_PREALL:
    tcb.flag |= TCB_FLAG_DEP_TYPE_PRE_ALL;
    break;

  default:
    LOG(LOG_ERR, "subgraph %u, precursor_cnt=%d", sg_id,
        graph.get_subgraph(sg_id).precursor_cnt);
    return AIPU_STATUS_ERROR_INVALID_GBIN;
  }

  if (m_sg_cnt == get_graph().get_bss_cnt() &&
      TCB_FLAG_DEP_TYPE(tcb.flag) == TCB_FLAG_DEP_TYPE_PRE_ALL) {
    /* only depends last sg instead of all cores, if first sg, depend none */
    tcb.flag &= ~TCB_FLAG_DEP_TYPE_PRE_ALL;
    if (sg_id != 0) {
      tcb_t last_sg;
      Task &task = m_sg_job[sg_id - 1].tasks[0];
      m_mem->read(task.tcb.pa, &last_sg, sizeof(tcb_t));
      tcb.flag |= TCB_FLAG_DEP_TYPE_GROUP;
      tcb.group_deps[0] = EN_GROUP_DEPEND | last_sg.groupid;
    }
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_1::setup_task_tcb(uint32_t sg_id, uint32_t grid_id,
                                      uint32_t core_id, uint32_t task_id,
                                      bool) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  GraphV3X &graph = get_graph();
  Task &task = m_sg_job[sg_id].tasks[task_id];
  tcb_t tcb;

  memset(&tcb, 0, sizeof(tcb_t));
  tcb.interrupt_en = EN_INTERRUPT_TEC_SIGNAL | EN_INTERRUPT_TEC_EXCEPTION |
                     EN_INTERRUPT_TEC_FAULT;
  tcb.flag = TCB_FLAG_TASK_TYPE_TASK;

  if (task_id == (m_task_per_sg - 1))
    tcb.flag |= TCB_FLAG_END_TYPE_GROUP_END;

  /* v3_1 last tcb must have a grid end, although we don't need a cluster
   * interrupt */
  if ((sg_id == (m_sg_cnt - 1)) && (task_id == (m_task_per_sg - 1)))
    tcb.flag |= TCB_FLAG_END_TYPE_GRID_END;

  /**
   * depend_all: set init group tcb and first tcb of tcb group
   * depend_groups: set init group and all tasks of tcb group
   */
  ret = config_tcb_deps(tcb, sg_id);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;
  if (task_id != 0 && TCB_FLAG_DEP_TYPE(tcb.flag) == TCB_FLAG_DEP_TYPE_PRE_ALL)
    tcb.flag &= ~TCB_FLAG_DEP_TYPE_PRE_ALL;

  tcb.spc = get_low_32(graph.m_text->align_asid_pa +
                       graph.get_subgraph(sg_id).text.offset);
  tcb.groupid = (uint16_t)m_group_id_idx;
  tcb.gridid = (uint16_t)grid_id;
  tcb.taskid = (uint16_t)task_id;
  tcb.ica_warmup_len = graph.get_subgraph(sg_id).warmup_len;
  tcb.grid_dim_x = 1;
  tcb.grid_dim_y = 1;
  tcb.grid_dim_z = 1;
  tcb.group_dim_x = m_task_per_sg;
  tcb.group_dim_y = 1;
  tcb.group_dim_z = 1;
  tcb.group_id_x = 1;
  tcb.group_id_y = 0;
  tcb.group_id_z = 0;
  tcb.task_id_x = (uint16_t)task_id;
  tcb.task_id_y = 0;
  tcb.task_id_z = 0;
  tcb.tcbp = get_low_32(task.tcb.pa - m_tcbs->asid_base);
  tcb.sp = get_low_32(task.stack->align_asid_pa);
  tcb.pp = get_low_32(m_rodata->align_asid_pa +
                      graph.get_subgraph(sg_id).rodata.offset);
  tcb.dp = get_low_32(task.private_data->align_asid_pa);

  /* const rodata */
  if (graph.m_crodata != nullptr && graph.m_crodata->size > 0)
    tcb.cp = get_low_32(graph.m_crodata->align_asid_pa);

  /* update profile buffer offset according to subgraph index */
  if (m_profiler.size() > 0)
    tcb.pprofiler = get_low_32(m_profiler[0].align_asid_pa +
                               graph.get_subgraph(sg_id).profiler_buf_size);

  tcb.pcoredump = 0;
  if (m_coredump && m_coredump->is_initialized()) {
    auto &tec_buffer = m_coredump->get_tecs_buf();
    uint32_t buf_idx = core_id * m_task_per_sg + task_id;
    tcb.pcoredump = tec_buffer.at(buf_idx).first | 0x29A;
  }

  if (graph.get_subgraph(sg_id).printfifo_size > 0) {
    uint32_t pa =
        m_pprint->align_asid_pa + AIPU_PAGE_SIZE * sg_id + 1024 * task_id;
    tcb.pprint = get_low_32(pa);
    tcb.interrupt_en |= EN_INTERRUPT_TEC_SIGNAL;
  }

  if (get_graph().is_dynamic_shape() && m_dyn_shape->is_set_dyn_shape_true() &&
      m_dyn_shape->get_config_shape_sz() > 0)
    tcb.global_param = get_low_32(m_model_global_param->align_asid_pa);

  /* flush TCB to AIPU mem */
  m_mem->write(task.tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_1::setup_tcb_group(uint32_t sg_id, uint32_t grid_id,
                                       uint32_t core_id, bool) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  /* setup task TCBs */
  for (uint32_t t = 0; t < m_task_per_sg; t++) {
    ret = setup_task_tcb(sg_id, grid_id, core_id, t);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
  }

  /* increase group index for each group */
  m_group_id_idx++;

  return ret;
}

aipu_status_t JobV3_1::setup_tcb_chain() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  tcb_t tcb;
  uint32_t core_id = 0;

  /* Grid init TCB */
  memset(&tcb, 0, sizeof(tcb_t));
  tcb.flag = TCB_FLAG_TASK_TYPE_GRID_INIT | TCB_FLAG_L2D_FLUSH;
  tcb.group_num = m_sg_cnt;
  tcb.grid_gridid = m_grid_id;
  tcb.grid_groupid = m_group_id_idx;
  tcb.grid_intrrupt_en = EN_INTERRUPT_GRID_ALL;
  if (m_sg_cnt == get_graph().get_bss_cnt())
    tcb.grid_intrrupt_en &= ~EN_INTERRUPT_GRID_DONE;

  setup_gm_sync_from_ddr(tcb);
  m_mem->write(m_init_tcb.pa, (const char *)&tcb, sizeof(tcb_t));

  for (uint32_t i = 0; i < get_graph().get_subgraph_cnt(); i++) {
    /* Group init TCB */
    memset(&tcb, 0, sizeof(tcb_t));
    tcb.flag = TCB_FLAG_TASK_TYPE_GROUP_INIT | TCB_FLAG_GRID_INIT;
    tcb.group_gridid = m_grid_id;
    tcb.group_groupid = m_group_id_idx;
    if (m_sg_cnt == get_graph().get_bss_cnt())
      tcb.group_interrupt_en = EN_INTERRUPT_GROUP_DONE;

    // SegMMU
    // config_tcb_smmu(tcb);

    ret = config_tcb_deps(tcb, get_graph().get_subgraph(i).id);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    /**
     * ASID0: feature map buffer region
     * the whole graph share one copy of reuse buffer for feature map.
     */
    tcb.asids[0] = get_low_32(m_mem->get_asid_base(0) | ASID_RD | ASID_WR);
    tcb.asids[1] = get_high_32(m_mem->get_asid_base(0));

    /**
     * ASID1: weight buffer region
     * if LLM model contains multiple BSSs, each BSS will locate in private
     * ASID1 region. so here, set ASID1 base register from weight buffer's
     * asid_base(pa).
     */
    if (get_graph().get_weight_buffer_info().size() > 0) {
      uint32_t bss_idx = get_graph().get_subgraph(i).bss_idx;
      DEV_PA_64 asid1_base =
          get_graph().get_weight_buffer_info()[bss_idx].wb_asid_base;
      tcb.asids[2] = get_low_32(asid1_base | ASID_RD | ASID_WR);
      tcb.asids[3] = get_high_32(asid1_base);
    } else {
      tcb.asids[2] = get_low_32(m_mem->get_asid_base(1) | ASID_RD | ASID_WR);
      tcb.asids[3] = get_high_32(m_mem->get_asid_base(1));
    }

    for (uint32_t j = 2; j < 4; j++) {
      tcb.asids[2 * j] = 0;
      tcb.asids[2 * j + 1] = 0;
    }
    m_mem->write(m_init_tcb.pa + sizeof(tcb_t) +
                     (m_task_per_sg + 1) * i * sizeof(tcb_t),
                 (const char *)&tcb, sizeof(tcb_t));

    /* Task TCB */
    ret = setup_tcb_group(get_graph().get_subgraph(i).id, m_grid_id, core_id);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    if (++core_id >= m_core_cnt)
      core_id = 0;
  }

  /**
   * store aligned TEXT and RO base at tail of text buffer for debugger
   */
  m_mem->write(get_graph().m_text->pa + get_graph().m_btext.size,
               &get_graph().m_text->align_asid_pa, 4);
  m_mem->write(get_graph().m_text->pa + get_graph().m_btext.size + 4,
               &m_rodata->align_asid_pa, 4);

  m_status = AIPU_JOB_STATUS_INIT;

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t JobV3_1::dump_for_emulation() {
  constexpr uint32_t INIT_NUM = 3;
  DEV_PA_64 dump_pa = 0;
  uint32_t dump_size = 0;
  char dump_name[4096] = {0};
  int emu_input_cnt =
      INIT_NUM + m_inputs.size() + (m_descriptor != nullptr ? 1 : 0);
  int emu_output_cnt = m_outputs.size();
  int file_id = -1;
  tcb_t tcb = {0};
  bool default_output_prefix = true;
  std::string runtime_cfg = m_dump_dir + "/runtime.cfg";
  std::string metadata_txt = m_dump_dir + "/metadata.txt";
  std::map<uint32_t, std::string> gm_info = {
      {512 << 10, "512K"}, {1 << 20, "1M"},   {2 << 20, "2M"},
      {4 << 20, "4M"},     {8 << 20, "8M"},   {16 << 20, "16M"},
      {32 << 20, "32M"},   {64 << 20, "64M"},
  };

  if (m_dump_emu == false)
    return AIPU_STATUS_SUCCESS;

  FileWrapper ofs(runtime_cfg, std::ios_base::in | std::ios_base::out |
                                   std::ios_base::trunc);
  FileWrapper ofsmt(metadata_txt, std::ios_base::in | std::ios_base::out |
                                      std::ios_base::trunc);
  if (!ofs.is_open() || !ofsmt.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  ofs << "[COMMON]\n";

  /* runtime.cfg: config */
  ofs << "#configuration 3:X3_1304, 4:X3_1304MP2, 5:X3_1304MP4\n";
  if (m_dev->get_config_code() != nullptr)
    ofs << "CONFIG=" << m_dev->get_config_code() << "\n";

  /* runtime.cfg: enable_avx */
  ofs << "#if ENABLE_AVX is true then using the intel SIMD instructions to "
         "speedup.\n";
  if (m_cfg->enable_avx)
    ofs << "ENABLE_AVX=true\n";
  else
    ofs << "ENABLE_AVX=false\n";

  /* runtime.cfg: log file path */
  ofs << "#Where log output to store is.\n";
  ofs << "LOG_FILEPATH=" << m_cfg->log_file_path << "\n";

  /* runtime.cfg: log_level */
  ofs << "#which level is your selected: 0:ERROR, 1: WARN, 2: INFO, 3: DEBUG\n";
  ofs << "LOG_LEVEL=" << m_cfg->log_level << "\n";

  /* runtime.cfg: verbose */
  ofs << "#if LOG_VERBOSE is true then print log to console. otherwise no\n";
  if (m_cfg->verbose)
    ofs << "LOG_VERBOSE=true\n";
  else
    ofs << "LOG_VERBOSE=false\n";

  /* runtime.cfg: enable_calloc */
  ofs << "#if ENABLE_CALLOC is true the allocation memory is set to zero.\n";
  if (m_cfg->enable_calloc)
    ofs << "ENABLE_CALLOC=true\n";
  else
    ofs << "ENABLE_CALLOC=false\n";

  /* runtime.cfg: en_l2d */
  ofs << "#if EN_L2D is true the l2d cache is enabled.\n";
  if (m_cfg->en_l2d)
    ofs << "EN_L2D=true\n";
  else
    ofs << "EN_L2D=false\n";

  /* runtime.cfg: gm_size */
  ofs << "#GM_V3_1 support: 512KiB,1MiB,2MiB,4MiB,8MiB,16MiB,32MiB,64MiB.\n";
  if (gm_info.count(m_cfg->gm_size) == 1)
    ofs << "GM_SIZE=" << gm_info[m_cfg->gm_size] << "\n";

  if (m_cfg->plugin_name != nullptr) {
    ofs << "#PLUGIN_FILENAME\n";
    ofs << "PLUGIN_FILENAME=" << m_cfg->plugin_name << "\n";
  }

  /* runtime.cfg: en_eval */
  if (m_cfg->en_fast_perf) {
    ofs << "\n[PROFILE]\n";
    ofs << "EN_FAST_PERF=1\n";
    ofs << "FREQ_MHZ=" << m_cfg->freq_mhz << "\n";
    ofs << "DDR_LATENCY_RD=" << m_cfg->ddr_latency_rd << "\n";
    ofs << "DDR_LATENCY_WR=" << m_cfg->ddr_latency_wr << "\n";
    ofs << "DDR_BW_BITS=" << m_cfg->ddr_bw << "\n";
    ofs << "DDR_BW_RATIO=" << m_cfg->ddr_bw_ratio << "\n";

    if (m_cfg->perf_report != nullptr)
      ofs << "PERF_REPORT=" << m_cfg->perf_report << "\n";

    if (m_profiler.size() == 1) {
      ofs << "PROFILE_BUF_ADDR=0x" << std::hex << m_profiler[0].pa << "\n";
      ofs << "PROFILE_BUF_SIZE=0x" << std::hex << m_profiler[0].size << "\n";
    }

    if (m_cfg->json_filename != nullptr) {
      ofs << "#JSON_FILENAME\n";
      ofs << "JSON_FILENAME=" << m_cfg->json_filename << "\n";
    }
  }
  ofs << "\n";

  ofs.dump_to_string(m_dumpcfg_header);

  /* runtime.cfg: [INPUT] */
  for (uint32_t bss_id = 0;
       bss_id < get_graph().get_weight_buffer_info().size(); bss_id++) {
    if (get_graph().get_weight_buffer_info()[bss_id].wb_weight != nullptr &&
        get_graph().get_weight_buffer_info()[bss_id].wb_weight->size > 0) {
      emu_input_cnt += 1;
      if (get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const !=
              nullptr &&
          get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->size !=
              0)
        emu_input_cnt += 1;
    } else
      emu_input_cnt +=
          get_graph().get_weight_buffer_info()[bss_id].wb_weights.size();
  }

  ofs << "[INPUT]\n";
  ofs << "COUNT=" << emu_input_cnt << "\n";

  /* dump temp.text */
  dump_pa = get_graph().m_text->pa;
  dump_size = get_graph().m_btext.size;
  if (dump_size != 0) {
    snprintf(dump_name, 128, "%s/%s.text", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".text\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.weight */
  for (uint32_t bss_id = 0;
       bss_id < get_graph().get_weight_buffer_info().size(); bss_id++) {
    if (get_graph().get_weight_buffer_info()[bss_id].wb_weight != nullptr &&
        get_graph().get_weight_buffer_info()[bss_id].wb_weight->req_size > 0) {
      dump_pa = get_graph().get_weight_buffer_info()[bss_id].wb_weight->pa;
      dump_size =
          get_graph().get_weight_buffer_info()[bss_id].wb_weight->req_size;
      if (dump_size != 0) {
        snprintf(dump_name, 128, "%s/%s.weight", m_dump_dir.c_str(),
                 m_dump_prefix.c_str());
        m_mem->dump_file(dump_pa, dump_name, dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
            << ".weight\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        m_dumpcfg_input.push_back({dump_name, dump_pa});

        if (get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const !=
                nullptr &&
            get_graph()
                    .get_weight_buffer_info()[bss_id]
                    .wb_zerocpy_const->size > 0) {
          dump_pa =
              get_graph().get_weight_buffer_info()[bss_id].wb_zerocpy_const->pa;
          dump_size = get_graph()
                          .get_weight_buffer_info()[bss_id]
                          .wb_zerocpy_const->req_size;
          snprintf(dump_name, 128, "%s/%s.zerocpy_const", m_dump_dir.c_str(),
                   m_dump_prefix.c_str());
          m_mem->dump_file(dump_pa, dump_name, dump_size);

          ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
              << ".zerocpy_const\n";
          ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
          m_dumpcfg_input.push_back({dump_name, dump_pa});
        }
      }
    } else {
      for (uint32_t i = 0;
           i < get_graph().get_weight_buffer_info()[bss_id].wb_weights.size();
           i++) {
        dumpcfg_input_desc input_desc;
        std::string name;

        dump_pa =
            get_graph().get_weight_buffer_info()[bss_id].wb_weights[i]->pa;
        dump_size =
            get_graph().get_weight_buffer_info()[bss_id].wb_weights[i]->size;
        name = m_dump_dir + "/" + m_dump_prefix + ".weight" + std::to_string(i);
        m_mem->dump_file(dump_pa, name.c_str(), dump_size);

        ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix
            << ".weight\n";
        ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
        input_desc.file = name;
        input_desc.base = dump_pa;
        m_dumpcfg_input.push_back(input_desc);
      }
    }
  }

  /* dump temp.rodata */
  dump_pa = m_rodata->pa;
  dump_size = m_rodata->size;
  snprintf(dump_name, 128, "%s/%s.ro", m_dump_dir.c_str(),
           m_dump_prefix.c_str());
  m_mem->dump_file(dump_pa, dump_name, dump_size);
  ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".ro\n";
  ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
  m_dumpcfg_input.push_back({dump_name, dump_pa});

  /* dump temp.dcr */
  if (m_descriptor != nullptr && m_descriptor->size != 0) {
    dump_pa = m_descriptor->pa;
    dump_size = m_descriptor->size;
    snprintf(dump_name, 128, "%s/%s.dcr", m_dump_dir.c_str(),
             m_dump_prefix.c_str());
    m_mem->dump_file(dump_pa, dump_name, dump_size);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".dcr\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }

  /* dump temp.tcb */
  dump_pa = m_tcbs->pa;
  dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
  snprintf(dump_name, 128, "%s/%s.tcb", m_dump_dir.c_str(),
           m_dump_prefix.c_str());
  m_mem->dump_file(dump_pa, dump_name, dump_size);

  ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".tcb\n";
  ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";

  /* dump temp.input[n] */
  for (uint32_t i = 0; i < m_inputs.size(); i++) {
    if (m_inputs[i].dump_ignore_flag)
      continue;

    dump_pa = m_inputs[i].pa;
    dump_size = m_inputs[i].size;
    snprintf(dump_name, 128, "%s/%s.input%u", m_dump_dir.c_str(),
             m_dump_prefix.c_str(), i);

    if (m_inputs[i].dmabuf_fd < 0)
      m_mem->dump_file(dump_pa, dump_name, dump_size);
    else
      dump_share_buffer(m_inputs[i], dump_name, true);

    ofs << "FILE" << std::dec << ++file_id << "=" << m_dump_prefix << ".input"
        << i << "\n";
    ofs << "BASE" << file_id << "=0x" << std::hex << dump_pa << "\n";
    m_dumpcfg_input.push_back({dump_name, dump_pa});
  }
  ofs << "\n";

  ofs << "[HOST]\n";
  ofs << "TCBP_HI=0x" << std::hex << get_high_32(m_init_tcb.pa) << "\n";
  ofs << "TCBP_LO=0x" << get_low_32(m_init_tcb.pa) << "\n";
  ofs << "TCB_NUM=0x" << std::hex << m_tot_tcb_cnt << "\n";
  m_dumpcfg_host = {m_partition_id, get_high_32(m_init_tcb.pa),
                    get_low_32(m_init_tcb.pa)};
  ofs << "\n";

  /* runtime.cfg: [OUTPUT] */
  ofs << "[OUTPUT]\n";
  ofs << "COUNT=" << std::dec << emu_output_cnt << "\n";

  /* dump output.bin[n] */
  if (strncmp(m_dump_output_prefix.c_str(), "temp", 4))
    default_output_prefix = false;

  for (uint32_t i = 0; i < m_outputs.size(); i++) {
    if (m_outputs[i].dump_ignore_flag)
      continue;

    dump_pa = m_outputs[i].pa;
    dump_size = m_outputs[i].size;

    if (default_output_prefix) {
      ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << ".output"
          << i << "\n";
      snprintf(dump_name, 128, "%s/%s.output%u", m_dump_dir.c_str(),
               m_dump_prefix.c_str(), i);
      m_dumpcfg_output.push_back({dump_name, dump_pa, dump_size});
    } else {
      if (i == 0) {
        ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << "\n";
      } else {
        ofs << "FILE" << std::dec << i << "=" << m_dump_output_prefix << i
            << "\n";
      }
    }

    ofs << "BASE" << std::dec << i << "=0x" << std::hex << dump_pa << "\n";
    ofs << "SIZE" << std::dec << i << "=0x" << std::hex << dump_size << "\n";
  }

  /* close runtime.cfg */
  ofs.close();

  /* dump metadata.txt */
  ofsmt << "Total TCBs Count: " << std::dec << m_tot_tcb_cnt << "\n";

  /* Grid/Group init TCB and Task TCB */
  for (uint32_t i = 0; i < m_tot_tcb_cnt; i++) {
    m_mem->read(m_init_tcb.pa + sizeof(tcb_t) * i, &tcb, sizeof(tcb_t));

    if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_GRID_INIT) {
      ofsmt << "\n***GRID INIT TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "group_num: " << std::dec << tcb.group_num << "\n";
      ofsmt << "grid_intrrupt_en: 0x" << std::hex << tcb.grid_intrrupt_en
            << "\n";
      ofsmt << "grid_groupid: " << std::dec << tcb.grid_groupid << "\n";
      ofsmt << "grid_gridid: " << tcb.grid_gridid << "\n";
      ofsmt << "gm_ctrl: 0x" << std::hex << tcb.gm_ctrl << "\n";
      ofsmt << "gm_sync: 0x" << tcb.gm_sync << "\n";
      ofsmt << "gm_addr_low: 0x" << tcb.gm_addr_low << "\n";
      ofsmt << "gm_addr_high: 0x" << tcb.gm_addr_high << "\n";
    } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_GROUP_INIT) {
      ofsmt << "\n***GROUP INIT TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "segmmu_ctrl: 0x" << tcb.segmmu_ctrl << "\n";
      ofsmt << "segmmu_remap_ctrl0: 0x" << tcb.segmmu_remap_ctrl0 << "\n";
      ofsmt << "segmmu_remap_ctrl1: 0x" << tcb.segmmu_remap_ctrl1 << "\n";
      ofsmt << "group_interrupt_en: " << std::hex << tcb.group_interrupt_en
            << "\n";
      ofsmt << "group_groupid: " << std::dec << tcb.group_groupid << "\n";
      ofsmt << "group_gridid: " << tcb.group_gridid << "\n";

      for (int j = 0; j < 8; j++) {
        ofsmt << "segmmu_seg" << std::dec << j << "_ctrl0: 0x" << std::hex
              << tcb.segmmu_seg_ctrl[2 * j] << "\n";
        ofsmt << "segmmu_seg" << std::dec << j << "_ctrl1: 0x" << std::hex
              << tcb.segmmu_seg_ctrl[2 * j + 1] << "\n";
      }

      for (int j = 0; j < 4; j++) {
        ofsmt << "ASID" << std::dec << j << "_LO: 0x" << std::hex
              << tcb.asids[2 * j] << "\n";
        ofsmt << "ASID" << std::dec << j << "_HI: 0x" << std::hex
              << tcb.asids[2 * j + 1] << "\n";
      }
    } else if (TCB_FLAG_TASK_TYPE(tcb.flag) == TCB_FLAG_TASK_TYPE_TASK) {
      ofsmt << "\n***TASK TCB " << std::dec << i << " ***\n";

      ofsmt << "flag: 0x" << std::hex << tcb.flag << "\n";
      ofsmt << "start_pc: 0x" << std::hex << tcb.spc << "\n";
      ofsmt << "interrupt_en: 0x" << tcb.interrupt_en << "\n";

      ofsmt << "group_id: " << std::dec << tcb.groupid << "\n";
      ofsmt << "grid_id: " << tcb.gridid << "\n";
      ofsmt << "task_id: " << tcb.taskid << "\n";
      ofsmt << "warm_len: " << tcb.ica_warmup_len << "\n";

      ofsmt << "grid_dim_x: " << tcb.grid_dim_x << "\n";
      ofsmt << "grid_dim_y: " << tcb.grid_dim_y << "\n";
      ofsmt << "grid_dim_z: " << tcb.grid_dim_z << "\n";

      ofsmt << "group_dim_x: " << tcb.group_dim_x << "\n";
      ofsmt << "group_dim_y: " << tcb.group_dim_y << "\n";
      ofsmt << "group_dim_z: " << tcb.group_dim_z << "\n";

      ofsmt << "group_id_x: " << tcb.group_id_x << "\n";
      ofsmt << "group_id_y: " << tcb.group_id_y << "\n";
      ofsmt << "group_id_z: " << tcb.group_id_z << "\n";

      ofsmt << "task_id_x: " << tcb.task_id_x << "\n";
      ofsmt << "task_id_y: " << tcb.task_id_y << "\n";
      ofsmt << "task_id_z: " << tcb.task_id_z << "\n";

      ofsmt << "sp: 0x" << std::hex << tcb.sp << "\n";
      ofsmt << "pp: 0x" << tcb.pp << "\n";
      ofsmt << "dp: 0x" << tcb.dp << "\n";
      ofsmt << "cp: 0x" << tcb.cp << "\n";
      ofsmt << "pprint: 0x" << tcb.pprint << "\n";
      ofsmt << "pprofiler: 0x" << tcb.pprofiler << "\n";
      ofsmt << "dsize: 0x" << tcb.dsize << "\n";
      ofsmt << "tcbp: 0x" << tcb.tcbp << "\n";

      ofsmt << "group_deps[0]: " << tcb.group_deps[0] << "\n";
      ofsmt << "group_deps[1]: " << tcb.group_deps[1] << "\n";
      ofsmt << "group_deps[2]: " << tcb.group_deps[2] << "\n";
      ofsmt << "group_deps[3]: " << tcb.group_deps[3] << "\n";
    } else {
      LOG(LOG_ERR, "invalid TCB type\n");
    }
  }

  ofsmt << "\n***IO Tensors***\n";
  for (uint32_t i = 0; i < m_inputs.size(); i++) {
    dump_pa = m_inputs[i].pa;
    dump_size = m_inputs[i].size;

    ofsmt << "input" << std::dec << i << "_addr: 0x" << std::hex << dump_pa
          << "\n";
    ofsmt << "input" << std::dec << i << "_size: 0x" << std::hex << dump_size
          << "\n";
  }

  for (uint32_t i = 0; i < m_outputs.size(); i++) {
    dump_pa = m_outputs[i].pa;
    dump_size = m_outputs[i].size;

    ofsmt << "output" << std::dec << i << "_addr: 0x" << std::hex << dump_pa
          << "\n";
    ofsmt << "output" << std::dec << i << "_size: 0x" << std::hex << dump_size
          << "\n";
  }

  ofsmt.dump_to_string(m_dumpcfg_meta);
  /* close metadata.txt */
  ofsmt.close();
  return AIPU_STATUS_SUCCESS;
}

#if defined(SIMULATION)
void JobV3_1::dumpcfg_alljob() {
  JobV3_1 *job = nullptr;
  std::vector<uint32_t> cluster_id[4];
  uint32_t count = 0, cmdpool_mask = 0;
  MainContext *ctx = static_cast<MainContext *>(get_graph().m_ctx);
  GraphTable &graphs = ctx->get_graphtable();
  GraphV3X *graph = nullptr;
  std::ostringstream oss;
  static bool dump_done = false;
  static std::mutex mtex;

  {
    std::lock_guard<std::mutex> _lock(mtex);
    if (dump_done)
      return;
    dump_done = true;
  }

  if (!m_dump_emu)
    return;

  FileWrapper ofs("./runtime.cfg", std::ios::out);
  FileWrapper ofsmt("./metadata.txt", std::ios::out);
  if (!ofs.is_open() || !ofsmt.is_open())
    return;

  /* runtime.cfg: [COMMON] */
  ofs << m_dumpcfg_header << "\n";

  /* runtime.cfg: [INPUT] */
  count = 0;
  auto graph_iter = graphs.begin();
  for (auto g : graphs) {
    graph_iter++;
    graph = static_cast<GraphV3X *>(g.second);
    auto job_iter = graph->m_jobs.begin();
    for (auto item : graph->m_jobs) {
      job_iter++;
      job = static_cast<JobV3_1 *>(item.second);
      for (uint32_t i = 0; i < job->m_dumpcfg_input.size(); i++) {
        oss << "FILE" << std::dec << count << "="
            << job->m_dumpcfg_input.at(i).file << "\n";
        oss << "BASE" << count << "=0x" << std::hex
            << job->m_dumpcfg_input.at(i).base << "\n";
        count++;
      }
    }
  }
  ofs << "[INPUT]\n";
  ofs << "COUNT=" << count << "\n";
  ofs << oss.str();
  ofs << "\n";

  /* runtime.cfg: [HOST] */
  oss.str("");
  count = 0;
  cmdpool_mask = 1;
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3_1 *>(item.second);
      if (cmdpool_mask & (1 << job->m_bind_cmdpool_id)) {
        oss << "SET_PARTITION" << std::dec << count << "="
            << job->m_dumpcfg_host.part_id << "\n";
        oss << "TCBP_HI" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_host.hi_addr << "\n";
        oss << "TCBP_LO" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_host.lo_addr << "\n";
        count++;
        cmdpool_mask &= ~(1 << job->m_bind_cmdpool_id);
      }
    }
  }
  ofs << "[HOST]\n";
  ofs << "COUNT=" << count << "\n";
  ofs << oss.str();
  ofs << "\n";

  /* runtime.cfg: [ALLOCATE_PARTITION] */
  count = 0;
  ofs << "[ALLOCATE_PARTITION]\n";
  for (int i = 0; i < 4; i++) {
    m_dev->get_cluster_id(i, cluster_id[i]);
    count += cluster_id[i].size();
  }
  ofs << "COUNT=" << count << "\n";
  for (int part_id = 0; part_id < 4; part_id++) {
    for (auto cluster_id : cluster_id[part_id])
      ofs << "CLUSTER" << std::dec << cluster_id << "=" << part_id << "\n";
  }
  ofs << "\n";

  /* runtime.cfg: [OUTPUT] */
  oss.str("");
  count = 0;
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3_1 *>(item.second);
      for (uint32_t i = 0; i < job->m_dumpcfg_output.size(); i++) {
        oss << "FILE" << std::dec << count << "="
            << job->m_dumpcfg_output.at(i).file << "\n";
        oss << "BASE" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_output.at(i).base << "\n";
        oss << "SIZE" << std::dec << count << "=0x" << std::hex
            << job->m_dumpcfg_output.at(i).size << "\n";
        count++;
      }
    }
  }
  ofs << "[OUTPUT]\n";
  ofs << "COUNT=" << std::dec << count << "\n";
  ofs << oss.str();
  ofs << "\n";

  /* clost runtime.cfg */
  ofs.close();

  /* gen metadata.txt */
  for (auto g : graphs) {
    graph = static_cast<GraphV3X *>(g.second);
    for (auto item : graph->m_jobs) {
      job = static_cast<JobV3_1 *>(item.second);
      ofsmt << job->m_dumpcfg_meta;
      ofsmt << "\n";
    }
    ofsmt << "\n";
  }
  ofsmt.close();
}
#endif
} // namespace aipudrv