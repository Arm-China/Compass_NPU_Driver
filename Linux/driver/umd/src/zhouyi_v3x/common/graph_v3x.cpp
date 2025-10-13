// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  graph_v3.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3 graph module implementation
 */

#include "graph_v3x.h"

#include <cstring>

#include "context.h"
#include "job_base.h"
#if (defined ZHOUYI_V3)
#include "zhouyi_v3x/zhouyi_v3/job_v3.h"
#elif (defined ZHOUYI_V3_2)
#include "zhouyi_v3x/zhouyi_v3_2/job_v3_2.h"
#endif
#include "kmd/tcb.h"
#include "parser_elf.h"
#include "utils/helper.h"
#include "utils/log.h"
#include "zhouyi_v3x/common/coredump.h"

namespace aipudrv {
GraphV3X::GraphV3X(void *ctx, GRAPH_ID id, DeviceBase *dev)
    : Graph(ctx, id, dev) {
  m_bss_vec.clear();
  m_subgraphs.clear();
  m_gmconfig.clear();
  m_hashtable.clear();
  m_parser = new ParserELF();

#ifndef SIMULATION
  const char *umd_coredump = getenv("UMD_COREDUMP_ENABLE");
  if (umd_coredump != nullptr &&
      (umd_coredump[0] == 'y' || umd_coredump[0] == 'Y'))
    m_coredump_en = true;
#endif
}

GraphV3X::~GraphV3X() {
  delete m_parser;
  m_parser = nullptr;
}

void GraphV3X::print_parse_info() {
  LOG(LOG_DEFAULT,
      "=====================Graph Parse Results====================");
  LOG(LOG_DEFAULT, "Target device: z%u-%u", m_hw_version, m_hw_config);
  LOG(LOG_DEFAULT, "--Text:      size 0x%lx", m_btext.size);
  LOG(LOG_DEFAULT, "--Rodata:    size 0x%lx", m_brodata.size);
  LOG(LOG_DEFAULT, "--DCR:       size 0x%lx", m_bdesc.size);
  LOG(LOG_DEFAULT, "--Data (CC): size 0x%lx", m_bdata.size);
  LOG(LOG_DEFAULT, "--Remap:     cnt  0x%lx", m_remap.size());

  /* TODO: provide weight information, includes: normal weight, extra weight,
   * shared weight */

  LOG(LOG_DEFAULT, "\n--Subgraph:  cnt  0x%x", get_subgraph_cnt());
  for (uint32_t i = 0; i < get_subgraph_cnt(); i++) {
    LOG(LOG_DEFAULT, "[subgraph #%d]\n", m_subgraphs[i].id);
    LOG(LOG_DEFAULT, "--Text:       offset 0x%lx, size 0x%lx",
        m_subgraphs[i].text.offset, m_subgraphs[i].text.size);
    LOG(LOG_DEFAULT, "--Rodata:     offset 0x%lx, size 0x%lx",
        m_subgraphs[i].rodata.offset, m_subgraphs[i].rodata.size);
    LOG(LOG_DEFAULT, "--DCR:        offset 0x%lx, size 0x%lx",
        m_subgraphs[i].dcr.offset, m_subgraphs[i].dcr.size);
    LOG(LOG_DEFAULT, "--printf:     size 0x%x", m_subgraphs[i].printfifo_size);
    LOG(LOG_DEFAULT, "--profiler:   size 0x%x",
        m_subgraphs[i].profiler_buf_size);
    LOG(LOG_DEFAULT, "--precursors: size 0x%lx",
        m_subgraphs[i].precursors.size());
  }

  LOG(LOG_DEFAULT, "\n--BSS:  cnt  0x%x", get_bss_cnt());
  for (uint32_t i = 0; i < get_bss_cnt(); i++) {
    LOG(LOG_DEFAULT, "[BSS: #%d]\n", get_bss(i).bss_id);
    LOG(LOG_DEFAULT, "--stack:      size 0x%x, align 0x%x",
        get_bss(i).stack_size, get_bss(i).stack_align_in_page);

    LOG(LOG_DEFAULT, "--static:     cnt 0x%lx",
        get_bss(i).static_sections.size());
    for (uint32_t j = 0; j < get_bss(i).static_sections.size(); j++) {
      LOG(LOG_DEFAULT, "----static section [%d]: size 0x%x, align 0x%x", j,
          get_bss(i).static_sections[j].size,
          get_bss(i).static_sections[j].align_in_page);
      for (uint32_t k = 0;
           k < get_bss(i).static_sections[j].sub_sections.size(); k++) {
        LOG(LOG_DEFAULT, "------subsection [%d]: offset 0x%x", k,
            get_bss(i).static_sections[j].sub_sections[k].offset_in_section);
      }
    }

    LOG(LOG_DEFAULT, "--reuse:      cnt 0x%lx",
        get_bss(i).reuse_sections.size());
    for (uint32_t j = 0; j < get_bss(i).reuse_sections.size(); j++) {
      LOG(LOG_DEFAULT, "----reuse section [%d]: size 0x%x, align 0x%x", j,
          get_bss(i).reuse_sections[j].size,
          get_bss(i).reuse_sections[j].align_in_page);
      for (uint32_t k = 0; k < get_bss(i).reuse_sections[j].sub_sections.size();
           k++) {
        LOG(LOG_DEFAULT, "------subsection [%d]: offset 0x%x", k,
            get_bss(i).reuse_sections[j].sub_sections[k].offset_in_section);
      }
    }
  }
  LOG(LOG_DEFAULT,
      "============================================================");
}

aipu_status_t GraphV3X::parse_gmconfig(int bss_id) {
  GMConfig *gmconfig = nullptr;
  GMConfigDesc gm_info_desc;
  uint32_t gm_buffer_cnt = 0;

  if (m_gmconfig.size() == 0)
    return AIPU_STATUS_SUCCESS;

  gmconfig = (GMConfig *)&m_gmconfig[0];
  if (gmconfig->GM_buf_idx[0].fm_index != 0 ||
      gmconfig->GM_buf_idx[1].fm_index != 0) {
    LOG(LOG_WARN, "beyond fm scope: gm0_fm: %d, gm1_fm: %d\n",
        gmconfig->GM_buf_idx[0].fm_index, gmconfig->GM_buf_idx[1].fm_index);
    return AIPU_STATUS_ERROR_INVALID_GM;
  }

  gm_buffer_cnt = gmconfig->GM_control & 0xf;
  if (gm_buffer_cnt == 0) {
    LOG(LOG_DEBUG, "no need config GM, gm buffer count is 0\n");
    return AIPU_STATUS_SUCCESS;
  } else if (gm_buffer_cnt >= 2) {
    LOG(LOG_WARN,
        "gm buffer count is %u, but runtime only handles 1 gm buffer\n",
        gm_buffer_cnt);
    gm_buffer_cnt = 1;
  }

  for (uint32_t i = 0; i < gm_buffer_cnt; i++) {
    gm_info_desc.gm_buf_idx = gmconfig->GM_buf_idx[i];
    gm_info_desc.sub_buf_type =
        GM_SUB_BUF_TYPE_IGNORE; // change it according to the condition
    if (gmconfig->GM_buf_idx[i].buf_type == GM_BUF_TYPE_REUSE) {
      for (auto item : get_bss(bss_id).io.inputs) {
        if (gmconfig->GM_buf_idx[i].buf_index == item.ref_section_iter) {
          gm_info_desc.sub_buf_type = GM_SUB_BUF_TYPE_INPUT;

          /* check whether one buffer is used as input and output */
          for (auto out_item : get_bss(bss_id).io.outputs) {
            if (gmconfig->GM_buf_idx[i].buf_index ==
                out_item.ref_section_iter) {
              gm_info_desc.sub_buf_type = GM_SUB_BUF_TYPE_INOUT;
              break;
            }
          }
          goto match;
        }
      }

      for (auto item : get_bss(bss_id).io.outputs) {
        if (gmconfig->GM_buf_idx[i].buf_index == item.ref_section_iter) {
          gm_info_desc.sub_buf_type = GM_SUB_BUF_TYPE_OUTPUT;
          goto match;
        }
      }

      /**
       * if the buffer isn't input or output, it must be an internal temp reuse
       * buffer
       */
      if (gmconfig->GM_buf_idx[i].buf_index <
          get_bss(bss_id).reuse_sections.size())
        gm_info_desc.sub_buf_type = GM_SUB_BUF_TYPE_TEMP;
    } else if (gmconfig->GM_buf_idx[i].buf_type == GM_BUF_TYPE_WEIGHT) {
      if (gmconfig->GM_buf_idx[i].buf_index <
          get_bss(bss_id).static_sections.size()) {
        gm_info_desc.sub_buf_type = GM_SUB_BUF_TYPE_IGNORE;
      }
    } else {
      LOG(LOG_ERR, "GM buffer type: [%d] invalid\n",
          gmconfig->GM_buf_idx[i].buf_type);
      return AIPU_STATUS_ERROR_INVALID_GM;
    }

  match:
    if (gm_info_desc.sub_buf_type != GM_SUB_BUF_TYPE_IGNORE)
      m_gm_config_desc[gmconfig->GM_buf_idx[i].buf_type]
                      [gmconfig->GM_buf_idx[i].buf_index] = gm_info_desc;
    else
      LOG(LOG_WARN, "Runtime current only supports put in/out/inout/temp "
                    "buffers into GM\n");
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GraphV3X::collect_fm_sections() {
  m_fmsec_info.reset();

  uint64_t offset = 0;
  m_fmsec_info.info[FMSection::Text] =
      FMSectionInfo::BufInfo{offset, m_btext.size + 16};
  offset += ALIGN_PAGE(m_btext.size + 16);

  m_fmsec_info.info[FMSection::Crodata] =
      FMSectionInfo::BufInfo{offset, m_bcrodata.size};
  offset += ALIGN_PAGE(m_bcrodata.size);

  if (!m_put_weight_gm) {
    uint64_t zcy_size = 0;
    for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); ++bss_id)
      zcy_size += ALIGN_PAGE(get_zerocpy_const_size(bss_id));
    m_fmsec_info.info[FMSection::ZcyConst] =
        FMSectionInfo::BufInfo{offset, zcy_size};
    offset += ALIGN_PAGE(zcy_size);
  }

  if (m_dynamic_shape) {
    m_fmsec_info.info[FMSection::ModelParam] =
        FMSectionInfo::BufInfo{offset, m_bglobalparam.size};
    offset += ALIGN_PAGE(m_bglobalparam.size);
  } else {
    m_fmsec_info.info[FMSection::ModelParam] = FMSectionInfo::BufInfo{0, 0};
  }

  m_fmsec_info.info[FMSection::Rodata] =
      FMSectionInfo::BufInfo{offset, m_brodata.size};
  offset += ALIGN_PAGE(m_brodata.size);

  if (!m_put_desc_gm) {
    m_fmsec_info.info[FMSection::Dcr] =
        FMSectionInfo::BufInfo{offset, m_bdesc.size};
    offset += ALIGN_PAGE(m_bdesc.size);
  }

  uint32_t tec_cnt = m_dev->tec_cnt_per_core(0);
  uint32_t tcb_cnt = 1 + get_subgraph_cnt() * (tec_cnt + 1);
  m_fmsec_info.info[FMSection::TcbChain] =
      FMSectionInfo::BufInfo{offset, tcb_cnt * sizeof(tcb_t)};
  offset += ALIGN_PAGE(tcb_cnt * sizeof(tcb_t));

  offset += k_tcb_reserved;

  m_max_ws_size = 0;
  uint32_t private_size = 0;
  for (uint32_t sg_idx = 0; sg_idx < get_subgraph_cnt(); sg_idx++) {
    const auto &sg = m_subgraphs[sg_idx];
    if (sg.precursor_cnt == SUBG_DEPEND_PREALL)
      private_size = 0;

    /* sg.private_buffers.size() should be 1, because it only has workspace */
    for (uint32_t pr_idx = 0; pr_idx < sg.private_buffers.size(); pr_idx++) {
      const GraphSectionDesc &section_desc = sg.private_buffers[pr_idx];
      private_size += ALIGN_PAGE(section_desc.size);
    }
    m_max_ws_size = std::max(m_max_ws_size, private_size);
  }

  if (!m_put_ws_gm) {
    m_fmsec_info.info[FMSection::TotalPriv] =
        FMSectionInfo::BufInfo{offset, m_max_ws_size};
    offset += ALIGN_PAGE(m_max_ws_size);
  }

  uint32_t total_reuse_size = 0;
  std::vector<uint32_t> gm_reuse_idx;
  for (uint32_t i = 0; i < m_bss_vec[0].reuse_sections.size(); ++i) {
    if (m_mem->is_gm_enable() &&
        m_gm_config_desc[GM_BUF_TYPE_REUSE].count(i) != 0) {
      gm_reuse_idx.push_back(i);
      continue;
    }
    const GraphSectionDesc &section_desc = m_bss_vec[0].reuse_sections[i];
    total_reuse_size += AIPU_ALIGN_BYTES(
        section_desc.size, section_desc.align_in_page * AIPU_PAGE_SIZE);
  }
  m_fmsec_info.info[FMSection::TotalReuse] =
      FMSectionInfo::BufInfo{offset, total_reuse_size};
  offset += ALIGN_PAGE(total_reuse_size);

  aipu_status_t ret = collect_gm_info();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_gmsec_info.remap_size > 0) {
    m_fmsec_info.info[FMSection::GM] =
        FMSectionInfo::BufInfo{offset, m_gmsec_info.remap_size};
    offset += ALIGN_PAGE(m_gmsec_info.remap_size);
  } else {
    m_fmsec_info.info[FMSection::GM] = FMSectionInfo::BufInfo{0, 0};
  }

  m_fmsec_info.info[FMSection::Printf] =
      FMSectionInfo::BufInfo{offset, get_subgraph_cnt() * AIPU_PAGE_SIZE};
  offset += ALIGN_PAGE(get_subgraph_cnt() * AIPU_PAGE_SIZE);

  if (m_coredump_en) {
    CoredumpAttr attr;
    m_dev->get_cluster_count(0, &attr.cluster_cnt);
    m_dev->get_core_count(0, 0, &attr.core_cnt);
    attr.tec_cnt = m_dev->tec_cnt_per_core(0);
    attr.lm_size = m_mem->get_lm_size();
    attr.sm_size = m_mem->get_sm_size();
    attr.gm_size = m_fmsec_info.info[FMSection::GM].size;
    uint32_t size = Coredump::get_coredump_buffer_size(attr);
    m_fmsec_info.info[FMSection::Coredump] =
        FMSectionInfo::BufInfo{offset, size};
    offset += ALIGN_PAGE(size);
  } else {
    m_fmsec_info.info[FMSection::Coredump] = FMSectionInfo::BufInfo{0, 0};
  }

  uint32_t reuse_sg_id = 0;
  bool dep_all_flag = false;
  std::vector<uint32_t> marked_sg;
  uint32_t total_task_size = 0;
  uint32_t total_dp_size = 0;
  for (uint32_t sg_id = 0; sg_id < get_subgraph_cnt(); ++sg_id) {
    if (m_subgraphs[sg_id].precursor_cnt == SUBG_DEPEND_PREALL) {
      reuse_sg_id = 0;
      dep_all_flag = true;
    }
    if (dep_all_flag && reuse_sg_id < marked_sg.size()) {
      reuse_sg_id++;
      continue;
    } else {
      dep_all_flag = false;
    }

    if (m_bss_vec[0].stack_align_in_page > 1)
      LOG(LOG_WARN, "stack align page: %u is bigger than 1",
          m_bss_vec[0].stack_align_in_page);

    uint32_t aligned_size =
        AIPU_ALIGN_BYTES(m_bss_vec[0].stack_size,
                         m_bss_vec[0].stack_align_in_page * AIPU_PAGE_SIZE);
    total_task_size += tec_cnt * aligned_size;
    total_dp_size += tec_cnt * ALIGN_PAGE(m_subgraphs[sg_id].private_data_size);
    marked_sg.push_back(sg_id);
  }
  m_fmsec_info.info[FMSection::Stack] =
      FMSectionInfo::BufInfo{offset, total_task_size};
  offset += ALIGN_PAGE(total_task_size);

  m_fmsec_info.info[FMSection::Dpdata] =
      FMSectionInfo::BufInfo{offset, total_dp_size};
  offset += ALIGN_PAGE(total_dp_size);

  uint32_t io_size = 0;
  for (uint32_t i = 0; i < m_bss_vec[0].io.inputs.size(); ++i) {
    io_size += ALIGN_PAGE(m_bss_vec[0].io.inputs[i].size);
  }
  for (uint32_t i = 0; i < m_bss_vec[0].io.outputs.size(); ++i) {
    io_size += ALIGN_PAGE(m_bss_vec[0].io.outputs[i].size);
  }
  m_fmsec_info.info[FMSection::ReservedIOVA] =
      FMSectionInfo::BufInfo{offset, io_size};

  m_fmsec_info.size = offset + get_alloc_pad_size();
  return AIPU_STATUS_SUCCESS;
}

/**
 * syncin data must put into gm firstly
 */
aipu_status_t GraphV3X::collect_gm_info() {
  m_gmsec_info.reset();

  if (!m_mem->is_gm_enable())
    return AIPU_STATUS_SUCCESS;

  uint32_t remap_size = 0;
  uint32_t sync_size = 0;

  if (m_put_weight_gm) {
    uint32_t const_size = 0;
    for (uint32_t i = 0; i < get_bss_cnt(); ++i) {
      const_size += ALIGN_PAGE(get_const_size(i));
      const_size += ALIGN_PAGE(get_zerocpy_const_size(i));
    }
    m_gmsec_info.info[GM_BUF_TYPE_WEIGHT] =
        GMSectionInfo::BufInfo{"weight", const_size, remap_size};
    remap_size += ALIGN_PAGE(const_size);
    sync_size += ALIGN_PAGE(const_size);
  }

  if (m_put_desc_gm) {
    uint32_t desc_size = m_bdesc.size;
    m_gmsec_info.info[GM_BUF_TYPE_DESCRIPTOR] =
        GMSectionInfo::BufInfo{"descriptor", desc_size, remap_size};
    remap_size += ALIGN_PAGE(desc_size);
    sync_size += ALIGN_PAGE(desc_size);
  }

  const auto &reuse_gm_desc = m_gm_config_desc[GMBufType::GM_BUF_TYPE_REUSE];
  if (reuse_gm_desc.size() != 0) {
    /* TODO: support multiple reuse sections to gm */
    uint32_t idx = reuse_gm_desc.begin()->first;
    GMConfigDesc config_desc = reuse_gm_desc.begin()->second;
    const auto &section = m_bss_vec[0].reuse_sections[idx];

    m_gmsec_info.info[GM_BUF_TYPE_REUSE] =
        GMSectionInfo::BufInfo{"reuse", section.size, remap_size};
    remap_size += ALIGN_PAGE(section.size);
    if (config_desc.sub_buf_type == GMSubBufType::GM_SUB_BUF_TYPE_INPUT)
      sync_size += ALIGN_PAGE(section.size);
  }

  if (m_put_ws_gm) {
    m_gmsec_info.info[GM_BUF_TYPE_WORKSPACE] =
        GMSectionInfo::BufInfo{"workspace", m_max_ws_size, remap_size};
    remap_size += ALIGN_PAGE(m_max_ws_size);
  }

  for (const auto &it : m_gmsec_info.info)
    LOG(LOG_DEBUG, "gm buffer name: %-10s size(unaligned): 0x%x",
        it.second.name.data(), it.second.size);

  if (remap_size > m_mem->get_gm_size() &&
      (m_put_weight_gm || m_put_desc_gm || m_put_ws_gm)) {
    LOG(LOG_ERR,
        "buf size 0x%x(256KB aligned) exceeds gm size 0x%x, please cancel "
        "'put_xx_gm'",
        remap_size, m_mem->get_gm_size());
    return AIPU_STATUS_ERROR_INVALID_GM;
  }

  uint32_t aligned_remap_size = aligned(remap_size, 1 << 18);
  LOG(LOG_DEBUG, "gm remap size 0x%x, aligned remap size 0x%x, sync size 0x%x",
      remap_size, aligned_remap_size, sync_size);
  m_gmsec_info.remap_size = aligned_remap_size;
  m_gmsec_info.sync_size = sync_size;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GraphV3X::create_job(
    JOB_ID *id, const aipu_global_config_simulation_t *glb_sim_cfg,
    aipu_global_config_hw_t *hw_cfg, aipu_create_job_cfg_t *job_config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t part_cnt = 0;

  if (job_config != nullptr) {
    m_dev->get_partition_count(&part_cnt);
    if (job_config->partition_id > part_cnt) {
      LOG(LOG_ERR, "partition id: %d invalid, max partition id: %d\n",
          job_config->partition_id, part_cnt - 1);
      return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;
    }

    if (job_config->qos_level > AIPU_JOB_QOS_HIGH) {
      LOG(LOG_ERR, "QoS level: %d invalid, max level: %d",
          job_config->qos_level, AIPU_JOB_QOS_HIGH);
      return AIPU_STATUS_ERROR_INVALID_QOS;
    }

#if (defined ZHOUYI_V3_2)
    if (job_config->dbg_dispatch &&
        (job_config->qos_level == AIPU_JOB_QOS_HIGH)) {
      LOG(LOG_ERR, "Can't dispatch one high QoS job and bind to a core\n");
      return AIPU_STATUS_ERROR_INVALID_QOS;
    }

    if (job_config->bind_enable && job_config->qos_level == AIPU_JOB_QOS_HIGH) {
      LOG(LOG_ERR, "Can't dispatch one high QoS job and bind to a core\n");
      return AIPU_STATUS_ERROR_INVALID_QOS;
    }
#endif
  }

  JobBase *job = nullptr;
#if (defined ZHOUYI_V3)
  job = new JobV3((MainContext *)m_ctx, *this, m_dev, job_config);
#elif (defined ZHOUYI_V3_2)
  job = new JobV3_2((MainContext *)m_ctx, *this, m_dev, job_config);
#endif
  *id = add_job(job);
  ret = job->init(glb_sim_cfg, hw_cfg);
  return ret;
}

aipu_status_t GraphV3X::get_tensor_count(aipu_tensor_type_t type,
                                         uint32_t *cnt) const {
  if (cnt == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  switch (type) {
  case AIPU_TENSOR_TYPE_INPUT:
    *cnt = (uint32_t)m_bss_vec[0].io.inputs.size();
    break;
  case AIPU_TENSOR_TYPE_OUTPUT:
    *cnt = (uint32_t)m_bss_vec[0].io.outputs.size();
    break;
  case AIPU_TENSOR_TYPE_PRINTF:
    *cnt = (uint32_t)m_bss_vec[0].io.printf.size();
    break;
  case AIPU_TENSOR_TYPE_PROFILER:
    *cnt = (uint32_t)m_bss_vec[0].io.profiler.size();
    break;
  case AIPU_TENSOR_TYPE_INTER_DUMP:
    *cnt = (uint32_t)m_bss_vec[0].io.inter_dumps.size();
    break;
  case AIPU_TENSOR_TYPE_OUT_TENSOR_SHAPE:
    *cnt = (uint32_t)m_bss_vec[0].io.outputs_shape.size();
    break;
  default:
    LOG(LOG_WARN, "no tensor with type: %d\n", type);
    return AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE;
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GraphV3X::get_tensor_descriptor(aipu_tensor_type_t type,
                                              uint32_t tensor,
                                              aipu_tensor_desc_t *desc) const {
  uint32_t cnt = 0;
  GraphIOTensorDesc io;

  get_tensor_count(type, &cnt);
  if (tensor >= cnt) {
    LOG(LOG_ERR, "tensor id: %d invalid, max id: %d", tensor, cnt - 1);
    return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
  }

  if (desc == nullptr) {
    LOG(LOG_ERR, "arg desc is NULL");
    return AIPU_STATUS_ERROR_NULL_PTR;
  }

  switch (type) {
  case AIPU_TENSOR_TYPE_INPUT:
    io = m_bss_vec[0].io.inputs[tensor];
    break;
  case AIPU_TENSOR_TYPE_OUTPUT:
    io = m_bss_vec[0].io.outputs[tensor];
    break;
  case AIPU_TENSOR_TYPE_PRINTF:
    io = m_bss_vec[0].io.printf[tensor];
    break;
  case AIPU_TENSOR_TYPE_PROFILER:
    io = m_bss_vec[0].io.profiler[tensor];
    break;
  case AIPU_TENSOR_TYPE_OUT_TENSOR_SHAPE:
    io = m_bss_vec[0].io.outputs_shape[tensor];
    break;
  default:
    LOG(LOG_WARN, "no tensor with type: %d\n", type);
    return AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
  }

  desc->id = tensor;
  desc->size = io.size;
  desc->scale = io.scale;
  desc->zero_point = io.zero_point;
  desc->data_type = io.data_type;

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GraphV3X::get_elf_note_size(const std::string &note_name,
                                          uint64_t &size) {
  ParserELF *parser = static_cast<ParserELF *>(m_parser);
  if (!parser) {
    return AIPU_STATUS_ERROR_NULL_PTR;
  }
  auto note = parser->get_bin_note(note_name);
  size = note.size;
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GraphV3X::get_elf_note(const std::string &note_name, char *data,
                                     uint64_t size) {
  ParserELF *parser = static_cast<ParserELF *>(m_parser);
  if (!parser) {
    return AIPU_STATUS_ERROR_NULL_PTR;
  }
  auto note = parser->get_bin_note(note_name);
  memcpy(data, note.va, note.size);
  return AIPU_STATUS_SUCCESS;
}
} // namespace aipudrv