// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  graph_v3.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3 graph module implementation
 */

#include "graph_v3x.h"

#include <cstring>

#include "context.h"

#if (defined ZHOUYI_V3)
#include "zhouyi_v3x/zhouyi_v3/job_v3.h"
#elif (defined ZHOUYI_V3_1)
#include "zhouyi_v3x/zhouyi_v3_1/job_v3_1.h"
#endif

#include "parser_elf.h"
#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
GraphV3X::GraphV3X(void *ctx, GRAPH_ID id, DeviceBase *dev)
    : Graph(ctx, id, dev) {
  m_bss_vec.clear();
  m_subgraphs.clear();
  m_gmconfig.clear();
  m_hashtable.clear();
  m_bshared_weight.offsets.clear();
  m_parser = new ParserELF();
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

  if (is_exclusive_weight()) {
    LOG(LOG_DEFAULT, "\n--Weight:    cnt  0x%lx", m_bweight.weight.size());
    for (uint32_t i = 0; i < m_bweight.weight.size(); i++) {
      LOG(LOG_DEFAULT, "[Weight #%d]:    size 0x%lx", i,
          m_bweight.weight[i].size);
    }
  } else {
    LOG(LOG_DEFAULT, "\n--Whole Shared Weight size: 0x%lx",
        m_bshared_weight.weight.size);
  }

  LOG(LOG_DEFAULT, "\n--Subgraph:  cnt  0x%lx", m_subgraphs.size());
  for (uint32_t i = 0; i < m_subgraphs.size(); i++) {
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

aipu_status_t GraphV3X::extract_gm_info(int bss_id) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  GMConfig *gmconfig = nullptr;
  GM_info_desc gm_info_desc = {0};
  uint32_t gm_buffer_cnt = 0;

  if (m_gmconfig.size() == 0)
    goto out;

  gmconfig = (GMConfig *)&m_gmconfig[0];

  if (gmconfig->GM_buf_idx[0].fm_index != 0 ||
      gmconfig->GM_buf_idx[1].fm_index != 0) {
    LOG(LOG_WARN, "beyond fm scope: gm0_fm: %d, gm1_fm: %d\n",
        gmconfig->GM_buf_idx[0].fm_index, gmconfig->GM_buf_idx[1].fm_index);
    ret = AIPU_STATUS_ERROR_INVALID_GM;
    goto out;
  }

  gm_buffer_cnt = gmconfig->GM_control & 0xf;
  if (gm_buffer_cnt < 1 || gm_buffer_cnt > 2) {
    LOG(LOG_DEBUG, "no need config GM\n");
    goto out;
  } else if (gm_buffer_cnt == 2) {
    gm_buffer_cnt = 1;
  }

  for (uint32_t i = 0; i < gm_buffer_cnt; i++) {
    gm_info_desc.gm_buf_idx = gmconfig->GM_buf_idx[i];
    gm_info_desc.gm_buf_type =
        GM_SUB_BUF_TYPE_IGNORE; // change it according to the condition
    if (gmconfig->GM_buf_idx[i].buf_type == GM_BUF_TYPE_REUSE) {
      for (auto item : get_bss(bss_id).io.inputs) {
        if (gmconfig->GM_buf_idx[i].buf_index == item.ref_section_iter) {
          gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_INPUT;

          /* check whether one buffer is used as input and output */
          for (auto out_item : get_bss(bss_id).io.outputs) {
            if (gmconfig->GM_buf_idx[i].buf_index ==
                out_item.ref_section_iter) {
              gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_INOUT;
              break;
            }
          }

          goto match;
        }
      }

      for (auto item : get_bss(bss_id).io.outputs) {
        if (gmconfig->GM_buf_idx[i].buf_index == item.ref_section_iter) {
          gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_OUTPUT;
          goto match;
        }
      }

      /**
       * if the buffer isn't input or output, it must be an internal temp reuse
       * buffer
       */
      if (gmconfig->GM_buf_idx[i].buf_index <
          get_bss(bss_id).reuse_sections.size())
        gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_TEMP;
    } else if (gmconfig->GM_buf_idx[i].buf_type == GM_BUF_TYPE_WEIGHT) {
      if (gmconfig->GM_buf_idx[i].buf_index <
          get_bss(bss_id).static_sections.size()) {
        gm_info_desc.gm_buf_type = GM_SUB_BUF_TYPE_IGNORE;
      }
    } else {
      LOG(LOG_ERR, "GM buffer type: [%d] invalid\n",
          gmconfig->GM_buf_idx[i].buf_type);
      ret = AIPU_STATUS_ERROR_INVALID_GM;
      goto out;
    }

  match:
    if (gm_info_desc.gm_buf_type != GM_SUB_BUF_TYPE_IGNORE)
      m_gm_info[gmconfig->GM_buf_idx[i].buf_type]
               [gmconfig->GM_buf_idx[i].buf_index] = gm_info_desc;
  }

out:
  return ret;
}

aipu_status_t GraphV3X::create_job(
    JOB_ID *id, const aipu_global_config_simulation_t *glb_sim_cfg,
    aipu_global_config_hw_t *hw_cfg, aipu_create_job_cfg_t *job_config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
#if (defined ZHOUYI_V3)
  JobV3 *job = nullptr;
#elif (defined ZHOUYI_V3_1)
  JobV3_1 *job = nullptr;
#endif
  uint32_t part_cnt = 0;

  if (job_config == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  m_dev->get_partition_count(&part_cnt);
  if (job_config->partition_id > part_cnt) {
    LOG(LOG_ERR, "partition id: %d invalid, max partition id: %d\n",
        job_config->partition_id, part_cnt - 1);
    return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;
  }

  if (job_config->qos_level > AIPU_JOB_QOS_HIGH) {
    LOG(LOG_ERR, "QoS level: %d invalid, max level: %d", job_config->qos_level,
        AIPU_JOB_QOS_HIGH);
    return AIPU_STATUS_ERROR_INVALID_QOS;
  }

#if (defined ZHOUYI_V3_1)
  if (job_config->dbg_dispatch &&
      (job_config->qos_level == AIPU_JOB_QOS_HIGH)) {
    LOG(LOG_ERR, "Can't dispatch one high QoS job and bind to a core\n");
    return AIPU_STATUS_ERROR_INVALID_QOS;
  }
#endif

#if (defined ZHOUYI_V3)
  job = new JobV3((MainContext *)m_ctx, *this, m_dev, job_config);
#elif (defined ZHOUYI_V3_1)
  job = new JobV3_1((MainContext *)m_ctx, *this, m_dev, job_config);
#endif
  *id = add_job(job);
  ret = job->init(glb_sim_cfg, hw_cfg);
  return ret;
}

aipu_status_t GraphV3X::get_tensor_count(aipu_tensor_type_t type,
                                         uint32_t *cnt) {
  if (cnt == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  switch (type) {
  case AIPU_TENSOR_TYPE_INPUT:
    *cnt = (uint32_t)get_bss(0).io.inputs.size();
    break;
  case AIPU_TENSOR_TYPE_OUTPUT:
    *cnt = (uint32_t)get_bss(0).io.outputs.size();
    break;
  case AIPU_TENSOR_TYPE_PRINTF:
    *cnt = (uint32_t)get_bss(0).io.printf.size();
    break;
  case AIPU_TENSOR_TYPE_PROFILER:
    *cnt = (uint32_t)get_bss(0).io.profiler.size();
    break;
  case AIPU_TENSOR_TYPE_INTER_DUMP:
    *cnt = (uint32_t)get_bss(0).io.inter_dumps.size();
    break;
  case AIPU_TENSOR_TYPE_OUT_TENSOR_SHAPE:
    *cnt = (uint32_t)get_bss(0).io.outputs_shape.size();
    break;
  default:
    LOG(LOG_WARN, "no tensor with type: %d\n", type);
    return AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE;
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t GraphV3X::get_tensor_descriptor(aipu_tensor_type_t type,
                                              uint32_t tensor,
                                              aipu_tensor_desc_t *desc) {
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
    io = get_bss(0).io.inputs[tensor];
    break;
  case AIPU_TENSOR_TYPE_OUTPUT:
    io = get_bss(0).io.outputs[tensor];
    break;
  case AIPU_TENSOR_TYPE_PRINTF:
    io = get_bss(0).io.printf[tensor];
    break;
  case AIPU_TENSOR_TYPE_PROFILER:
    io = get_bss(0).io.profiler[tensor];
    break;
  case AIPU_TENSOR_TYPE_OUT_TENSOR_SHAPE:
    io = get_bss(0).io.outputs_shape[tensor];
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