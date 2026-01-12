// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  graph.cpp
 * @brief AIPU User Mode Driver (UMD) graph module implementation
 */

#include "graph.h"

#include <sys/mman.h>
#include <sys/stat.h>

#include <cstring>
#include <iomanip>

#include "context.h"
#include "device/device.h"
#include "parser_base.h"
#include "share_weight_mgr.h"
#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
Graph::Graph(void *ctx, GRAPH_ID id, DeviceBase *dev)
    : GraphBase(ctx, id, dev) {
  m_btext.init(nullptr, 0);
  m_bcrodata.init(nullptr, 0);
  m_brodata.init(nullptr, 0);
  m_bdesc.init(nullptr, 0);
  m_bdata.init(nullptr, 0);
  m_bweight.weight.clear();
  m_bweight.extra_weight_infos.clear();
}

Graph::~Graph() {}

aipu_status_t Graph::load(std::istream &gbin, uint32_t size, bool ver_check,
                          const aipu_load_graph_cfg_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  ret = load_config(config);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = m_parser->parse_graph(gbin, size, *this);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return load_common(ver_check);
}

aipu_status_t Graph::load(const char *file, bool ver_check,
                          const aipu_load_graph_cfg_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  ret = load_config(config);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = m_parser->parse_graph(file, *this);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return load_common(ver_check);
}

aipu_status_t Graph::load_config(const aipu_load_graph_cfg_t *config) {
  if (config != nullptr) {
    if (config->extra_weight_path != nullptr) {
      if (access(config->extra_weight_path, F_OK) == 0) {
        m_bweight.extra_weight_path = config->extra_weight_path;
      } else {
        LOG(LOG_ERR, "Extra weight path: %s [non exist]",
            config->extra_weight_path);
        return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
      }
    }

    m_swt_mem_region = config->wt_mem_region;
    if (config->wt_idxes) {
      for (int i = 0; i < config->wt_idxes_cnt; i++)
        m_swt_idxes.insert(config->wt_idxes[i]);
    }

    m_put_weight_gm = config->put_weight_gm;
    m_put_desc_gm = config->put_desc_gm;
    m_put_ws_gm = config->put_ws_gm;
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t Graph::load_common(bool ver_check) {
  m_bin_target = version_to_target(0, m_isa, m_hw_config, m_hw_revision);
  if (m_bin_target.empty())
    return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

  LOG(LOG_INFO, "aipu binary target %s", m_bin_target.c_str());

  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t dev_isa = bin_to_dev_isa();

#ifdef SIMULATION
  MainContext *ctx = reinterpret_cast<MainContext *>(m_ctx);
  ret = get_device(m_bin_target, dev_isa, &m_dev, ctx->sim_cfg());
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ctx->set_dev(m_dev);
#endif

  if (ver_check &&
      !m_dev->has_target(m_arch, dev_isa, m_hw_config, m_hw_revision)) {
    LOG(LOG_ERR, "aipu.bin target %s, which cannot match with npu device",
        m_bin_target.c_str());
    return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;
  }

  if (m_isa <= ISAv5 && (m_put_weight_gm || m_put_desc_gm || m_put_ws_gm)) {
    LOG(LOG_ERR, "only >= v3_2 version supports put_xx_gm feature, but now %u",
        m_isa);
    return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
  }

  m_do_vcheck = ver_check;
  m_mem = m_dev->get_mem();
  m_mem->dump_tracking_log_start();

  if (m_isa > ISAv5)
    m_dynamic_asid0 = true;

  if (!m_dynamic_asid0) {
    if (m_btext.size != 0) {
      /**
       * expand 16 bytes more to export RO base for debugger.
       * there is no effect for text self.
       */
      ret = m_mem->malloc(m_btext.size + 16, 0, &m_text, "text");
      if (ret != AIPU_STATUS_SUCCESS)
        return ret;
      m_mem->write(m_text->pa, m_btext.va, m_btext.size);
    }

    if (m_bcrodata.size != 0) {
      ret = m_mem->malloc(m_bcrodata.size, 0, &m_crodata, "crodata");
      if (ret != AIPU_STATUS_SUCCESS)
        return ret;
      m_mem->write(m_crodata->pa, m_bcrodata.va, m_bcrodata.size);
    }
  }

  return collect_fm_sections();
}

std::string Graph::version_to_target(uint32_t arch, uint32_t hw_version,
                                     uint32_t hw_config, uint32_t hw_revision) {
  const static std::set<std::string> targets = {
      "Z1_0904", "Z1_1002",  "Z1_0701",  "Z1_0701_P", "Z2_1104",  "Z2_0901",
      "Z2_1002", "Z2_1004",  "Z3_1204",  "Z3_1104",   "Z3_1002",  "Z3_0901",
      "X1_1204", "X2_1204",  "Z5_0901",  "X3_1304",   "X3_1302",  "X3_1202",
      "X3_1204", "X3P_1304", "X3P_1302", "X3P_1202",  "X3P_1204", "X3S_1304",
  };

  std::ostringstream str;
  if (hw_version > ISAv3) {
    str << "X";

    hw_version -= ISAv3;
    str << hw_version;

    if (hw_revision == 1)
      str << "P";
    else if (hw_revision == 2)
      str << "S";
  } else {
    str << "Z" << hw_version;
  }

  str << "_" << std::setw(4) << std::setfill('0') << hw_config;
  if (hw_revision == 1 && hw_version < ISAv3)
    str << "_P";
  std::string target = str.str();
  if (targets.count(target) == 0) {
    LOG(LOG_ERR, "unsupport target %s", target.c_str());
    return "";
  }

  std::string detailed_target = get_target();
  if (detailed_target.find(target) != std::string::npos)
    target = detailed_target;
  return target;
}

/**
 * aipu.bin file + extra_weight.bin
 * aipu.bin buffer + extra_weight.bin
 */
BLOCKS_TABLE Graph::slice_weight(uint32_t bss_id) {
  BLOCKS_TABLE blk_table;
  for (auto &blk : blk_table)
    blk.clear();

  if (m_mapped_gfile.empty() && bss_id == 0)
    return blk_table;

  std::vector<GraphSectionDesc> &static_sections =
      get_static_section_ref(bss_id);
  uint32_t start = 0;
  bool new_blk = true;
  for (uint32_t i = 0; i < static_sections.size(); ++i) {
    if (new_blk) {
      auto blk = std::make_shared<Block>();
      if (bss_id > 0 && !m_bweight.extra_weight_infos.empty()) {
        blk->file = m_bweight.extra_weight_infos[bss_id - 1].file;
        blk->offset_to_file = static_sections[i].src_offset;
      } else {
        blk->file = m_mapped_gfile;
        blk->offset_to_file =
            m_bweight.weight[0].offset + static_sections[i].src_offset;
      }
      blk->offset_to_weight = static_sections[i].src_offset;
      blk_table[0][i] = blk;
      start = static_sections[i].src_offset;
      new_blk = false;
    }

    if (!new_blk) {
      if ((i + 1 == static_sections.size()) ||
          ((i + 1 < static_sections.size()) &&
           (static_sections[i + 1].src_offset - start >= MAX_BLOCK_SIZE))) {
        auto &start_blk = blk_table[0].rbegin()->second;
        start_blk->size =
            static_sections[i].src_offset - start + static_sections[i].size;
        blk_table[1][i] = start_blk;
        new_blk = true;
      }
    }
  }
  return blk_table;
}

/**
 * @brief zcy nullptr placeholder for >=v3_2
 *        according provided base pa of weight and zcy, generates each static
 * section desc, and write data into desc
 */
aipu_status_t Graph::setup_weight_buffer(std::vector<WeightBufferInfo> &weights,
                                         bool setup_zcy) {
  if (weights.size() != get_bss_cnt()) {
    LOG(LOG_ERR, "weight buffer vector size: %zu is not equal to bss cnt: %u",
        weights.size(), get_bss_cnt());
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  }

  if (!has_weight())
    return AIPU_STATUS_SUCCESS;

  for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++) {
    weights[bss_id].wb_weights.clear();

    auto blk_table = slice_weight(bss_id);

    uint32_t blk_offset = 0;
    const char *va = nullptr;
    if (bss_id < m_bweight.weight.size())
      va = m_bweight.weight[bss_id].va;

    std::vector<GraphSectionDesc> &static_sections =
        get_static_section_ref(bss_id);
    for (uint32_t i = 0; i < static_sections.size(); i++) {
      if (blk_table[0].count(i) != 0) {
        umd_mmap_file_helper(blk_table[0][i]->file.c_str(), (void **)&va,
                             blk_table[0][i]->size,
                             blk_table[0][i]->offset_to_file, true);
        blk_offset = blk_table[0][i]->offset_to_weight;
      }

      DEV_PA_64 pa = 0;
      BufferDesc *buf = nullptr;

      /* DONT use with dynamic asid0/1 */
      if (m_swt_idxes.count(i) == 0) {
        if (static_sections[i].type == SECTION_TYPE_CONSTANT) {
          buf = new BufferDesc;
          buf->reset();

          pa = weights[bss_id].wb_weight->pa + static_sections[i].dst_offset;
          buf->init(weights[bss_id].wb_weight->asid_base, pa,
                    static_sections[i].size, static_sections[i].size, 0,
                    (m_mem->get_asid1() << 8) | AIPU_MEM_REGION_DEFAULT);
          if (m_mem->write(pa, va + static_sections[i].src_offset - blk_offset,
                           static_sections[i].size) != static_sections[i].size)
            return AIPU_STATUS_ERROR_INVALID_SIZE;
        } else if (static_sections[i].type == SECTION_TYPE_ZEROCPY_CONSTANT &&
                   setup_zcy) {
          buf = new BufferDesc;
          buf->reset();

          pa = weights[bss_id].wb_zerocpy_const->pa +
               static_sections[i].dst_offset;
          buf->init(weights[bss_id].wb_zerocpy_const->asid_base, pa,
                    static_sections[i].size, static_sections[i].size);
          if (m_mem->write(pa, va + static_sections[i].src_offset - blk_offset,
                           static_sections[i].size) != static_sections[i].size)
            return AIPU_STATUS_ERROR_INVALID_SIZE;
        }
      } else {
        std::string name = std::string("weight_") + std::to_string(i);
        aipu_status_t ret =
            m_mem->malloc(static_sections[i].size + get_alloc_pad_size(),
                          static_sections[i].align_in_page, &buf, name.c_str(),
                          (m_mem->get_asid1() << 8) | m_swt_mem_region);
        if (ret != AIPU_STATUS_SUCCESS) {
          LOG(LOG_ERR, "malloc from memory type %u fail", m_swt_mem_region);
          return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
        }

        DEV_PA_64 base_pa = static_sections[i].type == SECTION_TYPE_CONSTANT
                                ? weights[bss_id].wb_weight->pa
                                : weights[bss_id].wb_zerocpy_const->pa;
        pa = buf->pa;
        if (buf->pa < base_pa || buf->pa + buf->size - base_pa > 0xE0000000) {
          LOG(LOG_ERR,
              "please make sure sram address is in asid 3G/3.5G range, now "
              "sram pa 0x%lx, base pa 0x%lx",
              buf->pa, base_pa);
          return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;
        }
        if (m_mem->write(pa, va + static_sections[i].src_offset - blk_offset,
                         static_sections[i].size) != static_sections[i].size)
          return AIPU_STATUS_ERROR_INVALID_SIZE;

        DEV_PA_64 asid_base = static_sections[i].type == SECTION_TYPE_CONSTANT
                                  ? weights[bss_id].wb_weight->asid_base
                                  : weights[bss_id].wb_zerocpy_const->asid_base;
        buf->asid_base = asid_base;
        buf->align_asid_pa = buf->pa - buf->asid_base;
      }

      weights[bss_id].wb_weights.push_back(buf);
      if (bss_id != 0)
        weights[0].wb_weights.push_back(buf);

      if (blk_table[1].count(i) != 0) {
        madvise((void *)va, blk_table[1][i]->size, MADV_DONTNEED);
        munmap((void *)va, blk_table[1][i]->size);
        va = nullptr;
      }
    }

    weights[bss_id].wb_asid_base = weights[bss_id].wb_weight->asid_base;
  }

  return AIPU_STATUS_SUCCESS;
}

/**
 * @brief update zcy desc for each dynamic asid0 job
 *        each dynamic asid0 job should maintain own 'WeightBufferInfo' for
 * different zcy base
 */
aipu_status_t Graph::setup_zcy_buffer(std::vector<WeightBufferInfo> &weights) {
  if (m_is_shared_weight)
    return m_sw_mgr->setup_zcy_buffer(weights, m_id);

  if (!has_weight())
    return AIPU_STATUS_SUCCESS;

  for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++) {
    if (get_zerocpy_const_size(bss_id) == 0)
      continue;

    auto blk_table = slice_weight(bss_id);

    uint32_t blk_offset = 0;
    const char *va = nullptr;

    if (bss_id < m_bweight.weight.size())
      va = m_bweight.weight[bss_id].va;

    BufferDesc *zcy_desc = weights[bss_id].wb_zerocpy_const;
    std::vector<GraphSectionDesc> &static_sections =
        get_static_section_ref(bss_id);
    for (uint32_t i = 0; i < static_sections.size(); i++) {
      if (blk_table[0].count(i) != 0) {
        umd_mmap_file_helper(blk_table[0][i]->file.c_str(), (void **)&va,
                             blk_table[0][i]->size,
                             blk_table[0][i]->offset_to_file, true);
        blk_offset = blk_table[0][i]->offset_to_weight;
      }

      if (static_sections[i].type == SECTION_TYPE_ZEROCPY_CONSTANT) {
        BufferDesc *buf = new BufferDesc;
        buf->reset();

        DEV_PA_64 sec_pa = zcy_desc->pa + static_sections[i].dst_offset;
        buf->init(zcy_desc->asid_base, sec_pa, static_sections[i].size,
                  static_sections[i].size);

        /* bss0 includes other bss */
        uint32_t offset = i;
        weights[bss_id].wb_weights[offset] = buf;
        for (uint32_t bss = 0; bss < bss_id; ++bss)
          offset += get_static_section_ref(bss).size();
        weights[0].wb_weights[offset] = buf;

        const char *sec_va = va + static_sections[i].src_offset - blk_offset;
        int64_t size = m_mem->write(sec_pa, sec_va, static_sections[i].size);
        if (size != static_cast<int64_t>(static_sections[i].size))
          return AIPU_STATUS_ERROR_INVALID_SIZE;
      }

      if (blk_table[1].count(i) != 0) {
        madvise((void *)va, blk_table[1][i]->size, MADV_DONTNEED);
        munmap((void *)va, blk_table[1][i]->size);
        va = nullptr;
      }
    }
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t Graph::alloc_weight_buffer() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (!has_weight() || (m_put_weight_gm && m_mem->is_gm_enable()))
    return AIPU_STATUS_SUCCESS;

  /* 1.alloc const buffer */
  std::string buf_name;
  uint32_t align_page = get_asid_align_page();
  m_weight.resize(get_bss_cnt());
  for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); ++bss_id) {
    buf_name = std::string("weight_") + std::to_string(bss_id);
    ret =
        m_mem->malloc(get_const_size(bss_id) + get_alloc_pad_size(), align_page,
                      &m_weight[bss_id].wb_weight, buf_name.c_str(),
                      (m_mem->get_asid1() << 8) | AIPU_MEM_REGION_DEFAULT);
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_ERR, "alloc weight buffer [fail]");
      return ret;
    }

    if (!m_dynamic_asid0) {
      buf_name = std::string("zcy_const_") + std::to_string(bss_id);
      if (get_zerocpy_const_size(bss_id) > 0) {
        ret = m_mem->malloc(
            get_zerocpy_const_size(bss_id) + get_alloc_pad_size(), 1,
            &m_weight[bss_id].wb_zerocpy_const, buf_name.c_str(),
            AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT);
        if (ret != AIPU_STATUS_SUCCESS) {
          LOG(LOG_ERR, "alloc zerocpy_const buffer [fail]");
          return ret;
        }
      }
    }
  }

  /* 2.write data to aipu buffer */
  return setup_weight_buffer(m_weight, !m_dynamic_asid0 /* setup_zcy */);
}

aipu_status_t Graph::unload() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  ret = destroy_jobs();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_text && m_text->size != 0)
    m_mem->free(&m_text);

  if (m_crodata && m_crodata->size != 0)
    m_mem->free(&m_crodata);

  if (m_weight.size() > 0) {
    /* m_weight[0] includes all BufferDesc, which may come from gathered and
     * scattered */
    std::vector<BufferDesc *> &wb_weights = m_weight[0].wb_weights;
    for (uint32_t i = 0; i < wb_weights.size(); ++i) {
      if (wb_weights[i] != nullptr) {
        if (m_weight[0].wb_weight == nullptr ||
            m_swt_idxes.count(i) != 0) /* scattered or specified */
          m_mem->free(&wb_weights[i]);
        else
          m_mem->free_bufferdesc(&wb_weights[i]);
      }
    }
    wb_weights.clear();

    for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++) {
      if (bss_id > m_weight.size())
        break;

      if (m_is_shared_weight || (m_put_weight_gm && m_mem->is_gm_enable()))
        break;

      if (m_weight[bss_id].wb_zerocpy_const != nullptr &&
          m_weight[bss_id].wb_zerocpy_const->size != 0)
        m_mem->free(&m_weight[bss_id].wb_zerocpy_const);

      if (m_weight[bss_id].wb_weight != nullptr &&
          m_weight[bss_id].wb_weight->size != 0)
        m_mem->free(&m_weight[bss_id].wb_weight);
    }
  }

  // m_mem->dump_tracking_log_end();
  return ret;
}

int32_t Graph::get_dynamic_shape_dim_num(uint32_t idx,
                                         bool max_shape_dim) const {
  if (!is_dynamic_shape())
    return 0;

  if (idx >= 0 && idx < m_input_shape_constraint.size()) {
    if (max_shape_dim)
      return m_input_shape_constraint.at(idx)[1].size();
    else
      return m_input_shape_constraint.at(idx)[0].size();
  }

  return 0;
}

bool Graph::get_dynamic_shape_data(uint32_t idx, bool max_shape_dim,
                                   uint32_t *data) const {
  if (!is_dynamic_shape())
    return false;

  if (data == nullptr) {
    LOG(LOG_ERR, "data ptr is NULL");
    return false;
  }

  if (idx >= 0 && idx < m_input_shape_constraint.size()) {
    if (max_shape_dim) {
      for (uint32_t i = 0; i < m_input_shape_constraint.at(idx)[1].size(); i++)
        *(data + i) = m_input_shape_constraint.at(idx)[1][i];
    } else {
      for (uint32_t i = 0; i < m_input_shape_constraint.at(idx)[0].size(); i++)
        *(data + i) = m_input_shape_constraint.at(idx)[0][i];
    }
  }

  return true;
}

aipu_status_t Graph::set_graph_extra_weight(const BinSection &extra_weight) {
  char *start = (char *)extra_weight.va;
  size_t extra_weight_bin_cnt = *(size_t *)start;

  start += sizeof(size_t);
  for (size_t i = 0; i < extra_weight_bin_cnt; i++) {
    WeightSection::ExtraWeightInfo extra_weight_info;
    size_t bin_name_len = *(size_t *)start;

    start += sizeof(size_t);
    char *plus_mark = std::strstr(start, "+");
    std::string name = std::string(start, plus_mark - start);
    extra_weight_info.hash =
        std::string(plus_mark + 1, bin_name_len - (plus_mark + 1 - start));
    start += bin_name_len;

    extra_weight_info.file = m_bweight.extra_weight_path + "/" + name;

    struct stat info;
    if (stat(extra_weight_info.file.c_str(), &info) != 0) {
      LOG(LOG_ERR, "cannot find extra weight file: %s",
          extra_weight_info.file.c_str());
      return AIPU_STATUS_ERROR_READ_FILE_FAIL;
    }

    bool match = check_hash(extra_weight_info.file, extra_weight_info.hash);
    if (!match) {
      LOG(LOG_ERR, "file:%s cannot match its hash code",
          extra_weight_info.file.c_str());
      return AIPU_STATUS_ERROR_UNKNOWN_BIN;
    }

    m_bweight.extra_weight_infos.push_back(extra_weight_info);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t Graph::set_constant_hashtable(const BinSection &table) {
  const char *p_data = (const char *)table.va;

  uint32_t hashtable_num = *(uint32_t *)p_data;
  p_data += sizeof(uint32_t);

  for (uint32_t i = 0; i < hashtable_num; ++i) {
    std::vector<ConstantHashItem> items;

    uint32_t item_num = *(uint32_t *)p_data;
    p_data += sizeof(uint32_t);
    for (uint32_t j = 0; j < item_num; ++j) {
      ConstantHashItem item;
      item.offset = *(uint32_t *)p_data;
      item.size = *(uint32_t *)(p_data + sizeof(uint32_t));
      for (uint32_t k = 0; k < item.hashcode.size(); ++k)
        item.hashcode[k] = *(uint8_t *)(p_data + 2 * sizeof(uint32_t) + k);

      items.push_back(item);
      p_data = p_data + 2 * sizeof(uint32_t) + item.hashcode.size();
    }
    m_hashtable.push_back(items);
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t Graph::set_static_section_param(
    uint32_t bss_id, const BSSStaticSectionDesc &bss_sec,
    GraphSectionDesc &section, uint32_t &cst_addr, uint32_t &zcy_cst_addr) {
  if (section.type == SECTION_TYPE_ZEROCPY_CONSTANT) {
    section.dst_offset = aligned(zcy_cst_addr, bss_sec.align_bytes);
    zcy_cst_addr = section.dst_offset + section.size;
  } else {
    section.dst_offset = aligned(cst_addr, bss_sec.align_bytes);
    cst_addr = section.dst_offset + section.size;
  }

  if (m_is_shared_weight) {
    if (bss_id >= m_hashtable.size()) {
      LOG(LOG_ERR, "bss number %u is not equal to hashtable %zu", bss_id,
          m_hashtable.size());
      return AIPU_STATUS_ERROR_INVALID_GBIN;
    }

    auto iter = m_hashtable[bss_id].begin();
    for (; iter < m_hashtable[bss_id].end(); ++iter) {
      /* BssDescriptor.size is aligned size, so it cannot use to measure */
      if (iter->offset ==
          bss_sec.src_offset /* && iter->size == bss_sec.size */)
        break;
    }
    if (iter == m_hashtable[bss_id].end()) {
      LOG(LOG_ERR,
          "graph id: 0x%lx, bss id: %u, section idx: %u, data offset: %u "
          "cannot find corresponding in hashtable",
          m_id, bss_id, section.slot_index, bss_sec.src_offset);
      return AIPU_STATUS_ERROR_NOT_FOUND_IN_HASHTABLE;
    }

    uint32_t index = iter - m_hashtable[bss_id].begin();
    std::for_each(m_hashtable.begin(), m_hashtable.begin() + bss_id,
                  [&index](const std::vector<ConstantHashItem> &it) {
                    index += it.size();
                  });
    section.hashcode = iter->hashcode;
  }

  return AIPU_STATUS_SUCCESS;
}

} // namespace aipudrv