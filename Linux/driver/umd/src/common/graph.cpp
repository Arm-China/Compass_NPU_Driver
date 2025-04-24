// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  graph.cpp
 * @brief AIPU User Mode Driver (UMD) graph module implementation
 */

#include "graph.h"

#include <sys/mman.h>

#include <cstring>

#include "parser_base.h"
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
                          aipu_load_graph_cfg_t *config) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  /**
   * decide weight allocation strategy
   */
  if (config != nullptr) {
    m_wt_mem_region = config->wt_mem_region;
    if (config->wt_idxes) {
      for (int i = 0; i < config->wt_idxes_cnt; i++)
        m_wt_idxes.insert(config->wt_idxes[i]);
    }

    if (config->extra_weight_path != nullptr) {
      if (access(config->extra_weight_path, F_OK) == 0) {
        m_bweight.extra_weight_path = config->extra_weight_path;
      } else {
        LOG(LOG_ERR, "Extra weight path: %s [non exist]",
            config->extra_weight_path);
        return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
      }
    }
  }

  ret = m_parser->parse_graph(gbin, size, *this);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  m_mem->dump_tracking_log_start();
  m_do_vcheck = ver_check;
  if (ver_check &&
      !m_dev->has_target(m_arch, m_hw_version, m_hw_config, m_hw_revision))
    return AIPU_STATUS_ERROR_TARGET_NOT_FOUND;

  /* alloc and load text buffer */
  if (m_btext.size != 0) {
    /**
     * expand 16 bytes more to export RO base for debugger.
     * there is no effect for text self.
     */
    ret = m_mem->malloc(m_btext.size + 16, 0, &m_text, "text");
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
    m_mem->write(m_text->pa, m_btext.va, m_btext.size);
  }

  if (m_bcrodata.size != 0) {
    ret = m_mem->malloc(m_bcrodata.size, 0, &m_crodata, "crodata");
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
    m_mem->write(m_crodata->pa, m_bcrodata.va, m_bcrodata.size);
  }

  /* weight buffer is deferred, because shared weight between graphs need all
   * graphs information */

finish:
  return ret;
}

aipu_status_t Graph::write_weight_buffer() {
  if (m_weight.size() != get_bss_cnt()) {
    LOG(LOG_ERR, "weight buffer vector size: %zu is not equal to bss cnt: %u",
        m_weight.size(), get_bss_cnt());
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  }

  GraphSectionDesc *static_section = nullptr;
  for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++) {
    if (!has_weight(bss_id))
      continue;

    uint32_t base_offset = get_bss_base_offset(bss_id);
    uint32_t zcy_base_offset = get_bss_zcy_base_offset(bss_id);
    std::vector<GraphSectionDesc> &static_sections =
        get_static_section_ref(bss_id);
    for (uint32_t i = 0; i < static_sections.size(); i++) {
      BufferDesc *buf = new BufferDesc;
      buf->reset();
      static_section = &static_sections[i];

      aipu_status_t ret = write_static_section(bss_id, static_section, buf,
                                               base_offset, zcy_base_offset);
      if (ret != AIPU_STATUS_SUCCESS)
        return ret;

      if (static_section->type == SECTION_TYPE_ZEROCPY_CONSTANT) {
        LOG(LOG_DEBUG,
            "zerocpy in static_sections[%d], size=0x%x, pa=0x%lx, a_b=0x%lx, "
            "asid_pa=0x%lx, dst_offset=0x%x",
            i, static_section->size, buf->pa, buf->asid_base,
            buf->align_asid_pa, static_section->dst_offset);
      }

      m_weight[bss_id].wb_weights.push_back(buf);
      if (bss_id != 0)
        m_weight[0].wb_weights.push_back(buf);
    }

    m_weight[bss_id].wb_asid_base = m_weight[bss_id].wb_weight->asid_base;

    /* munmap here is to release each bss to save host memory usage */
    if (bss_id != 0 && m_bweight.weight.size() > 1)
      munmap(const_cast<char *>(m_bweight.weight[bss_id].va),
             m_bweight.weight[bss_id].size);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t Graph::alloc_gathered_weight() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t weight_asid = 0;

  if (m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3 ||
      m_hw_version == AIPU_ISA_VERSION_ZHOUYI_V3_1)
    weight_asid = 1;

  /* 1.alloc */
  m_weight.resize(get_bss_cnt());
  for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); ++bss_id) {
    if (!has_weight(bss_id))
      continue;

    ret = m_mem->malloc(get_const_size(bss_id) + get_alloc_pad_size(), 1,
                        &m_weight[bss_id].wb_weight, "weight",
                        (weight_asid << 8) | AIPU_MEM_REGION_DEFAULT);
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_ERR, "alloc weight buffer [fail]");
      return ret;
    }

    if (get_zerocpy_const_size(bss_id) > 0) {
      ret =
          m_mem->malloc(get_zerocpy_const_size(bss_id) + get_alloc_pad_size(),
                        1, &m_weight[bss_id].wb_zerocpy_const, "zerocpy_const",
                        AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT);
      if (ret != AIPU_STATUS_SUCCESS) {
        LOG(LOG_ERR, "alloc zerocpy_const buffer [fail]");
        return ret;
      }
    }
  }

  /* 2.update data from va to pa */
  return write_weight_buffer();
}

aipu_status_t Graph::alloc_scattered_weight() {
  if (get_bss_cnt() > 1) {
    LOG(LOG_ALERT,
        "Not support to specify weight region on BSS count>0, now bss cnt: %u",
        get_bss_cnt());
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  }

  m_weight.resize(1);
  std::vector<struct GraphSectionDesc> &static_sections =
      get_static_section_ref(0);
  GraphSectionDesc *static_section = nullptr;
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  for (uint32_t i = 0; i < static_sections.size(); i++) {
    if (m_weight[0].wb_weights.size() != static_sections.size()) {
      BufferDesc *buf = nullptr;
      std::string str = "static_" + std::to_string(i);
      static_section = &static_sections[i];

      if ((m_wt_idxes.count(i) == 1) &&
          (static_section->type != SECTION_TYPE_ZEROCPY_CONSTANT)) {
        ret = m_mem->malloc(static_section->size + get_alloc_pad_size(),
                            static_section->align_in_page, &buf, str.c_str(),
                            m_wt_mem_region);
      } else {
        if (static_section->type == SECTION_TYPE_ZEROCPY_CONSTANT)
          ret = m_mem->malloc(static_section->size + get_alloc_pad_size(),
                              static_section->align_in_page, &buf, str.c_str(),
                              AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT);
        else {
          ret = m_mem->malloc(static_section->size + get_alloc_pad_size(),
                              static_section->align_in_page, &buf, str.c_str(),
                              AIPU_ASID1 | AIPU_MEM_REGION_DEFAULT);
        }
      }

      if (ret != AIPU_STATUS_SUCCESS) {
        LOG(LOG_ERR, "alloc weight buffer %d [fail]", i);
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
      }

      const char *va = m_bweight.weight[0].va + static_section->src_offset;
      m_mem->write(buf->pa, va, static_section->size);
      m_weight[0].wb_weights.push_back(buf);
    }
  }
  return AIPU_STATUS_SUCCESS;
}

/**
 * @brief each graph only needs to allocate one copy of weight
 *        for multiple jobs in order to reduce memory consumption.
 *
 * @note  if weight buffer is in DDR region, the whole weight data
 *        is put in one large buffer locating in ASID1.
 *        if intend to put weight buffer in SRAM or DTCM, the large
 *        weight buffer is split into more small buffers in order to
 *        put them more to specific region. the rest of buffer that
 *        can't be allocated from SRAM/DTCM is from ASID0 default.
 */
aipu_status_t Graph::alloc_weight_buffer() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  if (has_weight()) {
    if (m_wt_mem_region == AIPU_MEM_REGION_DEFAULT)
      ret = alloc_gathered_weight();
    else
      ret = alloc_scattered_weight();
  }
  return ret;
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
      if (m_weight[0].wb_weight == nullptr) /* scattered */
      {
        m_mem->free(&wb_weights[i]);
      } else {
        wb_weights[i]->reset();
        delete wb_weights[i];
        wb_weights[i] = nullptr;
      }
    }
    wb_weights.clear();

    for (uint32_t bss_id = 0; bss_id < get_bss_cnt(); bss_id++) {
      if (m_weight.size() <= bss_id)
        break;

      if (is_exclusive_weight()) {
        WeightBufferInfo &weight_buf_info = m_weight[bss_id];
        if (weight_buf_info.wb_zerocpy_const != nullptr &&
            weight_buf_info.wb_zerocpy_const->size != 0)
          m_mem->free(&weight_buf_info.wb_zerocpy_const);

        if (weight_buf_info.wb_weight != nullptr &&
            weight_buf_info.wb_weight->size != 0)
          m_mem->free(&weight_buf_info.wb_weight);
      }
    }
  }

  // m_mem->dump_tracking_log_end();
  return ret;
}

int32_t Graph::get_dynamic_shape_dim_num(uint32_t idx, bool max_shape_dim) {
  if (!is_dynamic_shape())
    return 0;

  if (idx >= 0 && idx < m_input_shape_constraint.size()) {
    if (!max_shape_dim)
      return m_input_shape_constraint[idx][0].size();
    else
      return m_input_shape_constraint[idx][1].size();
  }

  return 0;
}

bool Graph::get_dynamic_shape_data(uint32_t idx, bool max_shape_dim,
                                   uint32_t *data) {
  if (!is_dynamic_shape())
    return false;

  if (data == nullptr) {
    LOG(LOG_ERR, "data ptr is NULL");
    return false;
  }

  if (idx >= 0 && idx < m_input_shape_constraint.size()) {
    if (!max_shape_dim) {
      for (uint32_t i = 0; i < m_input_shape_constraint[idx][0].size(); i++)
        *(data + i) = m_input_shape_constraint[idx][0][i];
    } else {
      for (uint32_t i = 0; i < m_input_shape_constraint[idx][1].size(); i++)
        *(data + i) = m_input_shape_constraint[idx][1][i];
    }
  }

  return true;
}

aipu_status_t Graph::set_graph_extra_weight(const BinSection &extra_weight) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  char *start = (char *)extra_weight.va;
  size_t extra_weight_bin_cnt = *(size_t *)start;

  start += sizeof(size_t);
  for (size_t i = 0; i < extra_weight_bin_cnt; i++) {
    WeightSection::ExtraWeightInfo extra_weight_info;
    BinSection ew_binsection = {0};
    size_t bin_name_len = *(size_t *)start;
    std::string path;

    start += sizeof(size_t);
    char *plus_mark = std::strstr(start, "+");
    extra_weight_info.name = std::string(start, plus_mark - start);
    extra_weight_info.hash =
        std::string(plus_mark + 1, bin_name_len - (plus_mark + 1 - start));
    start += bin_name_len;

    path = m_bweight.extra_weight_path + "/" + extra_weight_info.name;
    ret = umd_mmap_file_helper(path.c_str(), (void **)&ew_binsection.va,
                               &ew_binsection.size);
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_ERR, "Mmap extra weight: %s [fail]", path.c_str());
      m_bweight.extra_weight_infos.clear();
      return ret;
    }

    set_graph_weight(ew_binsection);
    m_bweight.extra_weight_infos.push_back(extra_weight_info);
  }

  return ret;
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
    GraphSectionDesc &section, uint32_t &cst_addr_orig,
    uint32_t &zcy_cst_addr_orig, uint32_t &cst_addr, uint32_t &zcy_cst_addr) {
  if (section.type == SECTION_TYPE_ZEROCPY_CONSTANT) {
    uint32_t offset = aligned(zcy_cst_addr_orig, bss_sec.align_bytes);
    zcy_cst_addr_orig = offset + section.size;
  } else {
    uint32_t offset = aligned(cst_addr_orig, bss_sec.align_bytes);
    cst_addr_orig = offset + section.size;
  }

  if (is_exclusive_weight()) {
    if (section.type == SECTION_TYPE_ZEROCPY_CONSTANT) {
      section.dst_offset = aligned(zcy_cst_addr, bss_sec.align_bytes);
      zcy_cst_addr = section.dst_offset + section.size;
    } else {
      section.dst_offset = aligned(cst_addr, bss_sec.align_bytes);
      cst_addr = section.dst_offset + section.size;
    }
  } else {
    if (m_sw_mgr == nullptr) {
      LOG(LOG_ERR, "shared weight manager is nullptr.");
      return AIPU_STATUS_ERROR_NULL_PTR;
    }

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
    section.src_offset_in_share_weight = m_bshared_weight.offsets[index];
    section.hashcode = iter->hashcode;

    auto &table = m_sw_mgr->get_hashtable_desc();
    if (bss_id == table.size())
      table.resize(bss_id + 1);

    if (table[bss_id].count(section.hashcode) == 0) {
      if (section.type == SECTION_TYPE_ZEROCPY_CONSTANT) {
        section.dst_offset = aligned(zcy_cst_addr, bss_sec.align_bytes);
        zcy_cst_addr = section.dst_offset + section.size;
      } else {
        section.dst_offset = aligned(cst_addr, bss_sec.align_bytes);
        cst_addr = section.dst_offset + section.size;
      }
      table[bss_id].insert({section.hashcode, nullptr});
    }
  }
  return AIPU_STATUS_SUCCESS;
}

/**
 * Attention: if graphs' constant tenstor has same hashid, but different size,
 * it is dangerous and needs to rewrite again
 */
aipu_status_t Graph::write_static_section(uint32_t bss_id,
                                          const GraphSectionDesc *section,
                                          BufferDesc *desc, uint32_t cst_base,
                                          uint32_t zcy_cst_abse) {
  DEV_PA_64 pa = 0;

  if (is_exclusive_weight()) {
    if (section->type == SECTION_TYPE_ZEROCPY_CONSTANT) {
      pa = m_weight[bss_id].wb_zerocpy_const->pa + section->dst_offset;
      desc->init(m_weight[bss_id].wb_zerocpy_const->asid_base, pa,
                 section->size, section->size);
    } else {
      pa = m_weight[bss_id].wb_weight->pa + section->dst_offset;
      desc->init(m_weight[bss_id].wb_weight->asid_base, pa, section->size,
                 section->size, 0, AIPU_ASID1 | AIPU_MEM_REGION_DEFAULT);
    }

    if (m_mem->write(pa, m_bweight.weight[bss_id].va + section->src_offset,
                     section->size) != section->size)
      return AIPU_STATUS_ERROR_INVALID_SIZE;
  } else {
    if (m_sw_mgr == nullptr) {
      LOG(LOG_ERR, "shared weight manager is nullptr.");
      return AIPU_STATUS_ERROR_NULL_PTR;
    }

    auto &table = m_sw_mgr->get_hashtable_desc();
    if (bss_id >= table.size() || table[bss_id].count(section->hashcode) == 0) {
      LOG(LOG_ERR, "hashcode table is invalid");
      return AIPU_STATUS_ERROR_INVALID_OP;
    }

    /* each graph has an object to simplify free */
    if (table[bss_id][section->hashcode] != nullptr) {
      *desc = *table[bss_id].at(section->hashcode);
    } else {
      DEV_PA_64 pa_base = m_weight[bss_id].wb_weight->pa + cst_base;
      DEV_PA_64 asid_base = m_weight[bss_id].wb_weight->asid_base;
      uint32_t asid_cfg = AIPU_ASID1 | AIPU_MEM_REGION_DEFAULT;
      if (section->type == SECTION_TYPE_ZEROCPY_CONSTANT) {
        pa_base = m_weight[bss_id].wb_zerocpy_const->pa + zcy_cst_abse;
        asid_base = m_weight[bss_id].wb_zerocpy_const->asid_base;
        asid_cfg = AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT;
      }

      pa = pa_base + section->dst_offset;
      desc->init(asid_base, pa, section->size, section->size, 0, asid_cfg);
      table[bss_id].at(section->hashcode) = desc;

      uint64_t abs_offset =
          m_bshared_weight.weight.offset + section->src_offset_in_share_weight;
      char *src = nullptr;
      umd_mmap_file_helper(m_bshared_weight.weight.file.c_str(), (void **)&src,
                           section->size, abs_offset);
      m_mem->write(pa, src, section->size);
      munmap(src, section->size);
    }
  }

  return AIPU_STATUS_SUCCESS;
}

} // namespace aipudrv