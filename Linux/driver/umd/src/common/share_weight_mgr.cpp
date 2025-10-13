// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  share_weight_mgr.cpp
 * @brief AIPU User Mode Driver (UMD) share weight manager model implementation
 */

#include "share_weight_mgr.h"

#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <queue>
#include <set>

#include "context.h"
#include "graph.h"
#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
SharedWeightMgr::SharedWeightMgr(MainContext *ctx, const char *zip_file)
    : m_ctx(ctx), m_zip_file(zip_file) {
  m_mem = m_ctx->get_dev()->get_mem();
}

aipu_status_t SharedWeightMgr::parse() {
  const char *zip_file = m_zip_file.c_str();
  int32_t fd = open(zip_file, O_RDONLY);
  if (fd <= 0) {
    LOG(LOG_ERR, "open file failed: %s! (errno = %d)\n", zip_file, errno);
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
  }

  struct stat finfo;
  if (stat(zip_file, &finfo) != 0) {
    close(fd);
    LOG(LOG_ERR, "no such a file: %s! (errno = %d)\n", zip_file, errno);
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
  }

  /* 4K for file_head.bin and file_list.bin */
  uint32_t head_list_size = 4096;
  if (finfo.st_size < head_list_size)
    head_list_size = finfo.st_size;

  void *head = nullptr;
  aipu_status_t ret =
      umd_mmap_file_helper(zip_file, &head, head_list_size, 0, true);
  if (ret != AIPU_STATUS_SUCCESS) {
    close(fd);
    LOG(LOG_ERR, "RT failed in mapping graph file: %s! (errno = %d)\n",
        zip_file, errno);
    return ret;
  }

  /* file head */
  const char *p_data = (char *)head;
  uint32_t file_num = *(uint32_t *)p_data;
  p_data += sizeof(uint32_t);
  std::vector<std::pair<uint64_t, uint64_t>> offset_sizes;
  for (uint32_t i = 0; i < file_num; ++i) {
    uint64_t offset = *(uint64_t *)p_data;
    uint64_t size = *(uint64_t *)(p_data + sizeof(uint64_t));
    offset_sizes.push_back({offset, size});
    p_data += 2 * sizeof(uint64_t);
  }

  /* file list */
  std::string file_names((const char *)p_data, offset_sizes[1].second);
  auto names = split_string(file_names, "\n");
  if (names.size() != file_num) {
    close(fd);
    munmap(head, head_list_size);
    return AIPU_STATUS_ERROR_READ_FILE_FAIL;
  }

  for (uint32_t i = 0; i < file_num; ++i) {
    uint64_t offset = offset_sizes[i].first;
    uint64_t size = offset_sizes[i].second;
    if (names[i].find("aipuelf") != std::string::npos)
      m_belfs.push_back(FileSectionDesc(m_zip_file, offset, size));
    else if (names[i].find("merged_weight.bin") != std::string::npos)
      m_bweight.init(m_zip_file, offset, size);
    else if (names[i].find("offset.bin") != std::string::npos)
      m_boffset.init(m_zip_file, offset, size);
  }
  munmap(head, head_list_size);

  close(fd);
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t
SharedWeightMgr::alloc_weight_buffer(const std::vector<GRAPH_ID> &graph_ids) {
  for (auto &id : graph_ids) {
    Graph *gobj = reinterpret_cast<Graph *>(m_ctx->get_graph_object(id));
    if (gobj == nullptr)
      return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

    m_graphs.push_back(gobj);
  }

  std::for_each(m_graphs.begin(), m_graphs.end(), [this](Graph *gobj) {
    if (gobj->get_bss_cnt() > m_max_bss_cnt)
      m_max_bss_cnt = gobj->get_bss_cnt();
  });

  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  m_shared_weight.resize(m_max_bss_cnt);
  m_hashtable_buffer_desc.resize(m_max_bss_cnt);
  m_const_size.resize(m_graphs.size());
  m_const_base.resize(m_graphs.size());

  /* 1.load offset */
  ret = load_offset();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /* 2.recorrect */
  ret = correct_section_params();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /* 3.alloc */
  uint32_t pad_size = m_graphs[0]->get_alloc_pad_size();
  uint32_t align_page = m_graphs[0]->get_asid_align_page();
  for (uint32_t bss_id = 0; bss_id < m_max_bss_cnt; ++bss_id) {
    uint32_t max_cts_size = 0;
    uint32_t max_zcy_cst_size = 0;
    for (uint32_t idx = 0; idx < m_graphs.size(); ++idx) {
      /* assume that bss corresponding in each graph, default to align page at
       * beginning of each bss in different graph */
      if (bss_id < m_graphs[idx]->get_bss_cnt()) {
        uint32_t aligned_cst = ALIGN_PAGE(max_cts_size);
        m_const_base[idx][SECTION_TYPE_CONSTANT].push_back(aligned_cst);
        max_cts_size =
            aligned_cst + m_const_size[idx][SECTION_TYPE_CONSTANT][bss_id];

        uint32_t aligned_zcy = ALIGN_PAGE(max_zcy_cst_size);
        m_const_base[idx][SECTION_TYPE_ZEROCPY_CONSTANT].push_back(aligned_zcy);
        max_zcy_cst_size =
            aligned_zcy +
            m_const_size[idx][SECTION_TYPE_ZEROCPY_CONSTANT][bss_id];
      }
    }
    LOG(LOG_INFO, "[alloc size] bss id: %u, weight size: 0x%-10x zcy: 0x%-10x",
        bss_id, max_cts_size, max_zcy_cst_size);

    std::string buf_name = std::string("weight_") + std::to_string(bss_id);
    if (max_cts_size > 0) {
      ret = m_mem->malloc(max_cts_size + pad_size, align_page,
                          &m_shared_weight[bss_id].wb_weight, buf_name.c_str(),
                          (m_mem->get_asid1() << 8) | AIPU_MEM_REGION_DEFAULT);
      if (ret != AIPU_STATUS_SUCCESS) {
        free_shared_weight(graph_ids);
        return ret;
      }

      m_shared_weight[bss_id].wb_asid_base =
          m_shared_weight[bss_id].wb_weight->asid_base;
    }

    buf_name = std::string("zcy_const_") + std::to_string(bss_id);
    if (max_zcy_cst_size > 0 && !m_graphs[0]->is_dynamic_asid0()) {
      ret =
          m_mem->malloc(max_zcy_cst_size + pad_size, 0,
                        &m_shared_weight[bss_id].wb_zerocpy_const,
                        buf_name.c_str(), AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT);
      if (ret != AIPU_STATUS_SUCCESS) {
        free_shared_weight(graph_ids);
        return ret;
      }
    }
  }

  /* 4.write */
  ret = setup_weight_buffer();
  if (ret != AIPU_STATUS_SUCCESS) {
    free_shared_weight(graph_ids);
    return ret;
  }

  m_weight_ref_cnt += graph_ids.size();

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t
SharedWeightMgr::free_shared_weight(const std::vector<GRAPH_ID> &graph_ids) {
  m_weight_ref_cnt = m_weight_ref_cnt >= graph_ids.size()
                         ? (m_weight_ref_cnt - graph_ids.size())
                         : 0;
  if (m_weight_ref_cnt > 0) {
    LOG(LOG_DEBUG, "shared weight still has %u refers", m_weight_ref_cnt);
    return AIPU_STATUS_SUCCESS;
  }

  for (auto &buf : m_shared_weight) {
    if (buf.wb_weight != nullptr && buf.wb_weight->size != 0)
      m_mem->free(&buf.wb_weight);
    if (buf.wb_zerocpy_const != nullptr && buf.wb_zerocpy_const->size != 0)
      m_mem->free(&buf.wb_zerocpy_const);
  }
  LOG(LOG_INFO, "freed shared weight");
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t SharedWeightMgr::load_offset() {
  std::vector<uint32_t> sec_cnt;
  for (uint32_t i = 0; i < m_graphs.size(); ++i) {
    uint32_t eles = 0;
    std::for_each(m_graphs[i]->get_hashtable().begin(),
                  m_graphs[i]->get_hashtable().end(),
                  [&](const std::vector<ConstantHashItem> &items) {
                    eles += items.size();
                  });
    sec_cnt.push_back(eles);
  }

  m_offsets.resize(m_graphs.size());

  char *poffset = nullptr;
  aipu_status_t ret =
      umd_mmap_file_helper(m_boffset.file.c_str(), (void **)&poffset,
                           m_boffset.size, m_boffset.offset, true);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  uint32_t graph_idx = 0;
  uint32_t cnt = 0;
  for (uint32_t i = 0; i < m_boffset.size; i += sizeof(uint64_t)) {
    m_offsets[graph_idx].push_back(*(uint64_t *)poffset);
    poffset += sizeof(uint64_t);
    if (++cnt >= sec_cnt[graph_idx]) {
      cnt = 0;
      ++graph_idx;
    }
  }

  munmap(poffset, m_boffset.size);
  return AIPU_STATUS_SUCCESS;
}

/* search all followed graphs whether there has larger align bytes */
aipu_status_t SharedWeightMgr::correct_section_params() {
  for (uint32_t graph_idx = 0; graph_idx < m_graphs.size(); ++graph_idx) {
    for (uint32_t bss_idx = 0; bss_idx < m_graphs[graph_idx]->get_bss_cnt();
         ++bss_idx) {
      uint32_t cst_offset = 0;
      uint32_t zcy_offset = 0;
      uint32_t cst_offset_unshared = 0;
      uint32_t zcy_offset_unshared = 0;
      for (uint32_t sec_idx = 0;
           sec_idx <
           m_graphs[graph_idx]->get_static_section_ref(bss_idx).size();
           ++sec_idx) {
        auto &sec =
            m_graphs[graph_idx]->get_static_section_ref(bss_idx)[sec_idx];

        /* unify align bytes to max between graphs (Attention: same graph not in
         * consideration) */
        for (uint32_t next_idx = graph_idx + 1; next_idx < m_graphs.size();
             ++next_idx) {
          auto &shared_sec =
              m_graphs[next_idx]->get_shared_section_ref(bss_idx)[sec.type];
          if (shared_sec.count(sec.hashcode) != 0) {
            if (sec.align_bytes < shared_sec[sec.hashcode].align_bytes) {
              LOG(LOG_DEBUG,
                  "graph id: 0x%lx, sec id: %u, align bytes: 0x%x hashcode "
                  "same and replaced by graph id: 0x%lx, align bytes: 0x%x",
                  m_graphs[graph_idx]->id(), sec_idx, sec.align_bytes,
                  m_graphs[next_idx]->id(),
                  shared_sec[sec.hashcode].align_bytes);
              sec.align_bytes = shared_sec[sec.hashcode].align_bytes;
            }
          }
        }

        if (sec.type == SECTION_TYPE_ZEROCPY_CONSTANT) {
          uint32_t offset = aligned(zcy_offset_unshared, sec.align_bytes);
          zcy_offset_unshared = offset + sec.size;
        } else {
          uint32_t offset = aligned(cst_offset_unshared, sec.align_bytes);
          cst_offset_unshared = offset + sec.size;
        }

        /* zcy CANNOT share in dynamic asid0 */
        if (sec.type == SECTION_TYPE_ZEROCPY_CONSTANT &&
            m_graphs[graph_idx]->is_dynamic_asid0()) {
          sec.dst_offset = aligned(zcy_offset, sec.align_bytes);
          zcy_offset = sec.dst_offset + sec.size;
        }

        if (m_hashtable_buffer_desc[bss_idx][sec.type].count(sec.hashcode) ==
            0) {
          if (sec.type == SECTION_TYPE_ZEROCPY_CONSTANT &&
              !m_graphs[graph_idx]->is_dynamic_asid0()) {
            sec.dst_offset = aligned(zcy_offset, sec.align_bytes);
            zcy_offset = sec.dst_offset + sec.size;
          } else if (sec.type == SECTION_TYPE_CONSTANT) {
            sec.dst_offset = aligned(cst_offset, sec.align_bytes);
            cst_offset = sec.dst_offset + sec.size;
          }
          m_hashtable_buffer_desc[bss_idx][sec.type].insert(
              {sec.hashcode, nullptr});
        }

        /* set offset to shared weight */
        auto &hashtable = m_graphs[graph_idx]->get_hashtable();
        auto iter = hashtable[bss_idx].begin();
        for (; iter < hashtable[bss_idx].end(); ++iter) {
          if (iter->offset == sec.src_offset)
            break;
        }
        uint32_t index = iter - hashtable[bss_idx].begin();
        std::for_each(hashtable.begin(), hashtable.begin() + bss_idx,
                      [&index](const std::vector<ConstantHashItem> &it) {
                        index += it.size();
                      });
        sec.src_offset_in_share_weight = m_offsets[graph_idx][index];
      }
      m_const_size[graph_idx][SECTION_TYPE_CONSTANT].push_back(cst_offset);
      m_const_size[graph_idx][SECTION_TYPE_ZEROCPY_CONSTANT].push_back(
          zcy_offset);
      m_graphs[graph_idx]->get_shared_section_ref(bss_idx).clear();
      if (cst_offset != cst_offset_unshared ||
          zcy_offset != zcy_offset_unshared) {
        LOG(LOG_INFO,
            "graph idx: 0x%lx, bss idx: %u, [weight] size: 0x%-10x shared "
            "size: 0x%-10x differ size: 0x%-10x",
            m_graphs[graph_idx]->id(), bss_idx, cst_offset_unshared,
            cst_offset_unshared - cst_offset, cst_offset);
        LOG(LOG_INFO,
            "graph idx: 0x%lx, bss idx: %u, [zcy]    size: 0x%-10x shared "
            "size: 0x%-10x differ size: 0x%-10x",
            m_graphs[graph_idx]->id(), bss_idx, zcy_offset_unshared,
            zcy_offset_unshared - zcy_offset, zcy_offset);
      }
    }
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t SharedWeightMgr::setup_weight_buffer() {
  DEV_PA_64 pa = 0;

  for (uint32_t graph_idx = 0; graph_idx < m_graphs.size(); ++graph_idx) {
    for (uint32_t bss_idx = 0; bss_idx < m_graphs[graph_idx]->get_bss_cnt();
         ++bss_idx) {
      for (uint32_t sec_idx = 0;
           sec_idx <
           m_graphs[graph_idx]->get_static_section_ref(bss_idx).size();
           ++sec_idx) {
        auto &sec =
            m_graphs[graph_idx]->get_static_section_ref(bss_idx)[sec_idx];

        if (sec.type == SECTION_TYPE_ZEROCPY_CONSTANT &&
            m_graphs[graph_idx]->is_dynamic_asid0()) {
          m_shared_weight[bss_idx].wb_weights.push_back(nullptr);
          if (bss_idx != 0)
            m_shared_weight[0].wb_weights.push_back(nullptr);
          continue;
        }

        BufferDesc *buf = new BufferDesc; /* release by graph dctor */
        buf->reset();

        if (m_hashtable_buffer_desc[bss_idx][sec.type][sec.hashcode] !=
            nullptr) {
          *buf = *m_hashtable_buffer_desc[bss_idx][sec.type].at(sec.hashcode);
        } else {
          DEV_PA_64 pa_base =
              m_shared_weight[bss_idx].wb_weight->pa +
              m_const_base[graph_idx][SECTION_TYPE_CONSTANT][bss_idx];
          DEV_PA_64 asid_base = m_shared_weight[bss_idx].wb_weight->asid_base;
          uint32_t asid_cfg =
              (m_mem->get_asid1() << 8) | AIPU_MEM_REGION_DEFAULT;
          if (sec.type == SECTION_TYPE_ZEROCPY_CONSTANT) {
            pa_base =
                m_shared_weight[bss_idx].wb_zerocpy_const->pa +
                m_const_base[graph_idx][SECTION_TYPE_ZEROCPY_CONSTANT][bss_idx];
            asid_base = m_shared_weight[bss_idx].wb_zerocpy_const->asid_base;
            asid_cfg = AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT;
          }

          pa = pa_base + sec.dst_offset;
          buf->init(asid_base, pa, sec.size, sec.size, 0, asid_cfg);
          m_hashtable_buffer_desc[bss_idx][sec.type].at(sec.hashcode) = buf;

          uint64_t abs_offset =
              m_bweight.offset + sec.src_offset_in_share_weight;
          char *src = nullptr;
          umd_mmap_file_helper(m_bweight.file.c_str(), (void **)&src, sec.size,
                               abs_offset, true);
          m_mem->write(pa, src, sec.size);
          munmap(src, sec.size);
        }

        m_shared_weight[bss_idx].wb_weights.push_back(buf);
        if (bss_idx != 0)
          m_shared_weight[0].wb_weights.push_back(buf);
      }
    }

    /* shared weight will not save BufferDesc, released by each graph */
    m_graphs[graph_idx]->set_weight_buffer_info(m_shared_weight);
    for (uint32_t bss_idx = 0; bss_idx < m_graphs[graph_idx]->get_bss_cnt();
         ++bss_idx)
      m_shared_weight[bss_idx].wb_weights.clear();
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t
SharedWeightMgr::setup_zcy_buffer(std::vector<WeightBufferInfo> &weights,
                                  uint64_t graph_id) {
  Graph *gobj = reinterpret_cast<Graph *>(m_ctx->get_graph_object(graph_id));
  if (gobj == nullptr)
    return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

  for (uint32_t bss_idx = 0; bss_idx < gobj->get_bss_cnt(); ++bss_idx) {
    BufferDesc *zcy_desc = weights[bss_idx].wb_zerocpy_const;
    std::vector<GraphSectionDesc> &static_sections =
        gobj->get_static_section_ref(bss_idx);
    for (uint32_t i = 0; i < static_sections.size(); ++i) {
      if (static_sections[i].type == SECTION_TYPE_ZEROCPY_CONSTANT) {
        BufferDesc *buf = new BufferDesc;
        buf->reset();

        DEV_PA_64 sec_pa = zcy_desc->pa + static_sections[i].dst_offset;
        buf->init(zcy_desc->asid_base, sec_pa, static_sections[i].size,
                  static_sections[i].size);

        /* bss0 includes other bss id */
        uint32_t offset = i;
        weights[bss_idx].wb_weights[offset] = buf;
        for (uint32_t bss = 0; bss < bss_idx; ++bss)
          offset += gobj->get_static_section_ref(bss).size();
        weights[0].wb_weights[offset] = buf;

        uint64_t abs_offset =
            m_bweight.offset + static_sections[i].src_offset_in_share_weight;
        char *src = nullptr;
        umd_mmap_file_helper(m_bweight.file.c_str(), (void **)&src,
                             static_sections[i].size, abs_offset, true);
        m_mem->write(sec_pa, src, static_sections[i].size);
        munmap(src, static_sections[i].size);
      }
    }
  }
  return AIPU_STATUS_SUCCESS;
}

} // namespace aipudrv
