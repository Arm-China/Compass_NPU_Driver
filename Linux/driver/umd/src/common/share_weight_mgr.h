// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  share_weight_mgr.h
 * @brief AIPU User Mode Driver (UMD) share weight manager module header
 */

#ifndef _SHARE_WEIGHT_MGR_H_
#define _SHARE_WEIGHT_MGR_H_

#include <pthread.h>

#include <array>
#include <cstring>
#include <fstream>
#include <map>

#include "memory_base.h"
#include "parser_base.h"
#include "standard_api.h"

namespace aipudrv {
struct FileSectionDesc {
  std::string file;
  uint64_t offset;
  uint64_t size;

  FileSectionDesc(const std::string &file = "", uint64_t _offset = 0,
                  uint64_t _size = 0)
      : offset(_offset), size(_size) {}

  void init(const std::string &_file, uint64_t _offset, uint64_t _size) {
    file = _file;
    offset = _offset;
    size = _size;
  }
};

class MainContext;
class MemoryBase;
class Graph;
struct WeightBufferInfo;
/* std::vector<bss>, std::map<const_type, std::map<hashvalue, BufferDesc*>> */
using HASHTABLE_BUFFER = std::vector<
    std::map<uint32_t, std::map<std::array<uint8_t, 32>, BufferDesc *>>>;
class SharedWeightMgr {
private:
  MainContext *m_ctx = nullptr;
  MemoryBase *m_mem = nullptr;
  std::string m_zip_file;
  std::vector<FileSectionDesc> m_belfs;
  FileSectionDesc m_bweight;
  FileSectionDesc m_boffset;
  std::vector<Graph *> m_graphs;
  std::vector<std::vector<uint64_t>>
      m_offsets; /* offset to merged_weight.bin */
  uint32_t m_max_bss_cnt = 0;
  /* std::vector<bss_id>, std::map<hashcode, buffer_desc> */
  HASHTABLE_BUFFER m_hashtable_buffer_desc;
  uint32_t m_weight_ref_cnt = 0; /* need mutex protect? */
  std::vector<WeightBufferInfo> m_shared_weight;
  /* std::vector<graph_idx>, std::map<const_type, std::vector<bss_id>> */
  std::vector<std::map<uint32_t, std::vector<uint32_t>>> m_const_size;
  /* std::vector<graph_idx>, std::map<const_type, std::vector<bss_id>> */
  std::vector<std::map<uint32_t, std::vector<uint32_t>>> m_const_base;

  aipu_status_t load_offset();
  aipu_status_t correct_section_params();
  aipu_status_t setup_weight_buffer();

public:
  SharedWeightMgr(MainContext *ctx, const char *zip_file);
  ~SharedWeightMgr() {}

  aipu_status_t parse();

  std::string &file_path() { return m_zip_file; }

  std::vector<FileSectionDesc> &get_belfs() { return m_belfs; }

  uint32_t get_ref_cnt() { return m_weight_ref_cnt; }

  aipu_status_t alloc_weight_buffer(const std::vector<GRAPH_ID> &graph_ids);
  aipu_status_t free_shared_weight(const std::vector<GRAPH_ID> &graph_ids);
  aipu_status_t setup_zcy_buffer(std::vector<WeightBufferInfo> &weights,
                                 uint64_t graph_id);
};
} // namespace aipudrv

#endif /* _SHARE_WEIGHT_MGR_H_ */
