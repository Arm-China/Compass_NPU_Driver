// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  share_weight_mgr.h
 * @brief AIPU User Mode Driver (UMD) share weight manager module header
 */

#ifndef _SHARE_WEIGHT_MGR_H_
#define _SHARE_WEIGHT_MGR_H_

#include <pthread.h>

#include "memory_base.h"
#include "parser_base.h"
#include "standard_api.h"
#include <array>
#include <cstring>
#include <fstream>
#include <map>

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

using HASHTABLE_BUFFER =
    std::vector<std::map<std::array<uint8_t, 32>, BufferDesc *>>;
class ShareWeightMgr {
private:
  std::string m_zip_file;
  std::vector<FileSectionDesc> m_elfs;
  FileSectionDesc m_weight;
  FileSectionDesc m_offset;
  /* std::vector<bss_id>, std::map<hashcode, buffer_desc> */
  HASHTABLE_BUFFER m_hashtable_buffer_desc;

public:
  ShareWeightMgr(const char *zip_file) : m_zip_file(zip_file){};
  ~ShareWeightMgr() {}

  aipu_status_t parse();

  std::string &file_path() { return m_zip_file; }

  std::vector<FileSectionDesc> &get_elfs() { return m_elfs; }

  FileSectionDesc &get_weight() { return m_weight; }

  FileSectionDesc &get_offset() { return m_offset; }

  HASHTABLE_BUFFER &get_hashtable_desc() { return m_hashtable_buffer_desc; }
};
} // namespace aipudrv

#endif /* _SHARE_WEIGHT_MGR_H_ */
