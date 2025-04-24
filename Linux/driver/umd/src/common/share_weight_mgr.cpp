// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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

#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
aipu_status_t ShareWeightMgr::parse() {
  const char *zip_file = m_zip_file.c_str();
  int32_t fd = open(zip_file, O_RDWR);
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

  void *head =
      mmap(nullptr, head_list_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (head == MAP_FAILED) {
    close(fd);
    LOG(LOG_ERR, "RT failed in mapping graph file: %s! (errno = %d)\n",
        zip_file, errno);
    return AIPU_STATUS_ERROR_MAP_FILE_FAIL;
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
      m_elfs.push_back(FileSectionDesc(m_zip_file, offset, size));
    else if (names[i].find("merged_weight.bin") != std::string::npos)
      m_weight.init(m_zip_file, offset, size);
    else if (names[i].find("offset.bin") != std::string::npos)
      m_offset.init(m_zip_file, offset, size);
  }
  munmap(head, head_list_size);

  close(fd);
  return AIPU_STATUS_SUCCESS;
}

} // namespace aipudrv
