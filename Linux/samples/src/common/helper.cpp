// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include "helper.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "dbg.hpp"

static bool is_output_correct(const char *src1, const char *src2,
                              uint32_t cnt) {
  for (uint32_t out_chr = 0; out_chr < cnt; out_chr++) {
    if (src1[out_chr] != src2[out_chr]) {
      return false;
    }
  }
  return true;
}

int dump_file_helper(const char *fname, void *src, unsigned int size) {
  int ret = 0;
  int fd = 0;
  ssize_t wbytes = 0;
  ssize_t writen = 0;

  fd = open(fname, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
  if (fd == -1) {
    AIPU_ERR()("create bin file %s failed (errno = %d)!", fname, errno);
    ret = -1;
    goto finish;
  }
  if (ftruncate(fd, size) == -1) {
    AIPU_ERR()("create bin file %s failed (errno = %d)!", fname, errno);
    ret = -1;
    goto finish;
  }

  while (writen < size) {
    wbytes = write(fd, (const char *)src + writen, size - writen);
    if (wbytes <= 0) {
      AIPU_ERR()("write bin file failed: %s! (errno = %d)", fname, errno);
      ret = -1;
      goto finish;
    }
    writen += wbytes;
  }

finish:
  if (fd != -1) {
    close(fd);
  }
  return ret;
}

int load_file_helper(const char *fname, char **dest, uint32_t *size) {
  int ret = 0;
  int fd = 0;
  struct stat finfo;

  if ((nullptr == fname) || (nullptr == dest) || (nullptr == size)) {
    return -1;
  }

  *dest = nullptr;

  if (stat(fname, &finfo) != 0) {
    AIPU_ERR()("open file failed: %s! (errno = %d)\n", fname, errno);
    return -1;
  }

  fd = open(fname, O_RDONLY);
  if (fd <= 0) {
    AIPU_ERR()("open file failed: %s! (errno = %d)\n", fname, errno);
    return -1;
  }

  *dest = new char[finfo.st_size];
  *size = finfo.st_size;

  if (read(fd, *dest, finfo.st_size) < 0) {
    AIPU_ERR()("load file failed: %s! (errno = %d)\n", fname, errno);
    ret = -1;
    goto finish;
  }

finish:
  if (fd > 0) {
    close(fd);
  }
  if ((ret < 0) && (dest != nullptr) && (*dest != nullptr)) {
    delete[] * dest;
    *dest = nullptr;
  }
  return ret;
}

int unload_file_helper(char *data) {
  if (data != nullptr) {
    delete[] data;
    data = nullptr;
  }
  return 0;
}

int check_result_helper(const std::vector<char *> &outputs,
                        const std::vector<aipu_tensor_desc_t> &descs,
                        const std::vector<char *> &gt,
                        const std::vector<uint32_t> &gt_size) {
  if (outputs.size() != descs.size()) {
    AIPU_ERR()
    ("output data count (%lu) != benchmark tensor count (%lu)!\n",
     outputs.size(), descs.size());
    return -1;
  }

  if (gt.size() != gt_size.size()) {
    AIPU_ERR()
    ("output gt file count (%lu) != gt size count (%lu)!\n", gt.size(),
     gt_size.size());
    return -1;
  }

  if (gt.size() == 0) {
    AIPU_INFO()
    ("output gt file count is not provided, will not do comparision!\n");
    return 0;
  }

  uint32_t tot_size = 0;
  for (uint32_t i = 0; i < descs.size(); i++)
    tot_size += descs[i].size;

  int pass = 0;
  uint32_t offset = 0;
  for (uint32_t id = 0; id < outputs.size(); id++) {
    const char *out_va = outputs[id];
    const char *check_va = nullptr;

    if (gt.size() == 1) {
      check_va = gt[0] + offset;
      if (tot_size > gt_size[0]) {
        AIPU_ERR()
        ("total output size (0x%x) > total gt size: (0x%x)!\n", tot_size,
         gt_size[0]);
        return -1;
      }
    } else {
      check_va = gt[id];
      if (descs[id].size > gt_size[id]) {
        AIPU_ERR()
        ("%uth output descs.size (0x%x) > gt size: (0x%x)!\n", id,
         descs[id].size, gt_size[id]);
        return -1;
      }
    }

    bool ret = is_output_correct(out_va, check_va, descs[id].size);
    if (ret) {
      AIPU_CRIT()("Test Result Check PASS! (%u/%lu)\n", id + 1, outputs.size());
    } else {
      pass = -1;
      AIPU_ERR()
      ("Test Result Check FAILED! (%u/%lu)\n", id + 1, outputs.size());
    }
    offset += descs[id].size;
  }

  return pass;
}

int check_result(const std::vector<std::shared_ptr<char>> &outputs,
                 const std::vector<aipu_tensor_desc_t> &descs,
                 const std::vector<char *> &gt,
                 const std::vector<uint32_t> &gt_size) {
  if (outputs.size() == 0 || gt.size() == 0) {
    AIPU_ERR()
    ("output data count (%lu) gt count (%lu)!\n", outputs.size(), gt.size());
    return -1;
  }

  if (outputs.size() != descs.size()) {
    AIPU_ERR()
    ("output data count (%lu) != benchmark tensor count (%lu)!\n",
     outputs.size(), descs.size());
    return -1;
  }

  if (gt.size() != gt_size.size()) {
    AIPU_ERR()
    ("output gt file count (%lu) != gt size count (%lu)!\n", gt.size(),
     gt_size.size());
    return -1;
  }

  uint32_t tot_size = 0;
  for (uint32_t i = 0; i < descs.size(); i++)
    tot_size += descs[i].size;

  int pass = 0;
  uint32_t offset = 0;
  for (uint32_t id = 0; id < descs.size(); id++) {
    const char *out_va = outputs[id].get();
    const char *check_va = nullptr;

    if (gt.size() == 1) {
      check_va = gt[0] + offset;
      if (tot_size > gt_size[0]) {
        AIPU_ERR()
        ("total output size (0x%x) > total gt size: (0x%x)!\n", tot_size,
         gt_size[0]);
        return -1;
      }
    } else {
      check_va = gt[id];
      if (descs[id].size > gt_size[id]) {
        AIPU_ERR()
        ("%uth output descs.size (0x%x) > gt size: (0x%x)!\n", id,
         descs[id].size, gt_size[id]);
        return -1;
      }
    }

    bool ret = is_output_correct(out_va, check_va, descs[id].size);
    if (ret) {
      AIPU_CRIT()("Test Result Check PASS! (%u/%lu)\n", id + 1, outputs.size());
    } else {
      pass = -1;
      AIPU_ERR()
      ("Test Result Check FAILED! (%u/%lu)\n", id + 1, outputs.size());
    }
  }

  return pass;
}

/**
 * @brief create multiple level directory for specific benchmark case, thread
 * safe
 */
int help_create_dir(const char *path) {
  const char *delim = "/";
  char data[1024] = {0};
  char new_path[1024] = {0};
  char *p = nullptr;

  semOp_sp->semaphore_p();
  strncpy(data, path, sizeof(data) - 1);
  // printf("create_dir: %s\n", path);
  if (!strncmp(data, "/", 1)) {
    const char *split_dim = "/";
    strncpy(new_path, split_dim, sizeof(new_path) - 1);
  }

  p = strtok(data, delim);
  strcpy(new_path + strlen(new_path), p);
  strcat(new_path, "/");
  while ((p = strtok(NULL, delim))) {
    strcpy(new_path + strlen(new_path), p);
    strcat(new_path, "/");

    // printf("path: %s\n", new_path);
    if (!strcmp(p, ".") || (strlen(p) == 0))
      continue;

    if (access(new_path, F_OK) == 0)
      continue;

    if (mkdir(new_path, 0750) == -1) {
      printf("%s: mkdir %s failed\n", __FUNCTION__, new_path);
      semOp_sp->semaphore_v();
      exit(-1);
    }
  }

  semOp_sp->semaphore_v();
  return 0;
}

std::string timestamp_helper(int time_stamp_type) {
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm *now_tm = std::localtime(&now_time_t);
  char buffer[128] = {0};
  std::ostringstream ss;
  std::chrono::milliseconds ms;
  std::chrono::microseconds us;
  std::chrono::nanoseconds ns;

  strftime(buffer, sizeof(buffer), "%F %T", now_tm);
  ss.fill('0');

  switch (time_stamp_type) {
  case 0:
    ss << buffer;
    break;

  case 1:
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(
             now.time_since_epoch()) %
         1000;
    ss << buffer << ":" << ms.count();
    break;

  case 2:
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(
             now.time_since_epoch()) %
         1000;
    us = std::chrono::duration_cast<std::chrono::microseconds>(
             now.time_since_epoch()) %
         1000000;
    ss << buffer << ":" << ms.count() << ":" << us.count() % 1000;
    break;

  case 3:
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(
             now.time_since_epoch()) %
         1000;
    us = std::chrono::duration_cast<std::chrono::microseconds>(
             now.time_since_epoch()) %
         1000000;
    ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
             now.time_since_epoch()) %
         1000000000;
    ss << buffer << ":" << ms.count() << ":" << us.count() % 1000 << ":"
       << ns.count() % 1000;
    break;

  case 4:
    us = std::chrono::duration_cast<std::chrono::microseconds>(
             now.time_since_epoch()) %
         1000000;
    ss << buffer << ":" << us.count();
    break;

  default:
    ss << buffer;
    break;
  }

  return ss.str();
}

std::vector<std::string> split_string(const std::string &s,
                                      const std::string &splitter,
                                      const int keep_spliter) {
  std::vector<std::string> ret;
  int begin_pos = 0;
  auto pos = std::string::npos;
  int size = s.size();
  int split_size = splitter.size();
  while ((pos = s.find(splitter, begin_pos)) != std::string::npos &&
         begin_pos != size) {
    ret.push_back(s.substr(begin_pos, pos - begin_pos + keep_spliter));
    begin_pos = pos + split_size;
  }
  if (begin_pos != size) {
    ret.push_back(s.substr(begin_pos, size - begin_pos));
  }
  return ret;
}