// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  consumer.cpp
 * @brief AIPU UMD test application: dmabuf test as consumer
 */

/**
 * @brief receive a dmabuf which is filled with inferred output tensor, and
 * check the result.
 *
 */
#include "common.h"

int main(int argc, char *argv[]) {
  int dmabuf_output_fd = -1;
  int size = 0;
  char *data = nullptr;
  int ret = -1;
  char *gt = nullptr;

  AIPU_INFO()("file: %s\n", argv[1]);
  gt = (char *)map_file_data(argv[1], size);
  if (gt == nullptr) {
    AIPU_ERR() << "map_file_data [fail]\n";
    goto out;
  }
  AIPU_INFO()("map_file_data %s success\n", argv[1]);

  AIPU_INFO()("consumer recv dmabuf fd, wait ...\n");
  dmabuf_output_fd = recver(CONSUMER_SERVER_PATH);
  if (dmabuf_output_fd < 0) {
    AIPU_ERR() << "recver dmdbuf_fd [fail]\n";
    goto out;
  }
  AIPU_INFO()("consumer recv success\n");

  // dmabuf_dump_file("dmabuf_output.bin", dmabuf_output_fd, size);

  data = (char *)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
                      dmabuf_output_fd, 0);
  if (data == MAP_FAILED) {
    AIPU_ERR() << "mmap dmabuf [fail]\n";
    goto out;
  }

  if (is_output_correct(data, gt, size) == true) {
    AIPU_CRIT()("Test Result Check PASS!\n");
  } else {
    AIPU_ERR()("Test Result Check FAILED!\n");
  }

  ret = 0;
out:
  if (dmabuf_output_fd > 0)
    dmabuf_free(dmabuf_output_fd);
  if (data != nullptr)
    munmap(data, size);
  unmap_file_data(data, size);
  return ret;
}