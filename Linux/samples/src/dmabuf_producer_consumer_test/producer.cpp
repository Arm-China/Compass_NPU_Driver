// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  producer.cpp
 * @brief AIPU UMD test application: dmabuf test as producer
 */

/**
 * @brief request a dmabuf as output tensor, then fill this buffer with
 *        input data, finally transfer it to server side.
 *
 */
#include "common.h"

int main(int argc, char *argv[]) {
  int dmabuf_fd = -1;
  int size = 0, align_size = 0;
  int ret = -1;
  char *data = nullptr;

  AIPU_INFO()("file: %s\n", argv[1]);
  data = (char *)map_file_data(argv[1], size);
  if (data == nullptr) {
    AIPU_ERR() << "map_file_data [fail]\n";
    goto out;
  }
  AIPU_INFO()("map_file_data %s success\n", argv[1]);

  align_size = PAGE_ALIGN_SIZE(size);
  if ((dmabuf_fd = dmabuf_malloc(align_size)) < 0) {
    AIPU_ERR() << "dmabuf_malloc [fail]\n";
    goto out;
  }
  AIPU_INFO()("dmabuf_malloc dma_buf input success, dmabuf_fd=%d\n", dmabuf_fd);

  /* fill input data */
  if (dmabuf_fill(dmabuf_fd, data, size) != 0) {
    AIPU_ERR() << "dmabuf_fill [fail]\n";
    goto out;
  }
  AIPU_INFO()("producer send dma_buf fd, dmabuf_fd=%d\n", dmabuf_fd);

  if (sender(dmabuf_fd, PRODUCER_SERVER_PATH) < 0) {
    AIPU_ERR() << "send dmdbuf_fd [fail]\n";
    goto out;
  }
  AIPU_INFO()("sender %s success\n", PRODUCER_SERVER_PATH);

  ret = 0;
out:
  unmap_file_data(data, size);
  return ret;
}