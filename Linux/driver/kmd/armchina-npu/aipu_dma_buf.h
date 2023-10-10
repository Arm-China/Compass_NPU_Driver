/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_DMA_BUF_H__
#define __AIPU_DMA_BUF_H__

#include <linux/dma-buf.h>
#include "armchina_aipu.h"
#include "aipu_mm.h"

struct aipu_dma_buf_priv {
	struct aipu_memory_manager *mm;
	u64 dev_pa;   /* the addr configured into NPU */
	u64 dma_pa;   /* the original addr from dma subsystem */
	u64 bytes;
	void *va;
	struct sg_table *sgt;
};

int aipu_alloc_dma_buf(struct aipu_memory_manager *mm, struct aipu_dma_buf_request *request);
int aipu_free_dma_buf(struct aipu_memory_manager *mm, int fd);
int aipu_get_dma_buf_info(struct aipu_dma_buf *dmabuf_info);

#endif /* __AIPU_DMA_BUF_H__ */
