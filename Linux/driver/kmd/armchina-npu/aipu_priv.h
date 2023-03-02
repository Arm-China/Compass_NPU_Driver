/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __AIPU_PRIV_H__
#define __AIPU_PRIV_H__

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include "armchina_aipu_soc.h"
#include "aipu_irq.h"
#include "aipu_io.h"
#include "aipu_partition.h"
#include "aipu_job_manager.h"
#include "aipu_mm.h"


struct aipu_priv_operations {
	struct aipu_partition *(*create_partitions)(struct aipu_priv *aipu, int id, struct platform_device *p_dev);
	void (*destroy_partitions)(struct aipu_priv *aipu);
	int (*global_soft_reset)(struct aipu_priv *aipu);
};

/**
 * struct aipu_priv - AIPU private struct contains all AIPU info and shared resources
 * @version:       AIPU hardware version
 * @partition_cnt: AIPU partition/core count in system
 * @cluster_cnt:   total cluster count (x2 only)
 * @max_partition_cnt: maximun partition count of this arch (x2 only)
 * @max_cmd_pool_cnt: maximun command pool count of this arch (x2 only)
 * @partitions:    partition/core array
 * @reg:           AIPU register base (x2 only)
 * @irq_obj:       interrupt object (x2 only)
 * @dev:           device struct pointer (of core 0 for z1/z2/z3/x1)
 * @soc:           SoC private data
 * @soc_ops:       SoC operation pointer
 * @aipu_fops:     file operation struct
 * @misc:          misc driver struct
 * @job_manager:   job manager struct
 * @mm:            memory manager
 * @ops:           aipu_priv version specific operations
 * @reset_delay_us: global soft-reset delay time
 * @is_init:       init flag
 */
struct aipu_priv {
	int version;
	u32 partition_cnt;
	u32 cluster_cnt;
	u32 max_partition_cnt;
	u32 max_cmd_pool_cnt;
	struct aipu_partition *partitions;
	struct io_region reg;
	struct aipu_irq_object *irq_obj;
	struct device *dev;
	struct aipu_soc              *soc;
	struct aipu_soc_operations   *soc_ops;
	const struct file_operations *aipu_fops;
	struct miscdevice            misc;
	struct aipu_job_manager      job_manager;
	struct aipu_memory_manager   mm;
	struct aipu_priv_operations  *ops;
	int reset_delay_us;
	bool is_init;
};

int init_aipu_priv(struct aipu_priv *aipu, struct platform_device *p_dev,
		   const struct file_operations *fops, struct aipu_soc *soc,
		   struct aipu_soc_operations *soc_ops);
int deinit_aipu_priv(struct aipu_priv *aipu);
int aipu_priv_get_version(struct aipu_priv *aipu);
int aipu_priv_get_partition_cnt(struct aipu_priv *aipu);
int aipu_priv_query_partition_capability(struct aipu_priv *aipu, struct aipu_partition_cap *cap);
int aipu_priv_query_capability(struct aipu_priv *aipu, struct aipu_cap *cap);
int aipu_priv_io_rw(struct aipu_priv *aipu, struct aipu_io_req *io_req);
int aipu_priv_check_status(struct aipu_priv *aipu);
struct aipu_soc *get_soc(struct aipu_partition *partition);
struct aipu_soc_operations *get_soc_ops(struct aipu_partition *partition);
struct aipu_job_manager *get_job_manager(struct aipu_partition *partition);
#endif /* __AIPU_PRIV_H__ */
