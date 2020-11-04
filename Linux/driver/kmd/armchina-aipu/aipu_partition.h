/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __AIPU_PARTITION_H__
#define __AIPU_PARTITION_H__

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/atomic.h>
#include <uapi/misc/armchina_aipu.h>
#include "aipu_irq.h"
#include "aipu_io.h"
#include "zhouyi/zhouyi.h"
#include "config.h"

struct aipu_partition;
struct aipu_priv;

/**
 * struct aipu_operations - a struct contains AIPU hardware operation methods
 * @get_config:         get hardware configuration number
 * @enable_interrupt:   enable all AIPU interrupts
 * @disable_interrupt:  disable all AIPU interrupts
 * @trigger:            trigger a deferred-job to run on a reserved core/partition
 * @reserve:            reserve AIPU core/partition for a job/deferred-job
 * @is_idle:            is AIPU hardware idle or not
 * @read_status_reg:    read status register value
 * @print_hw_id_info:   print AIPU version ID registers information
 * @io_rw:              direct IO read/write operations
 * @upper_half:         interrupt upper half handler
 * @bottom_half:        interrupt bottom half handler
 * @sysfs_show:         show AIPU external register values
 * @soft_reset:         AIPU core/partition soft reset function
 * @initialize:         initialize AIPU core/partition after soft reset
 */
struct aipu_operations {
	int (*get_config)(struct aipu_partition *aipu);
	void (*enable_interrupt)(struct aipu_partition *aipu);
	void (*disable_interrupt)(struct aipu_partition *aipu);
	void (*trigger)(struct aipu_partition *aipu);
	int (*reserve)(struct aipu_partition *aipu, struct aipu_job_desc *udesc,
		       int do_trigger);
	bool (*is_idle)(struct aipu_partition *aipu);
	void (*print_hw_id_info)(struct aipu_partition *aipu);
	int (*io_rw)(struct aipu_partition *aipu, struct aipu_io_req *io_req);
	int (*upper_half)(void *data);
	void (*bottom_half)(void *data);
#ifdef CONFIG_SYSFS
	int (*sysfs_show)(struct aipu_partition *aipu, char *buf);
#endif
	int (*soft_reset)(struct aipu_partition *aipu, bool init_regs);
	void (*initialize)(struct aipu_partition *aipu);
	void (*set_partition)(struct aipu_partition *partition, u32 cluster_id);
	void (*config_partition_cmd_pool)(struct aipu_partition *partition);
	void (*destroy_command_pool)(struct aipu_partition *partition);
};

struct cluster_info {
	u32 id;
	u32 core_cnt;
	u32 tec_cnt;
	u32 gm_bytes;
};

/**
 * struct aipu_partition - a general struct describe a hardware AIPU partition or a single core
 * @id:              AIPU core/partition ID
 * @arch:            AIPU architecture number
 * @version:         AIPU hardware version number
 * @config:          AIPU hardware configuration number
 * @name:            AIPU name string
 * @max_sched_num:   maximum number of jobs can be scheduled in pipeline
 * @dev:             device struct pointer
 * @reg:             IO region array of this AIPU core
 * @ops:             operations of this core
 * @irq_obj:         interrupt object of this core
 * @priv:            pointer to aipu private struct
 * @reg_attr:        external register attribute
 * @clk_attr:        clock attribute
 * @disable_attr:    disable core attribute
 * @disable:         core disable flag (for debug usage)
 * @is_init:         init flag
 * @reset_lock:      soft reset mutex
 * @reset_delay_us:  soft reset delay in us
 * @dtcm_base:       DTCM base physical address
 * @dtcm_size:       DTCM size in bytes
 */
struct aipu_partition {
	u32 id;
	int arch;
	int version;
	int config;
	char name[10];
	int max_sched_num;
	struct device *dev;
	struct io_region *reg;
	struct aipu_operations *ops;
	struct aipu_irq_object *irq_obj;
	struct aipu_priv *priv;
	struct device_attribute *reg_attr;
	struct device_attribute *clk_attr;
	struct device_attribute *disable_attr;
	atomic_t disable;
	int is_init;
	struct mutex reset_lock; /* Protect soft reset */
	int reset_delay_us;
	u64 dtcm_base;
	u32 dtcm_size;
	u32 cluster_cnt;
	struct cluster_info clusters[8];
};

#endif /* __AIPU_PARTITION_H__ */
