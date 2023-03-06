/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __AIPU_SOC_H__
#define __AIPU_SOC_H__

#include <linux/platform_device.h>
#include <linux/device.h>
#include <armchina_aipu.h>

/**
 * struct aipu_soc - a struct contains AIPU SoC specific information
 * @priv: SoC private data structure
 * @mm:   Pointer to struct aipu_memory_manager
 *
 * This struct contains reference to SoC level private data, which is registered while probing,
 * and used as arguments of the corresponding SoC operation methods.
 */
struct aipu_soc {
	void *priv;
	void *mm;
};

/**
 * struct aipu_soc_operations - a struct contains SoC operation methods
 * @start_bw_profiling: start bandwidth profiling
 * @stop_bw_profiling:  stop bandwidth profiling
 * @read_profiling_reg: read profiling register values
 * @enable_clk:         enable clock/disable clock gating
 * @disable_clk:        disable clock/enable clock gating
 * @is_clk_enabled:     is in clock enabled or disabled
 * @is_aipu_irq:        is the shared interrupt is for an AIPU core or not
 * @init_mm:            customized mm initialization of SoC vendors
 * @deinit_mm:          customized mm de-initilization of SoC vendors
 * @malloc:             customized mm allocation of SoC vendors
 * @free:               customized mm free of SoC vendors
 * @mmap:               customized mm mmap of SoC vendors
 *
 * SoC vendors should register the SoC operations into struct aipu_private while
 * probing if they would like to implement and use their private SoC operation methods.
 */
struct aipu_soc_operations {
	void (*start_bw_profiling)(struct device *dev, struct aipu_soc *soc);
	void (*stop_bw_profiling)(struct device *dev, struct aipu_soc *soc);
	void (*read_profiling_reg)(struct device *dev, struct aipu_soc *soc,
				   struct aipu_ext_profiling_data *pdata);
	int (*enable_clk)(struct device *dev, struct aipu_soc *soc);
	int (*disable_clk)(struct device *dev, struct aipu_soc *soc);
	bool (*is_clk_enabled)(struct device *dev, struct aipu_soc *soc);
	bool (*is_aipu_irq)(struct device *dev, struct aipu_soc *soc, int core_id);
	int (*init_mm)(struct device *dev, struct aipu_soc *soc);
	int (*deinit_mm)(struct device *dev, struct aipu_soc *soc);
	int (*malloc)(struct device *dev, struct aipu_soc *soc,
		      struct aipu_buf_request *buf_req);
	int (*free)(struct device *dev, struct aipu_soc *soc, struct aipu_buf_desc *buf);
	int (*mmap)(struct device *dev, struct aipu_soc *soc, struct vm_area_struct *vma);
};

int armchina_aipu_probe(struct platform_device *p_dev, struct aipu_soc *soc,
			struct aipu_soc_operations *ops);
int armchina_aipu_remove(struct platform_device *p_dev);
int armchina_aipu_suspend(struct platform_device *p_dev, pm_message_t state);
int armchina_aipu_resume(struct platform_device *p_dev);

#endif /* __AIPU_SOC_H__ */
