/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_COMMON_H__
#define __AIPU_COMMON_H__

#include "aipu_priv.h"
#include "aipu_io.h"
#include "aipu_irq.h"

#ifdef CONFIG_SYSFS
typedef ssize_t (*sysfs_show_t)(struct device *dev, struct device_attribute *attr, char *buf);
typedef ssize_t (*sysfs_store_t)(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count);

ssize_t aipu_common_ext_register_sysfs_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf);
ssize_t aipu_common_ext_register_sysfs_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count);
ssize_t aipu_common_clock_sysfs_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf);
ssize_t aipu_common_clock_sysfs_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count);
ssize_t aipu_common_disable_sysfs_show(struct device *dev, struct device_attribute *attr,
				       char *buf);
ssize_t aipu_common_disable_sysfs_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count);
struct device_attribute *aipu_common_create_attr(struct device *dev,
						 struct device_attribute **attr,
						 const char *name, int mode,
						 sysfs_show_t show, sysfs_store_t store);
void aipu_common_destroy_attr(struct device *dev, struct device_attribute **attr);
#endif
int aipu_common_init_reg_irq(struct platform_device *p_dev, struct aipu_partition *partition,
			     struct io_region *reg, struct aipu_irq_object **irq_obj);

#endif /* __AIPU_COMMON_H__ */
