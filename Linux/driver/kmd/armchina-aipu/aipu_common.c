// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of.h>
#include "aipu_common.h"
#include "aipu_partition.h"

#define MAX_CHAR_SYSFS 4096

#ifdef CONFIG_SYSFS
ssize_t aipu_common_ext_register_sysfs_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	int ret = 0;
	char tmp[512];
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	if (unlikely(!partition))
		return 0;

	if (get_soc_ops(partition) &&
	    get_soc_ops(partition)->is_clk_enabled &&
	    !get_soc_ops(partition)->is_clk_enabled(dev, get_soc(partition))) {
		return snprintf(buf, MAX_CHAR_SYSFS,
		    "AIPU is suspended and external registers cannot be read!\n");
	}

	ret += snprintf(tmp, 1024, "----------------------------------------\n");
	strcat(buf, tmp);
	ret += snprintf(tmp, 1024, "   AIPU External Register Values\n");
	strcat(buf, tmp);
	ret += snprintf(tmp, 1024, "----------------------------------------\n");
	strcat(buf, tmp);
	ret += snprintf(tmp, 1024, "%-*s%-*s%-*s\n", 8, "Offset", 22, "Name", 10, "Value");
	strcat(buf, tmp);
	ret += snprintf(tmp, 1024, "----------------------------------------\n");
	strcat(buf, tmp);
	ret += partition->ops->sysfs_show(partition, buf);
	ret += snprintf(tmp, 1024, "----------------------------------------\n");
	strcat(buf, tmp);

	return ret;
}

ssize_t aipu_common_ext_register_sysfs_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	int i = 0;
	int ret = 0;
	char *token = NULL;
	char *buf_dup = NULL;
	int value[3] = { 0 };
	struct aipu_io_req io_req;
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	if (get_soc_ops(partition) &&
	    get_soc_ops(partition)->is_clk_enabled &&
	    !get_soc_ops(partition)->is_clk_enabled(dev, get_soc(partition)))
		return 0;

	buf_dup = kzalloc(1024, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;
	snprintf(buf_dup, 1024, buf);

	for (i = 0; i < 3; i++) {
		token = strsep(&buf_dup, "-");
		if (!token) {
			dev_err(dev, "[SYSFS] please echo as this format: <reg_offset>-<write time>-<write value>");
			goto out_free_buffer;
		}

		dev_dbg(dev, "[SYSFS] to convert str: %s", token);

		ret = kstrtouint(token, 0, &value[i]);
		if (ret) {
			dev_err(dev, "[SYSFS] convert str to int failed (%d): %s", ret, token);
			goto out_free_buffer;
		}
	}

	dev_dbg(dev, "[SYSFS] offset 0x%x, time 0x%x, value 0x%x",
		value[0], value[1], value[2]);

	io_req.rw = AIPU_IO_WRITE;
	io_req.offset = value[0];
	io_req.value = value[2];
	for (i = 0; i < value[1]; i++) {
		dev_info(dev, "[SYSFS] write 0x%x into register offset 0x%x", value[2], value[0]);
		partition->ops->io_rw(partition, &io_req);
	}

out_free_buffer:
	kfree(buf_dup);
	return count;
}

ssize_t aipu_common_clock_sysfs_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	/*
	 * If SoC level provides no clock operations,
	 * the state of AIPU is by default treated as normal.
	 */
	if (get_soc_ops(partition) &&
	    get_soc_ops(partition)->is_clk_enabled &&
	    !get_soc_ops(partition)->is_clk_enabled(dev, get_soc(partition)))
		return snprintf(buf, MAX_CHAR_SYSFS,
				"AIPU is in clock gating state and suspended.\n");
	else
		return snprintf(buf, MAX_CHAR_SYSFS, "AIPU is in normal working state.\n");
}

ssize_t aipu_common_clock_sysfs_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int do_suspend = 0;
	int do_resume = 0;
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	if (unlikely(!partition))
		return count;

	if (!get_soc_ops(partition) ||
	    !get_soc_ops(partition)->enable_clk ||
	    !get_soc_ops(partition)->disable_clk ||
	    !get_soc_ops(partition)->is_clk_enabled) {
		dev_info(dev, "operation is not supported.\n");
		return count;
	}

	if ((strncmp(buf, "1", 1) == 0))
		do_suspend = 1;
	else if ((strncmp(buf, "0", 1) == 0))
		do_resume = 1;

	if (get_soc_ops(partition)->is_clk_enabled(dev, get_soc(partition)) &&
	    partition->ops->is_idle(partition) && do_suspend) {
		dev_info(dev, "disable clock\n");
		get_soc_ops(partition)->disable_clk(partition->dev, get_soc(partition));
	} else if (!get_soc_ops(partition)->is_clk_enabled(dev, get_soc(partition)) && do_resume) {
		dev_info(dev, "enable clock\n");
		get_soc_ops(partition)->enable_clk(partition->dev, get_soc(partition));
	} else {
		dev_err(dev, "operation cannot be completed!\n");
	}

	return count;
}

ssize_t aipu_common_disable_sysfs_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	if (atomic_read(&partition->disable)) {
		return snprintf(buf, MAX_CHAR_SYSFS,
		    "AIPU partition #%d is disabled (echo 0 > /sys/devices/platform/[dev]/disable to enable it).\n",
		    partition->id);
	} else {
		return snprintf(buf, MAX_CHAR_SYSFS,
		    "AIPU partition #%d is enabled (echo 1 > /sys/devices/platform/[dev]/disable to disable it).\n",
		    partition->id);
	}
}

ssize_t aipu_common_disable_sysfs_store(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t count)
{
	int do_disable = 0;
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	if ((strncmp(buf, "1", 1) == 0))
		do_disable = 1;
	else if ((strncmp(buf, "0", 1) == 0))
		do_disable = 0;
	else
		do_disable = -1;

	if (atomic_read(&partition->disable) && !do_disable) {
		dev_info(dev, "enable partition...\n");
		atomic_set(&partition->disable, 0);
	} else if (!atomic_read(&partition->disable) && do_disable) {
		dev_info(dev, "disable partition...\n");
		atomic_set(&partition->disable, 1);
	}

	return count;
}

struct device_attribute *aipu_common_create_attr(struct device *dev,
						      struct device_attribute **attr,
						      const char *name, int mode,
						      sysfs_show_t show, sysfs_store_t store)
{
	if (!dev || !attr || !name)
		return ERR_PTR(-EINVAL);

	*attr = kzalloc(sizeof(*attr), GFP_KERNEL);
	if (!*attr)
		return ERR_PTR(-ENOMEM);

	(*attr)->attr.name = name;
	(*attr)->attr.mode = mode;
	(*attr)->show = show;
	(*attr)->store = store;
	device_create_file(dev, *attr);

	return *attr;
}

void aipu_common_destroy_attr(struct device *dev, struct device_attribute **attr)
{
	if (!dev || !attr || !*attr)
		return;

	device_remove_file(dev, *attr);
	kfree(*attr);
	*attr = NULL;
}
#endif

int aipu_common_init_reg_irq(struct platform_device *p_dev, struct aipu_partition *partition,
			     struct io_region *reg, struct aipu_irq_object **irq_obj)
{
	int ret = 0;
	struct resource *res = NULL;
	u64 base = 0;
	u64 size = 0;

	if (!p_dev || !reg || !irq_obj)
		return -EINVAL;

	res = platform_get_resource(p_dev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&p_dev->dev, "get aipu IO region failed\n");
		return -EINVAL;
	}
	base = res->start;
	size = res->end - res->start + 1;
	dev_dbg(&p_dev->dev, "get aipu IO region: [0x%llx, 0x%llx]\n", base, res->end);

	ret = init_aipu_ioregion(reg, base, size);
	if (ret) {
		dev_err(&p_dev->dev,
			"create aipu IO region failed: base 0x%llx, size 0x%llx\n", base, size);
		return ret;
	}
	dev_dbg(&p_dev->dev, "init aipu IO region done: [0x%llx, 0x%llx]\n", base, res->end);

	res = platform_get_resource(p_dev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&p_dev->dev, "get aipu IRQ number failed\n");
		ret = -EINVAL;
		goto init_irq_fail;
	}
	dev_dbg(&p_dev->dev, "get aipu IRQ number: 0x%x\n", (int)res->start);

	*irq_obj = aipu_create_irq_object(&p_dev->dev, res->start, partition, "aipu");
	if (!*irq_obj) {
		dev_err(&p_dev->dev, "create IRQ object failed: IRQ 0x%x\n", (int)res->start);
		ret = -EFAULT;
		goto init_irq_fail;
	}
	dev_dbg(&p_dev->dev, "init aipu IRQ done\n");

	/* success */
	goto finish;

init_irq_fail:
	deinit_aipu_ioregion(reg);
	reg->kern = NULL;
	reg->phys = 0;
	reg->size = 0;

finish:
	return ret;
}
