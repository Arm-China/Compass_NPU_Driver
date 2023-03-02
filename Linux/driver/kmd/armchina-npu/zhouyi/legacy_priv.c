// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/platform_device.h>
#include <linux/of.h>
#include "aipu_job_manager.h"
#include "aipu_mm.h"
#include "aipu_priv.h"
#include "aipu_common.h"
#include "zhouyi.h"
#include "z1.h"
#include "x1.h"

static int init_aipu_core(struct aipu_partition *core, int version, int id, struct aipu_priv *priv,
			  struct platform_device *p_dev)
{
	int ret = 0;

	if (!core || !p_dev || !priv)
		return -EINVAL;

	WARN_ON(core->is_init);
	WARN_ON(version != AIPU_ISA_VERSION_ZHOUYI_Z1 &&
		version != AIPU_ISA_VERSION_ZHOUYI_Z2 &&
		version != AIPU_ISA_VERSION_ZHOUYI_Z3 &&
		version != AIPU_ISA_VERSION_ZHOUYI_X1);

	core->version = version;
	core->id = id;
	core->dev = &p_dev->dev;
	core->priv = priv;
	atomic_set(&core->disable, 0);
	snprintf(core->name, sizeof(core->name), "aipu%d", id);
	mutex_init(&core->reset_lock);

	/* unused fields */
	core->cluster_cnt = 0;

#ifdef CONFIG_ARMCHINA_NPU_ARCH_Z1
	if (version == AIPU_ISA_VERSION_ZHOUYI_Z1) {
		core->max_sched_num = ZHOUYI_V1_MAX_SCHED_JOB_NUM;
		core->ops = get_zhouyi_v1_ops();
	}
#endif

#ifdef CONFIG_ARMCHINA_NPU_ARCH_X1
	if (version == AIPU_ISA_VERSION_ZHOUYI_Z2 ||
	    version == AIPU_ISA_VERSION_ZHOUYI_Z3 ||
	    version == AIPU_ISA_VERSION_ZHOUYI_X1) {
		core->max_sched_num = ZHOUYI_X1_MAX_SCHED_JOB_NUM;
		core->ops = get_zhouyi_x1_ops();
	}
#endif

	core->reg = devm_kzalloc(core->dev, sizeof(*core->reg), GFP_KERNEL);
	if (!core->reg)
		return -ENOMEM;

	ret = aipu_common_init_reg_irq(p_dev, core, core->reg, &core->irq_obj);
	if (ret)
		return ret;

	get_dtcm(&priv->mm, &core->dtcm_base, &core->dtcm_size);

#ifdef CONFIG_SYSFS
	if (IS_ERR(aipu_common_create_attr(core->dev, &core->reg_attr, "ext_registers", 0644,
					 aipu_common_ext_register_sysfs_show,
					 aipu_common_ext_register_sysfs_store))) {
		dev_err(core->dev, "[init_core] create sysfs attribute failed: ext_registers");
		ret = -EFAULT;
		goto init_sysfs_fail;
	}

	if (priv->soc_ops &&
	    priv->soc_ops->enable_clk && priv->soc_ops->disable_clk &&
	    IS_ERR(aipu_common_create_attr(core->dev, &core->clk_attr, "soc_clock", 0644,
					 aipu_common_clock_sysfs_show,
					 aipu_common_clock_sysfs_store))) {
		dev_err(core->dev, "[init_core] create sysfs attribute failed: soc_clock");
		ret = -EFAULT;
		goto init_sysfs_fail;
	}

	if (IS_ERR(aipu_common_create_attr(core->dev, &core->disable_attr, "disable", 0644,
					 aipu_common_disable_sysfs_show,
					 aipu_common_disable_sysfs_store))) {
		dev_err(core->dev, "[init_core] create sysfs attribute failed: disable");
		ret = -EFAULT;
		goto init_sysfs_fail;
	}
#else
	core->reg_attr = NULL;
	core->clk_attr = NULL;
	core->disable_attr = NULL;
#endif

	core->arch = AIPU_ARCH_ZHOUYI;
	core->config = core->ops->get_config(core);
	core->reset_delay_us = AIPU_CONFIG_DEFAULT_RESET_DELAY_US;

	ret = core->ops->soft_reset(core, true);
	if (ret) {
		dev_err(core->dev, "[init_core] soft reset failed");
		goto soft_reset_fail;
	}
	core->ops->print_hw_id_info(core);

	core->is_init = 1;
	goto finish;

soft_reset_fail:
#ifdef CONFIG_SYSFS
init_sysfs_fail:
	aipu_common_destroy_attr(core->dev, &core->reg_attr);
	aipu_common_destroy_attr(core->dev, &core->clk_attr);
	aipu_common_destroy_attr(core->dev, &core->disable_attr);
#endif
	aipu_destroy_irq_object(core->irq_obj);
	deinit_aipu_ioregion(core->reg);

finish:
	return ret;
}

/**
 * @deinit_aipu_core() - deinit a created aipu_partition struct
 * @core: pointer to struct aipu_common initialized in init_aipu_core()
 */
static void deinit_aipu_core(struct aipu_partition *core)
{
	if (!core)
		return;

	core->ops->disable_interrupt(core);
	deinit_aipu_ioregion(core->reg);

	if (core->irq_obj) {
		aipu_destroy_irq_object(core->irq_obj);
		core->irq_obj = NULL;
	}

#ifdef CONFIG_SYSFS
	aipu_common_destroy_attr(core->dev, &core->reg_attr);
	aipu_common_destroy_attr(core->dev, &core->clk_attr);
	aipu_common_destroy_attr(core->dev, &core->disable_attr);
#endif
	core->is_init = 0;
}

static struct aipu_partition *legacy_create_partitions(struct aipu_priv *aipu,
						       int id, struct platform_device *p_dev)
{
	int ret = 0;
	struct aipu_partition *partition = NULL;
	struct aipu_partition *new_partition_arr = NULL;
	int version = 0;
	int config = 0;

	if (!aipu || !p_dev)
		return ERR_PTR(-EINVAL);

	WARN_ON(!aipu->is_init);

	partition = devm_kzalloc(&p_dev->dev, sizeof(*partition), GFP_KERNEL);
	if (!partition)
		return ERR_PTR(-ENOMEM);

	zhouyi_detect_aipu_version(p_dev, &version, &config);
	if (version == AIPU_ISA_VERSION_ZHOUYI_X1)
		dev_info(&p_dev->dev, "AIPU core #%d detected: zhouyi-x1-%04d\n", id, config);
	else if (version == AIPU_ISA_VERSION_ZHOUYI_Z1 ||
		 version == AIPU_ISA_VERSION_ZHOUYI_Z2 ||
		 version == AIPU_ISA_VERSION_ZHOUYI_Z3)
		dev_info(&p_dev->dev, "AIPU core #%d detected: zhouyi-z%d-%04d\n", id, version, config);
	else
		return ERR_PTR(-EINVAL);

	ret = init_aipu_core(partition, version, id, aipu, p_dev);
	if (ret)
		return ERR_PTR(ret);

	new_partition_arr = kcalloc(aipu->partition_cnt + 1, sizeof(*new_partition_arr), GFP_KERNEL);
	if (!new_partition_arr) {
		partition = ERR_PTR(-ENOMEM);
		goto err_handle;
	}

	if (aipu->partition_cnt) {
		WARN_ON(!aipu->partitions);
		memcpy(new_partition_arr, aipu->partitions, aipu->partition_cnt * sizeof(*new_partition_arr));
		kfree(aipu->partitions);
		aipu->partitions = NULL;
	}

	new_partition_arr[aipu->partition_cnt] = *partition;
	aipu->partitions = new_partition_arr;
	aipu->partition_cnt++;

	aipu_job_manager_set_partitions_info(&aipu->job_manager, aipu->partition_cnt, aipu->partitions);

	goto finish;

err_handle:
	deinit_aipu_core(partition);

finish:
	return partition;
}

static void legacy_destroy_partitions(struct aipu_priv *aipu)
{
	int par_iter = 0;

	for (par_iter = 0; par_iter < aipu->partition_cnt; par_iter++)
		deinit_aipu_core(&aipu->partitions[par_iter]);
}

static struct aipu_priv_operations legacy_priv_ops = {
	.create_partitions = legacy_create_partitions,
	.destroy_partitions = legacy_destroy_partitions,
	.global_soft_reset = NULL,
};

struct aipu_priv_operations *get_legacy_priv_ops(void)
{
	return &legacy_priv_ops;
}
