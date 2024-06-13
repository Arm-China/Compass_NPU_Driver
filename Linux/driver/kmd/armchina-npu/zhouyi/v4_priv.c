// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#include <linux/platform_device.h>
#include <linux/of.h>
#include "aipu_priv.h"
#include "aipu_partition.h"
#include "aipu_common.h"
#include "zhouyi.h"
#include "v4.h"

static int init_aipu_cluster(struct aipu_partition *cluster, struct aipu_priv *aipu,
			     struct platform_device *p_dev, int version)
{
	int ret = 0;
	u32 val = 0;

	if (!cluster || !aipu || !p_dev)
		return -EINVAL;

	cluster->id = 0;
	cluster->priv = aipu;
	cluster->version = version;
	cluster->arch = AIPU_ARCH_ZHOUYI;
	cluster->dev = &p_dev->dev;
	cluster->reg = &cluster->priv->reg;
	cluster->irq_obj = cluster->priv->irq_obj;
	cluster->partition_mode = PARTITION_MODE_NONE;
	mutex_init(&cluster->reset_lock);
	cluster->ops = get_zhouyi_v4_ops();

	/* unused fields */
	cluster->reg_attr = NULL;
	cluster->clk_attr = NULL;
	cluster->disable_attr = NULL;
	cluster->config = 0;
	cluster->max_sched_num = 0;
	cluster->dtcm_base = 0;
	cluster->dtcm_size = 0;
	atomic_set(&cluster->disable, 0);

	cluster->cluster_cnt = 1;
	val = aipu_read32(cluster->reg, CLUSTER_CONFIG_REG_V4(0));
	cluster->clusters[0].core_cnt = GET_AIPU_CORE_NUM_V4(val);
	atomic_set(&cluster->clusters[0].en_core_cnt, GET_AIPU_CORE_NUM_V4(val));
	cluster->clusters[0].tec_cnt = GET_TEC_NUM_V4(val);

	/* get partition mode before initialize() */
	ret = of_property_count_u32_elems(p_dev->dev.of_node, "partition-mode");
	if (ret <= 0) {
		dev_info(&p_dev->dev, "use the no partition setting");
	} else {
		if (ret != PARTITION_MODE_CORE3_SCP)
			dev_warn(&p_dev->dev, "invalid partition mode: %d", ret);

		cluster->partition_mode = PARTITION_MODE_CORE3_SCP;
		dev_warn(&p_dev->dev, "use the core3 SCP partition setting");
	}

	cluster->ops->initialize(cluster);

#ifdef CONFIG_SYSFS
	if (IS_ERR(aipu_common_create_attr(cluster->dev, &cluster->reg_attr, "ext_registers",
					   0644, aipu_common_ext_register_sysfs_show,
					   aipu_common_ext_register_sysfs_store)))
		dev_err(cluster->dev, "[init_cluster] init sysfs <ext_registers> failed");
#endif

	cluster->is_init = true;
	return ret;
}

static struct aipu_partition *v4_create_cluster(struct aipu_priv *aipu,
						int id, struct platform_device *p_dev)
{
	int ret = 0;
	int version = 0;
	int config = 0;
	struct aipu_partition *cluster = NULL;

	if (!aipu || !p_dev)
		return ERR_PTR(-EINVAL);

	zhouyi_detect_aipu_version(p_dev, &version, &config, NULL);
	dev_info(&p_dev->dev, "AIPU detected: zhouyi-v4\n");

	WARN_ON(!aipu->is_init);

	cluster = devm_kzalloc(&p_dev->dev, sizeof(*cluster), GFP_KERNEL);
	if (!cluster)
		return ERR_PTR(-ENOMEM);

	/* do this before calling init_aipu_cluster() */
	ret = aipu_common_init_reg_irq(p_dev, cluster, &aipu->reg, &aipu->irq_obj);
	if (ret)
		return ERR_PTR(ret);

	ret = init_aipu_cluster(cluster, aipu, p_dev, version);
	if (ret)
		goto init_cluster_fail;

	aipu->partitions = cluster;
	aipu->partition_cnt = 1;
	aipu->cluster_cnt = 1;
	aipu_job_manager_set_partitions_info(&aipu->job_manager, aipu->partition_cnt, cluster);
	cluster->ops->print_hw_id_info(cluster);
	goto finish;

init_cluster_fail:
	aipu_destroy_irq_object(aipu->irq_obj);
	aipu->irq_obj = NULL;
	deinit_aipu_ioregion(&aipu->reg);
	aipu->reg.phys = 0;
	aipu->reg.kern = NULL;
	aipu->reg.size = 0;
	devm_kfree(&p_dev->dev, cluster);
	cluster = ERR_PTR(ret);

finish:
	return cluster;
}

static void v4_destroy_cluster(struct aipu_priv *aipu)
{
	int i = 0;

	if (aipu) {
		if (aipu->irq_obj)
			aipu_destroy_irq_object(aipu->irq_obj);
		aipu->irq_obj = NULL;
		if (aipu->reg.kern)
			deinit_aipu_ioregion(&aipu->reg);
		aipu->reg.kern = NULL;
		aipu->reg.phys = 0;
		aipu->reg.size = 0;
		for (i = 0; i < aipu->partition_cnt; i++)
			aipu_common_destroy_attr(aipu->partitions[i].dev,
						 &aipu->partitions[i].reg_attr);
	}
}

int v4_global_soft_reset(struct aipu_priv *aipu)
{
	/* to be implemented */
	return 0;
}

static struct aipu_priv_operations v4_priv_ops = {
	.create_partitions = v4_create_cluster,
	.destroy_partitions = v4_destroy_cluster,
	.global_soft_reset = v4_global_soft_reset,
};

struct aipu_priv_operations *get_v4_priv_ops(void)
{
	return &v4_priv_ops;
}
