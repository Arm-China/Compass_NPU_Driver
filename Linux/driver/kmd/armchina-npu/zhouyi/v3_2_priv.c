// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include "aipu_priv.h"
#include "aipu_partition.h"
#include "aipu_common.h"
#include "zhouyi.h"
#include "v3_2.h"

extern void gpiod_set_value(struct gpio_desc *desc, int value);
extern int gpiod_get_value(const struct gpio_desc *desc);

static int aipu_init_debugger_reg_v3_2(struct platform_device *p_dev, struct io_region *reg)
{
	int ret = 0;
	struct resource *res = NULL;
	u64 base = 0;
	u64 size = 0;

	if (!p_dev || !reg)
		return -EINVAL;

	res = platform_get_resource(p_dev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	dev_dbg(&p_dev->dev, "init aipu debugger IO region: [0x%llx, 0x%llx]\n",
		res->start, res->end - res->start + 1);

	base = res->start;
	size = res->end - res->start + 1;

	ret = init_aipu_ioregion(reg, base, size);
	if (ret) {
		dev_err(&p_dev->dev, "create aipu IO region failed.\n");
		return ret;
	}
	dev_dbg(&p_dev->dev, "init aipu IO region done: [0x%llx, 0x%llx]\n",
		base, res->end);
	return ret;
}

static u32 aipu_get_gm_size(u32 val)
{
    val &= 0x7;
    return val ? SZ_1M << (val - 1) : 0;
}

static int init_aipu_cluster(struct aipu_partition *cluster, struct aipu_priv *aipu,
			     struct platform_device *p_dev, int version)
{
	int ret = 0;
	u32 val = 0;
	u32 config = 0;

	if (!cluster || !aipu || !p_dev)
		return -EINVAL;

	cluster->id = 0;
	cluster->priv = aipu;
	cluster->version = version;
	cluster->arch = AIPU_ARCH_ZHOUYI;
	cluster->dev = &p_dev->dev;
	cluster->reg = &cluster->priv->reg;
	cluster->dbg_reg = &cluster->priv->dbg_reg;
	cluster->irq_obj = cluster->priv->irq_obj;
	cluster->partition_mode = PARTITION_MODE_NONE;
	mutex_init(&cluster->reset_lock);
	spin_lock_init(&cluster->io_lock);
	cluster->ops = get_zhouyi_v3_2_ops();

	/* unused fields */
	cluster->reg_attr = NULL;
	cluster->clk_attr = NULL;
	cluster->disable_attr = NULL;
	cluster->max_sched_num = 0;
	cluster->dtcm_base = 0;
	cluster->dtcm_size = 0;
	atomic_set(&cluster->disable, 0);

	cluster->cluster_cnt = 1;
	val = aipu_read32(cluster->reg, CLUSTER_CONFIG_REG_V3_2(0));
	cluster->clusters[0].core_cnt = GET_AIPU_CORE_NUM_V3_2(val);
	atomic_set(&cluster->clusters[0].en_core_cnt, GET_AIPU_CORE_NUM_V3_2(val));
	config = GET_AIFF_NUM_V3_2(val);
	if (config == 2)
		cluster->config = 1304;
	else if (config == 1)
		cluster->config = 1204;
	else
		cluster->config = 0;
	cluster->clusters[0].tec_cnt = GET_TEC_NUM_V3_2(val);

	if (version >= AIPU_ISA_VERSION_ZHOUYI_V3_2_0) {
		aipu_write32(&aipu->reg, AHB_INTERNAL_CSR_SELECTION_CTRL_REG,
		     SELECT_DEBUG_CORE_V3_2(0, 0));
		val = aipu_read32(cluster->reg, CLUSTER_GM_FEATURE_V3_2(0));
		cluster->clusters[0].gm_bytes = aipu_get_gm_size(val);
		aipu_write32(&aipu->reg, AHB_INTERNAL_CSR_SELECTION_CTRL_REG, DISABLE_DEBUG_V3_2);
		ret = aipu_mm_init_gm(&cluster->priv->mm, cluster->clusters[0].gm_bytes);
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

static struct aipu_partition *v3_2_create_cluster(struct aipu_priv *aipu,
						  int id,
						  struct platform_device *p_dev)
{
	int ret = 0;
	int version = 0;
	int config = 0;
	struct aipu_partition *cluster = NULL;

	if (!aipu || !p_dev)
		return ERR_PTR(-EINVAL);

	ret = zhouyi_detect_aipu_version(p_dev, &version, &config, NULL);
	if (ret)
		return ERR_PTR(ret);

	dev_info(&p_dev->dev, "AIPU detected: zhouyi%s\n",
		 version == AIPU_ISA_VERSION_ZHOUYI_V3_2_0 ? "-v3_2_0":
		 version == AIPU_ISA_VERSION_ZHOUYI_V3_2_1 ? "-v3_2_1":"");

	WARN_ON(!aipu->is_init);

	cluster = devm_kzalloc(&p_dev->dev, sizeof(*cluster), GFP_KERNEL);
	if (!cluster)
		return ERR_PTR(-ENOMEM);

	/* do this before calling init_aipu_cluster() */
	ret = aipu_common_init_reg_irq(p_dev, cluster, &aipu->reg, &aipu->irq_obj);
	if (ret)
		return ERR_PTR(ret);

	/*for debugger io region */
	ret = aipu_init_debugger_reg_v3_2(p_dev, &aipu->dbg_reg);
	if (ret) {
		dev_err(&p_dev->dev, "[init debugger] failed, please update fpga and dts.");
		//return ERR_PTR(ret);
	}

	if (aipu->reset_gpio) {
		aipu->ops->global_hw_reset(aipu);
	} else {
		ret = aipu->ops->global_soft_reset(aipu);
		if (ret) {
			dev_err(&p_dev->dev, "[init partition] global soft reset failed");
			return ERR_PTR(ret);
		}
	}

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
	deinit_aipu_ioregion(&aipu->dbg_reg);
	aipu->reg.phys = 0;
	aipu->reg.kern = NULL;
	aipu->reg.size = 0;
	devm_kfree(&p_dev->dev, cluster);
	cluster = ERR_PTR(ret);

finish:
	return cluster;
}

static void v3_2_destroy_cluster(struct aipu_priv *aipu)
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

		if (aipu->dbg_reg.kern)
			deinit_aipu_ioregion(&aipu->dbg_reg);
		aipu->dbg_reg.kern = NULL;
		aipu->dbg_reg.phys = 0;
		aipu->dbg_reg.size = 0;

		for (i = 0; i < aipu->partition_cnt; i++) {
			if (!aipu->partitions[i].is_init)
				return;
			aipu_common_destroy_attr(aipu->partitions[i].dev,
						 &aipu->partitions[i].reg_attr);
		}
	}
}

static int v3_2_global_soft_reset(struct aipu_priv *aipu)
{
	return zhouyi_soft_reset(&aipu->reg, PMU_TOP_SOFT_RESET_REG, aipu->reset_delay_us);
}

static void v3_2_global_hw_reset(struct aipu_priv *aipu)
{
	if (aipu && aipu->reset_gpio) {
		gpiod_set_value(aipu->reset_gpio, 0);
		udelay(1);
		gpiod_set_value(aipu->reset_gpio, 1);
		udelay(20);
		gpiod_set_value(aipu->reset_gpio, 0);
		udelay(20);
		dev_dbg(aipu->dev, "v3_2 global hw reset finished.\n");
	}
}

static struct aipu_priv_operations v3_2_priv_ops = {
	.create_partitions = v3_2_create_cluster,
	.destroy_partitions = v3_2_destroy_cluster,
	.global_soft_reset = v3_2_global_soft_reset,
	.global_hw_reset = v3_2_global_hw_reset,
};

struct aipu_priv_operations *get_v3_2_priv_ops(void)
{
	return &v3_2_priv_ops;
}
