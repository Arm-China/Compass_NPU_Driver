// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/irqreturn.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include "aipu_priv.h"
#include "x2.h"
#include "aipu_io.h"
#include "config.h"

static void zhouyi_x2_set_partition(struct aipu_partition *partition, u32 cluster_id)
{
	u32 nums = GET_NUMS(aipu_read32(partition->reg, CLUSTER_CONFIG_REG(cluster_id)));
	aipu_write32(partition->reg, CLUSTER_CONTROL_REG(cluster_id), ENABLE_CLUSTER(partition->id, nums));
}

static void zhouyi_x2_enable_interrupt(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;
	aipu_write32(partition->reg, CMD_POOL_INTR_CTRL_REG(cmd_pool_id),
		     EN_CORE_INTR | EN_CLUSTER_INTR | EN_ALL_TYPE_INTRS);
}

static void zhouyi_x2_config_partition_cmd_pool(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;
	aipu_write32(partition->reg, CMD_POOL_CONFIG_REG(cmd_pool_id), CONFIG_COMMAND_POOL(partition->id));
	zhouyi_x2_enable_interrupt(partition);
	aipu_write32(partition->reg, CMD_POOL_SECURE_REG_REG(cmd_pool_id), SET_NONSECURE_MODE);
}

static void zhouyi_x2_disable_interrupt(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;
	aipu_write32(partition->reg, CMD_POOL_INTR_CTRL_REG(cmd_pool_id), DISABLE_ALL_INTRS);
}

static void zhouyi_x2_trigger(struct aipu_partition *partition)
{
	/* no operation here */
}

static void zhouyi_x2_create_command_pool(struct aipu_partition *partition)
{
	if (IS_CMD_POOL_BUSY(aipu_read32(partition->reg, CMD_POOL_STATUS_REG(partition->id))))
		return;

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG, TSM_CREATE_CMD_POOL(partition->id, TSM_MAP_ALL));

	if (IS_CMD_FAIL(aipu_read32(partition->reg, TSM_STATUS_REG)))
		dev_err(partition->dev, "x2: create command pool #%d failed (cmd 0x%x)\n",
			partition->id, TSM_CREATE_CMD_POOL(partition->id, TSM_MAP_ALL));
	else
		dev_info(partition->dev, "x2: command pool #%d was created\n", partition->id);
}

static void zhouyi_x2_destroy_command_pool(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_LOW_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_INFO_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG, TSM_DESTROY_CMD_POOL(partition->id));
	dev_info(partition->dev, "x2: command pool #%d was destroyed\n", partition->id);
}

static void zhouyi_x2_abort_command_pool(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG, TSM_ABORT_CMD_POOL(partition->id));
	dev_dbg(partition->dev, "x2: command pool #%d was aborted\n", partition->id);
}

static int get_qos(u32 exec_flag)
{
	if (exec_flag & AIPU_JOB_EXEC_FLAG_QOS_FAST)
		return TSM_QOS_FAST;
	return TSM_QOS_SLOW;
}

static int zhouyi_x2_reserve(struct aipu_partition *partition, struct aipu_job_desc *udesc, int trigger_type)
{
	if (unlikely(!partition || !udesc))
		return -EINVAL;

	if (trigger_type == ZHOUYI_X2_TRIGGER_TYPE_DESTROY_CREATE)
		zhouyi_x2_destroy_command_pool(partition);

	if (trigger_type == ZHOUYI_X2_TRIGGER_TYPE_CREATE ||
	    trigger_type == ZHOUYI_X2_TRIGGER_TYPE_DESTROY_CREATE)
		zhouyi_x2_create_command_pool(partition);

	if (IS_CMD_POOL_IDLE(aipu_read32(partition->reg, CMD_POOL_STATUS_REG(partition->id)))) {
		dev_err(partition->dev, "x2: create command pool #%d failed (pool idle)\n", partition->id);
		return -EFAULT;
	}

	if (trigger_type == ZHOUYI_X2_TRIGGER_TYPE_CREATE ||
	    trigger_type == ZHOUYI_X2_TRIGGER_TYPE_DESTROY_CREATE) {
		aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, udesc->head_tcb_pa >> 32);
		aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_LOW_REG, (u32)udesc->head_tcb_pa);
	}

	dev_dbg(partition->dev, "[Job 0x%llx] x2 scheduler: TCB head 0x%llx\n",
		 udesc->job_id, udesc->head_tcb_pa);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_INFO_REG, (u16)udesc->job_id);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DISPATCH_CMD_POOL(partition->id, get_qos(udesc->exec_flag)));

	if (IS_CMD_FAIL(aipu_read32(partition->reg, TSM_STATUS_REG)))
		return -EFAULT;

	return 0;
}

static bool zhouyi_x2_is_idle(struct aipu_partition *partition)
{
	return !IS_CMD_POOL_FULL(aipu_read32(partition->reg, TSM_STATUS_REG));
}

static void zhouyi_x2_print_hw_id_info(struct aipu_partition *partition)
{
	struct aipu_priv *aipu = partition->priv;
	u32 iter = 0;

	dev_info(aipu->dev, "############# ZHOUYI X2 AIPU #############");
	dev_info(aipu->dev, "# Maximum Partition Count: %d", aipu->max_partition_cnt);
	dev_info(aipu->dev, "# Maximum Command Pool Count: %d", aipu->max_cmd_pool_cnt);
	dev_info(aipu->dev, "# Enabled Partition Count: %d", aipu->partition_cnt);
	dev_info(aipu->dev, "# Enabled Cluster Count: %d", aipu->cluster_cnt);

	for (iter = 0; iter < aipu->partition_cnt; iter++) {
		partition = &aipu->partitions[iter];
		dev_info(aipu->dev, "#\n");
		dev_info(aipu->dev, "# -- Partition #%u", partition->id);
		dev_info(aipu->dev, "# Cluster Count: %d", partition->cluster_cnt);
		dev_info(aipu->dev, "# Core Count per Cluster: %d",
			 partition->clusters[0].core_cnt);
		dev_info(aipu->dev, "# TEC Count per Core: %d",
			 partition->clusters[0].tec_cnt);
	}
	dev_info(aipu->dev, "##########################################");
}

static int zhouyi_x2_io_rw(struct aipu_partition *partition, struct aipu_io_req *io_req)
{
	if (unlikely(!io_req))
		return -EINVAL;

	if (!partition || io_req->offset > ZHOUYI_X2_MAX_REG_OFFSET)
		return -EINVAL;

	zhouyi_io_rw(partition->reg, io_req);
	return 0;
}

static void zhouyi_x2_initialize(struct aipu_partition *partition)
{
	partition->ops->enable_interrupt(partition);
}

static int partition_upper_half(struct aipu_partition *partition)
{
	struct aipu_priv *aipu = partition->priv;
	u32 i = 0;
	struct job_irq_info info;
	u32 status = 0;
	u32 tag_id = 0;
	info.tail_tcbp = aipu_read32(&aipu->reg, CMD_POOL_INTR_TCB_PTR_REG(partition->id));

	for (i = 0; i < aipu->partition_cnt; i++) {
		partition = &aipu->partitions[i];
		status = aipu_read32(&aipu->reg, CMD_POOL_INTR_STATUS_REG(partition->id));
		if (IS_CMD_POOL_IRQ(status))
			break;
	}

	info.cluster_id = GET_INTR_CLUSTER_ID(status);
	info.core_id = GET_INTR_CORE_ID(status);
	info.tec_id = GET_INTR_TEC_ID(status);

	if (IS_ABNORMAL(status)) {
		tag_id = aipu_read32(partition->reg, CMD_POOL_FIRST_BAD_CMD_REG(partition->id));
		info.tag_id = IS_BAD_TAG_ID_VALID(tag_id) ? GET_FIRST_BAD_TAG_ID(tag_id) : 0;
		aipu_write32(partition->reg, CMD_POOL_FIRST_BAD_CMD_REG(partition->id), 0);
	} else {
		info.tag_id = 0;
	}

	if (IS_DONE_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id), CLEAR_CMD_POOL_DONE);
	} else if (IS_WAIT_IRQ(status)){
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id), CLEAR_CMD_POOL_WAIT);
		return IRQ_HANDLED;
	} else if (IS_SIGNAL_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id), CLEAR_CMD_POOL_SIGNAL);
	} else if (IS_ERROR_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id), CLEAR_CMD_POOL_ERROR);
		zhouyi_x2_abort_command_pool(partition);
	} else if (IS_FAULT_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id), CLEAR_CMD_POOL_FAULT);
	} else if (IS_EXCEPTION_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id), CLEAR_CMD_POOL_EXCEPTION);
	}

	aipu_job_manager_irq_upper_half(partition, GET_INTR_TYPE(status), &info);
	aipu_irq_schedulework(partition->irq_obj);
	return IRQ_HANDLED;
}

static int zhouyi_x2_upper_half(void *data)
{
	struct aipu_partition *partition = (struct aipu_partition *)data;
	struct aipu_priv *aipu = partition->priv;
	int iter = 0;

	for (iter = 0; iter < aipu->partition_cnt; iter++)
		partition_upper_half(&aipu->partitions[iter]);

	return IRQ_HANDLED;
}

static void zhouyi_x2_bottom_half(void *data)
{
	aipu_job_manager_irq_bottom_half(data);
}

#ifdef CONFIG_SYSFS
static int zhouyi_x2_sysfs_show(struct aipu_partition *partition, char *buf)
{
	/* TBD */
	return 0;
}
#endif

int zhouyi_x2_soft_reset(struct aipu_partition *partition, bool init_regs)
{
	/* NOTE: this is a global soft-reset, not per-partition! */
	struct aipu_priv *aipu = partition->priv;
	return aipu->ops->global_soft_reset(aipu);
}

static struct aipu_operations zhouyi_x2_ops = {
	.get_config = NULL,
	.enable_interrupt = zhouyi_x2_enable_interrupt,
	.disable_interrupt = zhouyi_x2_disable_interrupt,
	.trigger = zhouyi_x2_trigger,
	.reserve = zhouyi_x2_reserve,
	.is_idle = zhouyi_x2_is_idle,
	.print_hw_id_info = zhouyi_x2_print_hw_id_info,
	.io_rw = zhouyi_x2_io_rw,
	.upper_half = zhouyi_x2_upper_half,
	.bottom_half = zhouyi_x2_bottom_half,
#ifdef CONFIG_SYSFS
	.sysfs_show = zhouyi_x2_sysfs_show,
#endif
	.soft_reset = zhouyi_x2_soft_reset,
	.initialize = zhouyi_x2_initialize,
	.set_partition = zhouyi_x2_set_partition,
	.config_partition_cmd_pool = zhouyi_x2_config_partition_cmd_pool,
	.destroy_command_pool = zhouyi_x2_destroy_command_pool,
};

struct aipu_operations *get_zhouyi_x2_ops(void)
{
	return &zhouyi_x2_ops;
}
