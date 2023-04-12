// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/irqreturn.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "aipu_priv.h"
#include "v3.h"
#include "aipu_io.h"
#include "config.h"

static void zhouyi_v3_set_partition(struct aipu_partition *partition, u32 cluster_id)
{
	u32 nums = GET_NUMS(aipu_read32(partition->reg, CLUSTER_CONFIG_REG(cluster_id)));

	aipu_write32(partition->reg, CLUSTER_CONTROL_REG(cluster_id),
		     ENABLE_CLUSTER(partition->id, nums));
}

static void zhouyi_v3_enable_core_cnt(struct aipu_partition *partition, u32 cluster_id,
				      u32 en_core_cnt)
{
	u32 nums = GET_NUMS(aipu_read32(partition->reg, CLUSTER_CONTROL_REG(cluster_id)));
	u32 en_aiff_cnt = GET_AIFF_NUM(nums);
	u32 en_tec_cnt = GET_TEC_NUM(nums);

	aipu_write32(partition->reg, CLUSTER_CONTROL_REG(cluster_id),
		     CONFIG_CLUSTER(partition->id, en_core_cnt, en_aiff_cnt, en_tec_cnt));
	dev_info(partition->dev, "configure cluster #%u done: en_core_cnt %u\n",
		 cluster_id, en_core_cnt);
}

static void zhouyi_v3_enable_interrupt(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;

	aipu_write32(partition->reg, CMD_POOL_INTR_CTRL_REG(cmd_pool_id),
		     EN_TEC_INTR | EN_CORE_INTR | EN_CLUSTER_INTR | EN_ALL_TYPE_INTRS);
}

static void zhouyi_v3_config_partition_cmd_pool(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;

	aipu_write32(partition->reg, CMD_POOL_CONFIG_REG(cmd_pool_id),
		     CONFIG_COMMAND_POOL(partition->id));
	aipu_write32(partition->reg, CMD_POOL_SECURE_REG_REG(cmd_pool_id), SET_NONSECURE_MODE);
}

static void zhouyi_v3_disable_interrupt(struct aipu_partition *partition)
{
	u32 cmd_pool_id = partition->id;

	aipu_write32(partition->reg, CMD_POOL_INTR_CTRL_REG(cmd_pool_id), DISABLE_ALL_INTRS);
}

static void zhouyi_v3_trigger(struct aipu_partition *partition)
{
	/* no operation here */
}

static int zhouyi_v3_create_command_pool(struct aipu_partition *partition)
{
	u32 status = 0;

	if (IS_CMD_POOL_BUSY(aipu_read32(partition->reg, CMD_POOL_STATUS_REG(partition->id))))
		return 0;

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_CREATE_CMD_POOL(partition->id, TSM_MAP_ALL));

	status = aipu_read32(partition->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		dev_err(partition->dev, "create command pool #%d failed (cmd 0x%x)\n",
			partition->id, TSM_CREATE_CMD_POOL(partition->id, TSM_MAP_ALL));
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		return -EFAULT;
	}

	dev_info(partition->dev, "command pool #%d was created\n", partition->id);
	return 0;
}

static int zhouyi_v3_abort_command_pool(struct aipu_partition *partition)
{
	u32 status = 0;

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_ABORT_CMD_POOL(partition->id));

	status = aipu_read32(partition->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		dev_err(partition->dev, "abort command pool #%d failed (cmd 0x%x)\n",
			partition->id, TSM_ABORT_CMD_POOL(partition->id));
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		return -EFAULT;
	}

	udelay(partition->priv->reset_delay_us);
	dev_info(partition->dev, "command pool #%d was aborted\n", partition->id);
	return 0;
}

static void zhouyi_v3_destroy_command_pool_internal(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_LOW_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_INFO_REG, 0);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DESTROY_CMD_POOL(partition->id));
}

static void zhouyi_v3_destroy_command_pool(struct aipu_partition *partition)
{
	u32 status = 0;

	zhouyi_v3_destroy_command_pool_internal(partition);
	status = aipu_read32(partition->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		zhouyi_v3_abort_command_pool(partition);
		zhouyi_v3_destroy_command_pool_internal(partition);
		if (IS_CMD_FAIL(status))
			aipu_write32(partition->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
	}

	dev_info(partition->dev, "command pool #%d was destroyed\n", partition->id);
}

static int get_qos(u32 exec_flag)
{
	if (exec_flag & AIPU_JOB_EXEC_FLAG_QOS_FAST)
		return TSM_QOS_FAST;
	return TSM_QOS_SLOW;
}

static int zhouyi_v3_reserve(struct aipu_partition *partition, struct aipu_job_desc *udesc,
			     int trigger_type)
{
	int ret = 0;

	if (unlikely(!partition || !udesc))
		return -EINVAL;

	if (trigger_type == ZHOUYI_V3_TRIGGER_TYPE_CREATE) {
		ret = zhouyi_v3_create_command_pool(partition);
		if (ret)
			return ret;
	}

	if (IS_CMD_POOL_IDLE(aipu_read32(partition->reg, CMD_POOL_STATUS_REG(partition->id)))) {
		dev_err(partition->dev, "create command pool #%d failed (pool idle)\n",
			partition->id);
		return -EFAULT;
	}

	if (trigger_type == ZHOUYI_V3_TRIGGER_TYPE_CREATE ||
	    trigger_type == ZHOUYI_V3_TRIGGER_TYPE_UPDATE_DISPATCH) {
		aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, udesc->head_tcb_pa >> 32);
		aipu_write32(partition->reg, TSM_CMD_SCHD_ADDR_LOW_REG, (u32)udesc->head_tcb_pa);
	}

	dev_dbg(partition->dev, "[Job 0x%llx] scheduler: TCB head 0x%llx\n",
		udesc->job_id, udesc->head_tcb_pa);

	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_INFO_REG, (u16)udesc->job_id);
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DISPATCH_CMD_POOL(partition->id, get_qos(udesc->exec_flag)));

	if (IS_CMD_FAIL(aipu_read32(partition->reg, TSM_STATUS_REG))) {
		dev_err(partition->dev, "dispatch command failed: job tail 0x%llx\n",
			udesc->tail_tcb_pa);
		return -EFAULT;
	}

	return ret;
}

int zhouyi_v3_exit_dispatch(struct aipu_partition *partition, u32 job_flag, u64 tcb_pa)
{
	/* assume command pool has been created */
	aipu_write32(partition->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DISPATCH_CMD_POOL(partition->id, get_qos(job_flag)));

	if (IS_CMD_FAIL(aipu_read32(partition->reg, TSM_STATUS_REG))) {
		dev_err(partition->dev, "dispatch command (exit) failed: job tail 0x%llx\n",
			tcb_pa);
		return -EFAULT;
	}

	dev_dbg(partition->dev, "[Job 0x%x] scheduler: exit TCB head 0x%llx\n", 0, tcb_pa);
	return 0;
}

static bool zhouyi_v3_is_idle(struct aipu_partition *partition)
{
	return !IS_CMD_POOL_FULL(aipu_read32(partition->reg, TSM_STATUS_REG));
}

static void zhouyi_v3_print_hw_id_info(struct aipu_partition *partition)
{
	struct aipu_priv *aipu = partition->priv;
	u32 iter = 0;

	dev_info(aipu->dev, "############# ZHOUYI V3 AIPU #############");
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

static int zhouyi_v3_io_rw(struct aipu_partition *partition, struct aipu_io_req *io_req)
{
	if (unlikely(!io_req))
		return -EINVAL;

	if (!partition || io_req->offset > ZHOUYI_V3_MAX_REG_OFFSET)
		return -EINVAL;

	zhouyi_io_rw(partition->reg, io_req);
	return 0;
}

static void zhouyi_v3_disable_tick_counter(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TICK_COUNTER_CONTROL_STATUS_REG, DISABLE_COUNTER);
}

static void zhouyi_v3_enable_tick_counter(struct aipu_partition *partition)
{
	aipu_write32(partition->reg, TICK_COUNTER_CONTROL_STATUS_REG, ENABLE_COUNTER);
}

static void zhouyi_v3_initialize(struct aipu_partition *partition)
{
	int iter = 0;

	for (iter = 0; iter < partition->cluster_cnt; iter++)
		zhouyi_v3_set_partition(partition, partition->clusters[iter].id);

	zhouyi_v3_config_partition_cmd_pool(partition);
	zhouyi_v3_enable_interrupt(partition);
	zhouyi_v3_disable_tick_counter(partition);
}

static int partition_upper_half(struct aipu_partition *partition)
{
	struct aipu_priv *aipu = partition->priv;
	u32 i = 0;
	struct job_irq_info info;
	u32 status = 0;
	u32 tag_id = 0;
	u16 sig_num = GET_IRQ_SIGNAL_NUM(aipu_read32(&aipu->reg,
		CMD_POOL_IRQ_SIG_REG(partition->id)));
	info.tail_tcbp = aipu_read32(&aipu->reg, CMD_POOL_INTR_TCB_PTR_REG(partition->id));
	info.sig_flag = aipu_read32(&aipu->reg, CMD_POOL_IRQ_SIGNAL_FLAG_REG(partition->id));
	info.tick_counter = ((u64)aipu_read32(&aipu->reg, TICK_COUNTER_HIGH_REG) << 32) +
		aipu_read32(&aipu->reg, TICK_COUNTER_LOW_REG);

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

	/* check fault first because it comes in prior to a done intr if any */
	if (IS_FAULT_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_FAULT);
	} else if (IS_DONE_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_DONE);
		if (IS_TEC_IRQ(status))
			status &= ~CLEAR_CMD_POOL_DONE;
	} else if (IS_ERROR_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_ERROR);
	} else if (IS_EXCEPTION_IRQ(status)) {
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_EXCEPTION);
	}

	aipu_job_manager_irq_upper_half(partition, GET_INTR_TYPE(status), &info);
	if (IS_SIGNAL_IRQ(status)) {
		WARN_ON(sig_num != 1);
		aipu_write32(partition->reg, CMD_POOL_STATUS_REG(partition->id),
			     CLEAR_CMD_POOL_SIGNAL);
	}

	aipu_irq_schedulework(partition->irq_obj);
	return IRQ_HANDLED;
}

static int zhouyi_v3_upper_half(void *data)
{
	struct aipu_partition *partition = (struct aipu_partition *)data;
	struct aipu_priv *aipu = partition->priv;
	int iter = 0;

	for (iter = 0; iter < aipu->partition_cnt; iter++)
		partition_upper_half(&aipu->partitions[iter]);

	return IRQ_HANDLED;
}

static void zhouyi_v3_bottom_half(void *data)
{
	aipu_job_manager_irq_bottom_half(data);
}

#ifdef CONFIG_SYSFS
static int zhouyi_v3_sysfs_show(struct aipu_partition *partition, char *buf)
{
	int ret = 0;
	char tmp[512];

	if (unlikely(!partition || !buf))
		return -EINVAL;

	ret += zhouyi_print_reg_info(partition->reg, tmp, "Cluster config",
				     CLUSTER_CONFIG_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "Cluster ctrl",
				     CLUSTER_CONTROL_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM schedule ctrl handle",
				     TSM_CMD_SCHD_CTRL_HANDLE_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM schedule ctrl info",
				     TSM_CMD_SCHD_CTRL_INFO_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM schedule addr high",
				     TSM_CMD_SCHD_ADDR_HIGH_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM schedule addr low",
				     TSM_CMD_SCHD_ADDR_LOW_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM config",
				     TSM_CONFIG_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM build info",
				     TSM_BUILD_INFO_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM status",
				     TSM_STATUS_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM soft reset",
				     TSM_SOFT_RESET_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "TSM revision",
				     TSM_REVISION_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "Tick counter high",
				     TICK_COUNTER_HIGH_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "Tick counter low",
				     TICK_COUNTER_LOW_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "Tick counter ctrl",
				     TICK_COUNTER_CONTROL_STATUS_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "CMD pool config",
				     CMD_POOL_CONFIG_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "CMD pool status",
				     CMD_POOL_STATUS_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "CMD pool intr status",
				     CMD_POOL_INTR_CTRL_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "CMD pool signal",
				     CMD_POOL_IRQ_SIG_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "CMD pool gignal flag",
				     CMD_POOL_IRQ_SIGNAL_FLAG_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "CMD pool TCBP",
				     CMD_POOL_INTR_TCB_PTR_REG(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(partition->reg, tmp, "Debug page selection",
				     DEBUG_PAGE_SELECTION_REG);
	strcat(buf, tmp);

	return ret;
}
#endif

int zhouyi_v3_soft_reset(struct aipu_partition *partition, bool init_regs)
{
	int ret = 0;
	struct aipu_priv *aipu = partition->priv;

	/* NOTE: this is a global soft-reset, not per-partition! */
	ret = aipu->ops->global_soft_reset(aipu);
	if (ret)
		return ret;

	partition->ops->initialize(partition);
	return 0;
}

u64 get_gm_size(u32 val)
{
	u64 _val = _GET_GM_SIZE(val);

	return _val ? 512 * SZ_1K : (SZ_1M << (_val - 1));
}

static struct aipu_operations zhouyi_v3_ops = {
	.get_config = NULL,
	.enable_interrupt = zhouyi_v3_enable_interrupt,
	.disable_interrupt = zhouyi_v3_disable_interrupt,
	.trigger = zhouyi_v3_trigger,
	.reserve = zhouyi_v3_reserve,
	.is_idle = zhouyi_v3_is_idle,
	.print_hw_id_info = zhouyi_v3_print_hw_id_info,
	.io_rw = zhouyi_v3_io_rw,
	.upper_half = zhouyi_v3_upper_half,
	.bottom_half = zhouyi_v3_bottom_half,
#ifdef CONFIG_SYSFS
	.sysfs_show = zhouyi_v3_sysfs_show,
#endif
	.soft_reset = zhouyi_v3_soft_reset,
	.initialize = zhouyi_v3_initialize,
	.destroy_command_pool = zhouyi_v3_destroy_command_pool,
	.abort_command_pool = zhouyi_v3_abort_command_pool,
	.exit_dispatch = zhouyi_v3_exit_dispatch,
	.disable_tick_counter = zhouyi_v3_disable_tick_counter,
	.enable_tick_counter = zhouyi_v3_enable_tick_counter,
	.enable_core_cnt = zhouyi_v3_enable_core_cnt,
};

struct aipu_operations *get_zhouyi_v3_ops(void)
{
	return &zhouyi_v3_ops;
}
