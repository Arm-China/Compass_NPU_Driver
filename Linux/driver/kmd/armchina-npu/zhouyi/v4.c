// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. */

#include <linux/irqreturn.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "aipu_priv.h"
#include "zhouyi.h"
#include "v4.h"
#include "aipu_io.h"
#include "config.h"

static void zhouyi_v4_enable_core_cnt(struct aipu_partition *cluster, u32 cluster_id,
				      u32 en_core_cnt)
{
	u32 nums = GET_NUMS(aipu_read32(cluster->reg, CLUSTER_CONFIG_REG(cluster_id)));
	u32 en_aiff_cnt = GET_AIFF_NUM(nums);
	u32 config = CONFIG_CLUSTER_V4(cluster->id, en_core_cnt, en_aiff_cnt,
				       cluster->partition_mode);
	u32 status = 0;

	config |= ((1 << en_core_cnt) - 1) << 24;
	aipu_write32(cluster->reg, CLUSTER_CONTROL_REG(cluster_id), config);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG);
	if (!en_core_cnt && IS_CMD_FAIL(status))
		aipu_write32(cluster->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));

	atomic_set(&cluster->clusters[cluster_id].en_core_cnt, en_core_cnt);
	dev_info(cluster->dev, "configure cluster #%u done: en_core_cnt %u (0x%x)\n",
		 cluster_id, en_core_cnt, config);
}

static void zhouyi_v4_enable_interrupt(struct aipu_partition *cluster, bool en_tec_intr)
{
	u32 flag = EN_CMD_POOL_INTR | EN_CLUSTER_INTR | EN_CORE_INTR | EN_ALL_TYPE_INTRS_V4;

	/* TEC interrupts to be updated based on the x3 design */
	if (en_tec_intr)
		flag |= EN_TEC_INTR;

	dev_dbg(cluster->dev, "configure interrupt flag 0x%x\n", flag);
	aipu_write32(cluster->reg, COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG, flag);
	aipu_write32(cluster->reg, COMMAND_POOL_SCP_INTERRUPT_CONTROL_REG, flag);
}

static void zhouyi_v4_disable_interrupt(struct aipu_partition *cluster)
{
	aipu_write32(cluster->reg, COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG, DISABLE_ALL_INTRS);
	aipu_write32(cluster->reg, COMMAND_POOL_SCP_INTERRUPT_CONTROL_REG, DISABLE_ALL_INTRS);
}

static void zhouyi_v4_trigger(struct aipu_partition *cluster)
{
	/* no operation here */
}

static int zhouyi_v4_create_command_pool(struct aipu_partition *cluster, int pool)
{
	int status = 0;

	if (pool == ZHOUYI_COMMAND_POOL_PCP) {
		if (!IS_POOL_IDLE(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)))
			return 0;
	} else if (pool == ZHOUYI_COMMAND_POOL_SCP) {
		if (!IS_POOL_IDLE(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG)))
			return 0;
	}

	if (cluster->partition_mode == PARTITION_MODE_NONE && pool == ZHOUYI_COMMAND_POOL_SCP) {
		dev_err(cluster->dev, "create SCP failed: invalid partition mode\n");
		return -EINVAL;
	}

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_CREATE_CMD_POOL(cluster->id, TSM_MAP_ALL) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		dev_err(cluster->dev, "create command pool failed\n");
		return -EFAULT;
	}

	return 0;
}

static int zhouyi_v4_abort_command_pool(struct aipu_partition *cluster, int pool)
{
	int status = 0;
	int cnt = 0;
	int abort_done_bit = PCP_ABORT_DONE_BIT;

	if (pool & ZHOUYI_COMMAND_POOL_SCP)
		abort_done_bit = SCP_ABORT_DONE_BIT;

	if (pool == ZHOUYI_COMMAND_POOL_PCP) {
		if (IS_POOL_IDLE(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)))
			return -EINVAL;
	} else if (pool == ZHOUYI_COMMAND_POOL_SCP) {
		if (IS_POOL_IDLE(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG)))
			return -EINVAL;
	}

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_ABORT_CMD_POOL(cluster->id) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		dev_err(cluster->dev, "abort command pool failed\n");
		aipu_write32(cluster->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		return -EFAULT;
	}

	for (cnt = 0; cnt < 5; cnt++) {
		if (IS_ABORT_DONE(aipu_read32(cluster->reg, TSM_STATUS_REG), abort_done_bit)) {
			aipu_write32(cluster->reg, TSM_STATUS_REG, CLEAR_ABORT(abort_done_bit));
			dev_dbg(cluster->dev, "command pool was aborted\n");
			return 0;
		}

		dev_info(cluster->dev, "waiting for abortion done...\n");
		udelay(cluster->priv->reset_delay_us);
	}

	return -EFAULT;
}

static int zhouyi_v4_destroy_command_pool(struct aipu_partition *cluster, int pool)
{
	int status = 0;
	bool try_again = false;

	if (pool == ZHOUYI_COMMAND_POOL_PCP) {
		if (IS_POOL_IDLE(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)))
			return -EINVAL;
	} else if (pool == ZHOUYI_COMMAND_POOL_SCP) {
		if (IS_POOL_IDLE(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG)))
			return -EINVAL;
	}

try_again:
	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DESTROY_CMD_POOL(cluster->id) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));

		if (try_again) {
			dev_err(cluster->dev, "destroy command pool failed\n");
			return -EFAULT;
		}

		try_again = true;
		zhouyi_v4_abort_command_pool(cluster, pool);
		goto try_again;
	}

	dev_dbg(cluster->dev, "command pool was destroyed\n");
	return 0;
}

static int zhouyi_v4_reserve(struct aipu_partition *cluster, struct aipu_job_desc *udesc,
			     int trigger_type, int pool)
{
	int ret = 0;
	int status = 0;
	int qos = get_qos(udesc->exec_flag);

	ret = zhouyi_v4_create_command_pool(cluster, pool);
	if (ret)
		return ret;

	status = aipu_read32(cluster->reg, TSM_STATUS_REG);
	if ((pool == ZHOUYI_COMMAND_POOL_PCP && IS_PCP_FULL(status, qos == TSM_QOS_FAST)) ||
	    (pool == ZHOUYI_COMMAND_POOL_SCP && IS_SCP_FULL(status, qos == TSM_QOS_FAST))) {
		dev_info(cluster->dev, "command pool is full\n");
		return ZHOUYI_V4_COMMAND_POOL_FULL;
	}

	dev_dbg(cluster->dev, "[Job 0x%llx] scheduler: TCB head 0x%llx\n",
		udesc->job_id, udesc->head_tcb_pa);

	aipu_write32(cluster->reg, TSM_CMD_SCHD_ADDR_HIGH_REG, udesc->head_tcb_pa >> 32);
	aipu_write32(cluster->reg, TSM_CMD_SCHD_ADDR_LOW_REG, (u32)udesc->head_tcb_pa);
	aipu_write32(cluster->reg, TSM_COMMAND_SCHEDULE_TCB_NUM_REG,
		     ((udesc->tail_tcb_pa - udesc->head_tcb_pa) >> 7) + 1);

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG,
		     TSM_DISPATCH_CMD_POOL_V4(pool, qos));

	status = aipu_read32(cluster->reg, TSM_STATUS_REG);
	if (IS_CMD_FAIL(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG, CLEAR_CMD_FAIL(status));
		dev_err(cluster->dev, "dispatch task failed\n");
		return -EINVAL;
	}

	dev_dbg(cluster->dev, "dispatch user job 0x%llx (0x%llx - 0x%llx, tcbp 0x%llx)",
		udesc->job_id, udesc->head_tcb_pa, udesc->tail_tcb_pa,
		udesc->last_task_tcb_pa);

	return 0;
}

int zhouyi_v4_exit_dispatch(struct aipu_partition *cluster, u32 job_flag, u64 tcb_pa)
{
	/* no operation */
	return 0;
}

static bool zhouyi_v4_is_idle(struct aipu_partition *cluster)
{
	/* to be implemented */
	return true;
}

static void zhouyi_v4_print_hw_id_info(struct aipu_partition *cluster)
{
	struct aipu_priv *aipu = cluster->priv;
	u32 iter = 0;

	dev_info(aipu->dev, "############# ZHOUYI V4 AIPU #############");
	dev_info(aipu->dev, "# Enabled Cluster Count: %d", aipu->cluster_cnt);

	for (iter = 0; iter < aipu->partition_cnt; iter++) {
		cluster = &aipu->partitions[iter];
		dev_info(aipu->dev, "#\n");
		dev_info(aipu->dev, "# -- Cluster #%u", cluster->id);
		dev_info(aipu->dev, "# Core Count: %d",
			 cluster->clusters[0].core_cnt);
		dev_info(aipu->dev, "# TEC Count: %d",
			 cluster->clusters[0].tec_cnt);
	}
	dev_info(aipu->dev, "##########################################");
}

static int zhouyi_v4_io_rw(struct aipu_partition *cluster, struct aipu_io_req *io_req)
{
	if (unlikely(!io_req))
		return -EINVAL;

	if (!cluster || io_req->offset > ZHOUYI_V4_MAX_REG_OFFSET)
		return -EINVAL;

	zhouyi_io_rw(cluster->reg, io_req);
	return 0;
}

static void zhouyi_v4_disable_tick_counter(struct aipu_partition *cluster)
{
	/* to be implemented */
}

static void zhouyi_v4_enable_tick_counter(struct aipu_partition *cluster)
{
	/* to be implemented */
}

static void zhouyi_v4_initialize(struct aipu_partition *cluster)
{
	zhouyi_v4_enable_core_cnt(cluster, 0, cluster->clusters[0].core_cnt);
	zhouyi_v4_enable_interrupt(cluster, false);
}

static int zhouyi_v4_upper_half(void *data)
{
	struct aipu_partition *cluster = (struct aipu_partition *)data;
	struct job_irq_info info;
	u32 status = 0;
	unsigned long irqs = 0;
	int id = 0;

	memset(&info, 0, sizeof(info));

	irqs = GET_V4_IRQS_BITS(aipu_read32(cluster->reg, TSM_INTERRUPT_STATUS_REG));
	for (id = 0; id < 16 && test_bit(id, &irqs); id++) {
		status = aipu_read32(cluster->reg, INTERRUPT_TYPE_INFO_REG(id));
		info.tail_tcbp = aipu_read32(cluster->reg, INTERRUPT_TCB_PTR_REG(id));
		info.cluster_id = 0;
		info.core_id = GET_INTR_CORE_ID(status);
		info.tec_id = GET_INTR_TEC_ID(status);

		if (IS_ERROR_INTR_V4(status) &&
		    (IS_POOL_ERROR(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)) ||
		     IS_POOL_ERROR(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG))))
			dev_dbg(cluster->dev, "TCB format error or bus error in PCP or SCP");

		aipu_job_manager_irq_upper_half(cluster, GET_INTR_TYPE(status), &info);
		aipu_irq_schedulework(cluster->irq_obj);

		aipu_write32(cluster->reg, TSM_INTERRUPT_STATUS_REG, BIT(id));
	}

	return IRQ_HANDLED;
}

static void zhouyi_v4_bottom_half(void *data)
{
	aipu_job_manager_irq_bottom_half(data);
}

#ifdef CONFIG_SYSFS
static int zhouyi_v4_sysfs_show(struct aipu_partition *cluster, char *buf)
{
	/* to be implemented */
	return 0;
}
#endif

int zhouyi_v4_soft_reset(struct aipu_partition *cluster, bool init_regs)
{
	/* to be implemented */
	return 0;
}

static struct aipu_operations zhouyi_v4_ops = {
	.get_config = NULL,
	.enable_interrupt = zhouyi_v4_enable_interrupt,
	.disable_interrupt = zhouyi_v4_disable_interrupt,
	.trigger = zhouyi_v4_trigger,
	.reserve = zhouyi_v4_reserve,
	.is_idle = zhouyi_v4_is_idle,
	.print_hw_id_info = zhouyi_v4_print_hw_id_info,
	.io_rw = zhouyi_v4_io_rw,
	.upper_half = zhouyi_v4_upper_half,
	.bottom_half = zhouyi_v4_bottom_half,
#ifdef CONFIG_SYSFS
	.sysfs_show = zhouyi_v4_sysfs_show,
#endif
	.soft_reset = zhouyi_v4_soft_reset,
	.initialize = zhouyi_v4_initialize,
	.destroy_command_pool = zhouyi_v4_destroy_command_pool,
	.abort_command_pool = zhouyi_v4_abort_command_pool,
	.exit_dispatch = zhouyi_v4_exit_dispatch,
	.disable_tick_counter = zhouyi_v4_disable_tick_counter,
	.enable_tick_counter = zhouyi_v4_enable_tick_counter,
	.enable_core_cnt = zhouyi_v4_enable_core_cnt,
};

struct aipu_operations *get_zhouyi_v4_ops(void)
{
	return &zhouyi_v4_ops;
}
