// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#include <linux/irqreturn.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "aipu_priv.h"
#include "zhouyi.h"
#include "v3_1.h"
#include "aipu_io.h"
#include "config.h"

static void zhouyi_v3_1_enable_core_cnt(struct aipu_partition *cluster, u32 cluster_id,
				      u32 en_core_cnt)
{
	u32 nums = GET_NUMS_V3_1(aipu_read32(cluster->reg, CLUSTER_CONFIG_REG_V3_1(cluster_id)));
	u32 en_aiff_cnt = GET_AIFF_NUM_V3_1(nums);
	u32 config = CONFIG_CLUSTER_V3_1(0xf, en_core_cnt, en_aiff_cnt,
				       4, cluster->partition_mode);
	u32 status = 0;

	config |= ((1 << en_core_cnt) - 1) << 24;
	aipu_write32(cluster->reg, CLUSTER_CONTROL_REG_V3_1(cluster_id), config);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (!en_core_cnt && IS_CMD_FAIL_V3_1(status))
		aipu_write32(cluster->reg, TSM_STATUS_REG_V3_1, CLEAR_CMD_FAIL_V3_1(status));

	atomic_set(&cluster->clusters[cluster_id].en_core_cnt, en_core_cnt);
	dev_info(cluster->dev, "configure cluster #%u done: en_core_cnt %u (0x%x)\n",
		 cluster_id, en_core_cnt, config);
}

static void zhouyi_v3_1_enable_interrupt(struct aipu_partition *cluster, bool en_tec_intr)
{
	u32 flag = EN_ALL_LEVEL_INTRS_V3_1 | EN_ALL_TYPE_INTRS_V3_1;

	if (en_tec_intr)
		flag |= EN_TEC_INTR_V3_1;

	dev_dbg(cluster->dev, "configure interrupt flag 0x%x\n", flag);
	aipu_write32(cluster->reg, COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG, flag);
}

static void zhouyi_v3_1_disable_interrupt(struct aipu_partition *cluster)
{
	aipu_write32(cluster->reg, COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG, DISABLE_ALL_INTRS_V3_1);
}

static void zhouyi_v3_1_trigger(struct aipu_partition *cluster)
{
	/* no operation here */
}

static int zhouyi_v3_1_create_command_pool(struct aipu_partition *cluster, int pool)
{
	int status = 0;

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1,
		     TSM_CREATE_CMD_POOL_V3_1(cluster->id, TSM_MAP_SINGLE_V3_1) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (IS_CMD_FAIL_V3_1(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG_V3_1, CLEAR_CMD_FAIL_V3_1(status));
		dev_err(cluster->dev, "create command pool failed\n");
		return -EFAULT;
	}

	return 0;
}

static int zhouyi_v3_1_abort_command_pool(struct aipu_partition *cluster, int pool)
{
	int status = 0;
	int cnt = 0;
	int abort_done_bit = PCP_ABORT_DONE_BIT;
	unsigned long irqs = 0;

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (!IS_ABORT_DONE(status, abort_done_bit)) {
		dev_err(cluster->dev, "abort cmd pool twice again.\n");
		return -EBUSY;
	}

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1,
		     TSM_ABORT_CMD_POOL_V3_1(cluster->id) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (IS_CMD_FAIL_V3_1(status)) {
		dev_err(cluster->dev, "abort command pool failed\n");
		aipu_write32(cluster->reg, TSM_STATUS_REG_V3_1, CLEAR_CMD_FAIL_V3_1(status));
		return -EFAULT;
	}

	for (cnt = 0; cnt < 5; cnt++) {
		status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
		if (IS_ABORT_DONE(status, abort_done_bit)) {
			aipu_write32(cluster->reg, TSM_STATUS_REG_V3_1,
				     status & CLEAR_ABORT(abort_done_bit));
			irqs = GET_V3_1_IRQS_BITS(aipu_read32(cluster->reg,
						TSM_INTERRUPT_STATUS_REG));
			aipu_write32(cluster->reg, TSM_INTERRUPT_STATUS_REG, irqs);
			dev_dbg(cluster->dev, "command pool was aborted\n");
			return 0;
		}

		dev_info(cluster->dev, "waiting for abortion done...\n");
		udelay(cluster->priv->reset_delay_us);
	}

	return -EFAULT;
}

static int zhouyi_v3_1_destroy_command_pool(struct aipu_partition *cluster, int pool)
{
	int status = 0;
	bool try_again = false;

	if (pool == ZHOUYI_COMMAND_POOL_PCP) {
		if (IS_POOL_BUSY(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)))
			return -EINVAL;
	}

try_again:
	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1,
		     TSM_DESTROY_CMD_POOL_V3_1(cluster->id) | pool);
	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (IS_CMD_FAIL_V3_1(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG_V3_1, CLEAR_CMD_FAIL_V3_1(status));

		if (try_again) {
			dev_err(cluster->dev, "destroy command pool failed\n");
			return -EFAULT;
		}

		try_again = true;
		zhouyi_v3_1_abort_command_pool(cluster, pool);
		goto try_again;
	}

	dev_dbg(cluster->dev, "command pool was destroyed\n");
	return 0;
}

static int zhouyi_v3_1_reserve(struct aipu_partition *cluster, struct aipu_job_desc *udesc,
			     int trigger_type, int pool)
{
	int ret = 0;
	int status = 0;
	int qos = get_qos(udesc->exec_flag);

	//create pool
	if (trigger_type == ZHOUYI_TRIGGER_TYPE_CREATE) {
		ret = zhouyi_v3_1_create_command_pool(cluster, pool);
		if (ret)
			return ret;
	}

	//dispatch pool
	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (pool == ZHOUYI_COMMAND_POOL_PCP && IS_PCP_FULL(status, qos == TSM_QOS_FAST)) {
		dev_info(cluster->dev, "command pool is full\n");
		return ZHOUYI_V3_1_COMMAND_POOL_FULL;
	}

	dev_info(cluster->dev, "[Job 0x%llx] scheduler: TCB head 0x%llx\n",
		 udesc->job_id, udesc->head_tcb_pa);

	aipu_write32(cluster->reg, TSM_CMD_SCHD_ADDR_HIGH_REG_V3_1, udesc->head_tcb_pa >> 32);
	aipu_write32(cluster->reg, TSM_CMD_SCHD_ADDR_LOW_REG_V3_1, (u32)udesc->head_tcb_pa);
	aipu_write32(cluster->reg, TSM_COMMAND_SCHEDULE_TCB_NUM_REG_V3_1,
		     ((udesc->tail_tcb_pa - udesc->head_tcb_pa) >> 7) + 1);

	if (udesc->exec_flag & AIPU_JOB_EXEC_FLAG_DBG_DISPATCH) {
		aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1,
			     TSM_DBG_DISPATCH_CMD_POOL_V3_1(pool, qos, udesc->core_id));
		dev_dbg(cluster->dev, "debug-dispatch user job 0x%llx (0x%llx - 0x%llx, tcbp 0x%llx) core %d",
			udesc->job_id, udesc->head_tcb_pa, udesc->tail_tcb_pa,
			udesc->last_task_tcb_pa, udesc->core_id);
	} else {
		aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1,
			     TSM_DISPATCH_CMD_POOL_V3_1(pool, qos));
		dev_dbg(cluster->dev, "dispatch user job 0x%llx (0x%llx - 0x%llx, tcbp 0x%llx)",
			udesc->job_id, udesc->head_tcb_pa, udesc->tail_tcb_pa,
			udesc->last_task_tcb_pa);
	}

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V3_1);
	if (IS_CMD_FAIL_V3_1(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG_V3_1, CLEAR_CMD_FAIL_V3_1(status));
		dev_err(cluster->dev, "dispatch task failed\n");
		return -EINVAL;
	}

	return 0;
}

static void zhouyi_v3_1_print_hw_id_info(struct aipu_partition *cluster)
{
	struct aipu_priv *aipu = cluster->priv;
	u32 iter = 0;

	dev_info(aipu->dev, "############# ZHOUYI V3_1 AIPU #############");
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

static int zhouyi_v3_1_io_rw(struct aipu_partition *cluster, struct aipu_io_req *io_req)
{
	if (unlikely(!io_req))
		return -EINVAL;

	if (!cluster || io_req->offset > ZHOUYI_V3_1_MAX_REG_OFFSET)
		return -EINVAL;

	zhouyi_io_rw(cluster->reg, io_req);
	return 0;
}

static void zhouyi_v3_1_disable_tick_counter(struct aipu_partition *cluster)
{
	aipu_write32(cluster->reg, TICK_COUNTER_CONTROL_STATUS_REG_V3_1, DISABLE_COUNTER_V3_1);
}

static void zhouyi_v3_1_enable_tick_counter(struct aipu_partition *cluster)
{
	aipu_write32(cluster->reg, TICK_COUNTER_CONTROL_STATUS_REG_V3_1, ENABLE_COUNTER_V3_1);
}

static void zhouyi_v3_1_initialize(struct aipu_partition *cluster)
{
	zhouyi_v3_1_enable_core_cnt(cluster, 0, cluster->clusters[0].core_cnt);
	zhouyi_v3_1_enable_interrupt(cluster, false);
	zhouyi_v3_1_disable_tick_counter(cluster);
}

static int zhouyi_v3_1_soft_reset_type(struct aipu_partition *cluster,
				     struct job_irq_info *info, int status)
{
	int ret = 0;

	if (cluster->partition_mode == PARTITION_MODE_NONE) {
		if (IS_CLUSTER_IRQ_V3_1(status) && IS_FAULT_IRQ_V3_1(status)) {
			dev_err(cluster->dev, "cluster fault global reset event.\n");
			cluster->event_type = AIPU_IRQ_EVENT_RESET;
		} else if (IS_POOL_IRQ_V3_1(status) && (IS_TIMEOUT_IRQ_V3_1(status) ||
				   IS_ERROR_IRQ_V3_1(status))) {
			dev_err(cluster->dev, "pool error/timeout global reset event.\n");
			cluster->event_type = AIPU_IRQ_EVENT_RESET;
		} else if (IS_TEC_IRQ_V3_1(status) && (IS_FAULT_IRQ_V3_1(status))) {
			dev_err(cluster->dev, "TEC Fault Error.\n");
		}
	}

	return ret;
}

static int zhouyi_v3_1_upper_half(void *data)
{
	struct aipu_partition *cluster = (struct aipu_partition *)data;
	struct job_irq_info info;
	u32 status = 0;
	unsigned long irqs = 0;
	int id = 0;

	memset(&info, 0, sizeof(info));

	irqs = GET_V3_1_IRQS_BITS(aipu_read32(cluster->reg, TSM_INTERRUPT_STATUS_REG));
	for (id = 0; id < TSM_IRQ_MAX_NUM; id++) {
		if (!test_bit(id, &irqs))
			continue;

		status = aipu_read32(cluster->reg, INTERRUPT_TYPE_INFO_REG(id));
		info.tail_tcbp = aipu_read32(cluster->reg, INTERRUPT_TCB_PTR_REG(id));
		info.sig_flag = aipu_read32(cluster->reg,  INTERRUPT_SIGNAL_FLAG_REG(id));
		info.cluster_id = GET_INTR_CLUSTER_ID_V3_1(status);
		info.core_id = GET_INTR_CORE_ID_V3_1(status);
		info.tec_id = GET_INTR_TEC_ID_V3_1(status);

		if (IS_CORE_IRQ_V3_1(status))
			continue;
		else if (IS_TEC_IRQ_V3_1(status) && (IS_DONE_IRQ_V3_1(status)))
			continue;

		//check if do global soft reset
		if (IS_RESET(status)) {
			if (zhouyi_v3_1_soft_reset_type(cluster, &info, status))
				dev_err(cluster->dev, "global reset fail.\n");
		}
		aipu_job_manager_irq_upper_half(cluster, status, &info);
		aipu_irq_schedulework(cluster->irq_obj);
	}
	aipu_write32(cluster->reg, TSM_INTERRUPT_STATUS_REG, irqs);

	return IRQ_HANDLED;
}

static void zhouyi_v3_1_bottom_half(void *data)
{
	aipu_job_manager_irq_bottom_half(data);
}

#ifdef CONFIG_SYSFS
static int zhouyi_v3_1_sysfs_show(struct aipu_partition *cluster, char *buf)
{
	int ret = 0;
	char tmp[1024];

	if (unlikely(!cluster || !buf))
		return -EINVAL;

	snprintf(tmp, 128, "--- TCB Dispatch ---\n");
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TCB Address High",
				     TSM_CMD_SCHD_ADDR_HIGH_REG_V3_1);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TCB Address Low",
				     TSM_CMD_SCHD_ADDR_LOW_REG_V3_1);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TCB Number",
				     TSM_COMMAND_SCHEDULE_TCB_NUM_REG_V3_1);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "CMD Schedule Ctrl Handle",
				     TSM_CMD_SCHD_CTRL_HANDLE_REG_V3_1);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TSM Status",
				     TSM_STATUS_REG_V3_1);
	strcat(buf, tmp);

	snprintf(tmp, 512, "\n--- Interrupts ---\n");
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "[TSM] Intr Status",
				     TSM_INTERRUPT_STATUS_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "[PCP] Pool Status",
				     COMMAND_POOL_PCP_STATUS_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "[PCP] Intr Control",
				     COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG);
	strcat(buf, tmp);

	return ret;
}
#endif

int zhouyi_v3_1_soft_reset(struct aipu_partition *cluster, bool init_regs)
{
	int ret = 0;
	struct aipu_priv *aipu = cluster->priv;

	/* NOTE: this is a global soft-reset, not per-partition! */
	ret = aipu->ops->global_soft_reset(aipu);
	if (ret)
		return ret;

	cluster->ops->initialize(cluster);
	return 0;
}












static struct aipu_operations zhouyi_v3_1_ops = {
	.get_config = NULL,
	.enable_interrupt = zhouyi_v3_1_enable_interrupt,
	.disable_interrupt = zhouyi_v3_1_disable_interrupt,
	.trigger = zhouyi_v3_1_trigger,
	.reserve = zhouyi_v3_1_reserve,
	.print_hw_id_info = zhouyi_v3_1_print_hw_id_info,
	.io_rw = zhouyi_v3_1_io_rw,
	.upper_half = zhouyi_v3_1_upper_half,
	.bottom_half = zhouyi_v3_1_bottom_half,
#ifdef CONFIG_SYSFS
	.sysfs_show = zhouyi_v3_1_sysfs_show,
#endif
	.soft_reset = zhouyi_v3_1_soft_reset,
	.initialize = zhouyi_v3_1_initialize,
	.destroy_command_pool = zhouyi_v3_1_destroy_command_pool,
	.abort_command_pool = zhouyi_v3_1_abort_command_pool,
	.disable_tick_counter = zhouyi_v3_1_disable_tick_counter,
	.enable_tick_counter = zhouyi_v3_1_enable_tick_counter,
	.enable_core_cnt = zhouyi_v3_1_enable_core_cnt,
};

struct aipu_operations *get_zhouyi_v3_1_ops(void)
{
	return &zhouyi_v3_1_ops;
}
