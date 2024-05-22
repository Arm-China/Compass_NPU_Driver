// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

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
	u32 nums = GET_NUMS_V4(aipu_read32(cluster->reg, CLUSTER_CONFIG_REG_V4(cluster_id)));
	u32 en_aiff_cnt = GET_AIFF_NUM_V4(nums);
	u32 config = CONFIG_CLUSTER_V4(0xf, en_core_cnt, en_aiff_cnt,
				       4, cluster->partition_mode);
	u32 status = 0;

	config |= ((1 << en_core_cnt) - 1) << 24;
	aipu_write32(cluster->reg, CLUSTER_CONTROL_REG_V4(cluster_id), config);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
	if (!en_core_cnt && IS_CMD_FAIL_V4(status))
		aipu_write32(cluster->reg, TSM_STATUS_REG_V4, CLEAR_CMD_FAIL_V4(status));

	atomic_set(&cluster->clusters[cluster_id].en_core_cnt, en_core_cnt);
	dev_info(cluster->dev, "configure cluster #%u done: en_core_cnt %u (0x%x)\n",
		 cluster_id, en_core_cnt, config);
}

static void zhouyi_v4_enable_interrupt(struct aipu_partition *cluster, bool en_tec_intr)
{
	/* TEC fault interrupts to be updated based on the x3 design */
	u32 flag = EN_ALL_LEVEL_INTRS_V4 | EN_ALL_TYPE_INTRS_V4;

	/* TEC interrupts to be updated based on the x3 design */
	if (en_tec_intr)
		flag |= EN_TEC_INTR_V4;

	dev_dbg(cluster->dev, "configure interrupt flag 0x%x\n", flag);
	aipu_write32(cluster->reg, COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG, flag);
	aipu_write32(cluster->reg, COMMAND_POOL_SCP_INTERRUPT_CONTROL_REG, flag);
}

static void zhouyi_v4_disable_interrupt(struct aipu_partition *cluster)
{
	aipu_write32(cluster->reg, COMMAND_POOL_PCP_INTERRUPT_CONTROL_REG, DISABLE_ALL_INTRS_V4);
	aipu_write32(cluster->reg, COMMAND_POOL_SCP_INTERRUPT_CONTROL_REG, DISABLE_ALL_INTRS_V4);
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

	if (IS_POOL_BUSY(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)) ||
	    IS_POOL_BUSY(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG)))
		return 0;

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V4,
		     TSM_CREATE_CMD_POOL_V4(cluster->id, TSM_MAP_SINGLE_V4) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
	if (IS_CMD_FAIL_V4(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG_V4, CLEAR_CMD_FAIL_V4(status));
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
		if (IS_POOL_BUSY(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)))
			return -EBUSY;
	} else if (pool == ZHOUYI_COMMAND_POOL_SCP) {
		if (IS_POOL_BUSY(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG)))
			return -EBUSY;
	}

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V4,
		     TSM_ABORT_CMD_POOL_V4(cluster->id) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
	if (IS_CMD_FAIL_V4(status)) {
		dev_err(cluster->dev, "abort command pool failed\n");
		aipu_write32(cluster->reg, TSM_STATUS_REG_V4, CLEAR_CMD_FAIL_V4(status));
		return -EFAULT;
	}

	for (cnt = 0; cnt < 5; cnt++) {
		status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
		if (IS_ABORT_DONE(status, abort_done_bit)) {
			aipu_write32(cluster->reg, TSM_STATUS_REG_V4,
				     status & CLEAR_ABORT(abort_done_bit));
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
	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V4,
		     TSM_DESTROY_CMD_POOL_V4(cluster->id) | pool);

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
	if (IS_CMD_FAIL_V4(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG_V4, CLEAR_CMD_FAIL_V4(status));

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

	//create pool
	if (trigger_type == ZHOUYI_TRIGGER_TYPE_CREATE) {
		ret = zhouyi_v4_create_command_pool(cluster, pool);
		if (ret)
			return ret;
	}

	//dispatch pool
	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
	if ((pool == ZHOUYI_COMMAND_POOL_PCP && IS_PCP_FULL(status, qos == TSM_QOS_FAST)) ||
	    (pool == ZHOUYI_COMMAND_POOL_SCP && IS_SCP_FULL(status, qos == TSM_QOS_FAST))) {
		dev_info(cluster->dev, "command pool is full\n");
		return ZHOUYI_V4_COMMAND_POOL_FULL;
	}

	dev_info(cluster->dev, "[Job 0x%llx] scheduler: TCB head 0x%llx\n",
		 udesc->job_id, udesc->head_tcb_pa);

	aipu_write32(cluster->reg, TSM_CMD_SCHD_ADDR_HIGH_REG_V4, udesc->head_tcb_pa >> 32);
	aipu_write32(cluster->reg, TSM_CMD_SCHD_ADDR_LOW_REG_V4, (u32)udesc->head_tcb_pa);
	aipu_write32(cluster->reg, TSM_COMMAND_SCHEDULE_TCB_NUM_REG_V4,
		     ((udesc->tail_tcb_pa - udesc->head_tcb_pa) >> 7) + 1);

	aipu_write32(cluster->reg, TSM_CMD_SCHD_CTRL_HANDLE_REG_V4,
		     TSM_DISPATCH_CMD_POOL_V4(pool, qos));

	status = aipu_read32(cluster->reg, TSM_STATUS_REG_V4);
	if (IS_CMD_FAIL_V4(status)) {
		aipu_write32(cluster->reg, TSM_STATUS_REG_V4, CLEAR_CMD_FAIL_V4(status));
		dev_err(cluster->dev, "dispatch task failed\n");
		return -EINVAL;
	}

	dev_info(cluster->dev, "dispatch user job 0x%llx (0x%llx - 0x%llx, tcbp 0x%llx)",
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
	aipu_write32(cluster->reg, AHB_INTERNAL_CSR_SELECTION_CTRL_REG, SELECT_DEBUG_CORE_V4(0, 0));
	aipu_write32(cluster->reg, PMU_TOP_CLOCK_POWER_CTRL_REG, 0x5); /* workaround for h/w */
}

static int zhouyi_v4_soft_reset_type(struct aipu_partition *cluster,
				     struct job_irq_info *info, int status)
{
	int ret = 0;

	// no partition for one cluster
	if (cluster->partition_mode == PARTITION_MODE_NONE ||
	    cluster->partition_mode == PARTITION_MODE_CORE3_SCP) {
		if (IS_CLUSTER_IRQ_V4(status) && IS_FAULT_IRQ_V4(status)) {
			pr_info("v4 cluster fault global reset event.\n");
			cluster->event_type = AIPU_IRQ_EVENT_RESET;
		} else if (IS_POOL_IRQ_V4(status) && (IS_TIMEOUT_IRQ_V4(status) ||
				   IS_ERROR_IRQ_V4(status))) {
			pr_info("v4 pool error/timeout global reset event.\n");
			cluster->event_type = AIPU_IRQ_EVENT_RESET;
		}
	}

	return ret;
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
	for (id = 0; id < TSM_IRQ_MAX_NUM; id++) {
		if (!test_bit(id, &irqs))
			continue;

		status = aipu_read32(cluster->reg, INTERRUPT_TYPE_INFO_REG(id));
		info.tail_tcbp = aipu_read32(cluster->reg, INTERRUPT_TCB_PTR_REG(id));
		info.sig_flag = aipu_read32(cluster->reg,  INTERRUPT_SIGNAL_FLAG_REG(id));
		info.cluster_id = GET_INTR_CLUSTER_ID_V4(status);
		info.core_id = GET_INTR_CORE_ID_V4(status);
		info.tec_id = GET_INTR_TEC_ID_V4(status);

		/* log print to be removed */
		if (IS_CLUSTER_IRQ_V4(status)) {
			pr_info("IRQ (id = %d): cluster status 0x%x, tcbp 0x%x\n",
				id, status, info.tail_tcbp);
		} else if (IS_CORE_IRQ_V4(status)) {
			pr_info("IRQ (id = %d): SKIP core status 0x%x, tcbp 0x%x\n",
				id, status, info.tail_tcbp);
			continue;
		/* Update Fault IRQ based on X3 hardware design */
		} else if (IS_TEC_IRQ_V4(status) && (IS_DONE_IRQ_V4(status) ||
				  IS_FAULT_IRQ_V4(status))) {
			pr_info("IRQ (id = %d): SKIP tec d/f status 0x%x, tcbp 0x%x\n",
				id, status, info.tail_tcbp);
			continue;
		}

		if (IS_ERROR_IRQ_V4(status) &&
		    (IS_POOL_ERROR(aipu_read32(cluster->reg, COMMAND_POOL_PCP_STATUS_REG)) ||
		     IS_POOL_ERROR(aipu_read32(cluster->reg, COMMAND_POOL_SCP_STATUS_REG))))
			dev_dbg(cluster->dev, "TCB format error or bus error in PCP or SCP");

		//check if do global soft reset
		if (IS_RESET(status)) {
			if (zhouyi_v4_soft_reset_type(cluster, &info, status))
				dev_err(cluster->dev, "global reset fail.\n");
		}
		aipu_job_manager_irq_upper_half(cluster, status, &info);
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
	int ret = 0;
	char tmp[128];

	if (unlikely(!cluster || !buf))
		return -EINVAL;

	snprintf(tmp, 128, "--- TCB Dispatch ---\n");
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TCB Address High",
				     TSM_CMD_SCHD_ADDR_HIGH_REG_V4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TCB Address Low",
				     TSM_CMD_SCHD_ADDR_LOW_REG_V4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TCB Number",
				     TSM_COMMAND_SCHEDULE_TCB_NUM_REG_V4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "CMD Schedule Ctrl Handle",
				     TSM_CMD_SCHD_CTRL_HANDLE_REG_V4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "TSM Status",
				     TSM_STATUS_REG_V4);
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

	snprintf(tmp, 512, "\n--- Debug Links ---\n");
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Debug Link Enable",
				     AHB_INTERNAL_CSR_SELECTION_CTRL_REG);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Cluster Control",
				     CLUSTER_CONTROL_REG_V4(0));
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Cluster Status",
				     0x2000 + 0x4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Status",
				     0x3000 + 0x4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Idle Status",
				     0x3000 + 0x8);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Config0",
				     0x3000 + 0x20);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Config1",
				     0x3000 + 0x24);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core TCBP",
				     0x3000 + 0x54);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Start Control",
				     0x3000 + 0x80);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Start PC",
				     0x3000 + 0x84);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core Intr Control",
				     0x3000 + 0x90);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core ASE #0 Low",
				     0x3000 + 0xe0);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core ASE #0 High",
				     0x3000 + 0xe4);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core ASE #1 Low",
				     0x3000 + 0xe8);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core ASE #1 High",
				     0x3000 + 0xec);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core ASE #2 Low",
				     0x3000 + 0xf0);
	strcat(buf, tmp);
	ret += zhouyi_print_reg_info(cluster->reg, tmp, "Core ASE #2 High",
				     0x3000 + 0xf4);
	strcat(buf, tmp);

	return ret;
}
#endif

int zhouyi_v4_soft_reset(struct aipu_partition *cluster, bool init_regs)
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
