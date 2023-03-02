// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/compat.h>
#include <armchina_aipu.h>
#include "armchina_aipu_soc.h"
#include "aipu_mm.h"
#include "aipu_job_manager.h"
#include "aipu_priv.h"
#include "zhouyi.h"
#include "config.h"

static struct aipu_priv *aipu;

static int aipu_open(struct inode *inode, struct file *filp)
{
	filp->private_data = aipu;
	return aipu_priv_check_status(aipu);
}

static int aipu_release(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct aipu_priv *aipu = filp->private_data;

	ret = aipu_job_manager_cancel_jobs(&aipu->job_manager, filp);
	if (ret)
		return ret;

	aipu_mm_free_buffers(&aipu->mm, filp);
	return 0;
}

static long aipu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aipu_priv *aipu = filp->private_data;

	u32 partition_cnt = 0;
	struct aipu_partition_cap *partition_cap = NULL;
	struct aipu_cap cap;
	struct aipu_buf_request buf_req;
	struct aipu_job_desc user_job;
	struct aipu_buf_desc desc;
	struct aipu_io_req io_req;
	struct aipu_job_status_query status;
	struct aipu_hw_status hw;
	u64 job_id;

	switch (cmd) {
	case AIPU_IOCTL_QUERY_CAP:
		ret = aipu_priv_query_capability(aipu, &cap);
		if (!ret && copy_to_user((struct aipu_cap __user *)arg, &cap, sizeof(cap)))
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_QUERY_PARTITION_CAP:
		partition_cnt = aipu_priv_get_partition_cnt(aipu);
		partition_cap = kcalloc(partition_cnt, sizeof(*partition_cap), GFP_KERNEL);
		if (partition_cap) {
			ret = aipu_priv_query_partition_capability(aipu, partition_cap);
			if (!ret &&
			    copy_to_user((struct aipu_partition_cap __user *)arg, partition_cap,
					 partition_cnt * sizeof(*partition_cap)))
				ret = -EINVAL;
			kfree(partition_cap);
			partition_cap = NULL;
		} else {
			ret = -ENOMEM;
		}
		break;
	case AIPU_IOCTL_REQ_BUF:
		if (!copy_from_user(&buf_req, (struct aipu_buf_request __user *)arg,
				    sizeof(buf_req))) {
			ret = aipu_mm_alloc(&aipu->mm, &buf_req, filp);
			if (!ret &&
			    copy_to_user((struct aipu_buf_request __user *)arg, &buf_req,
					 sizeof(buf_req)))
				ret = -EINVAL;
		} else {
			ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_FREE_BUF:
		if (!copy_from_user(&desc, (struct buf_desc __user *)arg, sizeof(desc)))
			ret = aipu_mm_free(&aipu->mm, &desc, filp);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_DISABLE_SRAM:
		ret = aipu_mm_disable_sram_allocation(&aipu->mm, filp);
		break;
	case AIPU_IOCTL_ENABLE_SRAM:
		ret = aipu_mm_enable_sram_allocation(&aipu->mm, filp);
		break;
	case AIPU_IOCTL_SCHEDULE_JOB:
		if (!copy_from_user(&user_job, (struct user_job_desc __user *)arg,
				    sizeof(user_job)))
			ret = aipu_job_manager_scheduler(&aipu->job_manager, &user_job, filp);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_QUERY_STATUS:
		if (!copy_from_user(&status, (struct job_status_query __user *)arg,
				    sizeof(status))) {
			ret = aipu_job_manager_get_job_status(&aipu->job_manager, &status, filp);
			if (!ret &&
			    copy_to_user((struct job_status_query __user *)arg, &status,
					 sizeof(status)))
				ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_KILL_TIMEOUT_JOB:
		if (!copy_from_user(&job_id, (u64 __user *)arg, sizeof(job_id)))
			ret = aipu_job_manager_invalidate_timeout_job(&aipu->job_manager, job_id);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_REQ_IO:
		if (!copy_from_user(&io_req, (struct aipu_io_req __user *)arg, sizeof(io_req))) {
			ret = aipu_priv_io_rw(aipu, &io_req);
			if (!ret &&
			    copy_to_user((struct aipu_io_req __user *)arg, &io_req,
					 sizeof(io_req)))
				ret = -EINVAL;
		} else {
			ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_GET_HW_STATUS:
		ret = aipu_job_manager_get_hw_status(&aipu->job_manager, &hw);
		if (!ret && copy_to_user((struct aipu_hw_status __user *)arg, &hw, sizeof(hw)))
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_ABORT_CMD_POOL:
		ret = aipu_job_manager_abort_cmd_pool(&aipu->job_manager);
		break;
	case AIPU_IOCTL_DISABLE_TICK_COUNTER:
		ret = aipu_job_manager_disable_tick_counter(&aipu->job_manager);
		break;
	case AIPU_IOCTL_ENABLE_TICK_COUNTER:
		ret = aipu_job_manager_enable_tick_counter(&aipu->job_manager);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long aipu_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);

	return aipu_ioctl(filp, cmd, arg);
}
#endif

static int aipu_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct aipu_priv *aipu = filp->private_data;

	return aipu_mm_mmap_buf(&aipu->mm, vma, filp);
}

static unsigned int aipu_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct aipu_priv *aipu = filp->private_data;

	if (aipu_job_manager_has_end_job(&aipu->job_manager, filp, wait, task_pid_nr(current)))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations aipu_fops = {
	.owner = THIS_MODULE,
	.open = aipu_open,
	.poll = aipu_poll,
	.unlocked_ioctl = aipu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = aipu_compat_ioctl,
#endif
	.mmap = aipu_mmap,
	.release = aipu_release,
};

/**
 * @armchina_aipu_probe() - probe operation for platfom driver provided by ArmChina
 * @p_dev: pointer to the AIPU platform device struct.
 * @soc:   pointer to aipu SoC struct contains soc specific data.
 *         this argument can be NULL if SoC vendor has no AIPU related soc data structure.
 * @ops:   pointer to aipu SoC operation struct.
 *         this argument can be NULL if SoC vendor does not provide any soc operation. In
 *         this case the SoC related operations in AIPU driver are unavailable.
 *
 * This function should be called in a SoC vendor provided xx_aipu_probe() function,
 * which is registered to the platfom_driver struct; if no such xx_aipu_probe() is provided,
 * SoC vendor should directly register this function to the platfom_driver struct.
 *
 * Return: 0 on success and error code otherwise.
 */
int armchina_aipu_probe(struct platform_device *p_dev, struct aipu_soc *soc,
			struct aipu_soc_operations *ops)
{
	int ret = 0;
	struct device *dev = &p_dev->dev;
	struct aipu_partition *partition = NULL;
	u32 id = 0;

	/* create & init aipu priv struct shared by all clusters/cores */
	if (!aipu) {
		aipu = devm_kzalloc(dev, sizeof(*aipu), GFP_KERNEL);
		if (!aipu)
			return -ENOMEM;

		dev_info(dev, "AIPU KMD probe start...\n");
		ret = init_aipu_priv(aipu, p_dev, &aipu_fops, soc, ops);
		if (ret)
			return ret;
	}

	of_property_read_u32(dev->of_node, "core-id", &id);

	WARN_ON(!aipu->ops);
	partition = aipu->ops->create_partitions(aipu, id, p_dev);
	if (IS_ERR(partition))
		goto out_clean;

	platform_set_drvdata(p_dev, partition);
	goto finish;

out_clean:
	armchina_aipu_remove(p_dev);

finish:
	return ret;
}
EXPORT_SYMBOL(armchina_aipu_probe);

/**
 * @armchina_aipu_remove() - remove operation for platfom driver provided by ArmChina
 * @p_dev: pointer to the AIPU platform device struct
 *
 * This function should be called in a SoC vendor provided xx_aipu_remove() function,
 * which is registered to the platfom_driver struct; if no such xx_aipu_remove() is provided,
 * this function should be directly registered to the platfom_driver struct.
 *
 * Return: 0 on success and error code otherwise.
 */
int armchina_aipu_remove(struct platform_device *p_dev)
{
	if (!aipu || !aipu->is_init)
		return 0;
	return deinit_aipu_priv(aipu);
}
EXPORT_SYMBOL(armchina_aipu_remove);

/**
 * @armchina_aipu_suspend() - suspend operation for platfom driver provided by ArmChina
 * @p_dev: pointer to the AIPU platform device struct.
 * @state: power state device is entering.
 *
 * This function disables the AIPU clock from SoC level with the disable_clk method
 * registered in SoC probing. SoC vendor can directly register this function to the
 * platfom_driver struct or implements its private xx_aipu_suspend() as a replacement.
 */
int armchina_aipu_suspend(struct platform_device *p_dev, pm_message_t state)
{
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	partition->ops->soft_reset(partition, false);

	if (aipu && aipu->soc_ops && aipu->soc_ops->disable_clk)
		aipu->soc_ops->disable_clk(partition->dev, aipu->soc);
	return 0;
}
EXPORT_SYMBOL(armchina_aipu_suspend);

/**
 * @armchina_aipu_resume() - resume operation for platfom driver provided by ArmChina
 * @p_dev: pointer to the AIPU platform device struct.
 *
 * This function enables the AIPU clock from SoC level with the enable_clk method
 * registered in SoC probing, and enable AIPU interrupts. SoC vendor should directly
 * register this function to the platfom_driver struct, or implements its private
 * xx_aipu_resume() calls this function.
 */
int armchina_aipu_resume(struct platform_device *p_dev)
{
	struct aipu_partition *partition = platform_get_drvdata(p_dev);

	if (aipu && aipu->soc_ops && aipu->soc_ops->enable_clk)
		aipu->soc_ops->enable_clk(partition->dev, aipu->soc);

	partition->ops->soft_reset(partition, true);
	return 0;
}
EXPORT_SYMBOL(armchina_aipu_resume);
