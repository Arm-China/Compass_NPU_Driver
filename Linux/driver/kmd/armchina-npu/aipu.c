// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

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
#include "aipu_dma_buf.h"
#include "aipu_job_manager.h"
#include "aipu_priv.h"
#include "zhouyi.h"
#include "config.h"

static struct aipu_priv *aipu;

static int aipu_open(struct inode *inode, struct file *filp)
{
	filp->private_data = aipu;

	if(aipu && aipu->soc_ops && aipu->soc_ops->soc_pm_runtime_get_sync)
	{
		aipu->soc_ops->soc_pm_runtime_get_sync(aipu->dev,aipu->soc);
	}
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

	if(aipu && aipu->soc_ops && aipu->soc_ops->soc_pm_runtime_put)
	{
		aipu->soc_ops->soc_pm_runtime_put(aipu->dev,aipu->soc);
	}
	return 0;
}

static long aipu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct aipu_priv *aipu = filp->private_data;
	struct aipu_job_manager *manager = &aipu->job_manager;

	u32 partition_cnt = 0;
	struct aipu_partition_cap *partition_cap = NULL;
	struct aipu_cap cap;
	struct aipu_buf_request buf_req;
	struct aipu_job_desc user_job;
	struct aipu_buf_desc desc;
	struct aipu_io_req io_req;
	struct aipu_job_status_query status;
	struct aipu_hw_status hw;
	struct aipu_config_clusters config_clusters;
	struct aipu_dma_buf_request dmabuf_req;
	struct aipu_dma_buf dmabuf_info;
	struct aipu_group_id_desc group_id_desc;
	int fd = 0;

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
			ret = aipu_mm_free(&aipu->mm, &desc, filp, true);
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
			ret = aipu_job_manager_scheduler(manager, &user_job, filp);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_QUERY_STATUS:
		if (!copy_from_user(&status, (struct job_status_query __user *)arg,
				    sizeof(status))) {
			ret = aipu_job_manager_get_job_status(manager, &status, filp);
			if (!ret &&
			    copy_to_user((struct job_status_query __user *)arg, &status,
					 sizeof(status)))
				ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_KILL_TIMEOUT_JOB:
		if (!copy_from_user(&job_id, (u64 __user *)arg, sizeof(job_id)))
			ret = aipu_job_manager_invalidate_timeout_job(manager, job_id);
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
		ret = aipu_job_manager_get_hw_status(manager, &hw);
		if (!ret && copy_to_user((struct aipu_hw_status __user *)arg, &hw, sizeof(hw)))
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_ABORT_CMD_POOL:
		ret = aipu_job_manager_abort_cmd_pool(manager);
		break;
	case AIPU_IOCTL_DISABLE_TICK_COUNTER:
		ret = aipu_job_manager_disable_tick_counter(manager);
		break;
	case AIPU_IOCTL_ENABLE_TICK_COUNTER:
		ret = aipu_job_manager_enable_tick_counter(manager);
		break;
	case AIPU_IOCTL_CONFIG_CLUSTERS:
		if (!copy_from_user(&config_clusters, (struct aipu_config_clusters __user *)arg,
				    sizeof(config_clusters)))
			ret = aipu_job_manager_config_clusters(manager, &config_clusters);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_ALLOC_DMA_BUF:
		if (!copy_from_user(&dmabuf_req, (struct aipu_dma_buf_request __user *)arg,
				    sizeof(dmabuf_req))) {
			ret = aipu_alloc_dma_buf(&aipu->mm, &dmabuf_req);
			if (!ret && copy_to_user((struct aipu_dma_buf_request __user *)arg,
						 &dmabuf_req, sizeof(dmabuf_req)))
				ret = -EINVAL;
		} else {
			ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_FREE_DMA_BUF:
		if (!copy_from_user(&fd, (int __user *)arg, sizeof(fd)))
			ret = aipu_free_dma_buf(&aipu->mm, fd);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_GET_DMA_BUF_INFO:
		if (!copy_from_user(&dmabuf_info, (struct aipu_dma_buf __user *)arg,
				    sizeof(dmabuf_info))) {
			ret = aipu_get_dma_buf_info(&dmabuf_info);
			if (!ret && copy_to_user((struct aipu_dma_buf __user *)arg,
						 &dmabuf_info, sizeof(dmabuf_info)))
				ret = -EINVAL;
		} else {
			ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_ATTACH_DMA_BUF:
		if (!copy_from_user(&dmabuf_info, (struct aipu_dma_buf __user *)arg,
				    sizeof(dmabuf_info))) {
			ret = aipu_attach_dma_buf(&aipu->mm, &dmabuf_info);
			if (!ret && copy_to_user((struct aipu_dma_buf __user *)arg,
						 &dmabuf_info, sizeof(dmabuf_info)))
				ret = -EINVAL;
		} else {
			ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_DETACH_DMA_BUF:
		if (!copy_from_user(&fd, (int __user *)arg, sizeof(fd)))
			ret = aipu_detach_dma_buf(&aipu->mm, fd);
		else
			ret = -EINVAL;
		break;
	case AIPU_IOCTL_GET_DRIVER_VERSION:
		ret = copy_to_user((char __user *)arg, KMD_VERSION, sizeof(KMD_VERSION));
		break;
	case AIPU_IOCTL_ALLOC_GRID_ID:
		ret = aipu_job_manager_alloc_grid_id(manager);
		if (ret >= 0)
			ret = copy_to_user((char __user *)arg, &ret, sizeof(ret));
		break;
	case AIPU_IOCTL_ALLOC_GROUP_ID:
		if (!copy_from_user(&group_id_desc, (struct aipu_group_id_desc __user *)arg,
				    sizeof(group_id_desc))) {
			ret = aipu_job_manager_alloc_group_id(manager, &group_id_desc);
			if (!ret && copy_to_user((char __user *)arg, &group_id_desc,
						 sizeof(group_id_desc)))
				ret = -EINVAL;
		} else {
			ret = -EINVAL;
		}
		break;
	case AIPU_IOCTL_FREE_GROUP_ID:
		if (!copy_from_user(&group_id_desc, (struct aipu_group_id_desc __user *)arg,
				    sizeof(group_id_desc)))
			ret = aipu_job_manager_free_group_id(manager, &group_id_desc);
		else
			ret = -EINVAL;
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

		dev_info(dev, "AIPU KMD (v%s) probe start...\n", KMD_VERSION);

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
	int ret = 0;

	ret = aipu_job_manager_suspend(get_job_manager(partition));

	if (ret == 0 && aipu && aipu->soc_ops && aipu->soc_ops->disable_clk)
		aipu->soc_ops->disable_clk(partition->dev, aipu->soc);
	return ret;
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
	int ret = 0;

	ret = aipu_job_manager_resume(get_job_manager(partition));

	if (ret == 0 && aipu && aipu->soc_ops && aipu->soc_ops->enable_clk)
		aipu->soc_ops->enable_clk(partition->dev, aipu->soc);

	return ret;
}
EXPORT_SYMBOL(armchina_aipu_resume);

/**
 * @armchina_aipu_alloc_dma_buf() - allocate a dma-buf buffer for other importers
 * @request: buffer allocation request struct.
 *
 */
int armchina_aipu_alloc_dma_buf(struct aipu_dma_buf_request *request)
{
	return aipu_alloc_dma_buf(&aipu->mm, request);
}
EXPORT_SYMBOL(armchina_aipu_alloc_dma_buf);

/**
 * @armchina_aipu_free_dma_buf() - free a dma-buf buffer
 * @fd: file descriptor related to a buffer allocated by armchina_aipu_alloc_dma_buf.
 *
 */
int armchina_aipu_free_dma_buf(int fd)
{
	return aipu_free_dma_buf(&aipu->mm, fd);
}
EXPORT_SYMBOL(armchina_aipu_free_dma_buf);
