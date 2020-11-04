// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/slab.h>
#include <linux/of_address.h>
#include "aipu_priv.h"
#include "config.h"
#include "z1.h"
#include "z2.h"
#include "x2.h"

static int init_misc_dev(struct aipu_priv *aipu)
{
	aipu->misc.minor = MISC_DYNAMIC_MINOR;
	aipu->misc.name = "aipu";
	aipu->misc.fops = aipu->aipu_fops;
	aipu->misc.mode = 0666;
	return misc_register(&aipu->misc);
}

static void deinit_misc_dev(struct aipu_priv *aipu)
{
	if (aipu && aipu->misc.fops) {
		misc_deregister(&aipu->misc);
		memset(&aipu->misc, 0, sizeof(aipu->misc));
	}
}

/**
 * @init_aipu_priv() - initialize an input AIPU private data struct
 * @aipu:  pointer to the aipu private struct to be initialized
 * @p_dev: pointer to the platform device struct
 * @fops:  pointer to the file_operations struct
 * @soc:   pointer to the SoC private data structure
 * @soc_ops: pointer to the SoC operations struct
 *
 * This function should be called while driver probing. It should be called
 * only one time.
 *
 * Return: 0 on success and error code otherwise.
 */
int init_aipu_priv(struct aipu_priv *aipu, struct platform_device *p_dev,
		   const struct file_operations *fops, struct aipu_soc *soc,
		   struct aipu_soc_operations *soc_ops)
{
	int ret = 0;
	int version = 0;
	int config = 0;

	if (!aipu || !p_dev || !fops)
		return -EINVAL;

	if (aipu->is_init)
		return 0;

	aipu->dev = &p_dev->dev;
	aipu->aipu_fops = fops;
	aipu->soc = soc;
	aipu->soc_ops = soc_ops;
	aipu->reset_delay_us = AIPU_CONFIG_DEFAULT_RESET_DELAY_US;

	/* init during partitions creation */
	aipu->partition_cnt = 0;
	aipu->cluster_cnt = 0;
	aipu->max_partition_cnt = 0;
	aipu->max_cmd_pool_cnt = 0;
	aipu->partitions = NULL;
	aipu->irq_obj = NULL;
	aipu->reg.kern = NULL;
	aipu->reg.phys = 0;
	aipu->reg.size = 0;
	aipu->ops = NULL;

	zhouyi_detect_aipu_version(p_dev, &version, &config);
	dev_dbg(aipu->dev, "AIPU core0 ISA version %d, configuration %d\n", version, config);
	aipu->version = version;

#if (defined BUILD_ZHOUYI_X2 || defined BUILD_ZHOUYI_ALL)
	if (version == AIPU_ISA_VERSION_ZHOUYI_X2)
		aipu->ops = get_x2_priv_ops();
#endif

#if (defined BUILD_ZHOUYI_Z1 || defined BUILD_ZHOUYI_Z2 || defined BUILD_ZHOUYI_Z3 || \
	defined BUILD_ZHOUYI_X1 || defined BUILD_ZHOUYI_ALL)
	if (version > 0 && version <= AIPU_ISA_VERSION_ZHOUYI_X1)
		aipu->ops = get_legacy_priv_ops();
#endif

	if (!aipu->ops) {
		ret = -EINVAL;
		dev_err(aipu->dev, "unidentified hardware version number: %d\n", version);
		goto finish;
	}

	ret = init_misc_dev(aipu);
	if (ret)
		goto err_handle;

	ret = aipu_init_mm(&aipu->mm, p_dev, version);
	if (ret)
		goto err_handle;

	ret = init_aipu_job_manager(&aipu->job_manager, &aipu->mm, aipu);
	if (ret)
		goto err_handle;

	aipu->is_init = true;
	goto finish;

err_handle:
	deinit_aipu_priv(aipu);

finish:
	return ret;
}

/**
 * @brief deinit an AIPU private data struct
 * @aipu: pointer to the aipu private struct initialized in init_aipu_priv()
 *
 * Return: 0 on success and error code otherwise.
 */
int deinit_aipu_priv(struct aipu_priv *aipu)
{
	if (!aipu)
		return 0;

	aipu->ops->destroy_partitions(aipu);
	aipu_deinit_mm(&aipu->mm);
	deinit_aipu_job_manager(&aipu->job_manager);
	deinit_misc_dev(aipu);
	aipu->is_init = 0;

	return 0;
}

/**
 * @aipu_priv_get_version() - get AIPU hardware version number wrapper
 * @aipu: pointer to the aipu private struct initialized in init_aipu_priv()
 *
 * Return: AIPU ISA version
 */
int aipu_priv_get_version(struct aipu_priv *aipu)
{
	if (likely(aipu))
		return aipu->version;
	return 0;
}

/**
 * @aipu_priv_get_partition_cnt() - get AIPU partition count
 *        For Z1/Z2/Z3/X1, a *partition* represents an AIPU core
 *        For X2, a *partition* represents a group of AIPU clusters in the same power domain
 * @aipu: pointer to the aipu private struct initialized in init_aipu_priv()
 *
 * Return AIPU partition count
 */
int aipu_priv_get_partition_cnt(struct aipu_priv *aipu)
{
	if (likely(aipu))
		return aipu->partition_cnt;
	return 0;
}

/**
 * @aipu_priv_query_partition_capability() - query AIPU capability wrapper (per partition capability)
 * @aipu: pointer to the aipu private struct initialized in init_aipu_priv()
 * @cap:  pointer to the capability struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_priv_query_partition_capability(struct aipu_priv *aipu, struct aipu_partition_cap *cap)
{
	int id = 0;
	struct aipu_partition *partition = NULL;
	int iter = 0;

	if (unlikely(!aipu && !cap))
		return -EINVAL;

	for (id = 0; id < aipu->partition_cnt; id++) {
		partition = &aipu->partitions[id];
		cap[id].id = id;
		cap[id].arch = partition->arch;
		cap[id].version = partition->version;
		cap[id].config = partition->config;
		cap[id].info.reg_base = partition->reg->phys;
		cap[id].cluster_cnt = partition->cluster_cnt;
		for (iter = 0; iter < partition->cluster_cnt; iter++) {
			cap[id].clusters[iter].core_cnt = partition->clusters[iter].core_cnt;
			cap[id].clusters[iter].tec_cnt = partition->clusters[iter].tec_cnt;
		}
	}

	return 0;
}

/**
 * @aipu_priv_query_capability() - query AIPU capability wrapper (multi-partition common capability)
 * @aipu: pointer to the aipu private struct initialized in init_aipu_priv()
 * @cap:  pointer to the capability struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_priv_query_capability(struct aipu_priv *aipu, struct aipu_cap *cap)
{
	int id = 0;
	struct aipu_partition_cap *ins_cap = NULL;

	if (unlikely(!aipu || !cap))
		return -EINVAL;

	cap->partition_cnt = aipu_priv_get_partition_cnt(aipu);
	cap->is_homogeneous = 1;

	ins_cap = kcalloc(cap->partition_cnt, sizeof(*ins_cap), GFP_KERNEL);
	if (!ins_cap)
		return -ENOMEM;

	aipu_priv_query_partition_capability(aipu, ins_cap);
	for (id = 1; id < cap->partition_cnt; id++) {
		if (ins_cap[id].arch != ins_cap[id - 1].arch ||
		    ins_cap[id].version != ins_cap[id - 1].version ||
		    ins_cap[id].config != ins_cap[id - 1].config) {
			cap->is_homogeneous = 0;
			break;
		}
	}

	if (cap->is_homogeneous)
		cap->partition_cap = ins_cap[0];

	aipu_mm_get_asid(&aipu->mm, cap);
	aipu_mm_get_gm(&aipu->mm, cap);
	cap->dtcm_base = aipu->partitions[0].dtcm_base;
	cap->dtcm_size = aipu->partitions[0].dtcm_size;

	kfree(ins_cap);
	return 0;
}

/**
 * @aipu_priv_io_rw() - AIPU external register read/write wrapper
 * @aipu:   pointer to the aipu private struct initialized in init_aipu_priv()
 * @io_req: pointer to the io_req struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_priv_io_rw(struct aipu_priv *aipu, struct aipu_io_req *io_req)
{
	int ret = -EINVAL;
	int id = 0;

	if (!aipu || !io_req || io_req->core_id >= aipu->partition_cnt)
		return ret;

	id = io_req->core_id;
	return aipu->partitions[id].ops->io_rw(&aipu->partitions[id], io_req);
}

/**
 * @aipu_priv_check_status() - check if aipu status is ready for usage
 * @aipu: pointer to the aipu private struct initialized in init_aipu_priv()
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_priv_check_status(struct aipu_priv *aipu)
{
	if (aipu && aipu->is_init)
		return 0;
	return -EINVAL;
}

inline struct aipu_soc *get_soc(struct aipu_partition *partition)
{
	if (partition && partition->priv)
		return partition->priv->soc;
	return NULL;
}

inline struct aipu_soc_operations *get_soc_ops(struct aipu_partition *partition)
{
	if (partition && partition->priv)
		return partition->priv->soc_ops;
	return NULL;
}

inline struct aipu_job_manager *get_job_manager(struct aipu_partition *partition)
{
	if (partition && partition->priv)
		return &partition->priv->job_manager;
	return NULL;
}