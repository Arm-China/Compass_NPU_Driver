// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include "aipu_job_manager.h"
#include "aipu_priv.h"
#include "aipu_common.h"
#include "zhouyi.h"

static struct aipu_thread_wait_queue *do_create_thread_wait_queue(int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *new_wait_queue =
		kzalloc(sizeof(*new_wait_queue), GFP_KERNEL);

	if (unlikely(!new_wait_queue))
		return ERR_PTR(-ENOMEM);
	new_wait_queue->ref_cnt = 0;
	new_wait_queue->uthread_id = uthread_id;
	new_wait_queue->filp = filp;
	init_waitqueue_head(&new_wait_queue->p_wait);
	INIT_LIST_HEAD(&new_wait_queue->node);
	return new_wait_queue;
}

static struct aipu_thread_wait_queue *get_thread_wait_queue(struct aipu_thread_wait_queue *head,
							    int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *curr = NULL;

	if (unlikely(!head))
		return ERR_PTR(-EINVAL);

	list_for_each_entry(curr, &head->node, node) {
		if ((curr->uthread_id == uthread_id && uthread_id) ||
		    (curr->filp == filp && filp))
			return curr;
	}
	return ERR_PTR(-EINVAL);
}

static struct aipu_thread_wait_queue *create_thread_wait_queue(struct aipu_thread_wait_queue *head,
							       int uthread_id, struct file *filp)
{
	struct aipu_thread_wait_queue *queue = get_thread_wait_queue(head, uthread_id, filp);

	if (IS_ERR(queue)) {
		queue = do_create_thread_wait_queue(uthread_id, filp);
		if (!IS_ERR(queue) && head)
			list_add_tail(&queue->node, &head->node);
		else
			return queue;
	}

	queue->ref_cnt++;
	return queue;
}

static void delete_wait_queue(struct aipu_thread_wait_queue **wait_queue_head)
{
	struct aipu_thread_wait_queue *curr = NULL;
	struct aipu_thread_wait_queue *next = NULL;

	if (wait_queue_head && *wait_queue_head) {
		list_for_each_entry_safe(curr, next, &(*wait_queue_head)->node, node) {
			list_del(&curr->node);
			kfree(curr);
		}
		kfree(*wait_queue_head);
		*wait_queue_head = NULL;
	}
}

static int init_aipu_job(struct aipu_job *job, struct aipu_job_desc *desc,
			 struct aipu_thread_wait_queue *queue, struct file *filp)
{
	if (unlikely(!job))
		return -EINVAL;

	if (likely(desc))
		job->desc = *desc;
	else
		memset(&job->desc, 0, sizeof(job->desc));

	if (queue)
		job->thread_queue = &queue->p_wait;
	else
		job->thread_queue = NULL;

	job->filp = filp;
	job->uthread_id = task_pid_nr(current);
	job->core_id = -1;
	job->state = AIPU_JOB_STATE_IDLE;
	INIT_LIST_HEAD(&job->node);
	job->sched_time = ns_to_ktime(0);
	job->done_time = ns_to_ktime(0);
	job->wake_up = 0;
	job->prev_tail_tcb = 0;

	return 0;
}

static void destroy_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job)
{
	struct aipu_thread_wait_queue *job_aipu_wait_queue = NULL;

	WARN_ON(!job);

	if (likely(job->thread_queue)) {
		job_aipu_wait_queue =
			container_of(job->thread_queue, struct aipu_thread_wait_queue, p_wait);
		job_aipu_wait_queue->ref_cnt--;
	}
	kmem_cache_free(manager->job_cache, job);
}

static struct aipu_job *create_aipu_job(struct aipu_job_manager *manager,
					struct aipu_job_desc *desc,
					struct aipu_thread_wait_queue *queue, struct file *filp)
{
	int ret = 0;
	struct aipu_job *new_aipu_job = NULL;

	new_aipu_job = kmem_cache_alloc(manager->job_cache, GFP_KERNEL);
	if (unlikely(!new_aipu_job))
		return ERR_PTR(-ENOMEM);

	ret = init_aipu_job(new_aipu_job, desc, queue, filp);
	if (unlikely(ret)) {
		destroy_aipu_job(manager, new_aipu_job);
		new_aipu_job = NULL;
		return ERR_PTR(ret);
	}

	return new_aipu_job;
}

static void remove_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job)
{
	WARN_ON(!job);
	list_del(&job->node);
	destroy_aipu_job(manager, job);
}

static void delete_job_queue(struct aipu_job_manager *manager, struct aipu_job **head)
{
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;

	if (head && *head) {
		list_for_each_entry_safe(curr, next, &(*head)->node, node) {
			remove_aipu_job(manager, curr);
		}
		kmem_cache_free(manager->job_cache, *head);
		*head = NULL;
	}
}

inline bool is_job_version_match(struct aipu_partition *core, struct aipu_job_desc *user_job)
{
	if (core->arch == user_job->aipu_arch && user_job->version_compatible)
		return true;

	return (core->arch == user_job->aipu_arch) &&
		(core->version == user_job->aipu_version) &&
		(core->config == user_job->aipu_config);
}

inline bool is_job_ok_for_core(struct aipu_partition *core, struct aipu_job_desc *user_job)
{
	return is_job_version_match(core, user_job) && (core->dtcm_size >> 10) >=
		user_job->dtcm_size_kb;
}

static bool is_user_job_valid(struct aipu_job_manager *manager, struct aipu_job_desc *user_job)
{
	int id = 0;
	struct aipu_partition *core = NULL;
	struct aipu_priv *priv = NULL;

	if (unlikely(!manager || !user_job))
		return false;

	priv = (struct aipu_priv *)manager->priv;

	if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    priv->version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
	    user_job->partition_id < manager->partition_cnt)
		return true;

	if (user_job->is_defer_run) {
		id = user_job->partition_id;
		if (id < manager->partition_cnt)
			return is_job_ok_for_core(&manager->partitions[id], user_job);
		return false;
	}

	for (id = 0; id < manager->partition_cnt; id++) {
		core = &manager->partitions[id];
		if (is_job_ok_for_core(core, user_job))
			return true;
	}

	return false;
}

static int get_available_core_no_lock(struct aipu_job_manager *manager, struct aipu_job *job)
{
	int id = 0;
	struct aipu_partition *core = NULL;

	if (unlikely(!manager))
		return -1;

	for (id = 0; id < manager->partition_cnt; id++) {
		core = &manager->partitions[id];
		if (!atomic_read(&core->disable) && manager->idle_bmap[id] &&
		    is_job_ok_for_core(core, &job->desc))
			return id;
	}

	return -1;
}

static void reserve_core_for_job_no_lock(struct aipu_job_manager *manager, struct aipu_job *job,
					 int do_trigger)
{
	struct aipu_partition *sched_core = NULL;

	WARN_ON(job->core_id < 0);
	WARN_ON(job->core_id >= manager->partition_cnt);

	sched_core = &manager->partitions[job->core_id];
	manager->idle_bmap[job->core_id] = 0;
	if (job->desc.enable_prof) {
		get_soc_ops(sched_core)->start_bw_profiling(sched_core->dev, get_soc(sched_core));
		job->sched_time = ktime_get();
	}

	if (do_trigger)
		job->state = AIPU_JOB_STATE_RUNNING;

	sched_core->ops->reserve(sched_core, &job->desc, do_trigger);

	if (do_trigger)
		dev_dbg(sched_core->dev, "[Job %lld of Thread %d] trigger job running done\n",
			job->desc.job_id, job->uthread_id);
	else
		dev_dbg(sched_core->dev, "[Job %lld of Thread %d] reserve for deferred job done\n",
			job->desc.job_id, job->uthread_id);
}

static int config_exit_tcb(struct aipu_job_manager *manager, struct aipu_buf_desc *desc)
{
	int i = 0;
	struct aipu_tcb *tcb = NULL;

	tcb = aipu_mm_get_tcb_va(manager->mm, desc->pa);
	if (!tcb) {
		dev_err(manager->dev, "buffer for exit TCB not found (0x%llx)\n", desc->pa);
		return -EINVAL;
	}

	memset(tcb, 0, sizeof(*tcb));

	tcb->flag = TCB_FLAG_TASK_TYPE_INIT;
	tcb->next = 0;
	tcb->gm_ctrl = 0xF;
	tcb->gm_rgnx_ctrl[0] = 0xC0000000;
	tcb->gm_rgnx_ctrl[0] = 0xC0000000;
	for (i = AIPU_BUF_ASID_0; i < ZHOUYI_ASID_COUNT; i++) {
		u64 base = aipu_mm_get_asid_base(manager->mm, i);
		u64 size = aipu_mm_get_asid_size(manager->mm, i);

		if (size) {
			tcb->asids[i].v32.lo = (base | ASID_RD | ASID_WR) & U32_MAX;
			tcb->asids[i].v32.hi = base >> 32;
		}
	}
	return 0;
}

static int schedule_v3_job_no_lock(struct aipu_job_manager *manager, struct aipu_job *job)
{
	int ret = 0;
	int trigger_type = 0;
	int partition_id = job->desc.partition_id;
	struct aipu_partition *partition = &manager->partitions[partition_id];
	struct command_pool *pool = &manager->pools[partition_id];
	struct qos *qlist = NULL;

	ret = aipu_mm_set_tcb_tail(manager->mm, job->desc.tail_tcb_pa);
	if (ret)
		return ret;

	if (job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_QOS_SLOW)
		qlist = &pool->qlist[AIPU_JOB_QOS_SLOW];
	else
		qlist = &pool->qlist[AIPU_JOB_QOS_FAST];

	if (!pool->created)
		trigger_type = ZHOUYI_V3_TRIGGER_TYPE_CREATE;
	else if (pool->aborted || !qlist->pool_head)
		trigger_type = ZHOUYI_V3_TRIGGER_TYPE_UPDATE_DISPATCH;
	else
		trigger_type = ZHOUYI_V3_TRIGGER_TYPE_DISPATCH;

	/* Driver will clean related TCBs in the list as soon as the job is done.
	 * If userspace schedules a TCB chain already in the list end, it means that
	 * an exit dispatch should be done before these TCBs can be resued.
	 */
	if ((job->desc.tail_tcb_pa == qlist->curr_tail && !pool->aborted) ||
	    (trigger_type == ZHOUYI_V3_TRIGGER_TYPE_CREATE &&
	     (job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SINGLE_GROUP))) {
		ret = config_exit_tcb(manager, &manager->exit_tcb);
		if (ret)
			return ret;

		if (trigger_type != ZHOUYI_V3_TRIGGER_TYPE_CREATE) {
			ret = aipu_mm_link_tcb(manager->mm, qlist->curr_tail,
					       manager->exit_tcb.pa, 0);
			if (ret)
				return ret;
		}

		ret = partition->ops->exit_dispatch(partition, job->desc.exec_flag,
						    manager->exit_tcb.pa);
		if (ret) {
			dev_err(partition->dev, "exit dispatch failed: job ID 0x%llx",
				job->desc.job_id);
			return ret;
		}

		qlist->curr_tail = manager->exit_tcb.pa;

		if (trigger_type != ZHOUYI_V3_TRIGGER_TYPE_CREATE) {
			ret = aipu_mm_unlink_tcb(manager->mm, job->desc.tail_tcb_pa);
			if (ret)
				return ret;
		}
	}

	if (!qlist->pool_head) {
		qlist->pool_head = job->desc.head_tcb_pa;
		job->prev_tail_tcb = 0;
	} else {
		ret = aipu_mm_link_tcb(manager->mm, qlist->curr_tail,
				       job->desc.head_tcb_pa, job->desc.job_id);
		if (ret)
			return ret;

		job->prev_tail_tcb = qlist->curr_tail;
	}

	qlist->curr_head = job->desc.head_tcb_pa;
	qlist->curr_tail = job->desc.tail_tcb_pa;

	ret = partition->ops->reserve(partition, &job->desc, trigger_type);
	if (!ret && trigger_type == ZHOUYI_V3_TRIGGER_TYPE_CREATE)
		pool->created = true;

	if (!ret && pool->aborted)
		pool->aborted = false;

	return ret;
}

static int schedule_new_job(struct aipu_job_manager *manager, struct aipu_job_desc *user_job,
			    struct file *filp, int do_trigger)
{
	int ret = 0;
	struct aipu_job *kern_job = NULL;
	struct aipu_thread_wait_queue *queue = NULL;
	unsigned long flags;

	mutex_lock(&manager->wq_lock);
	if (user_job->enable_poll_opt)
		queue = create_thread_wait_queue(manager->wait_queue_head, 0, filp);
	else
		queue = create_thread_wait_queue(manager->wait_queue_head,
						 task_pid_nr(current), NULL);
	mutex_unlock(&manager->wq_lock);

	WARN_ON(IS_ERR(queue));

	kern_job = create_aipu_job(manager, user_job, queue, filp);
	if (IS_ERR(kern_job))
		return PTR_ERR(kern_job);

	spin_lock_irqsave(&manager->lock, flags);
	if (do_trigger) {
		kern_job->state = AIPU_JOB_STATE_PENDING;
		list_add_tail(&kern_job->node, &manager->scheduled_head->node);

		if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
			ret = schedule_v3_job_no_lock(manager, kern_job);
			if (!ret)
				kern_job->state = AIPU_JOB_STATE_RUNNING;
			else
				kern_job->state = AIPU_JOB_STATE_DEFERRED; /* TSM is full */
		} else {
			/*
			 * For a job using SRAM managed by AIPU Gbuilder, it should be
			 * executed exclusively in serial with other Gbuilder-managed-SRAM ones,
			 * and parallel scheduling is not allowed.
			 *
			 * Pending it if there has been a Gbuilder-managed-SRAM job running;
			 * otherwise mark the flag and reserve a core for running.
			 */
			if (kern_job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX) {
				if (manager->exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
					goto unlock;
				else
					manager->exec_flag |= AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;
			}
			kern_job->core_id = get_available_core_no_lock(manager, kern_job);
			if (kern_job->core_id >= 0)
				reserve_core_for_job_no_lock(manager, kern_job, do_trigger);
		}
	} else {
		if (user_job->aipu_version < AIPU_ISA_VERSION_ZHOUYI_V3 &&
		    (user_job->partition_id >= manager->partition_cnt ||
		     !manager->idle_bmap[user_job->partition_id])) {
			dev_err(manager->dev, "schedule new job (0x%llx) failed: invalid core ID %u",
				kern_job->desc.job_id, user_job->partition_id);
			ret = -EINVAL;
			goto unlock;
		}

		kern_job->state = AIPU_JOB_STATE_DEFERRED;
		kern_job->core_id = user_job->partition_id; /* only valid for v1 & v2 */
		list_add_tail(&kern_job->node, &manager->scheduled_head->node);

		if (user_job->aipu_version < AIPU_ISA_VERSION_ZHOUYI_V3)
			reserve_core_for_job_no_lock(manager, kern_job, do_trigger);
	}
unlock:
	spin_unlock_irqrestore(&manager->lock, flags);
	return ret;
}

static int trigger_deferred_job_run(struct aipu_job_manager *manager,
				    struct aipu_job_desc *user_job)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;
	struct aipu_partition *sched_core = NULL;
	int triggered = 0;

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->uthread_id == task_pid_nr(current) &&
		    curr->desc.job_id == user_job->job_id &&
		    curr->state == AIPU_JOB_STATE_DEFERRED) {
			curr->state = AIPU_JOB_STATE_RUNNING;
			if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
				/**
				 * for debugger: it should ensure that the NPUs are free to accept
				 * new jobs before the deferred job is scheduled.
				 */
				schedule_v3_job_no_lock(manager, curr);
			} else {
				sched_core = &manager->partitions[curr->core_id];
				sched_core->ops->trigger(sched_core);
			}
			triggered = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	if (!triggered) {
		dev_err(manager->dev, "trigger deferred job (0x%llx) failed",
			user_job->job_id);
		return -EINVAL;
	}

	return 0;
}

/**
 * @init_aipu_job_manager() - initialize an existing job manager struct during driver probe phase
 * @manager: pointer to the struct job_manager struct to be initialized
 * @mm: pointer to the struct aipu_memory_manager
 * @priv: pointer to the struct aipu_priv
 *
 * Return: 0 on success and error code otherwise.
 */
int init_aipu_job_manager(struct aipu_job_manager *manager, struct aipu_memory_manager *mm,
			  void *priv)
{
	int ret = 0;
	struct aipu_cap cap;
	struct aipu_buf_request buf;

	if (!manager || !mm || !priv)
		return -EINVAL;

	manager->is_init = 0;
	manager->version = ((struct aipu_priv *)priv)->version;
	manager->dev = ((struct aipu_priv *)priv)->dev;
	manager->partition_cnt = 0;
	manager->partitions = NULL;
	manager->pools = NULL;
	manager->idle_bmap = NULL;
	manager->job_cache =
		kmem_cache_create("aipu_job_cache", sizeof(struct aipu_job), 0, SLAB_PANIC, NULL);
	manager->scheduled_head = create_aipu_job(manager, NULL, NULL, NULL);
	INIT_LIST_HEAD(&manager->scheduled_head->node);
	spin_lock_init(&manager->lock);
	manager->wait_queue_head = create_thread_wait_queue(NULL, 0, NULL);
	mutex_init(&manager->wq_lock);
	manager->exec_flag = 0;
	manager->mm = mm;
	manager->priv = priv;
	aipu_mm_get_asid(mm, &cap);
	manager->asid0_base = cap.asid0_base;
	atomic_set(&manager->tick_counter, 0);
	atomic_set(&manager->is_suspend, 0);

	WARN_ON(IS_ERR(manager->scheduled_head));
	WARN_ON(IS_ERR(manager->wait_queue_head));

	if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		memset(&buf, 0, sizeof(buf));
		buf.bytes = sizeof(struct aipu_tcb);
		buf.align_in_page = 1;
		buf.data_type = AIPU_MM_DATA_TYPE_TCB;
		ret = aipu_mm_alloc(mm, &buf, NULL);
		if (ret) {
			dev_err(manager->dev, "init job manager failed: TCB alloc failed");
			return ret;
		}

		manager->exit_tcb = buf.desc;
		ret = config_exit_tcb(manager, &buf.desc);
		if (ret) {
			dev_err(manager->dev, "init job manager failed: config exit TCB failed");
			return ret;
		}
		ret = aipu_mm_set_tcb_tail(manager->mm, manager->exit_tcb.pa);
		if (ret) {
			dev_err(manager->dev, "init job manager failed: set TCB tail failed");
			return ret;
		}
	} else {
		memset(&manager->exit_tcb, 0, sizeof(manager->exit_tcb));
	}

	manager->is_init = 1;
	return ret;
}

/**
 * @deinit_aipu_job_manager() - de-init the job manager
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 */
void deinit_aipu_job_manager(struct aipu_job_manager *manager)
{
	if (!manager || !manager->is_init)
		return;

	kfree(manager->idle_bmap);
	manager->idle_bmap = NULL;
	delete_job_queue(manager, &manager->scheduled_head);
	delete_wait_queue(&manager->wait_queue_head);
	mutex_destroy(&manager->wq_lock);
	kmem_cache_destroy(manager->job_cache);
	manager->job_cache = NULL;
	manager->is_init = 0;
	aipu_mm_free(manager->mm, &manager->exit_tcb, NULL);
}

/**
 * @aipu_job_manager_set_partitions_info() - set multicore info. while probing
 * @manager:  pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @partition_cnt: AIPU core count
 * @partitions:    pointer to AIPU core struct array
 */
void aipu_job_manager_set_partitions_info(struct aipu_job_manager *manager, int partition_cnt,
					  struct aipu_partition *partitions)
{
	WARN_ON(!manager || !partition_cnt || !partitions);
	manager->partition_cnt = partition_cnt;
	manager->partitions = partitions;
	manager->pools = devm_kzalloc(partitions[0].dev, partition_cnt * sizeof(*manager->pools),
				      GFP_KERNEL);
	kfree(manager->idle_bmap);
	manager->idle_bmap = kmalloc_array(partition_cnt, sizeof(bool), GFP_KERNEL);
	memset(manager->idle_bmap, 1, partition_cnt);
}

/**
 * @aipu_job_manager_scheduler() - schedule a job flushed from userland
 * @manager:  pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @user_job: pointer to the userspace job descriptor
 * @filp:     pointer to the device char file
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_scheduler(struct aipu_job_manager *manager, struct aipu_job_desc *user_job,
			       struct file *filp)
{
	int ret = 0;

	if (unlikely(!manager || !user_job || !filp))
		return -EINVAL;

	if (unlikely(!is_user_job_valid(manager, user_job))) {
		dev_err(manager->dev, "[scheduler] invalid user job (0x%llx)", user_job->job_id);
		return -EINVAL;
	}

	if (atomic_read(&manager->is_suspend)) {
		dev_err(manager->dev, "[scheduler] the NPU hw is not available now");
		return -ENODEV;
	}

	if (!user_job->is_defer_run)
		ret = schedule_new_job(manager, user_job, filp, 1);
	else if (!user_job->do_trigger)
		ret = schedule_new_job(manager, user_job, filp, 0);
	else
		ret = trigger_deferred_job_run(manager, user_job);

	if (ret)
		dev_err(manager->dev,
			"[scheduler] schedule job (0x%llx) failed: is_defer_run %d, do_trigger %d",
			user_job->job_id, user_job->is_defer_run, user_job->do_trigger);
	return ret;
}

static void aipu_job_manager_real_time_printk(struct aipu_job_manager *manager,
					      struct aipu_partition *partition,
					      struct job_irq_info *info)
{
	struct aipu_tcb *tcb = NULL;
	char *buf = NULL;

	if (GET_PRINF_SIZE(info->sig_flag)) {
		/* here: tail TCBP is the exact TCB sending a printf signal */
		tcb = aipu_mm_get_tcb_va(manager->mm, info->tail_tcbp);
		if (!tcb)
			dev_dbg(partition->dev, "real time printk: no TCB found (0x%x)\n",
				info->tail_tcbp);

		buf = aipu_mm_get_va(manager->mm, manager->asid0_base + tcb->pprint);
		if (buf)
			dev_info(partition->dev, "[real-time printk 0x%x] %s", tcb->pprint, buf);
		else
			dev_dbg(partition->dev, "real time printk: no pbuf found (0x%x)\n",
				tcb->pprint);
	}
}

static bool is_v3_job_done_or_excep(struct aipu_job *job, struct job_irq_info *info,
				    u64 asid_base, int flag)
{
	return info->tail_tcbp == (u32)(job->desc.last_task_tcb_pa - asid_base) &&
	       (IS_DONE_IRQ(flag) || IS_EXCEPTION_IRQ(flag));
}

static bool do_abortion(int flag, struct job_irq_info *info)
{
	if (!info)
		return false;

	return IS_SERIOUS_ERR(flag) || IS_EXCEPTION_SIGNAL(info->sig_flag);
}

static bool is_job_end(struct aipu_job *job, struct aipu_partition *partition,
		       struct job_irq_info *info, u64 asid_base, int flag)
{
	if (job->state != AIPU_JOB_STATE_RUNNING)
		return false;

	if (job->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3)
		return is_v3_job_done_or_excep(job, info, asid_base, flag) ||
			do_abortion(flag, info);
	return job->core_id == partition->id;
}

static bool is_job_abnormal(struct aipu_job *job, int flag, struct job_irq_info *info)
{
	if (job->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3)
		return IS_ABNORMAL(flag) || IS_EXCEPTION_SIGNAL(info->sig_flag);
	return flag != 0;
}

/**
 * @aipu_job_manager_irq_upper_half() - aipu interrupt upper half handler
 * @core:           pointer to the aipu core struct
 * @exception_flag: exception flag
 * @info:           pointer to struct job interrupt info. (for v3 only)
 */
void aipu_job_manager_irq_upper_half(struct aipu_partition *partition, int flag,
				     struct job_irq_info *info)
{
	struct aipu_job *curr = NULL;
	struct aipu_job_manager *manager = NULL;
	int handled = 0;
	int triggered = 0;

	if (unlikely(!partition))
		return;

	manager = get_job_manager(partition);

	if (IS_SIGNAL_IRQ(flag) && IS_PRINTF_SIGNAL(info->sig_flag))
		aipu_job_manager_real_time_printk(manager, partition, info);

	spin_lock(&manager->lock);
	if (do_abortion(flag, info)) {
		partition->ops->abort_command_pool(partition);
		manager->pools[partition->id].aborted = true;
	}

	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (is_job_end(curr, partition, info, manager->asid0_base, flag)) {
			if (unlikely(is_job_abnormal(curr, flag, info)))
				curr->state = AIPU_JOB_STATE_EXCEP;
			else
				curr->state = AIPU_JOB_STATE_SUCCESS;

			if (curr->desc.enable_prof) {
				curr->done_time = ktime_get();
				get_soc_ops(partition)->stop_bw_profiling(partition->dev,
									  get_soc(partition));
				get_soc_ops(partition)->read_profiling_reg(partition->dev,
									   get_soc(partition),
									   &curr->pdata);
			}

			if (atomic_read(&manager->tick_counter) && info)
				curr->pdata.tick_counter = info->tick_counter;
			else
				curr->pdata.tick_counter = 0;

			if (curr->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
				manager->exec_flag &= ~AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;

			handled = 1;
			break;
		}
	}

	/* handled == false means a job was invalidated before done */

	if (!atomic_read(&partition->disable)) {
		list_for_each_entry(curr, &manager->scheduled_head->node, node) {
			if (curr->state == AIPU_JOB_STATE_PENDING &&
			    is_job_ok_for_core(partition, &curr->desc)) {
				if (curr->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX) {
					if (manager->exec_flag & AIPU_JOB_EXEC_FLAG_SRAM_MUTEX)
						continue;
					else
						manager->exec_flag |= AIPU_JOB_EXEC_FLAG_SRAM_MUTEX;
				}
				curr->core_id = partition->id;
				reserve_core_for_job_no_lock(manager, curr, 1);
				triggered = 1;
				break;
			}
		}
	}

	if (!triggered)
		manager->idle_bmap[partition->id] = 1;

	spin_unlock(&manager->lock);
}

static void aipu_job_manager_destroy_command_pool_no_lock(struct aipu_job_manager *manager,
							  struct aipu_partition *partition)
{
	struct command_pool *pool;

	if (!manager || !partition)
		return;

	pool = manager->pools;
	if (pool && pool->created) {
		partition->ops->destroy_command_pool(partition);
		memset(pool->qlist, 0, sizeof(*pool->qlist) * AIPU_JOB_QOS_MAX);
		pool->created = false;
	}
}

/**
 * @aipu_job_manager_irq_bottom_half() - aipu interrupt bottom half handler
 * @core: pointer to the aipu core struct
 */
void aipu_job_manager_irq_bottom_half(struct aipu_partition *core)
{
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	struct aipu_job_manager *manager = NULL;
	unsigned long flags;
	bool do_destroy = core->version == AIPU_ISA_VERSION_ZHOUYI_V3;

	if (unlikely(!core))
		return;

	manager = get_job_manager(core);

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->state >= AIPU_JOB_STATE_EXCEP && !curr->wake_up &&
		    (curr->desc.aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3 ||
		     curr->core_id == core->id)) {
			if (curr->desc.enable_prof)
				curr->pdata.execution_time_ns =
				  (long)ktime_to_ns(ktime_sub(curr->done_time, curr->sched_time));

			if (curr->desc.aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3 &&
			    curr->prev_tail_tcb)
				aipu_mm_unlink_tcb(manager->mm, curr->prev_tail_tcb);
		}

		/* destroy the v3 command pool if all jobs are done */
		if (curr->state < AIPU_JOB_STATE_EXCEP)
			do_destroy = false;
	}

	if (!is_grid_end(manager->mm, manager->pools->qlist[AIPU_JOB_QOS_SLOW].curr_tail) ||
	    !is_grid_end(manager->mm, manager->pools->qlist[AIPU_JOB_QOS_FAST].curr_tail))
		do_destroy = false;

	if (do_destroy) {
		aipu_job_manager_destroy_command_pool_no_lock(manager, core);
	} else {
		aipu_mm_pin_tcb(manager->mm, manager->pools->qlist[AIPU_JOB_QOS_SLOW].curr_tail);
		aipu_mm_pin_tcb(manager->mm, manager->pools->qlist[AIPU_JOB_QOS_FAST].curr_tail);
	}

	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->state >= AIPU_JOB_STATE_EXCEP && !curr->wake_up &&
		    (curr->desc.aipu_version >= AIPU_ISA_VERSION_ZHOUYI_V3 ||
		     curr->core_id == core->id)) {
			wake_up_interruptible(curr->thread_queue);
			curr->wake_up = 1;
#ifdef DEBUG
			/* debug */
			print_core_id(manager->mm, curr->desc.head_tcb_pa, curr->desc.tail_tcb_pa);
#endif
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);
}

int aipu_job_manager_abort_cmd_pool(struct aipu_job_manager *manager)
{
	int ret = 0;
	unsigned long flags;
	struct aipu_partition *partition = NULL;
	struct aipu_job *curr = NULL;

	if (!manager)
		return -EINVAL;

	partition = &manager->partitions[0];

	spin_lock_irqsave(&manager->lock, flags);
	partition->ops->abort_command_pool(partition);
	manager->pools[partition->id].aborted = true;

	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state == AIPU_JOB_STATE_SUCCESS)
			continue;
		curr->state = AIPU_JOB_STATE_EXCEP;
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	aipu_job_manager_irq_bottom_half(partition);

	return ret;
}

/**
 * @aipu_job_manager_cancel_jobs() - cancel all jobs flushed for certain user closing fd
 * @param manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @param filp: file struct pointer
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_cancel_jobs(struct aipu_job_manager *manager, struct file *filp)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	struct aipu_thread_wait_queue *curr_wq = NULL;
	struct aipu_thread_wait_queue *next_wq = NULL;
	struct aipu_partition *par = NULL;

	if (!manager || !filp)
		return -EINVAL;

	/* jobs should be cleaned first */
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->filp == filp) {
			par = &manager->partitions[curr->core_id];
			if (curr->desc.aipu_version < AIPU_ISA_VERSION_ZHOUYI_V3 &&
			    (curr->state == AIPU_JOB_STATE_DEFERRED ||
			     (curr->state == AIPU_JOB_STATE_RUNNING && par->ops->is_idle(par))))
				manager->idle_bmap[curr->core_id] = 1;
			remove_aipu_job(manager, curr);
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	mutex_lock(&manager->wq_lock);
	list_for_each_entry_safe(curr_wq, next_wq, &manager->wait_queue_head->node, node) {
		if (!curr_wq->ref_cnt) {
			list_del(&curr_wq->node);
			kfree(curr_wq);
		}
	}
	mutex_unlock(&manager->wq_lock);

	return 0;
}

/**
 * @aipu_job_manager_invalidate_timeout_job() - invalidate/kill a timeout job
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @job_id:  job ID
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_invalidate_timeout_job(struct aipu_job_manager *manager, int job_id)
{
	int ret = 0;
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	unsigned long flags;

	if (!manager)
		return -EINVAL;

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (curr->uthread_id == task_pid_nr(current) &&
		    curr->desc.job_id == job_id) {
			remove_aipu_job(manager, curr);
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	return ret;
}

/**
 * @aipu_job_manager_get_job_status() - get AIPU jobs' statuses after a polling event returns
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @job_status: job status array stores statuys info filled by this API
 * @filp: file struct pointer
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_job_manager_get_job_status(struct aipu_job_manager *manager,
				    struct aipu_job_status_query *job_status, struct file *filp)
{
	int ret = 0;
	struct aipu_job_status_desc *status = NULL;
	struct aipu_job *curr = NULL;
	struct aipu_job *next = NULL;
	int poll_iter = 0;
	unsigned long flags;

	if (unlikely(!manager || !job_status || job_status->max_cnt < 1))
		return -EINVAL;

	status = kcalloc(job_status->max_cnt, sizeof(*status), GFP_KERNEL);
	if (!status)
		return -ENOMEM;

	job_status->poll_cnt = 0;
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry_safe(curr, next, &manager->scheduled_head->node, node) {
		if (job_status->poll_cnt == job_status->max_cnt)
			break;

		if (curr->state < AIPU_JOB_STATE_EXCEP)
			continue;

		if (curr->filp != filp)
			continue;

		if ((job_status->of_this_thread && curr->uthread_id == task_pid_nr(current)) ||
		    !job_status->of_this_thread) {
			status[poll_iter].job_id = curr->desc.job_id;
			status[poll_iter].thread_id = curr->uthread_id;
			status[poll_iter].state = (curr->state == AIPU_JOB_STATE_SUCCESS) ?
			    AIPU_JOB_STATE_DONE : AIPU_JOB_STATE_EXCEPTION;
			if (curr->desc.enable_prof || curr->pdata.tick_counter)
				status[poll_iter].pdata = curr->pdata;

			remove_aipu_job(manager, curr);
			curr = NULL;

			job_status->poll_cnt++;
			poll_iter++;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	ret = copy_to_user((struct job_status_desc __user *)job_status->status, status,
			   job_status->poll_cnt * sizeof(*status));

	kfree(status);
	return ret;
}

/**
 * @aipu_job_manager_has_end_job() - check if a user thread has end job(s) to query
 * @manager: pointer to the struct job_manager initialized in init_aipu_job_manager()
 * @filp: file struct pointer
 * @wait: wait table struct from kernel
 * @uthread_id: thread ID
 *
 * Return: true if there is/are AIPU job(s) done and false otherwise.
 */
bool aipu_job_manager_has_end_job(struct aipu_job_manager *manager, struct file *filp,
				  struct poll_table_struct *wait, int uthread_id)
{
	bool ret = false;
	struct aipu_job *curr = NULL;
	struct aipu_thread_wait_queue *wq = NULL;
	unsigned long flags;

	if (unlikely(!manager || !filp))
		return -EINVAL;

	mutex_lock(&manager->wq_lock);
	list_for_each_entry(wq, &manager->wait_queue_head->node, node) {
		if (wq->uthread_id == uthread_id || wq->filp == filp) {
			poll_wait(filp, &wq->p_wait, wait);
			break;
		}
	}
	mutex_unlock(&manager->wq_lock);

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->filp == filp &&
		    curr->wake_up == 1 &&
		    (curr->desc.enable_poll_opt || curr->uthread_id == uthread_id)) {
			ret = true;
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	return ret;
}

int aipu_job_manager_get_hw_status(struct aipu_job_manager *manager, struct aipu_hw_status *hw)
{
	unsigned long flags;
	struct aipu_job *curr = NULL;

	if (!manager || !hw)
		return -EINVAL;

	hw->status = AIPU_STATUS_IDLE;
	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		/* exception jobs should be cleared and hw is reset */
		if (curr->state == AIPU_JOB_STATE_RUNNING ||
		    curr->state == AIPU_JOB_STATE_DEFERRED) {
			hw->status = AIPU_STATUS_BUSY;
			break;
		}
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	return 0;
}

int aipu_job_manager_disable_tick_counter(struct aipu_job_manager *manager)
{
	if (!manager)
		return -EINVAL;

	if (manager->version != AIPU_ISA_VERSION_ZHOUYI_V3 || !atomic_read(&manager->tick_counter))
		return 0;

	if (atomic_dec_and_test(&manager->tick_counter))
		manager->partitions[0].ops->disable_tick_counter(&manager->partitions[0]);

	return 0;
}

int aipu_job_manager_enable_tick_counter(struct aipu_job_manager *manager)
{
	if (!manager)
		return -EINVAL;

	if (manager->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return 0;

	if (atomic_inc_return(&manager->tick_counter) == 1)
		manager->partitions[0].ops->enable_tick_counter(&manager->partitions[0]);

	return 0;
}

int aipu_job_manager_config_clusters(struct aipu_job_manager *manager,
				     struct aipu_config_clusters *cfg)
{
	int ret = 0;
	unsigned long flags;
	struct aipu_job *curr = NULL;
	struct aipu_partition *partition = NULL;
	int idx = 0;
	u32 en_count, core_cnt;

	if (!manager || !cfg)
		return -EINVAL;

	if (manager->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return 0;

	partition = &manager->partitions[0];

	spin_lock_irqsave(&manager->lock, flags);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->state == AIPU_JOB_STATE_RUNNING ||
		    curr->state == AIPU_JOB_STATE_DEFERRED) {
			ret = -EBUSY;
			dev_err(manager->dev, "config clusters failed: aipu is busy now");
			goto unlock;
		}
	}

	for (idx = 0; idx < partition->cluster_cnt; idx++) {
		en_count = cfg->clusters[idx].en_core_cnt;
		core_cnt = partition->clusters[idx].core_cnt;

		if (en_count > core_cnt) {
			ret = -EINVAL;
			dev_err(manager->dev, "invalid enable core count: %u (> tot count %u)",
				en_count, core_cnt);
			goto unlock;
		}
		partition->ops->enable_core_cnt(partition, idx, en_count);
	}

	atomic_set(&manager->is_suspend, !en_count);

unlock:
	spin_unlock_irqrestore(&manager->lock, flags);
	return ret;
}

int aipu_job_manager_suspend(struct aipu_job_manager *manager)
{
	struct aipu_config_clusters cfg;

	if (!manager)
		return -EINVAL;

	memset(&cfg, 0, sizeof(cfg));

	if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3)
		return aipu_job_manager_config_clusters(manager, &cfg);

	atomic_set(&manager->is_suspend, 1);
	return 0;
}

int aipu_job_manager_resume(struct aipu_job_manager *manager)
{
	struct aipu_config_clusters cfg;

	if (!manager)
		return -EINVAL;

	cfg.clusters[0].en_core_cnt = manager->partitions[0].clusters[0].core_cnt;

	if (manager->version == AIPU_ISA_VERSION_ZHOUYI_V3)
		return aipu_job_manager_config_clusters(manager, &cfg);

	atomic_set(&manager->is_suspend, 0);
	return 0;
}
