// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. */

#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/file.h>
#include <linux/fs.h>
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

static int init_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job,
			 struct aipu_job_desc *desc, struct aipu_thread_wait_queue *queue,
			 struct file *filp)
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
	job->prof_filp = NULL;
#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	job->prof_head = kmem_cache_alloc(manager->prof_cache, GFP_KERNEL);
	job->prof_head->data = NULL;
	job->prof_head->size = 0;
	INIT_LIST_HEAD(&job->prof_head->node);
#else
	job->prof_head = NULL;
#endif

	return 0;
}

static void destroy_aipu_job(struct aipu_job_manager *manager, struct aipu_job *job)
{
	struct aipu_thread_wait_queue *job_aipu_wait_queue = NULL;

	WARN_ON(!job);

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	if (job->prof_filp) {
		fput(job->prof_filp);
		job->prof_filp = NULL;
	}

	if (job->prof_head) {
		struct profiler *curr = NULL;
		struct profiler *next = NULL;

		list_for_each_entry_safe(curr, next, &job->prof_head->node, node) {
			kfree(curr->data);
			curr->data = NULL;
			curr->size = 0;
			kmem_cache_free(manager->prof_cache, curr);
		}

		kmem_cache_free(manager->prof_cache, job->prof_head);
		job->prof_head = NULL;
	}
#endif

	if (likely(job->thread_queue)) {
		job_aipu_wait_queue =
			container_of(job->thread_queue, struct aipu_thread_wait_queue, p_wait);
		if (job_aipu_wait_queue->ref_cnt)
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

	ret = init_aipu_job(manager, new_aipu_job, desc, queue, filp);
	if (unlikely(ret)) {
		destroy_aipu_job(manager, new_aipu_job);
		new_aipu_job = NULL;
		return ERR_PTR(ret);
	}

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	if (desc && desc->profile_fd) {
		new_aipu_job->prof_filp = fget(desc->profile_fd);
		if (IS_ERR(new_aipu_job->prof_filp)) {
			dev_err(manager->dev, "get profiler struct file failed: fd %llu\n",
				desc->profile_fd);
			new_aipu_job->prof_filp = NULL;
		}
	}
#endif

	if (queue)
		queue->ref_cnt++;
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
	if (!is_job_version_match(core, user_job)) {
		dev_err(core->dev, "invalid specified arch %d or version %d or configuration %d\n",
			core->arch, core->version, core->config);
		return false;
	}

	if (user_job->dtcm_size_kb > (core->dtcm_size >> 10)) {
		dev_err(core->dev, "the requested dtcm size (%dKB) > total dtcm size (%dKB)\n",
			user_job->dtcm_size_kb, core->dtcm_size >> 10);
		return false;
	}

	return true;
}

static bool is_user_job_valid(struct aipu_job_manager *manager, struct aipu_job_desc *user_job)
{
	int partition_id = user_job->partition_id;
	int core_id = user_job->core_id;
	int core_cnt = 0;
	struct aipu_partition *partition = NULL;
	struct aipu_partition *core = NULL;

	/* for v3: partition->clusters->cores arch */
	if (user_job->aipu_version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (manager->version != AIPU_ISA_VERSION_ZHOUYI_V3 ||
		    partition_id >= manager->partition_cnt) {
			dev_err(manager->dev, "invalid version number (%d) or partition ID (%d)",
				user_job->aipu_version, partition_id);
			return false;
		}

		if (user_job->exec_flag & AIPU_JOB_EXEC_FLAG_DBG_DISPATCH) {
			partition = &manager->partitions[partition_id];
			core_cnt = atomic_read(&partition->clusters[0].en_core_cnt);
			if (core_id >= core_cnt) {
				dev_err(partition->dev, "invalid core ID (%d) >= enabled core count (%d)",
					core_id, core_cnt);
				return false;
			}
		}

		return true;
	}

	/* for non-v3 archs, a partition just means an AIPU core */
	core_cnt = manager->partition_cnt;

	/* debugger deferred execution: reserve the specified core ID */
	if (user_job->is_defer_run) {
		if (core_id >= core_cnt) {
			dev_err(manager->dev, "invalid core ID (%d) >= core count (%d)",
				core_id, core_cnt);
			return false;
		}

		core = &manager->partitions[core_id];
		return is_job_ok_for_core(core, user_job);
	}

	/* for non-v3 normal execution: find an available core ID */
	for (core_id = 0; core_id < core_cnt; core_id++) {
		core = &manager->partitions[core_id];
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

static int config_exit_tcb(struct aipu_job_manager *manager, struct aipu_tcb *tcb)
{
	int i = 0;

	if (!tcb)
		return -EINVAL;

	memset(tcb, 0, sizeof(*tcb));

	tcb->flag = TCB_FLAG_TASK_TYPE_INIT;
	tcb->next = 0;
	tcb->gm_ctrl = 0xF;
	tcb->gm_rgnx_ctrl[0] = 0xC0000000;
	tcb->gm_rgnx_ctrl[1] = 0xC0000000;
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

static void check_enable_tec_interrupts(struct aipu_job_manager *manager, struct aipu_job *job)
{
	struct aipu_tcb *tcb = NULL;

	if (!manager || !job || manager->tec_intr_en)
		return;

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	if (job->desc.profile_fd > 0)
		goto enable_tec_intr;
#endif

	tcb = aipu_mm_get_tcb(manager->mm, job->desc.last_task_tcb_pa);
	if (!tcb) {
		dev_err(manager->dev, "get TCB va failed: 0x%llx\n", job->desc.last_task_tcb_pa);
		return;
	}

	if (!tcb->pprint)
		return;

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
enable_tec_intr:
#endif
	manager->partitions[0].ops->enable_interrupt(&manager->partitions[0], true);
	manager->tec_intr_en = true;
	dev_info(manager->dev, "TEC interrupts are enabled\n");
}

static int schedule_v3_job_no_lock(struct aipu_job_manager *manager, struct aipu_job *job)
{
	int ret = 0;
	int trigger_type = 0;
	int partition_id = job->desc.partition_id;
	struct aipu_partition *partition = &manager->partitions[partition_id];
	struct command_pool *pool = &manager->pools[partition_id];
	struct qos *qlist = NULL;
	struct aipu_tcb *tail_tcb = NULL;

	tail_tcb = aipu_mm_set_tcb_tail(manager->mm, job->desc.tail_tcb_pa);
	if (!tail_tcb) {
		dev_err(partition->dev, "[%d] get tail TCB failed: 0x%llx\n",
			task_pid_nr(current), job->desc.tail_tcb_pa);
		return -EINVAL;
	}

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

	check_enable_tec_interrupts(manager, job);

	/**
	 * 1. debug-dispatch tasks cannot be linked to any existing command pool.
	 * 2. command pool containing debug-dispatch tasks cannot be linked by any task TCB;
	 * 3. we must create a new command pool to enable debug-dispatch.
	 */
	if (pool->debug) {
		dev_err(partition->dev, "debug-dispatch command pool cannot be linked");
		return -EINVAL;
	}

	if (job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_DBG_DISPATCH)
		trigger_type = ZHOUYI_V3_TRIGGER_TYPE_DEBUG_DISPATCH;

	/* remove init tcb of single group jobs to make them run in parallel on multi-core */
	if (atomic_read(&partition->clusters[0].en_core_cnt) != 1 &&
	    pool->last_exec_flag & AIPU_JOB_EXEC_FLAG_SINGLE_GROUP &&
	    job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SINGLE_GROUP  &&
	    !(pool->last_exec_flag & AIPU_JOB_EXEC_FLAG_SEG_MMU)   &&
	    !(job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SEG_MMU))
		job->desc.head_tcb_pa = job->desc.first_task_tcb_pa;

	/* Driver will clean related TCBs in the list as soon as the job is done.
	 * If userspace schedules a TCB chain already in the list end, it means that
	 * an exit dispatch should be done before these TCBs can be resued.
	 */
	if ((job->desc.tail_tcb_pa == qlist->curr_tail && !pool->aborted) ||
	    (trigger_type == ZHOUYI_V3_TRIGGER_TYPE_CREATE &&
	     (job->desc.exec_flag & AIPU_JOB_EXEC_FLAG_SINGLE_GROUP))) {
		ret = config_exit_tcb(manager, manager->exit_tcb);
		if (ret)
			return ret;

		if (trigger_type != ZHOUYI_V3_TRIGGER_TYPE_CREATE) {
			ret = aipu_mm_link_tcb(manager->mm, qlist->curr_tail,
					       manager->exit_tcb_desc.pa, 0);
			if (ret) {
				dev_err(partition->dev, "link TCB 0x%llx to 0x%llx failed (step 1)",
					job->desc.head_tcb_pa, qlist->curr_tail);
				return ret;
			}
		}

		ret = partition->ops->exit_dispatch(partition, job->desc.exec_flag,
						    manager->exit_tcb_desc.pa);
		if (ret) {
			dev_err(partition->dev, "exit dispatch failed: job ID 0x%llx",
				job->desc.job_id);
			return ret;
		}

		qlist->curr_tail = manager->exit_tcb_desc.pa;

		if (trigger_type != ZHOUYI_V3_TRIGGER_TYPE_CREATE &&
		    job->desc.tail_tcb_pa) {
			ret = aipu_mm_unlink_tcb(manager->mm, job->desc.tail_tcb_pa, false);
			if (ret) {
				dev_err(partition->dev, "unlink TCB from 0x%llx failed",
					job->desc.tail_tcb_pa);
				return ret;
			}
		}
	}

	if (!qlist->pool_head) {
		qlist->pool_head = job->desc.head_tcb_pa;
		job->prev_tail_tcb = 0;
	} else {
		ret = aipu_mm_link_tcb(manager->mm, qlist->curr_tail,
				       job->desc.head_tcb_pa, job->desc.job_id);
		if (ret) {
			dev_err(partition->dev, "link TCB 0x%llx to 0x%llx failed (step 2)",
				job->desc.head_tcb_pa, qlist->curr_tail);
			return ret;
		}

		job->prev_tail_tcb = qlist->curr_tail;
	}

	qlist->curr_head = job->desc.head_tcb_pa;
	qlist->curr_tail = job->desc.tail_tcb_pa;
	qlist->tail_tcb  = tail_tcb;
	pool->last_exec_flag = job->desc.exec_flag;

	ret = partition->ops->reserve(partition, &job->desc, trigger_type);
	if (!ret) {
		if (trigger_type == ZHOUYI_V3_TRIGGER_TYPE_CREATE ||
		    trigger_type == ZHOUYI_V3_TRIGGER_TYPE_DEBUG_DISPATCH)
			pool->created = true;

		if (trigger_type == ZHOUYI_V3_TRIGGER_TYPE_DEBUG_DISPATCH)
			pool->debug = true;

		if (pool->aborted)
			pool->aborted = false;
	}

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
		    (user_job->core_id >= manager->partition_cnt ||
		     !manager->idle_bmap[user_job->core_id])) {
			dev_err(manager->dev, "schedule new job (0x%llx) failed: invalid core ID %u",
				kern_job->desc.job_id, user_job->core_id);
			ret = -EINVAL;
			goto unlock;
		}

		kern_job->state = AIPU_JOB_STATE_DEFERRED;
		kern_job->core_id = user_job->core_id;
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
#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	manager->prof_cache =
		kmem_cache_create("aipu_prof_cache", sizeof(struct profiler), 0, SLAB_PANIC, NULL);
#else
	manager->prof_cache = NULL;
#endif
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

		manager->exit_tcb_desc = buf.desc;
		manager->exit_tcb = aipu_mm_set_tcb_tail(manager->mm, manager->exit_tcb_desc.pa);
		if (!manager->exit_tcb) {
			dev_err(manager->dev, "init job manager failed: get tail TCB failed");
			return -EINVAL;
		}

		ret = config_exit_tcb(manager, manager->exit_tcb);
		if (ret) {
			dev_err(manager->dev, "init job manager failed: config exit TCB failed");
			return ret;
		}
	} else {
		memset(&manager->exit_tcb_desc, 0, sizeof(manager->exit_tcb_desc));
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
#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	kmem_cache_destroy(manager->prof_cache);
	manager->prof_cache = NULL;
#endif
	manager->is_init = 0;
	if (manager->exit_tcb_desc.bytes)
		aipu_mm_unlink_tcb(manager->mm, manager->exit_tcb_desc.pa, true);
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
		tcb = aipu_mm_get_tcb(manager->mm, manager->asid0_base + info->tail_tcbp);
		if (!tcb) {
			dev_err(partition->dev, "real time printk: no TCB found (0x%x)\n",
				info->tail_tcbp);
			return;
		}

		buf = aipu_mm_get_va(manager->mm, manager->asid0_base + tcb->pprint);
		if (buf)
			dev_info(partition->dev, "[real-time printk 0x%x] %s", tcb->pprint, buf);
		else
			dev_err(partition->dev, "real time printk: no pbuf found (0x%x)\n",
				tcb->pprint);
	}
}

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
static void aipu_job_manager_real_time_get_pdata(struct aipu_job_manager *manager,
						 struct job_irq_info *info)
{
	u32 *info_va = NULL;
	char *data_va = NULL;
	u64 info_pa = 0;
	u64 data_pa = 0;
	u32 data_size = 0;
	struct aipu_job *curr = NULL;
	u64 tcbp = 0;
	struct file *f = NULL;
	struct profiler *prof = NULL;

	WARN_ON(!manager || !info);
	tcbp = manager->asid0_base + info->tail_tcbp;
	info_pa = manager->asid0_base + GET_PROFILER_BUF_PA(info->sig_flag);
	info_va = (u32 *)aipu_mm_get_va(manager->mm, info_pa);
	if (!info_va) {
		dev_err(manager->dev, "profiler info buffer is not found: 0x%llx\n", info_pa);
		return;
	}

	data_pa = manager->asid0_base + info_va[0];
	data_size = info_va[1];
	data_va = (char *)aipu_mm_get_va(manager->mm, data_pa);
	if (!data_va) {
		dev_err(manager->dev, "profiler data buffer is not found: 0x%llx\n", data_pa);
		return;
	}

	if (!data_size) {
		dev_err(manager->dev, "profiler data buffer size is 0\n");
		return;
	}

	spin_lock(&manager->lock);
	list_for_each_entry(curr, &manager->scheduled_head->node, node) {
		if (curr->desc.head_tcb_pa <= tcbp && tcbp <= curr->desc.tail_tcb_pa) {
			f = curr->prof_filp;
			if (!f)
				dev_err(manager->dev, "no opened profiler file to be written\n");
			break;
		}
	}
	spin_unlock(&manager->lock);

	if (curr && f) {
		prof = kmem_cache_alloc(manager->prof_cache, GFP_ATOMIC);
		if (!prof) {
			dev_err(manager->dev, "allocate struct profiler failed\n");
			return;
		}

		prof->data = kmalloc(data_size, GFP_ATOMIC);
		if (!prof->data)
			return;

		memcpy(prof->data, data_va, data_size);
		prof->size = data_size;
		list_add_tail(&prof->node, &curr->prof_head->node);
		dev_dbg(manager->dev, "get profiler data: size 0x%x\n", data_size);
	}
}

static void aipu_job_manager_dump_pdata(struct aipu_job_manager *manager,
					struct aipu_job *job)
{
	int ret = 0;
	char *data_va = NULL;
	u64 data_pa = 0;
	u32 data_size = 0;
	struct profiler *prof = NULL;
	struct file *f = NULL;

	WARN_ON(!manager || !job);

	if (!job->prof_filp)
		return;

	if (job->prof_head) {
		f = job->prof_filp;
		list_for_each_entry(prof, &job->prof_head->node, node) {
			if (prof->data && prof->size) {
				ret = kernel_write(f, prof->data, prof->size, &f->f_pos);
				dev_info(manager->dev, "write profiler data size: %d\n", ret);
			}
		}
	}

	data_pa = job->desc.profile_pa;
	data_size = job->desc.profile_sz;
	if (!data_size)
		return;

	data_va = (char *)aipu_mm_get_va(manager->mm, data_pa);
	if (!data_va) {
		dev_err(manager->dev, "(old) profiler buffer is not found: 0x%llx\n", data_pa);
		return;
	}

	if (!job->prof_filp) {
		dev_err(manager->dev, "(old) prof_filp not found\n");
		return;
	}

	ret = kernel_write(job->prof_filp, data_va, data_size, &job->prof_filp->f_pos);
	dev_info(manager->dev, "(old) write profiler data size: %u\n", ret);
}
#endif

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

	if (IS_SIGNAL_IRQ(flag)) {
		if (IS_PRINTF_SIGNAL(info->sig_flag)) {
			aipu_job_manager_real_time_printk(manager, partition, info);
		}

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
		if (IS_PROFILER_SIGNAL(info->sig_flag)) {
			pr_info("profiler signal intr...\n");
			aipu_job_manager_real_time_get_pdata(manager, info);
		}
#endif
		return;
	}

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

			if (manager->pools->debug)
				manager->dbg_do_destroy = true;

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
							  struct aipu_partition *partition,
							  bool unlink)
{
	struct command_pool *pool;

	if (!manager || !partition)
		return;

	pool = manager->pools;
	if (pool && pool->created) {
		if (unlink) {
			/* only do unlink without free */
			u64 q_slow_tail = pool->qlist[AIPU_JOB_QOS_SLOW].curr_tail;
			u64 q_fast_tail = pool->qlist[AIPU_JOB_QOS_FAST].curr_tail;

			if (q_slow_tail)
				aipu_mm_unlink_tcb(manager->mm, q_slow_tail, false);

			if (q_fast_tail)
				aipu_mm_unlink_tcb(manager->mm, q_fast_tail, false);
		}

		partition->ops->destroy_command_pool(partition);
		memset(pool->qlist, 0, sizeof(*pool->qlist) * AIPU_JOB_QOS_MAX);
		pool->created = false;
		pool->debug = false;
		manager->tec_intr_en = false;
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
				aipu_mm_unlink_tcb(manager->mm, curr->prev_tail_tcb, false);
		}

		/* destroy the v3 command pool if all jobs are done */
		if (curr->state < AIPU_JOB_STATE_EXCEP)
			do_destroy = false;
	}

	if (!is_grid_end(manager->pools->qlist[AIPU_JOB_QOS_SLOW].tail_tcb) ||
	    !is_grid_end(manager->pools->qlist[AIPU_JOB_QOS_FAST].tail_tcb))
		do_destroy = false;

	if (manager->dbg_do_destroy) {
		do_destroy = true;
		manager->dbg_do_destroy = false;
	}

	if (do_destroy) {
		aipu_job_manager_destroy_command_pool_no_lock(manager, core, true);
	} else {
		u64 q_slow_tail = manager->pools->qlist[AIPU_JOB_QOS_SLOW].curr_tail;
		u64 q_fast_tail = manager->pools->qlist[AIPU_JOB_QOS_FAST].curr_tail;

		if (q_slow_tail)
			aipu_mm_pin_tcb(manager->mm, q_slow_tail);
		if (q_fast_tail)
			aipu_mm_pin_tcb(manager->mm, q_fast_tail);
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
	bool multi_process = false;
	bool do_destroy = false;
	u64 q_slow_tail = 0;
	u64 q_fast_tail = 0;
	struct command_pool *pool;

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
		} else {
			multi_process = true;
		}
	}

	pool = manager->pools;
	if (pool && pool->created) {
		q_slow_tail = pool->qlist[AIPU_JOB_QOS_SLOW].curr_tail;
		q_fast_tail = pool->qlist[AIPU_JOB_QOS_FAST].curr_tail;
	}

	if (!multi_process && manager->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		do_destroy = true;
		aipu_job_manager_destroy_command_pool_no_lock(manager,
							      &manager->partitions[0],
							      false);
	}
	spin_unlock_irqrestore(&manager->lock, flags);

	if (do_destroy) {
		/* free buffers out of spinlock */
		if (q_slow_tail)
			aipu_mm_unlink_tcb(manager->mm, q_slow_tail, true);

		if (q_fast_tail)
			aipu_mm_unlink_tcb(manager->mm, q_fast_tail, true);
	}

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
	struct aipu_job **done_jobs = NULL;
	int poll_iter = 0;
	unsigned long flags;

	if (unlikely(!manager || !job_status || job_status->max_cnt < 1))
		return -EINVAL;

	status = kcalloc(job_status->max_cnt, sizeof(*status), GFP_KERNEL);
	if (!status)
		return -ENOMEM;

	done_jobs = kcalloc(job_status->max_cnt, sizeof(*done_jobs), GFP_KERNEL);
	if (!done_jobs) {
		kfree(status);
		return -ENOMEM;
	}

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

			done_jobs[poll_iter] = curr;
			job_status->poll_cnt++;
			poll_iter++;
		}
	}

#if AIPU_CONFIG_ENABLE_INTR_PROFILING
	spin_unlock_irqrestore(&manager->lock, flags);

	for (poll_iter = 0; poll_iter < job_status->poll_cnt; poll_iter++)
		aipu_job_manager_dump_pdata(manager, done_jobs[poll_iter]);

	spin_lock_irqsave(&manager->lock, flags);
#endif

	for (poll_iter = 0; poll_iter < job_status->poll_cnt; poll_iter++)
		remove_aipu_job(manager, done_jobs[poll_iter]);
	spin_unlock_irqrestore(&manager->lock, flags);

	ret = copy_to_user((struct job_status_desc __user *)job_status->status, status,
			   job_status->poll_cnt * sizeof(*status));

	kfree(status);
	kfree(done_jobs);
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
		if (wq->ref_cnt &&
		    (wq->uthread_id == uthread_id || wq->filp == filp)) {
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
	u32 en_count = 0;
	u32 core_cnt = 0;

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
