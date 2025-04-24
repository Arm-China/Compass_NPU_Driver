/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_JOB_MANAGER_H__
#define __AIPU_JOB_MANAGER_H__

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <armchina_aipu.h>
#include "aipu_partition.h"
#include "aipu_mm.h"

enum aipu_state_kern {
	AIPU_JOB_STATE_IDLE,
	AIPU_JOB_STATE_PENDING,
	AIPU_JOB_STATE_DEFERRED,
	AIPU_JOB_STATE_RUNNING,
	AIPU_JOB_STATE_EXCEP, /* exception */
	AIPU_JOB_STATE_CORED, /* coredump */
	AIPU_JOB_STATE_SUCCESS
};

/**
 * struct waitqueue - maintain the waitqueue for a user thread
 * @uthread_id: user thread owns this waitqueue
 * @filp: file struct pointer
 * @ref_cnt: struct reference count
 * @p_wait: wait queue head for polling
 * @node: list head struct
 */
struct aipu_thread_wait_queue {
	int uthread_id;
	struct file *filp;
	int ref_cnt;
	wait_queue_head_t p_wait;
	struct list_head node;
};

/**
 * struct job_irq_info - maintain data from the interrupt handler (v3)
 * @cluster_id: ID of the cluster trigger interrupts
 * @core_id:    ID of the core trigger interrupts
 * @tec_id:     ID of the TEC trigger interrupts
 * @tag_id:     tag ID of the job trigger interrupts
 * @tail_tcbp:  TCBP of the job trigger interrupts
 * @sig_flag:   signal flag (if any)
 * @tick_counter: count in tick counter (if any)
 */
struct job_irq_info {
	u32 cluster_id;
	u32 core_id;
	u32 tec_id;
	u32 tag_id;
	u32 tail_tcbp;
	u32 sig_flag;
	u64 tick_counter;
};

/**
 * struct profiler - maintain profiling data information (v3)
 * @data: pointer to the profiling data
 * @size: size of the data
 * @node: list node
 */
struct profiler {
	void *data;
	u32 size;
	struct list_head node;
};

/**
 * struct aipu_job - job struct describing a job under scheduling in job manager
 *        Job status will be tracked as soon as interrupt or user evenets come in.
 * @uthread_id: ID of user thread scheduled this job
 * @filp: file struct pointer used when scheduling this job
 * @desc: job descriptor from userland
 * @core_id: ID of an aipu core this job scheduled on
 * @thread_queue: wait queue of this job to be waken up
 * @state: job state
 * @node: list node
 * @sched_time: job scheduled time (enabled by profiling flag in desc)
 * @done_time: job termination time (enabled by profiling flag in desc)
 * @pdata: profiling data (enabled by profiling flag in desc)
 * @wake_up: wake up flag
 * @prev_tail_tcb: address of the tail TCB of the previous job linking this job (v3 only)
 * @prof_filp: pointer to a struct file (the profiler data dump file created in user mode)
 * @prof_head: head of the profiler data list
 */
struct aipu_job {
	int uthread_id;
	struct file *filp;
	struct aipu_job_desc desc;
	int core_id;
	wait_queue_head_t *thread_queue;
	int state;
	struct list_head node;
	ktime_t sched_time;
	ktime_t done_time;
	struct aipu_ext_profiling_data pdata;
	int wake_up;
	u64 curr_hold_tcb;
	struct file *prof_filp;
	struct profiler *prof_head;
};

enum aipu_job_qos {
	AIPU_JOB_QOS_SLOW = 0,
	AIPU_JOB_QOS_FAST = 1,
	AIPU_JOB_QOS_MAX  = 2,
};

/**
 * struct qos - maintain data in a QoS queue (v3)
 * @pool_head: address of the head of this queue
 * @curr_head: address of the head of the last enqueued TCB lists
 * @curr_tail: address of the tail of the last enqueued TCB lists (i.e. tail of the queue)
 * @tail_tcb:  kernel virtual address of the tail TCB buffer
 */
struct qos {
	u64 pool_head;
	u64 curr_head;
	u64 curr_tail;
	struct aipu_tcb *tail_tcb;
};

/**
 * struct qos - maintain data in a command pool (v3)
 * @id: command pool ID
 * @qlist: TCB queue in different QoS lists
 * @created: is this command pool created
 * @aborted: is this command pool aborted
 * @debug:   is this command pool for debug dispatch
 * @last_exec_flag: execution flag of the last job in this command pool
 */
struct command_pool {
	u32 id;
	struct qos qlist[AIPU_JOB_QOS_MAX];
	bool created;
	bool aborted;
	bool debug;
	u32  last_exec_flag;
};

/**
 * struct aipu_job_manager - job manager
 *        Maintain all jobs and update their statuses
 * @version:         NPU version number
 * @partition_cnt:   aipu partition/core count
 * @dev:             pointer to struct device
 * @partitions:      aipu partition/core struct pointer array
 * @pools:           v3 command pools
 * @idle_bmap:       idle flag bitmap for every partition/core
 * @scheduled_head:  scheduled job list head
 * @lock:            spinlock
 * @wait_queue_head: wait queue list head
 * @wq_lock:         waitqueue lock
 * @job_cache:       slab cache of aipu_job
 * @prof_cache:      slab cache of struct profiler
 * @is_init:         init flag
 * @exec_flag:       execution flags propagated to all jobs
 * @mm:              reference to memory manager
 * @priv:            pointer to aipu_priv struct
 * @asid0_base:      base address of ASID 0
 * @exit_tcb_desc:   buffer descriptor of the exit_TCB
 * @exit_tcb:        buffer of the exit_TCB
 * @tick_counter:    atomic lock for tick counter
 * @is_suspend:      is suspended or not
 * @dbg_do_destroy:  do destroy flag (enabled after a debug-dispatch job ends)
 * @tec_intr_en:     is TEC interrupts enabled or not
 * @grid_id:         the latest grid ID allocated for userspace
 * @group_id_bmap:   gruop ID bitmap
 * @group_id_num:    number of group IDs available
 * @id_lock:         lock to protect the grid Id & group IDs
 */
struct aipu_job_manager {
	int version;
	int partition_cnt;
	struct device *dev;
	struct aipu_partition *partitions;
	struct command_pool *pools;
	bool *idle_bmap;
	struct aipu_job *scheduled_head;
	spinlock_t lock; /* Protect cores and jobs status */
	struct aipu_thread_wait_queue *wait_queue_head;
	struct mutex wq_lock; /* Protect thread wait queue */
	struct kmem_cache *job_cache;
	struct kmem_cache *prof_cache;
	int is_init;
	int exec_flag;
	struct aipu_memory_manager *mm;
	void *priv;
	u64 asid0_base;
	struct aipu_buf_desc exit_tcb_desc;
	struct aipu_tcb *exit_tcb;
	atomic_t tick_counter;
	atomic_t is_suspend;
	bool dbg_do_destroy;
	bool tec_intr_en;
	u16  grid_id;
	unsigned long *group_id_bmap;
	u16  group_id_num;
	struct mutex id_lock; /* Protect grid & group IDs */
};

int init_aipu_job_manager(struct aipu_job_manager *manager, struct aipu_memory_manager *mm,
			  void *priv);
void deinit_aipu_job_manager(struct aipu_job_manager *manager);
void aipu_job_manager_set_partitions_info(struct aipu_job_manager *manager, int partition_cnt,
					  struct aipu_partition *partitions);
int aipu_job_manager_scheduler(struct aipu_job_manager *manager, struct aipu_job_desc *user_job,
			       struct file *filp);
void aipu_job_manager_irq_upper_half(struct aipu_partition *core, int exception_flag,
				     struct job_irq_info *info);
void aipu_job_manager_irq_bottom_half(struct aipu_partition *core);
int aipu_job_manager_cancel_jobs(struct aipu_job_manager *manager, struct file *filp);
int aipu_job_manager_invalidate_timeout_job(struct aipu_job_manager *manager, int job_id);
int aipu_job_manager_get_job_status(struct aipu_job_manager *manager,
				    struct aipu_job_status_query *job_status, struct file *filp);
bool aipu_job_manager_has_end_job(struct aipu_job_manager *manager, struct file *filp,
				  struct poll_table_struct *wait, int uthread_id);
int aipu_job_manager_get_hw_status(struct aipu_job_manager *manager, struct aipu_hw_status *hw);
int aipu_job_manager_abort_cmd_pool(struct aipu_job_manager *manager);
int aipu_job_manager_disable_tick_counter(struct aipu_job_manager *manager);
int aipu_job_manager_enable_tick_counter(struct aipu_job_manager *manager);
int aipu_job_manager_config_clusters(struct aipu_job_manager *manager,
				     struct aipu_config_clusters *cfg);
int aipu_job_manager_suspend(struct aipu_job_manager *manager);
int aipu_job_manager_resume(struct aipu_job_manager *manager);
int aipu_job_manager_alloc_grid_id(struct aipu_job_manager *manager);
int aipu_job_manager_alloc_group_id(struct aipu_job_manager *manager,
				    struct aipu_group_id_desc *desc);
int aipu_job_manager_free_group_id(struct aipu_job_manager *manager,
				   struct aipu_group_id_desc *desc);

#endif /* __AIPU_JOB_MANAGER_H__ */
