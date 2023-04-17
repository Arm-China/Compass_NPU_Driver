// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#include "aipu_tcb.h"
#include "aipu_mm.h"

struct tcb_buf *create_tcb_buf(struct aipu_memory_manager *mm)
{
	struct tcb_buf *tcb = NULL;

	tcb = devm_kzalloc(mm->dev, sizeof(*tcb), GFP_KERNEL);
	if (tcb)
		INIT_LIST_HEAD(&tcb->node);
	return tcb;
}

bool is_grid_end(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_tcb *tcb = aipu_get_tcb_va(mm, tail);

	if (!tcb)
		return true;
	return IS_GRID_END(tcb->flag);
}

struct aipu_tcb *get_first_task_tcb(struct aipu_tcb *head, struct aipu_tcb *tail)
{
	if (!head || !tail)
		return NULL;

	while (head != tail) {
		if (IS_TASK_TCB(head->flag))
			break;
		head = (struct aipu_tcb *)((unsigned long)head + sizeof(*head));
	}
	return head;
}

struct aipu_tcb *get_next_group_tcb(struct aipu_tcb *head, struct aipu_tcb *tail)
{
	if (!head || !tail)
		return NULL;

	head = (struct aipu_tcb *)((unsigned long)head + sizeof(*head) * 4);
	if (head > tail)
		head = tail;

	return head;
}

int print_core_id(struct aipu_memory_manager *mm, u64 head, u64 tail)
{
	struct aipu_tcb *head_tcb = aipu_get_tcb_va(mm, head);
	struct aipu_tcb *tail_tcb = (struct aipu_tcb *)((unsigned long)head_tcb + tail - head);
	struct aipu_tcb *tcb;

	tcb = get_first_task_tcb(head_tcb, tail_tcb);
	if (!tcb) {
		dev_err(mm->dev, "invalid head tcb address: 0x%llx\n", head);
		return -EINVAL;
	}

	if (tcb == tail_tcb && !IS_TASK_TCB(tcb->flag))
		return 0;

	while (tcb <= tail_tcb && IS_TASK_TCB(tcb->flag)) {
		dev_info(mm->dev, "group %u execution core ID: %u",
			 tcb->groupid, tcb->_coreid);
		tcb = get_next_group_tcb(tcb, tail_tcb);
		if (tcb == tail_tcb)
			break;
	}

	return 0;
}

static struct tcb_buf *aipu_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
						 struct aipu_mem_region **reg, u64 iova,
						 char *log_str)
{
	struct aipu_mem_region *region = NULL;
	struct tcb_buf *curr = NULL;

	if (!mm || !reg)
		return NULL;

	list_for_each_entry(region, &mm->mem[AIPU_MEM_REGION_TYPE_MEMORY].reg->list, list) {
		list_for_each_entry(curr, &region->tcb_buf_head->node, node) {
			if (iova >= curr->head && iova <= curr->tail) {
				*reg = region;
				return curr;
			}
		}
	}

	if (log_str)
		dev_dbg(mm->dev, "[%s] no TCB buffer is found at iova 0x%llx", log_str, iova);

	*reg = NULL;
	return NULL;
}

struct aipu_tcb *aipu_get_tcb_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;
	struct aipu_tcb *tcb = NULL;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_find_tcb_buf_no_lock(mm, &reg, dev_pa, NULL);
	if (!tbuf || !reg)
		goto unlock;

	tcb = (struct aipu_tcb *)((char *)(reg->base_va) + dev_pa + reg->host_aipu_offset -
	       reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return tcb;
}

int aipu_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tcb = NULL;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mm->slock, flags);
	tcb = aipu_find_tcb_buf_no_lock(mm, &reg, tail, "set_tail");
	if (!tcb || !reg) {
		ret = -EFAULT;
		goto unlock;
	}

	tcb->tail = tail;
	tcb->tail_tcb = (struct aipu_tcb *)((char *)(reg->base_va) + tail + reg->host_aipu_offset -
			reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

int aipu_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		  int next_job_id)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mm->slock, flags);

	/**
	 * if no prev TCB is found, job manager should destroy the pool,
	 * and re-create & re-dispatch with the new head.
	 */
	tbuf = aipu_find_tcb_buf_no_lock(mm, &reg, prev_tail, "link_tcb");
	if (!tbuf) {
		ret = -EFAULT;
		goto unlock;
	}

	/* tail TCB should be set first */
	if (!tbuf->tail_tcb) {
		dev_err(mm->dev, "tail TCB was not set before linking it");
		ret = -EINVAL;
		goto unlock;
	}

	tbuf->tail_tcb->next = next_head_32;
	tbuf->dep_job_id = next_job_id;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

int aipu_unlink_tcb(struct aipu_memory_manager *mm, u64 prev_tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	struct aipu_buf_desc buf;
	struct aipu_virt_page *page = NULL;
	unsigned long flags;
	struct tcb_buf *tcb = NULL;
	int ret = 0;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_find_tcb_buf_no_lock(mm, &reg, prev_tail, "unlink_tcb");
	if (!tbuf || !reg) {
		ret = -EFAULT;
		goto unlock;
	}

	/* tail TCB should be set first */
	if (!tbuf->tail_tcb) {
		dev_err(mm->dev, "tail TCB was not set before unlinking it");
		ret = -EINVAL;
		goto unlock;
	}

	tbuf->tail_tcb->next = 0;
	tbuf->dep_job_id = 0;
	tbuf->pinned = false;

	page = tbuf->page;
	if (!page) {
		dev_err(mm->dev, "page of this TCB buffer is null");
		ret = -EFAULT;
		goto unlock;
	}

	if (!page->locked) {
		buf.pa = tbuf->head;
		buf.bytes = page->contiguous_alloc_len << PAGE_SHIFT;
		ret = aipu_mm_free_in_region_no_lock(mm, &buf, reg, &tcb);
		if (ret)
			goto unlock;

		/* tbuf has been deleted already */
		tbuf = NULL;

		WARN_ON(!tcb);
		list_del(&tcb->node);
		devm_kfree(mm->dev, tcb);
		tcb = NULL;
	}

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

void aipu_pin_tcb(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_find_tcb_buf_no_lock(mm, &reg, tail, NULL);
	if (!tbuf || !reg)
		goto unlock;

	tbuf->pinned = true;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
}
