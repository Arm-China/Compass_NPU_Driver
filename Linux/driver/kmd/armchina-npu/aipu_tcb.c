// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#include "aipu_tcb.h"
#include "aipu_mm.h"

bool is_grid_end(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_tcb *tcb = aipu_mm_get_tcb_va(mm, tail);

	if (!tcb)
		return true;
	return IS_GRID_END(tcb->flag);
}

static struct aipu_tcb *get_first_task_tcb(struct aipu_tcb *head, struct aipu_tcb *tail)
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

static struct aipu_tcb *get_next_group_tcb(struct aipu_tcb *head, struct aipu_tcb *tail)
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
	struct aipu_tcb *head_tcb = aipu_mm_get_tcb_va(mm, head);
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
