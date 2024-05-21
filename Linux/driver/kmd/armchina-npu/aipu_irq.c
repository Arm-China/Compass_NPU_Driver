// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include "aipu_irq.h"
#include "aipu_partition.h"

static irqreturn_t aipu_irq_handler_upper_half(int irq, void *dev_id)
{
	struct aipu_partition *partition =
		(struct aipu_partition *)(((struct device *)dev_id)->driver_data);

	if (partition && partition->ops && partition->ops->upper_half)
		return partition->ops->upper_half(partition);

	pr_info("unexpected interrupt enter before finish zhouyi's initialization.\n");
	return IRQ_NONE;
}

static void aipu_irq_handler_bottom_half(struct work_struct *work)
{
	struct aipu_irq_object *irq_obj = NULL;
	struct aipu_partition *partition = NULL;

	if (work) {
		irq_obj = container_of(work, struct aipu_irq_object, work);
		partition = irq_obj->partition;
		partition->ops->bottom_half(partition);
	}
}

/**
 * @aipu_create_irq_object() - initialize an AIPU IRQ object
 * @irqnum:      interrupt number
 * @partition:   aipu_partition struct pointer
 * @description: irq object description string
 *
 * Return: pointer to the created irq_object on success and NULL otherwise.
 */
struct aipu_irq_object *aipu_create_irq_object(struct device *dev, u32 irqnum, void *partition,
					       char *description)
{
	int ret = 0;
	struct aipu_irq_object *irq_obj = NULL;

	if (!partition || !description || !dev)
		return NULL;

	irq_obj = kzalloc(sizeof(*irq_obj), GFP_KERNEL);
	if (!irq_obj)
		return NULL;

	irq_obj->aipu_wq = NULL;
	irq_obj->irqnum = 0;
	irq_obj->dev = dev;

	irq_obj->aipu_wq = create_singlethread_workqueue(description);
	if (!irq_obj->aipu_wq)
		goto err_handle;

	INIT_WORK(&irq_obj->work, aipu_irq_handler_bottom_half);

	/**
	 * Flag IRQF_ONESHOT is used when sharing interrupts with other devices using thread_irq.
	 * Remove it if you have no such a usage scenario.
	 */
	ret = request_irq(irqnum, aipu_irq_handler_upper_half,
			  IRQF_ONESHOT | IRQF_SHARED | IRQF_PROBE_SHARED,
			  description, irq_obj->dev);
	if (ret) {
		dev_err(dev, "request_irq failed: irqnum %u, ret %d", irqnum, ret);
		goto err_handle;
	}

	irq_obj->irqnum = irqnum;
	irq_obj->partition = partition;

	goto finish;

err_handle:
	aipu_destroy_irq_object(irq_obj);
	irq_obj = NULL;

finish:
	return irq_obj;
}

/**
 * @aipu_destroy_irq_object() - destroy a created aipu_irq_object
 * @irq_obj: interrupt object created in aipu_create_irq_object()
 */
void aipu_destroy_irq_object(struct aipu_irq_object *irq_obj)
{
	if (irq_obj) {
		if (irq_obj->aipu_wq) {
			flush_workqueue(irq_obj->aipu_wq);
			destroy_workqueue(irq_obj->aipu_wq);
			irq_obj->aipu_wq = NULL;
		}
		if (irq_obj->irqnum)
			free_irq(irq_obj->irqnum, irq_obj->dev);
		kfree(irq_obj);
		flush_scheduled_work();
	}
}

/**
 * @aipu_irq_schedulework() - workqueue schedule API
 * @irq_obj: interrupt object created in aipu_create_irq_object()
 */
void aipu_irq_schedulework(struct aipu_irq_object *irq_obj)
{
	if (irq_obj)
		queue_work(irq_obj->aipu_wq, &irq_obj->work);
}

/**
 * @aipu_irq_flush_workqueue() - workqueue flush API
 * @irq_obj: interrupt object created in aipu_create_irq_object()
 */
void aipu_irq_flush_workqueue(struct aipu_irq_object *irq_obj)
{
	flush_workqueue(irq_obj->aipu_wq);
}
