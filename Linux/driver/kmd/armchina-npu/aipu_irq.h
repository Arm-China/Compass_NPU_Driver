/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __AIPU_IRQ_H__
#define __AIPU_IRQ_H__

#include <linux/device.h>
#include <linux/workqueue.h>

typedef int  (*aipu_irq_uhandler_t) (void *arg);
typedef void (*aipu_irq_bhandler_t) (void *arg);
typedef void (*aipu_irq_trigger_t) (void *arg);
typedef void (*aipu_irq_ack_t) (void *arg);

/**
 * struct aipu_irq_object - interrupt object for every single AIPU instance
 * @irqnum:    interrupt number used to request IRQ
 * @partition: aipu_partition struct pointer
 * @work:      work struct
 * @dev:       device pointer
 * @aipu_wq:   workqueue struct pointer
 */
struct aipu_irq_object {
	u32 irqnum;
	void *partition;
	struct work_struct work;
	struct device *dev;
	struct workqueue_struct *aipu_wq;
};

struct aipu_irq_object *aipu_create_irq_object(struct device *dev, u32 irqnum, void *partition,
					       char *description);
void aipu_irq_schedulework(struct aipu_irq_object *irq_obj);
void aipu_irq_flush_workqueue(struct aipu_irq_object *irq_obj);
void aipu_destroy_irq_object(struct aipu_irq_object *irq_obj);

#endif /* __AIPU_IRQ_H__ */
