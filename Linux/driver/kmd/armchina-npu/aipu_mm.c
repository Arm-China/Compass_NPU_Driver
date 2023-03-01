// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Arm Technology (China) Co. Ltd. All rights reserved. */

#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_iommu.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/iommu.h>
#include <linux/bitmap.h>
#include <linux/version.h>
#include "config.h"
#include "aipu_priv.h"
#include "aipu_mm.h"
#include "aipu_common.h"
#include "x1.h"

static struct device *aipu_mm_create_child_dev(struct device *dev, u32 idx)
{
	struct device *child = NULL;

	child = devm_kzalloc(dev, sizeof(*child), GFP_KERNEL);
	if (!child)
		return NULL;

	device_initialize(child);
	dev_set_name(child, "%s:%s%u", dev_name(dev), "reserved_", idx);
	child->parent = dev;
	child->coherent_dma_mask = dev->coherent_dma_mask;
	child->dma_mask = dev->dma_mask;
	child->dma_parms = devm_kzalloc(dev, sizeof(*child->dma_parms),
					GFP_KERNEL);
	child->bus = dev->bus;
	if (!child->dma_parms)
		goto err;

	return child;

err:
	put_device(child);
	return NULL;
}

static unsigned long pa_to_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg, u64 pa)
{
	return (pa + reg->host_aipu_offset - reg->base_iova) >> PAGE_SHIFT;
}

static int aipu_mm_init_pages(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	if (!mm || !reg)
		return -EINVAL;

	reg->count = reg->bytes >> PAGE_SHIFT;
	reg->bitmap = devm_kzalloc(reg->dev,
				   BITS_TO_LONGS(reg->count) * sizeof(long), GFP_KERNEL);
	if (!reg->bitmap)
		return -ENOMEM;

	reg->pages = vzalloc(reg->count * sizeof(struct aipu_virt_page *));
	if (!reg->pages)
		return -ENOMEM;

	return 0;
}

static struct tcb_buf *create_tcb_buf(struct aipu_memory_manager *mm)
{
	struct tcb_buf *tcb = NULL;

	tcb = devm_kzalloc(mm->dev, sizeof(*tcb), GFP_KERNEL);
	if (tcb)
		INIT_LIST_HEAD(&tcb->node);
	return tcb;
}

static int aipu_mm_init_mem_region(struct aipu_memory_manager *mm, struct aipu_mem_region *reg, u32 idx,
				   enum aipu_mem_region_type type, bool reserved)
{
	int ret = 0;
	void *va = NULL;
	bool enable_iommu = false;

	if (!mm || !reg || !reg->bytes)
		return -EINVAL;

	if (!mm->has_iommu && !reserved) {
		dev_err(mm->dev, "memory should be reserved if no IOMMU exists\n");
		return -EINVAL;
	}

	if (!reserved &&
	    (type = AIPU_MEM_REGION_TYPE_SRAM || type == AIPU_MEM_REGION_TYPE_DTCM)) {
		dev_err(mm->dev, "to init a SRAM/DTCM region, you should reserve it: %d\n", type);
		return -EINVAL;
	}

	reg->dev = aipu_mm_create_child_dev(mm->dev, idx);

	/* only head of the list is created; for x2; */
	reg->tcb_buf_head = create_tcb_buf(mm);
	if (!reg->tcb_buf_head)
		goto err;

	/* unused for normal memory */
	reg->cluster_id = 0;
	reg->qos = AIPU_GM_QOS_NONE;

	/* CMA reserved */
	if (reserved) {
		ret = of_reserved_mem_device_init_by_idx(reg->dev, mm->dev->of_node, idx);
		if (ret) {
			dev_err(mm->dev, "init reserved mem failed: idx %d (%d)\n", idx, ret);
			goto err;
		}
	}

	if (mm->has_iommu) {
		ret = dma_set_coherent_mask(reg->dev, DMA_BIT_MASK(31));
		if (ret) {
			dev_err(mm->dev, "DMA set coherent mask failed: idx %d (%d)!\n", idx, ret);
			goto err;
		}
		enable_iommu = true;
	}

	if (mm->has_iommu)
		reg->attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	else
		reg->attrs = 0;

	/* get memory from system in init stage if the flag is enabled */
	if (AIPU_CONFIG_ENABLE_MEM_MANAGEMENT) {
		if (!reserved && AIPU_CONFIG_USE_DEFAULT_MEM_SIZE == 1 &&
		    AIPU_CONFIG_DEFAULT_MEM_SIZE < reg->bytes)
			reg->bytes = AIPU_CONFIG_DEFAULT_MEM_SIZE;

		va = dma_alloc_attrs(reg->dev, reg->bytes, &reg->base_iova, GFP_KERNEL, reg->attrs);
		if (!va) {
			dev_err(reg->dev, "dma_alloc_attrs failed: idx %d (bytes: 0x%llx, attrs %ld)\n",
				idx, reg->bytes, reg->attrs);
			ret = -EINVAL;
			goto err;
		}
		reg->base_va = va;

		/* to be used in driver mem management (aipu_mm_alloc_in_region_no_lock) */
		reg->count = reg->bytes >> PAGE_SHIFT;
		reg->bitmap = devm_kzalloc(reg->dev,
					   BITS_TO_LONGS(reg->count) * sizeof(long), GFP_KERNEL);
		if (!reg->bitmap)
			return -ENOMEM;

		reg->pages = vzalloc(reg->count * sizeof(struct aipu_virt_page *));
		if (!reg->pages)
			return -ENOMEM;

		reg->base_pfn = PFN_DOWN(reg->base_iova);
	} else {
		/* no allocation from system in init stage */
		reg->base_va = NULL;
		reg->base_iova = 0;
		reg->count = 0;
		reg->bitmap = NULL;
		reg->pages = NULL;
		reg->base_pfn = 0;
	}

	dev_info(reg->dev, "init %s region done: %s [0x%llx, 0x%llx]\n",
		 type == AIPU_MEM_REGION_TYPE_MEMORY ? "MEMORY" :
		 (type == AIPU_MEM_REGION_TYPE_SRAM ? "SRAM" :
		  (type == AIPU_MEM_REGION_TYPE_DTCM ? "DTCM" : "GM")),
		 enable_iommu ? "iova" : "pa",
		 reg->base_iova, reg->base_iova + reg->bytes - 1);
	goto finish;

err:
	if (reg->base_va) {
		dma_free_attrs(reg->dev, reg->bytes, reg->base_va, reg->base_iova, reg->attrs);
		reg->base_va = NULL;
	}
	of_reserved_mem_device_release(reg->dev);

finish:
	return ret;
}

static void aipu_mm_deinit_mem_region(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	if (!mm || !reg || !reg->bytes)
		return;

	/* release allocation to system */
	if (reg->base_va) {
		dma_free_attrs(reg->dev, reg->bytes, reg->base_va, reg->base_iova, reg->attrs);
		reg->base_va = NULL;
		reg->base_iova = 0;
	}

	of_reserved_mem_device_release(reg->dev);

	if (reg->pages) {
		vfree(reg->pages);
		reg->pages = NULL;
		reg->count = 0;
	}

	if (reg->bitmap) {
		devm_kfree(reg->dev, reg->bitmap);
		reg->bitmap = NULL;
	}

	reg->bytes = 0;
	reg->base_va = NULL;
}

static int get_free_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg, struct aipu_buf_request *buf_req)
{
	unsigned long alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	unsigned long align_order = order_base_2(buf_req->align_in_page);
	unsigned long mask = (1UL << align_order) - 1;
	unsigned long offset = reg->base_pfn & ((1UL << align_order) - 1);
	unsigned long count = reg->count;
	unsigned long start = 0;

	if (reg->qos == AIPU_GM_QOS_ALL && mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED) {
		if (buf_req->region == AIPU_BUF_REGION_QOS_SLOW_GM)
			count = count >> 1;
		else if (buf_req->region == AIPU_BUF_REGION_QOS_FAST_GM)
			start = count >> 1;
	}

	return bitmap_find_next_zero_area_off(reg->bitmap, count, start, alloc_nr, mask, offset);
}

static u64 get_gm_base(struct aipu_memory_manager *mm, struct aipu_mem_region *gm, struct aipu_buf_request *buf_req)
{
	u64 gm_base = gm->base_iova - gm->host_aipu_offset;

	if (gm->qos == AIPU_GM_QOS_NONE)
		return 0;

	if (mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED && buf_req->region == AIPU_BUF_REGION_QOS_FAST_GM)
		gm_base += (gm->bytes >> 1);

	return gm_base;
}

static int aipu_mm_alloc_in_region_no_lock(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
					   struct aipu_mem_region *reg, struct file *filp,
					   struct tcb_buf **tcb_alloc)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct tcb_buf *tcb = NULL;

	/* filp == NULL means GM initialization or exit TCB allocation */
	if (!mm || !buf_req || !reg)
		return -EINVAL;

	alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	bitmap_no = get_free_bitmap_no(mm, reg, buf_req);
	if (bitmap_no >= reg->count)
		return -ENOMEM;

	bitmap_set(reg->bitmap, bitmap_no, alloc_nr);
	if (!reg->pages[bitmap_no]) {
		reg->pages[bitmap_no] =
			devm_kzalloc(reg->dev, sizeof(struct aipu_virt_page), GFP_KERNEL);
		if (!reg->pages[bitmap_no])
			return -ENOMEM;
	}
	reg->pages[bitmap_no]->contiguous_alloc_len = alloc_nr;
	reg->pages[bitmap_no]->filp = filp;
	reg->pages[bitmap_no]->tid = task_pid_nr(current);
	reg->pages[bitmap_no]->locked = true;
	reg->pages[bitmap_no]->tcb = NULL;

	buf_req->desc.dev_offset = reg->base_iova + (bitmap_no << PAGE_SHIFT) - reg->host_aipu_offset;
	buf_req->desc.pa = buf_req->desc.dev_offset;
	buf_req->desc.bytes = alloc_nr * PAGE_SIZE;
	buf_req->desc.asid = AIPU_BUF_ASID_0;
	buf_req->desc.region = reg->type;
	buf_req->desc.gm_base = get_gm_base(mm, reg, buf_req);

	if (buf_req->data_type == AIPU_MM_DATA_TYPE_TCB) {
		if (!tcb_alloc)
			return -EINVAL;

		tcb = create_tcb_buf(mm);
		if (!tcb)
			return -ENOMEM;

		reg->pages[bitmap_no]->tcb = tcb;
		tcb->page = reg->pages[bitmap_no];
		tcb->pfn = bitmap_no;
		tcb->head = buf_req->desc.pa;

		/**
		 * when head/tail is updated in job manager, all the following fields
		 * should be updated together.
		 *
		 * NOTE: all the nodes of the TCB chain scheduled in a job should be within
		 * this buf!
		 */
		tcb->tail = tcb->head + buf_req->desc.bytes - sizeof(struct aipu_tcb);
		tcb->tail_tcb = NULL;
		tcb->dep_job_id = 0;
		tcb->pinned = false;
		*tcb_alloc = tcb;
	}

	dev_dbg(reg->dev,
		"[MM] allocation done: iova 0x%llx, bytes 0x%llx,  map_num = %d\n",
		buf_req->desc.pa, buf_req->desc.bytes, reg->pages[bitmap_no]->map_num);

	return 0;
}

static int aipu_mm_free_in_region_no_lock(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf,
					  struct aipu_mem_region *reg, struct tcb_buf **tcb)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct aipu_virt_page *page = NULL;

	if (!mm || !buf || !reg)
		return -EINVAL;

	bitmap_no = pa_to_bitmap_no(mm, reg, buf->pa);
	if (bitmap_no >= reg->count)
		return -EINVAL;

	page = reg->pages[bitmap_no];
	if (!page)
		return -EINVAL;

	alloc_nr = page->contiguous_alloc_len;
	if (!alloc_nr)
		return -EINVAL;

	if (tcb)
		*tcb = NULL;

	/* do not free if this tcb is linked by a job or pinned;
	 * marked as unlocked and will be released at an appropriate point;
	 */
	page->locked = false;
	if (page->tcb && tcb) {
		if (page->tcb->dep_job_id || page->tcb->pinned) {
			dev_dbg(reg->dev, "[MM] deferred free TCB: iova 0x%llx\n", buf->pa);
			return 0;
		}

		*tcb = page->tcb;
		page->tcb = NULL;
	}

	bitmap_clear(reg->bitmap, bitmap_no, alloc_nr);
	memset(page, 0, sizeof(struct aipu_virt_page));

	dev_dbg(reg->dev, "[MM] free done: iova 0x%llx, bytes 0x%llx\n", buf->pa, buf->bytes);

	return 0;
}

static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm, u64 iova)
{
	int type;
	struct aipu_mem_region *reg = NULL;

	for (type = AIPU_MEM_REGION_TYPE_MEMORY; type < AIPU_MEM_REGION_TYPE_MAX; type++) {
		u64 internal = iova + mm->mem[type].offset;
		list_for_each_entry(reg, &mm->mem[type].reg->list, list) {
			if (internal >= reg->base_iova && (internal < reg->base_iova + reg->bytes))
				return reg;
		}
	}

	return NULL;
}

static void aipu_mm_free_filp_in_region(struct aipu_memory_manager *mm,
					struct aipu_mem_region *reg, struct file *filp)
{
	unsigned long i = 0;
	unsigned long offset = 0;
	unsigned long flags;
	struct tcb_buf *tcb = NULL;

	if (!mm || !reg || !reg->bitmap || !filp)
		return;

	mutex_lock(&mm->lock);
	while ((i = find_next_bit(reg->bitmap, reg->count, offset)) != reg->count) {
		offset = i + reg->pages[i]->contiguous_alloc_len;
		if (reg->pages[i] && reg->pages[i]->filp == filp) {
			spin_lock_irqsave(&mm->slock, flags);
			tcb = reg->pages[i]->tcb;
			if (tcb && (tcb->dep_job_id || tcb->pinned)) {
				reg->pages[i]->locked = false;
				spin_unlock_irqrestore(&mm->slock, flags);
				continue;
			}
			spin_unlock_irqrestore(&mm->slock, flags);
			bitmap_clear(reg->bitmap, i, reg->pages[i]->contiguous_alloc_len);
			memset(reg->pages[i], 0, sizeof(struct aipu_virt_page));
		}
	}
	mutex_unlock(&mm->lock);
}

static struct aipu_virt_page *aipu_mm_find_page(struct aipu_memory_manager *mm,
						struct aipu_mem_region *reg,
						struct file *filp, u64 iova)
{
	unsigned long page_no = 0;
	struct aipu_virt_page *page = NULL;

	if (!mm || !reg || !filp || (iova % PAGE_SIZE))
		return NULL;

	page_no = (iova - reg->base_iova) >> PAGE_SHIFT;
	if (page_no >= reg->count)
		return NULL;

	page = reg->pages[page_no];
	if (!page || page->map_num || page->filp != filp)
		return NULL;

	return page;
}

static ssize_t aipu_gm_policy_sysfs_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);
	struct aipu_memory_manager *mm = &partition->priv->mm;

	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		return snprintf(buf, 128, "[%d] AIPU GM is shared by tasks of all QoS level.\n",
				AIPU_GM_POLICY_SHARED);
	} else if (mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED) {
		return snprintf(buf, 128,
				"[%d] AIPU GM is divided half-by-half for QoS slow & fast tasks, respectively.\n",
				AIPU_GM_POLICY_HALF_DIVIDED);
	}

	return snprintf(buf, 128, "[%d] AIPU has no GM (or GM is disabled).\n",
			AIPU_GM_POLICY_NONE);
}

static ssize_t aipu_gm_policy_sysfs_store(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_partition *partition = platform_get_drvdata(p_dev);
	struct aipu_memory_manager *mm = &partition->priv->mm;

	mutex_lock(&mm->lock);
	if ((strncmp(buf, "0", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_NONE;
	else if ((strncmp(buf, "1", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_SHARED;
	else if ((strncmp(buf, "2", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_HALF_DIVIDED;
	else
		dev_err(mm->dev, "[sysfs] invalid AIPU GM policy: you should set gm_policy to be 0/1/2");
	mutex_unlock(&mm->lock);

	return count;
}

static int aipu_mm_create_region(struct aipu_memory_manager *mm, struct aipu_mem_region_list *head,
				 u64 addr, u64 size, u64 offset, u32 idx)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !head || !size)
		return -EINVAL;

	/* for the same memory position, AIPU & host CPU may use different addresses to access:
	        aipu_access_addr = host_access_addr - offset
	   if offset == 0, AIPU & host CPU use the same address to access a memory position.
	*/
	if (offset > addr) {
		dev_err(mm->dev, "invalid memory base address (0x%llx) or host-aipu-offset (0x%llx)\n",
			addr, offset);
		return -EINVAL;
	}

	if (!mm->has_iommu) {
		u64 upper = addr + size - offset;

		/*
		 * Z1 only accepts 0~3G region;
		 * Z2/Z3/X1/X2 has ASE registers therefore accepts 0~3G for lower 32 bits;
		 */
		if (mm->version == AIPU_ISA_VERSION_ZHOUYI_Z2 ||
		    mm->version == AIPU_ISA_VERSION_ZHOUYI_Z3 ||
		    mm->version == AIPU_ISA_VERSION_ZHOUYI_X1 ||
		    mm->version == AIPU_ISA_VERSION_ZHOUYI_X2)
			upper &= U32_MAX;

		if (upper > mm->limit) {
			dev_err(mm->dev,
				"region is beyond valid region used by AIPU (0x%llx > 0x%llx)\n",
				upper, mm->limit);
			return -EINVAL;
		}
	}

	reg = devm_kzalloc(mm->dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	reg->base_pa = addr;
	reg->bytes = size;
	reg->host_aipu_offset = offset;
	ret = aipu_mm_init_mem_region(mm, reg, idx, head->type, true);
	if (ret)
		return ret;

	list_add(&reg->list, &head->reg->list);
	head->cnt++;
	return ret;
}

/**
 * @aipu_init_mm() - initialize mm module during driver probe phase
 * @mm:      pointer to memory manager struct to be initialized
 * @p_dev:   pointer to the platform device struct
 * @version: AIPU ISA version
 * @soc:     SoC specific private data provided by vendors
 * @soc_ops: SoC specific operations provided by vendors
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version,
		 struct aipu_soc *soc, struct aipu_soc_operations *soc_ops)
{
	int ret = 0;
	enum aipu_mem_region_type type;
	struct iommu_group *group = NULL;
	struct device_node *np = NULL;
	struct resource res;
	struct aipu_mem_region *reg = NULL;
	u64 host_aipu_offset = 0;
	int idx = 0;

	if (!mm || !p_dev)
		return -EINVAL;

	memset(mm, 0, sizeof(*mm));
	mm->version = version;
	mm->limit = 0xC0000000;
	mm->dev = &p_dev->dev;
	mutex_init(&mm->lock);
	mm->sram_disable_head = devm_kzalloc(mm->dev, sizeof(*mm->sram_disable_head), GFP_KERNEL);
	if (!mm->sram_disable_head)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->sram_disable_head->list);
	spin_lock_init(&mm->slock);

	mm->soc = soc;
	mm->soc_ops = soc_ops;
	if (soc_ops && soc_ops->init_mm)
		return soc_ops->init_mm(mm->dev, soc);

	for (type = AIPU_MEM_REGION_TYPE_MEMORY; type < AIPU_MEM_REGION_TYPE_MAX; type++) {
		reg = devm_kzalloc(mm->dev, sizeof(*reg), GFP_KERNEL);
		if (!reg)
			return -ENOMEM;

		reg->type = type;
		reg->qos = AIPU_GM_QOS_NONE;
		INIT_LIST_HEAD(&reg->list);

		mm->mem[type].reg = reg;
		mm->mem[type].cnt = 0;
		mm->mem[type].disable = 0;
		mm->mem[type].type = type;
		mm->mem[type].base = U64_MAX;
		mm->mem[type].offset = 0;
		mm->mem[type].tot_size = 0;

		/* we do not limit the count of memory/SRAM regions in DTS */
		mm->mem[type].max_cnt = AIPU_CONFIG_MAX_RESERVED_REGIONS;
	}

	/* we accept at maximum 2 GM regions: for QoS fast & slow */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_X2)
		mm->mem[AIPU_MEM_REGION_TYPE_GM].max_cnt = 2;
	else
		mm->mem[AIPU_MEM_REGION_TYPE_GM].max_cnt = 0;

	/* currently, we only support one DTCM region in x1/x2 */
	if (mm->version != AIPU_ISA_VERSION_ZHOUYI_X1 && mm->version != AIPU_ISA_VERSION_ZHOUYI_X2)
		mm->mem[AIPU_MEM_REGION_TYPE_DTCM].max_cnt = 0;
	else
		mm->mem[AIPU_MEM_REGION_TYPE_DTCM].max_cnt = 1;

	/**
	 * Device tree binding for Zhouyi X2:
	 *
	 *    gm-policy = <POLICY_NO>;
	 *
	 * where POLICY_NO should be one of the following values:
	 * 0: no GM;
	 * 1: GM is shared by all tasks in one cluster (by default if you do not provide this attribute)
	 * 2: GM is divided half-by-half: for QoS slow & fast tasks, respectively
	 */
	if (version == AIPU_ISA_VERSION_ZHOUYI_X2) {
		ret = of_property_read_u32(mm->dev->of_node, "gm-policy", &mm->gm_policy);
		if (ret || mm->gm_policy > AIPU_GM_POLICY_HALF_DIVIDED)
			mm->gm_policy = AIPU_GM_POLICY_SHARED;

		if (IS_ERR(aipu_common_create_attr(mm->dev, &mm->gm_policy_attr, "gm_policy", 0644,
						   aipu_gm_policy_sysfs_show,
						   aipu_gm_policy_sysfs_store))) {
			mm->gm_policy_attr = NULL;
			dev_err(mm->dev, "create gm_policy attr failed");
		}

		dev_info(mm->dev, "GM policy is %s",
			 mm->gm_policy == AIPU_GM_POLICY_SHARED ? "shared" : "half-by-half");
	} else {
		mm->gm_policy = AIPU_GM_POLICY_NONE;
		mm->gm_policy_attr = NULL;
	}

	group = iommu_group_get(mm->dev);
	if (group)
		mm->has_iommu = true;
	iommu_group_put(group);
	dev_info(mm->dev, "AIPU is%s behind an IOMMU\n", mm->has_iommu ? "" : " not");

	dev_info(mm->dev, "driver mem management is %s\n",
		 AIPU_CONFIG_ENABLE_MEM_MANAGEMENT ? "enabled" : "disabled");

	/*
	 * If AIPU is behind an IOMMU, in devicetree, memory-region attribute is optional;
	 * otherwise it must be specified;
	 *
	 * [memory] type is necessary; [sram] & [dtcm] types are optional;
	 *
	 * KMD can accept multiple DRAM regions and/or multiple SRAM regions;
	 */
	while (idx < AIPU_CONFIG_MAX_RESERVED_REGIONS) {
		u64 size = 0;
		np = of_parse_phandle(mm->dev->of_node, "memory-region", idx);
		if (!np)
			break;

		if (!strcmp(np->name, "memory")) {
			type = AIPU_MEM_REGION_TYPE_MEMORY;
		} else if (!strcmp(np->name, "sram")) {
			type = AIPU_MEM_REGION_TYPE_SRAM;
		} else if (!strcmp(np->name, "dtcm")) {
			type = AIPU_MEM_REGION_TYPE_DTCM;
		} else {
			dev_err(mm->dev, "invalid memory region name: %s\n", np->name);
			ret = -EINVAL;
			goto err;
		}

		if (mm->mem[type].cnt == mm->mem[type].max_cnt) {
			idx++;
			continue;
		}

		/* bypass mapping SoC SRAM if CPU cannot access it */
		if (type == AIPU_MEM_REGION_TYPE_SRAM && !AIPU_CONFIG_HOST_MAP_SRAM) {
			idx++;
			continue;
		}

		if (of_address_to_resource(np, 0, &res)) {
			of_node_put(np);
			ret = -EINVAL;
			goto err;
		}

		size = res.end - res.start + 1;
		if (type == AIPU_MEM_REGION_TYPE_DTCM && size > ZHOUYI_X1_DTCM_MAX_BYTES) {
			dev_info(mm->dev, "the DTCM size will be clipped to the maximum configurable value\n");
			size = ZHOUYI_X1_DTCM_MAX_BYTES;
		}

		if (of_property_read_u64(np, "host-aipu-offset", &host_aipu_offset))
			host_aipu_offset = 0;

		/* dtcm register is configured based on these info */
		if (!mm->mem[type].offset)
			mm->mem[type].offset = host_aipu_offset;
		if (mm->mem[type].base > res.start)
			mm->mem[type].base = res.start;
		mm->mem[type].tot_size += size;

		/* bypass mapping DTCM if CPU cannot access it */
		if (type == AIPU_MEM_REGION_TYPE_DTCM && !AIPU_CONFIG_HOST_MAP_DTCM) {
			idx++;
			continue;
		}

		ret = aipu_mm_create_region(mm, &mm->mem[type], res.start, size, host_aipu_offset, idx);
		if (ret)
			goto err;

		if (type == AIPU_MEM_REGION_TYPE_MEMORY || type == AIPU_MEM_REGION_TYPE_SRAM) {
			u64 asid_base = ((res.start - host_aipu_offset) >> 32) << 32;
			if (!mm->asid_base) {
				mm->asid_base = asid_base;
			} else {
				/* all the memory/sram regions should be within the same 4G space */
				if (mm->asid_base != asid_base) {
					dev_err(mm->dev, "invalid asid base: 0x%llx\n", asid_base);
					goto err;
				}
			}
		}

		of_node_put(np);
		idx++;
	}

	goto finish;

err:
	aipu_deinit_mm(mm);

finish:
	return ret;
}

/**
 * @aipu_deinit_mm() - de-initialize mm module while kernel module unloading
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 */
int aipu_deinit_mm(struct aipu_memory_manager *mm)
{
	int type = 0;
	struct aipu_mem_region *reg = NULL;

	if (mm->soc_ops && mm->soc_ops->deinit_mm)
		return mm->soc_ops->deinit_mm(mm->dev, mm->soc);

	/* deinit GM regions in a different way */
	for (type = 0; type < AIPU_MEM_REGION_TYPE_GM; type++) {
		list_for_each_entry(reg, &mm->mem[type].reg->list, list)
			aipu_mm_deinit_mem_region(mm, reg);
	}

	aipu_mm_deinit_gm(mm);

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_X2 && mm->gm_policy_attr) {
		aipu_common_destroy_attr(mm->dev, &mm->gm_policy_attr);
		mm->gm_policy_attr = NULL;
	}

	return 0;
}

/**
 * @aipu_mm_alloc() - alloc memory buffer for user request
 * @mm:      pointer to memory manager struct initialized in aipu_init_mm()
 * @buf_req: pointer to buffer request struct from userland
 * @filp:    pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	int type;
	unsigned long flags;
	struct tcb_buf *tcb = NULL;

	if (!mm || !buf_req)
		return -EINVAL;

	if (!buf_req->bytes || !is_power_of_2(buf_req->align_in_page))
		return -EINVAL;

	if (mm->soc_ops && mm->soc_ops->malloc)
		return mm->soc_ops->malloc(mm->dev, mm->soc, buf_req);

	if (buf_req->region == AIPU_BUF_REGION_QOS_SLOW_GM || buf_req->region == AIPU_BUF_REGION_QOS_FAST_GM)
		type = AIPU_MEM_REGION_TYPE_GM;
	else if (buf_req->region < AIPU_MEM_REGION_TYPE_MAX)
		type = buf_req->region;
	else
		return -EINVAL;

	/* fall back to SRAM if DTCM/GM is not applicable for certain archs */
	if (mm->version < AIPU_ISA_VERSION_ZHOUYI_X1 && type == AIPU_MEM_REGION_TYPE_DTCM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	if ((mm->version != AIPU_ISA_VERSION_ZHOUYI_X2 || mm->gm_policy == AIPU_GM_POLICY_NONE) &&
	    type == AIPU_MEM_REGION_TYPE_GM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	/* fall back to MEMORY if SRAM/DTCM/GM region count is 0 */
	if (!mm->mem[type].cnt)
		type = AIPU_MEM_REGION_TYPE_MEMORY;

	mutex_lock(&mm->lock);

	/* allocate from the specified regions first */
	list_for_each_entry(reg, &mm->mem[type].reg->list, list) {
		ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, reg, filp, &tcb);
		if (!ret)
			break;
	}

	/* try to allocate from memory regions if SRAM/DTCM/GM has no free buffer */
	if (ret && type != AIPU_MEM_REGION_TYPE_MEMORY) {
		dev_info(mm->dev, "mm allocate region type %d failed, fall back to memory regions\n", type);
		list_for_each_entry(reg, &mm->mem[AIPU_MEM_REGION_TYPE_MEMORY].reg->list, list) {
			ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, reg, filp, &tcb);
			if (!ret)
				break;
		}
	}

	if (ret) {
		dev_err(mm->dev,
			"[MM] buffer allocation failed for: bytes 0x%llx, page align %d\n",
			buf_req->bytes, buf_req->align_in_page);
		goto unlock;
	}

	WARN_ON(buf_req->desc.pa % (buf_req->align_in_page << PAGE_SHIFT));

unlock:
	mutex_unlock(&mm->lock);
	if (reg && tcb) {
		spin_lock_irqsave(&mm->slock, flags);
		list_add_tail(&tcb->node, &reg->tcb_buf_head->node);
		spin_unlock_irqrestore(&mm->slock, flags);
	}
	return ret;
}

/**
 * @aipu_mm_free() - free buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be released
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	unsigned long flags;
	struct tcb_buf *tcb = NULL;

	if (!mm || !buf || !filp)
		return -EINVAL;

	if (mm->soc_ops && mm->soc_ops->free)
		return mm->soc_ops->free(mm->dev, mm->soc, buf);

	reg = aipu_mm_find_region(mm, buf->pa);
	if (!reg)
		return -EINVAL;

	mutex_lock(&mm->lock);
	ret = aipu_mm_free_in_region_no_lock(mm, buf, reg, &tcb);
	mutex_unlock(&mm->lock);

	if (reg && tcb) {
		spin_lock_irqsave(&mm->slock, flags);
		list_del(&tcb->node);
		devm_kfree(mm->dev, tcb);
		tcb = NULL;
		spin_unlock_irqrestore(&mm->slock, flags);
	}

	return ret;
}

char* aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;

	if (!mm)
		return NULL;

	reg = aipu_mm_find_region(mm, dev_pa);
	if (!reg)
		return NULL;

	return (char*)((unsigned long)reg->base_va + dev_pa + reg->host_aipu_offset - reg->base_iova);
}

/**
 * @aipu_mm_free_buffers() - free all the buffers allocated from one fd
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 */
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_mem_region *reg = NULL;
	int type;

	for (type = AIPU_MEM_REGION_TYPE_MEMORY; type < AIPU_MEM_REGION_TYPE_MAX; type++) {
		list_for_each_entry(reg, &mm->mem[type].reg->list, list) {
			aipu_mm_free_filp_in_region(mm, reg, filp);
		}
	}
}

/**
 * @aipu_mm_mmap_buf() - mmap an allocated buffer for user thread
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @vma: pointer to the vm_area_struct
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_mmap_buf(struct aipu_memory_manager *mm, struct vm_area_struct *vma,
		     struct file *filp)
{
	int ret = 0;
	u64 offset = 0;
	int len = 0;
	size_t mmap_size = 0;
	unsigned long vm_pgoff = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_virt_page *first_page = NULL;

	if (!mm || !vma)
		return -EINVAL;

	if (mm->soc_ops && mm->soc_ops->mmap)
		return mm->soc_ops->mmap(mm->dev, mm->soc, vma);

	offset = vma->vm_pgoff * PAGE_SIZE;
	len = vma->vm_end - vma->vm_start;

	reg = aipu_mm_find_region(mm, offset);
	if (!reg)
		return -EINVAL;

	offset += reg->host_aipu_offset;
	first_page = aipu_mm_find_page(mm, reg, filp, offset);
	if (!first_page)
		return -EINVAL;

	vm_pgoff = vma->vm_pgoff;
	vma->vm_pgoff = 0;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (mm->has_iommu && !reg->attrs) {
		vma->vm_pgoff = (offset - reg->base_iova) >> PAGE_SHIFT;
		mmap_size = reg->bytes;
	} else {
		mmap_size = first_page->contiguous_alloc_len << PAGE_SHIFT;
	}

	ret = dma_mmap_attrs(reg->dev, vma,
			     (void *)((u64)reg->base_va + offset - reg->base_iova),
			     (dma_addr_t)offset, mmap_size, reg->attrs);

	vma->vm_pgoff = vm_pgoff;
	if (!ret)
		first_page->map_num++;

	return ret;
}

/**
 * @aipu_mm_disable_sram_allocation() - disable buffer allocations from soc sram
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_disable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm)
		return -EINVAL;

	/* If there is no SRAM in this system, it cannot be disabled. */
	if (!mm->mem[AIPU_MEM_REGION_TYPE_SRAM].cnt)
		return -EPERM;

	mutex_lock(&mm->lock);
	/* If SRAM is under using by driver & AIPU, it cannot be disabled. */
	list_for_each_entry(reg, &mm->mem[AIPU_MEM_REGION_TYPE_SRAM].reg->list, list) {
		if (!bitmap_empty(reg->bitmap, reg->count)) {
			ret = -EPERM;
			break;
		}
	}

	if (!ret) {
		int found = 0;

		list_for_each_entry(sram_disable_per_fd, &mm->sram_disable_head->list, list) {
			if (sram_disable_per_fd->filp == filp) {
				sram_disable_per_fd->cnt++;
				found = 1;
				break;
			}
		}
		if (!found) {
			sram_disable_per_fd = kzalloc(sizeof(*sram_disable_per_fd), GFP_KERNEL);
			if (!sram_disable_per_fd) {
				ret = -ENOMEM;
				goto unlock;
			}
			sram_disable_per_fd->cnt++;
			sram_disable_per_fd->filp = filp;
			list_add(&sram_disable_per_fd->list, &mm->sram_disable_head->list);
		}
		mm->mem[AIPU_MEM_REGION_TYPE_SRAM].disable++;
	}
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

/**
 * @aipu_mm_enable_sram_allocation() - enable buffer allocations from soc sram (disabled before)
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_enable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp)
{
	int ret = 0;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm)
		return -EINVAL;

	if (!mm->mem[AIPU_MEM_REGION_TYPE_SRAM].cnt)
		return -EPERM;

	mutex_lock(&mm->lock);
	if (mm->mem[AIPU_MEM_REGION_TYPE_SRAM].disable == 0) {
		ret = -EPERM;
		goto unlock;
	}

	list_for_each_entry(sram_disable_per_fd, &mm->sram_disable_head->list, list) {
		if (sram_disable_per_fd->filp == filp) {
			if (sram_disable_per_fd->cnt)
				sram_disable_per_fd->cnt--;
			break;
		}
	}
	mm->mem[AIPU_MEM_REGION_TYPE_SRAM].disable--;
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

static struct tcb_buf *aipu_mm_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
					    struct aipu_mem_region **reg, u64 iova)
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

	*reg = NULL;
	return NULL;
}

struct aipu_tcb *aipu_mm_get_tcb_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;
	struct aipu_tcb *tcb = NULL;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, dev_pa);
	if (!tbuf || !reg)
		goto unlock;

	tcb = (struct aipu_tcb *)((char*)(reg->base_va) + dev_pa + reg->host_aipu_offset -
	       reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return tcb;
}

int aipu_mm_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tcb = NULL;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mm->slock, flags);
	tcb = aipu_mm_find_tcb_buf_no_lock(mm, &reg, tail);
	if (!tcb || !reg) {
		ret = -EFAULT;
		goto unlock;
	}

	tcb->tail = tail;
	tcb->tail_tcb = (struct aipu_tcb *)((char*)(reg->base_va) + tail + reg->host_aipu_offset -
			reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
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
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, prev_tail);
	if (!tbuf) {
		ret = -EFAULT;
		goto unlock;
	}

	/* tail TCB should be set first */
	if (!tbuf->tail_tcb) {
		ret = -EINVAL;
		goto unlock;
	}

	tbuf->tail_tcb->next = next_head_32;
	tbuf->dep_job_id = next_job_id;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 prev_tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	struct aipu_buf_desc buf;
	struct aipu_virt_page *page = NULL;
	unsigned long flags;
	struct tcb_buf *tcb = NULL;
	int ret = 0;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, prev_tail);
	if (!tbuf || !reg) {
		ret = -EFAULT;
		goto unlock;
	}

	/* tail TCB should be set first */
	if (!tbuf->tail_tcb) {
		ret = -EINVAL;
		goto unlock;
	}

	tbuf->tail_tcb->next = 0;
	tbuf->dep_job_id = 0;
	tbuf->pinned = false;

	page = tbuf->page;
	if (!page) {
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

void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, tail);
	if (!tbuf || !reg)
		goto unlock;

	tbuf->pinned = true;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
}

void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	if (!mm || !cap)
		return;

	cap->asid0_base = mm->asid_base;
	cap->asid1_base = mm->asid_base;
	cap->asid2_base = mm->asid_base;
	cap->asid3_base = mm->asid_base;
}

static void aipu_mm_get_gm_region_no_lock(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
					  struct aipu_buf_desc *buf, struct aipu_mem_region *gm)
{
	gm->base_iova = buf->pa + reg->host_aipu_offset;
	gm->base_pa = gm->base_iova; /* should not be used */
	gm->base_va = (void *)((unsigned long *)reg->base_va + gm->base_iova - reg->base_iova);
	gm->bytes = buf->bytes;
	gm->base_pfn = PFN_DOWN(gm->base_iova);
	gm->type = reg->type;
	gm->tcb_buf_head = NULL;
	gm->dev = reg->dev;
	gm->attrs = reg->attrs;
	aipu_mm_init_pages(mm, gm);
}

int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes, int cluster_id)
{
	int ret = 0;
	struct aipu_buf_request buf_req;
	struct aipu_mem_region *reg = NULL;
	struct aipu_mem_region *gm = NULL;

	if (!mm || !bytes || mm->gm_policy == AIPU_GM_POLICY_NONE)
		return -EINVAL;

	buf_req.align_in_page = 1;
	buf_req.data_type = AIPU_MM_DATA_TYPE_NONE;
	buf_req.region = AIPU_BUF_REGION_DEFAULT;
	buf_req.asid = AIPU_BUF_ASID_0;
	buf_req.bytes = bytes;

	list_for_each_entry(reg, &mm->mem[AIPU_MEM_REGION_TYPE_MEMORY].reg->list, list) {
		ret = aipu_mm_alloc_in_region_no_lock(mm, &buf_req, reg, NULL, NULL);
		if (!ret)
			break;
	}

	if (ret)
		return ret;

	gm = devm_kzalloc(mm->dev, sizeof(*gm), GFP_KERNEL);
	gm->cluster_id = cluster_id;
	gm->qos = AIPU_GM_QOS_ALL;
	aipu_mm_get_gm_region_no_lock(mm, reg, &buf_req.desc, gm);
	list_add(&gm->list, &mm->mem[AIPU_MEM_REGION_TYPE_GM].reg->list);

	mm->mem[AIPU_MEM_REGION_TYPE_GM].base = gm->base_iova;
	mm->mem[AIPU_MEM_REGION_TYPE_GM].tot_size = bytes;
	dev_info(mm->dev, "cluster #%d GM region allocated: pa [0x%llx, 0x%llx]",
		 cluster_id, gm->base_iova, gm->base_iova + gm->bytes - 1);

	return ret;
}

void aipu_mm_deinit_gm(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region *gm = NULL;

	if (!mm || mm->gm_policy == AIPU_GM_POLICY_NONE)
		return;

	list_for_each_entry(gm, &mm->mem[AIPU_MEM_REGION_TYPE_GM].reg->list, list) {
		if (gm->pages) {
			vfree(gm->pages);
			gm->pages = NULL;
		}
	}
}

int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next)
{
	int ret = 0;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_X2)
		return -EINVAL;

	if (next == mm->gm_policy)
		return ret;

	mutex_lock(&mm->lock);
	mm->gm_policy = next;
	mutex_unlock(&mm->lock);
	return ret;
}

void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	if (!cap)
		return;

	cap->gm0_base = 0;
	cap->gm0_size = 0;
	cap->gm1_base = 0;
	cap->gm1_size = 0;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_X2 ||
	    AIPU_GM_POLICY_NONE == mm->gm_policy)
		return;

	cap->gm0_base = mm->mem[AIPU_MEM_REGION_TYPE_GM].base -
		mm->mem[AIPU_MEM_REGION_TYPE_GM].offset;

	mutex_lock(&mm->lock);
	if (AIPU_GM_POLICY_SHARED == mm->gm_policy) {
		cap->gm0_size = mm->mem[AIPU_MEM_REGION_TYPE_GM].tot_size;
	} else {
		cap->gm0_size = mm->mem[AIPU_MEM_REGION_TYPE_GM].tot_size >> 1;
		cap->gm1_base = cap->gm0_base + cap->gm0_size;
		cap->gm1_size = cap->gm0_size;
	}
	mutex_unlock(&mm->lock);
}

void get_dtcm(struct aipu_memory_manager *mm, u64 *base, u32 *size)
{
	struct aipu_mem_region_list *region;

	if (!mm || !base || !size)
		return;

	region = &mm->mem[AIPU_MEM_REGION_TYPE_DTCM];
	*base = region->base - region->offset;
	*size = region->tot_size;
}