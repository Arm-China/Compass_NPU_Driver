// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

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
#include "v2.h"

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

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	child->dma_pfn_offset = dev->dma_pfn_offset;
#else
	child->dma_range_map = dev->dma_range_map;
#endif

	return child;

err:
	put_device(child);
	return NULL;
}

static struct list_head *get_region_head(struct aipu_memory_manager *mm, int asid)
{
	return &mm->mem[asid].obj->list;
}

struct tcb_buf *create_tcb_buf(struct aipu_memory_manager *mm)
{
	struct tcb_buf *tcb = NULL;

	tcb = devm_kzalloc(mm->dev, sizeof(*tcb), GFP_KERNEL);
	if (tcb)
		INIT_LIST_HEAD(&tcb->node);
	return tcb;
}

static struct tcb_buf *aipu_mm_find_tcb_buf_no_lock(struct aipu_memory_manager *mm,
						    struct aipu_mem_region **reg, u64 iova,
						    char *log_str)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct tcb_buf *curr = NULL;

	if (!mm || !reg)
		return NULL;

	list_for_each_entry(obj, get_region_head(mm, AIPU_BUF_ASID_0), list) {
		list_for_each_entry(curr, &obj->reg->tcb_buf_head->node, node) {
			if (iova >= curr->head && iova <= curr->tail) {
				*reg = obj->reg;
				return curr;
			}
		}
	}

	if (log_str)
		dev_dbg(mm->dev, "[%s] no TCB buffer is found at iova 0x%llx", log_str, iova);

	*reg = NULL;
	return NULL;
}

static unsigned long pa_to_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
				     u64 pa)
{
	return (pa - reg->base_iova) >> PAGE_SHIFT;
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

	reg->base_pfn = PFN_DOWN(reg->base_iova);

	return 0;
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

	/* reserved */
	if (reg->base_pa) {
		of_reserved_mem_device_release(reg->dev);
		reg->base_pa = 0;
	}

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

static struct aipu_mem_region *aipu_mm_create_region(struct aipu_memory_manager *mm,
						     enum aipu_mem_region_type type,
						     u64 addr, u64 size, u32 idx,
						     bool reserved)
{
	int ret = 0;
	void *va = NULL;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !size)
		return ERR_PTR(-EINVAL);

	if (!mm->has_iommu && !reserved) {
		dev_err(mm->dev, "memory should be reserved if no IOMMU exists\n");
		return ERR_PTR(-EINVAL);
	}

	reg = devm_kzalloc(mm->dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return ERR_PTR(-ENOMEM);

	reg->base_pa = addr;
	reg->bytes = size;

	reg->dev = aipu_mm_create_child_dev(mm->dev, idx);

	/* only head of the list is created; for v3; */
	reg->tcb_buf_head = create_tcb_buf(mm);
	if (!reg->tcb_buf_head) {
		return ERR_PTR(-ENOMEM);
	}

	/* unused for normal memory */
	reg->cluster_id = 0;
	reg->qos = AIPU_GM_QOS_NONE;

	if (reserved ||
	    (mm->has_iommu && AIPU_CONFIG_FORCE_CONTIGUOUS)) {
		ret = of_reserved_mem_device_init_by_idx(reg->dev, mm->dev->of_node, idx);
		if (ret) {
			dev_err(reg->dev, "init reserved mem failed: idx %d (%d)\n",
				idx, ret);
			return ERR_PTR(ret);
		}
	}

	ret = dma_set_coherent_mask(reg->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(reg->dev, "DMA set coherent mask failed: idx %d (%d)!\n", idx, ret);
		goto err;
	}

	if (mm->has_iommu && AIPU_CONFIG_FORCE_CONTIGUOUS)
		reg->attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	else
		reg->attrs = 0;

	va = dma_alloc_attrs(reg->dev, reg->bytes, &reg->base_iova, GFP_KERNEL, reg->attrs);
	if (!va) {
		dev_err(reg->dev, "dma_alloc_attrs failed: idx %d (bytes: 0x%llx, attrs %ld)\n",
			idx, reg->bytes, reg->attrs);
		ret = -EINVAL;
		goto err;
	}
	reg->base_va = va;

	ret = aipu_mm_init_pages(mm, reg);
	if (ret)
		goto err;

	dev_info(reg->dev, "init %s region done: %s [0x%llx, 0x%llx]\n",
		 type == AIPU_MEM_REGION_TYPE_MEMORY ? "MEMORY" :
		 (type == AIPU_MEM_REGION_TYPE_SRAM ? "SRAM" :
		  (type == AIPU_MEM_REGION_TYPE_DTCM ? "DTCM" : "GM")),
		 mm->has_iommu ? "iova" : "pa",
		 reg->base_iova, reg->base_iova + reg->bytes - 1);
	goto finish;

err:
	aipu_mm_deinit_mem_region(mm, reg);
	return ERR_PTR(ret);

finish:
	return reg;
}

static int get_free_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
			      struct aipu_buf_request *buf_req)
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

static u64 get_gm_base(struct aipu_memory_manager *mm, struct aipu_mem_region *gm,
		       struct aipu_buf_request *buf_req)
{
	u64 gm_base = gm->base_iova;

	if (gm->qos == AIPU_GM_QOS_NONE)
		return 0;

	if (mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED &&
	    buf_req->region == AIPU_BUF_REGION_QOS_FAST_GM)
		gm_base += (gm->bytes >> 1);

	return gm_base;
}

static int aipu_mm_alloc_in_region_no_lock(struct aipu_memory_manager *mm,
					   struct aipu_buf_request *buf_req,
					   struct aipu_mem_region *reg, struct file *filp,
					   struct tcb_buf **tcb_alloc)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct tcb_buf *tcb = NULL;

	/* filp == NULL means GM initialization or exit TCB allocation */
	if (!mm || !buf_req || !reg || reg->invalid)
		return -EINVAL;

	alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	bitmap_no = get_free_bitmap_no(mm, reg, buf_req);
	if (bitmap_no >= reg->count) {
		dev_dbg(reg->dev, "alloc in region failed: no free buffer");
		return -ENOMEM;
	}

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

	buf_req->desc.dev_offset = reg->base_iova + (bitmap_no << PAGE_SHIFT);
	buf_req->desc.pa = buf_req->desc.dev_offset;
	buf_req->desc.bytes = alloc_nr * PAGE_SIZE;
	buf_req->desc.asid = buf_req->asid;
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

int aipu_mm_free_in_region_no_lock(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf,
				   struct aipu_mem_region *reg, struct tcb_buf **tcb)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct aipu_virt_page *page = NULL;

	if (!mm || !buf || !reg || reg->invalid)
		return -EINVAL;

	bitmap_no = pa_to_bitmap_no(mm, reg, buf->pa);
	if (bitmap_no >= reg->count) {
		dev_err(reg->dev,
			"free in region failed: no such an allocated buffer: pa 0x%llx",
			buf->pa);
		return -EINVAL;
	}

	page = reg->pages[bitmap_no];
	if (!page) {
		dev_err(reg->dev, "free in region failed: null page at bitmap_no %lu",
			bitmap_no);
		return -EINVAL;
	}

	alloc_nr = page->contiguous_alloc_len;
	if (!alloc_nr) {
		dev_err(reg->dev, "free in region failed: zero alloc_nr is invalid");
		return -EINVAL;
	}

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

static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm, u64 iova,
						   char *log_str)
{
	int idx = 0;
	struct aipu_mem_region *reg = NULL;

	for (idx = 0; idx < mm->reg_cnt; idx++) {
		reg = mm->regs[idx];
		if (iova >= reg->base_iova && (iova < reg->base_iova + reg->bytes))
			return reg;
	}

	dev_err(mm->dev, "[%s] invalid buffer address 0x%llx not found",
		log_str ? log_str : "find_reg", iova);
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

	if (!mm || !reg || (iova % PAGE_SIZE))
		return NULL;

	page_no = (iova - reg->base_iova) >> PAGE_SHIFT;
	if (page_no >= reg->count)
		return NULL;

	page = reg->pages[page_no];
	if (!page || page->filp != filp)
		return NULL;

	if (page->map_num && page->filp)
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
				"[%d] GM is divided half-by-half for QoS slow & fast tasks.\n",
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
		dev_err(mm->dev, "[sysfs] invalid GM policy: gm_policy should be 0/1/2");
	mutex_unlock(&mm->lock);

	return count;
}

static void add_region_list(struct aipu_memory_manager *mm, int asid,
				    struct aipu_mem_region *reg) {
	struct aipu_mem_region_obj *obj = devm_kzalloc(mm->dev, sizeof(*obj), GFP_KERNEL);

	obj->reg = reg;
	INIT_LIST_HEAD(&obj->list);

	list_add(&obj->list, &mm->mem[asid].obj->list);
	mm->mem[asid].cnt++;
}

static int aipu_mm_add_reserved_regions(struct aipu_memory_manager *mm)
{
	int ret = 0;
	int idx = 0;
	int asid_idx = 0;
	struct device_node *np = NULL;
	enum aipu_mem_region_type type;
	struct resource res;
	struct aipu_mem_region *reg = NULL;
	bool asid_set = false;
	u32 dtcm_cnt = 0;
	u32 reg_cnt = 0;

	do {
		u64 size = 0;

		np = of_parse_phandle(mm->dev->of_node, "memory-region", idx);
		if (!np)
			break;

		/* get type of this region */
		if (!strcmp(np->name, "memory")) {
			type = AIPU_MEM_REGION_TYPE_MEMORY;
		} else if (!strcmp(np->name, "sram")) {
			type = AIPU_MEM_REGION_TYPE_SRAM;
		} else if (!strcmp(np->name, "dtcm")) {
			type = AIPU_MEM_REGION_TYPE_DTCM;
		} else {
			dev_err(mm->dev, "invalid memory region name: %s\n", np->name);
			continue;
		}

		/* bypass mapping SoC SRAM if CPU cannot access it */
		if (type == AIPU_MEM_REGION_TYPE_SRAM && !AIPU_CONFIG_HOST_MAP_SRAM) {
			dev_info(mm->dev, "dts: bypass this sram region (cpu cannot access it)\n");
			continue;
		}

		if (type == AIPU_MEM_REGION_TYPE_DTCM && dtcm_cnt == mm->dtcm_max_cnt) {
			dev_err(mm->dev, "dts: bypass this dtcm region for the max_cnt limit\n");
			continue;
		}

		if (type == AIPU_MEM_REGION_TYPE_DTCM &&
		    mm->version == AIPU_ISA_VERSION_ZHOUYI_V1) {
			dev_err(mm->dev, "dts: bypass because v1 does not support dtcm\n");
			continue;
		}

		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			of_node_put(np);
			dev_err(mm->dev, "of_address_to_resource failed (ret = %d)", ret);
			continue;
		}

		size = res.end - res.start + 1;
		if (type == AIPU_MEM_REGION_TYPE_DTCM && size > ZHOUYI_V2_DTCM_MAX_BYTES) {
			dev_info(mm->dev,
				 "the DTCM will be clipped to the maximum configurable value\n");
			size = ZHOUYI_V2_DTCM_MAX_BYTES;
		}

		reg = aipu_mm_create_region(mm, type, res.start, size, idx, true);
		if (IS_ERR(reg)) {
			dev_err(mm->dev, "create new region failed (ret = %ld)", PTR_ERR(reg));
			continue;
		}

		mm->regs[reg_cnt] = reg;

		/* get asid of this region */
		if (mm->version > AIPU_ISA_VERSION_ZHOUYI_V1) {
			for (asid_idx = 0; asid_idx < ZHOUYI_ASID_COUNT; asid_idx++) {
				u32 asid = 0;

				if (!of_property_read_u32_index(np, "asid", asid_idx, &asid)) {
					if (asid >= ZHOUYI_ASID_COUNT) {
						dev_err(mm->dev, "dts: invalid asid: %u\n", asid);
						continue;
					}

					add_region_list(mm, asid, reg);
					asid_set = true;
				} else {
					break;
				}
			}

			if (!asid_set) {
				dev_err(mm->dev, "dts: reg asid is not set, use default asid\n");
				add_region_list(mm, AIPU_BUF_ASID_0, reg);
				add_region_list(mm, AIPU_BUF_ASID_1, reg);
			}
		} else {
			/* use ASID 0 for V1 */
			add_region_list(mm, AIPU_BUF_ASID_0, reg);
		}

		of_node_put(np);
		if (type == AIPU_MEM_REGION_TYPE_DTCM)
			dtcm_cnt++;
		reg_cnt++;
	} while (++idx < AIPU_CONFIG_MAX_RESERVED_REGIONS);

	return reg_cnt;
}

static int aipu_mm_add_iommu_region(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region *reg = NULL;

	/* this feature is to be updated */
	reg = aipu_mm_create_region(mm, AIPU_MEM_REGION_TYPE_MEMORY,
				    0, AIPU_CONFIG_DEFAULT_MEM_SIZE, 0, false);
	if (IS_ERR(reg)) {
		dev_err(mm->dev, "create new smmu region failed (ret = %ld)", PTR_ERR(reg));
		return PTR_ERR(reg);
	}

	mm->regs[0] = reg;
	mm->reg_cnt = 1;

	add_region_list(mm, AIPU_BUF_ASID_0, reg);
	add_region_list(mm, AIPU_BUF_ASID_1, reg);

	return 0;
}

static void aipu_mm_set_asid_base(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	int asid = AIPU_BUF_ASID_0;

	for (asid = AIPU_BUF_ASID_0; asid < ZHOUYI_ASID_COUNT; asid++) {
		u64 first_base = 0;
		u64 asid_base = 0;
		u32 range = 0;
		u32 high = 0;
		u32 end  = 0;

		list_for_each_entry(obj, get_region_head(mm, asid), list) {
			reg = obj->reg;
			end = (u32)reg->base_iova + reg->bytes;

			if (!range) {
				high = end;
				first_base = reg->base_iova;
				asid_base = first_base;
				range = reg->bytes;
			}

			if (reg->base_iova >> 32 != first_base >> 32) {
				dev_err(mm->dev, "invalid region: [0x%llx, 0x%llx]",
					reg->base_iova, reg->base_iova + reg->bytes - 1);
				dev_err(mm->dev, "one asid should use the same hi32 base");
				reg->invalid = true;
				continue;
			}

			high = end > high ? end : high;

			if (reg->base_iova != first_base) {
				asid_base = ((first_base >> 32) << 32);
				range = high;
			}

			mm->mem[asid].valid_cnt++;
		}

		if (range) {
			mm->mem[asid].base = asid_base;
			mm->mem[asid].range = range;
			dev_info(mm->dev, "set ASID %d done: base: 0x%llx, range 0x%x\n",
				 asid, asid_base, range);
		}
	}
}

/**
 * @aipu_init_mm() - initialize mm module during driver probe phase
 * @mm:      pointer to memory manager struct to be initialized
 * @p_dev:   pointer to the platform device struct
 * @version: AIPU ISA version
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version)
{
	int ret = 0;
	struct iommu_group *group = NULL;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	int asid = 0;

	if (!mm || !p_dev)
		return -EINVAL;

	memset(mm, 0, sizeof(*mm));
	mm->version = version;
	mm->dev = &p_dev->dev;
	mutex_init(&mm->lock);
	mm->sram_disable_head = devm_kzalloc(mm->dev, sizeof(*mm->sram_disable_head), GFP_KERNEL);
	if (!mm->sram_disable_head)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->sram_disable_head->list);
	spin_lock_init(&mm->slock);

	for (asid = AIPU_BUF_ASID_0; asid < ZHOUYI_ASID_COUNT; asid++) {
		obj = devm_kzalloc(mm->dev, sizeof(*obj), GFP_KERNEL);
		if (!obj)
			return -ENOMEM;

		reg = devm_kzalloc(mm->dev, sizeof(*reg), GFP_KERNEL);
		if (!reg)
			return -ENOMEM;

		reg->type = AIPU_MEM_REGION_TYPE_MAX;
		reg->qos = AIPU_GM_QOS_NONE;
		obj->reg = reg;
		INIT_LIST_HEAD(&obj->list);

		mm->mem[asid].obj = obj;
		mm->mem[asid].cnt = 0;        /* to be updated */
		mm->mem[asid].valid_cnt = 0;  /* to be updated */
		mm->mem[asid].base = U64_MAX; /* to be calculated */
		mm->mem[asid].range = 0;      /* to be calculated */
	}

	/* we accept at maximum 2 GM regions: for QoS fast & slow */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3)
		mm->gm_max_cnt = 2;

	/* currently, we only support one DTCM region in v2_2 */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V2_2)
		mm->dtcm_max_cnt = 1;

	/**
	 * Device tree binding for Zhouyi V3:
	 *
	 *    gm-policy = <POLICY_NO>;
	 *
	 * where POLICY_NO should be one of the following values:
	 * 0: no GM;
	 * 1: GM is shared by all tasks in 1 cluster (by default if this attribute is not provided)
	 * 2: GM is divided half-by-half: for QoS slow & fast tasks, respectively
	 */
	if (version == AIPU_ISA_VERSION_ZHOUYI_V3) {
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
	 * KMD can accept multiple DRAM regions and/or multiple SRAM regions;
	 */
	if (AIPU_CONFIG_ENABLE_MEM_MANAGEMENT) {
		if (mm->has_iommu) {
			ret = aipu_mm_add_iommu_region(mm);
			if (ret)
				goto err;
		} else {
			mm->reg_cnt = aipu_mm_add_reserved_regions(mm);
			if (!mm->reg_cnt) {
				dev_err(mm->dev, "you shall reserve mem region(s) \
					if no iommu presents");
				ret = -EINVAL;
				goto err;
			}
		}
	} else {
		/* to be supported */
		mm->reg_cnt = 0;
	}

	if (version > AIPU_ISA_VERSION_ZHOUYI_V1)
		aipu_mm_set_asid_base(mm);

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
	int idx = 0;
	struct aipu_mem_region *reg = NULL;

	/* deinit GM regions in a different way */
	for (idx = 0; idx < mm->reg_cnt; idx++) {
		reg = mm->regs[idx];

		if (reg->type != AIPU_MEM_REGION_TYPE_GM) {
			aipu_mm_deinit_mem_region(mm, reg);
		} else if (mm->gm_policy != AIPU_GM_POLICY_NONE) {
			if (reg->pages) {
				vfree(reg->pages);
				reg->pages = NULL;
			}
		}
		mm->regs[idx] = NULL;
	}

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3 && mm->gm_policy_attr) {
		aipu_common_destroy_attr(mm->dev, &mm->gm_policy_attr);
		mm->gm_policy_attr = NULL;
	}

	memset(&mm->mem, 0, sizeof(*mm->mem) * ZHOUYI_ASID_COUNT);
	mm->reg_cnt = 0;

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
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	int type;
	unsigned long flags;
	struct tcb_buf *tcb = NULL;
	bool allocated = false;
	bool fall_back = false;

	if (!mm || !buf_req)
		return -EINVAL;

	if (!buf_req->bytes || !is_power_of_2(buf_req->align_in_page) ||
	    buf_req->asid >= ZHOUYI_ASID_COUNT) {
		dev_err(mm->dev, "[malloc] invalid alloc request: bytes 0x%llx, align %u, asid %u",
			buf_req->bytes, buf_req->align_in_page, buf_req->asid);
		return -EINVAL;
	}

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V1)
		buf_req->asid = AIPU_BUF_ASID_0;

	if (buf_req->region == AIPU_BUF_REGION_QOS_SLOW_GM ||
	    buf_req->region == AIPU_BUF_REGION_QOS_FAST_GM) {
		type = AIPU_MEM_REGION_TYPE_GM;
	} else if (buf_req->region < AIPU_MEM_REGION_TYPE_MAX) {
		type = buf_req->region;
	} else {
		dev_err(mm->dev, "[malloc] invalid alloc request: region type %d",
			buf_req->region);
		return -EINVAL;
	}

	/* fall back to SRAM if DTCM/GM is not applicable for certain archs */
	if (mm->version < AIPU_ISA_VERSION_ZHOUYI_V2_2 && type == AIPU_MEM_REGION_TYPE_DTCM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	if ((mm->version != AIPU_ISA_VERSION_ZHOUYI_V3 || mm->gm_policy == AIPU_GM_POLICY_NONE) &&
	    type == AIPU_MEM_REGION_TYPE_GM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	mutex_lock(&mm->lock);

	/* should check after lock */
	if (type == AIPU_MEM_REGION_TYPE_SRAM && mm->sram_disable)
		type = AIPU_MEM_REGION_TYPE_MEMORY;

alloc:
	list_for_each_entry(obj, get_region_head(mm, buf_req->asid), list) {
		reg = obj->reg;
		if (reg->type == type) {
			ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, reg, filp, &tcb);
			if (!ret) {
				allocated = true;
				break;
			}
		}
	}

	/* fall back to main memory */
	if (!allocated && !fall_back && type != AIPU_MEM_REGION_TYPE_MEMORY) {
		type = AIPU_MEM_REGION_TYPE_MEMORY;
		fall_back = true;
		goto alloc;
	}

	if (!allocated) {
		dev_err(mm->dev,
			"[malloc] buffer allocation failed for: bytes 0x%llx, page align %d\n",
			buf_req->bytes, buf_req->align_in_page);
	} else if (fall_back) {
		dev_info(mm->dev,
			 "[malloc] buffer allocation done (fall back to memory): bytes 0x%llx\n",
			 buf_req->bytes);
	}

	WARN_ON(buf_req->desc.pa % (buf_req->align_in_page << PAGE_SHIFT));
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

	if (!mm || !buf)
		return -EINVAL;

	reg = aipu_mm_find_region(mm, buf->pa, "free");
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

/**
 * @aipu_mm_free_buffers() - free all the buffers allocated from one fd
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 */
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp)
{
	int idx = 0;

	for (idx = 0; idx < mm->reg_cnt; idx++)
		aipu_mm_free_filp_in_region(mm, mm->regs[idx], filp);
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

	offset = vma->vm_pgoff * PAGE_SIZE;
	len = vma->vm_end - vma->vm_start;

	reg = aipu_mm_find_region(mm, offset, "mmap");
	if (!reg)
		return -EINVAL;

	first_page = aipu_mm_find_page(mm, reg, filp, offset);
	if (!first_page) {
		dev_err(mm->dev, "[mmap] page not found at offset 0x%llx", offset);
		return -EINVAL;
	}

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
	else
		dev_err(mm->dev, "dma_mmap_attrs failed at offset 0x%llx (ret = %d)", offset, ret);

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
	int idx = 0;
	struct aipu_mem_region *reg = NULL;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm)
		return -EINVAL;

	mutex_lock(&mm->lock);
	/* If SRAM is under using by driver & AIPU, it cannot be disabled. */
	for (idx = 0; idx < mm->reg_cnt; idx++) {
		reg = mm->regs[idx];

		if (reg->type == AIPU_MEM_REGION_TYPE_SRAM &&
		    !bitmap_empty(reg->bitmap, reg->count)) {
			dev_err(mm->dev, "the SRAM region to be disabled is under using");
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
		mm->sram_disable++;
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

	mutex_lock(&mm->lock);
	if (mm->sram_disable == 0) {
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
	mm->sram_disable--;
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

/**
 * @aipu_mm_get_va() - get the kernel virtual address of an allocated buffer.
 *                     currently do not support non-tcb va from importers.
 * @mm:     pointer to memory manager struct initialized in aipu_init_mm()
 * @dev_pa: device physical address returned by aipu_mm_alloc()
 *
 * Return: 0 on success and error code otherwise.
 */
char *aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;

	if (!mm)
		return NULL;

	reg = aipu_mm_find_region(mm, dev_pa, "get_va");
	if (!reg)
		return NULL;

	return (char *)((unsigned long)reg->base_va + dev_pa - reg->base_iova);
}

/**
 * @aipu_mm_get_tcb_va() - get the kernel virtual address of a TCB buffer.
 * @mm:     pointer to memory manager struct initialized in aipu_init_mm()
 * @dev_pa: device physical address returned by aipu_mm_alloc()
 *
 * Return: pointer of a TCB on success and NULL otherwise.
 */
struct aipu_tcb *aipu_mm_get_tcb_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;
	struct aipu_tcb *tcb = NULL;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, dev_pa, NULL);
	if (!tbuf || !reg)
		goto unlock;

	tcb = (struct aipu_tcb *)((char *)(reg->base_va) + dev_pa - reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return tcb;
}

/**
 * @aipu_mm_set_tcb_tail() - set the pointer to the tail TCB of a TCB list.
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @tail: address of the tail TCB of a TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tcb = NULL;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mm->slock, flags);
	tcb = aipu_mm_find_tcb_buf_no_lock(mm, &reg, tail, "set_tail");
	if (!tcb || !reg) {
		ret = -EFAULT;
		goto unlock;
	}

	tcb->tail = tail;
	tcb->tail_tcb = (struct aipu_tcb *)((char *)(reg->base_va) + tail - reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

/**
 * @aipu_mm_link_tcb() - link a TCB list to the tail of an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be linked
 * @next_head_32: head address of the next TCB list
 * @next_job_id:  Job ID of the next TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
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
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, prev_tail, "link_tcb");
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

/**
 * @aipu_mm_unlink_tcb() - unlink a TCB list from an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be unlinked
 *
 * Return: 0 on success and error code otherwise.
 */
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
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, prev_tail, "unlink_tcb");
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

/**
 * @aipu_mm_pin_tcb() - pin a TCB list until it is ready to be freed in a future moment
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @tail: address of the tail TCB of a TCB list
 */
void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct tcb_buf *tbuf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, &reg, tail, NULL);
	if (!tbuf || !reg)
		goto unlock;

	tbuf->pinned = true;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
}

u64 aipu_mm_get_asid_base(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	return mm->mem[asid].base;
}

u64 aipu_mm_get_asid_size(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	return mm->mem[asid].range;
}

void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	if (!mm || !cap)
		return;

	cap->asid0_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0);
	cap->asid1_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_1);
	cap->asid2_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_2);
	cap->asid3_base = aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_3);
}

static int aipu_mm_get_gm_region_no_lock(struct aipu_memory_manager *mm,
					  struct aipu_buf_desc *buf,
					  struct aipu_mem_region *gm)
{
	struct aipu_mem_region *reg = aipu_mm_find_region(mm, buf->pa, "get_gm");
	if (!reg)
		return -EINVAL;

	gm->base_iova = buf->pa;
	gm->base_pa = 0;
	gm->base_va = (void *)((unsigned long *)reg->base_va + gm->base_iova - reg->base_iova);
	gm->bytes = buf->bytes;
	gm->base_pfn = PFN_DOWN(gm->base_iova);
	gm->type = reg->type;
	gm->tcb_buf_head = NULL;
	gm->dev = reg->dev;
	gm->attrs = reg->attrs;
	return aipu_mm_init_pages(mm, gm);
}

int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes, int cluster_id)
{
	int ret = 0;
	struct aipu_buf_request buf_req;
	struct aipu_mem_region *gm = NULL;

	if (!mm || !bytes || mm->gm_policy == AIPU_GM_POLICY_NONE) {
		dev_err(mm->dev, "invalid GM initialization");
		return -EINVAL;
	}

	buf_req.align_in_page = 1;
	buf_req.data_type = AIPU_MM_DATA_TYPE_NONE;
	buf_req.region = AIPU_BUF_REGION_DEFAULT;
	buf_req.asid = AIPU_BUF_ASID_0;
	buf_req.bytes = bytes;

	ret = aipu_mm_alloc(mm, &buf_req, NULL);
	if (ret) {
		dev_err(mm->dev, "GM allocation failed: bytes 0x%x", bytes);
		return ret;
	}

	gm = devm_kzalloc(mm->dev, sizeof(*gm), GFP_KERNEL);
	gm->cluster_id = cluster_id;
	gm->qos = AIPU_GM_QOS_ALL;

	ret = aipu_mm_get_gm_region_no_lock(mm, &buf_req.desc, gm);
	if (ret) {
		dev_err(mm->dev, "init GM region failed\n");
		return ret;
	}

	mm->gm = gm;

	dev_info(mm->dev, "cluster #%d GM region allocated: pa [0x%llx, 0x%llx]",
		 cluster_id, gm->base_iova, gm->base_iova + gm->bytes - 1);

	return ret;
}

void aipu_mm_deinit_gm(struct aipu_memory_manager *mm)
{
	if (!mm || mm->gm)
		return;

	if (mm->gm->pages) {
		vfree(mm->gm->pages);
			mm->gm->pages = NULL;
	}
}

int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next)
{
	int ret = 0;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_V3)
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

	if (!mm || !mm->gm)
		return;

	cap->gm0_base = mm->gm->base_iova;

	mutex_lock(&mm->lock);
	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		cap->gm0_size = mm->gm->bytes;
	} else {
		cap->gm0_size = mm->gm->bytes >> 1;
		cap->gm1_base = cap->gm0_base + cap->gm0_size;
		cap->gm1_size = cap->gm0_size;
	}
	mutex_unlock(&mm->lock);
}

void get_dtcm(struct aipu_memory_manager *mm, u64 *base, u32 *size)
{
	struct aipu_mem_region *reg = NULL;
	int idx = 0;

	if (!mm || !base || !size)
		return;

	for (idx = 0; idx < mm->reg_cnt; idx++) {
		reg = mm->regs[idx];

		if (mm->regs[idx]->type == AIPU_MEM_REGION_TYPE_DTCM) {
			*base = reg->base_iova;
			*size = reg->bytes;
			return;
		}
	}

	*base = 0;
	*size = 0;
}
