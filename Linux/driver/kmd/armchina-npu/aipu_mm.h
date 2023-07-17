/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __AIPU_MM_H__
#define __AIPU_MM_H__

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <armchina_aipu.h>
#include "aipu_tcb.h"
#include "zhouyi.h"

#define DEFERRED_FREE  1

enum aipu_gm_policy {
	AIPU_GM_POLICY_NONE         = 0,
	AIPU_GM_POLICY_SHARED       = 1,
	AIPU_GM_POLICY_HALF_DIVIDED = 2,
};

enum aipu_gm_qos {
	AIPU_GM_QOS_NONE  = AIPU_BUF_REGION_DEFAULT,
	AIPU_GM_QOS_SLOW  = AIPU_BUF_REGION_QOS_SLOW_GM,
	AIPU_GM_QOS_FAST  = AIPU_BUF_REGION_QOS_FAST_GM,
	AIPU_GM_QOS_ALL   = 3,
};

enum aipu_mem_region_type {
	AIPU_MEM_REGION_TYPE_MEMORY = 0,
	AIPU_MEM_REGION_TYPE_SRAM   = 1,
	AIPU_MEM_REGION_TYPE_DTCM   = 2,
	AIPU_MEM_REGION_TYPE_GM     = 3,
	AIPU_MEM_REGION_TYPE_MAX    = 4,
};

struct aipu_mem_region_obj;

/**
 * struct aipu_tcb_buf - TCB buffer descriptor
 * @pfn: pfn number
 * @head: address of the list head
 * @tail: address of the list tail
 * @dep_job_id: ID of the job depends on this buffer list (if any)
 * @tail_tcb: tail of the TCB list
 * @node: list node
 * @pinned: is this buffer should be maintained after executions
 * @reg: pointer to the region contains this TCB
 */
struct aipu_tcb_buf {
	u64 pfn;
	u64 head;
	u64 tail;
	int dep_job_id;
	struct aipu_tcb *tail_tcb;
	struct list_head node;
	bool pinned;
	struct aipu_mem_region *reg;
};

/**
 * struct aipu_virt_page - virtual page
 * @tid: ID of thread requested this page (and the following pages)
 * @filp: filp requested this page
 * @contiguous_alloc_len: count of immediately following pages allocated in together
 * @locked: is this page locked (should not be freed at this moment)
 * @tcb: reference to a corresponding TCB descriptor
 */
struct aipu_virt_page {
	int tid;
	struct file *filp;
	unsigned long contiguous_alloc_len;
	bool locked;
	struct aipu_tcb_buf *tcb;
};

/**
 * struct aipu_mem_region - AIPU memory region
 * @type: region type: memory/sram/dtcm/gm
 * @reserved: is this a reserved region or not
 * @base_iova: region base iova (bus address)
 * @base_pa: region base physical address
 * @base_va: region base virtual address
 * @bytes: total bytes of this region
 * @base_pfn: region base page frame number
 * @pages: page array
 * @bitmap: region bitmap
 * @count: bitmap bit count/page count
 * @host_aipu_offset: address space offset between host CPU and AIPU
 * @dev: region specific device (for multiple DMA/CMA regions)
 * @attrs: attributes for DMA API
 * @tcb_buf_head: list head of tbuf
 * @cluster_id: the ID of the cluster owns this GM region
 * @qos: qos level of this GM region
 * @invalid: if this region is invalid (cannot be used) or not
 * @filp: pointer to struct file requesting this region
 * @obj: pointer to the region object
 * @locked: is this region locked (therefore cannot be released) or not
 */
struct aipu_mem_region {
	enum aipu_mem_region_type type;
	bool reserved;
	dma_addr_t base_iova;
	dma_addr_t base_pa;
	void *base_va;
	u64 bytes;
	unsigned long base_pfn;
	struct aipu_virt_page **pages;
	unsigned long *bitmap;
	unsigned long count;
	u64 host_aipu_offset;
	struct device *dev;
	unsigned long attrs;
	struct aipu_tcb_buf *tcb_buf_head;
	/* for gm only */
	int cluster_id;
	int qos;
	bool invalid;
	struct file *filp;
	struct aipu_mem_region_obj *obj;
	bool locked;
};

/**
 * struct aipu_mem_region_obj - object struct contains a region
 *     We link an object rather than the region directly because
 *     in some cases, a region might be linked in multiple lists.
 * @reg: pointer to a region
 * @list: list head
 */
struct aipu_mem_region_obj {
	struct aipu_mem_region *reg;
	struct list_head list;
};

/**
 * struct aipu_mem_region_list - memory region list share the same ASID
 * @head: region objects
 * @cnt: region count
 * @valid_cnt: valid region count
 * @base: base address of the regions
 * @range: address range (i.e. max_addr - min_addr) of the regions
 */
struct aipu_mem_region_list {
	struct aipu_mem_region_obj *head;
	int cnt;
	int valid_cnt;
	dma_addr_t base;
	u32 range;
};

/**
 * struct aipu_sram_disable_per_fd - SRAM disable list records disable operations
 * @cnt: current total disable operation count
 * @filp: file opinter
 * @list: file pointer list
 */
struct aipu_sram_disable_per_fd {
	int cnt;
	struct file *filp;
	struct list_head list;
};

/**
 * struct aipu_memory_manager - AIPU memory management struct (MM)
 * @version: AIPU ISA version number
 * @has_iommu: system has an IOMMU for AIPU to use or not
 * @dev: device struct pointer (AIPU core 0)
 * @lock: lock for reg and sram_disable_head
 * @res_cnt: reserved region count
 * @mem: list of all reserved or allocated memory regions
 * @ase: array of reserved regions in different asids
 * @gm: V3 GM region
 * @gm_policy: GM policy determined by customer (AIPU_GM_POLICY_SHARED/AIPU_GM_POLICY_HALF_DIVIDED)
 * @gm_max_cnt: maximum count of GM region
 * @dtcm_max_cnt: maximum count of DTCM region
 * @sram_disable_head: SRAM disable list
 * @sram_disable: disable count of SRAM
 * @gm_policy_attr: GM policy sysfs attribute, for v3 only
 * @slock:   TCB buffer lock
 * @default_asid_base: ASID region 0/1 base address by default
 * @default_asid_size: ASID region 0/1 size by default
 * @obj_cache: slab cache of the region objects
 * @reg_cache: slab cache of the regions
 * @tbuf_cache: slab cache of the tcb descriptors
 */
struct aipu_memory_manager {
	int version;
	bool has_iommu;
	struct device *dev;
	struct mutex lock; /* Protect sram disabled head struct */
	int res_cnt;
	struct aipu_mem_region_list mem;
	struct aipu_mem_region_list ase[ZHOUYI_ASID_COUNT];
	struct aipu_mem_region *gm;
	int gm_policy;
	int gm_max_cnt;
	int dtcm_max_cnt;
	struct aipu_sram_disable_per_fd *sram_disable_head;
	int sram_disable;
	struct device_attribute *gm_policy_attr;
	spinlock_t slock; /* Protect tcb_buf list */
	u64 default_asid_base;
	u32 default_asid_size;
	struct kmem_cache *obj_cache;
	struct kmem_cache *reg_cache;
	struct kmem_cache *tbuf_cache;
};

int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version);
int aipu_deinit_mm(struct aipu_memory_manager *mm);
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp);
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp,
		 bool unlock);
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp);
char *aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa);
int aipu_mm_mmap_buf(struct aipu_memory_manager *mm, struct vm_area_struct *vma,
		     struct file *filp);
int aipu_mm_disable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp);
int aipu_mm_enable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp);
void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap);
u64 aipu_mm_get_asid_base(struct aipu_memory_manager *mm, u32 asid);
u64 aipu_mm_get_asid_size(struct aipu_memory_manager *mm, u32 asid);
int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes, int cluster_id);
void aipu_mm_deinit_gm(struct aipu_memory_manager *mm);
int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next);
void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap);
void get_dtcm(struct aipu_memory_manager *mm, u64 *base, u32 *size);

bool is_grid_end(struct aipu_memory_manager *mm, u64 tail);
int print_core_id(struct aipu_memory_manager *mm, u64 head, u64 tail);
int aipu_mm_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail);
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id);
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 prev_tail, bool free_tcb);
void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail);

#endif /* __AIPU_MM_H__ */
