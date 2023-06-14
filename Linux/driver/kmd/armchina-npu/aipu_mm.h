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

struct tcb_buf;

struct aipu_direct_mem {
	struct aipu_buf_desc desc;
	void *va;
	struct file *filp;
	struct list_head list;
};

/**
 * struct aipu_virt_page - virtual page
 * @tid: ID of thread requested this page (and the following pages)
 * @filp: filp requested this page
 * @map_num: number of mmap to userspace
 * @contiguous_alloc_len: count of immediately following pages allocated in together
 * @locked: is this page locked (should not be freed at this moment)
 * @tcb: reference to a corresponding TCB descriptor
 */
struct aipu_virt_page {
	int tid;
	struct file *filp;
	int map_num;
	unsigned long contiguous_alloc_len;
	bool locked;
	struct tcb_buf *tcb;
};

/**
 * struct tcb_buf - TCB buffer descriptor
 * @page: reference to the virtual page
 * @pfn: pfn number
 * @head: address of the list head
 * @tail: address of the list tail
 * @dep_job_id: ID of the job depends on this buffer list (if any)
 * @tail_tcb: tail of the TCB list
 * @node: list node
 * @pinned: is this buffer should be maintained after executions
 */
struct tcb_buf {
	struct aipu_virt_page *page;
	u64 pfn;
	u64 head;
	u64 tail;
	int dep_job_id;
	struct aipu_tcb *tail_tcb;
	struct list_head node;
	bool pinned;
};

/**
 * struct aipu_mem_region - AIPU memory region
 * @type: region type: memory/sram/dtcm/gm
 * @base_iova: region base iova (bus address)
 * @base_pa: region base physical address
 * @base_va: region base virtual address
 * @bytes: total bytes of this region
 * @base_pfn: region base page frame number
 * @pages: page array
 * @bitmap: region bitmap
 * @count: bitmap bit count/page count
 * @dev: region specific device (for multiple DMA/CMA regions)
 * @attrs: attributes for DMA API
 * @tcb_buf_head: TCB buffer list
 * @cluster_id: the ID of the cluster owns this GM region
 * @qos: qos level of this GM region
 * @invalid: if this region is invalid (cannot be used) or not
 */
struct aipu_mem_region {
	enum aipu_mem_region_type type;
	dma_addr_t base_iova;
	dma_addr_t base_pa;
	void *base_va;
	u64 bytes;
	unsigned long base_pfn;
	struct aipu_virt_page **pages;
	unsigned long *bitmap;
	unsigned long count;
	struct device *dev;
	unsigned long attrs;
	struct tcb_buf *tcb_buf_head;
	/* for gm only */
	int cluster_id;
	int qos;
	bool invalid;
};

struct aipu_mem_region_obj {
	struct aipu_mem_region *reg;
	struct list_head list;
};

/**
 * struct aipu_mem_region_list - memory region list share the same ASID
 * @obj: region objects
 * @cnt: region count
 * @valid_cnt: valid region count
 * @base: base address of the regions
 * @range: address range (i.e. max_addr - min_addr) of the regions
 */
struct aipu_mem_region_list {
	struct aipu_mem_region_obj *obj;
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
 * @reg_cnt: region count
 * @regs: array of all reserved regions
 * @mem: array of reserved regions in different asids
 * @gm: V3 GM region
 * @gm_policy: GM policy determined by customer (AIPU_GM_POLICY_SHARED/AIPU_GM_POLICY_HALF_DIVIDED)
 * @gm_max_cnt: maximum count of GM region
 * @dtcm_max_cnt: maximum count of DTCM region
 * @sram_disable_head: SRAM disable list
 * @sram_disable: disable count of SRAM
 * @gm_policy_attr: GM policy sysfs attribute, for v3 only
 * @slock:   TCB buffer lock
 */
struct aipu_memory_manager {
	int version;
	bool has_iommu;
	struct device *dev;
	struct mutex lock; /* Protect sram disabled head struct */
	int reg_cnt;
	struct aipu_mem_region *regs[AIPU_CONFIG_MAX_RESERVED_REGIONS];
	struct aipu_mem_region_list mem[ZHOUYI_ASID_COUNT];
	struct aipu_mem_region *gm;
	struct aipu_direct_mem *direct_mem;
	int gm_policy;
	int gm_max_cnt;
	int dtcm_max_cnt;
	struct aipu_sram_disable_per_fd *sram_disable_head;
	int sram_disable;
	struct device_attribute *gm_policy_attr;
	spinlock_t slock; /* Protect tcb_buf list */
};

int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version);
int aipu_deinit_mm(struct aipu_memory_manager *mm);
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp);
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp);
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
struct aipu_tcb *aipu_mm_get_tcb_va(struct aipu_memory_manager *mm, u64 dev_pa);
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id);
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 prev_tail);
void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail);

#endif /* __AIPU_MM_H__ */
