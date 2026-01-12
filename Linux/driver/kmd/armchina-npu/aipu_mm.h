/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#ifndef __AIPU_MM_H__
#define __AIPU_MM_H__

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/iova.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <asm/cacheflush.h>
#include <armchina_aipu.h>
#include "aipu_tcb.h"
#include "zhouyi.h"

#define DEFERRED_FREE  1
#define DMA_BUF_EXEC_ID 1

enum aipu_gm_policy {
	AIPU_GM_POLICY_NONE         = 0,
	AIPU_GM_POLICY_SHARED       = 1,
	AIPU_GM_POLICY_HALF_DIVIDED = 2,
};

enum aipu_mem_region_type {
	AIPU_MEM_REGION_TYPE_MEMORY = 0,
	AIPU_MEM_REGION_TYPE_SRAM   = 1,
	AIPU_MEM_REGION_TYPE_DTCM   = 2,
	AIPU_MEM_REGION_TYPE_GM     = 3,
	AIPU_MEM_REGION_TYPE_MAX    = 4,
};

enum aipu_mem_hold_tcb_status {
	AIPU_MEM_HOLD_TYPE_IDLE     = 0,
	AIPU_MEM_HOLD_TYPE_LINKING  = 1,
	AIPU_MEM_HOLD_TYPE_LINK_PREV = 2,
	AIPU_MEM_HOLD_TYPE_LINK_NEXT = 3,
	AIPU_MEM_HOLD_TYPE_LINKED   = 4,
	AIPU_MEM_HOLD_TYPE_MAX      = 6,
};

struct aipu_mem_region_obj;
struct aipu_job;
struct aipu_hold_tcb_buf {
	u64 head;
	int status;
	struct aipu_buf_desc desc;
	struct list_head node;
	struct aipu_tcb *hold_tcb;
	u64 prev_head;
	u64 prev_tail;
	u64 next_head;
	u64 prev_hold_tcb;
	struct aipu_mem_region *reg;
	int nums;
	int index;
	int hold_index;
	struct aipu_tcb_buf *prev_tbuf;
};

struct __cache_ops {
    void (*flush_range)(void *vaddr, size_t size);
    void (*clean_range)(void *vaddr, size_t size);
    void (*invalidate_range)(void *vaddr, size_t size);
    size_t cache_line_size;
};

struct aipu_phy_block {
	struct list_head list;
	struct page **pages;
	struct sg_table sgt;
	dma_addr_t dma_addr;
	u64 iova_start;
	u64 size;
	u32 page_count;
	void* va;
	struct dma_buf *dmabuf;
	bool bind_memory;
};

struct aipu_iova_buffer {
	u64 iova_start;
	u64 iova_size;
	u64 allocated_size;
	struct list_head phy_blocks;
	struct mutex phy_mutex;
	u32 ref_count;
	u8 region;
	u8 asid;
	struct file *filp;
	int tgid;
	int pid;
	struct list_head node;
	u64 exec_id;
};

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
	u64 range;
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
 * @gm_bytes: V3 GM size (in bytes)
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
 * @importer_bufs: buffers from dma-buf importer(s)
 */
struct aipu_memory_manager {
	int version;
	bool has_iommu;
	struct device *dev;
	struct mutex lock; /* Protect sram disabled head/importer bufs struct */
	int res_cnt;
	u32 valid_asid_cnt;
	struct aipu_mem_region_list mem;
	struct aipu_mem_region_list ase[ZHOUYI_ASID_COUNT];
	int gm_bytes;
	int gm_policy;
	int gm_max_cnt;
	int dtcm_max_cnt;
	struct aipu_sram_disable_per_fd *sram_disable_head;
	int sram_disable;
	struct device_attribute *gm_policy_attr;
	spinlock_t slock; /* Protect tcb_buf list */
	spinlock_t shlock; /* Protect hold tcb_buf list */
	u64 default_asid_base;
	u32 default_asid_size;
	struct kmem_cache *obj_cache;
	struct kmem_cache *reg_cache;
	struct kmem_cache *tbuf_cache;
	struct kmem_cache *hold_tbuf_cache;
	struct aipu_dma_buf_importer *importer_bufs;
	struct aipu_hold_tcb_buf *hold_tcb_head;
	u64 dma_mask;
	struct iommu_domain *iommu_domain;
	struct iova_domain iova_domain;
	struct mutex buffer_mutex;
	struct list_head buffer_list;
	u64 iova_base;
	u64 iova_size;
	u32 buffer_count;
	u64 host_aipu_offset;
	struct __cache_ops cache_ops;
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
u32 aipu_mm_get_asid_cnt(struct aipu_memory_manager *mm);
int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes);
int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next);
void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap);
void get_dtcm(struct aipu_memory_manager *mm, u64 *base, u32 *size);

bool is_grid_end(struct aipu_tcb *tcb);
int print_core_id(struct aipu_memory_manager *mm, u64 head, u64 tail);
struct aipu_tcb *aipu_mm_get_tcb(struct aipu_memory_manager *mm, u64 pa);
struct aipu_tcb *aipu_mm_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail);
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id);
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 prev_tail, bool free_tcb);
void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail);
int aipu_mm_hold_tcb_buf_alloc(struct aipu_memory_manager *mm, struct aipu_job *kjob);
struct aipu_hold_tcb_buf *aipu_mm_get_hold_htbuf(struct aipu_memory_manager *mm, u64 hold_tcb_pa);
void aipu_mm_set_final_htbuf_index(struct aipu_memory_manager *mm, int index);
int aipu_alloc_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp);
int aipu_free_dma_iova_phy(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp);
void aipu_cache_flush(struct aipu_memory_manager *mm, void *vaddr, size_t size);
void aipu_cache_clean(struct aipu_memory_manager *mm, void *vaddr, size_t size);
void aipu_cache_invalidate(struct aipu_memory_manager *mm, void *vaddr, size_t size);
struct aipu_phy_block *aipu_get_block_buffer(struct aipu_memory_manager *mm,
					     u64 iova, char* str);
struct aipu_iova_buffer *aipu_get_iova_buffer_by_exec_id(struct aipu_memory_manager *mm,
							 u64 exec_id, char* str);
int aipu_rebind_dma_iova_phy(struct aipu_memory_manager *mm,
			     struct aipu_rebind_buf_desc *desc,
			     struct file *filp);
int aipu_bind_dma_iova_phy(struct aipu_memory_manager *mm,
			   struct aipu_bind_buf_desc *desc,
			   struct file *filp);
int aipu_copy_block_by_dma_fd(struct aipu_memory_manager *mm,
			      struct aipu_dma_buf *dmabuf_info);
#endif /* __AIPU_MM_H__ */
