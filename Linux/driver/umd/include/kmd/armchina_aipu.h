// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef __UAPI_MISC_ARMCHINA_AIPU_H__
#define __UAPI_MISC_ARMCHINA_AIPU_H__

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * Zhouyi KMD currently supports Zhouyi Z1/Z2/Z3/X1/X2 AIPU hardwares.
 *
 * Structures defined in this header are shared by all hardware versions, but some fields
 * of the structs are specific to certain hardware version(s).
 *
 * In the following member descriptions,
 * [must]     mean that the fields must be filled by user mode driver in all cases.
 * [optional] mean that the fields is optional for user mode driver to enable or not.
 * [alloc]    mean that the buffer(s) represented by the fields must be allocated
 *            by user mode driver before calling IOCTL.
 * [kmd]      mean that the fields should be filled by kernel mode driver
 *            if the calls are successful.
 * [only]     mean that the fields are only for the hardware versions specified to use.
 */

/**
 * emum aipu_arch - AIPU architecture number
 * @AIPU_ARCH_ZHOUYI: AIPU architecture is Zhouyi.
 *
 * This enum is used to indicate the architecture of an AIPU core in the system.
 */
enum aipu_arch {
	AIPU_ARCH_ZHOUYI = 0,
};

/**
 * emum aipu_isa_version - AIPU ISA version number
 * @AIPU_ISA_VERSION_ZHOUYI_Z1: AIPU ISA version is Zhouyi Z1.
 * @AIPU_ISA_VERSION_ZHOUYI_Z2: AIPU ISA version is Zhouyi Z2.
 * @AIPU_ISA_VERSION_ZHOUYI_Z3: AIPU ISA version is Zhouyi Z3.
 * @AIPU_ISA_VERSION_ZHOUYI_X1: AIPU ISA version is Zhouyi X1.
 * @AIPU_ISA_VERSION_ZHOUYI_X2: AIPU ISA version is Zhouyi X2.
 *
 * Zhouyi architecture has multiple ISA versions released.
 * This enum is used to indicate the ISA version of an AIPU core in the system.
 */
enum aipu_isa_version {
	AIPU_ISA_VERSION_ZHOUYI_Z1 = 1,
	AIPU_ISA_VERSION_ZHOUYI_Z2 = 2,
	AIPU_ISA_VERSION_ZHOUYI_Z3 = 3,
	AIPU_ISA_VERSION_ZHOUYI_X1 = 4,
	AIPU_ISA_VERSION_ZHOUYI_X2 = 5,
};

/**
 * What is an AIPU partition?
 *
 *     For Z1/Z2/Z3/X1, a *partition* represents an AIPU core, and the partition count
 *         is the count of AIPU cores in current system.
 *     For X2, a *partition* represents a group of clusters in the same power/computation
 *         domain, and the partition count is the count of such cluster groups. All partitions
 *         share the same TSM and external register base address.
 */

/**
 * struct aipu_partition_cap - Capability of an AIPU partition
 *
 * @id:      [kmd] AIPU partition ID
 * @arch:    [kmd] Architecture number
 * @version: [kmd] ISA version number
 * @config:  [kmd][z1/z2/z3/x1 only] Configuration number
 * @info:    [kmd] Debugging information
 * @cluster_cnt: [kmd][x2 only] Cluster count of this partition
 * @clusters:    [kmd][x2 only] Cluster capacity of this partition
 *
 * For example,
 *    Z2-1104:
 *    arch == AIPU_ARCH_ZHOUYI (0)
 *    version == AIPU_ISA_VERSION_ZHOUYI_Z2 (2)
 *    config == 1104
 *
 *    X2:
 *    arch == AIPU_ARCH_ZHOUYI (0)
 *    version == AIPU_ISA_VERSION_ZHOUYI_X2 (5)
 *    config == 0, not applicable
 */
struct aipu_partition_cap {
	__u32 id;
	__u32 arch;
	__u32 version;
	__u32 config;
	struct aipu_debugger_info {
		__u64 reg_base;	/* External register base address (physical) */
	} info;
	__u32 cluster_cnt;
	struct aipu_cluster_cap {
		__u16 core_cnt; /* homogeneous cores */
		__u16 tec_cnt;
	} clusters[8]; /* every partition has at maximum 8 clusters */
};

/**
 * struct aipu_cap - Common capability of the AIPU partition(s)
 * @partition_cnt:  [kmd] Count of AIPU partition(s) in the system
 * @is_homogeneous: [kmd] IS homogeneous AIPU system or not (1/0)
 * @asid0_base:     [kmd] ASID 0 base address
 * @asid1_base:     [kmd] ASID 1 base address
 * @asid2_base:     [kmd] ASID 2 base address
 * @asid3_base:     [kmd] ASID 3 base address
 * @dtcm_base:      [kmd][x1/x2] DTCM base address
 * @dtcm_size:      [kmd][x1/x2] DTCM size
 * @gm0_base:       [kmd][x2 only] GM region 0 base address (valid if gm0_size > 0)
 * @gm1_base:       [kmd][x2 only] GM region 1 base address (valid if gm1_size > 0)
 * @gm0_size:       [kmd][x2 only] GM region 0 size (in bytes)
 * @gm1_size:       [kmd][x2 only] GM region 1 size (in bytes)
 * @partition_cap:  [kmd] Capability of the single AIPU partition
 *
 * For Z1/Z2/Z3/X1, AIPU driver supports the management of multiple AIPU cores.
 * This struct is used to indicate the common capability of all AIPU core(s).
 * User mode driver should get this capability via AIPU_IOCTL_QUERY_CAP command.
 * If the core count is 1, the per-core capability is in the partition_cap member;
 * otherwise user mode driver should get all the per-core capabilities as the
 * partition_cnt indicates via AIPU_IOCTL_QUERY_PARTITION_CAP command.
 *
 * For X2, user mode driver should get the cluster counts by AIPU_IOCTL_QUERY_PARTITION_CAP
 * command if partition_cnt > 1.
 */
struct aipu_cap {
	__u32 partition_cnt;
	__u32 is_homogeneous;
	__u64 asid0_base;
	__u64 asid1_base;
	__u64 asid2_base;
	__u64 asid3_base;
	__u64 dtcm_base;
	__u32 dtcm_size;
	__u64 gm0_base;
	__u64 gm1_base;
	__u32 gm0_size;
	__u32 gm1_size;
	struct aipu_partition_cap partition_cap;
};

/**
 * enum aipu_mm_data_type - Data/Buffer type
 * @AIPU_MM_DATA_TYPE_NONE:   No type
 * @AIPU_MM_DATA_TYPE_TEXT:   Text (instructions)
 * @AIPU_MM_DATA_TYPE_RODATA: Read-only data (parameters)
 * @AIPU_MM_DATA_TYPE_STACK:  Stack
 * @AIPU_MM_DATA_TYPE_STATIC: Static data (weights)
 * @AIPU_MM_DATA_TYPE_REUSE:  Reuse data (feature maps)
 * @AIPU_MM_DATA_TYPE_TCB:    X2 TCB
 */
enum aipu_mm_data_type {
	AIPU_MM_DATA_TYPE_NONE,
	AIPU_MM_DATA_TYPE_TEXT,
	AIPU_MM_DATA_TYPE_RODATA,
	AIPU_MM_DATA_TYPE_STACK,
	AIPU_MM_DATA_TYPE_STATIC,
	AIPU_MM_DATA_TYPE_REUSE,
	AIPU_MM_DATA_TYPE_TCB,
};

/**
 * enum aipu_buf_region - ASID regions
 * @AIPU_BUF_ASID_0: [z2/z3/x1/x2 only] ASID 0 region
 * @AIPU_BUF_ASID_1: [z2/z3/x1/x2 only] ASID 1 region
 * @AIPU_BUF_ASID_2: [z2/z3/x1/x2 only] ASID 2 region
 * @AIPU_BUF_ASID_3: [x2 only] ASID 3 region
 */
enum aipu_buf_region {
	AIPU_BUF_ASID_0 = 0,
	AIPU_BUF_ASID_1 = 1,
	AIPU_BUF_ASID_2 = 2,
	AIPU_BUF_ASID_3 = 3,
};

/**
 * enum aipu_buf_region - buffer region type
 * @AIPU_BUF_DEFAULT:     [z1/z2/z3/x1/x2] default DDR region
 * @AIPU_BUF_REGION_SRAM: [z1/z2/z3/x1/x2] SRAM region
 * @AIPU_BUF_REGION_DTCM: [x1/x2] DTCM region
 * @AIPU_BUF_REGION_QOS_SLOW_GM: [x2 only] GM region
 * @AIPU_BUF_REGION_QOS_FAST_GM: [x2 only] GM region
 */
enum aipu_buf_region_type {
	AIPU_BUF_REGION_DEFAULT     = 0,
	AIPU_BUF_REGION_SRAM        = 1,
	AIPU_BUF_REGION_DTCM        = 2,
	AIPU_BUF_REGION_QOS_SLOW_GM = 3,
	AIPU_BUF_REGION_QOS_FAST_GM = 4,
};

/**
 * struct aipu_buf_desc - Buffer description.
 *                        KMD returns this info. to the requesting user thread in one ioctl
 * @pa:         [kmd] Buffer physical base address
 * @dev_offset: [kmd] Device offset used in mmap
 * @bytes:      [kmd] Buffer size in bytes
 * @region:     [kmd] this allocated buffer is in memory/SRAM/DTCM/GM region?
 * @asid:       [kmd] ASID region of this buffer
 * @gm_base:    [kmd] GM base address if the region member is QoS slow/fast GM
 */
struct aipu_buf_desc {
	__u64 pa;
	__u64 dev_offset;
	__u64 bytes;
	__u8  region;
	__u8  asid;
	__u64 gm_base;
};

/**
 * struct aipu_buf_request - Buffer allocation request structure.
 * @bytes:         [must] Buffer size to allocate (in bytes)
 * @align_in_page: [must] Buffer address alignment (must be a power of 2)
 * @data_type:     [must] Type of data in this buffer/Type of this buffer
 * @region:        [kmd] set to request a buffer in a default DDR region or a GM region
 * @asid:          [z2/z3/x1/x2 only, optional] from which region (ASID 0/1/2/3) to request the buffer
 * @desc:          [kmd]  Descriptor of the successfully allocated buffer
 */
struct aipu_buf_request {
	__u64 bytes;
	__u32 align_in_page;
	__u32 data_type;
	__u8  region;
	__u8  asid;
	struct aipu_buf_desc desc;
};

/**
 * enum aipu_job_execution_flag - Flags for AIPU's executions
 * @AIPU_JOB_EXEC_FLAG_NONE:         No flag
 * @AIPU_JOB_EXEC_FLAG_SRAM_MUTEX:   The job uses SoC SRAM exclusively.
 * @AIPU_JOB_EXEC_FLAG_QOS_SLOW:     [x2 only] Quality of Service (QoS) slow
 * @AIPU_JOB_EXEC_FLAG_QOS_FAST:     [x2 only] QoS fast
 * @AIPU_JOB_EXEC_FLAG_SINGLE_GROUP: [x2 only] the scheduled job is a single group task
 * @AIPU_JOB_EXEC_FLAG_MULTI_GROUP:  [x2 only] the scheduled job is a multi-groups task
 */
enum aipu_job_execution_flag {
	AIPU_JOB_EXEC_FLAG_NONE         = 0,
	AIPU_JOB_EXEC_FLAG_SRAM_MUTEX   = 1 << 0,
	AIPU_JOB_EXEC_FLAG_QOS_SLOW     = 1 << 1,
	AIPU_JOB_EXEC_FLAG_QOS_FAST     = 1 << 2,
	AIPU_JOB_EXEC_FLAG_SINGLE_GROUP = 1 << 3,
	AIPU_JOB_EXEC_FLAG_MULTI_GROUP  = 1 << 4,
};

/**
 * struct aipu_job_desc - Description of a job to be scheduled.
 * @is_defer_run:      [z1/z2/z3/x1 only, optional] Reserve an AIPU core for this job and defer the running of it
 * @version_compatible:[z1/z2/z3/x1 only, optional] Is this job compatible on AIPUs with different ISA version
 * @core_id:           [z1/z2/z3/x1 optional] ID of the core to reserve
 * @partition_id:      [x2 must] ID of the partition requested to schedule a job onto
 * @do_trigger:        [z1/z2/z3/x1 only, optional] Trigger the previously scheduled deferred job to run
 * @aipu_arch:         [must] Target device architecture
 * @aipu_version:      [must] Target device ISA version
 * @aipu_config:       [z1/z2/z3/x1 only, must] Target device configuration
 * @start_pc_addr:     [z1/z2/z3/x1 only, must] Address of the start PC (buf_pa - asid_base)
 * @intr_handler_addr: [z1/z2/z3/x1 only, must] Address of the AIPU interrupt handler (buf_pa - asid_base)
 * @data_0_addr:       [z1/z2/z3/x1 only, must] Address of the 0th data buffer (buf_pa - asid_base)
 * @data_1_addr:       [z1/z2/z3/x1 only, must] Address of the 1th data buffer (buf_pa - asid_base)
 * @job_id:            [z1/z2/z3/x1 only, must] ID of this job
 * @enable_prof:       [z1/z2/z3/x1 only, optional] Enable performance profiling counters in SoC (if any)
 * @enable_poll_opt:   [z1/z2/z3/x1 only, optional] Enable optimizations for job status polling
 * @exec_flag:         [optional] Combinations of execution flags
 * @dtcm_size_kb:      [x1 only, optional] DTCM size in KB
 * @head_tcb_pa:       [x2 only, must] base address of the head TCB of this job
 * @last_task_tcb_pa:  [x2 only, must] base address of the last task TCB of this job
 * @tail_tcb_pa:       [x2 only, must] base address of the tail TCB of this job
 *
 * For fields is_defer_run/do_trigger/enable_prof/enable_asid/enable_poll_opt,
 * set them to be 1/0 to enable/disable the corresponding operations.
 */
struct aipu_job_desc {
	__u32 is_defer_run;
	__u32 version_compatible;
	union {
		__u32 core_id;
		__u32 partition_id;
	};
	__u32 do_trigger;
	__u32 aipu_arch;
	__u32 aipu_version;
	__u32 aipu_config;
	__u32 start_pc_addr;
	__u32 intr_handler_addr;
	__u32 data_0_addr;
	__u32 data_1_addr;
	__u64 job_id;
	__u32 enable_prof;
	__u32 enable_poll_opt;
	__u32 exec_flag;
	__u32 dtcm_size_kb;
	__u64 head_tcb_pa;
	__u64 last_task_tcb_pa;
	__u64 tail_tcb_pa;
};

/**
 * struct aipu_job_status_desc - Jod execution status.
 * @job_id:    [kmd] Job ID
 * @thread_id: [kmd] ID of the thread scheduled this job
 * @state:     [kmd] Execution state: done or exception
 * @pdata:     [kmd] External profiling results
 */
struct aipu_job_status_desc {
	__u64 job_id;
	__u32 thread_id;
#define AIPU_JOB_STATE_DONE      0x1
#define AIPU_JOB_STATE_EXCEPTION 0x2
	__u32 state;
	struct aipu_ext_profiling_data {
		__s64 execution_time_ns; /* [kmd] Execution time */
		__u32 rdata_tot_msb;     /* [kmd] Total read transactions (MSB) */
		__u32 rdata_tot_lsb;     /* [kmd] Total read transactions (LSB) */
		__u32 wdata_tot_msb;     /* [kmd] Total write transactions (MSB) */
		__u32 wdata_tot_lsb;     /* [kmd] Total write transactions (LSB) */
		__u32 tot_cycle_msb;     /* [kmd] Total cycle counts (MSB) */
		__u32 tot_cycle_lsb;     /* [kmd] Total cycle counts (LSB) */
	} pdata;
};

/**
 * struct aipu_job_status_query - Query status of (a) job(s) scheduled before.
 * @max_cnt:        [must] Maximum number of job status to query
 * @of_this_thread: [must] Get status of jobs scheduled by this thread/all threads share fd (1/0)
 * @status:         [alloc] Pointer to an array (length is max_cnt) to store the status
 * @poll_cnt:       [kmd] Count of the successfully polled job(s)
 */
struct aipu_job_status_query {
	__u32 max_cnt;
	__u32 of_this_thread;
	struct aipu_job_status_desc *status;
	__u32 poll_cnt;
};

/**
 * struct aipu_io_req - AIPU core IO operations request.
 * @core_id: [must] Core ID
 * @offset:  [must] Register offset
 * @rw:      [must] Read or write operation
 * @value:   [must]/[kmd] Value to be written/value readback
 */
struct aipu_io_req {
	__u32 core_id;
	__u32 offset;
	enum aipu_rw_attr {
		AIPU_IO_READ,
		AIPU_IO_WRITE
	} rw;
	__u32 value;
};

/*
 * AIPU IOCTL List
 */
#define AIPU_IOCTL_MAGIC 'A'
/**
 * DOC: AIPU_IOCTL_QUERY_CAP
 *
 * @Description
 *
 * ioctl to query the common capability of AIPUs
 *
 * User mode driver should call this before calling AIPU_IOCTL_QUERYCORECAP.
 */
#define AIPU_IOCTL_QUERY_CAP _IOR(AIPU_IOCTL_MAGIC, 0, struct aipu_cap)
/**
 * DOC: AIPU_IOCTL_QUERY_PARTITION_CAP
 *
 * @Description
 *
 * ioctl to query the capability of an AIPU partition
 *
 * User mode driver only need to call this when the partition count returned by AIPU_IOCTL_QUERYCAP > 1.
 */
#define AIPU_IOCTL_QUERY_PARTITION_CAP _IOR(AIPU_IOCTL_MAGIC, 1, struct aipu_partition_cap)
/**
 * DOC: AIPU_IOCTL_REQ_BUF
 *
 * @Description
 *
 * ioctl to request to allocate a coherent buffer
 *
 * This fails if kernel driver cannot find a free buffer meets the size/alignment request.
 */
#define AIPU_IOCTL_REQ_BUF _IOWR(AIPU_IOCTL_MAGIC, 2, struct aipu_buf_request)
/**
 * DOC: AIPU_IOCTL_FREE_BUF
 *
 * @Description
 *
 * ioctl to request to free a coherent buffer allocated by AIPU_IOCTL_REQBUF
 *
 */
#define AIPU_IOCTL_FREE_BUF _IOW(AIPU_IOCTL_MAGIC, 3, struct aipu_buf_desc)
/**
 * DOC: AIPU_IOCTL_DISABLE_SRAM
 *
 * @Description
 *
 * ioctl to disable the management of SoC SRAM in kernel driver
 *
 * This fails if the there is no SRAM in the system or the SRAM has already been allocated.
 */
#define AIPU_IOCTL_DISABLE_SRAM _IO(AIPU_IOCTL_MAGIC, 4)
/**
 * DOC: AIPU_IOCTL_ENABLE_SRAM
 *
 * @Description
 *
 * ioctl to enable the management of SoC SRAM in kernel driver disabled by AIPU_IOCTL_DISABLE_SRAM
 */
#define AIPU_IOCTL_ENABLE_SRAM _IO(AIPU_IOCTL_MAGIC, 5)
/**
 * DOC: AIPU_IOCTL_SCHEDULE_JOB
 *
 * @Description
 *
 * ioctl to schedule a user job to kernel mode driver for execution
 *
 * This is a non-blocking operation therefore user mode driver should check the job status
 * via AIPU_IOCTL_QUERY_STATUS.
 */
#define AIPU_IOCTL_SCHEDULE_JOB _IOW(AIPU_IOCTL_MAGIC, 6, struct aipu_job_desc)
/**
 * DOC: AIPU_IOCTL_QUERY_STATUS
 *
 * @Description
 *
 * ioctl to query the execution status of one or multiple scheduled job(s)
 */
#define AIPU_IOCTL_QUERY_STATUS _IOWR(AIPU_IOCTL_MAGIC, 7, struct aipu_job_status_query)
/**
 * DOC: AIPU_IOCTL_KILL_TIMEOUT_JOB
 *
 * @Description
 *
 * ioctl to kill a timeout job and clean it from kernel mode driver; works for Z1/Z2/Z2/X1 only.
 */
#define AIPU_IOCTL_KILL_TIMEOUT_JOB _IOW(AIPU_IOCTL_MAGIC, 8, __u32)
/**
 * DOC: AIPU_IOCTL_REQ_IO
 *
 * @Description
 *
 * ioctl to read/write an external register of an AIPU core; works for Z1/Z2/Z2/X1 only.
 */
#define AIPU_IOCTL_REQ_IO _IOWR(AIPU_IOCTL_MAGIC, 9, struct aipu_io_req)

#endif /* __UAPI_MISC_ARMCHINA_AIPU_H__ */
