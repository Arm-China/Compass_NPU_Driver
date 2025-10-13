// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  standard_api.h
 * @brief Zhouyi AIPU User Mode Driver (UMD) Standard API header (for aipu
 * v1/v2/v3)
 * @version 1.0
 */

#ifndef _STANDARD_API_H_
#define _STANDARD_API_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct ctx_handle {
  uint32_t handle;
} aipu_ctx_handle_t;

typedef enum { DEV_IDLE = 0, DEV_BUSY, DEV_EXCEPTION } device_status_t;

typedef enum {
  AIPU_DATA_TYPE_NONE = 0,
  AIPU_DATA_TYPE_BOOL = 1,
  AIPU_DATA_TYPE_U8 = 2,
  AIPU_DATA_TYPE_S8 = 3,
  AIPU_DATA_TYPE_U16 = 4,
  AIPU_DATA_TYPE_S16 = 5,
  AIPU_DATA_TYPE_U32 = 6,
  AIPU_DATA_TYPE_S32 = 7,
  AIPU_DATA_TYPE_U64 = 8,
  AIPU_DATA_TYPE_S64 = 9,
  AIPU_DATA_TYPE_F16 = 0xa,
  AIPU_DATA_TYPE_F32 = 0xb,
  AIPU_DATA_TYPE_F64 = 0xc,
  AIPU_DATA_TYPE_BF16 = 0xd,

  // byte-aligned u/int4
  AIPU_DATA_TYPE_ALIGNED_U4 = 0x12,
  AIPU_DATA_TYPE_ALIGNED_S4 = 0x13,

  // byte-aligned u/int12
  AIPU_DATA_TYPE_ALIGNED_U12 = 0x14,
  AIPU_DATA_TYPE_ALIGNED_S12 = 0x15,

  AIPU_DATA_TYPE_COMPACT_U4 = 0x20,
  AIPU_DATA_TYPE_COMPACT_S4 = 0x21,
  AIPU_DATA_TYPE_COMPACT_U12 = 0x22,
  AIPU_DATA_TYPE_COMPACT_S12 = 0x23,
} aipu_data_type_t;

typedef enum {
  AIPU_TENSOR_TYPE_INPUT = 0,
  AIPU_TENSOR_TYPE_OUTPUT = 1,
  AIPU_TENSOR_TYPE_INTER_DUMP = 2,
  AIPU_TENSOR_TYPE_PRINTF = 3,
  AIPU_TENSOR_TYPE_PROFILER = 4,
  AIPU_TENSOR_TYPE_LAYER_COUNTER = 5,
  AIPU_TENSOR_TYPE_ERROR_CODE = 6, /* only for v1/v2 */
  /**
   * output tensor shape is also as a tensor in AIPU,
   *  - 'aipu_get_tensor_cnt' to get output's counter, which is same with
   * 'AIPU_TENSOR_TYPE_OUTPUT'
   *  - 'aipu_get_tensor_descriptor' to get specified output descriptor, 'size'
   * and 'data_type' describe shape data type and its's size. e.g.[N,H,W,C] with
   * uint16 type, size=sizeof(uint16)*rank=8
   *  - 'aipu_get_tensor' to get specified output shape information
   *    e.g.[N,H,W,C] with uint16 type, occupies 8bytes, each tensor buffer can
   * cast to a uint16 dimension
   */
  AIPU_TENSOR_TYPE_OUT_TENSOR_SHAPE = 7,
} aipu_tensor_type_t;

typedef struct {
  uint32_t id;
  uint32_t size;
  float scale;
  int32_t zero_point;
  aipu_data_type_t data_type;
} aipu_tensor_desc_t;

/**
 * @brief AIPU job status; returned by status querying API
 * aipu_get_job_status().
 */
typedef enum {
  AIPU_JOB_STATUS_NO_STATUS = 0, /**< no status */
  AIPU_JOB_STATUS_DONE,          /**< job execution successfully */
  AIPU_JOB_STATUS_EXCEPTION, /**< job execution failed, encountering exception
                              */
  AIPU_JOB_COREDUMP,         /**< job triggers coredump */
} aipu_job_status_t;

typedef struct {
  uint64_t instr_base;
  void *simulation_aipu;
  void *simulation_mem_engine;
} aipu_debugger_job_info_t;

typedef enum {
  /**
   * AIPU_GLOBAL_CONFIG_TYPE_[*]: for aipu_config_global() only;
   * AIPU_JOB_CONFIG_TYPE_[*]: for aipu_config_job() only;
   * AIPU_CONFIG_TYPE_[*]: for aipu_config_global/aipu_config_job();
   */

  /* 1~16 */
  AIPU_JOB_CONFIG_TYPE_DUMP_TEXT = 0x1,
  AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT = 0x2,
  AIPU_JOB_CONFIG_TYPE_DUMP_RODATA = 0x4,
  AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR = 0x8,
  AIPU_JOB_CONFIG_TYPE_DUMP_INPUT = 0x10,
  AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT = 0x20,
  AIPU_JOB_CONFIG_TYPE_DUMP_REUSE = 0x40,
  AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN = 0x80,
  AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION = 0x100,
  AIPU_JOB_CONFIG_TYPE_DUMP_PROFILE = 0x200,
  AIPU_JOB_CONFIG_TYPE_DYNAMIC_PARAMS = 0x400,

  /* 16~24 */
  AIPU_CONFIG_TYPE_SIMULATION = 0x10000,
  AIPU_CONFIG_TYPE_HW = 0x20000,

  /* 24~32 */
  AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK = 0x1000000,
  AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK = 0x2000000,
} aipu_config_type_t;

typedef struct {
  /* dump_dir is used as file dump-path */
  const char *dump_dir;
  /* name prefix of dump files */
  const char *prefix;
  /* name prefix of output dump files */
  const char *output_prefix;
  /* name prefix of profile/printf data files */
  const char *misc_prefix;
} aipu_job_config_dump_t;

typedef struct {
  /* data_dir is used as aipu v1/v2 simulation data file directory */
  const char *data_dir;
} aipu_job_config_simulation_t;

/**
 * @brief Simulation related configuration
 *
 * @todo align with simulator 'config_t'
 */
typedef struct {
  uint32_t log_level;        /**< simulator log level, default 0 */
  bool verbose;              /**< simulator log print, default false */
  const char *log_file_path; /**< simulator log file path */
  const char *json_filename; /**< specify json filename */
  const char *plugin_name;   /**< specify plugin dynamic library filename */

  const char
      *simulator; /**< only for v1/v2, specify executable simulator full path */
  const char *npu_arch_desc; /**< only for >=v3, specify aipu target, such as
                                'X2_1204MP3' */
  uint32_t gm_size;   /**< only for >=v3, v3 default 4M,v3_2 default 8M */
  bool enable_avx;    /**< only for >=v3, default false */
  bool enable_calloc; /**< only for >=v3, default false */

  bool en_eval; /**< only for <=v3, enable perf evaluation, default false */

  bool en_l2d; /**< [[deprecated]]: only for >=v3_2, default false */
  /* fast perf evaluation config for >=v3_2 */
  bool en_fast_perf; /**< enable fast perf, default false */
  uint32_t freq_mhz; /**< fast perf: frequency setting, default 1000 */
  uint32_t
      ddr_latency_rd; /**< fast perf: ddr latency read setting, default 0 */
  uint32_t
      ddr_latency_wr; /**< fast perf: ddr latency write setting, default 0 */
  uint32_t ddr_bw; /**< fast perf: ddr bandwidth setting, indicate how many bits
                      of the axi data bus, default 256 */
  float ddr_bw_ratio; /**< [[deprecated]]: fast perf: ddr bandwidth ratio
                         setting, default 1.0 */
  const char
      *perf_report; /**< fast perf: report filename, default './perf.csv' */
} aipu_global_config_simulation_t;

/**
 * @brief HW related configuration
 */
typedef struct {
  /**
   * set false if job polling thread isn't commit thread;
   * set true if job polling thread is commit thread.
   *
   * default true(no config via this structure): the polling thread is identical
   * with commit thread.
   *
   * eg: poll job in non-commit thread
   *   hw_config->poll_in_commit_thread = false;
   *   aipu_config_global(ctx, AIPU_CONFIG_TYPE_HW, hw_config);
   *
   * note: the config is effictive in only one process context.
   */
  bool poll_in_commit_thread;

  /**
   * only for v3, default true (no config via this structure), and we also
   * suggest set it true in v3, some signal irqs are coupling with tec done
   * interrupt, such as operator printf. we suggest enable tec done irq
   * defaultly, otherwise it maybe HANG when operator has printf and tec done is
   * disable
   */
  bool enable_tec_done_irq;
} aipu_global_config_hw_t;

/**
 * @brief function prototype for job's callback handler
 *
 * @param[in] job_id    Done job's id
 * @param[in] job_state Done job's state
 *
 * @retval 0 for successfully calling, other value for abnormally calling
 */
#if defined(BUILD_PYTHON_API)
#include <functional>
typedef std::function<int(uint64_t, aipu_job_status_t)>
    aipu_job_callback_func_t;
#else
typedef int (*aipu_job_callback_func_t)(uint64_t job_id,
                                        aipu_job_status_t job_state);
#endif

/**
 * @brief AIPU core info struct; returned by UMD API for AIPU debugger to use
 */
typedef struct aipu_core_info {
  uint64_t reg_base; /**< core register base address */
} aipu_core_info_t;

typedef enum {
  AIPU_SHARE_BUF_IN_ONE_PROCESS =
      0x0, /**< >=v3_2 doesn't support, please use DMABUF instead */
  AIPU_SHARE_BUF_DMABUF = 0x1,
  AIPU_SHARE_BUF_CUSTOMED =
      0x2, /**< >=v3_2 doesn't support, please use DMABUF instead */
  AIPU_SHARE_BUF_ATTACH_DMABUF = 0x3
} aipu_share_case_type_t;

/**
 * @struct aipu_shared_tensor_t
 *
 * @brief case1: AIPU_SHARE_BUF_IN_ONE_PROCESS
 *        mark one tensor buffer of one graph as shared with other graphs. (in
 * one process context). buffer comes from
 * 'aipu_ioctl/AIPU_IOCTL_ALLOC_SHARE_BUF'
 *
 * @brief case2: AIPU_SHARE_BUF_DMABUF
 *        describe a shared buffer based on dma_buf mechanism.
 *        this dma_buf is managed by NPU KMD. (among multiple processes).
 *        dma-buf comes from 'aipu_ioctl/AIPU_IOCTL_ALLOC_DMABUF'
 *
 * @brief case3: AIPU_SHARE_BUF_CUSTOMED
 *        specify a shared buffer allocated in external memory manager.
 *        (not original NPU driver)
 *
 * @brief case4: AIPU_SHARE_BUF_ATTACH_DMABUF
 *        describe a shared buffer based on dma_buf mechanism.
 *        this dma_buf is managed by other modules(not KMD). (among multiple
 * processes). dma-buf comes from other module
 *
 * @note for case1:
 *       the share action is based on one process contex, not among multiple
 * processes. it has to set 'shared_case_type' as AIPU_SHARE_BUF_IN_ONE_PROCESS.
 *       need parameters: {type, tensor_idx, pa, shared_case_type}
 *
 * @note for case2:
 *       share a common buffer via dma_buf framework among multiple
 * modules/processes. this dma_buf is managed by KMD self. it has to set
 * 'shared_case_type' as AIPU_SHARE_BUF_DMABUF. need paramsters: {type,
 * tensor_idx, dma-buf fd, offset, shared_case_type}
 *
 * @note for case3:
 *       specify a shared buffer allocated in customed memory manager. it has to
 * set 'shared_case_type' as AIPU_SHARE_BUF_CUSTOMED. in addition, since the
 * buffer is not allocated by original NPU driver, it has to confirm the buffer
 * matches the ASID constraint. need parameters: {type, tensor_idx, pa,
 * shared_case_type}
 *
 * @note for case4:
 *       share a common buffer via dma_buf framework among multiple
 * modules/processes. this dma_buf is managed by other modules(not KMD). it has
 * to set 'shared_case_type' as AIPU_SHARE_BUF_ATTACH_DMABUF. need paramsters:
 * {type, tensor_idx, fd, offset, shared_case_type}
 *
 * @attention if buffer is from external, such as case3/case4, we suggest
 * provided_size=actural_size + 2K bytes on v3 target
 * @attention 'AIPU_SHARE_BUF_IN_ONE_PROCESS' and 'AIPU_SHARE_BUF_CUSTOMED' are
 * deprecated in >=v3_2 version, please use dmabuf instead. >=v3_2 driver
 * supports dynamic asid0, driver will rebind dmabuf to job buffer based 3GB
 *            range
 * @attention if >=v3_2 dmabuf's size is larger than graph's io buffer size, you
 * need to set 'reserved_iova_size' in 'aipu_create_job_cfg_t' to make sure
 * enough iova size to bind dmabuf. because >=v3_2 defaultly only reserves io
 * buffer size for iova.
 */
typedef struct aipu_shared_tensor_info {
  /* the common fields */
  aipu_tensor_type_t type; /**< the shared tensor's type: input/output */
  uint32_t tensor_idx;     /**< the shared tensor's index */

  uint64_t id; /**< [[deprecate]] */
  uint64_t pa; /**< the physical address of shared tensor, ignored for dma_buf
                  share */

  /* fields for case2/4, recommended */
  int dmabuf_fd; /**< the fd corresponding to shared buffer from dma_buf
                    allocator */
  uint32_t offset_in_dmabuf; /**< offset in dma_buf fd, deprecate in >=v3_2 */

  /* it has to set this type, indicates sharing is for which case. */
  int shared_case_type;
} aipu_shared_tensor_info_t;

/**
 * @struct aipu_dmabuf_op_t
 *
 * @brief fill data to dma_buf or get data from dma_buf through this struct,
 *        and maily for `aipu_ioctl/AIPU_IOCTL_WRITE_DMABUF` and
 * `aipu_ioctl/AIPU_IOCTL_READ_DMABUF`
 *
 */
typedef struct aipu_dmabuf_op {
  int dmabuf_fd; /**< the fd corresponding to shared buffer from dma_buf
                    allocator */
  uint32_t offset_in_dmabuf; /**< the shared address offset in dma_buf which is
                                specified by 'fd' */
  uint32_t size; /**< the data size: filled data size or fetched data size */
  char *data;    /**< [in] for dma-buf write, [out] for dma-buf read */
} aipu_dmabuf_op_t;

/**
 * @struct aipu_share_buf_t
 *
 * @brief request common share buffer, must specify 'size' field,
 *        and mainly for `aipu_ioctl/AIPU_IOCTL_ALLOC_SHARE_BUF` ,
 * `aipu_ioctl/AIPU_IOCTL_FREE_SHARE_BUF`
 *
 */
typedef struct aipu_share_buf {
  uint64_t pa;   /**< [out] for share buf alloc, [in] for share buffer free */
  uint64_t va;   /**< [out] buffer va, only for share buf alloc, and ignored by
                    share buf free */
  uint32_t size; /**< buffer size: filled by USER */
  uint32_t mem_type; /**< memory region type: AIPU_MEM_REGION_DEFAULT,
                          AIPU_MEM_REGION_SRAM, ignored by share buf free */
} aipu_share_buf_t;

/**
 * @struct aipu_dma_buf_req_t
 *
 * @brief request dma-buf from KMD, @param bytes is provided by user, @param fd
 * will be filled by KMD, and mainly for `aipu_ioctl`
 *
 * @note map to KMD's `struct aipu_dma_buf_request`
 */
typedef struct aipu_dma_buf_req {
  int fd; /**< [out] dma-buf fd provided by KMD */
  uint64_t bytes;
} aipu_dma_buf_req_t;

/**
 * @struct aipu_dma_buf_desc_t
 *
 * @brief dma-buf descriptor structure,
 *        and mainly for `aipu_ioctl/AIPU_IOCTL_GET_DMABUF_INFO`,
 * `aipu_ioctl/AIPU_IOCTL_ATTACH_DMABUF`
 *
 * @note map to KMD's `struct aipu_dma_buf`
 */
typedef struct aipu_dma_buf_desc {
  int fd;
  uint64_t pa;    /**< [out] pa of dma-buf fd */
  uint64_t bytes; /**< [out] bytes of dma-buf fd */
} aipu_dma_buf_desc_t;

/**
 * @struct aipu_driver_version
 *
 * @brief get version number of UMD & KMD, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_VERSION`
 *
 * @note the version number is determined in compiling phase,
 *       currently both version numbers is identical. the max
 *       buffer space is 16 bytes.
 */
typedef struct aipu_driver_version {
  char umd_version[16]; /**< the buffer for storing UMD version string */
  char kmd_version[16]; /**< the buffer for storing KMD version string */
} aipu_driver_version_t;

/**
 * @struct aipu_bin_buildversion_t
 *
 * @brief get build version number of model binary, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION`
 *
 * @note the version number is determined by NN compiler.
 */
typedef struct aipu_bin_buildversion {
  uint64_t graph_id; /**< the graph id to be searched */
  uint32_t
      aipubin_buildversion; /**< [out] the build version of searched graph */
} aipu_bin_buildversion_t;

/**
 * [[deprecate]].due to dynamic inputs are same to graph's inputs number
 * @struct aipu_dynshape_num_t
 *
 * @brief get how many input counter of dynamic shape graph, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_DS_NUM`
 *
 * @note dynamic shape is generated by NN compiler. eg:
 *       [[N1*H1*W1*C1], [N2*H2*W2*C2]], ds_num is 2.
 */
typedef struct aipu_dynshape_num {
  uint64_t graph_id; /**< the graph id to be searched */
  uint32_t *ds_num;  /**< the dynamic shape count graph support */
} aipu_dynshape_num_t;

/**
 * [[deprecate]].use 'aipu_dynshape_rank_t' instead
 * @struct aipu_dynshape_dim_num_t
 *
 * @brief get specific shape dimensions of specified shape, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_DS_DIM_NUM`
 *
 * @note 'ds_dim_num' indicates the shape has how many uint32 words,
 *       N*H*W*C => 4 words
 */
typedef struct aipu_dynshape_dim_num {
  uint64_t graph_id;    /**< the graph id to be inquired */
  uint32_t ds_idx;      /**< the dynamic shape index */
  bool max_threshhold;  /**< true: for max shape, false: for min shape */
  uint32_t *ds_dim_num; /**< the dynamic dim number of requested shape index */
} aipu_dynshape_dim_num_t;

/**
 * [[deprecate]].use 'aipu_dynshape_dimension_t' instead
 * @struct aipu_dynshape_info_t
 *
 * @brief dynamic shape information generated by NN compiler, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_DS_INFO`
 *
 * @note 'ds_data' is uint32 array indicates a shape, eg:
 *        N*H*W*C => [u32_N, u32_H, u32_W, u32_C], total 4 words.
 */
typedef struct aipu_dynshape_info {
  uint64_t graph_id;   /**< the graph id to be searched */
  uint32_t ds_idx;     /**< the dynamic shape index */
  bool max_threshhold; /**< true: for max shape, false: for min shape */
  uint32_t *ds_data;   /**< the dynamic shape information of inquired */
} aipu_dynshape_info_t;

/**
 * @struct aipu_dynshape_rank_t
 *
 * @brief get rank of specified input, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_DS_RANK`
 *
 * @note e.g.[N,H,W,C] rank is 4, and 'ds_idx' is the index of inputs which
 * includes static and dynamic input
 */
typedef struct aipu_dynshape_rank {
  uint64_t graph_id; /**< the graph id to be inquired */
  uint32_t ds_idx;   /**< the dynamic shape index */
  uint32_t rank;     /**< [out] the rank of 'ds_idx'th input */
} aipu_dynshape_rank_t;

/**
 * @struct aipu_dynshape_dimension_t
 *
 * @brief get min/max dimension constraint of specified input, and mainly for
 * `aipu_ioctl/AIPU_IOCTL_GET_DS_DIM_CONSTRAINT`. rank information should get
 * from `aipu_ioctl/AIPU_IOCTL_GET_DS_RANK`
 *
 * @note e.g.min_dim=[1,64,64,1], max_dim=[1,256,256,1]
 */
typedef struct aipu_dynshape_dimension {
  uint64_t graph_id; /**< the graph id to be searched */
  uint32_t ds_idx;   /**< the dynamic shape index */
  uint32_t *min_dim; /**< [out] the minium dimensions of 'ds_idx'th input */
  uint32_t *max_dim; /**< [out] the maximum dimensions of 'ds_idx'th input */
} aipu_dynshape_dimension_t;

/**
 * @struct aipu_dynshape_item_t
 *
 * @brief dynamic shape information of `ds_idx`
 *
 * @note 'ds_data' is uint32 array indicates a shape, eg:
 *        N*H*W*C => [u32_N, u32_H, u32_W, u32_C], total 4 words.
 *        you can provide dyanmic shape information only, and leave static shape
 * as default
 */
typedef struct aipu_dynshape_item {
  uint32_t ds_idx;   /**< the dynamic shape index */
  uint32_t *ds_data; /**< the dynamic shape information of configured */
} aipu_dynshape_item_t;

/**
 * @struct aipu_dynshape_param_t
 *
 * @brief dynamic shape information to be confirgured for one graph.
 *
 * @note input_shape_cnt can be less than input tensors number of graph, but you
 * are suggested to providing full input shapes. Otherwise, not provided shape
 * will be set as minimum shape which is same with building time
 */
typedef struct aipu_dynshape_param {
  uint32_t input_shape_cnt;          /**< input shape counter provided */
  aipu_dynshape_item_t *shape_items; /**< configured input shape info */
} aipu_dynshape_param_t;

typedef enum {
  AIPU_JOB_PART0 = 0x0,
  AIPU_JOB_PART1 = 0x1,
  AIPU_JOB_PART2 = 0x2,
  AIPU_JOB_PART3 = 0x3,
} aipu_job_part_t;

typedef enum {
  AIPU_JOB_QOS_SLOW = 0x0,
  AIPU_JOB_QOS_HIGH = 0x1
} aipu_job_qos_t;

typedef enum {
  AIPU_MEM_REGION_DEFAULT = 0, /**< DDR */
  AIPU_MEM_REGION_SRAM = 1,    /**< On-Chip Memory */
  AIPU_MEM_REGION_DTCM = 2,    /**< Data Tightly Couped Memory, only for v2 */
} aipu_mem_region_t;

/**
 * @union aipu_load_graph_cfg
 *
 * @brief support some configuration done in loading graph stage.
 *
 * @note wt_mem_region
 *       if config weight buffer region for aipu v1/v2, just ignore partition_id
 * & qos_level, set them as 0 and only set fm_mem_region field. the buffer is
 * allocated successfully only if there's enough space in region marked by
 * `fm_mem_region`.
 *
 *       usually it only needs to set `fm_mem_region` in struct
 * aipu_create_job_cfg to control feature map buffer allocated from which memory
 * region. if it's sure that the region's free space is enough large, you can
 * set `wt_mem_region` to try to allocate buffer from it. if it fail to allocate
 *       buffer from marked region, it will try according to region order:
 * DTCM->SRAM->DDR.
 *
 *       Attention: if it hopes to allcate weight buffer from SRAM, it has to
 * confirm the SRAM range locates in ASID1 address scope.
 *
 * @note wt_idxes
 *       the indexes of weight tensors, those tensor buffers firstly try to be
 * allocated from region specified in 'wt_mem_region'.
 *
 * @note wt_idxes_cnt
 *       the indexes of weight tensors, those tensor buffers firstly try to be
 * allocated from region specified in 'wt_mem_region'.
 *
 * @note extra_weight_path
 *       specify path of extra weight, which is generated by model compile
 * @note put_weight_gm, default false
 *       put whole weight to NPU global memory, only for small model
 * accelerating
 * @note put_desc_gm, default false
 *       put whole descriptor to NPU global memory, only for small model
 * accelerating
 * @note put_ws_gm, default false
 *       put whole workspace to NPU global memory, only for small model
 * accelerating
 */
typedef struct aipu_load_graph_cfg {
  union {
    uint32_t misc = 0;
    struct {
      uint8_t wt_mem_region : 4; /**< default 0, weight buffer memory region */
    };
  };

  int32_t *wt_idxes; /**< specify weights allocated from 'wt_mem_region'. make
                        sure region in asid1 3G/3.5G range, otherwise DONT use
                        this feature */
  int32_t wt_idxes_cnt;          /**< the emement number in wt_idxes */
  const char *extra_weight_path; /**< the extra weight files path */
  bool put_weight_gm;
  bool put_desc_gm;
  bool put_ws_gm;
} aipu_load_graph_cfg_t;

/**
 * @union aipu_create_job_cfg
 *
 * @brief config job's partition, qos(priority), feature map
 *        and weight buffer region on creating a job.
 *
 * @note fm_mem_region
 *       if config feature buffer region for aipu v1/v2, just ignore
 * partition_id & qos_level, set them as 0 and only set fm_mem_region field. the
 * buffer is allocated successfully only if there's enough space in region
 * marked by `fm_mem_region`.
 *
 *       Attention: if it hopes to allcate weight buffer from SRAM, it has to
 * confirm the SRAM range locates in ASID0 address scope.
 *
 * @note fm_idxes
 *       the indexes of feature map tensors, those tensor buffers will firstly
 * try to be allocated from region specified in 'fm_mem_region'.
 *
 * @note dbg_dispatch and dbg_core_id, only for >=v3, and will be deprecate in
 * the future it can dispatch job to some core for debug. it needs not to set
 * them in normal cases. replaced by 'bind_xx' now
 *
 * @note bind_enable, bind_cluster_cnt, bind_cluster_ids and bind_core_ids
 *       it can dispatch job to cluster or cores, and >v3 supports multi-cores
 * binding
 *
 * @note reserved_iova_size only for >=v3_2
 *       driver will reserve iova size equal to io buffer for dmabuf rebinding
 * at least, but if dmabuf needs larger size, you need to specify it here.
 *
 * @attention bind core only makes model parallelism with 1 subgraph in first
 * model
 */
typedef struct aipu_create_job_cfg {
  union {
    uint32_t misc = 0;
    struct {
      uint8_t partition_id : 4; /**< [[deprecate]] always 0 */
      uint8_t dbg_dispatch : 1; /**< will be deprecated in the future, replaced
                                   by 'bind_xx' fields */
      uint8_t dbg_core_id : 3;  /**< will be deprecated in the future. range [0,
                                   max_core_id in cluster], only  support 1 core
                                   binding */
      uint8_t qos_level : 4; /**< defalut 0, low priority, only for aipu v3 */
      uint8_t
          fm_mem_region : 4; /**< default 0, feature map buffer memory region */
    };
  };

  bool bind_enable;         /**< enable bind dispatch */
  uint8_t bind_cluster_ids; /**< bitx represents clusterx, for example 0x02 is
                               binding to cluster1 current only support 1
                               cluster binding */
  uint8_t
      bind_core_ids; /**< bitx represents corex, for example 0x03 is binding to
                        core0&core1 multi-core binding only supports >v3 */

  uint32_t
      reserved_iova_size; /**< only for >v3, default is graph's io size. dynamic
                             asid0' iova size is reserved for dmabuf */

  int32_t *fm_idxes; /**< specify feature maps allocated from 'fm_mem_region'.
                        If SRAM or DTCM, make sure it in ASID0 region */
  int32_t fm_idxes_cnt; /**< the emement number in fm_idxes */

  aipu_dynshape_param_t
      *dynshape; /**< [[deprecate]], replaced by bellow 'dynshape_params' */
  aipu_dynshape_param_t dynshape_params; /**< dynamic shape parameter,
                                              you can also set or update it by
                                            'aipu_config_job' */
} aipu_create_job_cfg_t;

/**
 * @brief only for hardware
 *
 */
typedef enum {
  AIPU_IOCTL_READ_DBGREG = 0,
  AIPU_IOCTL_WRITE_DBGREG,
  AIPU_IOCTL_READ_TSMREG,
  AIPU_IOCTL_WRITE_TSMREG,
  AIPU_IOCTL_READ_MEM,
  AIPU_IOCTL_WRITE_MEM,
} aipu_ioctl_rw_t;

typedef struct {
  aipu_ioctl_rw_t type; /* dbg or tsm register rw */
  uint32_t offset;
  uint32_t value;
} aipu_reg_rw_t;

typedef struct {
  aipu_ioctl_rw_t type; /* mem rw */
  uint64_t job_id;
  uint32_t size;
  uint32_t addr; /* npu 32bit memory space */
  void *data;
} aipu_mem_rw_t;

/**
 * @brief ioctl commands for scattered operations
 *
 * @note it should be named `AIPU_UMD_IOCTL_xx`
 */
typedef enum {
  AIPU_IOCTL_SET_PROFILE = 255,
  AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION,
  AIPU_IOCTL_GET_DS_NUM,     /* [[deprecate]] */
  AIPU_IOCTL_GET_DS_DIM_NUM, /* [[deprecate]] */
  AIPU_IOCTL_GET_DS_INFO,    /* [[deprecate]] */
  AIPU_IOCTL_GET_DS_RANK,
  AIPU_IOCTL_GET_DS_DIM_CONSTRAINT,
  AIPU_IOCTL_SET_DYNAMIC_ASID1, /* only v3 need, for core bind and multi-model
                                   parallel, default is false */
  AIPU_IOCTL_ALLOC_SHARE_BUF,
  AIPU_IOCTL_FREE_SHARE_BUF,

  /* map to KMD's ioctl */
  AIPU_IOCTL_ALLOC_DMABUF,
  AIPU_IOCTL_FREE_DMABUF,
  AIPU_IOCTL_GET_DMABUF_INFO,
  AIPU_IOCTL_WRITE_DMABUF,
  AIPU_IOCTL_READ_DMABUF,
  AIPU_IOCTL_ATTACH_DMABUF,
  AIPU_IOCTL_DETACH_DMABUF,
  AIPU_IOCTL_GET_VERSION,
  AIPU_IOCTL_ABORT_CMDPOOL,
  AIPU_IOCTL_ENABLE_TICKCOUNTER,
  AIPU_IOCTL_DISABLE_TICKCOUNTER,

  /* register or memory rw */
  AIPU_IOCTL_RW_REGISTER,
  AIPU_IOCTL_RW_MEMORY,
  AIPU_IOCTL_UMD_MAX,
} aipu_ioctl_cmd_t;

/**
 * @brief This aipu_status_t enumeration captures the result of any API function
 *        that has been executed. Success is represented by AIPU_STATUS_SUCCESS
 *        which has a value of zero. Error statuses are assigned positive
 * integers and their identifiers start with the AIPU_STATUS_ERROR prefix.
 */
typedef enum {
  /* AIPU driver runtime error code */
  AIPU_STATUS_SUCCESS = 0x0,
  AIPU_STATUS_ERROR_NULL_PTR = 0x1,
  AIPU_STATUS_ERROR_INVALID_CTX = 0x2,
  AIPU_STATUS_ERROR_OPEN_DEV_FAIL = 0x3,
  AIPU_STATUS_ERROR_DEV_ABNORMAL = 0x4,
  AIPU_STATUS_ERROR_DEINIT_FAIL = 0x5,
  AIPU_STATUS_ERROR_INVALID_CONFIG = 0x6,
  AIPU_STATUS_ERROR_UNKNOWN_BIN = 0x7,
  AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED = 0x8,
  AIPU_STATUS_ERROR_INVALID_GBIN = 0x9,
  AIPU_STATUS_ERROR_TARGET_NOT_FOUND = 0xA,
  AIPU_STATUS_ERROR_INVALID_GRAPH_ID = 0xB,
  AIPU_STATUS_ERROR_OPEN_FILE_FAIL = 0xC,
  AIPU_STATUS_ERROR_MAP_FILE_FAIL = 0xD,
  AIPU_STATUS_ERROR_READ_FILE_FAIL = 0xE,
  AIPU_STATUS_ERROR_WRITE_FILE_FAIL = 0xF,
  AIPU_STATUS_ERROR_INVALID_JOB_ID = 0x10,
  AIPU_STATUS_ERROR_JOB_EXCEPTION = 0x11,
  AIPU_STATUS_ERROR_JOB_TIMEOUT = 0x12,
  AIPU_STATUS_ERROR_OP_NOT_SUPPORTED = 0x13,
  AIPU_STATUS_ERROR_INVALID_OP = 0x14,
  AIPU_STATUS_ERROR_INVALID_SIZE = 0x15,
  AIPU_STATUS_ERROR_BUF_ALLOC_FAIL = 0x16,
  AIPU_STATUS_ERROR_BUF_FREE_FAIL = 0x17,
  AIPU_STATUS_ERROR_INVALID_CORE_ID = 0x18,
  AIPU_STATUS_ERROR_RESERVE_SRAM_FAIL = 0x19,
  AIPU_STATUS_ERROR_INVALID_TENSOR_ID = 0x1A,
  AIPU_STATUS_ERROR_INVALID_CLUSTER_ID = 0x1B,
  AIPU_STATUS_ERROR_INVALID_PARTITION_ID = 0x1C,
  AIPU_STATUS_ERROR_PRINTF_FAIL = 0x1D,
  AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE = 0x1E,
  AIPU_STATUS_ERROR_INVALID_GM = 0x1F,
  AIPU_STATUS_ERROR_INVALID_SEGMMU = 0x20,
  AIPU_STATUS_ERROR_INVALID_QOS = 0x21,
  AIPU_STATUS_ERROR_INVALID_TENSOR_CNT = 0x22,
  AIPU_STATUS_ERROR_TIMEOUT = 0x23,
  AIPU_STATUS_ERROR_NO_BATCH_QUEUE = 0x24,
  AIPU_STATUS_ERROR_MARK_SHARED_TENSOR = 0x25,
  AIPU_STATUS_ERROR_SET_SHARED_TENSOR = 0x26,
  AIPU_STATUS_ERROR_DMABUF_SHARED_IO = 0x27,
  AIPU_STATUS_ERROR_GET_SHAPE_FAILED = 0x28,
  AIPU_STATUS_ERROR_SET_SHAPE_FAILED = 0x29,
  AIPU_STATUS_ERROR_NOT_CONFIG_SHAPE = 0x30,
  AIPU_STATUS_ERROR_UNMATCH_OUT_SHAPE = 0x31,
  AIPU_STATUS_ERROR_ZERO_TENSOR_SIZE = 0x32,
  AIPU_STATUS_ERROR_ALLOC_GRIP_ID = 0x33,
  AIPU_STATUS_ERROR_ALLOC_GROUP_ID = 0x34,
  AIPU_STATUS_ERROR_NOT_FOUND_IN_HASHTABLE = 0x35,
  AIPU_STATUS_ERROR_INVALID_COREDUMP = 0x36,
  AIPU_STATUS_ERROR_JOB_DISPATCH_FAIL = 0x37,
  AIPU_STATUS_MAX = 0x38,
  /* AIPU layer library runtime error code */
  AIPU_STATUS_ERROR_UNKNOWN_ERROR = 0x200,
  AIPU_STATUS_ERROR_KEYBOARD_INTERRUPT = 0x300,
  AIPU_STATUS_ERROR_SYSTEM_ERR = 0x301,
  AIPU_STATUS_ERROR_OUT_OF_SPEC = 0x380,
  AIPU_STATUS_ERROR_OUT_OF_AIFF_SPEC = 0x381,
  AIPU_STATUS_ERROR_OUT_OF_TPC_SPEC = 0x382,
  AIPU_STATUS_ERROR_OUT_OF_DMA_SPEC = 0x383,
  AIPU_STATUS_ERROR_OUT_OF_MEM_RANGE = 0x400,
  AIPU_STATUS_ERROR_OUT_OF_SRAM_RANGE = 0x401,
  AIPU_STATUS_ERROR_OUT_OF_LSRAM0_RANGE = 0x402,
  AIPU_STATUS_ERROR_OUT_OF_LSRAM1_RANGE = 0x403,
  AIPU_STATUS_ERROR_OUT_OF_LSRAM_RANGE = 0x404,
  AIPU_STATUS_ERROR_OUT_OF_GSRAM0_RANGE = 0x405,
  AIPU_STATUS_ERROR_OUT_OF_GSRAM1_RANGE = 0x406,
  AIPU_STATUS_ERROR_OUT_OF_GSRAM_RANGE = 0x407,
  AIPU_STATUS_ERROR_ARITHMETIC_ERR = 0x480,
  AIPU_STATUS_ERROR_FLOAT_POINT_ERR = 0x481,
  AIPU_STATUS_ERROR_UNDERFLOW_ERR = 0x482,
  AIPU_STATUS_ERROR_OVERFLOW_ERR = 0x483,
  AIPU_STATUS_ERROR_NOT_A_NUMBER_ERR = 0x484,
  AIPU_STATUS_ERROR_INFINITY_ERR = 0x485,
  AIPU_STATUS_ERROR_STRING_LENGTH_ERR = 0x486,
  AIPU_STATUS_ERROR_ZERO_DIVISION_ERR = 0x487,
} aipu_status_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief This API is used to initialize AIPU UMD context
 *
 * @param[out] ctx Pointer to a memory location allocated by application where
 * UMD stores the opaque context handle struct
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note Before invoking any other UMD API calls, any UMD application must
 * initialize a context first.
 */
aipu_status_t aipu_init_context(aipu_ctx_handle_t **ctx);

/**
 * @brief This API is used to destroy AIPU UMD context
 *
 * @param[in] ctx Pointer to a context handle struct returned by
 * aipu_init_context
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_deinit_context(const aipu_ctx_handle_t *ctx);

/**
 * @brief This API is used to track what happens when an error code is returned
 * by a UMD API.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  status Status returned by UMD standard API
 * @param[out] msg    Pointer to a memory location allocated by application
 * where UMD stores the message string pointer
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_error_message(const aipu_ctx_handle_t *ctx,
                                     aipu_status_t status, const char **msg);

/**
 * @brief This API is used to configure a specified global option.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  types  Configuration type(s)
 * @param[out] config Pointer to a memory location allocated by application
 * where application stores the configuration data struct
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note accepted types/config:
 * AIPU_CONFIG_TYPE_SIMULATION/aipu_global_config_simulation_t
 * @note accepted types/config: AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK/none
 * @note accepted types/config: AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK/none
 * @note accepted types/config: AIPU_CONFIG_TYPE_HW
 */
aipu_status_t aipu_config_global(const aipu_ctx_handle_t *ctx, uint64_t types,
                                 void *config);

/**
 * @brief This API loads an offline built AIPU executable graph binary from file
 * system.
 *
 * @param[in]  ctx   Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  garph Executable graph binary file path
 * @param[out] id    Pointer to a memory location allocated by application where
 * UMD stores the graph ID
 * @param[in]  config Pointer to specific configuration struct
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_load_graph(const aipu_ctx_handle_t *ctx, const char *graph,
                              uint64_t *id,
                              aipu_load_graph_cfg_t *config = nullptr);

/**
 * @brief This API loads a graph with the form of AIPU executable graph binary
 * array.
 *
 * @param[in]  ctx   Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  graph_buf The start address of buffer which stores graph binary
 * data
 * @param[in]  graph_size The byte size of graph binary data in 'graph_buf'
 * @param[out] id    Pointer to a memory location allocated by application where
 * UMD stores the graph ID
 * @param[in]  config Pointer to specific configuration struct
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_load_graph_helper(const aipu_ctx_handle_t *ctx,
                                     const char *graph_buf, uint32_t graph_size,
                                     uint64_t *id,
                                     aipu_load_graph_cfg_t *config = nullptr);

/**
 * @brief This API loads an offline built and share weight AIPU binary from file
 * system.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  share_weight_graph File path of share weight file
 * @param[out] ids    Pointer to returned ids by driver, and length is @param
 * id_cnt return by driver
 * @param[out] id_cnt Num of ids returned by driver
 * @note Use 'aipu_unload_graph' to unload share weight graph one by one
 */
aipu_status_t aipu_load_share_weight_graph(
    const aipu_ctx_handle_t *ctx, const char *share_weight_graph,
    uint64_t **ids, uint32_t *id_cnt, aipu_load_graph_cfg_t *config = nullptr);

/**
 * @brief This API is used to unload a loaded graph
 *
 * @param[in] ctx   Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] graph Graph ID returned by aipu_load_graph
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_unload_graph(const aipu_ctx_handle_t *ctx, uint64_t graph);

/**
 * @brief This API is used to create a new job for a graph with provided buffer
 * handle.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  graph  Graph ID returned by aipu_load_graph
 * @param[out] job    Pointer to a memory location allocated by application
 * where UMD stores the new created job ID
 * @param[in]  config Specify job's partition id and QoS level, only for aipu
 * v3. specify memory region for feature map and weight buffer
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note The application can create one or multiple jobs by calling this API one
 * or multiple times.
 * @note The application can schedule one created job one or multiple times by
 * calling aipu_finish_job/aipu_flush_job, and at last clean it by calling
 * aipu_clean_job.
 * @note Through 'config' parameter, the feature map and weight buffer can be
 * allocated from specific memory regions. The allocate order is:
 * DTCM->SRAM->DDR. For example: if if intend to allocate feature map buffer
 * from DTCM, it will try to allocate from DTCM, if there's no enough free
 * space, it tries to check SRAM, then DDR until fail.
 */
aipu_status_t aipu_create_job(const aipu_ctx_handle_t *ctx, uint64_t graph,
                              uint64_t *job,
                              aipu_create_job_cfg_t *config = nullptr);

/**
 * @brief This API is used to flush a new computation job onto AIPU (blocking)
 *
 * @param[in] ctx      Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job      Job ID returned by aipu_create_job
 * @param[in] time_out Time out (in millisecond) specified by application for
 * this job (A timeout of value <= 0 means an infinite timeout.)
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_finish_job(const aipu_ctx_handle_t *ctx, uint64_t job,
                              int32_t time_out);

/**
 * @brief This API is used to flush a new computation job onto AIPU
 * (non-blocking)
 *
 * @param[in] ctx      Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job      Job ID returned by aipu_create_job
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note A flushed job cannot be flushed again before the previous scheduled one
 * is done.
 */
aipu_status_t aipu_flush_job(const aipu_ctx_handle_t *ctx, uint64_t job,
                             aipu_job_callback_func_t cb_func = nullptr);

/**
 * @brief This API is used to get the execution status of a flushed job
 * (non-blocking)
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  job    Job ID returned by aipu_create_job
 * @param[out] status Pointer to a memory location allocated by the application
 * where UMD stores the job status AIPU_JOB_STATUS_DONE: job is normally done
 *                    AIPU_JOB_STATUS_EXCEPTION: exception occurring on this job
 *                    AIPU_JOB_STATUS_NO_STATUS: job is in handling
 * @param[in]  timeout timeout value(ms) to poll job's status
 *                     timeout > 0: the max polling time window is 'timeout'
 *                     timeout = 0: non-blocking and return job's status
 * immediately. timeout = -1: blocking until job is really done or exception.
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note This API should be used by the application after aipu_flush_job
 * successfully returns.
 */
aipu_status_t aipu_get_job_status(const aipu_ctx_handle_t *ctx, uint64_t job,
                                  aipu_job_status_t *status,
                                  int32_t timeout = 0);

/**
 * @brief This API is used to clean a finished job object scheduled by
 * aipu_finish_job/aipu_flush_job
 *
 * @param[in] ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job Job ID returned by aipu_create_job
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_clean_job(const aipu_ctx_handle_t *ctx, uint64_t job);

/**
 * @brief This API is used to get tensor count of specified type
 *
 * @param[in]  ctx  Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  id   Job ID returned by aipu_create_job, or graph ID returned by
 * aipu_load_graph
 * @param[in]  type Tensor type
 * @param[out] cnt  Pointer to a memory location allocated by application where
 * UMD stores the count
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_tensor_count(const aipu_ctx_handle_t *ctx, uint64_t id,
                                    aipu_tensor_type_t type, uint32_t *cnt);

/**
 * @brief This API is used to get tensor's descriptor of specified type
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  id     Job ID returned by aipu_create_job, or graph ID returned
 * by aipu_load_graph
 * @param[in]  type   Tensor type
 * @param[in]  tensor Tensor ID
 * @param[out] desc   Pointer to a memory location allocated by application
 * where UMD stores the tensor descriptor
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_tensor_descriptor(const aipu_ctx_handle_t *ctx,
                                         uint64_t id, aipu_tensor_type_t type,
                                         uint32_t tensor,
                                         aipu_tensor_desc_t *desc);

/**
 * @brief This API is used to load input tensor data
 *
 * @param[in] ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job    Job ID returned by aipu_create_job
 * @param[in] tensor Input tensor ID
 * @param[in] data   Data of input tensor
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_load_tensor(const aipu_ctx_handle_t *ctx, uint64_t job,
                               uint32_t tensor, const void *data);

/**
 * @brief This API is used to get tensor data of specified type
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  job    Job ID returned by aipu_create_job
 * @param[in]  type   Tensor type
 * @param[in]  tensor Input tensor ID
 * @param[out] data   Data of output tensor
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_tensor(const aipu_ctx_handle_t *ctx, uint64_t job,
                              aipu_tensor_type_t type, uint32_t tensor,
                              void *data);

/**
 * @brief This API is used to configure a specified option of a job.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  job    Job ID returned by aipu_create_job
 * @param[in]  types  Configuration type(s)
 * @param[out] config Pointer to a memory location allocated by application
 * where application stores the configuration data struct
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note accepted types/config:
 * AIPU_JOB_CONFIG_TYPE_DUMP_[*]/aipu_job_config_dump_t
 * @note accepted types/config:
 * AIPU_JOB_CONFIG_TYPE_DYNAMIC_PARAMS/aipu_dynshape_param_t
 * @note accepted types/config:
 * AIPU_CONFIG_TYPE_SIMULATION/aipu_job_config_simulation_t
 */
aipu_status_t aipu_config_job(const aipu_ctx_handle_t *ctx, uint64_t job,
                              uint64_t types, void *config);

/**
 * @brief This API is used to specify a shared buffer as job's io buffer.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  job    Job ID returned by aipu_create_job
 * @param[in]  shared_buf Pointer to shared buffer allcated through dma_buf
 * system
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note this API is only used on HW
 * @note please set `disable_input_buffer_reuse` and
 * `disable_output_buffer_reuse` in compiling phase
 */
aipu_status_t aipu_specify_iobuf(const aipu_ctx_handle_t *ctx, uint64_t job_id,
                                 aipu_shared_tensor_info_t *shared_buf);

/**
 * @brief This API is used to get AIPU cluster count.
 *
 * @param[in]  ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[out] cnt Pointer to a memory location allocated by application where
 * UMD stores the partition count
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_partition_count(const aipu_ctx_handle_t *ctx,
                                       uint32_t *cnt);

/**
 * @brief This API is used to get AIPU cluster count.
 *
 * @param[in]  ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  partition_id Which partition it get cluster count from
 * @param[out] cnt Pointer to a memory location allocated by application where
 * UMD stores the cluster count
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_cluster_count(const aipu_ctx_handle_t *ctx,
                                     uint32_t partition_id, uint32_t *cnt);

/**
 * @brief This API is used to get AIPU core count in a specific cluster.
 *
 * @param[in]  ctx     Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  partition_id Partition ID
 * @param[in]  cluster Cluster ID
 * @param[out] cnt     Pointer to a memory location allocated by application
 * where UMD stores the core count
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note Cluster ID is numbered within [0, cluster_cnt).
 */
aipu_status_t aipu_get_core_count(const aipu_ctx_handle_t *ctx,
                                  uint32_t partition_id, uint32_t cluster,
                                  uint32_t *cnt);

/**
 * @brief This API is used to get information of an AIPU core for debugger to
 use
 *
 * @param[in]  ctx     Pointer to a context handle struct returned by
 AIPU_init_ctx
 * @param[in]  id      core_id for aipu v1/v2, partition_id for aipu v3
 * @param[out] info    Pointer to a memory location allocated by application
 where UMD stores
 *                     the core information
 *
 * @retval AIPU_STATUS_SUCCESS if successful

 * @note The core or partition ID should be less than core count returned by
 aipu_get_core_count.
 */
aipu_status_t aipu_debugger_get_core_info(const aipu_ctx_handle_t *ctx,
                                          uint32_t id, aipu_core_info_t *info);

/**
 * @brief This API is used by debugger to get information of a job
 *
 * @param[in]  ctx  Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  job  Job ID
 * @param[out] info Pointer to a memory location allocated by application where
 * UMD stores a pointer to the job information
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_debugger_get_job_info(const aipu_ctx_handle_t *ctx,
                                         uint64_t job,
                                         aipu_debugger_job_info_t *info);

/**
 * @brief This API bind a created job to an idle AIPU core for execution later;
 *        External registers of the specified AIPU core is writen after this API
 returns,
 *        but the start PC register is not triggerred to run.
 *
 * @param[in] ctx     Pointer to a context handle struct returned by
 aipu_init_context
 * @param[in] job_id  Job ID returned by aipu_create_job
 * @param[in] id      ID of an AIPU core or ID of a AIPU partition
 *
 * @retval AIPU_STATUS_SUCCESS if successful

 * @note For the same core or job, it should only be bind once, unless the job
 is done.
 * @note The core or partition to be bind should be in idle state; otherwise UMD
 returns error code
 */
aipu_status_t aipu_debugger_bind_job(const aipu_ctx_handle_t *ctx, uint32_t id,
                                     uint64_t job_id);

/**
 * @brief This API trigger a previously bind job to run on a target AIPU core;
 *        This API is a blocking API which returns after the job execution ends
 * on hardware.
 *
 * @param[in] ctx     Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job_id  Job ID returned by aipu_create_job and bind by
 * aipu_debugger_bind_job
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_debugger_run_job(const aipu_ctx_handle_t *ctx,
                                    uint64_t job_id);

/**
 * @brief This API allocates a buffer for AIPU debugger's usage
 *
 * @param[in] ctx     Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] size    Size of the requested buffer in byte
 * @param[out] va     Pointer to a virtual address stores the base address of
 * the buffer
 * @param[in] job_id  Specify job id
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note This API shall be used after aipu_load_graph and before calling
 * aipu_debugger_bind_job.
 * @note When this API is used in multi-core scenario, only the core under
 * debugging should be used to schedule a debugger job (in a single user
 * thread), and other cores should keep in idle state.
 */
aipu_status_t aipu_debugger_malloc(const aipu_ctx_handle_t *ctx, uint32_t size,
                                   void **va, uint64_t job_id = 0);

/**
 * @brief This API frees a buffer allocated by aipu_debugger_malloc
 *
 * @param[in] ctx  Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] va   Virtual address to free
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_debugger_free(const aipu_ctx_handle_t *ctx, void *va);

/**
 * @brief this API print AIPU execution log information after corresponding job
 * ends
 *
 * @param[in] printf_base   Pointer to a tensor buffer where stores the printf
 * log data
 * @param[in] redirect_file File path to store printf data
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note Application should get the printf log data via aipu_get_tensor before
 * print it. Only support aipu v1/v2.
 */
aipu_status_t aipu_printf(char *printf_base, char *redirect_file);

/**
 * @brief This API is used to get NPU's ARCH information.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[out] target Pointer to a memory location allocated by application
 * where UMD stores HW ARCH information
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note Simulator should not use this API, because simulator has specified
 * target at beginning.
 */
aipu_status_t aipu_get_target(const aipu_ctx_handle_t *ctx, char *target);

/**
 * @brief This API is used to get NPU status.
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[out] status Pointer to a memory location allocated by application
 * where UMD stores NPU status information
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_device_status(const aipu_ctx_handle_t *ctx,
                                     device_status_t *status);

/**
 * @brief This API is used to create a batch queue.
 *
 * @param[in] ctx      Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] graph_id Graph id
 * @param[out] queue_id Pointer to store batch queue id
 *
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note Each thread is subject to call this function to get private batch queue
 * ID. It isn't allowed to share batch queue ID between threads.
 */
aipu_status_t aipu_create_batch_queue(const aipu_ctx_handle_t *ctx,
                                      uint64_t graph_id, uint32_t *queue_id);

/**
 * @brief This API is used to clean a specific batch queue.
 *
 * @param[in] ctx      Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] graph_id Graph id
 * @param[in] queue_id Pointer to store batch queue id
 *
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_clean_batch_queue(const aipu_ctx_handle_t *ctx,
                                     uint64_t graph_id, uint32_t queue_id);

/**
 * @brief This API is used to do basic config for batch inference on simulator
 * and HW.
 *
 * @param[in] ctx      Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] graph_id Graph id
 * @param[in] queue_id Batch queue id
 * @param[in] types    Dump options for each batch job
 * @param[in] dump_cfg The root path to store dump files of each batch
 *
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note set dump options(AIPU_JOB_CONFIG_TYPE_DUMP_TEXT ...)
 */
aipu_status_t aipu_config_batch_dump(const aipu_ctx_handle_t *ctx,
                                     uint64_t graph_id, uint32_t queue_id,
                                     uint64_t types,
                                     aipu_job_config_dump_t *dump_cfg);

/**
 * @brief This API is used to add a group buffers of one frame inference to
 * batch queue.
 *
 * @param[in] ctx        Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] graph_id   Graph id
 * @param[in] queue_id   Batch queue id
 * @param[in] inputs     Buffer pointers for input tensors
 * @param[in] outputs    Buffer pointers for output tensors
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_add_batch(const aipu_ctx_handle_t *ctx, uint64_t graph_id,
                             uint32_t queue_id, char *inputs[],
                             char *outputs[]);

/**
 * @brief This API is used to run multiple batch inference.
 *
 * @param[in] ctx      Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] graph_id Graph id
 * @param[in] queue_id Batch queue id
 * @param[in] create_cfg Config for all batches in one queue
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_finish_batch(const aipu_ctx_handle_t *ctx, uint64_t graph_id,
                                uint32_t queue_id,
                                aipu_create_job_cfg_t *create_cfg);

/**
 * @brief This API is used to send specific command to NPU driver.
 *
 * @param[in] ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] cmd command in @note part
 * @param[inout] arg input or output argument according to 'cmd'
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 *
 * @note support commands currently
 *       AIPU_IOCTL_SET_PROFILE:
 *           dynamically enable/disable profiling feature of aipu v3 simulation.
 *           arg: pointer of int
 *            - 1: enable profiling
 *            - 0: disable profiling
 *       AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION:
 *           get model binary's build version.
 *           arg: pointer of struct `aipu_bin_buildversion_t`
 *       AIPU_IOCTL_GET_DS_NUM: [[deprecate]]
 *           get dynamic shape input number of provided graph id (acturally
 * return all inputs number) arg: pointer of struct `aipu_dynshape_num_t`
 *       AIPU_IOCTL_GET_DS_DIM_NUM: [[deprecate]]
 *           get rank of specified graph id, input idx
 *           arg: pointer of struct `aipu_dynshape_dim_num_t`
 *       AIPU_IOCTL_GET_DS_INFO: [[deprecate]]
 *           get min/max dimensions of specified graph id, input idx and min/max
 * flag arg: pointer of struct `aipu_dynshape_info_t` AIPU_IOCTL_GET_DS_RANK:
 *           get rank of specified input of graph id
 *           arg: pointer of struct `aipu_dynshape_rank_t`
 *       AIPU_IOCTL_GET_DS_DIM_CONSTRAINT:
 *           get min/max dimensions of specified input of graph id
 *           arg: pointer of struct `aipu_dynshape_dimension_t`
 *       AIPU_IOCTL_SET_DYNAMIC_ASID1:
 *           only for v3, default is false
 *           arg: pointer of bool
 *            - false: multi-model parallel, and asid0/1 will share same 3G
 * memory space
 *            - true: only one model dispatched on NPU at the same time
 *       AIPU_IOCTL_ALLOC_SHARE_BUF:
 *           request a shared buffer.
 *           arg: pointer of struct `aipu_share_buf_t`
 *       AIPU_IOCTL_FREE_SHARE_BUF:
 *           free a shared buffer.
 *           arg: pointer of struct `aipu_share_buf_t`
 *       AIPU_IOCTL_ALLOC_DMABUF
 *           request dma_buf from KMD.
 *           arg: pointer of struct `aipu_dma_buf_req_t`
 *            - bytes: request size (filled by user)
 *            - fd: fd corresponding to dma_buf (filled by KMD)
 *       AIPU_IOCTL_FREE_DMABUF
 *           free a dma_buf with its fd.
 *           arg: pointer of int which is a dmabuf fd returned by
 * `AIPU_IOCTL_ALLOC_DMABUF` AIPU_IOCTL_GET_DMABUF_INFO: get dma_buf infomation,
 * which includes pa and bytes by provide dma_buf fd arg: pointer of struct
 * `aipu_dma_buf_desc_t` AIPU_IOCTL_WRITE_DMABUF write a existing buffer to the
 * dma_buf allocated via `AIPU_IOCTL_ALLOC_DMABUF`. arg: pointer of struct
 * `aipu_dmabuf_op_t` AIPU_IOCTL_READ_DMABUF provide a buffer to read data from
 * then dma_buf allocated via 'AIPU_IOCTL_ALLOC_DMABUF'. arg: pointer of struct
 * `aipu_dmabuf_op_t` AIPU_IOCTL_ATTACH_DMABUF attach a dma_buf's fd exported
 * from other modules, it will return 'pa' and 'bytes', which can use to bind io
 * buffer by calling 'aipu_specify_iobuf' arg: pointer of struct
 * `aipu_dma_buf_desc_t` AIPU_IOCTL_DETACH_DMABUF detatch a dma_buf based on its
 * fd arg: pointer of int which is dmabuf fd exported from other modules
 *       AIPU_IOCTL_GET_VERSION
 *           get UMD and KMD release version.
 *           arg: pointer of struct `aipu_driver_version_t`
 *       AIPU_IOCTL_ABORT_CMDPOOL:
 *           abort hw command pool manually, v3_2 command pool will be destroyed
 * when all jobs done, then if you try to abort command pool, it will return
 * fail, only for >=v3 arg: no AIPU_IOCTL_ENABLE_TICKCOUNTER: enable performance
 * counter arg: no AIPU_IOCTL_DISABLE_TICKCOUNTER: disable performance counter
 *           arg: no
 *       AIPU_IOCTL_RW_REGISTER:
 *           read/write register group according different `aipu_ioctl_rw_t`'s
 * memory association type arg: pointer of `aipu_reg_rw_t` AIPU_IOCTL_RW_MEMORY:
 *           read/write memory according different `aipu_ioctl_rw_t`'s memory
 * association type arg: pointer of `aipu_mem_rw_t`
 */
aipu_status_t aipu_ioctl(aipu_ctx_handle_t *ctx, uint32_t cmd,
                         void *arg = nullptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _STANDARD_API_H_ */
