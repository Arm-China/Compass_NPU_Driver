// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  graph.h
 * @brief AIPU User Mode Driver (UMD) graph module header
 */

#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <sys/mman.h>

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <vector>

#include "graph_base.h"
#include "parser_base.h"
#include "standard_api.h"

namespace aipudrv {
enum GraphRemapLoadType {
  PARAM_MAP_LOAD_TYPE_REUSE,
  PARAM_MAP_LOAD_TYPE_STATIC,
};

struct GraphSubSectionDesc {
  uint32_t offset_in_section; /**< offset in a section where this subsection
                                 based in */
};

struct GraphSectionDesc {
  uint32_t size; /**< section data size */
  uint32_t align_bytes;
  uint32_t
      align_in_page;   /**< section assress alignment requirement (in page) */
  uint32_t type;       /**< weight const or zerocpy_const(15) */
  uint32_t slot_index; /**< index in each static/reuse section */
  uint32_t
      src_offset; /*< compact and provided source data offset to each bss */
  uint64_t
      src_offset_in_share_weight; /*< [only for shared weight].source offset to
                                     merged_weight.bin beginning */
  uint32_t dst_offset;            /*< aligned dst data offset to each bss */

  std::array<uint8_t, 32> hashcode{0}; /**< hashcode for each static section */

  std::vector<GraphSubSectionDesc>
      sub_sections; /**< sub-section(s) in this section */

  void init() /**< section initializer */
  {
    size = 0;
    align_bytes = 0;
    align_in_page = 1;
    type = 0;
    slot_index = 0;
    src_offset = 0;
    src_offset_in_share_weight = 0;
    dst_offset = 0;
    sub_sections.clear();
  }
};

struct GraphIOTensors {
  std::vector<GraphIOTensorDesc> inputs;
  std::vector<GraphIOTensorDesc> outputs;
  std::vector<GraphIOTensorDesc> inter_dumps;
  std::vector<GraphIOTensorDesc> profiler;
  std::vector<GraphIOTensorDesc> printf;
  std::vector<GraphIOTensorDesc> layer_counter;
  std::vector<GraphIOTensorDesc> err_code;
  std::vector<GraphIOTensorDesc> segmmus;
  std::vector<GraphIOTensorDesc> outputs_shape;
};

struct GraphParamMapLoadDesc {
  uint32_t offset_in_map; /**< parameter load offset in rodata parameter map */
  uint32_t load_type;     /**< data type: reuse/static */
  uint32_t buf_type;      /**< buffer type: input/output/segmmu */
  uint32_t ref_section_iter; /**< referenced section iterator */
  uint32_t ref_sub_section_iter;
  uint32_t sub_section_offset; /**< subsection offset in its section */
  uint32_t addr_mask;
  void init(uint32_t offset, uint32_t sec_type, uint32_t _buf_type,
            uint32_t sec_iter, uint32_t sub_sec_iter, uint32_t sub_sec_offset,
            uint32_t mask) {
    offset_in_map = offset;
    load_type = sec_type;
    buf_type = _buf_type;
    ref_section_iter = sec_iter;
    ref_sub_section_iter = sub_sec_iter;
    sub_section_offset = sub_sec_offset;
    addr_mask = mask;
  }
};

struct WeightBufferInfo {
  /* only affect weight in a gathered buffer case */
  BufferDesc *wb_weight = nullptr;
  BufferDesc *wb_zerocpy_const = nullptr;

  /* describe each static section, scattered has independent malloced buffer */
  std::vector<BufferDesc *> wb_weights;

  /* weight buffer ASID base address */
  DEV_PA_64 wb_asid_base = 0;
};

struct ConstantHashItem {
  uint32_t offset;
  uint32_t size;
  std::array<uint8_t, 32> hashcode{};
};

/* 1.union has trivial copy constructor issue, 2.std::variant(c++17) */
struct WeightSection {
  std::vector<BinSection> weight; /* size=0: no weight or shared weight */
  struct ExtraWeightInfo {
    std::string file;
    std::string hash;
  };
  std::vector<ExtraWeightInfo> extra_weight_infos;
  std::string extra_weight_path = "./";
};

struct Block {
  std::string file;
  uint32_t
      offset_to_file; /* offset to beggining of aipu.bin or extra_weight.bin */
  uint32_t offset_to_weight; /* offset to beggining of weight.bin */
  uint32_t size;
};

using HASH_SECTION_TABLE =
    std::map<uint32_t, std::map<std::array<uint8_t, 32>, GraphSectionDesc>>;
using BLOCKS_TABLE = std::array<std::map<uint32_t, std::shared_ptr<Block>>,
                                2>; /* [start, end] section id */

class ParserBase;
class SharedWeightMgr;
class Graph : public GraphBase {
private:
  uint32_t m_zerocpy_const_size = 0;
  uint32_t m_const_size = 0;
  std::string m_bin_target;
  static constexpr uint32_t MAX_BLOCK_SIZE = 0x4000000; /* 64M */
  // BLOCKS_TABLE m_blk_table;

protected:
  ParserBase *m_parser = nullptr;
  SharedWeightMgr *m_sw_mgr = nullptr;

  /* section descriptions in the graph binary */
  BinSection m_btext;
  BinSection m_bcrodata;
  BinSection m_brodata;
  BinSection m_bdesc;
  BinSection m_bdata;
  std::vector<RemapEntry> m_remap;
  /* dynamic shape */
  BinSection m_bglobalparam;
  WeightSection m_bweight;

protected:
  /* Buffers in memory for AIPU's access */
  BufferDesc *m_text = nullptr;
  BufferDesc *m_crodata = nullptr;
  std::vector<WeightBufferInfo> m_weight;

  std::vector<std::vector<ConstantHashItem>> m_hashtable;
  std::vector<uint64_t> m_offsets;
  std::map<uint32_t, uint32_t>
      m_const_base_offset; /* for each bss const base offset */
  std::map<uint32_t, uint32_t>
      m_zcy_const_base_offset; /* for each bss zcy const base offset  */
  bool m_is_shared_weight = false;
  bool m_do_vcheck = true;

  /* DTCM size, KB unit */
  int m_dtcm_size = 0;

public:
  /* entry: <min shape (N, H, W, C), max shape (N, H, W, C)> etc */
  std::map<int, std::array<std::vector<uint32_t>, 2>> m_input_shape_constraint;

  /* entry: <min size, max size>, size = N*H*W*C */
  std::map<int, std::array<uint64_t, 2>> m_input_shape_threshhold;

  bool m_dynamic_shape = false;
  bool m_dynamic_asid0 = false;

  bool m_put_weight_gm = false;
  bool m_put_desc_gm = false;
  bool m_put_ws_gm = false;

private:
  aipu_status_t load_config(const aipu_load_graph_cfg_t *config);
  aipu_status_t load_common(bool ver_check);

  std::string version_to_target(uint32_t arch, uint32_t hw_version,
                                uint32_t hw_config, uint32_t hw_revision);
  BLOCKS_TABLE slice_weight(uint32_t bss_id);

protected:
  virtual const std::string get_target() const { return ""; };

public:
  int32_t get_dynamic_shape_dim_num(uint32_t idx,
                                    bool max_shape_dim) const override;
  bool get_dynamic_shape_data(uint32_t idx, bool max_shape_dim,
                              uint32_t *data) const override;
  virtual aipu_status_t update_dynamic_io_tensor_size(aipu_tensor_type_t type) {
    return AIPU_STATUS_SUCCESS;
  }

  virtual aipu_data_type_t get_io_tensor_type(int idx) const {
    return AIPU_DATA_TYPE_S8;
  }

public:
  virtual void set_stack(uint32_t bss_id, uint32_t size, uint32_t align) = 0;
  virtual void add_param(uint32_t bss_id, GraphParamMapLoadDesc param) = 0;
  virtual void add_static_section(uint32_t bss_id,
                                  GraphSectionDesc section) = 0;
  virtual void add_reuse_section(uint32_t bss_id, GraphSectionDesc section) = 0;
  virtual void set_io_tensors(uint32_t bss_id, GraphIOTensors io) = 0;
  virtual void set_graph_comment(const char *data, uint64_t size){};
  virtual void set_gmconfig(BinSection &gm_section) {}
  virtual void set_segmmu(BinSection &segmmu_section) {}
  virtual void set_graphjson(BinSection &graphjson_section) {}
  virtual aipu_status_t parse_gmconfig(int bss_id) {
    return AIPU_STATUS_SUCCESS;
  }
  virtual uint32_t get_alloc_pad_size() const { return 0; }
  virtual uint32_t get_asid_align_page() const { return 0; }
  virtual std::vector<GraphSectionDesc> &
  get_static_section_ref(uint32_t bss_id) = 0;
  virtual HASH_SECTION_TABLE &get_shared_section_ref(uint32_t) {
    static HASH_SECTION_TABLE table = {};
    return table;
  };
  virtual GraphIOTensors &get_bss_io_ref(uint32_t bss_id) = 0;
  aipu_status_t set_constant_hashtable(const BinSection &table);

  aipu_status_t set_static_section_param(uint32_t bss_id,
                                         const BSSStaticSectionDesc &bss_sec,
                                         GraphSectionDesc &section,
                                         uint32_t &const_addr,
                                         uint32_t &zcy_addr);

public:
  aipu_status_t load(std::istream &gbin, uint32_t size, bool ver_check = true,
                     const aipu_load_graph_cfg_t *config = nullptr) override;
  aipu_status_t load(const char *file, bool ver_check = true,
                     const aipu_load_graph_cfg_t *config = nullptr) override;
  aipu_status_t unload() override;
  aipu_status_t alloc_weight_buffer() override;
  aipu_status_t setup_weight_buffer(std::vector<WeightBufferInfo> &weights,
                                    bool setup_zcy = true);
  aipu_status_t setup_zcy_buffer(std::vector<WeightBufferInfo> &weights);

  virtual void print_parse_info() = 0;
  virtual aipu_status_t create_job(JOB_ID *id,
                                   const aipu_global_config_simulation_t *cfg,
                                   aipu_global_config_hw_t *hw_cfg,
                                   aipu_create_job_cfg_t *config = nullptr) = 0;
  virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type,
                                         uint32_t *cnt) const = 0;
  virtual aipu_status_t
  get_tensor_descriptor(aipu_tensor_type_t type, uint32_t tensor,
                        aipu_tensor_desc_t *desc) const = 0;

public:
  /* Set functions */
  void set_shared_weight_mgr(SharedWeightMgr *mgr) {
    m_sw_mgr = mgr;
    m_is_shared_weight = true;
  }
  void set_parser(ParserBase *parser) { m_parser = parser; }
  void set_mapped_gfile(const std::string &file) { m_mapped_gfile = file; }
  void set_graph_text(const char *data, uint64_t size) {
    m_btext.va = data;
    m_btext.size = size;
  }
  void set_graph_crodata(const char *data, uint64_t size) {
    m_bcrodata.va = data;
    m_bcrodata.size = size;
  }
  void set_graph_dp(const char *data, uint64_t size) {
    m_bdata.va = data;
    m_bdata.size = size;
  }
  void set_graph_rodata(const BinSection &rodata) { m_brodata = rodata; }
  void set_graph_desc(const BinSection &desc) { m_bdesc = desc; }
  const BinSection &get_graph_desc() { return m_bdesc; }
  void set_graph_weight(const BinSection &weight) {
    m_bweight.weight.push_back(weight);
  }

  aipu_status_t set_graph_extra_weight(const BinSection &extra_weight);

  void add_remap(RemapEntry remap) { m_remap.push_back(remap); }
  void set_dtcm_size(int dtcm_sz) { m_dtcm_size = dtcm_sz; }

  void
  set_weight_buffer_info(const std::vector<WeightBufferInfo> &weight_buf_info) {
    m_weight = weight_buf_info;
  }

  std::vector<WeightBufferInfo> &get_weight_buffer_info() { return m_weight; }

  virtual uint32_t get_bss_cnt() const { return 1; }

  bool has_weight(uint32_t bss_id = 0) const {
    bool exclusive =
        m_bweight.weight.size() > bss_id && m_bweight.weight[bss_id].size != 0;
    /* bool shared = m_weight[bss_id].is_shared_weight &&
     * m_weight[bss_id].wb_weight != nullptr; */
    return exclusive;
  }

  virtual void set_const_size(uint32_t bss_id, uint32_t const_size,
                              uint32_t zerocpy_const_size) {
    if (bss_id > 0)
      return;

    /**
     * if one graph doesn't need weight, it just reserves
     * 4KB as default placehold for whole flow.
     */
    if (const_size == 0)
      const_size = 4096;

    m_const_size = const_size;
    m_zerocpy_const_size = zerocpy_const_size;
  }

  virtual uint32_t get_zerocpy_const_size(uint32_t bss_id = 0) const {
    if (bss_id == 0)
      return m_zerocpy_const_size;
    return 0;
  }

  virtual uint32_t get_const_size(uint32_t bss_id = 0) const {
    if (bss_id == 0)
      return m_const_size;
    return 0;
  }

  std::vector<std::vector<ConstantHashItem>> &get_hashtable() {
    return m_hashtable;
  }

  void set_modle_global_param(BinSection mgp_section) {
    uint32_t input_shape_offset = *(uint32_t *)mgp_section.va;

    if (input_shape_offset >= mgp_section.size) {
      m_dynamic_shape = false;
      LOG(LOG_WARN, "ModelGlobalParam input_shape_offset [invalid]");
      return;
    }

    m_bglobalparam = mgp_section;
    m_dynamic_shape = true;
  }

#define GET_U32_FROM_PTR_ADV(ptr) (*(uint32_t *)ptr++)

  bool set_input_shape_constrait(BinSection &isc_section) {
    uint32_t *start = (uint32_t *)isc_section.va;
    uint32_t constraint_num = GET_U32_FROM_PTR_ADV(start);

    LOG(LOG_DEBUG, "dynamic inputs number: %u", constraint_num / 2);
    for (uint32_t i = 0; i < constraint_num; i++) {
      uint32_t rank = GET_U32_FROM_PTR_ADV(start);
      std::vector<uint32_t> shape_vec;
      uint32_t input_idx = i / 2;

      LOG(LOG_DEBUG, "input idx: %u, rank: %u", input_idx, rank);
      for (uint32_t j = 0; j < rank; j++) {
        shape_vec.push_back(GET_U32_FROM_PTR_ADV(start));
        LOG(LOG_DEBUG, " - %s[%u]: %u", (i % 2 == 0 ? "min" : "max"), j,
            shape_vec[j]);
      }

      if (shape_vec.size() > 0) {
        uint64_t size = 1;

        for (uint32_t k = 0; k < shape_vec.size(); k++)
          size *= shape_vec[k];
        m_input_shape_constraint[input_idx][i % 2] = std::move(shape_vec);
        m_input_shape_threshhold[input_idx][i % 2] = size;
      }
    }

    return true;
  }

  bool is_dynamic_shape() const override { return m_dynamic_shape; }

  bool is_dynamic_asid0() const { return m_dynamic_asid0; }

  uint32_t get_dynamic_shape_num() const {
    if (!is_dynamic_shape())
      return 0;
    else
      return m_input_shape_constraint.size();
  }

  virtual void set_enrty(uint32_t offset){};

  uint32_t bin_to_dev_isa() {
    struct ISA {
      ISAType type;
      uint32_t rev;
    };

    /* <dev_isa, bin_isa> */
    static const std::map<uint32_t, ISA> table = {
        {AIPU_ISA_VERSION_ZHOUYI_V1, {ISAType::ISAv1, 0}},
        {AIPU_ISA_VERSION_ZHOUYI_V2_0, {ISAType::ISAv2, 0}},
        {AIPU_ISA_VERSION_ZHOUYI_V2_1, {ISAType::ISAv3, 0}},
        {AIPU_ISA_VERSION_ZHOUYI_V2_2, {ISAType::ISAv31, 0}},
        {AIPU_ISA_VERSION_ZHOUYI_V3, {ISAType::ISAv5, 0}},
        // {AIPU_ISA_VERSION_ZHOUYI_V3_1, {ISAType::ISAv6, 0}},
        {AIPU_ISA_VERSION_ZHOUYI_V3_2_0, {ISAType::ISAv6, 1}},
        {AIPU_ISA_VERSION_ZHOUYI_V3_2_1, {ISAType::ISAv6, 2}},
    };

    for (const auto &it : table) {
      if (it.second.type == m_isa && it.second.rev == m_hw_revision)
        return it.first;
    }
    return 0;
  }

public:
  Graph(void *ctx, GRAPH_ID id, DeviceBase *dev);
  virtual ~Graph();
  Graph(const Graph &graph) = delete;
  Graph &operator=(const Graph &graph) = delete;

  friend class JobBase;
  friend class JobV3X;
  friend class Coredump;
};
} // namespace aipudrv

#endif /* _GRAPH_H_ */
