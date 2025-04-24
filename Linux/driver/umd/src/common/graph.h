// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
#include <vector>

#include "graph_base.h"
#include "parser_base.h"
#include "share_weight_mgr.h"
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
  uint32_t
      align_in_page;   /**< section assress alignment requirement (in page) */
  uint32_t type;       /**< weight const or zerocpy_const(15) */
  uint32_t slot_index; /**< index in each static/reuse section */
  uint32_t
      src_offset; /*< compact and provided source data offset to each bss */
  uint64_t
      src_offset_in_share_weight; /*< [shared weight].source offset to
                                     merged_weight.bin instead of zip file */
  uint32_t dst_offset;            /*< aligned dst data offset to each bss */

  std::array<uint8_t, 32> hashcode{
      0}; /**< [shared weight].shared data hashcode */

  std::vector<GraphSubSectionDesc>
      sub_sections; /**< sub-section(s) in this section */

  void init() /**< section initializer */
  {
    size = 0;
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
  std::vector<struct GraphIOTensorDesc> inputs;
  std::vector<struct GraphIOTensorDesc> outputs;
  std::vector<struct GraphIOTensorDesc> inter_dumps;
  std::vector<struct GraphIOTensorDesc> profiler;
  std::vector<struct GraphIOTensorDesc> printf;
  std::vector<struct GraphIOTensorDesc> layer_counter;
  std::vector<struct GraphIOTensorDesc> err_code;
  std::vector<struct GraphIOTensorDesc> segmmus;
  std::vector<struct GraphIOTensorDesc> outputs_shape;
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

  /* wb_weights in gathered buffers only mark relative pa which is offset to
   * whole buffer */
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
  std::vector<struct BinSection>
      weight; /* size=0: no weight or shared weight */
  struct ExtraWeightInfo {
    std::string name;
    std::string hash;
  };
  std::vector<ExtraWeightInfo> extra_weight_infos;
  std::string extra_weight_path;
};

struct SharedWeightSection {
  FileSectionDesc weight; /* offset: offset of merged_weight.bin to zip file,
                             size: merged_weight size */
  std::vector<uint64_t> offsets; /* offset to merged_weight.bin */
  uint32_t offsets_start; /* bellow `vector<offsets>` is whole offset.bin at
                             beggining, then clip by `offsets_start` and
                             `hashtable.size` */
};

class ParserBase;

class Graph : public GraphBase {
private:
  uint32_t m_zerocpy_const_size = 0;
  uint32_t m_const_size = 0;

protected:
  ParserBase *m_parser = nullptr;
  /* section descriptions in the graph binary */
  struct BinSection m_btext;
  struct BinSection m_bcrodata;
  struct BinSection m_brodata;
  struct BinSection m_bdesc;
  struct BinSection m_bdata;
  std::vector<RemapEntry> m_remap;
  /* dynamic shape */
  struct BinSection m_bglobalparam;
  struct WeightSection m_bweight;
  struct SharedWeightSection m_bshared_weight;

protected:
  /* Buffers in memory for AIPU's access */
  BufferDesc *m_text = nullptr;
  BufferDesc *m_crodata = nullptr;
  std::vector<struct WeightBufferInfo> m_weight;

  std::vector<std::vector<ConstantHashItem>> m_hashtable;
  std::vector<std::uint32_t>
      m_const_base_offset; /* for each bss const base offset */
  std::vector<std::uint32_t>
      m_zcy_const_base_offset; /* for each bss zcy const base offset  */
  ShareWeightMgr *m_sw_mgr = nullptr;

  bool m_do_vcheck = true;

  /* DTCM size, KB unit */
  int m_dtcm_size = 0;

public:
  /* entry: <min shape (N, H, W, C), max shape (N, H, W, C)> etc */
  std::map<int, std::vector<std::vector<uint32_t>>> m_input_shape_constraint;

  /* entry: <min size, max size>, size = N*H*W*C */
  std::map<int, std::vector<uint64_t>> m_input_shape_threshhold;

  bool m_dynamic_shape = false;

private:
  aipu_status_t alloc_gathered_weight();
  aipu_status_t alloc_scattered_weight();

public:
  virtual int32_t get_dynamic_shape_dim_num(uint32_t idx, bool max_shape_dim);
  virtual bool get_dynamic_shape_data(uint32_t idx, bool max_shape_dim,
                                      uint32_t *data);
  virtual aipu_status_t update_dynamic_io_tensor_size(aipu_tensor_type_t type) {
    return AIPU_STATUS_SUCCESS;
  }

  virtual aipu_data_type_t get_io_tensor_type(int idx) const {
    return AIPU_DATA_TYPE_S8;
  }

public:
  virtual void set_stack(uint32_t bss_id, uint32_t size, uint32_t align) = 0;
  virtual void add_param(uint32_t bss_id,
                         struct GraphParamMapLoadDesc param) = 0;
  virtual void add_static_section(uint32_t bss_id,
                                  struct GraphSectionDesc section) = 0;
  virtual void add_reuse_section(uint32_t bss_id,
                                 struct GraphSectionDesc section) = 0;
  virtual void set_io_tensors(uint32_t bss_id, struct GraphIOTensors io) = 0;
  virtual void set_gmconfig(BinSection &gm_section) {}
  virtual void set_segmmu(BinSection &segmmu_section) {}
  virtual aipu_status_t extract_gm_info(int bss_id) {
    return AIPU_STATUS_SUCCESS;
  }
  virtual uint32_t get_alloc_pad_size() const { return 0; }
  virtual std::vector<struct GraphSectionDesc> &
  get_static_section_ref(uint32_t bss_id) = 0;
  virtual GraphIOTensors &get_bss_io_ref(uint32_t bss_id) = 0;
  aipu_status_t set_constant_hashtable(const BinSection &table);

  aipu_status_t
  set_static_section_param(uint32_t bss_id, const BSSStaticSectionDesc &bss_sec,
                           GraphSectionDesc &section, uint32_t &const_addr_orig,
                           uint32_t &zcy_addr_orig, uint32_t &const_addr,
                           uint32_t &zcy_addr);

  aipu_status_t write_static_section(uint32_t bss_id,
                                     const GraphSectionDesc *section,
                                     BufferDesc *desc, uint32_t cst_base = 0,
                                     uint32_t zcy_cst_abse = 0);

public:
  aipu_status_t load(std::istream &gbin, uint32_t size, bool ver_check = true,
                     aipu_load_graph_cfg_t *config = nullptr) override;
  aipu_status_t unload() override;
  aipu_status_t alloc_weight_buffer() override;
  aipu_status_t write_weight_buffer() override;

  virtual void print_parse_info() = 0;
  virtual aipu_status_t create_job(JOB_ID *id,
                                   const aipu_global_config_simulation_t *cfg,
                                   aipu_global_config_hw_t *hw_cfg,
                                   aipu_create_job_cfg_t *config = nullptr) = 0;
  virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type,
                                         uint32_t *cnt) = 0;
  virtual aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type,
                                              uint32_t tensor,
                                              aipu_tensor_desc_t *desc) = 0;

public:
  /* Set functions */
  void set_parser(ParserBase *parser) { m_parser = parser; }
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

  virtual uint32_t get_bss_cnt() { return 1; }

  bool is_exclusive_weight() const { return m_bweight.weight.size() > 0; }

  bool has_weight(uint32_t bss_id = 0) {
    bool exclusive =
        m_bweight.weight.size() > bss_id && m_bweight.weight[bss_id].size != 0;
    bool shared = m_bshared_weight.weight.size != 0 &&
                  get_static_section_ref(bss_id).size() != 0;
    return exclusive || shared;
  }

  void set_sw_mgr(ShareWeightMgr *sw_mgr) { m_sw_mgr = sw_mgr; }

  ShareWeightMgr *get_sw_mgr() { return m_sw_mgr; }

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

  virtual uint32_t get_zerocpy_const_size(uint32_t bss_id) {
    if (bss_id == 0)
      return m_zerocpy_const_size;
    return 0;
  }

  virtual uint32_t get_const_size(uint32_t bss_id) {
    if (bss_id == 0)
      return m_const_size;
    return 0;
  }

  uint32_t get_hashtable_items_cnt() {
    uint32_t eles = 0;
    std::for_each(m_hashtable.begin(), m_hashtable.end(),
                  [&](const std::vector<ConstantHashItem> &items) {
                    eles += items.size();
                  });
    return eles;
  }

  aipu_status_t set_shared_weight(const FileSectionDesc &desc) {
    m_bshared_weight.weight = desc;
    return AIPU_STATUS_SUCCESS;
  }

  aipu_status_t set_shared_offset(const FileSectionDesc &desc,
                                  uint32_t offsets_start) {
    char *offset = nullptr;
    aipu_status_t ret = umd_mmap_file_helper(
        desc.file.c_str(), (void **)&offset, desc.size, desc.offset);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    m_bshared_weight.offsets_start = offsets_start;
    for (uint32_t i = 0; i < desc.size; i += sizeof(uint64_t)) {
      m_bshared_weight.offsets.push_back(*(uint64_t *)offset);
      offset += sizeof(uint64_t);
    }
    munmap(offset, desc.size);

    return ret;
  }

  void clip_shared_offset() {
    auto &offsets = m_bshared_weight.offsets;
    if (m_hashtable.size() == 0 || offsets.size() == 0)
      return;

    offsets.erase(offsets.begin(),
                  offsets.begin() +
                      m_bshared_weight.offsets_start / sizeof(uint64_t));
    offsets.erase(offsets.begin() + get_hashtable_items_cnt(), offsets.end());
  }

  /* shared weight: each bss has an base destination offset */
  void set_bss_base_offset(uint32_t base_offset) {
    m_const_base_offset.push_back(base_offset);
  }

  uint32_t get_bss_base_offset(uint32_t bss_id) const {
    if (bss_id >= m_const_base_offset.size())
      return 0;
    return m_const_base_offset[bss_id];
  }

  void set_bss_zcy_base_offset(uint32_t base_offset) {
    m_zcy_const_base_offset.push_back(base_offset);
  }

  uint32_t get_bss_zcy_base_offset(uint32_t bss_id) const {
    if (bss_id >= m_zcy_const_base_offset.size())
      return 0;
    return m_zcy_const_base_offset[bss_id];
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
    uint32_t num_inputs = GET_U32_FROM_PTR_ADV(start);

    for (uint32_t i = 0; i < num_inputs; i++) {
      uint32_t dim = GET_U32_FROM_PTR_ADV(start);
      std::vector<uint32_t> shape_vec;

      for (uint32_t j = 0; j < dim; j++)
        shape_vec.push_back(GET_U32_FROM_PTR_ADV(start));

      if (shape_vec.size() > 0) {
        uint64_t size = 1;

        m_input_shape_constraint[i / 2].push_back(shape_vec);
        for (uint32_t k = 0; k < shape_vec.size(); k++) {
          size *= shape_vec[k];

          if (shape_vec[k] == 0)
            LOG(LOG_INFO, "graph id: 0x%lx, input idx %d, %s dim %d is 0", m_id,
                i / 2, (i % 2 == 0 ? "min" : "max"), k);
        }

        m_input_shape_threshhold[i / 2].push_back(size);
      }
    }

    return true;
  }

  bool is_dynamic_shape() { return m_dynamic_shape; }

  uint32_t get_dynamic_shape_num() {
    if (!is_dynamic_shape())
      return 0;
    else
      return m_input_shape_constraint.size();
  }

  virtual void set_enrty(uint32_t offset){};

  virtual DEV_PA_64 debugger_get_instr_base() { return m_text->pa; }

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
