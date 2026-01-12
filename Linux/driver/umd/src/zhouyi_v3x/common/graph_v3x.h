// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  graph_v3x.h
 * @brief AIPU User Mode Driver (UMD) aipu v3x graph module header
 */

#ifndef _GRAPH_V3_H_
#define _GRAPH_V3_H_

#include <string.h>

#include <map>
#include <vector>

#include "graph.h"

namespace aipudrv {
enum GMBufType {
  GM_BUF_TYPE_REUSE = 0,
  GM_BUF_TYPE_WEIGHT,
  GM_BUF_TYPE_DESCRIPTOR, /* for runtime, instead of compile */
  GM_BUF_TYPE_WORKSPACE,  /* for runtime, instead of compile */
  GM_BUF_TYPE_MAX,
};

enum GMSubBufType {
  GM_SUB_BUF_TYPE_IGNORE = 0,
  GM_SUB_BUF_TYPE_INPUT,
  GM_SUB_BUF_TYPE_OUTPUT,
  GM_SUB_BUF_TYPE_INOUT,
  GM_SUB_BUF_TYPE_TEMP,
  GM_SUB_BUF_TYPE_MAX
};

enum SGDependency {
  SUBG_DEPEND_NONE = 0,
  SUBG_DEPEND_IMMEDIATE = 1,
  SUBG_DEPEND_PREGROUPS = 1,
  SUBG_DEPEND_PREALL = -1,
};

enum class FMSection {
  Text,
  Crodata,
  ZcyConst,
  ModelParam,
  Rodata,
  Dcr,
  TcbChain,
  TotalPriv,
  TotalReuse,
  GM,
  Stack,
  Dpdata,
  Printf,
  Profile,
  Coredump,
  ReservedIOVA, /* always last section */
};

/* section: .note.aipu.globalparam */
struct DSModelGlobalParam {
  uint32_t input_shape_offset;
  uint32_t num_params;
  /* std::vector<uint32_t> params; */
};

/* buffer index desc for GM */
struct BssBufferIndex {
  uint32_t fm_index;  // the idx of feature map list
  uint32_t buf_type;  // 0:reuse buffer, 1: static(constant) buffer
  uint32_t buf_index; // the index of buffer
  uint32_t resver0;
};

struct GMConfigDesc {
  GMSubBufType sub_buf_type = GM_SUB_BUF_TYPE_IGNORE;
  BssBufferIndex gm_buf_idx;
};

/* section: .note.aipu.gmconfig */
struct GMConfig {
  uint32_t GM_control = 0;
  uint32_t GM_region_ctrl[2] = {0};
  uint32_t reserve0 = 0;
  uint32_t reserve1 = 0;
  uint32_t reserve2 = 0;
  BssBufferIndex GM_buf_idx[2] = {0};
};

struct SegMMUConfig {
  struct MMUAddr {
    uint32_t control[2];
  };

  MMUAddr seg[4] = {0};
  uint32_t SegMMU_ctl = 0;
  uint32_t SegMMU_remap = 0;
  uint32_t reserve0 = 0;
  uint32_t reserve1 = 0;
  uint32_t reserve2 = 0;
  uint32_t reserve3 = 0;
};

/* section: .note.aipu.segmmu */
struct SegMMUList {
  uint32_t num_mmu;
  SegMMUConfig *segmmu;
};

struct BinSubGraphSection {
  char *va;
  uint64_t offset;
  uint64_t size;
  void load(char *_va, uint64_t _offset, uint64_t _size) {
    va = _va;
    offset = _offset;
    size = _size;
  }
};

struct Subgraph {
  uint32_t id;
  uint32_t bss_idx;
  BinSubGraphSection text;
  BinSubGraphSection rodata;
  BinSubGraphSection dcr;
  uint32_t printfifo_size;
  uint32_t profiler_buf_size;
  uint32_t private_data_size;
  uint32_t warmup_len;
  std::vector<uint32_t> precursors;
  int32_t precursor_cnt;
  std::vector<GraphParamMapLoadDesc> private_buffers_map;
  std::vector<GraphSectionDesc> private_buffers;
};

struct ConstInfo {
  uint32_t const_sz;
  uint32_t zero_copy_sz;
};

struct BSS {
  uint32_t bss_id;
  uint32_t stack_size;
  uint32_t stack_align_in_page;
  uint32_t const_size;
  uint32_t zerocpy_const_size;
  std::vector<GraphParamMapLoadDesc> param_map;
  std::vector<GraphSectionDesc> static_sections;
  std::vector<GraphSectionDesc> reuse_sections;
  /* std::map<sec.type, std::map<hash, GraphSectionDesc>> */
  std::map<uint32_t, std::map<std::array<uint8_t, 32>, GraphSectionDesc>>
      shared_static_sections;
  GraphIOTensors io;
};

struct FMSectionInfo {
  struct BufInfo {
    uint64_t offset;
    uint64_t size;
  };
  uint32_t size;
  std::map<FMSection, BufInfo> info;

  void reset() {
    size = 0;
    info.clear();
  }
};

struct GMSectionInfo {
  struct BufInfo {
    std::string name;
    uint32_t size;
    uint32_t offset;
  };

  uint32_t remap_size;
  uint32_t sync_size;
  std::map<GMBufType, BufInfo> info;

  void reset() {
    remap_size = 0;
    sync_size = 0;
    info.clear();
  }
};

class GraphV3X : public Graph {
private:
  /* m_bss_vec[0].reuse_sections includes all bss reuse sections,
   * static_sections not exactly */
  std::vector<BSS> m_bss_vec;
  std::vector<Subgraph> m_subgraphs;
  std::vector<GMConfig> m_gmconfig;
  std::string m_comment;
  std::string m_target;
  BinSection m_bsegmmu;
  BinSection m_bgraphjson;
  bool m_fake_subgraph = false;
  bool m_coredump_en = false;
  uint32_t m_max_ws_size = 0;
  FMSectionInfo m_fmsec_info; /* all asid0 buffers */
  GMSectionInfo m_gmsec_info; /* attention: different job may have different gm
                                 resource */
  static constexpr uint32_t k_tcb_reserved = 0x400000; /* 1K*4K=4M */

public:
  /* 2: REUSE/STATIC, key: index */
  std::map<uint32_t, GMConfigDesc> m_gm_config_desc[2];
  uint32_t m_segmmu_num = 0;

private:
  const std::string get_target() const override { return m_target; }

public:
  aipu_status_t parse_gmconfig(int bss_id);
  aipu_status_t collect_gm_info();
  aipu_status_t collect_fm_sections() override;

  aipu_status_t get_elf_note_size(const std::string &note_name,
                                  uint64_t &size) override;
  aipu_status_t get_elf_note(const std::string &note_name, char *data,
                             uint64_t size) override;

  aipu_status_t get_tensor_count(aipu_tensor_type_t type,
                                 uint32_t *cnt) const override;
  aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type, uint32_t tensor,
                                      aipu_tensor_desc_t *desc) const override;
  aipu_status_t create_job(JOB_ID *id,
                           const aipu_global_config_simulation_t *cfg,
                           aipu_global_config_hw_t *hw_cfg,
                           aipu_create_job_cfg_t *config = nullptr) override;

  void print_parse_info() override;

  bool is_gm_buffer(uint32_t idx, uint32_t type) {
    return m_mem->is_gm_enable() && m_gm_config_desc[type].count(idx) == 1;
  }

public:
  void set_subgraph(Subgraph sg) { m_subgraphs.push_back(sg); }

  void set_fake_subgraph() { m_fake_subgraph = true; }

  uint32_t get_subgraph_cnt() {
    return m_fake_subgraph ? 0 : m_subgraphs.size();
  }

  const Subgraph &get_subgraph(uint32_t sg_id) const {
    return m_subgraphs[sg_id];
  }

  void set_bss(BSS bss) { m_bss_vec.push_back(bss); }

  BSS &get_bss(uint32_t bss_id) { return m_bss_vec[bss_id]; }

  uint32_t get_bss_cnt() const override { return m_bss_vec.size(); }

  GraphIOTensors &get_bss_io_ref(uint32_t bss_id) override {
    return m_bss_vec[bss_id].io;
  }

  void set_graph_comment(const char *data, uint64_t size) override {
    if (size != 0) {
      m_comment.assign(data, size);
      m_comment = replace(m_comment, 0, '\n');
      m_comment = replace(m_comment, "\n\n", "\n");

      std::string key = "target:X"; /* much safer? */
      size_t start = m_comment.find(key);
      if (start != std::string::npos) {
        start += key.length();
        size_t end = m_comment.find(';', start);
        if (end != std::string::npos)
          m_target = std::string("X") + m_comment.substr(start, end - start);
      }
      LOG(LOG_DEBUG, "aipu.bin comment:\n%s", m_comment.c_str());
    }
  }

  void set_gmconfig(BinSection &gm_section) {
    GMConfig gmconfig = {0};

    memcpy((void *)&gmconfig, gm_section.va, gm_section.size);
    m_gmconfig.push_back(gmconfig);
  }

  void set_segmmu(BinSection &segmmu_section) override {
    m_segmmu_num = *(uint32_t *)segmmu_section.va;

    /* extract the head 4 bytes segmmu num information */
    if (m_segmmu_num != 0)
      m_bsegmmu.init(segmmu_section.va + 4, segmmu_section.size - 4);
  }

  void set_graphjson(BinSection &graphjson_section) override {
    m_bgraphjson.init(graphjson_section.va, graphjson_section.size);
  }

  void set_stack(uint32_t bss_id, uint32_t size, uint32_t align) {
    if (bss_id < (uint32_t)m_bss_vec.size()) {
      m_bss_vec[bss_id].stack_size = size;
      m_bss_vec[bss_id].stack_align_in_page = align;
    }
  }
  void add_param(uint32_t bss_id, GraphParamMapLoadDesc param) {
    if (bss_id < (uint32_t)m_bss_vec.size()) {
      m_bss_vec[bss_id].param_map.push_back(param);
    }
  }

  void add_static_section(uint32_t bss_id, GraphSectionDesc section) {
    if (bss_id < (uint32_t)m_bss_vec.size()) {
      m_bss_vec[bss_id].static_sections.push_back(section);

      /* cache each section with hashcode to quick search */
      if (m_is_shared_weight) {
        if (m_bss_vec[bss_id].shared_static_sections[section.type].count(
                section.hashcode) != 0) {
          auto &old_section =
              m_bss_vec[bss_id]
                  .shared_static_sections[section.type][section.hashcode];
          if (old_section.align_bytes < section.align_bytes) {
            LOG(LOG_WARN,
                "graph id: 0x%lx bss id: %u section type: %s exist same "
                "hashcode, and (old align < new align) : (0x%x <  0x%x)",
                m_id, bss_id,
                (section.type == SECTION_TYPE_ZEROCPY_CONSTANT ? "zerocopy"
                                                               : "weight"),
                old_section.align_bytes, section.align_bytes);
            return;
          }
        }
        m_bss_vec[bss_id]
            .shared_static_sections[section.type][section.hashcode] = section;
      }
    }
  }

  void add_reuse_section(uint32_t bss_id, GraphSectionDesc section) {
    if (bss_id < (uint32_t)m_bss_vec.size())
      m_bss_vec[bss_id].reuse_sections.push_back(section);

    if (bss_id != 0)
      m_bss_vec[0].reuse_sections.push_back(section);
  }
  void set_io_tensors(uint32_t bss_id, GraphIOTensors io) {
    if (bss_id < (uint32_t)m_bss_vec.size())
      m_bss_vec[bss_id].io = io;
  }

  void set_const_size(uint32_t bss_id, uint32_t const_size,
                      uint32_t zerocpy_const_size) override {
    /**
     * if one graph doesn't need weight, it just reserves
     * 4KB as default placehold for whole flow.
     */
    if (const_size == 0)
      const_size = 4096;

    if (bss_id < (uint32_t)m_bss_vec.size()) {
      m_bss_vec[bss_id].const_size = const_size;
      m_bss_vec[bss_id].zerocpy_const_size = zerocpy_const_size;
    }
  }

  uint32_t get_const_size(uint32_t bss_id = 0) const override {
    return bss_id < (uint32_t)m_bss_vec.size() ? m_bss_vec[bss_id].const_size
                                               : 0;
  }

  uint32_t get_zerocpy_const_size(uint32_t bss_id = 0) const override {
    return bss_id < (uint32_t)m_bss_vec.size()
               ? m_bss_vec[bss_id].zerocpy_const_size
               : 0;
  }

  std::vector<GraphSectionDesc> &
  get_static_section_ref(uint32_t bss_id) override {
    return m_bss_vec[bss_id].static_sections;
  }

  HASH_SECTION_TABLE &get_shared_section_ref(uint32_t bss_id) override {
    return m_bss_vec[bss_id].shared_static_sections;
  };

  uint32_t get_alloc_pad_size() const override {
    return m_isa == ISAv5 ? 0x800 : 0;
  }

  uint32_t get_asid_align_page() const override {
    return m_isa == ISAv6 ? 256 : 1;
  }

  aipu_data_type_t get_io_tensor_type(int idx) const override {
    return m_bss_vec[0].io.inputs[idx].data_type;
  }

  const std::string &get_section_name(FMSection sec) {
    const static std::string ret = "unknown";
    const static std::map<FMSection, std::string> table = {
        {FMSection::Text, "text"},
        {FMSection::Crodata, "crodata"},
        {FMSection::ZcyConst, "zcy_const"},
        {FMSection::ModelParam, "model_param"},
        {FMSection::Rodata, "rodata"},
        {FMSection::Dcr, "dcr"},
        {FMSection::TcbChain,
         "tcbs"}, /* which is aligned with malloc(name) for x2 holdtcb */
        {FMSection::TotalPriv, "total_priv"},
        {FMSection::TotalReuse, "total_reuse"},
        {FMSection::GM, "gm"},
        {FMSection::Stack, "Stack"},
        {FMSection::Dpdata, "Dpdata"},
        {FMSection::Printf, "printf"},
        {FMSection::Profile, "profile"},
        {FMSection::Coredump, "coredump"},
        {FMSection::ReservedIOVA, "reserved_iova"},
    };
    return table.count(sec) != 0 ? table.at(sec) : ret;
  }

  const FMSectionInfo &get_fmsec_info() { return m_fmsec_info; }

  const GMSectionInfo &get_gmsec_info() { return m_gmsec_info; }

public:
  GraphV3X(void *ctx, GRAPH_ID id, DeviceBase *dev);
  ~GraphV3X();
  GraphV3X(const GraphV3X &graph) = delete;
  GraphV3X &operator=(const GraphV3X &graph) = delete;

  friend class JobV3X;
  friend class JobV3;
  friend class JobV3_2;
};
} // namespace aipudrv

#endif /* _GRAPH_V3_H_ */