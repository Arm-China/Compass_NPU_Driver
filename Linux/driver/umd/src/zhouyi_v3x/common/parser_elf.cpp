// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  parser_v3.cpp
 * @brief AIPU User Mode Driver (UMD) ELF parser module implementation
 */

#include "parser_elf.h"

#include <cstring>

namespace aipudrv {
ParserELF::ParserELF() : ParserBase() {}

ParserELF::~ParserELF() {}

aipu_status_t ParserELF::parse_graph_header_bottom(std::istream &gbin) {
  gbin.read((char *)&m_header, sizeof(ELFHeaderBottom));
  if (gbin.gcount() != sizeof(ELFHeaderBottom))
    return AIPU_STATUS_ERROR_INVALID_GBIN;

  return AIPU_STATUS_SUCCESS;
}

BinSection ParserELF::get_bin_note(const std::string &note_name) {
  BinSection ro = {nullptr, 0};

  if (m_note) {
    ELFIO::note_section_accessor notes(m_elf, m_note);
    ELFIO::Elf_Word no_notes = notes.get_notes_num();
    for (ELFIO::Elf_Word j = 0; j < no_notes; ++j) {
      ELFIO::Elf_Word type;
      std::string name;
      void *desc;
      ELFIO::Elf_Word size;

      if (notes.get_note(j, type, name, desc, size)) {
        if (name == note_name) {
          ro.va = (char *)desc;
          ro.size = size;
          if (m_elf.get_base_ptr() != nullptr)
            ro.offset = (char *)desc - (char *)m_elf.get_base_ptr();
          break;
        }
      }
    }
  }
  return ro;
}

ELFIO::section *ParserELF::get_elf_section(const std::string &section_name) {
  ELFIO::Elf_Half no = m_elf.sections.size();

  for (ELFIO::Elf_Half i = 0; i < no; ++i) {
    ELFIO::section *sec = m_elf.sections[i];

    if (section_name == sec->get_name())
      return sec;
  }
  return nullptr;
}

aipu_status_t ParserELF::parse_reuse_section(char *bss, uint32_t count,
                                             uint32_t id, Subgraph &subgraph,
                                             char **next) {
  GraphSectionDesc section_ir;
  BSSReuseSectionDesc *reuse_desc = nullptr;
  SubSectionDesc *sub_desc = nullptr;
  GraphParamMapLoadDesc param;
  char *reuse_sec_start = bss;
  char *reuse_subsec_start = nullptr;
  char *reuse_subsec_rocnt_start = nullptr;

  if (!reuse_sec_start)
    return AIPU_STATUS_ERROR_NULL_PTR;

  for (uint32_t reuse_sec_iter = 0; reuse_sec_iter < count; reuse_sec_iter++) {
    section_ir.init();
    reuse_desc = (BSSReuseSectionDesc *)reuse_sec_start;
    reuse_subsec_start =
        (char *)(reuse_sec_start + sizeof(BSSReuseSectionDesc));

    for (uint32_t sub_sec_iter = 0; sub_sec_iter < reuse_desc->sub_section_cnt;
         sub_sec_iter++) {
      GraphSubSectionDesc sub_desc_ir;
      sub_desc = (SubSectionDesc *)reuse_subsec_start;
      reuse_subsec_rocnt_start =
          (char *)(reuse_subsec_start + sizeof(SubSectionDesc));
      reuse_subsec_start =
          (char *)(reuse_subsec_start + sizeof(SubSectionDesc) +
                   sizeof(uint32_t) * sub_desc->offset_in_ro_cnt);

      /* get subsection desc. */
      sub_desc_ir.offset_in_section = sub_desc->offset_in_section_exec;
      section_ir.sub_sections.push_back(sub_desc_ir);

      /* update parameter map element */
      for (uint32_t i = 0; i < sub_desc->offset_in_ro_cnt;
           i++, reuse_subsec_rocnt_start += sizeof(uint32_t)) {
        uint32_t offset_in_ro = 0;

        offset_in_ro = *(uint32_t *)reuse_subsec_rocnt_start;
        param.init(offset_in_ro, PARAM_MAP_LOAD_TYPE_REUSE, sub_desc->type,
                   reuse_sec_iter, sub_sec_iter,
                   sub_desc->offset_in_section_exec, sub_desc->addr_mask);
        subgraph.private_buffers_map.push_back(param);
      }
    }

    reuse_sec_start = reuse_subsec_start;
    *next = reuse_subsec_start;

    /* update section descriptor */
    section_ir.align_in_page = ALIGN_ADDR(reuse_desc->align_bytes);
    section_ir.size = reuse_desc->size;
    subgraph.private_buffers.push_back(section_ir);
  }

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t ParserELF::parse_subgraph(char *start, uint32_t id,
                                        GraphV3X &gobj,
                                        uint64_t &sg_desc_size) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  Subgraph sg = {0};
  ElfSubGraphDesc gbin_sg_desc = {0};
  char *next = nullptr;

  if (start == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  memcpy(&gbin_sg_desc, start, sizeof(gbin_sg_desc) - sizeof(int32_t));
  sg_desc_size = sizeof(gbin_sg_desc) - sizeof(int32_t);

  sg.id = id;
  sg.bss_idx = gbin_sg_desc.fm_desc_offset; // BSS ID which sg refers to
  sg.text.load(nullptr, gbin_sg_desc.text_offset, 0);
  sg.rodata.load(nullptr, gbin_sg_desc.rodata_offset, gbin_sg_desc.rodata_size);
  sg.dcr.load(nullptr, gbin_sg_desc.dcr_offset, gbin_sg_desc.dcr_size);
  sg.printfifo_size = gbin_sg_desc.printfifo_size;
  sg.profiler_buf_size = gbin_sg_desc.profiler_buf_size;
  sg.private_data_size = gbin_sg_desc.private_data_size;
  sg.warmup_len = gbin_sg_desc.warmup_len;
  sg.precursor_cnt = gbin_sg_desc.precursor_cnt;

  /**
   * extract the subgraph dependency information
   */
  start += sizeof(gbin_sg_desc) - sizeof(int32_t);
  if (gbin_sg_desc.precursor_cnt > 0) {
    for (int32_t i = 0; i < gbin_sg_desc.precursor_cnt; i++) {
      ElfPrecursorDesc pre;
      memcpy(&pre, start, sizeof(pre));
      sg.precursors.push_back(pre.id);
      start += sizeof(pre);
    }
    sg_desc_size += sizeof(ElfPrecursorDesc) * gbin_sg_desc.precursor_cnt;
  }

  /**
   * extract the private buffer information,
   *
   * note:
   * gbin_sg_desc.private_buffer_cnt is valid until this point
   */
  gbin_sg_desc.private_buffer_cnt = *(int32_t *)start;
  sg_desc_size += sizeof(int32_t);
  start += sizeof(int32_t);
  if (gbin_sg_desc.private_buffer_cnt > 0) {
    ret = parse_reuse_section(start, gbin_sg_desc.private_buffer_cnt, id, sg,
                              &next);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;
    sg_desc_size += next - start;
  }

  LOG(LOG_DEBUG,
      "sug_graph: <id=%d, type=%d, text_offset=0x%x, fm_desc_offset=0x%x, "
      "rodata_offset=0x%x, rodata_size=0x%x, dcr_offset=0x%x, dcr_size=0x%x, "
      "pfifo_size=0x%x, prof_buf_offset=0x%x, warm_len=0x%x, prec_cnt=0x%x, "
      "priv_cnt=0x%x>",
      gbin_sg_desc.id, gbin_sg_desc.type, gbin_sg_desc.text_offset,
      gbin_sg_desc.fm_desc_offset, gbin_sg_desc.rodata_offset,
      gbin_sg_desc.rodata_size, gbin_sg_desc.dcr_offset, gbin_sg_desc.dcr_size,
      gbin_sg_desc.printfifo_size, gbin_sg_desc.profiler_buf_size,
      gbin_sg_desc.warmup_len, gbin_sg_desc.precursor_cnt,
      gbin_sg_desc.private_buffer_cnt);

  gobj.set_subgraph(sg);

  return ret;
}

aipu_status_t ParserELF::parse_no_subgraph(char *start, uint32_t id,
                                           GraphV3X &gobj,
                                           uint64_t &sg_desc_size) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  Subgraph sg = {0};
  FeatureMapList fm_list;
  char *start_va = nullptr;
  char *next = nullptr;

  if (start == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  sg.id = id;
  sg.text.load(nullptr, 0, 0);
  sg.rodata.load(nullptr, 0, 0);
  sg.dcr.load(nullptr, 0, 0);
  sg.printfifo_size = 0;
  sg.profiler_buf_size = 0;
  sg.precursor_cnt = 0;
  gobj.set_subgraph(sg);
  gobj.set_fake_subgraph();

  start_va = (char *)sections[ELFSectionFMList].va;
  memcpy(&fm_list, start_va, sizeof(fm_list));
  start_va += sizeof(fm_list);
  for (uint32_t i = 0; i < fm_list.num_fm_descriptor; i++) {
    BSS bss = {0};

    static_cast<GraphV3X &>(gobj).set_bss(bss);
    ret = parse_bss_section(start_va, 0, i, gobj, &next);
    if (ret != AIPU_STATUS_SUCCESS)
      return ret;

    start_va = next;
  }

  return ret;
}

aipu_status_t ParserELF::parse_graph_header_check(std::istream &gbin,
                                                  uint32_t gbin_sz) {
  ELFIO::Elf32_Ehdr header;
  unsigned int cur_pos = gbin.tellg();

  if (gbin_sz < (sizeof(ELFIO::Elf32_Ehdr)))
    return AIPU_STATUS_ERROR_INVALID_GBIN;

  gbin.read((char *)&header, sizeof(header));
  if (gbin.gcount() != sizeof(header))
    goto finish;

  if (header.e_type != 0x2) {
    LOG(LOG_ERR, "ELF e_type is invalid");
    goto finish;
  }

  if (header.e_machine != 0x29a) {
    LOG(LOG_ERR, "ELF e_machine is invalid");
    goto finish;
  }

  if (header.e_version != 0x1) {
    LOG(LOG_ERR, "ELF e_version is invalid");
    goto finish;
  }

  gbin.seekg(cur_pos, std::ios::beg);
  return AIPU_STATUS_SUCCESS;

finish:
  fflush(stdout);
  gbin.seekg(cur_pos, std::ios::beg);
  return AIPU_STATUS_ERROR_INVALID_GBIN;
}

aipu_status_t ParserELF::parse_graph(std::istream &gbin, uint32_t size,
                                     Graph &gobj) {
  aipu_status_t ret = parse_graph_header_check(gbin, size);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  /* real ELF parsing */
  if (!m_elf.load(gbin))
    return AIPU_STATUS_ERROR_INVALID_GBIN;

  return parse_graph_common(gobj);
}

aipu_status_t ParserELF::parse_graph(const char *file, Graph &gobj) {
  std::ifstream gbin;
  gbin.open(file, std::ifstream::in | std::ifstream::binary);
  if (!gbin.is_open())
    return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

  gbin.seekg(0, gbin.end);
  uint32_t size = gbin.tellg();
  gbin.seekg(0, gbin.beg);

  aipu_status_t ret = parse_graph_header_check(gbin, size);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  gbin.close();

  /* real ELF parsing */
  if (!m_elf.load(file))
    return AIPU_STATUS_ERROR_INVALID_GBIN;

  gobj.set_mapped_gfile(file);

  return parse_graph_common(gobj);
}

aipu_status_t ParserELF::parse_graph_common(Graph &gobj) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  ElfSubGraphList sg_desc_header = {0};
  FeatureMapList fm_list = {0};
  char *start = nullptr;
  char *next = nullptr;

  /* .text section parse */
  m_text = get_elf_section(".text");
  if (m_text == nullptr) {
    ret = AIPU_STATUS_ERROR_INVALID_GBIN;
    goto finish;
  }
  gobj.set_graph_text(m_text->get_data(), m_text->get_size());

  /* .rodata section parse */
  m_crodata = get_elf_section(".rodata");
  if (m_crodata != nullptr)
    gobj.set_graph_crodata(m_crodata->get_data(), m_crodata->get_size());

  /* .data section parse */
  m_data = get_elf_section(".data");
  if (m_data != nullptr)
    gobj.set_graph_dp(m_data->get_data(), m_data->get_size());

  m_comment = get_elf_section(".comment");
  if (m_comment != nullptr)
    gobj.set_graph_comment(m_comment->get_data(), m_comment->get_size());

  /* .note section parse */
  m_note = get_elf_section(".note.aipu");
  if (m_note == nullptr) {
    ret = AIPU_STATUS_ERROR_INVALID_GBIN;
    goto finish;
  }

  for (uint32_t i = 0; i < ELFSectionCnt; i++) {
    sections[i].init(nullptr, 0);
    sections[i] = get_bin_note(ELFSectionName[i]);
  }

  gobj.set_graph_rodata(sections[ELFSectionRodata]);
  if (sections[ELFSectionDesc].size != 0)
    gobj.set_graph_desc(sections[ELFSectionDesc]);

  if (sections[ELFSectionWeight].size != 0) {
    gobj.set_graph_weight(sections[ELFSectionWeight]);
    if (sections[ELFSectionExtraWeightName].size != 0) {
      ret = gobj.set_graph_extra_weight(sections[ELFSectionExtraWeightName]);
      if (ret != AIPU_STATUS_SUCCESS)
        goto finish;
    }
  }

  if (sections[ELFSectionConstantHashTable].size != 0) {
    ret = gobj.set_constant_hashtable(sections[ELFSectionConstantHashTable]);
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
  }

  if (sections[ELFSectionGmconfig].size != 0)
    gobj.set_gmconfig(sections[ELFSectionGmconfig]);

  /* [[deprecated]]: aipu.bin will set segmmu num=0 */
  if (sections[ELFSectionSegmmu].size != 0)
    gobj.set_segmmu(sections[ELFSectionSegmmu]);

  if (sections[ELFSectionGraphJson].size != 0)
    gobj.set_graphjson(sections[ELFSectionGraphJson]);

  if (sections[ELFSectionCompilerMsg].size != 0) {
    memcpy((void *)&m_aipu_compile_msg, sections[ELFSectionCompilerMsg].va,
           sections[ELFSectionCompilerMsg].size);
    gobj.set_buildversion(m_aipu_compile_msg.build_version);
    gobj.set_arch(AIPU_ARCH(m_aipu_compile_msg.device));
    gobj.set_hw_config(AIPU_CONFIG(m_aipu_compile_msg.device));
    gobj.set_hw_revision(AIPU_REVISION(m_aipu_compile_msg.device));
    gobj.set_disable_input_reuse(
        AIPU_DISABLE_INPUT_REUSE(m_aipu_compile_msg.reserve0[0]));
    gobj.set_disable_output_reuse(
        AIPU_DISABLE_OUTPUT_REUSE(m_aipu_compile_msg.reserve0[0]));
    gobj.set_isa(AIPU_VERSION(m_aipu_compile_msg.device));
  }

  if (m_aipu_compile_msg.flag & (1 << 6)) {
    if (sections[ELFSectionGlobalParam].size != 0)
      gobj.set_modle_global_param(sections[ELFSectionGlobalParam]);

    if (sections[ELFSectionInputShapeConstraint].size != 0) {
      if (!gobj.set_input_shape_constrait(
              sections[ELFSectionInputShapeConstraint]))
        LOG(LOG_ALERT, "Section InputShapeConstraint violation");
    }
  }

  /* parse Subgraph section */
  start = (char *)sections[ELFSectionSubGraphs].va;
  memcpy(&sg_desc_header, start, sizeof(sg_desc_header));

  LOG(LOG_INFO, "sg cnt: %d", sg_desc_header.subgraphs_cnt);
  start += sizeof(sg_desc_header);
  if (sg_desc_header.subgraphs_cnt > 0) {
    for (uint32_t i = 0; i < sg_desc_header.subgraphs_cnt; i++) {
      uint64_t sg_desc_size = 0;
      ret =
          parse_subgraph(start, i, static_cast<GraphV3X &>(gobj), sg_desc_size);
      if (ret)
        goto finish;

      start += sg_desc_size;
    }

    /* parse BSS section */
    start = (char *)sections[ELFSectionFMList].va;
    memcpy(&fm_list, start, sizeof(fm_list));
    start += sizeof(fm_list);
    for (uint32_t i = 0; i < fm_list.num_fm_descriptor; i++) {
      BSS bss = {0};

      static_cast<GraphV3X &>(gobj).set_bss(bss);
      ret =
          parse_bss_section(start, 0, i, static_cast<GraphV3X &>(gobj), &next);
      if (ret != AIPU_STATUS_SUCCESS)
        return ret;

      start = next;
    }
    sort_io(gobj.get_bss_io_ref(0));

    start = (char *)sections[ELFSectionRemap].va;
    ret = parse_remap_section(start, gobj);

    if (sg_desc_header.subgraphs_cnt != 0)
      ret = gobj.parse_gmconfig(0);
  } else if (sg_desc_header.subgraphs_cnt == 0) {
    /**
     * although there is no subgraph, to reuse the rest logic,
     * here, just construct one empty Subgraph. finally it doesn't
     * use any information in this Subgraph for running.
     */
    uint64_t sg_desc_size = 0;
    ret = parse_no_subgraph(start, 0, static_cast<GraphV3X &>(gobj),
                            sg_desc_size);
    if (ret)
      goto finish;

    /* resort reuse buffer for input and output result */
    sort_io(gobj.get_bss_io_ref(0));
  }

finish:
  return ret;
}

} // namespace aipudrv