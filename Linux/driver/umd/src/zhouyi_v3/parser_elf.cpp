// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  parser_elf.cpp
 * @brief AIPU User Mode Driver (UMD) ELF parser module implementation
 */

#include <cstring>
#include "parser_elf.h"

aipudrv::ParserELF::ParserELF(): ParserBase()
{
}

aipudrv::ParserELF::~ParserELF()
{
}

aipu_status_t aipudrv::ParserELF::parse_graph_header_bottom(std::istream& gbin)
{
    gbin.read((char*)&m_header, sizeof(ELFHeaderBottom));
    if (gbin.gcount() != sizeof(ELFHeaderBottom))
        return AIPU_STATUS_ERROR_INVALID_GBIN;

    return AIPU_STATUS_SUCCESS;
}

aipudrv::BinSection aipudrv::ParserELF::get_bin_note(const std::string& note_name)
{
    aipudrv::BinSection ro = {nullptr, 0};

    if (m_note)
    {
        ELFIO::note_section_accessor notes(m_elf, m_note);
        ELFIO::Elf_Word no_notes = notes.get_notes_num();
        for (ELFIO::Elf_Word j = 0; j < no_notes; ++j)
        {
            ELFIO::Elf_Word type;
            std::string name;
            void *desc;
            ELFIO::Elf_Word size;

            if (notes.get_note(j, type, name, desc, size))
            {
                if (name == note_name)
                {
                    ro.va = (char *)desc;
                    ro.size = size;
                    break;
                }
            }
        }
    }
    return ro;
}

ELFIO::section* aipudrv::ParserELF::get_elf_section(const std::string& section_name)
{
    ELFIO::Elf_Half no = m_elf.sections.size();

    for (ELFIO::Elf_Half i = 0; i < no; ++i)
    {
        ELFIO::section *sec = m_elf.sections[i];

        if (section_name == sec->get_name())
            return sec;
    }
    return nullptr;
}

aipu_status_t aipudrv::ParserELF::parse_reuse_section(char* bss, uint32_t count, uint32_t id,
    Subgraph &subgraph, char** next)
{
    GraphSectionDesc section_ir;
    BSSReuseSectionDesc *reuse_desc = nullptr;
    SubSectionDesc *sub_desc = nullptr;
    GraphParamMapLoadDesc param;
    char *reuse_sec_start = bss;
    char *reuse_subsec_start = nullptr;
    char *reuse_subsec_rocnt_start = nullptr;

    if (!reuse_sec_start)
        return AIPU_STATUS_ERROR_NULL_PTR;

    for (uint32_t reuse_sec_iter = 0; reuse_sec_iter < count; reuse_sec_iter++)
    {
        section_ir.init();
        reuse_desc = (BSSReuseSectionDesc *)reuse_sec_start;
        reuse_subsec_start = (char*)(reuse_sec_start + sizeof(BSSReuseSectionDesc));

        for (uint32_t sub_sec_iter = 0; sub_sec_iter < reuse_desc->sub_section_cnt; sub_sec_iter++)
        {
            GraphSubSectionDesc sub_desc_ir;
            sub_desc = (SubSectionDesc *)reuse_subsec_start;
            reuse_subsec_rocnt_start = (char*)(reuse_subsec_start + sizeof(SubSectionDesc));
            reuse_subsec_start = (char*)(reuse_subsec_start + sizeof(SubSectionDesc)
                + sizeof(uint32_t) * sub_desc->offset_in_ro_cnt);

            /* get subsection desc. */
            sub_desc_ir.offset_in_section = sub_desc->offset_in_section_exec;
            section_ir.sub_sections.push_back(sub_desc_ir);

            /* update parameter map element */
            for (uint32_t i = 0; i < sub_desc->offset_in_ro_cnt;
                i++, reuse_subsec_rocnt_start+=sizeof(uint32_t))
            {
                uint32_t offset_in_ro = 0;

                offset_in_ro = *(uint32_t*)reuse_subsec_rocnt_start;
                param.init(offset_in_ro, PARAM_MAP_LOAD_TYPE_REUSE,
                    sub_desc->type, reuse_sec_iter, sub_sec_iter,
                    sub_desc->offset_in_section_exec, sub_desc->addr_mask);
                subgraph.private_buffers_map.push_back(param);
            }
        }

        reuse_sec_start = reuse_subsec_start;
        *next = reuse_subsec_start;

        /* update section descriptor */
        section_ir.load_src = nullptr;
        section_ir.align_in_page = ALIGN_ADDR(reuse_desc->align_bytes);
        section_ir.size = reuse_desc->size;
        section_ir.support_dma_buf = reuse_desc->type != 0;
        subgraph.private_buffers.push_back(section_ir);
    }

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::ParserELF::parse_subgraph(char* start, uint32_t id, GraphV3& gobj,
        uint64_t& sg_desc_size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    Subgraph sg;
    ElfSubGraphDesc gbin_sg_desc;
    FeatureMapList  fm_list;
    char* bss = nullptr;
    char* next = nullptr;

    if (nullptr == start)
        return AIPU_STATUS_ERROR_NULL_PTR;

    memcpy(&gbin_sg_desc, start, sizeof(gbin_sg_desc));
    sg_desc_size = sizeof(gbin_sg_desc);

    LOG(LOG_ALERT, "sug_graph: <id=%d, type=%d, text_offset=0x%x, fm_desc_offset=0x%x, "
        "rodata_offset=0x%x, rodata_size=0x%x, dcr_offset=0x%x, dcr_size=0x%x, "
        "pfifo_size=0x%x, prof_buf_offset=0x%x, prec_cnt=0x%x, priv_cnt=0x%x>",
        gbin_sg_desc.id, gbin_sg_desc.type, gbin_sg_desc.text_offset, gbin_sg_desc.fm_desc_offset,
        gbin_sg_desc.rodata_offset, gbin_sg_desc.rodata_size, gbin_sg_desc.dcr_offset,
        gbin_sg_desc.dcr_size, gbin_sg_desc.printfifo_size, gbin_sg_desc.profiler_buf_size,
        gbin_sg_desc.precursor_cnt, gbin_sg_desc.private_buffer_cnt);

    sg.id = id;
    sg.text.load(nullptr, gbin_sg_desc.text_offset, 0);
    sg.rodata.load(nullptr, gbin_sg_desc.rodata_offset, gbin_sg_desc.rodata_size);
    sg.dcr.load(nullptr, gbin_sg_desc.dcr_offset, gbin_sg_desc.dcr_size);
    sg.printfifo_size    = gbin_sg_desc.printfifo_size;
    sg.profiler_buf_size = gbin_sg_desc.profiler_buf_size;
    sg.private_data_size = gbin_sg_desc.private_data_size;
    sg.precursor_cnt = gbin_sg_desc.precursor_cnt;

    start += sizeof(gbin_sg_desc);
    if (gbin_sg_desc.precursor_cnt > 0)
    {
        for(int32_t i = 0; i < gbin_sg_desc.precursor_cnt; i++)
        {
            struct ElfPrecursorDesc pre;
            memcpy(&pre, start, sizeof(pre));
            sg.precursors.push_back(pre.id);
            start += sizeof(pre);
        }
        sg_desc_size += sizeof(struct ElfPrecursorDesc) * gbin_sg_desc.precursor_cnt;
    }

    if (gbin_sg_desc.private_buffer_cnt > 0)
    {
        ret = parse_reuse_section(start, gbin_sg_desc.private_buffer_cnt, id, sg, &next);
        if (AIPU_STATUS_SUCCESS != ret)
            return ret;
        sg_desc_size += next - start;
    }

    gobj.set_subgraph(sg);

    /* currently all subgraphs share one copy of feature map */
    if (gobj.get_subgraph_cnt() == 1)
    {
        bss = (char*)sections[ELFSectionFMList].va + gbin_sg_desc.fm_desc_offset;
        memcpy(&fm_list, bss, sizeof(fm_list));
        bss += sizeof(fm_list);
        for (uint32_t i = 0; i < fm_list.num_fm_descriptor; i++)
        {
            ret = parse_bss_section(bss, 0, id, gobj, &next);
            if (AIPU_STATUS_SUCCESS != ret)
                return ret;

            bss = next;
        }
    }

    return ret;
}

aipu_status_t aipudrv::ParserELF::parse_no_subgraph(char* start, uint32_t id, GraphV3& gobj,
        uint64_t& sg_desc_size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    Subgraph sg = {0};
    FeatureMapList  fm_list;
    char* bss = nullptr;
    char* next = nullptr;

    if (nullptr == start)
        return AIPU_STATUS_ERROR_NULL_PTR;

    sg.id = id;
    sg.text.load(nullptr, 0, 0);
    sg.rodata.load(nullptr, 0, 0);
    sg.dcr.load(nullptr, 0, 0);
    sg.printfifo_size    = 0;
    sg.profiler_buf_size = 0;
    sg.precursor_cnt = 0;
    gobj.set_subgraph(sg);
    gobj.set_fake_subgraph();

    bss = (char*)sections[ELFSectionFMList].va;
    memcpy(&fm_list, bss, sizeof(fm_list));
    bss += sizeof(fm_list);
    for (uint32_t i = 0; i < fm_list.num_fm_descriptor; i++)
    {
        ret = parse_bss_section(bss, 0, id, gobj, &next);
        if (AIPU_STATUS_SUCCESS != ret)
            return ret;

        bss = next;
    }

    return ret;
}

aipu_status_t aipudrv::ParserELF::parse_graph_header_check(std::istream& gbin, uint32_t gbin_sz)
{
    ELFIO::Elf32_Ehdr header;
    unsigned int cur_pos = gbin.tellg();

    if (gbin_sz < (sizeof(ELFIO::Elf32_Ehdr)))
        return AIPU_STATUS_ERROR_INVALID_GBIN;

    gbin.read((char*)&header, sizeof(header));
    if (gbin.gcount() != sizeof(header))
        goto finish;

    if (header.e_type != 0x2)
    {
        fprintf(stderr, "ELF e_type is invalid\n");
        goto finish;
    }

    if (header.e_machine != 0x29a)
    {
        fprintf(stderr, "ELF e_machine is invalid\n");
        goto finish;
    }

    if (header.e_version != 0x1)
    {
        fprintf(stderr, "ELF e_version is invalid\n");
        goto finish;
    }

    gbin.seekg(cur_pos, std::ios::beg);
    return AIPU_STATUS_SUCCESS;

finish:
    gbin.seekg(cur_pos, std::ios::beg);
    return AIPU_STATUS_ERROR_INVALID_GBIN;
}

aipu_status_t aipudrv::ParserELF::parse_graph(std::istream& gbin, uint32_t size, Graph& gobj)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    struct ElfSubGraphList sg_desc_header;
    char* start = nullptr;

    ret = parse_graph_header_check(gbin, size);
    if (ret != AIPU_STATUS_SUCCESS)
        goto finish;

    /* real ELF parsing */
    if (!m_elf.load(gbin))
    {
        ret = AIPU_STATUS_ERROR_INVALID_GBIN;
        goto finish;
    }

    /* .text section parse */
    m_text = get_elf_section(".text");
    if (nullptr == m_text)
    {
        ret = AIPU_STATUS_ERROR_INVALID_GBIN;
        goto finish;
    }
    gobj.set_graph_text(m_text->get_data(), m_text->get_size());

    /* .rodata section parse */
    m_crodata = get_elf_section(".rodata");
    if (nullptr != m_crodata)
        gobj.set_graph_crodata(m_crodata->get_data(), m_crodata->get_size());

    /* .data section parse */
    m_data = get_elf_section(".data");
    if (nullptr != m_data)
        gobj.set_graph_dp(m_data->get_data(), m_data->get_size());

    /* .note section parse */
    m_note = get_elf_section(".note.aipu");
    if (nullptr == m_note)
    {
        ret = AIPU_STATUS_ERROR_INVALID_GBIN;
        goto finish;
    }

    for (uint32_t i = 0; i < ELFSectionCnt; i++)
    {
        sections[i].init(nullptr, 0);
        sections[i] = get_bin_note(ELFSectionName[i]);
    }

    gobj.set_graph_rodata(sections[ELFSectionRodata]);
    if (sections[ELFSectionDesc].size != 0)
        gobj.set_graph_desc(sections[ELFSectionDesc]);

    if (sections[ELFSectionWeight].size != 0)
        gobj.set_graph_weight(sections[ELFSectionWeight]);

    if (sections[ELFSectionGmconfig].size != 0)
        gobj.set_gmconfig(sections[ELFSectionGmconfig]);

    if (sections[ELFSectionSegmmu].size != 0)
        gobj.set_segmmu(sections[ELFSectionSegmmu]);

    if (sections[ELFSectionCompilerMsg].size != 0)
    {
        memcpy((void *)&m_aipu_compile_msg, sections[ELFSectionCompilerMsg].va,
            sections[ELFSectionCompilerMsg].size);
        gobj.set_arch(AIPU_ARCH(m_aipu_compile_msg.device));
        gobj.set_hw_version(AIPU_VERSION(m_aipu_compile_msg.device));
        gobj.set_hw_config(AIPU_CONFIG(m_aipu_compile_msg.device));
        gobj.set_hw_revision(AIPU_REVISION(m_aipu_compile_msg.device));
    }

    start = (char*)sections[ELFSectionSubGraphs].va;
    memcpy(&sg_desc_header, start, sizeof(sg_desc_header));

    start += sizeof(sg_desc_header);
    if (sg_desc_header.subgraphs_cnt > 0) {
        for (uint32_t i = 0; i < sg_desc_header.subgraphs_cnt; i++)
        {
            uint64_t sg_desc_size = 0;
            ret = parse_subgraph(start, i, static_cast<GraphV3&>(gobj), sg_desc_size);
            if (ret)
                goto finish;

            start += sg_desc_size;
        }
    } else if (sg_desc_header.subgraphs_cnt == 0) {
        /**
         * although there is no subgraph, to reuse the rest logic,
         * here, just construct one empty Subgraph. finally it doesn't
         * use any information in this Subgraph for running.
         */
        uint64_t sg_desc_size = 0;
        ret = parse_no_subgraph(start, 0, static_cast<GraphV3&>(gobj), sg_desc_size);
        if (ret)
            goto finish;

        start += sg_desc_size;
    }

    start = (char*)sections[ELFSectionRemap].va;
    ret = parse_remap_section(start, gobj);

    if (sg_desc_header.subgraphs_cnt != 0)
        ret = gobj.extract_gm_info(0);

finish:
    return ret;
}
