// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  parser_legacy.cpp
 * @brief AIPU User Mode Driver (UMD) legacy parser module implementation
 */

#include <cstring>
#include "parser_legacy.h"
#include "utils/helper.h"
#include "utils/log.h"

aipudrv::ParserLegacy::ParserLegacy()
{
}

aipudrv::ParserLegacy::~ParserLegacy()
{
    for (uint32_t i = 0; i < m_sections.size(); i++)
    {
        delete[] m_sections[i].va;
    }
}

aipu_status_t aipudrv::ParserLegacy::parse_graph_header_check(std::istream& gbin, uint32_t gbin_sz)
{
    BinHeaderTop top_header;
    LegacyHeaderBottom bot_header;
    unsigned int header_sz = 0, tmp = 0;
    unsigned int cur_pos = gbin.tellg();
    #define GBIN_HEADER_MIN_SZ (104)

    gbin.read((char*)&top_header, sizeof(BinHeaderTop));
    gbin.read((char*)&bot_header, sizeof(LegacyHeaderBottom));
    if (gbin.gcount() != sizeof(LegacyHeaderBottom))
        goto finish;

    header_sz = top_header.header_size;
    if (header_sz < GBIN_HEADER_MIN_SZ || header_sz > gbin_sz)
    {
        fprintf(stderr, "graph bin header sz invalid\n");
        goto finish;
    }

    if (top_header.file_size <  gbin_sz || top_header.file_size >  gbin_sz)
    {
        fprintf(stderr, "graph bin file sz invalid\n");
        goto finish;
    }

    if (bot_header.entry > gbin_sz)
    {
        fprintf(stderr, "graph bin entry invalid\n");
        goto finish;
    }

    tmp = bot_header.text_offset + bot_header.text_size;
    if (((bot_header.text_size > 0) && (tmp < header_sz || tmp > gbin_sz))
        || (bot_header.text_size == 0) || (bot_header.text_offset < header_sz))
    {
        fprintf(stderr, "graph bin text sz invalid\n");
        goto finish;
    }

    tmp = bot_header.data_offset + bot_header.data_size;
    if (((bot_header.data_size > 0) && (tmp < header_sz || tmp > gbin_sz))
        || (bot_header.data_offset < header_sz))
    {
        fprintf(stderr, "graph bin data sz invalid\n");
        goto finish;
    }

    tmp = bot_header.bss_offset + bot_header.bss_size;
    if (((bot_header.bss_size > 0) && (tmp < header_sz || tmp > gbin_sz))
        || (bot_header.bss_offset < header_sz))
    {
        fprintf(stderr, "graph bin bss sz invalid\n");
        goto finish;
    }

    tmp = bot_header.rodata_offset + bot_header.rodata_size;
    if (((bot_header.rodata_size > 0) && (tmp < header_sz || tmp > gbin_sz))
        || (bot_header.rodata_offset < header_sz))
    {
        fprintf(stderr, "graph bin rodata sz invalid\n");
        goto finish;
    }

    tmp = bot_header.dcr_offset + bot_header.dcr_size;
    if (((bot_header.dcr_size > 0) && (tmp < header_sz || tmp > gbin_sz))
        || (bot_header.dcr_offset < header_sz))
    {
        fprintf(stderr, "graph bin dcr sz invalid\n");
        goto finish;
    }

    gbin.seekg(cur_pos, std::ios::beg);
    return AIPU_STATUS_SUCCESS;

finish:
    gbin.seekg(cur_pos, std::ios::beg);
    return AIPU_STATUS_ERROR_INVALID_GBIN;
}

aipu_status_t aipudrv::ParserLegacy::parse_graph_header_bottom(std::istream& gbin, Graph& gobj)
{
    LegacyHeaderBottom header;
    LegacySectionDesc desc;

    gbin.read((char*)&header, sizeof(LegacyHeaderBottom));
    if (gbin.gcount() != sizeof(LegacyHeaderBottom))
        return AIPU_STATUS_ERROR_INVALID_GBIN;

    gobj.set_enrty(header.entry);
    gobj.set_dtcm_size(*(int *)(&header.extra_data[4]));

    /* Do NOT modify the initialization sequence */
    desc.init(header.rodata_offset, header.rodata_size);
    m_section_descs.push_back(desc);
    desc.init(header.dcr_offset, header.dcr_size);
    m_section_descs.push_back(desc);
    desc.init(header.text_offset, header.text_size);
    m_section_descs.push_back(desc);
    desc.init(header.data_offset, header.data_size);
    m_section_descs.push_back(desc);
    gbin.seekg(0, gbin.end);
    desc.init(header.bss_offset, (uint32_t)gbin.tellg() - header.bss_offset);
    m_section_descs.push_back(desc);

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::ParserLegacy::parse_graph(std::istream& gbin, uint32_t size, Graph& gobj)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    BinSection section;
    char* remap = nullptr;

    if (size < (BIN_HDR_TOP_SIZE + sizeof(LegacyHeaderBottom)))
        return AIPU_STATUS_ERROR_INVALID_GBIN;

    ret = parse_graph_header_check(gbin, size);
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    gbin.seekg(0, gbin.beg);
    ret = parse_graph_header_top(gbin, size, gobj);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    ret = parse_graph_header_bottom(gbin, gobj);
    if (AIPU_STATUS_SUCCESS != ret)
        return ret;

    for (uint32_t i = 0; i < SECTION_TYPE_MAX; i++)
    {
        gbin.seekg(m_section_descs[i].offset, gbin.beg);
        section.size = m_section_descs[i].size;
        section.va = new char[section.size];
        gbin.read((char*)section.va, section.size);
        if (gbin.gcount() != (int)section.size)
            return AIPU_STATUS_ERROR_INVALID_GBIN;

        m_sections.push_back(section);
    }

    gobj.set_graph_rodata(m_sections[SECTION_TYPE_RODATA]);
    gobj.set_graph_desc(m_sections[SECTION_TYPE_DESCRIPTOR]);
    gobj.set_graph_weight(m_sections[SECTION_TYPE_WEIGHT]);
    gobj.set_graph_text(m_sections[SECTION_TYPE_TEXT].va, m_sections[SECTION_TYPE_TEXT].size);

    ret = parse_bss_section((char*)m_sections[SECTION_TYPE_BSS].va,
        m_sections[SECTION_TYPE_BSS].size, 0, gobj, &remap);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    ret = parse_remap_section(remap, gobj);

finish:
    return ret;
}