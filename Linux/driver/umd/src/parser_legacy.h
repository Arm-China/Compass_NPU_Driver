// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  parser_legacy.h
 * @brief AIPU User Mode Driver (UMD) legacy graph parser class header
 */

#ifndef _PARSER_LEGACY_H_
#define _PARSER_LEGACY_H_

#include <fstream>
#include <stdint.h>
#include "standard_api.h"
#include "parser_base.h"
#include "graph.h"

namespace aipudrv
{
struct LegacyHeaderBottom {
    uint32_t entry;
    uint32_t text_offset;
    uint32_t text_size;
    uint32_t rodata_offset;
    uint32_t rodata_size;
    uint32_t dcr_offset;
    uint32_t dcr_size;
    uint32_t data_offset;
    uint32_t data_size;
    uint32_t bss_offset;
    uint32_t bss_size;
    uint32_t misc_offset;
    uint32_t misc_size;
    int8_t   extra_data[8];
};

struct LegacySectionDesc
{
    uint32_t offset;
    uint32_t size;
    void init(uint32_t _offset, uint32_t _size)
    {
        offset = _offset;
        size   = _size;
    }
};

class ParserLegacy: public ParserBase
{
private:
    std::vector<LegacySectionDesc> m_section_descs;
    std::vector<BinSection> m_sections;

private:
    aipu_status_t parse_graph_header_bottom(std::istream& gbin, Graph& gobj);
    aipu_status_t parse_graph_header_check(std::istream& gbin, uint32_t size);

public:
    aipu_status_t parse_graph(std::istream& gbin, uint32_t size, Graph& gobj);
    BinSection get_bin_section(SectionType type);

public:
    ParserLegacy(const ParserLegacy& parser) = delete;
    ParserLegacy& operator=(const ParserLegacy& parser) = delete;
    virtual ~ParserLegacy();
    ParserLegacy();
};
}

#endif /* _PARSER_LEGACY_H_ */