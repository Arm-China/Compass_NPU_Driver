// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  parser_elf.h
 * @brief AIPU User Mode Driver (UMD) ELF parser class header
 */

#ifndef _PARSER_ELF_H_
#define _PARSER_ELF_H_

#include <fstream>
#include "elfio/elfio.hpp"
#include "parser_base.h"
#include "graph_v3.h"

namespace aipudrv
{

#ifndef ELF_GRAPH_MIN_VERSION
#define ELF_GRAPH_MIN_VERSION 1
#endif

#ifndef ELF_GRAPH_MAJOR_VERSION
#define ELF_GRAPH_MAJOR_VERSION 1
#endif

#ifndef MAJOR_VERSION
#define MAJOR_VERSION 5
#endif

#ifndef MINOR_VERSION
#define MINOR_VERSION 2
#endif

#ifndef BUILDTOOL_BUILD_NUMBER
#define BUILDTOOL_BUILD_NUMBER 1
#endif

 #define AIPUBIN_BT_VERSION_NUMBER ((((MAJOR_VERSION)&0xff)) | (((MINOR_VERSION)&0xff) << 8)\
        | (BUILDTOOL_BUILD_NUMBER & 0xffff) << 16)

/* section: .note.aipu.compilermsg */
struct AIPUCompilerMsg
{
    uint32_t file_version = (ELF_GRAPH_MIN_VERSION << 24) | ((ELF_GRAPH_MAJOR_VERSION & 0xff) << 16);
    uint32_t build_version = AIPUBIN_BT_VERSION_NUMBER;
    uint32_t device = 0;
    uint32_t flag = 0;
    int32_t reserve0[8] = {0};
};

struct ELFHeaderBottom {
    uint32_t elf_offset;
    uint32_t elf_size;
    uint32_t extra_data[8] = {0};
};

struct FeatureMapList
{
    uint32_t reserve0 = 0;
    uint32_t reserve1 = 0;
    uint32_t num_fm_descriptor = 0;
};

struct ElfSubGraphList
{
    uint32_t subgraphs_cnt;
};

struct ElfSubGraphDesc
{
    uint32_t id;
    uint32_t type;
    uint32_t text_offset;
    uint32_t fm_desc_offset;
    uint32_t rodata_offset;
    uint32_t rodata_size;
    uint32_t dcr_offset;
    uint32_t dcr_size;
    uint32_t printfifo_size;
    uint32_t profiler_buf_size;
    uint32_t private_data_size;
    uint32_t reserve1;
    uint32_t reserve2;
    uint32_t reserve3;
    int32_t precursor_cnt; // count of ElfPrecursorDesc
    int32_t private_buffer_cnt; // count of private BSSReuseSectionDesc
};

struct ElfPrecursorDesc
{
    uint32_t id;
};

enum ELFSection {
    ELFSectionRodata = 0,
    ELFSectionDesc,
    ELFSectionWeight,
    ELFSectionFMList,
    ELFSectionRemap,
    ELFSectionSubGraphs,
    ELFSectionCompilerMsg,
    ELFSectionGmconfig,
    ELFSectionSegmmu,
    ELFSectionCnt
};

class ParserELF : public ParserBase
{
private:
    ELFHeaderBottom m_header;
    ELFIO::elfio m_elf;
    AIPUCompilerMsg m_aipu_compile_msg;

private:
    ELFIO::section* m_text = nullptr;
    ELFIO::section* m_crodata = nullptr;
    ELFIO::section* m_data = nullptr;
    ELFIO::section* m_note = nullptr;

    BinSection sections[ELFSectionCnt];
    const char* ELFSectionName[ELFSectionCnt] = {
        "rodata",
        "desc",
        "weight",
        "fmlist",
        "remap",
        "subgraphs",
        "compilermsg",
        "gmconfig",
        "segmmu"
    };

private:
    BinSection get_bin_note(const std::string& note_name);
    ELFIO::section* get_elf_section(const std::string &section_name);
    aipu_status_t parse_subgraph(char* start, uint32_t id, GraphV3& gobj,
        uint64_t& sg_desc_size);
    aipu_status_t parse_no_subgraph(char* start, uint32_t id, GraphV3& gobj,
        uint64_t& sg_desc_size);
    aipu_status_t parse_graph_header_check(std::istream& gbin, uint32_t gbin_sz);

private:
    aipu_status_t parse_graph_header_bottom(std::istream& gbin);

public:
    virtual aipu_status_t parse_graph(std::istream& gbin, uint32_t size,
        Graph& gobj);

protected:
    aipu_status_t parse_reuse_section(char* bss, uint32_t count, uint32_t id,
        Subgraph &subgraph, char** next);

public:
    ParserELF(const ParserELF& parser) = delete;
    ParserELF& operator=(const ParserELF& parser) = delete;
    virtual ~ParserELF();
    ParserELF();
};
}

#endif /* _PARSER_ELF_H_ */