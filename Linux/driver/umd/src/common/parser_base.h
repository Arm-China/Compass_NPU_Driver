// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  parser_base.h
 * @brief AIPU User Mode Driver (UMD) parser base class header
 */

#ifndef _PARSER_BASE_H_
#define _PARSER_BASE_H_

#include <fstream>
#include <stdint.h>
#include <vector>
#include "standard_api.h"

namespace aipudrv
{

#define ALIGN_PAGE(bytes) (((bytes)+(4096)-1)&(~((4096)-1)))
#define ALIGN_ADDR(bytes) ((ALIGN_PAGE(bytes))/(4096))

/**
 * aipu v1/v2 version: v0.5
 * aipu v3 ELF version: v1.1
 */
#define AIPU_LOADABLE_GRAPH_V0005     ((0 << 8) + 5)
#define AIPU_LOADABLE_GRAPH_ELF_V0    ((1 << 8) + 1)

struct BinHeaderTop {
    char     magic[16];
    uint32_t device;
    uint32_t version;
    uint32_t build_version;
    uint32_t header_size;
    uint32_t file_size;
    uint32_t type;
    uint32_t flag;
};

struct BinSection {
    const char* va;
    uint64_t size;
    void init(const char* _va, uint64_t _size)
    {
        va = _va;
        size = _size;
    }
};

#define BIN_MAGIC_ARR_LEN      16
#define BIN_EXTRA_DATA_ARR_LEN 8
#define BIN_HDR_TOP_SIZE       (sizeof(BinHeaderTop))

#define MAGIC                  "AIPU BIN"

#define AIPU_ARCH(device)           (((device)>>20)&0xFF)
#define AIPU_VERSION(device)        (((device)>>16)&0xF)
#define AIPU_CONFIG(device)         ((device)&0xFFFF)
#define AIPU_REVISION(device)       ((device)>>28)
#define BUILD_MAJOR(build_version)  ((build_version)&0xF)
#define BUILD_MINOR(build_version)  (((build_version)>>8)&0xF)
#define BUILD_NUMBER(build_version) ((build_version)>>16)
#define GMINOR(gversion)            ((gversion)>>24)
#define GMAJOR(gversion)            (((gversion)>>16) & 0xFF)
#define GRAPH_VERSION(gv)           ((GMAJOR(gv) << 8) + GMINOR(gv))
#define GET_ASID_FLAG(flag)         (flag & 0xF)
#define IS_ASID_ENABLED(flag)       (flag & 0xF)
#define GET_REMAP_FLAG(flag)        ((flag >> 4) & 0x1)
#define GET_SRAM_FLAG(flag)         ((flag >> 5) & 0x1)

#define AIPU_REVISION_P             0x1

/**
 * Do NOT modify the sequence and value of the types.
 */
enum SectionType {
    SECTION_TYPE_RODATA     = 0,
    SECTION_TYPE_DESCRIPTOR = 1,
    SECTION_TYPE_TEXT       = 2,
    SECTION_TYPE_WEIGHT     = 3,
    SECTION_TYPE_BSS        = 4,
    SECTION_TYPE_MAX        = 5,
};

enum SubSectionType {
    SECTION_TYPE_INPUT      = 0,
    SECTION_TYPE_OUTPUT     = 1,
    SECTION_TYPE_INTER_DUMP = 2,
    SECTION_TYPE_PROF_DATA  = 10,
    SECTION_TYPE_PLOG_DATA  = 12,
    SECTION_TYPE_LAYER_COUNTER = 13,
    SECTION_TYPE_ERROR_CODE = 14,
    SECTION_TYPE_ZEROCPY_CONSTANT = 15,
    SECTION_TYPE_SEGMMU = 255,
};

/**
 * @brief struct BSS header v3/4/5
 *        stack is used by AIPU inference layers;
 *        static sections store static data like weights or bias;
 *        reuse sections store input/output/intermediate reused data;
 *        memory should be allocated for all stack & static & reuse sections;
 */
struct BSSHeader {
    uint32_t stack_size;              /**< stack size */
    uint32_t stack_align_bytes;       /**< static base address align requirement (in bytes) */
    uint32_t static_section_desc_cnt; /**< number of static section descriptors */
    uint32_t reuse_section_desc_cnt;  /**< number of reuse section descriptors */
};

/**
 * @brief struct sub-section data descriptor v3/4/5
 *        sub-section data need no additional memory allocation
 *
 * @note for segmmu type, ID field meaning
 *      16:31	Identify  core0-core15 use this buffer, for example 0b11 means,
 *              core0 and core1 share this buffer for segmmu.
 *      8:15	Identify which segment [0-3] are available, [4-255] are reserved.
 *      0:7	    Identify which control [0-1] are available,  value [2,255] are reserved.
 *      and addr_mask is also used as mask for rewrite segmmu controls.
 */
struct SubSectionDesc {
    uint32_t offset_in_section_exec; /**< offset in a section */
    uint32_t size;                   /**< tensor size */
    uint32_t type;                   /**< tensor type: 0: input; 1: output; 2: saved/dump; 3: intermediate */
    uint32_t id;                     /**< tensor ID, used for identifying different inputs/outputs/segmmus */
    uint32_t data_type;              /**< tensor layout type: 0: none; 1: bool; 2: uint8; 3: int8; 4: uint16; 5: int16; */
    float    scale;                  /**< scale */
    int32_t  zero_point;             /**< zero point */
    uint32_t reserved[3];            /**< reserved unused */
    uint32_t addr_mask;              /**< sub-section addr mask, ro_new = (addr & addr_mask) | (ro_old & ~addr_mask) */
    uint32_t offset_in_ro_cnt;       /**< number of offset(s) in rodata section where sub-section base address should be loaded */
};

/**
 * @brief struct static data section descriptor v3/4/5
 *        a static section will contain one or more sub-sections inside
 */
struct BSSStaticSectionDesc {
    uint32_t offset_in_file;  /**< static data offset in loadable graph file */
    uint32_t size;            /**< section size */
    uint32_t align_bytes;     /**< section base address align requirement (in bytes) */
    uint32_t sub_section_cnt; /**< number of sub-sections in this section */
};

/**
 * @brief struct reuse buffer section descriptor v5
 */
struct BSSReuseSectionDesc {
    uint32_t size;            /**< section size */
    uint32_t align_bytes;     /**< section base address align requirement (in bytes) */
    uint32_t type;            /**< section type (reserved) */
    uint32_t sub_section_cnt; /**< number of sub-sections in this section */
};

/**
 * @brief struct remap section descriptor
 */
struct RemapSectionDesc {
    uint32_t entry_cnt;
};

/**
 * @brief struct remap entry descriptor
 */
struct RemapEntry {
    uint32_t type;
    uint32_t next_addr_entry_offset;
    uint32_t next_type;
    uint32_t next_offset;
};

struct GraphIOTensorDesc {
    bool     monopolized;
    uint32_t size;
    uint32_t id;
    uint32_t ref_section_iter;
    uint32_t offset_in_section;
    float    scale;
    float    zero_point;
    aipu_data_type_t data_type;
    bool support_dma_buf;

    /**
     * for dumping data from dma_buf, it needs below
     * information to handle.
     */
    mutable int dmabuf_fd;
    mutable uint32_t dmabuf_size; // total size of dma_buf which specified by fd
    mutable int offset_in_dmabuf; // the offset from which this IO buffer starts

    const void set_dmabuf_info(int _fd, uint32_t _dmabuf_size, int _offset_in_dmabuf) const
    {
        dmabuf_fd = _fd;
        dmabuf_size = _dmabuf_size;
        offset_in_dmabuf = _offset_in_dmabuf;
    }
};

class Graph;
class ParserBase
{
private:
    aipu_status_t sort_io_tensor(std::vector<GraphIOTensorDesc>& tensors) const;
    template<typename sub_section_desc_type>
    aipu_status_t fill_io_tensor_desc_inner(uint32_t reuse_sec_iter,
        uint32_t sub_sec_iter, const sub_section_desc_type& sub_section_load,
        struct GraphIOTensors& desc, bool support_dma_buf) const;

protected:
    aipu_status_t parse_bss_section(char* bss, uint32_t size, uint32_t id,
        Graph& gobj, char** next) const;
    aipu_status_t parse_remap_section(char* remap, Graph& gobj);

public:
    virtual aipu_status_t parse_graph(std::istream& gbin, uint32_t size, Graph& gobj) = 0;

public:
    static uint32_t get_graph_bin_version(std::istream& gbin);
    static void print_graph_header_top(const BinHeaderTop& top);
    static aipu_status_t parse_graph_header_top(std::istream& gbin, uint32_t size, Graph& gobj);

public:
    ParserBase(const ParserBase& parser) = delete;
    ParserBase& operator=(const ParserBase& parser) = delete;
    virtual ~ParserBase();
    ParserBase();
};
}

#endif /* _PARSER_BASE_H_ */