// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  memory_base.h
 * @brief AIPU User Mode Driver (UMD) memory base module header
 */

#ifndef _MEMORY_BASE_H_
#define _MEMORY_BASE_H_

#include <map>
#include <pthread.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "standard_api.h"
#include "kmd/armchina_aipu.h"
#include "type.h"
#include "utils/log.h"

namespace aipudrv
{
#define AIPU_PAGE_SIZE  (4 * 1024)
#define MB_SIZE         (1 * 1024 * 1024UL)

struct GMRegion
{
    DEV_PA_64 base;
    uint32_t size;
};

struct BufferDesc
{
    DEV_PA_64 pa;       /**< device physical base address, pa =  asid_base + align_asid_pa */
    DEV_PA_64 align_asid_pa; /**< the offset PA relative to ASID base addr */
    DEV_PA_64 asid_base;/**< ASID base PA addr */
    uint64_t  size;     /**< buffer size */
    uint64_t  req_size; /**< requested size (<= buffer size) */
    uint64_t  dev_offset;
    DEV_PA_64 gm_base; /**< GM base address for this buffer if allcated from GM */
    uint32_t  ram_region = AIPU_BUF_REGION_DEFAULT;

    void init(DEV_PA_64 _asid_base, DEV_PA_64 _pa, uint64_t _size,
        uint64_t _req_size, uint64_t _offset = 0, uint32_t _ram_region = 0, DEV_PA_64 _gm_base = 0)
    {
        pa = _pa;
        asid_base = _asid_base;
        align_asid_pa = _pa - asid_base;
        size = _size;
        req_size = _req_size;
        dev_offset = _offset;
        ram_region = _ram_region;
        gm_base = _gm_base;
    }
    void reset()
    {
        pa = 0;
        align_asid_pa = 0;
        asid_base = 0;
        size = 0;
        req_size = 0;
        dev_offset = 0;
        ram_region = AIPU_BUF_REGION_DEFAULT;
        gm_base = 0;
    }
};

struct Buffer
{
    char* va;
    BufferDesc desc;
    void init(char* _va, BufferDesc _desc)
    {
        va = _va;
        desc = _desc;
    }
};

enum MemOperation
{
    MemOperationAlloc,
    MemOperationFree,
    MemOperationRead,
    MemOperationWrite,
    MemOperationBzero,
    MemOperationDump,
    MemOperationReload,
    MemOperationCnt,
};

struct MemTracking
{
    DEV_PA_64    pa;
    uint64_t     size;
    MemOperation op;
    std::string  log;
    bool         is_32_op;
    uint32_t     data;
    void init(DEV_PA_64 _pa, uint64_t _size, MemOperation _op,
        const char* str)
    {
        pa = _pa;
        size = _size;
        op = _op;
        is_32_op = false;
        log = std::string(str);
    }
    void init(DEV_PA_64 _pa, uint64_t _size, MemOperation _op,
        std::string str)
    {
        pa = _pa;
        size = _size;
        op = _op;
        is_32_op = false;
        log = str;
    }
};

class MemoryBase
{
private:
    const char* MemOperationStr[MemOperationCnt] = {
        "alloc",
        "free",
        "read",
        "write",
        "bzero",
        "dump",
        "reload",
    };

private:
    mutable std::ofstream mem_dump;
    mutable std::vector<MemTracking> m_tracking;
    mutable uint32_t m_tracking_idx = 0;
    mutable pthread_rwlock_t m_tlock;
    mutable uint32_t start = 0;
    mutable uint32_t end = 0;
    uint32_t m_enable_mem_dump = DUMP_MEM_OP_MASK;
    std::string m_file_name = "mem_info.log";
    DEV_PA_64 m_asid_base[4] = {0};
    GMRegion m_gm_region[2] = {0};
    bool m_gm_enable = true;

    /* only for X1 */
    DEV_PA_64 m_dtcm_base = 0;
    uint32_t m_dtcm_size = 0;

protected:
    std::map<DEV_PA_64, Buffer> m_allocated;
    std::map<DEV_PA_64, Buffer> m_reserved;
    mutable pthread_rwlock_t m_lock;

private:
    std::string get_tracking_log(DEV_PA_64 pa) const;
    auto get_allocated_buffer(std::map<DEV_PA_64, Buffer> *buffer_pool, uint64_t addr) const;

protected:
    uint64_t get_page_cnt(uint64_t bytes) const
    {
        return ceil((double)bytes/AIPU_PAGE_SIZE);
    }

    uint64_t get_page_no(DEV_PA_64 pa) const
    {
        return floor((double)pa/AIPU_PAGE_SIZE);
    }

    void add_tracking(DEV_PA_64 pa, uint64_t size, MemOperation op,
        const char* str, bool is_32_op, uint32_t data) const;
    int mem_read(uint64_t addr, void *dest, size_t size) const;
    int mem_write(uint64_t addr, const void *src, size_t size);

public:
    void dump_tracking_log_start() const;
    void dump_tracking_log_end() const;
    void write_line(const char* log) const;
    int mem_bzero(uint64_t addr, size_t size);
    void set_asid_base(int i, DEV_PA_64 base)
    {
        m_asid_base[i] = base;
    }

    DEV_PA_64 get_asid_base(int i)
    {
        return m_asid_base[i];
    }

    void set_dtcm_info(DEV_PA_64 base, uint32_t size)
    {
        m_dtcm_base = base;
        m_dtcm_size = size;
    }

    DEV_PA_64 get_dtcm_base()
    {
        return m_dtcm_base;
    }

    uint32_t get_dtcm_size()
    {
        return m_dtcm_size;
    }

    void set_gm_base(int i, DEV_PA_64 base, uint32_t size)
    {
        m_gm_region[i].base = base;
        m_gm_region[i].size = size;
    }

    DEV_PA_64 get_gm_base(int i)
    {
        return m_gm_region[i].base;
    }

    uint32_t get_gm_size(int i)
    {
        return m_gm_region[i].size;
    }

    void set_gm_enable(bool enable)
    {
        m_gm_enable = enable;
    }

    bool is_gm_enable()
    {
        return m_gm_enable;
    }

    bool is_both_gm_region_enable()
    {
        return m_gm_enable && (get_gm_size(0) != 0) && (get_gm_size(1) != 0);
    }

public:
    /* Interfaces */
    int pa_to_va(uint64_t addr, uint64_t size, char** va) const;
    virtual aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc* buf,
        const char* str = nullptr, uint32_t asid_qos_cfg = 0) = 0;
    virtual aipu_status_t free(const BufferDesc* buf, const char* str = nullptr) = 0;
    virtual aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc* desc, const char* str = nullptr) = 0;
    virtual int read(uint64_t addr, void *dest, size_t size) const = 0;
    virtual int write(uint64_t addr, const void *src, size_t size) = 0;
    virtual int zeroize(uint64_t addr, size_t size) = 0;
    virtual aipu_status_t dump_file(DEV_PA_64 src, const char* name, uint32_t size);
    virtual aipu_status_t load_file(DEV_PA_64 dest, const char* name, uint32_t size);
    virtual void gm_init(uint32_t gm_size_idx) {}

    int write32(DEV_PA_64 dest, uint32_t src)
    {
        return mem_write(dest, (char*)&src, sizeof(src));
    }
    int read32(uint32_t* desc, DEV_PA_64 src)
    {
        return mem_read(src, (char*)desc, sizeof(*desc));
    }

public:
    MemoryBase();
    virtual ~MemoryBase();
    MemoryBase(const MemoryBase& mem) = delete;
    MemoryBase& operator=(const MemoryBase& mem) = delete;
};
}

#endif /* _MEMORY_BASE_H_ */
