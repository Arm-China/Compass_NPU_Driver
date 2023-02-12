// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  umemory.h
 * @brief AIPU User Mode Driver (UMD) userspace memory module header
 */

#ifndef _UMEMORY_H_
#define _UMEMORY_H_

#include "memory_base.h"
#include "simulator/mem_engine_base.h"

namespace aipudrv
{

#define TOTAL_SIM_MEM_SZ (14UL << 28)

enum {
    MEM_REGION_DDR = 0,
    MEM_REGION_SRAM = 1,
    MEM_REGION_DTCM = 2,
    MEM_REGION_GM0 = 3,
    MEM_REGION_GM1 = 4,
    MME_REGION_MAX = 5
};

struct MemBlock {
    uint64_t base;
    uint64_t size;
    uint64_t bit_cnt;
    bool     *bitmap;
};

class UMemory: public MemoryBase, public sim_aipu::IMemEngine
{
private:
    MemBlock m_memblock[MME_REGION_MAX] = {
        { .base = 0, .size = TOTAL_SIM_MEM_SZ },

        /* SRAM memory region, this base is higher than DDR base default */
        { .base = 0, .size = 0 },

        /* the base address for DTCM is fixed, currently only for X1 */
        { .base = 0xD0000000, .size = 8 * MB_SIZE },

        /* the base address for GM region0/1 are specified dynamicly */
        { .base = 0, .size = 0 },
        { .base = 0, .size = 0 }
    };
    BufferDesc *desc;
    bool m_gm_mean = true;

private:
    uint32_t get_next_alinged_page_no(uint32_t start, uint32_t align, int mem_region = 0);
    auto     get_allocated_buffer(uint64_t addr) const;

public:
    void gm_init(uint32_t gm_size_idx);
    aipu_status_t malloc_internal(uint32_t size, uint32_t align, BufferDesc* desc,
        const char* str, uint32_t asid_mem_cfg = 0);
    virtual aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc* desc,
        const char* str = nullptr, uint32_t asid_mem_cfg = 0);
    virtual aipu_status_t free(const BufferDesc* desc, const char* str = nullptr);
    aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc* desc, const char* str = nullptr);
    virtual int read(uint64_t addr, void *dest, size_t size) const
    {
        return mem_read(addr, dest, size);
    };
    virtual int write(uint64_t addr, const void *src, size_t size)
    {
        return mem_write(addr, src, size);
    };
    virtual int zeroize(uint64_t addr, size_t size)
    {
        return mem_bzero(addr, size);
    };
    virtual size_t size() const
    {
        return m_memblock[0].size;
    };
    virtual bool invalid(uint64_t addr) const
    {
        return (addr < m_memblock[0].base) || (addr >= (m_memblock[0].base + m_memblock[0].size));
    };

public:
    static UMemory* get_memory()
    {
        static UMemory mem_instance;
        return &mem_instance;
    }

    virtual ~UMemory();
    UMemory(const UMemory& mem) = delete;
    UMemory& operator=(const UMemory& mem) = delete;

private:
    UMemory();
};
}

#endif /* _UMEMORY_H_ */
