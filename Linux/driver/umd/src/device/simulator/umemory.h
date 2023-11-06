// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
#define SIM_SRAM_SZ      (0)
#define SIM_DTCM_SZ      (4 << 20) /* 4 MB */

enum {
    MEM_REGION_DDR  = 0,
    MEM_REGION_SRAM = 1,
    MEM_REGION_DTCM = 2,
    MME_REGION_MAX  = 4
};

enum {
    MEM_REGION_GM0  = 0,
    MEM_REGION_GM1  = 1
};

enum {
    /* map MEM_REGION_DDR as ASID-0 */
    ASID_REGION_0 = 0,

    /**
     * map MEM_REGION_SRAM as ASID-1 if sram's size is non-zero,
     * otherwise, map MEM_REGION_DDR as ASID-1
     */
    ASID_REGION_1 = 1,

    ASID_REGION_2 = 2,
    ASID_REGION_3 = 3,
    ASID_MAX
};

struct MemBlock {
    uint64_t base;
    uint64_t size;
    uint64_t bit_cnt;
    bool     *bitmap;
};

/**
 * 1: set all ASIDs with identical base address
 * 0: set ASIDs with non-identical base address
 */
#define SHARE_ONE_ASID 1

class UMemory: public MemoryBase, public sim_aipu::IMemEngine
{
private:
    MemBlock m_memblock[ASID_MAX][MME_REGION_MAX] = {
        {
            { .base = 0, .size = (TOTAL_SIM_MEM_SZ - SIM_SRAM_SZ) },

            /* SRAM memory region, this base is higher than DDR base default */
            { .base = (TOTAL_SIM_MEM_SZ - SIM_SRAM_SZ), .size = SIM_SRAM_SZ },

            /* the base address for DTCM is fixed, currently only for aipu v2(X1) */
            { .base = 0xD0000000, .size = SIM_DTCM_SZ },

            {0, 0}
        },
        {
            { .base = 1ul << 32, .size = 1ul << 32 },
            {0, 0}, {0, 0}, {0, 0}
        },
        { {0, 0}, {0, 0}, {0, 0}, {0, 0} },
        { {0, 0}, {0, 0}, {0, 0}, {0, 0} }
    };
    BufferDesc *desc = nullptr;
    bool m_gm_mean = false;

private:
    uint32_t get_next_alinged_page_no(uint32_t start, uint32_t align, int mem_region = 0);

public:
    uint64_t get_memregion_base(int32_t asid, int32_t region)
    {
        if (asid == ASID_REGION_0 || asid == ASID_REGION_2 || asid == ASID_REGION_3)
        {
            if ((region == MEM_REGION_SRAM) && (m_memblock[ASID_REGION_0][MEM_REGION_SRAM].size != 0))
                return m_memblock[ASID_REGION_0][MEM_REGION_SRAM].base;
            else if (region == MEM_REGION_DTCM)
                return m_memblock[ASID_REGION_0][MEM_REGION_DTCM].base;
            else
                return m_memblock[ASID_REGION_0][MEM_REGION_DDR].base;
        } else {
            return m_memblock[ASID_REGION_1][MEM_REGION_DDR].base;
        }
    }

    uint32_t get_memregion_size(int32_t asid, int region)
    {
        if (asid == ASID_REGION_0 || asid == ASID_REGION_2 || asid == ASID_REGION_3)
        {
            return m_memblock[ASID_REGION_0][region].size;
        } else {
            return m_memblock[ASID_REGION_1][MEM_REGION_DDR].size;
        }
    }

public:
    void gm_init(uint32_t gm_size_idx);
    aipu_status_t malloc_internal(uint32_t size, uint32_t align, BufferDesc* desc,
        const char* str, uint32_t asid_mem_cfg = 0);
    virtual aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc** desc,
        const char* str = nullptr, uint32_t asid_mem_cfg = 0);
    virtual aipu_status_t free(BufferDesc* desc, const char* str = nullptr);
    aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc** desc, const char* str = nullptr);
    aipu_status_t free_all(void);
    virtual bool invalid(uint64_t addr) const;
    virtual bool get_info(uint64_t addr, uint64_t &base, uint32_t &size) const;
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
        return m_memblock[ASID_REGION_0][MEM_REGION_DDR].size;
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
