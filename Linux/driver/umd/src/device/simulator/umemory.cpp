// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  umemory.cpp
 * @brief AIPU User Mode Driver (UMD) userspace memory module implementation
 */

#include <unistd.h>
#include <cstring>
#include "umemory.h"
#include "utils/log.h"
#include "utils/helper.h"

aipudrv::UMemory::UMemory(): MemoryBase(), sim_aipu::IMemEngine()
{
    /**
     * GM allocate strategy:
     * env[UMD_GM_MEAN] == 'y/Y', mean
     * env[UMD_GM_MEAN] != 'y/Y', all GM region for GM region0
     */
    const char *gm_mean = getenv("UMD_GM_MEAN");

    /**
     * DTCM size: 1, 2, 4, 8, 16, 32(MB)
     * Z1/Z2/Z3: no DTCM
     * X1: support
     * X2: current non-support
     */
    const char *dtcm_sz = getenv("UMD_DTCM_SZ");
    if (dtcm_sz != nullptr)
    {
        uint32_t size = atoi(dtcm_sz);

        if (size > 32)
        {
            size = 32;
            LOG(LOG_WARN, "DTCM size is beyond the scope, use max size 32MB\n");
        }

        m_memblock[MEM_REGION_DTCM].size = size * MB_SIZE;
    }

    /**
     * default mem region config
     * Z1/2/3/X1: no GM
     * X2: default 4MB
     */
    for (int i = 0; i < MME_REGION_MAX; i++)
    {
        m_memblock[i].bit_cnt = m_memblock[i].size / AIPU_PAGE_SIZE;
        m_memblock[i].bitmap = new bool[m_memblock[i].bit_cnt];
        memset(m_memblock[i].bitmap, true, m_memblock[i].bit_cnt);
    }

    if (is_gm_enable())
    {
        set_gm_base(0, m_memblock[MEM_REGION_GM0].base, m_memblock[MEM_REGION_GM0].size);
        set_gm_base(1, m_memblock[MEM_REGION_GM1].base, m_memblock[MEM_REGION_GM1].size);
    }

    if (gm_mean != nullptr)
    {
        if (gm_mean[0] == 'y' || gm_mean[0] == 'Y')
            m_gm_mean = true;
        else
            m_gm_mean = false;
    }
}

aipudrv::UMemory::~UMemory()
{
    for (int i = 0; i < MME_REGION_MAX; i++)
        delete[] m_memblock[i].bitmap;

    for (auto bm_iter = m_allocated.begin(); bm_iter != m_allocated.end(); bm_iter++)
        free(&bm_iter->second.desc, nullptr);

    m_allocated.clear();
}

void aipudrv::UMemory:: gm_init(uint32_t gm_size)
{
    /**
     * 512 * 1024,
     * 1 * MB_SIZE,
     * 2 * MB_SIZE,
     * 4 * MB_SIZE,
     * 8 * MB_SIZE,
     * 16 * MB_SIZE,
     * 32 * MB_SIZE,
     * 64 * MB_SIZE
     */
    if (is_gm_enable())
    {
        if (m_gm_mean)
        {
            m_memblock[MEM_REGION_GM0].base = 0;
            m_memblock[MEM_REGION_GM0].size = gm_size >> 1;

            m_memblock[MEM_REGION_GM1].base = m_memblock[MEM_REGION_GM0].base
                + m_memblock[MEM_REGION_GM0].size;
            m_memblock[MEM_REGION_GM1].size = gm_size >> 1;
        } else {
            m_memblock[MEM_REGION_GM0].base = 0;
            m_memblock[MEM_REGION_GM0].size = gm_size;

            m_memblock[MEM_REGION_GM1].base = m_memblock[MEM_REGION_GM0].base
                + m_memblock[MEM_REGION_GM0].size;
            m_memblock[MEM_REGION_GM1].size = 0;
        }

        m_memblock[MEM_REGION_DDR].base = m_memblock[MEM_REGION_GM1].base
            + m_memblock[MEM_REGION_GM1].size;
        m_memblock[MEM_REGION_DDR].size = TOTAL_SIM_MEM_SZ - m_memblock[MEM_REGION_SRAM].size
            - m_memblock[MEM_REGION_GM0].size - m_memblock[MEM_REGION_GM1].size;

        set_gm_base(0, m_memblock[MEM_REGION_GM0].base, m_memblock[MEM_REGION_GM0].size);
        set_gm_base(1, m_memblock[MEM_REGION_GM1].base, m_memblock[MEM_REGION_GM1].size);
    }

    for (int i = 0; i < MME_REGION_MAX; i++)
    {
        if (m_memblock[i].size == 0)
            continue;

        m_memblock[i].bit_cnt = m_memblock[i].size / AIPU_PAGE_SIZE;

        if (m_memblock[i].bitmap)
            delete m_memblock[i].bitmap;
        m_memblock[i].bitmap = new bool[m_memblock[i].bit_cnt];
        memset(m_memblock[i].bitmap, true, m_memblock[i].bit_cnt);
    }

    for (int i = 0; i < MME_REGION_MAX; i++)
    {
        LOG(LOG_ALERT, "mem region [%d]: base=0x%lx, size=0x%lx", i,
            m_memblock[i].base, m_memblock[i].size);
    }
}

uint32_t aipudrv::UMemory::get_next_alinged_page_no(uint32_t start, uint32_t align, int mem_region)
{
    uint32_t no = start;

    while (no < m_memblock[mem_region].bit_cnt)
    {
        uint64_t pa = m_memblock[mem_region].base + no * AIPU_PAGE_SIZE;
        if ((pa % (align * AIPU_PAGE_SIZE)) == 0)
            return no;
        no++;
    }
    return m_memblock[mem_region].bit_cnt;
}

aipu_status_t aipudrv::UMemory::malloc_internal(uint32_t size, uint32_t align, BufferDesc* desc,
    const char* str, uint32_t asid_mem_cfg)
{
    aipu_status_t ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    uint64_t malloc_size, malloc_page = 0, i = 0;
    int mem_region = asid_mem_cfg & 0xff;
    Buffer buf;

    if ((size > m_memblock[mem_region].size) || (0 == size))
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    if (nullptr == desc)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (0 == align)
        align = 1;

    malloc_page = get_page_cnt(size);
    malloc_size = malloc_page * AIPU_PAGE_SIZE;

    if (malloc_page > m_memblock[mem_region].bit_cnt)
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

    pthread_rwlock_wrlock(&m_lock);
    i = get_next_alinged_page_no(0, align, mem_region);
    while ((i + malloc_page) < m_memblock[mem_region].bit_cnt)
    {
        uint64_t j = i;
        for (; j < (i + malloc_page); j++)
        {
            if (m_memblock[mem_region].bitmap[j] == false)
            {
                i = get_next_alinged_page_no(j + 1, align, mem_region);
                break;
            }
        }

        if (j == i + malloc_page)
        {
            desc->init(0, m_memblock[mem_region].base + i * AIPU_PAGE_SIZE,
                malloc_size, size, 0, mem_region, m_memblock[mem_region].base);
            buf.init(new char[malloc_size], *desc);
            memset(buf.va, 0, malloc_size);
            m_allocated[desc->pa] = buf;
            LOG(LOG_INFO, "m_allocated.size=%ld, buffer_pa=%lx", m_allocated.size(), desc->pa);
            for (uint32_t j = 0; j < malloc_page; j++)
                m_memblock[mem_region].bitmap[i + j] = false;

            ret = AIPU_STATUS_SUCCESS;
            break;
        }
    }
    pthread_rwlock_unlock(&m_lock);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(desc->pa, desc->size, MemOperationAlloc, str, false, 0);

    return ret;
}

aipu_status_t aipudrv::UMemory::malloc(uint32_t size, uint32_t align, BufferDesc* desc,
    const char* str, uint32_t asid_mem_cfg)
{
    aipu_status_t ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    uint32_t mem_region = asid_mem_cfg & 0xff;

    if (mem_region != MEM_REGION_DDR)
        ret = malloc_internal(size, align, desc, str, asid_mem_cfg);

    if (ret != AIPU_STATUS_SUCCESS)
        ret = malloc_internal(size, align, desc, str, MEM_REGION_DDR);

    return ret;
}

aipu_status_t aipudrv::UMemory::free(const BufferDesc* desc, const char* str)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint64_t b_start, b_end;
    auto iter = m_allocated.begin();

    if (nullptr == desc)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (0 == desc->size)
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    pthread_rwlock_wrlock(&m_lock);
    iter = m_allocated.find(desc->pa);
    if (iter == m_allocated.end())
    {
        ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
        goto unlock;
    }

    iter->second.ref_put();
    if (iter->second.get_Buffer_refcnt() == 0)
    {
        b_start = (iter->second.desc.pa - m_memblock[desc->ram_region].base) / AIPU_PAGE_SIZE;
        b_end = b_start + iter->second.desc.size / AIPU_PAGE_SIZE;
        for (uint64_t i = b_start; i < b_end; i++)
            m_memblock[desc->ram_region].bitmap[i] = true;

        LOG(LOG_INFO, "free buffer_pa=%lx\n", iter->second.desc.pa);
        delete []iter->second.va;
        iter->second.va = nullptr;
        m_allocated.erase(desc->pa);
    }

unlock:
    pthread_rwlock_unlock(&m_lock);
    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(desc->pa, desc->size, MemOperationFree, str, false, 0);

    return ret;
}

aipu_status_t aipudrv::UMemory::reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc* desc, const char* str)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    Buffer buf;

    if (0 == size)
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    if (nullptr == desc)
        return AIPU_STATUS_ERROR_NULL_PTR;

    pthread_rwlock_wrlock(&m_lock);
    desc->init(0, addr, size, size);
    buf.desc = *desc;
    buf.va = new char[size];
    memset(buf.va, 0, size);
    m_reserved[desc->pa] = buf;
    pthread_rwlock_unlock(&m_lock);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(desc->pa, desc->size, MemOperationAlloc, str, false, 0);

    return ret;
}