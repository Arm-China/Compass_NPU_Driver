// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
     * aipu v2(X1): support
     * other aipu version: current non-support
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

        m_memblock[ASID_REGION_0][MEM_REGION_DTCM].size = size * MB_SIZE;
    }

    /**
     * default mem region config
     * aipu v3: default 4MB
     * other aipu version: no GM
     */
    for (int i = 0; i < MME_REGION_MAX; i++)
    {
        if (m_memblock[ASID_REGION_0][i].size >= AIPU_PAGE_SIZE)
        {
            m_memblock[ASID_REGION_0][i].bit_cnt = m_memblock[ASID_REGION_0][i].size / AIPU_PAGE_SIZE;
            m_memblock[ASID_REGION_0][i].bitmap = new bool[m_memblock[ASID_REGION_0][i].bit_cnt];
            memset(m_memblock[ASID_REGION_0][i].bitmap, true, m_memblock[ASID_REGION_0][i].bit_cnt);
        }
    }

    if (!SHARE_ONE_ASID && m_memblock[ASID_REGION_1][0].size >= AIPU_PAGE_SIZE)
    {
        m_memblock[ASID_REGION_1][0].bit_cnt = m_memblock[ASID_REGION_1][0].size / AIPU_PAGE_SIZE;
        m_memblock[ASID_REGION_1][0].bitmap = new bool[m_memblock[ASID_REGION_1][0].bit_cnt];
        memset(m_memblock[ASID_REGION_1][0].bitmap, true, m_memblock[ASID_REGION_1][0].bit_cnt);
    }

    for (int i = 0; i < MME_REGION_MAX; i++)
    {
        LOG(LOG_ALERT, "ASID 0: mem region [%2d]: base=0x%.8lx, size=0x%lx", i,
            m_memblock[ASID_REGION_0][i].base, m_memblock[ASID_REGION_0][i].size);
    }

    if (!SHARE_ONE_ASID)
    {
        LOG(LOG_ALERT, "ASID 1: mem region [ 0]: base=0x%.12lx, size=0x%lx",
            m_memblock[ASID_REGION_1][0].base, m_memblock[ASID_REGION_1][0].size);
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
    free_all();
    for (int i = 0; i < MME_REGION_MAX; i++)
    {
        if (m_memblock[ASID_REGION_0][i].bitmap)
            delete[] m_memblock[ASID_REGION_0][i].bitmap;
    }

    if (m_memblock[ASID_REGION_1][0].bitmap)
        delete[] m_memblock[ASID_REGION_1][0].bitmap;
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
            set_gm_size(MEM_REGION_GM0, gm_size >> 1);
            set_gm_size(MEM_REGION_GM1, gm_size >> 1);
        } else {
            set_gm_size(MEM_REGION_GM0, gm_size);
            set_gm_size(MEM_REGION_GM1, 0);
        }
    }
}

uint32_t aipudrv::UMemory::get_next_alinged_page_no(uint32_t start, uint32_t align, int asid_mem_region)
{
    uint32_t no = start;
    uint32_t asid = (asid_mem_region >> 8) & 0xff;
    uint32_t mem_region = asid_mem_region & 0xff;

    while (no < m_memblock[asid][mem_region].bit_cnt)
    {
        uint64_t pa = m_memblock[asid][mem_region].base + no * AIPU_PAGE_SIZE;
        if ((pa % (align * AIPU_PAGE_SIZE)) == 0)
            return no;
        no++;
    }
    return m_memblock[asid][mem_region].bit_cnt;
}

aipu_status_t aipudrv::UMemory::malloc_internal(uint32_t size, uint32_t align, BufferDesc* desc,
    const char* str, uint32_t asid_mem_region)
{
    aipu_status_t ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    uint64_t malloc_size, malloc_page = 0, i = 0;
    uint32_t asid = (asid_mem_region >> 8) & 0xff;
    uint32_t mem_region = asid_mem_region & 0xff;
    Buffer buf;

    if ((size > m_memblock[asid][mem_region].size) || (0 == size))
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    if (0 == align)
        align = 1;

    malloc_page = get_page_cnt(size);
    malloc_size = malloc_page * AIPU_PAGE_SIZE;

    if (malloc_page > m_memblock[asid][mem_region].bit_cnt)
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

    pthread_rwlock_wrlock(&m_lock);
    i = get_next_alinged_page_no(0, align, (asid << 8) | mem_region);
    while ((i + malloc_page) < m_memblock[asid][mem_region].bit_cnt)
    {
        uint64_t j = i;
        for (; j < (i + malloc_page); j++)
        {
            if (m_memblock[asid][mem_region].bitmap[j] == false)
            {
                i = get_next_alinged_page_no(j + 1, align, (asid << 8) | mem_region);
                break;
            }
        }

        if (j == i + malloc_page)
        {
            desc->init(get_asid_base(asid), m_memblock[asid][mem_region].base + i * AIPU_PAGE_SIZE,
                malloc_size, size, 0, (asid << 8) | mem_region, m_memblock[asid][mem_region].base);
            buf.init(new char[malloc_size], desc);
            memset(buf.va, 0, malloc_size);
            m_allocated[desc->pa] = buf;
            LOG(LOG_INFO, "m_allocated.size=%ld, buffer_pa=%lx", m_allocated.size(), desc->pa);
            for (uint32_t j = 0; j < malloc_page; j++)
                m_memblock[asid][mem_region].bitmap[i + j] = false;

            ret = AIPU_STATUS_SUCCESS;
            break;
        }
    }
    pthread_rwlock_unlock(&m_lock);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(desc->pa, desc->size, MemOperationAlloc, str, false, 0);

    return ret;
}

aipu_status_t aipudrv::UMemory::malloc(uint32_t size, uint32_t align, BufferDesc** desc,
    const char* str, uint32_t asid_mem_cfg)
{
    aipu_status_t ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    uint32_t mem_region = asid_mem_cfg & 0xff;
    uint32_t asid = (asid_mem_cfg >> 8) & 0xff;

    if (nullptr == *desc)
    {
        *desc = new BufferDesc;
        (*desc)->reset();
    }

    if (SHARE_ONE_ASID == 1)
        asid = ASID_REGION_0;

    if ( asid == ASID_REGION_0 && mem_region == MEM_REGION_SRAM)
    {
        if (m_memblock[asid][mem_region].size == 0)
            mem_region = MEM_REGION_DDR;
    }

    if (mem_region != MEM_REGION_DDR)
        ret = malloc_internal(size, align, *desc, str, (asid << 8) | mem_region);

    if (ret != AIPU_STATUS_SUCCESS)
        ret = malloc_internal(size, align, *desc, str, (asid << 8) | MEM_REGION_DDR);

    return ret;
}

aipu_status_t aipudrv::UMemory::free(BufferDesc* desc, const char* str)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint64_t b_start, b_end;
    bool reserve_mem_flag = false;
    auto iter = m_allocated.begin();
    DEV_PA_64 pa = 0;
    uint64_t size = 0;

    if (nullptr == desc)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (0 == desc->size)
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    pthread_rwlock_wrlock(&m_lock);
    iter = m_allocated.find(desc->pa);
    if (iter == m_allocated.end())
    {
        iter = m_reserved.find(desc->pa);
        if (iter == m_reserved.end())
        {
            ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
            goto unlock;
        }
        reserve_mem_flag = true;
    }

    iter->second.ref_put();
    if (iter->second.get_Buffer_refcnt() == 0)
    {
        int mem_region = desc->ram_region;
        int asid = desc->asid;
        b_start = (iter->second.desc->pa - m_memblock[asid][mem_region].base) / AIPU_PAGE_SIZE;
        b_end = b_start + iter->second.desc->size / AIPU_PAGE_SIZE;
        for (uint64_t i = b_start; i < b_end; i++)
            m_memblock[asid][mem_region].bitmap[i] = true;

        LOG(LOG_INFO, "free buffer_pa=%lx\n", iter->second.desc->pa);
        delete []iter->second.va;
        iter->second.va = nullptr;
        if (!reserve_mem_flag)
        {
            m_allocated.erase(desc->pa);
        } else {
            m_reserved.erase(desc->pa);
            reserve_mem_flag = false;
        }
        pa = desc->pa;
        size = desc->size;
        desc->reset();
        delete desc;
    }

unlock:
    pthread_rwlock_unlock(&m_lock);
    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(pa, size, MemOperationFree, str, false, 0);

    return ret;
}

aipu_status_t aipudrv::UMemory::reserve_mem(DEV_PA_32 addr, uint32_t size,
    BufferDesc** desc, const char* str)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint64_t malloc_size, malloc_page = 0, i = 0;
    Buffer buf;
    uint32_t mem_region = MEM_REGION_DDR;
    uint32_t asid = ASID_REGION_0;

    if (0 == size)
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    if (nullptr == *desc)
    {
        *desc = new BufferDesc;
        (*desc)->reset();
    }

    pthread_rwlock_wrlock(&m_lock);

    if (addr > m_memblock[asid][MEM_REGION_SRAM].base && m_memblock[asid][MEM_REGION_SRAM].size != 0)
    {
        mem_region = MEM_REGION_SRAM;
        asid = ASID_REGION_1;
    }

    /* clear bitmap for reserved memory page */
    malloc_page = get_page_cnt(size);
    malloc_size = malloc_page * AIPU_PAGE_SIZE;
    if (malloc_page > m_memblock[asid][mem_region].bit_cnt)
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

    (*desc)->init(get_asid_base(asid), addr, malloc_size, size, 0, (asid << 8) | mem_region);
    buf.desc = *desc;
    buf.va = new char[size];
    memset(buf.va, 0, size);
    buf.ref_get();
    m_reserved[(*desc)->pa] = buf;

    i = get_next_alinged_page_no((addr - get_asid_base(asid)) >> 12, 1, (asid << 8) | mem_region);
    for (uint32_t j = 0; j < malloc_page; j++)
        m_memblock[asid][mem_region].bitmap[i + j] = false;

    pthread_rwlock_unlock(&m_lock);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking((*desc)->pa, (*desc)->size, MemOperationAlloc, str, false, 0);

    return ret;
}

bool aipudrv::UMemory::invalid(uint64_t addr) const
{
    bool found = true;
    auto iter = m_allocated.end();

    for (auto item : m_allocated_buf_map)
    {
        iter = get_allocated_buffer(item, addr);
        if (iter == item->end())
        {
            found = false;
            continue;
        } else {
            found = true;
            break;
        }
    }

    return (!found) ? true : false;
}

bool aipudrv::UMemory::get_info(uint64_t addr, uint64_t &base, uint32_t &size) const
{
    bool found = true;
    auto iter = m_allocated.end();

    for (auto item : m_allocated_buf_map)
    {
        iter = get_allocated_buffer(item, addr);
        if (iter == item->end())
        {
            found = false;
            continue;
        } else {
            found = true;
            break;
        }
    }

    if (!found)
    {
        return false;
    } else {
        base = iter->second.desc->pa;
        size = iter->second.desc->size;
        return true;
    }
}

aipu_status_t aipudrv::UMemory::free_all(void)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint64_t b_start = 0, b_end = 0;
    const char *promt = nullptr;
    DEV_PA_64 pa = 0;
    uint64_t size = 0;

    pthread_rwlock_wrlock(&m_lock);
    for (auto mem_map : m_allocated_buf_map)
    {
        for (auto iter = mem_map->begin(); iter != mem_map->end(); iter++)
        {
            BufferDesc *desc = iter->second.desc;
            int mem_region = desc->ram_region;
            int asid = desc->asid;

            if (desc->size == 0)
            {
                LOG(LOG_ALERT, "this buffer size=0, check it\n");
                continue;
            }

            b_start = (desc->pa - m_memblock[asid][mem_region].base) / AIPU_PAGE_SIZE;
            b_end = b_start + desc->size / AIPU_PAGE_SIZE;
            for (uint64_t i = b_start; i < b_end; i++)
                m_memblock[asid][mem_region].bitmap[i] = true;

            LOG(LOG_INFO, "free buffer_pa=%lx\n", desc->pa);
            delete []iter->second.va;
            pa = desc->pa;
            size = desc->size;
            desc->reset();
            delete desc;
            pthread_rwlock_unlock(&m_lock);
            (mem_map == &m_reserved) ? promt = "rsv" : promt = "normal";
            add_tracking(pa, size, MemOperationFree, promt, false, 0);
            pthread_rwlock_wrlock(&m_lock);
        }

        mem_map->clear();
    }

    pthread_rwlock_unlock(&m_lock);
    return ret;
}
