// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  ukmemory.h
 * @brief AIPU User Mode Driver (UMD) user & kernel space memory module header
 */

#ifndef _UKMEMORY_H_
#define _UKMEMORY_H_

#include <map>
#include <pthread.h>
#include "memory_base.h"

namespace aipudrv
{
class UKMemory: public MemoryBase
{
private:
    int m_fd = 0;

public:
    virtual aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc* desc,
        const char* str = nullptr, uint32_t asid_mem_cfg = 0);
    virtual aipu_status_t free(const BufferDesc* desc, const char* str = nullptr);
    virtual aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc* desc, const char* str = nullptr);
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

public:
    static UKMemory* get_memory(int fd)
    {
        if (nullptr == m_mem)
        {
            m_mem = new UKMemory(fd);
        }

        return m_mem;
    }

    static void put_memory()
    {
        delete m_mem;
        m_mem = nullptr;
    }

    virtual ~UKMemory();
    UKMemory(const UKMemory& mem) = delete;
    UKMemory& operator=(const UKMemory& mem) = delete;

private:
    UKMemory(int fd);
    static UKMemory* m_mem;
};
}

#endif /* _UKMEMORY_H_ */
