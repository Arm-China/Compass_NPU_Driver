// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
    virtual aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc** desc,
        const char* str = nullptr, uint32_t asid_mem_cfg = 0);
    virtual aipu_status_t free(BufferDesc* desc, const char* str = nullptr);
    virtual aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc** desc,
        const char* str = nullptr);
    aipu_status_t free_all(void);
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
        static UKMemory mem_instance(fd);
        return &mem_instance;
    }

    virtual ~UKMemory();
    UKMemory(const UKMemory& mem) = delete;
    UKMemory& operator=(const UKMemory& mem) = delete;

private:
    UKMemory(int fd);
};
}

#endif /* _UKMEMORY_H_ */
