// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  ukmemory.cpp
 * @brief AIPU User Mode Driver (UMD) user & kernel space memory module implementation
 */

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include <assert.h>
#include "kmd/armchina_aipu.h"
#include "ukmemory.h"
#include "utils/log.h"
#include "utils/helper.h"

aipudrv::UKMemory* aipudrv::UKMemory::m_mem = nullptr;

aipudrv::UKMemory::UKMemory(int fd): MemoryBase()
{
    m_fd = fd;
}

aipudrv::UKMemory::~UKMemory()
{
    auto bm_iter = m_allocated.begin();
    for (; bm_iter != m_allocated.end(); bm_iter++)
        free(&bm_iter->second.desc, nullptr);

    m_allocated.clear();
}

aipu_status_t aipudrv::UKMemory::malloc(uint32_t size, uint32_t align, BufferDesc* desc,
    const char* str, uint32_t asid_mem_cfg)
{
    int kret = 0;
    Buffer buf;
    char* ptr = nullptr;
    unsigned long cmd = AIPU_IOCTL_REQ_BUF, free_cmd = AIPU_IOCTL_FREE_BUF;
    aipu_buf_request buf_req = {0};
    DEV_PA_64 base = 0;

    buf_req.bytes = size;
    buf_req.align_in_page = (align == 0) ? 1: align;

    /* asid_mem_cfg, bit[15:8]: asid idx, bit[7:0] buf region */
    buf_req.asid = (asid_mem_cfg & 0xff00) >> 8;
    buf_req.region = asid_mem_cfg & 0xff;

    assert(desc != nullptr);

    if (0 == size)
        return AIPU_STATUS_ERROR_INVALID_SIZE;

    /* specific command for tcb alloc/free */
    if ((str != nullptr) && (!strncmp(str, "tcbs", 4)))
        buf_req.data_type = AIPU_MM_DATA_TYPE_TCB;

    kret = ioctl(m_fd, cmd, &buf_req);
    if (kret != 0)
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

    ptr = (char*)mmap(NULL, buf_req.desc.bytes, PROT_READ | PROT_WRITE, MAP_SHARED,
        m_fd, buf_req.desc.dev_offset);
    if (ptr == MAP_FAILED)
    {
        ioctl(m_fd, free_cmd, &buf_req.desc);
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    }

    /**
     * adjust PA filled into RO section.
     * depend on condition:
     * - ASID low 32-bits address is non-zero
     * - buffer alocated from DTCM region, only for X1
     */
    if (buf_req.desc.region == AIPU_BUF_REGION_DTCM)
        base = get_dtcm_base() - 0xD0000000;
    else
        base = get_asid_base(buf_req.desc.asid);

    desc->init(base, buf_req.desc.pa,
        buf_req.desc.bytes, size, buf_req.desc.dev_offset,
        buf_req.desc.region, buf_req.desc.gm_base);

    buf.init(ptr, *desc);
    pthread_rwlock_wrlock(&m_lock);
    m_allocated[buf_req.desc.pa] = buf;
    pthread_rwlock_unlock(&m_lock);
    add_tracking(buf_req.desc.pa, size, MemOperationAlloc, str, false, 0);

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::UKMemory::free(const BufferDesc* desc, const char* str)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    int kret = 0;
    aipu_buf_desc kdesc;
    auto iter = m_allocated.begin();
    unsigned long free_cmd = AIPU_IOCTL_FREE_BUF;

    assert(desc != nullptr);
    pthread_rwlock_wrlock(&m_lock);
    iter = m_allocated.find(desc->pa);
    if ((iter == m_allocated.end()) ||
        (iter->second.desc.size != desc->size))
    {
        ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
        goto unlock;
    }

    kdesc.pa = desc->pa;
    kdesc.bytes = desc->size;
    munmap(iter->second.va, kdesc.bytes);
    kret = ioctl(m_fd, free_cmd, &kdesc);
    if (kret != 0)
    {
        ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
        goto unlock;
    }

    m_allocated.erase(desc->pa);

unlock:
    pthread_rwlock_unlock(&m_lock);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(desc->pa, desc->size, MemOperationFree, str, false, 0);

    return ret;
}

aipu_status_t aipudrv::UKMemory::reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc* desc, const char* str)
{
    return AIPU_STATUS_SUCCESS;
}