// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  memory_base.cpp
 * @brief AIPU User Mode Driver (UMD) memory base module implementation
 */

#include <cstring>
#include <mutex>
#include "memory_base.h"
#include "utils/log.h"
#include "utils/helper.h"

aipudrv::MemoryBase::MemoryBase()
{
    const char *mem_op_env = getenv("UMD_MEM_OP");
    const char *gm_enable = getenv("UMD_GM_ENABLE");
    int32_t mem_op_idx = 0;

    pthread_rwlock_init(&m_tlock, NULL);
    pthread_rwlock_init(&m_lock, NULL);

    if (mem_op_env != nullptr)
    {
        mem_op_idx = atoi(mem_op_env);
        if ((mem_op_idx > 0) && (mem_op_idx & DUMP_ALL_MEM_OP_MASK))
            m_enable_mem_dump = mem_op_idx & DUMP_ALL_MEM_OP_MASK;
    }

    if (m_enable_mem_dump)
    {
        mem_dump.open(m_file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
        mem_dump.close();
    }

    if (gm_enable != nullptr)
    {
        if (gm_enable[0] == 'y' || gm_enable[0] == 'Y')
            set_gm_enable(true);
        else
            set_gm_enable(false);
    }
}

aipudrv::MemoryBase::~MemoryBase()
{
    auto bm_iter = m_allocated.begin();
    for (; bm_iter != m_allocated.end(); bm_iter++)
        delete[] bm_iter->second.va;

    pthread_rwlock_destroy(&m_lock);
    pthread_rwlock_destroy(&m_tlock);
}

std::string aipudrv::MemoryBase::get_tracking_log(DEV_PA_64 pa) const
{
    std::string log;

    for (uint32_t i = 0; i < m_tracking.size(); i++)
    {
        if ((pa >= m_tracking[i].pa) &&
            (pa < (m_tracking[i].pa + m_tracking[i].size)))
            log = m_tracking[i].log;
    }

    return log;
}

void aipudrv::MemoryBase::add_tracking(DEV_PA_64 pa, uint64_t size, MemOperation op,
    const char* str, bool is_32_op, uint32_t data) const
{
    std::string log;
    MemTracking tracking;
    char f_log[1024];

    if ((m_enable_mem_dump & (1 << op)) != (uint32_t)(1 << op))
        return;

    pthread_rwlock_wrlock(&m_lock);
    if (nullptr == str)
        log = get_tracking_log(pa);
    else
        log = str;

    tracking.init(pa, size, op, log);
    if (is_32_op)
    {
        tracking.is_32_op = true;
        tracking.data = data;
    }

    if (op == MemOperationAlloc)
        m_tracking.push_back(tracking);

    if (tracking.is_32_op)
    {
        snprintf(f_log, 1024, "%-6u 0x%-16lx %-9s %-9s 0x%-8lx 0x%-8x",
            m_tracking_idx,
            tracking.pa,
            tracking.log.c_str(),
            MemOperationStr[tracking.op],
            tracking.size,
            tracking.data
        );
    } else {
        snprintf(f_log, 1024, "%-6u 0x%-16lx %-9s %-9s 0x%-8lx %s",
            m_tracking_idx,
            tracking.pa,
            tracking.log.c_str(),
            MemOperationStr[tracking.op],
            tracking.size,
            "N/A"
        );
    }
    write_line(f_log);
    m_tracking_idx++;
    pthread_rwlock_unlock(&m_lock);
}

void aipudrv::MemoryBase::dump_tracking_log_start() const
{
    static std::mutex mtex;
    char log[1024];

    std::lock_guard<std::mutex> lock_(mtex);
    if ((m_enable_mem_dump == 0) || (start != 0))
        return;

    snprintf(log, 1024, "===========================Memory Info Dump============================");
    write_line(log);
    snprintf(log, 1024, "No.    Address            Type      OP        Size       Data");
    write_line(log);
    snprintf(log, 1024, "------------------------------------------------------------------");
    write_line(log);
    start = 1;
}

void aipudrv::MemoryBase::dump_tracking_log_end() const
{
    static std::mutex mtex;
    char log[1024];

    std::lock_guard<std::mutex> lock_(mtex);
    if ((m_enable_mem_dump == 0) || (end != 0))
        return;

    snprintf(log, 1024, "=======================================================================");
    write_line(log);
    end = 1;
}

void aipudrv::MemoryBase::write_line(const char* log) const
{
    if (m_enable_mem_dump)
    {
        mem_dump.open(m_file_name.c_str(), std::ofstream::out | std::ofstream::app);
        mem_dump << log << std::endl;
        mem_dump.close();
    }
}

std::map<aipudrv::DEV_PA_64, aipudrv::Buffer>::iterator
aipudrv::MemoryBase::get_allocated_buffer(std::map<DEV_PA_64, Buffer> *buffer_pool, uint64_t addr) const
{
    std::map<aipudrv::DEV_PA_64, aipudrv::Buffer>::iterator iter;

    for (iter = buffer_pool->begin(); iter != buffer_pool->end(); iter++)
    {
        if ((addr >= iter->second.desc.pa) &&
            (addr < (iter->second.desc.pa + iter->second.desc.size)))
            return iter;
    }
    return buffer_pool->end();
}

int aipudrv::MemoryBase::mark_shared_buffer(uint64_t addr, uint64_t size)
{
    int ret = 0;
    auto iter = m_allocated.end();

    pthread_rwlock_wrlock(&m_lock);
    iter = get_allocated_buffer((std::map<DEV_PA_64, Buffer> *)&m_allocated, addr);
    if (iter == m_allocated.end())
    {
        ret = -1;
        LOG(LOG_ERR, "invalid pa addr 0x%lx/size 0x%lx is used: no buffer\n", addr, size);
        dump_stack();
        goto unlock;
    }

    /* found the buffer in m_allocated/m_reserved */
    if ((addr + size) > (iter->second.desc.pa + iter->second.desc.size))
    {
        ret = -2;
        LOG(LOG_ERR, "invalid pa addr 0x%lx/size 0x%lx is used: out of range\n", addr, size);
        dump_stack();
        goto unlock;
    }

    iter->second.ref_get();

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

int aipudrv::MemoryBase::get_shared_buffer(uint64_t addr, uint64_t size, Buffer &buffer)
{
    int ret = 0;
    auto iter = m_allocated.end();

    pthread_rwlock_wrlock(&m_lock);
    iter = get_allocated_buffer((std::map<DEV_PA_64, Buffer> *)&m_allocated, addr);
    if (iter == m_allocated.end())
    {
        ret = -1;
        LOG(LOG_ERR, "invalid pa addr 0x%lx/size 0x%lx is used: no buffer\n", addr, size);
        dump_stack();
        goto unlock;
    }

    buffer = iter->second;

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

int aipudrv::MemoryBase::pa_to_va(uint64_t addr, uint64_t size, char** va) const
{
    int ret = 0;
    auto iter = m_allocated.end();

    pthread_rwlock_wrlock(&m_lock);
    *va = nullptr;
    iter = get_allocated_buffer((std::map<DEV_PA_64, Buffer> *)&m_allocated, addr);
    if (iter == m_allocated.end())
    {
        iter = m_reserved.end();
        iter = get_allocated_buffer((std::map<DEV_PA_64, Buffer> *)&m_reserved, addr);
        if (iter == m_reserved.end())
        {
            ret = -1;
            LOG(LOG_ERR, "invalid pa addr 0x%lx is used: no such a buffer\n", addr);
            dump_stack();
            goto unlock;
        }
    }

    /* found the buffer in m_allocated/m_reserved */
    if ((addr + size) > (iter->second.desc.pa + iter->second.desc.size))
    {
        ret = -2;
        LOG(LOG_ERR, "invalid pa addr 0x%lx/size 0x%lx is used: out of range\n", addr, size);
        dump_stack();
        goto unlock;
    }

    *va = iter->second.va + addr - iter->second.desc.pa;

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

int aipudrv::MemoryBase::mem_read(uint64_t addr, void *dest, size_t size) const
{
    int ret = -1;
    char* src = nullptr;

    if (pa_to_va(addr, size, &src) == 0)
    {
        memcpy(dest, src, size);
        ret = size;
        add_tracking(addr, size, MemOperationRead, nullptr, (size == 4), *(uint32_t*)src);
    }

    return ret;
}

int aipudrv::MemoryBase::mem_write(uint64_t addr, const void *src, size_t size)
{
    int ret = -1;
    char* dest = nullptr;

    if (pa_to_va(addr, size, &dest) == 0)
    {
        memcpy(dest, src, size);
        ret = size;
        add_tracking(addr, size, MemOperationWrite, nullptr, (size == 4), *(uint32_t*)src);
    }

    return ret;
}

int aipudrv::MemoryBase::mem_bzero(uint64_t addr, size_t size)
{
    int ret = 0;
    char* dest = nullptr;

    if (pa_to_va(addr, size, &dest) == 0)
    {
        memset(dest, 0, size);
        ret = size;
    }

    add_tracking(addr, size, MemOperationBzero, nullptr, 0, 0);
    return ret;
}

aipu_status_t aipudrv::MemoryBase::dump_file(DEV_PA_64 src, const char* name, uint32_t size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    char* va = nullptr;

    if (pa_to_va(src, size, &va) == 0)
        ret = umd_dump_file_helper(name, va, size);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(src, size, MemOperationDump, nullptr, (size == 4), *(uint32_t*)va);

    return ret;
}

aipu_status_t aipudrv::MemoryBase::load_file(DEV_PA_64 dest, const char* name, uint32_t size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    char* va = nullptr;

    if (pa_to_va(dest, size, &va) == 0)
        ret = umd_load_file_helper(name, va, size);

    if (ret == AIPU_STATUS_SUCCESS)
        add_tracking(dest, size, MemOperationReload, nullptr, (size == 4), *(uint32_t*)va);

    return ret;
}