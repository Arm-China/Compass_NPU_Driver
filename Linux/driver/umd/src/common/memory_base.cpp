// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  memory_base.cpp
 * @brief AIPU User Mode Driver (UMD) memory base module implementation
 */

#include "memory_base.h"

#include <sys/syscall.h>
#include <unistd.h>

#include <cstring>
#include <mutex>

#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
MemoryBase::MemoryBase() {
  const char *mem_log_file = getenv("UMD_MEM_LOG_FILE");
  const char *mem_op_env = getenv("UMD_MEM_OP");
  const char *gm_enable = getenv("UMD_GM_ENABLE");
  int32_t mem_op_idx = 0;

  pthread_rwlock_init(&m_lock, NULL);
  pthread_rwlock_init(&m_tracking_lock, NULL);

  if (mem_op_env != nullptr) {
    mem_op_idx = atoi(mem_op_env);
    if ((mem_op_idx > 0) && (mem_op_idx & DUMP_ALL_MEM_OP_MASK))
      m_enable_mem_dump = mem_op_idx & DUMP_ALL_MEM_OP_MASK;
  }

  if (m_enable_mem_dump) {
    if (mem_log_file != nullptr)
      m_file_name = mem_log_file;

    LOG(LOG_ALERT, "memory log file: %s", m_file_name.c_str());
    m_dump_stream.open(m_file_name.c_str(),
                       std::ofstream::out | std::ofstream::trunc);
    m_dump_stream.close();
    m_dump_stream.open(m_file_name.c_str(),
                       std::ofstream::out | std::ofstream::app);
  }

  if (gm_enable != nullptr) {
    if (gm_enable[0] == 'y' || gm_enable[0] == 'Y')
      set_gm_enable(true);
    else
      set_gm_enable(false);
  }
}

MemoryBase::~MemoryBase() {
  pthread_rwlock_destroy(&m_lock);
  pthread_rwlock_destroy(&m_tracking_lock);
  m_dump_stream.close();
}

std::string MemoryBase::get_tracking_log(DEV_PA_64 pa) const {
  std::string log;

  for (uint32_t i = 0; i < m_tracking.size(); i++) {
    if ((pa >= m_tracking[i].pa) &&
        (pa < (m_tracking[i].pa + m_tracking[i].size)))
      log = m_tracking[i].log;
  }

  return log;
}

void MemoryBase::add_tracking(DEV_PA_64 pa, uint64_t size, MemOperation op,
                              const char *str, bool is_32_op,
                              uint32_t data) const {
  static uint32_t tracking_idx = 0;
  std::string log;
  MemTracking tracking = {0};
  char f_log[1024] = {0};

  if ((m_enable_mem_dump & (1 << op)) != (uint32_t)(1 << op))
    return;

  pthread_rwlock_wrlock(&m_tracking_lock);
  if (str == nullptr)
    log = get_tracking_log(pa);
  else
    log = str;

  tracking.init(pa, size, op, log);
  if (is_32_op) {
    tracking.is_32_op = true;
    tracking.data = data;
  }

  if (op == MemOperationAlloc)
    m_tracking.push_back(tracking);

  if (tracking.is_32_op) {
    snprintf(f_log, 1024, "%-6u 0x%-16lx %-14s %-9s 0x%-8lx 0x%-8x",
             tracking_idx, tracking.pa, tracking.log.c_str(),
             MemOperationStr[tracking.op], tracking.size, tracking.data);
  } else {
    snprintf(f_log, 1024, "%-6u 0x%-16lx %-14s %-9s 0x%-8lx %s    %-8ld",
             tracking_idx, tracking.pa, tracking.log.c_str(),
             MemOperationStr[tracking.op], tracking.size, "N/A", gettid());
  }
  write_line(f_log);
  tracking_idx++;
  pthread_rwlock_unlock(&m_tracking_lock);
}

void MemoryBase::dump_tracking_log_start() const {
  static bool start = false;

  pthread_rwlock_wrlock(&m_tracking_lock);
  if ((m_enable_mem_dump != 0) && !start) {
    char log[1024];
    snprintf(log, 1024,
             "===========================Memory Info "
             "Dump============================");
    write_line(log);
    snprintf(log, 1024,
             "No.    Address            Type           OP        Size       "
             "Data   Tid");
    write_line(log);
    snprintf(
        log, 1024,
        "------------------------------------------------------------------");
    write_line(log);
    start = true;
  }
  pthread_rwlock_unlock(&m_tracking_lock);
}

void MemoryBase::dump_tracking_log_end() const {
  static bool end = false;

  pthread_rwlock_wrlock(&m_tracking_lock);
  if ((m_enable_mem_dump != 0) && !end) {
    char log[1024];
    snprintf(log, 1024,
             "================================================================="
             "======");
    write_line(log);
    end = true;
  }
  pthread_rwlock_unlock(&m_tracking_lock);
}

void MemoryBase::write_line(const char *log) const {
  if (m_enable_mem_dump) {
    m_dump_stream << log << std::endl;
  }
}

std::map<DEV_PA_64, Buffer>::iterator
MemoryBase::get_allocated_buffer(std::map<DEV_PA_64, Buffer> *buffer_pool,
                                 uint64_t addr) const {
  std::map<DEV_PA_64, Buffer>::iterator iter;

  for (iter = buffer_pool->begin(); iter != buffer_pool->end(); iter++) {
    if ((addr >= iter->second.desc->pa) &&
        (addr < (iter->second.desc->pa + iter->second.desc->size +
                 iter->second.desc->binded_iova_range)))
      return iter;
  }
  return buffer_pool->end();
}

int MemoryBase::get_shared_buffer(uint64_t addr, uint64_t size,
                                  Buffer &buffer) {
  int ret = 0;

  pthread_rwlock_wrlock(&m_lock);
  auto iter =
      get_allocated_buffer((std::map<DEV_PA_64, Buffer> *)&m_allocated, addr);
  if (iter == m_allocated.end()) {
    ret = -1;
    LOG(LOG_ERR, "invalid pa addr 0x%lx/size 0x%lx is used: no buffer\n", addr,
        size);
    dump_stack();
    goto unlock;
  }

  buffer = iter->second;

unlock:
  pthread_rwlock_unlock(&m_lock);
  return ret;
}

int MemoryBase::pa_to_va(uint64_t addr, uint64_t size, char **va) const {
  int ret = 0;
  bool found = true;

  pthread_rwlock_wrlock(&m_lock);
  *va = nullptr;
  auto iter = m_allocated.end();

  for (auto item : m_allocated_buf_map) {
    iter = get_allocated_buffer(item, addr);
    if (iter == item->end()) {
      found = false;
      continue;
    } else {
      found = true;
      break;
    }
  }

  if (!found) {
    ret = -1;
    LOG(LOG_ERR, "invalid pa addr 0x%lx is used: no such a buffer\n", addr);
    dump_stack();
    goto unlock;
  }

  /* found the buffer in m_allocated/m_reserved */
  if ((addr + size) > (iter->second.desc->pa + iter->second.desc->size +
                       iter->second.desc->binded_iova_range)) {
    ret = -2;
    LOG(LOG_ERR, "invalid pa addr 0x%lx/size 0x%lx is used: out of range\n",
        addr, size);
    dump_stack();
    goto unlock;
  }

  *va = iter->second.va + addr - iter->second.desc->pa;

unlock:
  pthread_rwlock_unlock(&m_lock);
  return ret;
}

int64_t MemoryBase::mem_read(uint64_t addr, void *dest, size_t size) const {
  int64_t ret = -1;
  char *src = nullptr;
  if (size == 0)
    return 0;

  if ((pa_to_va(addr, size, &src) == 0) && (dest != nullptr)) {
    memcpy(dest, src, size);
    ret = size;
    add_tracking(addr, size, MemOperationRead, nullptr, (size == 4),
                 *(uint32_t *)src);
  }

  return ret;
}

int64_t MemoryBase::mem_write(uint64_t addr, const void *src, size_t size) {
  int64_t ret = -1;
  char *dest = nullptr;
  if (size == 0)
    return 0;

  if ((pa_to_va(addr, size, &dest) == 0) && (src != nullptr)) {
    memcpy(dest, src, size);
    ret = size;
    add_tracking(addr, size, MemOperationWrite, nullptr, (size == 4),
                 *(uint32_t *)src);
  }

  return ret;
}

int MemoryBase::mem_bzero(uint64_t addr, size_t size) {
  int ret = 0;
  char *dest = nullptr;
  if (size == 0)
    return 0;

  if (pa_to_va(addr, size, &dest) == 0) {
    memset(dest, 0, size);
    ret = size;
  }

  add_tracking(addr, size, MemOperationBzero, nullptr, 0, 0);
  return ret;
}

aipu_status_t MemoryBase::dump_file(DEV_PA_64 src, const char *name,
                                    uint32_t size) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  char *va = nullptr;

  if (pa_to_va(src, size, &va) == 0)
    ret = umd_dump_file_helper(name, va, size);

  if (ret == AIPU_STATUS_SUCCESS)
    add_tracking(src, size, MemOperationDump, nullptr, (size == 4),
                 *(uint32_t *)va);

  return ret;
}

aipu_status_t MemoryBase::load_file(DEV_PA_64 dest, const char *name,
                                    uint32_t size) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  char *va = nullptr;

  if (pa_to_va(dest, size, &va) == 0)
    ret = umd_load_file_helper(name, va, size);

  if (ret == AIPU_STATUS_SUCCESS)
    add_tracking(dest, size, MemOperationReload, nullptr, (size == 4),
                 *(uint32_t *)va);

  return ret;
}

void MemoryBase::free_bufferdesc(BufferDesc **desc) {
  if (*desc != nullptr) {
    (*desc)->reset();
    delete *desc;
    *desc = nullptr;
  }
}
} // namespace aipudrv