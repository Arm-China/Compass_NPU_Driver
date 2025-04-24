// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  ukmemory.cpp
 * @brief AIPU User Mode Driver (UMD) user & kernel space memory module
 * implementation
 */

#include "ukmemory.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>

#include "kmd/armchina_aipu.h"
#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
UKMemory::UKMemory(int fd) : MemoryBase() { m_fd = fd; }

UKMemory::~UKMemory() { free_all(); }

aipu_status_t UKMemory::malloc(uint32_t size, uint32_t align, BufferDesc **desc,
                               const char *str, uint32_t asid_mem_cfg) {
  int kret = 0;
  Buffer buf;
  char *ptr = nullptr;
  unsigned long cmd = AIPU_IOCTL_REQ_BUF, free_cmd = AIPU_IOCTL_FREE_BUF;
  aipu_buf_request buf_req = {0};
  DEV_PA_64 base = 0;

  buf_req.bytes = size;
  buf_req.align_in_page = (align == 0) ? 1 : align;

  /* asid_mem_cfg, bit[15:8]: asid idx, bit[7:0] buf region */
  buf_req.asid = (asid_mem_cfg & 0xff00) >> 8;
  buf_req.region = asid_mem_cfg & 0xff;

  if (*desc == nullptr) {
    *desc = new BufferDesc;
    (*desc)->reset();
  }

  if (size == 0)
    return AIPU_STATUS_ERROR_INVALID_SIZE;

  /* specific command for tcb alloc/free */
  if ((str != nullptr) && (!strncmp(str, "tcbs", 4)))
    buf_req.data_type = AIPU_MM_DATA_TYPE_TCB;

  kret = ioctl(m_fd, cmd, &buf_req);
  if (kret != 0) {
    LOG(LOG_ALERT, "alloc buffer: size 0x%x [fail]", size);
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  }

  ptr = (char *)mmap(NULL, buf_req.desc.bytes, PROT_READ | PROT_WRITE,
                     MAP_SHARED, m_fd, buf_req.desc.dev_offset);
  if (ptr == MAP_FAILED) {
    ioctl(m_fd, free_cmd, &buf_req.desc);
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  }

  /**
   * adjust PA filled into RO section.
   * depend on condition:
   * - ASID low 32-bits address is non-zero
   * - buffer alocated from DTCM region, only for aipu v2
   */
  if (buf_req.desc.region == AIPU_BUF_REGION_DTCM)
    base = get_dtcm_base() - 0xD0000000;
  else
    base = get_asid_base(buf_req.desc.asid);

  (*desc)->init(base, buf_req.desc.pa, buf_req.desc.bytes, size,
                buf_req.desc.dev_offset, buf_req.desc.region);

  buf.init(ptr, *desc);
  pthread_rwlock_wrlock(&m_lock);
  m_allocated[buf_req.desc.pa] = buf;
  pthread_rwlock_unlock(&m_lock);
  add_tracking(buf_req.desc.pa, size, MemOperationAlloc, str, false, 0);

  return AIPU_STATUS_SUCCESS;
}

aipu_status_t UKMemory::free(BufferDesc **desc, const char *str) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  int kret = 0;
  aipu_buf_desc kdesc;
  unsigned long free_cmd = AIPU_IOCTL_FREE_BUF;
  DEV_PA_64 pa = 0;
  uint64_t size = 0;

  if (*desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  pthread_rwlock_wrlock(&m_lock);
  auto iter = m_allocated.find((*desc)->pa);
  if ((iter == m_allocated.end()) ||
      (iter->second.desc->size != (*desc)->size)) {
    ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
    goto unlock;
  }

  iter->second.ref_put();
  if (iter->second.get_Buffer_refcnt() == 0) {
    kdesc.pa = (*desc)->pa;
    kdesc.bytes = (*desc)->size;
    munmap(iter->second.va, kdesc.bytes);
    kret = ioctl(m_fd, free_cmd, &kdesc);
    if (kret != 0) {
      LOG(LOG_ERR, "free buffer 0x%lx [fail]", (*desc)->pa);
      ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
      goto unlock;
    }

    LOG(LOG_DEBUG, "free buffer_pa=%lx", iter->second.desc->pa);
    m_allocated.erase((*desc)->pa);
    pa = (*desc)->pa;
    size = (*desc)->size;
    (*desc)->reset();
    delete *desc;
    *desc = nullptr;
  }

unlock:
  pthread_rwlock_unlock(&m_lock);

  if (ret == AIPU_STATUS_SUCCESS)
    add_tracking(pa, size, MemOperationFree, str, false, 0);

  return ret;
}

aipu_status_t UKMemory::free_phybuffer(BufferDesc *desc, const char *str) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  int kret = 0;
  aipu_buf_desc kdesc;
  unsigned long free_cmd = AIPU_IOCTL_FREE_BUF;
  DEV_PA_64 pa = 0;
  uint64_t size = 0;

  if (desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  pthread_rwlock_wrlock(&m_lock);
  auto iter = m_allocated.find(desc->pa);
  if ((iter == m_allocated.end()) || (iter->second.desc->size != desc->size)) {
    ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
    goto unlock;
  }

  iter->second.ref_put();
  if (iter->second.get_Buffer_refcnt() == 0) {
    kdesc.pa = desc->pa;
    kdesc.bytes = desc->size;
    munmap(iter->second.va, kdesc.bytes);
    kret = ioctl(m_fd, free_cmd, &kdesc);
    if (kret != 0) {
      LOG(LOG_ERR, "free buffer 0x%lx [fail]", desc->pa);
      ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
      goto unlock;
    }

    LOG(LOG_DEBUG, "free buffer_pa=%lx", iter->second.desc->pa);
    m_allocated.erase(desc->pa);
    pa = desc->pa;
    size = desc->size;
  }

unlock:
  pthread_rwlock_unlock(&m_lock);

  if (ret == AIPU_STATUS_SUCCESS)
    add_tracking(pa, size, MemOperationFree, str, false, 0);

  return ret;
}

aipu_status_t UKMemory::reserve_mem(DEV_PA_32 addr, uint32_t size,
                                    BufferDesc **desc, const char *str) {
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t UKMemory::free_all(void) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  BufferDesc *desc = nullptr;
  aipu_buf_desc kdesc;
  int kret = 0;
  DEV_PA_64 pa = 0;
  uint64_t size = 0;

  pthread_rwlock_wrlock(&m_lock);
  for (auto iter = m_allocated.begin(); iter != m_allocated.end(); iter++) {
    desc = iter->second.desc;
    kdesc.pa = desc->pa;
    pa = desc->pa;
    kdesc.bytes = desc->size;
    size = desc->size;
    munmap(iter->second.va, kdesc.bytes);
    kret = ioctl(m_fd, AIPU_IOCTL_FREE_BUF, &kdesc);
    if (kret != 0) {
      LOG(LOG_ERR, "free buffer 0x%lx [fail]", desc->pa);
      ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
    }
    desc->reset();
    delete desc;
    desc = nullptr;
    pthread_rwlock_unlock(&m_lock);
    add_tracking(pa, size, MemOperationFree, "normal", false, 0);
    pthread_rwlock_wrlock(&m_lock);
  }

  m_allocated.clear();
  pthread_rwlock_unlock(&m_lock);
  return ret;
}
} // namespace aipudrv