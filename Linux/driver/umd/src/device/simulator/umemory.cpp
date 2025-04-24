// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  umemory.cpp
 * @brief AIPU User Mode Driver (UMD) userspace memory module implementation
 */

#include "umemory.h"

#include <unistd.h>

#include <cstring>

#include "utils/helper.h"
#include "utils/log.h"

namespace aipudrv {
UMemory::UMemory() : MemoryBase(), sim_aipu::IMemEngine() {
  /**
   * GM allocate strategy:
   * env[UMD_GM_MEAN] == 'y/Y', mean
   * env[UMD_GM_MEAN] != 'y/Y', all GM region for GM region0
   */
  const char *gm_mean = getenv("UMD_GM_MEAN");

  /**
   * ASID number, control how many ASID regions which are used in simulation
   * scenario. scope [ASID_MAX, UMD_ASID_NUM]
   */
  const char *asid_num = getenv("UMD_ASID_NUM");
  if (asid_num != nullptr) {
    uint32_t num = atoi(asid_num);

    if (num < ASID_MAX) {
      num = ASID_MAX;
    } else if (num > 101) {
      num = 101;
      LOG(LOG_WARN, "ASID num is beyond the scope, use max num 101 (~300GB)\n");
    }

    m_asid_max = num;
  }

  /**
   * default mem region config
   * aipu v3: default 4MB
   * other aipu version: no GM
   */
  for (int i = 0; i < MEM_REGION_MAX; i++) {
    if (m_memblock[ASID_REGION_0][i].size >= AIPU_PAGE_SIZE) {
      m_memblock[ASID_REGION_0][i].bit_cnt =
          m_memblock[ASID_REGION_0][i].size / AIPU_PAGE_SIZE;
      m_memblock[ASID_REGION_0][i].bitmap =
          new bool[m_memblock[ASID_REGION_0][i].bit_cnt];
      memset(m_memblock[ASID_REGION_0][i].bitmap, true,
             m_memblock[ASID_REGION_0][i].bit_cnt);
    }
  }
  set_asid_base(0, m_memblock[ASID_REGION_0][0].base);

  for (int i = 0; i < MEM_REGION_MAX; i++) {
    LOG(LOG_ALERT, "ASID 0: mem region [%2d]: base=0x%.8lx, size=0x%lx", i,
        m_memblock[ASID_REGION_0][i].base, m_memblock[ASID_REGION_0][i].size);
  }

  /**
   * limit each ASID region to 3GB, and base address is 4GB alighment.
   */
  for (int region = ASID_REGION_1; region < m_asid_max; region++) {
    m_memblock[region][0].base = static_cast<uint64_t>(region)
                                 << 32;     // (region | 1ul) << 32;
    m_memblock[region][0].size = 3ul << 30; // 3GB
    m_memblock[region][0].bit_cnt = m_memblock[region][0].size / AIPU_PAGE_SIZE;
    m_memblock[region][0].bitmap = new bool[m_memblock[region][0].bit_cnt];
    memset(m_memblock[region][0].bitmap, true, m_memblock[region][0].bit_cnt);
    set_asid_base(region, m_memblock[region][0].base);

    LOG(LOG_ALERT, "ASID %d: mem region [ 0]: base=0x%.12lx, size=0x%lx",
        region, m_memblock[region][0].base, m_memblock[region][0].size);
  }

  if (gm_mean != nullptr) {
    if (gm_mean[0] == 'y' || gm_mean[0] == 'Y')
      m_gm_mean = true;
    else
      m_gm_mean = false;
  }
}

void UMemory::update_from_env() {
  /**
   * DTCM size: 1, 2, 4, 8, 16, 32(MB)
   * aipu v2(X1): support
   * other aipu version: current non-support
   */
  const char *dtcm_sz = getenv("UMD_DTCM_SZ");
  if (dtcm_sz != nullptr) {
    uint32_t size = atoi(dtcm_sz);

    if (size > 32) {
      size = 32;
      LOG(LOG_WARN, "DTCM size is beyond the scope, use max size 32MB\n");
    }

    uint64_t new_size_in_mb = size * MB_SIZE;
    if (m_memblock[ASID_REGION_0][MEM_REGION_DTCM].size != new_size_in_mb) {
      m_memblock[ASID_REGION_0][MEM_REGION_DTCM].size = new_size_in_mb;
      uint64_t bit_cnt = new_size_in_mb / AIPU_PAGE_SIZE;
      m_memblock[ASID_REGION_0][MEM_REGION_DTCM].bit_cnt = bit_cnt;
      delete[] m_memblock[ASID_REGION_0][MEM_REGION_DTCM].bitmap;
      m_memblock[ASID_REGION_0][MEM_REGION_DTCM].bitmap = new bool[bit_cnt];
      memset(m_memblock[ASID_REGION_0][MEM_REGION_DTCM].bitmap, true, bit_cnt);
    }
  }
}

UMemory::~UMemory() {
  free_all();
  for (int i = 0; i < MEM_REGION_MAX; i++) {
    if (m_memblock[ASID_REGION_0][i].bitmap) {
      delete[] m_memblock[ASID_REGION_0][i].bitmap;
      m_memblock[ASID_REGION_0][i].bitmap = nullptr;
    }
  }

  for (int region = ASID_REGION_1; region < m_asid_max; region++) {
    if (m_memblock[region][0].bitmap) {
      delete[] m_memblock[region][0].bitmap;
      m_memblock[region][0].bitmap = nullptr;
    }
  }
}

void UMemory::gm_init(uint32_t gm_size) {
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
  if (is_gm_enable()) {
    if (m_gm_mean) {
      set_gm_size(MEM_REGION_GM0, gm_size >> 1);
      set_gm_size(MEM_REGION_GM1, gm_size >> 1);
    } else {
      set_gm_size(MEM_REGION_GM0, gm_size);
      set_gm_size(MEM_REGION_GM1, 0);
    }
  }
}

uint32_t UMemory::get_next_alinged_page_no(uint32_t start, uint32_t align,
                                           int asid_mem_region) {
  uint32_t no = start;
  uint32_t asid = (asid_mem_region >> 8) & 0xff;
  uint32_t mem_region = asid_mem_region & 0xff;

  while (no < m_memblock[asid][mem_region].bit_cnt) {
    uint64_t pa = m_memblock[asid][mem_region].base + no * AIPU_PAGE_SIZE;
    if ((pa % (align * AIPU_PAGE_SIZE)) == 0)
      return no;
    no++;
  }
  return m_memblock[asid][mem_region].bit_cnt;
}

aipu_status_t UMemory::malloc_internal(uint32_t size, uint32_t align,
                                       BufferDesc *desc, const char *str,
                                       uint32_t asid_mem_region) {
  aipu_status_t ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  uint64_t malloc_size, malloc_page = 0, i = 0;
  uint32_t asid = (asid_mem_region >> 8) & 0xff;
  uint32_t mem_region = asid_mem_region & 0xff;
  Buffer buf;

  if ((size > m_memblock[asid][mem_region].size) || (size == 0))
    return AIPU_STATUS_ERROR_INVALID_SIZE;

  if (align == 0)
    align = 1;

  malloc_page = get_page_cnt(size);
  malloc_size = malloc_page * AIPU_PAGE_SIZE;

  if (malloc_page > m_memblock[asid][mem_region].bit_cnt)
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

  pthread_rwlock_wrlock(&m_lock);
  i = get_next_alinged_page_no(0, align, (asid << 8) | mem_region);
  while ((i + malloc_page) < m_memblock[asid][mem_region].bit_cnt) {
    uint64_t j = i;
    for (; j < (i + malloc_page); j++) {
      if (m_memblock[asid][mem_region].bitmap[j] == false) {
        i = get_next_alinged_page_no(j + 1, align, (asid << 8) | mem_region);
        break;
      }
    }

    if (j == i + malloc_page) {
      desc->init(get_asid_base(asid),
                 m_memblock[asid][mem_region].base + i * AIPU_PAGE_SIZE,
                 malloc_size, size, 0, (asid << 8) | mem_region);
      buf.init(new char[malloc_size], desc);
      memset(buf.va, 0, malloc_size);
      m_allocated[desc->pa] = buf;
      LOG(LOG_INFO, "m_allocated.size=%ld, buffer_pa=%lx", m_allocated.size(),
          desc->pa);
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

aipu_status_t UMemory::malloc(uint32_t size, uint32_t align, BufferDesc **desc,
                              const char *str, uint32_t asid_mem_cfg) {
  aipu_status_t ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  uint32_t mem_region = asid_mem_cfg & 0xff;
  uint32_t asid = (asid_mem_cfg >> 8) & 0xff;

  if (*desc == nullptr) {
    *desc = new BufferDesc;
    (*desc)->reset();
  }

  if (SHARE_ONE_ASID == 1)
    asid = ASID_REGION_0;

  if (asid == ASID_REGION_0 && mem_region == MEM_REGION_SRAM) {
    if (m_memblock[asid][mem_region].size == 0)
      mem_region = MEM_REGION_DDR;
  }

  if (mem_region != MEM_REGION_DDR)
    ret = malloc_internal(size, align, *desc, str, (asid << 8) | mem_region);

  if (ret != AIPU_STATUS_SUCCESS) {
    for (uint32_t asid_idx = asid; asid_idx < ASID_MAX; asid_idx++) {
      ret = malloc_internal(size, align, *desc, str,
                            (asid_idx << 8) | MEM_REGION_DDR);
      if (ret == AIPU_STATUS_SUCCESS)
        break;
    }
  }

  return ret;
}

aipu_status_t UMemory::free(BufferDesc **desc, const char *str) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t b_start, b_end;
  bool reserve_mem_flag = false;
  DEV_PA_64 pa = 0;
  uint64_t size = 0;

  if (*desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  if ((*desc)->size == 0)
    return AIPU_STATUS_ERROR_INVALID_SIZE;

  pthread_rwlock_wrlock(&m_lock);
  auto iter = m_allocated.find((*desc)->pa);
  if (iter == m_allocated.end()) {
    iter = m_reserved.find((*desc)->pa);
    if (iter == m_reserved.end()) {
      ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
      goto unlock;
    }
    reserve_mem_flag = true;
  }

  iter->second.ref_put();
  if (iter->second.get_Buffer_refcnt() == 0) {
    int mem_region = (*desc)->ram_region;
    int asid = (*desc)->asid;
    b_start = (iter->second.desc->pa - m_memblock[asid][mem_region].base) /
              AIPU_PAGE_SIZE;
    b_end = b_start + iter->second.desc->size / AIPU_PAGE_SIZE;
    for (uint64_t i = b_start; i < b_end; i++)
      m_memblock[asid][mem_region].bitmap[i] = true;

    LOG(LOG_INFO, "free buffer_pa=%lx\n", iter->second.desc->pa);
    delete[] iter->second.va;
    iter->second.va = nullptr;
    if (!reserve_mem_flag) {
      m_allocated.erase((*desc)->pa);
    } else {
      m_reserved.erase((*desc)->pa);
      reserve_mem_flag = false;
    }
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

aipu_status_t UMemory::free_phybuffer(BufferDesc *desc, const char *str) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t b_start, b_end;
  bool reserve_mem_flag = false;
  DEV_PA_64 pa = 0;
  uint64_t size = 0;

  if (desc == nullptr)
    return AIPU_STATUS_ERROR_NULL_PTR;

  if (desc->size == 0)
    return AIPU_STATUS_ERROR_INVALID_SIZE;

  pthread_rwlock_wrlock(&m_lock);
  auto iter = m_allocated.find(desc->pa);
  if (iter == m_allocated.end()) {
    iter = m_reserved.find(desc->pa);
    if (iter == m_reserved.end()) {
      ret = AIPU_STATUS_ERROR_BUF_FREE_FAIL;
      goto unlock;
    }
    reserve_mem_flag = true;
  }

  iter->second.ref_put();
  if (iter->second.get_Buffer_refcnt() == 0) {
    int mem_region = desc->ram_region;
    int asid = desc->asid;
    b_start = (iter->second.desc->pa - m_memblock[asid][mem_region].base) /
              AIPU_PAGE_SIZE;
    b_end = b_start + iter->second.desc->size / AIPU_PAGE_SIZE;
    for (uint64_t i = b_start; i < b_end; i++)
      m_memblock[asid][mem_region].bitmap[i] = true;

    LOG(LOG_INFO, "free buffer_pa=%lx\n", iter->second.desc->pa);
    delete[] iter->second.va;
    iter->second.va = nullptr;
    if (!reserve_mem_flag) {
      m_allocated.erase(desc->pa);
    } else {
      m_reserved.erase(desc->pa);
      reserve_mem_flag = false;
    }
    pa = desc->pa;
    size = desc->size;
  }

unlock:
  pthread_rwlock_unlock(&m_lock);
  if (ret == AIPU_STATUS_SUCCESS)
    add_tracking(pa, size, MemOperationFree, str, false, 0);

  return ret;
}

aipu_status_t UMemory::reserve_mem(DEV_PA_32 addr, uint32_t size,
                                   BufferDesc **desc, const char *str) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t malloc_size, malloc_page = 0, i = 0;
  Buffer buf;
  uint32_t mem_region = MEM_REGION_DDR;
  uint32_t asid = ASID_REGION_0;

  if (size == 0)
    return AIPU_STATUS_ERROR_INVALID_SIZE;

  if (*desc == nullptr) {
    *desc = new BufferDesc;
    (*desc)->reset();
  }

  pthread_rwlock_wrlock(&m_lock);

  if (addr > m_memblock[asid][MEM_REGION_SRAM].base &&
      m_memblock[asid][MEM_REGION_SRAM].size != 0) {
    mem_region = MEM_REGION_SRAM;
    asid = ASID_REGION_1;
  }

  /* clear bitmap for reserved memory page */
  malloc_page = get_page_cnt(size);
  malloc_size = malloc_page * AIPU_PAGE_SIZE;
  if (malloc_page > m_memblock[asid][mem_region].bit_cnt)
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

  (*desc)->init(get_asid_base(asid), addr, malloc_size, size, 0,
                (asid << 8) | mem_region);
  buf.desc = *desc;
  buf.va = new char[size];
  memset(buf.va, 0, size);
  buf.ref_get();
  m_reserved[(*desc)->pa] = buf;

  i = get_next_alinged_page_no((addr - get_asid_base(asid)) >> 12, 1,
                               (asid << 8) | mem_region);
  for (uint32_t j = 0; j < malloc_page; j++)
    m_memblock[asid][mem_region].bitmap[i + j] = false;

  pthread_rwlock_unlock(&m_lock);

  if (ret == AIPU_STATUS_SUCCESS)
    add_tracking((*desc)->pa, (*desc)->size, MemOperationAlloc, str, false, 0);

  return ret;
}

bool UMemory::invalid(uint64_t addr) const {
  bool found = true;

  pthread_rwlock_rdlock(&m_lock);
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
  pthread_rwlock_unlock(&m_lock);

  return (!found) ? true : false;
}

bool UMemory::get_info(uint64_t addr, uint64_t &base, uint32_t &size) const {
  bool found = true;

  pthread_rwlock_rdlock(&m_lock);
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

  if (found) {
    base = iter->second.desc->pa;
    size = iter->second.desc->size;
  }
  pthread_rwlock_unlock(&m_lock);

  return found;
}

aipu_status_t UMemory::free_all(void) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t b_start = 0, b_end = 0;
  const char *promt = nullptr;
  DEV_PA_64 pa = 0;
  uint64_t size = 0;

  pthread_rwlock_wrlock(&m_lock);
  for (auto mem_map : m_allocated_buf_map) {
    for (auto iter = mem_map->begin(); iter != mem_map->end(); iter++) {
      BufferDesc *desc = iter->second.desc;
      int mem_region = desc->ram_region;
      int asid = desc->asid;

      if (desc->size == 0) {
        LOG(LOG_ALERT, "this buffer size=0, check it\n");
        continue;
      }

      b_start = (desc->pa - m_memblock[asid][mem_region].base) / AIPU_PAGE_SIZE;
      b_end = b_start + desc->size / AIPU_PAGE_SIZE;
      for (uint64_t i = b_start; i < b_end; i++)
        m_memblock[asid][mem_region].bitmap[i] = true;

      LOG(LOG_INFO, "free buffer_pa=%lx\n", desc->pa);
      delete[] iter->second.va;
      iter->second.va = nullptr;
      pa = desc->pa;
      size = desc->size;
      desc->reset();
      delete desc;
      desc = nullptr;
      (mem_map == &m_reserved) ? promt = "rsv" : promt = "normal";
      pthread_rwlock_unlock(&m_lock);
      add_tracking(pa, size, MemOperationFree, promt, false, 0);
      pthread_rwlock_wrlock(&m_lock);
    }

    mem_map->clear();
  }

  pthread_rwlock_unlock(&m_lock);
  return ret;
}
} // namespace aipudrv
