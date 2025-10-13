// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  ukmemory.h
 * @brief AIPU User Mode Driver (UMD) user & kernel space memory module header
 */

#ifndef _UKMEMORY_H_
#define _UKMEMORY_H_

#include <pthread.h>

#include <map>

#include "memory_base.h"

#define TSM_PAGE_SELECTION_CONTROL 0x1F00
#define TSM_LOCAL_MEMORY_FEATURE 0x2848
#define TSM_SHARE_MEMORY_FEATURE 0x284C
#define TSM_CORE_STATUS 0x3004
#define TSM_COMMAND_POOL_STATUS 0x804

#define EN_SELECT(n) (n << 12)
#define EN_CLUSTER(n) (n << 4)
#define EN_CORE(n) (n << 0)
#define CORE_IDLE (1 << 4)

#define X2_CMDPOOL_EXCEPTION (1 << 2 | 1 << 3 | 1 << 4)
#define X3_CMDPOOL_EXCEPTION (1 << 4 | 1 << 5)
#define V3X_CMDPOOL_IDLE (1 << 6)

#define X2_CLUSTER_IDLE (1 << 5)
#define X2_BUS_IDLE (1 << 6)
#define X3_CLUSTER_IDLE (1 << 4)
#define X3_BUS_IDLE (1 << 5)

#define X1_DEV_EXCEPTION (1 << 2)
#define X1_DEV_IDLE (1 << 17)

namespace aipudrv {
class UKMemory : public MemoryBase {
private:
  int m_fd = 0;

private:
  aipu_status_t free_all(void);

public:
  aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc **desc,
                       const char *str = nullptr, uint32_t asid_mem_cfg = 0,
                       uint32_t rsv_ivoa_size = 0) override;
  aipu_status_t free(BufferDesc **desc, const char *str = nullptr) override;
  aipu_status_t free_phybuffer(BufferDesc *desc,
                               const char *str = nullptr) override;
  uint32_t get_lm_size() override; /* not thread-safe */
  uint32_t get_sm_size() override; /* not thread-safe */
  aipu_status_t refresh_binded_iova(BufferDesc &desc, DEV_PA_64 pa,
                                    uint64_t size) override;
  int64_t read(uint64_t addr, void *dest, size_t size) const override {
    return mem_read(addr, dest, size);
  };
  int64_t write(uint64_t addr, const void *src, size_t size) override {
    return mem_write(addr, src, size);
  };
  int64_t zeroize(uint64_t addr, size_t size) override {
    return mem_bzero(addr, size);
  };

public:
  static UKMemory *get_memory(int fd) {
    static UKMemory mem_instance(fd);
    mem_instance.m_fd = fd;
    return &mem_instance;
  }

  virtual ~UKMemory();
  UKMemory(const UKMemory &mem) = delete;
  UKMemory &operator=(const UKMemory &mem) = delete;

private:
  UKMemory(int fd);
};
} // namespace aipudrv

#endif /* _UKMEMORY_H_ */
