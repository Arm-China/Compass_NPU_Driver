// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  umemory.h
 * @brief AIPU User Mode Driver (UMD) userspace memory module header
 */

#ifndef _UMEMORY_H_
#define _UMEMORY_H_

#include "memory_base.h"

namespace aipudrv {
namespace reg_addr {
constexpr uint32_t TSM_CMD_SCHED_CTRL = 0x0;
constexpr uint32_t TSM_CMD_SCHED_ADDR_HI = 0x8;
constexpr uint32_t TSM_CMD_SCHED_ADDR_LO = 0xC;
constexpr uint32_t TSM_BUILD_INFO = 0x14;
constexpr uint32_t TSM_STATUS = 0x18;
constexpr uint32_t TSM_CMD_TCB_NUMBER = 0x1C;
constexpr uint32_t TSM_CMD_POOL0_CONFIG = 0x800;
constexpr uint32_t TSM_CMD_POOL0_STATUS = 0x804;
constexpr uint32_t CLUSTER0_CONFIG = 0xC00;
constexpr uint32_t CLUSTER0_CTRL = 0xC04;
}; // namespace reg_addr

namespace reg_ctl {
constexpr uint32_t CREATE_CMD_POOL = 0x1;
constexpr uint32_t DESTROY_CMD_POOL = 0x2;
constexpr uint32_t DISPATCH_CMD_POOL = 0x4;
constexpr uint32_t CMD_POOL0_IDLE = (1 << 6);
constexpr uint32_t CLUSTER_PRESENT = (1 << 12);
constexpr uint32_t CLUSTER_ENABLE = (1 << 12);
}; // namespace reg_ctl

enum {
  MEM_REGION_DDR = 0,
  MEM_REGION_SRAM = 1,
  MEM_REGION_RSV = 2,
  MEM_REGION_DTCM = 3,
  MEM_REGION_MAX = 4
};

enum { MEM_REGION_GM0 = 0, MEM_REGION_GM1 = 1 };

/**
 * 1: set all ASIDs with identical base address
 * 0: set ASIDs with non-identical base address
 */
#define SHARE_ONE_ASID 0

enum {
  /* map MEM_REGION_DDR as ASID-0 */
  ASID_REGION_0 = 0,

  /**
   * map MEM_REGION_SRAM as ASID-1 if sram's size is non-zero,
   * otherwise, map MEM_REGION_DDR as ASID-1
   */
  ASID_REGION_1 = 1,

  ASID_REGION_2 = 2,
  ASID_REGION_3 = 3,
#if SHARE_ONE_ASID
  ASID_MAX = 4
#else
  ASID_MAX = 11
#endif
};

struct MemBlock {
  uint64_t base;
  uint64_t size;
  uint64_t bit_cnt;
  bool *bitmap;
};

class UMemory : public MemoryBase {
private:
  static constexpr uint64_t TOTAL_SIM_MEM_SZ = (3UL << 30); /* DDR: 3GB */
  static constexpr uint64_t SIM_SRAM_SZ = (0);              /* SRAM: 0MB */
  static constexpr uint64_t SIM_DTCM_SZ = (4 << 20);        /* DTCM: 4MB */

  MemBlock m_memblock[ASID_MAX][MEM_REGION_MAX] = {
      /* only asid0 has sram/dtcm */
      {
          {.base = 0, .size = (TOTAL_SIM_MEM_SZ - SIM_SRAM_SZ)},

          /* SRAM memory region, this base is higher than DDR base default */
          {.base = (TOTAL_SIM_MEM_SZ - SIM_SRAM_SZ), .size = SIM_SRAM_SZ},

          /* reserved for layer counter debug, absolute address */
          {.base = 0xC1000000, .size = AIPU_PAGE_SIZE},

          /* the base address for DTCM is fixed, currently only for aipu v2(X1)
           */
          {.base = 0xD0000000, .size = SIM_DTCM_SZ},

          /* {0, 0} */
      }};

  bool m_gm_mean = false;
  int m_asid_max = ASID_MAX;

private:
  uint32_t get_next_alinged_page_no(uint32_t start, uint32_t align,
                                    int mem_region = 0);
  aipu_status_t malloc_internal(uint32_t size, uint32_t align, BufferDesc *desc,
                                const char *str, uint32_t asid_mem_cfg = 0);
  aipu_status_t free_all(void);

public:
  uint64_t get_memregion_base(int32_t asid, int32_t region) {
    if (asid == ASID_REGION_0) {
      if ((region == MEM_REGION_SRAM) &&
          (m_memblock[ASID_REGION_0][MEM_REGION_SRAM].size != 0))
        return m_memblock[ASID_REGION_0][MEM_REGION_SRAM].base;
      else if (region == MEM_REGION_DTCM)
        return m_memblock[ASID_REGION_0][MEM_REGION_DTCM].base;
      else
        return m_memblock[ASID_REGION_0][MEM_REGION_DDR].base;
    }
    return m_memblock[asid][MEM_REGION_DDR].base;
  }

  uint32_t get_memregion_size(int32_t asid, int region) {
    return m_memblock[asid][region].size;
  }

public:
  void gm_init(uint32_t gm_size_idx);
  aipu_status_t malloc(uint32_t size, uint32_t align, BufferDesc **desc,
                       const char *str = nullptr, uint32_t asid_mem_cfg = 0,
                       uint32_t rsv_iova_size = 0) override;
  aipu_status_t free(BufferDesc **desc, const char *str = nullptr) override;
  aipu_status_t free_phybuffer(BufferDesc *desc,
                               const char *str = nullptr) override;
  aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size, BufferDesc **desc,
                            const char *str = nullptr,
                            uint32_t region = 0) override;

  bool invalid(uint64_t addr) const;
  bool get_info(uint64_t addr, uint64_t &base, uint32_t &size) const;
  size_t size() const {
    return m_memblock[ASID_REGION_0][MEM_REGION_DDR].size;
  };

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
  static UMemory *get_memory() {
    static UMemory mem_instance;
    mem_instance.update_from_env();
    return &mem_instance;
  }

  void update_from_env();
  virtual ~UMemory();
  UMemory(const UMemory &mem) = delete;
  UMemory &operator=(const UMemory &mem) = delete;

private:
  UMemory();
};
} // namespace aipudrv

#endif /* _UMEMORY_H_ */
