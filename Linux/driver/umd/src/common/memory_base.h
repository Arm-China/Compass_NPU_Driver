// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  memory_base.h
 * @brief AIPU User Mode Driver (UMD) memory base module header
 */

#ifndef _MEMORY_BASE_H_
#define _MEMORY_BASE_H_

#include <math.h>
#include <pthread.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "kmd/armchina_aipu.h"
#include "standard_api.h"
#include "type.h"
#include "utils/log.h"

namespace aipudrv {
#define MB_SIZE (1 * 1024 * 1024UL)
#define AIPU_PAGE_SIZE (4 * 1024)
#define AIPU_ALIGN_BYTES(size, align) (((size) + ((align)-1)) & ~((align)-1))

#define AIPU_ASID0 (0 << 8)
#define AIPU_ASID1 (1 << 8)

struct GMRegion {
  uint32_t size;
};

struct BufferDesc {
  DEV_PA_64
  pa; /**< device physical base address, pa =  asid_base + align_asid_pa */
  DEV_PA_64 align_asid_pa; /**< the offset PA relative to ASID base addr */
  DEV_PA_64 asid_base;     /**< ASID base PA addr */
  uint64_t size;           /**< buffer size */
  uint64_t req_size;       /**< requested size (<= buffer size) */
  uint64_t exec_id;
  uint64_t iova_size;         /**< whole reserved iova size */
  uint64_t binded_iova_range; /**< max binded iova range to access */
  uint64_t dev_offset;
  uint32_t ram_region = AIPU_BUF_REGION_DEFAULT;
  int32_t asid; /**< ASID index */

  void init(DEV_PA_64 _asid_base, DEV_PA_64 _pa, uint64_t _size,
            uint64_t _req_size, uint64_t _offset = 0,
            uint32_t _asid_ram_region = 0, uint64_t _exec_id = 0,
            uint64_t _iova_size = 0) {
    pa = _pa;
    asid_base = _asid_base;
    align_asid_pa = _pa - asid_base;
    asid = (_asid_ram_region >> 8) & 0xff;
    size = _size;
    req_size = _req_size;
    dev_offset = _offset;
    ram_region = _asid_ram_region & 0xff;
    exec_id = _exec_id;
    iova_size = _iova_size;
  }

  void reset() {
    pa = 0;
    align_asid_pa = 0;
    asid_base = 0;
    size = 0;
    req_size = 0;
    dev_offset = 0;
    exec_id = 0;
    iova_size = 0;
    binded_iova_range = 0;
    ram_region = AIPU_BUF_REGION_DEFAULT;
  }
};

struct Buffer {
  std::atomic_int refcnt{0};
  char *va;
  BufferDesc *desc;

  Buffer() {
    va = nullptr;
    desc = nullptr;
    refcnt = 0;
  }

  Buffer &operator=(const Buffer &_Buffer) {
    this->va = _Buffer.va;
    this->desc = _Buffer.desc;
    this->refcnt = _Buffer.refcnt.load();

    return *this;
  }

  void init(char *_va, BufferDesc *_desc) {
    va = _va;
    desc = _desc;
    refcnt = 1;
  }

  void reset() {
    va = nullptr;
    desc = nullptr;
    refcnt = 0;
  }

  void ref_get() { refcnt++; }

  void ref_put() {
    refcnt--;
    LOG(LOG_DEBUG, "Buffer.refcnt=%d, buffer_pa=%lx", refcnt.load(),
        this->desc->pa);
  }

  int get_Buffer_refcnt() { return this->refcnt.load(); }
};

enum MemOperation {
  MemOperationAlloc,
  MemOperationFree,
  MemOperationSub,
  MemOperationRead,
  MemOperationWrite,
  MemOperationBzero,
  MemOperationDump,
  MemOperationReload,
  MemOperationCnt,
};

struct MemTracking {
  DEV_PA_64 pa;
  uint64_t size;
  MemOperation op;
  std::string log;
  bool is_32_op;
  uint32_t data;
  void init(DEV_PA_64 _pa, uint64_t _size, MemOperation _op, const char *str) {
    pa = _pa;
    size = _size;
    op = _op;
    is_32_op = false;
    log = std::string(str);
  }
  void init(DEV_PA_64 _pa, uint64_t _size, MemOperation _op,
            const std::string &str) {
    pa = _pa;
    size = _size;
    op = _op;
    is_32_op = false;
    log = str;
  }
};

class DeviceBase;
class MemoryBase {
private:
  const char *MemOperationStr[MemOperationCnt] = {
      "alloc", "free", "sub", "read", "write", "bzero", "dump", "reload",
  };

private:
  mutable std::ofstream m_dump_stream;
  mutable std::vector<MemTracking> m_tracking;
  mutable pthread_rwlock_t m_tracking_lock;
  uint32_t m_enable_mem_dump = DUMP_MEM_OP_MASK;
  std::string m_file_name = "mem_info.log";
  std::vector<DEV_PA_64> m_asid_base_vec;
  GMRegion m_gm_region[2] = {0};
  bool m_gm_enable = true;
  uint32_t m_asid1 = 1; /* x2 has two mode: 0 for multi-model parallel(default),
                           1 for single model */

  /* only for X1 */
  DEV_PA_64 m_dtcm_base = 0;
  uint32_t m_dtcm_size = 0;

protected:
  DeviceBase *m_dev = nullptr;
  std::map<DEV_PA_64, Buffer> m_allocated;
  std::map<DEV_PA_64, Buffer> m_reserved;
  std::map<DEV_PA_64, Buffer> *m_allocated_buf_map[2] = {
      (std::map<DEV_PA_64, Buffer> *)&m_allocated,
      (std::map<DEV_PA_64, Buffer> *)&m_reserved};
  mutable pthread_rwlock_t m_lock;

private:
  std::string get_tracking_log(DEV_PA_64 pa) const;

protected:
  uint64_t get_page_cnt(uint64_t bytes) const {
    return ceil((double)bytes / AIPU_PAGE_SIZE);
  }

  uint64_t get_page_no(DEV_PA_64 pa) const {
    return floor((double)pa / AIPU_PAGE_SIZE);
  }

  int64_t mem_read(uint64_t addr, void *dest, size_t size) const;
  int64_t mem_write(uint64_t addr, const void *src, size_t size);
  std::map<aipudrv::DEV_PA_64, aipudrv::Buffer>::iterator
  get_allocated_buffer(std::map<DEV_PA_64, Buffer> *buffer_pool,
                       uint64_t addr) const;

public:
  void add_tracking(DEV_PA_64 pa, uint64_t size, MemOperation op,
                    const char *str, bool is_32_op, uint32_t data) const;
  void dump_tracking_log_start() const;
  void dump_tracking_log_end() const;
  void write_line(const char *log) const;
  int mem_bzero(uint64_t addr, size_t size);

  void set_dev(DeviceBase *dev) { m_dev = dev; }

  void set_asid_base(int i, DEV_PA_64 base) {
    if (i == 0)
      m_asid_base_vec.clear();

    m_asid_base_vec.push_back(base);
  }

  void reset_asid_base(uint32_t i, DEV_PA_64 base) {
    if (i < m_asid_base_vec.size())
      m_asid_base_vec[i] = base;
  }

  DEV_PA_64 get_asid_base(int i) { return m_asid_base_vec[i]; }

  void set_asid1(uint32_t asid1) { m_asid1 = asid1; }

  uint32_t get_asid1() { return m_asid1; }

  void set_dtcm_info(DEV_PA_64 base, uint32_t size) {
    m_dtcm_base = base;
    m_dtcm_size = size;
  }

  DEV_PA_64 get_dtcm_base() { return m_dtcm_base; }

  uint32_t get_dtcm_size() { return m_dtcm_size; }

  void set_gm_size(int i, uint32_t size) { m_gm_region[i].size = size; }

  uint32_t get_gm_size(int i = 0) { return m_gm_region[i].size; }

  void set_gm_enable(bool enable) { m_gm_enable = enable; }

  bool is_gm_enable() { return m_gm_enable; }

  bool is_both_gm_region_enable() {
    return m_gm_enable && (get_gm_size(0) != 0) && (get_gm_size(1) != 0);
  }

public:
  /* Interfaces */
  int pa_to_va(uint64_t addr, uint64_t size, char **va) const;
  int get_shared_buffer(uint64_t addr, uint64_t size, Buffer &buffer);
  virtual aipu_status_t
  malloc(uint32_t size, uint32_t align, BufferDesc **buf,
         const char *str = nullptr,
         uint32_t asid_qos_cfg = AIPU_ASID0 | AIPU_MEM_REGION_DEFAULT, /*0*/
         uint32_t rsv_iova_size = 0) = 0;
  virtual aipu_status_t free(BufferDesc **buf, const char *str = nullptr) = 0;
  virtual void free_bufferdesc(BufferDesc **desc);
  virtual aipu_status_t free_phybuffer(BufferDesc *desc,
                                       const char *str = nullptr) = 0;
  virtual aipu_status_t reserve_mem(DEV_PA_32 addr, uint32_t size,
                                    BufferDesc **desc,
                                    const char *str = nullptr,
                                    uint32_t region = 0) {
    return AIPU_STATUS_SUCCESS;
  };
  virtual int64_t read(uint64_t addr, void *dest, size_t size) const = 0;
  virtual int64_t write(uint64_t addr, const void *src, size_t size) = 0;
  virtual int64_t zeroize(uint64_t addr, size_t size) = 0;
  virtual aipu_status_t dump_file(DEV_PA_64 src, const char *name,
                                  uint32_t size);
  virtual aipu_status_t load_file(DEV_PA_64 dest, const char *name,
                                  uint32_t size);
  virtual void gm_init(uint32_t gm_size_idx) {}
  virtual uint32_t get_lm_size() { return 0; }
  virtual uint32_t get_sm_size() { return 0; }
  virtual aipu_status_t refresh_binded_iova(BufferDesc &desc, DEV_PA_64 pa,
                                            uint64_t size) {
    return AIPU_STATUS_SUCCESS;
  }

  int write32(DEV_PA_64 dest, uint32_t src) {
    return mem_write(dest, (char *)&src, sizeof(src));
  }
  int read32(uint32_t *desc, DEV_PA_64 src) {
    return mem_read(src, (char *)desc, sizeof(*desc));
  }

public:
  MemoryBase();
  virtual ~MemoryBase();
  MemoryBase(const MemoryBase &mem) = delete;
  MemoryBase &operator=(const MemoryBase &mem) = delete;
};
} // namespace aipudrv

#endif /* _MEMORY_BASE_H_ */
