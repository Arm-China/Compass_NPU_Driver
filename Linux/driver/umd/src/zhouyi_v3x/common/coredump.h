// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  coredump.h
 * @brief AIPU debug core dump
 */

#ifndef _COREDUMP_H_
#define _COREDUMP_H_

#include <map>
#include <vector>

#include "device_base.h"
#include "elfio/elfio.hpp"
#include "elfio/elfio_note.hpp"
#include "kmd/tcb.h"
#include "memory_base.h"
#include "standard_api.h"

namespace aipudrv {
struct CoredumpBufferTec /* first address: 0x29A(12bit alignment, pa|0x29A) */
{
  uint32_t buf_valid;
  uint32_t buf_shared_address; /* [UW].CoredumpBufferShare address: 0x29A(12bit
                                  alignment, pa|0x29A) */
  uint32_t gpr[32];            /* general purpose: r0-r31, r24:tcbp */
  uint32_t t_reg[32][8];
  uint32_t p_reg[8];
  struct CpxReg {
    uint32_t addr;
    uint32_t value;
  } cp0_reg[67]; /* 45 + 10 + 12 */
  struct MemBuffer {
    uint32_t addr;
    uint32_t size; /* [UW].tec local memory size */
  } lsram_info;
  uint32_t lsram_data[0]; /* tec local memory size */
};

struct CoredumpBufferShare {
  uint32_t memory_count; /* [UW].all sm + all gm counter*/
  struct DdrInfo {
    uint32_t addr;      /* [UW].only write gm address */
    uint32_t size;      /* [UW].sm/gm memory size */
  } ddr_info[0];        /* all sm + all gm counter */
  uint32_t ddr_data[0]; /* all cores sm + gm size */
};

struct NoteNPUMisc {
  uint32_t process_id;
  uint32_t thread_id;
  uint32_t npu_arch;
  uint64_t instr_base;
  uint64_t graph_id;
  uint64_t job_id;

  void write(std::ostream &oss) {
    oss.write((char *)&process_id, sizeof(uint32_t));
    oss.write((char *)&thread_id, sizeof(uint32_t));
    oss.write((char *)&npu_arch, sizeof(uint32_t));
    oss.write((char *)&instr_base, sizeof(uint64_t));
    oss.write((char *)&graph_id, sizeof(uint64_t));
    oss.write((char *)&job_id, sizeof(uint64_t));
  }
};

struct NoteTsmRegister {
  std::vector<std::pair<uint32_t, uint32_t>> tsm_regs;

  void write(std::ostream &oss) {
    if (tsm_regs.size() == 0)
      LOG(LOG_WARN, "tsm registers size is 0!");

    for (auto reg : tsm_regs) {
      oss.write((char *)&reg.first, sizeof(uint32_t));
      oss.write((char *)&reg.second, sizeof(uint32_t));
    }
  }
};

/* note.tcb, note.stack, note.registers */
struct NoteCommonBuffer {
  char *buf = nullptr;
  uint32_t size = 0;

  void write(std::ostream &oss) {
    if (buf != nullptr && size != 0)
      oss.write((char *)buf, sizeof(tcb_t));
  }
};

/* base, offsets */
using regs_map = std::map<uint32_t, std::vector<uint32_t>>;

class JobV3X;
class Coredump {
private:
  JobV3X *m_job = nullptr;
  DeviceBase *m_dev = nullptr;
  MemoryBase *m_mem = nullptr;
  ELFIO::elfio m_elf;
  ELFIO::section *m_note = nullptr;

  static constexpr uint32_t ALIGNMENT_ADDR = 0x1000;
  static constexpr uint32_t EM_AIPU = 0x29A;
  const std::string COREDUMP_FILE_PREFIX = "aipu.coredump";

  bool m_is_initialized = false;
  uint32_t m_clusters_per_partition =
      0;                            /* assume cluster per partition are same */
  uint32_t m_cores_per_cluster = 0; /* assume cores per cluster are same */
  uint32_t m_tecs_per_core = 0;     /* assume tecs per core are same */
  uint32_t m_lsm_size = 0;
  uint32_t m_sm_size = 0;
  uint32_t m_gm_cnt = 1;
  std::vector<std::pair<DEV_PA_32, char *>> m_tecs_bufs;
  std::vector<std::pair<BufferDesc *, char *>> m_gm_bufs;
  std::vector<char *> m_sm_bufs;
  std::map<uint32_t, BufferDesc *> m_job_gm_bufs;
  BufferDesc *m_tec_buf_desc = nullptr;
  BufferDesc *m_share_buf_desc = nullptr;

  std::vector<std::pair<uint32_t, uint32_t>> get_tsm_regs(uint32_t,
                                                          uint32_t core_id);
  std::vector<regs_map> get_v3_tsm_regs(uint32_t cluster_id, uint32_t core_id);
  std::vector<regs_map> get_v3_1_tsm_regs(uint32_t cluster_id,
                                          uint32_t core_id);
  bool get_note_idx(const std::string &name, uint32_t &idx);
  void remove_note(const std::string &name);

public:
  Coredump(JobV3X *job, DeviceBase *dev);
  ~Coredump();
  Coredump(const Coredump &Coredump) = delete;
  Coredump &operator=(const Coredump &Coredump) = delete;

  aipu_status_t init();
  aipu_status_t alloc_coredump();
  void free_coredump();

  aipu_status_t do_coredump();

  void set_bin_note(const void *src, uint32_t size, const std::string &name);
  void write();

  bool is_initialized() const { return m_is_initialized; }

  const std::vector<std::pair<DEV_PA_32, char *>> &get_tecs_buf() const {
    return m_tecs_bufs;
  }

  aipu_status_t set_gm_info(uint32_t gm_id, BufferDesc *buf) {
    if (gm_id >= m_gm_cnt)
      return AIPU_STATUS_ERROR_INVALID_GM;

    m_job_gm_bufs[gm_id] = buf;
    return AIPU_STATUS_SUCCESS;
  }
};

} /* namespace aipudrv */

#endif /* _COREDUMP_H_ */
