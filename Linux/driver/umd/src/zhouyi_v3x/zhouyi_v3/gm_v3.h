// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef _GM_V3_H_
#define _GM_V3_H_

#include "zhouyi_v3x/common/gm_v3x.h"

namespace aipudrv {
enum { EM_GM_BUF_INPUT = 0, EM_GM_BUF_OUTPUT, EM_GM_BUF_MAX };

struct ValidSyncBuffer {
  struct SyncBuffer {
    DEV_PA_64 sync_pa;
    uint32_t sync_size;
  };
  SyncBuffer valid_sync_buf[EM_GM_BUF_MAX];
};

class JobV3;
class GM_V3 : public GM_V3X {
private:
  bool m_gm_asm = false;
  std::vector<BufferDesc *> m_gm_alloc_buffer;

public:
  /**
   * index 0: record the input info, DDR->GM
   * index 1: record the output info, GM->DDR
   */
  DEV_PA_64 m_gm_sync_buf_base[EM_GM_BUF_MAX] = {0};
  uint32_t m_gm_sync_buf_size[EM_GM_BUF_MAX] = {0};
  int m_gm_sync_buf_cnt[EM_GM_BUF_MAX] = {0};
  DEV_PA_64 m_gm_map_base = 0;

private:
  void set_valid_map_base(BufferDesc &buf);
  bool gm_need_remap();
  void set_valid_sync_region(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                             BufferDesc &buf, ValidSyncBuffer &region);

public:
  aipu_status_t gm_malloc(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                          std::string &name, BufferDesc *buf) override;
  aipu_status_t setup_buffer(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                             BufferDesc *buf);
  bool is_gm_buffer(uint32_t idx, uint32_t buf_type) override;
  void gm_dynamic_switch(uint32_t core_cnt) override;
  bool gm_need_sync_out() override;
  void setup_gm_sync_from_ddr(tcb_v3::tcb_t &tcb);

public:
  GM_V3(JobV3 &job);
  ~GM_V3();
};
} // namespace aipudrv

#endif /* _GM_V3_H_ */