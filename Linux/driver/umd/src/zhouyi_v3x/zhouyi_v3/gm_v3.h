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
public:
  /**
   * index 0: record the input info, DDR->GM
   * index 1: record the output info, GM->DDR
   */
  DEV_PA_64 m_gm_sync_buf_base[EM_GM_BUF_MAX] = {0};
  uint32_t m_gm_sync_buf_size[EM_GM_BUF_MAX] = {0};
  int m_gm_sync_buf_cnt[EM_GM_BUF_MAX] = {0};

public:
  aipu_status_t gm_malloc(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                          std::string &buf_name, BufferDesc *buf) override;
  bool is_gm_buffer(uint32_t idx, uint32_t buf_type) override;
  bool gm_need_remap() override;
  void gm_dynamic_switch(uint32_t core_cnt) override;
  void set_valid_sync_region(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                             BufferDesc &buf, ValidSyncBuffer &region);
  bool gm_need_sync_out() override;
  void set_valid_map_base(BufferDesc &buf) override;
  void setup_gm_sync_from_ddr(tcb_t &tcb) override;

public:
  GM_V3(JobV3 &job);
  ~GM_V3();
};
} // namespace aipudrv

#endif /* _GM_V3_H_ */