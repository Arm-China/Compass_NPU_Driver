// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef _GM_V3_1_H_
#define _GM_V3_1_H_

#include "zhouyi_v3x/common/gm_v3x.h"

namespace aipudrv {
enum gm_buf_type { EM_GM_BUF_INPUT = 0, EM_GM_BUF_MAX };

class JobV3_1;
class GM_V3_1 : public GM_V3X {
public:
  uint32_t m_gm_buf_map_size = 0; /* v3 no map size */
  uint32_t m_gm_buf_sync_size = 0;

public:
  aipu_status_t gm_malloc(uint32_t bss_id, uint32_t idx, uint32_t buf_type,
                          std::string &buf_name, BufferDesc *buf) override;
  bool gm_is_gm_buffer(uint32_t idx, uint32_t buf_type) override;
  bool gm_need_remap() override;
  void set_valid_map_base(BufferDesc &buf) override;

public:
  GM_V3_1(JobV3_1 &job);
  ~GM_V3_1();
};
} // namespace aipudrv

#endif /* _GM_V3_1_H_ */