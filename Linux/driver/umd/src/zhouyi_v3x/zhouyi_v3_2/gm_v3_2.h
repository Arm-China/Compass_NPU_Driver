// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef _GM_V3_2_H_
#define _GM_V3_2_H_

#include <map>

#include "zhouyi_v3x/common/gm_v3x.h"

namespace aipudrv {
/**
 * 1.gm is enable or disable
 * 2.gm config from aipu.bin: temp/in/out/inout/ignore(weight), current only
 * temp 3.gm config from runtime: weight/descriptor/workspace
 */

/* runtime specify gm buf type */
enum class GMBufTypeRT {
  GM_BUF_WEIGHT,
  GM_BUF_DESC,
  GM_BUF_WS,
  GM_BUF_REUSE,
};

struct GMBufInfo {
  std::string name;
  uint32_t size;
  uint32_t offset;
};

class JobV3_2;
class GM_V3_2 : public GM_V3X {
public:
  void setup_gm_sync_from_ddr(tcb_v3_2::tcb_t &tcb);

public:
  GM_V3_2(JobV3_2 &job);
  ~GM_V3_2();
};
} // namespace aipudrv

#endif /* _GM_V3_2_H_ */