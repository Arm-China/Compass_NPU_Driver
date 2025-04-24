// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef _GM_V3X_H_
#define _GM_V3X_H_

#include <memory>
#include <vector>

#include "zhouyi_v3x/common/graph_v3x.h"

namespace aipudrv {
class JobV3X;
class GM_V3X {
protected:
  bool m_gm_asm = false;
  std::vector<BufferDesc *> m_gm_free_buffer;
  std::vector<BufferDesc *> m_gm_alloc_buffer;

public:
  JobV3X &m_job;
  GraphV3X &m_graph;

  DEV_PA_64 m_gm_map_base = 0;

public:
  virtual aipu_status_t gm_malloc(uint32_t bss_id, uint32_t idx,
                                  uint32_t buf_type, std::string &buf_name,
                                  BufferDesc *buf) = 0;
  virtual bool gm_is_gm_buffer(uint32_t idx, uint32_t buf_type) = 0;
  virtual bool gm_need_remap() = 0;
  virtual void set_valid_map_base(BufferDesc &buf) = 0;
  virtual void gm_dynamic_switch(uint32_t core_cnt){};
  virtual bool gm_need_sync_out() { return true; };

public:
  GM_V3X(JobV3X &job, GraphV3X &graph) : m_job(job), m_graph(graph) {}
  virtual ~GM_V3X(){};
};
} // namespace aipudrv

#endif /* _GM_V3X_H_ */