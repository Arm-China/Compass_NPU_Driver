// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef _GM_V3X_H_
#define _GM_V3X_H_

#include <memory>
#include <vector>

#include "kmd/tcb.h"
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
  MemoryBase *m_mem = nullptr;

  DEV_PA_64 m_gm_map_base = 0;

public:
  virtual aipu_status_t gm_malloc(uint32_t bss_id, uint32_t idx,
                                  uint32_t buf_type, std::string &buf_name,
                                  BufferDesc *buf) {
    return AIPU_STATUS_SUCCESS;
  };
  virtual bool is_gm_buffer(uint32_t idx, uint32_t buf_type) { return false; };
  virtual bool gm_need_remap() { return false; };
  virtual void set_valid_map_base(BufferDesc &buf){};
  virtual void gm_dynamic_switch(uint32_t core_cnt){};
  virtual void setup_gm_sync_from_ddr(tcb_t &tcb) = 0;
  virtual bool gm_need_sync_out() { return true; };

public:
  GM_V3X(JobV3X &job, GraphV3X &graph) : m_job(job), m_graph(graph) {}
  virtual ~GM_V3X(){};
};
} // namespace aipudrv

#endif /* _GM_V3X_H_ */