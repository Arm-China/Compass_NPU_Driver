// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  synamic_shape.h
 * @brief AIPU User Mode Driver (UMD) dynamic shape header
 */

#ifndef _DYNAMIC_SHAPE_H_
#define _DYNAMIC_SHAPE_H_

#include <map>
#include <mutex>
#include <vector>

#include "graph_base.h"
#include "standard_api.h"
#include "zhouyi_v3x/common/graph_v3x.h"

namespace aipudrv {
class JobV3;
class DynamicShape {
private:
  JobBase &m_jobbase;
  Graph &m_graph;
  std::mutex m_dynamic_out_shape_updated_mtx;
  bool m_dynamic_out_shape_updated = false;
  bool m_dynamic_shape_set_done = false;

  /* entry: idx: <N, H, W, C> */
  std::map<int, std::vector<uint32_t>> m_config_in_tensor_shape;

  /* new set input tensor size for dynamic shape */
  std::map<int, uint32_t> m_config_in_tensor_size;

  /* new output tensor size for dynamic shape */
  std::map<int, uint32_t> m_config_out_tensor_size;

public:
  bool set_dynamic_shape_data(aipu_dynshape_param_t *shape_param);
  aipu_status_t update_dynamic_io_tensor_size(aipu_tensor_type_t type);

public:
  bool is_set_dyn_shape_true() { return m_dynamic_shape_set_done; }

  bool in_config_shape(uint32_t idx) {
    return m_config_in_tensor_shape.count(idx) == 1;
  }

  uint32_t get_config_shape_sz() { return m_config_in_tensor_shape.size(); }

  uint32_t get_config_shape_dim_sz(uint32_t input_idx) {
    if (m_config_in_tensor_shape.count(input_idx))
      return m_config_in_tensor_shape[input_idx].size();
    else
      return 0;
  }

  uint32_t get_config_shape_item(uint32_t input_idx, uint32_t dim_idx) {
    return m_config_in_tensor_shape[input_idx][dim_idx];
  }

  uint32_t get_config_in_tensor_size(uint32_t input_idx) {
    return m_config_in_tensor_size[input_idx];
  }

  void set_config_out_tensor_size(uint32_t output_idx, uint32_t size) {
    m_config_out_tensor_size[output_idx] = size;
  }

  uint32_t get_config_out_tensor_size(uint32_t output_idx) {
    return m_config_out_tensor_size[output_idx];
  }

  void clear_config_out_tensor_size() { m_config_out_tensor_size.clear(); }

  // false: not updated yet
  bool testset_dynamic_out_shape_updated() {
    bool flag = false;

    flag = m_dynamic_out_shape_updated;
    if (!m_dynamic_out_shape_updated)
      m_dynamic_out_shape_updated = true;

    return flag;
  }

public:
  DynamicShape(JobBase &_jobbase, GraphV3X &_graph,
               aipu_dynshape_param_t *dyn_params);
  ~DynamicShape();
};

} // namespace aipudrv
#endif