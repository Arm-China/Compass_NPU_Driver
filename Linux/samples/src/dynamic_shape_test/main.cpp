// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  main.cpp
 * @brief AIPU UMD test application: dynamic shape test, including shared weight
 * case
 * @note
 * - format1: aipu_dynamic_shape_test -b aipu.bin -i input.bin -d ./output -a
 * <aipu_target>
 * - format2: aipu_dynamic_shape_test -b aipu.bin -i input.bin -d ./output -a
 * <aipu_target> \ -r "1,256,4096/1,1,256,256" -w <extra_weight_dir>
 * - format3: aipu_dynamic_shape_test -b aipu.zip -i input1.bin,input2.bin \
 *                                    -d ./output -a <aipu_target> -r
 * "1,256,4096/1,1,256,256" -g 0
 * - format4: aipu_dynamic_shape_test -b aipu.zip -i
 * input1.bin,input2.bin,input1.bin,input2.bin \ -d ./output -a <aipu_target> -r
 * "1,256,4096/1,1,256,256:1,1,256,256" (format3/4 is mainly for zip file graph,
 * inputs of different grpah need to be orderd, and use  ':' seperates two
 * graphs dynamic shape, '-g' specify graph index, and if not provided, run all
 * graphs)
 * @note doesn't support '-c' to compare with gt now
 * tail dynamic shape can be empty if it is 0 with initialize state
 * aipu.bin should support dynamic shape, you can get shape info and
 * set shape by aipu_ioctl interface
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <vector>

#include "common/cmd_line_parsing.h"
#include "common/dbg.hpp"
#include "common/helper.h"
#include "standard_api.h"

using namespace std;

using TensorShape = std::vector<uint32_t>;

/**
 * format1: -r "[1,256,4096],[1,1,256,256]:[1,1,256,256]"
 * `:` seperates different graph's inputs shape
 */
#if 0
static std::vector<std::vector<TensorShape>> get_inputs_shape(const std::string& str)
{
    std::vector<std::vector<TensorShape>> graphs_shape;

    auto graphs = split_string(str, ":");
    for (auto g : graphs)
    {
        g.erase(0, g.find_first_not_of("\""));
        g.erase(g.find_last_not_of("\"") + 1);
        auto inputs = split_string(g, "],[");

        std::vector<TensorShape> inputs_shape;
        for (auto i : inputs)
        {
            i.erase(0, i.find_first_not_of("["));
            i.erase(i.find_last_not_of("]") + 1);
            auto dims = split_string(i, ",");

            TensorShape shape;
            for (auto d : dims)
                shape.push_back(std::stoi(d));
            inputs_shape.push_back(shape);
        }
        graphs_shape.push_back(inputs_shape);
    }
    return graphs_shape;
}
#endif

/**
 * format2: -r 1,256,4096/1,1,256,256:1,1,256,256
 *  `:` seperates different graph's inputs shape
 */
static std::vector<std::vector<TensorShape>>
get_inputs_shape(const std::string &str) {
  std::vector<std::vector<TensorShape>> graphs_shape;

  auto graphs_inputs = split_string(str, ":");
  for (auto g : graphs_inputs) {
    g.erase(0, g.find_first_not_of("\""));
    g.erase(g.find_last_not_of("\"") + 1);
    auto inputs = split_string(g, "/");

    std::vector<TensorShape> inputs_shape;
    for (auto i : inputs) {
      i.erase(0, i.find_first_not_of("["));
      i.erase(i.find_last_not_of("]") + 1);
      auto dims = split_string(i, ",");

      TensorShape shape;
      for (auto d : dims)
        shape.push_back(std::stoi(d));
      inputs_shape.push_back(shape);
    }
    graphs_shape.push_back(inputs_shape);
  }
  return graphs_shape;
}

int main(int argc, char *argv[]) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  aipu_ctx_handle_t *ctx;
  uint64_t *graph_ids;
  uint32_t ids_cnt;
  uint64_t graph_id;
  const char *msg = nullptr;
  uint32_t output_cnt;
  aipu_job_config_dump_t mem_dump_config = {0};
  aipu_load_graph_cfg_t graph_cfg = {0};
  aipu_create_job_cfg create_job_cfg = {0};
  std::vector<aipu_dynshape_num_t> dynshape_num = {};
  std::vector<uint64_t> graph_ids_vec;
  std::vector<uint64_t> job_ids;
  /* 1.outer vector is for each graph, 2.inner vector is for each inputs */
  std::vector<std::vector<TensorShape>> inputs_shape;
  std::vector<aipu_dynshape_dim_num_t> dim_num_vec;
  std::vector<aipu_dynshape_param_t> dynshape_params;
  std::vector<std::vector<aipu_tensor_desc_t>> output_desc;
  std::vector<std::vector<char *>> output_data;
  cmd_opt_t opt;
  int pass = 0;
  bool is_dynamic = false;
  bool is_zip_file = false;
  uint32_t graph_loop = 0;
  uint32_t input_cnt = 0;
  uint32_t input_offset = 0;
  constexpr int RUN_ALL = -1;
  int32_t graph_idx = RUN_ALL;
  uint32_t inputs_shape_cnt = 0;

  if (init_test_bench(argc, argv, &opt, "dynamic_shape_test")) {
    AIPU_ERR()("invalid command line options/args\n");
    goto finish;
  }

  if (opt.loop_cnt != 0)
    AIPU_CRIT()
    ("aipu_dynamic_shape_test doesn't support to specify outer loop counter\n");

  if (opt.frame_cnt != 0)
    AIPU_CRIT()
    ("aipu_dynamic_shape_test doesn't support to specify inner loop counter\n");

  if (opt.input_shape[0] != '\0')
    is_dynamic = true;

  if (opt.bin_files[0].find("zip") != std::string::npos)
    is_zip_file = true;

  graph_idx = opt.graph_idx;

  if (is_dynamic)
    inputs_shape = get_inputs_shape(opt.input_shape);

  ret = aipu_init_context(&ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_init_context: %s\n", msg);
    goto finish;
  }
  AIPU_INFO()("aipu_init_context success\n");

  memset((void *)&graph_cfg, 0, sizeof(graph_cfg));

  if (is_zip_file) {
    ret = aipu_load_share_weight_graph(ctx, opt.bin_files[0].c_str(),
                                       &graph_ids, &ids_cnt);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("load shared weight graph: %s\n", msg);
      goto deinit_ctx;
    }
    graph_ids_vec.assign(graph_ids, graph_ids + ids_cnt);
  } else {
    if (opt.extra_weight_dir.length() > 0)
      graph_cfg.extra_weight_path = opt.extra_weight_dir.c_str();
    ret = aipu_load_graph(ctx, opt.bin_files[0].c_str(), &graph_id, &graph_cfg);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("load aipu graph: %s\n", msg);
      goto deinit_ctx;
    }
    graph_ids_vec.push_back(graph_id);
  }

  if (inputs_shape.size() != graph_ids_vec.size() && graph_idx == RUN_ALL) {
    AIPU_ERR()
    ("please provid correct input shapes. If not provid '-g', sample tries to "
     "run all graphs\n");
    goto deinit_ctx;
  }

  inputs_shape_cnt = 0;
  for (auto &is : inputs_shape)
    inputs_shape_cnt += is.size();
  if (inputs_shape_cnt != opt.inputs.size()) {
    AIPU_ERR()
    ("please provid correct inputs shape cnt: %u or input files: %zu\n",
     inputs_shape_cnt, opt.inputs.size());
    goto deinit_ctx;
  }

  if (inputs_shape.size() != graph_ids_vec.size()) {
    std::vector<std::vector<TensorShape>> start(graph_idx);
    std::vector<std::vector<TensorShape>> tail(graph_ids_vec.size() -
                                               graph_idx - 1);
    inputs_shape.insert(inputs_shape.begin(), start.begin(), start.end());
    inputs_shape.insert(inputs_shape.end(), tail.begin(), tail.end());
  }

  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()
    ("aipu_load_graph_helper: %s (%s)\n", msg, opt.bin_files[0].c_str());
    goto deinit_ctx;
  }
  AIPU_INFO()("aipu_load_graph_helper success: %s\n", opt.bin_files[0].c_str());

  if (is_dynamic) {
    dynshape_num.resize(graph_ids_vec.size());
    dynshape_params.resize(graph_ids_vec.size());
    for (graph_loop = 0; graph_loop < graph_ids_vec.size(); ++graph_loop) {
      if (graph_idx != RUN_ALL && graph_idx != (int)graph_loop)
        continue;

      aipu_dynshape_param_t dynshape_param;
      /**
       * AIPU_IOCTL_GET_DS_NUM: get dynamic shape input tensor num
       */
      dynshape_num[graph_loop].graph_id = graph_ids_vec[graph_loop];
      dynshape_num[graph_loop].ds_num = (uint32_t *)malloc(sizeof(uint32_t));
      ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_DS_NUM, &dynshape_num[graph_loop]);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_ioctl: %s\n", msg);
        goto unload_graph;
      }

      if (!(*dynshape_num[graph_loop].ds_num)) {
        AIPU_ERR()("dynamic shape tensor num = 0, please check aipu.bin\n");
        goto unload_graph;
      }
      AIPU_DBG()("dynamic shape num: %d\n", *(dynshape_num[graph_loop].ds_num));

      /**
       * AIPU_IOCTL_GET_DS_DIM_NUM: get input tensor dim num
       * AIPU_IOCTL_GET_DS_INFO: get input tensor shape info
       *     max_threshhold = true return max shape info
       *     max_threshhold = false return min shape info
       */
      dynshape_param.input_shape_cnt = inputs_shape[graph_loop].size();
      dynshape_param.shape_items = (aipu_dynshape_item_t *)malloc(
          sizeof(aipu_dynshape_item_t) * inputs_shape[graph_loop].size());

      for (uint32_t id = 0; id < dynshape_param.input_shape_cnt; id++) {
        aipu_dynshape_dim_num_t dynshape_dim_num = {0};
        dynshape_dim_num.graph_id = graph_ids_vec[graph_loop];
        dynshape_dim_num.ds_idx = id;
        dynshape_dim_num.max_threshhold = true;
        dynshape_dim_num.ds_dim_num = (uint32_t *)malloc(sizeof(uint32_t));
        ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_DS_DIM_NUM, &dynshape_dim_num);
        if (ret != AIPU_STATUS_SUCCESS) {
          aipu_get_error_message(ctx, ret, &msg);
          AIPU_ERR()("aipu_ioctl: %s\n", msg);
          goto unload_graph;
        }

        aipu_dynshape_info_t dynshape_info = {0};
        dynshape_info.graph_id = graph_ids_vec[graph_loop];
        dynshape_info.ds_idx = id;
        dynshape_info.max_threshhold = true;
        dynshape_info.ds_data = (uint32_t *)malloc(
            sizeof(uint32_t) * (*dynshape_dim_num.ds_dim_num));
        AIPU_DBG()
        ("dynamic shape dim num: %d\n", *(dynshape_dim_num.ds_dim_num));
        ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_DS_INFO, &dynshape_info);
        if (ret != AIPU_STATUS_SUCCESS) {
          aipu_get_error_message(ctx, ret, &msg);
          AIPU_ERR()("aipu_ioctl: %s\n", msg);
          goto unload_graph;
        }

        dynshape_param.shape_items[id].ds_idx = id;
        dynshape_param.shape_items[id].ds_data = (uint32_t *)malloc(
            sizeof(uint32_t) * (*dynshape_dim_num.ds_dim_num));
        for (uint32_t dim_id = 0; dim_id < *dynshape_dim_num.ds_dim_num;
             dim_id++) {
          AIPU_DBG()
          ("dynamic shape max shape: %d\n", *(dynshape_info.ds_data + dim_id));
          AIPU_DBG()
          ("dynamic shape set shape: %d\n",
           inputs_shape[graph_loop][id][dim_id]);
          *(dynshape_param.shape_items[id].ds_data + dim_id) =
              inputs_shape[graph_loop][id][dim_id];
        }

        free(dynshape_info.ds_data);
        dynshape_info.ds_data = nullptr;
      }
      dynshape_params[graph_loop] = dynshape_param;
    }
  }

  job_ids.resize(graph_ids_vec.size());
  output_desc.resize(graph_ids_vec.size());
  output_data.resize(graph_ids_vec.size());
  for (graph_loop = 0; graph_loop < graph_ids_vec.size(); ++graph_loop) {
    if (graph_idx != RUN_ALL && graph_idx != (int)graph_loop)
      continue;

    if (is_dynamic)
      create_job_cfg.dynshape = &dynshape_params[graph_loop];

    ret = aipu_create_job(ctx, graph_ids_vec[graph_loop], &job_ids[graph_loop],
                          &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_create_job: %s\n", msg);
      goto unload_graph;
    }
    AIPU_INFO()("aipu_create_job success\n");

    ret = aipu_get_tensor_count(ctx, graph_ids_vec[graph_loop],
                                AIPU_TENSOR_TYPE_INPUT, &input_cnt);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
      goto unload_graph;
    }

    mem_dump_config.dump_dir = opt.dump_dir;
    if (mem_dump_config.dump_dir[0] != '\0') {
      std::string path = mem_dump_config.dump_dir;
      if (graph_ids_vec.size() > 1)
        path = path + "/graph_" + std::to_string(graph_loop);
      help_create_dir(path.c_str());
      mem_dump_config.dump_dir = path.c_str();
      uint64_t cfg_types =
          AIPU_JOB_CONFIG_TYPE_DUMP_TEXT | AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT |
          AIPU_JOB_CONFIG_TYPE_DUMP_RODATA |
          AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR |
          AIPU_JOB_CONFIG_TYPE_DUMP_INPUT | AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT |
          AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN |
          AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;
      ret = aipu_config_job(ctx, job_ids[graph_loop], cfg_types,
                            &mem_dump_config);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_config_job: %s\n", msg);
        goto clean_job;
      }
      AIPU_INFO()("set dump config success\n");
    }

    for (uint32_t id = 0; id < inputs_shape[graph_loop].size(); id++) {
      ret = aipu_load_tensor(ctx, job_ids[graph_loop], id,
                             opt.inputs[id + input_offset]);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_load_tensor: %s\n", msg);
        goto clean_job;
      }
      AIPU_INFO()
      ("load input tensor %d from %s (%u/%u)\n", id,
       opt.input_files[id + input_offset].c_str(), id + 1,
       inputs_shape[graph_loop].size());
    }

    ret = aipu_finish_job(ctx, job_ids[graph_loop], -1);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_finish_job: %s\n", msg);
      goto clean_job;
    }
    AIPU_INFO()("aipu_finish_job success\n");

    ret = aipu_get_tensor_count(ctx, graph_ids_vec[graph_loop],
                                AIPU_TENSOR_TYPE_OUTPUT, &output_cnt);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
      goto clean_job;
    }
    AIPU_DBG()("aipu_get_tensor_count success: output cnt = %d\n", output_cnt);

    /**
     * after finish job, output tensor shape info has write to output tensor
     * desc
     */
    for (uint32_t i = 0; i < output_cnt; i++) {
      aipu_tensor_desc_t desc;
      ret = aipu_get_tensor_descriptor(ctx, graph_ids_vec[graph_loop],
                                       AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
        goto clean_job;
      }
      output_desc[graph_loop].push_back(desc);
    }

    /* alloc output tensor space */
    for (uint32_t i = 0; i < output_desc[graph_loop].size(); i++) {
      char *output = new char[output_desc[graph_loop][i].size * 2];
      output_data[graph_loop].push_back(output);
    }

    for (uint32_t i = 0; i < output_desc[graph_loop].size(); i++) {
      ret = aipu_get_tensor(ctx, job_ids[graph_loop], AIPU_TENSOR_TYPE_OUTPUT,
                            i, output_data[graph_loop][i]);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor: %s\n", msg);
        goto clean_job;
      }
      AIPU_DBG()
      ("get output tensor %u success (%u/%lu)\n", i, i + 1,
       output_desc[graph_loop].size());
    }

    pass = check_result_helper(output_data[graph_loop], output_desc[graph_loop],
                               opt.gts, opt.gts_size);

  clean_job:
    ret = aipu_clean_job(ctx, job_ids[graph_loop]);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_clean_job: %s\n", msg);
      goto unload_graph;
    }
    AIPU_INFO()("aipu_clean_job success\n");

  unload_graph:
    ret = aipu_unload_graph(ctx, graph_ids_vec[graph_loop]);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_unload_graph: %s\n", msg);
      goto deinit_ctx;
    }
    AIPU_INFO()("aipu_unload_graph success\n");

    input_offset += input_cnt;
  }

deinit_ctx:
  ret = aipu_deinit_context(ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_deinit_ctx: %s\n", msg);
    goto finish;
  }
  AIPU_INFO()("aipu_deinit_ctx success\n");

finish:
  if (AIPU_STATUS_SUCCESS != ret)
    pass = -1;

  for (uint32_t i = 0; i < graph_ids_vec.size(); ++i) {
    if (graph_idx != RUN_ALL && graph_idx != (int)i)
      continue;

    if (output_data.size() == 0)
      break;

    for (uint32_t j = 0; j < output_data[i].size(); ++j) {
      delete[] output_data[i][j];
      output_data[i][j] = nullptr;
    }
  }

  if (is_dynamic) {
    for (uint32_t i = 0; i < dynshape_num.size(); ++i) {
      if (dynshape_num[i].ds_num != nullptr) {
        free(dynshape_num[i].ds_num);
        dynshape_num[i].ds_num = nullptr;
      }
    }

    for (uint32_t i = 0; i < dynshape_params.size(); ++i) {
      for (uint32_t j = 0; j < dynshape_params[i].input_shape_cnt; ++j) {
        if (dynshape_params[i].shape_items[j].ds_data != nullptr) {
          free(dynshape_params[i].shape_items[j].ds_data);
          dynshape_params[i].shape_items[j].ds_data = nullptr;
        }
      }

      if (dynshape_params[i].shape_items != nullptr) {
        free(dynshape_params[i].shape_items);
        dynshape_params[i].shape_items = nullptr;
      }
    }
  }

  deinit_test_bench(&opt);

  return pass;
}
