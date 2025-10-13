// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  main.cpp
 * @brief AIPU UMD test application: basic benchmark test for arm64 platforms
 *        - using non-blocking API aipu_flush_job
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

int main(int argc, char *argv[]) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  aipu_ctx_handle_t *ctx;
  const char *msg = nullptr;
  uint64_t graph_id;
  uint32_t input_cnt, output_cnt;
  vector<aipu_tensor_desc_t> input_desc;
  vector<char *> input_data; /* multi-jobs use the same input vector */
  vector<aipu_tensor_desc_t> output_desc;
  vector<vector<std::shared_ptr<char>>> job_outputs;
  vector<char *> gt;
  cmd_opt_t opt;
  aipu_create_job_cfg_t create_job_cfg = {0};
  int pass = -1;

  std::vector<uint64_t> jobs_id;
  std::vector<aipu_job_status_t> jobs_status;

  uint32_t pipeline_job_cnt = 2;
  uint32_t frame_cnt = 1;

  aipu_global_config_simulation_t sim_glb_config;
  memset(&sim_glb_config, 0, sizeof(sim_glb_config));

  AIPU_CRIT() << "usage: ./aipu_flush_test -b aipu.bin -i input0.bin -c "
                 "output.bin -d ./\n";

  if (init_test_bench(argc, argv, &opt, "flush_job_test")) {
    AIPU_ERR()("invalid command line options/args\n");
    goto finish;
  }

  if (opt.loop_cnt != 0)
    pipeline_job_cnt = opt.loop_cnt;

  if (opt.frame_cnt != 0)
    frame_cnt = opt.frame_cnt;

  jobs_id.resize(pipeline_job_cnt);
  jobs_status.resize(pipeline_job_cnt, AIPU_JOB_STATUS_NO_STATUS);

  if (opt.dump_dir[0] != '\0')
    AIPU_CRIT()("aipu_flush_test doesn't support to dump files\n");

  sim_glb_config.log_level = opt.log_level;
  sim_glb_config.verbose = opt.verbose;
  sim_glb_config.simulator = opt.simulator;

  ret = aipu_init_context(&ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_init_context: %s\n", msg);
    goto finish;
  }
  AIPU_INFO()("aipu_init_context success\n");

  ret = aipu_config_global(ctx, AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_config_global: %s\n", msg);
    goto deinit_ctx;
  }
  AIPU_INFO()("set global simulation config success\n");

  ret = aipu_load_graph(ctx, opt.bin_files[0].c_str(), &graph_id);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_load_graph: %s (%s)\n", msg, opt.bin_files[0].c_str());
    goto deinit_ctx;
  }
  AIPU_INFO()("aipu_load_graph success: %s\n", opt.bin_files[0].c_str());

  ret =
      aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, &input_cnt);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
    goto unload_graph;
  }
  // AIPU_INFO()("aipu_get_tensor_count success: input cnt = %d\n", input_cnt);

  for (uint32_t i = 0; i < input_cnt; i++) {
    aipu_tensor_desc_t desc;
    ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_INPUT, i,
                                     &desc);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
      goto unload_graph;
    }
    input_desc.push_back(desc);
  }

  ret = aipu_get_tensor_count(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT,
                              &output_cnt);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_get_tensor_count: %s\n", msg);
    goto unload_graph;
  }
  // AIPU_INFO()("aipu_get_tensor_count success: output cnt = %d\n",
  // output_cnt);

  for (uint32_t i = 0; i < output_cnt; i++) {
    aipu_tensor_desc_t desc;
    ret = aipu_get_tensor_descriptor(ctx, graph_id, AIPU_TENSOR_TYPE_OUTPUT, i,
                                     &desc);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_get_tensor_descriptor: %s\n", msg);
      goto unload_graph;
    }
    output_desc.push_back(desc);
  }
  // fprintf(stderr, "[TEST INFO] aipu_get_tensor_descriptor done\n");

  for (uint32_t job = 0; job < pipeline_job_cnt; job++) {
    ret = aipu_create_job(ctx, graph_id, &jobs_id[job], &create_job_cfg);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_create_job: %s\n", msg);
      goto unload_graph;
    }
    AIPU_INFO()("create job %lx success\n", jobs_id[job]);
  }

  if (opt.inputs.size() != input_cnt) {
    fprintf(stdout,
            "[TEST WARN] input file count (%u) != input tensor count (%u)\n",
            (uint32_t)opt.inputs.size(), input_cnt);
  }

  for (uint32_t job = 0; job < pipeline_job_cnt; job++) {
    vector<std::shared_ptr<char>> outputs;
    for (uint32_t i = 0; i < output_cnt; i++) {
      std::shared_ptr<char> output(new char[output_desc[i].size],
                                   std::default_delete<char[]>());
      outputs.push_back(output);
    }
    job_outputs.push_back(outputs);
  }

  /* run with with multiple frames */
  for (uint32_t frame = 0; frame < frame_cnt; frame++) {
    AIPU_INFO()("Frame #%u\n", frame);
    for (uint32_t job = 0; job < pipeline_job_cnt; job++) {
      AIPU_INFO()("Job #%u\n", job);
      for (uint32_t i = 0; i < min((uint32_t)opt.inputs.size(), input_cnt);
           i++) {
        if (input_desc[i].size > opt.inputs_size[i]) {
          AIPU_ERR()
          ("input file %s len 0x%x < input tensor %u size 0x%x\n",
           opt.input_files[i].c_str(), opt.inputs_size[i], i,
           input_desc[i].size);
          goto clean_job;
        }
        ret = aipu_load_tensor(ctx, jobs_id[job], i, opt.inputs[i]);
        if (ret != AIPU_STATUS_SUCCESS) {
          aipu_get_error_message(ctx, ret, &msg);
          AIPU_ERR()("aipu_load_tensor: %s\n", msg);
          goto clean_job;
        }
        AIPU_INFO()
        ("load job %lx input tensor %d from %s (%u/%u)\n", jobs_id[job], i,
         opt.input_files[i].c_str(), i + 1, input_cnt);
      }

      ret = aipu_flush_job(ctx, jobs_id[job]);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_flush_job: %s\n", msg);
        goto clean_job;
      }
      AIPU_INFO()("flush job %lx success\n", jobs_id[job]);
    }

    for (uint32_t job = 0; job < pipeline_job_cnt; job++) {
      while ((jobs_status[job] != AIPU_JOB_STATUS_DONE) &&
             (jobs_status[job] != AIPU_JOB_STATUS_EXCEPTION)) {
        ret = aipu_get_job_status(ctx, jobs_id[job], &jobs_status[job], 2000);
        if (ret == AIPU_STATUS_ERROR_TIMEOUT) {
          AIPU_INFO()("flush job timeout\n");
          continue;
        } else if (ret != AIPU_STATUS_SUCCESS) {
          aipu_get_error_message(ctx, ret, &msg);
          AIPU_ERR()("aipu_get_job_status: %s\n", msg);
          goto clean_job;
        }
        AIPU_INFO()("job %lx still running...\n", jobs_id[job]);
      }

      if (jobs_status[job] == AIPU_JOB_STATUS_DONE) {
        AIPU_INFO()("job %lx done\n", jobs_id[job]);
      } else {
        fprintf(stdout, "[TEST ERROR] job %lx exception\n", jobs_id[job]);
        goto clean_job;
      }

      for (uint32_t i = 0; i < output_cnt; i++) {
        memset(job_outputs[job][i].get(), 0, output_desc[i].size);
        ret = aipu_get_tensor(ctx, jobs_id[job], AIPU_TENSOR_TYPE_OUTPUT, i,
                              job_outputs[job][i].get());
        if (ret != AIPU_STATUS_SUCCESS) {
          aipu_get_error_message(ctx, ret, &msg);
          AIPU_ERR()("aipu_get_tensor: %s\n", msg);
          goto clean_job;
        }
        AIPU_INFO()
        ("get output tensor %u success (%u/%u)\n", i, i + 1, output_cnt);
      }

      pass = check_result(job_outputs[job], output_desc, opt.gts, opt.gts_size);
      jobs_status[job] = AIPU_JOB_STATUS_NO_STATUS;
      if (pass == -1)
        break;
    }
  }

clean_job:
  if (ret != AIPU_STATUS_SUCCESS)
    pass = -1;

  for (uint32_t job = 0; job < pipeline_job_cnt; job++) {
    ret = aipu_clean_job(ctx, jobs_id[job]);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_clean_job: %s\n", msg);
      goto unload_graph;
    }
    AIPU_INFO()("clean job %lx success\n", jobs_id[job]);
  }

unload_graph:
  if (ret != AIPU_STATUS_SUCCESS)
    pass = -1;

  ret = aipu_unload_graph(ctx, graph_id);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_unload_graph: %s\n", msg);
    goto deinit_ctx;
  }
  AIPU_INFO()("aipu_unload_graph success\n");

deinit_ctx:
  if (ret != AIPU_STATUS_SUCCESS)
    pass = -1;

  ret = aipu_deinit_context(ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_deinit_ctx: %s\n", msg);
    goto finish;
  }
  AIPU_INFO()("aipu_deinit_ctx success\n");

finish:
  if (ret != AIPU_STATUS_SUCCESS)
    pass = -1;

  deinit_test_bench(&opt);
  return pass;
}