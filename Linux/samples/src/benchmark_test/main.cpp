// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  main.cpp
 * @brief AIPU UMD test application: basic benchmark test for arm64 platforms
 */

#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
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

#define X2_DYNAMIC_ASID1 0
#define BIND_CORE 0

namespace {

int dump_perfdata(aipu_ctx_handle_t *m_ctx, uint64_t graph_id, uint64_t job_id,
                  const std::string &dump_path = "") {
  aipu_status_t sts = AIPU_STATUS_SUCCESS;
  const char *msg = nullptr;
  aipu_tensor_desc_t desc;
  string perfdata_fname;
  uint32_t cnt;
  int ret = 0;

  sts = aipu_get_tensor_count(m_ctx, graph_id, AIPU_TENSOR_TYPE_PROFILER, &cnt);
  if (sts != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(m_ctx, sts, &msg);
    AIPU_ERR()
    ("aipu_get_tensor_descriptor(%d): %s\n", AIPU_TENSOR_TYPE_PROFILER, msg);
    ret = -1;
    return ret;
  } else if (cnt == 0) {
    AIPU_CRIT()
        << "No profiler tensor in aipu.bin, and will not dump profile binary\n";
    ret = -1;
    return ret;
  }

  sts = aipu_get_tensor_descriptor(m_ctx, graph_id, AIPU_TENSOR_TYPE_PROFILER,
                                   0, &desc);
  if (sts != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(m_ctx, sts, &msg);
    AIPU_ERR()
    ("aipu_get_tensor_descriptor(%d): %s\n", AIPU_TENSOR_TYPE_PROFILER, msg);
    ret = -1;
    return ret;
  }

  perfdata_fname =
      dump_path.empty() ? "./PerfData.bin" : (dump_path + "/PerfData.bin");
  AIPU_INFO()("perfdata file: %s\n", perfdata_fname.c_str());

  ofstream ofs(perfdata_fname, ios::binary);
  if (!ofs.is_open()) {
    AIPU_ERR()("open: %s [fail]\n", perfdata_fname.c_str());
    ret = -1;
    return ret;
  }

  unique_ptr<char[]> buffer(new char[desc.size]);
  sts = aipu_get_tensor(m_ctx, job_id, AIPU_TENSOR_TYPE_PROFILER, 0,
                        buffer.get());
  if (sts != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(m_ctx, sts, &msg);
    AIPU_ERR()("get profiler tensor: %s [fail]\n", msg);
    ret = -1;
    goto finish;
  }
  AIPU_INFO()("get profiler tensor success");

  ofs.write(buffer.get(), desc.size);

finish:
  ofs.close();
  return ret;
}

}; // namespace

int main(int argc, char *argv[]) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  aipu_ctx_handle_t *ctx;
  const char *msg = nullptr;
  uint64_t graph_id, job_id;
  uint32_t input_cnt, output_cnt;
  vector<aipu_tensor_desc_t> input_desc;
  vector<char *> input_data;
  vector<aipu_tensor_desc_t> output_desc;
  vector<char *> output_data;
  vector<char *> gt;
  cmd_opt_t opt;
  uint32_t frame_cnt = 2;
  int pass = -1;
  aipu_create_job_cfg create_job_cfg = {0};
  // struct aipu_config_clusters config_cluster;
  // config_cluster.clusters[0].en_core_cnt = 2;
  // bool en_config = false;
  aipu_driver_version_t drv_ver = {0};
  aipu_bin_buildversion_t buildver = {0};
  aipu_load_graph_cfg_t load_graph_cfg = {0};
#if X2_DYNAMIC_ASID1
  bool dynamic_asid1 = true;
#endif

  if (init_test_bench(argc, argv, &opt, "benchmark_test")) {
    AIPU_ERR()("invalid command line options/args\n");
    goto finish;
  }

  if (opt.loop_cnt != 0)
    AIPU_CRIT()
  ("aipu_benchmark_test doesn't support to specify outer loop counter\n");

  if (opt.frame_cnt != 0)
    frame_cnt = opt.frame_cnt;

  ret = aipu_init_context(&ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_init_context: %s\n", msg);
    goto finish;
  }
  AIPU_INFO()("aipu_init_context success\n");

  aipu_job_config_dump_t mem_dump_config;
  memset(&mem_dump_config, 0, sizeof(mem_dump_config));
  mem_dump_config.dump_dir = opt.dump_dir;

  // get driver's UMD and KMD version
  memset(drv_ver.umd_version, 0, sizeof(drv_ver.umd_version));
  memset(drv_ver.kmd_version, 0, sizeof(drv_ver.kmd_version));
  ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_VERSION, &drv_ver);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_ioctl: %s\n", msg);
    goto deinit_ctx;
  }
  AIPU_INFO()
  ("Driver UMD: %s, KMD: %s\n", drv_ver.umd_version, drv_ver.kmd_version);

  if (opt.profile_en) {
    ret = aipu_ioctl(ctx, AIPU_IOCTL_ENABLE_TICKCOUNTER, nullptr);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_ioctl: %s\n", msg);
      goto finish;
    }
    AIPU_INFO()("aipu_ioctl, enable tick counter success\n");
  }

#if X2_DYNAMIC_ASID1
  /* only for x2, asid0/1 will use different base, especially for large model */
  ret = aipu_ioctl(ctx, AIPU_IOCTL_SET_DYNAMIC_ASID1, &dynamic_asid1);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_ioctl: %s\n", msg);
    return -1;
  }
#endif

  if (opt.extra_weight_dir.length() > 0)
    load_graph_cfg.extra_weight_path = opt.extra_weight_dir.c_str();
  /* only when the model is extremlly small */
  // load_graph_cfg.put_weight_gm = true;
  // load_graph_cfg.put_desc_gm = true;
  // load_graph_cfg.put_ws_gm = true;
  ret = aipu_load_graph(ctx, opt.bin_files[0].c_str(), &graph_id,
                        &load_graph_cfg);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_load_graph: %s (%s)\n", msg, opt.bin_files[0].c_str());
    goto deinit_ctx;
  }
  AIPU_INFO()("aipu_load_graph success: %s\n", opt.bin_files[0].c_str());

  buildver.graph_id = graph_id;
  ret = aipu_ioctl(ctx, AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION, &buildver);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_ioctl: %s\n", msg);
    goto deinit_ctx;
  }
  AIPU_INFO()("AIPU BIN buildversion: %x\n", buildver.aipubin_buildversion);

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

  /**
   * here you can specify feature map and weight buffer from
   * specific regions. the low level allcation logic will firstly
   * try to allocate buffer from those regions. if there's no enough
   * free buffer, it will try according to the below order:
   * DTCM->SRAM->DDR, until fail to allocate.
   */
  create_job_cfg.fm_mem_region =
      AIPU_MEM_REGION_DEFAULT; /* AIPU_MEM_REGION_SRAM */
#if BIND_CORE
  create_job_cfg.dbg_dispatch = 1;
  create_job_cfg.dbg_core_id = 0; /* core id */
#endif
  ret = aipu_create_job(ctx, graph_id, &job_id, &create_job_cfg);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_create_job: %s\n", msg);
    goto unload_graph;
  }
  AIPU_INFO()("aipu_create_job success\n");

  if (mem_dump_config.dump_dir[0] != '\0') {
    uint64_t cfg_types =
        AIPU_JOB_CONFIG_TYPE_DUMP_TEXT | AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT |
        AIPU_JOB_CONFIG_TYPE_DUMP_RODATA |
        AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR | AIPU_JOB_CONFIG_TYPE_DUMP_INPUT |
        AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT | AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN |
        AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION;
    ret = aipu_config_job(ctx, job_id, cfg_types, &mem_dump_config);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_config_job: %s\n", msg);
      goto clean_job;
    }
    AIPU_INFO()("set dump config success\n");
  }

  if (opt.inputs.size() != input_cnt) {
    AIPU_INFO()
    ("input file count (%u) != input tensor count (%u)\n",
     (uint32_t)opt.inputs.size(), input_cnt);
  }

  for (uint32_t i = 0; i < output_cnt; i++) {
    char *output = new char[output_desc[i].size];
    output_data.push_back(output);
  }

  // if (en_config)
  // {
  //     ret = aipu_ioctl(ctx, AIPU_IOCTL_CONFIG_CLUSTERS, &config_cluster);
  //     if (ret != AIPU_STATUS_SUCCESS)
  //     {
  //         aipu_get_error_message(ctx, ret, &msg);
  //         AIPU_ERR()("aipu_ioctl (config clusters): %s\n", msg);
  //         goto unload_graph;
  //     }
  // }

  /* run with with multiple frames */

  for (uint32_t frame = 0; frame < frame_cnt; frame++) {
    AIPU_INFO()("Frame #%u\n", frame);
    for (uint32_t i = 0; i < min((uint32_t)opt.inputs.size(), input_cnt); i++) {
      if (input_desc[i].size > opt.inputs_size[i]) {
        AIPU_ERR()
        ("input file %s len 0x%x < input tensor %u size 0x%x\n",
         opt.input_files[i].c_str(), opt.inputs_size[i], i, input_desc[i].size);
        goto clean_job;
      }
      ret = aipu_load_tensor(ctx, job_id, i, opt.inputs[i]);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_load_tensor: %s\n", msg);
        goto clean_job;
      }
      AIPU_INFO()
      ("load input tensor %d from %s (%u/%u)\n", i, opt.input_files[i].c_str(),
       i + 1, input_cnt);
    }

    ret = aipu_finish_job(ctx, job_id, -1);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_finish_job: %s\n", msg);
      goto clean_job;
    }
    AIPU_INFO()("aipu_finish_job success\n");

    if (opt.profile_en)
      dump_perfdata(ctx, graph_id, job_id, std::string(opt.dump_dir));

    for (uint32_t i = 0; i < output_cnt; i++) {
      ret = aipu_get_tensor(ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT, i,
                            output_data[i]);
      if (ret != AIPU_STATUS_SUCCESS) {
        aipu_get_error_message(ctx, ret, &msg);
        AIPU_ERR()("aipu_get_tensor: %s\n", msg);
        goto clean_job;
      }
      AIPU_INFO()
      ("get output tensor %u success (%u/%u)\n", i, i + 1, output_cnt);
    }

    pass = check_result_helper(output_data, output_desc, opt.gts, opt.gts_size);
    if (pass == -1)
      break;
  }

clean_job:
  if (ret != AIPU_STATUS_SUCCESS)
    pass = -1;

  ret = aipu_clean_job(ctx, job_id);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_clean_job: %s\n", msg);
    goto unload_graph;
  }
  AIPU_INFO()("aipu_clean_job success\n");

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
  if (opt.profile_en) {
    ret = aipu_ioctl(ctx, AIPU_IOCTL_DISABLE_TICKCOUNTER, nullptr);
    if (ret != AIPU_STATUS_SUCCESS) {
      aipu_get_error_message(ctx, ret, &msg);
      AIPU_ERR()("aipu_ioctl: %s\n", msg);
      goto finish;
    }
    AIPU_INFO()("aipu_ioctl, disable tick counter success\n");
  }

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

  for (uint32_t i = 0; i < output_data.size(); i++) {
    delete[] output_data[i];
    output_data[i] = nullptr;
  }

  deinit_test_bench(&opt);

  return pass;
}
