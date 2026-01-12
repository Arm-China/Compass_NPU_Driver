// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  main.cpp
 * @brief AIPU UMD test application: npu hardware/software reset test for juno
 * arm64 platforms
 */

#include "common/cmd_line_parsing.h"
#include "common/dbg.hpp"
#include "common/helper.h"
#include "standard_api.h"

int main(int argc, char *argv[]) {
  cmd_opt_t opt;
  if (init_test_bench(argc, argv, &opt, "reset test")) {
    AIPU_ERR()("invalid command line options/args\n");
    return -1;
  }

  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  aipu_ctx_handle_t *ctx;
  const char *msg = nullptr;
  ret = aipu_init_context(&ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_init_context: %s\n", msg);
    return -1;
  }
  AIPU_INFO()("aipu_init_context success\n");

  uint32_t cmd =
      (opt.reset_type == 0) ? AIPU_IOCTL_HW_RESET : AIPU_IOCTL_SW_RESET;
  ret = aipu_ioctl(ctx, cmd, nullptr);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_init_context: %s\n", msg);
    goto deinit_ctx;
  }
  AIPU_INFO()("reset success\n");

deinit_ctx:
  ret = aipu_deinit_context(ctx);
  if (ret != AIPU_STATUS_SUCCESS) {
    aipu_get_error_message(ctx, ret, &msg);
    AIPU_ERR()("aipu_deinit_ctx: %s\n", msg);
    goto deinit_bench;
  }
  AIPU_INFO()("aipu_deinit_ctx success\n");

deinit_bench:
  deinit_test_bench(&opt);

  return ret;
}
