/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2023 Arm Technology (China) Co. Ltd. All rights reserved. */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <linux/sizes.h>

#define AIPU_CONFIG_ENABLE_MEM_MANAGEMENT   1
#define AIPU_CONFIG_HOST_MAP_SRAM           0
#define AIPU_CONFIG_HOST_MAP_DTCM           0
#define AIPU_CONFIG_USE_DEFAULT_MEM_SIZE    0
#define AIPU_CONFIG_DEFAULT_MEM_SIZE        (64 * SZ_1M)
#define AIPU_CONFIG_MAX_RESERVED_REGIONS    32
#define AIPU_CONFIG_DEFAULT_RESET_DELAY_US  50

#endif /* __CONFIG_H__ */
