// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  aipu_printf.h
 * @brief AIPU Debug Log print header
 */

#ifndef _AIPU_PRINTF_H_
#define _AIPU_PRINTF_H_

/**
 * log buffer header size, fixed 8 bytes
 */
#define LOG_HEADER_SZ 8

/**
 * log buffer size, fix 1MB
 */
#define BUFFER_LEN (1024 * 1024)

/**
 * buffer header format
 */
typedef struct {
    int overwrite_flag;
    int write_offset;
} aipu_log_buffer_header_t;

#endif /* _AIPU_PRINTF_H_ */
