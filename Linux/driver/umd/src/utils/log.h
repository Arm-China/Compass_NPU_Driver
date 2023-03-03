// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  log.h
 * @brief UMD logging macro header
 */

#ifndef _LOG_H_
#define _LOG_H_

#include <cstdio>
#include <unistd.h>
#include <sys/syscall.h>
#include "debug.h"

#define gettid() syscall(SYS_gettid)

/**
 * @brief Log level
 */
enum LogLevel
{
    LOG_ERR,     /**< serious error messages only */
    LOG_WARN,    /**< warning messages */
    LOG_ALERT,   /**< alert messages */
    LOG_INFO,    /**< normal informational messages */
    LOG_DEBUG,   /**< debug messages, should be closed after debug done */
    LOG_DEFAULT, /**< default logging messages */
    LOG_CLOSE    /**< close logging messages printing */
};

extern volatile int32_t UMD_LOG_LEVEL;
#define LOG_DETAILED(flag, FMT, ARGS...) \
    printf("%s%s:%d:%s: " FMT "\n", (flag), __FILE__, __LINE__, __PRETTY_FUNCTION__, ## ARGS);
/**
 * @brief Log macro
 * @param LogLevel log level
 * @param FMT format
 * @param ARGS var arguments
 */
#define LOG(LogLevel, FMT, ARGS...) do { \
    if (LogLevel > UMD_LOG_LEVEL) \
        break; \
    if (LogLevel==LOG_ERR) \
        LOG_DETAILED("[UMD ERROR] ", FMT, ## ARGS) \
    else if (LogLevel==LOG_WARN) \
        LOG_DETAILED("[UMD WARN] ", FMT, ## ARGS) \
    else if (LogLevel==LOG_ALERT) \
        printf("[UMD ALERT] <%ld> " FMT "\n", gettid(), ## ARGS); \
    else if (LogLevel==LOG_INFO) \
        printf("[UMD INFO] <%ld> " FMT "\n", gettid(), ## ARGS); \
    else if (LogLevel==LOG_DEBUG) \
        printf("[UMD DEBUG] <%ld> " FMT "\n", gettid(), ## ARGS); \
    else if (LogLevel==LOG_DEFAULT) \
        printf("" FMT "\n", ## ARGS); \
    } while (0)

#endif /* _LOG_H_ */