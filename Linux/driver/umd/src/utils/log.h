// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
#include "helper.h"
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
    printf("%s%s%s:%d:%s: " FMT "\n", umd_timestamp_helper(4).c_str(), \
        (flag), __FILE__, __LINE__, __PRETTY_FUNCTION__, ## ARGS);
/**
 * @brief Log macro
 * @param LogLevel log level
 * @param FMT format
 * @param ARGS var arguments
 */
#ifdef __ANDROID__
#include <android/log.h>
#include <sys/system_properties.h>
#define LOG(LogLevel, FMT, ARGS...) do { \
    if (LogLevel > UMD_LOG_LEVEL) \
        break; \
    if (LogLevel==LOG_ERR) \
        __android_log_print(ANDROID_LOG_ERROR, "UMD", "[%s:%d:%s]" FMT, __FUNCTION__, __LINE__, __PRETTY_FUNCTION__, ## ARGS); \
    else if (LogLevel==LOG_WARN) \
        __android_log_print(ANDROID_LOG_WARN, "UMD", "[%s:%d:%s]" FMT, __FUNCTION__, __LINE__, __PRETTY_FUNCTION__, ## ARGS); \
    else if (LogLevel==LOG_ALERT) \
        __android_log_print(ANDROID_LOG_WARN, "UMD", "[%ld]" FMT, gettid(), ## ARGS); \
    else if (LogLevel==LOG_INFO) \
        __android_log_print(ANDROID_LOG_INFO, "UMD", "[%ld]" FMT, gettid(), ## ARGS); \
    else if (LogLevel==LOG_DEBUG) \
        __android_log_print(ANDROID_LOG_DEBUG, "UMD", "[%ld]" FMT, gettid(), ## ARGS); \
    else if (LogLevel==LOG_DEFAULT) \
        __android_log_print(ANDROID_LOG_INFO, "UMD", "[%ld]" FMT, gettid(), ## ARGS); \
    } while (0)
#else
#define LOG(LogLevel, FMT, ARGS...) do { \
    if (LogLevel > UMD_LOG_LEVEL) \
        break; \
    if (LogLevel==LOG_ERR) \
        LOG_DETAILED("[UMD ERROR] ", FMT, ## ARGS) \
    else if (LogLevel==LOG_WARN) \
        LOG_DETAILED("[UMD WARN] ", FMT, ## ARGS) \
    else if (LogLevel==LOG_ALERT) \
        printf("%s [UMD ALERT] <%ld> " FMT "\n", umd_timestamp_helper(4).c_str(), gettid(), ## ARGS); \
    else if (LogLevel==LOG_INFO) \
        printf("%s [UMD INFO] <%ld> " FMT "\n", umd_timestamp_helper(4).c_str(), gettid(), ## ARGS); \
    else if (LogLevel==LOG_DEBUG) \
        printf("%s [UMD DEBUG] <%ld> " FMT "\n", umd_timestamp_helper(4).c_str(), gettid(), ## ARGS); \
    else if (LogLevel==LOG_DEFAULT) \
        printf("" FMT "\n", ## ARGS); \
    } while (0)
#endif
#endif /* _LOG_H_ */