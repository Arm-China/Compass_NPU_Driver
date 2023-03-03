// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "standard_api.h"
#include "aipu_printf.h"

aipu_status_t aipu_printf(char *log_buffer_base, char *redirect_file)
{
    #define LOG_PRIFX "LOG DUMP: "
    enum {EM_REDIRECT_FAIL = -1, EM_REDIRECT_TERMINAL = 0, EM_REDIRECT_FILE = 1};
    int ret = 0, redirect_flag = -1, redirect_fd = EM_REDIRECT_FAIL;
    char end_char = 0;
    char *end_pos = NULL;
    char *start_log_base = NULL;
    aipu_log_buffer_header_t *header = NULL;

    if (nullptr == log_buffer_base)
        return AIPU_STATUS_ERROR_NULL_PTR;

    header = (aipu_log_buffer_header_t *)log_buffer_base;

    if (redirect_file == NULL) {
        redirect_flag = EM_REDIRECT_TERMINAL;
    } else {
        redirect_fd = open(redirect_file, O_CREAT|O_RDWR, 0755);
        if (redirect_fd < 0) {
            printf(LOG_PRIFX "open %s, ret=%d [fail]\n", redirect_file, redirect_fd);
            ret = redirect_fd;
            goto out;
        }

        redirect_flag = EM_REDIRECT_FILE;
    }

    if (header->overwrite_flag == 0 && header->write_offset != 0) {
        /**
         * this branch is for buffer non-rewind handling
         * dump [0, n]
         */
        start_log_base = log_buffer_base + LOG_HEADER_SZ;

        if (redirect_flag == EM_REDIRECT_TERMINAL) {
            /* output log to terminal */
            end_pos = log_buffer_base + header->write_offset + LOG_HEADER_SZ;
            end_char = *end_pos;
            *end_pos = '\0';
            printf("%s", start_log_base);
            *end_pos = end_char;
        } else if (redirect_flag == EM_REDIRECT_FILE) {
            /* output log to specified file */
            ret = write(redirect_fd, start_log_base, header->write_offset);
            if (ret < 0) {
                close(redirect_fd);
                goto out;
            } else
                close(redirect_fd);
        }

        ret = header->write_offset;
    } else if (header->overwrite_flag == 1) {
        /**
         * this branch for buffer rewind handling
         * first, dump [n + 1, 1MB - 1]
         * second, dump [0, n]
         */
        start_log_base = log_buffer_base + LOG_HEADER_SZ + header->write_offset;

        if (redirect_flag == EM_REDIRECT_TERMINAL) {
            end_pos = log_buffer_base + BUFFER_LEN - 1;
            end_char = *end_pos;
            *end_pos = '\0';
            printf("%s", start_log_base);
            putchar(end_char);
            *end_pos = end_char;

            if (header->write_offset != 0) {
                start_log_base = log_buffer_base + LOG_HEADER_SZ;
                end_pos = log_buffer_base + header->write_offset + LOG_HEADER_SZ;
                end_char = *end_pos;
                *end_pos = '\0';
                printf("%s", log_buffer_base + LOG_HEADER_SZ);
                *end_pos = end_char;
            }
        } else if (redirect_flag == EM_REDIRECT_FILE) {
            int write_len = BUFFER_LEN - LOG_HEADER_SZ - header->write_offset;
            ret = write(redirect_fd, start_log_base, write_len);
            if (ret < 0) {
                close(redirect_fd);
                goto out;
            }

            if (header->write_offset != 0) {
                start_log_base = log_buffer_base + LOG_HEADER_SZ;
                ret = write(redirect_fd, start_log_base, header->write_offset);
                if (ret < 0) {
                    close(redirect_fd);
                    goto out;
                }
            }

            close(redirect_fd);
        }

        ret = BUFFER_LEN - LOG_HEADER_SZ;
    } else {
        /* no valid log information */
        ret = 0;
    }

out:
    if (ret < 0)
        return AIPU_STATUS_ERROR_PRINTF_FAIL;

    return AIPU_STATUS_SUCCESS;
}