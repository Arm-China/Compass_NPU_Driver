// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  helper.cpp
 * @brief UMD helper function implementation
 */

#include <mutex>
#include <cstring>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#ifndef __ANDROID__
#include <execinfo.h>
#endif
#include <cxxabi.h>
#include "standard_api.h"
#include "log.h"
#include "helper.h"

aipu_status_t umd_dump_file_helper(const char* fname, const void* src, unsigned int size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    int fd = 0;
    int wbytes = 0;

    if ((nullptr == fname) || (nullptr == src))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    if (0 == size)
    {
        ret = AIPU_STATUS_ERROR_INVALID_SIZE;
        goto finish;
    }

    fd = open(fname, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
    if (fd == -1)
    {
        LOG(LOG_ERR, "create bin file failed: %s! (errno = %d)\n", fname, errno);
        ret = AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
        goto finish;
    }
    if (ftruncate(fd, size) == -1)
    {
        LOG(LOG_ERR, "create bin file failed: %s! (errno = %d)\n", fname, errno);
        ret = AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
        goto finish;
    }
    wbytes = write(fd, src, size);
    if (wbytes != (int)size)
    {
        LOG(LOG_ERR, "write bin file %s failed, need to write 0x%x bytes, \
            successfully write 0x%x bytes (errno = %d)!\n", fname, size, wbytes, errno);
        ret = AIPU_STATUS_ERROR_WRITE_FILE_FAIL;
        goto finish;
    }

finish:
    if (fd > 0)
    {
        close(fd);
    }
    return ret;
}

aipu_status_t umd_load_file_helper(const char* fname, void* dest, unsigned int size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    int fd = 0;

    if ((nullptr == fname) || (nullptr == dest))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    if (0 == size)
    {
        ret = AIPU_STATUS_ERROR_INVALID_SIZE;
        goto finish;
    }

    fd = open(fname, O_RDONLY);
    if (fd <= 0)
    {
        LOG(LOG_ERR, "open file failed: %s! (errno = %d)\n", fname, errno);
        ret = AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
        goto finish;
    }
    if (read(fd, dest, size) < 0)
    {
        LOG(LOG_ERR, "load file failed: %s! (errno = %d)\n", fname, errno);
        ret = AIPU_STATUS_ERROR_READ_FILE_FAIL;
    }

finish:
    if (fd > 0)
    {
        close(fd);
    }
    return ret;
}

aipu_status_t umd_mmap_file_helper(const char* fname, void** data, unsigned int* size)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    int fd = 0;
    void* p_file = nullptr;
    struct stat finfo;

    if ((nullptr == fname) || (nullptr == data) || (nullptr == size))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    if (stat(fname, &finfo) != 0)
    {
        LOG(LOG_ERR, "no such a file: %s! (errno = %d)\n", fname, errno);
        ret = AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
        goto finish;
    }

    fd = open(fname, O_RDWR);
    if (fd <= 0)
    {
        LOG(LOG_ERR, "open file failed: %s! (errno = %d)\n", fname, errno);
        ret = AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
        goto finish;
    }

    p_file = mmap(nullptr, finfo.st_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if (MAP_FAILED == p_file)
    {
        ret = AIPU_STATUS_ERROR_MAP_FILE_FAIL;
        LOG(LOG_ERR, "RT failed in mapping graph file: %s! (errno = %d)\n", fname, errno);
        goto finish;
    }

    /* success */
    *data = p_file;
    *size = finfo.st_size;

finish:
    if (fd > 0)
    {
        close(fd);
    }
    return ret;
}

void umd_draw_line_helper(std::ofstream& file, char ch, uint32_t num)
{
    uint32_t max_len = 4096;
    char buffer[max_len];
    if (!file.is_open())
    {
        return;
    }

    for (uint32_t i = 0; (i < max_len) && (i < num); i++)
    {
        buffer[i] = ch;
    }

    if (num < (max_len - 1))
    {
        buffer[num] = '\n';
        buffer[num + 1] = '\0';
    }
    else
    {
        buffer[max_len - 2] = '\n';
        buffer[max_len - 1] = '\0';
    }

    file.write(buffer, strlen(buffer));
}

bool umd_is_valid_ptr(const void* lower_bound, const void* upper_bound,
        const void* ptr, uint32_t size)
{
    return ((unsigned long)ptr >= (unsigned long)lower_bound) &&
            (((unsigned long)ptr + size) < (unsigned long)upper_bound);
}

void dump_stack(void)
{
    void *addrlist[30] = {0};
    int addrlen = 0;
    char **symbollist = nullptr;
    size_t funcnamesize = 256;
    char *funcname = nullptr;

#ifndef __ANDROID__
    static std::mutex mtex;
    addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void*));
    fprintf(stderr, "\nStack backtrace:\n");
    if (addrlen == 0) {
        fprintf(stderr, "  <empty, possibly corrupt>\n");
        return;
    }

    symbollist = backtrace_symbols(addrlist, addrlen);
    funcname = (char*)malloc(funcnamesize);

    std::lock_guard<std::mutex> _lock(mtex);
    /**
     * iterate over the returned symbol lines. skip the first, it is the
     * address of this function.
     */
    for (int i = 1; i < addrlen; i++)
    {
        char *begin_name = nullptr, *begin_offset = nullptr, *end_offset = nullptr;

        /**
         * find parentheses and +address offset surrounding the mangled name:
         * ./module(function+0x15c) [0x8048a6d]
         */
        for (char *p = symbollist[i]; *p; ++p)
        {
            if (*p == '(')
                begin_name = p;
            else if (*p == '+')
                begin_offset = p;
            else if (*p == ')' && begin_offset) {
                end_offset = p;
                break;
            }
        }

        if (begin_name && begin_offset && end_offset
            && begin_name < begin_offset)
        {
            *begin_name++ = '\0';
            *begin_offset++ = '\0';
            *end_offset = '\0';

            /**
             * mangled name is now in [begin_name, begin_offset) and caller
             * offset in [begin_offset, end_offset). now apply
             * __cxa_demangle():
             */
            int status;
            char* ret = abi::__cxa_demangle(begin_name,
                            funcname, &funcnamesize, &status);
            if (status == 0) {
                funcname = ret; // use possibly realloc()-ed string
                fprintf(stderr, "  %s : %s+%s\n",
                    symbollist[i], funcname, begin_offset);
            } else {
                /**
                 * demangling failed. Output function name as a C function with
                 * no arguments.
                 */
                fprintf(stderr, "  %s : %s()+%s\n",
                    symbollist[i], begin_name, begin_offset);
            }
        } else {
            /* couldn't parse the line? print the whole line. */
            fprintf(stderr, "  %s\n", symbollist[i]);
        }
    }

    free(funcname);
    free(symbollist);
#endif
}
