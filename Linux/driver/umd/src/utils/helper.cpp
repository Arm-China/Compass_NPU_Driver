// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  helper.cpp
 * @brief UMD helper function implementation
 */
#include <ctime>
#include <string>
#include <chrono>
#include <sstream>
#include <iostream>
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
    void *address_array[40] = {0};
    int address_len = 0;
    char **sym_array = nullptr;
    size_t func_name_sz = 256;
    char *pfunc_name = nullptr;

#ifndef __ANDROID__
    static std::mutex mtex;
    address_len = backtrace(address_array, sizeof(address_array) / sizeof(void*));
    fprintf(stderr, "\nStack backtrace:\n");
    if (address_len == 0) {
        fprintf(stderr, "  <empty, stack corrupt>\n");
        return;
    }

    sym_array = backtrace_symbols(address_array, address_len);
    pfunc_name = (char*)malloc(func_name_sz);

    std::lock_guard<std::mutex> _lock(mtex);
    /**
     * ignore the first, as it is the address of this function.
     */
    for (int i = 1; i < address_len; i++)
    {
        char *mangled_name_start = nullptr, *begin_offset = nullptr, *end_offset = nullptr;

        /**
         * find parentheses and +address offset:
         * ./object(function+0x200) [0x105426]
         */
        for (char *ptr = sym_array[i]; *ptr; ++ptr)
        {
            if (*ptr == '(')
                mangled_name_start = ptr;
            else if (*ptr == '+')
                begin_offset = ptr;
            else if (*ptr == ')' && begin_offset) {
                end_offset = ptr;
                break;
            }
        }

        if (mangled_name_start && begin_offset && end_offset
            && mangled_name_start < begin_offset)
        {
            int status = -1;
            char *start_name = nullptr;
            *mangled_name_start++ = '\0';
            *begin_offset++ = '\0';
            *end_offset = '\0';

            start_name = abi::__cxa_demangle(mangled_name_start,
                    pfunc_name, &func_name_sz, &status);
            if (status == 0) {
                pfunc_name = start_name;
                fprintf(stderr, "  %s : %s+%s\n",
                    sym_array[i], pfunc_name, begin_offset);
            } else {
                fprintf(stderr, "  %s : %s()+%s\n",
                    sym_array[i], mangled_name_start, begin_offset);
            }
        } else {
            fprintf(stderr, "  %s\n", sym_array[i]);
        }
    }

    free(pfunc_name);
    free(sym_array);
#endif
}

extern volatile char UMD_LOG_TIMESTAMP;
std::string umd_timestamp_helper(int time_stamp_type)
{
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
	std::tm* now_tm = std::localtime(&now_time_t);
	char buffer[128] = {0};
	std::ostringstream ss;
	std::chrono::milliseconds ms;
	std::chrono::microseconds us;
	std::chrono::nanoseconds ns;

    if (UMD_LOG_TIMESTAMP == 'n' || UMD_LOG_TIMESTAMP == 'N')
        return "";

    strftime(buffer, sizeof(buffer), "%F %T", now_tm);
    ss.fill('0');

	switch (time_stamp_type)
	{
        case 0:
            ss << buffer;
            break;

        case 1:
            ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            ss << buffer << ":" << ms.count();
            break;

        case 2:
            ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()) % 1000000;
            ss << buffer << ":" << ms.count() << ":" << us.count() % 1000;
            break;

        case 3:
            ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()) % 1000000;
            ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()) % 1000000000;
            ss << buffer << ":" << ms.count() << ":" << us.count() % 1000
                << ":" << ns.count() % 1000;
            break;

        case 4:
            us = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()) % 1000000;
            ss << buffer << ":" << us.count();
            break;

        default:
            ss << buffer;
            break;
	}

	return ss.str();
}