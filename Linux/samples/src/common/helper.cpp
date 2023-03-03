// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <memory>
#include <vector>
#include <string.h>
#include <errno.h>
#include "helper.h"
#include "dbg.hpp"

static bool is_output_correct(volatile char* src1, char* src2, uint32_t cnt)
{
    for (uint32_t out_chr = 0; out_chr < cnt; out_chr++)
    {
        if (src1[out_chr] != src2[out_chr])
        {
            return false;
        }
    }
    return true;
}

int dump_file_helper(const char* fname, void* src, unsigned int size)
{
    int ret = 0;
    int fd = 0;

    fd = open(fname, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
    if (fd == -1)
    {
        AIPU_ERR()("create bin file %s failed (errno = %d)!", fname, errno);
        ret = -1;
        goto finish;
    }
    if (ftruncate(fd, size) == -1)
    {
        AIPU_ERR()("create bin file %s failed (errno = %d)!", fname, errno);
        ret = -1;
        goto finish;
    }
    ret = write(fd, src, size);
    if (ret != (int)size)
    {
        AIPU_ERR()("write bin file %s failed, need to write 0x%x bytes, \
            successfully write 0x%x bytes (errno = %d)!", fname, size, ret, errno);
        ret = -1;
        goto finish;
    }
    ret = 0;

finish:
    if (fd != -1)
    {
        close(fd);
    }
    return ret;
}

int load_file_helper(const char* fname, char** dest, uint32_t* size)
{
    int ret = 0;
    int fd = 0;
    struct stat finfo;

    if ((nullptr == fname) || (nullptr == dest) || (nullptr == size))
    {
        return -1;
    }

    *dest = nullptr;

    if (stat(fname, &finfo) != 0)
    {
        AIPU_ERR()("open file failed: %s! (errno = %d)\n", fname, errno);
        return -1;
    }

    fd = open(fname, O_RDONLY);
    if (fd <= 0)
    {
        AIPU_ERR()("open file failed: %s! (errno = %d)\n", fname, errno);
        return -1;
    }

    *dest = new char[finfo.st_size];
    *size = finfo.st_size;

    if (read(fd, *dest, finfo.st_size) < 0)
    {
        AIPU_ERR()("load file failed: %s! (errno = %d)\n", fname, errno);
        ret = -1;
        goto finish;
    }

finish:
    if (fd > 0)
    {
        close(fd);
    }
    if ((ret < 0) && (nullptr != dest) && (nullptr != *dest))
    {
        delete[] *dest;
        *dest = nullptr;
    }
    return ret;
}

int unload_file_helper(char* data)
{
    if (data != nullptr)
    {
        delete[] data;
    }
    return 0;
}

int check_result_helper(const std::vector<char*>& outputs, const std::vector<aipu_tensor_desc_t>& descs,
    char* gt, uint32_t gt_size)
{
    int offset = 0;
    void* check_base = NULL;
    int tot_size = 0;
    volatile char* out_va = nullptr;
    char* check_va = nullptr;
    uint32_t size = 0;
    int ret = 0;
    int pass = 0;

    if (outputs.size() != descs.size())
    {
        AIPU_ERR()("output data count (%lu) != benchmark tensor count (%lu)!\n",
            outputs.size(), descs.size());
        return -1;
    }

    for (uint32_t i = 0; i < descs.size(); i++)
    {
        tot_size += descs[i].size;
    }

    check_base = gt;

    for (uint32_t id = 0; id < outputs.size(); id++)
    {
        out_va = (volatile char*)outputs[id];
        check_va = (char*)((unsigned long)check_base + offset);
        size = descs[id].size;
        if ((offset + size) > gt_size)
        {
            AIPU_ERR()("gt file length (0x%x) < output tensor size!\n", gt_size);
            return -1;
        }
        ret = is_output_correct(out_va, check_va, size);
        if (ret == true)
        {
            AIPU_CRIT()("Test Result Check PASS! (%u/%lu)\n", id + 1,
                outputs.size());
        }
        else
        {
            pass = -1;
            AIPU_ERR()("Test Result Check FAILED! (%u/%lu)\n", id + 1,
                outputs.size());
        }
        offset += size;
    }

    return pass;
}

int check_result(std::vector< std::shared_ptr<char> >outputs, const std::vector<aipu_tensor_desc_t>& descs,
    char* gt, uint32_t gt_size)
{
    int offset = 0;
    void* check_base = NULL;
    int tot_size = 0;
    volatile char* out_va = nullptr;
    char* check_va = nullptr;
    uint32_t size = 0;
    int ret = 0;
    int pass = 0;

    if (outputs.size() != descs.size())
    {
        AIPU_ERR()("output data count (%lu) != benchmark tensor count (%lu)!\n",
            outputs.size(), descs.size());
        return -1;
    }

    for (uint32_t i = 0; i < descs.size(); i++)
    {
        tot_size += descs[i].size;
    }

    check_base = gt;

    for (uint32_t id = 0; id < descs.size(); id++)
    {
        out_va = (volatile char*)outputs[id].get();
        check_va = (char*)((unsigned long)check_base + offset);
        size = descs[id].size;
        if ((offset + size) > gt_size)
        {
            AIPU_ERR()("gt file length (0x%x) < output tensor size!\n", gt_size);
            return -1;
        }
        ret = is_output_correct(out_va, check_va, size);
        if (ret == true)
        {
            AIPU_CRIT()("Test Result Check PASS! (%u/%lu)\n", id + 1,
                outputs.size());
        }
        else
        {
            pass = -1;
            AIPU_ERR()("Test Result Check FAILED! (%u/%lu)\n", id + 1,
                outputs.size());
        }
        offset += size;
    }

    return pass;
}

/**
 * @brief create multiple level directory for specific benchmark case, thread safe
 */
int help_create_dir(const char *path)
{
    const char *delim = "/";
    char data[1024] = {0};
    char new_path[1024] = {0};
    char *p = nullptr;

    semOp_sp->semaphore_p();
    strncpy(data, path, strlen(path));
    // printf("create_dir: %s\n", path);
    if (!strncmp(data, "/", 1))
    {
        const char *split_dim = "/";
        strncpy(new_path, split_dim, 1);
    }

    p = strtok(data, delim);
    strcpy(new_path + strlen(new_path), p);
    strcat(new_path, "/");
    while((p = strtok(NULL, delim))) {
        strcpy(new_path + strlen(new_path), p);
        strcat(new_path, "/");

        // printf("path: %s\n", new_path);
        if(!strcmp(p, ".") || (strlen(p) == 0))
            continue;

        if(access(new_path, F_OK) == 0)
            continue;

        if(mkdir(new_path, 0750) == -1){
            printf("%s: mkdir %s failed\n", __FUNCTION__, new_path);
            semOp_sp->semaphore_v();
            exit(-1);
        }
    }

    semOp_sp->semaphore_v();
    return 0 ;
}