// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  main.cpp
 * @brief IPU UMD test application: dmabuf vmap test
 */

/**
 * @brief request one dma_buf and filled in user mode firstly, then
 *        filled the same dma_buf in kernel mode.
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <string.h>
#include <errno.h>
#include <vector>
#include <math.h>
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"
#include "kmd/armchina_aipu.h"

using namespace std;

/* dma_buf importer */
#define DEV_IMPORTER "/dev/importer"

/* dma_buf exporter */
#define DEV_EXPORTER "/dev/aipu"

/**
 * the size of requested dma_buf,change it accordingly.
 */
#define DMABUF_SZ (1 << 20)

/**
 * the fd of requested dma_buf.
 */
int dmabuf_fd = 0;

/**
 * request one dma_buf from dma_buf exporter(NUP driver module),
 * record its fd to 'dmabuf_fd'.
 */
int dmabuf_malloc(uint64_t size)
{
    int ret = 0;
    int fd = 0;
    struct aipu_dma_buf_request dma_buf_req = {0};

    fd = open(DEV_EXPORTER, O_RDWR);
    if (fd < 0)
    {
        ret = -1;
        AIPU_ERR() << "open " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    dma_buf_req.bytes = size;
    ret = ioctl(fd, AIPU_IOCTL_ALLOC_DMA_BUF, &dma_buf_req);
    if (ret < 0)
    {
        AIPU_ERR() << "ioctl " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    dmabuf_fd = dma_buf_req.fd;

out:
    close(fd);
    return ret;
}

/**
 * free allocated dma_buf
 */
int dmabuf_free(int _fd)
{
    int ret = 0;
    int fd = 0;

    fd = open(DEV_EXPORTER, O_RDWR);
    if (fd < 0)
    {
        ret = -1;
        AIPU_ERR() << "open " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    ret = ioctl(fd, AIPU_IOCTL_FREE_DMA_BUF, &_fd);
    if (ret < 0)
    {
        AIPU_ERR() << "ioctl " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

out:
    close(fd);
    return ret;
}

/**
 * map physical pages of requested dma_buf to user mode,
 * then fill its with input data which is taken as model's
 * input data.
 */
int dmabuf_fill(int fd, const char *data, uint32_t size)
{
    int ret = 0;
    char *va = nullptr;

    va = (char *)mmap(NULL, DMABUF_SZ, PROT_READ|PROT_WRITE, MAP_SHARED, dmabuf_fd, 0);
    if (va == MAP_FAILED)
    {
        ret = -1;
        AIPU_ERR() << "mmap dmabuf [fail]\n";
        goto out;
    }

    memcpy(va, data, size);
    munmap(va, DMABUF_SZ);

out:
    return ret;
}

/**
 * map physical pages of requested dma_buf to user mode,
 * then read the content of the dma_buf.
 */
int dmabuf_read(int fd)
{
    int ret = 0;
    char *va = nullptr;

    va = (char *)mmap(NULL, DMABUF_SZ, PROT_READ|PROT_WRITE, MAP_SHARED, dmabuf_fd, 0);
    if (va == MAP_FAILED)
    {
        ret = -1;
        AIPU_ERR() << "mmap dmabuf [fail]\n";
        goto out;
    }

    printf("read from dma_buf: %s\n", va);
    munmap(va, DMABUF_SZ);

out:
    return ret;
}

int main(int argc, char* argv[])
{
    int ret = -1;
    int fd = 0;
    const char *magic = "This is string wroten by user!";

    if (dmabuf_malloc(DMABUF_SZ) < 0)
    {
        AIPU_ERR() << "dmabuf_malloc [fail]\n";
        goto out;
    }

    if (dmabuf_fill(dmabuf_fd, magic, strlen(magic) + 1) != 0)
    {
        AIPU_ERR() << "dmabuf_fill [fail]\n";
        goto out;
    }

    // read the content filled in user mode
    dmabuf_read(dmabuf_fd);

    fd = open(DEV_IMPORTER, O_RDONLY);
    if (fd < 0)
    {
        AIPU_ERR()("open %s [fail]\n", DEV_IMPORTER);
        goto out;
    }

    // request kernel to fill special content to dma_buf
    ret = ioctl(fd, 0, &dmabuf_fd);
    if (ret < 0)
    {
        AIPU_ERR()("ioctl %s [fail]\n", DEV_IMPORTER);
        goto out;
    }
    close(fd);

    // read the content filled in kernel mode
    dmabuf_read(dmabuf_fd);

    dmabuf_free(dmabuf_fd);

    ret = 0;
out:
    return ret;
}