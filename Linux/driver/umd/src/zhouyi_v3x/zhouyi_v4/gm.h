// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef __GM__
#define __GM__

#include <vector>
#include <memory>
#include <pthread.h>
#include "../common/graph_v3x.h"
#include "kmd/tcb.h"

namespace aipudrv
{

enum gm_buf_type {
    EM_GM_BUF_INPUT = 0,
    EM_GM_BUF_MAX
};

struct ValidSyncBuffer
{
    struct SyncBuffer
    {
        DEV_PA_64 sync_pa;
        uint32_t sync_size;
    };
    SyncBuffer valid_sync_buf[EM_GM_BUF_MAX];
};

class JobV4;
class GM_V4
{
    private:
    bool m_gm_asm = false;
    std::vector<BufferDesc *> m_gm_free_buffer;
    std::vector<BufferDesc *> m_gm_alloc_buffer;

    public:
    JobV4 &m_job;
    const GraphV3X &m_graph;

    DEV_PA_64 m_gm_buf_base = 0;
    uint32_t m_gm_buf_sync_size= 0;

    public:
    aipu_status_t gm_malloc(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
        std::string &buf_name, BufferDesc *buf);
    bool gm_is_gm_buffer(uint32_t idx, uint32_t buf_type);
    bool gm_need_remap();
    void get_valid_map_base(BufferDesc &buf);

    public:
    GM_V4(JobV4 &_job);
    ~GM_V4();
};
}

#endif