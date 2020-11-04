// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef __GM__
#define __GM__

#include <vector>
#include <memory>
#include <pthread.h>
#include "graph_v3.h"
#include "kmd/tcb.h"

namespace aipudrv
{

enum gm_buf_type {
    EM_GM_BUF_INPUT = 0,
    EM_GM_BUF_OUTPUT,
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

class JobV3;
class GM_V3
{
    private:
    bool m_small_model = false;
    std::vector<BufferDesc *> m_gm_free_buffer;
    std::vector<BufferDesc *> m_gm_alloc_buffer;

    public:
    JobV3 &m_job;
    const GraphV3 &m_graph;

    /**
     * index 0: record the input info, DDR->GM
     * index 1: record the output info, GM->DDR
     */
    DEV_PA_64 m_gm_base = 0;
    DEV_PA_64 m_gm_buf_base_pa[EM_GM_BUF_MAX] = {0};
    uint32_t m_gm_buf_map_size[EM_GM_BUF_MAX] = {0};
    int m_gm_sync_buf_cnt[EM_GM_BUF_MAX] = {0};

    public:
    aipu_status_t gm_malloc(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
        std::string &buf_name, BufferDesc &buf);
    bool gm_is_gm_buffer(uint32_t idx, uint32_t buf_type);
    void get_valid_sync_region(uint32_t sg_id, uint32_t idx, uint32_t buf_type,
        BufferDesc &buf, ValidSyncBuffer &region);
    void get_valid_map_base(BufferDesc &buf);
    bool gm_need_remap();
    bool gm_need_sync_out();

    public:
    GM_V3(JobV3 &_job);
    ~GM_V3();
};
}

#endif