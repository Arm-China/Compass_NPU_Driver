// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>
#include "graph_test.h"
#include "aipu.h"

TEST_CASE_FIXTURE(GraphTest, "load")
{
    aipu_status_t ret;

    ret = p_gobj->load(gbin, 0, m_do_vcheck);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_GBIN);

    ret = p_gobj->load(gbin, fsize, m_do_vcheck);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "unload")
{
    aipu_status_t ret;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    ret = p_gobj->unload();
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "create_job")
{
    aipu_status_t ret;
    aipu_create_job_cfg create_job_cfg = {0};
    JOB_ID id;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    ret = p_gobj->create_job(&id, &m_sim_cfg, &create_job_cfg);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "get_tensor_count")
{
    aipu_status_t ret;
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);

    ret = p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_INPUT, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_INPUT, &cnt);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "get_tensor_descriptor")
{
    aipu_status_t ret;
    aipu_tensor_desc_t desc;
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_INPUT, &cnt);

    ret = p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_INPUT, 0, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_INPUT, cnt, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_TENSOR_ID);

    ret = p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_INPUT, 0, &desc);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

#if (defined ZHOUYI_V3)
TEST_CASE_FIXTURE(GraphTest, "get_subgraph_cnt")
{
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);

    cnt = graphv3->get_subgraph_cnt();
    CHECK((cnt != 0) == true);
}
#endif
