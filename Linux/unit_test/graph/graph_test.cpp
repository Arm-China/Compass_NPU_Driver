// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
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
    aipu_global_config_hw_t m_hw_cfg = {0};
    JOB_ID id;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    ret = p_gobj->create_job(&id, &m_sim_cfg, &m_hw_cfg, &create_job_cfg);
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

TEST_CASE_FIXTURE(GraphTest, "get_batch_queue_id")
{
    aipu_status_t ret;
    uint32_t queue_id = 0;

    p_gobj->load(gbin, fsize, m_do_vcheck);

    ret = p_gobj->get_batch_queue_id(&queue_id);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "add_batch")
{
    aipu_status_t ret;
    char **input_buf, **output_buf[1];
    vector<aipu_tensor_desc_t> desc_vec;
    aipu_tensor_desc_t desc;
    uint32_t queue_id = 0;
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_OUTPUT, &cnt);
    for (uint32_t i = 0; i < cnt; i++) {
        p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
        desc_vec.push_back(desc);
    }
    p_gobj->get_batch_queue_id(&queue_id);
    input_buf = new char*[1];
    output_buf[0] = new char*[cnt];
    for (uint32_t i = 0; i < cnt; i++) {
        output_buf[0][i] = new char[desc_vec[i].size];
    }
    input_buf[0] = input_dest;
    ret = p_gobj->add_batch(queue_id, input_buf, 1, output_buf[0], cnt);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "config_for_batch")
{
    aipu_status_t ret;
    uint32_t queue_id = 0;

    aipu_job_config_dump_t mem_dump_config;
    memset(&mem_dump_config, 0, sizeof(mem_dump_config));
    mem_dump_config.dump_dir = "./";
    p_gobj->load(gbin, fsize, m_do_vcheck);
    p_gobj->get_batch_queue_id(&queue_id);

    ret =  p_gobj->config_for_batch(queue_id, AIPU_CONFIG_TYPE_SIMULATION, &mem_dump_config);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "clean_batch_queue")
{
    aipu_status_t ret;
    char **input_buf, **output_buf[1];
    vector<aipu_tensor_desc_t> desc_vec;
    aipu_tensor_desc_t desc;
    uint32_t queue_id = 0;
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_OUTPUT, &cnt);
    for (uint32_t i = 0; i < cnt; i++) {
        p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
        desc_vec.push_back(desc);
    }
    p_gobj->get_batch_queue_id(&queue_id);
    input_buf = new char*[1];
    output_buf[0] = new char*[cnt];
    for (uint32_t i = 0; i < cnt; i++) {
        output_buf[0][i] = new char[desc_vec[i].size];
    }
    input_buf[0] = input_dest;
    p_gobj->add_batch(queue_id, input_buf, 1, output_buf[0], cnt);

    ret = p_gobj->clean_batch_queue(100);
    CHECK(ret == AIPU_STATUS_ERROR_NO_BATCH_QUEUE);

    ret = p_gobj->clean_batch_queue(queue_id);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(GraphTest, "is_valid_batch_queue")
{
    bool ret;
    char **input_buf, **output_buf[1];
    vector<aipu_tensor_desc_t> desc_vec;
    aipu_tensor_desc_t desc;
    uint32_t queue_id = 0;
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_OUTPUT, &cnt);
    for (uint32_t i = 0; i < cnt; i++) {
        p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
        desc_vec.push_back(desc);
    }
    p_gobj->get_batch_queue_id(&queue_id);
    input_buf = new char*[1];
    output_buf[0] = new char*[cnt];
    for (uint32_t i = 0; i < cnt; i++) {
        output_buf[0][i] = new char[desc_vec[i].size];
    }
    input_buf[0] = input_dest;
    p_gobj->add_batch(queue_id, input_buf, 1, output_buf[0], cnt);

    ret = p_gobj->is_valid_batch_queue(100);
    CHECK(ret == false);

    ret = p_gobj->is_valid_batch_queue(queue_id);
    CHECK(ret == true);
}

TEST_CASE_FIXTURE(GraphTest, "get_batch_queue_size")
{
    uint32_t batch_queue_size = 0;
    char **input_buf, **output_buf[1];
    vector<aipu_tensor_desc_t> desc_vec;
    aipu_tensor_desc_t desc;
    uint32_t queue_id = 0;
    uint32_t cnt;

    p_gobj->load(gbin, fsize, m_do_vcheck);
    p_gobj->get_tensor_count(AIPU_TENSOR_TYPE_OUTPUT, &cnt);
    for (uint32_t i = 0; i < cnt; i++) {
        p_gobj->get_tensor_descriptor(AIPU_TENSOR_TYPE_OUTPUT, i, &desc);
        desc_vec.push_back(desc);
    }
    p_gobj->get_batch_queue_id(&queue_id);
    input_buf = new char*[1];
    output_buf[0] = new char*[cnt];
    for (uint32_t i = 0; i < cnt; i++) {
        output_buf[0][i] = new char[desc_vec[i].size];
    }
    input_buf[0] = input_dest;
    p_gobj->add_batch(queue_id, input_buf, 1, output_buf[0], cnt);

    batch_queue_size = p_gobj->get_batch_queue_size(queue_id);
    CHECK(batch_queue_size != 0);
}