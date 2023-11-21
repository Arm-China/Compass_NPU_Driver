// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>
#include "job_test.h"
#include "standard_api.h"
#include "aipu.h"

TEST_CASE_FIXTURE(JobTest, "init")
{
    aipu_status_t ret;

    ret = p_job->init(&m_sim_cfg, &m_hw_cfg);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

#if (defined ZHOUYI_V3)
TEST_CASE_FIXTURE(JobTest, "get_part_id")
{
    uint32_t part_id;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    part_id = job_v3->get_part_id();
    CHECK((part_id == 0) == true);
}

TEST_CASE_FIXTURE(JobTest, "get_qos")
{
    uint32_t qos;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    qos = job_v3->get_qos();
    CHECK((qos == 0) == true);
}
#endif

TEST_CASE_FIXTURE(JobTest, "load_tensor")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg, &m_hw_cfg);

    ret = p_job->load_tensor(0, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_job->load_tensor(3, input_file.c_str());
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_TENSOR_ID);

    ret = p_job->load_tensor(0, input_file.c_str());
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "schedule")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif

    ret = p_job->schedule();
    CHECK(ret == AIPU_STATUS_SUCCESS);
    aipu_job_status_t status;
    p_job->get_status_blocking(&status, -1);
}

TEST_CASE_FIXTURE(JobTest, "get_status_blocking")
{
    aipu_status_t ret;
    aipu_job_status_t status;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif
    p_job->schedule();

    ret = p_job->get_status_blocking(&status, -1);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "destroy")
{
    aipu_status_t ret;
    aipu_job_status_t status;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif
    p_job->schedule();

    p_job->get_status_blocking(&status, -1);
    ret = p_job->destroy();
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "get_tensor")
{
    aipu_status_t ret;
    aipu_job_status_t status;
    char* output_data;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif
    p_job->schedule();
    p_job->get_status_blocking(&status, -1);
    output_data = new char[out_desc.size];

    ret = p_job->get_tensor(AIPU_TENSOR_TYPE_OUTPUT, 0, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_job->get_tensor(AIPU_TENSOR_TYPE_OUTPUT, 5, output_data);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_TENSOR_ID);

    ret = p_job->get_tensor(AIPU_TENSOR_TYPE_OUTPUT, 0, output_data);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

#if (defined SIMULATION)
TEST_CASE_FIXTURE(JobTest, "config_simulation")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());

#if (defined ZHOUYI_V12)
    ret = p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);
#endif

    ret = p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}
#endif

TEST_CASE_FIXTURE(JobTest, "config_mem_dump")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());

    ret = p_job->config_mem_dump(AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, nullptr);
    CHECK(ret == AIPU_STATUS_SUCCESS);

    ret = p_job->config_mem_dump(AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, &mem_dump_config);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "bind_core")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg, &m_hw_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif

    ret = p_job->bind_core(0);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "specify_io_buffer")
{
    aipu_status_t ret;
    aipu_share_buf_t share_buf = {0};
    aipu_shared_tensor_info_t tensor_info;

    memset(&tensor_info, 0, sizeof(tensor_info));
    share_buf.size = 1 << 20;
    share_buf.pa = 0x3f6f000;
    p_job->init(&m_sim_cfg, &m_hw_cfg);

    tensor_info.pa = share_buf.pa;
    tensor_info.tensor_idx = 0;
    tensor_info.shared_case_type = 0x3;;
    tensor_info.type = AIPU_TENSOR_TYPE_INPUT;
    ret = p_job->specify_io_buffer(tensor_info);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_OP);

    tensor_info.pa = share_buf.pa;
    tensor_info.tensor_idx = 10;
    tensor_info.shared_case_type = AIPU_SHARE_BUF_IN_ONE_PROCESS;;
    tensor_info.type = AIPU_TENSOR_TYPE_INPUT;
    ret = p_job->specify_io_buffer(tensor_info);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_TENSOR_ID);

    tensor_info.pa = share_buf.pa;
    tensor_info.tensor_idx = 0;
    tensor_info.shared_case_type = AIPU_SHARE_BUF_IN_ONE_PROCESS;;
    tensor_info.type = AIPU_TENSOR_TYPE_PROFILER;
    ret = p_job->specify_io_buffer(tensor_info);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_TENSOR_ID);

    tensor_info.pa = share_buf.pa;
    tensor_info.tensor_idx = 0;
    tensor_info.shared_case_type = AIPU_SHARE_BUF_IN_ONE_PROCESS;;
    tensor_info.type = AIPU_TENSOR_TYPE_INPUT;
    ret = p_job->specify_io_buffer(tensor_info);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

