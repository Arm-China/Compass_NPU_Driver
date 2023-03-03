// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>
#include "job_test.h"
#include "standard_api.h"
#include "aipu.h"

TEST_CASE_FIXTURE(JobTest, "init")
{
    aipu_status_t ret;

    ret = p_job->init(&m_sim_cfg);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

#if (defined ZHOUYI_V3)
TEST_CASE_FIXTURE(JobTest, "get_part_id")
{
    uint32_t part_id;

    p_job->init(&m_sim_cfg);
    part_id = job_v3->get_part_id();
    CHECK((part_id == 0) == true);
}

TEST_CASE_FIXTURE(JobTest, "get_qos")
{
    uint32_t qos;

    p_job->init(&m_sim_cfg);
    qos = job_v3->get_qos();
    CHECK((qos == 0) == true);
}
#endif

TEST_CASE_FIXTURE(JobTest, "load_tensor")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg);

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

    p_job->init(&m_sim_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif

    ret = p_job->schedule();
    CHECK(ret == AIPU_STATUS_SUCCESS);
#ifndef SIMULATION
    aipu_job_status_t status;
    p_job->get_status_blocking(&status, -1);
#endif
}

TEST_CASE_FIXTURE(JobTest, "get_status_blocking")
{
    aipu_status_t ret;
    aipu_job_status_t status;

    p_job->init(&m_sim_cfg);
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

    p_job->init(&m_sim_cfg);
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

    p_job->init(&m_sim_cfg);
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

    p_job->init(&m_sim_cfg);
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

    p_job->init(&m_sim_cfg);
    p_job->load_tensor(0, input_file.c_str());

    ret = p_job->config_mem_dump(AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, nullptr);
    CHECK(ret == AIPU_STATUS_SUCCESS);

    ret = p_job->config_mem_dump(AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, &mem_dump_config);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "bind_core")
{
    aipu_status_t ret;

    p_job->init(&m_sim_cfg);
    p_job->load_tensor(0, input_file.c_str());
#if (defined SIMULATION)
    p_job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
#endif

    ret = p_job->bind_core(0);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(JobTest, "mark_shared_tensor")
{
    aipu_status_t ret;
    DEV_PA_64 addr = 0;

    p_job->init(&m_sim_cfg);

    ret = p_job->mark_shared_tensor(AIPU_TENSOR_TYPE_INPUT, 0, addr);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}