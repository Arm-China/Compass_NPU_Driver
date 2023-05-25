// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#include <stdlib.h>
#include "context_test.h"
#include "standard_api.h"
#include "aipu.h"

TEST_CASE_FIXTURE(ContextTest, "init")
{
    aipu_status_t ret;

    ret = p_ctx->init();
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(ContextTest, "deinit")
{
    aipu_status_t ret;

    p_ctx->init();
    ret = p_ctx->deinit();
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(ContextTest, "get_status_msg")
{
    aipu_status_t status = AIPU_STATUS_ERROR_INVALID_OP;
    const char* msg = nullptr;
    aipu_status_t ret;

    ret = p_ctx->get_status_msg(status, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->get_status_msg(status, &msg);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(ContextTest, "get_graph_object")
{
    GRAPH_ID id = 1;

    GraphBase* p_gobj = p_ctx->get_graph_object(id);
    CHECK((p_gobj == nullptr) == true);
}

TEST_CASE_FIXTURE(ContextTest, "get_job_object")
{
    JOB_ID id = 1;

    JobBase* p_gobj = p_ctx->get_job_object(id);
    CHECK((p_gobj == nullptr) == true);
}

TEST_CASE_FIXTURE(ContextTest, "load_graph")
{
    string graph_file = "./benchmark/aipu.bin";
    aipu_status_t ret;
    uint64_t graph_id;

    p_ctx->init();

    ret = p_ctx->load_graph(nullptr, &graph_id);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->load_graph(graph_file.c_str(), nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->load_graph(graph_file.c_str(), &graph_id);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(ContextTest, "unload_graph")
{
    string graph_file = "./benchmark/aipu.bin";
    aipu_status_t ret;
    uint64_t graph_id;

    p_ctx->init();

    ret = p_ctx->unload_graph(0);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_GRAPH_ID);

    ret = p_ctx->load_graph(graph_file.c_str(), &graph_id);
    ret = p_ctx->unload_graph(graph_id);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(ContextTest, "create_job")
{
    string graph_file = "./benchmark/aipu.bin";
    JOB_ID job_id = 0;
    aipu_status_t ret;
    aipu_create_job_cfg create_job_cfg = {0};
    uint64_t graph_id;

    p_ctx->init();
    ret = p_ctx->load_graph(graph_file.c_str(), &graph_id);

    ret = p_ctx->create_job(graph_id, nullptr, &create_job_cfg);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->create_job(1, &job_id, &create_job_cfg);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_GRAPH_ID);

#if (defined SIMULATION)
    aipu_global_config_simulation_t sim_glb_config;
    memset(&sim_glb_config, 0, sizeof(sim_glb_config));
#if (defined ZHOUYI_V12)
    sim_glb_config.simulator = "./simulator/aipu_simulator_x1";
    sim_glb_config.log_level = 3;
    ret = p_ctx->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
#endif
#if (defined ZHOUYI_V3)
    sim_glb_config.log_level = 3;
    ret = p_ctx->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
#endif
#endif
    ret = p_ctx->create_job(graph_id, &job_id, &create_job_cfg);

    CHECK(ret == AIPU_STATUS_SUCCESS);
}

#if (defined SIMULATION)
TEST_CASE_FIXTURE(ContextTest, "config_simulation")
{
    aipu_global_config_simulation_t sim_glb_config;
    memset(&sim_glb_config, 0, sizeof(sim_glb_config));
    aipu_status_t ret;

    p_ctx->init();
#if (defined ZHOUYI_V12)
    sim_glb_config.simulator = "./simulator/aipu_simulator_x1";
    sim_glb_config.log_level = 3;
#endif
#if (defined ZHOUYI_V3)
    sim_glb_config.log_level = 3;
    sim_glb_config.x2_arch_desc = "X2_1204";
#endif

    ret = p_ctx->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}
#endif

TEST_CASE_FIXTURE(ContextTest, "get_cluster_count")
{
    aipu_status_t ret;
    uint32_t partition_id = 0;

    p_ctx->init();

    ret = p_ctx->get_cluster_count(partition_id, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);
#ifndef SIMULATION
    uint32_t cluster_count = 0;
    ret = p_ctx->get_cluster_count(partition_id, &cluster_count);
    CHECK(ret == AIPU_STATUS_SUCCESS);
#endif
}

#ifndef SIMULATION
TEST_CASE_FIXTURE(ContextTest, "get_core_count")
{
    uint32_t cluster = 0;
    uint32_t core_cnt = 0;
    uint32_t partition_id = 0;
    aipu_status_t ret;

    p_ctx->init();

    ret = p_ctx->get_core_count(partition_id, cluster, nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->get_core_count(partition_id, 5, &core_cnt);
    CHECK(ret == AIPU_STATUS_ERROR_INVALID_CLUSTER_ID);

    ret = p_ctx->get_core_count(partition_id, cluster, &core_cnt);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}

TEST_CASE_FIXTURE(ContextTest, "get_partition_count")
{
    uint32_t partition_cnt = 0;
    aipu_status_t ret;

    p_ctx->init();

    ret = p_ctx->get_partition_count(nullptr);
    CHECK(ret == AIPU_STATUS_ERROR_NULL_PTR);

    ret = p_ctx->get_partition_count(&partition_cnt);
    CHECK(ret == AIPU_STATUS_SUCCESS);
}
#endif
