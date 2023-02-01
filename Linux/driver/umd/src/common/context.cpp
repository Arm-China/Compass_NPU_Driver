// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  context.cpp
 * @brief AIPU User Mode Driver (UMD) context module implementation
 */

#include <set>
#include <queue>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include "context.h"
#include "type.h"
#include "utils/log.h"
#include "utils/debug.h"
#include "utils/helper.h"
#include "device.h"
#include "graph_legacy.h"
#include "graph_v3.h"
#include "super_graph.h"
#include "job_base.h"
#include "parser_base.h"

volatile int32_t UMD_LOG_LEVEL = LOG_WARN;
aipudrv::MainContext::MainContext()
{
    const char *umd_log_level_env = getenv("UMD_LOG_LEVEL");

    m_dev = nullptr;
    pthread_rwlock_init(&m_glock, NULL);
    m_sim_cfg.z1_simulator = nullptr;
    m_sim_cfg.z2_simulator = nullptr;
    m_sim_cfg.z3_simulator = nullptr;
    m_sim_cfg.x1_simulator = nullptr;
    m_sim_cfg.x2_arch_desc = nullptr;
    m_sim_cfg.log_file_path = new char[1024];
    strcpy((char*)m_sim_cfg.log_file_path, "./");
    m_sim_cfg.log_level = 0;
    m_sim_cfg.verbose = false;
    m_sim_cfg.enable_avx = false;
    m_sim_cfg.enable_calloc = false;
    m_sim_cfg.en_eval = false;

    if (umd_log_level_env != nullptr)
    {
        int32_t log_level = umd_log_level_env[0] - '0';
        if (log_level > LOG_WARN && log_level <= LOG_CLOSE)
            UMD_LOG_LEVEL = log_level;
    }
}

aipudrv::MainContext::~MainContext()
{
    pthread_rwlock_destroy(&m_glock);
    if (m_sim_cfg.z1_simulator != nullptr)
        delete[] m_sim_cfg.z1_simulator;

    if (m_sim_cfg.z2_simulator != nullptr)
        delete[] m_sim_cfg.z2_simulator;

    if (m_sim_cfg.z3_simulator != nullptr)
        delete[] m_sim_cfg.z3_simulator;

    if (m_sim_cfg.x1_simulator != nullptr)
        delete[] m_sim_cfg.x1_simulator;

    if (m_sim_cfg.x2_arch_desc != nullptr)
        delete[] m_sim_cfg.x2_arch_desc;

    delete[] m_sim_cfg.log_file_path;
}

bool aipudrv::MainContext::is_deinit_ok()
{
    return true;
}

aipu_status_t aipudrv::MainContext::init()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    /* arm64 platform init m_dev here; simulation init m_dev later */
    ret = get_device(&m_dev);
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    /* init m_dram here as debugger may allocate buffer after contex initializing done */
    #ifndef SIMULATION
    if (m_dev->get_mem() != nullptr)
        m_dram = m_dev->get_mem();
    #endif

    return ret;
}

void aipudrv::MainContext::force_deinit()
{
    GraphTable::iterator iter;

    pthread_rwlock_wrlock(&m_glock);
    for (iter = m_graphs.begin(); iter != m_graphs.end(); iter++)
        iter->second->unload();

    m_graphs.clear();
    pthread_rwlock_unlock(&m_glock);
    if (m_dev != nullptr)
    {
        put_device(m_dev);
        m_dev = nullptr;
        m_dram = nullptr;
    }
}

aipu_status_t aipudrv::MainContext::deinit()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (!is_deinit_ok())
    {
        ret = AIPU_STATUS_ERROR_DEINIT_FAIL;
        goto finish;
    }

    force_deinit();

finish:
    return ret;
}

aipu_status_t aipudrv::MainContext::get_status_msg(aipu_status_t status, const char** msg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (nullptr == msg)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    if ((status > AIPU_STATUS_MAX) && (status == m_last_err))
        *msg = m_last_err_msg;
    else
        *msg = get_static_msg(status);

finish:
    return ret;
}

uint64_t aipudrv::MainContext::create_unique_graph_id_inner() const
{
    uint64_t id_candidate = (1UL << 32);

    while (m_graphs.count(id_candidate) == 1)
        id_candidate += (1UL << 32);

    return id_candidate;
}

aipu_status_t aipudrv::MainContext::create_graph_object(std::istream& gbin, uint32_t size,
    uint64_t id, GraphBase** gobj)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* p_gobj = nullptr;
    uint32_t g_version = 0;

    if (nullptr == gobj)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    g_version = ParserBase::get_graph_bin_version(gbin);
    ret = test_get_device(g_version, &m_dev, &m_sim_cfg);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    m_dram = m_dev->get_mem();

#if (defined ZHOUYI_V12)
    if (AIPU_LOADABLE_GRAPH_V0005 == g_version)
        p_gobj = new GraphLegacy(this, id, m_dev);
#endif
#if (defined ZHOUYI_V3)
    if (AIPU_LOADABLE_GRAPH_ELF_V0 == g_version)
        p_gobj = new GraphV3(this, id, m_dev);
#endif

    if (nullptr == p_gobj)
    {
        ret = AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED;
        goto finish;
    }

    ret = p_gobj->load(gbin, size, m_do_vcheck);
    if (AIPU_STATUS_SUCCESS != ret)
    {
        destroy_graph_object(&p_gobj);
        goto finish;
    }

    /* success or return nullptr */
    *gobj = p_gobj;

finish:
    return ret;
}

aipudrv::GraphBase* aipudrv::MainContext::get_graph_object(GRAPH_ID id)
{
    GraphBase* p_gobj = nullptr;
    pthread_rwlock_rdlock(&m_glock);
    if (0 != m_graphs.count(id))
        p_gobj = m_graphs[id];
    pthread_rwlock_unlock(&m_glock);

    return p_gobj;
}

aipudrv::JobBase* aipudrv::MainContext::get_job_object(JOB_ID id)
{
    GraphBase* p_gobj = get_graph_object(job_id2graph_id(id));
    if (p_gobj == nullptr)
        return nullptr;

    return p_gobj->get_job(id);
}

aipu_status_t aipudrv::MainContext::destroy_graph_object(GraphBase** gobj)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if ((nullptr == gobj) || (nullptr == (*gobj)))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    ret = (*gobj)->unload();
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* success */
    delete *gobj;
    *gobj = nullptr;

finish:
    return ret;
}

aipu_status_t aipudrv::MainContext::load_graph(const char* graph_file, GRAPH_ID* id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* gobj = nullptr;
    uint64_t _id = 0;
    std::ifstream gbin;
    int fsize = 0;

    if ((nullptr == graph_file) || (nullptr == id))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    gbin.open(graph_file, std::ifstream::in | std::ifstream::binary);
    if (!gbin.is_open())
        return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;

    gbin.seekg (0, gbin.end);
    fsize = gbin.tellg();
    gbin.seekg (0, gbin.beg);

    /* push nullptr into graphs to pin this graph ID */
    pthread_rwlock_wrlock(&m_glock);
    _id = create_unique_graph_id_inner();
    m_graphs[_id] = nullptr;
    pthread_rwlock_unlock(&m_glock);

    ret = create_graph_object(gbin, fsize, _id, &gobj);
    if (AIPU_STATUS_SUCCESS != ret)
    {
        pthread_rwlock_wrlock(&m_glock);
        m_graphs.erase(_id);
        pthread_rwlock_unlock(&m_glock);
        goto finish;
    }

    /* success: update graphs[_id] */
    pthread_rwlock_wrlock(&m_glock);
    m_graphs[_id] = gobj;
    pthread_rwlock_unlock(&m_glock);
    *id = _id;

    /* TBD */
    return ret;

finish:
    gbin.close();
    return ret;
}

aipu_status_t aipudrv::MainContext::load_graph(const char* graph_buf, uint32_t graph_size, GRAPH_ID* id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* gobj = nullptr;
    uint64_t _id = 0;
    std::stringstream gbin;
    std::string *gbin_str = nullptr;

    if ((nullptr == graph_buf) || (nullptr == id))
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    if ((graph_size <= 0) || (graph_size > UINT32_MAX))
    {
        ret = AIPU_STATUS_ERROR_INVALID_SIZE;
        goto finish;
    }

    /* convert graph data array to stringstream */
    gbin_str = new std::string(graph_buf, graph_size);
    gbin << *gbin_str;

    /* push nullptr into graphs to pin this graph ID */
    pthread_rwlock_wrlock(&m_glock);
    _id = create_unique_graph_id_inner();
    m_graphs[_id] = nullptr;
    pthread_rwlock_unlock(&m_glock);

    ret = create_graph_object(gbin, graph_size, _id, &gobj);
    if (AIPU_STATUS_SUCCESS != ret)
    {
        pthread_rwlock_wrlock(&m_glock);
        m_graphs.erase(_id);
        pthread_rwlock_unlock(&m_glock);
        goto finish;
    }

    /* success: update graphs[_id] */
    pthread_rwlock_wrlock(&m_glock);
    m_graphs[_id] = gobj;
    pthread_rwlock_unlock(&m_glock);
    *id = _id;

finish:
    delete gbin_str;
    return ret;
}

aipu_status_t aipudrv::MainContext::unload_graph(GRAPH_ID id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* p_gobj = nullptr;

    p_gobj = get_graph_object(id);
    if (nullptr == p_gobj)
    {
        ret = AIPU_STATUS_ERROR_INVALID_GRAPH_ID;
        goto finish;
    }

    ret = destroy_graph_object(&p_gobj);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    /* p_gobj becomes NULL after destroy */

    /* success */
    pthread_rwlock_wrlock(&m_glock);
    m_graphs.erase(id);
    pthread_rwlock_unlock(&m_glock);

finish:
    return ret;
}

aipu_status_t aipudrv::MainContext::create_job(GRAPH_ID graph, JOB_ID* id, aipu_create_job_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* p_gobj = nullptr;

    if (nullptr == id)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    p_gobj = get_graph_object(graph);
    if (nullptr == p_gobj)
    {
        ret = AIPU_STATUS_ERROR_INVALID_GRAPH_ID;
        goto finish;
    }

    ret = p_gobj->create_job(id, &m_sim_cfg, config);

finish:
    return ret;
}

aipu_status_t aipudrv::MainContext::get_simulation_instance(void** simulator, void** memory)
{
    return m_dev->get_simulation_instance(simulator, memory);
}

aipu_status_t aipudrv::MainContext::get_partition_count(uint32_t* cnt)
{
    return m_dev->get_partition_count(cnt);
}

aipu_status_t aipudrv::MainContext::get_cluster_count(uint32_t partition_id, uint32_t* cnt)
{
    return m_dev->get_cluster_count(partition_id, cnt);
}

aipu_status_t aipudrv::MainContext::get_core_count(uint32_t partition_id, uint32_t cluster, uint32_t* cnt)
{
    return m_dev->get_core_count(partition_id, cluster, cnt);
}

aipu_status_t aipudrv::MainContext::get_core_info(uint32_t id, aipu_core_info_t* info)
{
    return m_dev->get_core_info(id, info);
}

aipu_status_t aipudrv::MainContext::debugger_get_job_info(uint64_t job, aipu_debugger_job_info_t* info)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* p_gobj = get_graph_object(get_graph_id(job));
    if (nullptr == p_gobj)
    {
        ret = AIPU_STATUS_ERROR_INVALID_JOB_ID;
        goto finish;
    }

    if (nullptr == info)
    {
        ret = AIPU_STATUS_ERROR_NULL_PTR;
        goto finish;
    }

    ret = get_simulation_instance(&info->simulation_aipu, &info->simulation_mem_engine);
    if (AIPU_STATUS_SUCCESS != ret)
        goto finish;

    info->instr_base = p_gobj->debugger_get_instr_base() & 0xffffffffUL;

finish:
    return ret;
}

aipu_status_t aipudrv::MainContext::config_simulation(uint64_t types, aipu_global_config_simulation_t* config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    char *sim_npu_arch_env = getenv("SIM_NPU_ARCH");

    if (nullptr == config)
        return AIPU_STATUS_ERROR_INVALID_JOB_ID;

    if (config->z1_simulator != nullptr)
    {
        if (m_sim_cfg.z1_simulator == nullptr)
            m_sim_cfg.z1_simulator = new char[1024];

        strcpy((char*)m_sim_cfg.z1_simulator, config->z1_simulator);
    }
    if (config->z2_simulator != nullptr)
    {
        if (m_sim_cfg.z2_simulator == nullptr)
            m_sim_cfg.z2_simulator = new char[1024];

        strcpy((char*)m_sim_cfg.z2_simulator, config->z2_simulator);
    }
    if (config->z3_simulator != nullptr)
    {
        if (m_sim_cfg.z3_simulator == nullptr)
            m_sim_cfg.z3_simulator = new char[1024];

        strcpy((char*)m_sim_cfg.z3_simulator, config->z3_simulator);
    }
    if (config->x1_simulator != nullptr)
    {
        if (m_sim_cfg.x1_simulator == nullptr)
            m_sim_cfg.x1_simulator = new char[1024];

        strcpy((char*)m_sim_cfg.x1_simulator, config->x1_simulator);
    }
    if (config->log_file_path != nullptr)
    {
        strcpy((char*)m_sim_cfg.log_file_path, config->log_file_path);
    }

    m_sim_cfg.log_level = config->log_level;
    m_sim_cfg.verbose = config->verbose;
    m_sim_cfg.enable_avx = config->enable_avx;
    m_sim_cfg.enable_calloc = config->enable_calloc;
    m_sim_cfg.en_eval = config->en_eval;

    if ((config->x2_arch_desc != nullptr) || (sim_npu_arch_env != nullptr))
    {
        if (m_sim_cfg.x2_arch_desc == nullptr)
            m_sim_cfg.x2_arch_desc = new char[64];

        if (config->x2_arch_desc != nullptr)
            strcpy((char*)m_sim_cfg.x2_arch_desc, config->x2_arch_desc);
        else
            strcpy((char*)m_sim_cfg.x2_arch_desc, sim_npu_arch_env);

        ret = set_target(AIPU_LOADABLE_GRAPH_ELF_V0, &m_dev, &m_sim_cfg);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
    }
out:
    return ret;
}

aipu_status_t aipudrv::MainContext::debugger_malloc(uint32_t size, void** va)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    BufferDesc buf;
    char* alloc_va = nullptr;
    uint32_t core_cnt = 0;

    if ((nullptr == va) || (nullptr == m_dram))
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = m_dram->malloc(size, 1, &buf, "dbg");
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    if (m_dram->pa_to_va(buf.pa, buf.size, &alloc_va) != 0)
        return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;

    /**
     * debugger for Jtag-only need data address 0/1 register (0x14/0x18) as
     * channel between server & client
     *
     * for all cores:
     * dreg0: buffer base address in device space
     * dreg1: magic number requested by debugger
     */
    m_dev->get_core_count(0, 0, &core_cnt);
    if (m_dev->get_npu_version() == AIPU_ISA_VERSION_ZHOUYI_X2)
    {
        m_dev->write_reg(0, 0x0c, buf.pa);
        m_dev->write_reg(0, 0x08, 0x1248FFA5);
    } else {
        for (uint32_t id = 0; id < core_cnt; id++)
        {
            m_dev->write_reg(id, 0x14, buf.pa);
            m_dev->write_reg(id, 0x18, 0x1248FFA5);
        }
    }

    m_dbg_buffers[alloc_va] = buf;
    *va = alloc_va;
    return ret;
}

aipu_status_t aipudrv::MainContext::debugger_free(void* va)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (nullptr == va)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (m_dbg_buffers.count(va) == 0)
        return AIPU_STATUS_ERROR_BUF_FREE_FAIL;

    ret = m_dram->free(&m_dbg_buffers[va], "dbg");
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    m_dbg_buffers.erase(va);
    return ret;
}

aipu_status_t aipudrv::MainContext::aipu_get_target(char *target)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t isa = AIPU_VERSION_ZHOUYI_V1;
    uint32_t config = 904;
    std::string isa_version, arch_cfg;
    std::stringstream config_ss;
    std::set<std::string> arch_set = {
        "Z1_0904", "Z1_1002", "Z1_0701",
        "Z2_1104", "Z2_1002", "Z2_0901",
        "Z3_1104", "Z3_0901", "X1_1204",
        "X2_1204"
    };

    #if SIMULATION
    const char *nul_ptr = "null";
    strncpy(target, nul_ptr, 4);
    return ret;
    #endif

    isa = m_dev->get_npu_version();
    config = m_dev->get_npu_config();

    switch (isa)
    {
        case AIPU_VERSION_ZHOUYI_V1:
            isa_version = "Z1_";
            break;
        case AIPU_VERSION_ZHOUYI_V2:
            isa_version = "Z2_";
            break;
        case AIPU_VERSION_ZHOUYI_V3:
            isa_version = "Z3_";
            break;
        case AIPU_VERSION_ZHOUYI_X1:
            isa_version = "X1_";
            break;
        case AIPU_VERSION_ZHOUYI_X2:
            isa_version = "X2_";
            break;
        default:
            return AIPU_STATUS_ERROR_INVALID_CONFIG;
    }

    config_ss << std::setw(4) << std::setfill('0') << config;
    arch_cfg = isa_version + config_ss.str();
    if (arch_set.count(arch_cfg.c_str()) == 0)
        return AIPU_STATUS_ERROR_INVALID_CONFIG;

    strncpy(target, arch_cfg.c_str(), 7);
    return ret;
}

aipu_status_t aipudrv::MainContext::aipu_get_device_status(device_status_t *status)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    uint32_t reg_addr = 0x4, value = 0;

    if (status == nullptr)
        return AIPU_STATUS_ERROR_NULL_PTR;

    *status = DEV_IDLE;
    #ifdef SIMULATION
    return ret;
    #endif

    if (m_dev->get_npu_version() == AIPU_ISA_VERSION_ZHOUYI_X2)
        reg_addr = 0x804;

    ret = convert_ll_status(m_dev->read_reg(0, reg_addr, &value));

    if (m_dev->get_npu_version() == AIPU_ISA_VERSION_ZHOUYI_X2)
    {
        if (value & X2_CMDPOOL_IDLE)
            *status = DEV_IDLE;
        else if (!(value & X2_CMDPOOL_IDLE))
            ; // *status = DEV_BUSY;

        if (value & X2_CMDPOOL_EXCEPTION)
            *status = DEV_EXCEPTION;
    } else {
        if (value & X1_DEV_IDLE)
            *status = DEV_IDLE;
        else if (!(value & X1_DEV_IDLE))
            *status = DEV_BUSY;

        if (value & X1_DEV_EXCEPTION)
            *status = DEV_EXCEPTION;
    }

    return ret;
}

aipu_status_t aipudrv::MainContext::get_status(JobBase *job, aipu_job_status_t *status)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    const char *msg = nullptr;
    int timeout = 3600 * 50; // prompt per 1 hour

    auto sleep_ms = [](int ms){
            struct timeval delay;
            delay.tv_sec = 0;
            delay.tv_usec = ms * 1000;
            select(0, NULL, NULL, NULL, &delay);
    };

    while ((*status != AIPU_JOB_STATUS_DONE) && (*status != AIPU_JOB_STATUS_EXCEPTION))
    {
        ret = job->get_status(status);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            get_status_msg(ret, &msg);
            LOG(LOG_ERR, "job status: %s\n", msg);
            goto out;
        }

        if ((*status == AIPU_JOB_STATUS_DONE) || (*status == AIPU_JOB_STATUS_EXCEPTION))
            break;

        sleep_ms(20);
        if (--timeout == 0)
        {
            ret = AIPU_STATUS_ERROR_TIMEOUT;
            LOG(LOG_WARN, "job: %p polled over 1h\n", job);
            goto out;
        }
    }

out:
    return ret;
}

aipu_status_t aipudrv::MainContext::run_batch(GraphBase &graph, uint32_t queue_id, aipu_create_job_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    JOB_ID job_id;
    JobBase *job = nullptr;
    aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;
    typedef struct job_info {
        JOB_ID job_id;
        JobBase *job;
        batch_info_t *batch;
    } job_info_t;

    job_info_t job_info_item;
    std::queue<job_info_t> job_queue;
    uint32_t types = 0;
    uint32_t batch_num = 0;
    uint32_t max_in_flight = 3;
    uint32_t batch_queue_size = 0;
    const char *umd_max_batch = getenv("UMD_MAX_BATCH");

    if (!graph.is_valid_batch_queue(queue_id))
        return AIPU_STATUS_ERROR_NO_BATCH_QUEUE;

    if (umd_max_batch != nullptr)
    {
        int max_batch = atoi(umd_max_batch);
        max_in_flight = (max_batch != 0) ? max_batch : max_in_flight;
    }

    batch_queue_size = graph.get_batch_queue_size(queue_id);
    types = graph.get_batch_dump_type(queue_id);
repeat:
    for (; batch_num < batch_queue_size; batch_num++)
    {
        if (job_queue.size() >= max_in_flight)
            break;

        batch_info_t &batch = graph.get_batch_queue_item(queue_id, batch_num);
        ret = graph.create_job(&job_id, &m_sim_cfg, config);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;

        job = graph.get_job(job_id);
        #ifdef SIMULATION
        if (types & AIPU_CONFIG_TYPE_SIMULATION)
        {
            aipu_job_config_simulation_t sim_config = {0};

            sim_config.data_dir = graph.get_batch_dump_path(queue_id);
            ret = job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_config);
            if (ret != AIPU_STATUS_SUCCESS)
                goto out;
        }
        #endif

        if (types & (~AIPU_CONFIG_TYPE_SIMULATION))
        {
            aipu_job_config_dump_t dump_config = {0};

            dump_config.dump_dir = graph.get_batch_dump_path(queue_id);
            ret = job->config_mem_dump(types, &dump_config);
            if (ret != AIPU_STATUS_SUCCESS)
                goto out;
        }

        for (uint32_t in_idx = 0; in_idx < batch.inputs.size(); in_idx++)
        {
            ret = job->load_tensor(in_idx, batch.inputs[in_idx]);
            if (ret != AIPU_STATUS_SUCCESS)
                goto out;
        }

        ret = job->schedule();
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;

        job_info_item.job_id = job_id;
        job_info_item.job = job;
        job_info_item.batch = &batch;
        job_queue.push(job_info_item);
    }

    while (job_queue.size() > 0)
    {
        #ifdef SIMULATION
        uint32_t min_in_flight = (job_queue.size() < max_in_flight) ?
            job_queue.size() : max_in_flight;
        for (uint32_t i = 0; i < min_in_flight; i++)
        {
            job_info_item = job_queue.front();
            job_queue.pop();

            status = AIPU_JOB_STATUS_NO_STATUS;
            ret = get_status(job_info_item.job, &status);
            if (ret != AIPU_STATUS_SUCCESS)
                goto out;

            for (uint32_t out_idx = 0; out_idx < job_info_item.batch->outputs.size(); out_idx++)
            {
                ret = job_info_item.job->get_tensor(AIPU_TENSOR_TYPE_OUTPUT, out_idx,
                    job_info_item.batch->outputs[out_idx]);
                if (ret != AIPU_STATUS_SUCCESS)
                    goto out;
            }

            ret = graph.destroy_job(job_info_item.job_id);
            if (ret != AIPU_STATUS_SUCCESS)
                goto out;
        }
        #else
        job_info_t job_info_item = job_queue.front();
        job_queue.pop();
        status = AIPU_JOB_STATUS_NO_STATUS;
        ret = get_status(job_info_item.job, &status);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;

        if (status == AIPU_JOB_STATUS_EXCEPTION)
        {
            LOG(LOG_ERR, "job exception, check HW status\n");
            goto out;
        }

        for (uint32_t out_idx = 0; out_idx < job_info_item.batch->outputs.size(); out_idx++)
        {
            ret = job_info_item.job->get_tensor(AIPU_TENSOR_TYPE_OUTPUT, out_idx,
                job_info_item.batch->outputs[out_idx]);
            if (ret != AIPU_STATUS_SUCCESS)
                goto out;
        }

        ret = graph.destroy_job(job_info_item.job_id);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
        #endif

        if (batch_num < graph.get_batch_queue_size(queue_id))
            goto repeat;
    }

out:
    for(uint32_t i = 0; i < job_queue.size(); i++)
    {
        job_info_t job_info_item = job_queue.front();
        job_queue.pop();
        graph.destroy_job(job_info_item.job_id);
    }
    graph.clean_batches(queue_id);
    return ret;
}

aipu_status_t aipudrv::MainContext::ioctl_cmd(uint32_t cmd, void *arg)
{
    aipu_status_t ret= AIPU_STATUS_SUCCESS;

    #if SIMULATION
        return ret;
    #endif

   ret = convert_ll_status(m_dev->ioctl_cmd(cmd, arg));
   return ret;
}
