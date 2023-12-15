// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
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
#include "graph_v1v2.h"
#include "graph_v3.h"
#include "super_graph.h"
#include "job_base.h"
#include "parser_base.h"

volatile int32_t UMD_LOG_LEVEL = LOG_WARN;
aipudrv::MainContext::MainContext()
{
    const char *umd_log_level_env = getenv("UMD_LOG_LEVEL");

    m_dev = nullptr;
    m_dram = nullptr;
    pthread_rwlock_init(&m_glock, NULL);
    m_sim_cfg.simulator = nullptr;
    m_sim_cfg.x2_arch_desc = nullptr;
    m_sim_cfg.plugin_name = nullptr;
    m_sim_cfg.json_filename = nullptr;
    m_sim_cfg.log_file_path = new char[BUF_LEN];
    strcpy((char*)m_sim_cfg.log_file_path, "./");
    m_sim_cfg.log_level = 0;
    m_sim_cfg.verbose = false;
    m_sim_cfg.enable_avx = false;
    m_sim_cfg.enable_calloc = false;
    m_sim_cfg.en_eval = false;
    m_sim_cfg.gm_size = 4 * MB_SIZE;

    m_hw_cfg.poll_in_commit_thread = true;

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
    if (m_sim_cfg.simulator != nullptr)
        delete[] m_sim_cfg.simulator;

    if (m_sim_cfg.x2_arch_desc != nullptr)
        delete[] m_sim_cfg.x2_arch_desc;

    if (m_sim_cfg.plugin_name != nullptr)
        delete[] m_sim_cfg.plugin_name;

    if (m_sim_cfg.json_filename != nullptr)
        delete[] m_sim_cfg.json_filename;

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

    m_umd_version = MACRO_UMD_VERSION;

    return ret;
}

void aipudrv::MainContext::force_deinit()
{
    GraphTable::iterator iter;

    pthread_rwlock_wrlock(&m_glock);
    for (iter = m_graphs.begin(); iter != m_graphs.end(); iter++)
        iter->second->unload();

    m_graphs.clear();

    if (put_device(m_dev))
        m_dram = nullptr;

    pthread_rwlock_unlock(&m_glock);
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
    static std::mutex mtex;
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

    {
        std::lock_guard<std::mutex> lock_(mtex);
        if (m_dram == nullptr)
            m_dram = m_dev->get_mem();
    }

#if (defined ZHOUYI_V12)
    if (AIPU_LOADABLE_GRAPH_V0005 == g_version)
        p_gobj = new GraphV12(this, id, m_dev);
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

    gbin.seekg(0, gbin.end);
    fsize = gbin.tellg();
    gbin.seekg(0, gbin.beg);

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

    ret = p_gobj->create_job(id, &m_sim_cfg, &m_hw_cfg, config);

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

    // aipu v3 GM size
    std::set<uint32_t> gm_sz_cfg = {
        512 << 10, // 512KB
        1 << 20,   // 1MB
        2 << 20,
        4 << 20,
        8 << 20,
        16 << 20,
        32 << 20,
        64 << 20
    };

    if (nullptr == config)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (config->simulator != nullptr)
    {
        if (m_sim_cfg.simulator == nullptr)
            m_sim_cfg.simulator = new char[BUF_LEN];

        strncpy((char*)m_sim_cfg.simulator, config->simulator, BUF_LEN);
    }

    if (config->plugin_name != nullptr)
    {
        if (m_sim_cfg.plugin_name == nullptr)
            m_sim_cfg.plugin_name = new char[BUF_LEN];

        strncpy((char*)m_sim_cfg.plugin_name, config->plugin_name, BUF_LEN);
    }

    if (config->json_filename != nullptr)
    {
        if (m_sim_cfg.json_filename == nullptr)
            m_sim_cfg.json_filename = new char[BUF_LEN];

        strncpy((char*)m_sim_cfg.json_filename, config->json_filename, BUF_LEN);
    }

    if (config->log_file_path != nullptr)
    {
        strncpy((char*)m_sim_cfg.log_file_path, config->log_file_path, BUF_LEN);
    }

    m_sim_cfg.log_level = config->log_level;
    m_sim_cfg.verbose = config->verbose;
    m_sim_cfg.enable_avx = config->enable_avx;
    m_sim_cfg.enable_calloc = config->enable_calloc;
    m_sim_cfg.en_eval = config->en_eval;

    m_sim_cfg.gm_size = config->gm_size;
    if (gm_sz_cfg.count(m_sim_cfg.gm_size) != 1)
        m_sim_cfg.gm_size = 4 * MB_SIZE;

    if ((config->x2_arch_desc != nullptr) || (sim_npu_arch_env != nullptr))
    {
        if (m_sim_cfg.x2_arch_desc == nullptr)
            m_sim_cfg.x2_arch_desc = new char[64];

        if (config->x2_arch_desc != nullptr)
            strncpy((char*)m_sim_cfg.x2_arch_desc, config->x2_arch_desc, 64);
        else
            strncpy((char*)m_sim_cfg.x2_arch_desc, sim_npu_arch_env, 64);

        ret = set_target(AIPU_LOADABLE_GRAPH_ELF_V0, &m_dev, &m_sim_cfg);
        if (ret != AIPU_STATUS_SUCCESS)
            goto out;
    }
out:
    return ret;
}

aipu_status_t aipudrv::MainContext::config_hw(uint64_t types, aipu_global_config_hw_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (nullptr == config)
        return AIPU_STATUS_ERROR_NULL_PTR;

    m_hw_cfg = *config;
    return ret;
}

aipu_status_t aipudrv::MainContext::debugger_malloc(uint32_t size, void** va)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    BufferDesc *buf = nullptr;
    char* alloc_va = nullptr;
    uint32_t core_cnt = 0;

    if ((nullptr == va) || (nullptr == m_dram))
        return AIPU_STATUS_ERROR_NULL_PTR;

    ret = m_dram->malloc(size, 1, &buf, "dbg");
    if (ret != AIPU_STATUS_SUCCESS)
        return ret;

    if (m_dram->pa_to_va(buf->pa, buf->size, &alloc_va) != 0)
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
    if (m_dev->get_npu_version() == AIPU_ISA_VERSION_ZHOUYI_V3)
    {
        m_dev->write_reg(0, 0x0c, buf->pa);
        m_dev->write_reg(0, 0x08, 0x1248FFA5);
    } else {
        for (uint32_t id = 0; id < core_cnt; id++)
        {
            m_dev->write_reg(id, 0x14, buf->pa);
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
    uint32_t isa = AIPU_ISA_VERSION_ZHOUYI_V1;
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
    strcpy(target, nul_ptr);
    return ret;
    #endif

    isa = m_dev->get_npu_version();
    config = m_dev->get_npu_config();

    switch (isa)
    {
        case AIPU_ISA_VERSION_ZHOUYI_V1:
            isa_version = "Z1_";
            break;
        case AIPU_ISA_VERSION_ZHOUYI_V2_0:
            isa_version = "Z2_";
            break;
        case AIPU_ISA_VERSION_ZHOUYI_V2_1:
            isa_version = "Z3_";
            break;
        case AIPU_ISA_VERSION_ZHOUYI_V2_2:
            isa_version = "X1_";
            break;
        case AIPU_ISA_VERSION_ZHOUYI_V3:
            isa_version = "X2_";
            if (m_dev->get_npu_core_cnt() == 3)
            {
                arch_cfg = "X2_1204MP3";
                strncpy(target, arch_cfg.c_str(), 10);
                return ret;
            } else {
                config = 1204;
            }
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

    if (m_dev->get_npu_version() == AIPU_ISA_VERSION_ZHOUYI_V3)
        reg_addr = 0x804;

    ret = convert_ll_status(m_dev->read_reg(0, reg_addr, &value));

    if (m_dev->get_npu_version() == AIPU_ISA_VERSION_ZHOUYI_V3)
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
    int timeout = 3600; // prompt per 1 hour

    while ((*status != AIPU_JOB_STATUS_DONE) && (*status != AIPU_JOB_STATUS_EXCEPTION))
    {
        ret = job->get_status_blocking(status, 1000);
        if (ret == AIPU_STATUS_ERROR_TIMEOUT)
        {
            if (--timeout == 0)
            {
                ret = AIPU_STATUS_ERROR_TIMEOUT;
                LOG(LOG_WARN, "job: %p polled over 1h\n", job);
            }
            continue;
        } else if (ret != AIPU_STATUS_SUCCESS) {
            get_status_msg(ret, &msg);
            LOG(LOG_ERR, "job status: %s\n", msg);
            goto out;
        }
    }

out:
    return ret;
}

aipu_status_t aipudrv::MainContext::run_batch(GraphBase &graph, uint32_t queue_id,
    aipu_create_job_cfg_t *config)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS, oldret = AIPU_STATUS_SUCCESS;
    JOB_ID job_id = 0;
    JobBase *job = nullptr;
    aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;
    typedef struct job_info {
        JOB_ID job_id;
        JobBase *job;
        batch_info_t *batch;
    } job_info_t;

    job_info_t job_info_item = {0};
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
        ret = graph.create_job(&job_id, &m_sim_cfg, &m_hw_cfg, config);
        if (ret == AIPU_STATUS_ERROR_BUF_ALLOC_FAIL)
        {
            break;
        } else if (ret != AIPU_STATUS_SUCCESS) {
            oldret = ret;
            goto poll_job_sts;
        }

        job = graph.get_job(job_id);
        #ifdef SIMULATION
        if (types & AIPU_CONFIG_TYPE_SIMULATION)
        {
            aipu_job_config_simulation_t sim_config = {0};

            sim_config.data_dir = graph.get_batch_dump_path(queue_id);
            ret = job->config_simulation(AIPU_CONFIG_TYPE_SIMULATION, &sim_config);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                oldret = ret;
                goto poll_job_sts;
            }
        }
        #endif

        if (types & (~AIPU_CONFIG_TYPE_SIMULATION)) {
            aipu_job_config_dump_t dump_config = {0};

            dump_config.dump_dir = graph.get_batch_dump_path(queue_id);
            ret = job->config_mem_dump(types, &dump_config);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                oldret = ret;
                goto poll_job_sts;
            }
        }

        for (uint32_t in_idx = 0; in_idx < batch.inputs.size(); in_idx++)
        {
            ret = job->load_tensor(in_idx, batch.inputs[in_idx]);
            if (ret != AIPU_STATUS_SUCCESS)
            {
                oldret = ret;
                goto poll_job_sts;
            }
        }

        ret = job->schedule();
        if (ret != AIPU_STATUS_SUCCESS)
        {
            oldret = ret;
            goto poll_job_sts;
        }

        job_info_item.job_id = job_id;
        job_info_item.job = job;
        job_info_item.batch = &batch;
        job_queue.push(job_info_item);
    }

poll_job_sts:
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

        if ((oldret == AIPU_STATUS_SUCCESS) && (batch_num < graph.get_batch_queue_size(queue_id)) )
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

    if (oldret != AIPU_STATUS_SUCCESS)
        return oldret;
    else
        return ret;
}

aipu_status_t aipudrv::MainContext::ioctl_cmd(uint32_t cmd, void *arg)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    GraphBase* p_gobj = nullptr;

    if (nullptr == arg)
        return AIPU_STATUS_ERROR_NULL_PTR;

    if (cmd >= AIPU_IOCTL_SET_PROFILE && cmd <= AIPU_IOCTL_FREE_SHARE_BUF)
    {
        if (cmd == AIPU_IOCTL_SET_PROFILE) {
            m_dev->enable_profiling((*(int *)arg) != 0);
        } else if (cmd == AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION) {
            aipu_bin_buildversion_t *buildver = (aipu_bin_buildversion_t *)arg;

            if (!aipudrv::valid_graph_id(buildver->graph_id))
                return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

            p_gobj = get_graph_object(buildver->graph_id);
            if (nullptr == p_gobj)
                return AIPU_STATUS_ERROR_INVALID_GRAPH_ID;

            buildver->aipubin_buildversion = p_gobj->get_buildversion();
        } else if (cmd == AIPU_IOCTL_ALLOC_SHARE_BUF) {
            aipu_share_buf_t *share_buf = (aipu_share_buf_t *)arg;
            BufferDesc *buf = nullptr;// new BufferDesc;

            ret = m_dram->malloc(share_buf->size, 1, &buf, "share", share_buf->mem_type);
            if (ret != AIPU_STATUS_SUCCESS)
                return ret;

            if (m_dram->pa_to_va(buf->pa, buf->size, (char **)&share_buf->va) != 0)
                return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
            share_buf->pa = buf->pa;
        } else if (cmd == AIPU_IOCTL_FREE_SHARE_BUF) {
            aipu_share_buf_t *share_buf = (aipu_share_buf_t *)arg;
            Buffer buffer;

            if(m_dram->get_shared_buffer(share_buf->pa, share_buf->size, buffer) != 0)
                return AIPU_STATUS_ERROR_SET_SHARED_TENSOR;

            ret = m_dram->free(&buffer.desc, "share");
            if (ret != AIPU_STATUS_SUCCESS)
                return ret;
        }
    } else {
        #ifndef SIMULATION
        ret = convert_ll_status(m_dev->ioctl_cmd(cmd, arg));
        #endif

        if (ret == AIPU_STATUS_SUCCESS)
        {
            switch (cmd)
            {
                case AIPU_IOCTL_GET_VERSION:
                    aipu_driver_version_t *drv_ver = (aipu_driver_version_t *)arg;
                    strncpy(drv_ver->umd_version, m_umd_version.c_str(),
                        m_umd_version.length());
                    break;
            }
        }
    }

   return ret;
}