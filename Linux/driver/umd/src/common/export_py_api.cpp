// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  export_py_api.cpp
 * @brief AIPU User Mode Driver (UMD) C++ to Python API implementation
 * @version 1.0
 */

#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <vector>
#include <tuple>
#include "standard_api.h"
#include "kmd/armchina_aipu.h"
#include "pybind11/pybind11.h"
#include <pybind11/functional.h>
#include "pybind11/stl.h"
#include "pybind11/numpy.h"

#define PATH_LEN 1024

namespace py = pybind11;

typedef enum {
    AIPU_IOCTL_EN_TICK_COUNTER = 0,
    AIPU_IOCTL_DI_TICK_COUNTER
} aipu_ioctl_tickcounter_t;

typedef enum {
    D_INT8   = 0x08,
    D_UINT8  = 0x108,
    D_INT16  = 0x10,
    D_UINT16 = 0x110,
    D_INT32  = 0x20,
    D_UINT32 = 0x120,
} data_type_t;

class NPU
{
    public:
    /**
     * @brief This API is used to initialize AIPU UMD context
     *
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_OPEN_DEV_FAIL
     * @retval AIPU_STATUS_ERROR_DEV_ABNORMAL
     *
     * @note Before invoking any other UMD API calls, any UMD application must initialize a context first.
     */
    aipu_status_t aipu_init_context_py()
    {
        return aipu_init_context(&m_ctx);
    }

    /**
     * @brief This API is used to destroy AIPU UMD context
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_DEINIT_FAIL
     */
    aipu_status_t aipu_deinit_context_py()
    {
        return aipu_deinit_context(m_ctx);
    }

    /**
     * @brief This API is used to track what happens when an error code is returned by a UMD API.
     *
     * @param[in]  status Status returned by UMD standard API
     *
     * @retval error mesage
     */
    std::string aipu_get_error_message_py(uint64_t status)
    {
        const char *status_msg = nullptr;

        aipu_get_error_message(m_ctx, (aipu_status_t)status, (const char**)&status_msg);
        return std::string(status_msg);
    }

    /**
     * @par types: AIPU_CONFIG_TYPE_HW/AIPU_CONFIG_TYPE_SIMULATION
     * @par global_cfg_simulation: (python dict type)
     *      {
     *          "simulator" : "simulator path",
     *          "log_file_path" : "simulator log path",
     *          "x2_arch_desc" : [X2_1204 | X2_1204MP3],
     *          "plugin_name" : "simulator plugin file name",
     *          "json_filename" : "simulator json file name",
     *          "log_level" : "simulator log level, 0|1|2|3",
     *          "gm_size" : "simulated gm size, 512KB, 1MB, 2MB, 4MB, 8MB,16MB, 32MB",
     *          "verbose" : "simulator log verbose, 0|1",
     *          "enable_avx" : "0|1",
     *          "enable_calloc" : "0|1",
     *          "en_eval" : "0|1",
     *      }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_CONFIG
     *
     * @note accepted types/config: AIPU_CONFIG_TYPE_SIMULATION/aipu_global_config_simulation_t
     * @note accepted types/config: AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK/none
     * @note accepted types/config: AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK/none
     * @note accepted types/config: AIPU_CONFIG_TYPE_HW
     *
     */
    aipu_status_t aipu_config_global_py(uint64_t types,
        std::map<std::string, std::string> global_cfg_simulation)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::string str_key[] = {"simulator", "log_file_path", "x2_arch_desc", "plugin_name", "json_filename"};
        std::string int_key[] = {"log_level", "gm_size", "verbose", "enable_avx", "enable_calloc", "en_eval"};

        if (global_cfg_simulation.count(str_key[0]) == 1)
        {
            m_global_config_simulation.simulator = new char[PATH_LEN];
            memset((void *)m_global_config_simulation.simulator, 0, PATH_LEN);
            strncpy((char*)m_global_config_simulation.simulator, (char*)global_cfg_simulation[str_key[0]].c_str(),
                strlen(global_cfg_simulation[str_key[0]].c_str()));
        }

        if (global_cfg_simulation.count(str_key[1]) == 1)
        {
            m_global_config_simulation.log_file_path = new char[PATH_LEN];
            memset((void *)m_global_config_simulation.log_file_path, 0, PATH_LEN);
            strncpy((char*)m_global_config_simulation.log_file_path, (char*)global_cfg_simulation[str_key[1]].c_str(),
                strlen(global_cfg_simulation[str_key[1]].c_str()));
        }

        if (global_cfg_simulation.count(str_key[2]) == 1)
        {
            m_global_config_simulation.x2_arch_desc = new char[PATH_LEN];
            memset((void *)m_global_config_simulation.x2_arch_desc, 0, PATH_LEN);
            strncpy((char*)m_global_config_simulation.x2_arch_desc, (char*)global_cfg_simulation[str_key[2]].c_str(),
                strlen(global_cfg_simulation[str_key[2]].c_str()));
        }

        if (global_cfg_simulation.count(str_key[3]) == 1)
        {
            m_global_config_simulation.plugin_name = new char[PATH_LEN];
            memset((void *)m_global_config_simulation.plugin_name, 0, PATH_LEN);
            strncpy((char*)m_global_config_simulation.plugin_name, (char*)global_cfg_simulation[str_key[3]].c_str(),
                strlen(global_cfg_simulation[str_key[3]].c_str()));
        }

        if (global_cfg_simulation.count(str_key[4]) == 1)
        {
            m_global_config_simulation.json_filename = new char[PATH_LEN];
            memset((void *)m_global_config_simulation.json_filename, 0, PATH_LEN);
            strncpy((char*)m_global_config_simulation.json_filename, (char*)global_cfg_simulation[str_key[4]].c_str(),
                strlen(global_cfg_simulation[str_key[4]].c_str()));
        }

        if (global_cfg_simulation.count(int_key[0]) == 1)
            m_global_config_simulation.log_level = atoi(global_cfg_simulation[int_key[0]].c_str());

        if (global_cfg_simulation.count(int_key[1]) == 1)
            m_global_config_simulation.gm_size = atoi(global_cfg_simulation[int_key[1]].c_str());

        if (global_cfg_simulation.count(int_key[2]) == 1)
            m_global_config_simulation.verbose = atoi(global_cfg_simulation[int_key[2]].c_str());

        if (global_cfg_simulation.count(int_key[3]) == 1)
            m_global_config_simulation.enable_avx = atoi(global_cfg_simulation[int_key[3]].c_str());

        if (global_cfg_simulation.count(int_key[4]) == 1)
            m_global_config_simulation.enable_calloc = atoi(global_cfg_simulation[int_key[4]].c_str());

        if (global_cfg_simulation.count(int_key[5]) == 1)
            m_global_config_simulation.en_eval = atoi(global_cfg_simulation[int_key[5]].c_str());

        if (types & AIPU_CONFIG_TYPE_HW)
            ret = aipu_config_global(m_ctx, AIPU_CONFIG_TYPE_HW, &m_global_config_hw);
        else if (types & AIPU_CONFIG_TYPE_SIMULATION) {
            ret = aipu_config_global(m_ctx, AIPU_CONFIG_TYPE_SIMULATION, &m_global_config_simulation);
        } else {
            ret = aipu_config_global(m_ctx, types, nullptr);
        }

        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_config_global: %s\n", status_msg);
        }

        return ret;
    }

    /**
     * @brief This API loads an offline built AIPU executable graph binary from file system.
     *
     * @param[in]  graph_bin Executable graph binary file path
     *
     * return value map
     * {
     *     "ret": retval
     *     "data": graph_id
     * }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_OPEN_FILE_FAIL
     * @retval AIPU_STATUS_ERROR_MAP_FILE_FAIL
     * @retval AIPU_STATUS_ERROR_UNKNOWN_BIN
     * @retval AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED
     * @retval AIPU_STATUS_ERROR_INVALID_GBIN
     * @retval AIPU_STATUS_ERROR_TARGET_NOT_FOUND
     * @retval AIPU_STATUS_ERROR_BUF_ALLOC_FAIL
     * @retval AIPU_STATUS_ERROR_RESERVE_SRAM_FAIL
     * @retval AIPU_STATUS_ERROR_INVALID_GM
     */
    std::map<std::string, uint64_t> aipu_load_graph_py(std::string &graph_bin)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::map<std::string, uint64_t> retmap;
        uint64_t graph_id = -1;

        ret = aipu_load_graph(m_ctx, graph_bin.c_str(), &graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_graph: %s\n", status_msg);
            graph_id = -1;
        }

        retmap["ret"] = ret;
        retmap["data"] = graph_id;
        return retmap;
    }

    /**
     * @brief This API loads a graph with the form of AIPU executable graph binary array.
     *
     * @param[in]  graph_buf The start address of buffer which stores graph binary data
     * @param[in]  graph_size The byte size of graph binary data in 'graph_buf'
     *
     * return value map
     * {
     *     "ret": retval
     *     "data": graph_id
     * }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_SIZE
     * @retval AIPU_STATUS_ERROR_OPEN_FILE_FAIL
     * @retval AIPU_STATUS_ERROR_MAP_FILE_FAIL
     * @retval AIPU_STATUS_ERROR_UNKNOWN_BIN
     * @retval AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED
     * @retval AIPU_STATUS_ERROR_INVALID_GBIN
     * @retval AIPU_STATUS_ERROR_TARGET_NOT_FOUND
     * @retval AIPU_STATUS_ERROR_BUF_ALLOC_FAIL
     * @retval AIPU_STATUS_ERROR_RESERVE_SRAM_FAIL
     * @retval AIPU_STATUS_ERROR_INVALID_GM
     */
    std::map<std::string, uint64_t> aipu_load_graph_helper_py(const char *graph_buffer, uint32_t graph_size)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::map<std::string, uint64_t> retmap;
        uint64_t graph_id = -1;

        ret = aipu_load_graph_helper(m_ctx, graph_buffer, graph_size, &graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_graph: %s\n", status_msg);
            graph_id = -1;
        }

        retmap["ret"] = ret;
        retmap["data"] = graph_id;
        return retmap;
    }

    /**
     * @brief This API is used to unload a loaded graph
     *
     * @param[in] graph_id Graph ID returned by aipu_load_graph
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_GRAPH_ID
     */
    aipu_status_t aipu_unload_graph_py(uint64_t graph_id)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;

        ret = aipu_unload_graph(m_ctx, graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_unload_graph: %s\n", status_msg);
        }
        return ret;
    }

    /**
     * @brief This API is used to create a new job for a graph with provided buffer handle.
     *
     * @param[in] graph_id: loaded graph id
     * @param[in] job_cfg: job's config item
     *      {
     *          "partition_id" : "0-3",
     *          "dbg_dispatch" : "0|1",
     *          "dbg_core_id" : "0-2",
                "qos_level" : "0|1",
                "fm_mem_region" : "feature map priority allocation region",
                "wt_mem_region" : "weight priority allocation region",
     *      }
     * @param[in] fm_idxes: specify which feature maps are allocated from specific region
     * @param[in] wt_idxes: specify which weight maps are allocated from specific region
     *
     * return value map
     * {
     *     "ret": retval
     *     "data": job_id
     * }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_GRAPH_ID
     * @retval AIPU_STATUS_ERROR_BUF_ALLOC_FAIL
     */
    std::map<std::string, uint64_t> aipu_create_job_py(uint64_t graph_id, std::map<std::string, int> job_cfg,
        std::vector<int> fm_idxes, std::vector<int> wt_idxes)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::map<std::string, uint64_t> retmap;
        uint64_t job_id = -1;
        aipu_create_job_cfg_t create_job_config = {0};
        std::string keys[] = {"partition_id", "dbg_dispatch", "dbg_core_id",
            "qos_level", "fm_mem_region", "wt_mem_region"};

        create_job_config.misc = 0;
        if (job_cfg.count(keys[0]))
            create_job_config.partition_id = job_cfg[keys[0]];

        if (job_cfg.count(keys[1]))
            create_job_config.dbg_dispatch = job_cfg[keys[1]];

        if (job_cfg.count(keys[2]))
            create_job_config.dbg_core_id = job_cfg[keys[2]];

        if (job_cfg.count(keys[3]))
            create_job_config.qos_level = job_cfg[keys[3]];

        if (job_cfg.count(keys[4]))
            create_job_config.fm_mem_region = job_cfg[keys[4]];

        if (job_cfg.count(keys[5]))
            create_job_config.wt_mem_region = job_cfg[keys[5]];

        if (fm_idxes.size() > 0)
        {
            create_job_config.fm_idxes_cnt = fm_idxes.size();
            create_job_config.fm_idxes = new int[create_job_config.fm_idxes_cnt];

            for (int i = 0; i < create_job_config.fm_idxes_cnt; i++)
                create_job_config.fm_idxes[i] = fm_idxes[i];
        }

        if (wt_idxes.size() > 0)
        {
            create_job_config.wt_idxes_cnt = wt_idxes.size();
            create_job_config.wt_idxes = new int[create_job_config.wt_idxes_cnt];

            for (int i = 0; i < create_job_config.wt_idxes_cnt; i++)
                create_job_config.wt_idxes[i] = wt_idxes[i];
        }

        ret = aipu_create_job(m_ctx, graph_id, &job_id, &create_job_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_finish_job: %s\n", status_msg);
            job_id = -1;
        }

        /* free job config items */
        if (create_job_config.fm_idxes)
            delete []create_job_config.fm_idxes;

        if (create_job_config.wt_idxes)
            delete []create_job_config.wt_idxes;

        retmap["ret"] = ret;
        retmap["data"] = job_id;
        return retmap;
    }

    /**
     * @par job_id: created job id
     * @par types:  AIPU_CONFIG_TYPE_SIMULATION/AIPU_JOB_CONFIG_TYPE_DUMP_xxx
     * @par job_config_dump: job dump config
     *      {
     *          "dump_dir" : "dump_dir is used as file dump-path, dump required",
     *          "prefix" : "name prefix of dump files, optional",
     *          "output_prefix" : "name prefix of output dump files, optional",
     *          "misc_prefix" : "name prefix of profile/printf data files, optional",
     *          "data_dir" : "data_dir is used as aipu v1/v2 simulation data file directory, dump required"
     *      }
     *
     */
    aipu_status_t aipu_config_job_py(uint64_t job_id, uint64_t types,
        std::map<std::string, std::string> job_config_dump)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        aipu_job_config_dump_t aipu_job_config_dump = {0};
        aipu_job_config_simulation_t aipu_job_config_simulation = {0};
        std::string keys[] = {"dump_dir", "prefix", "output_prefix", "misc_prefix", "data_dir"};

        if (job_config_dump.count(keys[0]) == 1)
        {
            aipu_job_config_dump.dump_dir = new char[PATH_LEN];
            memset((void *)aipu_job_config_dump.dump_dir, 0, PATH_LEN);
            strncpy((char *)aipu_job_config_dump.dump_dir, (char *)job_config_dump[keys[0]].c_str(),
                strlen(job_config_dump[keys[0]].c_str()));
        }

        if (job_config_dump.count(keys[1]) == 1)
        {
            aipu_job_config_dump.prefix = new char[PATH_LEN];
            memset((void *)aipu_job_config_dump.prefix, 0, PATH_LEN);
            strncpy((char *)aipu_job_config_dump.prefix, (char *)job_config_dump[keys[1]].c_str(),
                strlen(job_config_dump[keys[1]].c_str()));
        }

        if (job_config_dump.count(keys[2]) == 1)
        {
            aipu_job_config_dump.output_prefix = new char[PATH_LEN];
            memset((void *)aipu_job_config_dump.output_prefix, 0, PATH_LEN);
            strncpy((char *)aipu_job_config_dump.output_prefix, (char *)job_config_dump[keys[2]].c_str(),
                strlen(job_config_dump[keys[2]].c_str()));
        }

        if (job_config_dump.count(keys[3]) == 1)
        {
            aipu_job_config_dump.misc_prefix = new char[PATH_LEN];
            memset((void *)aipu_job_config_dump.misc_prefix, 0, PATH_LEN);
            strncpy((char *)aipu_job_config_dump.misc_prefix, (char *)job_config_dump[keys[3]].c_str(),
                strlen(job_config_dump[keys[3]].c_str()));
        }

        if (job_config_dump.count(keys[4]) == 1)
        {
            aipu_job_config_simulation.data_dir = new char[PATH_LEN];
            memset((void *)aipu_job_config_simulation.data_dir, 0, PATH_LEN);
            strncpy((char *)aipu_job_config_simulation.data_dir, (char *)job_config_dump[keys[4]].c_str(),
                strlen(job_config_dump[keys[4]].c_str()));
        }

        if (types == AIPU_CONFIG_TYPE_SIMULATION)
            ret = aipu_config_job(m_ctx, job_id, types, &aipu_job_config_simulation);
        else
            ret = aipu_config_job(m_ctx, job_id, types, &aipu_job_config_dump);

        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_config_job: %s\n", status_msg);
        }

        /* free job config dump items */
        if (aipu_job_config_dump.dump_dir)
            delete []aipu_job_config_dump.dump_dir;

        if (aipu_job_config_dump.prefix)
            delete []aipu_job_config_dump.prefix;

        if (aipu_job_config_dump.output_prefix)
            delete []aipu_job_config_dump.output_prefix;

        if (aipu_job_config_dump.misc_prefix)
            delete []aipu_job_config_dump.misc_prefix;

        /* free job_config_simulation data_dir */
        if (aipu_job_config_simulation.data_dir)
            delete []aipu_job_config_simulation.data_dir;

        return ret;
    }

    /**
     * @brief This API is used to flush a new computation job onto AIPU (blocking)
     *
     * @param[in] job_id      Job ID returned by aipu_create_job
     * @param[in] time_out Time out (in millisecond) specified by application for this job
     *                     (A timeout of value <= 0 means an infinite timeout.)
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     * @retval AIPU_STATUS_ERROR_JOB_EXCEPTION
     * @retval AIPU_STATUS_ERROR_JOB_TIMEOUT
     */
    aipu_status_t aipu_finish_job_py(uint64_t job_id, int32_t time_out)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;

        ret = aipu_finish_job(m_ctx, job_id, time_out);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_finish_job: %s\n", status_msg);
        }
        return ret;
    }

    /**
     * @brief This API is used to flush a new computation job onto AIPU (non-blocking)
     *
     * @param[in] job_id      Job ID returned by aipu_create_job
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     *
     */
    aipu_status_t aipu_flush_job_py(uint64_t job_id, aipu_job_callback_func_t py_cb)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        ret = aipu_flush_job(m_ctx, job_id, py_cb);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_flush_job: %s\n", status_msg);
        }
        return ret;
    }

    /**
     * @brief This API is used to get the execution status of a flushed job (non-blocking)
     *
     * @param[in]  job_id    Job ID returned by aipu_create_job
     * @param[in]  timeout timeout value(ms) to poll job's status
     *                     timeout > 0: the max polling time window is 'timeout'
     *                     timeout = 0: non-blocking and return job's status immediately.
     *                     timeout = -1: blocking until job is really done or exception.
     *
     * return value map
     * {
     *     "ret": retval
     *     "data": job status
     * }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_TIMEOUT
     *
     * @note This API should be used by the application after aipu_flush_job successfully returns.
     */
    std::map<std::string, int> aipu_get_job_status_py(uint64_t job_id, int32_t timeout)
    {
        aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        std::map<std::string, int> retmap;
        const char *status_msg = nullptr;

        ret = aipu_get_job_status(m_ctx, job_id, &status, timeout);

        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_job_status: %s\n", status_msg);
            status = AIPU_JOB_STATUS_NO_STATUS;
        }

        retmap["ret"] = ret;
        retmap["data"] = (int)status;
        return retmap;
    }

    /**
     * @brief This API is used to clean a finished job object
     *        scheduled by aipu_finish_job/aipu_flush_job
     *
     * @param[in] job_id Job ID returned by aipu_create_job
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     */
    aipu_status_t aipu_clean_job_py(uint64_t job_id)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;

        ret = aipu_clean_job(m_ctx, job_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_clean_job: %s\n", status_msg);
        }
        return ret;
    }

    /**
     * @brief This API is used to get tensor count of specified type
     *
     * @param[in]  id   Job ID returned by aipu_create_job, or graph ID returned by aipu_load_graph
     * @param[in]  type Tensor type
     *
     * return value map:
     * {
     *     "ret" : retval
     *     "data" : tensor count
     * }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_GRAPH_ID
     */
    std::map<std::string, int> aipu_get_tensor_count_py(uint64_t graph_id, aipu_tensor_type_t type)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::map<std::string, int> retmap;
        uint32_t num = 0;

        ret = aipu_get_tensor_count(m_ctx, graph_id, type, &num);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_count: %s\n", status_msg);
            num = 0;
        }

        retmap["ret"] = ret;
        retmap["data"] = num;
        return retmap;
    }

    /**
     * @brief This API is used to get tensor's descriptor of specified type
     *
     * @param[in]  id     Job ID returned by aipu_create_job, or graph ID returned by aipu_load_graph
     * @param[in]  type   Tensor type
     * @param[in]  tensor Tensor ID
     *
     * @retval normal tensor descriptor, otherwise a full zero descriptor returned.
     */
    aipu_tensor_desc_t aipu_get_tensor_descriptor_py(uint64_t graph_id, aipu_tensor_type_t type,
        uint32_t tensor)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        aipu_tensor_desc_t desc{0};

        ret = aipu_get_tensor_descriptor(m_ctx, graph_id, type, tensor, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", status_msg);
            return {0};
        }

        return desc;
    }

    /**
     * @brief This API is used to load input tensor data from input bin
     *
     * @param[in] job    Job ID returned by aipu_create_job
     * @param[in] tensor Input tensor ID
     * @param[in] filename   Input bin's full path
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_TENSOR_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     */
    aipu_status_t aipu_load_tensor_file_py(uint64_t job_id, uint32_t tensor, std::string &filename)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        void* data = nullptr;
        aipu_tensor_desc_t desc;
        int fd = 0;

        ret = aipu_get_tensor_descriptor(m_ctx, job_id, AIPU_TENSOR_TYPE_INPUT, tensor, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", status_msg);
            return ret;
        }

        fd = open(filename.c_str(), O_RDONLY);
        if (fd <= 0)
        {
            fprintf(stderr, "[PY UMD ERROR] open file failed: %s! \n", filename.c_str());
            return AIPU_STATUS_ERROR_OPEN_FILE_FAIL;
        }

        data = mmap(nullptr, desc.size, PROT_READ, MAP_PRIVATE, fd, 0);
        if (MAP_FAILED == data)
        {
            fprintf(stderr, "[PY UMD ERROR] RT failed in mapping graph file: %s! (errno = %d)\n", filename.c_str(), errno);
            close(fd);
            return AIPU_STATUS_ERROR_MAP_FILE_FAIL;
        }

        ret = aipu_load_tensor(m_ctx, job_id, tensor, data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", status_msg);
        }

        munmap(data, desc.size);

        if (fd > 0)
            close(fd);

        return ret;
    }

    /**
     * @brief This API is used to load input tensor data from bytes
     *
     * @param[in] job    Job ID returned by aipu_create_job
     * @param[in] tensor Input tensor ID
     * @param[in] data   Raw python bytes
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_TENSOR_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     */
    aipu_status_t aipu_load_tensor_rawbytes_py(uint64_t job_id, uint32_t tensor, char *data)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;

        ret = aipu_load_tensor(m_ctx, job_id, tensor, data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", status_msg);
        }

        return ret;
    }

    /**
     * @brief This API is used to load input tensor data from rawbytes
     *
     * @param[in] job    Job ID returned by aipu_create_job
     * @param[in] tensor Input tensor ID
     * @param[in] raw_bytes   Raw python bytes
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_TENSOR_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     */
    aipu_status_t aipu_load_tensor_pybytes_py(uint64_t job_id, uint32_t tensor, py::bytes raw_bytes)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        char *data = PyBytes_AsString(raw_bytes.ptr());

        ret = aipu_load_tensor(m_ctx, job_id, tensor, data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", status_msg);
        }

        return ret;
    }

    /**
     * @brief This API is used to load input tensor data from numpy array
     *
     * @param[in] job    Job ID returned by aipu_create_job
     * @param[in] tensor Input tensor ID
     * @param[in] numpy_array   Numpy array
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_TENSOR_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     */
    aipu_status_t aipu_load_tensor_numpyarray_py(uint64_t job_id, uint32_t tensor,
        py::array_t<int> numpy_array, data_type_t data_size)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        aipu_tensor_desc_t desc;
        const char *status_msg = nullptr;
        const int *data = numpy_array.data();
        uint8_t * in_data = nullptr;

        ret = aipu_get_tensor_descriptor(m_ctx, job_id, AIPU_TENSOR_TYPE_INPUT, tensor, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", status_msg);
            goto finish;
        }

        if (data_size == D_INT32 || data_size == D_UINT32)
        {
            in_data = (uint8_t *)data;
        }
        else if (data_size == D_INT16  ||
                 data_size == D_INT8   ||
                 data_size == D_UINT16 ||
                 data_size == D_UINT8)
        {
            int for_loop = desc.size / ((data_size & 0xFF) / D_INT8);
            in_data = new uint8_t[desc.size];
            for (int i = 0; i < for_loop; i++)
            {
                if (data_size == D_INT8 || data_size == D_UINT8)
                    *(in_data + i) = *((uint8_t *)data + 4*i);

                if (data_size == D_INT16 || data_size == D_UINT16)
                    *((uint16_t *)in_data + i) = *((uint16_t *)data + 2*i);
            }
        }
        else
        {
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: data type error.\n");
            goto finish;
        }

        ret = aipu_load_tensor(m_ctx, job_id, tensor, in_data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", status_msg);
        }

        if (in_data && ((data_size & 0xFF) != D_INT32))
            delete[] in_data;
    finish:
        return ret;
    }

    /**
     * @brief This API is used to get tensor data of specified type
     *
     * @param[in]  job    Job ID returned by aipu_create_job
     * @param[in]  type   Tensor type
     * @param[in]  tensor Input tensor ID
     *
     * return value map
     * {
     *     "ret": {retval}
     *     "data": {out data}
     * }
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
     * @retval AIPU_STATUS_ERROR_INVALID_TENSOR_ID
     * @retval AIPU_STATUS_ERROR_INVALID_OP
     */
    std::map<std::string, std::vector<int64_t> > aipu_get_tensor_py(uint64_t job_id,
        aipu_tensor_type_t type, uint32_t tensor, data_type_t data_size)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::map<std::string, std::vector<int64_t> > retmap;
        unsigned char *out_data = nullptr;
        aipu_tensor_desc_t desc;

        ret = aipu_get_tensor_descriptor(m_ctx, job_id, type, tensor, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", status_msg);
            retmap["ret"] = {ret};
            retmap["data"] = {};
            goto finish;
        }

        out_data = new unsigned char[desc.size];
        ret = aipu_get_tensor(m_ctx, job_id, type, tensor, out_data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor: %s\n", status_msg);
            retmap["ret"] = {ret};
            retmap["data"] = {};
            goto finish;
        }

        retmap["ret"] = {ret};

        if (data_size == D_UINT8)
        {
            for (uint32_t i = 0; i < desc.size; i++)
                retmap["data"].push_back((int64_t)out_data[i]);
        }
        else if (data_size == D_INT8)
        {
            int8_t *p = (int8_t *)out_data;
            for (uint32_t i = 0; i < desc.size; i++)
                retmap["data"].push_back((int64_t)p[i]);
        }
        else if (data_size == D_INT16)
        {
            int16_t *p = (int16_t *)out_data;
            for (uint32_t i = 0; i < (desc.size/2); i++)
                retmap["data"].push_back((int64_t)p[i]);
        }
        else if (data_size == D_UINT16)
        {
            uint16_t *p = (uint16_t *)out_data;
            for (uint32_t i = 0; i < (desc.size/2); i++)
                retmap["data"].push_back((int64_t)p[i]);
        }
        else if (data_size == D_INT32)
        {
            int32_t *p = (int32_t *)out_data;
            for (uint32_t i = 0; i < (desc.size/4); i++)
                retmap["data"].push_back((int64_t)p[i]);
        }
        else if (data_size == D_UINT32)
        {
            uint32_t *p = (uint32_t *)out_data;
            for (uint32_t i = 0; i < (desc.size/4); i++)
                retmap["data"].push_back((int64_t)p[i]);
        }
        else
        {
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor: invalid data type.\n");
            retmap["data"] = {};
            retmap["ret"] = {-66666};
        }

    finish:
        if (out_data)
            delete[] out_data;
        return retmap;
    }

    std::map<std::string, std::vector<uint64_t> > aipu_ioctl_py(uint32_t cmd,
        std::map<std::string, uint64_t> py_arg)
    {
        void *arg = nullptr;
        std::map<std::string, std::vector<uint64_t> > retmap;
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;

        switch (cmd)
        {
            case AIPU_IOCTL_EN_TICK_COUNTER:
                cmd = AIPU_IOCTL_ENABLE_TICK_COUNTER;
                break;

            case AIPU_IOCTL_DI_TICK_COUNTER:
                cmd = AIPU_IOCTL_DISABLE_TICK_COUNTER;
                break;

            case AIPU_IOCTL_FREE_SHARE_BUF:
                {
                std::string str_key[] = {"pa", "va", "size"};
                arg = malloc(sizeof(aipu_share_buf_t));
                aipu_share_buf_t *p = (aipu_share_buf_t *)arg;
                memset(p, 0, sizeof(aipu_share_buf_t));
                if (py_arg.count(str_key[0]) == 1)
                    p->pa = (uint64_t)py_arg[str_key[0]];

                if (py_arg.count(str_key[1]) == 1)
                    p->va = (uint64_t)py_arg[str_key[1]];

                if (py_arg.count(str_key[2]) == 1)
                    p->size = (uint32_t)py_arg[str_key[2]];

                }
                break;

            case AIPU_IOCTL_ALLOC_SHARE_BUF:
                {
                std::string str_key[] = {"size", "mem_type"};
                arg = malloc(sizeof(aipu_share_buf_t));
                aipu_share_buf_t *p = (aipu_share_buf_t *)arg;
                memset(p, 0, sizeof(aipu_share_buf_t));
                if (py_arg.count(str_key[0]) == 1)
                    p->size = (uint32_t)py_arg[str_key[0]];

                if (py_arg.count(str_key[1]) == 1)
                    p->mem_type = (uint32_t)py_arg[str_key[1]];
                }
                break;

            case AIPU_IOCTL_SET_PROFILE:
                {
                std::string str_key[] = {"set_profile"};
                arg = malloc(sizeof(int));
                int *p = (int *)arg;
                *p = 0;
                if (py_arg.count(str_key[0]) == 1)
                    *p = (int)py_arg[str_key[0]];
                }
                break;

            case AIPU_IOCTL_ALLOC_DMABUF:
                {
                std::string str_key[] = {"dmabuf_fd", "bytes"};
                arg = malloc(sizeof(struct aipu_dma_buf_request));
                struct aipu_dma_buf_request *p = (struct aipu_dma_buf_request *)arg;
                memset(p, 0, sizeof(struct aipu_dma_buf_request));
                if (py_arg.count(str_key[0]) == 1)
                    p->fd = (int)py_arg[str_key[0]];

                if (py_arg.count(str_key[1]) == 1)
                    p->bytes = (uint64_t)py_arg[str_key[1]];
                }
                break;

            case AIPU_IOCTL_FREE_DMABUF:
                {
                std::string str_key[] = {"dmabuf_fd"};
                arg = malloc(sizeof(int));
                int *p = (int *)arg;
                *p = 0;
                if (py_arg.count(str_key[0]) == 1)
                    *p = (int)py_arg[str_key[0]];
                }
                break;

            case AIPU_IOCTL_READ_DMABUF:
                {
                std::string str_key[] = {"dmabuf_fd", "offset_in_dmabuf", "size"};
                arg = malloc(sizeof(aipu_dmabuf_op_t));
                aipu_dmabuf_op_t *p = (aipu_dmabuf_op_t *)arg;
                memset(p, 0, sizeof(aipu_dmabuf_op_t));
                if (py_arg.count(str_key[0]) == 1)
                    p->dmabuf_fd = (int)py_arg[str_key[0]];

                if (py_arg.count(str_key[1]) == 1)
                    p->offset_in_dmabuf = (uint32_t)py_arg[str_key[1]];

                if (py_arg.count(str_key[2]) == 1)
                    p->size = (uint32_t)py_arg[str_key[2]];

                p->data = new char[p->size];

                }
                break;

            default:
                fprintf(stderr, "[PY UMD ERROR] aipu_ioctl invalid cmd\n");
                retmap["data"] = {};
                ret = AIPU_STATUS_ERROR_INVALID_OP;
                goto finish;

        }

        ret = aipu_ioctl(m_ctx, cmd, arg);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_ioctl: %s\n", status_msg);
            retmap["data"] = {};
            goto finish;
        }

        if (cmd == AIPU_IOCTL_READ_DMABUF)
        {
            aipu_dmabuf_op_t *p = (aipu_dmabuf_op_t *)arg;
            for (uint32_t i = 0; i < p->size; i++)
                retmap["data"].push_back((uint64_t)p->data[i]);

            delete []p->data;
        }
        else if (cmd == AIPU_IOCTL_ALLOC_DMABUF)
        {
            struct aipu_dma_buf_request *p = (struct aipu_dma_buf_request *)arg;
            retmap["data"].push_back((uint64_t)p->fd);
        }
        else if (cmd == AIPU_IOCTL_ALLOC_SHARE_BUF)
        {
            aipu_share_buf_t *p = (aipu_share_buf_t *)arg;
            retmap["data"].push_back(p->pa);
            retmap["data"].push_back(p->va);
        }
        else
        {
            retmap["data"] = {};
        }

    finish:
        retmap["ret"] = {ret};
        if (arg)
            free(arg);
        return retmap;
    }

    aipu_status_t aipu_ioctl_write_dmabuf_py(uint32_t cmd,
        std::map<std::string, uint64_t> py_arg, char *data)
    {
        aipu_dmabuf_op_t *p = nullptr;
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::string str_key[] = {"dmabuf_fd", "offset_in_dmabuf", "size"};

        if (cmd != AIPU_IOCTL_WRITE_DMABUF)
        {
            fprintf(stderr, "[PY UMD ERROR] aipu_ioctl with 3 args must use cmd: AIPU_IOCTL_WRITE_DMABUF\n");
            ret = AIPU_STATUS_ERROR_INVALID_OP;
            goto finish;
        }

        p = (aipu_dmabuf_op_t *)malloc(sizeof(aipu_dmabuf_op_t));
        memset(p, 0, sizeof(aipu_dmabuf_op_t));
        if (py_arg.count(str_key[0]) == 1)
            p->dmabuf_fd = (int)py_arg[str_key[0]];

        if (py_arg.count(str_key[1]) == 1)
            p->offset_in_dmabuf = (uint32_t)py_arg[str_key[1]];

        if (py_arg.count(str_key[2]) == 1)
            p->size = (uint32_t)py_arg[str_key[2]];

        p->data = new char[p->size];
        memcpy(p->data, data, p->size);

        ret = aipu_ioctl(m_ctx, cmd, p);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_ioctl: %s\n", status_msg);
            goto finish;
        }

    finish:
        if (p && p->data)
            delete []p->data;

        if (p)
            free(p);

        return ret;
    }

    aipu_status_t aipu_specify_iobuf_py(uint64_t job_id, std::map<std::string, uint64_t> py_arg)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::string str_key[] = {"type", "tensor_idx", "id", "pa", "dmabuf_fd", "offset_in_dmabuf", "shared_case_type"};
        aipu_shared_tensor_info_t share_tensor;
        memset(&share_tensor, 0, sizeof(aipu_shared_tensor_info_t));

        if (py_arg.count(str_key[0]) == 1)
            share_tensor.type = (aipu_tensor_type_t)py_arg[str_key[0]];

        if (py_arg.count(str_key[1]) == 1)
            share_tensor.tensor_idx = (uint32_t)py_arg[str_key[1]];

        if (py_arg.count(str_key[2]) == 1)
            share_tensor.id = (uint64_t)py_arg[str_key[2]];

        if (py_arg.count(str_key[3]) == 1)
            share_tensor.pa = (uint64_t)py_arg[str_key[3]];

        if (py_arg.count(str_key[4]) == 1)
            share_tensor.dmabuf_fd = (int)py_arg[str_key[4]];

        if (py_arg.count(str_key[5]) == 1)
            share_tensor.offset_in_dmabuf = (uint32_t)py_arg[str_key[5]];

        if (py_arg.count(str_key[6]) == 1)
            share_tensor.shared_case_type = (uint32_t)py_arg[str_key[6]];

        ret = aipu_specify_iobuf(m_ctx, job_id, &share_tensor);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_specify_iobuf: %s\n", status_msg);
        }

        return ret;
    }

    public:
    NPU() {};
    NPU(const NPU& aipu) = delete;
    NPU& operator=(const NPU& aipu) = delete;

    virtual ~NPU()
    {
        /* free global config items */
        if (m_global_config_simulation.simulator)
            delete []m_global_config_simulation.simulator;

        if (m_global_config_simulation.log_file_path)
            delete []m_global_config_simulation.log_file_path;

        if (m_global_config_simulation.x2_arch_desc)
            delete []m_global_config_simulation.x2_arch_desc;

        if (m_global_config_simulation.plugin_name)
            delete []m_global_config_simulation.plugin_name;

        if (m_global_config_simulation.json_filename)
            delete []m_global_config_simulation.json_filename;
    };

    private:
    aipu_ctx_handle_t* m_ctx = nullptr;
    aipu_global_config_hw_t m_global_config_hw = {0};
    aipu_global_config_simulation_t m_global_config_simulation = {0};
};


/**
 * Decare entry module: libaipudriv
 */
PYBIND11_MODULE(libaipudrv, m) {
    m.doc() = "CompassNPU UMD python APIs";

    py::class_<aipu_ctx_handle_t>(m, "aipu_ctx_handle_t")
        .def(py::init<>())
        .def_readwrite("handle", &aipu_ctx_handle_t::handle);

    py::enum_<device_status_t>(m, "device_status_t")
        .value("DEV_IDLE", device_status_t::DEV_IDLE)
        .value("DEV_BUSY", device_status_t::DEV_BUSY)
        .value("DEV_EXCEPTION", device_status_t::DEV_EXCEPTION)
        .export_values();

    py::enum_<aipu_data_type_t>(m, "aipu_data_type_t")
        .value("AIPU_DATA_TYPE_NONE", aipu_data_type_t::AIPU_DATA_TYPE_NONE)
        .value("AIPU_DATA_TYPE_BOOL", aipu_data_type_t::AIPU_DATA_TYPE_BOOL)
        .value("AIPU_DATA_TYPE_U8", aipu_data_type_t::AIPU_DATA_TYPE_U8)
        .value("AIPU_DATA_TYPE_S8", aipu_data_type_t::AIPU_DATA_TYPE_S8)
        .value("AIPU_DATA_TYPE_U16", aipu_data_type_t::AIPU_DATA_TYPE_U16)
        .value("AIPU_DATA_TYPE_S16", aipu_data_type_t::AIPU_DATA_TYPE_S16)
        .value("AIPU_DATA_TYPE_U32", aipu_data_type_t::AIPU_DATA_TYPE_U32)
        .value("AIPU_DATA_TYPE_S32", aipu_data_type_t::AIPU_DATA_TYPE_S32)
        .value("AIPU_DATA_TYPE_U64", aipu_data_type_t::AIPU_DATA_TYPE_U64)
        .value("AIPU_DATA_TYPE_S64", aipu_data_type_t::AIPU_DATA_TYPE_S64)
        .value("AIPU_DATA_TYPE_f16", aipu_data_type_t::AIPU_DATA_TYPE_f16)
        .value("AIPU_DATA_TYPE_f32", aipu_data_type_t::AIPU_DATA_TYPE_f32)
        .value("AIPU_DATA_TYPE_f64", aipu_data_type_t::AIPU_DATA_TYPE_f64)
        .export_values();

    py::enum_<aipu_tensor_type_t>(m, "aipu_tensor_type_t")
        .value("AIPU_TENSOR_TYPE_INPUT", aipu_tensor_type_t::AIPU_TENSOR_TYPE_INPUT)
        .value("AIPU_TENSOR_TYPE_OUTPUT", aipu_tensor_type_t::AIPU_TENSOR_TYPE_OUTPUT)
        .value("AIPU_TENSOR_TYPE_INTER_DUMP", aipu_tensor_type_t::AIPU_TENSOR_TYPE_INTER_DUMP)
        .value("AIPU_TENSOR_TYPE_PRINTF", aipu_tensor_type_t::AIPU_TENSOR_TYPE_PRINTF)
        .value("AIPU_TENSOR_TYPE_PROFILER", aipu_tensor_type_t::AIPU_TENSOR_TYPE_PROFILER)
        .value("AIPU_TENSOR_TYPE_LAYER_COUNTER", aipu_tensor_type_t::AIPU_TENSOR_TYPE_LAYER_COUNTER)
        .value("AIPU_TENSOR_TYPE_ERROR_CODE", aipu_tensor_type_t::AIPU_TENSOR_TYPE_ERROR_CODE)
        .export_values();

    py::class_<aipu_tensor_desc_t>(m, "aipu_tensor_desc_t")
        .def(py::init<>())
        .def_readwrite("id", &aipu_tensor_desc_t::id)
        .def_readwrite("size", &aipu_tensor_desc_t::size)
        .def_readwrite("scale", &aipu_tensor_desc_t::scale)
        .def_readwrite("zero_point", &aipu_tensor_desc_t::zero_point)
        .def_readwrite("data_type", &aipu_tensor_desc_t::data_type);

    py::enum_<aipu_job_status_t>(m, "aipu_job_status_t")
        .value("AIPU_JOB_STATUS_NO_STATUS", aipu_job_status_t::AIPU_JOB_STATUS_NO_STATUS)
        .value("AIPU_JOB_STATUS_DONE", aipu_job_status_t::AIPU_JOB_STATUS_DONE)
        .value("AIPU_JOB_STATUS_EXCEPTION", aipu_job_status_t::AIPU_JOB_STATUS_EXCEPTION)
        .export_values();

    py::class_<aipu_debugger_job_info_t>(m, "aipu_debugger_job_info_t")
        .def(py::init<>())
        .def_readonly("instr_base", &aipu_debugger_job_info_t::instr_base)
        .def_readwrite("simulation_aipu", &aipu_debugger_job_info_t::simulation_aipu)
        .def_readwrite("simulation_mem_engine", &aipu_debugger_job_info_t::simulation_mem_engine);

    py::enum_<aipu_config_type_t>(m, "aipu_config_type_t")
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_TEXT", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_TEXT)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_RODATA", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_RODATA)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_INPUT", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_INPUT)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_REUSE", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_REUSE)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION)
        .value("AIPU_JOB_CONFIG_TYPE_DUMP_PROFILE", aipu_config_type_t::AIPU_JOB_CONFIG_TYPE_DUMP_PROFILE)
        .value("AIPU_CONFIG_TYPE_SIMULATION", aipu_config_type_t::AIPU_CONFIG_TYPE_SIMULATION)
        .value("AIPU_CONFIG_TYPE_HW", aipu_config_type_t::AIPU_CONFIG_TYPE_HW)
        .value("AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK", aipu_config_type_t::AIPU_GLOBAL_CONFIG_TYPE_DISABLE_VER_CHECK)
        .value("AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK", aipu_config_type_t::AIPU_GLOBAL_CONFIG_TYPE_ENABLE_VER_CHECK)
        .export_values();

    py::class_<aipu_job_config_dump_t>(m, "aipu_job_config_dump_t")
        .def(py::init<>())
        .def_readwrite("dump_dir", &aipu_job_config_dump_t::dump_dir)
        .def_readwrite("prefix", &aipu_job_config_dump_t::prefix)
        .def_readwrite("output_prefix", &aipu_job_config_dump_t::output_prefix)
        .def_readwrite("misc_prefix", &aipu_job_config_dump_t::misc_prefix);

    py::class_<aipu_job_config_simulation_t>(m, "aipu_job_config_simulation_t")
        .def(py::init<>())
        .def_readwrite("data_dir", &aipu_job_config_simulation_t::data_dir);

    py::class_<aipu_global_config_simulation_t>(m, "aipu_global_config_simulation_t")
        .def(py::init<>())
        .def_readwrite("simulator", &aipu_global_config_simulation_t::simulator)
        .def_readwrite("log_file_path", &aipu_global_config_simulation_t::log_file_path)
        .def_readwrite("x2_arch_desc", &aipu_global_config_simulation_t::x2_arch_desc)
        .def_readwrite("plugin_name", &aipu_global_config_simulation_t::plugin_name)
        .def_readwrite("json_filename", &aipu_global_config_simulation_t::json_filename)
        .def_readwrite("log_level", &aipu_global_config_simulation_t::log_level)
        .def_readwrite("gm_size", &aipu_global_config_simulation_t::gm_size)
        .def_readwrite("verbose", &aipu_global_config_simulation_t::verbose)
        .def_readwrite("enable_avx", &aipu_global_config_simulation_t::enable_avx)
        .def_readwrite("enable_calloc", &aipu_global_config_simulation_t::enable_calloc)
        .def_readwrite("en_eval", &aipu_global_config_simulation_t::en_eval);

    py::class_<aipu_global_config_hw_t>(m, "aipu_global_config_hw_t")
        .def(py::init<>())
        .def_readwrite("poll_in_commit_thread", &aipu_global_config_hw_t::poll_in_commit_thread);

    #if 0
    py::class_<callback_args_t>(m, "callback_args_t")
        .def(py::init<>())
        .def_readwrite("func_arg", &callback_args_t::func_arg)
        .def_readwrite("job_id", &callback_args_t::job_id)
        .def_readwrite("job_state", &callback_args_t::job_state);

    py::class_<callback_wrapper_t>(m, "callback_wrapper_t")
        .def(py::init<>())
        .def_readwrite("cb_func", &callback_wrapper_t::cb_func)
        .def_readwrite("cb_args", &callback_wrapper_t::cb_args);
    #endif

    py::class_<aipu_core_info_t>(m, "aipu_core_info_t")
        .def(py::init<>())
        .def_readwrite("reg_base", &aipu_core_info_t::reg_base);

    py::enum_<aipu_job_part_t>(m, "aipu_job_part_t")
        .value("AIPU_JOB_PART0", aipu_job_part_t::AIPU_JOB_PART0)
        .value("AIPU_JOB_PART1", aipu_job_part_t::AIPU_JOB_PART1)
        .value("AIPU_JOB_PART2", aipu_job_part_t::AIPU_JOB_PART2)
        .value("AIPU_JOB_PART3", aipu_job_part_t::AIPU_JOB_PART3)
        .export_values();

    py::enum_<aipu_job_qos_t>(m, "aipu_job_qos_t")
        .value("AIPU_JOB_QOS_SLOW", aipu_job_qos_t::AIPU_JOB_QOS_SLOW)
        .value("AIPU_JOB_QOS_HIGH", aipu_job_qos_t::AIPU_JOB_QOS_HIGH)
        .export_values();

    py::enum_<aipu_mem_region_t>(m, "aipu_mem_region_t")
        .value("AIPU_MEM_REGION_DEFAULT", aipu_mem_region_t::AIPU_MEM_REGION_DEFAULT)
        .value("AIPU_MEM_REGION_SRAM", aipu_mem_region_t::AIPU_MEM_REGION_SRAM)
        .value("AIPU_MEM_REGION_DTCM", aipu_mem_region_t::AIPU_MEM_REGION_DTCM)
        .export_values();

    py::class_<aipu_shared_tensor_info_t>(m, "aipu_shared_tensor_info_t")
        .def(py::init<>())
        .def_readwrite("type", &aipu_shared_tensor_info_t::type)
        .def_readwrite("tensor_idx", &aipu_shared_tensor_info_t::tensor_idx)
        .def_readwrite("id", &aipu_shared_tensor_info_t::id)
        .def_readwrite("pa", &aipu_shared_tensor_info_t::pa)
        .def_readwrite("dmabuf_fd", &aipu_shared_tensor_info_t::dmabuf_fd)
        .def_readwrite("offset_in_dmabuf", &aipu_shared_tensor_info_t::offset_in_dmabuf);

    py::enum_<aipu_ioctl_cmd_t>(m, "aipu_ioctl_cmd_t")
        .value("AIPU_IOCTL_ALLOC_SHARE_BUF", aipu_ioctl_cmd_t::AIPU_IOCTL_ALLOC_SHARE_BUF)
        .value("AIPU_IOCTL_FREE_SHARE_BUF", aipu_ioctl_cmd_t::AIPU_IOCTL_FREE_SHARE_BUF)
        .value("AIPU_IOCTL_SET_PROFILE", aipu_ioctl_cmd_t::AIPU_IOCTL_SET_PROFILE)
        .value("AIPU_IOCTL_ALLOC_DMABUF", aipu_ioctl_cmd_t::AIPU_IOCTL_ALLOC_DMABUF)
        .value("AIPU_IOCTL_FREE_DMABUF", aipu_ioctl_cmd_t::AIPU_IOCTL_FREE_DMABUF)
        .value("AIPU_IOCTL_WRITE_DMABUF", aipu_ioctl_cmd_t::AIPU_IOCTL_WRITE_DMABUF)
        .value("AIPU_IOCTL_READ_DMABUF", aipu_ioctl_cmd_t::AIPU_IOCTL_READ_DMABUF)
        .export_values();

    py::enum_<aipu_share_case_type_t>(m, "aipu_share_case_type_t")
        .value("AIPU_SHARE_BUF_IN_ONE_PROCESS", aipu_share_case_type_t::AIPU_SHARE_BUF_IN_ONE_PROCESS)
        .value("AIPU_SHARE_BUF_DMABUF", aipu_share_case_type_t::AIPU_SHARE_BUF_DMABUF)
        .value("AIPU_SHARE_BUF_CUSTOMED", aipu_share_case_type_t::AIPU_SHARE_BUF_CUSTOMED)
        .export_values();

    py::enum_<aipu_ioctl_tickcounter_t>(m, "aipu_ioctl_tickcounter_t")
        .value("AIPU_IOCTL_EN_TICK_COUNTER", aipu_ioctl_tickcounter_t::AIPU_IOCTL_EN_TICK_COUNTER)
        .value("AIPU_IOCTL_DI_TICK_COUNTER", aipu_ioctl_tickcounter_t::AIPU_IOCTL_DI_TICK_COUNTER)
        .export_values();

    py::enum_<data_type_t>(m, "data_type_t")
        .value("D_INT8", data_type_t::D_INT8)
        .value("D_INT16", data_type_t::D_INT16)
        .value("D_INT32", data_type_t::D_INT32)
        .value("D_UINT8", data_type_t::D_UINT8)
        .value("D_UINT16", data_type_t::D_UINT16)
        .value("D_UINT32", data_type_t::D_UINT32)
        .export_values();

    py::enum_<aipu_status_t>(m, "aipu_status_t")
        .value("AIPU_STATUS_SUCCESS", aipu_status_t::AIPU_STATUS_SUCCESS)
        .value("AIPU_STATUS_ERROR_NULL_PTR", aipu_status_t::AIPU_STATUS_ERROR_NULL_PTR)
        .value("AIPU_STATUS_ERROR_INVALID_CTX", aipu_status_t::AIPU_STATUS_ERROR_INVALID_CTX)
        .value("AIPU_STATUS_ERROR_OPEN_DEV_FAIL", aipu_status_t::AIPU_STATUS_ERROR_OPEN_DEV_FAIL)
        .value("AIPU_STATUS_ERROR_DEV_ABNORMAL", aipu_status_t::AIPU_STATUS_ERROR_DEV_ABNORMAL)
        .value("AIPU_STATUS_ERROR_DEINIT_FAIL", aipu_status_t::AIPU_STATUS_ERROR_DEINIT_FAIL)
        .value("AIPU_STATUS_ERROR_INVALID_CONFIG", aipu_status_t::AIPU_STATUS_ERROR_INVALID_CONFIG)
        .value("AIPU_STATUS_ERROR_UNKNOWN_BIN", aipu_status_t::AIPU_STATUS_ERROR_UNKNOWN_BIN)
        .value("AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED", aipu_status_t::AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED)
        .value("AIPU_STATUS_ERROR_INVALID_GBIN", aipu_status_t::AIPU_STATUS_ERROR_INVALID_GBIN)
        .value("AIPU_STATUS_ERROR_TARGET_NOT_FOUND", aipu_status_t::AIPU_STATUS_ERROR_TARGET_NOT_FOUND)
        .value("AIPU_STATUS_ERROR_INVALID_GRAPH_ID", aipu_status_t::AIPU_STATUS_ERROR_INVALID_GRAPH_ID)
        .value("AIPU_STATUS_ERROR_OPEN_FILE_FAIL", aipu_status_t::AIPU_STATUS_ERROR_OPEN_FILE_FAIL)
        .value("AIPU_STATUS_ERROR_MAP_FILE_FAIL", aipu_status_t::AIPU_STATUS_ERROR_MAP_FILE_FAIL)
        .value("AIPU_STATUS_ERROR_READ_FILE_FAIL", aipu_status_t::AIPU_STATUS_ERROR_READ_FILE_FAIL)
        .value("AIPU_STATUS_ERROR_WRITE_FILE_FAIL", aipu_status_t::AIPU_STATUS_ERROR_WRITE_FILE_FAIL)
        .value("AIPU_STATUS_ERROR_INVALID_JOB_ID", aipu_status_t::AIPU_STATUS_ERROR_INVALID_JOB_ID)
        .value("AIPU_STATUS_ERROR_JOB_EXCEPTION", aipu_status_t::AIPU_STATUS_ERROR_JOB_EXCEPTION)
        .value("AIPU_STATUS_ERROR_JOB_TIMEOUT", aipu_status_t::AIPU_STATUS_ERROR_JOB_TIMEOUT)
        .value("AIPU_STATUS_ERROR_OP_NOT_SUPPORTED", aipu_status_t::AIPU_STATUS_ERROR_OP_NOT_SUPPORTED)
        .value("AIPU_STATUS_ERROR_INVALID_OP", aipu_status_t::AIPU_STATUS_ERROR_INVALID_OP)
        .value("AIPU_STATUS_ERROR_INVALID_SIZE", aipu_status_t::AIPU_STATUS_ERROR_INVALID_SIZE)
        .value("AIPU_STATUS_ERROR_BUF_ALLOC_FAIL", aipu_status_t::AIPU_STATUS_ERROR_BUF_ALLOC_FAIL)
        .value("AIPU_STATUS_ERROR_BUF_FREE_FAIL", aipu_status_t::AIPU_STATUS_ERROR_BUF_FREE_FAIL)
        .value("AIPU_STATUS_ERROR_INVALID_CORE_ID", aipu_status_t::AIPU_STATUS_ERROR_INVALID_CORE_ID)
        .value("AIPU_STATUS_ERROR_RESERVE_SRAM_FAIL", aipu_status_t::AIPU_STATUS_ERROR_RESERVE_SRAM_FAIL)
        .value("AIPU_STATUS_ERROR_INVALID_TENSOR_ID", aipu_status_t::AIPU_STATUS_ERROR_INVALID_TENSOR_ID)
        .value("AIPU_STATUS_ERROR_INVALID_CLUSTER_ID", aipu_status_t::AIPU_STATUS_ERROR_INVALID_CLUSTER_ID)
        .value("AIPU_STATUS_ERROR_INVALID_PARTITION_ID", aipu_status_t::AIPU_STATUS_ERROR_INVALID_PARTITION_ID)
        .value("AIPU_STATUS_ERROR_PRINTF_FAIL", aipu_status_t::AIPU_STATUS_ERROR_PRINTF_FAIL)
        .value("AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE", aipu_status_t::AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE)
        .value("AIPU_STATUS_ERROR_INVALID_GM", aipu_status_t::AIPU_STATUS_ERROR_INVALID_GM)
        .value("AIPU_STATUS_ERROR_INVALID_SEGMMU", aipu_status_t::AIPU_STATUS_ERROR_INVALID_SEGMMU)
        .value("AIPU_STATUS_ERROR_INVALID_QOS", aipu_status_t::AIPU_STATUS_ERROR_INVALID_QOS)
        .value("AIPU_STATUS_ERROR_INVALID_TENSOR_CNT", aipu_status_t::AIPU_STATUS_ERROR_INVALID_TENSOR_CNT)
        .value("AIPU_STATUS_ERROR_TIMEOUT", aipu_status_t::AIPU_STATUS_ERROR_TIMEOUT)
        .value("AIPU_STATUS_ERROR_NO_BATCH_QUEUE", aipu_status_t::AIPU_STATUS_ERROR_NO_BATCH_QUEUE)
        .value("AIPU_STATUS_ERROR_MARK_SHARED_TENSOR", aipu_status_t::AIPU_STATUS_ERROR_MARK_SHARED_TENSOR)
        .value("AIPU_STATUS_ERROR_SET_SHARED_TENSOR", aipu_status_t::AIPU_STATUS_ERROR_SET_SHARED_TENSOR)
        .value("AIPU_STATUS_MAX", aipu_status_t::AIPU_STATUS_MAX)
        .value("AIPU_STATUS_ERROR_UNKNOWN_ERROR", aipu_status_t::AIPU_STATUS_ERROR_UNKNOWN_ERROR)
        .value("AIPU_STATUS_ERROR_KEYBOARD_INTERRUPT", aipu_status_t::AIPU_STATUS_ERROR_KEYBOARD_INTERRUPT)
        .value("AIPU_STATUS_ERROR_SYSTEM_ERR", aipu_status_t::AIPU_STATUS_ERROR_SYSTEM_ERR)
        .value("AIPU_STATUS_ERROR_OUT_OF_SPEC", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_SPEC)
        .value("AIPU_STATUS_ERROR_OUT_OF_AIFF_SPEC", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_AIFF_SPEC)
        .value("AIPU_STATUS_ERROR_OUT_OF_TPC_SPEC", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_TPC_SPEC)
        .value("AIPU_STATUS_ERROR_OUT_OF_DMA_SPEC", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_DMA_SPEC)
        .value("AIPU_STATUS_ERROR_OUT_OF_MEM_RANGE", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_MEM_RANGE)
        .value("AIPU_STATUS_ERROR_OUT_OF_SRAM_RANGE", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_SRAM_RANGE)
        .value("AIPU_STATUS_ERROR_OUT_OF_LSRAM0_RANGE", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_LSRAM0_RANGE)
        .value("AIPU_STATUS_ERROR_OUT_OF_LSRAM1_RANGE", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_LSRAM1_RANGE)
        .value("AIPU_STATUS_ERROR_OUT_OF_GSRAM0_RANGE", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_GSRAM0_RANGE)
        .value("AIPU_STATUS_ERROR_OUT_OF_GSRAM1_RANGE", aipu_status_t::AIPU_STATUS_ERROR_OUT_OF_GSRAM1_RANGE)
        .value("AIPU_STATUS_ERROR_ARITHMETIC_ERR", aipu_status_t::AIPU_STATUS_ERROR_ARITHMETIC_ERR)
        .value("AIPU_STATUS_ERROR_FLOAT_POINT_ERR", aipu_status_t::AIPU_STATUS_ERROR_FLOAT_POINT_ERR)
        .value("AIPU_STATUS_ERROR_UNDERFLOW_ERR", aipu_status_t::AIPU_STATUS_ERROR_UNDERFLOW_ERR)
        .value("AIPU_STATUS_ERROR_OVERFLOW_ERR", aipu_status_t::AIPU_STATUS_ERROR_OVERFLOW_ERR)
        .value("AIPU_STATUS_ERROR_NOT_A_NUMBER_ERR", aipu_status_t::AIPU_STATUS_ERROR_NOT_A_NUMBER_ERR)
        .value("AIPU_STATUS_ERROR_INFINITY_ERR", aipu_status_t::AIPU_STATUS_ERROR_INFINITY_ERR)
        .value("AIPU_STATUS_ERROR_STRING_LENGTH_ERR", aipu_status_t::AIPU_STATUS_ERROR_STRING_LENGTH_ERR)
        .value("AIPU_STATUS_ERROR_ZERO_DIVISION_ERR", aipu_status_t::AIPU_STATUS_ERROR_ZERO_DIVISION_ERR)
        .export_values();

    py::class_<NPU>(m, "NPU")
        .def(py::init())
        .def("aipu_init_context", &NPU::aipu_init_context_py)
        .def("aipu_deinit_context", &NPU::aipu_deinit_context_py)
        .def("aipu_get_error_message", &NPU::aipu_get_error_message_py, py::return_value_policy::copy)
        .def("aipu_config_global", &NPU::aipu_config_global_py)
        .def("aipu_load_graph", &NPU::aipu_load_graph_py, py::return_value_policy::copy)
        .def("aipu_load_graph", &NPU::aipu_load_graph_helper_py, py::return_value_policy::copy)
        .def("aipu_unload_graph", &NPU::aipu_unload_graph_py)
        .def("aipu_create_job", &NPU::aipu_create_job_py)
        .def("aipu_config_job", &NPU::aipu_config_job_py)
        .def("aipu_finish_job", &NPU::aipu_finish_job_py)
        .def("aipu_flush_job", &NPU::aipu_flush_job_py)
        .def("aipu_get_job_status", &NPU::aipu_get_job_status_py)
        .def("aipu_clean_job", &NPU::aipu_clean_job_py)
        .def("aipu_get_tensor_count", &NPU::aipu_get_tensor_count_py)
        .def("aipu_get_tensor_descriptor", &NPU::aipu_get_tensor_descriptor_py, py::return_value_policy::copy)
        .def("aipu_load_tensor_from_file", &NPU::aipu_load_tensor_file_py)
        .def("aipu_load_tensor", &NPU::aipu_load_tensor_rawbytes_py)
        .def("aipu_load_tensor", &NPU::aipu_load_tensor_pybytes_py)
        .def("aipu_load_tensor", &NPU::aipu_load_tensor_numpyarray_py)
        .def("aipu_get_tensor", &NPU::aipu_get_tensor_py, py::return_value_policy::copy)
        .def("aipu_ioctl", &NPU::aipu_ioctl_py, py::return_value_policy::copy)
        .def("aipu_ioctl", &NPU::aipu_ioctl_write_dmabuf_py)
        .def("aipu_specify_iobuf", &NPU::aipu_specify_iobuf_py);
}