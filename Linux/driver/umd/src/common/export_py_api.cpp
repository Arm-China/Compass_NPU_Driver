// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
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
#define ORIG_VERSION 1

namespace py = pybind11;

#if ORIG_VERSION
class Graph
{
public:
    /**
     * Internal
     */
    aipu_status_t create_job(aipu_create_job_cfg_t *create_job_config = nullptr)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        ret = aipu_create_job(m_ctx, m_graph_id, &m_job_id, create_job_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_create_job: %s\n", m_status_msg);
        }
        return ret;
    }

public:
    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to get the number of input tensor(s) of a graph.
     */
    int GetInputTensorNumber()
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        if (m_input_cnt != 0)
        {
            return m_input_cnt;
        }

        ret = aipu_get_tensor_count(m_ctx, m_graph_id, AIPU_TENSOR_TYPE_INPUT, &m_input_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_count: %s\n", m_status_msg);
        }
        return m_input_cnt;
    }

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to get the number of output tensor(s) of a graph.
     */
    int GetOutputTensorNumber()
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        if (m_output_cnt != 0)
        {
            return m_output_cnt;
        }

        ret = aipu_get_tensor_count(m_ctx, m_graph_id, AIPU_TENSOR_TYPE_OUTPUT, &m_output_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_count: %s\n", m_status_msg);
        }
        return m_output_cnt;
    }

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to get the number of dump tensor(s) of a graph.
     */
    int GetDumpTensorNumber()
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        if (m_dump_cnt != 0)
        {
            return m_dump_cnt;
        }

        ret = aipu_get_tensor_count(m_ctx, m_graph_id, AIPU_TENSOR_TYPE_INTER_DUMP, &m_dump_cnt);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_count: %s\n", m_status_msg);
        }
        return m_dump_cnt;
    }

#if (defined BUILD_WITH_NUMPY)
    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to load input tensor(s) via a NumPy object.
     *
     * @param[in] id    An input tensor ID
     * @param[in] data  Pointer to the input tensor data
     * @param[in] bytes Tensor size in byte (C++ API only)
     *
     * @python_api LoadInputTensor(int id, numpy obj)
     */
    int LoadInputTensor(int id, int* data, int bytes)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        ret = aipu_load_tensor(m_ctx, m_job_id, id, data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", m_status_msg);
        }
        return ret;
    }
#endif

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to load input tensor(s) via a binary file name.
     *
     * @param[in] id        An input tensor ID
     * @param[in] filename  Name of the input tensor specified by id which contains binary data
     */
    int LoadInputTensor(int id, const char* filename)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        int fd = 0;
        void* data = nullptr;
        aipu_tensor_desc_t desc;

        ret = aipu_get_tensor_descriptor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_INPUT, id, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", m_status_msg);
            return ret;
        }

        fd = open(filename, O_RDONLY);
        if (fd <= 0)
        {
            fprintf(stderr, "[PY UMD ERROR] open file failed: %s! (errno = %d)\n", filename, errno);
            return errno;
        }

        data = mmap(nullptr, desc.size, PROT_READ, MAP_PRIVATE, fd, 0);
        if (MAP_FAILED == data)
        {
            fprintf(stderr, "[PY UMD ERROR] RT failed in mapping graph file: %s! (errno = %d)\n", filename, errno);
            close(fd);
            return -1;
        }

        ret = aipu_load_tensor(m_ctx, m_job_id, id, data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", m_status_msg);
        }

        munmap(data, desc.size);

        if (fd > 0)
        {
            close(fd);
        }
        return ret;
    }

    /**
     * @brief This API is exposed as a a SWIG-Python API.
     *        It is used to run a loaded graph after all input tensors are loaded.
     */
    int Run()
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        aipu_job_config_dump_t mem_dump_config;

        memset(&mem_dump_config, 0, sizeof(mem_dump_config));
        mem_dump_config.prefix = nullptr;
        mem_dump_config.dump_dir = "./";

        ret = aipu_config_job(m_ctx, m_job_id, AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, &mem_dump_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_config_job(type dump output): %s\n", m_status_msg);
            return ret;
        }

#if defined(SIMULATION)
        aipu_job_config_simulation_t sim_job_config;
        memset(&sim_job_config, 0, sizeof(sim_job_config));
        sim_job_config.data_dir= "./";
        ret = aipu_config_job(m_ctx, m_job_id, AIPU_CONFIG_TYPE_SIMULATION, &sim_job_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_config_job(type simulation): %s\n", m_status_msg);
            return ret;
        }
#endif

        ret = aipu_finish_job(m_ctx, m_job_id, -1);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_finish_job: %s\n", m_status_msg);
            return ret;
        }

        return ret;
    }

    /**
     * @brief This API is exposed as a a SWIG-Python API.
     *        It is used to get output tensor(s) after then graph execution finishes.
     *
     * @param[in] id  A output tensor ID
     */
    std::vector<int> GetOutputTensor(int id)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        std::vector<int> output;
        unsigned char* out_data;
        aipu_tensor_desc_t desc;

        ret = aipu_get_tensor_descriptor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_OUTPUT, id, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", m_status_msg);
            return output;
        }

        out_data = new unsigned char[desc.size];
        ret = aipu_get_tensor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_OUTPUT, id, out_data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor: %s\n", m_status_msg);
            goto finish;
        }

        for (uint32_t i = 0; i < desc.size; i++)
        {
            output.push_back((int)out_data[i]);
        }

    finish:
        delete[] out_data;
        return output;
    }

    /**
     * @brief This API is exposed as a a SWIG-Python API.
     *        It is used to get dump tensor(s) after then graph execution finishes.
     *
     * @param[in] id  A dump tensor ID
     */
    std::vector<int> GetDumpTensor(int id)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        std::vector<int> output;
        unsigned char* out_data;
        aipu_tensor_desc_t desc;

        ret = aipu_get_tensor_descriptor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_INTER_DUMP, id, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", m_status_msg);
            return output;
        }

        out_data = new unsigned char[desc.size];
        ret = aipu_get_tensor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_INTER_DUMP, id, out_data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor: %s\n", m_status_msg);
            goto finish;
        }

        for (uint32_t i = 0; i < desc.size; i++)
        {
            output.push_back((int)out_data[i]);
        }

    finish:
        delete[] out_data;
        return output;
    }

    /**
     * @brief This API is exposed as a a SWIG-Python API.
     *        It is used to get profile tensor(s) after then graph execution finishes.
     *
     * @param[in] id  A profile tensor ID
     */
    std::vector<int> GetProfileTensor(int id)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        std::vector<int> output;
        unsigned char* out_data;
        aipu_tensor_desc_t desc;

        ret = aipu_get_tensor_descriptor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_PROFILER, id, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", m_status_msg);
            return output;
        }

        out_data = new unsigned char[desc.size];
        ret = aipu_get_tensor(m_ctx, m_job_id, AIPU_TENSOR_TYPE_PROFILER, id, out_data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor: %s\n", m_status_msg);
            goto finish;
        }

        for (uint32_t i = 0; i < desc.size; i++)
        {
            output.push_back((int)out_data[i]);
        }

    finish:
        delete[] out_data;
        return output;
    }

public:
    Graph(aipu_ctx_handle_t* ctx, uint64_t graph_id)
    {
        m_ctx = ctx;
        m_graph_id = graph_id;
    }
    Graph(const Graph& graph) = delete;
    Graph& operator=(const Graph& graph) = delete;
    ~Graph()
    {
        if (m_job_id)
        {
            aipu_clean_job(m_ctx, m_job_id);
            m_job_id = 0;
        }
        aipu_unload_graph(m_ctx, m_graph_id);
        m_graph_id = 0;
    }

private:
    aipu_ctx_handle_t* m_ctx;
    const char* m_status_msg;
    uint64_t m_graph_id;
    uint64_t m_job_id;
    uint32_t m_input_cnt = 0;
    uint32_t m_output_cnt = 0;
    uint32_t m_dump_cnt = 0;
};

class Aipu
{
public:
    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to provide aipu v1/v2 simulator path.
     *
     * @param[in] sim  Name of the simulator with path
     */
    void SetSimulator(const char* sim)
    {
        if (sim != nullptr)
        {
            strcpy(m_sim, sim);
            m_enable_v1v2 = true;
        }
    }

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to provide x2 ARCH version to make
     *        enable the specific Simulator.
     *
     * @param[in] x2_arch_desc  ARCH version
     */
    void SetX2Arch(const char *x2_arch_desc)
    {
        if (x2_arch_desc != nullptr)
        {
            strcpy(m_x2_arch_desc, x2_arch_desc);
            m_enable_x2 = true;
        }
    }

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is only used to bind a X2 Job to a specific partition
     *        and with a specific QoS level.
     *      *
     * @param[in] part_id currently only support defalut 0
     * @param[in] qos_level currently only defalut low(0), but support high(1) and low(0)
     */
    void SetX2JobConfig(uint8_t part_id, uint8_t qos_level)
    {
        m_x2_create_job_config.partition_id = part_id;
        m_x2_create_job_config.qos_level = qos_level;
    }

    /**
     * @brief This API is used to send specific command to NPU driver.
     *
     * @param[in] cmd cmd
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_DEV_ABNORMAL
     *
     * @note support commands currently
     *       AIPU_IOCTL_ENABLE_TICK_COUNTER:
     *           enable performance counter, no arg
     *       AIPU_IOCTL_DISABLE_TICK_COUNTER:
     *           disable performance counter, no arg
     */
    #define IOCTL_ENABLE_TICK_COUNTER  0
    #define IOCTL_DISABLE_TICK_COUNTER 1
    int MiscIoctl(int cmd)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        uint32_t ioctl_cmd = 0;

        switch (cmd)
        {
            case IOCTL_ENABLE_TICK_COUNTER:
                ioctl_cmd = AIPU_IOCTL_ENABLE_TICK_COUNTER;
                break;

            case IOCTL_DISABLE_TICK_COUNTER:
                ioctl_cmd = AIPU_IOCTL_DISABLE_TICK_COUNTER;
                break;

            default:
                fprintf(stderr, "[PY UMD ERROR] AipuIoctl: no support cmd %d\n", cmd);
                return -1;
        }

        ret = aipu_ioctl(m_ctx, ioctl_cmd, nullptr);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] AipuIoctl: %s\n", m_status_msg);
            return ret;
        }

        return ret;
    }

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to load a graph.
     *
     * @param[in] filename  Name of the graph binary file
     */
    Graph* LoadGraph(char* filename)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        Graph* graph = nullptr;

#if (defined SIMULATION)
        aipu_global_config_simulation_t sim_glb_config;

        memset(&sim_glb_config, 0, sizeof(sim_glb_config));
        if (m_enable_v1v2)
        {
            sim_glb_config.simulator = m_sim;
        } else {
            sim_glb_config.simulator = nullptr;
        }

        if (m_enable_x2)
        {
            sim_glb_config.x2_arch_desc = m_x2_arch_desc;
        } else {
            sim_glb_config.x2_arch_desc = nullptr;
        }

#if ((defined RTDEBUG) && (RTDEBUG == 1))
        sim_glb_config.log_level = 3;
#else
        sim_glb_config.log_level = 0;
#endif
        sim_glb_config.verbose = 0;
        ret = aipu_config_global(m_ctx, AIPU_CONFIG_TYPE_SIMULATION, &sim_glb_config);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_config_global: %s\n", m_status_msg);
            return nullptr;
        }
#endif

        ret = aipu_load_graph(m_ctx, filename, &m_graph_id);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] AIPU_load_graph_helper: %s\n", m_status_msg);
            return nullptr;
        }

        graph = new Graph(m_ctx, m_graph_id);
        ret = graph->create_job(&m_x2_create_job_config);

        if (ret != AIPU_STATUS_SUCCESS) {
            delete graph;
            graph = nullptr;
        }
        return graph;
    }

    /**
     * @brief This API is exposed as a SWIG-Python API.
     *        It is used to unload a graph.
     *
     * @param[in] graph Pointer to the graph object
     */
    void UnloadGraph(Graph* graph)
    {
        if (graph != nullptr)
        {
            delete graph;
        }
    }

public:
    static Aipu& get_aipu()
    {
        static Aipu aipu;
        return aipu;
    }

    int init_dev()
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        if (m_ctx != nullptr)
        {
            return 0;
        }

        ret = aipu_init_context(&m_ctx);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &m_status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_init_context: %s\n", m_status_msg);
            m_ctx = nullptr;
        }
        return ret;
    }

    void deinit_dev()
    {
        if (m_ctx)
        {
            aipu_deinit_context(m_ctx);
            m_ctx = nullptr;
        }
    }

public:
    Aipu(){};
    Aipu(const Aipu& aipu) = delete;
    Aipu& operator=(const Aipu& aipu) = delete;
    ~Aipu(){};

private:
    aipu_ctx_handle_t* m_ctx = nullptr;
    const char* m_status_msg;
    char        m_sim[1024];
    bool        m_enable_v1v2 = false;
    char        m_x2_arch_desc[16];
    bool        m_enable_x2 = false;
    aipu_create_job_cfg_t m_x2_create_job_config = {0};
    uint64_t    m_graph_id;
};

Aipu& OpenDevice()
{
    Aipu& aipu = Aipu::get_aipu();
    aipu.init_dev();
    return aipu;
}

void CloseDevice()
{
    Aipu& aipu = Aipu::get_aipu();
    aipu.deinit_dev();
}
#endif

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
    std::string aipu_get_error_message_py(aipu_status_t status)
    {
        const char *status_msg = nullptr;

        aipu_get_error_message(m_ctx, status, (const char**)&status_msg);
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
    aipu_status_t aipu_flush_job_py(uint64_t job_id)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        callback_wrapper_t *cb_wrap = nullptr;

        ret = aipu_flush_job(m_ctx, job_id, cb_wrap);
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
        py::array_t<char> numpy_array)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        const char *data = numpy_array.data();

         ret = aipu_load_tensor(m_ctx, job_id, tensor, data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_load_tensor: %s\n", status_msg);
        }

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
    std::map<std::string, std::vector<int> > aipu_get_tensor_py(uint64_t job_id,
        aipu_tensor_type_t type, uint32_t tensor)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;
        const char *status_msg = nullptr;
        std::map<std::string, std::vector<int> > retmap;
        unsigned char *out_data = nullptr;
        aipu_tensor_desc_t desc;

        ret = aipu_get_tensor_descriptor(m_ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT, tensor, &desc);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor_descriptor: %s\n", status_msg);
            retmap["ret"] = {ret};
            retmap["data"] = {};
            goto finish;
        }

        out_data = new unsigned char[desc.size];
        ret = aipu_get_tensor(m_ctx, job_id, AIPU_TENSOR_TYPE_OUTPUT, tensor, out_data);
        if (ret != AIPU_STATUS_SUCCESS)
        {
            aipu_get_error_message(m_ctx, ret, &status_msg);
            fprintf(stderr, "[PY UMD ERROR] aipu_get_tensor: %s\n", status_msg);
            retmap["ret"] = {ret};
            retmap["data"] = {};
            goto finish;
        }

        retmap["ret"] = {ret};
        for (uint32_t i = 0; i < desc.size; i++)
            retmap["data"].push_back((int)out_data[i]);

    finish:
        if (out_data)
            delete[] out_data;
        return retmap;
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
        .value("AIPU_IOCTL_MARK_SHARED_TENSOR", aipu_ioctl_cmd_t::AIPU_IOCTL_MARK_SHARED_TENSOR)
        .value("AIPU_IOCTL_SET_SHARED_TENSOR", aipu_ioctl_cmd_t::AIPU_IOCTL_SET_SHARED_TENSOR)
        .value("AIPU_IOCTL_SET_PROFILE", aipu_ioctl_cmd_t::AIPU_IOCTL_SET_PROFILE)
        .value("AIPU_IOCTL_ALLOC_DMABUF", aipu_ioctl_cmd_t::AIPU_IOCTL_ALLOC_DMABUF)
        .value("AIPU_IOCTL_FREE_DMABUF", aipu_ioctl_cmd_t::AIPU_IOCTL_FREE_DMABUF)
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

    #if ORIG_VERSION
    /* bind global function */
    m.def("OpenDevice", &OpenDevice, py::return_value_policy::reference);
    m.def("CloseDevice", &CloseDevice, "Close device");

    /* bind to Graph class */
    py::class_<Graph>(m, "Graph")
        .def(py::init<aipu_ctx_handle_t *, uint64_t>())
        .def("create_job", &Graph::create_job)
        .def("GetInputTensorNumber", &Graph::GetInputTensorNumber)
        .def("GetOutputTensorNumber", &Graph::GetOutputTensorNumber)
        .def("GetDumpTensorNumber", &Graph::GetDumpTensorNumber)
        .def("LoadInputTensor", &Graph::LoadInputTensor)
        .def("Run", &Graph::Run)
        .def("GetOutputTensor", &Graph::GetOutputTensor)
        .def("GetDumpTensor", &Graph::GetDumpTensor)
        .def("GetProfileTensor", &Graph::GetProfileTensor)
        .def("Run", &Graph::Run)
        .def("GetOutputTensor", &Graph::GetOutputTensor)
        .def("GetDumpTensor", &Graph::GetDumpTensor);

    /* bind to Aipu class */
    py::class_<Aipu>(m, "Aipu")
        .def(py::init())
        .def("SetSimulator", &Aipu::SetSimulator)
        .def("SetX2Arch", &Aipu::SetX2Arch)
        .def("SetX2JobConfig", &Aipu::SetX2JobConfig)
        .def("MiscIoctl", &Aipu::MiscIoctl)
        .def("LoadGraph", &Aipu::LoadGraph, py::return_value_policy::reference)
        .def("UnloadGraph", &Aipu::UnloadGraph)
        .def("get_aipu", &Aipu::get_aipu)
        .def("init_dev", &Aipu::init_dev)
        .def("deinit_dev", &Aipu::deinit_dev);
    #endif

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
        .def("aipu_get_tensor", &NPU::aipu_get_tensor_py, py::return_value_policy::copy);
}