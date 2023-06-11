// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  swig_cpp2py_api.hpp
 * @brief AIPU User Mode Driver (UMD) C++ to Python API header
 * @version 1.0
 */

#ifndef _SWIG_CPP2PY_API_HPP_
#define _SWIG_CPP2PY_API_HPP_

#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <vector>
#include "standard_api.h"

class Graph
{
public:
    /**
     * Internal
     */
    int create_job(aipu_create_job_cfg_t *create_job_config = nullptr)
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
     * @param[in] ctx Pointer to a context handle struct returned by aipu_init_context
     * @param[in] cmd cmd
     * @param[inout] arg input or output argument according to 'cmd'
     *
     * @retval AIPU_STATUS_SUCCESS
     * @retval AIPU_STATUS_ERROR_NULL_PTR
     * @retval AIPU_STATUS_ERROR_INVALID_CTX
     * @retval AIPU_STATUS_ERROR_DEV_ABNORMAL
     *
     * @note support commands currently
     *       AIPU_IOCTL_MARK_SHARED_TENSOR:
     *           mark a shared buffer from belonged job, arg {aipu_shared_tensor_info_t}
     *           the marked buffer isn't freed on destroying job and directly used by new job.
     *       AIPU_IOCTL_SET_SHARED_TENSOR:
     *           specify a marked shared buffer to new job, arg {aipu_shared_tensor_info_t}
     *           marking shared buffer and setting shared buffer to new job must be performed
     *           in same one process context.
     *       AIPU_IOCTL_ENABLE_TICK_COUNTER:
     *           enable performance counter, no arg
     *       AIPU_IOCTL_DISABLE_TICK_COUNTER:
     *           disable performance counter, no arg
     *       AIPU_IOCTL_CONFIG_CLUSTERS:
     *           config number of enabled cores in a cluster, arg {struct aipu_config_clusters}
     *           it is used to disable some core's clock to reduce power consumption.
     *       AIPU_IOCTL_SET_PROFILE:
     *           dynamically enable/disable profiling feature of aipu v3 simulation. arg {1/0}
     *           1: enable profiling
     *           0: disable profiling
     *      AIPU_IOCTL_ALLOC_DMABUF
     *           request dma_buf from KMD. arg {struct aipu_dma_buf_request}
     *           aipu_dma_buf_request->bytes: request size (filled by UMD)
     *           aipu_dma_buf_request->fd: fd corresponding to dma_buf (filled by KMD)
     *      AIPU_IOCTL_FREE_DMABUF
     *           free a dma_buf with its fd. arg { fd }
     */
    int MiscIoctl(aipu_ctx_handle_t *ctx, uint32_t cmd, void *arg = nullptr)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        ret = aipu_ioctl(m_ctx, cmd, arg);
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
        graph->create_job(&m_x2_create_job_config);

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

private:
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

/**
 * @brief This API is exposed as a SWIG-Python API.
 *        It is used to open the AIPU device which should be called first of all.
 */
Aipu& OpenDevice();
/**
 * @brief This API is exposed as a SWIG-Python API.
 *        It is used to close an opened AIPU device which should be called at last.
 */
void  CloseDevice();

#endif /* _SWIG_CPP2PY_API_HPP_ */