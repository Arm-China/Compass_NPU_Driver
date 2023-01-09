// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph_base.h
 * @brief AIPU User Mode Driver (UMD) graph base module header
 */

#ifndef _GRAPH_BASE_H_
#define _GRAPH_BASE_H_

#include <fstream>
#include <map>
#include <set>
#include <pthread.h>
#include "standard_api.h"
#include "device_base.h"
#include "memory_base.h"
#include "type.h"

namespace aipudrv
{

typedef struct batch_info
{
    std::vector<char *> inputs;
    std::vector<char *> outputs;

    batch_info ()
    {
        inputs.clear();
        outputs.clear();
    }

    batch_info &operator ()(char *input_buf[], uint32_t input_cnt,
        char *output_buf[], uint32_t output_cnt)
    {
        for (uint32_t i = 0; i < input_cnt; i++)
            inputs.push_back(input_buf[i]);

        for (uint32_t i = 0; i < output_cnt; i++)
            outputs.push_back(output_buf[i]);

        return *this;
    }

    void clear()
    {
        for (auto input : inputs)
            delete input;

        for (auto output : outputs)
            delete output;

        inputs.clear();
        outputs.clear();
    }
} batch_info_t;

class JobBase;
class GraphBase
{
protected:
    void*    m_ctx;
    GRAPH_ID m_id;
    uint32_t m_gversion = 0;
    uint32_t m_arch = 0;
    uint32_t m_hw_version = 0;
    uint32_t m_hw_config = 0;
    uint32_t m_hw_revision = 0;
    uint32_t m_asid_flag = 0;
    uint32_t m_remap_flag = 0;
    uint32_t m_sram_flag = 0;
    uint32_t m_wt_mem_region = AIPU_MEM_REGION_DEFAULT;

protected:
    DeviceBase* m_dev;
    MemoryBase* m_mem;

protected:
    std::map<JOB_ID, JobBase*> m_jobs;
    pthread_rwlock_t m_lock;

protected:
    virtual JOB_ID create_job_id_inner();
    JOB_ID add_job(JobBase* job);
    aipu_status_t destroy_jobs();

protected:
    pthread_rwlock_t m_alloc_wt_lock;

public:
    uint32_t m_input_cnt = 0;
    uint32_t m_output_cnt = 0;
    // std::vector<batch_info_t> m_batches;
    typedef struct {
        uint32_t batch_dump_types;
        std::string batch_dump_dir;
        std::vector<batch_info_t> batches;
    } batch_set_t;
    std::map<uint32_t, batch_set_t> m_batch_queue;
    pthread_rwlock_t m_batch_queue_lock;


public:
    virtual void print_parse_info() = 0;
    virtual aipu_status_t load(std::istream& gbin, uint32_t size, bool ver_check = true) = 0;
    virtual aipu_status_t unload() = 0;
    virtual aipu_status_t create_job(JOB_ID* id, const aipu_global_config_simulation_t* cfg,
        aipu_create_job_cfg_t *config = nullptr) = 0;
    virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt) = 0;
    virtual aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type,
        uint32_t tensor, aipu_tensor_desc_t* desc) = 0;
    virtual DEV_PA_64 debugger_get_instr_base() = 0;

    JobBase* get_job(JOB_ID id)
    {
        JobBase* job = nullptr;
        pthread_rwlock_rdlock(&m_lock);
        job = (m_jobs.count(id) ? m_jobs[id]: nullptr);
        pthread_rwlock_unlock(&m_lock);
        return job;
    }
    aipu_status_t destroy_job(JOB_ID id);

public:
    aipu_status_t get_batch_queue_id(uint32_t *queue_id);
    aipu_status_t clean_batch_queue(uint32_t queue_id);
    aipu_status_t clean_batches(uint32_t queue_id);
    bool is_valid_batch_queue(uint32_t queue_id);
    uint32_t get_batch_queue_size(uint32_t queue_id);
    batch_info_t& get_batch_queue_item(uint32_t queue_id, uint32_t batch_num);
    aipu_status_t add_batch(uint32_t queue_id, char *inputs[], uint32_t input_cnt,
        char *outputs[], uint32_t output_cnt);
    aipu_status_t config_for_batch(uint32_t queue_id, uint64_t types, aipu_job_config_dump_t *config);
    uint32_t get_batch_dump_type(uint32_t queue_id);
    const char* get_batch_dump_path(uint32_t queue_id);

public:
    /* Set functions */
    void set_gversion(uint32_t version)
    {
        m_gversion = version;
    }
    void set_arch(uint32_t arch)
    {
        m_arch = arch;
    }
    void set_hw_version(uint32_t hw_version)
    {
        m_hw_version = hw_version;
    }
    void set_hw_config(uint32_t hw_config)
    {
        m_hw_config = hw_config;
    }
    void set_hw_revision(uint32_t hw_revision)
    {
        m_hw_revision = hw_revision;
    }
    void set_asid_flag(uint32_t asid_flag)
    {
        m_asid_flag = asid_flag;
    }
    void set_sram_flag(uint32_t sram_flag)
    {
        m_sram_flag = sram_flag;
    }
    void set_remap_flag(uint32_t flag)
    {
        m_remap_flag = flag;
    }

    /* Get functions */
    uint32_t get_gversion()
    {
        return m_gversion;
    }
    uint32_t get_remap_flag()
    {
        return m_remap_flag;
    }
    uint32_t get_config()
    {
        return m_hw_config;
    }

    void set_weight_region(uint32_t mem_region)
    {
        m_wt_mem_region = mem_region;
    }

    uint32_t get_weight_region()
    {
        return m_wt_mem_region;
    }

public:
    GraphBase(void* ctx, GRAPH_ID id, DeviceBase* dev);
    virtual ~GraphBase();
    GraphBase(const GraphBase& base) = delete;
    GraphBase& operator=(const GraphBase& base) = delete;
};
}

#endif /* _GRAPH_BASE_H_ */
