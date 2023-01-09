// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  graph_base.cpp
 * @brief AIPU User Mode Driver (UMD) graph base module implementation
 */
#include <assert.h>
#include "graph_base.h"
#include "job_base.h"

aipudrv::GraphBase::GraphBase(void* ctx, GRAPH_ID id, DeviceBase* dev):
    m_ctx(ctx),
    m_id(id),
    m_dev(dev)
{
    m_mem = m_dev->get_mem();
    pthread_rwlock_init(&m_lock, NULL);
    pthread_rwlock_init(&m_alloc_wt_lock, NULL);
    pthread_rwlock_init(&m_batch_queue_lock, NULL);
}

aipudrv::GraphBase::~GraphBase()
{
    pthread_rwlock_destroy(&m_lock);
    pthread_rwlock_destroy(&m_alloc_wt_lock);
    pthread_rwlock_destroy(&m_batch_queue_lock);
}

aipudrv::JOB_ID aipudrv::GraphBase::create_job_id_inner()
{
    uint64_t id_candidate = create_full_job_id(m_id, 1);

    while (m_jobs.count(id_candidate) == 1)
    {
        id_candidate++;
    }
    return id_candidate;
}

aipudrv::JOB_ID aipudrv::GraphBase::add_job(JobBase* job)
{
    assert(job != nullptr);
    pthread_rwlock_wrlock(&m_lock);
    job->set_id(create_job_id_inner());
    m_jobs[job->get_id()] = job;
    pthread_rwlock_unlock(&m_lock);
    return job->get_id();
}

aipu_status_t aipudrv::GraphBase::destroy_jobs()
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    auto iter = m_jobs.begin();

    pthread_rwlock_wrlock(&m_lock);
    while(iter != m_jobs.end())
    {
        ret = iter->second->destroy();
        if (ret != AIPU_STATUS_SUCCESS)
            goto unlock;

        delete iter->second;
        m_jobs.erase(iter);
        iter = m_jobs.begin();
    }

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

aipu_status_t aipudrv::GraphBase::destroy_job(JOB_ID id)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    pthread_rwlock_wrlock(&m_lock);
    if (m_jobs.count(id) != 0)
    {
        ret = m_jobs[id]->destroy();
        if (ret != AIPU_STATUS_SUCCESS)
            goto unlock;

        delete m_jobs[id];
        m_jobs.erase(id);
    }

unlock:
    pthread_rwlock_unlock(&m_lock);
    return ret;
}

aipu_status_t aipudrv::GraphBase::get_batch_queue_id(uint32_t *queue_id)
{
    static uint32_t batch_queue_id = 0;

    pthread_rwlock_wrlock(&m_batch_queue_lock);
    while(m_batch_queue.count(batch_queue_id) != 0)
        batch_queue_id++;

    m_batch_queue[batch_queue_id].batches.clear();
    m_batch_queue[batch_queue_id].batch_dump_dir = "";
    m_batch_queue[batch_queue_id].batch_dump_types = 0;
    *queue_id = batch_queue_id;
    pthread_rwlock_unlock(&m_batch_queue_lock);

    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::GraphBase::config_for_batch(uint32_t queue_id, uint64_t types,
    aipu_job_config_dump_t *config)
{
    pthread_rwlock_wrlock(&m_batch_queue_lock);
    if (m_batch_queue.count(queue_id) == 0)
        return AIPU_STATUS_ERROR_NO_BATCH_QUEUE;

    m_batch_queue[queue_id].batch_dump_dir = config->dump_dir;
    m_batch_queue[queue_id].batch_dump_types = types;
    pthread_rwlock_unlock(&m_batch_queue_lock);
    return AIPU_STATUS_SUCCESS;
}

uint32_t aipudrv::GraphBase::get_batch_dump_type(uint32_t queue_id)
{
    if (m_batch_queue.count(queue_id) == 0)
        return 0;

    return m_batch_queue[queue_id].batch_dump_types;
}

const char* aipudrv::GraphBase::get_batch_dump_path(uint32_t queue_id)
{
    if (m_batch_queue.count(queue_id) == 0)
        return nullptr;

    return m_batch_queue[queue_id].batch_dump_dir.c_str();
}

aipu_status_t aipudrv::GraphBase::clean_batches(uint32_t queue_id)
{
    if (!is_valid_batch_queue(queue_id))
        return AIPU_STATUS_ERROR_NO_BATCH_QUEUE;

    pthread_rwlock_wrlock(&m_batch_queue_lock);
    m_batch_queue[queue_id].batches.clear();
    pthread_rwlock_unlock(&m_batch_queue_lock);
    return AIPU_STATUS_SUCCESS;
}

aipu_status_t aipudrv::GraphBase::clean_batch_queue(uint32_t queue_id)
{
    if (!is_valid_batch_queue(queue_id))
        return AIPU_STATUS_ERROR_NO_BATCH_QUEUE;

    pthread_rwlock_wrlock(&m_batch_queue_lock);
    if (m_batch_queue.count(queue_id) != 0)
    {
        m_batch_queue[queue_id].batches.clear();
        m_batch_queue.erase(queue_id);
    }
    pthread_rwlock_unlock(&m_batch_queue_lock);

    return AIPU_STATUS_SUCCESS;
}

bool aipudrv::GraphBase::is_valid_batch_queue(uint32_t queue_id)
{
    if (m_batch_queue.count(queue_id) != 0)
        return true;
    else
        return false;
}

uint32_t aipudrv::GraphBase::get_batch_queue_size(uint32_t queue_id)
{
    if (m_batch_queue.count(queue_id) != 0)
        return m_batch_queue[queue_id].batches.size();
    else
        return 0;
}

aipudrv::batch_info_t& aipudrv::GraphBase::get_batch_queue_item(uint32_t queue_id, uint32_t batch_num)
{
    return m_batch_queue[queue_id].batches[batch_num];
}

aipu_status_t aipudrv::GraphBase::add_batch(uint32_t queue_id, char *inputs[], uint32_t input_cnt,
    char *outputs[], uint32_t output_cnt)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    batch_info_t inter_batch;

    if (!is_valid_batch_queue(queue_id))
        return AIPU_STATUS_ERROR_NO_BATCH_QUEUE;

    inter_batch(inputs, input_cnt, outputs, output_cnt);
    pthread_rwlock_wrlock(&m_batch_queue_lock);
    m_batch_queue[queue_id].batches.push_back(inter_batch);
    pthread_rwlock_unlock(&m_batch_queue_lock);
    return ret;
}
