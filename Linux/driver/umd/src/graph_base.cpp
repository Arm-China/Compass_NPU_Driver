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
}

aipudrv::GraphBase::~GraphBase()
{
    pthread_rwlock_destroy(&m_lock);
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
