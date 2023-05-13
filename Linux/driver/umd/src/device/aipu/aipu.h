// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  aipu.h
 * @brief AIPU User Mode Driver (UMD) aipu module header
 */

#ifndef _AIPU_H_
#define _AIPU_H_

#include <vector>
#include <mutex>
#include "device_base.h"
#include "type.h"
#include "ukmemory.h"

namespace aipudrv
{

/**
 * @brief the queue for storing jobs which are done on HW cores.
 */
class JobStatusQueue
{
    private:
    std::map<JOB_ID, aipu_job_status_desc> m_jobstsq;
    pthread_rwlock_t m_jobstsq_lock;

    public:
    void push_q(aipu_job_status_desc job_status_desc)
    {
        pthread_rwlock_wrlock(&m_jobstsq_lock);
        m_jobstsq[job_status_desc.job_id] = job_status_desc;
        pthread_rwlock_unlock(&m_jobstsq_lock);
    }

    aipu_job_status_desc *pop_q(JOB_ID job_id)
    {
        aipu_job_status_desc *desc = nullptr;
        pthread_rwlock_wrlock(&m_jobstsq_lock);
        if (m_jobstsq.count(job_id))
        {
            desc = &m_jobstsq[job_id];
            m_jobstsq.erase(job_id);
        }
        pthread_rwlock_unlock(&m_jobstsq_lock);

        return desc;
    }

    bool is_job_exist(JOB_ID job_id)
    {
        return m_jobstsq.count(job_id) == 1;
    }

    public:
    JobStatusQueue()
    {
        pthread_rwlock_init(&m_jobstsq_lock, NULL);
    }

    ~JobStatusQueue()
    {
        pthread_rwlock_destroy(&m_jobstsq_lock);
    }

    JobStatusQueue(const JobStatusQueue& jobq) = delete;
    JobStatusQueue& operator=(const JobStatusQueue& jobq) = delete;
};

class Aipu : public DeviceBase
{
protected:
    int m_fd = 0;
    JobStatusQueue m_job_sts_queue;
    bool m_tick_counter = false;

private:
    aipu_ll_status_t init();
    void deinit();

public:
    virtual bool has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev);
    virtual aipu_status_t schedule(const JobDesc& job);
    virtual aipu_ll_status_t read_reg(uint32_t core_id, uint32_t offset, uint32_t* value);
    virtual aipu_ll_status_t write_reg(uint32_t core_id, uint32_t offset, uint32_t value);
    aipu_ll_status_t get_status(std::vector<aipu_job_status_desc>& jobs_status,
        uint32_t max_cnt, bool of_this_thread, callback_wrapper_t *cb_wrap = nullptr);
    virtual aipu_ll_status_t get_status(std::vector<aipu_job_status_desc>& jobs_status,
        uint32_t max_cnt, void *jobbase = nullptr);
    virtual aipu_ll_status_t poll_status(std::vector<aipu_job_status_desc>& jobs_status,
        uint32_t max_cnt, int32_t time_out, bool of_this_thread, void *jobbase = nullptr,
        callback_wrapper_t *cb_wrap = nullptr);

public:
    virtual aipu_ll_status_t ioctl_cmd(uint32_t cmd, void *arg);

public:
    static aipu_status_t get_aipu(DeviceBase** dev)
    {
        aipu_status_t ret = AIPU_STATUS_SUCCESS;

        if (nullptr == dev)
            return AIPU_STATUS_ERROR_NULL_PTR;

        std::lock_guard<std::mutex> lock_(m_tex);
        if (m_aipu == nullptr)
        {
            m_aipu = new Aipu();
            ret = convert_ll_status(m_aipu->init());
            if (ret != AIPU_STATUS_SUCCESS)
            {
                delete m_aipu;
                m_aipu = nullptr;
                return ret;
            }
        }

        m_aipu->inc_ref_cnt();
        *dev = m_aipu;
        return AIPU_STATUS_SUCCESS;
    };

    static void put_aipu(DeviceBase* dev)
    {
        delete (Aipu *)dev;
        dev = nullptr;
        m_aipu = nullptr;
    }

    virtual ~Aipu();
    Aipu(const Aipu& dev) = delete;
    Aipu& operator=(const Aipu& dev) = delete;

private:
    Aipu();
    static std::mutex m_tex;
    static Aipu* m_aipu;
};
}

#endif /* _AIPU_H_ */
