// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  aipu.cpp
 * @brief AIPU User Mode Driver (UMD) aipu module header
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include "aipu.h"
#include "ukmemory.h"
#include "job_base.h"
#include "helper.h"

aipudrv::Aipu* aipudrv::Aipu::m_aipu = nullptr;
std::mutex aipudrv::Aipu::m_tex;

aipudrv::Aipu::Aipu()
{
    m_dev_type = DEV_TYPE_AIPU;
}

aipudrv::Aipu::~Aipu()
{
    deinit();
}

aipu_ll_status_t aipudrv::Aipu::init()
{
    aipu_ll_status_t ret = AIPU_LL_STATUS_SUCCESS;
    int kret = 0;
    aipu_cap cap;
    aipu_partition_cap *part_caps = NULL;

    m_fd = open("/dev/aipu", O_RDWR | O_SYNC);
    if (m_fd <= 0)
    {
        m_fd = 0;
        LOG(LOG_ERR, "open /dev/aipu [fail]");
        dump_stack();
        return AIPU_LL_STATUS_ERROR_OPEN_FAIL;
    }

    kret = ioctl(m_fd, AIPU_IOCTL_QUERY_CAP, &cap);
    if (kret || (0 == cap.partition_cnt))
    {
        LOG(LOG_ERR, "query capability [fail]");
        ret = AIPU_LL_STATUS_ERROR_IOCTL_QUERY_CAP_FAIL;
        goto fail;
    }

    part_caps = new aipu_partition_cap[cap.partition_cnt];
    kret = ioctl(m_fd, AIPU_IOCTL_QUERY_PARTITION_CAP, part_caps);
    if (kret)
    {
        LOG(LOG_ERR, "query partition [fail]");
        delete[] part_caps;
        ret = AIPU_LL_STATUS_ERROR_IOCTL_QUERY_CORE_CAP_FAIL;
        goto fail;
    }

    for (uint32_t i = 0; i < cap.partition_cnt; i++)
        m_part_caps.push_back(part_caps[i]);

    delete[] part_caps;
    m_dram = UKMemory::get_memory(m_fd);
    m_dram->set_asid_base(0, cap.asid0_base);
    m_dram->set_asid_base(1, cap.asid1_base);
    m_dram->set_asid_base(2, cap.asid2_base);
    m_dram->set_asid_base(3, cap.asid3_base);
    if (m_part_caps.at(0).version == AIPU_ISA_VERSION_ZHOUYI_V1)
    {
        cap.asid0_base = 0;
        cap.asid1_base = 0;
        cap.asid2_base = 0;
        cap.asid3_base = 0;
        m_dram->set_asid_base(0, cap.asid0_base);
        m_dram->set_asid_base(1, cap.asid1_base);
        m_dram->set_asid_base(2, cap.asid2_base);
        m_dram->set_asid_base(3, cap.asid3_base);
    }
    LOG(LOG_DEBUG, "asid0: 0x%llx, asid1: 0x%llx, asid2: 0x%llx, asid3: 0x%llx\n",
        cap.asid0_base, cap.asid1_base, cap.asid2_base, cap.asid3_base);

    if (m_part_caps.at(0).version == AIPU_ISA_VERSION_ZHOUYI_V2_2)
        m_dram->set_dtcm_info(cap.dtcm_base, cap.dtcm_size);

    if (m_part_caps.at(0).version == AIPU_ISA_VERSION_ZHOUYI_V3)
    {
        if (cap.gm0_size == 0)
            return AIPU_LL_STATUS_ERROR_IOCTL_QUERY_STATUS_FAIL;

        if (m_dram->is_gm_enable())
        {
            m_dram->set_gm_size(0, cap.gm0_size);
            m_dram->set_gm_size(1, cap.gm1_size);
        }
    }

    /* success */
    if ((m_part_caps.at(0).version >= AIPU_ISA_VERSION_ZHOUYI_V1)
        && (m_part_caps.at(0).version <= AIPU_ISA_VERSION_ZHOUYI_V2_2))
    {
        m_partition_cnt = 0;
        m_cluster_cnt = 0;

        /* indicate core count for aipu v1/v2 */
        m_core_cnt = cap.partition_cnt;
    } else {
        m_partition_cnt = cap.partition_cnt;

        /* defaut get the below count from cluster0 in partition0 */
        m_cluster_cnt = m_part_caps.at(0).cluster_cnt;
        m_core_cnt = m_part_caps.at(0).clusters[0].core_cnt;
    }

    return ret;
fail:
    close(m_fd);
    return ret;
}

void aipudrv::Aipu::deinit()
{
    if (m_dram != nullptr)
        m_dram = nullptr;

    if (m_fd > 0)
    {
        ioctl_cmd(AIPU_IOCTL_DISABLE_TICK_COUNTER, nullptr);
        close(m_fd);
        m_fd = 0;
    }
}

bool aipudrv::Aipu::has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev)
{
    for (uint32_t i = 0; i < m_part_caps.size(); i++)
    {
        if ((arch == m_part_caps[i].arch) &&
            (version == m_part_caps[i].version) &&
            ((version == AIPU_ISA_VERSION_ZHOUYI_V3) || (config == m_part_caps[i].config)))
            return true;
    }

    return false;
}

aipu_ll_status_t aipudrv::Aipu::read_reg(uint32_t core_id, uint32_t offset, uint32_t* value)
{
    int kret = 0;
    aipu_io_req ioreq;

    if (nullptr == value)
        return AIPU_LL_STATUS_ERROR_NULL_PTR;

    ioreq.core_id = core_id;
    ioreq.rw = aipu_io_req::AIPU_IO_READ;
    ioreq.offset = offset;
    kret = ioctl(m_fd, AIPU_IOCTL_REQ_IO, &ioreq);
    if (kret)
    {
        LOG(LOG_ERR, "request register read [fail]");
        return AIPU_LL_STATUS_ERROR_IOCTL_REQ_IO_FAIL;
    }

    /* success */
    *value = ioreq.value;
    return AIPU_LL_STATUS_SUCCESS;
}

aipu_ll_status_t aipudrv::Aipu::write_reg(uint32_t core_id, uint32_t offset, uint32_t value)
{
    int kret = 0;
    aipu_io_req ioreq;

    ioreq.core_id = core_id;
    ioreq.rw = aipu_io_req::AIPU_IO_WRITE;
    ioreq.offset = offset;
    ioreq.value = value;
    kret = ioctl(m_fd, AIPU_IOCTL_REQ_IO, &ioreq);
    if (kret)
    {
        LOG(LOG_ERR, "request register write [fail]");
        return AIPU_LL_STATUS_ERROR_IOCTL_REQ_IO_FAIL;
    }

    return AIPU_LL_STATUS_SUCCESS;
}

aipu_status_t aipudrv::Aipu::schedule(const JobDesc& job)
{
    int kret = 0;

    kret = ioctl(m_fd, AIPU_IOCTL_SCHEDULE_JOB, &job.kdesc);
    if (kret)
    {
        LOG(LOG_ERR, "schedule job [fail]");
        return AIPU_STATUS_ERROR_INVALID_OP;
    }

    return AIPU_STATUS_SUCCESS;
}

aipu_ll_status_t aipudrv::Aipu::get_status(std::vector<aipu_job_status_desc>& jobs_status,
    uint32_t max_cnt, bool of_this_thread, void *jobbase)
{
    aipu_ll_status_t ret = AIPU_LL_STATUS_SUCCESS;
    int kret = 0;
    aipu_job_status_query status_query;
    JobBase *job = (JobBase *)jobbase;
    JobBase *done_job = nullptr;
    callback_wrapper_t *cb_wrap = nullptr;

    status_query.of_this_thread = of_this_thread;
    status_query.max_cnt = max_cnt;
    status_query.status = new aipu_job_status_desc[max_cnt];
    kret = ioctl(m_fd, AIPU_IOCTL_QUERY_STATUS, &status_query);
    if (kret)
    {
        LOG(LOG_ERR, "query job status [fail]");
        ret = AIPU_LL_STATUS_ERROR_IOCTL_QUERY_STATUS_FAIL;
        goto clean;
    }

    for (uint32_t i = 0; i < status_query.poll_cnt; i++)
    {
        m_job_sts_queue.push_q(status_query.status[i]);

        done_job = job->get_base_graph().get_job(status_query.status[i].job_id);
        if (done_job != nullptr)
        {
            cb_wrap = done_job->get_job_cb();
            /* deliver done job to backend timely */
            if (cb_wrap != nullptr && cb_wrap->cb_func != nullptr
                && cb_wrap->cb_args != nullptr)
            {
                cb_wrap->cb_args->job_id = status_query.status[i].job_id;
                cb_wrap->cb_args->job_state = (aipu_job_status_t)status_query.status[i].state;
                cb_wrap->cb_func(cb_wrap->cb_args);
            }
        }
    }

clean:
    delete[] status_query.status;
    return ret;
}

aipu_ll_status_t aipudrv::Aipu::get_status(std::vector<aipu_job_status_desc>& jobs_status,
    uint32_t max_cnt, void *jobbase)
{
    return poll_status(jobs_status, max_cnt, 0, true, jobbase);
}

aipu_ll_status_t aipudrv::Aipu::poll_status(std::vector<aipu_job_status_desc>& jobs_status,
    uint32_t max_cnt, int32_t time_out, bool of_this_thread, void *jobbase)
{
    aipu_ll_status_t ret = AIPU_LL_STATUS_SUCCESS;
    int kret = 0;
    struct pollfd poll_list;
    JobBase *job = (JobBase *)jobbase;

    /**
     * the laterly committed job maybe finished firstly, but the current polling job
     * isn't the laterly committed job, so it has to cache the laterly committed but
     * firstly finished job. so it's necesssary to check the cache queue containing
     * finished job firstly. if there's no target job, switch to poll NPU HW.
     */
    if (m_job_sts_queue.is_job_exist(job->get_id()))
    {
        jobs_status.push_back(*m_job_sts_queue.pop_q(job->get_id()));
        return ret;
    }

    poll_list.fd = m_fd;
    poll_list.events = POLLIN | POLLPRI;

    do
    {
        kret = poll(&poll_list, 1, time_out);
        if (kret < 0)
        {
            LOG(LOG_ERR, "poll /dev/aipu [fail]");
            return AIPU_LL_STATUS_ERROR_POLL_FAIL;
        } else if (kret == 0) {
            return AIPU_LL_STATUS_ERROR_POLL_TIMEOUT;
        }

        /* normally return */
        if ((poll_list.revents & POLLIN) == POLLIN)
            ret = get_status(jobs_status, max_cnt, of_this_thread, jobbase);

        if (m_job_sts_queue.is_job_exist(job->get_id()))
        {
            jobs_status.push_back(*m_job_sts_queue.pop_q(job->get_id()));
            return ret;
        }
    } while (time_out == -1);

    return ret;
}

aipu_ll_status_t aipudrv::Aipu::ioctl_cmd(uint32_t cmd, void *arg)
{
    aipu_ll_status_t ret = AIPU_LL_STATUS_SUCCESS;
    int kret = 0;

    switch (cmd)
    {
        case AIPU_IOCTL_ABORT_CMD_POOL:
            kret = ioctl(m_fd, AIPU_IOCTL_ABORT_CMD_POOL);
            if (kret < 0)
            {
                LOG(LOG_ERR, "abort cmdpool [fail]");
                ret = AIPU_LL_STATUS_ERROR_IOCTL_ABORT_CMDPOOL;
            }
            break;

        case AIPU_IOCTL_ENABLE_TICK_COUNTER:
            if (!m_tick_counter)
            {
                kret = ioctl(m_fd, AIPU_IOCTL_ENABLE_TICK_COUNTER);
                if (kret < 0)
                {
                    LOG(LOG_ERR, "enable tick counter [fail]");
                    ret = AIPU_LL_STATUS_ERROR_IOCTL_TICK_COUNTER;
                }
                m_tick_counter = true;
            }
            break;

        case AIPU_IOCTL_DISABLE_TICK_COUNTER:
            if (m_tick_counter)
            {
                kret = ioctl(m_fd, AIPU_IOCTL_DISABLE_TICK_COUNTER);
                if (kret < 0)
                {
                    LOG(LOG_ERR, "disable tick counter [fail]");
                    ret = AIPU_LL_STATUS_ERROR_IOCTL_TICK_COUNTER;
                }
                m_tick_counter = false;
            }
            break;

        case AIPU_IOCTL_CONFIG_CLUSTERS:
            kret = ioctl(m_fd, AIPU_IOCTL_CONFIG_CLUSTERS, arg);
            if (kret < 0)
            {
                LOG(LOG_ERR, "config cluster [fail]");
                ret = AIPU_LL_STATUS_ERROR_CONFIG_CLUSTER;
            }
            break;

        case AIPU_IOCTL_ALLOC_DMABUF:
            kret = ioctl(m_fd, AIPU_IOCTL_ALLOC_DMA_BUF, arg);
            if (kret < 0)
            {
                LOG(LOG_ERR, "alloc dma_buf [fail]");
                ret = AIPU_LL_STATUS_ERROR_IOCTL_FAIL;
            }
            break;

        case AIPU_IOCTL_FREE_DMABUF:
            kret = ioctl(m_fd, AIPU_IOCTL_FREE_DMA_BUF, *(uint64_t *)arg);
            if (kret < 0)
            {
                LOG(LOG_ERR, "free dma_buf [fail]");
                ret = AIPU_LL_STATUS_ERROR_IOCTL_FAIL;
            }
            break;

        case AIPU_IOCTL_GET_DMA_BUF_INFO:
            kret = ioctl(m_fd, AIPU_IOCTL_GET_DMA_BUF_INFO, arg);
            if (kret < 0)
            {
                LOG(LOG_ERR, "get dma_buf [fail]");
                ret = AIPU_LL_STATUS_ERROR_IOCTL_FAIL;
            }
            break;

        default:
            LOG(LOG_ERR, "AIPU can't support cmd: %d\n", cmd);
            ret = AIPU_LL_STATUS_ERROR_OPERATION_UNSUPPORTED;
    }

    return ret;
}