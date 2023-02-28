// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  device_base.h
 * @brief AIPU User Mode Driver (UMD) device module header
 */

#ifndef _DEVICE_BASE_H_
#define _DEVICE_BASE_H_

#include <atomic>
#include "kmd/armchina_aipu.h"
#include "memory_base.h"
#include "type.h"

typedef enum {
    AIPU_LL_STATUS_SUCCESS,
    AIPU_LL_STATUS_ERROR_OPERATION_UNSUPPORTED,
    AIPU_LL_STATUS_ERROR_NULL_PTR,
    AIPU_LL_STATUS_ERROR_OPEN_FAIL,
    AIPU_LL_STATUS_ERROR_IOCTL_QUERY_CAP_FAIL,
    AIPU_LL_STATUS_ERROR_IOCTL_QUERY_CORE_CAP_FAIL,
    AIPU_LL_STATUS_ERROR_IOCTL_REQ_IO_FAIL,
    AIPU_LL_STATUS_ERROR_POLL_FAIL,
    AIPU_LL_STATUS_ERROR_POLL_TIMEOUT,
    AIPU_LL_STATUS_ERROR_IOCTL_QUERY_STATUS_FAIL,
    AIPU_LL_STATUS_ERROR_IOCTL_ABORT_CMDPOOL,
    AIPU_LL_STATUS_ERROR_IOCTL_TICK_COUNTER,
} aipu_ll_status_t;

inline aipu_status_t convert_ll_status(aipu_ll_status_t status)
{
    switch(status)
    {
        case AIPU_LL_STATUS_SUCCESS:
            return AIPU_STATUS_SUCCESS;

        case AIPU_LL_STATUS_ERROR_OPERATION_UNSUPPORTED:
            return AIPU_STATUS_ERROR_OP_NOT_SUPPORTED;

        case AIPU_LL_STATUS_ERROR_NULL_PTR:
            return AIPU_STATUS_ERROR_NULL_PTR;

        case AIPU_LL_STATUS_ERROR_OPEN_FAIL:
            return AIPU_STATUS_ERROR_OPEN_DEV_FAIL;

        case AIPU_LL_STATUS_ERROR_POLL_TIMEOUT:
            return AIPU_STATUS_ERROR_TIMEOUT;

        case AIPU_LL_STATUS_ERROR_IOCTL_QUERY_CAP_FAIL:
        case AIPU_LL_STATUS_ERROR_IOCTL_QUERY_CORE_CAP_FAIL:
        case AIPU_LL_STATUS_ERROR_IOCTL_REQ_IO_FAIL:
        case AIPU_LL_STATUS_ERROR_POLL_FAIL:
        case AIPU_LL_STATUS_ERROR_IOCTL_QUERY_STATUS_FAIL:
        case AIPU_LL_STATUS_ERROR_IOCTL_ABORT_CMDPOOL:
        case AIPU_LL_STATUS_ERROR_IOCTL_TICK_COUNTER:
            return AIPU_STATUS_ERROR_DEV_ABNORMAL;

        default:
            return AIPU_STATUS_SUCCESS;
    }
}

namespace aipudrv
{
struct JobDesc
{
    /* job descriptor for KMD, part of the members shared with x86 simulation */
    struct aipu_job_desc kdesc;

    /* shared */
    uint32_t aipu_revision;
    bool dump_reuse;

    /* x2 only */
    DEV_PA_64 tcb_head;
    DEV_PA_64 tcb_tail;

    /* x2 simulation */
    void *jobbase;

    /* z1/2/3 only */
    DEV_PA_64 instruction_base_pa;

    /* z1/2/3 simulation only */
    uint32_t text_size;
    DEV_PA_64 weight_pa;
    uint32_t weight_size;
    uint32_t rodata_size;
    DEV_PA_64 dcr_pa;
    uint32_t dcr_size;
    uint32_t stack_size;
    std::vector<BufferDesc> reuses;
    std::vector<BufferDesc> weights;
    std::vector<BufferDesc> outputs;
    std::vector<BufferDesc> profile;
    std::map<std::string, BufferDesc> misc_outputs;
    std::string output_dir;
    std::string simulator;
    std::string log_path;
    uint32_t log_level;
    bool en_eval;
};

enum DeviceType
{
    DEV_TYPE_NONE             = 0,
    DEV_TYPE_SIMULATOR_LEGACY = 1,
    DEV_TYPE_SIMULATOR_V3     = 2,
    DEV_TYPE_AIPU             = 3,
};

class DeviceBase
{
protected:
    std::vector<aipu_partition_cap> m_part_caps;
    DeviceType  m_dev_type = DEV_TYPE_NONE;
    MemoryBase* m_dram = nullptr;
    uint32_t m_partition_cnt = 0;
    uint32_t m_cluster_cnt = 0;
    uint32_t m_core_cnt = 1;
    std::atomic_int m_ref_cnt{0};

public:
    virtual bool has_target(uint32_t arch, uint32_t version, uint32_t config, uint32_t rev) = 0;
    virtual aipu_ll_status_t ioctl_cmd(uint32_t cmd, void *arg)
    {
        return AIPU_LL_STATUS_SUCCESS;
    };
    virtual aipu_status_t get_cluster_id(uint32_t part_id, std::vector<uint32_t> &)
    {
        return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;
    }
    virtual uint32_t tec_cnt_per_core(uint32_t partition_idx)
    {
        if ((partition_idx >= 0) && (partition_idx < m_partition_cnt))
            return m_part_caps.at(partition_idx).clusters[0].tec_cnt;
        else
            return 0;
    }
    virtual aipu_status_t schedule(const JobDesc& job) = 0;
    virtual aipu_status_t get_simulation_instance(void** simulator, void** memory)
    {
        *simulator = nullptr;
        *memory = nullptr;
        return AIPU_STATUS_SUCCESS;
    }
    MemoryBase* get_mem()
    {
        return m_dram;
    }
    virtual aipu_ll_status_t read_reg(uint32_t core_id, uint32_t offset, uint32_t* value)
    {
        return AIPU_LL_STATUS_ERROR_OPERATION_UNSUPPORTED;
    }
    virtual aipu_ll_status_t write_reg(uint32_t core_id, uint32_t offset, uint32_t value)
    {
        return AIPU_LL_STATUS_ERROR_OPERATION_UNSUPPORTED;
    }
    /* non-blocking */
    virtual aipu_ll_status_t get_status(std::vector<aipu_job_status_desc>& jobs_status,
        uint32_t max_cnt, void *jobbase = nullptr)
    {
        return AIPU_LL_STATUS_SUCCESS;
    }
    /* blocking with timeout */
    virtual aipu_ll_status_t poll_status(std::vector<aipu_job_status_desc>& jobs_status,
        uint32_t max_cnt, int32_t time_out, bool of_this_thread, void *jobbase = nullptr)
    {
        return AIPU_LL_STATUS_SUCCESS;
    }
    int dec_ref_cnt()
    {
        return --m_ref_cnt;
    }
    int inc_ref_cnt()
    {
        return ++m_ref_cnt;
    }
    virtual int get_config_code()
    {
        return 0;
    }

public:
    aipu_status_t get_partition_count(uint32_t* cnt)
    {
        if (cnt == nullptr)
            return AIPU_STATUS_ERROR_NULL_PTR;

        if (m_partition_cnt >= 0)
        {
            *cnt = m_partition_cnt;
            return AIPU_STATUS_SUCCESS;
        }
        return AIPU_STATUS_ERROR_INVALID_OP;
    }

    aipu_status_t get_cluster_count(uint32_t partition_id, uint32_t* cnt)
    {
        if (cnt == nullptr)
            return AIPU_STATUS_ERROR_NULL_PTR;

        if (partition_id == 0)
        {
            *cnt = m_cluster_cnt;
            return AIPU_STATUS_SUCCESS;
        } else if (partition_id < m_partition_cnt) {
            if (m_part_caps.at(partition_id).cluster_cnt > 0)
            {
                *cnt = m_part_caps.at(partition_id).cluster_cnt;
                return AIPU_STATUS_SUCCESS;
            } else {
                return AIPU_STATUS_ERROR_INVALID_OP;
            }
        }

        return AIPU_STATUS_ERROR_INVALID_OP;
    }

    aipu_status_t get_next_cluster_id(uint32_t partition_id, uint32_t &next_id)
    {
        static uint32_t next_cluster_in_part[4] = {0};

        if (partition_id < m_partition_cnt)
        {
            if (m_part_caps.at(partition_id).cluster_cnt > 0)
            {
                next_id = next_cluster_in_part[partition_id]++;
                if (next_cluster_in_part[partition_id] >= m_part_caps.at(partition_id).cluster_cnt)
                    next_cluster_in_part[partition_id] = 0;
                return AIPU_STATUS_SUCCESS;
            } else {
                return AIPU_STATUS_ERROR_INVALID_OP;
            }
        }

        return AIPU_STATUS_ERROR_INVALID_OP;
    }

    /**
     * currently implement the logic according to all clusters in one partition
     * are the homegeneous ARCH. the core count is same in separate clusters.
     */
    aipu_status_t get_core_count(uint32_t partition_id, uint32_t cluster_id, uint32_t* cnt)
    {
        if (cnt == nullptr)
            return AIPU_STATUS_ERROR_NULL_PTR;

        *cnt = 0;
        if (partition_id > m_partition_cnt)
            return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;

        if ((partition_id == 0) && (cluster_id == 0))
        {
            *cnt = m_core_cnt;
        } else {
            #if SIMULATION
            if (cluster_id > m_cluster_cnt)
                return AIPU_STATUS_ERROR_INVALID_CLUSTER_ID;
            #else
            if (cluster_id > m_part_caps.at(partition_id).cluster_cnt)
                return AIPU_STATUS_ERROR_INVALID_CLUSTER_ID;
            #endif
            *cnt = m_part_caps.at(partition_id).clusters[cluster_id].core_cnt;
        }

        if (*cnt <= 0)
            return AIPU_STATUS_ERROR_INVALID_OP;
        else
            return AIPU_STATUS_SUCCESS;
    }

    aipu_status_t get_core_info(uint32_t id, aipu_core_info_t* info)
    {
        if (info != nullptr)
        {
            uint32_t cnt = 0;

            if (m_part_caps[0].version == AIPU_ISA_VERSION_ZHOUYI_X2)
                cnt = m_partition_cnt;
            else
                cnt = m_core_cnt;

            if (id >= 0 && id < cnt)
            {
                info->reg_base = m_part_caps[id].info.reg_base;
                return AIPU_STATUS_SUCCESS;
            }

            return AIPU_STATUS_ERROR_INVALID_OP;
        }
        return AIPU_STATUS_ERROR_NULL_PTR;
    }

    DeviceType get_dev_type()
    {
        return m_dev_type;
    }

    uint32_t get_npu_version()
    {
        return m_part_caps[0].version;
    }

    uint32_t get_npu_config()
    {
        return m_part_caps[0].config;
    }

public:
    DeviceBase(){};
    virtual ~DeviceBase(){};
    DeviceBase(const DeviceBase& dev) = delete;
    DeviceBase& operator=(const DeviceBase& dev) = delete;
};
}

#endif /* _DEVICE_BASE_H_ */
