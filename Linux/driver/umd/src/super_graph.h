// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  super_base.h
 * @brief AIPU User Mode Driver (UMD) graph base module header
 */

#ifndef _SUPER_GRAPH_H_
#define _SUPER_GRAPH_H_

#include <fstream>
#include "standard_api.h"
#include "graph_base.h"

namespace aipudrv
{
class SuperGraph: public GraphBase
{
    /* To be implemented! */
public:
    virtual void print_parse_info(){};
    virtual aipu_status_t load(std::ifstream& gbin, uint32_t size, bool ver_check = true){return AIPU_STATUS_SUCCESS;};
    virtual aipu_status_t unload(){return AIPU_STATUS_SUCCESS;};
    virtual aipu_status_t create_job(JOB_ID* id, const aipu_global_config_simulation_t* cfg){return AIPU_STATUS_SUCCESS;};
    virtual aipu_status_t destroy_job(JOB_ID id){return AIPU_STATUS_SUCCESS;};
    virtual aipu_status_t get_tensor_count(aipu_tensor_type_t type, uint32_t* cnt){return AIPU_STATUS_SUCCESS;};
    virtual aipu_status_t get_tensor_descriptor(aipu_tensor_type_t type,
        uint32_t tensor, aipu_tensor_desc_t* desc){return AIPU_STATUS_SUCCESS;};
    virtual JobBase* get_job_object(JOB_ID id){return nullptr;};

public:
    SuperGraph(void* ctx, GRAPH_ID id, DeviceBase* dev): GraphBase(ctx, id, dev){};
    virtual ~SuperGraph(){};
    SuperGraph(const SuperGraph& base) = delete;
    SuperGraph& operator=(const SuperGraph& base) = delete;
};
}

#endif /* _SUPER_GRAPH_H_ */
