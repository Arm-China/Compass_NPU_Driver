// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  ctx_ref_map.h
 * @brief AIPU User Mode Driver (UMD) context reference map module header
 */

#ifndef _CTX_REF_MAP_H_
#define _CTX_REF_MAP_H_

#include <map>
#include <pthread.h>
#include "standard_api.h"
#include "context.h"

namespace aipudrv
{
class CtxRefMap
{
private:
    std::map<uint32_t, MainContext*> data;
    pthread_mutex_t lock;

private:
    MainContext*  get_ctx_ref_inner(uint32_t handle);

public:
    uint32_t      create_ctx_ref();
    MainContext*  get_ctx_ref(uint32_t handle);
    aipu_status_t destroy_ctx_ref(uint32_t handle);

public:
    static CtxRefMap& get_ctx_map()
    {
        static CtxRefMap ctxmap;
        return ctxmap;
    }
    CtxRefMap(const CtxRefMap& ctx) = delete;
    CtxRefMap& operator=(const CtxRefMap& ctx) = delete;
    ~CtxRefMap();

private:
    CtxRefMap();
};
}

#endif /* _CTX_REF_MAP_H_ */