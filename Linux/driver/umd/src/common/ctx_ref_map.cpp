// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  ctx_ref_map.cpp
 * @brief AIPU User Mode Driver (UMD) context reference map module implementation
 */

#include "ctx_ref_map.h"

aipudrv::CtxRefMap::CtxRefMap()
{
    data.clear();
    pthread_mutex_init(&lock, NULL);
}

aipudrv::CtxRefMap::~CtxRefMap()
{
    std::map<uint32_t, MainContext*>::iterator iter;
    pthread_mutex_lock(&lock);
    for (iter = data.begin(); iter != data.end(); iter++)
    {
        iter->second->force_deinit();
        delete iter->second;
        iter->second = nullptr;
    }
    data.clear();
    pthread_mutex_unlock(&lock);
    pthread_mutex_destroy(&lock);
}

uint32_t aipudrv::CtxRefMap::create_ctx_ref()
{
    uint32_t handle = 0xFFFFFFFF;

    pthread_mutex_lock(&lock);
    while(nullptr != get_ctx_ref_inner(handle))
    {
        handle--;
    }
    data[handle] = new MainContext;
    pthread_mutex_unlock(&lock);

    return handle;
}

aipudrv::MainContext* aipudrv::CtxRefMap::get_ctx_ref_inner(uint32_t handle)
{
    MainContext* ctx = nullptr;

    if (data.count(handle) == 1)
        ctx = data[handle];
    return ctx;
}

aipudrv::MainContext* aipudrv::CtxRefMap::get_ctx_ref(uint32_t handle)
{
    MainContext* ctx = nullptr;
    pthread_mutex_lock(&lock);
    ctx = get_ctx_ref_inner(handle);
    pthread_mutex_unlock(&lock);
    return ctx;
}

aipu_status_t aipudrv::CtxRefMap::destroy_ctx_ref(uint32_t handle)
{
    aipu_status_t ret = AIPU_STATUS_SUCCESS;
    MainContext* ctx = nullptr;

    pthread_mutex_lock(&lock);
    ctx = get_ctx_ref_inner(handle);
    if (nullptr != ctx)
    {
        delete ctx;
        data.erase(handle);
    } else {
        ret = AIPU_STATUS_ERROR_INVALID_CTX;
    }
    pthread_mutex_unlock(&lock);

    return ret;
}