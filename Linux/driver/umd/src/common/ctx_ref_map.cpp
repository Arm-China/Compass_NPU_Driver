// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  ctx_ref_map.cpp
 * @brief AIPU User Mode Driver (UMD) context reference map module
 * implementation
 */

#include "ctx_ref_map.h"

namespace aipudrv {
CtxRefMap::CtxRefMap() {
  m_data.clear();
  pthread_mutex_init(&m_lock, NULL);
}

CtxRefMap::~CtxRefMap() {
  std::map<uint32_t, MainContext *>::iterator iter;
  pthread_mutex_lock(&m_lock);
  for (iter = m_data.begin(); iter != m_data.end(); iter++) {
    iter->second->force_deinit();
    delete iter->second;
    iter->second = nullptr;
  }
  m_data.clear();
  pthread_mutex_unlock(&m_lock);
  pthread_mutex_destroy(&m_lock);
}

uint32_t CtxRefMap::create_ctx_ref() {
  uint32_t handle = 0xFFFFFFFF;

  pthread_mutex_lock(&m_lock);
  while (nullptr != get_ctx_ref_inner(handle)) {
    handle--;
  }
  m_data[handle] = new MainContext;
  pthread_mutex_unlock(&m_lock);

  return handle;
}

MainContext *CtxRefMap::get_ctx_ref_inner(uint32_t handle) {
  MainContext *ctx = nullptr;

  if (m_data.count(handle) == 1)
    ctx = m_data[handle];
  return ctx;
}

MainContext *CtxRefMap::get_ctx_ref(uint32_t handle) {
  MainContext *ctx = nullptr;
  pthread_mutex_lock(&m_lock);
  ctx = get_ctx_ref_inner(handle);
  pthread_mutex_unlock(&m_lock);
  return ctx;
}

aipu_status_t CtxRefMap::destroy_ctx_ref(uint32_t handle) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  MainContext *ctx = nullptr;

  pthread_mutex_lock(&m_lock);
  ctx = get_ctx_ref_inner(handle);
  if (ctx != nullptr) {
    delete ctx;
    ctx = nullptr;
    m_data.erase(handle);
  } else {
    ret = AIPU_STATUS_ERROR_INVALID_CTX;
  }
  pthread_mutex_unlock(&m_lock);

  return ret;
}
} // namespace aipudrv