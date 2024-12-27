// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  internal_api.h
 * @brief Zhouyi AIPU User Mode Driver (UMD) Internal API header (for aipu v1/v2/v3)
 * @version 1.0
 */

#ifndef _INTERNAL_API_H_
#define _INTERNAL_API_H_

#include "../standard_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief This API is used to load output tensor data
 *
 * @param[in] ctx    Pointer to a context handle struct returned by aipu_init_context
 * @param[in] job    Job ID returned by aipu_create_job
 * @param[in] tensor Output tensor ID
 * @param[in] data   Data of output tensor
 *
 * @retval AIPU_STATUS_SUCCESS
 * @retval AIPU_STATUS_ERROR_NULL_PTR
 * @retval AIPU_STATUS_ERROR_INVALID_CTX
 * @retval AIPU_STATUS_ERROR_INVALID_JOB_ID
 * @retval AIPU_STATUS_ERROR_INVALID_TENSOR_ID
 * @retval AIPU_STATUS_ERROR_INVALID_OP
 */
aipu_status_t aipu_load_output_tensor(const aipu_ctx_handle_t* ctx, uint64_t job, uint32_t tensor, const void* data);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _INTERNAL_API_H_ */
