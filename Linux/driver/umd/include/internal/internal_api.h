// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  internal_api.h
 * @brief Zhouyi AIPU User Mode Driver (UMD) Internal API header (for aipu
 * v1/v2/v3)
 * @version 1.0
 */

#ifndef _INTERNAL_API_H_
#define _INTERNAL_API_H_

#include "../standard_api.h"

typedef enum {
  AIPU_MEMCPY_HOST_TO_DEVICE = 0,
  AIPU_MEMCPY_DEVICE_TO_HOST,
  AIPU_MEMCPY_DEVICE_TO_DEVICE,
} aipu_memcpy_kind_t;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief This API is used to load output tensor data
 *
 * @param[in] ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job    Job ID returned by aipu_create_job
 * @param[in] tensor Output tensor ID
 * @param[in] data   Data of output tensor
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_load_output_tensor(const aipu_ctx_handle_t *ctx,
                                      uint64_t job, uint32_t tensor,
                                      const void *data);

/**
 * @brief This API is used to write a value to a specific offset of RO params of
 * a Job
 *
 * @param[in] ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] job_id Job ID returned by aipu_create_job
 * @param[in] offset Offset of rodata base where the value will be written
 * @param[in] value Value to be written to the specified offset
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_rewrite_rodata(const aipu_ctx_handle_t *ctx, uint64_t job_id,
                                  uint32_t offset, uint32_t value);

/**
 * @brief This API is used to allocate memory of AIPU device
 *
 * @param[in]  ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  size   Requested allocation size in bytes
 * @param[in]  align  The alignment of the memory in AIPU page size
 * @param[in]  asid   The ASID where the memory is allocated in
 * @param[out] handle Pointer to allocated device memory
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_malloc(const aipu_ctx_handle_t *ctx, uint32_t size,
                          uint32_t align, uint32_t asid, void **handle);

/**
 * @brief This API is used to free memory of AIPU device
 *
 * @param[in] ctx    Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] handle Pointer to allocated device memory
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_free(const aipu_ctx_handle_t *ctx, void **handle);

/**
 * @brief This API is used to copy data between host and AIPU device
 *
 * @param[in]  ctx  Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  dst  Destination memory address
 * @param[in]  src  Source memory address
 * @param[in]  size Size in bytes to copy
 * @param[in]  kind Type of transfer
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_memcpy(const aipu_ctx_handle_t *ctx, void *dst,
                          const void *src, uint32_t size,
                          aipu_memcpy_kind_t kind);

/**
 * @brief This API is used to get the virtual data pointer in BufferDesc
 *
 * @param[in]  ctx          Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in]  buffer_desc  The buffer descriptor which need to get its virtual
 * data pointer
 * @param[in]  data_ptr     Pointer to the returned data pointer
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_va(const aipu_ctx_handle_t *ctx, const void *buffer_desc,
                          char **data_ptr);

/**
 * @brief This API is used to get the size of the note section of an ELF file
 * for Jobv3x
 *
 * @param[in] ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] gid Graph ID
 * @param[in] section_name Name of the section to read, the name is end '\0'
 * @param[out] size Buffer to store the size of the section data, the data
 * buffer should be allocated by caller
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_get_elf_note_size(const aipu_ctx_handle_t *ctx, uint64_t gid,
                                     const char *section_name, uint64_t *size);

/**
 * @brief This API is used to read the note section of an ELF file for Jobv3x
 *
 * @param[in] ctx Pointer to a context handle struct returned by
 * aipu_init_context
 * @param[in] gid Graph ID
 * @param[in] section_name Name of the section to read, the name is end '\0'
 * @param[out] data Buffer to store the section data, the data buffer should be
 * allocated by caller
 * @param[out] size Pointer to store the size of the section data read, if size
 * is nullptr, no operation will be performed on size.
 *
 * @retval AIPU_STATUS_SUCCESS if successful
 */
aipu_status_t aipu_read_elf_note(const aipu_ctx_handle_t *ctx, uint64_t gid,
                                 const char *section_name, char *data,
                                 uint64_t *size);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _INTERNAL_API_H_ */
