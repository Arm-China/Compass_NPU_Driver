// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  sha256.h
 * @brief UMD secure hash algorithm function header
 */

#ifndef _SHA256_H_
#define _SHA256_H_

#include <stdint.h>

#include <string>
#include <vector>

const std::string sha256_hex(const void *src, size_t n_bytes);
const std::vector<uint8_t> sha256_bytes(const void *src, size_t n_bytes);
void sha256_hex(const void *src, size_t n_bytes, char *dst_hex65);
void sha256_bytes(const void *src, size_t n_bytes, void *dst_bytes32);

#endif /* _SHA256_H_ */