// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


#ifndef _HELPER_H_
#define _HELPER_H_

#include <iostream>
#include <vector>
#include <memory>
#include "standard_api.h"

int dump_file_helper(const char* fname, void* src, unsigned int size);
int load_file_helper(const char* fname, char** dest, uint32_t* size);
int unload_file_helper(char* data);
int check_result_helper(const std::vector<char*>& outputs,
    const std::vector<aipu_tensor_desc_t>& descs, char* gt, uint32_t gt_size);
int check_result(std::vector< std::shared_ptr<char> >outputs,
    const std::vector<aipu_tensor_desc_t>& descs, char* gt, uint32_t gt_size);

#endif /* _HELPER_H_ */