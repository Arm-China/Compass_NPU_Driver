// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


#include "context.h"

std::map<uint32_t, std::string> aipudrv::MainContext::umd_status_string =
{
    { AIPU_STATUS_SUCCESS,
        "AIPU UMD API executes successfully." },
    { AIPU_STATUS_ERROR_NULL_PTR,
        "AIPU UMD API input argument(s) contain NULL pointer." },
    { AIPU_STATUS_ERROR_INVALID_CTX,
        "AIPU UMD runtime context is invalid. Please init first." },
    { AIPU_STATUS_ERROR_OPEN_DEV_FAIL,
        "AIPU device open operation failed. Please check kernel module insmod status or hardware status." },
    { AIPU_STATUS_ERROR_DEV_ABNORMAL,
        "AIPU device state is abnormal. Please check kernel module insmod status or hardware status." },
    { AIPU_STATUS_ERROR_DEINIT_FAIL,
        "AIPU UMD runtime context de-initialization failed." },
    { AIPU_STATUS_ERROR_INVALID_CONFIG,
        "AIPU UMD runtime configuration input(s) is/are invalid." },
    { AIPU_STATUS_ERROR_UNKNOWN_BIN,
        "The type of the binary loaded is unknown. It cannot be loaded or executed." },
    { AIPU_STATUS_ERROR_GVERSION_UNSUPPORTED,
        "The version of the loadable graph binary provided is not supported on UMD." },
    { AIPU_STATUS_ERROR_INVALID_GBIN,
        "Loadable graph binary is invalid which contains unrecognized or invalid items." },
    { AIPU_STATUS_ERROR_TARGET_NOT_FOUND,
        "The specified target device is not found at runtime. Please double check the binary target." },
    { AIPU_STATUS_ERROR_INVALID_GRAPH_ID,
        "Graph ID provided is an invalid one which has been unloaded or never existed." },
    { AIPU_STATUS_ERROR_OPEN_FILE_FAIL,
        "UMD fails in opening the specified file." },
    { AIPU_STATUS_ERROR_MAP_FILE_FAIL,
        "UMD fails in maping the specified file." },
    { AIPU_STATUS_ERROR_READ_FILE_FAIL,
        "UMD fails in reading the specified file." },
    { AIPU_STATUS_ERROR_WRITE_FILE_FAIL,
        "UMD fails in writing the specified file." },
    { AIPU_STATUS_ERROR_INVALID_JOB_ID,
        "Job ID provided is an invalid one which has been cleaned or never existed." },
    { AIPU_STATUS_ERROR_JOB_EXCEPTION,
        "The flushed job encounters exception during execution." },
    { AIPU_STATUS_ERROR_JOB_TIMEOUT,
        "The flushed job execution timeout." },
    { AIPU_STATUS_ERROR_OP_NOT_SUPPORTED,
        "This API call here is not supported on this platform." },
    { AIPU_STATUS_ERROR_INVALID_OP,
        "This API call here is invalid." },
    { AIPU_STATUS_ERROR_INVALID_SIZE,
        "Job size provided is an invalid one." },
    { AIPU_STATUS_ERROR_BUF_ALLOC_FAIL,
        "UMD fails in allocating buffers." },
    { AIPU_STATUS_ERROR_BUF_FREE_FAIL,
        "UMD fails in releasing buffers." },
    { AIPU_STATUS_ERROR_INVALID_CORE_ID,
        "The AIPU core ID application provides is invalid and cannot be found in system." },
    { AIPU_STATUS_ERROR_RESERVE_SRAM_FAIL,
        "UMD fails in reserving SRAM as the executable binary requested because there is no SoC SRAM or SRAM is busy." },
    { AIPU_STATUS_ERROR_INVALID_TENSOR_ID,
        "The tensor ID application provides is invalid." },
    { AIPU_STATUS_ERROR_INVALID_CLUSTER_ID,
        "The AIPU cluster ID application provides is invalid and cannot be found in system." },
    { AIPU_STATUS_ERROR_INVALID_PARTITION_ID,
        "The AIPU partition ID application provides is invalid." },
    { AIPU_STATUS_ERROR_PRINTF_FAIL,
        "UMD fails in parsing the printf buffer and print corresponding logs." },
    { AIPU_STATUS_ERROR_INVALID_TENSOR_TYPE,
        "The specified tensor type is invalid for this operation." },
    { AIPU_STATUS_ERROR_INVALID_GM,
        "GM is invalid." },
    { AIPU_STATUS_ERROR_INVALID_SEGMMU,
        "Segment MMU is invalid." },
    { AIPU_STATUS_ERROR_INVALID_QOS,
        "The Specified QoS level is invalid." },
    { AIPU_STATUS_MAX,
        "Status Max value which should not be returned to application." },
    /* AIPU layer library runtime error code */
    { AIPU_STATUS_ERROR_UNKNOWN_ERROR,
        "Unknown runtime error." },
    { AIPU_STATUS_ERROR_KEYBOARD_INTERRUPT,
        "The execution of the program was interrupted by the user." },
    { AIPU_STATUS_ERROR_SYSTEM_ERR,
        "System error." },
    { AIPU_STATUS_ERROR_OUT_OF_SPEC,
        "Inputs or data are out of the specification of Zhouyi NPU hardware." },
    { AIPU_STATUS_ERROR_OUT_OF_AIFF_SPEC,
        "Inputs or data are out of the specification of Zhouyi NPU AIFF hardware." },
    { AIPU_STATUS_ERROR_OUT_OF_TPC_SPEC,
        "Inputs or data are out of the specification of Zhouyi NPU TPC hardware." },
    { AIPU_STATUS_ERROR_OUT_OF_DMA_SPEC,
        "Inputs or data are out of the specification of Zhouyi NPU DMA hardware." },
    { AIPU_STATUS_ERROR_OUT_OF_MEM_RANGE,
        "Access to DDR or SRAM is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_SRAM_RANGE,
        "Access to SRAM (any type) is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_LSRAM0_RANGE,
        "Access to Local SRAM0 is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_LSRAM1_RANGE,
        "Access to Local SRAM1 is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_LSRAM_RANGE,
        "Access to Local SRAM is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_GSRAM0_RANGE,
        "Access to Global SRAM0 is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_GSRAM1_RANGE,
        "Access to Global SRAM1 is out of range." },
    { AIPU_STATUS_ERROR_OUT_OF_GSRAM_RANGE,
        "Access to Global SRAM is out of range." },
    { AIPU_STATUS_ERROR_ARITHMETIC_ERR,
        "Arithmetic error." },
    { AIPU_STATUS_ERROR_FLOAT_POINT_ERR,
        "Float point error." },
    { AIPU_STATUS_ERROR_UNDERFLOW_ERR,
        "Underflow error." },
    { AIPU_STATUS_ERROR_OVERFLOW_ERR,
        "Overflow error." },
    { AIPU_STATUS_ERROR_NOT_A_NUMBER_ERR,
        "Float point NAN error." },
    { AIPU_STATUS_ERROR_INFINITY_ERR,
        "Float point INF error." },
    { AIPU_STATUS_ERROR_STRING_LENGTH_ERR,
    "String length too long." },
    { AIPU_STATUS_ERROR_ZERO_DIVISION_ERR,
        "Zero division error." }
};

const char* aipudrv::MainContext::get_static_msg(aipu_status_t err)
{
    int idx = AIPU_STATUS_MAX;

    if (err <= AIPU_STATUS_MAX)
    {
        return umd_status_string[err].c_str();
    }

    switch(err)
    {
        case AIPU_STATUS_ERROR_UNKNOWN_ERROR:
            idx += 1;
            break;
        case AIPU_STATUS_ERROR_KEYBOARD_INTERRUPT:
            idx += 2;
            break;
        case AIPU_STATUS_ERROR_SYSTEM_ERR:
            idx += 3;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_SPEC:
            idx += 4;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_AIFF_SPEC:
            idx += 5;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_TPC_SPEC:
            idx += 6;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_DMA_SPEC:
            idx += 7;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_MEM_RANGE:
            idx += 8;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_SRAM_RANGE:
            idx += 9;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_LSRAM0_RANGE:
            idx += 10;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_LSRAM1_RANGE:
            idx += 11;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_LSRAM_RANGE:
            idx += 12;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_GSRAM0_RANGE:
            idx += 13;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_GSRAM1_RANGE:
            idx += 14;
            break;
        case AIPU_STATUS_ERROR_OUT_OF_GSRAM_RANGE:
            idx += 15;
            break;
        case AIPU_STATUS_ERROR_ARITHMETIC_ERR:
            idx += 16;
            break;
        case AIPU_STATUS_ERROR_FLOAT_POINT_ERR:
            idx += 17;
            break;
        case AIPU_STATUS_ERROR_UNDERFLOW_ERR:
            idx += 18;
            break;
        case AIPU_STATUS_ERROR_OVERFLOW_ERR:
            idx += 19;
            break;
        case AIPU_STATUS_ERROR_NOT_A_NUMBER_ERR:
            idx += 20;
            break;
        case AIPU_STATUS_ERROR_INFINITY_ERR:
            idx += 21;
            break;
        case AIPU_STATUS_ERROR_STRING_LENGTH_ERR:
            idx += 22;
            break;
        case AIPU_STATUS_ERROR_ZERO_DIVISION_ERR:
            idx += 23;
            break;
        default:
            idx += 1;
            break;
    }

    return umd_status_string[idx].c_str();
}
