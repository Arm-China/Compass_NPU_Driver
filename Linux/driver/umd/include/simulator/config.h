// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


#ifndef __AIPU_CONFIG_H__
#define __AIPU_CONFIG_H__

#include <stdint.h>
#include <string>

namespace sim_aipu
{
    struct aiff_log_ctl_t
    {
        bool is_valid = 0;
        uint8_t mtp_iact_ld = 0;
        uint8_t mtp_wgt_ld = 0;
        uint8_t mtp = 0;
        uint8_t itp_iact_ld = 0;
        uint8_t itp_params_ld = 0;
        uint8_t itp = 0;
        uint8_t wrb = 0;
        uint8_t ptp_iact_ld = 0;
        uint8_t ptp_wgt_ld = 0;
        uint8_t ptp_params_ld = 0;
        uint8_t ptp = 0;
    };
    struct aiff_dump_ctl_t
    {
        bool is_valid = 0;
        uint8_t dump_fmt = 0;
        uint8_t mtp_iact_ld = 0;
        uint8_t mtp_wgt_ld = 0;
        uint8_t mtp = 0;
        uint8_t itp_iact_ld = 0;
        uint8_t itp_params_ld = 0;
        uint8_t itp = 0;
        uint8_t wrb = 0;
        uint8_t ptp_iact_ld = 0;
        uint8_t ptp_wgt_ld = 0;
        uint8_t ptp_params_ld = 0;
        uint8_t ptp = 0;
    };
    struct config_t
    {
        enum
        {
            X2_1204 = 1,
            X2_1204MP3,
            X3_1304,
            X3_1304MP4,
        };
        int code;
        bool enable_avx;
        bool enable_calloc;
        int64_t max_pkg_num;
        struct
        {
            std::string filepath;
            int level;
            bool verbose;
        } log;
        aiff_log_ctl_t aiff_log;
        aiff_dump_ctl_t aiff_dump;
        bool en_eval = 0;
        bool en_probe = 0;
        bool en_fast_perf = 0;
        uint32_t gm_size;
        std::string plugin_filename;
        std::string json_filename;
    };
} //!sim_aipu

#endif //!__AIPU_CONFIG_H__
