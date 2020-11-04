// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


#ifndef __AIPU_H__
#define __AIPU_H__

#include <stdint.h>
#include <memory>

#define TSM_CMD_SCHED_ADDR_HI    0x8
#define TSM_CMD_SCHED_ADDR_LO    0xC
#define TSM_CMD_SCHED_CTRL       0x0
#define CREATE_CMD_POOL          0x1
#define DESTROY_CMD_POOL         0x2
#define DISPATCH_CMD_POOL        0x4
#define CMD_POOL0_STATUS         0x804
#define CLUSTER0_CONFIG          0xC00
#define CLUSTER0_CTRL            0xC04
#define CMD_POOL0_IDLE           (1 << 6)

namespace sim_aipu
{
    class IMemEngine;
    class IDbgLite;

    class Aipu
    {
    public:
        Aipu(const struct config_t &, IMemEngine &);
        ~Aipu();

        Aipu(const Aipu &) = delete;
        Aipu &operator=(const Aipu &) = delete;

        int read_register(uint32_t addr, uint32_t &v) const;

        int write_register(uint32_t addr, uint32_t v);

        static int version();

        void set_dbg_lite(const std::shared_ptr<IDbgLite> &);

    private:
        std::unique_ptr<struct AipuImpl> impl_;
    };
} //!sim_aipu

#endif //!__AIPU_H__
