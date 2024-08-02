// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0


#ifndef __MEM_ENGINE_BASE_H__
#define __MEM_ENGINE_BASE_H__

#include <stddef.h>
#include <stdint.h>
#include <memory>

namespace sim_aipu
{
    class IMemEngine;

    std::unique_ptr<IMemEngine> make_mem_engine(bool enable_calloc = false);
    std::unique_ptr<IMemEngine> make_mem_engine(uint64_t msize, uint32_t psize, bool enable_calloc);

    class IMemEngine
    {
    public:
        virtual ~IMemEngine() = default;

        virtual int64_t read(uint64_t addr, void *dest, size_t size) const = 0;

        virtual int64_t write(uint64_t addr, const void *src, size_t size) = 0;

        virtual int64_t zeroize(uint64_t addr, size_t size) = 0;

        virtual size_t size() const = 0;

        virtual bool invalid(uint64_t addr) const = 0;

        virtual bool get_info(uint64_t addr, uint64_t &base, size_t &size) const
        {
            (void) addr;
            base = 0;
            size = this->size();
            return true;
        }
    };
} // namespace sim_aipu
#endif // !__MEM_ENGINE_BASE_H__
