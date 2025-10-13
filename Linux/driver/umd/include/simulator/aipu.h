#ifndef __AIPU_H__
#define __AIPU_H__

#include <stdint.h>
#include <memory>

#include "config.h"

namespace sim_aipu
{
    class IMemEngine;
    class IDbgLite;

    class Aipu
    {
    public:
        /** @brief Consturcts the aipu.

        @param config_t Specifying construction parameters(e.g. config_t{.code} for config single core or multi core).
        @param IMemEngine Is external DDR.
        */
        Aipu(const config_t &, IMemEngine &);
        ~Aipu();

        Aipu(const Aipu &) = delete;
        Aipu &operator=(const Aipu &) = delete;

        /** @brief Reads the specified register value.

        @param addr Given a addr for a register that is a host register address @ref host register definition.
        @param v Is reads the specified register value.

        @return On success, the number of bytes read is returned.
        On error, negative is returned, and strerror(errnum) gets error appropriately.
        */
        int read_register(uint32_t addr, uint32_t &v) const;

        /** @brief Writes the specified register value.

        @param addr Given a addr for a register that is a host register address @ref host register definition.
        @param v Is the value to be written to the specified register.

        @return On success, the number of bytes written is returned.
        On error, negative is returned, and strerror(errnum) gets error appropriately.
        */
        int write_register(uint32_t addr, uint32_t v);

        /** @brief Setup debug module.

        @param IDbgLite @ref IDbgLite definition.
        */
        void set_dbg_lite(const std::shared_ptr<IDbgLite> &);

        /** @brief Gets the aipu version.

        @return Version bumber (e.g. 0x040200 that is version 4.2.0).
        */
        static int version();

        /** @brief The switch for profiling.

        @param en True to enable profiling statistics, otherwise no.
        */
        void enable_profiling(bool en);

        /** @brief Dump profiling report

        @param none
        */
        void dump_profiling();

        void set_event_handler(event_handler_t, void *context);

    private:
        std::unique_ptr<class AipuImpl> impl_;
    };

} //!sim_aipu

#endif //!__AIPU_H__
