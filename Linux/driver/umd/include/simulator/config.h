#ifndef __AIPU_CONFIG_H__
#define __AIPU_CONFIG_H__

#include <stdint.h>
#include <string>
#include <vector>

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

    /** @brief sim_aipu::config_t.

    These parameters are specified then it can construct your expect the aipu.

    @note enable_avx is use the intel AVX instruction to speed up, but it is not yet.
    */
    struct config_t
    {
        enum
        {
            X2_1204 = 1,
            X2_1204MP3,
            X3_1304,
            X3_1304MP2,
            X3_1304MP4,
        };
        int code;                           //!< select one core(1204) or tree core(1204MP3).
        bool enable_avx;                    //!< use Intel avx instruction (not yet).
        bool enable_calloc;                 //!< clean internal resources.
        int64_t max_pkg_num;                //!< run max instruction packet.
        struct
        {
            std::string filepath;           //!< output log path.
            int level;                      //!< log level.
            bool verbose;                   //!< output log to console.
        } log;
        aiff_log_ctl_t aiff_log;            //!< set aiff submodule log.
        aiff_dump_ctl_t aiff_dump;          //!< set aiff submodule dump.
        bool en_eval = 0;                   //!< profiling statistics.
        bool en_probe = 0;                  //!< function unit coverage rate.
        bool en_fast_perf = 0;              //!< fast evaluation of perf count.
        bool en_l2d = 0;                    //!< fast l2d cache for x3.
        uint32_t gm_size;                   //!< set gm size in bytes.
        std::string plugin_filename;        //!< plugin dynamic library filename.
        std::string json_filename;          //!< json filename.
        uint32_t freq_mhz = 1000;           //!< fast evaluation: frequency setting.
        uint32_t ddr_latency_rd = 0;      //!< fast evaluation: ddr latency read setting
        uint32_t ddr_latency_wr = 0;       //!< fast evaluation: ddr latency write setting
        uint32_t ddr_bw = 256;               //!< fast evaluation: ddr bandwidth setting, indicate how may bits of the axi data bus
        float ddr_bw_ratio = 1.0;            //!< fast evaluation: ddr bandwidth ratio setting
        std::string perf_report;            //!< fast evaluation: performance report
        std::vector<uint8_t> isa_events; // isa events to be profiled, default is None.
    };

    enum { AIPU_EV_GRID_END, };
    using event_handler_t = void (*)(uint32_t event, uint64_t value, void *context);
} //!sim_aipu

#endif //!__AIPU_CONFIG_H__
