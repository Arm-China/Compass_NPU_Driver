/*!!!THIS IS A PURE C FILE, DON'T ADD ANY C++ STUFF HERE.!!!*/
#ifndef __AIPU_CONFIG_H__
#define __AIPU_CONFIG_H__

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

enum {
  X2_1204 = 1,
  X2_1204MP3,
  X3_1304,
  X3_1304MP2,
  X3_1304MP4,
  X3_1302MP2,
  X3_1202,
  X3P_1304,
  X3P_1304MP2,
  X3P_1304MP4,
  X3S_1304,
  X3S_1304MP2,
  X3S_1304MP4,
};

enum {
  PERF_MODE_NONE,  //!< non perf mode
  PERF_MODE_FAST,  //!< fast evaluation
  PERF_MODE_EVAL,  //!< profiling
  PERF_MODE_IDU,   //!< only IDU evaluation
  PERF_MODE_PROBE, //!< function unit coverage rate.
};

struct config_t {
  int code;            //!< select one core(1204) or tree core(1204MP3).
  bool enable_avx;     //!< use Intel avx instruction (not yet).
  bool enable_calloc;  //!< clean internal resources.
  int64_t max_pkg_num; //!< run max instruction packet.
  uint32_t gm_size;    //!< set gm size in bytes.
  struct {
    char filepath[PATH_MAX]; //!< output log path.
    int level;               //!< log level.
    bool verbose;            //!< output log to console.
  } log;
  char plugin_filename[PATH_MAX]; //!< plugin dynamic library filename.
  char graph_filename[PATH_MAX];  //!< input graph.json
  bool print_subg_info; //!< prints subgraph information, when graph_filename is
                        //!< set.
  int8_t fp_mode; //!< x3 only, floating-point model: 0 for softfloat (default),
                  //!< 1 for cpu naive float
  struct {
    int mode;                       //!< see PERF_MODE_xxx
    char report_filename[PATH_MAX]; //!< output perf file
    uint32_t sys_freq; //!< fast evaluation: set sys freq in mhz[default: 1000].
    struct {
      uint32_t rd_latency; //!< fast evaluation: ddr latency read setting
      uint32_t wr_latency; //!< fast evaluation: ddr latency write setting
      uint32_t bandwidth;  //!< fast evaluation: ddr bandwidth setting, indicate
                           //!< how may bits of the axi data bus
    } ddr;
    union {
      struct {
        uint8_t event_size;
        uint8_t isa_events[16];
      }; // fast mode
      struct {
        uint8_t vliw_mode;
      }; // idu mode;
    };
  } perf;
};

enum { AIPU_EV_GRID_END, AIPU_EV_SYNC_FLAG };
typedef void (*event_handler_t)(uint32_t event, uint64_t value, void *context);

#endif //!__AIPU_CONFIG_H__
