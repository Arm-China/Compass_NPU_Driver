#ifndef __AIPU_C_H__
#define __AIPU_C_H__

#include <stddef.h>
#include <stdint.h>
#include <sys/cdefs.h>

#include "config.h"

__BEGIN_DECLS

// external impl [optional]
struct mem_interface_t {
  void *cxt;
  int64_t (*read)(uint64_t addr, void *dest, size_t size, void *cxt);
  int64_t (*write)(uint64_t addr, const void *src, size_t size, void *cxt);
  int64_t (*zeroize)(uint64_t addr, size_t size, void *cxt);
  size_t (*size)(void *cxt);
  bool (*invalid)(uint64_t addr, void *cxt);
  bool (*get_info)(uint64_t addr, uint64_t *base, size_t *size, void *cxt);
};

typedef enum { EV_RT_PRT } event_t;
struct tec_interface_t;
struct dbg_lite_t {
  void *cxt;
  bool (*check_break)(struct tec_interface_t *tec, void *cxt);
  bool (*handle)(struct tec_interface_t *tec, void *cxt);
  void (*on_event)(struct tec_interface_t *tec, event_t, void *, size_t,
                   void *cxt);
};

typedef void *sim_aipu_t;
struct sim_aipu_api_t {
  sim_aipu_t (*create)(const struct config_t *, struct mem_interface_t *);
  sim_aipu_t (*create1)(void *); // input parameter is c++ aipu obj
  int (*destroy)(sim_aipu_t);
  int (*read_register)(sim_aipu_t, uint32_t addr, uint32_t *v);
  int (*write_register)(sim_aipu_t, uint32_t addr, uint32_t v);
  void (*set_dbg_lite)(sim_aipu_t, struct dbg_lite_t *);
  int (*version)(sim_aipu_t);
  void (*enable_profiling)(sim_aipu_t, bool en);
  void (*dump_profiling)(sim_aipu_t);
  void (*set_event_handler)(sim_aipu_t, event_handler_t, void *cxt);
};

#define SIM_AIPU_API_INIT_AS_STR "sim_aipu_api_init"
typedef struct sim_aipu_api_t *(*fn_sim_aipu_api_init_t)();
struct sim_aipu_api_t *sim_aipu_api_init();

typedef enum {
  GRF,
  FRF,
  TRF,
  PRF,
  SRF,
} RfType;

enum {
  STAT_NONE = 0,
  STAT_EXIT = 0x1,
  STAT_DIS_WFE = 0x02,
};
// sim aipu internal impl
struct tec_interface_t {
  void *cxt;
  uint8_t (*id)(void *cxt);
  uint8_t (*core_id)(void *cxt);
  uint8_t (*cluster_id)(void *cxt);

  uint8_t (*tcount)(void *cxt);
  const uint8_t *(*get_tpr)(uint8_t n, void *cxt);
  void (*set_tpr)(uint8_t n, const uint8_t data[32], void *cxt);

  uint8_t (*pcount)(void *cxt);
  uint32_t (*get_ppr)(uint8_t n, void *cxt);
  void (*set_ppr)(uint8_t n, uint32_t data, void *cxt);

  int8_t (*gpr_count)(void *cxt);
  uint32_t (*get_gpr)(uint8_t n, void *cxt);
  void (*set_gpr)(uint8_t n, uint32_t data, void *cxt);

  int8_t (*fpr_count)(void *cxt);
  uint32_t (*get_fpr)(uint8_t n, void *cxt);
  void (*set_fpr)(uint8_t n, uint32_t data, void *cxt);

  int8_t (*spr_count)(void *cxt);
  uint32_t (*get_spr)(uint8_t n, void *cxt);
  void (*set_spr)(uint8_t n, uint32_t data, void *cxt);

  void (*sync_reg)(uint8_t n, int rf_type, void *cxt);
  void (*sync_regs)(void *cxt);

  uint32_t (*get_pc)(void *cxt);
  void (*set_pc)(uint32_t new_pc, void *cxt);
  uint32_t (*get_next_pc)(void *cxt);
  void (*set_next_pc)(uint32_t new_pc, void *cxt);

  int (*write_sram)(uint32_t addr, const void *src, uint32_t size, void *cxt);
  int (*read_sram)(uint32_t addr, void *dest, uint32_t size, void *cxt);

  int (*read_cpx_csr)(uint8_t gid, uint32_t addr, uint32_t *v, void *cxt);
  int (*write_cpx_csr)(uint8_t gid, uint32_t addr, uint32_t v, void *cxt);

  int (*get_status)(void *cxt);
  void (*set_status)(int val, void *cxt);
};

__END_DECLS

#endif //!__AIPU_C_H__
