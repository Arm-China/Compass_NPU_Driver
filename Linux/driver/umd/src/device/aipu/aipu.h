// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  aipu.h
 * @brief AIPU User Mode Driver (UMD) aipu module header
 */

#ifndef _AIPU_H_
#define _AIPU_H_

#include <mutex>
#include <vector>

#include "device_base.h"
#include "type.h"

namespace aipudrv {
class Aipu : public DeviceBase {
protected:
  int m_fd = 0;
  bool m_tick_counter = false;

private:
  aipu_ll_status_t init();
  void deinit();
  aipu_ll_status_t get_status(uint32_t max_cnt, bool of_this_thread,
                              void *jobbase = nullptr);

public:
  bool has_target(uint32_t arch, uint32_t version, uint32_t config,
                  uint32_t rev) override;
  aipu_status_t schedule(const JobDesc &job) override;
  aipu_ll_status_t read_reg(uint32_t partition_id, uint32_t offset,
                            uint32_t *value,
                            RegType type = RegType::AIPU_HOST_REG) override;
  aipu_ll_status_t write_reg(uint32_t partition_id, uint32_t offset,
                             uint32_t value,
                             RegType type = RegType::AIPU_HOST_REG) override;
  aipu_ll_status_t poll_status(uint32_t max_cnt, int32_t time_out,
                               bool of_this_thread,
                               void *jobbase = nullptr) override;

public:
  aipu_ll_status_t ioctl_cmd(uint32_t cmd, void *arg) override;
  int get_grid_id(uint16_t &grid_id) override;
  int get_start_group_id(int group_cnt, uint16_t &start_group_id) override;
  int put_start_group_id(uint16_t start_group_id, int group_cnt) override;
  const char *get_config_code() const override;

public:
  static aipu_status_t get_aipu(DeviceBase **dev) {
    aipu_status_t ret = AIPU_STATUS_SUCCESS;

    if (dev == nullptr)
      return AIPU_STATUS_ERROR_NULL_PTR;

    std::lock_guard<std::mutex> lock_(m_tex);
    if (m_aipu == nullptr) {
      m_aipu = new Aipu();
      ret = convert_ll_status(m_aipu->init());
      if (ret != AIPU_STATUS_SUCCESS) {
        delete m_aipu;
        m_aipu = nullptr;
        return ret;
      }
    }

    m_aipu->inc_ref_cnt();
    *dev = m_aipu;
    return AIPU_STATUS_SUCCESS;
  };

  static bool put_aipu(DeviceBase **dev) {
    if (dev == nullptr)
      return false;

    std::lock_guard<std::mutex> lock_(m_tex);
    if (m_aipu->dec_ref_cnt() == 0) {
      delete (Aipu *)*dev;
      *dev = nullptr;
      m_aipu = nullptr;
      return true;
    }
    return false;
  }

  aipu_ll_status_t get_device_status(device_status_t *status) override;

  virtual ~Aipu();
  Aipu(const Aipu &dev) = delete;
  Aipu &operator=(const Aipu &dev) = delete;

private:
  Aipu();
  static std::mutex m_tex;
  static Aipu *m_aipu;
  std::mutex m_group_id_mtx;
  std::map<uint16_t, struct aipu_id_desc> m_group_id_table;
  std::mutex m_dma_buf_mtx;
  std::map<int, struct aipu_dma_buf> m_dma_buf_map;
};
} // namespace aipudrv

#endif /* _AIPU_H_ */
