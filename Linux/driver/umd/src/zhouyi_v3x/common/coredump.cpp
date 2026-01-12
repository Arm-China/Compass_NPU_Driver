// Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#include "coredump.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ctime>
#include <thread>
#include <vector>

#include "device/aipu/ukmemory.h"
#include "standard_api.h"
#include "zhouyi_v3x/common/job_v3x.h"

namespace aipudrv {
Coredump::Coredump(JobV3X *job, DeviceBase *dev) {
  m_job = job;
  m_dev = dev;
  m_mem = dev->get_mem();

  m_elf.create(ELFCLASS32, ELFDATA2LSB);
  m_elf.set_type(ET_CORE);
  m_elf.set_machine(EM_AIPU);
}

Coredump::~Coredump() {
  if (m_desc != nullptr)
    free_coredump();
}

aipu_status_t Coredump::init() {
  if (m_is_initialized)
    return AIPU_STATUS_SUCCESS;

  /* assume 1 partition, otherwise coredump data structure needs to change */
  uint32_t partition_cnt = 0;
  aipu_status_t ret =
      convert_ll_status(m_dev->get_partition_count(&partition_cnt));
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = convert_ll_status(m_dev->get_cluster_count(0, &m_attr.cluster_cnt));
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = convert_ll_status(m_dev->get_core_count(0, 0, &m_attr.core_cnt));
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  m_attr.tec_cnt = m_dev->tec_cnt_per_core(0);

  LOG(LOG_INFO,
      "partition cnt: %u, cluster per partition: %u, core per cluster: %u, tec "
      "per core: %u",
      partition_cnt, m_attr.cluster_cnt, m_attr.core_cnt, m_attr.tec_cnt);

  ret = get_lm_sm_info();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (m_mem->get_gm_size(1) != 0)
    m_gm_cnt = 2;

  if (m_attr.sm_size == 0 || m_attr.lm_size == 0) {
    LOG(LOG_ERR,
        "exist 0 size memory. core shared memory size: %u, tec local memory "
        "size: %u",
        m_attr.sm_size, m_attr.lm_size);
    return AIPU_STATUS_ERROR_INVALID_SIZE;
  }

  m_is_initialized = true;
  return AIPU_STATUS_SUCCESS;
}

uint32_t Coredump::get_coredump_buffer_size(const CoredumpAttr &attr) {
  return ALIGN_PAGE(get_total_tec_size(attr)) + get_total_share_size(attr);
}

aipu_status_t Coredump::setup_coredump_buffer(const BufferDesc &desc) {
  uint32_t total_tec_size = get_total_tec_size(m_attr);

  DEV_PA_32 tec_pa_base = desc.align_asid_pa;
  DEV_PA_32 share_pa_base = tec_pa_base + ALIGN_PAGE(total_tec_size);
  char *tec_va_base = nullptr;
  if (m_mem->pa_to_va(desc.pa, desc.size, &tec_va_base) != 0)
    return AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
  char *share_va_base = tec_va_base + ALIGN_PAGE(total_tec_size);

  /* coredump tec buffer */
  uint32_t total_tecs = m_attr.cluster_cnt * m_attr.core_cnt * m_attr.tec_cnt;
  uint32_t each_tec_buffer_size =
      aligned(sizeof(CoredumpBufferTec) + m_attr.lm_size, ALIGNMENT_ADDR);
  for (uint32_t i = 0; i < total_tecs; ++i) {
    CoredumpBufferTec *coredump_buffer_tec =
        (CoredumpBufferTec *)(tec_va_base + each_tec_buffer_size * i);
    coredump_buffer_tec->buf_shared_address = share_pa_base | 0x29A;
    coredump_buffer_tec->lsram_info.size = m_attr.lm_size;
    std::pair<DEV_PA_32, char *> pa_va_pair =
        std::make_pair(tec_pa_base + each_tec_buffer_size * i,
                       tec_va_base + each_tec_buffer_size * i);
    m_tecs_bufs.push_back(pa_va_pair);
  }

  /* coredump share buffer */
  uint32_t total_cores = m_attr.cluster_cnt * m_attr.core_cnt;
  CoredumpBufferShare *coredump_buffer_share =
      (CoredumpBufferShare *)share_va_base;
  coredump_buffer_share->memory_count = total_cores + 1 /*gm*/;
  CoredumpBufferShare::DdrInfo *ddr_info =
      (CoredumpBufferShare::DdrInfo *)(share_va_base + sizeof(uint32_t));
  char *sm_buf_base = share_va_base + sizeof(uint32_t) +
                      coredump_buffer_share->memory_count *
                          sizeof(CoredumpBufferShare::DdrInfo);
  for (uint32_t i = 0; i < total_cores; ++i) {
    ddr_info[i].addr = 0;
    ddr_info[i].size = m_attr.sm_size;
    m_sm_bufs.push_back(sm_buf_base + i * m_attr.sm_size);
  }

  /* coredump gm buffer */
  char *gm_buf_base = sm_buf_base + total_cores * m_attr.sm_size;
  uint32_t i = 0;
  uint32_t offset = 0;
  for (auto &it : m_job_gm_bufs) {
    ddr_info[total_cores + i].addr = get_low_32(it.second->align_asid_pa);
    ddr_info[total_cores + i].size = it.second->size;
    m_gm_bufs.push_back(std::make_pair(it.second, gm_buf_base + offset));
    i += 1;
    offset += it.second->size;
  }
  return AIPU_STATUS_SUCCESS;
}

aipu_status_t Coredump::alloc_setup_buffer() {
  uint32_t size = get_coredump_buffer_size(m_attr);
  aipu_status_t ret =
      m_mem->malloc(size, ALIGN_ADDR(ALIGNMENT_ADDR), &m_desc, "coredump",
                    (0 << 8) | AIPU_BUF_REGION_DEFAULT);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  return setup_coredump_buffer(*m_desc);
}

void Coredump::free_coredump() {
  if (m_desc != nullptr && m_desc->size != 0)
    m_mem->free(&m_desc, "coredump");
}

aipu_status_t Coredump::do_coredump() {
  std::ostringstream oss;

  /* note: npu misc */
  NoteNPUMisc misc;
  misc.process_id = static_cast<uint32_t>(getpid());
  misc.thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
  misc.npu_arch = m_dev->get_npu_version();
  misc.instr_base = m_job->m_text->align_asid_pa;
  misc.graph_id = get_graph_id(m_job->m_id);
  misc.job_id = m_job->m_id;
  misc.write(oss);
  set_bin_note(oss.str().c_str(), oss.str().size(), "npumisc");

  /* note: rodata */
  DEV_PA_64 rodata_base = m_job->m_rodata->align_asid_pa;
  DEV_PA_64 rodata_pa = m_job->m_rodata->pa;
  char *rodata_va = nullptr;
  m_mem->pa_to_va(rodata_pa, m_job->m_rodata->size, &rodata_va);
  oss.clear();
  oss.str(std::string());
  oss.write((char *)&rodata_base, sizeof(uint32_t));
  oss.write(rodata_va, m_job->m_rodata->size);
  set_bin_note(oss.str().c_str(), oss.str().size(), "rodata");

  /* note: text */
  ELFIO::section *text = m_elf.sections.add(".text");
  text->set_type(SHT_PROGBITS);
  DEV_PA_64 text_pa = m_job->m_text->pa;
  char *text_va = nullptr;
  m_mem->pa_to_va(text_pa, m_job->graph().m_btext.size + 16, &text_va);
  text->set_address(misc.instr_base);
  text->set_data((const char *)text_va, m_job->graph().m_btext.size + 16);

  std::vector<uint32_t> buf_valid_idx;
  for (uint32_t i = 0; i < m_tecs_bufs.size(); i++) {
    CoredumpBufferTec *tec_buf =
        reinterpret_cast<CoredumpBufferTec *>(m_tecs_bufs[i].second);
    if (tec_buf->buf_valid == 1)
      buf_valid_idx.push_back(i);
  }

  if (buf_valid_idx.size() == 0) {
    LOG(LOG_WARN, "Cannot find valid flag in coredump tec buffers");
    return AIPU_STATUS_ERROR_INVALID_COREDUMP;
  }

  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  for (auto idx : buf_valid_idx) {
    CoredumpBufferTec *tec_buf =
        reinterpret_cast<CoredumpBufferTec *>(m_tecs_bufs[idx].second);
    uint32_t cp0_reg0 = tec_buf->cp0_reg[0].value;
    uint32_t cluster_id = (cp0_reg0 >> 8) & 0xFF;
    uint32_t core_id = (cp0_reg0 >> 4) & 0xF;
    uint32_t tec_id = cp0_reg0 & 0xF;

    const std::string common_suffix =
        std::string("cluster") + std::to_string(cluster_id) + "core" +
        std::to_string(core_id) + "tec" + std::to_string(tec_id);

    /* note: tec reg */
    uint32_t tec_reg_size =
        sizeof(CoredumpBufferTec::gpr) + sizeof(CoredumpBufferTec::t_reg) +
        sizeof(CoredumpBufferTec::p_reg) + sizeof(CoredumpBufferTec::cp0_reg);
    std::string note_name = std::string("tec_reg_") + common_suffix;
    set_bin_note(tec_buf->gpr, tec_reg_size, note_name);

    note_name = std::string("lm_") + common_suffix;
    set_bin_note(tec_buf->lsram_data, tec_buf->lsram_info.size, note_name);

    /* note: tcb */
    uint32_t tcbp_start = tec_buf->gpr[24];
    DEV_PA_64 tcbp_pa =
        m_job->m_tcbs->asid_base + tec_buf->gpr[24]; /* m_tcbs->align_asid_pa */
    char *tcbp_va = nullptr;
    m_mem->pa_to_va(tcbp_pa, tcb_ctl::TCB_LEN, &tcbp_va);
    note_name = "tcb_" + common_suffix;
    oss.clear();
    oss.str(std::string());
    oss.write((char *)&tcbp_start, sizeof(uint32_t));
    oss.write(tcbp_va, tcb_ctl::TCB_LEN);
    set_bin_note(oss.str().c_str(), oss.str().size(), note_name);

    /* note: stack */
    uint32_t sp_start = tec_buf->gpr[29];
    DEV_PA_64 sp_pa =
        m_job->m_sgt_allocated[0]->tasks[0].stack->asid_base +
        tec_buf
            ->gpr[29]; /* m_sgt_allocated[0]->tasks[0].stack->align_asid_pa */

    uint32_t sp = 0;
    if (m_job->graph().get_isa() == ISAv5)
      sp = reinterpret_cast<tcb_v3::tcb_t *>(tcbp_va)->task.sp;
    else if (m_job->graph().get_isa() == ISAv6)
      sp = reinterpret_cast<tcb_v3_2::tcb_t *>(tcbp_va)->task.sp;

    uint32_t stack_size = sp - tec_buf->gpr[29];
    char *sp_va = nullptr;
    m_mem->pa_to_va(sp_pa, stack_size, &sp_va);
    note_name = "stack_" + common_suffix;
    oss.clear();
    oss.str(std::string());
    oss.write((char *)&sp_start, sizeof(uint32_t));
    oss.write(sp_va, stack_size);
    set_bin_note(oss.str().c_str(), oss.str().size(), note_name);

    /* note: sm */
    note_name = std::string("sm_cluster") + std::to_string(cluster_id) +
                "core" + std::to_string(core_id);
    uint32_t buf_idx = cluster_id * m_attr.core_cnt + core_id;
    uint32_t buf_cnt = m_sm_bufs.size();
    if (buf_cnt > buf_idx)
      set_bin_note(m_sm_bufs[buf_idx], m_attr.sm_size, note_name);
    else
      LOG(LOG_WARN,
          "shared memory is invalid: buffers count: %u, buffer required index: "
          "%u",
          buf_cnt, buf_idx);

    /* note: gm, WARN: only support 1 cluster, because gm management is 1
     * cluster */
    note_name = std::string("gm_cluster") + std::to_string(cluster_id);
    for (uint32_t i = 0; i < m_gm_bufs.size(); ++i) {
      const BufferDesc *desc = m_gm_bufs[i].first;
      uint32_t gm_addr = get_low_32(desc->align_asid_pa);
      note_name += std::string("_") + std::to_string(i);
      oss.clear();
      oss.str(std::string());
      oss.write((char *)&gm_addr, sizeof(uint32_t));
      oss.write(m_gm_bufs[i].second, desc->req_size);
      set_bin_note(oss.str().c_str(), oss.str().size(), note_name);
    }

    /* note: tsm reg (NOTE: real-time register value should get from kmd
     * irq-isr) */
    oss.clear();
    oss.str(std::string());
    NoteTsmRegister note_tsm_reg;
    note_tsm_reg.tsm_regs = get_tsm_regs(cluster_id, core_id);
    note_tsm_reg.write(oss);
    note_name = std::string("tsm_reg_cluster") + std::to_string(cluster_id) +
                "core" + std::to_string(core_id);
    set_bin_note(oss.str().c_str(), oss.str().size(), note_name);
  }

  write();

  if (m_job->graph().get_isa() == ISAv6)
    ret =
        convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ABORT_CMDPOOL, nullptr));

  return ret;
}

/* note: lm/sm only used in coredump, and page selection is not thread-safe */
aipu_status_t Coredump::get_lm_sm_info() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  static std::mutex mtx;

  std::lock_guard<std::mutex> lg(mtx);
  uint32_t core_id = 0;
  m_dev->write_reg(core_id, TSM_PAGE_SELECTION_CONTROL,
                   EN_SELECT(1) | EN_CLUSTER(0) | EN_CORE(0));

  int32_t kret =
      m_dev->read_reg(core_id, TSM_LOCAL_MEMORY_FEATURE, &m_attr.lm_size);
  if (kret != AIPU_LL_STATUS_SUCCESS) {
    m_attr.lm_size = 0;
    LOG(LOG_ERR, "get invalid tec local memory size, set to 0!");
    ret = AIPU_STATUS_ERROR_INVALID_SIZE;
    goto out;
  }
  m_attr.lm_size = (1 << ((m_attr.lm_size & 0xF) - 1)) * 32768;

  kret = m_dev->read_reg(core_id, TSM_SHARE_MEMORY_FEATURE, &m_attr.sm_size);
  if (kret != AIPU_LL_STATUS_SUCCESS) {
    m_attr.sm_size = 0;
    LOG(LOG_ERR, "get invalid core share memory size!");
    ret = AIPU_STATUS_ERROR_INVALID_SIZE;
    goto out;
  }
  m_attr.sm_size = (1 << ((m_attr.sm_size & 0xF) - 1)) * 32768;

out:
  m_dev->write_reg(core_id, TSM_PAGE_SELECTION_CONTROL,
                   EN_SELECT(0) | EN_CLUSTER(0) | EN_CORE(0));
  return ret;
}

bool Coredump::get_note_idx(const std::string &name, uint32_t &idx) {
  if (m_note) {
    ELFIO::note_section_accessor notes(m_elf, m_note);
    ELFIO::Elf_Word no_notes = notes.get_notes_num();
    for (ELFIO::Elf_Word i = 0; i < no_notes; ++i) {
      ELFIO::Elf_Word type;
      std::string note_name;
      void *desc;
      ELFIO::Elf_Word descsz;

      if (notes.get_note(i, type, note_name, desc, descsz)) {
        if (note_name == name) {
          idx = i;
          return true;
        }
      }
    }
  }
  return false;
}

void Coredump::remove_note(const std::string &name) {
  uint32_t idx = 0;
  if (get_note_idx(name, idx)) {
    ELFIO::note_section_accessor notes(m_elf, m_note);
    notes.remove_note(idx);
  }
}

void Coredump::set_bin_note(const void *src, uint32_t size,
                            const std::string &name) {
  if (m_note == nullptr) {
    m_note = m_elf.sections.add(".note.coredump");
    m_note->set_type(SHT_NOTE);
  }
  remove_note(name);
  ELFIO::note_section_accessor notes(m_elf, m_note);
  notes.add_note(0x01, name, src, size);
}

void Coredump::write() {
  auto generate_suffix = []() {
    static uint32_t cnt = 0;
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%d-%m-%Y_%H-%M-%S", timeinfo);
    return std::string(buffer) + "_" + std::to_string(cnt++);
  };

  std::string filename = COREDUMP_FILE_PREFIX + "_" + generate_suffix();
  m_elf.save(filename);
}

std::vector<RegsMap> Coredump::get_v3_tsm_regs(uint32_t, uint32_t core_id) {
  // clang-format off
    /* tsm base */
    const static RegsMap tsm_base_map = {
        {0x00,
            {
                0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x20, 0x50, 0x60, 0x64, 0x68,
            }
        },
        {0x800,
            {
                0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24,
            }
        },
        {0xc00,
            {
                0x00, 0x04
            }
        },
    };

    /* debug selection */
    const static RegsMap ahb_cluster = {
        {0x2000,
            {
                0x00, 0x04, 0x08, 0x20, 0x30, 0x34, 0x38, 0x3c,
                0x40, 0x44, 0x64, 0x70, 0x74, 0x78, 0x7c,
                0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
                0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xd0, 0xd8, 0xdc,
                0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc
            }
        },
        {0x2800,
            {
                0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x20,
                0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58
            }
        },
    };
    const static RegsMap ahb_core = {
        {0x3000,
            {
                0x00, 0x04, 0x08, 0x20, 0x24, 0x50, 0x54, 0x80,
                0x84, 0x88, 0x90, 0x94, 0x98, 0x9c, 0xc0, 0xc4,
                0xd0, 0xd8, 0xdc, 0xe0, 0xe4, 0xe8, 0xec, 0xf0,
                0xf4, 0xf8, 0xfc, 0x100, 0x140, 0x180, 0x1c0,
                0x104, 0x144, 0x184, 0x1c4, 0x108, 0x148, 0x188,
                0x1c8, 0x10c, 0x14c, 0x18c, 0x1cc,
            }
        },
    };
  // clang-format on

  return {tsm_base_map, ahb_cluster, ahb_core};
}

std::vector<RegsMap> Coredump::get_v3_2_tsm_regs(uint32_t, uint32_t core_id) {
  // clang-format off
    /* tsm base */
    const static RegsMap tsm_base_map = {
        {0x00,
            {
                0x00, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x4c,
                0x50, 0x60, 0x64, 0x68, 0x70, 0x80,
            }
        },
        {0x800,
            {
                0x04, 0x08, 0x1c,
            }
        },
        {0x900,
            {
                0x04, 0x08, 0x1c,
            }
        },
        {0xa00,
            {
                0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
                0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
                0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
                0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
                0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
                0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
                0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
                0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,
            }
        },
    };

    /* debug selection */
    const static RegsMap ahb_cluster = {
        {0x2000,
            {
                0x00, 0x04, 0x24, 0x40, 0x48, 0x4c, 0x50, 0x58, 0x5c,
                0x68, 0x70, 0x80, 0x90,
            }
        },
        {0x2800,
            {
                0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x20,
                0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58
            }
        },
    };
    const static RegsMap ahb_core = {
        {0x3000,
            {
                0x00, 0x04, 0x08, 0x0c, 0x20, 0x24, 0x4c, 0x50, 0x54,
                0x58, 0x60, 0x6c, 0x74, 0x78, 0x7c, 0x80, 0x84, 0x88,
                0x90, 0x94, 0x98, 0x9c, 0xa0, 0xa4, 0xa8, 0xac,
                0xb0, 0xb4, 0xb8, 0xbc, 0xc0, 0xc4, 0xc8, 0xcc,
                0xd0, 0xd4, 0xd8, 0xdc, 0xe0, 0xe4, 0xe8, 0xec,
                0xf0, 0xf4, 0xf8, 0xfc, 0x100, 0x140, 0x180, 0x1c0,
                0x104, 0x144, 0x184, 0x1c4, 0x108, 0x148, 0x188,
                0x1c8, 0x10c, 0x14c, 0x18c, 0x1cc,
            }
        },
    };
  // clang-format on

  return {tsm_base_map, ahb_cluster, ahb_core};
}

std::vector<std::pair<uint32_t, uint32_t>>
Coredump::get_tsm_regs(uint32_t cluster_id, uint32_t core_id) {
  static std::mutex debug_link_mutex;
  std::lock_guard<std::mutex> lock(debug_link_mutex);
  std::vector<RegsMap> regs;

  if (m_job->graph().get_isa() == ISAv5)
    regs = get_v3_tsm_regs(cluster_id, core_id);
  else if (m_job->graph().get_isa() == ISAv6)
    regs = get_v3_2_tsm_regs(cluster_id, core_id);

  /* core_id will not impact tsm base registers and cluster */
  m_dev->write_reg(core_id, TSM_PAGE_SELECTION_CONTROL,
                   EN_SELECT(1) | EN_CLUSTER(cluster_id) | EN_CORE(core_id));

  std::vector<std::pair<uint32_t, uint32_t>> regs_pair;
  for (auto &reg : regs) {
    for (auto it : reg) {
      for (auto inner_offset : it.second) {
        uint32_t actural_offset = it.first + inner_offset;
        uint32_t value = 0;
        m_dev->read_reg(0, actural_offset, &value);
        regs_pair.push_back(std::make_pair(actural_offset, value));
      }
    }
  }
  return regs_pair;
}

} /* namespace aipudrv */