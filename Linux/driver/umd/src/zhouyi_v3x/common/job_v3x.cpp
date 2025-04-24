// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

/**
 * @file  job_v3_1.cpp
 * @brief AIPU User Mode Driver (UMD) aipu v3_1 job module implementation
 */

#include "job_v3x.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <mutex>

#include "context.h"
#include "utils/helper.h"

namespace aipudrv {
JobV3X::JobV3X(MainContext *ctx, GraphBase &graph, DeviceBase *dev,
               aipu_create_job_cfg_t *config)
    : JobBase(ctx, graph, dev), m_partition_id(config->partition_id),
      m_core_id(config->dbg_core_id), m_qos(config->qos_level),
      m_dbg_dispatch(config->dbg_dispatch),
      m_fm_mem_region(config->fm_mem_region) {
  m_init_tcb.init(0);
  m_sgt_allocated.clear();

#if defined(SIMULATION)
  /**
   * initialize an invalid cmdpool id, set it on scheduling to sim
   */
  m_bind_cmdpool_id = 0xffffffff;
#endif

  m_segmmu_num = get_graph().m_segmmu_num;

  if (config->fm_idxes) {
    for (int i = 0; i < config->fm_idxes_cnt; i++)
      m_fm_idxes.insert(config->fm_idxes[i]);
  }

  if (get_graph().is_dynamic_shape())
    m_dyn_shape = new DynamicShape(*this, get_graph(), config->dynshape);

#ifndef SIMULATION
  const char *umd_coredump = getenv("UMD_COREDUMP");
  if (umd_coredump != nullptr &&
      (umd_coredump[0] == 'y' || umd_coredump[0] == 'Y')) {
    m_coredump = new Coredump(this, dev);
    if (m_coredump->init() != AIPU_STATUS_SUCCESS) {
      LOG(LOG_WARN, "coredump initializes failed!");
    }
  }
#endif
}

JobV3X::~JobV3X() {
  if (m_gm != nullptr) {
    delete m_gm;
    m_gm = nullptr;
  }

  if (m_dyn_shape != nullptr) {
    delete m_dyn_shape;
    m_dyn_shape = nullptr;
  }

  if (m_coredump != nullptr) {
    delete m_coredump;
    m_coredump = nullptr;
  }
}

aipu_status_t JobV3X::setup_rodata_sg(
    uint32_t sg_id, const std::vector<struct GraphParamMapLoadDesc> &param_map,
    std::vector<BufferDesc *> &reuse_buf, std::vector<BufferDesc *> &static_buf,
    std::set<uint32_t> *dma_buf_idx) {
  BufferDesc rodata, dcr;
  // const std::vector<struct GraphParamMapLoadDesc>& param_map =
  //     get_graph().get_subgraph(sg_id).param_map;
  // std::vector<BufferDesc>& reuse_buf  = m_sg_job[sg_id].reuses;
  // std::vector<BufferDesc>& static_buf = m_sg_job[sg_id].weights;

  rodata.init(0, m_rodata->pa, m_rodata->size, m_rodata->req_size);
  if (m_descriptor != nullptr) {
    dcr.init(0, m_descriptor->pa, m_descriptor->size, m_descriptor->req_size);
    return setup_rodata(param_map, reuse_buf, static_buf, rodata, &dcr,
                        dma_buf_idx);
  } else {
    return setup_rodata(param_map, reuse_buf, static_buf, rodata, nullptr,
                        dma_buf_idx);
  }
}

aipu_status_t JobV3X::alloc_priv_buffer() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  SubGraphTask sg_task = {0};
  uint32_t priv_offset = 0;
  uint32_t private_size = 0;
  uint32_t max_private_size = 0;

  /* caculate the total size of each buffer type */
  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    const auto &sg = get_graph().get_subgraph(sg_idx);
    if (sg.precursor_cnt == SUBG_DEPEND_PREALL)
      private_size = 0;

    /* sg.private_buffers.size() should be 1, because it only has workspace */
    for (uint32_t pr_idx = 0; pr_idx < sg.private_buffers.size(); pr_idx++) {
      const GraphSectionDesc &section_desc = sg.private_buffers[pr_idx];
      private_size += ALIGN_PAGE(section_desc.size);
    }
    max_private_size = std::max(max_private_size, private_size);
  }

  /* allocate buffer only once for each type */
  if (max_private_size > 0) {
    ret = m_mem->malloc(max_private_size, 0, &m_top_priv_buf, "tot_priv");
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_DEBUG,
          "optmize alloc private buffer, size: 0x%x [fail], try scatter alloc",
          max_private_size);
      goto priv_alloc_fail;
    }
  }

  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    const auto &sg = get_graph().get_subgraph(sg_idx);
    sg_task.reset(sg_idx, sg.bss_idx);

    if (sg.precursor_cnt == SUBG_DEPEND_PREALL)
      priv_offset = 0;

    /* each subgraph has private buffer core-accessed */
    for (uint32_t k = 0; k < sg.private_buffers.size(); k++) {
      const GraphSectionDesc &section_desc = sg.private_buffers[k];
      BufferDesc *bufferDesc = nullptr;

      if (section_desc.size != 0) {
        bufferDesc = new BufferDesc;
        bufferDesc->reset();
        bufferDesc->init(m_top_priv_buf->asid_base,
                         m_top_priv_buf->pa + priv_offset,
                         ALIGN_PAGE(section_desc.size), section_desc.size);
        priv_offset += ALIGN_PAGE(section_desc.size);

        if (m_dump_reuse)
          m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

        sg_task.reuse_priv_buffers.push_back(bufferDesc);
      } else {
        LOG(LOG_WARN, "priv %d: size == 0", k);
      }
    }

    m_sg_job.push_back(sg_task);
  }
  return AIPU_STATUS_SUCCESS;

priv_alloc_fail:
  if (m_top_priv_buf != nullptr && m_top_priv_buf->size > 0)
    m_mem->free(&m_top_priv_buf);

  return ret;
}

aipu_status_t JobV3X::alloc_reuse_buffer() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t pad_size = get_graph().get_alloc_pad_size();

  /* allocate reuse buffers, all subgraphs share one copy of reuse buffers */
  for (uint32_t bss_id = 0; bss_id < get_graph().get_bss_cnt(); bss_id++) {
    BSSBuffer bssBuffer;
    if (bss_id == 0) {
      for (uint32_t k = 0; k < get_graph().get_bss(0).reuse_sections.size();
           k++) {
        const GraphSectionDesc &section_desc =
            get_graph().get_bss(0).reuse_sections[k];
        BufferDesc *bufferDesc = nullptr;

        if (section_desc.size != 0) {
          std::string buf_name = "reuse_" + std::to_string(k);

          /* handle buffer if allocated from GM_V3_1 */
          if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE)) {
            bufferDesc = new BufferDesc;
            buf_name = "gm_" + buf_name;
            ret = m_gm->gm_malloc(bss_id, k, GM_BUF_TYPE_REUSE, buf_name,
                                  bufferDesc);
          } else {
            if ((m_fm_idxes.count(k) == 1) ||
                (m_fm_mem_region != AIPU_MEM_REGION_DEFAULT))
              ret = m_mem->malloc(section_desc.size + pad_size,
                                  section_desc.align_in_page, &bufferDesc,
                                  buf_name.c_str(), m_fm_mem_region);
            else
              ret = m_mem->malloc(section_desc.size + pad_size,
                                  section_desc.align_in_page, &bufferDesc,
                                  buf_name.c_str(), AIPU_MEM_REGION_DEFAULT);
          }

          if (ret != AIPU_STATUS_SUCCESS) {
            LOG(LOG_ERR, "alloc reuse buffer %d [fail]", k);
            goto add_bss_buffer;
          }

          if (m_dump_reuse)
            m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

          bssBuffer.reuses.push_back(bufferDesc);
        } else {
          LOG(LOG_WARN, "reuse %d: size == 0", k);
        }
      }

      /* init task weights address, share a common copy */
      bssBuffer.weights = &get_graph().get_weight_buffer_info()[0].wb_weights;

    add_bss_buffer:
      m_bss_buffer_vec.push_back(bssBuffer);
      if (ret != AIPU_STATUS_SUCCESS)
        goto out;
    }
  }

  if (get_subgraph_cnt() > 0 &&
      get_graph().get_subgraph(0).printfifo_size > 0) {
    std::string buf_name = "printf";
    ret = m_mem->malloc(get_subgraph_cnt() * AIPU_PAGE_SIZE, 0, &m_pprint,
                        buf_name.c_str());
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
  }

out:
  return ret;
}

int JobV3X::alloc_reuse_buffer_optimized() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t reuse_buf_total_size = 0;
  uint32_t offset = 0;
  int retval = 0;
  uint32_t pad_size = get_graph().get_alloc_pad_size();

  if (m_fm_mem_region != AIPU_MEM_REGION_DEFAULT) {
    retval = -1;
    LOG(LOG_DEBUG, "don't try optimization if specify memory region");
    goto opt_alloc_fail;
  }

  for (uint32_t k = 0; k < get_graph().get_bss(0).reuse_sections.size(); k++) {
    /**
     * the below two types buffer can't pass
     * centralized memory allocation flow.
     */
    if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE))
      continue;

    if (m_fm_idxes.count(k) == 1)
      continue;

    const GraphSectionDesc &section_desc =
        get_graph().get_bss(0).reuse_sections[k];
    reuse_buf_total_size += ALIGN_PAGE(section_desc.size);
    m_top_reuse_idx.insert(k);
  }

  ret = m_mem->malloc(reuse_buf_total_size + pad_size, 0, &m_top_reuse_buf,
                      "tot_reuse");
  if (ret != AIPU_STATUS_SUCCESS) {
    retval = -1;
    LOG(LOG_DEBUG,
        "optmize alloc reuse buffer, size: 0x%x [fail], try scatter alloc",
        reuse_buf_total_size);
    goto opt_alloc_fail;
  }

  for (uint32_t bss_id = 0; bss_id < get_graph().get_bss_cnt(); bss_id++) {
    BSSBuffer bssBuffer;

    /* allocate reuse buffers, all subgraphs share one copy of reuse buffers */
    if (bss_id == 0) {
      for (uint32_t k = 0; k < get_graph().get_bss(0).reuse_sections.size();
           k++) {
        const GraphSectionDesc &section_desc =
            get_graph().get_bss(0).reuse_sections[k];
        BufferDesc *bufferDesc = nullptr;

        if (section_desc.size != 0) {
          std::string buf_name = "reuse_" + std::to_string(k);
          bufferDesc = new BufferDesc;
          bufferDesc->reset();

          /* handle buffer if allocated from GM_V3_1 */
          if (m_gm->gm_is_gm_buffer(k, GM_BUF_TYPE_REUSE)) {
            buf_name = "gm_" + buf_name;
            ret = m_gm->gm_malloc(bss_id, k, GM_BUF_TYPE_REUSE, buf_name,
                                  bufferDesc);
            if (ret != AIPU_STATUS_SUCCESS) {
              retval = -3;
              LOG(LOG_ERR, "alloc GM_V3_1 reuse buffer %d [fail]", k);
              goto add_bss_buffer;
            }
          } else {
            if (m_fm_idxes.count(k) == 1) {
              ret = m_mem->malloc(section_desc.size + pad_size,
                                  section_desc.align_in_page, &bufferDesc,
                                  buf_name.c_str(), m_fm_mem_region);
              if (ret != AIPU_STATUS_SUCCESS) {
                retval = -4;
                LOG(LOG_ERR, "alloc specified reuse buffer %d [fail]", k);
                goto add_bss_buffer;
              }
            } else {
              bufferDesc->init(
                  m_top_reuse_buf->asid_base, m_top_reuse_buf->pa + offset,
                  ALIGN_PAGE(section_desc.size), section_desc.size);
              offset += ALIGN_PAGE(section_desc.size);
            }
          }

          if (m_dump_reuse)
            m_mem->mem_bzero(bufferDesc->pa, bufferDesc->size);

          bssBuffer.reuses.push_back(bufferDesc);
        } else {
          LOG(LOG_WARN, "opt reuse %d: size == 0", k);
        }
      }

      /* init task weights address, share a common copy */
      bssBuffer.weights = &get_graph().get_weight_buffer_info()[0].wb_weights;
    }

  add_bss_buffer:
    m_bss_buffer_vec.push_back(bssBuffer);
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
  }

  if (get_subgraph_cnt() > 0 &&
      get_graph().get_subgraph(0).printfifo_size > 0) {
    std::string buf_name = "printf";
    ret = m_mem->malloc(get_subgraph_cnt() * AIPU_PAGE_SIZE, 0, &m_pprint,
                        buf_name.c_str());
    if (ret != AIPU_STATUS_SUCCESS) {
      retval = -6;
      goto out;
    }
  }

  m_optimized_reuse_alloc = true;
  return retval;

opt_alloc_fail:
  if (m_top_reuse_buf != nullptr && m_top_reuse_buf->size > 0)
    m_mem->free(&m_top_reuse_buf);

  m_top_reuse_idx.clear();

out:
  return retval;
}

aipu_status_t JobV3X::init_per_task_data() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t sg_idx = 0;
  uint32_t head_cnt = 0;
  bool dep_all_flag = false;

  for (uint32_t i = 0; i < m_sg_cnt; i++) {
    SubGraphTask &sg_task = m_sg_job[i];
    get_tcb_head_cnt(i, head_cnt);

    if (i != 0) {
      if (get_graph().get_subgraph(i).precursor_cnt == SUBG_DEPEND_PREALL) {
        sg_idx = 0;
        dep_all_flag = true;
      }

      if (dep_all_flag && sg_idx < m_sgt_allocated.size()) {
        for (uint32_t j = 0; j < m_task_per_sg; j++) {
          Task task;
          memset((void *)&task, 0, sizeof(task));

          task = m_sgt_allocated[sg_idx]->tasks[j];
          task.tcb.init(m_tcbs->pa +
                        (head_cnt + i * m_task_per_sg + j) * sizeof(tcb_t));
          sg_task.tasks.push_back(task);
        }

        sg_idx++;
        continue;
      } else {
        dep_all_flag = false;
      }
    }

    {
      /* 1 init per-task data structs */
      for (uint32_t j = 0; j < m_task_per_sg; j++) {
        Task task;
        memset((void *)&task, 0, sizeof(task));

        /* 1.1. init task tcb */
        task.tcb.init(m_tcbs->pa +
                      (head_cnt + i * m_task_per_sg + j) * sizeof(tcb_t));

        /* 1.2. allocate task stack */
        task.stack = nullptr;
        ret = m_mem->malloc(get_graph().get_bss(0).stack_size,
                            get_graph().get_bss(0).stack_align_in_page,
                            &task.stack, "stack");
        if (ret != AIPU_STATUS_SUCCESS)
          goto out;

        /* 1.3. allocate and load task dp */
        if (get_graph().get_subgraph(i).private_data_size != 0) {
          task.private_data = nullptr;
          ret = m_mem->malloc(get_graph().get_subgraph(i).private_data_size, 0,
                              &task.private_data, "dp_data");
          if (ret != AIPU_STATUS_SUCCESS)
            goto out;

          m_mem->mem_bzero(task.private_data->pa, task.private_data->size);
        }
        sg_task.tasks.push_back(task);
      }
      m_sgt_allocated.push_back(&sg_task);
    }
  }

out:
  return ret;
}

aipu_status_t JobV3X::alloc_load_job_buffers() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  SubGraphTask sg_task;
  int retval = 0;

  /* 0. allocate and set model global parameter if need */
  if (get_graph().is_dynamic_shape() && m_dyn_shape->is_set_dyn_shape_true() &&
      m_dyn_shape->get_config_shape_sz() > 0) {
    DS_ModelGlobalParam *modelGlobalParam =
        (DS_ModelGlobalParam *)get_graph().m_bglobalparam.va;
    uint32_t input_shape_offset = modelGlobalParam->input_shape_offset;

    ret = m_mem->malloc(get_graph().m_bglobalparam.size, 0,
                        &m_model_global_param, "modelparam");
    if (ret != AIPU_STATUS_SUCCESS) {
      LOG(LOG_ERR, "alloc model global param [fail]");
      goto finish;
    }

    /* store original section data */
    m_mem->write(m_model_global_param->pa, get_graph().m_bglobalparam.va,
                 sizeof(DS_ModelGlobalParam));

    for (uint32_t input_idx = 0; input_idx < m_dyn_shape->get_config_shape_sz();
         input_idx++) {
      if (m_dyn_shape->in_config_shape(input_idx)) {
        for (uint32_t dim_idx = 0;
             dim_idx < m_dyn_shape->get_config_shape_dim_sz(input_idx);
             dim_idx++) {
          uint32_t shape_item =
              m_dyn_shape->get_config_shape_item(input_idx, dim_idx);

          m_mem->write(m_model_global_param->pa + input_shape_offset,
                       &shape_item, sizeof(uint32_t));
          input_shape_offset += sizeof(uint32_t);
        }
      } else {
        ret = AIPU_STATUS_ERROR_NOT_CONFIG_SHAPE;
        LOG(LOG_ERR, "input shape %d is not configured", input_idx);
        goto finish;
      }
    }
  }

  /* 1. allocate and load job rodata */
  if (get_graph().m_brodata.size != 0) {
    ret = m_mem->malloc(get_graph().m_brodata.size, 0, &m_rodata, "rodata");
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;

    m_mem->write(m_rodata->pa, get_graph().m_brodata.va,
                 get_graph().m_brodata.size);
  }

  /* 2. allocate and load job descriptor */
  if (get_graph().m_bdesc.size != 0) {
    ret = m_mem->malloc(get_graph().m_bdesc.size, 0, &m_descriptor, "dcr");
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;

    m_mem->write(m_descriptor->pa, get_graph().m_bdesc.va,
                 get_graph().m_bdesc.size);
  }

  /* 3. allocate and reset job TCBs */
  ret = m_mem->malloc(m_tot_tcb_cnt * sizeof(tcb_t), 0, &m_tcbs, "tcbs");
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;
  if (m_tcbs->align_asid_pa == 0 || m_tcbs->pa == 0) {
    BufferDesc *tmp_holdDesc = nullptr;

    /* free the orginal buffer starting from addr 0x0 */
    m_mem->free(&m_tcbs);

    ret = m_mem->malloc(0x1000, 0, &tmp_holdDesc, "holdDesc");
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;

    ret = m_mem->malloc(m_tot_tcb_cnt * sizeof(tcb_t), 0, &m_tcbs, "tcbs");
    if (ret != AIPU_STATUS_SUCCESS) {
      m_mem->free(&tmp_holdDesc);
      goto finish;
    }

    m_mem->free(&tmp_holdDesc);
  }

  m_mem->zeroize(m_tcbs->pa, m_tot_tcb_cnt * sizeof(tcb_t));
  m_init_tcb.init(m_tcbs->pa);

  /* 4. allocate subgraph buffers */
  ret = alloc_priv_buffer();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* 5. allocate subgraph buffers */
  retval = alloc_reuse_buffer_optimized();
  if (retval == -1) {
    ret = alloc_reuse_buffer();
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
  } else if (retval < -1) {
    ret = AIPU_STATUS_ERROR_BUF_ALLOC_FAIL;
    goto finish;
  }

  /* 6. init each subgraph's task tcbs */
  ret = init_per_task_data();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* 7. get IO buffer address, all subgraphs share the same copy of reuse
   * buffers */
  create_io_buffers(get_graph().get_bss(0).io, m_bss_buffer_vec[0].reuses);
  if (get_subgraph_cnt() == 0)
    goto finish;

  /* 8. setup rodata & dcr, update entry for all subgraphs in global RO/DCR
   * section */
  ret =
      setup_rodata_sg(0, get_graph().get_bss(0).param_map,
                      m_bss_buffer_vec[0].reuses, *m_bss_buffer_vec[0].weights);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* update subgraph private buffers PA in RO/DCR section */
  for (uint32_t sg_idx = 0; sg_idx < m_sg_cnt; sg_idx++) {
    std::vector<BufferDesc *> invalid_buf;

    LOG(LOG_INFO, "sg_idx: %d", sg_idx);
    ret = setup_rodata_sg(sg_idx,
                          get_graph().get_subgraph(sg_idx).private_buffers_map,
                          m_sg_job[sg_idx].reuse_priv_buffers, invalid_buf);
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
  }

  /* 9. setup remap */
  setup_remap(*m_rodata, m_descriptor);

  /* 10. parse SegMMU config */
  ret = setup_segmmu(m_sg_job[0]);
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /* 11. parse SegMMU config */
  if (m_coredump && m_coredump->is_initialized()) {
    ret = m_coredump->alloc_coredump();
    if (ret != AIPU_STATUS_SUCCESS)
      goto finish;
  }

finish:
  if (ret) {
    for (uint32_t i = 0; i < m_sg_job.size(); i++)
      free_sg_buffers(m_sg_job[i]);

    free_job_buffers();
  }
  return ret;
}

/* TODO: check va/pa is in asid0 */
aipu_status_t
JobV3X::specify_io_buffer(aipu_shared_tensor_info_t &tensor_info) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  const std::vector<struct JobIOBuffer> *iobuffer_vec = nullptr;
  BufferDesc *bufferDesc = nullptr;
  const char *str = "free_input";
  uint32_t reuse_index = 0;
  uint32_t type = tensor_info.type;
  uint32_t index = tensor_info.tensor_idx;
  uint64_t offset = tensor_info.offset_in_dmabuf;
  int fd = tensor_info.dmabuf_fd;
  bool update_ro = true;
  int share_case_type = tensor_info.shared_case_type;
  uint64_t buffer_pa = tensor_info.pa;
  struct aipu_dma_buf dma_buf {
    fd, 0, 0
  };

  /* shared buffer shouldn't reuse with any other reuse buffers */
  switch (type) {
  case AIPU_TENSOR_TYPE_INPUT:
    iobuffer_vec = &m_inputs;
    if (!get_graph().get_disable_input_reuse()) {
      LOG(LOG_WARN, "if use shared input buffer, you'd better disable input "
                    "reuse at aipu compiler side");
    }
    break;

  case AIPU_TENSOR_TYPE_OUTPUT:
    iobuffer_vec = &m_outputs;
    if (!get_graph().get_disable_output_reuse()) {
      LOG(LOG_WARN, "if use shared output buffer, you'd better disable output "
                    "reuse at aipu compiler side");
    }
    str = "free_output";
    break;

  default:
    ret = AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
    LOG(LOG_ERR, "tensor type: %d, index: %d [not exist]", type, index);
    goto out;
  }

  if (index >= iobuffer_vec->size()) {
    ret = AIPU_STATUS_ERROR_INVALID_TENSOR_ID;
    goto out;
  }

  reuse_index = (*iobuffer_vec)[index].ref_section_iter;
  if (type == AIPU_TENSOR_TYPE_INPUT) {
    for (uint32_t i = 0; i < get_graph().get_bss(0).io.outputs.size(); i++) {
      auto &idtensor_desc = get_graph().get_bss(0).io.outputs[i];
      if (idtensor_desc.ref_section_iter == reuse_index)
        return AIPU_STATUS_ERROR_DMABUF_SHARED_IO;
    }
  } else {
    for (uint32_t i = 0; i < get_graph().get_bss(0).io.inputs.size(); i++) {
      auto &idtensor_desc = get_graph().get_bss(0).io.inputs[i];
      if (idtensor_desc.ref_section_iter == reuse_index)
        return AIPU_STATUS_ERROR_DMABUF_SHARED_IO;
    }
  }

  /* free io buffer allocated internally,replace it with new buffer */
  bufferDesc = m_bss_buffer_vec[0].reuses[reuse_index];
  m_bss_buffer_vec[0].dma_buf_idx.insert(reuse_index);
  if (!m_optimized_reuse_alloc) {
    ret = m_mem->free_phybuffer(bufferDesc, str);
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
  }

  switch (share_case_type) {
  case AIPU_SHARE_BUF_IN_ONE_PROCESS:
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    update_io_buffers(get_graph().get_bss(0).io, m_bss_buffer_vec[0].reuses);
    break;
  case AIPU_SHARE_BUF_CUSTOMED:
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dump_ignore_flag(true);
    break;
  case AIPU_SHARE_BUF_DMABUF:
    ret = convert_ll_status(
        m_dev->ioctl_cmd(AIPU_IOCTL_GET_DMABUF_INFO, &dma_buf));
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;

    buffer_pa = dma_buf.pa + offset;
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, offset);
    break;
  case AIPU_SHARE_BUF_ATTACH_DMABUF:
    ret =
        convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ATTACH_DMABUF, &dma_buf));
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;

    buffer_pa = dma_buf.pa + offset;
    bufferDesc->init(m_mem->get_asid_base(0), buffer_pa, bufferDesc->size,
                     bufferDesc->req_size);
    (*iobuffer_vec)[index].set_dmabuf_info(fd, dma_buf.bytes, offset);
    break;
  default:
    ret = AIPU_STATUS_ERROR_INVALID_OP;
    goto out;
  }

  LOG(LOG_DEBUG, "specify_io_buffer: pa=%lx, size=%lx, share_case_type=%d",
      buffer_pa, bufferDesc->size, share_case_type);

  if (update_ro) {
    ret = setup_rodata_sg(
        0, get_graph().get_bss(0).param_map, m_bss_buffer_vec[0].reuses,
        *m_bss_buffer_vec[0].weights, &m_bss_buffer_vec[0].dma_buf_idx);
    if (ret != AIPU_STATUS_SUCCESS)
      goto out;
  }

out:
  return ret;
}

void JobV3X::free_sg_buffers(SubGraphTask &sg_task) {
  if (m_top_priv_buf != nullptr && m_top_priv_buf->size > 0) {
    m_mem->free(&m_top_priv_buf, "tot_priv");
    m_top_priv_buf_freed = true;
  }

  if (m_top_priv_buf_freed) {
    for (uint32_t i = 0; i < sg_task.reuse_priv_buffers.size(); i++)
      m_mem->free_bufferdesc(&sg_task.reuse_priv_buffers[i]);
  } else {
    for (uint32_t i = 0; i < sg_task.reuse_priv_buffers.size(); i++)
      m_mem->free(&sg_task.reuse_priv_buffers[i]);
  }
  sg_task.reuse_priv_buffers.clear();

  for (uint32_t i = 0; i < m_sgt_allocated.size(); i++) {
    for (uint32_t j = 0; j < m_task_per_sg; j++) {
      Task *task;
      task = &m_sgt_allocated[i]->tasks[j];
      m_mem->free(&task->stack);
      m_mem->free(&task->private_data);
    }
  }
  m_sgt_allocated.clear();
}

aipu_status_t JobV3X::free_job_buffers() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (m_model_global_param && m_model_global_param->size != 0)
    m_mem->free(&m_model_global_param, "modelparam");

  if (m_rodata && m_rodata->size != 0)
    m_mem->free(&m_rodata, "rodata");

  if (m_descriptor && m_descriptor->size != 0)
    m_mem->free(&m_descriptor, "dcr");

  if (m_tcbs && m_tcbs->size != 0)
    m_mem->free(&m_tcbs, "tcbs");

  if (m_pprint && m_pprint->size != 0)
    m_mem->free(&m_pprint, "printf");

  if (m_coredump)
    m_coredump->free_coredump();

  m_init_tcb.init(0);

  for (uint32_t i = 0; i < m_sg_job.size(); i++) {
    free_sg_buffers(m_sg_job[i]);
    m_sg_job[i].reset(i, -1);
  }

  for (uint32_t bss_idx = 0; bss_idx < m_bss_buffer_vec.size(); bss_idx++) {
    if (m_top_reuse_buf != nullptr && m_top_reuse_buf->size > 0) {
      m_mem->free(&m_top_reuse_buf, "tot_reuse");
      for (uint32_t i = 0; i < m_bss_buffer_vec[bss_idx].reuses.size(); i++) {
        if (m_gm->gm_is_gm_buffer(i, GM_BUF_TYPE_REUSE))
          m_mem->free(&m_bss_buffer_vec[bss_idx].reuses[i]);
        else
          m_mem->free_bufferdesc(&m_bss_buffer_vec[bss_idx].reuses[i]);
      }
      m_top_reuse_idx.clear();
    } else {
      for (uint32_t i = 0; i < m_bss_buffer_vec[bss_idx].reuses.size(); i++) {
        if (m_bss_buffer_vec[bss_idx].dma_buf_idx.count(i) == 1) {
          m_mem->free_bufferdesc(&m_bss_buffer_vec[bss_idx].reuses[i]);
          continue;
        }
        m_mem->free(&m_bss_buffer_vec[bss_idx].reuses[i]);
      }
    }

    m_bss_buffer_vec[bss_idx].reuses.clear();
    m_bss_buffer_vec[bss_idx].weights = nullptr;
  }

  m_sg_job.clear();
  m_bss_buffer_vec.clear();

  m_inputs.clear();
  m_outputs.clear();
  m_inter_dumps.clear();
  m_profiler.clear();
  m_printf.clear();
  m_layer_counter.clear();
  return ret;
}

aipu_status_t JobV3X::init(const aipu_global_config_simulation_t *cfg,
                           const aipu_global_config_hw_t *hw_cfg) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;

  if (get_graph().is_dynamic_shape()) {
    /**
     * it has to check whether dynamic input shape is
     * set correctly here.
     */
    if (m_dyn_shape->is_set_dyn_shape_true() == false) {
      ret = AIPU_STATUS_ERROR_SET_SHAPE_FAILED;
      goto finish;
    }
  }

  m_cfg = cfg;
  m_hw_cfg = hw_cfg;

  m_dev->get_core_count(m_partition_id, 0, &m_core_cnt);
  set_job_params(get_graph().get_subgraph_cnt(),
                 m_dev->tec_cnt_per_core(m_partition_id),
                 get_graph().m_remap_flag, m_core_cnt);

  if (m_dev->get_grid_id(m_grid_id) < 0) {
    ret = AIPU_STATUS_ERROR_ALLOC_GRIP_ID;
    goto finish;
  }

  init_group_id(m_sg_cnt);

  /* allocate and load job buffers */
  ret = alloc_load_job_buffers();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  /**
   * no need to create TCBs if there's no subgraphs,
   * just directly return.
   */
  if (get_subgraph_cnt() == 0) {
    m_status = AIPU_JOB_STATUS_INIT;
    goto finish;
  }

  ret = setup_tcb_chain();
  if (ret != AIPU_STATUS_SUCCESS)
    goto finish;

  if (m_backup_tcb != nullptr)
    m_mem->read(m_init_tcb.pa, m_backup_tcb.get(),
                m_tot_tcb_cnt * sizeof(tcb_t));

finish:
  return ret;
}

aipu_status_t JobV3X::schedule() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  JobDesc desc;

  ret = validate_schedule_status();
  if (ret != AIPU_STATUS_SUCCESS) {
    LOG(LOG_ERR, "Job state %d is invalid", m_status);
    return ret;
  }

  if (get_subgraph_cnt() == 0) {
    LOG(LOG_WARN, "Subgraph counter is 0, and it will not dispatch");
    m_status = AIPU_JOB_STATUS_DONE;
    return AIPU_STATUS_SUCCESS;
  }

  if (get_graph().m_text->size == 0) {
    LOG(LOG_WARN, "Graph text size is 0, and it will not dispatch");
    m_status = AIPU_JOB_STATUS_DONE;
    return AIPU_STATUS_SUCCESS;
  }

  /* with the backup tcbchain if run the job again */
  if (m_backup_tcb != nullptr && m_backup_tcb_used == true)
    m_mem->write(m_init_tcb.pa, m_backup_tcb.get(),
                 m_tot_tcb_cnt * sizeof(tcb_t));
  m_backup_tcb_used = true;

  dump_job_shared_buffers();
  dump_job_private_buffers(*m_rodata, m_descriptor);
  dump_specific_buffers();
  ret = dump_for_emulation();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  memset(&desc.kdesc, 0, sizeof(desc.kdesc));

  /* for simulation */
  desc.kdesc.job_id = m_id;
  desc.kdesc.version_compatible = !get_graph().m_do_vcheck;
  desc.kdesc.aipu_config = get_graph().m_hw_config;
  desc.jobbase = this;
  desc.tcb_number = m_tot_tcb_cnt;
  desc.tcb_head = m_init_tcb.pa;
  desc.tcb_tail =
      m_init_tcb.pa +
      (m_tot_tcb_cnt - 1) * sizeof(tcb_t); /* last task tcb pa for sim */

  /* for HW */
  desc.kdesc.exec_flag = (m_qos == AIPU_JOB_QOS_HIGH)
                             ? AIPU_JOB_EXEC_FLAG_QOS_FAST
                             : AIPU_JOB_EXEC_FLAG_QOS_SLOW;
  desc.kdesc.core_id = 0;
  if (m_dbg_dispatch) {
    desc.kdesc.exec_flag |= AIPU_JOB_EXEC_FLAG_DBG_DISPATCH;
    desc.kdesc.core_id = m_core_id;
  }
  desc.kdesc.enable_poll_opt = !m_hw_cfg->poll_in_commit_thread;
  desc.kdesc.profile_fd = m_profile_fd;
  if (m_profiler.size() > 0) {
    desc.kdesc.profile_pa = m_profiler[0].pa;
    desc.kdesc.profile_sz = m_profiler[0].size;
  }
  desc.kdesc.aipu_version = get_graph().m_hw_version;
  desc.kdesc.partition_id = m_partition_id;
  desc.kdesc.exec_flag |= (m_segmmu_num > 0) ? AIPU_JOB_EXEC_FLAG_SEG_MMU : 0;
  desc.kdesc.exec_flag |= (m_sg_cnt == 1) ? AIPU_JOB_EXEC_FLAG_SINGLE_GROUP
                                          : AIPU_JOB_EXEC_FLAG_MULTI_GROUP;

  desc.kdesc.head_tcb_pa = m_init_tcb.pa;
  desc.kdesc.tail_tcb_pa = m_init_tcb.pa + (m_tot_tcb_cnt - 1) * sizeof(tcb_t);
  desc.kdesc.first_task_tcb_pa = m_init_tcb.pa + sizeof(tcb_t);
  desc.kdesc.last_task_tcb_pa =
      m_init_tcb.pa + (m_tot_tcb_cnt - 1) * sizeof(tcb_t);

  /* for debugger */
  desc.kdesc.is_defer_run = m_is_defer_run;
  desc.kdesc.do_trigger = m_do_trigger;
  desc.kdesc.is_coredump_en = false;
  if (m_coredump != nullptr && m_coredump->is_initialized())
    desc.kdesc.is_coredump_en = true;

  ret = m_dev->schedule(desc);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if ((m_is_defer_run == true) && (m_do_trigger == false))
    m_status = AIPU_JOB_STATUS_BIND;
  else
    m_status = AIPU_JOB_STATUS_SCHED;

  return ret;
}

aipu_status_t JobV3X::destroy() { return free_job_buffers(); }

void JobV3X::dump_specific_buffers() {
  DEV_PA_64 dump_pa;
  uint32_t dump_size;

  if (m_dump_tcb) {
    dump_pa = m_tcbs->pa;
    dump_size = m_tot_tcb_cnt * sizeof(tcb_t);
    if (dump_size != 0)
      dump_buffer(dump_pa, nullptr, dump_size, "TCBs");
  }

  if (m_dump_profile && m_profiler.size() > 0) {
    std::string profile_file_name =
        m_dump_dir + "/" + m_dump_misc_prefix + "_PerfData.bin";
    m_profile_fd = open(profile_file_name.c_str(), O_RDWR | O_CREAT, 0644);
    if (m_profile_fd < 0)
      LOG(LOG_ALERT, "open: %s [fail], ret: %d", profile_file_name.c_str(),
          m_profile_fd);
    else
      chmod(profile_file_name.c_str(), 0644);

    convert_ll_status(m_dev->ioctl_cmd(AIPU_IOCTL_ENABLE_TICKCOUNTER, nullptr));
  }
}

aipu_status_t JobV3X::bind_core(uint32_t partition_id) {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint32_t partition_cnt = 0;

  ret = m_dev->get_partition_count(&partition_cnt);
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  if (partition_id >= partition_cnt)
    return AIPU_STATUS_ERROR_INVALID_PARTITION_ID;

  ret = validate_schedule_status();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  m_is_defer_run = true;
  m_do_trigger = false;
  m_partition_id = partition_id;
  ret = schedule();

  return ret;
}

aipu_status_t JobV3X::debugger_run() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  aipu_job_status_t status = AIPU_JOB_STATUS_NO_STATUS;

  if (m_status != AIPU_JOB_STATUS_BIND)
    return AIPU_STATUS_ERROR_INVALID_OP;

  m_is_defer_run = true;
  m_do_trigger = true;
  ret = schedule();
  if (ret != AIPU_STATUS_SUCCESS)
    return ret;

  ret = get_status_blocking(&status, -1);
  if ((AIPU_STATUS_SUCCESS == ret) && (AIPU_JOB_STATUS_DONE != status))
    ret = AIPU_STATUS_ERROR_JOB_EXCEPTION;

  return ret;
}

aipu_status_t JobV3X::parse_dynamic_out_shape() {
  aipu_status_t ret = AIPU_STATUS_SUCCESS;
  uint64_t size = 0;
  uint32_t data[96] = {0};

  if (get_graph().is_dynamic_shape() && m_dyn_shape->is_set_dyn_shape_true() &&
      (m_dyn_shape->get_config_shape_sz() == m_inputs.size())) {
    if (!m_dyn_shape->testset_dynamic_out_shape_updated()) {
      if (m_outputs_shape.size() != m_outputs.size()) {
        ret = AIPU_STATUS_ERROR_UNMATCH_OUT_SHAPE;
        LOG(LOG_ERR, "DS out tensor cnt != Original out tensor cnt");
        goto out;
      }

      for (uint32_t i = 0; i < m_outputs_shape.size(); i++) {
        m_mem->read(m_outputs_shape[i].pa, (char *)data,
                    m_outputs_shape[i].size);
        size = 1;
        for (uint32_t j = 0; j < m_outputs_shape[i].size / 4; j++) {
          LOG(LOG_INFO, "output idx: %u, shape idx: %u, dim: %u", i, j,
              data[j]);
          size *= data[j];
        }

        if (size == 0) {
          m_dyn_shape->clear_config_out_tensor_size();
          ret = AIPU_STATUS_ERROR_ZERO_TENSOR_SIZE;
          LOG(LOG_ERR, "Invalid dynamic out shape %d: size (0)", i);
          goto out;
        }

        if (m_outputs[i].type == AIPU_DATA_TYPE_U16 ||
            m_outputs[i].type == AIPU_DATA_TYPE_S16 ||
            m_outputs[i].type == AIPU_DATA_TYPE_F16 ||
            m_outputs[i].type == AIPU_DATA_TYPE_BF16)
          size <<= 1;
        else if (m_outputs[i].type == AIPU_DATA_TYPE_U32 ||
                 m_outputs[i].type == AIPU_DATA_TYPE_S32 ||
                 m_outputs[i].type == AIPU_DATA_TYPE_F32)
          size <<= 2;

        m_dyn_shape->set_config_out_tensor_size(i, size);
      }

      m_dyn_shape->update_dynamic_io_tensor_size(AIPU_TENSOR_TYPE_OUTPUT);
      update_single_io_buffers(get_graph().get_bss(0).io.outputs, m_outputs,
                               m_bss_buffer_vec[0].reuses);
    }
  }

out:
  return ret;
}
} // namespace aipudrv