
#!/bin/env python3
import os
import sys
import numpy as np
import mmap
from enum import IntEnum
from common.helper import *
from common.log import *

#
# sgsf_finish.py:
#   this script runs job with sync mode.
#
# usage:
#   python3 sgsf_finish.py -s /home/benchmark/resnet50 -l bin/sim/debug/ -e ./aipu_simulator_x1 -d ./output -a X3P_1304 [-p]
#    -e: only for v1v2 simulator
#    -a: only for >=v3 simulator
#
# feature:
#   1.support simulator
#   2.support hardware
#   3.support profiler simulator/hardware
#   4.support share-buf
#   5.support dma-buf (only for hardware)
#
# note:
#   resnet50 {aipu.bin, input0.bin, output.bin}
#   bin/sim/debug/: libaipudrv.so path
#   simulator profiler enable/disable only affects at initialization phase
#


class RunMode(IntEnum):
    Normal = 0
    ShareBuf = 1
    DMABuf = 2  # only for hardware


# global switch
LOAD_FROM_FILE_FLAG = False
runmode = RunMode.Normal

log = Log(EM_LOG_TYPE_ERR | EM_LOG_TYPE_ALT | EM_LOG_TYPE_WAR | EM_LOG_TYPE_INF)
opt = Parse_Cmdline(log)
helper = Help(log)

# run here to ensure libaipudrv.so's path is added after cmdline arguments parsed
from libaipudrv import *  # noqa


def sgsf_finish():
    context_init = False
    graph_load = False
    job_create = False

    npu = NPU()
    is_hw = hw_env_check(log)
    try:
        ret = npu.aipu_init_context()
        if ret != AIPU_STATUS_SUCCESS:
            raise RuntimeError(f'aipu_init_context [fail], err: {npu.aipu_get_error_message(ret)}')
        context_init = True
        log.info('aipu_init_context [ok]')

        if is_hw and opt.m_profile_en:
            ret, = npu.aipu_ioctl(AIPU_IOCTL_ENABLE_TICKCOUNTER)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(
                    f'aipu_ioctl(AIPU_IOCTL_ENABLE_TICKCOUNTER) [fail], err: {npu.aipu_get_error_message(ret)}')
        elif is_hw is False:
            global_cfg = aipu_global_config_simulation_t()
            global_cfg.simulator = opt.m_simulator
            global_cfg.log_file_path = opt.m_dump_path

            # driver will load profiler enable flag according NPU target
            global_cfg.en_eval = True if opt.m_profile_en else False
            global_cfg.en_fast_perf = True if opt.m_profile_en else False
            global_cfg.freq_mhz = 1000
            global_cfg.ddr_latency_rd = 0
            global_cfg.ddr_latency_wr = 0
            global_cfg.ddr_bw = 512
            global_cfg.ddr_bw_ratio = 1
            if len(opt.m_dump_path) == 0:
                global_cfg.perf_report = "./perf.csv"
            else:
                global_cfg.perf_report = os.path.join(opt.m_dump_path, "perf.csv")

            ret = npu.aipu_config_global(AIPU_CONFIG_TYPE_SIMULATION, global_cfg)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_config_global [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info('aipu_config_global [ok]')

        # print version
        ret, umd, kmd = npu.aipu_ioctl(AIPU_IOCTL_GET_VERSION)
        if ret != AIPU_STATUS_SUCCESS:
            raise RuntimeError(f'aipu_ioctl(AIPU_IOCTL_GET_VERSION) [fail], err: {npu.aipu_get_error_message(ret)}')
        log.info(f"umd version {umd}" + (f", kmd version {kmd}" if is_hw else ", kmd has no version in simulator"))

        # loop all benchmarks
        for benchmark in opt.m_benchmarks_list:
            log.debug(f'Test: <{benchmark["model"]}>')

            aipu_bin = benchmark["model"]
            check_bin = benchmark["check_bin"]
            check_bin_size = benchmark["check_bin_size"]
            input_bins = benchmark["input_bins"]

            # load way1: from aipu.bin path
            load_cfg = aipu_load_graph_cfg_t()
            load_cfg.extra_weight_path = opt.m_extra_weight_path
            if LOAD_FROM_FILE_FLAG:
                ret, graph_id = npu.aipu_load_graph_from_file(aipu_bin, load_cfg)
            else:
                # load way2: from aipu.bin's data buffer and size
                graph_data = np.fromfile(aipu_bin, dtype=np.uint8)
                ret, graph_id = npu.aipu_load_graph(graph_data, graph_data.size, load_cfg)

            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_load_graph [fail], err: {npu.aipu_get_error_message(ret)}')
            graph_load = True
            log.info(f'load model: {aipu_bin}, {graph_id:x}')

            _, bin_ver = npu.aipu_ioctl(AIPU_IOCTL_GET_AIPUBIN_BUILDVERSION, graph_id=graph_id)
            log.info(f"aipu.bin version {bin_ver & 0xff}.{(bin_ver >> 8) & 0xff}.{(bin_ver >> 16) & 0xffff}")

            ret, input_cnt = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_INPUT)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_get_tensor_count [fail], err: {npu.aipu_get_error_message(ret)}')

            # dma-buf
            dmabuf_fds = []
            pas = []
            if is_hw and runmode == RunMode.DMABuf:
                for i in range(input_cnt):
                    desc = npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_INPUT, i)

                    ret, dmabuf_fd = npu.aipu_ioctl(AIPU_IOCTL_ALLOC_DMABUF, bytes=desc.size)
                    if ret != AIPU_STATUS_SUCCESS:
                        raise RuntimeError(
                            f'aipu_ioctl(AIPU_IOCTL_ALLOC_DMABUF) [fail], err: {npu.aipu_get_error_message(ret)}')
                    log.info(f"dma-buf fd {dmabuf_fd}")
                    dmabuf_fds.append(dmabuf_fd)

                    ret, pa, size = npu.aipu_ioctl(AIPU_IOCTL_GET_DMABUF_INFO, dmabuf_fd=dmabuf_fd)
                    if ret != AIPU_STATUS_SUCCESS:
                        raise RuntimeError(
                            f'aipu_ioctl(AIPU_IOCTL_GET_DMABUF_INFO) [fail], err: {npu.aipu_get_error_message(ret)}')
                    log.info(f"dma-buf pa {pa:x}")
                    pas.append(pa)

                    # load data to dma-buf
                    with open(input_bins[i], mode='rb') as filp:
                        input_data = filp.read()
                        mm = mmap.mmap(fileno=dmabuf_fd, length=desc.size, flags=mmap.MAP_SHARED,
                                       prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=0)
                        mm.write(input_data)
                        mm.close()

            # share-buf
            sharebuf_pas = []
            sharebuf_arrays = []
            if runmode == RunMode.ShareBuf:
                for i in range(input_cnt):
                    desc = npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_INPUT, i)
                    ret, pa, arr = npu.aipu_ioctl(AIPU_IOCTL_ALLOC_SHARE_BUF, size=desc.size)

                    # load data to share-buf
                    with open(input_bins[i], mode='rb') as filp:
                        input_data = np.fromfile(filp, dtype=np.uint8, count=desc.size)
                        arr[:] = input_data

                    sharebuf_pas.append(pa)
                    sharebuf_arrays.append(arr)

            job_cfg = aipu_create_job_cfg_t()
            ret, job_id = npu.aipu_create_job(graph_id, job_cfg)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_create_job [fail], err: {npu.aipu_get_error_message(ret)}')
            job_create = True
            log.debug('aipu_create_job [ok]')

            if is_hw is False:
                # profile
                # ret,profile_cnt = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_PROFILER)
                # if ret != AIPU_STATUS_SUCCESS:
                #     raise RuntimeError(f'aipu_get_tensor_count [fail], err: {npu.aipu_get_error_message(ret)}')

                # if opt.m_profile_en is False:
                #     npu.aipu_ioctl(AIPU_IOCTL_SET_PROFILE, set_profile=0)
                # log.info((f'enable' if opt.m_profile_en else 'disable') + f' profile, and aipu.bin profile cnt {profile_cnt}')

                # v1&v2 temp.* directory
                cfg = aipu_job_config_simulation_t()
                cfg.data_dir = "./" if len(opt.m_dump_path) == 0 else opt.m_dump_path
                ret = npu.aipu_config_job(job_id, AIPU_CONFIG_TYPE_SIMULATION, cfg)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(f'aipu_config_job v1v2 simulator [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info('aipu_config_job only for v1v2 simulator [ok]')

            if os.path.isdir(opt.m_dump_path):
                cfg = aipu_job_config_dump_t()
                cfg.dump_dir = opt.m_dump_path
                cfg_types = (int(AIPU_JOB_CONFIG_TYPE_DUMP_TEXT) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_WEIGHT) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_RODATA) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_DESCRIPTOR) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_INPUT) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_TCB_CHAIN) |
                             int(AIPU_JOB_CONFIG_TYPE_DUMP_EMULATION))
                ret = npu.aipu_config_job(job_id, cfg_types, cfg)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(f'aipu_config_job dump [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info(f'aipu_config_job dump [ok]')

            if runmode == RunMode.Normal:
                if LOAD_FROM_FILE_FLAG:  # load 2-1
                    for i in range(len(input_bins)):
                        ret = npu.aipu_load_tensor_from_file(job_id, i, input_bins[i])
                        if ret != AIPU_STATUS_SUCCESS:
                            raise RuntimeError(f'aipu_load_tensor file [fail], err: {npu.aipu_get_error_message(ret)}')
                        log.info(f'load input: {input_bins[i]}')
                else:  # load 2-2
                    input_data = []
                    for i in range(len(input_bins)):
                        with open(input_bins[i], mode='rb') as filp:
                            input_data = filp.read()
                            ret = npu.aipu_load_tensor(job_id, i, input_data)
                            if ret != AIPU_STATUS_SUCCESS:
                                raise RuntimeError(f'aipu_load_tensor [fail], err: {npu.aipu_get_error_message(ret)}')
                            log.info(f'load input: {input_bins[i]}')
                log.info(f'running in Normal mode')
            elif runmode == RunMode.DMABuf:
                if not is_hw:
                    raise RuntimeError(f'dma-buf can only use on hardware')

                # configuration
                for i in range(input_cnt):
                    shared_tensor = aipu_shared_tensor_info_t()
                    shared_tensor.tensor_idx = i
                    shared_tensor.type = AIPU_TENSOR_TYPE_INPUT
                    shared_tensor.pa = pas[i]
                    shared_tensor.dmabuf_fd = dmabuf_fds[i]
                    shared_tensor.offset_in_dmabuf = 0
                    shared_tensor.shared_case_type = AIPU_SHARE_BUF_DMABUF
                    ret = npu.aipu_specify_iobuf(job_id, shared_tensor)
                    if ret != AIPU_STATUS_SUCCESS:
                        raise RuntimeError(f'aipu_specify_iobuf [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info(f'running in DMABuf mode')
            elif runmode == RunMode.ShareBuf:
                for i in range(input_cnt):
                    shared_tensor = aipu_shared_tensor_info_t()
                    shared_tensor.tensor_idx = i
                    shared_tensor.type = AIPU_TENSOR_TYPE_INPUT
                    shared_tensor.pa = sharebuf_pas[i]
                    shared_tensor.shared_case_type = AIPU_SHARE_BUF_IN_ONE_PROCESS
                    ret = npu.aipu_specify_iobuf(job_id, shared_tensor)
                    if ret != AIPU_STATUS_SUCCESS:
                        raise RuntimeError(f'aipu_specify_iobuf [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info(f'running in ShareBuf mode')

            ret = npu.aipu_finish_job(job_id, -1)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_finish_job [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info(f'aipu_finish_job [ok]')

            if opt.m_profile_en:
                ret, num = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_PROFILER)
                if ret != AIPU_STATUS_SUCCESS or num == 0:
                    raise RuntimeError(
                        f'enable profiler, but profiler tensor number is 0, or ret(fail): {npu.aipu_get_error_message(ret)}')
                ret, profile_data = npu.aipu_get_tensor(job_id, AIPU_TENSOR_TYPE_PROFILER, 0)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(f'aipu_get_tensor [fail], err: {npu.aipu_get_error_message(ret)}')
                profile_path = "./PerfData.bin" if len(
                    opt.m_dump_path) == 0 else os.path.join(opt.m_dump_path, "PerfData.bin")
                with open(profile_path, "wb") as f:
                    f.write(profile_data)

            ret, output_cnt = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_OUTPUT)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_get_tensor_count [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info(f'aipu_get_tensor_count [ok], output cnt: {output_cnt}')

            outputs_data = []
            for i in range(output_cnt):
                ret, output_data = npu.aipu_get_tensor(job_id, AIPU_TENSOR_TYPE_OUTPUT, i)
                if ret != AIPU_STATUS_SUCCESS:  # note the return value checking for this API
                    raise RuntimeError(f'aipu_get_tensor [fail], err: {npu.aipu_get_error_message(ret)}')
                outputs_data.append(output_data)

            output_desc = []
            for i in range(output_cnt):
                output_desc.append(npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_OUTPUT, i))
                log.info(f'output {i}: {output_desc[i].id}, size: {output_desc[i].size}')

            helper.check_result_helper(outputs_data, output_desc, check_bin, check_bin_size)

            ret = npu.aipu_clean_job(job_id)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_clean_job [fail], err: {npu.aipu_get_error_message(ret)}')
            job_create = False
            log.info(f'aipu_clean_job [ok]')

            ret = npu.aipu_unload_graph(graph_id)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_unload_graph [fail], err: {npu.aipu_get_error_message(ret)}')
            graph_load = False
            log.info(f'aipu_unload_graph [ok]')

    finally:
        if is_hw and opt.m_profile_en:
            ret, = npu.aipu_ioctl(AIPU_IOCTL_DISABLE_TICKCOUNTER)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(
                    f'aipu_ioctl(AIPU_IOCTL_DISABLE_TICKCOUNTER) [fail], err: {npu.aipu_get_error_message(ret)}')

        if is_hw and runmode == RunMode.DMABuf:
            for i in range(input_cnt):
                ret, = npu.aipu_ioctl(AIPU_IOCTL_FREE_DMABUF, dmabuf_fd=dmabuf_fds[i])
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(
                        f'aipu_ioctl(AIPU_IOCTL_FREE_DMABUF) [fail], err: {npu.aipu_get_error_message(ret)}')

        if runmode == RunMode.ShareBuf:
            for i in range(input_cnt):
                ret, = npu.aipu_ioctl(AIPU_IOCTL_FREE_SHARE_BUF, pa=sharebuf_pas[i], size=desc.size)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(
                        f'aipu_ioctl(AIPU_IOCTL_FREE_SHARE_BUF) [fail], err: {npu.aipu_get_error_message(ret)}')

        if job_create is True:
            ret = npu.aipu_clean_job(job_id)
            if ret != AIPU_STATUS_SUCCESS:
                log.error(f'aipu_clean_job [fail], err: {npu.aipu_get_error_message(ret)}')
            else:
                log.info(f'aipu_clean_job [ok]')
        if graph_load is True:
            ret = npu.aipu_unload_graph(graph_id)
            if ret != AIPU_STATUS_SUCCESS:
                log.error(f'aipu_unload_graph [fail], err: {npu.aipu_get_error_message(ret)}')
            else:
                log.info(f'aipu_unload_graph [ok]')
        if context_init is True:
            ret = npu.aipu_deinit_context()
            if ret != AIPU_STATUS_SUCCESS:
                log.error(f'aipu_deinit_context [fail], err: {npu.aipu_get_error_message(ret)}')
            else:
                log.info(f'aipu_deinit_context [ok]')


if __name__ == "__main__":
    sgsf_finish()
