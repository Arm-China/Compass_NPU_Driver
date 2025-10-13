
#!/bin/env python3
import os
import sys
import copy
import numpy as np
from common.helper import *
from common.log import *

#
# sgsf_flush.py:
# 	this script runs job on hardware with async mode
#
# usage:
#   python3 sgsf_flush.py  -s /home/benchmark/resnet50 -l bin/sim/debug/ -e ./aipu_simulator_x1 -d ./output
#
# note:
#   resnet50 {aipu.bin, input0.bin, output.bin}
#   bin/sim/debug/: libaipudrv.so path
#

# global switch
LOAD_FROM_FILE_FLAG = False

log = Log(EM_LOG_TYPE_ERR | EM_LOG_TYPE_ALT | EM_LOG_TYPE_WAR | EM_LOG_TYPE_INF)
parseCmdline_obj = Parse_Cmdline(log)
helper_obj = Help(log)

frame_cnt = 2
# run here to ensure libaipudrv.so's path is added after cmdline arguments parsed
from libaipudrv import *  # noqa


def sgsf_flush():
    context_init = False
    graph_load = False

    npu = NPU()
    is_hw = hw_env_check(log)
    try:
        ret = npu.aipu_init_context()
        if ret != AIPU_STATUS_SUCCESS:
            raise RuntimeError(f'aipu_init_context [fail], err: {npu.aipu_get_error_message(ret)}')
        context_init = True
        log.info('aipu_init_context [ok]')

        if is_hw is False:
            global_cfg = aipu_global_config_simulation_t()
            global_cfg.simulator = parseCmdline_obj.m_simulator
            global_cfg.log_file_path = parseCmdline_obj.m_dump_path
            ret = npu.aipu_config_global(AIPU_CONFIG_TYPE_SIMULATION, global_cfg)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_config_global [fail], err: {npu.aipu_get_error_message(ret)}')
            log.debug('aipu_config_global [ok]')

        # loop all benchmarks
        for benchmark in parseCmdline_obj.m_benchmarks_list:
            log.debug(f'Test: <{benchmark["model"]}>')

            aipu_bin = benchmark["model"]
            check_bin = benchmark["check_bin"]
            check_bin_size = benchmark["check_bin_size"]
            input_bins = benchmark["input_bins"]

            # load way1: from aipu.bin path
            load_cfg = aipu_load_graph_cfg_t()
            load_cfg.extra_weight_path = parseCmdline_obj.m_extra_weight_path
            if LOAD_FROM_FILE_FLAG:
                ret, grpah_id = npu.aipu_load_graph_from_file(aipu_bin, load_cfg)
            else:
                # load way2: from aipu.bin's data buffer and size
                with open(aipu_bin, mode='rb') as filp:
                    graph_data = filp.read()
                    graph_len = len(graph_data)
                    ret, graph_id = npu.aipu_load_graph(graph_data, graph_len, load_cfg)

            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_load_graph [fail], err: {npu.aipu_get_error_message(ret)}')
            graph_load = True
            log.info(f'load model: {aipu_bin}, {graph_id:x}')

            ret, input_cnt = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_INPUT)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_get_tensor_count [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info(f'aipu_get_tensor_count [ok], input cnt: {input_cnt}')

            for i in range(input_cnt):
                input_desc = npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_INPUT, i)
                log.info(f'input {i}: {input_desc.id}, size: {input_desc.size}')

            ret, output_cnt = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_OUTPUT)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_get_tensor_count [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info(f'aipu_get_tensor_count [ok], output cnt: {output_cnt}')

            output_desc = []
            for i in range(output_cnt):
                output_desc.append(npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_OUTPUT, i))
                log.info(f'output {i}: {output_desc[i].id}, size: {output_desc[i].size}')

            job_id_list = []
            for frame in range(frame_cnt):
                job_cfg = aipu_create_job_cfg_t()
                ret, job_id = npu.aipu_create_job(graph_id, job_cfg)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(f'aipu_create_job [fail], err: {npu.aipu_get_error_message(ret)}')
                job_id_list.append(job_id)
                log.info(f'aipu_create_job [ok] job_id: {job_id:x}')

                # config dump path
                if os.path.isdir(parseCmdline_obj.m_dump_path):
                    cfg = aipu_job_config_dump_t()
                    cfg.dump_dir = parseCmdline_obj.m_dump_path
                    ret = npu.aipu_config_job(job_id, AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, cfg)
                    if ret != AIPU_STATUS_SUCCESS:
                        raise RuntimeError(f'aipu_config_job [fail], err: {npu.aipu_get_error_message(ret)}')
                    log.info('aipu_config_job [ok]')

                # config v1v2 simulator
                if is_hw is False:
                    # v1&v2 temp.* directory
                    cfg = aipu_job_config_simulation_t()
                    cfg.data_dir = "./" if len(parseCmdline_obj.m_dump_path) == 0 else parseCmdline_obj.m_dump_path
                    ret = npu.aipu_config_job(job_id, AIPU_CONFIG_TYPE_SIMULATION, cfg)
                    if ret != AIPU_STATUS_SUCCESS:
                        raise RuntimeError(
                            f'aipu_config_job v1v2 simulator [fail], err: {npu.aipu_get_error_message(ret)}')
                    log.info('aipu_config_job v1v2 simulator [ok]')

                if LOAD_FROM_FILE_FLAG:  # load 2-1
                    for i in range(len(input_bins)):
                        ret = npu.aipu_load_tensor_from_file(job_id, i, input_bins[i])
                        if ret != AIPU_STATUS_SUCCESS:
                            raise RuntimeError(f'aipu_load_tensor file [fail], err: {npu.aipu_get_error_message(ret)}')
                        log.info(f'load input: {input_bins[i]}')
                else:  # load 2-2
                    for i in range(len(input_bins)):
                        with open(input_bins[i], mode='rb') as filp:
                            input_data = filp.read()
                            ret = npu.aipu_load_tensor(job_id, i, input_data)
                            if ret != AIPU_STATUS_SUCCESS:
                                raise RuntimeError(f'aipu_load_tensor [fail], err: {npu.aipu_get_error_message(ret)}')
                            log.info(f'load file: {input_bins[i]}')

            job_id_list_realtime = copy.deepcopy(job_id_list)
            for frame in range(frame_cnt):
                job_id = job_id_list[frame]
                ret = npu.aipu_flush_job(job_id, None)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(f'aipu_flush_job [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info(f'aipu_flush_job [ok], job id: {job_id:x}')

                status = AIPU_JOB_STATUS_NO_STATUS
                while status == AIPU_JOB_STATUS_NO_STATUS:
                    ret, status = npu.aipu_get_job_status(job_id, -1)
                    log.debug("poll status...")

                outputs_data = []
                if status == AIPU_JOB_STATUS_EXCEPTION:
                    raise RuntimeError(f'aipu_get_job_status: {job_id} <exception>')
                elif status == AIPU_JOB_STATUS_DONE:
                    log.info(f'aipu_get_job_status: {job_id:x}, done')
                    for i in range(output_cnt):
                        ret, output_data = npu.aipu_get_tensor(job_id, AIPU_TENSOR_TYPE_OUTPUT, i)
                        if ret != AIPU_STATUS_SUCCESS:  # note the return value checking for this API
                            raise RuntimeError(f'aipu_flush_job [fail], err: {npu.aipu_get_error_message(ret)}')
                        outputs_data.append(output_data)

                    helper_obj.check_result_helper(outputs_data, output_desc, check_bin, check_bin_size)

                ret = npu.aipu_clean_job(job_id)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(f'aipu_clean_job [fail], err: {npu.aipu_get_error_message(ret)}')
                if job_id in job_id_list_realtime:
                    job_id_list_realtime.remove(job_id)
                log.info(f'aipu_clean_job [ok], job id: {job_id:x}')

            ret = npu.aipu_unload_graph(graph_id)
            if ret != AIPU_STATUS_SUCCESS:
                errmsg = npu.aipu_get_error_message(ret)
                raise RuntimeError(f'aipu_unload_graph [fail], err: {errmsg}')
            graph_load = False
            log.info(f'aipu_unload_graph [ok]')

    finally:
        for job_id in job_id_list_realtime:
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
    sgsf_flush()
