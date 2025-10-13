
#!/bin/env python3
import os
import sys
import struct
import numpy as np
from common.helper import *
from common.log import *

#
# dyshape.py:
# 	this script runs dynamic shape job with sync mode, only >=v3 supports dynamic shape
#
# usage:
#   python3 dyshape.py -s /home/benchmark/resnet50 -l bin/juno/debug/ -r "1,1,1,1024/1,1/1,1,128,128" -a X3P_1304
#    -a: only for >=v3
#
# note:
#   resnet50 {aipu.bin, input0.bin, output.bin}
#   bin/juno/debug/: libaipudrv.so path
#   "1,1,1,1024/1,1/1,1,128,128": input shape to config

# global switch
LOAD_FROM_FILE_FLAG = False

log = Log(EM_LOG_TYPE_ERR | EM_LOG_TYPE_ALT | EM_LOG_TYPE_WAR | EM_LOG_TYPE_INF)
parseCmdline_obj = Parse_Cmdline(log)
helper_obj = Help(log)

# run here to ensure libaipudrv.so's path is added after cmdline arguments parsed
from libaipudrv import *  # noqa


def dynshape():
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

        if is_hw is False:
            global_cfg = aipu_global_config_simulation_t()
            global_cfg.log_file_path = parseCmdline_obj.m_dump_path
            ret = npu.aipu_config_global(AIPU_CONFIG_TYPE_SIMULATION, global_cfg)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_config_global [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info('aipu_config_global [ok]')

        # loop all benchmarks
        for benchmark in parseCmdline_obj.m_benchmarks_list:
            log.debug(f'Test: <{benchmark["model"]}>')

            aipu_bin = benchmark["model"]
            check_bin = benchmark["check_bin"]
            check_bin_size = benchmark["check_bin_size"]
            input_bins = benchmark["input_bins"]

            load_cfg = aipu_load_graph_cfg_t()
            load_cfg.extra_weight_path = parseCmdline_obj.m_extra_weight_path
            # load way1: from aipu.bin path
            if LOAD_FROM_FILE_FLAG:
                ret, graph_id = npu.aipu_load_graph_from_file(aipu_bin, load_cfg)
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
            log.info(f'aipu_get_tensor_count [ok], inpput cnt: {input_cnt}')

            for i in range(input_cnt):
                ret, rank = npu.aipu_ioctl(AIPU_IOCTL_GET_DS_RANK, graph_id=graph_id, ds_idx=i)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(
                        f'aipu_ioctl(AIPU_IOCTL_GET_DS_RANK) [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info(f'input {i} rank {rank}')

                ret, min_shape, max_shape = npu.aipu_ioctl(
                    AIPU_IOCTL_GET_DS_DIM_CONSTRAINT, graph_id=graph_id, ds_idx=i)
                if ret != AIPU_STATUS_SUCCESS:
                    raise RuntimeError(
                        f'aipu_ioctl(AIPU_IOCTL_GET_DS_DIM_CONSTRAINT) [fail], err: {npu.aipu_get_error_message(ret)}')
                log.info(f'input {i} min shape {min_shape} max shape {max_shape}')

            ret, output_cnt = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_OUTPUT)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_get_tensor_count [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info(f'aipu_get_tensor_count [ok], output cnt: {output_cnt}')

            output_desc = []
            for i in range(output_cnt):
                output_desc.append(npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_OUTPUT, i))
                log.info(f'output {i}: {output_desc[i].id}, size: {output_desc[i].size}')

            job_cfg = aipu_create_job_cfg_t()
            job_cfg.dynshape = parseCmdline_obj.m_input_shape
            log.info(f"input shape setting: {job_cfg.dynshape}")
            ret, job_id = npu.aipu_create_job(graph_id, job_cfg)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_create_job [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info('aipu_create_job [ok]')

            # config dump path
            if os.path.isdir(parseCmdline_obj.m_dump_path):
                cfg = aipu_job_config_dump_t()
                cfg.dump_dir = parseCmdline_obj.m_dump_path
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
                    errmsg = npu.aipu_get_error_message(ret)
                    raise RuntimeError(f'aipu_config_job [fail], err: {pu.aipu_get_error_message(ret)}')
                log.info('aipu_config_job [ok]')

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

            # used for same job with different dynamic shape
            dyn_param = aipu_dynshape_param_t()
            dyn_param.shape_items = parseCmdline_obj.m_input_shape
            ret = npu.aipu_config_job(job_id, AIPU_JOB_CONFIG_TYPE_DYNAMIC_PARAMS, dyn_param)

            ret = npu.aipu_finish_job(job_id, -1)
            if ret != AIPU_STATUS_SUCCESS:
                raise RuntimeError(f'aipu_finish_job [fail], err: {npu.aipu_get_error_message(ret)}')
            log.info(f'aipu_finish_job [ok]')

            # outputs_shape = []
            # for i in range(output_cnt):
            #     ret, output_data = npu.aipu_get_tensor(job_id, AIPU_TENSOR_TYPE_OUT_TENSOR_SHAPE, i)
            #     if ret != AIPU_STATUS_SUCCESS:  # note the return value checking for this API
            #         raise RuntimeError(f'aipu_get_tensor [fail], err: {npu.aipu_get_error_message(ret)}')
            #     # type same with input
            #     outputs_shape.append(struct.unpack('I', output_data))
            #     # if isinstance(output_data, bytes) or isinstance(output_data, bytearray):
            #     # else if isinstance(output_data, np.ndarray):
            #     # outputs_shape.append(output_data)
            # log.info(f'output shape: {outputs_shape}')

            outputs_data = []
            for i in range(output_cnt):
                ret, output_data = npu.aipu_get_tensor(job_id, AIPU_TENSOR_TYPE_OUTPUT, i)
                if ret != AIPU_STATUS_SUCCESS:  # note the return value checking for this API
                    raise RuntimeError(f'aipu_get_tensor [fail], err: {npu.aipu_get_error_message(ret)}')
                outputs_data.append(output_data)

            helper_obj.check_result_helper(outputs_data, output_desc, check_bin, check_bin_size)

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
    dynshape()
