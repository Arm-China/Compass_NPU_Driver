
#!/bin/env python3
import os
import sys
import numpy as np
from common.helper import *
from common.log import *

#
# hw_sgsf_flush.py:
# 	this script is for running in hardware board via finish job async mode.
#
# usage:
#   python3 hw_sgsf_flush.py  -s /home/benchmark/resnet50 -l bin/juno/debug/
#
# note:
#   resnet50 {aipu.bin, input0.bin, output.bin}
#   bin/juno/debug/: libaipudrv.so path
#

# global switch
LOAD_FROM_FILE_FLAG = False

log = Log(EM_LOG_TYPE_ERR | EM_LOG_TYPE_ALT | EM_LOG_TYPE_WAR | EM_LOG_TYPE_INF)
parseCmdline_obj = Parse_Cmdline(log)
helper_obj = Help(log)
hw_env_check(log)

# run here to ensure libaipudrv.so's path is added after cmdline arguments parsed
from libaipudrv import *

load_cfg = {
	"wt_mem_region": 0
}
wt_idxes = []

job_cfg = {
	"partition_id" : 0,
	"dbg_dispatch" : 0,
	"dbg_core_id" : 0,
	"qos_level" : 0
}
fm_idxes = []

frame_cnt = 2

npu = NPU()
ret = npu.aipu_init_context()
if ret != AIPU_STATUS_SUCCESS:
    errmsg = npu.aipu_get_error_message(ret)
    log.error(f'aipu_init_context [fail], err: {errmsg}')
    exit(-1)
else:
	log.debug('aipu_init_context [ok]')

# loop all benchmarks
for benchmark in parseCmdline_obj.m_benchmarks_list:
	log.debug(f'Test: <{benchmark["model"]}>')

	aipu_bin = benchmark["model"]
	check_bin = benchmark["check_bin"]
	check_bin_size = benchmark["check_bin_size"]
	input_bins = benchmark["input_bins"]

	# load way1: from aipu.bin path
	if LOAD_FROM_FILE_FLAG:
		retmap = npu.aipu_load_graph(aipu_bin, load_cfg, wt_idxes)
	else:
	# load way2: from aipu.bin's data buffer and size
		with open(aipu_bin, mode='rb') as filp:
			graph_data = filp.read()
			graph_len = len(graph_data)
			log.debug(f'graph_len: {graph_len}')

			retmap = npu.aipu_load_graph(graph_data, graph_len, load_cfg, wt_idxes)

	if retmap["ret"] != AIPU_STATUS_SUCCESS:
		errmsg = npu.aipu_get_error_message(retmap["ret"])
		log.error(f'aipu_load_graph [fail], err: {errmsg}')
		npu.aipu_deinit_context()
		exit(-1)
	else:
		graph_id = retmap["data"]
		log.info(f'load model: {aipu_bin}, {graph_id:x}')

	retmap = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_INPUT)
	if retmap["ret"] != AIPU_STATUS_SUCCESS:
		errmsg = npu.aipu_get_error_message(retmap["ret"])
		log.error(f'aipu_get_tensor_count [fail], err: {errmsg}')
		npu.aipu_deinit_context()
		exit(-1)
	else:
		input_cnt = retmap["data"]
		log.debug(f'aipu_get_tensor_count [ok], input cnt: {input_cnt}')

	input_desc = []
	for i in range(input_cnt):
		input_desc.append(npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_INPUT, i))
		log.debug(f'input {i}: {input_desc[i].id}, size: {input_desc[i].size}')

	retmap = npu.aipu_get_tensor_count(graph_id, AIPU_TENSOR_TYPE_OUTPUT)
	if retmap["ret"] != AIPU_STATUS_SUCCESS:
		errmsg = npu.aipu_get_error_message(retmap["ret"])
		log.error(f'aipu_get_tensor_count [fail], err: {errmsg}')
		npu.aipu_unload_graph(graph_id)
		npu.aipu_deinit_context()
		exit(-1)
	else:
		output_cnt = retmap["data"]
		log.debug(f'aipu_get_tensor_count [ok], output cnt: {output_cnt}')

	output_desc = []
	for i in range(output_cnt):
		output_desc.append(npu.aipu_get_tensor_descriptor(graph_id, AIPU_TENSOR_TYPE_OUTPUT, i))
		log.debug(f'output {i}: {output_desc[i].id}, size: {output_desc[i].size}')

	# create multiple jobs
	job_id_list = []
	for frame in range(frame_cnt):
		retmap = npu.aipu_create_job(graph_id, job_cfg, fm_idxes)
		if retmap["ret"] != AIPU_STATUS_SUCCESS:
			errmsg = npu.aipu_get_error_message(retmap["ret"])
			log.error(f'aipu_create_job [fail], err: {errmsg}')
			npu.aipu_unload_graph(graph_id)
			npu.aipu_deinit_context()
			exit(-1)
		else:
			job_id = retmap["data"]
			job_id_list.append(job_id)
			log.debug('aipu_create_job [ok]')

		# config dump path
		if os.path.isdir(parseCmdline_obj.m_dump_path):
			job_cfg_dump = {
				"dump_dir" : parseCmdline_obj.m_dump_path,
				"data_dir" : parseCmdline_obj.m_dump_path
			}

			ret = npu.aipu_config_job(job_id, AIPU_JOB_CONFIG_TYPE_DUMP_OUTPUT, job_cfg_dump)
			if ret != AIPU_STATUS_SUCCESS:
				errmsg = npu.aipu_get_error_message(ret)
				log.error(f'aipu_config_job [fail], err: {errmsg}')
				npu.aipu_clean_job(job_id)
				npu.aipu_unload_graph(graph_id)
				npu.aipu_deinit_context()
				exit(-1)
			else:
				log.debug('aipu_config_job [ok]')

		if LOAD_FROM_FILE_FLAG: # load 2-1
			for i in range(len(input_bins)):
				ret = npu.aipu_load_tensor_from_file(job_id, i, input_bins[i])
				if ret != AIPU_STATUS_SUCCESS:
					npu.aipu_clean_job(job_id)
					npu.aipu_unload_graph(graph_id)
					npu.aipu_deinit_context()
					exit(-1)
				else:
					log.info(f'load file: {input_bins[i]}')
		else: # load 2-2
			for i in range(len(input_bins)):
				with open(input_bins[i], mode='rb') as filp:
					input_data = filp.read()

					ret = npu.aipu_load_tensor(job_id, i, input_data)
					if ret != AIPU_STATUS_SUCCESS:
						errmsg = npu.aipu_get_error_message(retmap["ret"])
						log.error(f'aipu_load_tensor [fail], err: {errmsg}')
						npu.aipu_clean_job(job_id)
						npu.aipu_unload_graph(graph_id)
						npu.aipu_deinit_context()
						exit(-1)
					else:
						log.info(f'load file: {input_bins[i]}')

	# flush multiple jobs
	for frame in range(frame_cnt):
		job_id = job_id_list[frame]
		ret = npu.aipu_flush_job(job_id, None)
		if ret != AIPU_STATUS_SUCCESS:
			errmsg = npu.aipu_get_error_message(ret)
			log.error(f'aipu_flush_job [fail], err: {errmsg}')
			npu.aipu_clean_job(job_id)
			npu.aipu_unload_graph(graph_id)
			npu.aipu_deinit_context()
			exit(-1)
		else:
			log.debug(f'aipu_flush_job [ok]')

		status = AIPU_JOB_STATUS_NO_STATUS
		while status == AIPU_JOB_STATUS_NO_STATUS:
			retmap = npu.aipu_get_job_status(job_id, -1)
			status = retmap["data"]
			log.debug("poll status...")

		output_data = []
		if status == AIPU_JOB_STATUS_EXCEPTION:
			log.error(f'aipu_get_job_status: {job_id} <exception>')
			exit(-1)
		elif status == AIPU_JOB_STATUS_DONE:
			log.debug(f'aipu_get_job_status: {job_id:x}, done')

			for i in range(output_cnt):
				retmap = npu.aipu_get_tensor(job_id, AIPU_TENSOR_TYPE_OUTPUT, i, D_UINT8)
				if retmap["ret"][0] != AIPU_STATUS_SUCCESS: # note the return value checking for this API
					errmsg = npu.aipu_get_error_message(retmap["ret"][0])
					log.error(f'aipu_flush_job [fail], err: {errmsg}')
					npu.aipu_clean_job(job_id)
					npu.aipu_unload_graph(graph_id)
					npu.aipu_deinit_context()
					exit(-1)
				else:
					output_data.append(retmap["data"])

			helper_obj.check_result_helper(output_data, output_desc, check_bin, check_bin_size)

		ret = npu.aipu_clean_job(job_id)
		if ret != AIPU_STATUS_SUCCESS:
			errmsg = npu.aipu_get_error_message(ret)
			log.error(f'aipu_clean_job [fail], err: {errmsg}')
			npu.aipu_unload_graph(graph_id)
			npu.aipu_deinit_context()
			exit(-1)
		else:
			log.debug(f'aipu_clean_job [ok]')

	ret = npu.aipu_unload_graph(graph_id)
	if ret != AIPU_STATUS_SUCCESS:
		errmsg = npu.aipu_get_error_message(ret)
		log.error(f'aipu_unload_graph [fail], err: {errmsg}')
		npu.aipu_deinit_context()
		exit(-1)
	else:
		log.debug(f'aipu_unload_graph [ok]')

ret = npu.aipu_deinit_context()
if ret != AIPU_STATUS_SUCCESS:
	errmsg = npu.aipu_get_error_message(ret)
	log.error(f'aipu_deinit_context [fail], err: {errmsg}')
	exit(-1)
else:
	log.debug(f'aipu_deinit_context [ok]')
