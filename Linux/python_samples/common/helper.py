import mmap
import os
import sys
import argparse


class Parse_Cmdline:
    m_benchmark_path = ""
    m_group_benchmark_path = ""
    m_dump_path = ""
    m_so_lib_path = ""
    m_extra_weight_path = ""
    m_simulator = ""
    m_target = ""
    m_profile_en = False
    m_input_shape = []
    m_benchmarks_list = []

    logger = None

    def __init__(self, logger):
        self.logger = logger
        parser = argparse.ArgumentParser(description="Cmdline Parser", add_help=True)
        parser.add_argument('-s', '--single_benchmark_path', type=str,
                            help='-s: specify the single benchmark path, eg: resnet {aipu.bin, input0.bin, output.bin}')
        parser.add_argument('-g', '--group_benchmark_path', type=str,
                            help='-g: specify top path for benchmarks, eg: top_dir {resnet {bins}, yolo {bins}}')
        parser.add_argument('-d', '--dump_path', type=str,
                            help='-d: specify dump path')
        parser.add_argument('-l', '--so_lib_path', type=str,
                            help='-l: specify aipu driver so library path')
        parser.add_argument('-e', '--simulator', type=str,
                            help='-e: specify full simulator path, such as `path/of/aipu_simulator_x1`, just for v1&v2 simulation')
        parser.add_argument('-r', '--input_shape', type=str,
                            help='-r: specify new input shape for all input tensors')
        parser.add_argument('-w', '--extra_weight_path', type=str,
                            help='-w: specify extra weight path')
        parser.add_argument('-a', '--target', type=str,
                            help='-a: specify aipu target, only for >= v3')
        parser.add_argument('-p', '--profile', action='store_true',
                            help='-p: enable profile, only for >= v3 and sgsf_finish.py')

        args = parser.parse_args()

        if args.single_benchmark_path != None:
            self.logger.debug(f"-s arg: {args.single_benchmark_path}")
            if os.path.isdir(args.single_benchmark_path):
                self.m_benchmark_path = args.single_benchmark_path
                self.parse_single_benchmark(self.m_benchmark_path)
            else:
                self.logger.error(f'-s arg: {args.single_benchmark_path} [not exist]')
                exit(-1)

        if args.group_benchmark_path != None:
            self.logger.debug(f"-g arg: {args.group_benchmark_path}")
            if os.path.isdir(args.group_benchmark_path):
                self.m_group_benchmark_path = args.group_benchmark_path
                self.parse_group_benchmark(self.m_group_benchmark_path)
            else:
                self.logger.error(f'-g arg: {args.group_benchmark_path} [not exist]')
                exit(-1)

        if args.dump_path != None:
            self.logger.debug(f"-d arg: {args.dump_path}")
            if os.path.isdir(args.dump_path) is False:
                os.makedirs(args.dump_path)
            self.m_dump_path = args.dump_path

        if args.so_lib_path != None:
            if os.path.isdir(args.so_lib_path):
                self.m_so_lib_path = args.so_lib_path
                sys.path.append(args.so_lib_path)
            else:
                self.logger.error(f'-l arg: {args.so_lib_path} [not exist]')
                exit(-1)

        if args.target != None:
            self.m_target = args.target

        if args.simulator != None:
            self.logger.debug(f"--simulator arg: {args.simulator}")
            if os.path.isfile(args.simulator):
                self.m_simulator = args.simulator
            else:
                self.logger.error(f'--simulator arg: {args.simulator} [not exist]')
                exit(-1)

        if args.input_shape != None:
            self.m_input_shape = [[int(dim_val) for dim_val in shape_str.split(",")]
                                  for shape_str in args.input_shape.split("/")]

        if args.extra_weight_path != None:
            self.logger.debug(f"-d arg: {args.extra_weight_path}")
            if os.path.isdir(args.extra_weight_path):
                self.m_extra_weight_path = args.extra_weight_path
            else:
                self.logger.error(f'-w arg: {args.extra_weight_path} [not exist]')
                exit(-1)

        self.m_profile_en = True if args.profile == 1 else False

    def parse_single_benchmark(self, bench_dir):
        self.logger.debug(f'benchdir: {bench_dir}')
        aipu_bin = bench_dir + "/aipu.bin"
        if os.path.exists(aipu_bin) == False:
            self.logger.error(f'{aipu_bin} [not exist]')
            exit(-1)

        check_bin = bench_dir + "/output.bin"
        if (os.path.exists(check_bin)) == False:
            self.logger.error(f'{check_bin} [not exist]')
            exit(-1)
        else:
            check_bin_size = os.path.getsize(check_bin)

        input_bins = []
        for i in range(10):
            input_bin = bench_dir + "/input" + str(i) + ".bin"
            if os.path.exists(input_bin) == False:
                break
            else:
                input_bins.append(input_bin)

        single_benchmark_dic = {}
        single_benchmark_dic["model"] = aipu_bin
        single_benchmark_dic["check_bin"] = check_bin
        single_benchmark_dic["check_bin_size"] = check_bin_size
        single_benchmark_dic["input_bins"] = input_bins

        self.m_benchmarks_list.append(single_benchmark_dic)

    def parse_group_benchmark(self, bench_dir):
        for entry in os.listdir(bench_dir):
            bench_full_path = os.path.join(bench_dir, entry)
            self.parse_single_benchmark(bench_full_path)


class Help:
    m_logger = None

    def __init__(self, logger):
        self.m_logger = logger

    def is_output_correct(self, src, dst, len):
        for i in range(len):
            if src[i] != dst[i]:
                return False
            else:
                return True

    def check_result_helper(self, outputs, output_descs, gt_file, gt_size):
        if os.path.exists(gt_file) == False:
            self.m_logger.error(f'gt_file: {gt_file} not exist')
            exit(-1)

        file_sz = 0
        for id in range(len(outputs)):
            file_sz += output_descs[id].size
            if file_sz > gt_size:
                self.m_logger.error('gt file length ({gt_size}) < output tensor size ({file_sz})!')
                exit(-1)

        with open(gt_file, 'rb') as f:
            mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)

            offset = 0
            for id in range(len(outputs)):
                self.m_logger.debug(f'out{id}: {output_descs[id].size:x} bytes')
                if bytes(outputs[id]) == mm[offset: offset + output_descs[id].size]:
                    self.m_logger.alert(f'Test Result Check PASS! ({id + 1}/{len(outputs)})')
                else:
                    self.m_logger.alert(f'Test Result Check Faild! ({id + 1}/{len(outputs)})')

                offset += output_descs[id].size


def hw_env_check(log):
    if os.path.exists("/dev/aipu") == False:
        log.info('no device: /dev/aipu, will try to run simulator')
        return False
    return True
