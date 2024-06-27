# Samples for UMD Python APIs

The samples can quickly deploy NPU inference application through exported UMD Python APIs. They can be split into two parts, one is for simulation environmetn, another is for HW environment.

## 1. For simulation environment
- Compile Python Wrapper library (libaipudrv.so)

```bash
-- for aipu v1/v2/v3
# ./build_all.sh -p sim -v v3 -a python_api [-d]

-- for aipu v4
# ./build_all.sh -p sim -v v4 -a python_api [-d]

Refer to Linux/README.md for more detai information.
```

- sim_sgsf_finish.py: run a single graph frame via simulator with synchronous mode

```bash
# python3 sim_sgsf_finish.py -e ./aipu_simulator_x2 -s ./resnet50/ -l ./lib -d ./output/

arguments:
-e: specify simualtor path (./aipu_simulator_x2)
-s: single benchmark path (./resnet50), it has to put model,input and check files in it. eg: aipu.bin,input0.bin, output.bin
-l: specify UMD Python Wrapper library path (./lib), it has libaipudrv.so in it.
-d: specify dump file path, create it firstly

The other arguments can get from -h.
```

- sim_sgsf_flush.py: run a single graph frame via simulator with asynchronous mode

```bash
# python3 sim_sgsf_flush.py -e ./aipu_simulator_x2 -s ./resnet50/ -d ./output/ -l ./lib
```

## 2. For hardware environment
- Compile Python Wrapper library (libaipudrv.so)

```bash
-- for aipu v1/v2/v3
# ./build_all.sh -p juno -v v3 -a python_api [-d]

-- for aipu v4
# ./build_all.sh -p juno -v v4 -a python_api [-d]

Refer to Linux/README.md for more detai information.
```

- hw_sgsf_finish.py: run a single graph frame via simulator with synchronous mode

```bash
# python3 hw_sgsf_finish.py -s ./resnet50/ -l ./lib -d ./output/

arguments:
-s: single benchmark path (./resnet50), it has to put model,input and check files in it. eg: aipu.bin,input0.bin, output.bin
-d: specify dump file path, create it firstly
-l: specify UMD Python Wrapper library path (./lib), it has libaipudrv.so in it.

The other arguments can get from -h.
```

- hw_sgsf_flush.py: run a single graph frame via simulator with asynchronous mode

```bash
# python3 hw_sgsf_flush.py -e ./aipu_simulator_x2 -s ./resnet50/ -d ./output/ -l ./lib
```

## 3. Note
- These cases will cover both UMD and KMD part.
- When setup running environment, it needs to move the whole folder 'python_samples' to target environment.