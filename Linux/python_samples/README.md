# Samples for UMD Python APIs

The samples can quickly deploy NPU inference application through exported UMD Python APIs. They can be split into two parts, one is for simulation environmetn, another is for HW environment.

## 1. Scripts
- Compile Python Wrapper library (libaipudrv.so)
- If you try to run on simulator `-p` is `sim`, otherwise, `-p` is `juno`. You need to check `/dev/aipu` on hardware

```bash
# for aipu v1/v2/v3
$ ./build_all.sh -p <sim/juno> -v v3 -a python_api [-d]

# for aipu v3_1
$ ./build_all.sh -p <sim/juno> -v v3_1 -a python_api [-d]
```
Refer to Linux/README.md for more detai information.

- sgsf_finish.py: run a single graph frame with synchronous mode

```bash
$ python3 sgsf_finish.py -e ./aipu_simulator_x1 -s ./resnet50/ -l ./lib -d ./output/
```
arguments:  
- -e: specify full executable simualtor path only valid on v1&v2 (./aipu_simulator_x1)  
- -s: single benchmark path (./resnet50), it has to put model,input and check files in it. eg: aipu.bin,input0.bin, output.bin  
- -l: specify UMD Python Wrapper library path (./lib), it has libaipudrv.so in it.  
- -d: specify dump file path, create it firstly 

The other arguments can get from -h.  

- sgsf_flush.py: run a single graph frame via simulator with asynchronous mode
```bash
$ python3 sgsf_flush.py -e ./aipu_simulator_x1 -s ./resnet50/ -d ./output/ -l ./lib
```

- dynshape.py: run with dynaic input shapes
```
$ python3 dyshape.py  -s /home/benchmark/resnet50 -l bin/juno/debug/ -r "1,1,1,1024/1,1/1,1,128,128"
```

- multi_batch.py: run with multiple batches
```
$ python3 multi_batch.py -s /home/benchmark/resnet50 -l bin/sim/debug/ -e ./aipu_simulator_x1 -d ./output
```

## 2. Note
- These cases will cover both UMD and KMD part.
- When setup running environment, it needs to move the whole folder 'python_samples' to target environment.