# Compass_NPU_driver
Compass_NPU_driver includes two parts: kernel mode driver(KMD) and user mode driver(UMD), actually UMD is a user mode library. KMD is a standard Linux char driver model for NPU. UMD can be compiled as dynamical and static library accordingly. User application directly call top APIs in UMD and then indirectly call the interface of KMD to interract with NPU hardware.

## 1. Folders
### driver
include kernel driver (KMD) and user lib (UMD).
### devicetree
the reference related to how to config dts to enable AIPU on embedded boards.
### samples
the samples related to how to call UMD APIs to implement inference APPs.
### out-of-box
the simplest demo running on simulator.
### unit_test
the unit test cases for UMD and KMD.

## 2. Compile driver

### 2.1 Cross compile KMD/UMD for HW board
Here take Juno(arm64) board as an example, If you use another board, please change below env variables accordingly in bash_env_setup.sh (for bash) or env_setup.sh (for csh)

- change work directory to Linux/

- set common env variables

- set top directory for compile package
CONFIG_DRV_BTENVAR_BASE_DIR=/project/ai/scratch01

- specify cross-compile toolchain path
CONFIG_DRV_BTENVAR_CROSS_CXX_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin

- specify Linux kernel source code top directory
CONFIG_DRV_BTENVAR_JUNO_KPATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP/kernel/linux-5.11.18

- specify compiler for compiling samples
COMPASS_DRV_BTENVAR_CROSS_CXX=aarch64-linux-gnu-g++

- specify corss-copiler for compiling Linux kernel
COMPASS_DRV_BTENVAR_CROSS_COMPILE_GNU=aarch64-linux-gnu-

- compile commands

```bash
# cd Linux
# source bash_env_setup.sh (for bash env)
or
# source env_setup.sh (for csh env)
# ./build_all.sh -p juno -v x1 [-d]
```

- If the command runs normally, a folder named 'bin' is created, the corresponding KMD driver(aipu.ko) and UMD library(libaipudrv.so) are generated and stored in it.

### 2.2 Compile UMD for x86_64 simulator
Set below env variables accordingly.

#### set common env variables for simulation

- set top directory for compile package
CONFIG_DRV_BTENVAR_BASE_DIR=/project/ai/scratch01

- specify X86 g++ and lib path
CONFIG_DRV_BRENVAR_X86_CLPATH=/arm/tools/gnu/gcc/7.3.0/rhe7-x86_64/lib64 (optional)
COMPASS_DRV_BTENVAR_X86_CXX=g++

- specify path where stores Z1/Z2/Z3/X1 simulators
CONFIG_DRV_RTENVAR_SIM_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_SIMULATOR/
COMPASS_DRV_RTENVAR_SIM_LPATH=${CONFIG_DRV_RTENVAR_SIM_PATH}/lib/

#### specify path where store X2 simulator library
- CONFIG_DRV_BRENVAR_Z5_SIM_LPATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_SIMULATOR/z5_lib/

- compile commands

```bash
# cd Linux
# source bash_env_setup.sh (for bash env)
or
# source env_setup.sh (for csh env)
# ./build_all.sh -p sim -v x1 [-d]
```

- If the command run normally, a folder named 'bin' is created, the UMD library(libaipudrv.so)
is generated and stored in it.

2.3 Recommend document

- driver/kmd/README.txt: some useful detail for compiling and using driver.

## 3. Compile samples

### 3.1 compile samples for HW board
Here take Juno(arm64) board as an example, it has to do #2.1 firstly before doing this step. then

```bash
# cd Linux
# source bash_env_setup.sh
# ./build_all.sh -p juno -v x1 -t sample [-d]
```

After perform this command successfully, the samples are also stored in forder 'bin'.
- aipu_benchmark_test: the general model, run AI model follow single graph with single frame.
- aipu_flush_job_test: the pipeline model, run AI model follow one graph multiple frames.
- aipu_mthread_test: the multiple thread model, each thread run a set of inference jobs.

### 3.2 Compile UMD for x86_64 simulator
it has to do #2.2 firstly before doing this step. then

```bash
# cd Linux
# source bash_env_setup.sh
# ./build_all.sh -p sim -v x1 -t sample [-d]
```

After perform this command successfully, the samples are also stored in forder 'bin'.
- aipu_simulation_test: the general one graph with one frame model
(note: it doesn't support pipeline and multiple thread model on simulator)

### 3.3 Recommend document

- samples/READM.md: some detail to run samples.

## 4. Compile & run out-of-box demo

- Read the detail in out-of-box README.md

## 5. Compile & run unit_test

- Read the detail in unit_test README.md