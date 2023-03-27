# Compass_NPU_driver
Compass_NPU_driver includes two parts: kernel mode driver(KMD) and user mode library(UMD). KMD is a standard Linux char driver model for NPU. UMD can be compiled as dynamicly and static libraty accordingly. User application directly call top APIs in UMD and then indirectly call the interface of KMD to interract with NPU hardware.

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
Here take Juno(arm64) board as an example, If you use nother board, please change below env variables accordingly in bash_env_setup.sh (for bash) or env_setup.sh (for csh)

#### prepare the depending BSP resource, eg: Linux source code, cross-compiler...

```bash
/home/ai/scrach01
|
|-- AIPU_BSP
    |-- kernel
    |   |-- linux-5.11.18
    |
    `-- toolchain
        |-- gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu
```

- change work directory to Linux/ in Compass_NPU_driver

- set common env variables

- set top directory for compile package
CONFIG_DRV_BTENVAR_BASE_DIR=/home/ai/scrach01
CONFIG_DRV_BTENVAR_BSP_BASE_DIR=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP

- specify cross-compile toolchain path
CONFIG_DRV_BTENVAR_CROSS_CXX_PATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin

- specify Linux kernel source code top directory
CONFIG_DRV_BTENVAR_JUNO_KPATH=${CONFIG_DRV_BTENVAR_BSP_BASE_DIR}/kernel/linux-5.11.18

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

# ./build_all.sh -p juno [-d]
```

- If the command run normally, a folder named 'bin' is created, the corresponding KMD driver(aipu.ko) and UMD library(libaipudrv.so) are generated and stored in it.

### 2.2 Compile UMD for x86_64 simulator
Set below env variables accordingly.

#### prepare simulation depending resource, eg: aipu_simulator_x1, libaipu_simulator_z2.so...

```bash
/home/ai/scratch01/
|
|-- AIPU_SIMULATOR
    |
    |-- bin
    |   |-- aipu_simulator_x1
    |   |-- aipu_simulator_x2
    |   |-- aipu_simulator_z1
    |   |-- aipu_simulator_z2
    |   |-- aipu_simulator_z3
    |
    `-- lib
        |-- libaipu_simulator_x1.so
        |-- libaipu_simulator_x2.so
        |-- libaipu_simulator_z1.so
        |-- libaipu_simulator_z2.so
        |-- libaipu_simulator_z3.so
```

#### set common env variables for simulation

- set top directory for compile package
CONFIG_DRV_BTENVAR_BASE_DIR=/home/ai/scratch01
CONFIG_DRV_RTENVAR_SIM_BASE_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_SIMULATOR

- specify X86 g++ and lib path
CONFIG_DRV_BRENVAR_X86_CLPATH=/arm/tools/gnu/gcc/7.3.0/rhe7-x86_64/lib64 (optional)
COMPASS_DRV_BTENVAR_X86_CXX=g++

- specify path where store aipu v1/v2/v3 simulators and libraries
CONFIG_DRV_RTENVAR_SIM_PATH=${CONFIG_DRV_RTENVAR_SIM_BASE_PATH}/bin/
COMPASS_DRV_RTENVAR_SIM_LPATH=${CONFIG_DRV_RTENVAR_SIM_BASE_PATH}/lib/

- compile commands

```bash
# cd Linux

# source bash_env_setup.sh (for bash env)
or
# source env_setup.sh (for csh env)
# ./build_all.sh -p sim [-d]
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
# ./build_all.sh -p juno -t sample [-d]
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
# ./build_all.sh -p sim -t sample [-d]
```

After perform this command successfully, the samples are also stored in forder 'bin'.
- aipu_simulation_test: the general one graph with one frame model
(note: it doesn't support pipeline and multiple thread model on simulator)

### 3.3 How to run samples

- samples/READM.md: some detail to run samples.


## 4. Compile & run unit_test

- Read the detail in unit_test README.md
