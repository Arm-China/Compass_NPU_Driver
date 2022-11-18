# Compass_NPU 驱动
Compass_NPU 驱动包括两部分：内核态驱动(KMD)和用户态驱动(UMD)，内核态驱动是一个标准的Linux字符设备驱动，而用户态驱动其实是辅助应用开发的一个库，它可以被编译成动态库或静态库的形式。应用程序通过直接调用用户态驱动提供的高层API，进而间接的调用到内核态驱动提供的接口，达到与硬件交互的目的。

## 1. 目录
### driver
存放内核态(KMD)和用户态(UMD)驱动的源码实现
### devicetree
如何在一些嵌入式平台上使用该驱动，对应的的设备文件树文件(DTS)的配置参考
### samples
如何调用用户态驱动(UMD)中的API来实现具体推理应用的参考例程
### out-of-box
一个在X86平台上的模拟器上跑的简单应用demo
### unit_test
一组内核态驱动(UMD)和用户态驱动(UMD)的单元测试源码

## 2. 编译驱动

### 2.1 交叉编译KMD和UMD
这里以JUNO(Arm64)开发板为例，如果使用其它的嵌入式平台，请根据实际环境，在bash_env_setup.sh 或 env_setup.sh中，设置以下的环境变量。

- 切换工作目录到驱动源码包中路径 Linux/

- 设置如下公用的环境变量

- 设置顶层编译包依赖的总路径
CONFIG_DRV_BTENVAR_BASE_DIR=/project/ai/scratch01

- 指定交叉编译工具链的路径
CONFIG_DRV_BTENVAR_CROSS_CXX_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP/toolchain/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin

- 指定Linux内核源码包路径
CONFIG_DRV_BTENVAR_JUNO_KPATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_BSP/kernel/linux-5.11.18

- 指定编译UMD库和样例的C++编译器
COMPASS_DRV_BTENVAR_CROSS_CXX=aarch64-linux-gnu-g++

- 指定交叉编译KMD的工具链
COMPASS_DRV_BTENVAR_CROSS_COMPILE_GNU=aarch64-linux-gnu-

- 编译命令

```bash
# cd Linux
# source bash_env_setup.sh (for bash env)
or
# source env_setup.sh (for csh env)
# ./build_all.sh -p juno -v x1 [-d]
```

- 如果以上命令成功执行，一个驱动加载模块aipu.ko和一个用户态动态链接库libaipudrv.so将产生，并且被存放在bin文件夹中。

### 2.2 编译配合X86上模拟的UMD

#### 设置如下公用环境变量

- 设置顶层编译包依赖的总路径
CONFIG_DRV_BTENVAR_BASE_DIR=/project/ai/scratch01

- 指定X86平台的g++编译器和依赖库路径
CONFIG_DRV_BRENVAR_X86_CLPATH=/arm/tools/gnu/gcc/7.3.0/rhe7-x86_64/lib64 (可选)
COMPASS_DRV_BTENVAR_X86_CXX=g++

- 针对Z1/Z2/Z3/X1模拟的时候，指定模拟器存放的路径
CONFIG_DRV_RTENVAR_SIM_PATH=${CONFIG_DRV_BTENVAR_BASE_DIR}/AIPU_SIMULATOR/
COMPASS_DRV_RTENVAR_SIM_LPATH=${CONFIG_DRV_RTENVAR_SIM_PATH}/lib/

- 编译命令

```bash
# cd Linux
# source bash_env_setup.sh (for bash env)
or
# source env_setup.sh (for csh env)
# ./build_all.sh -p sim -v x1 [-d]
```

- 如果以上命令成功执行，一个驱动加载模块aipu.ko和一个用户态动态链接库libaipudrv.so将产生，并且被存放在bin文件夹中。

2.3 推荐阅读的文档

- driver/kmd/README.txt: 关于编译和使用KMD的细节参考。

## 3. 编译样例

### 3.1 针对特定嵌入式平台，编译对应的参考样例
这里以JUNO(Arm64)开发板为例，在此之前也要先完成#2.1中的步骤。

```bash
# cd Linux
# source bash_env_setup.sh
# ./build_all.sh -p juno -v x1 -t sample [-d]
```

上述命令成功执行之后，对应的样例生成的可执行文件也将被放在bin文件夹中。
- aipu_benchmark_test: 调用aipu_finish_job API(阻塞) 的单图单帧的推理应用例程。
- aipu_flush_job_test: 调用aipu_flush_job API(非阻塞) 的单图多帧的推理应用例程。
- aipu_mthread_test: 多线程例程，每个线程独立加载graph并进行推算.

### 3.2 针对X86模拟器，编译对应的参考样例
在执行下面的命令之前，也要先完成前面#2.2中的相应步骤。

```bash
# cd Linux
# source bash_env_setup.sh
# ./build_all.sh -p sim -v x1 -t sample [-d]
```

上述命令成功执行之后，对应的样例生成的可执行文件也将被放在bin文件夹中。
After perform this command successfully, the samples are also stored in forder 'bin'.
- aipu_simulation_test: 调用aipu_finish_job API(阻塞) 的单图单帧的推理应用例程。
(注意: 在模拟器上不支持非阻塞和多线程应用实现)

### 3.3 推荐阅读

- samples/READM.md: 关于所以参考样例的说明

## 4. 编译和运行开箱测试例程

- 请参考out-of-box中的README.md

## 5. 编译和运行单元测试

- 请参考unit_test中的README.md