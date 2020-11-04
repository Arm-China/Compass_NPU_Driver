###########################################################
#   Arm China Zhouyi AIPU Kernel Mode Driver (KMD) Readme
###########################################################

1. File Hierarchy
=================
.
|-- README.txt                      -> This readme
|-- Makefile                        -> KMD makefile (out of tree)
|-- include                         -> API header for user space
|   `-- uapi
|       `-- misc
|           `-- armchina_aipu.h
|-- armchina-aipu                   -> KMD core source files
|   |-- Kconfig
|   |-- Makefile
|   |-- aipu.c                      -> Architecture independent implementations
|   |-- aipu_common.c
|   |-- aipu_common.h
|   |-- aipu_io.c
|   |-- aipu_io.h
|   |-- aipu_irq.c
|   |-- aipu_irq.h
|   |-- aipu_job_manager.c
|   |-- aipu_job_manager.h
|   |-- aipu_mm.c
|   |-- aipu_mm.h
|   |-- aipu_partition.h
|   |-- aipu_priv.c
|   |-- aipu_priv.h
|   |-- aipu_tcb.h
|   |-- config.h
|   |-- include                     -> API header for SoC vendor's integrations
|   |   `-- armchina_aipu_soc.h
|   `-- zhouyi                      -> Architecture specific implementations
|       |-- Makefile
|       |-- x2.c                    -> Zhouyi X2 specific
|       |-- x2.h
|       |-- x2_priv.c
|       |-- z1.c                    -> Zhouyi Z1 specific
|       |-- z1.h
|       |-- z2.c                    -> Zhouyi Z2/Z3/X1 specific
|       |-- z2.h
|       |-- legacy_priv.c           -> Common for Zhouyi Z1/Z2/Z3/X1
|       |-- zhouyi.c                -> Common for Zhouyi architectures
|       `-- zhouyi.h
|-- default                         -> SoC integration implementation sample (default version)
|   `-- default.c
|-- armchina,zhouyi-npu.yaml        -> Linux kernel version DTS bindings
|-- checkpatch.pl                   -> Linux kernel checkpatch script
|-- const_structs.checkpatch
|-- kmd_check_patch.sh
`-- spelling.txt

2. Build AIPU KMD
=================
2.1 Build an out-of-tree module
-------------------------------
Before building, you should update the kernel path and toolchain configs in env_setup.sh.
You may build the out-of-tree KMD module with our build_all.sh script:

To build with the default SoC impl.:

    $./build_all.sh -p [juno/6cg] -v x1   #juno/6cg using default SoC impl.

If you implement your specific SoC integration instead of using the default samples,
you are suggested to update the build_all.sh and build with

    $./build_all.sh -p [MY_SOC] -v x1

The above build commands by default build all the implementations of different Zhouyi architectures.
If you would like to build for a single/compatible Zhouyi architecture(s), try to execute like:

    $./build_all.sh -p MY_SOC -v z1     #build Zhouyi Z1 only
    $./build_all.sh -p MY_SOC -v z2     #build Zhouyi Z2/Z3/X1 compatible version
    $./build_all.sh -p MY_SOC -v z3     #build Zhouyi Z2/Z3/X1 compatible version
    $./build_all.sh -p MY_SOC -v x1     #build Zhouyi Z2/Z3/X1 compatible version

2.2 Build in kernel
-------------------
KMD supports to be built in kernel. You should copy the armchina_aipu.h header into
./include/uapi/misc directory before building. The source files are suggested to
be built under ./drivers/misc.

3. Device Tree Source (DTS) Bindings
====================================
To bring-up Zhouyi SDK environment on your SoC chip/board, DTS with Zhouyi AIPU node
should be configured correctly.

3.1. Zhouyi AIPU Attributes
---------------------------
3.1.1 Compatible
----------------
By default, based on the architecture version of the AIPU on your chip,
the value of the compatible attribute should be one of the following strings:

    "armchina,zhouyi-v1"    ->  Zhouyi Z1
    "armchina,zhouyi-v2"    ->  Zhouyi Z2
    "armchina,zhouyi-v3"    ->  Zhouyi Z3
    "armchina,zhouyi-x1"    ->  Zhouyi X1
    "armchina,zhouyi-x2"    ->  Zhouyi X2
    "armchina,zhouyi"       ->  Zhouyi Z1/Z2/Z3/X1/X2

For example, you may configure for Zhouyi X1 AIPU as:

    /* case 1 */
    /* assume that 0x64000000 is the base address of AIPU registers on your SoC */
    aipu@64000000 {
        compatible = "armchina,zhouyi-x1";
        /* other attributes */
        /* ... */
    };

    or as:

    /* case 2 */
    aipu@64000000 {
        compatible = "armchina,zhouyi";
        /* other attributes */
        /* ... */
    };

This attribute is mandatory.

3.1.2 Core ID
-------------
Zhouyi driver supports mutiple AIPU instances for Zhouyi Z1/Z2/Z3/X1. You should provide the same
number of aipu nodes as AIPU core instances and specify corresponding core-id attribute in every node.

For example, assume that your SoC contains 2 Zhouyi AIPU cores, you may configure as:

    /* assume that 0x64000000/0x65000000 is the base address of registers of AIPU core #0/#1 */
    aipu0@64000000 {
        compatible = "armchina,zhouyi";
        core-id = <0>;
        /* other attributes of core #0 */
        /* ... */
    };

    aipu1@65000000 {
        compatible = "armchina,zhouyi";
        core-id = <1>;
        /* other attributes of core #1 */
        /* ... */
    };

This attribute is mandatory for Z1/Z2/Z3/X1 multiple instances scenario. For a single core case or X2,
you can omit it.

3.1.3 Register base
-------------------
Zhouyi driver supports at most two register base addresses:

    > the 1st one represents the base address of external registers of an AIPU core on your SoC;
    > the 2nd one represents the base address of SoC level registers which are related to AIPU;

The 1st one is mandatory for all architectures and the 2nd one is optional according to your SoC specific configurations.

For example, you may configure as followed for different cases:

    /* case 1 */
    /* assume that 0x64000000 is the base address of AIPU registers on your SoC
     * and there is no SoC level AIPU related registers.
     */
    aipu@64000000 {
        compatible = "armchina,zhouyi";
        reg = <0x0 0x64000000 0x0 0x1000>;
        /* other attributes */
        /* ... */
    };

    /* case 2 */
    /* assume that 0x1280010000 is the base address of AIPU registers on your SoC
     * and there is no SoC level AIPU related registers.
     */
    aipu@1280010000 {
        compatible = "armchina,zhouyi";
        reg = <0x12 0x80010000 0x0 0x1000>;
        /* other attributes */
        /* ... */
    };

    /* case 3 */
    /* assume that 0x1280010000 is the base address of AIPU registers on your SoC
     * and 0x12C0010000 is the base address of your SoC level AIPU related registers.
     */
    aipu@1280010000 {
        compatible = "armchina,zhouyi";
        reg = <0x12 0x80010000 0x0 0x1000>,
              <0x12 0xC0010000 0x0 0x80>;
        /* other attributes */
        /* ... */
    };

3.1.4 Memory Region
-------------------
AIPU driver supports Arm SMMU to virtualize addresses for AIPU. If without a SMMU, SoC vendor should
reserve memory region for host and AIPU core(s) to access as workspaces.

AIPU driver supports to reserve three types of memory region:

    Types       H/W
    ------------------------
    *memory* -> DDR
    *sram*   -> SoC SRAM
    *dtcm*   -> SoC SRAM/DDR

The reserved memory region node name in DTS should be exactly "memory", "sram", or "dtcm"; otherwise
driver will be failed in probing.

For *memory* or *sram* type, you can reserve one or mutiple region(s) for it. We limit the maximum count of
regions to be reserved to be 32 because Linux kernel accepts 64 at maximum.

For *dtcm* type, currently we only accept to reserve at maximum one region. If you would like to reserve
an address in SRAM or memory to be used as DTCM region, please use the *dtcm* type. *dtcm* type only works
for X1 and X2.

The regions can be reserved in any sequence.

An example:

    reserved-memory {
        #address-cells = <2>;
        #size-cells = <2>;
        ranges;

        aipu_res_0: memory@A0000000 {
            compatible = "shared-dma-pool";
            reusable;
            reg = <0x0 0xA0000000 0x0 0x10000000>;
        };

        aipu_res_1: dtcm@C0000000 {
            compatible = "shared-dma-pool";
            reusable;
            reg = <0x0 0xC0000000 0x0 0x800000>;
        };

        aipu_res_2: sram@C0800000 {
            compatible = "shared-dma-pool";
            reusable;
            reg = <0x0 0xC0800000 0x0 0x400000>;
        };

        aipu_res_3: sram@C1000000 {
            compatible = "shared-dma-pool";
            reusable;
            reg = <0x0 0xC1000000 0x0 0x800000>;
        };

        aipu_res_4: memory@B0000000 {
            compatible = "shared-dma-pool";
            reusable;
            reg = <0x0 0xB0000000 0x0 0x10000000>;
        };
    };

    aipu0@64000000 {
        /* other attributes */
        memory-region = <&aipu_res_0 &aipu_res_1 &aipu_res_2 &aipu_res_3 &aipu_res_4>;
    };

3.1.4.1 Without A SMMU
----------------------
Memory region(s) should be reserved for this scenario. To put it specifically, SoC vendor may choose to
make AIPU share the same region with other devices or monopolize it, to reserve in a DMA way or in a
CMA way. Those memory reservation methods result in different system memory management in Linux and
AIPU uses those addresses with no difference.

As given in Zhouyi Z1/Z2/Z3/X1/X2 specification, the valid address space AIPU could use is within
[0x0, 0x100000000). To avoid touching the reserved regions, AIPU driver furtherly restricts the memory
region reserved/allocated for AIPU:

    > For Zhouyi Z1: within [0x0, 0xC0000000);
    > For Zhouyi Z1/Z3/X1: lower 32 bits address within [0x0, 0xC0000000); no limits for higher 32 bits;
    > For Zhouyi X2: lower 32 bits address within [0x0, 0xE0000000); no limits for higher 32 bits;

Note that how many bytes to reserve depends on your target application scenario. The following values
to reserve are just for the purpose of usage demo. For example,

    /* case 1 */
    /* reserve 256MB via DMA pool */
    reserved-memory {
        #address-cells = <2>;
        #size-cells = <2>;
        ranges;

        aipu_res_0: memory@1000000000 {
            compatible = "shared-dma-pool";
            no-map;
            reg = <0x10 0x00000000 0x0 0x10000000>;
        };
    };

    /* case 2 */
    /* reserve 256MB via CMA pool and do not share with other devices */
    reserved-memory {
        #address-cells = <2>;
        #size-cells = <2>;
        ranges;

        aipu_res_0: memory@1000000000 {
            compatible = "shared-dma-pool";
            reusable;
            reg = <0x10 0x00000000 0x0 0x10000000>;
        };
    };

Reference the reserved region in an aipu node:

    aipu@1280010000 {
            compatible = "armchina,zhouyi";
            memory-region = <&aipu_res_0>;
            /* other attributes */
            /* ... */
    };

For multiple instances scenario, AIPU cores may share the same memory region and the reserved region
can be referenced in only one AIPU node.

This attribute is mandatory if AIPU is not behind a SMMU.

3.1.4.2 With A SMMU
-------------------
When a SMMU presents, AIPU uses IOVAs instead of pure device DMA addresses. Those IOVAs are mapped from
system managed physically discontiguous regions, or from a physically contiguous DMA region if there is
a DMA/CMA reservation at the same time.

Currently, AIPU driver supports SMMU-401. You may bind it as followed:

    smmu401: smmu@12C0100000 {
        compatible = "arm,mmu-401";
        reg = <0x12 0xC0100000 0x0 0x10000>;
        #iommu-cells = <1>;
        status = "okay";
        #global-interrupts = <1>;
        interrupt-parent = <&gic>;
        interrupts = <0 90 1>,
            <0 90 1>, <0 90 1>, <0 90 1>, <0 90 1>,
            <0 90 1>, <0 90 1>, <0 90 1>, <0 90 1>,
            <0 90 1>, <0 90 1>, <0 90 1>, <0 90 1>,
            <0 90 1>, <0 90 1>, <0 90 1>, <0 90 1>;
    };

    aipu@1280010000 {
        compatible = "armchina,zhouyi";
        iommus = <&smmu401 0x0>;
        /* other attributes */
        /* ... */
    };

To use a physically contiguous DMA region behind, you may reserve it as followed:

    aipu@1280010000 {
        compatible = "armchina,zhouyi";
        memory-region = <&aipu_res_0>;
        iommus = <&smmu401 0x0>;
        /* other attributes */
        /* ... */
    };

3.1.4.3 SoC SRAM
----------------
SoC SRAM is optional for Zhouyi. Zhouyi driver supports to configure SoC SRAM for AIPU to consume as
feature map reuse buffers.

Please find the example in 3.1.4.

Note that the higher 32 bits of the base address of SoC SRAM should be equal to that of the reserved
DRAM. This ensures all the reuse buffers to share the same ASID parameters.

3.1.4.4 DTCM
------------
AIPU Data Tightly Coupled Memory (DTCM) is an address space for high-speed accessing (like SRAM) on
Zhouyi X1. Zhouyi X1 SoC vendor may provide a DTCM region to speedup the model inference.

If DTCM is configured, the size of it should be a value within a list (spec requires):

    {512KB, 1MB, 2MB, 3MB, ..., 31MB, 32MB}

DTCM region is optional for X1/X2. Please find the example in 3.1.4.

3.1.5 Host-AIPU-Offset
----------------------
Sometimes to access the main memory, the physical addresses (cpu_pa) issued from host CPU and the
addresses used by a device (dma_addr) do not equal. For this scenario, we use the host-aipu-offset
attribute to describe the gap between them.

You must specify this attribute in a reserved memory region node if this offset != 0.

For example, if for the same memory space, dma_addr = cpu_pa - 0x80000000, you may configure as
followed:

    /* case 1 */
    /* cpu_pa space = dma_addr space = [0xA0000000, 0xB0000000) */
    reserved-memory {
        #address-cells = <2>;
        #size-cells = <2>;
        ranges;

        aipu_res_0: memory@A0000000 {
            compatible = "shared-dma-pool";
            no-map;
            reg = <0x0 0xA0000000 0x0 0x10000000>;
        };
    };

    /* case 2 */
    /* cpu_pa space = dma_addr space = [0xA0000000, 0xB0000000) */
    reserved-memory {
        #address-cells = <2>;
        #size-cells = <2>;
        ranges;

        aipu_res_0: memory@A0000000 {
            compatible = "shared-dma-pool";
            no-map;
            reg = <0x0 0xA0000000 0x0 0x10000000>;
            host-aipu-offset = <0x0 0x0>;
        };
    };

    /* case 3 */
    /* cpu_pa space = [0xA0000000, 0xB0000000), dma_addr space = [0x20000000, 0x30000000) */
    reserved-memory {
        #address-cells = <2>;
        #size-cells = <2>;
        ranges;

        aipu_res_0: memory@A0000000 {
            compatible = "shared-dma-pool";
            no-map;
            reg = <0x0 0xA0000000 0x0 0x10000000>;
            host-aipu-offset = <0x0 0x80000000>;
        };
    };

This attribute is optional and by default this value in driver is 0 if not activated in DTS.

3.1.6 Interrupts
----------------
Interrupt numbers and the trigger types should be specified as followed:

    /* case 1 */
    aipu@64000000 {
        compatible = "armchina,zhouyi";
        interrupts = <0 168 IRQ_TYPE_EDGE_RISING>;
        /* other attributes */
        /* ... */
    };

    /* case 2 */
    aipu@1281010000 {
        compatible = "armchina,zhouyi";
        interrupt-parent = <&gic>;
        interrupts = <0 90 1>;
        /* other attributes */
        /* ... */
    };

Shared IRQ number is allowed. AIPU interrupt handler returns IRQ_NONE immediately
if the incoming interrupt is from another device.

This attribute is mandatory in all cases.

3.1.7 Cluster-Partition Pair
----------------------------
Zhouyi X2 hardware is organized in such architecture: partition > cluster > core > TEC.
Driver should configure the partition number and cluster info. in each partition.

We use the cluster-partition pair in DTS to specify their relationships:

    cluster-partition = <cluster_id_1 partition_id_i>, <cluster_id_2 partition_id_j>,
                        <cluster_id_3 partition_id_k>, ..., <cluster_id_n partition_id_w>;

which means that there are 4 clusters in all and you:

    configure cluster_1 into partition_i;
    configure cluster_2 into partition_j;
    configure cluster_3 into partition_k;
    configure cluster_4 into partition_w;

where i, j, k, w can be the same or different with each other, depending on your decision and
the maximum partition count.

You should notice that:

    1. the configured partition count should be <= the maximum count allowed by your AIPU h/w arch.;
    2. one cluster should be configured into one and only one partition;
    3. one partition can hold one single cluster or multiple ones;
    4. the cluster-partition relation cannot be changed at runtime after configured by DTS;
    5. the cluster_id and partition_id should be a uint32 number;

For example, if the AIPU on your system has only one cluster, you can configure as:

    /* case 1 */
    aipu@64000000 {
        compatible = "armchina,zhouyi";
        cluster-partition = <0 0>;
        /* other attributes */
        /* ... */
    };

This attribute is mandatory for X2. For other architectures, you can omit it.

3.1.8 GM-Policy
---------------
In Zhouyi X2, every AIPU cluster has a General Memory (GM) region caching the address of
frequently updated data.

We allow AIPU users to configure the usage policy of this region:

    [policy 1] shared by tasks of all QoS level in the cluster
    [policy 2] divided half-by-half: one for tasks of QoS slow, another for tasks of QoS fast

You should set this attribute as:

    /* policy 1 */
    aipu@64000000 {
        compatible = "armchina,zhouyi-x2";
        gm-policy = <1>;
        /* other attributes */
        /* ... */
    };

    /* policy 2 */
    aipu@64000000 {
        compatible = "armchina,zhouyi-x2";
        gm-policy = <2>;
        /* other attributes */
        /* ... */
    };

If you do not set this attribute, by default driver will adopt policy 1. For AIPU architectures
without GM feature, this attribute will be omitted.

4. Implement Specific SoC Operations
====================================
If you would like to implement the probing interfaces for your own specific SoC *MY_SOC*,
create ./MY_SOC/MY_SOC.c and implement the interfaces defined in armchina_aipu_soc.h. The clock
or power configurations for AIPU of your SoC can be added.

Note that your MY_SOC_probe()/MY_SOC_remove() implementations should call
armchina_aipu_probe()/armchina_aipu_remove(), respectively.

5. Soft-reset Delay Time
========================
For Zhouyi X1, you may configure AIPU_CONFIG_DEFAULT_RESET_DELAY_US in ./armchina-aipu/config.h
to set the soft-reset delay time. When a soft-reset is triggered, driver will check the hardware
status after this time delay since the reset happens. If the status is abnormal, driver will report
error and stop to schedule tasks. By default this delay value is 50us.

This feature only works for X1/X2.

6. Sysfs Debug Interfaces
=========================
Zhouyi driver supports sysfs interfaces for user space to track the status of AIPU. After insmoding
AIPU ko, assume the base addr of your AIPU registers is 0x64000000, you may find the AIPU sysfs
directory:

    /sys/devices/platform/64000000.aipu0

For multi-instances, you may find more sub-directories like:

    /sys/devices/platform/65000000.aipu1
    /sys/devices/platform/66000000.aipu2
    /sys/devices/platform/67000000.aipu3

You can read/write the corresponding sysfs attributes of a core according to your SoC configurations.

6.1 External Registers
----------------------
You can always read/write to AIPU external registers as long as AIPU is not in power-off or
clock-gating states. Note that some registers are specific of an AIPU architecture and not common
ones of Zhouyi, and some registers are read-only for host to access. Make sure to read/write to the
correct registers with corresponding permissions. In most cases you shall not directly write the
external registers via sysfs, unless you are debugging issues or testing something.

To read the current values in external registers of AIPU core #0, execute:

    $cat /sys/devices/platform/64000000.aipu0/ext_registers

Then you can find the values be printed on screen like:

    ----------------------------------------
    AIPU Core0 External Register Values
    ----------------------------------------
    Offset  Name                  Value
    ----------------------------------------
    0x0     Ctrl Reg              0x0000000f
    0x4     Status Reg            0x00050000
    0x8     Start PC Reg          0x2000000c
    0xc     Intr PC Reg           0x20000010
    0x10    IPI Ctrl Reg          0x00000000
    0x14    Data Addr 0 Reg       0x2107b000
    0x18    Data Addr 1 Reg       0x21094000
    0x1c    Data Addr 2 Reg       0x00000000
    0x20    Data Addr 3 Reg       0x00000000
    0xc0    ASE0 Ctrl Reg         0xc0010000
    0xc4    ASE0 High Base Reg    0x00000000
    0xc8    ASE0 Low Base Reg     0x00000000
    0xcc    ASE1 Ctrl Reg         0x80010000
    0xd0    ASE1 High Base Reg    0x00000000
    0xd4    ASE1 Low Base Reg     0x00000000
    0xd8    ASE2 Ctrl Reg         0x00000000
    0xdc    ASE2 High Base Reg    0x00000000
    0xe0    ASE2 Low Base Reg     0x00000000
    0xe4    ASE3 Ctrl Reg         0x00000000
    0xe8    ASE3 High Base Reg    0x00000000
    0xec    ASE3 Low Base Reg     0x00000000
    0x180   DTCM Ctrl Reg         0x00000001
    0x184   DTCM High Base Reg    0x00000000
    0x188   DTCM Low Base Reg     0x00000000
    0x200   Soft Reset Reg        0x00000002
    ----------------------------------------

To write values into a register, you should follow the following format:

    $echo <reg_offset>-<write_time>-<write_value> > /sys/devices/platform/64000000.aipu0/ext_registers

where:
    <reg_offset>  is the offset of a register you would like to write
    <write_time>  is how many time you would like to do the write operation
    <write_value> is the real value you would like to configure

For example, to write Data Address 0 Register (offset 0x14) with 0x1000, just execute:

    $echo 0x14-1-0x1000 > /sys/devices/platform/64000000.aipu0/ext_registers

to do soft-reset on Zhouyi X1, execute:

    $echo 0x200-1-0x1 > /sys/devices/platform/64000000.aipu0/ext_registers

Please do not configure external registers when there are unfinished inferencing tasks. This may
result in undefined behaviors.

6.2 Clock
---------
If you have defined SoC level clock related methods in *struct aipu_soc_operations* and register the
struct in your probe function, you may find a clock operating interface under sysfs directory.

To read the clock state, execute:

    $cat /sys/devices/platform/64000000.aipu0/soc_clock

then corresponding logs will be printed on screen:

    "AIPU is in clock gating state and suspended."
    "AIPU is in normal working state."

To do clocking gating/resume, execute the following commands respectively:

    $echo 1 > /sys/devices/platform/64000000.aipu0/soc_clock       #suspend
    $echo 0 > /sys/devices/platform/64000000.aipu0/soc_clock       #resume

This feature only works for Z1/Z2/Z3/X1.

6.3 Disable A Core
------------------
You can soft-disable an AIPU core to prevent scheduling of jobs to it.

To check if AIPU core #0 is under disabled state, execute:

    $cat /sys/devices/platform/64000000.aipu0/disable

then corresponding logs will be printed on screen:

    "AIPU core #0 is enabled."
    "AIPU core #0 is disabled."

To disbale/enable AIPU core #0, execute:

    $echo 1 > /sys/devices/platform/64000000.aipu0/disable      #disable
    $echo 0 > /sys/devices/platform/64000000.aipu0/disable      #enable

This feature only works for Z1/Z2/Z3/X1.

7. User Space Driver (UMD)
==========================
To ensure that your tasks are scheduled and executed correctly, you shall use the exact UMD and KMD
in the same package we released to you. For specification of UMD, please refer to the Zhouyi Software
Programming Guide (Chapter of Driver).

8. Coding Style
===============
To check the coding style of AIPU KMD, you can execute:

    $./kmd_check_patch.sh

This script uses Linux kernel checkpatch scripts.

9. Compatibility
================
Zhouyi AIPU KMD supports Linux kernel version from 4.x to 5.x. Specifically, we tested the code on
the following Linux kernel versions:

    4.9.51
    4.9.168
    5.4.66
    5.10.5
    5.11.18
    5.12.8
    5.13.8
    5.14.6
    5.15.5

If you find that KMD has compatibility issues with any kernel version (> 4.9.0), please contact us
and report the issues. Thanks.

10. License
===========
GNU GPL v2
