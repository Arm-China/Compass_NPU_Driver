# SPDX-License-Identifier: GPL-2.0
subdir-ccflags-y += -I$(src)

ZHOUYI_V12_FILES := zhouyi/v12_priv.o
ZHOUYI_V3_FILES     := zhouyi/v3.o zhouyi/v3_priv.o

armchina_npu-y +=  zhouyi/zhouyi.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V1) += $(ZHOUYI_V12_FILES) zhouyi/v1.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V2) += $(ZHOUYI_V12_FILES) zhouyi/v2.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V3) += $(ZHOUYI_V3_FILES)