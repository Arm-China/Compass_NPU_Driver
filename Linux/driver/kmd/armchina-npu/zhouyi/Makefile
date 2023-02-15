# SPDX-License-Identifier: GPL-2.0
subdir-ccflags-y += -I$(src)

ZHOUYI_LEGACY_FILES := zhouyi/legacy_priv.o
ZHOUYI_X2_FILES     := zhouyi/x2.o zhouyi/x2_priv.o

armchina_npu-y +=  zhouyi/zhouyi.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_Z1) += $(ZHOUYI_LEGACY_FILES) zhouyi/z1.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_X1) += $(ZHOUYI_LEGACY_FILES) zhouyi/x1.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_X2) += $(ZHOUYI_X2_FILES)