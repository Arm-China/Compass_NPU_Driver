# SPDX-License-Identifier: GPL-2.0
subdir-ccflags-y += -I$(src)

ZHOUYI_LEGACY_FILES := zhouyi/z1.o zhouyi/z2.o zhouyi/legacy_priv.o
ZHOUYI_X2_FILES		:= zhouyi/x2.o zhouyi/x2_priv.o

armchina_npu-y +=  zhouyi/zhouyi.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_LEGACY) += $(ZHOUYI_LEGACY_FILES)
armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_X2)     += $(ZHOUYI_X2_FILES)