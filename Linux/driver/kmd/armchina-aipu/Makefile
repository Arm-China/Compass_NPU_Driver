# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.

subdir-ccflags-y += -I$(src)/include
subdir-ccflags-y += -I$(src)/zhouyi

obj-$(CONFIG_ARMCHINA_NPU)  += armchina_npu.o
armchina_npu-y       := aipu.o aipu_core.o aipu_io.o aipu_irq.o  \
			aipu_job_manager.o aipu_mm.o aipu_priv.o \
			aipu_soc_default.o

include $(src)/zhouyi/Makefile
armchina_npu-y       += $(ZHOUYI_FILES)
