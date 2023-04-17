# SPDX-License-Identifier: GPL-2.0

subdir-ccflags-y += -I$(src)
subdir-ccflags-y += -I$(src)/include
subdir-ccflags-y += -I$(src)/zhouyi

obj-$(CONFIG_ARMCHINA_NPU) += armchina_npu.o
armchina_npu-y := aipu.o aipu_common.o aipu_io.o aipu_irq.o  \
			aipu_job_manager.o aipu_mm.o aipu_priv.o aipu_tcb.o

include $(src)/zhouyi/Makefile
include $(src)/default/Makefile
include $(src)/r329/Makefile