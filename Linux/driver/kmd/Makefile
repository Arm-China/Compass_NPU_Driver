# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.

MODULE_NAME := aipu
SRC_DIR := .
EXTRA_CFLAGS += -DKMD_VERSION=\"$(COMPASS_DRV_BTENVAR_KMD_VERSION)\" \
                   -I$(PWD)/armchina-npu/ -I$(PWD)/armchina-npu/include -I$(PWD)/armchina-npu/zhouyi \
                   -D$(BUILD_AIPU_VERSION_KMD)

COMM_OBJ := $(SRC_DIR)/armchina-npu/aipu.o \
            $(SRC_DIR)/armchina-npu/aipu_common.o \
            $(SRC_DIR)/armchina-npu/aipu_io.o \
            $(SRC_DIR)/armchina-npu/aipu_irq.o \
            $(SRC_DIR)/armchina-npu/aipu_job_manager.o \
            $(SRC_DIR)/armchina-npu/aipu_mm.o \
            $(SRC_DIR)/armchina-npu/aipu_priv.o \
            $(SRC_DIR)/armchina-npu/zhouyi/zhouyi.o

ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_Z1)
    AIPU_OBJ := $(SRC_DIR)/armchina-npu/zhouyi/z1.o \
                $(SRC_DIR)/armchina-npu/zhouyi/legacy_priv.o
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_Z2)
    AIPU_OBJ := $(SRC_DIR)/armchina-npu/zhouyi/z2.o \
                $(SRC_DIR)/armchina-npu/zhouyi/legacy_priv.o
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_Z3)
    AIPU_OBJ := $(SRC_DIR)/armchina-npu/zhouyi/z2.o \
                $(SRC_DIR)/armchina-npu/zhouyi/legacy_priv.o
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_X1)
    AIPU_OBJ := $(SRC_DIR)/armchina-npu/zhouyi/z2.o \
                $(SRC_DIR)/armchina-npu/zhouyi/legacy_priv.o
else ifeq ($(BUILD_AIPU_VERSION_KMD), BUILD_ZHOUYI_X2)
    AIPU_OBJ := $(SRC_DIR)/armchina-npu/zhouyi/x2.o \
                $(SRC_DIR)/armchina-npu/zhouyi/x2_priv.o
else
    #build all
    AIPU_OBJ := $(SRC_DIR)/armchina-npu/zhouyi/z1.o \
                $(SRC_DIR)/armchina-npu/zhouyi/z2.o \
                $(SRC_DIR)/armchina-npu/zhouyi/x2.o \
                $(SRC_DIR)/armchina-npu/zhouyi/legacy_priv.o \
                $(SRC_DIR)/armchina-npu/zhouyi/x2_priv.o
endif

INIT_OBJ := $(SRC_DIR)/default/default.o

OBJS     := $(COMM_OBJ) $(AIPU_OBJ) $(SOC_OBJ)

ifneq ($(KERNELRELEASE),)
	obj-m := $(MODULE_NAME).o
	$(MODULE_NAME)-objs := $(INIT_OBJ) $(OBJS)
else
	COMPASS_DRV_BTENVAR_KPATH ?= /lib/modules/`uname -r`/build
	PWD :=$(shell pwd)

all:
	$(MAKE) -C $(COMPASS_DRV_BTENVAR_KPATH) M=$(PWD) modules
clean:
	$(MAKE) -C $(COMPASS_DRV_BTENVAR_KPATH) M=$(PWD) clean
endif
