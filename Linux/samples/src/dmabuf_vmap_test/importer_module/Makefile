#
# set KERNEL_DIR and CROSS_COMPILE firstly, then make
#
KERNEL_DIR := $(CONFIG_DRV_BTENVAR_JUNO_KPATH)
CROSS_COMPILE := $(CONFIG_DRV_BTENVAR_CROSS_CXX_PATH)/aarch64-linux-gnu-
ARCH := arm64

CC:= $(CROSS_COMPILE)gcc
LD:= $(CROSS_COMPILE)ld

PWD := $(shell pwd)
MODULE_NAME := importer
obj-m := importer.o

all: modules

modules:
	@$(call check_env)
	$(MAKE) ARCH=$(ARCH) -C $(KERNEL_DIR) M=$(PWD) modules
	@echo "clean temp files"; rm -fr *.o *.symvers *.order *.mod.c *.mod .*.cmd .tmp_versions
clean:
	$(MAKE) ARCH=$(ARCH) -C $(KERNEL_DIR) M=$(PWD) clean

.PHONY: all clean

SHELL:=/bin/bash
.ONESHELL:

define check_env
	if [ ! -e $(CC) ]; then
		echo "CC: $(CC) not exist"
		exit 1
	else
		echo "CC: $(CC)"
	fi

	if [ ! -e $(KERNEL_DIR) ]; then
		echo "KERNEL_DIR: $(KERNEL_DIR) not exist"
		exit 1
	else
		echo "KERNEL_DIR: $(KERNEL_DIR)"
	fi
endef
