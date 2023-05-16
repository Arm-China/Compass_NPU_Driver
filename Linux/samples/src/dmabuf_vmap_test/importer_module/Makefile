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

all: build_env modules

modules:
	$(MAKE) ARCH=$(ARCH) -C $(KERNEL_DIR) M=$(PWD) modules
	@rm -fr *.o *.symvers *.order *.mod.c *.mod .*.cmd .tmp_versions
clean:
	$(MAKE) ARCH=$(ARCH) -C $(KERNEL_DIR) M=$(PWD) clean

build_env:
	@$(call check_env)

.PHONY: all clean

define check_env
	echo "Set KERNEL_DIR and CROSS_COMPILE firstly, then make"
	echo "KERNEL_DIR: " $(KERNEL_DIR)
	echo "CC: " $(CC)
endef
