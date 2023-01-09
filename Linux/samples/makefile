# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

MD  := mkdir -p
RM  := rm -rf

SRC_ROOT := ./src
UMD_INC := ../driver/umd/include
TARGET := $(BUILD_AIPU_DRV_ODIR)/aipu_$(BUILD_TEST_CASE)
CXXFLAGS := -Wall -Werror -std=c++14 -I$(SRC_ROOT) -I$(UMD_INC)
LDFLAGS = -L$(BUILD_AIPU_DRV_ODIR) -laipudrv
ANDROIDCXXFLAGS := --target=aarch64-none-linux-android21 \
                   --gcc-toolchain=$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64 \
                   --sysroot=$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/sysroot \
                   -DGTEST_HAS_RTTI=0 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D__STDC_LIMIT_MACROS -DANDROID -Wno-error
ANDROIDLDFLAGS := -Wl,-O3 -Wl,--gc-sections \
                  -Wl,--exclude-libs,$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/lib/gcc/aarch64-linux-android/4.9.x/libgcc.a \
                  -Wl,--exclude-libs,$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/aarch64-linux-android/lib64/libatomic.a \
                  -static-libstdc++ -Wl,--build-id -Wl,--warn-shared-textrel -Wl,--fatal-warnings -Qunused-arguments -Wl,-z,noexecstack  -Wl,-z,defs

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    ifneq ($(BUILD_AIPU_VERSION), z123)
        LDFLAGS += -L$(CONFIG_DRV_BRENVAR_X2_SIM_LPATH) -l$(COMPASS_DRV_BRENVAR_X2_SIM_LNAME)
    endif
endif

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    CXXFLAGS += -DSIMULATION=1
endif

ifeq ($(BUILD_DEBUG_FLAG), debug)
    CXXFLAGS += -O0 -g -DRTDEBUG=1
else
    CXXFLAGS += -O2 -DRTDEBUG=0
endif

ifeq ($(BUILD_AIPU_VERSION), z123)
    CXXFLAGS += -DZHOUYI_V12
endif

ifeq ($(BUILD_AIPU_VERSION), x2)
    CXXFLAGS += -DZHOUYI_V3
endif

ifeq ($(BUILD_AIPU_VERSION), all)
    CXXFLAGS += -DZHOUYI_V12
    CXXFLAGS += -DZHOUYI_V3
endif

ifneq ($(BUILD_ANDROID_NDK), )
    LDFLAGS += -pthread
    CXXFLAGS += $(ANDROIDCXXFLAGS)
    LDFLAGS += $(ANDROIDCXXFLAGS) $(ANDROIDLDFLAGS)
else
    LDFLAGS += -lpthread
endif

SRCS += $(wildcard $(SRC_ROOT)/$(BUILD_TEST_CASE)/*.cpp) \
        $(wildcard $(SRC_ROOT)/common/cmd_line_parsing.cpp) \
        $(wildcard $(SRC_ROOT)/common/helper.cpp)
SRC_DIRS := $(SRC_ROOT)/common $(SRC_ROOT)/$(BUILD_TEST_CASE)
SRC_DIRS := $(patsubst $(SRC_ROOT)/%, %, $(SRC_DIRS))
OBJS = $(patsubst $(SRC_ROOT)/%.cpp, $(COMPASS_DRV_BTENVAR_TEST_BUILD_DIR)/%.o, $(SRCS))

all: build-repo $(TARGET)

$(COMPASS_DRV_BTENVAR_TEST_BUILD_DIR)/%.o: $(SRC_ROOT)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@
	$(RM) $(OBJS)

build-repo:
	@$(call make-repo)

clean:
	$(RM) $(OBJS) $(BUILD_AIPU_DRV_ODIR)/*test

.PHONY: all clean

define make-repo
	$(MD) $(COMPASS_DRV_BTENVAR_TEST_BUILD_DIR)
	for dir in $(SRC_DIRS); \
	do \
		$(MD) $(COMPASS_DRV_BTENVAR_TEST_BUILD_DIR)/$$dir; \
	done
endef
