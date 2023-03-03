# Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

MD  := mkdir -p
RM  := rm -rf

SRC_ROOT := ./src
INC_DIR  := ./include

SRC_COMMON = $(SRC_ROOT)/common/
SRC_ZHOUYI_LEGACY = $(SRC_ROOT)/zhouyi_v1v2/
SRC_ZHOUYI_V3 = $(SRC_ROOT)/zhouyi_v3/
SRC_MISC = $(SRC_ROOT)/misc/
SRC_UTIL = $(SRC_ROOT)/utils/
SRC_DEVICE = $(SRC_ROOT)/device/

CXXFLAGS := -std=c++14 -fPIC -Wall -Werror
INCD := -I$(INC_DIR) -I$(SRC_ROOT) -I$(SRC_COMMON) -I$(SRC_DEVICE) -I$(SRC_MISC) -I$(SRC_UTIL) -I$(SRC_ZHOUYI_LEGACY) -I$(SRC_ZHOUYI_V3)
LDFLAGS = -shared -Wl,-soname,$(COMPASS_DRV_BTENVAR_UMD_SO_NAME)
ANDROIDCXXFLAGS := --target=aarch64-none-linux-android21 \
                   --gcc-toolchain=$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64 \
                   --sysroot=$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/sysroot \
                   -DGTEST_HAS_RTTI=0 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D__STDC_LIMIT_MACROS -DANDROID -Wno-error
ANDROIDLDFLAGS := -Wl,-O3 -Wl,--gc-sections \
                  -Wl,--exclude-libs,$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/lib/gcc/aarch64-linux-android/4.9.x/libgcc.a \
                  -Wl,--exclude-libs,$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/aarch64-linux-android/lib64/libatomic.a \
                  -static-libstdc++ -Wl,--build-id -Wl,--warn-shared-textrel -Wl,--fatal-warnings -Qunused-arguments -Wl,-z,noexecstack  -Wl,-z,defs

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

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    CXXFLAGS += -DSIMULATION
endif

SRC_DIRS = $(SRC_MISC) $(SRC_COMMON) $(SRC_UTIL) $(SRC_DEVICE) $(SRC_ZHOUYI_LEGACY) $(SRC_ZHOUYI_V3)
SRCS = $(SRC_COMMON)/context.cpp           \
       $(SRC_COMMON)/ctx_ref_map.cpp       \
       $(SRC_COMMON)/graph_base.cpp        \
       $(SRC_COMMON)/graph.cpp             \
       $(SRC_COMMON)/job_base.cpp          \
       $(SRC_COMMON)/parser_base.cpp       \
       $(SRC_COMMON)/memory_base.cpp       \
       $(SRC_COMMON)/standard_api_impl.cpp \
       $(SRC_COMMON)/status_string.cpp     \
       $(SRC_MISC)/aipu_printf.cpp       \
       $(SRC_UTIL)/helper.cpp

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    SRC_DIRS += $(SRC_DEVICE)simulator
    SRCS += $(SRC_DEVICE)/simulator/umemory.cpp
else
    SRC_DIRS += $(SRC_DEVICE)/aipu
    SRCS += $(SRC_DEVICE)/aipu/aipu.cpp \
            $(SRC_DEVICE)/aipu/ukmemory.cpp
endif

ifneq ($(BUILD_AIPU_VERSION), z123)
    INCD += -I./3rdparty
    SRCS += $(SRC_ZHOUYI_V3)/graph_v3.cpp   \
            $(SRC_ZHOUYI_V3)/job_v3.cpp     \
            $(SRC_ZHOUYI_V3)/parser_elf.cpp \
            $(SRC_ZHOUYI_V3)/gm.cpp
    ifeq ($(BUILD_TARGET_PLATFORM), sim)
        LDFLAGS += -L$(CONFIG_DRV_BRENVAR_X2_SIM_LPATH) -l$(COMPASS_DRV_BRENVAR_X2_SIM_LNAME)
        SRCS += $(SRC_DEVICE)/simulator/simulator_v3.cpp
    endif
endif

ifneq ($(BUILD_AIPU_VERSION), x2)
    SRCS += $(SRC_ZHOUYI_LEGACY)/graph_v1v2.cpp \
            $(SRC_ZHOUYI_LEGACY)/job_v1v2.cpp   \
            $(SRC_ZHOUYI_LEGACY)/parser_v1v2.cpp
    ifeq ($(BUILD_TARGET_PLATFORM), sim)
        SRCS += $(SRC_DEVICE)/simulator/simulator.cpp
    endif
endif

ifeq ($(BUILD_UMD_API_TYPE), python_api)
    SRCS += $(SRC_COMMON)/swig_cpp2py_api.cpp
    CXXFLAGS += -fwrapv -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2
    INCD += -I$(CONFIG_DRV_RTENVAR_PY_INCD_PATH)
endif

ifeq ($(BUILD_WITH_NUMPY), yes)
    INCD += -I$(CONFIG_DRV_RTENVAR_NUMPY_INCD_PATH)
    CXXFLAGS += -DBUILD_WITH_NUMPY
    SWIGFLAGS += -DBUILD_WITH_NUMPY
endif

ifneq ($(BUILD_ANDROID_NDK), )
    LDFLAGS += -pthread
    CXXFLAGS += $(ANDROIDCXXFLAGS)
    LDFLAGS += $(ANDROIDCXXFLAGS) $(ANDROIDLDFLAGS)
else
    LDFLAGS += -lpthread
endif

SRC_DIRS := $(patsubst $(SRC_ROOT)/%, %, $(SRC_DIRS))
OBJS := $(patsubst $(SRC_ROOT)/%.cpp, $(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/%.o, $(SRCS))
TARGET := $(BUILD_AIPU_DRV_ODIR)/$(COMPASS_DRV_BTENVAR_UMD_SO_NAME_FULL)
A_TARGET := $(BUILD_AIPU_DRV_ODIR)/$(COMPASS_DRV_BTENVAR_UMD_A_NAME_FULL)
PY_TARGET := $(BUILD_AIPU_DRV_ODIR)/_$(COMPASS_DRV_BTENVAR_UMD_SO_NAME)

standard_api: build-repo $(TARGET) $(A_TARGET)
python_api:  build-repo $(PY_TARGET)

$(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/%.o: $(SRC_ROOT)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCD) -c $< -o $@

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

$(A_TARGET): $(OBJS)
	$(AR) crus $@ $(OBJS)

$(PY_TARGET): $(OBJS) $(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/libaipudrv_wrap.o
	$(CXX) $^ $(LDFLAGS) -lstdc++ -o $@

$(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/libaipudrv_wrap.o: libaipudrv.i
	swig $(SWIGFLAGS) -c++ -python libaipudrv.i
	$(CXX) $(CXXFLAGS) $(INCD) -c libaipudrv_wrap.cxx -o $@
	mv libaipudrv.py $(BUILD_AIPU_DRV_ODIR)
	rm -f libaipudrv_wrap.cxx

build-repo:
	@$(call make-repo)

clean:
	$(RM) $(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)

.phony: standard_api python_api build-repo clean

define make-repo
	$(MD) $(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)
	for dir in $(SRC_DIRS); \
	do \
		$(MD) $(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/$$dir; \
	done
endef
