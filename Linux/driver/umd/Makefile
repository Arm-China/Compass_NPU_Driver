# Copyright (C) 2023-2025 Arm Technology (China) Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

MD  := mkdir -p
RM  := rm -rf

SRC_ROOT := ./src
INC_DIR  := ./include

SRC_COMMON = $(SRC_ROOT)/common/
SRC_ZHOUYI_V1V2 = $(SRC_ROOT)/zhouyi_v1v2/
SRC_ZHOUYI_V3X = $(SRC_ROOT)/zhouyi_v3x/
SRC_ZHOUYI_V3X_COMMON = $(SRC_ZHOUYI_V3X)/common
SRC_ZHOUYI_V3 = $(SRC_ZHOUYI_V3X)/zhouyi_v3/
SRC_ZHOUYI_V3_2 = $(SRC_ZHOUYI_V3X)/zhouyi_v3_2/
SRC_MISC = $(SRC_ROOT)/misc/
SRC_UTIL = $(SRC_ROOT)/utils/
SRC_DEVICE = $(SRC_ROOT)/device/

CXXFLAGS := -std=c++14 -fPIC -Wall -Werror -rdynamic -MMD -MP
INCD := -I$(INC_DIR) -I$(SRC_ROOT) -I$(SRC_COMMON)

LDFLAGS = -shared -Wl,-soname,$(COMPASS_DRV_BTENVAR_UMD_SO_NAME)
ANDROIDCXXFLAGS := --target=aarch64-none-linux-android21 \
                   --gcc-toolchain=$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64 \
                   --sysroot=$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/sysroot \
                   -DGTEST_HAS_RTTI=0 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D__STDC_LIMIT_MACROS -DANDROID -Wno-error \
                   -Wno-non-pod-varargs -Wno-missing-braces -Wno-inconsistent-missing-override -Wno-unused-variable -Wno-overloaded-virtual \
                   -Wno-tautological-pointer-compare -Wno-unused-command-line-argument
ANDROIDLDFLAGS := -Wl,-O3 -Wl,--gc-sections \
                  -Wl,--exclude-libs,$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/lib/gcc/aarch64-linux-android/4.9.x/libgcc.a \
                  -Wl,--exclude-libs,$(BUILD_ANDROID_NDK)/toolchains/llvm/prebuilt/linux-x86_64/aarch64-linux-android/lib64/libatomic.a \
                  -static-libstdc++ -Wl,--build-id -Wl,--warn-shared-textrel -Wl,--fatal-warnings -Qunused-arguments -Wl,-z,noexecstack  -Wl,-z,defs

ifeq ($(BUILD_DEBUG_FLAG), debug)
    ifeq ($(BUILD_UMD_API_TYPE), python_api)
    CXXFLAGS += -O2 -g -DRTDEBUG=1
    else
    CXXFLAGS += -O0 -g -DRTDEBUG=1
    endif
else
    CXXFLAGS += -O2 -DRTDEBUG=0
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_v1v2)
    CXXFLAGS += -DZHOUYI_V12
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_v3)
    CXXFLAGS += -DZHOUYI_V12
    CXXFLAGS += -DZHOUYI_V3
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_v3_2)
    CXXFLAGS += -DZHOUYI_V12
    CXXFLAGS += -DZHOUYI_V3_2
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_all)
    CXXFLAGS += -DZHOUYI_V12
    CXXFLAGS += -DZHOUYI_V3
    CXXFLAGS += -DZHOUYI_V3_2
endif

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    CXXFLAGS += -DSIMULATION
endif

SRC_DIRS = $(SRC_MISC) $(SRC_COMMON) $(SRC_UTIL) $(SRC_DEVICE) $(SRC_ZHOUYI_V1V2) \
           $(SRC_ZHOUYI_V3X_COMMON) $(SRC_ZHOUYI_V3) $(SRC_ZHOUYI_V3_2)
SRCS = $(SRC_COMMON)/context.cpp           \
       $(SRC_COMMON)/ctx_ref_map.cpp       \
       $(SRC_COMMON)/graph_base.cpp        \
       $(SRC_COMMON)/graph.cpp             \
       $(SRC_COMMON)/job_base.cpp          \
       $(SRC_COMMON)/parser_base.cpp       \
       $(SRC_COMMON)/memory_base.cpp       \
       $(SRC_COMMON)/standard_api_impl.cpp \
       $(SRC_COMMON)/status_string.cpp     \
       $(SRC_COMMON)/share_weight_mgr.cpp  \
       $(SRC_MISC)/aipu_printf.cpp         \
       $(SRC_UTIL)/helper.cpp              \
       $(SRC_UTIL)/sha256.cpp

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    SRC_DIRS += $(SRC_DEVICE)simulator
    SRCS += $(SRC_DEVICE)/simulator/umemory.cpp
else
    SRC_DIRS += $(SRC_DEVICE)/aipu
    SRCS += $(SRC_DEVICE)/aipu/aipu.cpp \
            $(SRC_DEVICE)/aipu/ukmemory.cpp
endif

# V1V2 deps
V1V2_SRCS += $(SRC_ZHOUYI_V1V2)/graph_v1v2.cpp \
        $(SRC_ZHOUYI_V1V2)/job_v1v2.cpp \
        $(SRC_ZHOUYI_V1V2)/parser_v1v2.cpp
ifeq ($(BUILD_TARGET_PLATFORM), sim)
    V1V2_SRCS += $(SRC_DEVICE)/simulator/simulator.cpp
endif

SRCS += $(V1V2_SRCS)

# V3X deps
V3X_INCD += -I./3rdparty
V3X_SRCS += $(SRC_ZHOUYI_V3X_COMMON)/graph_v3x.cpp \
        $(SRC_ZHOUYI_V3X_COMMON)/job_v3x.cpp \
        $(SRC_ZHOUYI_V3X_COMMON)/parser_elf.cpp \
        $(SRC_ZHOUYI_V3X_COMMON)/dynamic_shape.cpp \
        $(SRC_ZHOUYI_V3X_COMMON)/coredump.cpp

ifeq ($(BUILD_TARGET_PLATFORM), sim)
V3X_SRCS += $(SRC_DEVICE)/simulator/wrapper.cpp
endif

# V3 deps
V3_SRCS += $(SRC_ZHOUYI_V3)/job_v3.cpp \
           $(SRC_ZHOUYI_V3)/gm_v3.cpp

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    V3_SRCS += $(SRC_DEVICE)/simulator/simulator_v3.cpp
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_v3)
    INCD += $(V3X_INCD)
    SRCS += $(V3X_SRCS) $(V3_SRCS)
endif

# V3_2 deps
V3_2_SRCS += $(SRC_ZHOUYI_V3_2)/job_v3_2.cpp \
             $(SRC_ZHOUYI_V3_2)/gm_v3_2.cpp

ifeq ($(BUILD_TARGET_PLATFORM), sim)
    V3_2_SRCS += $(SRC_DEVICE)/simulator/simulator_v3_2.cpp
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_v3_2)
    INCD += $(V3X_INCD)
    SRCS += $(V3X_SRCS) $(V3_2_SRCS)
endif

ifeq ($(BUILD_AIPU_VERSION), aipu_all)
    SRCS += $(V3X_SRCS)
    INCD += $(V3X_INCD)

    SRCS += $(V3_SRCS)
    SRCS += $(V3_2_SRCS)
endif

ifeq ($(BUILD_UMD_API_TYPE), python_api)
    SRCS += $(SRC_COMMON)/export_py_api.cpp
    CXXFLAGS += -fwrapv -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2
    CXXFLAGS += -DBUILD_PYTHON_API
    INCD += -I$(CONFIG_DRV_RTENVAR_PY_INCD_PATH) -I./3rdparty/pybind11/include/
endif

ifneq ($(BUILD_ANDROID_NDK), )
    LDFLAGS += -pthread -llog -ldl
    CXXFLAGS += $(ANDROIDCXXFLAGS)
    LDFLAGS += $(ANDROIDCXXFLAGS) $(ANDROIDLDFLAGS)
else
    LDFLAGS += -lpthread -ldl
endif

CXXFLAGS += -DMACRO_UMD_VERSION=\"$(COMPASS_DRV_BTENVAR_UMD_V_MAJOR).$(COMPASS_DRV_BTENVAR_UMD_V_MINOR)\"

SRC_DIRS := $(patsubst $(SRC_ROOT)/%, %, $(SRC_DIRS))
OBJS := $(patsubst $(SRC_ROOT)/%.cpp, $(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/%.o, $(SRCS))
DEPS = $(OBJS:%.o=%.d)
TARGET := $(BUILD_AIPU_DRV_ODIR)/$(COMPASS_DRV_BTENVAR_UMD_SO_NAME_FULL)
A_TARGET := $(BUILD_AIPU_DRV_ODIR)/$(COMPASS_DRV_BTENVAR_UMD_A_NAME_FULL)
PY_TARGET := $(BUILD_AIPU_DRV_ODIR)/$(COMPASS_DRV_BTENVAR_UMD_SO_NAME)

standard_api: build-repo $(TARGET) $(A_TARGET)
python_api: build-repo $(PY_TARGET)

$(COMPASS_DRV_BTENVAR_UMD_BUILD_DIR)/%.o: $(SRC_ROOT)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCD) -c $< -o $@

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

$(A_TARGET): $(OBJS)
	$(AR) crus $@ $(OBJS)

$(PY_TARGET): $(OBJS)
	$(CXX) $^ $(LDFLAGS) -lstdc++ -o $@

-include $(DEPS)

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
