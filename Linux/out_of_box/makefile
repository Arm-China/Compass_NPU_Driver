# Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

MD  := mkdir -p
RM  := rm -rf

TEST_CASE := simulation_test
SO_DIR := ./umd
BUILD_DIR := ./umd
DIR_TARGET := $(BUILD_DIR)
TARGET := $(BUILD_DIR)/aipu_$(TEST_CASE)
CXXFLAGS = -O0 -g -Wall -Werror -std=c++14 -I./src -DX86_LINUX=1
LDFLAGS = -L$(SO_DIR) -laipudrv -lpthread

SRC_DIR := ./src
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst %cpp, %o, $(SRCS))

all: $(DIR_TARGET) $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
$(DIR_TARGET):
	$(MD) $(BUILD_DIR)
$(TARGET): $(OBJS)
	$(CXX) $^ $(LDFLAGS) -o $@
	$(RM) $(OBJS)

clean:
	$(RM) $(OBJS) $(BUILD_DIR)

.PHONY: all clean
