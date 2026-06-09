# Makefile for simplified impedance/compliance test with 6-axis threshold compensation
# This uses the local working libfranka build:
# /home/hm-panda/libfranka/build/libfranka.so.0.7

CXX := g++
TARGET := general_axis_constraint_nullspace_sigma_only_open_collision
TOOL_OFFSET_TARGET := check_tool_offset
SRC := general_axis_constraint_nullspace_sigma_only_open_collision.cpp
TOOL_OFFSET_SRC := check_tool_offset.cpp
HEADERS := controller_common.h controller_types.h controller_helpers.h

FRANKA_DIR := /home/hm-panda/libfranka
EIGEN_INC := /usr/include/eigen3

CXXFLAGS := -std=c++14 -O2 -Wall -Wextra \
            -I$(FRANKA_DIR)/include \
            -I$(FRANKA_DIR)/examples \
            -I$(EIGEN_INC)

LDFLAGS := -L$(FRANKA_DIR)/build \
           -Wl,-rpath,$(FRANKA_DIR)/build

LDLIBS := -lfranka -lpthread

.PHONY: all run check clean

all: $(TARGET)

$(TARGET): $(SRC) $(HEADERS) $(FRANKA_DIR)/examples/examples_common.cpp
	$(CXX) $(CXXFLAGS) $(SRC) $(FRANKA_DIR)/examples/examples_common.cpp -o $@ $(LDFLAGS) $(LDLIBS)

$(TOOL_OFFSET_TARGET): $(TOOL_OFFSET_SRC) $(HEADERS) $(FRANKA_DIR)/examples/examples_common.cpp
	$(CXX) $(CXXFLAGS) $(TOOL_OFFSET_SRC) $(FRANKA_DIR)/examples/examples_common.cpp -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -f $(TARGET) $(TOOL_OFFSET_TARGET)
