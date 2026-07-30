# Makefile for the approach / set-up / grind controller.
# This uses the local working libfranka build:
# /home/hm-panda/libfranka/build/libfranka.so.0.7

CXX := g++
TARGET := surface_grinding_controller
TOOL_OFFSET_TARGET := check_tool_offset
MEASURE_PLANE_TARGET := measure_plane
CAPTURE_PLANE_POINT_TARGET := capture_plane_point
CAPTURE_TOOL_AXIS_TARGET := capture_tool_axis
HOME_GRIPPER_TARGET := home_gripper
INSPECT_CONFIG_TARGET := inspect_experiment_config
SRC := main.cpp
TOOLS_DIR := tools
TOOL_OFFSET_SRC := $(TOOLS_DIR)/check_tool_offset.cpp
TOOL_OFFSET_BIN := $(TOOLS_DIR)/$(TOOL_OFFSET_TARGET)
MEASURE_PLANE_SRC := $(TOOLS_DIR)/measure_plane.cpp
MEASURE_PLANE_BIN := $(TOOLS_DIR)/$(MEASURE_PLANE_TARGET)
CAPTURE_PLANE_POINT_SRC := $(TOOLS_DIR)/capture_plane_point.cpp
CAPTURE_PLANE_POINT_BIN := $(TOOLS_DIR)/$(CAPTURE_PLANE_POINT_TARGET)
CAPTURE_TOOL_AXIS_SRC := $(TOOLS_DIR)/capture_tool_axis.cpp
CAPTURE_TOOL_AXIS_BIN := $(TOOLS_DIR)/$(CAPTURE_TOOL_AXIS_TARGET)
HOME_GRIPPER_SRC := $(TOOLS_DIR)/home_gripper.cpp
HOME_GRIPPER_BIN := $(TOOLS_DIR)/$(HOME_GRIPPER_TARGET)
INSPECT_CONFIG_SRC := $(TOOLS_DIR)/inspect_experiment_config.cpp
INSPECT_CONFIG_BIN := $(TOOLS_DIR)/$(INSPECT_CONFIG_TARGET)

# Implementation units split out of the module headers. Each is
# compiled once into its own .o and linked into both programs, so the function
# bodies are no longer recompiled by every translation unit that includes them.
CONTROLLER_SRCS := config.cpp \
                   control_math.cpp \
                   runtime_io.cpp \
                   setup_report.cpp

# The controller program's own units: one run, split by what it does. The
# read-only tools do not link these.
RUN_SRCS := run_gains.cpp \
            run_loop.cpp \
            run_report.cpp

HEADERS := controller.h

FRANKA_DIR := /home/hm-panda/libfranka
EIGEN_INC := /usr/include/eigen3

CXXFLAGS := -std=c++14 -O2 -Wall -Wextra \
            -I. \
            -I$(FRANKA_DIR)/include \
            -I$(FRANKA_DIR)/examples \
            -I$(EIGEN_INC)

LDFLAGS := -L$(FRANKA_DIR)/build \
           -Wl,-rpath,$(FRANKA_DIR)/build

LDLIBS := -lfranka -lpthread

CONTROLLER_OBJS := $(CONTROLLER_SRCS:.cpp=.o)
RUN_OBJS := $(RUN_SRCS:.cpp=.o)
EXAMPLES_COMMON_OBJ := examples_common.o

.PHONY: all run check clean

all: $(TARGET)

# Compile each controller implementation unit on its own. Recompiles only when
# that .cpp or any header changes.
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# libfranka's shared example helper, built from outside this directory.
$(EXAMPLES_COMMON_OBJ): $(FRANKA_DIR)/examples/examples_common.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(SRC) $(CONTROLLER_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SRC) $(CONTROLLER_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(TOOL_OFFSET_TARGET): $(TOOL_OFFSET_BIN)

$(TOOL_OFFSET_BIN): $(TOOL_OFFSET_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(TOOL_OFFSET_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(MEASURE_PLANE_TARGET): $(MEASURE_PLANE_BIN) $(HOME_GRIPPER_BIN)

$(MEASURE_PLANE_BIN): $(MEASURE_PLANE_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MEASURE_PLANE_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(CAPTURE_PLANE_POINT_TARGET): $(CAPTURE_PLANE_POINT_BIN)

$(CAPTURE_PLANE_POINT_BIN): $(CAPTURE_PLANE_POINT_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(CAPTURE_PLANE_POINT_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(CAPTURE_TOOL_AXIS_TARGET): $(CAPTURE_TOOL_AXIS_BIN)

$(CAPTURE_TOOL_AXIS_BIN): $(CAPTURE_TOOL_AXIS_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(CAPTURE_TOOL_AXIS_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(HOME_GRIPPER_TARGET): $(HOME_GRIPPER_BIN)

$(HOME_GRIPPER_BIN): $(HOME_GRIPPER_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(HOME_GRIPPER_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(INSPECT_CONFIG_TARGET): $(INSPECT_CONFIG_BIN)

$(INSPECT_CONFIG_BIN): $(INSPECT_CONFIG_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INSPECT_CONFIG_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -f $(TARGET) $(TOOL_OFFSET_BIN) $(MEASURE_PLANE_BIN) $(CAPTURE_PLANE_POINT_BIN) $(CAPTURE_TOOL_AXIS_BIN) $(INSPECT_CONFIG_BIN) $(CONTROLLER_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ)
