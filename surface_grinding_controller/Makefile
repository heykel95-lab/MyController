# Makefile for the approach / set-up / grind controller.
# This uses the local working libfranka build:
# /home/hm-panda/libfranka/build/libfranka.so.0.7
#
# Layout: run/ is the program, modules/ is what it calls, tools/ are the
# read-only helpers. Object files go to build/; the binaries stay where they
# are run from.

CXX := g++
TARGET := surface_grinding_controller
BUILD := build

# run/: the program, in the order it executes.
MAIN_SRC := run/main.cpp
RUN_SRCS := run/run_gains.cpp \
            run/run_loop.cpp \
            run/run_report.cpp

# modules/: what a run calls. Compiled once and linked into every program.
MODULE_SRCS := modules/config.cpp \
               modules/control_math.cpp \
               modules/runtime_io.cpp \
               modules/setup_report.cpp

HEADERS := controller.h

TOOLS_DIR := tools
TOOL_OFFSET_TARGET := check_tool_offset
MEASURE_PLANE_TARGET := measure_plane
CAPTURE_PLANE_POINT_TARGET := capture_plane_point
CAPTURE_TOOL_AXIS_TARGET := capture_tool_axis
HOME_GRIPPER_TARGET := home_gripper
INSPECT_CONFIG_TARGET := inspect_experiment_config

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

MODULE_OBJS := $(addprefix $(BUILD)/,$(notdir $(MODULE_SRCS:.cpp=.o)))
RUN_OBJS := $(addprefix $(BUILD)/,$(notdir $(RUN_SRCS:.cpp=.o)))
EXAMPLES_COMMON_OBJ := $(BUILD)/examples_common.o

# The read-only tools need the modules; only the preflight also needs the gain
# construction that lives in run/.
TOOL_OBJS := $(MODULE_OBJS) $(EXAMPLES_COMMON_OBJ)
PREFLIGHT_OBJS := $(MODULE_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ)

.PHONY: all run check clean

all: $(TARGET)

$(BUILD):
	mkdir -p $(BUILD)

# One rule per source folder. Recompiles when that .cpp or a header changes.
$(BUILD)/%.o: modules/%.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD)/%.o: run/%.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# libfranka's shared example helper, built from outside this directory.
$(EXAMPLES_COMMON_OBJ): $(FRANKA_DIR)/examples/examples_common.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(MAIN_SRC) $(MODULE_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MAIN_SRC) $(MODULE_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(TOOL_OFFSET_TARGET): $(TOOL_OFFSET_BIN)

$(TOOL_OFFSET_BIN): $(TOOL_OFFSET_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(TOOL_OFFSET_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(MEASURE_PLANE_TARGET): $(MEASURE_PLANE_BIN) $(HOME_GRIPPER_BIN)

$(MEASURE_PLANE_BIN): $(MEASURE_PLANE_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MEASURE_PLANE_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(CAPTURE_PLANE_POINT_TARGET): $(CAPTURE_PLANE_POINT_BIN)

$(CAPTURE_PLANE_POINT_BIN): $(CAPTURE_PLANE_POINT_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(CAPTURE_PLANE_POINT_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(CAPTURE_TOOL_AXIS_TARGET): $(CAPTURE_TOOL_AXIS_BIN)

$(CAPTURE_TOOL_AXIS_BIN): $(CAPTURE_TOOL_AXIS_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(CAPTURE_TOOL_AXIS_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(HOME_GRIPPER_TARGET): $(HOME_GRIPPER_BIN)

$(HOME_GRIPPER_BIN): $(HOME_GRIPPER_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(HOME_GRIPPER_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(INSPECT_CONFIG_TARGET): $(INSPECT_CONFIG_BIN)

$(INSPECT_CONFIG_BIN): $(INSPECT_CONFIG_SRC) $(PREFLIGHT_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INSPECT_CONFIG_SRC) $(PREFLIGHT_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -rf $(BUILD)
	rm -f $(TARGET) $(TOOL_OFFSET_BIN) $(MEASURE_PLANE_BIN) \
	      $(CAPTURE_PLANE_POINT_BIN) $(CAPTURE_TOOL_AXIS_BIN) \
	      $(HOME_GRIPPER_BIN) $(INSPECT_CONFIG_BIN)
