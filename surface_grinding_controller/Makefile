# Makefile for the approach / set-up / grind controller.
# This uses the local working libfranka build:
# /home/hm-panda/libfranka/build/libfranka.so.0.7

CXX := g++
TARGET := surface_grinding_controller
TOOL_OFFSET_TARGET := check_tool_offset
MEASURE_PLANE_TARGET := measure_plane
HOME_GRIPPER_TARGET := home_gripper
SRC := main.cpp
TOOLS_DIR := tools
TOOL_OFFSET_SRC := $(TOOLS_DIR)/check_tool_offset.cpp
TOOL_OFFSET_BIN := $(TOOLS_DIR)/$(TOOL_OFFSET_TARGET)
MEASURE_PLANE_SRC := $(TOOLS_DIR)/measure_plane.cpp
MEASURE_PLANE_BIN := $(TOOLS_DIR)/$(MEASURE_PLANE_TARGET)
HOME_GRIPPER_SRC := $(TOOLS_DIR)/home_gripper.cpp
HOME_GRIPPER_BIN := $(TOOLS_DIR)/$(HOME_GRIPPER_TARGET)

# Implementation units split out of the module headers. Each is
# compiled once into its own .o and linked into both programs, so the function
# bodies are no longer recompiled by every translation unit that includes them.
CONTROLLER_SRCS := config.cpp \
                   control_math.cpp \
                   runtime_io.cpp \
                   setup_report.cpp

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

$(TARGET): $(SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(TOOL_OFFSET_TARGET): $(TOOL_OFFSET_BIN)

$(TOOL_OFFSET_BIN): $(TOOL_OFFSET_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(TOOL_OFFSET_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(MEASURE_PLANE_TARGET): $(MEASURE_PLANE_BIN) $(HOME_GRIPPER_BIN)

$(MEASURE_PLANE_BIN): $(MEASURE_PLANE_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MEASURE_PLANE_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

$(HOME_GRIPPER_TARGET): $(HOME_GRIPPER_BIN)

$(HOME_GRIPPER_BIN): $(HOME_GRIPPER_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(HOME_GRIPPER_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -f $(TARGET) $(TOOL_OFFSET_BIN) $(MEASURE_PLANE_BIN) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ)
