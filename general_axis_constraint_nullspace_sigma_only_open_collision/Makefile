# Makefile for simplified impedance/compliance test with 6-axis threshold compensation
# This uses the local working libfranka build:
# /home/hm-panda/libfranka/build/libfranka.so.0.7

CXX := g++
TARGET := general_axis_constraint_nullspace_sigma_only_open_collision
TOOL_OFFSET_TARGET := check_tool_offset
SRC := main.cpp
TOOL_OFFSET_SRC := check_tool_offset.cpp

# Implementation units split out of the controller_*.h headers. Each is
# compiled once into its own .o and linked into both programs, so the function
# bodies are no longer recompiled by every translation unit that includes them.
CONTROLLER_SRCS := controller_helpers.cpp \
                   controller_parameters.cpp \
                   controller_logging.cpp \
                   controller_printing.cpp

HEADERS := controller_common.h \
           controller_types.h \
           controller_helpers.h \
           controller_parameters.h \
           controller_logging.h \
           controller_printing.h

FRANKA_DIR := /home/hm-panda/libfranka
EIGEN_INC := /usr/include/eigen3

CXXFLAGS := -std=c++14 -O2 -Wall -Wextra \
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

$(TOOL_OFFSET_TARGET): $(TOOL_OFFSET_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(TOOL_OFFSET_SRC) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ) -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -f $(TARGET) $(TOOL_OFFSET_TARGET) $(CONTROLLER_OBJS) $(EXAMPLES_COMMON_OBJ)
