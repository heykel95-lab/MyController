# Makefile for simplified impedance/compliance test with safe interaction limits
# This uses the local working libfranka build:
# /home/hm-panda/libfranka/build/libfranka.so.0.7

CXX := g++
TARGET := simplified_impedance_safe_interaction
SRC := simplified_impedance_safe_interaction.cpp

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

$(TARGET): $(SRC) $(FRANKA_DIR)/examples/examples_common.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	./$(TARGET)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -f $(TARGET)
