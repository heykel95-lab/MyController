cmake_minimum_required(VERSION 3.10)

project(simplified_compliance_test LANGUAGES CXX)

# Use C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen.
find_package(Eigen3 REQUIRED)

# Find libfranka.
# This works if libfranka was installed with CMake package files.
find_package(Franka REQUIRED)

# Path to the libfranka examples folder on your system.
set(LIBFRANKA_EXAMPLES_DIR "/home/hm-panda/libfranka/examples")

# Build executable.
# examples_common.cpp is needed because it contains:
# - setDefaultBehavior(robot)
# - MotionGenerator
add_executable(simplified_compliance_test
    simplified_compliance_test.cpp
    ${LIBFRANKA_EXAMPLES_DIR}/examples_common.cpp
)

# Include directories.
target_include_directories(simplified_compliance_test PRIVATE
    ${LIBFRANKA_EXAMPLES_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link libraries.
target_link_libraries(simplified_compliance_test
    Franka::Franka
    Eigen3::Eigen
)
