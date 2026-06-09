#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"

using Array6 = std::array<double, 6>;
using Array7 = std::array<double, 7>;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6x7 = Eigen::Matrix<double, 6, 7>;
using Mat4x4 = Eigen::Matrix<double, 4, 4>;
