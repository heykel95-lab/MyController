#pragma once

#include "controller_types.h"

std::string trim(const std::string& input);

std::string removeSpaces(std::string value);

double parseDoubleValue(const std::string& input);

Parameters readParameters(const std::string& filename);
