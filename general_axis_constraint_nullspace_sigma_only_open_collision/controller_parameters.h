#pragma once

#include "controller_types.h"

std::string trim(const std::string& input);

std::string removeSpaces(std::string value);

double parseDoubleValue(const std::string& input);

// Reads and merges several parameter files into one Parameters. Files are read
// in order into a shared key->value map (later files override earlier on a
// duplicate key, though the split files keep keys disjoint). Missing files are
// skipped with a warning; if none open, defaults are used.
Parameters readParameters(const std::vector<std::string>& filenames);
