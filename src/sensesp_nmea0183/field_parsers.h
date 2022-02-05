#pragma once

#include <limits>

namespace sensesp {

// magic values for invalid data
constexpr float kInvalidFloat = std::numeric_limits<float>::lowest();
constexpr double kInvalidDouble = std::numeric_limits<double>::lowest();
constexpr int kInvalidInt = std::numeric_limits<int>::lowest();

bool ParseInt(int* value, char* s, bool allow_empty = false);
bool ParseFloat(float* value, char* s, bool allow_empty = false);
bool ParseLatLon(double* value, char* s, bool allow_empty = false);
bool ParseNS(double* value, char* s, bool allow_empty = false);
bool ParseEW(double* value, char* s, bool allow_empty = false);
bool ParseEW(float* value, char* s, bool allow_empty = false);
bool ParseChar(char* s, char expected);
bool ParseAV(bool* is_valid, char* s);

}  // namespace sensesp
