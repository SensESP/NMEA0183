#ifndef SENSESP_NMEA0183_FIELD_PARSERS_H_
#define SENSESP_NMEA0183_FIELD_PARSERS_H_

#include <limits>
#include <Arduino.h>

namespace sensesp::nmea0183 {

// magic values for invalid data
constexpr float kInvalidFloat = std::numeric_limits<float>::lowest();
constexpr double kInvalidDouble = std::numeric_limits<double>::lowest();
constexpr int kInvalidInt = std::numeric_limits<int>::lowest();

bool ParseString(String* value, const char* s, bool allow_empty = false);
bool ParseInt(int* value, const char* s, bool allow_empty = false);
bool ParseFloat(float* value, const char* s, bool allow_empty = false);
bool ParseDouble(double* value, const char* s, bool allow_empty = false);
bool ParseLatLon(double* value, const char* s, bool allow_empty = false);
bool ParseNS(double* value, const char* s, bool allow_empty = false);
bool ParseEW(double* value, const char* s, bool allow_empty = false);
bool ParseEW(float* value, const char* s, bool allow_empty = false);
bool ParseChar(char* value, const char expected, const char* s, bool allow_empty = false);
bool ParseAV(bool* is_valid, const char* s);

bool ParseTime(int* hour, int* minute, float* second, const char* s,
               bool allow_empty = false);

bool ParseDate(int* year, int* month, int* day, const char* s,
               bool allow_empty = false);

// Field parser macro that can be used to define sentence parsers as arrays of
// field parsers.

#define FLDP(f, ...) [&](const char *s){return Parse##f(__VA_ARGS__ __VA_OPT__(,) s);}

// Field Parser, optional field

#define FLDP_OPT(f, ...) [&](const char *s){return Parse##f(__VA_ARGS__ __VA_OPT__(,) s, false);}

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_FIELD_PARSERS_H_
