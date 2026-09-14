#include "field_parsers.h"

#include <stdio.h>

#include <cstring>
#include <ctime>

#include "sensesp.h"
#include "sensesp/types/nullable.h"

namespace sensesp::nmea0183 {

using sensesp::Nullable;

bool ParseString(String* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = "";
    return allow_empty;
  }
  *value = s;
  return true;
}

bool ParseInt(int* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = Nullable<int>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%d", value);
  return retval == 1;
}

bool ParseFloat(float* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = Nullable<float>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%f", value);
  return retval == 1;
}

bool ParseDouble(double* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = Nullable<double>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%lf", value);
  return retval == 1;
}

bool ParseLatLon(double* value, const char* s, bool allow_empty) {
  double degmin;
  if (s[0] == 0) {
    *value = Nullable<double>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%lf", &degmin);
  if (retval == 1) {
    int degrees = degmin / 100;
    double minutes = degmin - 100 * degrees;
    *value = degrees + minutes / 60;
    return true;
  } else {
    return false;
  }
}

bool ParseNS(double* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
  if (*value == Nullable<double>::invalid()) return true;

  switch (*s) {
    case 'N':
      break;
    case 'S':
      *value *= -1;
      break;
    default:
      return false;
  }
  return true;
}

bool ParseEW(double* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
  if (*value == Nullable<double>::invalid()) return true;

  switch (*s) {
    case 'E':
      break;
    case 'W':
      *value *= -1;
      break;
    default:
      return false;
  }
  return true;
}

bool ParseEW(float* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
  if (*value == Nullable<float>::invalid()) return true;

  switch (*s) {
    case 'E':
      break;
    case 'W':
      *value *= -1;
      break;
    default:
      return false;
  }
  return true;
}

#include <cstring>

bool ParseChar(char* value, const char expected, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = 0;
    return allow_empty;
  }
  if (strlen(s) > 1) {
    return false;
  }
  *value = *s;
  if (expected == 255) {
    return true;
  }
  return (*s == expected);
}

bool ParseAV(bool* is_valid, const char* s) {
  switch (*s) {
    case 'A':
      *is_valid = true;
      break;
    case 'V':
      *is_valid = false;
      break;
    default:
      return false;
  }
  return true;
}

bool ParseTime(int* hour, int* minute, float* second, const char* s,
               bool allow_empty) {
  if (s[0] == 0) {
    *hour = Nullable<int>::invalid();
    *minute = Nullable<int>::invalid();
    *second = Nullable<float>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%2d%2d%f", hour, minute, second);
  return retval == 3;
}

bool ParseDate(int* year, int* month, int* day, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *year = Nullable<int>::invalid();
    *month = Nullable<int>::invalid();
    *day = Nullable<int>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%2d%2d%2d", day, month, year);
  // date expressed as C struct tm
  *year += 100;
  *month -= 1;
  return retval == 3;
}

bool ParseEmpty(const char* s) {
  return s[0] == 0;
}

bool ConvertSpeedToMs(float* speed, char unit) {
  float conv_ratio;
  switch (unit) {
    case 'K':  // km/h
      conv_ratio = 0.277778;
      break;
    case 'M':  // m/s
      conv_ratio = 1.0;
      break;
    case 'N':  // knots
      conv_ratio = 0.514444;
      break;
    case 'S':  // statute miles/h
      conv_ratio = 0.44704;
      break;
    default:
      return false;
  }
  *speed *= conv_ratio;
  return true;
}

}  // namespace sensesp::nmea0183
