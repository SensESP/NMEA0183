#include "field_parsers.h"

#include <stdio.h>

#include <cstring>
#include <ctime>

#include "sensesp.h"

namespace sensesp {

bool ParseInt(int* value, char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = kInvalidInt;
    return allow_empty;
  }
  int retval = sscanf(s, "%d", value);
  return retval == 1;
}

bool ParseFloat(float* value, char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = kInvalidFloat;
    return allow_empty;
  }
  int retval = sscanf(s, "%f", value);
  return retval == 1;
}

bool ParseDouble(double* value, char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = kInvalidDouble;
    return allow_empty;
  }
  int retval = sscanf(s, "%lf", value);
  return retval == 1;
}

bool ParseLatLon(double* value, char* s, bool allow_empty) {
  double degmin;
  if (s[0] == 0) {
    *value = kInvalidDouble;
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

bool ParseNS(double* value, char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }

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

bool ParseEW(double* value, char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
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

bool ParseEW(float* value, char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
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

bool ParseChar(char* value, char* s, char expected, bool allow_empty) {
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

bool ParseAV(bool* is_valid, char* s) {
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

bool ParseTime(int* hour, int* minute, float* second, char* s,
               bool allow_empty) {
  if (s[0] == 0) {
    *hour = kInvalidInt;
    *minute = kInvalidInt;
    *second = kInvalidFloat;
    return allow_empty;
  }
  int retval = sscanf(s, "%2d%2d%f", hour, minute, second);
  return retval == 3;
}

bool ParseDate(int* year, int* month, int* day, char* s, bool allow_empty) {
  if (s[0] == 0) {
    *year = kInvalidInt;
    *month = kInvalidInt;
    *day = kInvalidInt;
    return allow_empty;
  }
  int retval = sscanf(s, "%2d%2d%2d", day, month, year);
  // date expressed as C struct tm
  *year += 100;
  *month -= 1;
  return retval == 3;
}

}  // namespace sensesp
