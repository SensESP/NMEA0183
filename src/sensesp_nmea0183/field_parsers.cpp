#include "field_parsers.h"

#include <stdio.h>

#include <ctime>

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

bool ParseChar(char* value, char* s, char expected, bool allow_empty) {
  if (s[0] == 0) {
    *value = 0;
    return allow_empty;
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

}  // namespace sensesp
