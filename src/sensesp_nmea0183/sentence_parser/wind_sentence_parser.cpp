#include "wind_sentence_parser.h"

namespace sensesp {

bool WIMWVSentenceParser::parse_fields(const char* field_strings,
                                       const int field_offsets[],
                                       int num_fields) {
  bool ok = true;

  float wind_speed;
  float wind_angle;

  // $WIMWV,a.a,R,s.s,N,A*hhhh<CR><LF>
  // where a.a is the apparent wind angle in degrees
  //       s.s is the relative wind speed in knots

  if (num_fields < 5) {
    ReportFailure(false, sentence_address());
    return false;
  }

  // 1 a.a = Apparent wind angle
  ok &= ParseFloat(&wind_angle, field_strings + field_offsets[0], false);

  // 2 R = Relative wind speed
  char r_value;
  ok &= ParseChar(&r_value, field_strings + field_offsets[1], 'R', false);

  // 3 s.s = Wind speed
  ok &= ParseFloat(&wind_speed, field_strings + field_offsets[2], false);

  // 4 N = Wind speed units
  char units;
  ok &= ParseChar(&units, field_strings + field_offsets[3], 255, false);

  // 5 A = Valid
  char a_value;
  ok &= ParseChar(&a_value, field_strings + field_offsets[4], 'A', false);

  ReportFailure(ok, sentence_address());
  if (!ok) {
    return false;
  }

  // K = km/hr
  // M = m/s
  // N = knots
  // S = statute miles/hr

  // convert wind speed units to m/s
  if (units == 'K') wind_speed = wind_speed * 0.277778;
  if (units == 'M') wind_speed = wind_speed;
  if (units == 'N') wind_speed = wind_speed * 0.514444;
  if (units == 'S') wind_speed = wind_speed * 0.44704;

  apparent_wind_speed_.set(wind_speed);

  // convert wind angle to radians
  wind_angle = wind_angle * DEG_TO_RAD;
  apparent_wind_angle_.set(wind_angle);

  return true;
}

}  // namespace sensesp
