#include "wind_sentence_parser.h"

#include "field_parsers.h"

namespace sensesp::nmea0183 {

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
    return false;
  }

  char r_value;
  char units;
  char a_value;

  std::function<bool(const char*)> fps[] = {
    // 1 a.a = Apparent wind angle
    FLDP_OPT(Float, &wind_angle),
    // 2 R = Relative wind speed
    FLDP_OPT(Char, &r_value, 'R'),
    // 3 s.s = Wind speed
    FLDP_OPT(Float, &wind_speed),
    // 4 N = Wind speed units
    FLDP_OPT(Char, &units, 255),
    // 5 A = Valid
    FLDP_OPT(Char, &a_value, 'A')
  };

  int i = 0;
  for (auto& fp : fps) {
    ok &= fp(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  // K = km/hr
  // M = m/s
  // N = knots
  // S = statute miles/hr

  float conv_ratio = 1.0;
  switch (units) {
    case 'K':
      conv_ratio = 0.277778;
      break;
    case 'M':
      conv_ratio = 1.0;
      break;
    case 'N':
      conv_ratio = 0.514444;
      break;
    case 'S':
      conv_ratio = 0.44704;
      break;
    default:
      return false;
  }

  wind_speed *= conv_ratio;

  apparent_wind_speed_.set(wind_speed);

  // convert wind angle to radians
  wind_angle = wind_angle * DEG_TO_RAD;
  apparent_wind_angle_.set(wind_angle);

  return true;
}

}  // namespace sensesp::nmea0183
