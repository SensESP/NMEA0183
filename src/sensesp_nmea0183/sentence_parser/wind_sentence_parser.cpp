#include "wind_sentence_parser.h"

#include "field_parsers.h"

namespace sensesp::nmea0183 {

bool WIMWVSentenceParser::parse_fields(const char* field_strings,
                                       const int field_offsets[],
                                       int num_fields) {
  bool ok = true;

  float wind_speed;
  float wind_angle;

  //      0,  1,2,  3,4,5
  // $WIMWV,a.a,R,s.s,N,A*hhhh<CR><LF>
  // where a.a is the apparent wind angle in degrees
  //       s.s is the relative wind speed in knots

  if (num_fields < 6) {
    return false;
  }

  char r_value;
  char units;
  char a_value;

  std::function<bool(const char*)> fps[] = {// 1 a.a = Apparent wind angle
                                            FLDP_OPT(Float, &wind_angle),
                                            // 2 R = Relative wind speed
                                            FLDP_OPT(Char, &r_value, 'R'),
                                            // 3 s.s = Wind speed
                                            FLDP_OPT(Float, &wind_speed),
                                            // 4 N = Wind speed units
                                            FLDP_OPT(Char, &units, 255),
                                            // 5 A = Valid
                                            FLDP_OPT(Char, &a_value, 'A')};

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]);
       i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
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

bool MWDSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float true_direction;
  float magnetic_direction;
  float speed_knots;
  float speed_ms;
  char t_char;
  char m_char;
  char n_char;
  char ms_char;

  // $xxMWD,dir_true,T,dir_mag,M,speed_kn,N,speed_ms,M*cs
  // eg. $WIMWD,225.0,T,220.0,M,12.5,N,6.4,M*1A

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Wind direction, degrees true
      FLDP_OPT(Float, &true_direction),
      // 2   T = true
      FLDP_OPT(Char, &t_char, 'T'),
      // 3   Wind direction, degrees magnetic
      FLDP_OPT(Float, &magnetic_direction),
      // 4   M = magnetic
      FLDP_OPT(Char, &m_char, 'M'),
      // 5   Wind speed, knots
      FLDP_OPT(Float, &speed_knots),
      // 6   N = knots
      FLDP_OPT(Char, &n_char, 'N'),
      // 7   Wind speed, m/s
      FLDP_OPT(Float, &speed_ms),
      // 8   M = m/s
      FLDP_OPT(Char, &ms_char, 'M'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (true_direction != kInvalidFloat) {
    true_wind_direction_.set(true_direction * DEG_TO_RAD);
  }
  // Prefer m/s; fallback to knots
  if (speed_ms != kInvalidFloat) {
    true_wind_speed_.set(speed_ms);
  } else if (speed_knots != kInvalidFloat) {
    true_wind_speed_.set(speed_knots * 1852.0 / 3600.0);
  }

  return true;
}

bool VWRSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float angle;
  char lr_char;
  float speed_knots;
  float speed_ms;
  float speed_kmh;
  char n_char;
  char ms_char;
  char k_char;

  // $xxVWR,angle,L/R,speed_kn,N,speed_ms,M,speed_kmh,K*cs
  // eg. $IIVWR,045.0,R,12.5,N,6.4,M,23.2,K*52

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Wind angle, 0-180 degrees
      FLDP_OPT(Float, &angle),
      // 2   L = port, R = starboard
      FLDP_OPT(Char, &lr_char, 255),
      // 3   Wind speed, knots
      FLDP_OPT(Float, &speed_knots),
      // 4   N = knots
      FLDP_OPT(Char, &n_char, 'N'),
      // 5   Wind speed, m/s
      FLDP_OPT(Float, &speed_ms),
      // 6   M = m/s
      FLDP_OPT(Char, &ms_char, 'M'),
      // 7   Wind speed, km/h
      FLDP_OPT(Float, &speed_kmh),
      // 8   K = km/h
      FLDP_OPT(Char, &k_char, 'K'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (angle != kInvalidFloat) {
    float signed_angle = angle * DEG_TO_RAD;
    if (lr_char == 'L') {
      signed_angle = -signed_angle;
    }
    apparent_wind_angle_.set(signed_angle);
  }

  // Prefer m/s; fallback to knots, then km/h
  if (speed_ms != kInvalidFloat) {
    apparent_wind_speed_.set(speed_ms);
  } else if (speed_knots != kInvalidFloat) {
    apparent_wind_speed_.set(speed_knots * 1852.0 / 3600.0);
  } else if (speed_kmh != kInvalidFloat) {
    apparent_wind_speed_.set(speed_kmh * 1000.0 / 3600.0);
  }

  return true;
}

}  // namespace sensesp::nmea0183
