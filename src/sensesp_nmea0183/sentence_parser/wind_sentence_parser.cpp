#include "wind_sentence_parser.h"

#include "field_parsers.h"
#include "sensesp/types/nullable.h"
using sensesp::Nullable;

namespace sensesp::nmea0183 {

bool MWVSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> wind_speed;
  Nullable<float> wind_angle;
  char r_value;
  char units;
  char a_value;

  //      0,  1,2,  3,4,5
  // $xxMWV,a.a,R,s.s,N,A*hh
  // where a.a is the apparent wind angle in degrees
  //       s.s is the wind speed

  if (num_fields < 6) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {// 1 a.a = Apparent wind angle
                                            FLDP_OPT(Float, wind_angle.ptr()),
                                            // 2 R = Relative wind speed
                                            FLDP_OPT(Char, &r_value, 'R'),
                                            // 3 s.s = Wind speed
                                            FLDP_OPT(Float, wind_speed.ptr()),
                                            // 4 N = Wind speed units
                                            FLDP_OPT(Char, &units, 255),
                                            // 5 A = Valid
                                            FLDP_OPT(Char, &a_value, 'A')};

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (wind_speed.is_valid()) {
    float speed = wind_speed;
    if (!ConvertSpeedToMs(&speed, units)) {
      return false;
    }
    apparent_wind_speed_.set(speed);
  }
  if (wind_angle.is_valid()) {
    apparent_wind_angle_.set(wind_angle * DEG_TO_RAD);
  }

  return true;
}

bool TrueWindMWVSentenceParser::parse_fields(const char* field_strings,
                                              const int field_offsets[],
                                              int num_fields) {
  Nullable<float> wind_speed;
  Nullable<float> wind_angle;
  char t_value;
  char units;
  char a_value;

  //      0,  1,2,  3,4,5
  // $xxMWV,a.a,T,s.s,N,A*hh
  // where a.a is the true wind direction in degrees
  //       s.s is the wind speed

  if (num_fields < 6) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {// 1 a.a = True wind direction
                                            FLDP_OPT(Float, wind_angle.ptr()),
                                            // 2 T = True wind
                                            FLDP_OPT(Char, &t_value, 'T'),
                                            // 3 s.s = Wind speed
                                            FLDP_OPT(Float, wind_speed.ptr()),
                                            // 4 N = Wind speed units
                                            FLDP_OPT(Char, &units, 255),
                                            // 5 A = Valid
                                            FLDP_OPT(Char, &a_value, 'A')};

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (wind_speed.is_valid()) {
    float speed = wind_speed;
    if (!ConvertSpeedToMs(&speed, units)) {
      return false;
    }
    true_wind_speed_.set(speed);
  }
  if (wind_angle.is_valid()) {
    true_wind_direction_.set(wind_angle * DEG_TO_RAD);
  }

  return true;
}

bool MWDSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  Nullable<float> true_direction;
  Nullable<float> magnetic_direction;
  Nullable<float> speed_knots;
  Nullable<float> speed_ms;
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
      FLDP_OPT(Float, true_direction.ptr()),
      // 2   T = true
      FLDP_OPT(Char, &t_char, 'T'),
      // 3   Wind direction, degrees magnetic
      FLDP_OPT(Float, magnetic_direction.ptr()),
      // 4   M = magnetic
      FLDP_OPT(Char, &m_char, 'M'),
      // 5   Wind speed, knots
      FLDP_OPT(Float, speed_knots.ptr()),
      // 6   N = knots
      FLDP_OPT(Char, &n_char, 'N'),
      // 7   Wind speed, m/s
      FLDP_OPT(Float, speed_ms.ptr()),
      // 8   M = m/s
      FLDP_OPT(Char, &ms_char, 'M'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (true_direction.is_valid()) {
    true_wind_direction_.set(true_direction * DEG_TO_RAD);
  }
  // Prefer m/s; fallback to knots
  if (speed_ms.is_valid()) {
    true_wind_speed_.set(speed_ms);
  } else if (speed_knots.is_valid()) {
    true_wind_speed_.set(speed_knots * 1852.0 / 3600.0);
  }

  return true;
}

bool VWRSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  Nullable<float> angle;
  char lr_char;
  Nullable<float> speed_knots;
  Nullable<float> speed_ms;
  Nullable<float> speed_kmh;
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
      FLDP_OPT(Float, angle.ptr()),
      // 2   L = port, R = starboard
      FLDP_OPT(Char, &lr_char, 255),
      // 3   Wind speed, knots
      FLDP_OPT(Float, speed_knots.ptr()),
      // 4   N = knots
      FLDP_OPT(Char, &n_char, 'N'),
      // 5   Wind speed, m/s
      FLDP_OPT(Float, speed_ms.ptr()),
      // 6   M = m/s
      FLDP_OPT(Char, &ms_char, 'M'),
      // 7   Wind speed, km/h
      FLDP_OPT(Float, speed_kmh.ptr()),
      // 8   K = km/h
      FLDP_OPT(Char, &k_char, 'K'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (angle.is_valid()) {
    float signed_angle = angle * DEG_TO_RAD;
    if (lr_char == 'L') {
      signed_angle = -signed_angle;
    }
    apparent_wind_angle_.set(signed_angle);
  }

  // Prefer m/s; fallback to knots, then km/h
  if (speed_ms.is_valid()) {
    apparent_wind_speed_.set(speed_ms);
  } else if (speed_knots.is_valid()) {
    apparent_wind_speed_.set(speed_knots * 1852.0 / 3600.0);
  } else if (speed_kmh.is_valid()) {
    apparent_wind_speed_.set(speed_kmh * 1000.0 / 3600.0);
  }

  return true;
}

}  // namespace sensesp::nmea0183
