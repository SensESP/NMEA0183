
#include "gnss_sentence_parser.h"

#include <functional>

#include "Arduino.h"

namespace sensesp::nmea0183 {

using namespace std::placeholders;

String gnss_quality_strings[] = {"no GPS",
                                 "GNSS Fix",
                                 "DGNSS fix",
                                 "Precise GNSS",
                                 "RTK fixed integer",
                                 "RTK float",
                                 "Estimated (DR) mode",
                                 "Manual input",
                                 "Simulator mode",
                                 "Error"};

static bool ParseSkyTraqPSTI030Mode(SkyTraqGNSSQuality* quality, const char* s) {
  switch (*s) {
    case 'N':
      *quality = SkyTraqGNSSQuality::no_gps;
      break;
    case 'A':
      *quality = SkyTraqGNSSQuality::gnss_fix;
      break;
    case 'D':
      *quality = SkyTraqGNSSQuality::dgnss_fix;
      break;
    case 'E':
      *quality = SkyTraqGNSSQuality::estimated_mode;
      break;
    case 'M':
      *quality = SkyTraqGNSSQuality::manual_input;
      break;
    case 'S':
      *quality = SkyTraqGNSSQuality::simulator_mode;
      break;
    case 'F':
      *quality = SkyTraqGNSSQuality::rtk_float;
      break;
    case 'R':
      *quality = SkyTraqGNSSQuality::rtk_fixed_integer;
      break;
    default:
      return false;
  }
  return true;
}

bool GGASentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  int hour;
  int minute;
  float second;
  Position position;
  int quality;
  int num_satellites;
  float horizontal_dilution;
  float geoidal_separation;
  float dgps_age;
  int dgps_id;
  char antenna_height_unit;
  char geoidal_separation_unit;

  // clang-format off
  // field     0         1          2 3           4 5 6  7     8    9 10   11 12  13   14 15
  // eg.  $GPGGA,hhmmss.ss,llll.ll   ,a,yyyyy.yy   ,a,x,xx,x.x  ,x.x , M,x.x , M,x.x,xxxx*hh
  // eg2. $GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04 ,17.0, M,17.6, M,   ,0000*75
  // eg3. $GNGGA,121224.00,          , ,           , ,0,00,99.99,    ,  ,    ,  ,   ,    *7E
  // clang-format on

  if (num_fields < 15) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1    = UTC of Position
      FLDP(Time, &hour, &minute, &second),
      // 2    = Latitude
      FLDP(LatLon, &position.latitude),
      // 3    = N or S
      FLDP(NS, &position.latitude),
      // 4    = Longitude
      FLDP(LatLon, &position.longitude),
      // 5    = E or W
      FLDP(EW, &position.longitude),
      // 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
      FLDP(Int, &quality),
      // 7    = Number of satellites in use [not those in view]
      FLDP(Int, &num_satellites),
      // 8    = Horizontal dilution of position
      FLDP(Float, &horizontal_dilution),
      // 9    = Antenna altitude above/below mean sea level (geoid)
      FLDP(Float, &position.altitude),
      // 10   = Meters  (Antenna height unit)
      FLDP(Char, &antenna_height_unit, 'M'),  // M = meters
      // 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
      //        mean sea level.  -=geoid is below WGS-84 ellipsoid)
      FLDP(Float, &geoidal_separation),
      // 12   = Meters  (Units of geoidal separation)
      FLDP(Char, &geoidal_separation_unit, 'M'),
      // 13   = Age in seconds since last update from diff. reference station
      FLDP(Float, &dgps_age),
      // 14   = Diff. reference station ID#
      FLDP(Int, &dgps_id)};

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  // notify relevant observers

  if (position.latitude != kInvalidDouble &&
      position.longitude != kInvalidDouble) {
    position_.set(position);
  }
  if (quality != kInvalidInt) {
    gnss_quality_.set(gnss_quality_strings[quality]);
  }

  num_satellites_.set(num_satellites);
  // remaining fields are relevant only if quality is not invalid (0)
  if (quality != 0) {
    if (horizontal_dilution != kInvalidFloat) {
      horizontal_dilution_.set(horizontal_dilution);
    }
    if (geoidal_separation != kInvalidFloat) {
      geoidal_separation_.set(geoidal_separation);
    }
    if (dgps_age != kInvalidFloat) {
      dgps_age_.set(dgps_age);
    }
    if (dgps_id != kInvalidInt) {
      dgps_id_.set(dgps_id);
    }
  }

  return true;
}

bool GLLSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  Position position;

  // eg.  $GPGLL,5133.81   ,N,00042.25   ,W              *75
  // eg2. $GNGLL,4916.45   ,N,12311.12   ,W,225444   ,A
  // eg3. $GNGLL,6011.07479,N,02503.05652,E,133453.00,A,D*7A
  // eg4. $GNGLL,          , ,           , ,121223.00,V,N*55

  if (num_fields < 5) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1    5133.81   Current latitude
      FLDP(LatLon, &position.latitude),
      // 2    N         North/South
      FLDP(NS, &position.latitude),
      // 3    00042.25  Current longitude
      FLDP(LatLon, &position.longitude),
      // 4    W         East/West
      FLDP(EW, &position.longitude)
      // ignore the UTC time of the fix and the status of the fix for now
  };

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  position.altitude = kPositionInvalidAltitude;

  // notify relevant observers

  if (position.latitude != kInvalidDouble &&
      position.longitude != kInvalidDouble) {
    position_.set(position);
  }

  return true;
}

bool RMCSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid = false;
  Position position;
  float speed;
  float true_course;
  float variation;

  // clang-format off
  // eg.  $GPRMC,220516,   A,5133.82,   N,00042.24,   W,173.8,231.8,130694,004.2,W  *70
  // eg2. $GNRMC,121042.00,A,6011.07385,N,02503.04396,E,0.087,     ,050222,     , ,D*64
  // eg3. $GNRMC,121224.00,V,          , ,           , ,     ,     ,060222,     , ,N*61
  // clang-format on

  if (num_fields < 12) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   220516     Time Stamp
      FLDP(Time, &time.tm_hour, &time.tm_min, &second),
      // 2   A          validity - A-ok, V-invalid
      FLDP(AV, &is_valid),
      // 3   5133.82    current Latitude
      FLDP(LatLon, &position.latitude),
      // 4   N          North/South
      FLDP(NS, &position.latitude),
      // 5   00042.24   current Longitude
      FLDP(LatLon, &position.longitude),
      // 6   W          East/West
      FLDP(EW, &position.longitude),
      // 7   173.8      Speed in knots
      FLDP(Float, &speed),
      // 8   231.8      True course
      FLDP(Float, &true_course),
      // 9   130694     Date Stamp
      FLDP(Date, &time.tm_year, &time.tm_mon, &time.tm_mday),
      // 10  004.2      Variation
      FLDP(Float, &variation),
      // 11  W          East/West
      FLDP(EW, &variation)

      // Positioning system mode indicator might be available as field 12, but
      // let's ignore it for now.
  };

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  position.altitude = kPositionInvalidAltitude;
  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  // notify relevant observers

  if (is_valid) {
    if (position.latitude != kInvalidDouble &&
        position.longitude != kInvalidDouble) {
      position_.set(position);
    }
    if (time.tm_year != kInvalidInt && time.tm_hour != kInvalidInt) {
      datetime_.set(mktime(&time));
    }
    if (speed != kInvalidFloat) {
      speed_.set(1852. * speed / 3600.);
    }
    if (true_course != kInvalidFloat) {
      true_course_.set(2 * PI * true_course / 360.);
    }
    if (variation != kInvalidFloat) {
      variation_.set(2 * PI * variation / 360.);
    }
  }

  return true;
}

bool VTGSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float true_track;
  float magnetic_track;
  float ground_speed;
  char true_track_symbol;
  char magnetic_track_symbol;
  char ground_speed_knots_unit;

  // clang-format off
  // eg.  $GNVTG,,T,,M,1.317,N,2.438,K,D*31
  // eg2. $GNVTG,, ,, ,     , ,     , ,N*2E
  // clang-format on

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   True track made good
      FLDP(Float, &true_track),
      // 2   T
      FLDP(Char, &true_track_symbol, 'T'),
      // 3   Magnetic track made good
      FLDP(Float, &magnetic_track),
      // 4   M
      FLDP(Char, &magnetic_track_symbol, 'M'),
      // 5   Ground speed, knots
      FLDP(Float, &ground_speed),
      // 6   N
      FLDP(Char, &ground_speed_knots_unit, 'N')
      // ignore the remaining fields for now
  };

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  // set observers

  if (true_track != kInvalidFloat) {
    true_course_.set(2 * PI * true_track / 360.);
  }
  // ignore magnetic track for now
  if (ground_speed != kInvalidFloat) {
    speed_.set(1852. * ground_speed / 3600.);
  }

  return true;
}

bool SkyTraqPSTI030SentenceParser::parse_fields(const char* field_strings,
                                         const int field_offsets[],
                                         int num_fields) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid = false;
  Position position;
  ENUVector velocity;
  SkyTraqGNSSQuality quality;
  float rtk_age;
  float rtk_ratio;

  // Example:
  // $PSTI,030,044606.000,A,2447.0924110,N,12100.5227860,E,103.323,0.00,0.00,0.00,180915,R,1.2,4.2*02

  // note: field offsets are one larger than in the reference because
  // the subsentence number is at offset 1

  if (num_fields < 15) {
    return false;
  }

  // Field  Name  Example  Description
  std::function<bool(const char*)> fps[] = {
      // 1  UTC time  044606.000  UTC time in hhmmss.sss format (000000.00 ~
      // 235959.999)
      FLDP(Time, &time.tm_hour, &time.tm_min, &second),
      // 2  Status  A  Status
      // ‘V’ = Navigation receiver warning
      // ‘A’ = Data Valid
      FLDP(AV, &is_valid),
      // 3  Latitude  2447.0924110  Latitude in dddmm.mmmmmmm format
      // Leading zeros transmitted
      FLDP(LatLon, &position.latitude),
      // 4  N/S indicator  N  Latitude hemisphere indicator
      // ‘N’ = North
      // ‘S’ = South
      FLDP(NS, &position.latitude),
      // 5  Longitude  12100.5227860 Longitude in dddmm.mmmmmmm format
      // Leading zeros transmitted
      FLDP(LatLon, &position.longitude),
      // 6  E/W Indicator  E  Longitude hemisphere indicator
      // 'E' = East
      // 'W' = West
      FLDP(EW, &position.longitude),
      // 7  Altitude  103.323  mean sea level (geoid), (‐9999.999 ~ 17999.999)
      FLDP(Float, &position.altitude),
      // 8  East Velocity  0.00  ‘East’ component of ENU velocity (m/s)
      FLDP(Float, &velocity.east),
      // 9  North Velocity  0.00  ‘North’ component of ENU velocity (m/s)
      FLDP(Float, &velocity.north),
      // 10  Up Velocity  0.00  ‘Up’ component of ENU velocity (m/s)
      FLDP(Float, &velocity.up),
      // 11  UTC Date  180915  UTC date of position fix, ddmmyy format
      FLDP(Date, &time.tm_year, &time.tm_mon, &time.tm_mday),
      // 12  Mode indicator  R  Mode indicator
      // ‘N’ = Data not valid
      // ‘A’ = Autonomous mode
      // ‘D’ = Differential mode
      // ‘E’ = Estimated (dead reckoning) mode
      // ‘M’ = Manual input mode
      // ‘S’ = Simulator mode
      // ‘F’ = Float RTK. Satellite system used in RTK mode, floating
      // integers
      // ‘R’ = Real Time Kinematic. System used in RTK mode with fixed
      // integers
      FLDP(SkyTraqPSTI030Mode, &quality),
      // 13  RTK Age  1.2  Age of differential
      FLDP(Float, &rtk_age),
      // 14  RTK Ratio  4.2  AR ratio factor for validation
      FLDP(Float, &rtk_ratio)};

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  // notify relevant observers

  gnss_quality_.set(gnss_quality_strings[quality]);
  rtk_age_.set(rtk_age);
  rtk_ratio_.set(rtk_ratio);

  if (is_valid) {
    position_.set(position);
    datetime_.set(mktime(&time));
    enu_velocity_.set(velocity);
  }

  return true;
}

bool SkyTraqPSTI032SentenceParser::parse_fields(const char* field_strings,
                                         const int field_offsets[],
                                         int num_fields) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid = false;
  ENUVector projection;
  SkyTraqGNSSQuality quality;
  float baseline_length;
  float baseline_course;

  // Example:
  // $PSTI,032,041457.000,170316,A,R,0.603,‐0.837,‐0.089,1.036,144.22,,,,,*30

  // note: field offsets are one larger than in the reference because
  // the subsentence number is at offset 1

  if (num_fields < 10) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1  UTC time  041457.000  UTC time in hhmmss.sss format
      // (000000.000~235959.999)
      FLDP(Time, &time.tm_hour, &time.tm_min, &second),
      // 2  UTC Date  170316  UTC date of position fix, ddmmyy format
      FLDP(Date, &time.tm_year, &time.tm_mon, &time.tm_mday),
      // 3  Status  A
      // Status
      // ‘V’ = Void
      // ‘A’ = Active
      FLDP(AV, &is_valid),
      // 4  Mode indicator  R
      // Mode indicator
      // ‘F’ = Float RTK. System used in RTK mode with float ambiguity
      // ‘R’ = Real Time Kinematic. System used in RTK mode with fixed
      // ambiguity
      FLDP(SkyTraqPSTI030Mode, &quality),
      // 5  East‐projection of
      // baseline  0.603  East‐projection of baseline, meters
      FLDP(Float, &projection.east),
      // 6  North‐projection of
      // baseline  ‐0.837  North‐projection of baseline, meters
      FLDP(Float, &projection.north),
      // 7  Up‐projection of
      // baseline  ‐0.089  Up‐projection of baseline, meters
      FLDP(Float, &projection.up),
      // 8  Baseline length  1.036  Baseline length, meters
      FLDP(Float, &baseline_length),
      // 9  Baseline course  144.22
      // Baseline course (angle between baseline vector and north
      // direction), degrees
      FLDP(Float, &baseline_course)
      // 10  Reserve    Reserve
      // 11  Reserve    Reserve
      // 12  Reserve    Reserve
      // 13  Reserve    Reserve
      // 14  Reserve    Reserve
  };

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  if (is_valid) {
    datetime_.set(mktime(&time));
    baseline_projection_.set(projection);
    baseline_length_.set(baseline_length);
    baseline_course_.set(2 * PI * baseline_course / 360.);
    gnss_quality_.set(gnss_quality_strings[quality]);
  }

  return true;
}

bool QuectelPQTMTARSentenceParser::parse_fields(const char* field_strings,
                                         const int field_offsets[],
                                         int num_fields) {
  bool ok = true;

  struct tm time;
  float second;
  float base_line_length;
  int heading_status;
  AttitudeVector attitude_degree;
  AttitudeVector attitude_accuracy_degree;
  int hdg_num_satellites;
  char dummy;

  // Example:
  // $PQTMTAR,1,165331.000,6,,0.232,2.321340,-6.849396,80.410065,0.081330,0.045079,0.054334,00*72

  if (num_fields < 14) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1 Message version. Should be 1.
      FLDP(Char, &dummy, '1'),
      // 2 UTC time 165331.000 UTC time in hhmmss.sss format (000000.000 ~
      // 235959.999)
      FLDP(Time, &time.tm_hour, &time.tm_min, &second),
      // 3 Heading status.
      FLDP(Int, &heading_status),
      // 4 Always empty.
      FLDP(Empty),
      // 5 Baseline length.
      FLDP(Float, &base_line_length),
      // 6 Pitch angle
      FLDP_OPT(Float, &attitude_degree.pitch),
      // 7 Roll angle
      FLDP_OPT(Float, &attitude_degree.roll),
      // 8 Yaw angle
      FLDP_OPT(Float, &attitude_degree.yaw),
      // 9 Pitch accuracy
      FLDP_OPT(Float, &attitude_accuracy_degree.pitch),
      // 10 Roll accuracy
      FLDP_OPT(Float, &attitude_accuracy_degree.roll),
      // 11 Yaw accuracy
      FLDP_OPT(Float, &attitude_accuracy_degree.yaw),
      // 12 Number of satellites used for heading calculation
      FLDP(Int, &hdg_num_satellites)};

  int i = 1;
  for (auto f : fps) {
    ok &= f(field_strings + field_offsets[i++]);
  }

  if (!ok) {
    return false;
  }

  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  datetime_.set(mktime(&time));
  base_line_length_.set(base_line_length);
  heading_status_.set(static_cast<QuectelRTKHeadingStatus>(heading_status));
  if (heading_status > 0) {
    attitude_.set(attitude_degree);
    attitude_accuracy_.set(attitude_accuracy_degree);
    hdg_num_satellites_.set(hdg_num_satellites);
  }

  return true;
}

}  // namespace sensesp::nmea0183
