
#include "gnss_sentence_parser.h"

#include "Arduino.h"

namespace sensesp {

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

static bool ParsePSTI030Mode(GNSSQuality* quality, char* s) {
  switch (*s) {
    case 'N':
      *quality = GNSSQuality::no_gps;
      break;
    case 'A':
      *quality = GNSSQuality::gnss_fix;
      break;
    case 'D':
      *quality = GNSSQuality::dgnss_fix;
      break;
    case 'E':
      *quality = GNSSQuality::estimated_mode;
      break;
    case 'M':
      *quality = GNSSQuality::manual_input;
      break;
    case 'S':
      *quality = GNSSQuality::simulator_mode;
      break;
    case 'F':
      *quality = GNSSQuality::rtk_float;
      break;
    case 'R':
      *quality = GNSSQuality::rtk_fixed_integer;
      break;
    default:
      return false;
  }
  return true;
}

bool GGASentenceParser::parse_fields(char* field_strings, int field_offsets[],
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
    ReportFailure(false, sentence_address());
    return false;
  }

  // 1    = UTC of Position
  ok &= ParseTime(&hour, &minute, &second, field_strings + field_offsets[1],
                  true);
  // 2    = Latitude
  ok &= ParseLatLon(&position.latitude, field_strings + field_offsets[2], true);
  // 3    = N or S
  ok &= ParseNS(&position.latitude, field_strings + field_offsets[3], true);
  // 4    = Longitude
  ok &=
      ParseLatLon(&position.longitude, field_strings + field_offsets[4], true);
  // 5    = E or W
  ok &= ParseEW(&position.longitude, field_strings + field_offsets[5], true);
  // 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
  ok &= ParseInt(&quality, field_strings + field_offsets[6]);
  // 7    = Number of satellites in use [not those in view]
  ok &= ParseInt(&num_satellites, field_strings + field_offsets[7]);
  // 8    = Horizontal dilution of position
  ok &= ParseFloat(&horizontal_dilution, field_strings + field_offsets[8]);
  // 9    = Antenna altitude above/below mean sea level (geoid)
  ok &= ParseFloat(&position.altitude, field_strings + field_offsets[9], true);
  // 10   = Meters  (Antenna height unit)
  ok &= ParseChar(field_strings + field_offsets[10], &antenna_height_unit, 'M',
                  true);
  // 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
  //        mean sea level.  -=geoid is below WGS-84 ellipsoid)
  ok &=
      ParseFloat(&geoidal_separation, field_strings + field_offsets[11], true);
  // 12   = Meters  (Units of geoidal separation)
  ok &= ParseChar(field_strings + field_offsets[12], &geoidal_separation_unit,
                  'M', true);
  // 13   = Age in seconds since last update from diff. reference station
  ok &= ParseFloat(&dgps_age, field_strings + field_offsets[13], true);
  // 14   = Diff. reference station ID#
  ok &= ParseInt(&dgps_id, field_strings + field_offsets[14], true);

  // 15   = Checksum
  // (validated already earlier)

  ReportFailure(ok, sentence_address());
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

bool GLLSentenceParser::parse_fields(char* field_strings, int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  Position position;

  // eg.  $GPGLL,5133.81   ,N,00042.25   ,W              *75
  // eg2. $GNGLL,4916.45   ,N,12311.12   ,W,225444   ,A
  // eg3. $GNGLL,6011.07479,N,02503.05652,E,133453.00,A,D*7A
  // eg4. $GNGLL,          , ,           , ,121223.00,V,N*55

  if (num_fields < 5) {
    ReportFailure(false, sentence_address());
    return false;
  }

  //       1    5133.81   Current latitude
  ok &= ParseLatLon(&position.latitude, field_strings + field_offsets[1], true);
  //       2    N         North/South
  ok &= ParseNS(&position.latitude, field_strings + field_offsets[2], true);
  //       3    00042.25  Current longitude
  ok &=
      ParseLatLon(&position.longitude, field_strings + field_offsets[3], true);
  //       4    W         East/West
  ok &= ParseEW(&position.longitude, field_strings + field_offsets[4], true);

  // ignore the UTC time of the fix and the status of the fix for now

  ReportFailure(ok, sentence_address());
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

bool RMCSentenceParser::parse_fields(char* field_strings, int field_offsets[],
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
    ReportFailure(false, sentence_address());
    return false;
  }

  // 1   220516     Time Stamp
  ok &= ParseTime(&time.tm_hour, &time.tm_min, &second,
                  field_strings + field_offsets[1], true);
  // 2   A          validity - A-ok, V-invalid
  ok &= ParseAV(&is_valid, field_strings + field_offsets[2]);
  // 3   5133.82    current Latitude
  ok &= ParseLatLon(&position.latitude, field_strings + field_offsets[3], true);
  // 4   N          North/South
  ok &= ParseNS(&position.latitude, field_strings + field_offsets[4], true);
  // 5   00042.24   current Longitude
  ok &=
      ParseLatLon(&position.longitude, field_strings + field_offsets[5], true);
  // 6   W          East/West
  ok &= ParseEW(&position.longitude, field_strings + field_offsets[6], true);
  // 7   173.8      Speed in knots
  ok &= ParseFloat(&speed, field_strings + field_offsets[7], true);
  // 8   231.8      True course
  ok &= ParseFloat(&true_course, field_strings + field_offsets[8], true);
  // 9   130694     Date Stamp
  ok &= ParseDate(&time.tm_year, &time.tm_mon, &time.tm_mday,
                  field_strings + field_offsets[9], true);
  // 10  004.2      Variation
  ok &= ParseFloat(&variation, field_strings + field_offsets[10], true);
  // 11  W          East/West
  ok &= ParseEW(&variation, field_strings + field_offsets[11], true);

  // Positioning system mode indicator might be available as field 12, but
  // let's ignore it for now.

  ReportFailure(ok, sentence_address());
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

bool VTGSentenceParser::parse_fields(char* field_strings, int field_offsets[],
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
    ReportFailure(false, sentence_address());
    return false;
  }

  // 1             True track made good
  ok &= ParseFloat(&true_track, field_strings + field_offsets[1], true);
  // 2 T
  ok &= ParseChar(&true_track_symbol, field_strings + field_offsets[2], 'T',
                  true);
  // 3             Magnetic track made good
  ok &= ParseFloat(&magnetic_track, field_strings + field_offsets[3], true);
  // 4 M
  ok &= ParseChar(&magnetic_track_symbol, field_strings + field_offsets[4], 'M',
                  true);
  // 5             Ground speed, knots
  ok &= ParseFloat(&ground_speed, field_strings + field_offsets[5], true);
  // 6 N
  ok &= ParseChar(&ground_speed_knots_unit, field_strings + field_offsets[6],
                  'N', true);

  // ignore the remaining fields for now

  ReportFailure(ok, sentence_address());
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

bool PSTI030SentenceParser::parse_fields(char* field_strings,
                                         int field_offsets[], int num_fields) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid = false;
  Position position;
  ENUVector velocity;
  GNSSQuality quality;
  float rtk_age;
  float rtk_ratio;

  // Example:
  // $PSTI,030,044606.000,A,2447.0924110,N,12100.5227860,E,103.323,0.00,0.00,0.00,180915,R,1.2,4.2*02

  // note: field offsets are one larger than in the reference because
  // the subsentence number is at offset 1

  if (num_fields < 15) {
    ReportFailure(false, sentence_address());
    return false;
  }

  // Field  Name  Example  Description
  // 1  UTC time  044606.000  UTC time in hhmmss.sss format (000000.00 ~
  // 235959.999)
  ok &= ParseTime(&time.tm_hour, &time.tm_min, &second,
                  field_strings + field_offsets[2]);
  // 2  Status  A  Status
  // ‘V’ = Navigation receiver warning
  // ‘A’ = Data Valid
  ok &= ParseAV(&is_valid, field_strings + field_offsets[3]);
  // 3  Latitude  2447.0924110  Latitude in dddmm.mmmmmmm format
  // Leading zeros transmitted
  ok &= ParseLatLon(&position.latitude, field_strings + field_offsets[4]);
  // 4  N/S indicator  N  Latitude hemisphere indicator
  // ‘N’ = North
  // ‘S’ = South
  ok &= ParseNS(&position.latitude, field_strings + field_offsets[5]);
  // 5  Longitude  12100.5227860 Longitude in dddmm.mmmmmmm format
  // Leading zeros transmitted
  ok &= ParseLatLon(&position.longitude, field_strings + field_offsets[6]);
  // 6  E/W Indicator  E  Longitude hemisphere indicator
  // 'E' = East
  // 'W' = West
  ok &= ParseEW(&position.longitude, field_strings + field_offsets[7]);
  // 7  Altitude  103.323  mean sea level (geoid), (‐9999.999 ~ 17999.999)
  ok &= ParseFloat(&position.altitude, field_strings + field_offsets[8]);
  // 8  East Velocity  0.00  ‘East’ component of ENU velocity (m/s)
  ok &= ParseFloat(&velocity.east, field_strings + field_offsets[9]);
  // 9  North Velocity  0.00  ‘North’ component of ENU velocity (m/s)
  ok &= ParseFloat(&velocity.north, field_strings + field_offsets[10]);
  // 10  Up Velocity  0.00  ‘Up’ component of ENU velocity (m/s)
  ok &= ParseFloat(&velocity.up, field_strings + field_offsets[11]);
  // 11  UTC Date  180915  UTC date of position fix, ddmmyy format
  ok &= ParseDate(&time.tm_year, &time.tm_mon, &time.tm_mday,
                  field_strings + field_offsets[12]);
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
  ok &= ParsePSTI030Mode(&quality, field_strings + field_offsets[13]);
  // 13  RTK Age  1.2  Age of differential
  ok &= ParseFloat(&rtk_age, field_strings + field_offsets[14]);
  // 14  RTK Ratio  4.2  AR ratio factor for validation
  ok &= ParseFloat(&rtk_ratio, field_strings + field_offsets[15]);

  ReportFailure(ok, sentence_address());
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

bool PSTI032SentenceParser::parse_fields(char* field_strings,
                                         int field_offsets[], int num_fields) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid = false;
  ENUVector projection;
  GNSSQuality quality;
  float baseline_length;
  float baseline_course;

  // Example:
  // $PSTI,032,041457.000,170316,A,R,0.603,‐0.837,‐0.089,1.036,144.22,,,,,*30

  // note: field offsets are one larger than in the reference because
  // the subsentence number is at offset 1

  if (num_fields < 10) {
    ReportFailure(false, sentence_address());
    return false;
  }

  // Field  Name  Example  Description
  // 1  UTC time  041457.000  UTC time in hhmmss.sss format
  // (000000.000~235959.999)
  ok &= ParseTime(&time.tm_hour, &time.tm_min, &second,
                  field_strings + field_offsets[2]);
  // 2  UTC Date  170316  UTC date of position fix, ddmmyy format
  ok &= ParseDate(&time.tm_year, &time.tm_mon, &time.tm_mday,
                  field_strings + field_offsets[3]);
  // 3  Status  A
  // Status
  // ‘V’ = Void
  // ‘A’ = Active
  ok &= ParseAV(&is_valid, field_strings + field_offsets[4]);
  if (is_valid) {
    // 4  Mode indicator  R
    // Mode indicator
    // ‘F’ = Float RTK. System used in RTK mode with float ambiguity
    // ‘R’ = Real Time Kinematic. System used in RTK mode with fixed
    // ambiguity
    ok &= ParsePSTI030Mode(&quality, field_strings + field_offsets[5]);
    // 5  East‐projection of
    // baseline  0.603  East‐projection of baseline, meters
    ok &= ParseFloat(&projection.east, field_strings + field_offsets[6]);
    // 6  North‐projection of
    // baseline  ‐0.837  North‐projection of baseline, meters
    ok &= ParseFloat(&projection.north, field_strings + field_offsets[7]);
    // 7  Up‐projection of
    // baseline  ‐0.089  Up‐projection of baseline, meters
    ok &= ParseFloat(&projection.up, field_strings + field_offsets[8]);
    // 8  Baseline length  1.036  Baseline length, meters
    ok &= ParseFloat(&baseline_length, field_strings + field_offsets[9]);
    // 9  Baseline course  144.22
    // Baseline course (angle between baseline vector and north
    // direction), degrees
    ok &= ParseFloat(&baseline_course, field_strings + field_offsets[10]);
    // 10  Reserve    Reserve
    // 11  Reserve    Reserve
    // 12  Reserve    Reserve
    // 13  Reserve    Reserve
    // 14  Reserve    Reserve
  }

  ReportFailure(ok, sentence_address());
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

}  // namespace sensesp
