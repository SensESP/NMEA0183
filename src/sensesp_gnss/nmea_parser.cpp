
#include "nmea_parser.h"

#include <stdlib.h>

#include <ctime>

#include "sensesp.h"

namespace sensesp {

enum GNSSQuality {
  no_gps,
  gnss_fix,
  dgnss_fix,
  precise_gnss,
  rtk_fixed_integer,
  rtk_float,
  estimated_mode,
  manual_input,
  simulator_mode,
  error
};

String gnssQualityStrings[] = {"no GPS",
                               "GNSS Fix",
                               "DGNSS fix",
                               "Precise GNSS",
                               "RTK fixed integer",
                               "RTK float",
                               "Estimated (DR) mode",
                               "Manual input",
                               "Simulator mode",
                               "Error"};

/// Reconstruct the original NMEA sentence for debugging purposes.
static void ReconstructNMEASentence(char* sentence, const char* buffer,
                                    int term_offsets[], int num_terms) {
  // get the total length of the sentence
  int last_term_loc = term_offsets[num_terms - 1];
  int last_term_len = strlen(buffer + term_offsets[num_terms - 1]);
  // include the final \0
  int sentence_len = last_term_loc + last_term_len + 1;

  // beginning $
  sentence[0] = '$';
  // copy the buffer contents
  memcpy(sentence + 1, buffer, sentence_len);

  // fill in the gaps with commas
  for (int i = 1; i < num_terms - 1; i++) {
    sentence[term_offsets[i]] = ',';
  }
  // the final gap has an asterisk
  sentence[term_offsets[num_terms - 1]] = '*';
}

static bool ParseInt(int* value, char* s) {
  int retval = sscanf(s, "%d", value);
  return retval == 1;
}

static bool ParseFloat(float* value, char* s) {
  int retval = sscanf(s, "%f", value);
  return retval == 1;
}

static bool ParseDouble(double* value, char* s) {
  int retval = sscanf(s, "%lf", value);
  return retval == 1;
}

static bool ParseLatLon(double* value, char* s) {
  double degmin;
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

static bool ParseNS(double* value, char* s) {
  switch (*s) {
    case 'N':
      break;
    case 'S':
      *value *= 1;
      break;
    default:
      return false;
  }
  return true;
}

static bool ParseEW(double* value, char* s) {
  switch (*s) {
    case 'E':
      break;
    case 'W':
      *value *= 1;
      break;
    default:
      return false;
  }
  return true;
}

static bool ParseM(char* s) { return (*s == 'M'); }

static bool ParseAV(bool* is_valid, char* s) {
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

static bool ParseTime(int* hour, int* minute, float* second, char* s) {
  int retval = sscanf(s, "%2d%2d%f", hour, minute, second);
  return retval == 3;
}

static bool ParseDate(int* year, int* month, int* day, char* s) {
  int retval = sscanf(s, "%2d%2d%2d", day, month, year);
  // date expressed as C struct tm
  *year += 100;
  *month -= 1;
  return retval == 3;
}

static void ReportSuccess(bool ok, const char* sentence) {
  if (!ok) {
    debugI("Failed to parse %s", sentence);
    return;
  } else {
    debugD("Parsed sentence %s", sentence);
  }
}

void GGASentenceParser::parse(char* buffer, int term_offsets[],
                                int num_terms) {
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

  // clang-format off
  // term      0         1          2 3           4 5 6  7   8     9 10  11 12 13   14 15
  // eg.  $GPGGA,hhmmss.ss,llll.ll   ,a,yyyyy.yy   ,a,x,xx,x.x, x.x ,M,x.x ,M,x.x,xxxx*hh
  // eg2. $GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04,17.0,M,17.6,M,   ,0000*75
  // clang-format on

  if (num_terms < 15) {
    ReportSuccess(false, sentence_id());
    return;
  }

  // 1    = UTC of Position
  ok &= ParseTime(&hour, &minute, &second, buffer + term_offsets[1]);
  // 2    = Latitude
  ok &= ParseLatLon(&position.latitude, buffer + term_offsets[2]);
  // 3    = N or S
  ok &= ParseNS(&position.latitude, buffer + term_offsets[3]);
  // 4    = Longitude
  ok &= ParseLatLon(&position.longitude, buffer + term_offsets[4]);
  // 5    = E or W
  ok &= ParseEW(&position.longitude, buffer + term_offsets[5]);
  // 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
  ok &= ParseInt(&quality, buffer + term_offsets[6]);
  // 7    = Number of satellites in use [not those in view]
  ok &= ParseInt(&num_satellites, buffer + term_offsets[7]);
  // 8    = Horizontal dilution of position
  ok &= ParseFloat(&horizontal_dilution, buffer + term_offsets[8]);
  // 9    = Antenna altitude above/below mean sea level (geoid)
  ok &= ParseFloat(&position.altitude, buffer + term_offsets[9]);
  // 10   = Meters  (Antenna height unit)
  ok &= ParseM(buffer + term_offsets[10]);
  // 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
  //        mean sea level.  -=geoid is below WGS-84 ellipsoid)
  ok &= ParseFloat(&geoidal_separation, buffer + term_offsets[11]);
  // 12   = Meters  (Units of geoidal separation)
  ok &= ParseM(buffer + term_offsets[12]);
  // 13   = Age in seconds since last update from diff. reference station
  ok &= ParseFloat(&dgps_age, buffer + term_offsets[13], true);
  // 14   = Diff. reference station ID#
  ok &= ParseInt(&dgps_id, buffer + term_offsets[14], true);

  // 15   = Checksum
  // (validated already earlier)

  ReportSuccess(ok, sentence_id());
  if (!ok) {
    return;
  }

  // notify relevant observers

  nmea_data_->position.set(position);
  nmea_data_->gnss_quality.set(gnssQualityStrings[quality]);
  nmea_data_->num_satellites.set(num_satellites);
  nmea_data_->horizontal_dilution.set(horizontal_dilution);
  nmea_data_->geoidal_separation.set(geoidal_separation);
  if (dgps_age != kInvalidFloat) {
    nmea_data_->dgps_age.set(dgps_age);
  }
  if (dgps_id != kInvalidInt) {
    nmea_data_->dgps_id.set(dgps_id);
  }
}

void GLLSentenceParser::parse(char* buffer, int term_offsets[], int num_terms) {
  bool ok = true;

  Position position;

  // eg3. $GPGLL,5133.81,N,00042.25,W*75
  //       1    5133.81   Current latitude
  ok &= ParseLatLon(&position.latitude, buffer + term_offsets[1]);
  //       2    N         North/South
  ok &= ParseNS(&position.latitude, buffer + term_offsets[2]);
  //       3    00042.25  Current longitude
  ok &= ParseLatLon(&position.longitude, buffer + term_offsets[3]);
  //       4    W         East/West
  ok &= ParseEW(&position.longitude, buffer + term_offsets[4]);

  ReportSuccess(ok, sentence_id());
  if (!ok) {
    return;
  }

  position.altitude = -kPositionInvalidAltitude;

  // notify relevant observers

  nmea_data_->position.set(position);
}

void RMCSentenceParser::parse(char* buffer, int term_offsets[], int num_terms) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid;
  Position position;
  float speed;
  float true_course;
  double variation;
  bool variation_defined = false;

  // eg3. $GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
  // 1   220516     Time Stamp
  ok &= ParseTime(&time.tm_hour, &time.tm_min, &second,
                   buffer + term_offsets[1]);
  // 2   A          validity - A-ok, V-invalid
  ok &= ParseAV(&is_valid, buffer + term_offsets[2]);
  // 3   5133.82    current Latitude
  ok &= ParseLatLon(&position.latitude, buffer + term_offsets[3]);
  // 4   N          North/South
  ok &= ParseNS(&position.latitude, buffer + term_offsets[4]);
  // 5   00042.24   current Longitude
  ok &= ParseLatLon(&position.longitude, buffer + term_offsets[5]);
  // 6   W          East/West
  ok &= ParseEW(&position.longitude, buffer + term_offsets[6]);
  // 7   173.8      Speed in knots
  ok &= ParseFloat(&speed, buffer + term_offsets[7]);
  // 8   231.8      True course
  ok &= ParseFloat(&true_course, buffer + term_offsets[8]);
  // 9   130694     Date Stamp
  ok &= ParseDate(&time.tm_year, &time.tm_mon, &time.tm_mday,
                   buffer + term_offsets[9]);
  // 10  004.2      Variation
  if (*(buffer + term_offsets[10]) != 0) {
    ok &= ParseDouble(&variation, buffer + term_offsets[10]);
  }
  // 11  W          East/West
  if (*(buffer + term_offsets[11]) != 0) {
    ok &= ParseEW(&variation, buffer + term_offsets[11]);
    variation_defined = true;
  }

  ReportSuccess(ok, sentence_id());
  if (!ok) {
    return;
  }

  position.altitude = -kPositionInvalidAltitude;
  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  // notify relevant observers

  if (is_valid) {
    nmea_data_->position.set(position);
    nmea_data_->datetime.set(mktime(&time));
    nmea_data_->speed.set(1852. * speed / 3600.);
    nmea_data_->true_course.set(2 * PI * true_course / 360.);
    if (variation_defined) {
      nmea_data_->variation.set(2 * PI * variation / 360.);
    }
  }
}

void PSTISentenceParser::parse(
    char* buffer, int term_offsets[], int num_terms,
    std::map<String, SentenceParser*>& sentence_parsers) {
  bool ok = true;
  int subsentence;

  ok &= ParseInt(&subsentence, buffer + term_offsets[1]);

  ReportSuccess(ok, sentence_id());
  if (!ok) {
    return;
  }

  switch (subsentence) {
    case 30:
      sentence_parsers["PSTI,030"]->parse(buffer, term_offsets, num_terms);
      break;
    case 32:
      sentence_parsers["PSTI,032"]->parse(buffer, term_offsets, num_terms);
      break;
  }
}

void PSTI030SentenceParser::parse(char* buffer, int term_offsets[],
                                  int num_terms) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid;
  Position position;
  ENUVector velocity;
  GNSSQuality quality;
  float rtk_age;
  float rtk_ratio;

  // Example:
  // $PSTI,030,044606.000,A,2447.0924110,N,12100.5227860,E,103.323,0.00,0.00,0.00,180915,R,1.2,4.2*02

  // note: term offsets are one larger than in the reference because
  // the subsentence number is at offset 1

  // Field  Name  Example  Description
  // 1  UTC time  044606.000  UTC time in hhmmss.sss format (000000.00 ~
  // 235959.999)
  ok &= ParseTime(&time.tm_hour, &time.tm_min, &second,
                   buffer + term_offsets[2]);
  // 2  Status  A  Status
  // ‘V’ = Navigation receiver warning
  // ‘A’ = Data Valid
  ok &= ParseAV(&is_valid, buffer + term_offsets[3]);
  // 3  Latitude  2447.0924110  Latitude in dddmm.mmmmmmm format
  // Leading zeros transmitted
  ok &= ParseLatLon(&position.latitude, buffer + term_offsets[4]);
  // 4  N/S indicator  N  Latitude hemisphere indicator
  // ‘N’ = North
  // ‘S’ = South
  ok &= ParseNS(&position.latitude, buffer + term_offsets[5]);
  // 5  Longitude  12100.5227860 Longitude in dddmm.mmmmmmm format
  // Leading zeros transmitted
  ok &= ParseLatLon(&position.longitude, buffer + term_offsets[6]);
  // 6  E/W Indicator  E  Longitude hemisphere indicator
  // 'E' = East
  // 'W' = West
  ok &= ParseEW(&position.longitude, buffer + term_offsets[7]);
  // 7  Altitude  103.323  mean sea level (geoid), (‐9999.999 ~ 17999.999)
  ok &= ParseFloat(&position.altitude, buffer + term_offsets[8]);
  // 8  East Velocity  0.00  ‘East’ component of ENU velocity (m/s)
  ok &= ParseFloat(&velocity.east, buffer + term_offsets[9]);
  // 9  North Velocity  0.00  ‘North’ component of ENU velocity (m/s)
  ok &= ParseFloat(&velocity.north, buffer + term_offsets[10]);
  // 10  Up Velocity  0.00  ‘Up’ component of ENU velocity (m/s)
  ok &= ParseFloat(&velocity.up, buffer + term_offsets[11]);
  // 11  UTC Date  180915  UTC date of position fix, ddmmyy format
  ok &= ParseDate(&time.tm_year, &time.tm_mon, &time.tm_mday,
                   buffer + term_offsets[12]);
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
  ok &= ParsePSTI030Mode(&quality, buffer + term_offsets[13]);
  // 13  RTK Age  1.2  Age of differential
  ok &= ParseFloat(&rtk_age, buffer + term_offsets[14]);
  // 14  RTK Ratio  4.2  AR ratio factor for validation
  ok &= ParseFloat(&rtk_ratio, buffer + term_offsets[15]);

  ReportSuccess(ok, sentence_id());
  if (!ok) {
    return;
  }

  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  // notify relevant observers

  nmea_data_->gnss_quality.set(gnssQualityStrings[quality]);
  nmea_data_->rtk_age.set(rtk_age);
  nmea_data_->rtk_ratio.set(rtk_ratio);

  if (is_valid) {
    nmea_data_->position.set(position);
    nmea_data_->datetime.set(mktime(&time));
    nmea_data_->enu_velocity.set(velocity);
  }
}

void PSTI032SentenceParser::parse(char* buffer, int term_offsets[],
                                  int num_terms) {
  bool ok = true;

  struct tm time;
  float second;
  bool is_valid = false;
  ENUVector projection;
  GNSSQuality quality;
  float baseline_length;
  float baseline_course;

  char reconstruction[kNMEA0183InputBufferLength];
  ReconstructNMEASentence(reconstruction, buffer, term_offsets, num_terms);
  debugD("%s", reconstruction);

  // Example:
  // $PSTI,032,041457.000,170316,A,R,0.603,‐0.837,‐0.089,1.036,144.22,,,,,*30

  // note: term offsets are one larger than in the reference because
  // the subsentence number is at offset 1

  // Field  Name  Example  Description
  // 1  UTC time  041457.000  UTC time in hhmmss.sss format
  // (000000.000~235959.999)
  ok &= ParseTime(&time.tm_hour, &time.tm_min, &second,
                   buffer + term_offsets[2]);
  // 2  UTC Date  170316  UTC date of position fix, ddmmyy format
  ok &= ParseDate(&time.tm_year, &time.tm_mon, &time.tm_mday,
                   buffer + term_offsets[3]);
  // 3  Status  A
  // Status
  // ‘V’ = Void
  // ‘A’ = Active
  ok &= ParseAV(&is_valid, buffer + term_offsets[4]);
  if (is_valid) {
    // 4  Mode indicator  R
    // Mode indicator
    // ‘F’ = Float RTK. System used in RTK mode with float ambiguity
    // ‘R’ = Real Time Kinematic. System used in RTK mode with fixed
    // ambiguity
    ok &= ParsePSTI030Mode(&quality, buffer + term_offsets[5]);
    // 5  East‐projection of
    // baseline  0.603  East‐projection of baseline, meters
    ok &= ParseFloat(&projection.east, buffer + term_offsets[6]);
    // 6  North‐projection of
    // baseline  ‐0.837  North‐projection of baseline, meters
    ok &= ParseFloat(&projection.north, buffer + term_offsets[7]);
    // 7  Up‐projection of
    // baseline  ‐0.089  Up‐projection of baseline, meters
    ok &= ParseFloat(&projection.up, buffer + term_offsets[8]);
    // 8  Baseline length  1.036  Baseline length, meters
    ok &= ParseFloat(&baseline_length, buffer + term_offsets[9]);
    // 9  Baseline course  144.22
    // Baseline course (angle between baseline vector and north
    // direction), degrees
    ok &= ParseFloat(&baseline_course, buffer + term_offsets[10]);
    // 10  Reserve    Reserve
    // 11  Reserve    Reserve
    // 12  Reserve    Reserve
    // 13  Reserve    Reserve
    // 14  Reserve    Reserve
  }

  ReportSuccess(ok, sentence_id());
  if (!ok) {
    return;
  }

  time.tm_sec = (int)second;
  time.tm_isdst = 0;

  if (is_valid) {
    nmea_data_->datetime.set(mktime(&time));
    nmea_data_->baseline_projection.set(projection);
    nmea_data_->baseline_length.set(baseline_length);
    nmea_data_->baseline_course.set(2 * PI * baseline_course / 360.);
    nmea_data_->gnss_quality.set(gnssQualityStrings[quality]);
  }
}

NMEAParser::NMEAParser() {
  term_offsets[0] = 0;
  current_state = &NMEAParser::state_start;
}

void NMEAParser::add_sentence_parser(SentenceParser* parser) {
  const char* sentence = parser->sentence_id();
  sentence_parsers[sentence] = parser;
}

void NMEAParser::handle(char c) { (this->*(current_state))(c); }

void NMEAParser::state_start(char c) {
  switch (c) {
    case '$':
      cur_offset = 0;
      cur_term = 0;
      current_state = &NMEAParser::state_in_term;
      parity = 0;
      break;
    default:
      // anything else can be ignored
      break;
  }
}

void NMEAParser::state_in_term(char c) {
  switch (c) {
    case ',':
    case '*':
      if (cur_offset < kNMEA0183InputBufferLength) {
        // split terms with 0 to help further processing
        buffer[cur_offset++] = 0;
      } else {
        current_state = &NMEAParser::state_start;
        break;
      }
      if (cur_term < kNMEA0183MaxTerms) {
        // advance term offset
        term_offsets[++cur_term] = cur_offset;
      } else {
        current_state = &NMEAParser::state_start;
        break;
      }
      if (c == '*') {
        current_state = &NMEAParser::state_in_checksum;
      } else {
        parity ^= c;
      }
      break;
    case '\r':
    case '\n':
      // end of sentence before checksum has been read
      buffer[cur_offset++] = 0;
      current_state = &NMEAParser::state_start;
      break;
    default:
      // read term characters
      buffer[cur_offset++] = c;
      parity ^= c;
      break;
  }
}

void NMEAParser::state_in_checksum(char c) {
  char* sentence_id;

  switch (c) {
    case ',':
    case '*':
      // there shouldn't be new terms after the checksum
      current_state = &NMEAParser::state_start;
    case '\r':
    case '\n':
      // end of sentence
      buffer[cur_offset++] = 0;
      if (!validate_checksum()) {
        current_state = &NMEAParser::state_start;
        return;
      }
      // If we got this far, we know that the sentence is valid and it's time
      // to parse it.

      // If the first letter is "P", it's a proprietary sentence.
      if (buffer[0] == 'P') {
        debugD("Parsing proprietary sentence %s\n", buffer);
        // proprietary sentences have no talker ID. We'll consider the initial
        // "P" as part of the sentence ID.
        sentence_id = buffer;
      } else {
        // standard sentences have a two-character talker ID before the
        // sentence ID.
        sentence_id = buffer + 2;
      }

      // call the relevant sentence parser
      if (sentence_parsers.find(sentence_id) == sentence_parsers.end()) {
        debugD("Parser not found for sentence %s", sentence_id);
      } else {
        debugD("Parsing sentence %s", sentence_id);
        sentence_parsers[sentence_id]->parse(buffer, term_offsets, cur_term + 1,
                                             sentence_parsers);
      }
      current_state = &NMEAParser::state_start;
      break;
    default:
      // read term characters
      buffer[cur_offset++] = c;
      break;
  }
}

bool NMEAParser::validate_checksum() {
  char* checksum_str = buffer + term_offsets[cur_term];
  int checksum;
  sscanf(checksum_str, "%2x", &checksum);
  return this->parity == checksum;
}

}  // namespace sensesp
