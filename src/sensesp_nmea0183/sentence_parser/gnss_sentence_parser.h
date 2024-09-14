#ifndef _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_
#define _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace sensesp::nmea0183 {

enum SkyTraQGNSSQuality {
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

enum QuectelRTKHeadingStatus {
  invalid,
  rtk = 4,
  dead_reckoning = 6,
};

extern String gnss_quality_strings[];

/// Parser for GGA - Global Positioning System Fix Data.
class GGASentenceParser : public SentenceParser {
 public:
  GGASentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "G.GGA"; }

  ObservableValue<Position> position_;
  ObservableValue<String> gnss_quality_;
  ObservableValue<int> num_satellites_;
  ObservableValue<float> horizontal_dilution_;
  ObservableValue<float> geoidal_separation_;
  ObservableValue<float> dgps_age_;
  ObservableValue<int> dgps_id_;
};

/// Parser for GLL - Geographic position, latitude / longitude
class GLLSentenceParser : public SentenceParser {
 public:
  GLLSentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "G.GLL"; }

  ObservableValue<Position> position_;
};

/// Parser for RMC - Recommended minimum specific GPS/Transit data
class RMCSentenceParser : public SentenceParser {
 public:
  RMCSentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "G.RMC"; }

  ObservableValue<Position> position_;
  ObservableValue<time_t> datetime_;
  ObservableValue<float> speed_;
  ObservableValue<float> true_course_;
  ObservableValue<float> variation_;
};

/// Parser for VTG - Track made good and ground speed
class VTGSentenceParser : public SentenceParser {
 public:
  VTGSentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..VTG"; }

  ObservableValue<float> true_course_;
  ObservableValue<float> speed_;
};

/// Parser for SkyTraQ proprietary STI,030 - Recommended Minimum 3D GNSS Data
class PSTI030SentenceParser : public SentenceParser {
 public:
  PSTI030SentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "PSTI,030"; }

  ObservableValue<Position> position_;
  ObservableValue<time_t> datetime_;
  ObservableValue<ENUVector> enu_velocity_;
  ObservableValue<String> gnss_quality_;
  ObservableValue<float> rtk_age_;
  ObservableValue<float> rtk_ratio_;
};

/// Parser for SkyTraQ proprietary STI,032 - RTK Baseline Data
class PSTI032SentenceParser : public SentenceParser {
 public:
  PSTI032SentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "PSTI,032"; }

  ObservableValue<time_t> datetime_;
  ObservableValue<ENUVector> baseline_projection_;
  ObservableValue<float> baseline_length_;
  ObservableValue<float> baseline_course_;
  ObservableValue<String> gnss_quality_;
};

struct AttitudeVector {
  float yaw;  // heading
  float pitch;
  float roll;
};

/// Parser for Quectel proprietary PQTMTAR - Time and Attitude
class PQTMTARSentenceParser : public SentenceParser {
 public:
  PQTMTARSentenceParser(NMEA0183* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "PQTMTAR"; }

  ObservableValue<time_t> datetime_;
  ObservableValue<QuectelRTKHeadingStatus> heading_status_;
  ObservableValue<float> base_line_length_;
  ObservableValue<AttitudeVector> attitude_;
  ObservableValue<AttitudeVector> attitude_accuracy_;
  ObservableValue<int> hdg_num_satellites_;
};

}  // namespace sensesp::nmea0183

#endif  // _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_
