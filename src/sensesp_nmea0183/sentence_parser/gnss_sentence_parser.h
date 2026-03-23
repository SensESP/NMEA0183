#ifndef _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_
#define _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"
#include "sensesp_nmea0183/data/gnss_data.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace sensesp::nmea0183 {

enum SkyTraqGNSSQuality {
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
  GGASentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.GGA"; }

  ObservableValue<Position> position_;
  ObservableValue<String> gnss_quality_;
  ObservableValue<int> quality_;  // Raw GGA quality indicator (0-8)
  ObservableValue<int> num_satellites_;
  ObservableValue<float> horizontal_dilution_;
  ObservableValue<float> geoidal_separation_;
  ObservableValue<float> dgps_age_;
  ObservableValue<int> dgps_id_;
};

/// Parser for GLL - Geographic position, latitude / longitude
class GLLSentenceParser : public SentenceParser {
 public:
  GLLSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.GLL"; }

  ObservableValue<Position> position_;
};

/// Parser for RMC - Recommended minimum specific GPS/Transit data
class RMCSentenceParser : public SentenceParser {
 public:
  RMCSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.RMC"; }

  ObservableValue<Position> position_;
  ObservableValue<time_t> datetime_;
  ObservableValue<float> speed_;
  ObservableValue<float> true_course_;
  ObservableValue<float> variation_;
};

/// Parser for VTG - Track made good and ground speed
class VTGSentenceParser : public SentenceParser {
 public:
  VTGSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "..VTG"; }

  ObservableValue<float> true_course_;
  ObservableValue<float> speed_;
};

/// Parser for GSV - GNSS Satellites in View
class GSVSentenceParser : public SentenceParser {
 public:
  GSVSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.GSV"; }

  /// Number of satellites with data blocks received in the GSV cycle
  ObservableValue<int> num_satellites_;
  /// Total number of SVs in view (from GSV header field 3)
  ObservableValue<int> total_svs_in_view_;
  ObservableValue<std::vector<GNSSSatellite>> satellites_;
  ObservableValue<GNSSSatellite> first_satellite_;
};

/// Parser for SkyTraq proprietary STI,030 - Recommended Minimum 3D GNSS Data
class SkyTraqPSTI030SentenceParser : public SentenceParser {
 public:
  SkyTraqPSTI030SentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "PSTI,030"; }

  ObservableValue<Position> position_;
  ObservableValue<time_t> datetime_;
  ObservableValue<ENUVector> enu_velocity_;
  ObservableValue<String> gnss_quality_;
  ObservableValue<float> rtk_age_;
  ObservableValue<float> rtk_ratio_;
};

/// Parser for SkyTraq proprietary STI,032 - RTK Baseline Data
class SkyTraqPSTI032SentenceParser : public SentenceParser {
 public:
  SkyTraqPSTI032SentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "PSTI,032"; }

  ObservableValue<time_t> datetime_;
  ObservableValue<ENUVector> baseline_projection_;
  ObservableValue<float> baseline_length_;
  ObservableValue<float> baseline_course_;
  ObservableValue<String> gnss_quality_;
};

/// Parser for Quectel proprietary PQTMTAR - Time and Attitude
class QuectelPQTMTARSentenceParser : public SentenceParser {
 public:
  QuectelPQTMTARSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "PQTMTAR"; }

  ObservableValue<time_t> datetime_;
  ObservableValue<String> rtk_quality_;
  ObservableValue<float> baseline_length_;
  ObservableValue<AttitudeVector> attitude_;
  ObservableValue<AttitudeVector> attitude_accuracy_;
  ObservableValue<int> hdg_num_satellites_;
};

/// Parser for GSA - GPS DOP and Active Satellites
class GSASentenceParser : public SentenceParser {
 public:
  GSASentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.GSA"; }

  ObservableValue<int> fix_type_;     // 1=no fix, 2=2D, 3=3D
  ObservableValue<float> pdop_;
  ObservableValue<float> hdop_;
  ObservableValue<float> vdop_;
};

/// Parser for ZDA - Time & Date
class ZDASentenceParser : public SentenceParser {
 public:
  ZDASentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.ZDA"; }

  ObservableValue<time_t> datetime_;
};

/// Parser for GBS - GNSS Satellite Fault Detection
class GBSSentenceParser : public SentenceParser {
 public:
  GBSSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "G.GBS"; }

  ObservableValue<float> lat_error_;  // meters
  ObservableValue<float> lon_error_;  // meters
  ObservableValue<float> alt_error_;  // meters
};

}  // namespace sensesp::nmea0183

#endif  // _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_
