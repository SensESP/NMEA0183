#ifndef _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_
#define _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

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

extern String gnss_quality_strings[];

/// Parser for GGA - Global Positioning System Fix Data.
class GGASentenceParser : public SentenceParser {
 public:
  GGASentenceParser(NMEA0183Input* nmea_io, ObservableValue<Position>* position,
                    ObservableValue<String>* gnss_quality,
                    ObservableValue<int>* num_satellites,
                    ObservableValue<float>* horizontal_dilution,
                    ObservableValue<float>* geoidal_separation,
                    ObservableValue<float>* dgps_age,
                    ObservableValue<int>* dgps_id)
      : SentenceParser(nmea_io),
        position_{position},
        gnss_quality_{gnss_quality},
        num_satellites_{num_satellites},
        horizontal_dilution_{horizontal_dilution},
        geoidal_separation_{geoidal_separation},
        dgps_age_{dgps_age},
        dgps_id_{dgps_id} {}
  bool parse_fields(char* buffer, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "G.GGA"; }

 private:
  ObservableValue<Position>* position_;
  ObservableValue<String>* gnss_quality_;
  ObservableValue<int>* num_satellites_;
  ObservableValue<float>* horizontal_dilution_;
  ObservableValue<float>* geoidal_separation_;
  ObservableValue<float>* dgps_age_;
  ObservableValue<int>* dgps_id_;
};

/// Parser for GLL - Geographic position, latitude / longitude
class GLLSentenceParser : public SentenceParser {
 public:
  GLLSentenceParser(NMEA0183Input* nmea_io, ObservableValue<Position>* position)
      : SentenceParser(nmea_io), position_{position} {}
  bool parse_fields(char* buffer, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "G.GLL"; }

 private:
  ObservableValue<Position>* position_;
};

/// Parser for RMC - Recommended minimum specific GPS/Transit data
class RMCSentenceParser : public SentenceParser {
 public:
  RMCSentenceParser(NMEA0183Input* nmea_io, ObservableValue<Position>* position,
                    ObservableValue<time_t>* datetime,
                    ObservableValue<float>* speed,
                    ObservableValue<float>* true_course,
                    ObservableValue<float>* variation)
      : SentenceParser(nmea_io),
        position_{position},
        datetime_{datetime},
        speed_{speed},
        true_course_{true_course},
        variation_{variation} {}
  bool parse_fields(char* buffer, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "G.RMC"; }

 private:
  ObservableValue<Position>* position_;
  ObservableValue<time_t>* datetime_;
  ObservableValue<float>* speed_;
  ObservableValue<float>* true_course_;
  ObservableValue<float>* variation_;
};

/// Parser for VTG - Track made good and ground speed
class VTGSentenceParser : public SentenceParser {
 public:
  VTGSentenceParser(NMEA0183Input* nmea_io, ObservableValue<float>* true_course,
                    ObservableValue<float>* speed)
      : SentenceParser(nmea_io),
        true_course_{true_course},
        speed_{speed} {}
  bool parse_fields(char* buffer, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..VTG"; }

 private:
  ObservableValue<float>* true_course_;
  ObservableValue<float>* speed_;
};

/// Parser for proprietary STI,030 - Recommended Minimum 3D GNSS Data
class PSTI030SentenceParser : public SentenceParser {
 public:
  PSTI030SentenceParser(NMEA0183Input* nmea_io,
                        ObservableValue<Position>* position,
                        ObservableValue<time_t>* datetime,
                        ObservableValue<ENUVector>* enu_velocity,
                        ObservableValue<String>* gnss_quality,
                        ObservableValue<float>* rtk_age,
                        ObservableValue<float>* rtk_ratio)
      : SentenceParser(nmea_io),
        position_{position},
        datetime_{datetime},
        enu_velocity_{enu_velocity},
        gnss_quality_{gnss_quality},
        rtk_age_{rtk_age},
        rtk_ratio_{rtk_ratio} {}

  bool parse_fields(char* buffer, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "PSTI,030"; }

 private:
  ObservableValue<Position>* position_;
  ObservableValue<time_t>* datetime_;
  ObservableValue<ENUVector>* enu_velocity_;
  ObservableValue<String>* gnss_quality_;
  ObservableValue<float>* rtk_age_;
  ObservableValue<float>* rtk_ratio_;
};

/// Parser for proprietary STI,032 - RTK Baseline Data
class PSTI032SentenceParser : public SentenceParser {
 public:
  PSTI032SentenceParser(NMEA0183Input* nmea_io, ObservableValue<time_t>* datetime,
                        ObservableValue<ENUVector>* baseline_projection,
                        ObservableValue<float>* baseline_length,
                        ObservableValue<float>* baseline_course,
                        ObservableValue<String>* gnss_quality)
      : SentenceParser(nmea_io),
        datetime_{datetime},
        baseline_projection_{baseline_projection},
        baseline_length_{baseline_length},
        baseline_course_{baseline_course},
        gnss_quality_{gnss_quality} {}

  bool parse_fields(char* buffer, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "PSTI,032"; }

 private:
  ObservableValue<time_t>* datetime_;
  ObservableValue<ENUVector>* baseline_projection_;
  ObservableValue<float>* baseline_length_;
  ObservableValue<float>* baseline_course_;
  ObservableValue<String>* gnss_quality_;
};

}  // namespace sensesp

#endif  // _SENSESP_NMEA0183_GNSS_SENTENCE_PARSER_H_
