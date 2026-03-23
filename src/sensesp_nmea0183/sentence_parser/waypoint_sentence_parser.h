#ifndef SENSESP_NMEA0183_WAYPOINT_SENTENCE_PARSER_H_
#define SENSESP_NMEA0183_WAYPOINT_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sentence_parser.h"

namespace sensesp::nmea0183 {

/// Parser for RMB - Recommended Minimum Navigation Information
class RMBSentenceParser : public SentenceParser {
 public:
  RMBSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..RMB"; }

  ObservableValue<float> cross_track_error_;           // meters (signed)
  ObservableValue<float> bearing_to_destination_;      // radians (true)
  ObservableValue<float> range_to_destination_;        // meters
  ObservableValue<float> destination_closing_velocity_; // m/s
  ObservableValue<String> destination_waypoint_id_;
};

/// Parser for APB - Autopilot Sentence "B"
class APBSentenceParser : public SentenceParser {
 public:
  APBSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..APB"; }

  ObservableValue<float> cross_track_error_;  // meters (signed)
  ObservableValue<float> heading_to_steer_;   // radians (true)
};

/// Parser for BWC - Bearing and Distance to Waypoint (Great Circle)
class BWCSentenceParser : public SentenceParser {
 public:
  BWCSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..BWC"; }

  ObservableValue<float> bearing_true_;      // radians
  ObservableValue<float> bearing_magnetic_;  // radians
  ObservableValue<float> distance_;          // meters
  ObservableValue<String> waypoint_id_;
  ObservableValue<Position> waypoint_position_;
};

/// Parser for WPL - Waypoint Location
class WPLSentenceParser : public SentenceParser {
 public:
  WPLSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..WPL"; }

  ObservableValue<Position> position_;
  ObservableValue<String> waypoint_id_;
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_WAYPOINT_SENTENCE_PARSER_H_
