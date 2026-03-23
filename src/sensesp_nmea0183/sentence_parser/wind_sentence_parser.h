#ifndef SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H_
#define SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp/system/observablevalue.h"
#include "sentence_parser.h"

namespace sensesp::nmea0183 {

/// Parser for WIMWV (Wind Speed and Angle) sentences
class WIMWVSentenceParser : public SentenceParser {
 public:
  WIMWVSentenceParser(NMEA0183Parser* nmea)
      : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "WIMWV"; }

  ObservableValue<float> apparent_wind_speed_;
  ObservableValue<float> apparent_wind_angle_;
};

/// Parser for MWD (Wind Direction and Speed, True) sentences
class MWDSentenceParser : public SentenceParser {
 public:
  MWDSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..MWD"; }

  ObservableValue<float> true_wind_direction_;  // radians
  ObservableValue<float> true_wind_speed_;      // m/s
};

/// Parser for VWR (Relative Wind Speed and Angle, deprecated) sentences
class VWRSentenceParser : public SentenceParser {
 public:
  VWRSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..VWR"; }

  ObservableValue<float> apparent_wind_angle_;  // radians (signed: port < 0)
  ObservableValue<float> apparent_wind_speed_;  // m/s
};

}  // namespace sensesp::nmea0183

#endif  // SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H
