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
  WIMWVSentenceParser(NMEA0183* nmea)
      : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "WIMWV"; }

  ObservableValue<float> apparent_wind_speed_;
  ObservableValue<float> apparent_wind_angle_;
};

}  // namespace sensesp::nmea0183

#endif  // SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H
