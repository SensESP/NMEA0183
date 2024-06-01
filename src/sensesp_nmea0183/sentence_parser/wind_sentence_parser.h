#ifndef SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H_
#define SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp/system/observablevalue.h"
#include "sentence_parser.h"

namespace sensesp {

/// Parser for WIMWV (Wind Speed and Angle) sentences
class WIMWVSentenceParser : public SentenceParser {
 public:
  WIMWVSentenceParser(NMEA0183Input* nmea_io,
                      ObservableValue<float>* apparent_wind_speed,
                      ObservableValue<float>* apparent_wind_angle)
      : SentenceParser(nmea_io),
        apparent_wind_speed_{apparent_wind_speed},
        apparent_wind_angle_{apparent_wind_angle} {}
  bool parse_fields(char* field_strings, int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "WIMWV"; }

 private:
  ObservableValue<float>* apparent_wind_speed_;
  ObservableValue<float>* apparent_wind_angle_;
};

}  // namespace sensesp

#endif  // SENSEP_NMEA0183_WIND_SENTENCE_PARSER_H
