#ifndef SENSESP_NMEA0183_WEATHER_SENTENCE_PARSER_H_
#define SENSESP_NMEA0183_WEATHER_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sentence_parser.h"

namespace sensesp::nmea0183 {

/// Parser for MDA - Meteorological Composite
class MDASentenceParser : public SentenceParser {
 public:
  MDASentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..MDA"; }

  ObservableValue<float> barometric_pressure_;  // Pascals
  ObservableValue<float> air_temperature_;      // Kelvin
  ObservableValue<float> water_temperature_;    // Kelvin
  ObservableValue<float> relative_humidity_;    // ratio (0-1)
  ObservableValue<float> dew_point_;            // Kelvin
  ObservableValue<float> true_wind_direction_;  // radians
  ObservableValue<float> true_wind_speed_;      // m/s
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_WEATHER_SENTENCE_PARSER_H_
