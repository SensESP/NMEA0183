#ifndef SENSESP_NMEA0183_NAVIGATION_SENTENCE_PARSER_H_
#define SENSESP_NMEA0183_NAVIGATION_SENTENCE_PARSER_H_

#include "field_parsers.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sentence_parser.h"

namespace sensesp::nmea0183 {

/// Parser for HDG - Heading, Deviation & Variation
class HDGSentenceParser : public SentenceParser {
 public:
  HDGSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..HDG"; }

  ObservableValue<float> magnetic_heading_;  // radians
  ObservableValue<float> deviation_;         // radians
  ObservableValue<float> variation_;         // radians
};

/// Parser for VHW - Water Speed and Heading
class VHWSentenceParser : public SentenceParser {
 public:
  VHWSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..VHW"; }

  ObservableValue<float> true_heading_;      // radians
  ObservableValue<float> magnetic_heading_;  // radians
  ObservableValue<float> water_speed_;       // m/s
};

/// Parser for DPT - Depth of Water
class DPTSentenceParser : public SentenceParser {
 public:
  DPTSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() { return "..DPT"; }

  ObservableValue<float> depth_;   // meters (below transducer)
  ObservableValue<float> offset_;  // meters (transducer offset)
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_NAVIGATION_SENTENCE_PARSER_H_
