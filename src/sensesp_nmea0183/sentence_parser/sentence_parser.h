#ifndef SENSESP_NMEA0183_SENTENCE_PARSER_H_
#define SENSESP_NMEA0183_SENTENCE_PARSER_H_

#include <map>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp/signalk/signalk_position.h"

namespace sensesp {

class NMEA0183;

/**
 * @brief NMEA 0183 sentence parser base class.
 *
 * This class is responsible for parsing NMEA 0183 sentences.
 *
 * Note: For the sake of convenience, all built-in SentenceParser children
 * take a reference to a NMEALocationData object. If you want to
 * provide your own SentenceParser children, you need to maintain their
 * target `ValueConsumer` objects separately and feed those objects (or a
 * container class) to your parsers manually.
 *
 */
class SentenceParser {
 public:
  SentenceParser(NMEA0183* nmea);
  void ignore_checksum(bool ignore) { ignore_checksum_ = ignore; }

  virtual const char* sentence_address() = 0;
  bool parse(char* buffer);

 protected:
  /**
   * @brief Parse the fields of a known sentence.
   *
   * @param field_strings A character buffer containing the fields of the
   * sentence, delimited by 0s.
   * @param field_offsets An array of offsets to the beginning of each field in
   * field_strings.
   * @param num_fields The number of fields in the sentence.
   */
  virtual bool parse_fields(char* field_strings, int field_offsets[],
                            int num_fields) = 0;
  bool validate_checksum(char* buffer);

 private:
  bool ignore_checksum_;
};

}  // namespace sensesp

#endif  // SENSESP_NMEA0183_SENTENCE_PARSER_H_
