#ifndef SENSESP_NMEA0183_SENTENCE_PARSER_H_
#define SENSESP_NMEA0183_SENTENCE_PARSER_H_

#include <map>

#include "sensesp/signalk/signalk_position.h"
#include "sensesp_nmea0183/nmea0183.h"

namespace sensesp::nmea0183 {

class NMEA0183;

/**
 * @brief NMEA 0183 sentence parser base class.
 *
 * This class is responsible for parsing NMEA 0183 sentences. When a sentence
 * is successfully received, a boolean true value is emitted.
 *
 */
class SentenceParser : public ValueProducer<bool> {
 public:
  SentenceParser(NMEA0183* nmea);
  void ignore_checksum(bool ignore) { ignore_checksum_ = ignore; }

  virtual const char* sentence_address() = 0;
  bool parse(const char* buffer);

  int get_rx_count() const { return rx_count_; }

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
  virtual bool parse_fields(const char* field_strings,
                            const int field_offsets[], int num_fields) = 0;
  bool validate_checksum(const char* buffer);

 private:
  bool ignore_checksum_;
  int rx_count_ = 0;  // Number of sentences successfully received
};

}  // namespace sensesp

#endif  // SENSESP_NMEA0183_SENTENCE_PARSER_H_
