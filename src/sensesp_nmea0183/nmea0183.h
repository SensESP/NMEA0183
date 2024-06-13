#ifndef SENSESP_NMEA0183_NMEA0183_H_
#define SENSESP_NMEA0183_NMEA0183_H_

#include "sensesp/sensors/sensor.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace sensesp {

void ReportFailure(bool ok, const char* sentence);

/// Maximum length of a single NMEA sentence. The standard defined
/// maximum is 82, but let's give it a bit of margin.
constexpr int kNMEA0183InputBufferLength = 164;
/// Maximum number of comma-separated fields in one NMEA sentence.
constexpr int kNMEA0183MaxFields = 25;

class SentenceParser;

int CalculateChecksum(const char* buffer, char seed = 0);
void AddChecksum(String& sentence);

/**
 * @brief NMEA 0183 parser class.
 *
 * @param stream Pointer to the Stream of incoming NMEA0183 data over
 * a serial connection.
 **/
class NMEA0183 {
 public:
  NMEA0183(Stream* stream);

  void register_sentence_parser(SentenceParser* parser);
  void handle(char c);

  /**
   * @brief Output a sentence to the serial port.
   *
   * It is assumed that the sentence does not include the CRLF newline.
   *
   * @param sentence
   */
  void output_raw(const char* sentence) const {
    stream_->println(sentence);
  }

 protected:
  Stream* stream_;
  // current sentence
  char input_buffer[kNMEA0183InputBufferLength];
  // offset for each sentence field in the buffer
  int field_offsets[kNMEA0183MaxFields];
  // pointer for the next character in buffer
  int input_offset = 0;
  void parse_sentence();
  std::vector<SentenceParser*> sentence_parsers;
};

}  // namespace sensesp

#endif  // SENSESP_NMEA0183_NMEA0183_H_
