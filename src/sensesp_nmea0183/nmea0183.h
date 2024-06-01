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

/**
 * @brief NMEA 0183 parser class.
 *
 * Data from an input stream is fed to the parse one character at a time.
 * The parser reads the input stream until a newline is encountered.
 *
 * At initialization, the parser registers a set of sentence parsers.
 * When input data matches a sentence, the parser calls the appropriate
 * sentence parser.
 *
 * @param rx_stream Pointer to the Stream of incoming GPS data over
 * a serial connection.
 **/
class NMEA0183 {
 public:
  NMEA0183(Stream* rx_stream);

  void register_sentence_parser(SentenceParser* parser);
  void handle(char c);

 protected:
  Stream* rx_stream_;
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
