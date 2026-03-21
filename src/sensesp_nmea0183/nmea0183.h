#ifndef SENSESP_NMEA0183_NMEA0183_H_
#define SENSESP_NMEA0183_NMEA0183_H_

#include "sensesp/sensors/sensor.h"
#include "sensesp/system/stream_producer.h"
#include "sensesp/transforms/filter.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace sensesp::nmea0183 {

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
 **/
class NMEA0183Parser : public ValueConsumer<String> {
 public:
  NMEA0183Parser() : ValueConsumer<String>() {}

  void register_sentence_parser(SentenceParser* parser);
  virtual void set(const String& line) override;

 protected:
  // offset for each sentence field in the buffer
  int field_offsets[kNMEA0183MaxFields];
  void parse_sentence(const String& sentence);
  std::vector<SentenceParser*> sentence_parsers;
};

/**
 * @brief NMEA 0183 I/O class.
 *
 * Reads NMEA 0183 sentences from a stream using the main event loop,
 * parses them, and allows writing sentences to the stream.
 */
class NMEA0183IO : public ValueConsumer<String> {
 public:
  NMEA0183IO(Stream* stream) : stream_(stream) {
    line_producer_ = std::make_shared<StreamLineProducer>(stream);

    sentence_filter_ = std::make_shared<Filter<String>>([](const String& line) {
      return line.startsWith("!") || line.startsWith("$");
    });

    line_producer_->connect_to(sentence_filter_)->connect_to(&parser_);
  }

  NMEA0183Parser parser_;
  std::shared_ptr<StreamLineProducer> line_producer_;
  std::shared_ptr<Filter<String>> sentence_filter_;

  virtual void set(const String& line) override {
    stream_->println(line);
  }

 protected:
  Stream* stream_;
};

/// @deprecated Use NMEA0183IO instead.
using NMEA0183IOTask = NMEA0183IO;

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_NMEA0183_H_
