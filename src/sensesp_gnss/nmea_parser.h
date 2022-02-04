#ifndef _nmea_parser_H_
#define _nmea_parser_H_

#include <map>

#include "sensesp/system/observablevalue.h"

#include "sensesp/types/position.h"
#include "sensesp/signalk/signalk_position.h"


namespace sensesp {

/// Maximum length of a single NMEA sentence.
constexpr int kNMEA0183InputBufferLength = 250;
/// Maximum number of comma-separated terms in one NMEA sentence.
constexpr int kNMEA0183MaxTerms = 25;

/**
 * @brief Container for all decoded NMEA 0183 location data.
 */
struct NMEALocationData {
  ObservableValue<Position> position;
  ObservableValue<String> gnss_quality;
  ObservableValue<int> num_satellites;
  ObservableValue<float> horizontal_dilution;
  ObservableValue<float> geoidal_separation;
  ObservableValue<float> dgps_age;
  ObservableValue<float> dgps_id;
  ObservableValue<time_t> datetime;
  ObservableValue<float> speed;
  ObservableValue<float> true_course;
  ObservableValue<float> variation;
  ObservableValue<ENUVector> enu_velocity;
  ObservableValue<float> rtk_age;
  ObservableValue<float> rtk_ratio;
  ObservableValue<ENUVector> baseline_projection;
  ObservableValue<float> baseline_length;
  ObservableValue<float> baseline_course;
};

/**
 * @brief NMEA 0183 sentence parser base class.
 */
class SentenceParser {
 public:
  SentenceParser(NMEALocationData* nmea_data);
  virtual void parse(char* buffer, int term_offsets[], int num_terms) = 0;
  virtual void parse(char* buffer, int term_offsets[], int num_terms,
                     std::map<String, SentenceParser*>& sentence_parsers) {
    parse(buffer, term_offsets, num_terms);
  }
  virtual const char* sentence() = 0;

 protected:
  NMEALocationData* nmea_data_;

 private:
};

/// Parser for GPGGA - Global Positioning System Fix Data.
class GPGGASentenceParser : public SentenceParser {
 public:
  GPGGASentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
  void parse(char* buffer, int term_offsets[], int num_terms) override final;
  const char* sentence() { return "GPGGA"; }

 private:
};

/// Parser for GPGLL - Geographic position, latitude / longitude
class GPGLLSentenceParser : public SentenceParser {
 public:
  GPGLLSentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
  void parse(char* buffer, int term_offsets[], int num_terms) override final;
  const char* sentence() { return "GPGLL"; }

 private:
};

/// Parser for GPRMC - Recommended minimum specific GPS/Transit data
class GPRMCSentenceParser : public SentenceParser {
 public:
  GPRMCSentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
  void parse(char* buffer, int term_offsets[], int num_terms) override final;
  const char* sentence() { return "GPRMC"; }

 private:
};

// class GPVTGSentenceParser : public SentenceParser {
// public:
//  GPVTGSentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
//  void parse(char* buffer, int term_offsets[], int num_terms) override final;
//  const char* sentence() { return "GPVTG"; }
// private:
//};

/// Top-level parser for PSTI* sentences.
class PSTISentenceParser : public SentenceParser {
 public:
  PSTISentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
  void parse(char* buffer, int term_offsets[], int num_terms) override final {}
  void parse(char* buffer, int term_offsets[], int num_terms,
             std::map<String, SentenceParser*>& sentence_parsers);
  const char* sentence() { return "PSTI"; }

 private:
};

/// Parser for STI,030 - Recommended Minimum 3D GNSS Data
class PSTI030SentenceParser : public SentenceParser {
 public:
  PSTI030SentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
  void parse(char* buffer, int term_offsets[], int num_terms) override final;
  const char* sentence() { return "PSTI,030"; }

 private:
};

/// Parser for STI,032 - RTK Baseline Data
class PSTI032SentenceParser : public SentenceParser {
 public:
  PSTI032SentenceParser(NMEALocationData* nmea_data) : SentenceParser{nmea_data} {}
  void parse(char* buffer, int term_offsets[], int num_terms) override final;
  const char* sentence() { return "PSTI,032"; }

 private:
};

/**
 * @brief NMEA 0183 parser class.
 * 
 * Data from an input stream is fed to the parse one character at a time.
 * 
 * At initialization, the parser registers a set of sentence parsers.
 * When input data matches a sentence, the parser calls the appropriate
 * sentence parser.
 */
class NMEAParser {
 public:
  NMEAParser();
  void handle(char c);
  void add_sentence_parser(SentenceParser* parser);

 private:
  void (NMEAParser::*current_state)(char);
  void state_start(char c);
  void state_in_term(char c);
  void state_in_checksum(char c);
  // current sentence
  char buffer[kNMEA0183InputBufferLength];
  // offset for each sentence term in the buffer
  int term_offsets[kNMEA0183MaxTerms];
  // pointer for the next character in buffer
  int cur_offset;
  // pointer for the current term in buffer
  int cur_term;
  int parity;
  bool validate_checksum();
  std::map<String, SentenceParser*> sentence_parsers;
};

}  // namespace sensesp

#endif
