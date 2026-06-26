#ifndef SENSESP_NMEA0183_PROPRIETARY_SENTENCE_PARSER_H_
#define SENSESP_NMEA0183_PROPRIETARY_SENTENCE_PARSER_H_

#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace sensesp::nmea0183 {

/**
 * @brief Base class for proprietary NMEA 0183 sentence parsers.
 *
 * Proprietary sentences carry a "$P" prefix followed by a manufacturer
 * mnemonic and a message type, e.g. "$PSTI,030,..." (SkyTraq) or
 * "$PQTMTAR,..." (Quectel). Subclasses supply the sentence address at
 * construction instead of overriding sentence_address(); the address is
 * matched against the sentence tail by the dispatcher and must cover
 * everything up to (but not including) the first data field.
 *
 * Comma-separated fields are numbered from the sentence start with the
 * address in field 0. When the message type is a separate comma-delimited
 * token (as in "$PSTI,030") it occupies field 1, so the first data field is
 * field 2; addresses without an embedded comma (as in "$PQTMTAR") keep the
 * first data field at field 1.
 */
class ProprietarySentenceParser : public SentenceParser {
 public:
  ProprietarySentenceParser(NMEA0183Parser* nmea, const char* address)
      : SentenceParser(nmea), address_(address) {}

  const char* sentence_address() override final { return address_; }

 private:
  const char* address_;
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_PROPRIETARY_SENTENCE_PARSER_H_
