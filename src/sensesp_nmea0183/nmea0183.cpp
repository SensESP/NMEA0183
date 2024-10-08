#include "nmea0183.h"

#include <math.h>

#include "sensesp.h"

namespace sensesp::nmea0183 {

/**
 * @brief String comparision with a maximum length and wildcard support.
 *
 * @param s1 String to compare.
 * @param s2 String to compare against. May contain '.' as a wildcard.
 * @param n Maximum number of characters to compare. Strings may be shorter.
 * @return int 0 if the strings are equal, 1 otherwise.
 */
static int strncmpwc(const char* s1, const char* s2, int n) {
  for (int i = 0; i < n; i++) {
    if (s1[i] == 0 && s2[i] == 0) {
      return 0;
    }
    if (s2[i] == '.' || s1[i] == s2[i]) {
      continue;
    }
    return 1;
  }
  return 0;
}

/**
 * @brief Calculate the NMEA 0183 checksum for the given buffer.
 *
 * The buffer should contain the sentence start character '$' (which is
 * ignored).
 *
 * @param buffer
 * @param seed
 * @return int
 */
int CalculateChecksum(const char* buffer, char seed) {
  int checksum = seed;
  // Skip the sentence start character
  for (const char* p = buffer + 1; *p != '*' && *p != 0; p++) {
    checksum ^= *p;
  }
  return checksum;
}

void AddChecksum(String& sentence) {
  int checksum = CalculateChecksum(sentence.c_str());
  char checksum_str[3];
  sprintf(checksum_str, "%02X", checksum);
  sentence += "*" + String(checksum_str);
}

void NMEA0183Parser::set(const String& line) {
  // Trim trailing whitespace
  String trimmed = line;
  trimmed.trim();

  // Parse the sentence
  parse_sentence(trimmed);
  return;
}

void NMEA0183Parser::parse_sentence(const String& sentence) {
  const char* sentence_str = sentence.c_str();
  const char* tail = sentence.c_str();

  // Check that the sentence starts with a dollar or an exclamation sign
  // (AIS sentences only)
  if (tail[0] != '$' && tail[0] != '!') {
    return;
  }
  // Move the tail pointer past the sentence begin character
  tail++;

  // Loop through sentence parsers and find the one that matches the sentence.

  for (auto parser : sentence_parsers) {
    int address_length = strlen(parser->sentence_address());
    if (strncmpwc(tail, parser->sentence_address(), address_length) == 0) {
      // Check that the address field is followed by a comma
      if (tail[address_length] != ',') {
        continue;
      }
      bool result = parser->parse(sentence_str);
      ESP_LOGV("SensESP/NMEA0183", "Parsed sentence %s with result %s",
               sentence_str, result ? "true" : "false");
      return;
    }
  }
  ESP_LOGV("SensESP/NMEA0183", "No parser found for sentence %s", sentence_str);
}

void ReportFailure(bool ok, const char* sentence) {
  if (!ok) {
    ESP_LOGW("SensESP/NMEA0183", "Failed to parse %s", sentence);
    return;
  }
}

void NMEA0183Parser::register_sentence_parser(SentenceParser* parser) {
  sentence_parsers.push_back(parser);
}

}  // namespace sensesp::nmea0183
