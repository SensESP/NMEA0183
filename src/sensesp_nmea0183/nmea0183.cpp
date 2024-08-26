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

int CalculateChecksum(const char* buffer, char seed) {
  int checksum = seed;
  for (const char* p = buffer; *p != '*' && *p != 0; p++) {
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

void NMEA0183::set(const String& line) {

    // Trim trailing whitespace
    String trimmed = line;
    trimmed.trim();

    // Parse the sentence
    parse_sentence(trimmed);
    return;
}

void NMEA0183::parse_sentence(const String& sentence) {
  const char* tail = sentence.c_str();

  // Check that the sentence starts with a dollar or an exclamation sign
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
      bool result = parser->parse(tail);
      ESP_LOGV("SensESP/NMEA0183", "Parsed sentence %s with result %s",
               sentence.c_str(), result ? "true" : "false");
      return;
    }
  }

  // Get the address field, max 5 characters, delimited by a comma
  char address[6];
  int i = 0;
  while (tail[i] != ',' && i < 5) {
    address[i] = tail[i];
    i++;
  }
  address[i] = '\0';
  // Verify that the address field is not empty and that a comma follows
  if (i == 0 || tail[i] != ',') {
    return;
  }
}

void ReportFailure(bool ok, const char* sentence) {
  if (!ok) {
    ESP_LOGW("SensESP/NMEA0183", "Failed to parse %s", sentence);
    return;
  }
}

void NMEA0183::register_sentence_parser(SentenceParser* parser) {
  sentence_parsers.push_back(parser);
}

}  // namespace sensesp::nmea0183
