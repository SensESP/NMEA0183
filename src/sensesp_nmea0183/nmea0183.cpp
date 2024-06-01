#include "nmea0183.h"

#include <math.h>

#include "sensesp.h"

namespace sensesp {

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

NMEA0183::NMEA0183(Stream* rx_stream) : rx_stream_{rx_stream} {
  // enable reading the serial port
  ReactESP::app->onAvailable(*rx_stream_, [this]() {
    while (rx_stream_->available()) {
      this->handle(rx_stream_->read());
    }
  });
}

void NMEA0183::handle(char c) {
  // Check that we're not overflowing the buffer
  if (input_offset == kNMEA0183InputBufferLength - 1) {
    input_offset = 0;
  }

  // Swallow trailing whitespace
  if (c == '\r') {
    return;
  }

  // Check if we've reached the end of the sentence
  if (c == '\n') {
    // Null-terminate the buffer
    input_buffer[input_offset] = '\0';

    // Parse the sentence
    parse_sentence();
    input_offset = 0;
    return;
  }

  // Add the character to the buffer
  input_buffer[input_offset++] = c;
}

void NMEA0183::parse_sentence() {
  char* tail = input_buffer;

  // Check that the sentence starts with a dollar sign
  if (tail[0] != '$') {
    return;
  }
  tail++;

  // Loop through sentence parsers and find the one that matches the sentence.

  for (auto parser : sentence_parsers) {
    int address_length = strlen(parser->sentence_address());
    if (strncmpwc(tail, parser->sentence_address(), address_length) == 0) {
      // Check that the address field is followed by a comma
      if (tail[address_length] != ',') {
        continue;
      }
      tail += address_length + 1;
      bool result = parser->parse(tail);
      ESP_LOGV("SensESP/NMEA0183", "Parsed sentence %s with result %s",
               input_buffer, result ? "true" : "false");
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

}  // namespace sensesp
