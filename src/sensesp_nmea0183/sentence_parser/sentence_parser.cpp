#include "sentence_parser.h"

#include "sensesp_nmea0183/nmea0183.h"

namespace sensesp {

SentenceParser::SentenceParser(NMEA0183Input* nmea_io)
    : ignore_checksum_{false} {
  nmea_io->register_sentence_parser(this);
}

bool SentenceParser::parse(char* buffer) {
  if (!ignore_checksum_) {
    if (!validate_checksum(buffer)) {
      ESP_LOGW("SensESP/NMEA0183", "Invalid checksum in sentence %s,%s",
               sentence_address(), buffer);
      return false;
    }
  }

  char field_strings[kNMEA0183InputBufferLength];
  // The first field starts at the beginning of the buffer
  int field_offsets[kNMEA0183MaxFields] = {0};

  // Split the sentence into fields. field_strings is otherwise a copy
  // of buffer, but the commas are replaced with 0s. field_offsets
  // contains the offsets of the beginning of each field in buffer.
  // Since the first field starts at the beginning of the buffer,
  // the first offset is 0.

  int num_fields = 1;
  for (int i = 0; buffer[i] != 0; i++) {
    if (num_fields >= kNMEA0183MaxFields) {
      ESP_LOGW("SensESP/NMEA0183", "Too many fields in sentence %s,%s",
               sentence_address(), buffer);
      return false;
    }
    if (buffer[i] == ',') {
      field_offsets[num_fields] = i + 1;
      field_strings[i] = 0;
      num_fields++;
    } else if (buffer[i] == '*') {
      field_strings[i] = 0;
      break;
    } else if (buffer[i] == '\n' || buffer[i] == '\r') {
      field_strings[i] = 0;
      break;
    } else {
      field_strings[i] = buffer[i];
    }
  }

  return parse_fields(field_strings, field_offsets, num_fields);
}

bool SentenceParser::validate_checksum(char* buffer) {
  // Find the checksum field, delimited by a '*'
  char* checksum_str = strchr(buffer, '*');
  if (checksum_str == nullptr) {
    return false;
  }
  // Read the checksum value
  int checksum;
  int result = sscanf(checksum_str + 1, "%2x", &checksum);
  if (result != 1) {
    return false;
  }
  // Calculate the checksum. The checksum is the XOR of all bytes between '$'
  // and '*'. Our buffer doesn't include the address field and the first comma,
  // so start with XORing them.
  int chksum = calculate_checksum(sentence_address());
  chksum = calculate_checksum(",", chksum);
  chksum = calculate_checksum(buffer, chksum);

  return chksum == checksum;
}

int SentenceParser::calculate_checksum(const char* buffer, char seed) {
  int checksum = seed;
  for (const char* p = buffer; *p != '*' && *p != 0; p++) {
    checksum ^= *p;
  }
  return checksum;
}

}  // namespace sensesp
