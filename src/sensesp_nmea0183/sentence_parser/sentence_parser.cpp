#include "sentence_parser.h"

#include "sensesp_nmea0183/nmea0183.h"

namespace sensesp::nmea0183 {

SentenceParser::SentenceParser(NMEA0183Parser* nmea_io) : ignore_checksum_{false} {
  nmea_io->register_sentence_parser(this);
}

bool SentenceParser::parse(const char* buffer) {
  if (!ignore_checksum_) {
    if (!validate_checksum(buffer)) {
      ESP_LOGW("SensESP/NMEA0183", "Invalid checksum in sentence: %s", buffer);
      return false;
    }
  }

  char field_strings[kNMEA0183InputBufferLength];
  strncpy(field_strings, buffer, kNMEA0183InputBufferLength);
  field_strings[kNMEA0183InputBufferLength - 1] = 0;

  int i;
  int checksum_location = -1;

  if (!ignore_checksum_) {
    // Remove the checksum from field_strings
    for (i = 0; field_strings[i] != 0; i++) {
      if (field_strings[i] == '*') {
        field_strings[i] = 0;
        break;
      }
    }
  }

  // The first field starts at the beginning of the buffer
  int field_offsets[kNMEA0183MaxFields] = {0};

  // Split the sentence into fields. field_strings is otherwise a copy
  // of buffer, but the commas are replaced with 0s. field_offsets
  // contains the offsets of the beginning of each field in buffer.
  // Since the first field starts after the first comma,
  // the first field offset is 1. The sentence start character and the
  // sentence name are in the zeroth field.

  int num_fields = 0;
  for (i = 0; field_strings[i] != 0; i++) {
    if (num_fields >= kNMEA0183MaxFields) {
      ESP_LOGW("SensESP/NMEA0183", "Too many fields in sentence: %s", buffer);
      return false;
    }
    if (field_strings[i] == ',') {
      num_fields++;
      field_strings[i] = 0;
      field_offsets[num_fields] = i + 1;
    } else if (field_strings[i] == '\r' || field_strings[i] == '\n') {
      field_strings[i] = 0;
      break;
    }
  }
  if (i > 0) {
    num_fields++;
  }

  bool result = parse_fields(field_strings, field_offsets, num_fields);
  if (result) {
    rx_count_++;
    this->emit(true);
  }
  return result;
}

bool SentenceParser::validate_checksum(const char* buffer) {
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
  // and '*'.
  int chksum = CalculateChecksum(buffer);

  return chksum == checksum;
}

}  // namespace sensesp::nmea0183
