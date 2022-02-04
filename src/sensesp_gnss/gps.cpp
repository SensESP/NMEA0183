#include "gps.h"

#include <math.h>

#include "sensesp.h"

namespace sensesp {

NMEA0183Input::NMEA0183Input(Stream* rx_stream)
    : Sensor() {
  rx_stream_ = rx_stream;

  nmea_parser_.add_sentence_parser(new GPGGASentenceParser(&nmea_data_));
  nmea_parser_.add_sentence_parser(new GPGLLSentenceParser(&nmea_data_));
  nmea_parser_.add_sentence_parser(new GPRMCSentenceParser(&nmea_data_));
  nmea_parser_.add_sentence_parser(new PSTISentenceParser(&nmea_data_));
  nmea_parser_.add_sentence_parser(new PSTI030SentenceParser(&nmea_data_));
  nmea_parser_.add_sentence_parser(new PSTI032SentenceParser(&nmea_data_));

}

void NMEA0183Input::start() {
  // enable reading the serial port
  ReactESP::app->onAvailable(*rx_stream_, [this]() {
    while (rx_stream_->available()) {
      nmea_parser_.handle(rx_stream_->read());
    }
  });

  //#ifndef DEBUG_DISABLED
  //app.onRepeat(1000, [this](){
  //  debugD("GPS characters processed: %d", gps.charsProcessed());
  //  debugD("Sentences with fix: %d", gps.sentencesWithFix());
  //  debugD("Passed checksum: %d", gps.passedChecksum());
  //  debugD("Failed checksum: %d", gps.failedChecksum());
  //});
  //#endif
}

}  // namespace
