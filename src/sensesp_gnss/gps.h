#ifndef _gps_H_
#define _gps_H_

#include "sensesp/sensors/sensor.h"
#include "sensesp_gnss/nmea_parser.h"

namespace sensesp {

/**
 * @brief Support for a GPS module communicating with NMEA 0183
 * messages over a serial interface.
 *
 * @param rx_stream Pointer to the Stream of incoming GPS data over
 * a serial connection.
 **/
class NMEA0183Input : public Startable {
 public:
  NMEA0183Input(Stream* rx_stream);
  virtual void start() override final;
  NMEALocationData nmea_data_;
  void add_sentence_parser(SentenceParser* parser) {
    nmea_parser_.add_sentence_parser(parser);
  }
 private:
  Stream* rx_stream_;
  NMEAParser nmea_parser_;
};

}  // namespace sensesp

#endif
