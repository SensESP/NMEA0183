#ifndef _gps_H_
#define _gps_H_

#include "sensesp/sensors/sensor.h"
#include "sensesp_gnss/nmea_parser.h"

namespace sensesp {

/**
 * @brief Support for a GPS module communicating with NMEA-0183
 * messages over a serial interface.
 * 
 * @param rx_stream Pointer to the Stream of incoming GPS data over
 * a serial connection. 
 **/ 

class GPSInput : public Sensor {
 public:
  GPSInput(Stream* rx_stream);
  virtual void start() override final;
  NMEAData nmea_data_;
 private:
  Stream* rx_stream_;
  NMEAParser nmea_parser_;
};

}  // namespace sensesp

#endif
