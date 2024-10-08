/**
 * @file gnss_receiver.cpp
 * @brief GNSS (GPS/GLONASS/GALILEO) receiver example.
 *
 */

#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

constexpr int kGNSSBitRate = 9600;
constexpr int kGNSSRxPin = 15;
// set the Tx pin to -1 if you don't want to use it
constexpr int kGNSSTxPin = 13;

void setup() {
  SetupLogging();

  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("sensesp-gnss")->get_app();

  HardwareSerial* serial = &Serial1;
  serial->begin(kGNSSBitRate, SERIAL_8N1, kGNSSRxPin, kGNSSTxPin);

  NMEA0183IOTask* nmea0183_io_task = new NMEA0183IOTask(serial);

  ConnectGNSS(&nmea0183_io_task->parser_, new GNSSData());
}

void loop() { event_loop()->tick(); }
