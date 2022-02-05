/**
 * @file gnss_receiver.cpp
 * @brief GNSS (GPS/GLONASS/GALILEO) receiver example.
 *
 */

#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/wiring.h"

using namespace sensesp;

ReactESP app;

constexpr int kGNSSBitRate = 9600;
constexpr int kGNSSRxPin = 15;
// set the Tx pin to -1 if you don't want to use it
constexpr int kGNSSTxPin = 13;

void setup() {
  SensESPAppBuilder builder;
  sensesp_app = builder.set_hostname("sensesp-gnss")->get_app();

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  HardwareSerial* serial = &Serial1;
  serial->begin(kGNSSBitRate, SERIAL_8N1, kGNSSRxPin, kGNSSTxPin);

  NMEA0183Input* nmea_input = new NMEA0183Input(serial);
  ConnectLocationSKOutputs(nmea_input);

  sensesp_app->start();
}

void loop() { app.tick(); }
