/**
 * @file gnss_receiver.cpp
 * @brief GNSS (GPS/GLONASS/GALILEO) receiver example.
 *
 */

#include "sensesp_app_builder.h"
#include "sensesp/system/stream_producer.h"
#include "sensesp/transforms/filter.h"

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

  StreamLineProducer* ais_line_producer = new StreamLineProducer(serial);

  Filter<String>* sentence_filter = new Filter<String>([](const String& line) {
    return line.startsWith("!") || line.startsWith("$");
  });

  ais_line_producer->connect_to(sentence_filter);

  NMEA0183* nmea = new NMEA0183();
  sentence_filter->connect_to(nmea);

  ConnectGNSS(nmea, new GNSSData());
}

void loop() { SensESPBaseApp::get_event_loop()->tick(); }
