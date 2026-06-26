#include <unity.h>

#include "sensesp/types/position.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static SkyTraqPSTI030SentenceParser* psti030;
static SkyTraqPSTI032SentenceParser* psti032;
static QuectelPQTMTARSentenceParser* pqtmtar;

void setUp(void) {
  parser = new NMEA0183Parser();
  psti030 = new SkyTraqPSTI030SentenceParser(parser);
  psti032 = new SkyTraqPSTI032SentenceParser(parser);
  pqtmtar = new QuectelPQTMTARSentenceParser(parser);
}

void tearDown(void) {
  delete pqtmtar;
  delete psti032;
  delete psti030;
  delete parser;
}

// SkyTraq PSTI,030 — Recommended Minimum 3D GNSS Data.
// Fields after the "$PSTI,030" address: UTC time, status, lat, N/S, lon, E/W,
// altitude, E/N/U velocity, UTC date, mode, RTK age, RTK ratio.
void test_psti030_full(void) {
  parser->set(
      "$PSTI,030,044606.000,A,2447.0924110,N,12100.5227860,E,103.323,0.00,0.00,"
      "0.00,180915,R,1.2,4.2*02");

  TEST_ASSERT_EQUAL_INT(1, psti030->get_rx_count());
  TEST_ASSERT_EQUAL_STRING("RTK fixed integer",
                           psti030->gnss_quality_.get().c_str());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.2, psti030->rtk_age_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 4.2, psti030->rtk_ratio_.get());

  Position pos = psti030->position_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 24.784873, pos.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 121.008713, pos.longitude);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 103.323, pos.altitude);

  ENUVector vel = psti030->enu_velocity_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, vel.east);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, vel.north);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, vel.up);
}

// SkyTraq PSTI,032 — RTK Baseline Data.
// Fields after the "$PSTI,032" address: UTC time, UTC date, status, mode,
// E/N/U baseline projection, baseline length, baseline course.
void test_psti032_full(void) {
  parser->set(
      "$PSTI,032,041457.000,170316,A,R,0.603,-0.837,-0.089,1.036,144.22,,,,,"
      "*1C");

  TEST_ASSERT_EQUAL_INT(1, psti032->get_rx_count());
  TEST_ASSERT_EQUAL_STRING("RTK fixed integer",
                           psti032->gnss_quality_.get().c_str());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.036, psti032->baseline_length_.get());
  // Baseline course is reported in radians (144.22 degrees).
  TEST_ASSERT_FLOAT_WITHIN(0.001, 2.51711, psti032->baseline_course_.get());

  ENUVector proj = psti032->baseline_projection_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.001, 0.603, proj.east);
  TEST_ASSERT_FLOAT_WITHIN(0.001, -0.837, proj.north);
  TEST_ASSERT_FLOAT_WITHIN(0.001, -0.089, proj.up);
}

// Quectel PQTMTAR — Time and Attitude (heading status 4 = RTK fixed).
// Attitude angles are reported in degrees.
void test_pqtmtar_attitude(void) {
  parser->set(
      "$PQTMTAR,1,165331.000,4,,0.232,2.321340,-6.849396,80.410065,0.081330,"
      "0.045079,0.054334,00*70");

  TEST_ASSERT_EQUAL_INT(1, pqtmtar->get_rx_count());
  TEST_ASSERT_EQUAL_STRING("RTK fixed integer",
                           pqtmtar->rtk_quality_.get().c_str());
  TEST_ASSERT_EQUAL_INT(0, pqtmtar->hdg_num_satellites_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.001, 0.232, pqtmtar->baseline_length_.get());

  AttitudeVector att = pqtmtar->attitude_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 2.321340, att.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001, -6.849396, att.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 80.410065, att.yaw);

  AttitudeVector acc = pqtmtar->attitude_accuracy_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.081330, acc.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.045079, acc.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.054334, acc.yaw);
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_psti030_full);
  RUN_TEST(test_psti032_full);
  RUN_TEST(test_pqtmtar_attitude);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_psti030_full);
  RUN_TEST(test_psti032_full);
  RUN_TEST(test_pqtmtar_attitude);

  return UNITY_END();
}
#endif
