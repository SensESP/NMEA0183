#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static MWDSentenceParser* mwd;
static VWRSentenceParser* vwr;

void setUp(void) {
  parser = new NMEA0183Parser();
  mwd = new MWDSentenceParser(parser);
  vwr = new VWRSentenceParser(parser);
}

void tearDown(void) {
  delete vwr;
  delete mwd;
  delete parser;
}

void test_mwd_with_ms(void) {
  // MWD with both knots and m/s — should prefer m/s
  parser->set("$WIMWD,225.0,T,220.0,M,12.5,N,6.4,M*6B");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 225.0 * DEG_TO_RAD,
                           mwd->true_wind_direction_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 6.4, mwd->true_wind_speed_.get());
  TEST_ASSERT_EQUAL_INT(1, mwd->get_rx_count());
}

void test_mwd_knots_only(void) {
  // MWD with only knots (m/s field empty)
  parser->set("$WIMWD,180.0,T,175.0,M,15.0,N,,M*4A");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 180.0 * DEG_TO_RAD,
                           mwd->true_wind_direction_.get());
  // 15.0 knots = 15.0 * 1852 / 3600 = 7.7167 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.01, 7.7167, mwd->true_wind_speed_.get());
}

void test_vwr_starboard(void) {
  // VWR with R (starboard) — angle should be positive
  parser->set("$IIVWR,045.0,R,12.5,N,6.4,M,23.2,K*4F");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 45.0 * DEG_TO_RAD,
                           vwr->apparent_wind_angle_.get());
  // Should prefer m/s = 6.4
  TEST_ASSERT_FLOAT_WITHIN(0.01, 6.4, vwr->apparent_wind_speed_.get());
  TEST_ASSERT_EQUAL_INT(1, vwr->get_rx_count());
}

void test_vwr_port(void) {
  // VWR with L (port) — angle should be negative
  parser->set("$IIVWR,135.0,L,10.0,N,5.1,M,18.5,K*59");

  TEST_ASSERT_FLOAT_WITHIN(0.001, -135.0 * DEG_TO_RAD,
                           vwr->apparent_wind_angle_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 5.1, vwr->apparent_wind_speed_.get());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_mwd_with_ms);
  RUN_TEST(test_mwd_knots_only);
  RUN_TEST(test_vwr_starboard);
  RUN_TEST(test_vwr_port);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_mwd_with_ms);
  RUN_TEST(test_mwd_knots_only);
  RUN_TEST(test_vwr_starboard);
  RUN_TEST(test_vwr_port);

  return UNITY_END();
}
#endif
