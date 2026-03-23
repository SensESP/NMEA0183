#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static MWDSentenceParser* mwd;
static VWRSentenceParser* vwr;
static MWVSentenceParser* mwv;
static TrueWindMWVSentenceParser* mwv_true;

void setUp(void) {
  parser = new NMEA0183Parser();
  mwd = new MWDSentenceParser(parser);
  vwr = new VWRSentenceParser(parser);
  mwv = new MWVSentenceParser(parser);
  mwv_true = new TrueWindMWVSentenceParser(parser);
}

void tearDown(void) {
  delete mwv_true;
  delete mwv;
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

void test_mwv_apparent_non_wi_talker(void) {
  // MWV with II talker (integrated instruments) should parse as apparent wind
  parser->set("$IIMWV,045.0,R,12.5,N,A*0A");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 45.0 * DEG_TO_RAD,
                           mwv->apparent_wind_angle_.get());
  // 12.5 knots * 0.514444 = 6.4306 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.01, 6.4306, mwv->apparent_wind_speed_.get());
  TEST_ASSERT_EQUAL_INT(1, mwv->get_rx_count());
}

void test_mwv_apparent_rejects_true_wind(void) {
  // MWV with T (true) reference should be rejected by apparent wind parser
  parser->set("$IIMWV,225.0,T,6.4,M,A*3F");

  TEST_ASSERT_EQUAL_INT(0, mwv->get_rx_count());
}

void test_mwv_true_wind(void) {
  // MWV with T (true) reference should be parsed by true wind parser
  parser->set("$IIMWV,225.0,T,6.4,M,A*3F");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 225.0 * DEG_TO_RAD,
                           mwv_true->true_wind_direction_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 6.4, mwv_true->true_wind_speed_.get());
  TEST_ASSERT_EQUAL_INT(1, mwv_true->get_rx_count());
}

void test_mwv_true_rejects_apparent_wind(void) {
  // MWV with R (relative) reference should be rejected by true wind parser
  parser->set("$IIMWV,045.0,R,12.5,N,A*0A");

  // Apparent parser should get it, not the true wind parser
  TEST_ASSERT_EQUAL_INT(1, mwv->get_rx_count());
  TEST_ASSERT_EQUAL_INT(0, mwv_true->get_rx_count());
}

void test_mwv_true_wind_knots(void) {
  // MWV true wind in knots — should convert to m/s
  parser->set("$IIMWV,180.0,T,15.0,N,A*06");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 180.0 * DEG_TO_RAD,
                           mwv_true->true_wind_direction_.get());
  // 15.0 knots * 0.514444 = 7.7167 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.01, 7.7167, mwv_true->true_wind_speed_.get());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_mwd_with_ms);
  RUN_TEST(test_mwd_knots_only);
  RUN_TEST(test_vwr_starboard);
  RUN_TEST(test_vwr_port);
  RUN_TEST(test_mwv_apparent_non_wi_talker);
  RUN_TEST(test_mwv_apparent_rejects_true_wind);
  RUN_TEST(test_mwv_true_wind);
  RUN_TEST(test_mwv_true_rejects_apparent_wind);
  RUN_TEST(test_mwv_true_wind_knots);

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
  RUN_TEST(test_mwv_apparent_non_wi_talker);
  RUN_TEST(test_mwv_apparent_rejects_true_wind);
  RUN_TEST(test_mwv_true_wind);
  RUN_TEST(test_mwv_true_rejects_apparent_wind);
  RUN_TEST(test_mwv_true_wind_knots);

  return UNITY_END();
}
#endif
