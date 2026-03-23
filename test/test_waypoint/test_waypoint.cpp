#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static RMBSentenceParser* rmb;
static APBSentenceParser* apb;
static BWCSentenceParser* bwc;
static WPLSentenceParser* wpl;

void setUp(void) {
  parser = new NMEA0183Parser();
  rmb = new RMBSentenceParser(parser);
  apb = new APBSentenceParser(parser);
  bwc = new BWCSentenceParser(parser);
  wpl = new WPLSentenceParser(parser);
}

void tearDown(void) {
  delete wpl;
  delete bwc;
  delete apb;
  delete rmb;
  delete parser;
}

void test_rmb_valid(void) {
  parser->set(
      "$GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*20");

  // XTE: 0.66 NM to port = -0.66 * 1852 = -1222.32 m
  TEST_ASSERT_FLOAT_WITHIN(1.0, -1222.32, rmb->cross_track_error_.get());
  // Bearing: 52.5 degrees
  TEST_ASSERT_FLOAT_WITHIN(0.001, 52.5 * DEG_TO_RAD,
                           rmb->bearing_to_destination_.get());
  // Range: 1.3 NM = 2407.6 m
  TEST_ASSERT_FLOAT_WITHIN(1.0, 2407.6, rmb->range_to_destination_.get());
  // Closing velocity: 0.5 knots = 0.2572 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.2572,
                           rmb->destination_closing_velocity_.get());
  TEST_ASSERT_EQUAL_STRING("004",
                           rmb->destination_waypoint_id_.get().c_str());
}

void test_apb_valid(void) {
  parser->set("$GPAPB,A,A,0.10,R,N,V,V,011.0,M,DEST,011.0,M,011.0,M*22");

  // XTE: 0.10 NM to starboard = +185.2 m
  TEST_ASSERT_FLOAT_WITHIN(1.0, 185.2, apb->cross_track_error_.get());
  // Heading to steer: 11.0 degrees
  TEST_ASSERT_FLOAT_WITHIN(0.001, 11.0 * DEG_TO_RAD,
                           apb->heading_to_steer_.get());
}

void test_bwc_valid(void) {
  parser->set(
      "$GPBWC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM*21");

  // Bearing true: 213.8 degrees
  TEST_ASSERT_FLOAT_WITHIN(0.001, 213.8 * DEG_TO_RAD,
                           bwc->bearing_true_.get());
  // Bearing magnetic: 218.0 degrees
  TEST_ASSERT_FLOAT_WITHIN(0.001, 218.0 * DEG_TO_RAD,
                           bwc->bearing_magnetic_.get());
  // Distance: 4.6 NM = 8519.2 m
  TEST_ASSERT_FLOAT_WITHIN(1.0, 8519.2, bwc->distance_.get());
  TEST_ASSERT_EQUAL_STRING("EGLM", bwc->waypoint_id_.get().c_str());
  // Waypoint position
  Position pos = bwc->waypoint_position_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.001, 51.5003, pos.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.001, -0.7723, pos.longitude);
}

void test_wpl_valid(void) {
  parser->set("$GPWPL,4917.16,N,12310.64,W,003*65");

  Position pos = wpl->position_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.001, 49.2860, pos.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.001, -123.1773, pos.longitude);
  TEST_ASSERT_EQUAL_STRING("003", wpl->waypoint_id_.get().c_str());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_rmb_valid);
  RUN_TEST(test_apb_valid);
  RUN_TEST(test_bwc_valid);
  RUN_TEST(test_wpl_valid);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_rmb_valid);
  RUN_TEST(test_apb_valid);
  RUN_TEST(test_bwc_valid);
  RUN_TEST(test_wpl_valid);

  return UNITY_END();
}
#endif
