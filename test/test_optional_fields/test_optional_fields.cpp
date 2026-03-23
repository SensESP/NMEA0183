#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/navigation_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static GGASentenceParser* gga;
static RMCSentenceParser* rmc;
static VTGSentenceParser* vtg;
static GLLSentenceParser* gll;
static GSVSentenceParser* gsv;
static HDGSentenceParser* hdg;
static VHWSentenceParser* vhw;
static DPTSentenceParser* dpt;

void setUp(void) {
  parser = new NMEA0183Parser();
  gga = new GGASentenceParser(parser);
  rmc = new RMCSentenceParser(parser);
  vtg = new VTGSentenceParser(parser);
  gll = new GLLSentenceParser(parser);
  gsv = new GSVSentenceParser(parser);
  hdg = new HDGSentenceParser(parser);
  vhw = new VHWSentenceParser(parser);
  dpt = new DPTSentenceParser(parser);
}

void tearDown(void) {
  delete dpt;
  delete vhw;
  delete hdg;
  delete gsv;
  delete gll;
  delete vtg;
  delete rmc;
  delete gga;
  delete parser;
}

// Sentences from issue #10 — NEO-7M module, near-stationary.
// These have empty optional fields that previously caused parse failure.

void test_rmc_empty_course_and_variation(void) {
  // True course (field 8) and variation (fields 10-11) are empty
  parser->set(
      "$GPRMC,123905.00,A,5605.75487,N,00945.65192,E,0.033,,200425,,,A*73");

  TEST_ASSERT_EQUAL_INT(1, rmc->get_rx_count());
  // Speed should be parsed: 0.033 knots
  TEST_ASSERT_FLOAT_WITHIN(0.001, 0.033 * 1852.0 / 3600.0,
                           rmc->speed_.get());
  // Position should be parsed
  Position pos = rmc->position_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.001, 56.0959, pos.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.001, 9.7609, pos.longitude);
}

void test_vtg_empty_track(void) {
  // True track (field 1) and magnetic track (field 3) are empty
  parser->set("$GPVTG,,T,,M,0.033,N,0.061,K,A*24");

  TEST_ASSERT_EQUAL_INT(1, vtg->get_rx_count());
  // Speed should be parsed: 0.033 knots
  TEST_ASSERT_FLOAT_WITHIN(0.001, 0.033 * 1852.0 / 3600.0,
                           vtg->speed_.get());
}

void test_gga_no_fix_empty_position(void) {
  // Position, altitude, geoidal separation, DGPS fields all empty
  parser->set(
      "$GNGGA,121224.00,,,,,,0,00,99.99,,,,,,*7E");

  TEST_ASSERT_EQUAL_INT(1, gga->get_rx_count());
  TEST_ASSERT_EQUAL_INT(0, gga->quality_.get());
  TEST_ASSERT_EQUAL_STRING("no GPS", gga->gnss_quality_.get().c_str());
}

void test_gga_valid_still_works(void) {
  // Ensure the fix doesn't break normal parsing
  parser->set(
      "$GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04,17.0,M,17.6,M,,"
      "0000*75");

  TEST_ASSERT_EQUAL_INT(1, gga->get_rx_count());
  Position pos = gga->position_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.001, 60.1845, pos.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.001, 25.0507, pos.longitude);
  TEST_ASSERT_EQUAL_INT(2, gga->quality_.get());
}

void test_gll_empty_position(void) {
  // No fix — lat/lon fields empty
  parser->set("$GNGLL,,,,,121223.00,V,N*55");

  TEST_ASSERT_EQUAL_INT(1, gll->get_rx_count());
  // position_ should not be updated (lat/lon are sentinel values)
}

void test_gsv_empty_satellite_data(void) {
  // One satellite block with valid PRN but empty elevation/azimuth/SNR.
  // 9 fields triggers v4.10 new-format parsing; the empty 9th field
  // is interpreted as an empty signal_id (also FLDP_OPT).
  parser->set("$GPGSV,1,1,01,01,,,,*55");

  TEST_ASSERT_EQUAL_INT(1, gsv->get_rx_count());
}

void test_hdg_empty_heading_deviation_variation(void) {
  // All fields empty
  parser->set("$IIHDG,,,,,*67");

  TEST_ASSERT_EQUAL_INT(1, hdg->get_rx_count());
  // No observer values should be updated (all fields are sentinel)
}

void test_vhw_empty_heading_and_speed(void) {
  // All data fields empty, only unit indicators present
  parser->set("$IIVHW,,T,,M,,N,,K*55");

  TEST_ASSERT_EQUAL_INT(1, vhw->get_rx_count());
  // No observer values should be updated
}

void test_dpt_empty_depth_and_offset(void) {
  // Empty depth and offset
  parser->set("$IIDPT,,*40");

  TEST_ASSERT_EQUAL_INT(1, dpt->get_rx_count());
  // No observer values should be updated
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_rmc_empty_course_and_variation);
  RUN_TEST(test_vtg_empty_track);
  RUN_TEST(test_gga_no_fix_empty_position);
  RUN_TEST(test_gga_valid_still_works);
  RUN_TEST(test_gll_empty_position);
  RUN_TEST(test_gsv_empty_satellite_data);
  RUN_TEST(test_hdg_empty_heading_deviation_variation);
  RUN_TEST(test_vhw_empty_heading_and_speed);
  RUN_TEST(test_dpt_empty_depth_and_offset);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_rmc_empty_course_and_variation);
  RUN_TEST(test_vtg_empty_track);
  RUN_TEST(test_gga_no_fix_empty_position);
  RUN_TEST(test_gga_valid_still_works);
  RUN_TEST(test_gll_empty_position);
  RUN_TEST(test_gsv_empty_satellite_data);
  RUN_TEST(test_hdg_empty_heading_deviation_variation);
  RUN_TEST(test_vhw_empty_heading_and_speed);
  RUN_TEST(test_dpt_empty_depth_and_offset);

  return UNITY_END();
}
#endif
