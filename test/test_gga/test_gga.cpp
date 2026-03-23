#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static GGASentenceParser* gga;

void setUp(void) {
  parser = new NMEA0183Parser();
  gga = new GGASentenceParser(parser);
}

void tearDown(void) {
  delete gga;
  delete parser;
}

void test_gga_valid_sentence(void) {
  // Real-world GGA sentence
  parser->set(
      "$GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04,17.0,M,17.6,M,,"
      "0000*75");

  Position pos = gga->position_.get();
  TEST_ASSERT_FLOAT_WITHIN(0.001, 60.1845, pos.latitude);
  TEST_ASSERT_FLOAT_WITHIN(0.001, 25.0507, pos.longitude);
  TEST_ASSERT_EQUAL_INT(2, gga->quality_.get());
  TEST_ASSERT_EQUAL_STRING("DGNSS fix", gga->gnss_quality_.get().c_str());
  TEST_ASSERT_EQUAL_INT(11, gga->num_satellites_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.04, gga->horizontal_dilution_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.1, 17.0, gga->position_.get().altitude);
  TEST_ASSERT_FLOAT_WITHIN(0.1, 17.6, gga->geoidal_separation_.get());
  TEST_ASSERT_EQUAL_INT(1, gga->get_rx_count());
}

void test_gga_no_fix_parses_quality(void) {
  // GGA with no fix has empty position fields — optional field parsers accept
  // these, so the sentence parses and quality=0 is emitted.
  parser->set(
      "$GNGGA,121224.00,,,,,,0,00,99.99,,,,,,*7E");

  TEST_ASSERT_EQUAL_INT(1, gga->get_rx_count());
  TEST_ASSERT_EQUAL_INT(0, gga->quality_.get());
  TEST_ASSERT_EQUAL_STRING("no GPS", gga->gnss_quality_.get().c_str());
}

void test_gga_invalid_checksum(void) {
  // Sentence with wrong checksum — should not parse
  int rx_before = gga->get_rx_count();
  parser->set(
      "$GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04,17.0,M,17.6,M,,"
      "0000*FF");

  TEST_ASSERT_EQUAL_INT(rx_before, gga->get_rx_count());
}

void test_gga_rx_count_increments(void) {
  parser->set(
      "$GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04,17.0,M,17.6,M,,"
      "0000*75");
  parser->set(
      "$GNGGA,121042.00,6011.07385,N,02503.04396,E,2,11,1.04,17.0,M,17.6,M,,"
      "0000*75");

  TEST_ASSERT_EQUAL_INT(2, gga->get_rx_count());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_gga_valid_sentence);
  RUN_TEST(test_gga_no_fix_parses_quality);
  RUN_TEST(test_gga_invalid_checksum);
  RUN_TEST(test_gga_rx_count_increments);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_gga_valid_sentence);
  RUN_TEST(test_gga_no_fix_parses_quality);
  RUN_TEST(test_gga_invalid_checksum);
  RUN_TEST(test_gga_rx_count_increments);

  return UNITY_END();
}
#endif
