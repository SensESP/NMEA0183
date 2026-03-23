#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/navigation_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static DBTSentenceParser* dbt;
static MTWSentenceParser* mtw;

void setUp(void) {
  parser = new NMEA0183Parser();
  dbt = new DBTSentenceParser(parser);
  mtw = new MTWSentenceParser(parser);
}

void tearDown(void) {
  delete mtw;
  delete dbt;
  delete parser;
}

void test_dbt_all_units(void) {
  // DBT with all three depth units
  parser->set("$SDDBT,41.3,f,12.6,M,6.9,F*05");

  // Should prefer meters
  TEST_ASSERT_FLOAT_WITHIN(0.01, 12.6, dbt->depth_.get());
  TEST_ASSERT_EQUAL_INT(1, dbt->get_rx_count());
}

void test_dbt_feet_only(void) {
  // DBT with only feet populated (meters field empty)
  parser->set("$SDDBT,41.3,f,,M,,F*3E");

  // Should convert feet to meters: 41.3 * 0.3048 = 12.588
  TEST_ASSERT_FLOAT_WITHIN(0.01, 12.588, dbt->depth_.get());
}

void test_dbt_invalid_checksum(void) {
  int rx_before = dbt->get_rx_count();
  parser->set("$SDDBT,41.3,f,12.6,M,6.9,F*FF");

  TEST_ASSERT_EQUAL_INT(rx_before, dbt->get_rx_count());
}

void test_mtw_celsius_to_kelvin(void) {
  // MTW: 17.8 C should convert to 290.95 K
  parser->set("$YXMTW,17.8,C*1B");

  TEST_ASSERT_FLOAT_WITHIN(0.01, 290.95, mtw->water_temperature_.get());
  TEST_ASSERT_EQUAL_INT(1, mtw->get_rx_count());
}

void test_mtw_zero_celsius(void) {
  // 0 C = 273.15 K
  parser->set("$YXMTW,0.0,C*04");

  TEST_ASSERT_FLOAT_WITHIN(0.01, 273.15, mtw->water_temperature_.get());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_dbt_all_units);
  RUN_TEST(test_dbt_feet_only);
  RUN_TEST(test_dbt_invalid_checksum);
  RUN_TEST(test_mtw_celsius_to_kelvin);
  RUN_TEST(test_mtw_zero_celsius);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_dbt_all_units);
  RUN_TEST(test_dbt_feet_only);
  RUN_TEST(test_dbt_invalid_checksum);
  RUN_TEST(test_mtw_celsius_to_kelvin);
  RUN_TEST(test_mtw_zero_celsius);

  return UNITY_END();
}
#endif
