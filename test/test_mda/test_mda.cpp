#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/weather_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static MDASentenceParser* mda;

void setUp(void) {
  parser = new NMEA0183Parser();
  mda = new MDASentenceParser(parser);
}

void tearDown(void) {
  delete mda;
  delete parser;
}

void test_mda_full_sentence(void) {
  parser->set(
      "$WIMDA,29.7544,I,1.0076,B,16.1,C,,,42.6,,11.2,C,225.0,T,220.0,M,"
      "12.5,N,6.4,M*55");

  // Pressure: 1.0076 bars = 100760 Pa
  TEST_ASSERT_FLOAT_WITHIN(1.0, 100760.0, mda->barometric_pressure_.get());
  // Air temp: 16.1 C = 289.25 K
  TEST_ASSERT_FLOAT_WITHIN(0.01, 289.25, mda->air_temperature_.get());
  // Humidity: 42.6% = 0.426
  TEST_ASSERT_FLOAT_WITHIN(0.001, 0.426, mda->relative_humidity_.get());
  // Dew point: 11.2 C = 284.35 K
  TEST_ASSERT_FLOAT_WITHIN(0.01, 284.35, mda->dew_point_.get());
  // Wind direction: 225.0 degrees true
  TEST_ASSERT_FLOAT_WITHIN(0.001, 225.0 * DEG_TO_RAD,
                           mda->true_wind_direction_.get());
  // Wind speed: prefer m/s = 6.4
  TEST_ASSERT_FLOAT_WITHIN(0.01, 6.4, mda->true_wind_speed_.get());
  TEST_ASSERT_EQUAL_INT(1, mda->get_rx_count());
}

void test_mda_pressure_only(void) {
  parser->set("$WIMDA,29.7544,I,1.0076,B,,,,,,,,,,,,,,,,*64");

  // Should parse pressure even when other fields are empty
  TEST_ASSERT_FLOAT_WITHIN(1.0, 100760.0, mda->barometric_pressure_.get());
  TEST_ASSERT_EQUAL_INT(1, mda->get_rx_count());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_mda_full_sentence);
  RUN_TEST(test_mda_pressure_only);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_mda_full_sentence);
  RUN_TEST(test_mda_pressure_only);

  return UNITY_END();
}
#endif
