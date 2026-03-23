#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/navigation_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static HDMSentenceParser* hdm;
static HDTSentenceParser* hdt;

void setUp(void) {
  parser = new NMEA0183Parser();
  hdm = new HDMSentenceParser(parser);
  hdt = new HDTSentenceParser(parser);
}

void tearDown(void) {
  delete hdt;
  delete hdm;
  delete parser;
}

void test_hdm_valid_sentence(void) {
  // 101.1 degrees magnetic = 1.7645 radians
  parser->set("$HCHDM,101.1,M*28");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 101.1 * DEG_TO_RAD,
                           hdm->magnetic_heading_.get());
  TEST_ASSERT_EQUAL_INT(1, hdm->get_rx_count());
}

void test_hdt_valid_sentence(void) {
  // 98.3 degrees true = 1.7156 radians
  parser->set("$HCHDT,98.3,T*1B");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 98.3 * DEG_TO_RAD,
                           hdt->true_heading_.get());
  TEST_ASSERT_EQUAL_INT(1, hdt->get_rx_count());
}

void test_hdm_270_degrees(void) {
  // 270.5 degrees magnetic
  parser->set("$HCHDM,270.5,M*29");

  TEST_ASSERT_FLOAT_WITHIN(0.001, 270.5 * DEG_TO_RAD,
                           hdm->magnetic_heading_.get());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_hdm_valid_sentence);
  RUN_TEST(test_hdt_valid_sentence);
  RUN_TEST(test_hdm_270_degrees);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_hdm_valid_sentence);
  RUN_TEST(test_hdt_valid_sentence);
  RUN_TEST(test_hdm_270_degrees);

  return UNITY_END();
}
#endif
