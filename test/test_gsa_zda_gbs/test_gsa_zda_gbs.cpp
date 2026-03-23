#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static GSASentenceParser* gsa;
static ZDASentenceParser* zda;
static GBSSentenceParser* gbs;

void setUp(void) {
  parser = new NMEA0183Parser();
  gsa = new GSASentenceParser(parser);
  zda = new ZDASentenceParser(parser);
  gbs = new GBSSentenceParser(parser);
}

void tearDown(void) {
  delete gbs;
  delete zda;
  delete gsa;
  delete parser;
}

void test_gsa_3d_fix(void) {
  parser->set("$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39");

  TEST_ASSERT_EQUAL_INT(3, gsa->fix_type_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 2.5, gsa->pdop_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.3, gsa->hdop_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 2.1, gsa->vdop_.get());
  TEST_ASSERT_EQUAL_INT(1, gsa->get_rx_count());
}

void test_gsa_no_fix(void) {
  parser->set("$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30");

  TEST_ASSERT_EQUAL_INT(1, gsa->fix_type_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 99.99, gsa->pdop_.get());
}

void test_zda_datetime(void) {
  // ZDA: 16:00:12.71 on 2004-03-11
  parser->set("$GPZDA,160012.71,11,03,2004,-1,00*7D");

  time_t t = zda->datetime_.get();
  struct tm* tm = gmtime(&t);

  // Note: mktime uses local timezone; we verify key fields
  TEST_ASSERT_EQUAL_INT(2004 - 1900, tm->tm_year);
  TEST_ASSERT_EQUAL_INT(2, tm->tm_mon);  // March = 2 (0-based)
  TEST_ASSERT_EQUAL_INT(11, tm->tm_mday);
  TEST_ASSERT_EQUAL_INT(1, zda->get_rx_count());
}

void test_gbs_error_estimates(void) {
  parser->set("$GPGBS,235458.00,1.4,1.3,3.1,03,,-21.4,3.8*5B");

  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.4, gbs->lat_error_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.3, gbs->lon_error_.get());
  TEST_ASSERT_FLOAT_WITHIN(0.01, 3.1, gbs->alt_error_.get());
  TEST_ASSERT_EQUAL_INT(1, gbs->get_rx_count());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_gsa_3d_fix);
  RUN_TEST(test_gsa_no_fix);
  RUN_TEST(test_zda_datetime);
  RUN_TEST(test_gbs_error_estimates);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_gsa_3d_fix);
  RUN_TEST(test_gsa_no_fix);
  RUN_TEST(test_zda_datetime);
  RUN_TEST(test_gbs_error_estimates);

  return UNITY_END();
}
#endif
