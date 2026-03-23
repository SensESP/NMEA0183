#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static RTESentenceParser* rte;

void setUp(void) {
  parser = new NMEA0183Parser();
  rte = new RTESentenceParser(parser);
}

void tearDown(void) {
  delete rte;
  delete parser;
}

void test_rte_single_sentence(void) {
  parser->set("$GPRTE,1,1,c,ROUTE1,WP1,WP2,WP3*44");

  TEST_ASSERT_EQUAL_STRING("ROUTE1", rte->route_id_.get().c_str());
  std::vector<String> wps = rte->waypoints_.get();
  TEST_ASSERT_EQUAL_INT(3, wps.size());
  TEST_ASSERT_EQUAL_STRING("WP1", wps[0].c_str());
  TEST_ASSERT_EQUAL_STRING("WP2", wps[1].c_str());
  TEST_ASSERT_EQUAL_STRING("WP3", wps[2].c_str());
}

void test_rte_multi_sentence(void) {
  // First sentence — should not emit yet
  parser->set("$GPRTE,2,1,c,0,PBRCPK,CPNPT,BABRU*2F");

  // Second sentence — should emit accumulated waypoints
  parser->set("$GPRTE,2,2,c,0,FATEA,OCEAI*11");

  TEST_ASSERT_EQUAL_STRING("0", rte->route_id_.get().c_str());
  std::vector<String> wps = rte->waypoints_.get();
  TEST_ASSERT_EQUAL_INT(5, wps.size());
  TEST_ASSERT_EQUAL_STRING("PBRCPK", wps[0].c_str());
  TEST_ASSERT_EQUAL_STRING("CPNPT", wps[1].c_str());
  TEST_ASSERT_EQUAL_STRING("BABRU", wps[2].c_str());
  TEST_ASSERT_EQUAL_STRING("FATEA", wps[3].c_str());
  TEST_ASSERT_EQUAL_STRING("OCEAI", wps[4].c_str());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_rte_single_sentence);
  RUN_TEST(test_rte_multi_sentence);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_rte_single_sentence);
  RUN_TEST(test_rte_multi_sentence);

  return UNITY_END();
}
#endif
