#include <unity.h>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static RTESentenceParser* rte;
static uint32_t fake_millis;

void setUp(void) {
  fake_millis = 0;
  parser = new NMEA0183Parser();
  rte = new RTESentenceParser(parser,
                              RTESentenceParser::kDefaultSequenceTimeoutMs,
                              []() { return fake_millis; });
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

void test_rte_gap_discards_partial(void) {
  parser->set("$GPRTE,3,1,c,0,PBRCPK,CPNPT*44");  // first of 3
  parser->set("$GPRTE,3,3,c,0,FATEA,OCEAI*11");   // jumps to 3, 2 was dropped

  // The broken sequence is discarded; nothing is emitted.
  TEST_ASSERT_EQUAL_INT(0, rte->waypoints_.get().size());
  TEST_ASSERT_EQUAL_STRING("", rte->route_id_.get().c_str());
}

void test_rte_total_mismatch_discards_partial(void) {
  parser->set("$GPRTE,2,1,c,0,PBRCPK,CPNPT*45");  // first of 2
  parser->set("$GPRTE,3,2,c,0,FATEA,OCEAI*10");   // claims a different total

  TEST_ASSERT_EQUAL_INT(0, rte->waypoints_.get().size());
  TEST_ASSERT_EQUAL_STRING("", rte->route_id_.get().c_str());
}

void test_rte_route_id_change_discards_partial(void) {
  parser->set("$GPRTE,2,1,c,0,PBRCPK,CPNPT*45");  // first of 2, route 0
  parser->set("$GPRTE,2,2,c,1,FATEA,OCEAI*10");   // route changed to 1

  TEST_ASSERT_EQUAL_INT(0, rte->waypoints_.get().size());
  TEST_ASSERT_EQUAL_STRING("", rte->route_id_.get().c_str());
}

void test_rte_timeout_discards_partial(void) {
  parser->set("$GPRTE,2,1,c,0,PBRCPK,CPNPT*45");  // first of 2
  fake_millis += RTESentenceParser::kDefaultSequenceTimeoutMs + 1;
  parser->set("$GPRTE,2,2,c,0,FATEA,OCEAI*11");   // arrives after the timeout

  TEST_ASSERT_EQUAL_INT(0, rte->waypoints_.get().size());
  TEST_ASSERT_EQUAL_STRING("", rte->route_id_.get().c_str());
}

void test_rte_recovers_after_interruption(void) {
  // A broken sequence must not block a subsequent valid route.
  parser->set("$GPRTE,2,1,c,0,PBRCPK,CPNPT*45");
  parser->set("$GPRTE,2,2,c,1,FATEA,OCEAI*10");  // route change breaks it

  parser->set("$GPRTE,1,1,c,ROUTE2,WPA,WPB*5F");

  TEST_ASSERT_EQUAL_STRING("ROUTE2", rte->route_id_.get().c_str());
  std::vector<String> wps = rte->waypoints_.get();
  TEST_ASSERT_EQUAL_INT(2, wps.size());
  TEST_ASSERT_EQUAL_STRING("WPA", wps[0].c_str());
  TEST_ASSERT_EQUAL_STRING("WPB", wps[1].c_str());
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_rte_single_sentence);
  RUN_TEST(test_rte_multi_sentence);
  RUN_TEST(test_rte_gap_discards_partial);
  RUN_TEST(test_rte_total_mismatch_discards_partial);
  RUN_TEST(test_rte_route_id_change_discards_partial);
  RUN_TEST(test_rte_timeout_discards_partial);
  RUN_TEST(test_rte_recovers_after_interruption);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_rte_single_sentence);
  RUN_TEST(test_rte_multi_sentence);
  RUN_TEST(test_rte_gap_discards_partial);
  RUN_TEST(test_rte_total_mismatch_discards_partial);
  RUN_TEST(test_rte_route_id_change_discards_partial);
  RUN_TEST(test_rte_timeout_discards_partial);
  RUN_TEST(test_rte_recovers_after_interruption);

  return UNITY_END();
}
#endif
