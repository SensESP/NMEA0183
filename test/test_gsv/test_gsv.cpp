#include <unity.h>

#include <vector>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static GSVSentenceParser* gsv;

// One full GSV cycle as emitted by a UM982 (new-format: a signal ID precedes
// the checksum). The leading GPS L1 group spans four sentences -- the case that
// tripped the cycle-end detection. Totals: GPS L1 13, GPS L2 7, GLONASS 9,
// BeiDou 10, Galileo 8 = 47 satellite blocks across four constellations.
static const char* kCycle[] = {
    "$GPGSV,4,1,13,15,24,205,43,17,45,073,39,32,06,331,25,19,48,128,40,1*62",
    "$GPGSV,4,2,13,14,19,073,35,20,12,182,27,01,16,028,30,12,26,238,40,1*65",
    "$GPGSV,4,3,13,22,37,074,38,13,,,41,10,18,303,35,23,08,269,29,1*5B",
    "$GPGSV,4,4,13,24,71,243,46,1*51",
    "$GPGSV,2,1,07,14,19,073,37,20,12,182,31,01,16,028,24,13,,,41,8*51",
    "$GPGSV,2,2,07,10,18,303,34,23,08,269,24,24,71,243,44,8*52",
    "$GLGSV,3,1,09,73,79,058,35,72,49,278,31,80,22,078,30,71,25,214,37,1*73",
    "$GLGSV,3,2,09,81,17,009,28,65,22,338,27,82,37,057,34,74,45,268,35,1*71",
    "$GLGSV,3,3,09,83,21,117,29,1*45",
    "$GBGSV,3,1,10,36,62,256,40,08,38,054,42,13,12,024,23,28,27,049,40,1*75",
    "$GBGSV,3,2,10,27,34,099,40,19,06,207,26,22,32,254,40,21,28,315,24,1*71",
    "$GBGSV,3,3,10,14,14,336,37,39,13,268,32,1*70",
    "$GAGSV,2,1,08,09,32,182,39,26,18,051,37,31,65,088,40,23,16,064,31,1*79",
    "$GAGSV,2,2,08,03,24,314,39,13,13,000,25,16,50,223,39,05,56,255,41,1*79",
};
static const int kCycleLen = sizeof(kCycle) / sizeof(kCycle[0]);
static const int kExpectedTotal = 47;

static int emit_count = 0;
static std::vector<GNSSSatellite> last_emitted;

static void feed_cycle() {
  for (int i = 0; i < kCycleLen; i++) {
    parser->set(kCycle[i]);
  }
}

void setUp(void) {
  parser = new NMEA0183Parser();
  gsv = new GSVSentenceParser(parser);
  emit_count = 0;
  last_emitted.clear();
  gsv->satellites_.attach([]() {
    emit_count++;
    last_emitted = gsv->satellites_.get();
  });
}

void tearDown(void) {
  delete gsv;
  delete parser;
}

// A multi-constellation cycle must produce exactly one satellitesInView emission
// per cycle, carrying every satellite from every constellation. The regression
// emitted once per sentence of the leading group, flooding the path with partial
// fragments.
void test_gsv_one_merged_emit_per_cycle(void) {
  feed_cycle();  // establishes the first sentence type; no emit yet
  feed_cycle();  // the leading sentence of this cycle emits the previous one

  int emits_before = emit_count;
  feed_cycle();  // exactly one emission, carrying the whole prior cycle

  TEST_ASSERT_EQUAL_INT(1, emit_count - emits_before);
  TEST_ASSERT_EQUAL_INT(kExpectedTotal, (int)last_emitted.size());

  bool gps = false, glonass = false, galileo = false, beidou = false;
  for (const auto& s : last_emitted) {
    gps |= s.system == GNSSSystem::gps;
    glonass |= s.system == GNSSSystem::glonass;
    galileo |= s.system == GNSSSystem::galileo;
    beidou |= s.system == GNSSSystem::beidou;
  }
  TEST_ASSERT_TRUE(gps && glonass && galileo && beidou);
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_gsv_one_merged_emit_per_cycle);
  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_gsv_one_merged_emit_per_cycle);
  return UNITY_END();
}
#endif
