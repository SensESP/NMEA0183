#include <unity.h>

#include <vector>

#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

static NMEA0183Parser* parser;
static GSVSentenceParser* gsv;
static int emit_count = 0;
static std::vector<GNSSSatellite> last_emitted;

// An intermittent Galileo signal (signal id 7) group -- a real receiver reports
// it only in some cycles as those satellites come and go.
static const char* kSig7Group[] = {
    "$GAGSV,3,1,09,09,16,182,17,26,21,036,28,31,50,080,41,33,17,088,28,7*75",
    "$GAGSV,3,2,09,03,35,305,31,16,66,225,37,05,49,231,44,25,19,243,28,7*74",
};
static const int kSig7Len = sizeof(kSig7Group) / sizeof(kSig7Group[0]);

// A full cycle WITHOUT that signal: GPS L1/L2, GLONASS, BeiDou, Galileo L1/L2 =
// 46 satellite blocks across four constellations.
static const char* kCycle[] = {
    "$GPGSV,3,1,11,15,06,199,28,17,34,056,42,32,21,322,38,19,55,098,38,1*63",
    "$GPGSV,3,2,11,25,10,255,27,01,10,012,21,12,46,244,37,22,20,077,21,1*62",
    "$GPGSV,3,3,11,13,,,44,10,08,287,17,24,66,185,46,1*6C",
    "$GPGSV,2,1,06,32,21,322,31,25,10,255,30,01,10,012,25,13,,,38,8*51",
    "$GPGSV,2,2,06,10,08,287,30,24,66,185,47,8*65",
    "$GLGSV,2,1,06,73,57,086,38,72,36,251,32,82,31,031,25,75,15,274,18,1*7A",
    "$GLGSV,2,2,06,74,67,286,37,83,39,101,36,1*70",
    "$GBGSV,3,1,10,36,78,235,45,08,34,050,40,30,27,147,39,28,19,035,23,1*7F",
    "$GBGSV,3,2,10,27,39,079,37,42,12,344,33,22,18,241,38,21,29,296,31,1*77",
    "$GBGSV,3,3,10,14,22,322,30,39,29,274,38,1*79",
    "$GAGSV,2,1,08,09,16,182,26,26,21,036,34,31,50,080,41,33,17,088,33,1*76",
    "$GAGSV,2,2,08,03,35,305,36,16,66,225,42,05,49,231,39,25,19,243,37,1*73",
    "$GAGSV,2,1,05,31,50,080,24,03,35,305,20,16,66,225,29,05,49,231,26,2*7D",
    "$GAGSV,2,2,05,25,19,243,21,2*4A",
};
static const int kCycleLen = sizeof(kCycle) / sizeof(kCycle[0]);
static const int kCycleTotal = 46;

static void feed(const char** sentences, int n) {
  for (int i = 0; i < n; i++) {
    parser->set(sentences[i]);
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

// Reproduces the field failure: GSV output starts mid-cycle on an intermittent
// signal that then drops out. Anchoring the cycle on the first sentence ever
// seen latches onto that signal and never emits again -- the satellite paths
// freeze. Cycle detection must key on a group repeating, so emission stays
// steady regardless of where output started or which signals are present.
void test_gsv_recovers_from_intermittent_lead(void) {
  feed(kSig7Group, kSig7Len);  // "boot" mid-cycle on the intermittent signal
  for (int i = 0; i < 3; i++) {
    feed(kCycle, kCycleLen);  // then cycles that no longer contain it
  }

  TEST_ASSERT_GREATER_OR_EQUAL_INT(2, emit_count);  // not frozen
  TEST_ASSERT_EQUAL_INT(kCycleTotal, (int)last_emitted.size());

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
  RUN_TEST(test_gsv_recovers_from_intermittent_lead);
  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(test_gsv_recovers_from_intermittent_lead);
  return UNITY_END();
}
#endif
