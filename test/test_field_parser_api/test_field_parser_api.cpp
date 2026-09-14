#include <unity.h>

#include <ctime>

#include "sensesp_nmea0183/sentence_parser/field_parsers.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

// These tests call the field parsers the way a downstream consumer does: with
// plain int out-params, and with struct tm members, whose type is int and
// cannot be changed at the call site. Version 3.2.0 declared the out-params
// int32_t, which is long int on the ESP32 toolchains, and consumers stopped
// compiling.

void setUp(void) {}
void tearDown(void) {}

void test_parse_int_into_int() {
  int value = 0;

  TEST_ASSERT_TRUE(ParseInt(&value, "42"));
  TEST_ASSERT_EQUAL_INT(42, value);
}

void test_parse_int_empty_field() {
  int value = 0;

  TEST_ASSERT_TRUE(ParseInt(&value, "", true));
  TEST_ASSERT_EQUAL_INT(Nullable<int>::invalid(), value);

  TEST_ASSERT_FALSE(ParseInt(&value, ""));
}

void test_parse_int_garbage() {
  int value = 0;

  TEST_ASSERT_FALSE(ParseInt(&value, "abc"));
}

void test_parse_time_into_struct_tm() {
  struct tm t = {};
  float second = 0.0f;

  TEST_ASSERT_TRUE(ParseTime(&t.tm_hour, &t.tm_min, &second, "123456.78"));
  TEST_ASSERT_EQUAL_INT(12, t.tm_hour);
  TEST_ASSERT_EQUAL_INT(34, t.tm_min);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 56.78f, second);
}

void test_parse_date_into_struct_tm() {
  struct tm t = {};

  TEST_ASSERT_TRUE(ParseDate(&t.tm_year, &t.tm_mon, &t.tm_mday, "230925"));
  TEST_ASSERT_EQUAL_INT(23, t.tm_mday);
  TEST_ASSERT_EQUAL_INT(8, t.tm_mon);
  TEST_ASSERT_EQUAL_INT(125, t.tm_year);
}

#ifdef ARDUINO
void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_parse_int_into_int);
  RUN_TEST(test_parse_int_empty_field);
  RUN_TEST(test_parse_int_garbage);
  RUN_TEST(test_parse_time_into_struct_tm);
  RUN_TEST(test_parse_date_into_struct_tm);

  UNITY_END();
}

void loop() {}
#else
int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_parse_int_into_int);
  RUN_TEST(test_parse_int_empty_field);
  RUN_TEST(test_parse_int_garbage);
  RUN_TEST(test_parse_time_into_struct_tm);
  RUN_TEST(test_parse_date_into_struct_tm);

  return UNITY_END();
}
#endif
