# Nullable Migration and Field Parser Standardization

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace sentinel constants with `Nullable<T>` in all NMEA 0183 sentence parsers and standardize the `parse_fields()` pattern.

**Architecture:** Field parsers write `Nullable<T>::invalid()` on empty fields instead of `kInvalidFloat`/`kInvalidDouble`/`kInvalidInt`. Concrete parsers declare `Nullable<T>` locals, pass `.ptr()` to field parsers, and check `is_valid()` before emitting. Public API (`ObservableValue<T>`) unchanged.

**Tech Stack:** C++ (ESP32 Arduino), SensESP framework (`Nullable<T>` from `sensesp/types/nullable.h`), PlatformIO build system, Unity test framework.

**Spec:** `docs/superpowers/specs/2026-03-23-nullable-migration-design.md`

---

### Task 1: Update field parser signatures and invalid values

**Files:**
- Modify: `src/sensesp_nmea0183/sentence_parser/field_parsers.h`
- Modify: `src/sensesp_nmea0183/sentence_parser/field_parsers.cpp`

- [ ] **Step 1: Update field_parsers.h — remove sentinel constants, change int signatures to int32_t**

Replace the sentinel constants and update `ParseInt`/`ParseTime`/`ParseDate` signatures:

```cpp
// field_parsers.h — replace lines 4-13 with:
#include <Arduino.h>
#include <cstdint>
#include "sensesp/types/nullable.h"

namespace sensesp::nmea0183 {

using sensesp::Nullable;
```

Remove these three lines entirely:
```cpp
constexpr float kInvalidFloat = std::numeric_limits<float>::lowest();
constexpr double kInvalidDouble = std::numeric_limits<double>::lowest();
constexpr int kInvalidInt = std::numeric_limits<int>::lowest();
```

Change these signatures from `int*` to `int32_t*`:
```cpp
bool ParseInt(int32_t* value, const char* s, bool allow_empty = false);

bool ParseTime(int32_t* hour, int32_t* minute, float* second, const char* s,
               bool allow_empty = false);

bool ParseDate(int32_t* year, int32_t* month, int32_t* day, const char* s,
               bool allow_empty = false);
```

- [ ] **Step 2: Update field_parsers.cpp — write Nullable invalid values on empty fields**

In `ParseInt` (line 21-28): change `kInvalidInt` to `Nullable<int32_t>::invalid()` and `int*` to `int32_t*`:
```cpp
bool ParseInt(int32_t* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    *value = Nullable<int32_t>::invalid();
    return allow_empty;
  }
  int32_t retval = sscanf(s, "%d", value);
  return retval == 1;
}
```

In `ParseFloat` (line 30-37): change `kInvalidFloat` to `Nullable<float>::invalid()`:
```cpp
  if (s[0] == 0) {
    *value = Nullable<float>::invalid();
    return allow_empty;
  }
```

In `ParseDouble` (line 39-46): change `kInvalidDouble` to `Nullable<double>::invalid()`:
```cpp
  if (s[0] == 0) {
    *value = Nullable<double>::invalid();
    return allow_empty;
  }
```

In `ParseLatLon` (line 48-63): change `kInvalidDouble` to `Nullable<double>::invalid()`:
```cpp
  if (s[0] == 0) {
    *value = Nullable<double>::invalid();
    return allow_empty;
  }
```

In `ParseNS` (line 65-80): add guard against negating the invalid sentinel:
```cpp
bool ParseNS(double* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
  if (*value == Nullable<double>::invalid()) return true;  // no-op on invalid

  switch (*s) {
    case 'N':
      break;
    case 'S':
      *value *= -1;
      break;
    default:
      return false;
  }
  return true;
}
```

In `ParseEW(double*)` (line 82-96): add the same guard:
```cpp
bool ParseEW(double* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
  if (*value == Nullable<double>::invalid()) return true;  // no-op on invalid

  switch (*s) {
    case 'E':
      break;
    case 'W':
      *value *= -1;
      break;
    default:
      return false;
  }
  return true;
}
```

In `ParseEW(float*)` (line 98-112): add the same guard:
```cpp
bool ParseEW(float* value, const char* s, bool allow_empty) {
  if (s[0] == 0) {
    return allow_empty;
  }
  if (*value == Nullable<float>::invalid()) return true;  // no-op on invalid

  switch (*s) {
    case 'E':
      break;
    case 'W':
      *value *= -1;
      break;
    default:
      return false;
  }
  return true;
}
```

In `ParseTime` (line 145-155): change to `int32_t*` and use Nullable invalid values:
```cpp
bool ParseTime(int32_t* hour, int32_t* minute, float* second, const char* s,
               bool allow_empty) {
  if (s[0] == 0) {
    *hour = Nullable<int32_t>::invalid();
    *minute = Nullable<int32_t>::invalid();
    *second = Nullable<float>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%2d%2d%f", hour, minute, second);
  return retval == 3;
}
```

Note: `sscanf` with `%2d` writes to `int32_t*` safely on ESP32 where `int` == `int32_t`.

In `ParseDate` (line 157-169): change to `int32_t*` and use Nullable invalid values:
```cpp
bool ParseDate(int32_t* year, int32_t* month, int32_t* day, const char* s,
               bool allow_empty) {
  if (s[0] == 0) {
    *year = Nullable<int32_t>::invalid();
    *month = Nullable<int32_t>::invalid();
    *day = Nullable<int32_t>::invalid();
    return allow_empty;
  }
  int retval = sscanf(s, "%2d%2d%2d", day, month, year);
  // date expressed as C struct tm
  *year += 100;
  *month -= 1;
  return retval == 3;
}
```

- [ ] **Step 3: Build to verify field parser changes compile**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32`
Expected: Build succeeds (parsers still use old sentinel comparisons which will now compare against different values, but this is intermediate — all parsers will be updated in subsequent tasks).

- [ ] **Step 4: Commit**

```bash
git add src/sensesp_nmea0183/sentence_parser/field_parsers.h src/sensesp_nmea0183/sentence_parser/field_parsers.cpp
git commit -m "refactor(field-parsers): replace sentinel constants with Nullable<T>::invalid()

Change ParseInt/ParseTime/ParseDate signatures from int* to int32_t*
for Nullable<int32_t> portability. Add sentinel negation guards to
ParseNS and ParseEW. Remove kInvalidFloat/kInvalidDouble/kInvalidInt.

Closes #31 (partial)"
```

---

### Task 2: Migrate navigation sentence parsers

**Files:**
- Modify: `src/sensesp_nmea0183/sentence_parser/navigation_sentence_parser.cpp`

- [ ] **Step 1: Migrate HDGSentenceParser (lines 7-55)**

Replace raw `float` locals with `Nullable<float>`, use `.ptr()`, replace `kInvalidFloat` checks with `is_valid()`:

```cpp
bool HDGSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> heading;
  Nullable<float> deviation;
  Nullable<float> variation;

  if (num_fields < 6) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, heading.ptr()),
      FLDP_OPT(Float, deviation.ptr()),
      FLDP_OPT(EW, deviation.ptr()),
      FLDP_OPT(Float, variation.ptr()),
      FLDP_OPT(EW, variation.ptr()),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (heading.is_valid()) {
    magnetic_heading_.set(heading * DEG_TO_RAD);
  }
  if (deviation.is_valid()) {
    deviation_.set(deviation * DEG_TO_RAD);
  }
  if (variation.is_valid()) {
    variation_.set(variation * DEG_TO_RAD);
  }

  return true;
}
```

- [ ] **Step 2: Migrate VHWSentenceParser (lines 57-119)**

```cpp
bool VHWSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> true_heading;
  Nullable<float> magnetic_heading;
  Nullable<float> speed_knots;
  Nullable<float> speed_kmh;
  char t_char;
  char m_char;
  char n_char;
  char k_char;

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, true_heading.ptr()),
      FLDP_OPT(Char, &t_char, 'T'),
      FLDP_OPT(Float, magnetic_heading.ptr()),
      FLDP_OPT(Char, &m_char, 'M'),
      FLDP_OPT(Float, speed_knots.ptr()),
      FLDP_OPT(Char, &n_char, 'N'),
      FLDP_OPT(Float, speed_kmh.ptr()),
      FLDP_OPT(Char, &k_char, 'K'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (true_heading.is_valid()) {
    true_heading_.set(true_heading * DEG_TO_RAD);
  }
  if (magnetic_heading.is_valid()) {
    magnetic_heading_.set(magnetic_heading * DEG_TO_RAD);
  }
  if (speed_knots.is_valid()) {
    water_speed_.set(speed_knots * 1852.0 / 3600.0);
  } else if (speed_kmh.is_valid()) {
    water_speed_.set(speed_kmh * 1000.0 / 3600.0);
  }

  return true;
}
```

- [ ] **Step 3: Migrate DPTSentenceParser (lines 121-155)**

```cpp
bool DPTSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> depth;
  Nullable<float> offset;

  if (num_fields < 2) {
    return false;
  }

  bool ok = true;
  ok &= FLDP_OPT(Float, depth.ptr())(field_strings + field_offsets[1]);
  if (num_fields >= 3) {
    ok &= FLDP_OPT(Float, offset.ptr())(field_strings + field_offsets[2]);
  }

  if (!ok) {
    return false;
  }

  if (depth.is_valid()) {
    depth_.set(depth);
  }
  if (offset.is_valid()) {
    offset_.set(offset);
  }

  return true;
}
```

- [ ] **Step 4: Migrate DBTSentenceParser (lines 157-209)**

```cpp
bool DBTSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> depth_feet;
  Nullable<float> depth_meters;
  Nullable<float> depth_fathoms;
  char f_char;
  char m_char;
  char F_char;

  if (num_fields < 7) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, depth_feet.ptr()),
      FLDP_OPT(Char, &f_char, 'f'),
      FLDP_OPT(Float, depth_meters.ptr()),
      FLDP_OPT(Char, &m_char, 'M'),
      FLDP_OPT(Float, depth_fathoms.ptr()),
      FLDP_OPT(Char, &F_char, 'F'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (depth_meters.is_valid()) {
    depth_.set(depth_meters);
  } else if (depth_feet.is_valid()) {
    depth_.set(depth_feet * 0.3048);
  } else if (depth_fathoms.is_valid()) {
    depth_.set(depth_fathoms * 1.8288);
  }

  return true;
}
```

- [ ] **Step 5: Migrate MTWSentenceParser (lines 211-244)**

No `kInvalidFloat` checks — all fields are required. Only change: add Nullable include at top of file. No other changes needed for this parser.

Actually, `temperature` is required (FLDP not FLDP_OPT), so no Nullable needed here. Leave as-is but the `float temperature` local is fine since `FLDP` will fail the parse on empty fields.

- [ ] **Step 6: Migrate HDMSentenceParser (lines 246-281)**

```cpp
bool HDMSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> heading;
  char m_char;

  if (num_fields < 3) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, heading.ptr()),
      FLDP_OPT(Char, &m_char, 'M'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (heading.is_valid()) {
    magnetic_heading_.set(heading * DEG_TO_RAD);
  }

  return true;
}
```

- [ ] **Step 7: Migrate HDTSentenceParser (lines 283-318)**

Same pattern as HDM:

```cpp
bool HDTSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> heading;
  char t_char;

  if (num_fields < 3) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, heading.ptr()),
      FLDP_OPT(Char, &t_char, 'T'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (heading.is_valid()) {
    true_heading_.set(heading * DEG_TO_RAD);
  }

  return true;
}
```

- [ ] **Step 8: Add Nullable include to navigation_sentence_parser.cpp**

Add at the top of the file, after existing includes:
```cpp
#include "sensesp/types/nullable.h"
using sensesp::Nullable;
```

- [ ] **Step 9: Build to verify**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32`
Expected: Build succeeds.

- [ ] **Step 10: Run existing tests**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio test -e pioarduino_esp32 --without-uploading`
Expected: All tests pass.

- [ ] **Step 11: Commit**

```bash
git add src/sensesp_nmea0183/sentence_parser/navigation_sentence_parser.cpp
git commit -m "refactor(navigation): migrate to Nullable<T> and standardize parse_fields

Migrate HDG, VHW, DPT, DBT, HDM, HDT parsers. MTW unchanged (all
fields required). Replace kInvalidFloat checks with is_valid().

Closes #32 (partial)"
```

---

### Task 3: Migrate wind sentence parsers

**Files:**
- Modify: `src/sensesp_nmea0183/sentence_parser/wind_sentence_parser.cpp`

- [ ] **Step 1: Migrate MWVSentenceParser (lines 7-56)**

MWV uses `FLDP_OPT` for float fields but does not check validity before emitting — it always calls `set()`. This is a behavior change: with Nullable, we should guard the `set()` calls. However, `ConvertSpeedToMs` requires a valid speed value, so if `wind_speed` is invalid, we'd crash in the conversion. The existing code works because the `char` checks (`FLDP_OPT(Char, &r_value, 'R')`) fail if the reference indicator is wrong, causing early return.

Keep the existing unconditional `set()` pattern for MWV/TrueWindMWV since these parsers validate via the char fields (R/T, A) and `ConvertSpeedToMs`. If any field is empty, the char parse or speed conversion will fail.

However, there is a pre-existing bug: if wind angle/speed fields are empty but char fields (R/T, unit, A) are present, the empty float locals receive the invalid sentinel, and `ConvertSpeedToMs` operates on it, emitting a bogus value. Fix by adding Nullable guards:

```cpp
bool MWVSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> wind_speed;
  Nullable<float> wind_angle;
  char r_value;
  char units;
  char a_value;

  if (num_fields < 6) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, wind_angle.ptr()),
      FLDP_OPT(Char, &r_value, 'R'),
      FLDP_OPT(Float, wind_speed.ptr()),
      FLDP_OPT(Char, &units, 255),
      FLDP_OPT(Char, &a_value, 'A')};

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (wind_speed.is_valid()) {
    float speed = wind_speed;
    if (!ConvertSpeedToMs(&speed, units)) {
      return false;
    }
    apparent_wind_speed_.set(speed);
  }
  if (wind_angle.is_valid()) {
    apparent_wind_angle_.set(wind_angle * DEG_TO_RAD);
  }

  return true;
}
```

Apply the same pattern to TrueWindMWVSentenceParser (identical structure, different observables: `true_wind_speed_`, `true_wind_direction_`).

- [ ] **Step 2: Migrate MWDSentenceParser (lines 108-167)**

```cpp
bool MWDSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> true_direction;
  Nullable<float> magnetic_direction;
  Nullable<float> speed_knots;
  Nullable<float> speed_ms;
  char t_char;
  char m_char;
  char n_char;
  char ms_char;

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, true_direction.ptr()),
      FLDP_OPT(Char, &t_char, 'T'),
      FLDP_OPT(Float, magnetic_direction.ptr()),
      FLDP_OPT(Char, &m_char, 'M'),
      FLDP_OPT(Float, speed_knots.ptr()),
      FLDP_OPT(Char, &n_char, 'N'),
      FLDP_OPT(Float, speed_ms.ptr()),
      FLDP_OPT(Char, &ms_char, 'M'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (true_direction.is_valid()) {
    true_wind_direction_.set(true_direction * DEG_TO_RAD);
  }
  if (speed_ms.is_valid()) {
    true_wind_speed_.set(speed_ms);
  } else if (speed_knots.is_valid()) {
    true_wind_speed_.set(speed_knots * 1852.0 / 3600.0);
  }

  return true;
}
```

- [ ] **Step 3: Migrate VWRSentenceParser (lines 169-235)**

```cpp
bool VWRSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> angle;
  char lr_char;
  Nullable<float> speed_knots;
  Nullable<float> speed_ms;
  Nullable<float> speed_kmh;
  char n_char;
  char ms_char;
  char k_char;

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, angle.ptr()),
      FLDP_OPT(Char, &lr_char, 255),
      FLDP_OPT(Float, speed_knots.ptr()),
      FLDP_OPT(Char, &n_char, 'N'),
      FLDP_OPT(Float, speed_ms.ptr()),
      FLDP_OPT(Char, &ms_char, 'M'),
      FLDP_OPT(Float, speed_kmh.ptr()),
      FLDP_OPT(Char, &k_char, 'K'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (angle.is_valid()) {
    float signed_angle = angle * DEG_TO_RAD;
    if (lr_char == 'L') {
      signed_angle = -signed_angle;
    }
    apparent_wind_angle_.set(signed_angle);
  }

  if (speed_ms.is_valid()) {
    apparent_wind_speed_.set(speed_ms);
  } else if (speed_knots.is_valid()) {
    apparent_wind_speed_.set(speed_knots * 1852.0 / 3600.0);
  } else if (speed_kmh.is_valid()) {
    apparent_wind_speed_.set(speed_kmh * 1000.0 / 3600.0);
  }

  return true;
}
```

- [ ] **Step 4: Add Nullable include to wind_sentence_parser.cpp**

Add after existing includes:
```cpp
#include "sensesp/types/nullable.h"
using sensesp::Nullable;
```

- [ ] **Step 5: Build and test**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32 && pio test -e pioarduino_esp32 --without-uploading`
Expected: Build succeeds, all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/sensesp_nmea0183/sentence_parser/wind_sentence_parser.cpp
git commit -m "refactor(wind): migrate MWD and VWR to Nullable<T>

MWV and TrueWindMWV: add Nullable guards for wind angle/speed
to fix pre-existing bug where empty float fields with present char
fields would emit bogus values through ConvertSpeedToMs."
```

---

### Task 4: Migrate weather sentence parser

**Files:**
- Modify: `src/sensesp_nmea0183/sentence_parser/weather_sentence_parser.cpp`

- [ ] **Step 1: Migrate MDASentenceParser**

Replace all `float` locals that use `kInvalidFloat` checks with `Nullable<float>`:

```cpp
bool MDASentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> pressure_inhg;
  Nullable<float> pressure_bars;
  Nullable<float> air_temp;
  Nullable<float> water_temp;
  Nullable<float> humidity;
  Nullable<float> dew_point;
  Nullable<float> wind_dir_true;
  Nullable<float> wind_dir_mag;
  Nullable<float> abs_humidity;
  Nullable<float> wind_speed_kn;
  Nullable<float> wind_speed_ms;
  char i_char;
  char b_char;
  char c1_char;
  char c2_char;
  char c3_char;
  char t_char;
  char m_char;
  char n_char;
  char ms_char;

  if (num_fields < 21) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, pressure_inhg.ptr()),
      FLDP_OPT(Char, &i_char, 'I'),
      FLDP_OPT(Float, pressure_bars.ptr()),
      FLDP_OPT(Char, &b_char, 'B'),
      FLDP_OPT(Float, air_temp.ptr()),
      FLDP_OPT(Char, &c1_char, 'C'),
      FLDP_OPT(Float, water_temp.ptr()),
      FLDP_OPT(Char, &c2_char, 'C'),
      FLDP_OPT(Float, humidity.ptr()),
      FLDP_OPT(Float, abs_humidity.ptr()),
      FLDP_OPT(Float, dew_point.ptr()),
      FLDP_OPT(Char, &c3_char, 'C'),
      FLDP_OPT(Float, wind_dir_true.ptr()),
      FLDP_OPT(Char, &t_char, 'T'),
      FLDP_OPT(Float, wind_dir_mag.ptr()),
      FLDP_OPT(Char, &m_char, 'M'),
      FLDP_OPT(Float, wind_speed_kn.ptr()),
      FLDP_OPT(Char, &n_char, 'N'),
      FLDP_OPT(Float, wind_speed_ms.ptr()),
      FLDP_OPT(Char, &ms_char, 'M'),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (pressure_bars.is_valid()) {
    barometric_pressure_.set(pressure_bars * 100000.0);
  } else if (pressure_inhg.is_valid()) {
    barometric_pressure_.set(pressure_inhg * 3386.389);
  }

  if (air_temp.is_valid()) {
    air_temperature_.set(air_temp + 273.15);
  }
  if (water_temp.is_valid()) {
    water_temperature_.set(water_temp + 273.15);
  }
  if (humidity.is_valid()) {
    relative_humidity_.set(humidity / 100.0);
  }
  if (dew_point.is_valid()) {
    dew_point_.set(dew_point + 273.15);
  }
  if (wind_dir_true.is_valid()) {
    true_wind_direction_.set(wind_dir_true * DEG_TO_RAD);
  }
  if (wind_speed_ms.is_valid()) {
    true_wind_speed_.set(wind_speed_ms);
  } else if (wind_speed_kn.is_valid()) {
    true_wind_speed_.set(wind_speed_kn * 1852.0 / 3600.0);
  }

  return true;
}
```

- [ ] **Step 2: Add Nullable include**

```cpp
#include "sensesp/types/nullable.h"
using sensesp::Nullable;
```

- [ ] **Step 3: Build and test**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32 && pio test -e pioarduino_esp32 --without-uploading`

- [ ] **Step 4: Commit**

```bash
git add src/sensesp_nmea0183/sentence_parser/weather_sentence_parser.cpp
git commit -m "refactor(weather): migrate MDA parser to Nullable<T>"
```

---

### Task 5: Migrate GNSS sentence parsers

**Files:**
- Modify: `src/sensesp_nmea0183/sentence_parser/gnss_sentence_parser.cpp`

This is the largest file. Some parsers (SkyTraqPSTI030, SkyTraqPSTI032, QuectelPQTMTAR) use only required fields (FLDP) and don't check sentinels — they don't need Nullable. Others need careful migration.

- [ ] **Step 1: Migrate GGASentenceParser (lines 57-154)**

Key changes: parse lat/lon into separate Nullable locals instead of directly into Position struct. Required fields (FLDP) stay as raw `int32_t` — only optional fields (FLDP_OPT) use Nullable:

```cpp
bool GGASentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  int32_t hour, minute;
  float second;
  Nullable<double> latitude, longitude;
  Nullable<float> altitude;
  int32_t quality;
  int32_t num_satellites;
  Nullable<float> horizontal_dilution;
  Nullable<float> geoidal_separation;
  Nullable<float> dgps_age;
  Nullable<int32_t> dgps_id;
  char antenna_height_unit;
  char geoidal_separation_unit;

  if (num_fields < 15) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP(Time, &hour, &minute, &second),
      FLDP_OPT(LatLon, latitude.ptr()),
      FLDP_OPT(NS, latitude.ptr()),
      FLDP_OPT(LatLon, longitude.ptr()),
      FLDP_OPT(EW, longitude.ptr()),
      FLDP(Int, &quality),
      FLDP(Int, &num_satellites),
      FLDP_OPT(Float, horizontal_dilution.ptr()),
      FLDP_OPT(Float, altitude.ptr()),
      FLDP_OPT(Char, &antenna_height_unit, 'M'),
      FLDP_OPT(Float, geoidal_separation.ptr()),
      FLDP_OPT(Char, &geoidal_separation_unit, 'M'),
      FLDP_OPT(Float, dgps_age.ptr()),
      FLDP_OPT(Int, dgps_id.ptr())};

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (latitude.is_valid() && longitude.is_valid()) {
    Position position{latitude, longitude,
                      altitude.is_valid() ? (double)altitude
                                          : kPositionInvalidAltitude};
    position_.set(position);
  }
  gnss_quality_.set(gnss_quality_strings[quality]);
  quality_.set(quality);
  num_satellites_.set(num_satellites);

  if (quality != 0) {
    if (horizontal_dilution.is_valid()) {
      horizontal_dilution_.set(horizontal_dilution);
    }
    if (geoidal_separation.is_valid()) {
      geoidal_separation_.set(geoidal_separation);
    }
    if (dgps_age.is_valid()) {
      dgps_age_.set(dgps_age);
    }
    if (dgps_id.is_valid()) {
      dgps_id_.set(dgps_id);
    }
  }

  return true;
}
```

- [ ] **Step 2: Migrate GLLSentenceParser (lines 156-202)**

```cpp
bool GLLSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<double> latitude, longitude;

  if (num_fields < 5) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(LatLon, latitude.ptr()),
      FLDP_OPT(NS, latitude.ptr()),
      FLDP_OPT(LatLon, longitude.ptr()),
      FLDP_OPT(EW, longitude.ptr())
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (latitude.is_valid() && longitude.is_valid()) {
    Position position{latitude, longitude, kPositionInvalidAltitude};
    position_.set(position);
  }

  return true;
}
```

- [ ] **Step 3: Migrate RMCSentenceParser (lines 204-289)**

RMC uses `struct tm` with `int` fields. Required time/date fields stay as raw `int32_t`. Optional position/speed/course/variation use Nullable:

```cpp
bool RMCSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  int32_t hour, minute;
  float second;
  bool is_valid = false;
  Nullable<double> latitude, longitude;
  Nullable<float> speed;
  Nullable<float> true_course;
  Nullable<float> variation;
  int32_t year, month, day;

  if (num_fields < 12) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP(Time, &hour, &minute, &second),
      FLDP(AV, &is_valid),
      FLDP_OPT(LatLon, latitude.ptr()),
      FLDP_OPT(NS, latitude.ptr()),
      FLDP_OPT(LatLon, longitude.ptr()),
      FLDP_OPT(EW, longitude.ptr()),
      FLDP_OPT(Float, speed.ptr()),
      FLDP_OPT(Float, true_course.ptr()),
      FLDP(Date, &year, &month, &day),
      FLDP_OPT(Float, variation.ptr()),
      FLDP_OPT(EW, variation.ptr())
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (is_valid) {
    if (latitude.is_valid() && longitude.is_valid()) {
      Position position{latitude, longitude, kPositionInvalidAltitude};
      position_.set(position);
    }
    struct tm time = {};
    time.tm_year = year;
    time.tm_mon = month;
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min = minute;
    time.tm_sec = (int)second;
    time.tm_isdst = 0;
    datetime_.set(mktime(&time));

    if (speed.is_valid()) {
      speed_.set(1852. * speed / 3600.);
    }
    if (true_course.is_valid()) {
      true_course_.set(2 * PI * true_course / 360.);
    }
    if (variation.is_valid()) {
      variation_.set(2 * PI * variation / 360.);
    }
  }

  return true;
}
```

- [ ] **Step 4: Migrate VTGSentenceParser (lines 291-347)**

```cpp
bool VTGSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  Nullable<float> true_track;
  Nullable<float> magnetic_track;
  Nullable<float> ground_speed;
  char true_track_symbol;
  char magnetic_track_symbol;
  char ground_speed_knots_unit;

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, true_track.ptr()),
      FLDP_OPT(Char, &true_track_symbol, 'T'),
      FLDP_OPT(Float, magnetic_track.ptr()),
      FLDP_OPT(Char, &magnetic_track_symbol, 'M'),
      FLDP_OPT(Float, ground_speed.ptr()),
      FLDP_OPT(Char, &ground_speed_knots_unit, 'N')
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (true_track.is_valid()) {
    true_course_.set(2 * PI * true_track / 360.);
  }
  if (ground_speed.is_valid()) {
    speed_.set(1852. * ground_speed / 3600.);
  }

  return true;
}
```

- [ ] **Step 5: Migrate GSVSentenceParser (lines 349-545)**

GSV already uses `Nullable` for elevation/azimuth via `.ptr()`. Changes needed:
- `int num_sentences`, `int sentence_number`, `int num_satellites` → `int32_t` (since ParseInt now takes `int32_t*`)
- Static locals also change to `int32_t`
- No other Nullable changes needed since the satellite fields already use Nullable

Update the local variable declarations:
```cpp
  static int32_t num_sentences = 0;
  int32_t sentence_number = 0;
  // ...
  int32_t num_satellites = 0;
```

And the satellite ID field:
```cpp
  FLDP_OPT(Int, &sentence_satellites[j].id),
```
Also change `GNSSSatellite.id` and `GNSSSatellite.snr` from `int` to `int32_t` in `src/sensesp_nmea0183/data/gnss_data.h` to match the `ParseInt` signature change.

- [ ] **Step 6: Migrate GSASentenceParser (lines 805-850)**

```cpp
bool GSASentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  char mode;
  int32_t fix_type;
  Nullable<float> pdop;
  Nullable<float> hdop;
  Nullable<float> vdop;

  if (num_fields < 18) {
    return false;
  }

  bool ok = true;
  ok &= FLDP_OPT(Char, &mode, 255)(field_strings + field_offsets[1]);
  ok &= FLDP(Int, &fix_type)(field_strings + field_offsets[2]);
  ok &= FLDP_OPT(Float, pdop.ptr())(field_strings + field_offsets[15]);
  ok &= FLDP_OPT(Float, hdop.ptr())(field_strings + field_offsets[16]);
  ok &= FLDP_OPT(Float, vdop.ptr())(field_strings + field_offsets[17]);

  if (!ok) {
    return false;
  }

  fix_type_.set(fix_type);
  if (pdop.is_valid()) {
    pdop_.set(pdop);
  }
  if (hdop.is_valid()) {
    hdop_.set(hdop);
  }
  if (vdop.is_valid()) {
    vdop_.set(vdop);
  }

  return true;
}
```

- [ ] **Step 7: Migrate ZDASentenceParser (lines 852-893)**

ZDA uses only required fields (FLDP), so the int locals just need `int32_t`:

```cpp
bool ZDASentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  int32_t hour, minute;
  float second;
  int32_t day, month, year;

  if (num_fields < 5) {
    return false;
  }

  bool ok = true;
  ok &= FLDP(Time, &hour, &minute, &second)(
      field_strings + field_offsets[1]);
  ok &= FLDP(Int, &day)(field_strings + field_offsets[2]);
  ok &= FLDP(Int, &month)(field_strings + field_offsets[3]);
  ok &= FLDP(Int, &year)(field_strings + field_offsets[4]);

  if (!ok) {
    return false;
  }

  struct tm time = {};
  time.tm_hour = hour;
  time.tm_min = minute;
  time.tm_sec = (int)second;
  time.tm_mday = day;
  time.tm_mon = month - 1;
  time.tm_year = year - 1900;
  time.tm_isdst = 0;

  datetime_.set(mktime(&time));

  return true;
}
```

- [ ] **Step 8: Migrate GBSSentenceParser (lines 895-935)**

```cpp
bool GBSSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  int32_t hour, minute;
  float second;
  Nullable<float> lat_error;
  Nullable<float> lon_error;
  Nullable<float> alt_error;

  if (num_fields < 5) {
    return false;
  }

  bool ok = true;
  ok &= FLDP(Time, &hour, &minute, &second)(
      field_strings + field_offsets[1]);
  ok &= FLDP_OPT(Float, lat_error.ptr())(field_strings + field_offsets[2]);
  ok &= FLDP_OPT(Float, lon_error.ptr())(field_strings + field_offsets[3]);
  ok &= FLDP_OPT(Float, alt_error.ptr())(field_strings + field_offsets[4]);

  if (!ok) {
    return false;
  }

  if (lat_error.is_valid()) {
    lat_error_.set(lat_error);
  }
  if (lon_error.is_valid()) {
    lon_error_.set(lon_error);
  }
  if (alt_error.is_valid()) {
    alt_error_.set(alt_error);
  }

  return true;
}
```

- [ ] **Step 9: Update SkyTraq/Quectel parsers for int32_t**

These parsers use only required fields (FLDP) and don't check sentinels. Only change needed: `int` locals passed to `ParseInt`/`ParseTime`/`ParseDate` become `int32_t`, and `struct tm` is populated by assignment.

For **SkyTraqPSTI030** (line 547): replace `struct tm time;` pattern with `int32_t` locals:
```cpp
  int32_t hour, minute;
  float second;
  // ... other locals unchanged ...
  int32_t year, month, day;

  // In the fps array, change:
  FLDP(Time, &hour, &minute, &second),
  // ... later:
  FLDP(Date, &year, &month, &day),

  // After parsing, construct tm:
  struct tm time = {};
  time.tm_hour = hour;
  time.tm_min = minute;
  time.tm_sec = (int)second;
  time.tm_year = year;  // ParseDate already adds 100
  time.tm_mon = month;  // ParseDate already subtracts 1
  time.tm_mday = day;
  time.tm_isdst = 0;
```

For **SkyTraqPSTI032** (line 647): same pattern — `int32_t` locals for time/date fields, copy to `tm` after parsing.

For **QuectelPQTMTAR** (line 731): change `int heading_status` → `int32_t heading_status`, `int hdg_num_satellites` → `int32_t hdg_num_satellites`, and time fields to `int32_t` locals as above.

- [ ] **Step 10: Add Nullable include to gnss_sentence_parser.cpp**

Add after existing includes:
```cpp
#include "sensesp/types/nullable.h"
using sensesp::Nullable;
```

- [ ] **Step 11: Build and test**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32 && pio test -e pioarduino_esp32 --without-uploading`
Expected: Build succeeds, all tests pass.

- [ ] **Step 12: Commit**

```bash
git add src/sensesp_nmea0183/sentence_parser/gnss_sentence_parser.cpp
git commit -m "refactor(gnss): migrate to Nullable<T> and int32_t

Migrate GGA, GLL, RMC, VTG, GSA, GBS parsers to Nullable<T>.
Update GSV, ZDA, SkyTraq, Quectel for int32_t compatibility.
Position structs constructed from validated Nullable locals."
```

---

### Task 6: Migrate waypoint sentence parsers

**Files:**
- Modify: `src/sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.cpp`

- [ ] **Step 1: Migrate RMBSentenceParser (lines 7-90)**

```cpp
bool RMBSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool is_valid;
  Nullable<float> xte;
  char steer_dir;
  String origin_wp;
  String dest_wp;
  Nullable<double> dest_lat;
  Nullable<double> dest_lon;
  Nullable<float> range_nm;
  Nullable<float> bearing;
  Nullable<float> closing_velocity;
  char arrival_status;

  if (num_fields < 14) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP(AV, &is_valid),
      FLDP_OPT(Float, xte.ptr()),
      FLDP_OPT(Char, &steer_dir, 255),
      FLDP_OPT(String, &origin_wp),
      FLDP_OPT(String, &dest_wp),
      FLDP_OPT(LatLon, dest_lat.ptr()),
      FLDP_OPT(NS, dest_lat.ptr()),
      FLDP_OPT(LatLon, dest_lon.ptr()),
      FLDP_OPT(EW, dest_lon.ptr()),
      FLDP_OPT(Float, range_nm.ptr()),
      FLDP_OPT(Float, bearing.ptr()),
      FLDP_OPT(Float, closing_velocity.ptr()),
      FLDP_OPT(Char, &arrival_status, 255),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (xte.is_valid()) {
    float signed_xte = xte * 1852.0;
    if (steer_dir == 'L') {
      signed_xte = -signed_xte;
    }
    cross_track_error_.set(signed_xte);
  }
  if (bearing.is_valid()) {
    bearing_to_destination_.set(bearing * DEG_TO_RAD);
  }
  if (range_nm.is_valid()) {
    range_to_destination_.set(range_nm * 1852.0);
  }
  if (closing_velocity.is_valid()) {
    destination_closing_velocity_.set(closing_velocity * 1852.0 / 3600.0);
  }
  if (dest_wp.length() > 0) {
    destination_waypoint_id_.set(dest_wp);
  }

  return true;
}
```

- [ ] **Step 2: Migrate APBSentenceParser (lines 92-171)**

```cpp
bool APBSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool status1;
  bool status2;
  Nullable<float> xte;
  char steer_dir;
  char xte_units;
  char arrival_circle;
  char perpendicular;
  Nullable<float> bearing_origin_dest;
  char bearing_od_type;
  String dest_wp;
  Nullable<float> bearing_pos_dest;
  char bearing_pd_type;
  Nullable<float> heading_to_steer;
  char heading_type;

  if (num_fields < 15) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP(AV, &status1),
      FLDP(AV, &status2),
      FLDP_OPT(Float, xte.ptr()),
      FLDP_OPT(Char, &steer_dir, 255),
      FLDP_OPT(Char, &xte_units, 'N'),
      FLDP_OPT(Char, &arrival_circle, 255),
      FLDP_OPT(Char, &perpendicular, 255),
      FLDP_OPT(Float, bearing_origin_dest.ptr()),
      FLDP_OPT(Char, &bearing_od_type, 255),
      FLDP_OPT(String, &dest_wp),
      FLDP_OPT(Float, bearing_pos_dest.ptr()),
      FLDP_OPT(Char, &bearing_pd_type, 255),
      FLDP_OPT(Float, heading_to_steer.ptr()),
      FLDP_OPT(Char, &heading_type, 255),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (xte.is_valid()) {
    float signed_xte = xte * 1852.0;
    if (steer_dir == 'L') {
      signed_xte = -signed_xte;
    }
    cross_track_error_.set(signed_xte);
  }
  if (heading_to_steer.is_valid()) {
    heading_to_steer_.set(heading_to_steer * DEG_TO_RAD);
  }

  return true;
}
```

- [ ] **Step 3: Migrate BWCSentenceParser (lines 173-255)**

BWC time field uses FLDP_OPT but the parsed time values are not emitted, so raw locals are fine:

```cpp
bool BWCSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  int32_t hour, minute;
  float second;
  Nullable<double> lat;
  Nullable<double> lon;
  Nullable<float> bearing_true;
  Nullable<float> bearing_mag;
  Nullable<float> distance_nm;
  String waypoint_id;
  char t_char;
  char m_char;
  char n_char;

  if (num_fields < 13) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Time, &hour, &minute, &second),
      FLDP_OPT(LatLon, lat.ptr()),
      FLDP_OPT(NS, lat.ptr()),
      FLDP_OPT(LatLon, lon.ptr()),
      FLDP_OPT(EW, lon.ptr()),
      FLDP_OPT(Float, bearing_true.ptr()),
      FLDP_OPT(Char, &t_char, 'T'),
      FLDP_OPT(Float, bearing_mag.ptr()),
      FLDP_OPT(Char, &m_char, 'M'),
      FLDP_OPT(Float, distance_nm.ptr()),
      FLDP_OPT(Char, &n_char, 'N'),
      FLDP_OPT(String, &waypoint_id),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (bearing_true.is_valid()) {
    bearing_true_.set(bearing_true * DEG_TO_RAD);
  }
  if (bearing_mag.is_valid()) {
    bearing_magnetic_.set(bearing_mag * DEG_TO_RAD);
  }
  if (distance_nm.is_valid()) {
    distance_.set(distance_nm * 1852.0);
  }
  if (waypoint_id.length() > 0) {
    waypoint_id_.set(waypoint_id);
  }
  if (lat.is_valid() && lon.is_valid()) {
    Position pos{lat, lon, kPositionInvalidAltitude};
    waypoint_position_.set(pos);
  }

  return true;
}
```

- [ ] **Step 4: Migrate WPLSentenceParser (lines 257-302)**

WPL uses only required fields (FLDP). Only change: use Nullable for lat/lon in case we want consistency, but since these are required fields that fail the parse on empty, raw `double` is fine. Only change needed: nothing — WPL doesn't check sentinels. Leave as-is but change `double lat, lon` to be consistent... Actually, since `ParseLatLon` now writes `Nullable<double>::invalid()` on empty (and FLDP fails on empty), the raw doubles never see the invalid value in practice. Leave WPL unchanged.

Wait — `ParseLatLon` still takes `double*`, and a raw `double lat` works fine. No change needed.

- [ ] **Step 5: Migrate RTESentenceParser (lines 304-354)**

RTE uses only required fields for the header (FLDP). Change `int` locals to `int32_t`:

```cpp
bool RTESentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  int32_t num_sentences;
  int32_t sentence_number;
  char route_type;
  String route_id;

  if (num_fields < 5) {
    return false;
  }

  bool ok = true;
  ok &= FLDP(Int, &num_sentences)(field_strings + field_offsets[1]);
  ok &= FLDP(Int, &sentence_number)(field_strings + field_offsets[2]);
  ok &= FLDP(Char, &route_type, 255)(field_strings + field_offsets[3]);
  ok &= FLDP(String, &route_id)(field_strings + field_offsets[4]);

  if (!ok) {
    return false;
  }

  if (sentence_number == 1) {
    accumulated_waypoints_.clear();
    total_sentences_ = num_sentences;
  }

  for (int i = 5; i < num_fields; i++) {
    String wp_id;
    if (FLDP_OPT(String, &wp_id)(field_strings + field_offsets[i])) {
      if (wp_id.length() > 0) {
        accumulated_waypoints_.push_back(wp_id);
      }
    }
  }

  if (sentence_number == total_sentences_) {
    route_id_.set(route_id);
    waypoints_.set(accumulated_waypoints_);
  }

  return true;
}
```

Also change `total_sentences_` member in `waypoint_sentence_parser.h` from `int` to `int32_t`.

- [ ] **Step 6: Add Nullable include to waypoint_sentence_parser.cpp**

```cpp
#include "sensesp/types/nullable.h"
using sensesp::Nullable;
```

- [ ] **Step 7: Build and test**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32 && pio test -e pioarduino_esp32 --without-uploading`

- [ ] **Step 8: Commit**

```bash
git add src/sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.cpp src/sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.h
git commit -m "refactor(waypoint): migrate RMB, APB, BWC to Nullable<T>

WPL and RTE use only required fields; updated for int32_t only."
```

---

### Task 7: Update tests and final cleanup

**Files:**
- Modify: all test files that reference `kInvalidFloat`, `kInvalidDouble`, or `kInvalidInt`

- [ ] **Step 1: Search for remaining sentinel references**

Run: `grep -r "kInvalidFloat\|kInvalidDouble\|kInvalidInt" /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183/`
Expected: Only hits should be in test files or possibly in the data headers. Fix any remaining references.

- [ ] **Step 2: Update test files**

For any test that directly references `kInvalidFloat`/`kInvalidDouble`/`kInvalidInt`, replace with `Nullable<T>::invalid()` or restructure the assertion. Most tests check `ObservableValue::get()` or `rx_count` — those should be unaffected.

- [ ] **Step 3: Full build across all environments**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio run -e pioarduino_esp32 && pio run -e arduino_esp32`
Expected: Both build successfully.

- [ ] **Step 4: Run all tests**

Run: `cd /Users/mairas/w/hatlabs/esp32/SensESP/NMEA0183 && pio test -e pioarduino_esp32 --without-uploading`
Expected: All 9 test suites pass.

- [ ] **Step 5: Commit any remaining changes**

```bash
git add -A
git commit -m "test: update tests for Nullable<T> migration

Remove references to removed sentinel constants.

Closes #31, closes #32"
```
