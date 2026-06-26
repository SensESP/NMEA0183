# Nullable Migration and Field Parser Standardization

**Issues:** [#31](https://github.com/SensESP/NMEA0183/issues/31), [#32](https://github.com/SensESP/NMEA0183/issues/32)
**Date:** 2026-03-23

## Problem

The NMEA0183 library uses sentinel constants (`kInvalidFloat`, `kInvalidDouble`, `kInvalidInt`) to represent unparsed/empty fields. This pattern is fragile — callers must remember to check against magic constants — and inconsistent with the `Nullable<T>` type already used in `GNSSSatellite`. Concrete parser implementations also vary in structure: some use a lambda array + loop, others use inline calls, making the codebase harder to extend.

## Decision: Approach B — Nullable locals, keep parser signatures

### Why not the alternatives

- **Approach A (Nullable return types):** Changes every field parser signature for no external benefit, since the public API (`ObservableValue<float>`) stays unchanged.
- **Approach C (Wrapper at call sites only):** Sentinels still exist in field parsers, two validity systems coexist. Minimal readability improvement.
- **`std::optional<T>`:** C++17 not universally supported across all CI/build environments. `Nullable<T>` can be mechanically migrated to `std::optional` later.

## Design

### Constraint

The public API does not change. `ObservableValue<float>`, `ObservableValue<int>`, etc. remain as-is. `Nullable<T>` is internal to the parser layer.

### 1. Field parser changes

`ParseFloat`, `ParseInt`, `ParseDouble`, `ParseLatLon`, `ParseTime`, `ParseDate`, etc. keep their current pointer-based signatures. The change: on empty fields, they write `Nullable<T>::invalid()` instead of `kInvalidFloat`/`kInvalidInt`/`kInvalidDouble`.

The sentinel constants `kInvalidFloat`, `kInvalidDouble`, `kInvalidInt` are removed from `field_parsers.h`. This is a breaking change for any downstream code that references these constants directly. This is intentional — downstream code should also migrate to `Nullable<T>` and `is_valid()`.

**Sentinel value mismatch:** The old sentinels (`numeric_limits<T>::lowest()`) differ numerically from `Nullable<T>::invalid()` (`-1e9` for float/double, `0x7fffffff` for int32). This is safe because all validity checks are migrated to `is_valid()` simultaneously — no code path will compare raw values against the old constants after migration.

**Portability:** Use `Nullable<int32_t>` (not `Nullable<int>`) for integer locals. `Nullable<int>::invalid_value_` is only defined for ESP Arduino v3+, but `Nullable<int32_t>` is unconditionally defined. To make this self-consistent, change `ParseInt`, `ParseTime`, and `ParseDate` signatures from `int*` to `int32_t*`. This is the one signature change in the field parser layer — necessary to avoid type mismatch between `Nullable<int32_t>::ptr()` (returns `int32_t*`) and the parser parameter types.

**Includes:** `field_parsers.h` must `#include "sensesp/types/nullable.h"` and use `sensesp::Nullable<T>::invalid()` to obtain the invalid sentinel values.

**Default construction:** `Nullable<T>` default-constructs to value 0 with `is_valid() == true`. This is safe because the canonical pattern's parse loop covers all declared Nullable locals — every one gets written by a field parser. The `if (!ok) return false` guard catches parse failures before step 4 can emit stale defaults.

### 2. Concrete parser standardization

Every `parse_fields()` implementation follows a canonical pattern:

```cpp
bool ExampleSentenceParser::parse_fields(const char* field_strings,
                                          const int* field_offsets,
                                          int num_fields) {
  // 1. Declare Nullable locals
  Nullable<float> wind_angle;
  Nullable<float> wind_speed;

  // 2. Lambda array with FLDP/FLDP_OPT macros
  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, wind_angle.ptr()),
      FLDP_OPT(Float, wind_speed.ptr()),
  };

  // 3. Loop
  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
      ok &= fps[i - 1](field_strings + field_offsets[i]);
  }
  if (!ok) return false;

  // 4. Emit valid values
  if (wind_angle.is_valid()) {
      wind_angle_.set(wind_angle * DEG_TO_RAD);
  }
  if (wind_speed.is_valid()) {
      wind_speed_.set(wind_speed);
  }

  return true;
}
```

Key conventions:
- Nullable locals declared at top
- Lambda array with `FLDP`/`FLDP_OPT` macros, passed `.ptr()` pointers
- Single loop over field array
- Validity checks via `is_valid()` before emitting to `ObservableValue`
- Fallback logic (e.g., prefer m/s over knots in MWD/MDA) stays in step 4

### Special cases

#### Paired fields: LatLon + NS/EW

`ParseNS` and `ParseEW` mutate an existing value (multiply by -1 for S/W). Both the coordinate and its direction modifier must share the same `Nullable` variable:

```cpp
Nullable<double> latitude;
// Both parsers operate on the same Nullable via .ptr()
FLDP_OPT(LatLon, latitude.ptr()),
FLDP_OPT(NS, latitude.ptr()),
```

Do NOT create separate Nullable locals for the direction field. The same pattern applies to variation/deviation + EW in HDG.

**Sentinel negation hazard:** If the latitude field is empty (written as `Nullable<double>::invalid()` = `-1e9`) but NS contains `S`, `ParseNS` would negate the value to `+1e9`, which passes `is_valid()` — a silent data corruption. Fix: add a guard in `ParseNS` and `ParseEW` to no-op when the current value equals the Nullable invalid sentinel. This check goes inside the field parser functions themselves:

```cpp
bool ParseNS(double* value, const char* s, bool allow_empty) {
    if (s[0] == 0) return allow_empty;
    if (*value == Nullable<double>::invalid()) return true;  // no-op on invalid
    if (s[0] == 'S') *value *= -1;
    return true;
}
```

#### Multi-pointer fields: ParseTime / ParseDate

`ParseTime` writes through 3 pointers (hour, minute, second). `ParseDate` writes through 3 pointers (year, month, day). Each output gets its own Nullable local:

```cpp
Nullable<int32_t> hour, minute;
Nullable<float> second;
// Macro receives all three .ptr() arguments
FLDP(Time, hour.ptr(), minute.ptr(), second.ptr()),
```

When the time field is empty, `ParseTime` writes the Nullable invalid value to all three outputs. All become `!is_valid()`.

#### Position struct interaction

`Position` is a SensESP core type with raw `double` fields — not Nullable. Parse into separate Nullable locals, then construct Position only when valid:

```cpp
Nullable<double> latitude, longitude;
Nullable<float> altitude;
// ... parse fields ...
if (latitude.is_valid() && longitude.is_valid()) {
    Position position{latitude, longitude,
                      altitude.is_valid() ? (double)altitude : kPositionInvalidAltitude};
    position_.set(position);
}
```

`kPositionInvalidAltitude` is a SensESP core constant — it stays as-is since it belongs to the Position type, not to this library.

#### char fields: keep raw

`ParseChar` has different semantics (checks against `expected` character). `Nullable<char>::invalid()` is `0xff`, which doesn't align with `ParseChar`'s empty-field behavior. Keep `char` locals as raw `char` — they are typically required fields validated by the `expected` parameter, not optional values needing Nullable.

#### Variable-length sentences: DPT, RTE

`DPTSentenceParser` has conditional field parsing based on `num_fields`. It follows the canonical pattern but with a truncated lambda array or conditional tail:

```cpp
// Parse required fields via array
// Then conditionally parse trailing optional fields
if (num_fields >= 3) {
    ok &= FLDP_OPT(Float, offset.ptr())(field_strings + field_offsets[2]);
}
```

`RTESentenceParser` keeps structural differences for its multi-sentence accumulation and variable-length waypoint list, but uses `Nullable` + `is_valid()`.

### 3. Macro changes

`FLDP` and `FLDP_OPT` macros are unchanged. `.ptr()` returns a raw pointer, so macros don't need to know about `Nullable`.

### 4. Test changes

- Remove references to `kInvalidFloat`, `kInvalidDouble`, `kInvalidInt`
- Assertion patterns remain largely the same — tests check `ObservableValue::get()` (public API) and `rx_count`, not internal Nullable state

## Scope boundaries

This spec covers issues #31 and #32 only. The following remain separate:
- **#33** (RTE multi-sentence timeout) — follow-up after this refactor
- **#34** (ProprietarySentenceParser base class) — independent work
- **#35** (Architecture documentation) — written after patterns settle

## Files affected

- `src/sensesp_nmea0183/sentence_parser/field_parsers.h` — remove sentinel constants, add Nullable include
- `src/sensesp_nmea0183/sentence_parser/field_parsers.cpp` — write `Nullable<T>::invalid()` on empty fields
- `src/sensesp_nmea0183/sentence_parser/gnss_sentence_parser.cpp` — standardize pattern, use Nullable
- `src/sensesp_nmea0183/sentence_parser/navigation_sentence_parser.cpp` — standardize pattern, use Nullable
- `src/sensesp_nmea0183/sentence_parser/wind_sentence_parser.cpp` — standardize pattern, use Nullable
- `src/sensesp_nmea0183/sentence_parser/weather_sentence_parser.cpp` — standardize pattern, use Nullable
- `src/sensesp_nmea0183/sentence_parser/waypoint_sentence_parser.cpp` — standardize pattern, use Nullable
- `test/` — update sentinel references
