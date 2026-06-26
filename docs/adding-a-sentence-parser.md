# Adding a sentence parser

This guide explains how to add a parser for a new NMEA 0183 sentence. Read
[architecture.md](architecture.md) first for how parsers fit into the dispatch
and output pipeline.

A sentence parser is a `SentenceParser` subclass that declares which sentence it
handles, decodes the comma-separated fields into typed values, and publishes
those values through `ObservableValue<T>` members.

## Steps

### 1. Pick the domain file

Parsers are grouped by data domain under `src/sensesp_nmea0183/sentence_parser/`:

| File | Sentences |
|------|-----------|
| `gnss_sentence_parser.{h,cpp}` | Position, time, fix quality (GGA, RMC, VTG, GSV, GSA, ZDA, GBS, proprietary RTK) |
| `navigation_sentence_parser.{h,cpp}` | Heading, depth, water speed and temperature (HDG, HDM, HDT, VHW, DPT, DBT, MTW) |
| `wind_sentence_parser.{h,cpp}` | Apparent and true wind (MWV, MWD, VWR) |
| `weather_sentence_parser.{h,cpp}` | Meteorological composite (MDA) |
| `waypoint_sentence_parser.{h,cpp}` | Waypoint and autopilot (RMB, BWC, APB, WPL, RTE) |

Add your parser to the file that matches its data. Create a new pair of files
only for a new domain.

### 2. Declare the class

Declare the class in the domain header. It needs a constructor that forwards the
`NMEA0183Parser*` to the base class, overrides for `parse_fields()` and
`sentence_address()`, and one `ObservableValue<T>` member per output.

```cpp
/// Parser for HDT - Heading, True
class HDTSentenceParser : public SentenceParser {
 public:
  HDTSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "..HDT"; }

  ObservableValue<float> true_heading_;  // radians
};
```

### 3. Choose the sentence address

`sentence_address()` returns the characters to match after the leading `$` or
`!`. The dispatcher compares them with `.` as a wildcard and requires a comma
right after.

- `"..HDT"` matches any two-character talker ID: `$IIHDT`, `$HCHDT`, `$GPHDT`.
- `"G.GGA"` matches any GNSS talker: `$GPGGA`, `$GNGGA`, `$GLGGA`.
- `"..RMB"` matches RMB from any talker.

Proprietary sentences start with `P` and a manufacturer mnemonic. Return the
literal string, including a subsentence selector and its comma when the
manufacturer uses one:

- `"PQTMTAR"` matches `$PQTMTAR,...` (Quectel).
- `"PSTI,030"` matches `$PSTI,030,...` and is distinct from `"PSTI,032"`
  (SkyTraq). The comma is part of the match, and the dispatcher still requires
  the following comma, so `030` becomes field 1 after splitting.

### 4. Implement `parse_fields()`

`parse_fields()` follows one pattern across the library:

```cpp
bool HDTSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  Nullable<float> heading;
  char t_char;

  // $xxHDT,heading,T*cs
  // eg. $HCHDT,98.3,T*21

  if (num_fields < 3) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Heading, degrees true
      FLDP_OPT(Float, heading.ptr()),
      // 2   T = true
      FLDP_OPT(Char, &t_char, 'T'),
  };

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

The pattern, step by step:

1. **Declare locals.** Use `Nullable<T>` for optional fields and a plain `T` for
   required ones. Annotate each with the sentence layout in a comment, as the
   parsers in the repo do.
2. **Guard on field count.** Reject truncated sentences with
   `if (num_fields < N) return false;`. Because field 0 is the address, reading
   fields `1..L` needs `num_fields >= L + 1` (here `L == 2`, so `< 3`). Some
   parsers set a higher minimum to require the full standard sentence.
3. **Build the field-parser array.** `fps[i-1]` handles field `i`. Use the
   `FLDP` and `FLDP_OPT` macros (see below). For a `Nullable<T>` local, pass
   `x.ptr()`; for a plain local, pass `&x`.
4. **Run the array.** The loop applies each entry to its field. `ok &= ...`
   accumulates failures.
5. **Bail on failure.** `if (!ok) return false;` makes the dispatcher try the
   next parser.
6. **Emit guarded outputs.** Guard each optional output with
   `if (x.is_valid())` before calling `member_.set(...)`. Convert to SI units in
   the same step.
7. **Return true** so the base class increments `rx_count_` and emits the
   "received" pulse.

### 5. Register and wire it

Constructing the parser registers it (the base constructor calls
`register_sentence_parser`). A firmware sketch can construct it directly and
connect its members, but the convention is to add it to a `Connect*()` function
in `wiring.cpp`: create the parser, connect its `ObservableValue` members to a
data container struct in `data/`, then connect the container fields to
`SKOutput` paths. See `ConnectHeading` for a small example.

### 6. Write a test

Add a Unity test under `test/<suite>/<suite>.cpp`. Construct a parser, feed a
sentence with a valid checksum, and assert on the outputs and `get_rx_count()`:

```cpp
void test_hdt(void) {
  parser->set("$HCHDT,98.3,T*21");

  TEST_ASSERT_EQUAL_INT(1, hdt->get_rx_count());
  TEST_ASSERT_FLOAT_WITHIN(0.001, 98.3 * DEG_TO_RAD, hdt->true_heading_.get());
}
```

Use a real checksum (`AddChecksum()` computes one), or call
`ignore_checksum(true)` on the parser to skip the check. Cover empty optional
fields and invalid checksums too. Add the new suite to `test/README`. Build and
run:

```bash
pio test -e pioarduino_esp32 --without-uploading --without-testing  # compile only
pio test -e pioarduino_esp32                                        # run on device
```

## Field parser reference

Each field parser converts one null-terminated field string to a typed value
and returns `false` when the input is malformed. The trailing `allow_empty`
argument controls what happens for an empty field; the `FLDP_OPT` macro sets it
to `true`.

| Function | Signature | Empty field | Notes |
|----------|-----------|-------------|-------|
| `ParseString` | `(String* value, s, allow_empty=false)` | sets `*value = ""`, returns `allow_empty` | copies the field otherwise |
| `ParseInt` | `(int32_t* value, s, allow_empty=false)` | sets `Nullable<int32_t>::invalid()`, returns `allow_empty` | `sscanf` decimal |
| `ParseFloat` | `(float* value, s, allow_empty=false)` | sets `Nullable<float>::invalid()`, returns `allow_empty` | `sscanf` float |
| `ParseDouble` | `(double* value, s, allow_empty=false)` | sets `Nullable<double>::invalid()`, returns `allow_empty` | `sscanf` double |
| `ParseLatLon` | `(double* value, s, allow_empty=false)` | sets `Nullable<double>::invalid()`, returns `allow_empty` | converts `ddmm.mmmm` to decimal degrees |
| `ParseNS` | `(double* value, s, allow_empty=false)` | returns `allow_empty`, leaves value untouched | applies hemisphere sign in place; `S` negates. Call after `ParseLatLon` on the same variable. Returns true if the value is already invalid. |
| `ParseEW` | `(double*/float* value, s, allow_empty=false)` | returns `allow_empty`, leaves value untouched | as `ParseNS`; `W` negates |
| `ParseChar` | `(char* value, char expected, s, allow_empty=false)` | sets `*value = 0`, returns `allow_empty` | rejects multi-character fields; returns `*s == expected`, except `expected == 255` accepts any character |
| `ParseAV` | `(bool* is_valid, s)` | n/a | `A` -> true, `V` -> false, anything else fails. No `allow_empty`. |
| `ParseTime` | `(int32_t* hour, int32_t* minute, float* second, s, allow_empty=false)` | sets all three invalid, returns `allow_empty` | parses `hhmmss.sss` |
| `ParseDate` | `(int32_t* year, int32_t* month, int32_t* day, s, allow_empty=false)` | sets all three invalid, returns `allow_empty` | parses `ddmmyy`; stores `struct tm` values (`year += 100`, `month -= 1`) |
| `ParseEmpty` | `(s)` | returns true only when the field is empty | use for reserved or placeholder fields |
| `ConvertSpeedToMs` | `(float* speed, char unit)` | n/a | scales in place: `K` km/h, `M` m/s, `N` knots, `S` statute mph; false on an unknown unit. Helper, not a field parser. |

### The `FLDP` macros

The two macros turn a field parser into a `std::function<bool(const char*)>`
that captures its target by reference:

```cpp
#define FLDP(f, ...) \
  [&](const char* s) { return Parse##f(__VA_ARGS__ __VA_OPT__(, ) s); }

#define FLDP_OPT(f, ...) \
  [&](const char* s) { return Parse##f(__VA_ARGS__ __VA_OPT__(, ) s, true); }
```

- `FLDP(Float, &temperature)` expands to `ParseFloat(&temperature, s)` and
  treats an empty field as a parse failure. Use it for required fields.
- `FLDP_OPT(Float, heading.ptr())` expands to
  `ParseFloat(heading.ptr(), s, true)` and treats an empty field as
  "no value". Use it for optional fields.
- `FLDP(Empty)` expands to `ParseEmpty(s)`; the `__VA_OPT__` drops the comma
  when there are no extra arguments.

### `Nullable<T>` and optional fields

The library uses `Nullable<T>` from `sensesp/types/nullable.h` to represent a
missing optional value, instead of a sentinel constant in the parser. An empty
optional field writes `Nullable<T>::invalid()` into the local, and the emit is
guarded by `is_valid()`:

```cpp
Nullable<float> depth;          // optional field
...
FLDP_OPT(Float, depth.ptr()),   // empty input -> invalid()
...
if (depth.is_valid()) {
  depth_.set(depth);            // implicit conversion to T
}
```

`Nullable<T>::ptr()` returns a `T*` to the underlying value, which is what the
field parsers write to. A `Nullable<T>` converts implicitly to `T`, so
`depth_.set(depth)` works directly. `Nullable<bool>` is not supported; use a
plain `bool` with `ParseAV` for validity flags.

## Conventions

- **Naming.** A parser class is `<FORMATTER>SentenceParser`, for example
  `HDGSentenceParser`. `ObservableValue` members end with an underscore.
- **File grouping.** Keep a parser in the domain file that matches its data
  (gnss, navigation, wind, weather, waypoint).
- **Addresses.** Use `.` to wildcard the talker ID, `G.` for any GNSS talker,
  and a literal string for proprietary sentences.
- **Required vs optional.** `FLDP` plus a plain local for required fields;
  `FLDP_OPT` plus a `Nullable<T>` local for optional ones.
- **Field-count guard.** Require at least enough fields that every index your
  `fps` array reads exists.
- **SI units.** Convert before emitting: radians for angles, m/s for speed,
  meters for distance and depth, kelvin for temperature.
- **Shared addresses.** Two parsers may share an address if each rejects the
  other's sentences through a required field-value check.
