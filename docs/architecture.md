# NMEA 0183 library architecture

This library reads NMEA 0183 sentences from a serial stream, parses them, and
publishes the decoded values as SensESP observables that connect to Signal K.
This page describes the parts and how data moves through them. For a
step-by-step guide to writing a parser, see
[adding-a-sentence-parser.md](adding-a-sentence-parser.md).

## Parts

| Part | File | Role |
|------|------|------|
| `NMEA0183IO` | `nmea0183.h` | Reads lines from a `Stream`, filters them, and feeds them to the parser. Also writes sentences back to the stream. |
| `NMEA0183Parser` | `nmea0183.{h,cpp}` | Holds the registered sentence parsers and dispatches each incoming sentence to the one that matches. |
| `SentenceParser` | `sentence_parser/sentence_parser.{h,cpp}` | Base class. Validates the checksum, splits a sentence into fields, and calls `parse_fields()`. |
| Field parsers | `sentence_parser/field_parsers.{h,cpp}` | Free functions that convert one field string to a typed value (`ParseFloat`, `ParseLatLon`, ...). |
| Sentence parsers | `sentence_parser/<domain>_sentence_parser.{h,cpp}` | One subclass per sentence type. Decodes fields into `ObservableValue` members. |
| Data containers and wiring | `data/*.h`, `wiring.{h,cpp}` | `Connect*()` helpers that create parsers, group their outputs into a struct, and connect them to `SKOutput` paths. |

## Data flow

```
serial bytes
  -> StreamLineProducer            one String per line
  -> Filter<String>                keep lines starting with '$' or '!'
  -> NMEA0183Parser::set           trim, then parse_sentence
  -> SentenceParser::parse         validate checksum, split into fields
  -> SentenceParser::parse_fields  decode fields, ObservableValue::set
  -> ObservableValue<T> members
  -> SKOutput<T>                   Signal K deltas
```

`NMEA0183IO` wires the first three steps in its constructor:

```cpp
line_producer_->connect_to(sentence_filter_)->connect_to(&parser_);
```

### Dispatch

`NMEA0183Parser::parse_sentence()` selects the parser for a sentence:

1. The sentence must start with `$` or `!`; otherwise it is ignored.
2. For each registered parser, the text after the start character is compared
   against `parser->sentence_address()` with `strncmpwc`, a length-limited
   compare in which a `.` in the address matches any character.
3. A match also requires a comma immediately after the address, so `..HDG`
   matches `$IIHDG,...` but not `$IIHDGX,...`.
4. The first parser whose `parse()` returns true ends the dispatch. If none
   succeeds, the sentence is dropped.

Because dispatch stops at the first success, two parsers may share an address
and disambiguate inside `parse_fields()`. `MWVSentenceParser` and
`TrueWindMWVSentenceParser` both register `..MWV`; one requires `R` in the
reference field, the other `T`, so the parser that does not match returns false
and dispatch falls through to the other.

Parsers register themselves. The `SentenceParser` constructor calls
`register_sentence_parser(this)`, so constructing a parser with a
`NMEA0183Parser*` is enough to add it to the dispatch list.

### Sentence parsing

`SentenceParser::parse()` runs the same steps for every sentence type:

1. Validate the checksum: the XOR of all bytes between `$` and `*`, compared
   against the two hex digits after `*`. `ignore_checksum(true)` skips this.
2. Copy the sentence into a working buffer, drop the checksum, and replace each
   comma with a `\0`. `field_offsets[i]` then points to the start of field `i`,
   and `field_strings + field_offsets[i]` is that field as a null-terminated
   string.
3. Call `parse_fields(field_strings, field_offsets, num_fields)`.
4. On success, increment `rx_count_` and `emit(true)`.

Field 0 is the address field (`$GPGGA`), so the first data field is field 1.
`num_fields` counts field 0, so a sentence with two data fields has
`num_fields == 3`. A sentence may carry at most `kNMEA0183MaxFields` (25) fields
and `kNMEA0183InputBufferLength` (164) characters.

`SentenceParser` is a `ValueProducer<bool>`. `emit(true)` signals "a valid
sentence of this type arrived", which is separate from the decoded values. The
values flow out through the parser's own `ObservableValue<T>` members.

### Output and Signal K

Each parser exposes its results as `ObservableValue<T>` members, for example
`GGASentenceParser::position_`. The `Connect*()` functions in `wiring.cpp`
create the parsers, copy their outputs into a data container struct (such as
`GNSSData`), and connect the container fields to `SKOutput` paths. Firmware that
wants a different mapping can skip the helpers and connect parser members
directly.
