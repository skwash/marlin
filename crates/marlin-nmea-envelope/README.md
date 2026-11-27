# marlin-nmea-envelope

Sans-I/O NMEA 0183 envelope parser. Foundation crate of the `marlin` suite.

## What this crate does

- Locates NMEA 0183 sentence boundaries in a byte feed
- Verifies the XOR checksum (bytes between `$`/`!` and `*`)
- Splits the payload into fields (empty fields preserved as empty slices)
- Recognizes NMEA 4.10 TAG block prefixes (`\...*hh\`), captures
  their raw content bytes, and reads the parameters (`s`, `c`, `g`, `n`,
  `r`, `d`, `t`, and unknown keys) through a typed `TagBlock` view
- Exposes a unified [`SentenceSource`] trait with two implementations:
  - [`OneShot`] — one complete sentence per `feed` (for UDP datagrams)
  - [`Streaming`] — buffered scanning for TCP-style byte streams
- Plus a [`Parser`] enum, itself a `SentenceSource`, for choosing the
  source mode at runtime with zero-cost static dispatch

## What this crate does not do

- **No I/O.** The caller drives bytes in and parsed sentences out.
- **No typed decoding** of specific sentence types. That is the job of
  `marlin-nmea-0183` and `marlin-ais`.

## Quickstart

### UDP-datagram mode (one sentence per `feed`, no terminator required)

```rust
use marlin_nmea_envelope::{OneShot, SentenceSource};

let mut parser = OneShot::new();
parser.feed(b"$GPGGA,123519*77"); // checksum 0x77 = XOR of "GPGGA,123519"
let sentence = parser.next_sentence().unwrap().unwrap();
assert_eq!(sentence.sentence_type, "GGA");
assert!(sentence.checksum_ok);
```

### TCP-stream mode (multiple sentences per feed, terminators consumed)

```rust
use marlin_nmea_envelope::{Streaming, SentenceSource};

let mut parser = Streaming::new();
parser.feed(b"$GPGGA,123519*77\r\n$GPGGA,123519*77\r\n");
while let Some(result) = parser.next_sentence() {
    let sentence = result.expect("parse error");
    // ...
}
```

### Source mode chosen at runtime (config-driven)

```rust
use marlin_nmea_envelope::Parser;

let mut parser = if transport_is_udp {
    Parser::one_shot()
} else {
    Parser::streaming()
};
// Identical call shape for both modes.
parser.feed(&bytes);
while let Some(Ok(s)) = parser.next_sentence() { ... }
```

`Parser` is a `SentenceSource`, so the typed parsers in
`marlin-nmea-0183` and `marlin-ais` wrap it exactly as they wrap
`OneShot` or `Streaming`.

### TAG block parameters

```rust
use marlin_nmea_envelope::parse;

let sentence = parse(b"\\s:st1,c:1577836800,g:1-1-9*43\\$GPGGA,123519*77").unwrap();
let tags = sentence.tag_fields().unwrap();
assert_eq!(tags.source(), Some(b"st1".as_slice()));
assert_eq!(tags.timestamp_millis(), Some(1_577_836_800_000));
assert_eq!(tags.group().map(|g| g.id), Some(9));
```

`RawSentence::tag_fields` returns a `TagBlock`. A parameter that is absent
or malformed reads as `None`, and it never hides the other parameters.
`TagBlock::unknown_fields` iterates the keys that NMEA 4.10 does not define.

## Features

| Flag | Default | Purpose |
| --- | --- | --- |
| `tracing` | off | Emits `tracing` events for discarded garbage, buffer overflows, and TAG block checksum mismatches. Add a subscriber in the host application to see them. |

## Architectural commitments

- **Sans-I/O** — no sockets, no runtime, no file handles.
- **One parser core** shared by both modes.
- **`complete` nom parsers** only — no `Err::Incomplete` propagation.
- **Zero-copy** borrows; no allocation on the sentence hot path.
- **TAG block checksum mismatches are advisory**, not fatal (ADR-0006).
- **Panic-free** on all inputs; verified by cargo-fuzz.

## Minimum Supported Rust Version

1.82

## License

Licensed under either of Apache License 2.0 or MIT License at your option.

[`SentenceSource`]: https://docs.rs/marlin-nmea-envelope/latest/marlin_nmea_envelope/trait.SentenceSource.html
[`OneShot`]: https://docs.rs/marlin-nmea-envelope/latest/marlin_nmea_envelope/struct.OneShot.html
[`Streaming`]: https://docs.rs/marlin-nmea-envelope/latest/marlin_nmea_envelope/struct.Streaming.html
[`Parser`]: https://docs.rs/marlin-nmea-envelope/latest/marlin_nmea_envelope/enum.Parser.html
