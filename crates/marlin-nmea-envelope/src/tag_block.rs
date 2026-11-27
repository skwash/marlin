//! Typed view of an NMEA 4.10 TAG block.

use alloc::vec::Vec;

/// Timestamps above this value are milliseconds, not seconds.
///
/// 40 000 000 000 seconds is the year 3138, so no real clock in seconds
/// reaches it. A real clock in milliseconds passed it in 1970.
const MILLISECONDS_THRESHOLD: u64 = 40_000_000_000;

/// The parameter keys that NMEA 4.10 defines.
const KNOWN_KEYS: [&[u8]; 7] = [b"c", b"d", b"g", b"n", b"r", b"s", b"t"];

/// The `g` parameter of a TAG block: one sentence of a sentence group.
///
/// On the wire the parameter is `part-total-id`, for example `1-2-1481`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TagGroup {
    /// 1-based number of this sentence in the group.
    pub part: u32,
    /// Number of sentences in the group.
    pub total: u32,
    /// Identifier that ties the sentences of one group together.
    pub id: u32,
}

/// A borrowed, typed view of the content of one TAG block.
///
/// The view holds the raw bytes and parses each parameter when you ask
/// for it. A parameter that is absent or malformed reads as `None`. A
/// malformed parameter never hides the other parameters. When a key
/// occurs more than once, the last occurrence wins.
///
/// Get a view from [`RawSentence::tag_fields`](crate::RawSentence::tag_fields),
/// or from [`TagBlock::parse`] if you hold the content bytes already.
/// The checksum is not part of the view (ADR-0006).
///
/// ```
/// use marlin_nmea_envelope::parse;
///
/// let line = b"\\s:st1,c:1577836800,g:1-1-9*43\\$GPGGA,123519*77";
/// let tags = parse(line).unwrap().tag_fields().unwrap();
/// assert_eq!(tags.source(), Some(b"st1".as_slice()));
/// assert_eq!(tags.timestamp_millis(), Some(1_577_836_800_000));
/// assert_eq!(tags.group().map(|g| g.id), Some(9));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TagBlock<'a> {
    content: &'a [u8],
}

impl<'a> TagBlock<'a> {
    /// View `content`: the bytes between the backslashes of a TAG block,
    /// without the trailing `*hh`.
    #[must_use]
    pub fn parse(content: &'a [u8]) -> Self {
        Self { content }
    }

    /// The content bytes that this view parses.
    #[must_use]
    pub fn content(&self) -> &'a [u8] {
        self.content
    }

    /// Every `key:value` parameter in order of appearance.
    ///
    /// Empty parameters and parameters without a colon are skipped. The
    /// value is everything after the first colon.
    pub fn fields(&self) -> impl Iterator<Item = (&'a [u8], &'a [u8])> {
        self.content
            .split(|&b| b == b',')
            .filter_map(|field| {
                let colon = field.iter().position(|&b| b == b':')?;
                let key = field.get(..colon)?;
                let value = field.get(colon.saturating_add(1)..)?;
                Some((key, value))
            })
            .filter(|(key, _)| !key.is_empty())
    }

    /// The parameters whose key NMEA 4.10 does not define.
    pub fn unknown_fields(&self) -> impl Iterator<Item = (&'a [u8], &'a [u8])> {
        self.fields().filter(|(key, _)| !KNOWN_KEYS.contains(key))
    }

    /// The `s` parameter: the source (station) identifier.
    #[must_use]
    pub fn source(&self) -> Option<&'a [u8]> {
        self.last_value(b"s")
    }

    /// The `d` parameter: the destination identifier.
    #[must_use]
    pub fn destination(&self) -> Option<&'a [u8]> {
        self.last_value(b"d")
    }

    /// The `t` parameter: free text.
    #[must_use]
    pub fn text(&self) -> Option<&'a [u8]> {
        self.last_value(b"t")
    }

    /// The `c` parameter as Unix milliseconds.
    ///
    /// A value above 40 000 000 000 is already in milliseconds. A smaller
    /// value is in seconds and is multiplied by 1000. A value that does
    /// not fit in `u64` after the conversion reads as `None`.
    #[must_use]
    pub fn timestamp_millis(&self) -> Option<u64> {
        let value = parse_unsigned(self.last_value(b"c")?)?;
        if value > MILLISECONDS_THRESHOLD {
            Some(value)
        } else {
            value.checked_mul(1000)
        }
    }

    /// The `g` parameter.
    ///
    /// Empty parts are dropped before the parts are read, so `1-2--001`
    /// reads as part 1, total 2, id 1. The parameter is malformed, and
    /// reads as `None`, unless exactly three numbers remain.
    #[must_use]
    pub fn group(&self) -> Option<TagGroup> {
        let mut numbers = self
            .last_value(b"g")?
            .split(|&b| b == b'-')
            .filter(|part| !part.is_empty())
            .map(parse_unsigned);
        let part = u32::try_from(numbers.next()??).ok()?;
        let total = u32::try_from(numbers.next()??).ok()?;
        let id = u32::try_from(numbers.next()??).ok()?;
        if numbers.next().is_some() {
            return None;
        }
        Some(TagGroup { part, total, id })
    }

    /// The `n` parameter: the line count.
    #[must_use]
    pub fn line_count(&self) -> Option<u64> {
        parse_unsigned(self.last_value(b"n")?)
    }

    /// The `r` parameter: the relative time.
    #[must_use]
    pub fn relative_time(&self) -> Option<u64> {
        parse_unsigned(self.last_value(b"r")?)
    }

    /// Copy the content so the view can outlive the parser buffer.
    #[must_use]
    pub fn to_owned_block(&self) -> OwnedTagBlock {
        OwnedTagBlock {
            content: self.content.to_vec(),
        }
    }

    fn last_value(&self, key: &[u8]) -> Option<&'a [u8]> {
        self.fields()
            .filter(|(k, _)| *k == key)
            .map(|(_, value)| value)
            .last()
    }
}

/// An owned copy of the content of one TAG block.
///
/// Use [`as_tag_block`](Self::as_tag_block) to read the parameters.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct OwnedTagBlock {
    content: Vec<u8>,
}

impl OwnedTagBlock {
    /// Copy `content`: the bytes of a TAG block without the `*hh`.
    #[must_use]
    pub fn new(content: &[u8]) -> Self {
        Self {
            content: content.to_vec(),
        }
    }

    /// Borrow the parameters as a [`TagBlock`].
    #[must_use]
    pub fn as_tag_block(&self) -> TagBlock<'_> {
        TagBlock::parse(&self.content)
    }
}

impl<'a> From<TagBlock<'a>> for OwnedTagBlock {
    fn from(block: TagBlock<'a>) -> Self {
        block.to_owned_block()
    }
}

/// Parse ASCII decimal digits. Signs, spaces and an empty slice fail.
fn parse_unsigned(bytes: &[u8]) -> Option<u64> {
    if bytes.is_empty() || !bytes.iter().all(u8::is_ascii_digit) {
        return None;
    }
    core::str::from_utf8(bytes).ok()?.parse().ok()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use alloc::vec;

    fn block(content: &[u8]) -> TagBlock<'_> {
        TagBlock::parse(content)
    }

    #[test]
    fn reads_every_known_parameter() {
        let b = block(b"s:r003669945,c:1241544035,g:1-2-73874,n:157036,r:77,d:dest,t:hi");
        assert_eq!(b.source(), Some(&b"r003669945"[..]));
        assert_eq!(b.timestamp_millis(), Some(1_241_544_035_000));
        assert_eq!(
            b.group(),
            Some(TagGroup {
                part: 1,
                total: 2,
                id: 73874
            })
        );
        assert_eq!(b.line_count(), Some(157_036));
        assert_eq!(b.relative_time(), Some(77));
        assert_eq!(b.destination(), Some(&b"dest"[..]));
        assert_eq!(b.text(), Some(&b"hi"[..]));
        assert_eq!(b.unknown_fields().count(), 0);
    }

    #[test]
    fn timestamp_in_milliseconds_is_kept() {
        assert_eq!(
            block(b"c:1481189256123").timestamp_millis(),
            Some(1_481_189_256_123)
        );
    }

    #[test]
    fn timestamp_threshold_is_exclusive() {
        assert_eq!(
            block(b"c:40000000000").timestamp_millis(),
            Some(40_000_000_000_000)
        );
        assert_eq!(
            block(b"c:40000000001").timestamp_millis(),
            Some(40_000_000_001)
        );
    }

    #[test]
    fn timestamp_that_overflows_reads_as_none() {
        assert_eq!(block(b"c:99999999999999999999").timestamp_millis(), None);
    }

    #[test]
    fn group_drops_empty_parts() {
        let expected = Some(TagGroup {
            part: 1,
            total: 2,
            id: 1,
        });
        assert_eq!(block(b"g:1-2--001").group(), expected);
        assert_eq!(block(b"g:1-2-1").group(), expected);
    }

    #[test]
    fn group_needs_exactly_three_numbers() {
        assert_eq!(block(b"g:1-2").group(), None);
        assert_eq!(block(b"g:1-2-3-4").group(), None);
        assert_eq!(block(b"g:").group(), None);
        assert_eq!(block(b"g:1-x-3").group(), None);
        assert_eq!(block(b"g:1--2").group(), None);
    }

    #[test]
    fn malformed_parameter_keeps_the_others() {
        let b = block(b"s:abc,g:bad,c:12x,n:5");
        assert_eq!(b.source(), Some(&b"abc"[..]));
        assert_eq!(b.group(), None);
        assert_eq!(b.timestamp_millis(), None);
        assert_eq!(b.line_count(), Some(5));
    }

    #[test]
    fn unknown_parameters_are_preserved_in_order() {
        let b = block(b"s:a,T:2016-12-08 09.07.36,x:1,y:");
        let unknown: Vec<_> = b.unknown_fields().collect();
        assert_eq!(
            unknown,
            vec![
                (&b"T"[..], &b"2016-12-08 09.07.36"[..]),
                (&b"x"[..], &b"1"[..]),
                (&b"y"[..], &b""[..]),
            ]
        );
    }

    #[test]
    fn value_keeps_later_colons() {
        let b = block(b"t:a:b");
        assert_eq!(b.text(), Some(&b"a:b"[..]));
    }

    #[test]
    fn last_occurrence_wins() {
        assert_eq!(block(b"s:one,s:two").source(), Some(&b"two"[..]));
    }

    #[test]
    fn empty_and_colonless_parameters_are_skipped() {
        let b = block(b",,s:a,junk,:novalue,n:1");
        let all: Vec<_> = b.fields().collect();
        assert_eq!(all, vec![(&b"s"[..], &b"a"[..]), (&b"n"[..], &b"1"[..])]);
    }

    #[test]
    fn signs_and_spaces_do_not_parse_as_numbers() {
        assert_eq!(block(b"n:+5").line_count(), None);
        assert_eq!(block(b"n: 5").line_count(), None);
        assert_eq!(block(b"n:-5").line_count(), None);
    }

    #[test]
    fn empty_content_has_no_parameters() {
        let b = block(b"");
        assert_eq!(b.source(), None);
        assert_eq!(b.fields().count(), 0);
    }

    #[test]
    fn owned_copy_reads_the_same() {
        let owned = block(b"s:a,g:1-2-3").to_owned_block();
        assert_eq!(owned.as_tag_block().source(), Some(&b"a"[..]));
        assert_eq!(OwnedTagBlock::new(b"s:a,g:1-2-3"), owned);
    }
}
