//! Multi-sentence AIS reassembly.
//!
//! AIVDM/AIVDO encapsulates one AIS message into one or more NMEA 0183
//! sentences — the AIS payload is armored into 6-bit ASCII and split
//! when it exceeds the NMEA sentence-length ceiling. Downstream
//! decoders need the complete armored payload; this module stitches
//! fragments back together.
//!
//! # Design choices
//!
//! - **Two reassembly keys.** A fragment whose TAG block carries a
//!   valid `g:part-total-id` parameter is keyed on the TAG group:
//!   `(source, id)`, where the source is the `s` parameter. Any other
//!   fragment is keyed on the AIVDM header: `(channel, sequential_id)`.
//!   Two stations on one merged feed often reuse the same sequential
//!   id and channel, so only the TAG group tells their fragments apart.
//!   A `g` parameter is valid for a fragment only if its part and total
//!   equal the AIVDM fragment number and count. Otherwise the fragment
//!   falls back to the header key.
//!
//! - **The source may be absent on later fragments.** NMEA 4.10 puts
//!   `s` on the first sentence of a group only. A fragment without a
//!   source joins any open group with its id and count. The group
//!   learns its source from the first fragment that has one.
//!
//! - **Ordering.** Header-keyed fragments must arrive in order 1, 2,
//!   3, ... . Any other arrival yields
//!   [`AisError::ReassemblyOutOfOrder`] and discards the partial.
//!   Group-keyed fragments may arrive in any order: each one fills the
//!   slot of its part number, and the group completes when every slot
//!   is full. A part that arrives twice before the group completes
//!   restarts the group with the new fragment.
//!
//! - **Single-fragment fast path**: when `fragment_count == 1`, the
//!   reassembler returns a [`ReassembledPayload`] immediately without
//!   touching internal state. No allocation beyond the one `Vec<u8>`
//!   that owns the payload copy.
//!
//! - **Bounded-slots eviction for memory safety**: rather than a
//!   wall-clock timeout (this crate is sans-I/O and `#![no_std]`),
//!   the reassembler caps the number of concurrent partials. When the
//!   cap is exceeded, the oldest partial is evicted and
//!   [`AisError::ReassemblyTimeout`] is queued for the next
//!   [`take_pending_error`](AisReassembler::take_pending_error) call.
//!   A time limit is the usual remedy; this bounded-slot approach
//!   satisfies the underlying memory-safety goal clock-free. Time-based
//!   expiry is opt-in on top of it:
//!   [`AisReassembler::with_timeout_ms`] sets an age limit, and the
//!   caller supplies the clock through [`tick`](AisReassembler::tick)
//!   (ADR-0004). The reassembler keeps the last ticked time and
//!   stamps each fragment with it; it never reads a clock itself.
//!   Both keys share the slots and the age limit.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

use marlin_nmea_envelope::{OwnedTagBlock, TagBlock};

use crate::{AisError, AivdmHeader};

/// The output of [`AisReassembler::feed_fragment`] when a complete
/// armored payload is ready for [`crate::armor::decode`].
///
/// `payload` is the **armored** byte slice (ASCII-6-bit), concatenated
/// across fragments. `fill_bits` is the final fragment's fill count,
/// which applies to the whole concatenated stream (intermediate
/// fragments conventionally carry `fill_bits = 0`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReassembledPayload {
    /// Concatenated armored payload bytes.
    pub payload: Vec<u8>,
    /// Fill bits to strip from the end of the decoded bit stream
    /// (taken from the last fragment; 0..=5).
    pub fill_bits: u8,
    /// `is_own_ship` propagated from the first fragment.
    pub is_own_ship: bool,
    /// The TAG block of the first fragment, if it had one. Only
    /// [`AisReassembler::feed_fragment_tagged`] can set it.
    pub tag_block: Option<OwnedTagBlock>,
}

/// Default maximum number of concurrent multi-sentence assemblies held
/// by a reassembler. Sized for typical AIS receiver loads — real-world
/// channels rarely see more than a handful of in-flight multi-sentence
/// messages at once.
pub const DEFAULT_MAX_PARTIALS: usize = 16;

/// Multi-sentence reassembler.
///
/// Feed it [`AivdmHeader`] values in the order they arrive; it buffers
/// multi-fragment messages and returns a [`ReassembledPayload`] when a
/// complete armored payload is available. Single-fragment messages are
/// returned immediately on the first call.
///
/// # Errors
///
/// [`feed_fragment`](Self::feed_fragment) and
/// [`feed_fragment_tagged`](Self::feed_fragment_tagged) can surface:
///
/// - [`AisError::MalformedWrapper`] — a header-keyed multi-fragment
///   header missing `sequential_id`, or `fragment_number == 0`, or
///   `fragment_number > fragment_count`.
/// - [`AisError::ReassemblyOutOfOrder`] — a header-keyed fragment
///   arrived with a number other than the next expected one for its
///   key, or no partial is open on its `(channel, sequential_id)` key.
///   A group-keyed fragment never raises it.
///
/// Evictions caused by exceeding
/// [`max_partials`](Self::max_partials) surface
/// [`AisError::ReassemblyTimeout`] via
/// [`take_pending_error`](Self::take_pending_error), not via
/// [`feed_fragment`](Self::feed_fragment). This keeps "new fragment
/// accepted, but an old partial was dropped" a distinct signal from
/// "the current fragment failed."
#[derive(Debug)]
pub struct AisReassembler {
    partials: Vec<PartialMessage>,
    max_partials: usize,
    /// Caller-supplied timeout in monotonic milliseconds. `None`
    /// means time-based eviction is disabled; only bounded-slots
    /// eviction applies.
    timeout_ms: Option<u64>,
    /// The clock as of the last [`tick`](AisReassembler::tick), in
    /// monotonic milliseconds. `None` until the first tick; partials
    /// opened before then carry no stamp.
    now_ms: Option<u64>,
    /// Queue of errors deferred until the next
    /// [`take_pending_error`](AisReassembler::take_pending_error)
    /// call — today that's `ReassemblyTimeout` raised by slot or
    /// time eviction. Multiple simultaneous evictions are preserved
    /// one per call.
    pending_errors: VecDeque<AisError>,
}

/// What ties the fragments of one message together.
#[derive(Debug)]
enum PartialKey {
    /// The AIVDM header: `(channel, sequential_id)`.
    Header {
        channel: Option<u8>,
        sequential_id: u8,
    },
    /// The TAG block group: the `s` parameter, if seen, and the `g` id.
    Group { source: Option<Vec<u8>>, id: u32 },
}

/// A fragment that has arrived and waits for the rest of its message.
#[derive(Debug)]
struct Fragment {
    payload: Vec<u8>,
    fill_bits: u8,
    is_own_ship: bool,
    /// Kept for fragment number 1 only: the message reports the TAG
    /// block of its first sentence.
    tag_block: Option<OwnedTagBlock>,
}

impl Fragment {
    fn new(header: &AivdmHeader<'_>, tag_block: Option<&TagBlock<'_>>) -> Self {
        Self {
            payload: header.payload.to_vec(),
            fill_bits: header.fill_bits,
            is_own_ship: header.is_own_ship,
            tag_block: tag_block
                .filter(|_| header.fragment_number == 1)
                .map(TagBlock::to_owned_block),
        }
    }
}

#[derive(Debug)]
struct PartialMessage {
    key: PartialKey,
    fragment_count: u8,
    /// One slot per fragment, indexed by fragment number minus one.
    fragments: Vec<Option<Fragment>>,
    /// The reassembler's clock when the last fragment was seen.
    /// `None` means the partial was opened before the first
    /// [`tick`](AisReassembler::tick) and is **not** subject to time
    /// eviction — bounded-slots still applies.
    last_touched_ms: Option<u64>,
}

impl PartialMessage {
    fn new(key: PartialKey, fragment_count: u8, now_ms: Option<u64>) -> Self {
        Self {
            key,
            fragment_count,
            fragments: (0..fragment_count).map(|_| None).collect(),
            last_touched_ms: now_ms,
        }
    }

    /// Whether this partial belongs to the `(channel, sequential_id)`
    /// reassembly key. Every header lookup goes through here so the two
    /// channels can never alias each other's sequential-id space.
    fn has_header_key(&self, channel: Option<u8>, sequential_id: u8) -> bool {
        matches!(
            self.key,
            PartialKey::Header { channel: c, sequential_id: id }
                if c == channel && id == sequential_id
        )
    }

    /// Whether this partial is the open group `id` of `fragment_count`
    /// fragments, from a source that is compatible with `source`. A
    /// missing source on either side is compatible with any source.
    fn is_group(&self, id: u32, fragment_count: u8, source: Option<&[u8]>) -> bool {
        let PartialKey::Group {
            source: own_source,
            id: own_id,
        } = &self.key
        else {
            return false;
        };
        *own_id == id
            && self.fragment_count == fragment_count
            && match (own_source.as_deref(), source) {
                (Some(own), Some(other)) => own == other,
                _ => true,
            }
    }

    fn filled(&self) -> usize {
        self.fragments.iter().flatten().count()
    }

    fn has_fragment(&self, fragment_number: u8) -> bool {
        self.slot(fragment_number).is_some_and(Option::is_some)
    }

    fn slot(&self, fragment_number: u8) -> Option<&Option<Fragment>> {
        self.fragments
            .get(usize::from(fragment_number).checked_sub(1)?)
    }

    fn insert(&mut self, fragment_number: u8, fragment: Fragment, now_ms: Option<u64>) {
        let slot = usize::from(fragment_number)
            .checked_sub(1)
            .and_then(|i| self.fragments.get_mut(i));
        if let Some(slot) = slot {
            *slot = Some(fragment);
        }
        self.last_touched_ms = now_ms;
    }

    fn is_complete(&self) -> bool {
        self.fragments.iter().all(Option::is_some)
    }

    /// Join the fragments. Call only when
    /// [`is_complete`](Self::is_complete).
    fn assemble(self) -> ReassembledPayload {
        let mut assembled = ReassembledPayload {
            payload: Vec::new(),
            fill_bits: 0,
            is_own_ship: false,
            tag_block: None,
        };
        for (position, fragment) in self.fragments.into_iter().flatten().enumerate() {
            assembled.payload.extend_from_slice(&fragment.payload);
            assembled.fill_bits = fragment.fill_bits;
            if position == 0 {
                assembled.is_own_ship = fragment.is_own_ship;
                assembled.tag_block = fragment.tag_block;
            }
        }
        assembled
    }
}

impl Default for AisReassembler {
    fn default() -> Self {
        Self::new()
    }
}

impl AisReassembler {
    /// Construct a reassembler with [`DEFAULT_MAX_PARTIALS`] slots.
    #[must_use]
    pub fn new() -> Self {
        Self::with_max_partials(DEFAULT_MAX_PARTIALS)
    }

    /// Construct a reassembler with a caller-specified maximum number
    /// of concurrent partials. `max_partials` must be at least 1; a
    /// value of 0 is treated as 1 to keep the reassembler functional.
    #[must_use]
    pub fn with_max_partials(max_partials: usize) -> Self {
        Self {
            partials: Vec::new(),
            max_partials: max_partials.max(1),
            timeout_ms: None,
            now_ms: None,
            pending_errors: VecDeque::new(),
        }
    }

    /// Construct a reassembler with both bounded-slots eviction AND
    /// a clock-based timeout policy. Callers advance the clock with
    /// [`tick`](Self::tick) before feeding fragments.
    ///
    /// 60 000 ms (60 s) is a typical value. The
    /// library itself never calls a clock — time is the caller's
    /// responsibility, which keeps the crate `#![no_std]` and
    /// sans-I/O.
    #[must_use]
    pub fn with_timeout_ms(max_partials: usize, timeout_ms: u64) -> Self {
        Self {
            partials: Vec::new(),
            max_partials: max_partials.max(1),
            timeout_ms: Some(timeout_ms),
            now_ms: None,
            pending_errors: VecDeque::new(),
        }
    }

    /// Maximum number of in-flight partials this reassembler will
    /// hold before evicting the oldest.
    #[must_use]
    pub fn max_partials(&self) -> usize {
        self.max_partials
    }

    /// Current time-based eviction threshold, in monotonic
    /// milliseconds. `None` means disabled.
    #[must_use]
    pub fn timeout_ms(&self) -> Option<u64> {
        self.timeout_ms
    }

    /// Replace the time-based eviction threshold. Pass `None` to
    /// disable time eviction; stamped partials keep their stamps but
    /// no tick will expire them until a new timeout is set.
    pub fn set_timeout_ms(&mut self, timeout_ms: Option<u64>) {
        self.timeout_ms = timeout_ms;
    }

    /// Number of partials currently held. Mostly useful for tests and
    /// diagnostics.
    #[must_use]
    pub fn in_flight(&self) -> usize {
        self.partials.len()
    }

    /// Pop the next deferred error from the queue — today that's
    /// [`AisError::ReassemblyTimeout`] raised by slot-eviction or by
    /// [`tick`](Self::tick). The parser wrapper calls this between
    /// fragments so evictions are surfaced in order.
    pub fn take_pending_error(&mut self) -> Option<AisError> {
        self.pending_errors.pop_front()
    }

    /// Feed an AIVDM/AIVDO header into the reassembler, keyed on the
    /// AIVDM header. Same as
    /// [`feed_fragment_tagged`](Self::feed_fragment_tagged) with no
    /// TAG block.
    ///
    /// Returns:
    /// - `Ok(Some(payload))` — a complete armored payload is ready.
    /// - `Ok(None)` — fragment accepted; still waiting for more.
    /// - `Err(..)` — the fragment is malformed or violates the
    ///   reassembly contract; no complete payload was emitted.
    pub fn feed_fragment(
        &mut self,
        header: &AivdmHeader<'_>,
    ) -> Result<Option<ReassembledPayload>, AisError> {
        self.feed_fragment_tagged(header, None)
    }

    /// Feed an AIVDM/AIVDO header and the TAG block of its sentence.
    ///
    /// When the TAG block has a valid `g` parameter, the fragment is
    /// keyed on the TAG group and may arrive out of order; see the
    /// module documentation. Otherwise this behaves as
    /// [`feed_fragment`](Self::feed_fragment). Either way, the
    /// [`ReassembledPayload`] carries the TAG block of the first
    /// fragment.
    ///
    /// The partial it opens or appends to is stamped with the clock
    /// as of the last [`tick`](Self::tick). Before the first tick
    /// there is no stamp, and bounded-slots eviction is the only
    /// mechanism that can retire such a partial.
    pub fn feed_fragment_tagged(
        &mut self,
        header: &AivdmHeader<'_>,
        tag_block: Option<&TagBlock<'_>>,
    ) -> Result<Option<ReassembledPayload>, AisError> {
        // Validate the fragment-numbering invariants first — these
        // checks apply to single and multi-fragment messages alike.
        if header.fragment_count == 0
            || header.fragment_number == 0
            || header.fragment_number > header.fragment_count
        {
            return Err(AisError::MalformedWrapper);
        }

        // Fast path: a self-contained single-fragment message.
        if header.fragment_count == 1 {
            return Ok(Some(ReassembledPayload {
                payload: header.payload.to_vec(),
                fill_bits: header.fill_bits,
                is_own_ship: header.is_own_ship,
                tag_block: tag_block.map(TagBlock::to_owned_block),
            }));
        }

        let fragment = Fragment::new(header, tag_block);
        match tag_block.and_then(|tags| group_id(header, tags)) {
            Some(id) => {
                let source = tag_block.and_then(TagBlock::source);
                Ok(self.feed_grouped(header, id, source, fragment))
            }
            None => self.feed_in_order(header, fragment),
        }
    }

    /// Advance the clock to `now_ms` and evict any partial whose
    /// last-touched stamp is older than
    /// [`timeout_ms`](Self::timeout_ms). Each evicted partial queues
    /// one [`AisError::ReassemblyTimeout`] for retrieval via
    /// [`take_pending_error`](Self::take_pending_error). Fragments
    /// fed after this call are stamped with `now_ms`.
    ///
    /// Partials opened before the first tick have no stamp and are
    /// immune. Eviction is a no-op when `timeout_ms` is `None`; the
    /// clock still advances.
    pub fn tick(&mut self, now_ms: u64) {
        self.now_ms = Some(now_ms);
        let Some(timeout) = self.timeout_ms else {
            return;
        };
        let before = self.partials.len();
        self.partials.retain(|p| match p.last_touched_ms {
            Some(t) => now_ms.saturating_sub(t) <= timeout,
            None => true,
        });
        let evicted = before - self.partials.len();
        for _ in 0..evicted {
            self.pending_errors.push_back(AisError::ReassemblyTimeout);
        }
    }

    /// Feed a multi-fragment fragment keyed on the AIVDM header. The
    /// fragments of a message must arrive in order.
    fn feed_in_order(
        &mut self,
        header: &AivdmHeader<'_>,
        fragment: Fragment,
    ) -> Result<Option<ReassembledPayload>, AisError> {
        // Multi-fragment: sequential_id is required.
        let seq_id = header.sequential_id.ok_or(AisError::MalformedWrapper)?;

        if header.fragment_number == 1 {
            // If there's already a partial on the same key, replace it
            // silently — the old one got interrupted by a fresh fragment 1.
            // This matches real-world behavior when a multi-sentence
            // message is truncated and the transmitter immediately starts
            // a new one.
            self.partials
                .retain(|p| !p.has_header_key(header.channel, seq_id));
            let key = PartialKey::Header {
                channel: header.channel,
                sequential_id: seq_id,
            };
            self.open_partial(key, header, fragment);
            return Ok(None);
        }

        // Channels A and B carry independent sequential-id spaces, so
        // a partial on the other channel is simply not ours; a
        // fragment with no partial on its own key is out of order.
        let Some(idx) = self
            .partials
            .iter()
            .position(|p| p.has_header_key(header.channel, seq_id))
        else {
            return Err(AisError::ReassemblyOutOfOrder);
        };
        let in_order = self.partials.get(idx).is_some_and(|partial| {
            partial.fragment_count == header.fragment_count
                && partial.filled().checked_add(1) == Some(usize::from(header.fragment_number))
        });
        if !in_order {
            let _ = self.partials.remove(idx);
            return Err(AisError::ReassemblyOutOfOrder);
        }
        Ok(self.fill(idx, header.fragment_number, fragment))
    }

    /// Feed a multi-fragment fragment keyed on its TAG group. Never
    /// fails: a fragment out of order waits in its slot.
    fn feed_grouped(
        &mut self,
        header: &AivdmHeader<'_>,
        id: u32,
        source: Option<&[u8]>,
        fragment: Fragment,
    ) -> Option<ReassembledPayload> {
        let number = header.fragment_number;
        let count = header.fragment_count;

        let open = self
            .partials
            .iter()
            .position(|p| p.is_group(id, count, source));
        let free = self
            .partials
            .iter()
            .position(|p| p.is_group(id, count, source) && !p.has_fragment(number));

        if let Some(idx) = free {
            self.learn_source(idx, source);
            return self.fill(idx, number, fragment);
        }
        if let Some(idx) = open {
            // The part arrived twice: the old group was interrupted
            // and a new one has started.
            let _ = self.partials.remove(idx);
        }
        let key = PartialKey::Group {
            source: source.map(<[u8]>::to_vec),
            id,
        };
        self.open_partial(key, header, fragment);
        None
    }

    /// Record the source of a group that did not know it yet.
    fn learn_source(&mut self, idx: usize, source: Option<&[u8]>) {
        let Some(PartialMessage {
            key: PartialKey::Group { source: own, .. },
            ..
        }) = self.partials.get_mut(idx)
        else {
            return;
        };
        if own.is_none() {
            *own = source.map(<[u8]>::to_vec);
        }
    }

    /// Start a partial with its first arrived fragment.
    fn open_partial(&mut self, key: PartialKey, header: &AivdmHeader<'_>, fragment: Fragment) {
        // Evict if full.
        if self.partials.len() >= self.max_partials {
            // Drop the oldest slot (index 0 is the longest-standing).
            // Safe: len >= max_partials >= 1.
            let _ = self.partials.remove(0);
            self.pending_errors.push_back(AisError::ReassemblyTimeout);
        }

        let mut partial = PartialMessage::new(key, header.fragment_count, self.now_ms);
        partial.insert(header.fragment_number, fragment, self.now_ms);
        self.partials.push(partial);
    }

    /// Store `fragment` in the partial at `idx` and return the joined
    /// payload if that completes it.
    fn fill(
        &mut self,
        idx: usize,
        fragment_number: u8,
        fragment: Fragment,
    ) -> Option<ReassembledPayload> {
        let partial = self.partials.get_mut(idx)?;
        // Refresh the age stamp. A partial opened before the first
        // tick picks up a stamp on its first append after one.
        partial.insert(fragment_number, fragment, self.now_ms);
        if partial.is_complete() {
            return Some(self.partials.remove(idx).assemble());
        }
        None
    }
}

/// The TAG group id of a fragment, if its `g` parameter is valid for
/// the fragment: the part and total match the AIVDM header.
fn group_id(header: &AivdmHeader<'_>, tags: &TagBlock<'_>) -> Option<u32> {
    let group = tags.group()?;
    (group.part == u32::from(header.fragment_number)
        && group.total == u32::from(header.fragment_count))
    .then_some(group.id)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn header(
        frag_count: u8,
        frag_num: u8,
        seq_id: Option<u8>,
        channel: Option<u8>,
        payload: &[u8],
        fill_bits: u8,
    ) -> AivdmHeader<'_> {
        AivdmHeader {
            is_own_ship: false,
            fragment_count: frag_count,
            fragment_number: frag_num,
            sequential_id: seq_id,
            channel,
            payload,
            fill_bits,
        }
    }

    // -----------------------------------------------------------------
    // Single-fragment fast path
    // -----------------------------------------------------------------

    #[test]
    fn single_fragment_returns_immediately() {
        let mut r = AisReassembler::new();
        let h = header(1, 1, None, Some(b'A'), b"HELLO", 2);
        let out = r.feed_fragment(&h).unwrap().unwrap();
        assert_eq!(out.payload, b"HELLO");
        assert_eq!(out.fill_bits, 2);
        assert_eq!(r.in_flight(), 0);
    }

    #[test]
    fn single_fragment_propagates_is_own_ship() {
        let mut r = AisReassembler::new();
        let mut h = header(1, 1, None, Some(b'A'), b"X", 0);
        h.is_own_ship = true;
        let out = r.feed_fragment(&h).unwrap().unwrap();
        assert!(out.is_own_ship);
    }

    // -----------------------------------------------------------------
    // Two-fragment in-order happy path
    // -----------------------------------------------------------------

    #[test]
    fn two_fragments_in_order_complete() {
        let mut r = AisReassembler::new();
        assert!(r
            .feed_fragment(&header(2, 1, Some(3), Some(b'A'), b"AAA", 0))
            .unwrap()
            .is_none());
        assert_eq!(r.in_flight(), 1);
        let out = r
            .feed_fragment(&header(2, 2, Some(3), Some(b'A'), b"BBB", 2))
            .unwrap()
            .unwrap();
        assert_eq!(out.payload, b"AAABBB");
        assert_eq!(out.fill_bits, 2);
        assert_eq!(r.in_flight(), 0);
    }

    #[test]
    fn three_fragments_in_order_complete() {
        let mut r = AisReassembler::new();
        for (n, frag) in [(1u8, &b"AA"[..]), (2, &b"BB"[..])] {
            let h = header(3, n, Some(5), Some(b'A'), frag, 0);
            assert!(r.feed_fragment(&h).unwrap().is_none());
        }
        let out = r
            .feed_fragment(&header(3, 3, Some(5), Some(b'A'), b"CC", 4))
            .unwrap()
            .unwrap();
        assert_eq!(out.payload, b"AABBCC");
        assert_eq!(out.fill_bits, 4);
    }

    // -----------------------------------------------------------------
    // Out-of-order handling
    // -----------------------------------------------------------------

    #[test]
    fn out_of_order_yields_error_and_discards_partial() {
        let mut r = AisReassembler::new();
        assert!(r
            .feed_fragment(&header(3, 1, Some(1), Some(b'A'), b"AAA", 0))
            .unwrap()
            .is_none());
        // Skip fragment 2, jump to 3.
        match r.feed_fragment(&header(3, 3, Some(1), Some(b'A'), b"CCC", 0)) {
            Err(AisError::ReassemblyOutOfOrder) => {}
            other => panic!("expected ReassemblyOutOfOrder, got {other:?}"),
        }
        assert_eq!(r.in_flight(), 0);
    }

    #[test]
    fn fragment_without_prior_first_is_out_of_order() {
        let mut r = AisReassembler::new();
        match r.feed_fragment(&header(2, 2, Some(9), Some(b'A'), b"X", 0)) {
            Err(AisError::ReassemblyOutOfOrder) => {}
            other => panic!("expected ReassemblyOutOfOrder, got {other:?}"),
        }
    }

    #[test]
    fn inconsistent_fragment_count_is_out_of_order() {
        let mut r = AisReassembler::new();
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"A", 0))
            .unwrap();
        match r.feed_fragment(&header(3, 2, Some(1), Some(b'A'), b"B", 0)) {
            Err(AisError::ReassemblyOutOfOrder) => {}
            other => panic!("expected ReassemblyOutOfOrder, got {other:?}"),
        }
        assert_eq!(r.in_flight(), 0);
    }

    // -----------------------------------------------------------------
    // Same sequential id on the other channel is a different key
    // -----------------------------------------------------------------

    #[test]
    fn fragment_on_other_channel_is_out_of_order_and_keeps_partial() {
        let mut r = AisReassembler::new();
        r.feed_fragment(&header(2, 1, Some(4), Some(b'A'), b"A", 0))
            .unwrap();
        // Fragment 2 on B has no partial on (B, 4) — it is stray.
        match r.feed_fragment(&header(2, 2, Some(4), Some(b'B'), b"B", 0)) {
            Err(AisError::ReassemblyOutOfOrder) => {}
            other => panic!("expected ReassemblyOutOfOrder, got {other:?}"),
        }
        // The partial on (A, 4) is untouched and still completes.
        assert_eq!(r.in_flight(), 1);
        let out = r
            .feed_fragment(&header(2, 2, Some(4), Some(b'A'), b"a", 0))
            .unwrap()
            .unwrap();
        assert_eq!(out.payload, b"Aa");
        assert_eq!(r.in_flight(), 0);
    }

    // -----------------------------------------------------------------
    // Interleaving on different sequential IDs
    // -----------------------------------------------------------------

    #[test]
    fn interleaved_messages_on_different_seq_ids_both_complete() {
        let mut r = AisReassembler::new();
        // Two multi-fragment messages in flight, fragments arriving
        // interleaved: (1,seq=3), (1,seq=5), (2,seq=3), (2,seq=5).
        assert!(r
            .feed_fragment(&header(2, 1, Some(3), Some(b'A'), b"A3a", 0))
            .unwrap()
            .is_none());
        assert!(r
            .feed_fragment(&header(2, 1, Some(5), Some(b'A'), b"A5a", 0))
            .unwrap()
            .is_none());
        assert_eq!(r.in_flight(), 2);
        let m3 = r
            .feed_fragment(&header(2, 2, Some(3), Some(b'A'), b"A3b", 0))
            .unwrap()
            .unwrap();
        assert_eq!(m3.payload, b"A3aA3b");
        let m5 = r
            .feed_fragment(&header(2, 2, Some(5), Some(b'A'), b"A5b", 0))
            .unwrap()
            .unwrap();
        assert_eq!(m5.payload, b"A5aA5b");
        assert_eq!(r.in_flight(), 0);
    }

    // -----------------------------------------------------------------
    // Same sequential id on different channels: treated as separate
    // partials (both complete independently).
    // -----------------------------------------------------------------

    #[test]
    fn interleaved_messages_on_different_channels_both_complete() {
        let mut r = AisReassembler::new();
        // First fragment on A.
        r.feed_fragment(&header(2, 1, Some(7), Some(b'A'), b"Aa", 0))
            .unwrap();
        // First fragment on B with same seq_id — legitimately independent.
        r.feed_fragment(&header(2, 1, Some(7), Some(b'B'), b"Ba", 0))
            .unwrap();
        assert_eq!(r.in_flight(), 2);
        let from_a = r
            .feed_fragment(&header(2, 2, Some(7), Some(b'A'), b"Ab", 0))
            .unwrap()
            .unwrap();
        let from_b = r
            .feed_fragment(&header(2, 2, Some(7), Some(b'B'), b"Bb", 0))
            .unwrap()
            .unwrap();
        assert_eq!(from_a.payload, b"AaAb");
        assert_eq!(from_b.payload, b"BaBb");
    }

    #[test]
    fn same_sequence_id_on_both_channels_completes_both() {
        // Regression: the second fragments arrive in the opposite order
        // to the first ones — (1,A), (1,B), (2,B), (2,A). A seq-id-only
        // lookup matched B's second fragment against A's partial, raised
        // a channel mismatch, and dropped both messages.
        let mut r = AisReassembler::new();
        assert!(r
            .feed_fragment(&header(2, 1, Some(7), Some(b'A'), b"Aa", 0))
            .unwrap()
            .is_none());
        assert!(r
            .feed_fragment(&header(2, 1, Some(7), Some(b'B'), b"Ba", 0))
            .unwrap()
            .is_none());
        assert_eq!(r.in_flight(), 2);
        let from_b = r
            .feed_fragment(&header(2, 2, Some(7), Some(b'B'), b"Bb", 0))
            .unwrap()
            .unwrap();
        assert_eq!(r.in_flight(), 1);
        let from_a = r
            .feed_fragment(&header(2, 2, Some(7), Some(b'A'), b"Ab", 0))
            .unwrap()
            .unwrap();
        assert_eq!(from_b.payload, b"BaBb");
        assert_eq!(from_a.payload, b"AaAb");
        assert_eq!(r.in_flight(), 0);
        assert!(r.take_pending_error().is_none());
    }

    // -----------------------------------------------------------------
    // Missing sequential_id on multi-fragment → MalformedWrapper
    // -----------------------------------------------------------------

    #[test]
    fn multi_fragment_without_seq_id_is_malformed() {
        let mut r = AisReassembler::new();
        match r.feed_fragment(&header(2, 1, None, Some(b'A'), b"X", 0)) {
            Err(AisError::MalformedWrapper) => {}
            other => panic!("expected MalformedWrapper, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------
    // Malformed fragment numbering
    // -----------------------------------------------------------------

    #[test]
    fn zero_fragment_count_is_malformed() {
        let mut r = AisReassembler::new();
        match r.feed_fragment(&header(0, 1, None, Some(b'A'), b"X", 0)) {
            Err(AisError::MalformedWrapper) => {}
            other => panic!("expected MalformedWrapper, got {other:?}"),
        }
    }

    #[test]
    fn zero_fragment_number_is_malformed() {
        let mut r = AisReassembler::new();
        match r.feed_fragment(&header(2, 0, Some(1), Some(b'A'), b"X", 0)) {
            Err(AisError::MalformedWrapper) => {}
            other => panic!("expected MalformedWrapper, got {other:?}"),
        }
    }

    #[test]
    fn fragment_number_beyond_count_is_malformed() {
        let mut r = AisReassembler::new();
        match r.feed_fragment(&header(2, 3, Some(1), Some(b'A'), b"X", 0)) {
            Err(AisError::MalformedWrapper) => {}
            other => panic!("expected MalformedWrapper, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------
    // Eviction surfaces ReassemblyTimeout on the next error poll
    // -----------------------------------------------------------------

    #[test]
    fn bounded_slots_evict_oldest_and_queue_timeout() {
        let mut r = AisReassembler::with_max_partials(2);
        // Fill the two slots with first-fragments on different seq_ids.
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.feed_fragment(&header(2, 1, Some(2), Some(b'A'), b"b", 0))
            .unwrap();
        assert_eq!(r.in_flight(), 2);
        assert!(r.take_pending_error().is_none());

        // A third first-fragment triggers eviction of the oldest.
        r.feed_fragment(&header(2, 1, Some(3), Some(b'A'), b"c", 0))
            .unwrap();
        assert_eq!(r.in_flight(), 2);

        // The eviction was reported.
        match r.take_pending_error() {
            Some(AisError::ReassemblyTimeout) => {}
            other => panic!("expected ReassemblyTimeout, got {other:?}"),
        }
        // Only one timeout is queued.
        assert!(r.take_pending_error().is_none());
    }

    // -----------------------------------------------------------------
    // Restarted-in-flight: fragment 1 for the same key replaces
    // any prior partial silently (real AIS behavior when an interrupted
    // multi-sentence message is followed by a fresh one).
    // -----------------------------------------------------------------

    #[test]
    fn duplicate_fragment_one_restarts_partial() {
        let mut r = AisReassembler::new();
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"old", 0))
            .unwrap();
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"new", 0))
            .unwrap();
        let out = r
            .feed_fragment(&header(2, 2, Some(1), Some(b'A'), b"tail", 0))
            .unwrap()
            .unwrap();
        assert_eq!(out.payload, b"newtail");
    }

    // -----------------------------------------------------------------
    // with_max_partials(0) clamps to 1 — still functional
    // -----------------------------------------------------------------

    #[test]
    fn zero_max_partials_clamps_to_one() {
        let mut r = AisReassembler::with_max_partials(0);
        assert_eq!(r.max_partials(), 1);
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        assert_eq!(r.in_flight(), 1);
    }

    // -----------------------------------------------------------------
    // Clock-based timeout: tick, then feed
    // -----------------------------------------------------------------

    #[test]
    fn tick_expires_partial_past_timeout() {
        let mut r = AisReassembler::with_timeout_ms(16, 60_000);
        r.tick(1_000);
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        assert_eq!(r.in_flight(), 1);

        // 60.001 s later — still within timeout window.
        r.tick(1_000 + 60_000);
        assert_eq!(r.in_flight(), 1);
        assert!(r.take_pending_error().is_none());

        // One ms past timeout — expire.
        r.tick(1_000 + 60_001);
        assert_eq!(r.in_flight(), 0);
        match r.take_pending_error() {
            Some(AisError::ReassemblyTimeout) => {}
            other => panic!("expected ReassemblyTimeout, got {other:?}"),
        }
    }

    #[test]
    fn tick_expiring_multiple_partials_queues_multiple_timeouts() {
        let mut r = AisReassembler::with_timeout_ms(16, 1_000);
        r.tick(100);
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.tick(200);
        r.feed_fragment(&header(2, 1, Some(2), Some(b'A'), b"b", 0))
            .unwrap();
        r.tick(300);
        r.feed_fragment(&header(2, 1, Some(3), Some(b'A'), b"c", 0))
            .unwrap();
        assert_eq!(r.in_flight(), 3);

        // At t = 1500, all three are past their timeout (age > 1000).
        r.tick(1_500);
        assert_eq!(r.in_flight(), 0);
        // One ReassemblyTimeout per expired partial — the queue
        // preserves the count rather than collapsing to one signal.
        assert!(matches!(
            r.take_pending_error(),
            Some(AisError::ReassemblyTimeout)
        ));
        assert!(matches!(
            r.take_pending_error(),
            Some(AisError::ReassemblyTimeout)
        ));
        assert!(matches!(
            r.take_pending_error(),
            Some(AisError::ReassemblyTimeout)
        ));
        assert!(r.take_pending_error().is_none());
    }

    #[test]
    fn tick_without_timeout_set_is_noop() {
        let mut r = AisReassembler::new(); // no timeout
        r.tick(0);
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.tick(u64::MAX); // the end of time — should still not expire.
        assert_eq!(r.in_flight(), 1);
        assert!(r.take_pending_error().is_none());
    }

    #[test]
    fn partials_opened_before_the_first_tick_are_immune() {
        let mut r = AisReassembler::with_timeout_ms(16, 100);
        // No tick yet — no stamp.
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.tick(1_000_000);
        assert_eq!(r.in_flight(), 1);
        assert!(r.take_pending_error().is_none());
    }

    #[test]
    fn append_after_a_tick_stamps_a_partial_opened_before_it() {
        let mut r = AisReassembler::with_timeout_ms(16, 100);
        r.feed_fragment(&header(3, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.tick(1_000);
        r.feed_fragment(&header(3, 2, Some(1), Some(b'A'), b"b", 0))
            .unwrap();
        // Stamped at 1_000 on the append; 101 ms later it is evicted.
        r.tick(1_101);
        assert_eq!(r.in_flight(), 0);
        assert!(matches!(
            r.take_pending_error(),
            Some(AisError::ReassemblyTimeout)
        ));
    }

    #[test]
    fn append_refreshes_partial_timestamp() {
        // A multi-fragment message that arrives slowly: fragment 1 at
        // t=0, fragment 2 at t=500 (inside the 1000ms timeout). The
        // append must refresh the stamp so a later tick(t=1200)
        // computes age against 500, not 0.
        let mut r = AisReassembler::with_timeout_ms(16, 1_000);
        r.tick(0);
        r.feed_fragment(&header(3, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.tick(500);
        r.feed_fragment(&header(3, 2, Some(1), Some(b'A'), b"b", 0))
            .unwrap();

        // At t=1200 age-from-last-touch is 700 < 1000 — still alive.
        r.tick(1_200);
        assert_eq!(r.in_flight(), 1);
        assert!(r.take_pending_error().is_none());

        // Fragment 3 completes the message.
        r.tick(1_300);
        let out = r
            .feed_fragment(&header(3, 3, Some(1), Some(b'A'), b"c", 2))
            .unwrap()
            .unwrap();
        assert_eq!(out.payload, b"abc");
    }

    #[test]
    fn set_timeout_ms_toggles_eviction_policy() {
        let mut r = AisReassembler::new();
        assert_eq!(r.timeout_ms(), None);
        r.set_timeout_ms(Some(100));
        assert_eq!(r.timeout_ms(), Some(100));
        r.set_timeout_ms(None);
        assert_eq!(r.timeout_ms(), None);
    }

    // -----------------------------------------------------------------
    // Slot eviction still queues per-eviction timeouts (now via VecDeque).
    // -----------------------------------------------------------------

    #[test]
    fn slot_eviction_queues_one_timeout_per_eviction() {
        let mut r = AisReassembler::with_max_partials(1);
        // Three first-fragments on different seq_ids → two evictions.
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"a", 0))
            .unwrap();
        r.feed_fragment(&header(2, 1, Some(2), Some(b'A'), b"b", 0))
            .unwrap();
        r.feed_fragment(&header(2, 1, Some(3), Some(b'A'), b"c", 0))
            .unwrap();
        // Two evictions; two distinct timeout signals queued.
        assert!(matches!(
            r.take_pending_error(),
            Some(AisError::ReassemblyTimeout)
        ));
        assert!(matches!(
            r.take_pending_error(),
            Some(AisError::ReassemblyTimeout)
        ));
        assert!(r.take_pending_error().is_none());
    }

    // -----------------------------------------------------------------
    // TAG group keying
    // -----------------------------------------------------------------

    fn grouped(
        r: &mut AisReassembler,
        tag: &[u8],
        frag_count: u8,
        frag_num: u8,
        payload: &[u8],
    ) -> Option<ReassembledPayload> {
        let tags = TagBlock::parse(tag);
        r.feed_fragment_tagged(
            &header(frag_count, frag_num, Some(1), Some(b'A'), payload, 0),
            Some(&tags),
        )
        .unwrap()
    }

    #[test]
    fn group_completes_in_any_order() {
        let mut r = AisReassembler::new();
        assert!(grouped(&mut r, b"g:3-3-1", 3, 3, b"cc").is_none());
        assert!(grouped(&mut r, b"s:x,g:1-3-1", 3, 1, b"aa").is_none());
        let out = grouped(&mut r, b"g:2-3-1", 3, 2, b"bb").unwrap();
        assert_eq!(out.payload, b"aabbcc");
        assert_eq!(r.in_flight(), 0);
    }

    #[test]
    fn group_takes_fill_bits_from_the_last_part() {
        let mut r = AisReassembler::new();
        let tags = TagBlock::parse(b"g:2-2-1");
        let last = header(2, 2, Some(1), Some(b'A'), b"bb", 4);
        assert!(r
            .feed_fragment_tagged(&last, Some(&tags))
            .unwrap()
            .is_none());
        let first = header(2, 1, Some(1), Some(b'A'), b"aa", 0);
        let tags = TagBlock::parse(b"g:1-2-1");
        let out = r
            .feed_fragment_tagged(&first, Some(&tags))
            .unwrap()
            .unwrap();
        assert_eq!(out.fill_bits, 4);
    }

    #[test]
    fn group_does_not_need_a_sequential_id() {
        let mut r = AisReassembler::new();
        let tags = TagBlock::parse(b"g:1-2-1");
        let first = header(2, 1, None, None, b"aa", 0);
        assert!(r
            .feed_fragment_tagged(&first, Some(&tags))
            .unwrap()
            .is_none());
    }

    #[test]
    fn different_group_ids_do_not_mix() {
        let mut r = AisReassembler::new();
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"a1").is_none());
        assert!(grouped(&mut r, b"s:x,g:1-2-2", 2, 1, b"a2").is_none());
        assert_eq!(
            grouped(&mut r, b"s:x,g:2-2-2", 2, 2, b"b2")
                .unwrap()
                .payload,
            b"a2b2"
        );
        assert_eq!(
            grouped(&mut r, b"s:x,g:2-2-1", 2, 2, b"b1")
                .unwrap()
                .payload,
            b"a1b1"
        );
    }

    #[test]
    fn different_sources_with_the_same_id_do_not_mix() {
        let mut r = AisReassembler::new();
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"xa").is_none());
        assert!(grouped(&mut r, b"s:y,g:1-2-1", 2, 1, b"ya").is_none());
        assert_eq!(
            grouped(&mut r, b"s:y,g:2-2-1", 2, 2, b"yb")
                .unwrap()
                .payload,
            b"yayb"
        );
        assert_eq!(
            grouped(&mut r, b"s:x,g:2-2-1", 2, 2, b"xb")
                .unwrap()
                .payload,
            b"xaxb"
        );
    }

    #[test]
    fn same_id_with_a_different_total_is_another_group() {
        let mut r = AisReassembler::new();
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"aa").is_none());
        assert!(grouped(&mut r, b"s:x,g:1-3-1", 3, 1, b"cc").is_none());
        assert_eq!(r.in_flight(), 2);
    }

    #[test]
    fn repeated_part_restarts_the_group() {
        let mut r = AisReassembler::new();
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"old").is_none());
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"new").is_none());
        assert_eq!(r.in_flight(), 1);
        let out = grouped(&mut r, b"s:x,g:2-2-1", 2, 2, b"end").unwrap();
        assert_eq!(out.payload, b"newend");
    }

    #[test]
    fn group_and_header_keys_do_not_alias() {
        let mut r = AisReassembler::new();
        // A header-keyed partial on (A, 1) and a group on id 1.
        r.feed_fragment(&header(2, 1, Some(1), Some(b'A'), b"hh", 0))
            .unwrap();
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"gg").is_none());
        assert_eq!(r.in_flight(), 2);
        let out = r
            .feed_fragment(&header(2, 2, Some(1), Some(b'A'), b"HH", 0))
            .unwrap()
            .unwrap();
        assert_eq!(out.payload, b"hhHH");
    }

    #[test]
    fn group_partials_share_the_slot_limit_and_the_timeout() {
        let mut r = AisReassembler::with_timeout_ms(1, 10);
        r.tick(0);
        assert!(grouped(&mut r, b"s:x,g:1-2-1", 2, 1, b"aa").is_none());
        assert!(grouped(&mut r, b"s:x,g:1-2-2", 2, 1, b"bb").is_none());
        assert_eq!(r.in_flight(), 1);
        assert_eq!(r.take_pending_error(), Some(AisError::ReassemblyTimeout));
        r.tick(11);
        assert_eq!(r.in_flight(), 0);
        assert_eq!(r.take_pending_error(), Some(AisError::ReassemblyTimeout));
    }

    #[test]
    fn first_fragment_tag_block_is_reported_with_or_without_a_group() {
        let mut r = AisReassembler::new();
        let tags = TagBlock::parse(b"s:one,c:5");
        let first = header(2, 1, Some(1), Some(b'A'), b"aa", 0);
        r.feed_fragment_tagged(&first, Some(&tags)).unwrap();
        let second = header(2, 2, Some(1), Some(b'A'), b"bb", 0);
        let other = TagBlock::parse(b"s:two");
        let out = r
            .feed_fragment_tagged(&second, Some(&other))
            .unwrap()
            .unwrap();
        let reported = out.tag_block.unwrap();
        assert_eq!(reported.as_tag_block().source(), Some(b"one".as_slice()));
    }

    #[test]
    fn single_fragment_passes_its_tag_block_through() {
        let mut r = AisReassembler::new();
        let tags = TagBlock::parse(b"s:one");
        let h = header(1, 1, None, None, b"aa", 0);
        let out = r.feed_fragment_tagged(&h, Some(&tags)).unwrap().unwrap();
        assert_eq!(
            out.tag_block.unwrap().as_tag_block().source(),
            Some(b"one".as_slice())
        );
        assert!(r.feed_fragment(&h).unwrap().unwrap().tag_block.is_none());
    }
}
