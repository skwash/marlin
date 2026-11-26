//! Long-range AIS broadcast message — AIS message Type 27.
//!
//! 96-bit payload (ITU-R M.1371-5 Annex 8 §3.25, Table 78) for
//! satellite reception. It carries the Class A position fields at
//! reduced resolution: position in 1/10 minute, speed in whole knots
//! and course in whole degrees. There is no rate of turn, heading or
//! timestamp.

use marlin_field::FieldState;

use crate::position_report_a::NavStatus;
use crate::shared_types::{
    coded, sentinel, LONG_RANGE_COURSE, LONG_RANGE_LATITUDE, LONG_RANGE_LONGITUDE, LONG_RANGE_SPEED,
};
use crate::{AisError, BitReader};

/// Minimum valid payload size for Type 27 (ITU-R M.1371-5 Annex 8
/// §3.25, Table 78).
pub const LONG_RANGE_POSITION_REPORT_BITS: usize = 96;

/// Decoded long-range position report.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongRangePositionReport {
    /// Maritime Mobile Service Identity.
    pub mmsi: u32,
    /// Position accuracy flag: `true` for DGNSS-corrected fixes.
    pub position_accuracy: bool,
    /// RAIM (Receiver Autonomous Integrity Monitoring) flag.
    pub raim: bool,
    /// Navigation status, with the same codes as Types 1/2/3.
    pub navigation_status: FieldState<NavStatus>,
    /// Longitude in signed decimal degrees, from a 1/10 minute code.
    /// Not available on `181°`; invalid with the raw code on any other
    /// code beyond ±180°.
    pub longitude_deg: FieldState<f64>,
    /// Latitude in signed decimal degrees, from a 1/10 minute code.
    /// Not available on `91°`; invalid with the raw code on any other
    /// code beyond ±90°.
    pub latitude_deg: FieldState<f64>,
    /// Speed over ground in whole knots (0..=62). Not available on
    /// `63`.
    pub speed_over_ground_kn: FieldState<u8>,
    /// Course over ground in whole degrees (0..=359). Not available on
    /// `511`; invalid on `360..=510`.
    pub course_over_ground: FieldState<u16>,
    /// `true` when the position is the current GNSS position, `false`
    /// when it is not (wire bit `1`).
    pub gnss_position_current: bool,
}

/// Decode a Type 27 long-range position report.
///
/// # Errors
///
/// [`AisError::PayloadTooShort`] if `total_bits < 96`. Bits past 96 are
/// ignored.
#[allow(clippy::cast_possible_truncation)] // every narrowing is masked to its field width
pub fn decode_long_range_position_report(
    bits: &[u8],
    total_bits: usize,
) -> Result<LongRangePositionReport, AisError> {
    if total_bits < LONG_RANGE_POSITION_REPORT_BITS {
        return Err(AisError::PayloadTooShort);
    }
    let mut r = BitReader::new(bits, total_bits);
    let _ = r.u(6); // msg_type
    let _ = r.u(2); // repeat
    let mmsi = (r.u(30) & 0xFFFF_FFFF) as u32;
    let position_accuracy = r.b();
    let raim = r.b();
    let navigation_status = coded(
        r.u(4),
        sentinel::NAV_STATUS_NOT_AVAILABLE,
        NavStatus::from_code,
    );
    let longitude_deg = LONG_RANGE_LONGITUDE.read_signed(r.i(18));
    let latitude_deg = LONG_RANGE_LATITUDE.read_signed(r.i(17));
    let speed_over_ground_kn = LONG_RANGE_SPEED.read(r.u(6));
    let course_over_ground = LONG_RANGE_COURSE.read(r.u(9));
    let gnss_position_current = !r.b();

    Ok(LongRangePositionReport {
        mmsi,
        position_accuracy,
        raim,
        navigation_status,
        longitude_deg,
        latitude_deg,
        speed_over_ground_kn,
        course_over_ground,
        gnss_position_current,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::cast_possible_truncation
)]
mod tests {
    use super::*;
    use crate::testing::{undefined, BitWriter};

    /// Every Table 78 field a test may want to vary. The spare bit is
    /// zero.
    struct Fields {
        mmsi: u32,
        pos_acc: bool,
        raim: bool,
        nav_status: u8,
        lon_raw: i32,
        lat_raw: i32,
        sog: u8,
        cog: u16,
        not_gnss: bool,
    }

    impl Default for Fields {
        fn default() -> Self {
            Self {
                mmsi: 1,
                pos_acc: false,
                raim: false,
                nav_status: 0,
                lon_raw: 0,
                lat_raw: 0,
                sog: 0,
                cog: 0,
                not_gnss: false,
            }
        }
    }

    /// Pack `f` into a 96-bit Type 27 payload (Table 78).
    fn build(f: &Fields) -> (alloc::vec::Vec<u8>, usize) {
        let mut w = BitWriter::new();
        w.u(6, 27);
        w.u(2, 0);
        w.u(30, u64::from(f.mmsi));
        w.b(f.pos_acc);
        w.b(f.raim);
        w.u(4, u64::from(f.nav_status));
        w.i(18, i64::from(f.lon_raw));
        w.i(17, i64::from(f.lat_raw));
        w.u(6, u64::from(f.sog));
        w.u(9, u64::from(f.cog));
        w.b(f.not_gnss);
        w.b(false); // spare
        w.finish()
    }

    fn decode(f: &Fields) -> LongRangePositionReport {
        let (bits, total) = build(f);
        decode_long_range_position_report(&bits, total).unwrap()
    }

    #[test]
    fn payload_is_exactly_96_bits() {
        assert_eq!(build(&Fields::default()).1, LONG_RANGE_POSITION_REPORT_BITS);
    }

    #[test]
    fn decodes_happy_path() {
        let msg = decode(&Fields {
            mmsi: 577_305_000,
            pos_acc: true,
            raim: true,
            nav_status: 5,
            lon_raw: -4_000, // -6.6667°
            lat_raw: 3_000,  // 5°
            sog: 12,
            cog: 223,
            not_gnss: false,
        });
        assert_eq!(msg.mmsi, 577_305_000);
        assert!(msg.position_accuracy);
        assert!(msg.raim);
        assert_eq!(msg.navigation_status, FieldState::Value(NavStatus::Moored));
        assert!((msg.longitude_deg.value().unwrap() + 4_000.0 / 600.0).abs() < 1e-9);
        assert!((msg.latitude_deg.value().unwrap() - 5.0).abs() < 1e-9);
        assert_eq!(msg.speed_over_ground_kn, FieldState::Value(12));
        assert_eq!(msg.course_over_ground, FieldState::Value(223));
        assert!(msg.gnss_position_current);
    }

    #[test]
    fn gnss_bit_one_means_position_is_not_current() {
        let msg = decode(&Fields {
            not_gnss: true,
            ..Fields::default()
        });
        assert!(!msg.gnss_position_current);
    }

    #[test]
    fn every_not_available_code_decodes_to_not_available() {
        let msg = decode(&Fields {
            nav_status: 15,
            lon_raw: sentinel::LONG_RANGE_LON_NOT_AVAILABLE as i32,
            lat_raw: sentinel::LONG_RANGE_LAT_NOT_AVAILABLE as i32,
            sog: sentinel::LONG_RANGE_SOG_NOT_AVAILABLE,
            cog: sentinel::LONG_RANGE_COG_NOT_AVAILABLE,
            ..Fields::default()
        });
        assert_eq!(msg.navigation_status, FieldState::NotAvailable);
        assert_eq!(msg.longitude_deg, FieldState::NotAvailable);
        assert_eq!(msg.latitude_deg, FieldState::NotAvailable);
        assert_eq!(msg.speed_over_ground_kn, FieldState::NotAvailable);
        assert_eq!(msg.course_over_ground, FieldState::NotAvailable);
    }

    #[test]
    fn longitude_and_latitude_rows() {
        let east = 180 * 600;
        for (raw, expected) in [
            (east, FieldState::Value(180.0)),
            (-east, FieldState::Value(-180.0)),
            (
                east + 1,
                FieldState::Invalid(undefined(i64::from(east) + 1)),
            ),
            (
                sentinel::LONG_RANGE_LON_NOT_AVAILABLE as i32,
                FieldState::NotAvailable,
            ),
        ] {
            let msg = decode(&Fields {
                lon_raw: raw,
                ..Fields::default()
            });
            assert_eq!(msg.longitude_deg, expected, "lon raw {raw}");
        }
        let north = 90 * 600;
        for (raw, expected) in [
            (north, FieldState::Value(90.0)),
            (-north, FieldState::Value(-90.0)),
            (
                -north - 1,
                FieldState::Invalid(undefined(-i64::from(north) - 1)),
            ),
            (
                sentinel::LONG_RANGE_LAT_NOT_AVAILABLE as i32,
                FieldState::NotAvailable,
            ),
        ] {
            let msg = decode(&Fields {
                lat_raw: raw,
                ..Fields::default()
            });
            assert_eq!(msg.latitude_deg, expected, "lat raw {raw}");
        }
    }

    #[test]
    fn speed_and_course_rows() {
        for (raw, expected) in [
            (0, FieldState::Value(0)),
            (62, FieldState::Value(62)),
            (
                sentinel::LONG_RANGE_SOG_NOT_AVAILABLE,
                FieldState::NotAvailable,
            ),
        ] {
            let msg = decode(&Fields {
                sog: raw,
                ..Fields::default()
            });
            assert_eq!(msg.speed_over_ground_kn, expected, "sog raw {raw}");
        }
        for (raw, expected) in [
            (0, FieldState::Value(0)),
            (359, FieldState::Value(359)),
            (360, FieldState::Invalid(undefined(360))),
            (510, FieldState::Invalid(undefined(510))),
            (
                sentinel::LONG_RANGE_COG_NOT_AVAILABLE,
                FieldState::NotAvailable,
            ),
        ] {
            let msg = decode(&Fields {
                cog: raw,
                ..Fields::default()
            });
            assert_eq!(msg.course_over_ground, expected, "cog raw {raw}");
        }
    }

    #[test]
    fn navigation_status_reserved_code_is_invalid() {
        let msg = decode(&Fields {
            nav_status: 11,
            ..Fields::default()
        });
        assert_eq!(msg.navigation_status, FieldState::Invalid(undefined(11)));
    }

    #[test]
    fn longer_payload_decodes_and_ignores_the_tail() {
        let (mut bits, total) = build(&Fields {
            mmsi: 42,
            ..Fields::default()
        });
        bits.extend_from_slice(&[0xFF; 9]);
        let msg = decode_long_range_position_report(&bits, total + 72).unwrap();
        assert_eq!(msg.mmsi, 42);
    }

    #[test]
    fn too_short_payload_is_rejected() {
        let (bits, _) = build(&Fields::default());
        match decode_long_range_position_report(&bits, 95) {
            Err(AisError::PayloadTooShort) => {}
            other => panic!("expected PayloadTooShort, got {other:?}"),
        }
    }
}
