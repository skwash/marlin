//! Base station report and UTC/date response — AIS message Types 4 and 11.
//!
//! 168-bit payload (ITU-R M.1371-5 Annex 8 §3.2 and §3.10, Table 49).
//! Type 11 is the answer of a mobile station to a UTC/date inquiry
//! (Type 10) and has the layout of Type 4, so one struct and one decoder
//! serve both; the message type survives on [`AisMessageBody`]'s variant.
//!
//! [`AisMessageBody`]: crate::AisMessageBody

use marlin_field::FieldState;

use crate::shared_types::{read_epfd, whole_u16, whole_u8, Partition, LATITUDE, LONGITUDE};
use crate::{AisError, BitReader, EpfdType};

/// Minimum valid payload size for Types 4 and 11 (ITU-R M.1371-5 Annex 8
/// §3.2, Table 49).
pub const BASE_STATION_REPORT_BITS: usize = 168;

/// UTC year, 14 bits: 0 not available, 1..=9999 defined, 10000..=16383
/// invalid.
const UTC_YEAR: Partition<u16> = Partition {
    not_available: 0,
    over_range: None,
    defined: 1..=9999,
    scale: whole_u16,
};

/// UTC month, 4 bits: 0 not available, 1..=12 defined, 13..=15 invalid.
const UTC_MONTH: Partition<u8> = Partition {
    not_available: 0,
    over_range: None,
    defined: 1..=12,
    scale: whole_u8,
};

/// UTC day, 5 bits: 0 not available, 1..=31 defined.
const UTC_DAY: Partition<u8> = Partition {
    not_available: 0,
    over_range: None,
    defined: 1..=31,
    scale: whole_u8,
};

/// UTC hour, 5 bits: 24 not available, 0..=23 defined, 25..=31 invalid.
const UTC_HOUR: Partition<u8> = Partition {
    not_available: 24,
    over_range: None,
    defined: 0..=23,
    scale: whole_u8,
};

/// UTC minute or second, 6 bits: 60 not available, 0..=59 defined,
/// 61..=63 invalid.
const UTC_MINUTE_OR_SECOND: Partition<u8> = Partition {
    not_available: 60,
    over_range: None,
    defined: 0..=59,
    scale: whole_u8,
};

/// Decoded base station report (Type 4) or UTC/date response (Type 11).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BaseStationReport {
    /// Maritime Mobile Service Identity.
    pub mmsi: u32,
    /// UTC year. Not available on `0`; invalid with the raw code above
    /// `9999`.
    pub year: FieldState<u16>,
    /// UTC month, 1..=12. Not available on `0`.
    pub month: FieldState<u8>,
    /// UTC day of the month, 1..=31. Not available on `0`.
    pub day: FieldState<u8>,
    /// UTC hour, 0..=23. Not available on `24`.
    pub hour: FieldState<u8>,
    /// UTC minute, 0..=59. Not available on `60`.
    pub minute: FieldState<u8>,
    /// UTC second, 0..=59. Not available on `60`.
    pub second: FieldState<u8>,
    /// Position accuracy flag: `true` for DGNSS-corrected fixes.
    pub position_accuracy: bool,
    /// Longitude in signed decimal degrees. Not available on `181°`;
    /// invalid with the raw code on any other code beyond ±180°.
    pub longitude_deg: FieldState<f64>,
    /// Latitude in signed decimal degrees. Not available on `91°`;
    /// invalid with the raw code on any other code beyond ±90°.
    pub latitude_deg: FieldState<f64>,
    /// Type of electronic position-fixing device.
    pub epfd: FieldState<EpfdType>,
    /// RAIM (Receiver Autonomous Integrity Monitoring) flag.
    pub raim: bool,
    /// Radio status field — 19 bits, layout not decoded.
    pub radio_status: u32,
}

/// Decode a Type 4 base station report or a Type 11 UTC/date response.
///
/// # Errors
///
/// [`AisError::PayloadTooShort`] if `total_bits < 168`. Bits past 168
/// are ignored.
#[allow(clippy::cast_possible_truncation)] // every narrowing is masked to its field width
pub fn decode_base_station_report(
    bits: &[u8],
    total_bits: usize,
) -> Result<BaseStationReport, AisError> {
    if total_bits < BASE_STATION_REPORT_BITS {
        return Err(AisError::PayloadTooShort);
    }
    let mut r = BitReader::new(bits, total_bits);
    let _ = r.u(6); // msg_type
    let _ = r.u(2); // repeat
    let mmsi = (r.u(30) & 0xFFFF_FFFF) as u32;
    let year = UTC_YEAR.read(r.u(14));
    let month = UTC_MONTH.read(r.u(4));
    let day = UTC_DAY.read(r.u(5));
    let hour = UTC_HOUR.read(r.u(5));
    let minute = UTC_MINUTE_OR_SECOND.read(r.u(6));
    let second = UTC_MINUTE_OR_SECOND.read(r.u(6));
    let position_accuracy = r.b();
    let longitude_deg = LONGITUDE.read_signed(r.i(28));
    let latitude_deg = LATITUDE.read_signed(r.i(27));
    let epfd = read_epfd(&mut r);
    let _ = r.u(10); // spare
    let raim = r.b();
    let radio_status = (r.u(19) & 0x7_FFFF) as u32;

    Ok(BaseStationReport {
        mmsi,
        year,
        month,
        day,
        hour,
        minute,
        second,
        position_accuracy,
        longitude_deg,
        latitude_deg,
        epfd,
        raim,
        radio_status,
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
    use crate::shared_types::sentinel;
    use crate::testing::{undefined, BitWriter};

    /// Every Table 49 field a test may want to vary. The spare bits are
    /// zero.
    struct Fields {
        mmsi: u32,
        year: u16,
        month: u8,
        day: u8,
        hour: u8,
        minute: u8,
        second: u8,
        pos_acc: bool,
        lon_raw: i32,
        lat_raw: i32,
        epfd: u8,
        raim: bool,
        radio: u32,
    }

    impl Default for Fields {
        fn default() -> Self {
            Self {
                mmsi: 1,
                year: 2026,
                month: 11,
                day: 27,
                hour: 12,
                minute: 30,
                second: 15,
                pos_acc: false,
                lon_raw: 0,
                lat_raw: 0,
                epfd: 1,
                raim: false,
                radio: 0,
            }
        }
    }

    /// Pack `f` into a 168-bit Type 4 payload (Table 49).
    fn build(f: &Fields) -> (alloc::vec::Vec<u8>, usize) {
        let mut w = BitWriter::new();
        w.u(6, 4);
        w.u(2, 0);
        w.u(30, u64::from(f.mmsi));
        w.u(14, u64::from(f.year));
        w.u(4, u64::from(f.month));
        w.u(5, u64::from(f.day));
        w.u(5, u64::from(f.hour));
        w.u(6, u64::from(f.minute));
        w.u(6, u64::from(f.second));
        w.b(f.pos_acc);
        w.i(28, i64::from(f.lon_raw));
        w.i(27, i64::from(f.lat_raw));
        w.u(4, u64::from(f.epfd));
        w.u(10, 0); // spare
        w.b(f.raim);
        w.u(19, u64::from(f.radio));
        w.finish()
    }

    fn decode(f: &Fields) -> BaseStationReport {
        let (bits, total) = build(f);
        decode_base_station_report(&bits, total).unwrap()
    }

    #[test]
    fn payload_is_exactly_168_bits() {
        assert_eq!(build(&Fields::default()).1, BASE_STATION_REPORT_BITS);
    }

    #[test]
    fn decodes_happy_path() {
        let msg = decode(&Fields {
            mmsi: 3_669_702,
            year: 2007,
            month: 5,
            day: 14,
            hour: 19,
            minute: 57,
            second: 39,
            pos_acc: true,
            lon_raw: -45_811_417, // -76.3523617°
            lat_raw: 22_130_260,  // 36.8837667°
            epfd: 7,
            raim: true,
            radio: 0x5_A5A5,
        });
        assert_eq!(msg.mmsi, 3_669_702);
        assert_eq!(msg.year, FieldState::Value(2007));
        assert_eq!(msg.month, FieldState::Value(5));
        assert_eq!(msg.day, FieldState::Value(14));
        assert_eq!(msg.hour, FieldState::Value(19));
        assert_eq!(msg.minute, FieldState::Value(57));
        assert_eq!(msg.second, FieldState::Value(39));
        assert!(msg.position_accuracy);
        assert!((msg.longitude_deg.value().unwrap() + 76.352_361_7).abs() < 1e-6);
        assert!((msg.latitude_deg.value().unwrap() - 36.883_766_7).abs() < 1e-6);
        assert_eq!(msg.epfd, FieldState::Value(EpfdType::Surveyed));
        assert!(msg.raim);
        assert_eq!(msg.radio_status, 0x5_A5A5);
    }

    #[test]
    fn every_not_available_code_decodes_to_not_available() {
        let msg = decode(&Fields {
            year: 0,
            month: 0,
            day: 0,
            hour: 24,
            minute: 60,
            second: 60,
            lon_raw: sentinel::LON_NOT_AVAILABLE as i32,
            lat_raw: sentinel::LAT_NOT_AVAILABLE as i32,
            epfd: 0,
            ..Fields::default()
        });
        assert_eq!(msg.year, FieldState::NotAvailable);
        assert_eq!(msg.month, FieldState::NotAvailable);
        assert_eq!(msg.day, FieldState::NotAvailable);
        assert_eq!(msg.hour, FieldState::NotAvailable);
        assert_eq!(msg.minute, FieldState::NotAvailable);
        assert_eq!(msg.second, FieldState::NotAvailable);
        assert_eq!(msg.longitude_deg, FieldState::NotAvailable);
        assert_eq!(msg.latitude_deg, FieldState::NotAvailable);
        assert_eq!(msg.epfd, FieldState::NotAvailable);
    }

    #[test]
    fn date_and_time_rows() {
        let year = |raw| {
            decode(&Fields {
                year: raw,
                ..Fields::default()
            })
            .year
        };
        assert_eq!(year(1), FieldState::Value(1));
        assert_eq!(year(9999), FieldState::Value(9999));
        assert_eq!(year(10_000), FieldState::Invalid(undefined(10_000)));
        assert_eq!(year(16_383), FieldState::Invalid(undefined(16_383)));

        let month = |raw| {
            decode(&Fields {
                month: raw,
                ..Fields::default()
            })
            .month
        };
        assert_eq!(month(12), FieldState::Value(12));
        assert_eq!(month(13), FieldState::Invalid(undefined(13)));

        let day = |raw| {
            decode(&Fields {
                day: raw,
                ..Fields::default()
            })
            .day
        };
        assert_eq!(day(31), FieldState::Value(31));

        let hour = |raw| {
            decode(&Fields {
                hour: raw,
                ..Fields::default()
            })
            .hour
        };
        assert_eq!(hour(0), FieldState::Value(0));
        assert_eq!(hour(23), FieldState::Value(23));
        assert_eq!(hour(25), FieldState::Invalid(undefined(25)));

        let minute = |raw| {
            decode(&Fields {
                minute: raw,
                ..Fields::default()
            })
            .minute
        };
        assert_eq!(minute(59), FieldState::Value(59));
        assert_eq!(minute(61), FieldState::Invalid(undefined(61)));

        let second = |raw| {
            decode(&Fields {
                second: raw,
                ..Fields::default()
            })
            .second
        };
        assert_eq!(second(59), FieldState::Value(59));
        assert_eq!(second(63), FieldState::Invalid(undefined(63)));
    }

    #[test]
    fn longitude_and_latitude_rows() {
        let east = 180 * 600_000;
        let lon = |raw| {
            decode(&Fields {
                lon_raw: raw,
                ..Fields::default()
            })
            .longitude_deg
        };
        assert_eq!(lon(east), FieldState::Value(180.0));
        assert_eq!(
            lon(east + 1),
            FieldState::Invalid(undefined(i64::from(east) + 1))
        );
        let north = 90 * 600_000;
        let lat = |raw| {
            decode(&Fields {
                lat_raw: raw,
                ..Fields::default()
            })
            .latitude_deg
        };
        assert_eq!(lat(-north), FieldState::Value(-90.0));
        assert_eq!(
            lat(-north - 1),
            FieldState::Invalid(undefined(-i64::from(north) - 1))
        );
    }

    #[test]
    fn epfd_reserved_code_is_invalid() {
        let msg = decode(&Fields {
            epfd: 9,
            ..Fields::default()
        });
        assert_eq!(msg.epfd, FieldState::Invalid(undefined(9)));
    }

    #[test]
    fn longer_payload_decodes_and_ignores_the_tail() {
        let (mut bits, total) = build(&Fields {
            mmsi: 42,
            ..Fields::default()
        });
        bits.extend_from_slice(&[0xFF; 3]);
        let msg = decode_base_station_report(&bits, total + 24).unwrap();
        assert_eq!(msg.mmsi, 42);
    }

    #[test]
    fn too_short_payload_is_rejected() {
        let (bits, _) = build(&Fields::default());
        match decode_base_station_report(&bits, 167) {
            Err(AisError::PayloadTooShort) => {}
            other => panic!("expected PayloadTooShort, got {other:?}"),
        }
    }
}
