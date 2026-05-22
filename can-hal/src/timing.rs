//! CAN bit-timing types.
//!
//! These are validated value types - invalid configurations are unrepresentable
//! at the type level rather than caught at `connect()` time.

/// Bit-timing sample point as a fraction of the bit time.
///
/// Stored internally as per-mille (e.g., `875` for `87.5%`). The CAN
/// specification recommends sample points in the 70%–87.5% range; this type
/// enforces `[500, 950]` per-mille (50%–95%) so out-of-range values are
/// rejected before they reach a backend's timing solver.
///
/// Construct via [`SamplePoint::from_per_mille`] (`const fn`, so out-of-range
/// literals fail at compile time when wrapped in a `const` context), or via
/// the preset constants:
///
/// ```
/// use can_hal::SamplePoint;
///
/// // Compile-time literal (asserted at compile time when used in a const context):
/// const SP_87_5: SamplePoint = SamplePoint::from_per_mille(875);
/// assert_eq!(SP_87_5.per_mille(), 875);
///
/// // Or use a preset:
/// assert_eq!(SamplePoint::NOMINAL_DEFAULT.per_mille(), 700);
/// assert_eq!(SamplePoint::DATA_DEFAULT.per_mille(), 800);
/// ```
///
/// Out-of-range per-mille values panic - at compile time in a `const`
/// context, or at runtime otherwise:
///
/// ```compile_fail
/// # use can_hal::SamplePoint;
/// const TOO_HIGH: SamplePoint = SamplePoint::from_per_mille(2000);
/// ```
///
/// ```should_panic
/// # use can_hal::SamplePoint;
/// // Runtime construction with an out-of-range value panics.
/// SamplePoint::from_per_mille(2000);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SamplePoint(u16);

impl SamplePoint {
    /// Default nominal-phase sample point: 70%. Matches the cross-adapter
    /// interop convention used by the PCAN and Kvaser backends in this
    /// workspace.
    pub const NOMINAL_DEFAULT: Self = Self::from_per_mille(700);

    /// Default data-phase sample point: 80%. Matches the cross-adapter
    /// interop convention used by the PCAN and Kvaser backends in this
    /// workspace.
    pub const DATA_DEFAULT: Self = Self::from_per_mille(800);

    /// 75% sample point.
    pub const PCT_75: Self = Self::from_per_mille(750);

    /// 87.5% sample point - a common industrial recommendation for
    /// classic CAN at modest bus lengths.
    pub const PCT_87_5: Self = Self::from_per_mille(875);

    /// Construct from per-mille (e.g., `875` for `87.5%`, `700` for `70%`).
    ///
    /// Panics if `per_mille` is outside `[500, 950]`. In a `const` context
    /// (i.e., when initializing a `const` binding or inside `const { ... }`),
    /// this panic occurs at compile time.
    #[must_use]
    pub const fn from_per_mille(per_mille: u16) -> Self {
        assert!(
            per_mille >= 500 && per_mille <= 950,
            "sample point per-mille must be in [500, 950]"
        );
        Self(per_mille)
    }

    /// Get the per-mille value (e.g., `875` for `87.5%`).
    #[must_use]
    pub const fn per_mille(self) -> u16 {
        self.0
    }

    /// Convert to a fraction (e.g., `0.875` for `87.5%`). Not `const`
    /// because f32 division isn't `const`-stable on this crate's MSRV.
    #[must_use]
    pub fn as_fraction(self) -> f32 {
        f32::from(self.0) / 1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_default_is_70_percent() {
        assert_eq!(SamplePoint::NOMINAL_DEFAULT.per_mille(), 700);
        assert!((SamplePoint::NOMINAL_DEFAULT.as_fraction() - 0.70).abs() < 1e-6);
    }

    #[test]
    fn data_default_is_80_percent() {
        assert_eq!(SamplePoint::DATA_DEFAULT.per_mille(), 800);
        assert!((SamplePoint::DATA_DEFAULT.as_fraction() - 0.80).abs() < 1e-6);
    }

    #[test]
    fn pct_87_5_is_875_per_mille() {
        assert_eq!(SamplePoint::PCT_87_5.per_mille(), 875);
        assert!((SamplePoint::PCT_87_5.as_fraction() - 0.875).abs() < 1e-6);
    }

    #[test]
    fn from_per_mille_accepts_range_bounds() {
        assert_eq!(SamplePoint::from_per_mille(500).per_mille(), 500);
        assert_eq!(SamplePoint::from_per_mille(950).per_mille(), 950);
    }

    #[test]
    #[should_panic = "sample point per-mille must be in [500, 950]"]
    #[allow(clippy::let_underscore_must_use)]
    fn from_per_mille_rejects_too_low() {
        let _ = SamplePoint::from_per_mille(499);
    }

    #[test]
    #[should_panic = "sample point per-mille must be in [500, 950]"]
    #[allow(clippy::let_underscore_must_use)]
    fn from_per_mille_rejects_too_high() {
        let _ = SamplePoint::from_per_mille(951);
    }
}
