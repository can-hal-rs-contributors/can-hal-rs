//! PCAN driver and channel builder.

use std::ffi::CString;
use std::marker::PhantomData;
use std::sync::Arc;

use can_hal::SamplePoint;

use crate::channel::PcanChannel;
use crate::error::{check_status, PcanError};
use crate::ffi;
use crate::library::PcanLibrary;
use crate::mode::{Classic, Fd, FdExplicit, Initial};

/// PCAN-Basic controller clock in Hz.
///
/// All FD-capable PCAN devices use an 80 MHz CAN clock. Exposed so that
/// callers constructing explicit [`PcanFdTiming`] values can verify their
/// `(brp, tseg1, tseg2)` choices satisfy
/// `bit_rate = PCAN_CLOCK_HZ / (brp * (1 + tseg1 + tseg2))`.
pub const PCAN_CLOCK_HZ: u32 = 80_000_000;

/// Default nominal sample point used when [`PcanChannelBuilder::sample_point`]
/// is not called.
const DEFAULT_NOMINAL_SAMPLE_POINT: SamplePoint = SamplePoint::NOMINAL_DEFAULT;

/// Default data-phase sample point used when
/// [`PcanChannelBuilder::data_sample_point`] is not called.
const DEFAULT_DATA_SAMPLE_POINT: SamplePoint = SamplePoint::DATA_DEFAULT;

/// Preferred total TQ count for the nominal phase. The solver biases toward
/// this value among solutions with equal sample-point error, since it matches
/// the timing used by other backends in this workspace.
const PREFERRED_NOMINAL_TQ: u32 = 20;

/// Preferred total TQ count for the data phase.
const PREFERRED_DATA_TQ: u32 = 10;

/// SJW upper bound for default-derived timing. Matches the values used by
/// the existing hardware tests and prevents picking aggressive SJW values
/// that some controllers reject. The escape hatch [`PcanFdTiming`] is
/// available when a larger SJW is needed.
const DEFAULT_SJW_CAP: u32 = 4;

// PCAN-Basic FD timing-segment limits (TPCANBitrateFD string fields).
const NOMINAL_MAX_BRP: u32 = 1024;
const NOMINAL_MAX_TSEG1: u32 = 256;
const NOMINAL_MAX_TSEG2: u32 = 128;
const DATA_MAX_BRP: u32 = 1024;
const DATA_MAX_TSEG1: u32 = 32;
const DATA_MAX_TSEG2: u32 = 16;

/// Bus type for selecting a PCAN hardware family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PcanBusType {
    Usb,
    Pci,
    Lan,
}

/// Predefined classic-CAN bitrates supported by `CAN_Initialize`.
///
/// PCAN-Basic's classic-CAN init only accepts a fixed set of baud-rate
/// constants. Modeling them as an enum makes invalid bitrates
/// unrepresentable at compile time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ClassicBitrate {
    Br1M,
    Br800K,
    Br500K,
    Br250K,
    Br125K,
    Br100K,
    Br50K,
    Br20K,
    Br10K,
    Br5K,
}

impl ClassicBitrate {
    const fn as_pcan(self) -> u16 {
        match self {
            Self::Br1M => ffi::PCAN_BAUD_1M,
            Self::Br800K => ffi::PCAN_BAUD_800K,
            Self::Br500K => ffi::PCAN_BAUD_500K,
            Self::Br250K => ffi::PCAN_BAUD_250K,
            Self::Br125K => ffi::PCAN_BAUD_125K,
            Self::Br100K => ffi::PCAN_BAUD_100K,
            Self::Br50K => ffi::PCAN_BAUD_50K,
            Self::Br20K => ffi::PCAN_BAUD_20K,
            Self::Br10K => ffi::PCAN_BAUD_10K,
            Self::Br5K => ffi::PCAN_BAUD_5K,
        }
    }
}

/// PCAN-Basic driver - factory for opening CAN channels.
///
/// The default [`channel`](PcanDriver::channel) method opens a USB channel.
/// For PCI or LAN channels, use
/// [`channel_on_bus`](PcanDriver::channel_on_bus).
///
/// # Example
///
/// ```rust,ignore
/// use can_hal_pcan::{PcanDriver, ClassicBitrate};
///
/// let driver = PcanDriver::new()?;
/// let mut channel = driver
///     .channel(0)?
///     .classic(ClassicBitrate::Br500K)
///     .connect()?;
/// ```
pub struct PcanDriver {
    lib: Arc<PcanLibrary>,
}

impl PcanDriver {
    /// Create a new PCAN driver by loading the PCAN-Basic library from the
    /// default system path (`PCANBasic.dll` on Windows, `libpcanbasic.so` on
    /// Linux).
    pub fn new() -> Result<Self, PcanError> {
        let lib = PcanLibrary::load()?;
        Ok(Self { lib })
    }

    /// Create a new PCAN driver by loading the library from a custom path.
    pub fn with_library_path(path: &str) -> Result<Self, PcanError> {
        let lib = PcanLibrary::load_from(path)?;
        Ok(Self { lib })
    }

    /// Begin configuring a USB channel by 0-based index.
    pub fn channel(&self, index: u32) -> Result<PcanChannelBuilder<Initial>, PcanError> {
        self.channel_on_bus(PcanBusType::Usb, index)
    }

    /// Begin configuring a channel on a specific bus type.
    ///
    /// `index` is 0-based (0 through 15).
    pub fn channel_on_bus(
        &self,
        bus_type: PcanBusType,
        index: u32,
    ) -> Result<PcanChannelBuilder<Initial>, PcanError> {
        let bus_code = match bus_type {
            PcanBusType::Usb => 0,
            PcanBusType::Pci => 1,
            PcanBusType::Lan => 2,
        };
        // Channel index is 0..=15, so u32 -> u16 truncation cannot happen.
        #[allow(clippy::cast_possible_truncation)]
        let handle =
            ffi::pcan_handle(bus_code, index as u16).ok_or(PcanError::InvalidChannel(index))?;

        Ok(PcanChannelBuilder {
            lib: self.lib.clone(),
            handle,
            state: Initial,
            _mode: PhantomData,
        })
    }
}

/// Per-phase CAN FD timing parameters for the PCAN 80 MHz clock.
///
/// `bit_rate = 80_000_000 / (brp * (1 + tseg1 + tseg2))`
///
/// `sample_point = (1 + tseg1) / (1 + tseg1 + tseg2)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcanPhaseTiming {
    pub brp: u32,
    pub tseg1: u32,
    pub tseg2: u32,
    pub sjw: u32,
}

/// Hardware-level CAN FD timing configuration for both phases at the PCAN
/// 80 MHz clock. Pass to [`PcanChannelBuilder::fd_explicit`] when the
/// solver's output doesn't fit, for example to raise the SJW above the
/// solver's default cap of 4.
///
/// ```rust,ignore
/// use can_hal_pcan::{PcanDriver, PcanFdTiming, PcanPhaseTiming};
///
/// // 500 kbit/s nominal at 70%, 4 Mbit/s data at 80%, with nominal SJW=6
/// // (the solver caps at 4).
/// let timing = PcanFdTiming {
///     nominal: PcanPhaseTiming { brp: 8, tseg1: 13, tseg2: 6, sjw: 6 },
///     data:    PcanPhaseTiming { brp: 2, tseg1: 7,  tseg2: 2, sjw: 2 },
/// };
/// let channel = PcanDriver::new()?.channel(0)?.fd_explicit(timing).connect()?;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcanFdTiming {
    pub nominal: PcanPhaseTiming,
    pub data: PcanPhaseTiming,
}

impl PcanFdTiming {
    pub(crate) fn to_pcan_string(self) -> String {
        format!(
            "f_clock_mhz=80, \
             nom_brp={}, nom_tseg1={}, nom_tseg2={}, nom_sjw={}, \
             data_brp={}, data_tseg1={}, data_tseg2={}, data_sjw={}",
            self.nominal.brp,
            self.nominal.tseg1,
            self.nominal.tseg2,
            self.nominal.sjw,
            self.data.brp,
            self.data.tseg1,
            self.data.tseg2,
            self.data.sjw,
        )
    }
}

/// Phase selector used in solver error messages.
#[derive(Debug, Clone, Copy)]
enum TimingPhase {
    Nominal,
    Data,
}

impl TimingPhase {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Nominal => "nominal",
            Self::Data => "data",
        }
    }
}

/// Solve for a single phase of FD timing at the PCAN 80 MHz clock.
///
/// Searches for an exact-bitrate `(brp, tseg1, tseg2)` whose sample point is
/// closest to the requested value. Ties are broken by proximity to the
/// phase's preferred TQ count for cross-adapter interop.
///
/// Returns `None` if `bitrate_hz` is zero, does not evenly divide 80 MHz,
/// or if no `(brp, total_tq)` factoring lands inside the segment-length
/// constraints.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn solve_phase_timing(
    bitrate_hz: u32,
    sample_point: SamplePoint,
    phase: TimingPhase,
) -> Option<PcanPhaseTiming> {
    if bitrate_hz == 0 {
        return None;
    }
    if PCAN_CLOCK_HZ % bitrate_hz != 0 {
        return None;
    }
    let sample_point_f = sample_point.as_fraction();
    let divisor = PCAN_CLOCK_HZ / bitrate_hz;

    let (max_tseg1, max_tseg2, max_brp, preferred_tq) = match phase {
        TimingPhase::Nominal => (
            NOMINAL_MAX_TSEG1,
            NOMINAL_MAX_TSEG2,
            NOMINAL_MAX_BRP,
            PREFERRED_NOMINAL_TQ,
        ),
        TimingPhase::Data => (
            DATA_MAX_TSEG1,
            DATA_MAX_TSEG2,
            DATA_MAX_BRP,
            PREFERRED_DATA_TQ,
        ),
    };
    let max_total_tq = 1 + max_tseg1 + max_tseg2;

    let mut best: Option<(PcanPhaseTiming, f32)> = None;

    let mut total_tq = 3u32;
    while total_tq <= max_total_tq && total_tq <= divisor {
        if divisor % total_tq != 0 {
            total_tq += 1;
            continue;
        }
        let brp = divisor / total_tq;
        if brp == 0 || brp > max_brp {
            total_tq += 1;
            continue;
        }

        let tseg1_plus_one_f = (total_tq as f32) * sample_point_f;
        let tseg1_plus_one = tseg1_plus_one_f.round() as i64;
        let tseg1 = tseg1_plus_one - 1;
        let tseg2 = i64::from(total_tq) - 1 - tseg1;

        if tseg1 < 1 || tseg1 > i64::from(max_tseg1) || tseg2 < 1 || tseg2 > i64::from(max_tseg2) {
            total_tq += 1;
            continue;
        }
        let tseg1 = tseg1 as u32;
        let tseg2 = tseg2 as u32;

        let actual_sp = (1 + tseg1) as f32 / total_tq as f32;
        let sp_error = (actual_sp - sample_point_f).abs();
        let tq_distance = (i64::from(total_tq) - i64::from(preferred_tq)).unsigned_abs() as u32;
        let score = sp_error + (tq_distance as f32) * 0.001;

        // ISO 11898-1 requires SJW <= min(tseg1, tseg2). Cap by tseg1 too so
        // the solver never emits a value the hardware will reject.
        let sjw = tseg1.min(tseg2).min(DEFAULT_SJW_CAP);
        let timing = PcanPhaseTiming {
            brp,
            tseg1,
            tseg2,
            sjw,
        };

        if best
            .as_ref()
            .map_or(true, |(_, best_score)| score < *best_score)
        {
            best = Some((timing, score));
        }

        total_tq += 1;
    }

    best.map(|(t, _)| t)
}

/// Typestate-driven builder for a PCAN channel.
///
/// The type parameter `Mode` tracks which configuration path the builder is
/// on. Start with [`Initial`] (returned by [`PcanDriver::channel`]) and
/// transition to either [`Classic`] (via [`classic`](Self::classic)) or
/// [`Fd`] (via [`fd`](Self::fd)). Only the methods valid for the current
/// state are available, so calling - for example - `data_sample_point` on a
/// classic-configured builder is a compile error rather than a runtime no-op.
pub struct PcanChannelBuilder<Mode> {
    lib: Arc<PcanLibrary>,
    handle: u16,
    state: Mode,
    _mode: PhantomData<Mode>,
}

impl PcanChannelBuilder<Initial> {
    /// Configure the channel for classic CAN at one of PCAN-Basic's
    /// predefined bitrates. Infallible - invalid bitrates are not
    /// representable.
    ///
    /// # Sample point
    ///
    /// PCAN-Basic's classic-CAN initialization (`CAN_Initialize`) only
    /// accepts a fixed set of baud-rate constants and provides no API to
    /// customize timing segments - there is no sample-point control in
    /// classic mode. If you need a custom sample point, use [`fd`](Self::fd)
    /// at the same nominal and data bitrate (`fd(rate, rate)`) so the
    /// timing solver is engaged via `CAN_InitializeFD`. Note this requires
    /// FD-capable PCAN hardware.
    #[must_use]
    pub fn classic(self, bitrate: ClassicBitrate) -> PcanChannelBuilder<Classic> {
        PcanChannelBuilder {
            lib: self.lib,
            handle: self.handle,
            state: Classic::new(bitrate.as_pcan()),
            _mode: PhantomData,
        }
    }

    /// Configure the channel for CAN FD with the given nominal and data
    /// bitrates (both in Hz). Returns `Err` immediately if either bitrate
    /// does not evenly divide the 80 MHz PCAN clock, so problems surface at
    /// the call site instead of `connect()`. Sample points default to
    /// [`SamplePoint::NOMINAL_DEFAULT`] (70%) and [`SamplePoint::DATA_DEFAULT`]
    /// (80%); override on the returned builder.
    pub fn fd(self, nominal_hz: u32, data_hz: u32) -> Result<PcanChannelBuilder<Fd>, PcanError> {
        if nominal_hz == 0 || PCAN_CLOCK_HZ % nominal_hz != 0 {
            return Err(PcanError::UnsupportedBitrate(nominal_hz));
        }
        if data_hz == 0 || PCAN_CLOCK_HZ % data_hz != 0 {
            return Err(PcanError::UnsupportedBitrate(data_hz));
        }
        Ok(PcanChannelBuilder {
            lib: self.lib,
            handle: self.handle,
            state: Fd::new(
                nominal_hz,
                data_hz,
                DEFAULT_NOMINAL_SAMPLE_POINT,
                DEFAULT_DATA_SAMPLE_POINT,
            ),
            _mode: PhantomData,
        })
    }

    /// Configure the channel for CAN FD with explicit per-segment timing.
    ///
    /// This bypasses the solver entirely - `(brp, tseg1, tseg2, sjw)` are
    /// taken from `timing` as-is, the bitrates are implicit in the segment
    /// math (`80 MHz / (brp * (1 + tseg1 + tseg2))`), and the returned
    /// builder has no sample-point setters (only [`connect`](Self::connect)).
    /// Use this when you need control beyond what the solver provides
    /// (e.g., unusually large SJW values or a non-standard bitrate that
    /// doesn't divide 80 MHz evenly).
    #[must_use]
    pub fn fd_explicit(self, timing: PcanFdTiming) -> PcanChannelBuilder<FdExplicit> {
        PcanChannelBuilder {
            lib: self.lib,
            handle: self.handle,
            state: FdExplicit::new(timing),
            _mode: PhantomData,
        }
    }
}

impl PcanChannelBuilder<Classic> {
    /// Finalize configuration and go on-bus in classic-CAN mode.
    pub fn connect(self) -> Result<PcanChannel<Classic>, PcanError> {
        // Plug & Play hardware (USB, PCI, LAN): hw_type=0, io_port=0, interrupt=0
        // SAFETY: initialize() was loaded from PCANBasic.
        // self.handle is a valid PCAN channel handle and the bitrate constant is a valid PCAN_BAUD_* value.
        let status =
            unsafe { (self.lib.initialize)(self.handle, self.state.bitrate_const(), 0, 0, 0) };
        check_status(status)?;
        finalize_channel(self.lib, self.handle)
    }
}

impl PcanChannelBuilder<Fd> {
    /// Override the nominal-phase sample point. Defaults to
    /// [`SamplePoint::NOMINAL_DEFAULT`] (70%) when not called.
    #[must_use]
    pub fn sample_point(mut self, sample_point: SamplePoint) -> Self {
        self.state.set_sample_point(sample_point);
        self
    }

    /// Override the data-phase sample point. Defaults to
    /// [`SamplePoint::DATA_DEFAULT`] (80%) when not called.
    #[must_use]
    pub fn data_sample_point(mut self, sample_point: SamplePoint) -> Self {
        self.state.set_data_sample_point(sample_point);
        self
    }

    /// Finalize configuration and go on-bus in CAN FD mode.
    pub fn connect(self) -> Result<PcanChannel<Fd>, PcanError> {
        let nominal = solve_phase_timing(
            self.state.nominal_hz(),
            self.state.sample_point(),
            TimingPhase::Nominal,
        )
        .ok_or_else(|| {
            PcanError::UnsupportedTiming(format!(
                "no PCAN FD {} timing satisfies bitrate={} Hz, sample_point={}",
                TimingPhase::Nominal.as_str(),
                self.state.nominal_hz(),
                self.state.sample_point().as_fraction(),
            ))
        })?;
        let data = solve_phase_timing(
            self.state.data_hz(),
            self.state.data_sample_point(),
            TimingPhase::Data,
        )
        .ok_or_else(|| {
            PcanError::UnsupportedTiming(format!(
                "no PCAN FD {} timing satisfies bitrate={} Hz, sample_point={}",
                TimingPhase::Data.as_str(),
                self.state.data_hz(),
                self.state.data_sample_point().as_fraction(),
            ))
        })?;
        let timing = PcanFdTiming { nominal, data };

        initialize_fd_with_timing(&self.lib, self.handle, timing)?;
        finalize_channel(self.lib, self.handle)
    }
}

impl PcanChannelBuilder<FdExplicit> {
    /// Finalize configuration and go on-bus in CAN FD mode with the
    /// previously supplied explicit timing.
    pub fn connect(self) -> Result<PcanChannel<Fd>, PcanError> {
        initialize_fd_with_timing(&self.lib, self.handle, self.state.timing())?;
        finalize_channel(self.lib, self.handle)
    }
}

/// Construct the receive event for an already-initialized PCAN channel and
/// wrap it in [`PcanChannel`]. If event construction fails, uninitialize the
/// channel to avoid leaking the hardware handle.
fn finalize_channel<Mode>(
    lib: Arc<PcanLibrary>,
    handle: u16,
) -> Result<PcanChannel<Mode>, PcanError> {
    match crate::event::ReceiveEvent::new(lib.clone(), handle) {
        Ok(event) => Ok(PcanChannel::new(lib, handle, event)),
        Err(err) => {
            // SAFETY: handle was just successfully initialized via
            // CAN_Initialize{,FD}. Clean up to avoid leaking the channel.
            unsafe {
                let _ = (lib.uninitialize)(handle);
            }
            Err(err)
        }
    }
}

/// Apply a fully-specified `PcanFdTiming` via `CAN_InitializeFD`. Shared
/// between the solver-driven `<Fd>` and explicit `<FdExplicit>` connect
/// paths.
fn initialize_fd_with_timing(
    lib: &PcanLibrary,
    handle: u16,
    timing: PcanFdTiming,
) -> Result<(), PcanError> {
    let timing_string = timing.to_pcan_string();
    let c_timing = CString::new(timing_string.as_str())
        .map_err(|_| PcanError::InvalidFrame("timing string contains null byte".into()))?;
    // SAFETY: initialize_fd() was loaded from PCANBasic.
    // handle is a valid PCAN channel handle and c_timing is a valid null-terminated C string.
    let status = unsafe { (lib.initialize_fd)(handle, c_timing.as_ptr()) };
    check_status(status)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_bitrate_maps_to_pcan_constants() {
        assert_eq!(ClassicBitrate::Br500K.as_pcan(), ffi::PCAN_BAUD_500K);
        assert_eq!(ClassicBitrate::Br1M.as_pcan(), ffi::PCAN_BAUD_1M);
        assert_eq!(ClassicBitrate::Br5K.as_pcan(), ffi::PCAN_BAUD_5K);
    }

    #[test]
    fn solver_nominal_500k_default_sample_point() {
        let t = solve_phase_timing(500_000, SamplePoint::NOMINAL_DEFAULT, TimingPhase::Nominal)
            .unwrap();
        assert_eq!(t.brp, 8);
        assert_eq!(t.tseg1, 13);
        assert_eq!(t.tseg2, 6);
        assert_eq!(t.sjw, 4);
    }

    #[test]
    fn solver_nominal_1m_default_sample_point() {
        let t = solve_phase_timing(
            1_000_000,
            SamplePoint::NOMINAL_DEFAULT,
            TimingPhase::Nominal,
        )
        .unwrap();
        assert_eq!(t.brp, 4);
        assert_eq!(t.tseg1, 13);
        assert_eq!(t.tseg2, 6);
        assert_eq!(t.sjw, 4);
    }

    #[test]
    fn solver_nominal_250k_default_sample_point() {
        let t = solve_phase_timing(250_000, SamplePoint::NOMINAL_DEFAULT, TimingPhase::Nominal)
            .unwrap();
        assert_eq!(t.brp, 16);
        assert_eq!(t.tseg1, 13);
        assert_eq!(t.tseg2, 6);
    }

    #[test]
    fn solver_data_4m_default_sample_point() {
        let t =
            solve_phase_timing(4_000_000, SamplePoint::DATA_DEFAULT, TimingPhase::Data).unwrap();
        assert_eq!(t.brp, 2);
        assert_eq!(t.tseg1, 7);
        assert_eq!(t.tseg2, 2);
        assert_eq!(t.sjw, 2);
    }

    #[test]
    fn solver_data_2m_default_sample_point() {
        let t =
            solve_phase_timing(2_000_000, SamplePoint::DATA_DEFAULT, TimingPhase::Data).unwrap();
        assert_eq!(t.brp, 4);
        assert_eq!(t.tseg1, 7);
        assert_eq!(t.tseg2, 2);
    }

    #[test]
    fn solver_data_1m_default_sample_point() {
        let t =
            solve_phase_timing(1_000_000, SamplePoint::DATA_DEFAULT, TimingPhase::Data).unwrap();
        assert_eq!(t.brp, 8);
        assert_eq!(t.tseg1, 7);
        assert_eq!(t.tseg2, 2);
    }

    #[test]
    fn solver_nominal_500k_custom_sample_point() {
        let t = solve_phase_timing(500_000, SamplePoint::PCT_87_5, TimingPhase::Nominal).unwrap();
        assert_eq!(t.brp, 10);
        assert_eq!(t.tseg1, 13);
        assert_eq!(t.tseg2, 2);
    }

    #[test]
    fn solver_nominal_500k_85_percent() {
        let t = solve_phase_timing(
            500_000,
            SamplePoint::from_per_mille(850),
            TimingPhase::Nominal,
        )
        .unwrap();
        assert_eq!(t.brp, 8);
        assert_eq!(t.tseg1, 16);
        assert_eq!(t.tseg2, 3);
        assert_eq!(t.sjw, 3);
    }

    #[test]
    fn solver_rejects_non_divisible_bitrate() {
        assert!(
            solve_phase_timing(333_000, SamplePoint::NOMINAL_DEFAULT, TimingPhase::Nominal)
                .is_none()
        );
    }

    #[test]
    fn solver_rejects_zero_bitrate() {
        assert!(
            solve_phase_timing(0, SamplePoint::NOMINAL_DEFAULT, TimingPhase::Nominal).is_none()
        );
    }

    #[test]
    fn fd_timing_to_string_matches_legacy_format() {
        let timing = PcanFdTiming {
            nominal: PcanPhaseTiming {
                brp: 8,
                tseg1: 13,
                tseg2: 6,
                sjw: 4,
            },
            data: PcanPhaseTiming {
                brp: 2,
                tseg1: 7,
                tseg2: 2,
                sjw: 2,
            },
        };
        assert_eq!(
            timing.to_pcan_string(),
            "f_clock_mhz=80, \
             nom_brp=8, nom_tseg1=13, nom_tseg2=6, nom_sjw=4, \
             data_brp=2, data_tseg1=7, data_tseg2=2, data_sjw=2"
        );
    }
}
