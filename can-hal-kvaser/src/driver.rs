use std::marker::PhantomData;
use std::os::raw::c_long;
use std::sync::Arc;

use can_hal::SamplePoint;

use crate::channel::KvaserChannel;
use crate::error::{check_status, KvaserError};
use crate::event::ReceiveEvent;
use crate::ffi::{KvBusParamsTq, CAN_ERR_NOT_SUPPORTED, CAN_OPEN_CAN_FD};
use crate::library::KvaserLibrary;
use crate::mode::{Classic, ClassicExplicit, Fd, FdExplicit, Initial};

/// Assumed CAN controller clock frequency.
///
/// 80 MHz is standard on Kvaser U100, Leaf Pro HS v2, and most modern Kvaser
/// adapters. Used to compute the prescaler for the `canSetBusParamsFdTq` API.
/// Exposed so that callers constructing explicit [`BusParams`] /
/// [`BusParamsFd`] values can verify their `(tseg1, tseg2)` choices satisfy
/// `KVASER_CLOCK_HZ % (bitrate_hz * (1 + tseg1 + tseg2)) == 0`.
pub const KVASER_CLOCK_HZ: u32 = 80_000_000;

/// Default nominal sample point used when
/// [`KvaserChannelBuilder::sample_point`] is not called.
const DEFAULT_NOMINAL_SAMPLE_POINT: SamplePoint = SamplePoint::NOMINAL_DEFAULT;

/// Default data-phase sample point used when
/// [`KvaserChannelBuilder::data_sample_point`] is not called.
const DEFAULT_DATA_SAMPLE_POINT: SamplePoint = SamplePoint::DATA_DEFAULT;

/// Preferred total TQ count for the nominal phase.
const PREFERRED_NOMINAL_TQ: u32 = 20;

/// Preferred total TQ count for the data phase.
const PREFERRED_DATA_TQ: u32 = 10;

/// SJW upper bound for solver-derived timing.
const DEFAULT_SJW_CAP: u32 = 4;

// Conservative segment-length limits compatible with both the modern
// canSetBusParamsFdTq API and the legacy canSetBusParams + canSetBusParamsFd
// fallback.
const NOMINAL_MAX_TSEG1: u32 = 256;
const NOMINAL_MAX_TSEG2: u32 = 128;
const DATA_MAX_TSEG1: u32 = 32;
const DATA_MAX_TSEG2: u32 = 16;
const MAX_PRESCALER: u32 = 1024;

/// Nominal bus parameters: (tseg1, tseg2, sjw, noSamp, syncMode).
#[derive(Debug, Clone, Copy)]
pub struct BusParams {
    pub tseg1: u32,
    pub tseg2: u32,
    pub sjw: u32,
    pub no_samp: u32,
    pub sync_mode: u32,
}

/// FD data-phase bus parameters: (tseg1, tseg2, sjw).
#[derive(Debug, Clone, Copy)]
pub struct BusParamsFd {
    pub tseg1: u32,
    pub tseg2: u32,
    pub sjw: u32,
}

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

/// Solve for FD phase timing at the 80 MHz Kvaser clock.
///
/// Returns `None` if `bitrate_hz` is zero, does not evenly divide 80 MHz,
/// or if no `(prescaler, total_tq)` factoring lands inside the segment-length
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
) -> Option<(u32, u32, u32)> {
    if bitrate_hz == 0 {
        return None;
    }
    if KVASER_CLOCK_HZ % bitrate_hz != 0 {
        return None;
    }
    let sample_point_f = sample_point.as_fraction();
    let divisor = KVASER_CLOCK_HZ / bitrate_hz;

    let (max_tseg1, max_tseg2, preferred_tq) = match phase {
        TimingPhase::Nominal => (NOMINAL_MAX_TSEG1, NOMINAL_MAX_TSEG2, PREFERRED_NOMINAL_TQ),
        TimingPhase::Data => (DATA_MAX_TSEG1, DATA_MAX_TSEG2, PREFERRED_DATA_TQ),
    };
    let max_total_tq = 1 + max_tseg1 + max_tseg2;

    let mut best: Option<((u32, u32, u32), f32)> = None;

    let mut total_tq = 3u32;
    while total_tq <= max_total_tq && total_tq <= divisor {
        if divisor % total_tq != 0 {
            total_tq += 1;
            continue;
        }
        let prescaler = divisor / total_tq;
        if prescaler == 0 || prescaler > MAX_PRESCALER {
            total_tq += 1;
            continue;
        }

        let tseg1_plus_one = ((total_tq as f32) * sample_point_f).round() as i64;
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

        let sjw = tseg2.min(DEFAULT_SJW_CAP);

        if best
            .as_ref()
            .map_or(true, |(_, best_score)| score < *best_score)
        {
            best = Some(((tseg1, tseg2, sjw), score));
        }

        total_tq += 1;
    }

    best.map(|(t, _)| t)
}

fn solve_nominal(bitrate_hz: u32, sample_point: SamplePoint) -> Result<BusParams, KvaserError> {
    let (tseg1, tseg2, sjw) = solve_phase_timing(bitrate_hz, sample_point, TimingPhase::Nominal)
        .ok_or_else(|| {
            KvaserError::NotSupported(format!(
                "no Kvaser {} timing satisfies bitrate={} Hz, sample_point={}",
                TimingPhase::Nominal.as_str(),
                bitrate_hz,
                sample_point.as_fraction(),
            ))
        })?;
    Ok(BusParams {
        tseg1,
        tseg2,
        sjw,
        no_samp: 1,
        sync_mode: 0,
    })
}

fn solve_data(bitrate_hz: u32, sample_point: SamplePoint) -> Result<BusParamsFd, KvaserError> {
    let (tseg1, tseg2, sjw) = solve_phase_timing(bitrate_hz, sample_point, TimingPhase::Data)
        .ok_or_else(|| {
            KvaserError::NotSupported(format!(
                "no Kvaser {} timing satisfies bitrate={} Hz, sample_point={}",
                TimingPhase::Data.as_str(),
                bitrate_hz,
                sample_point.as_fraction(),
            ))
        })?;
    Ok(BusParamsFd { tseg1, tseg2, sjw })
}

/// Driver for KVASER CAN adapters using the CANlib API.
///
/// Loads `libcanlib.so.1` (Linux) or `canlib32.dll` (Windows) at
/// construction time.
///
/// # Example
///
/// ```rust,no_run
/// use can_hal_kvaser::KvaserDriver;
///
/// let driver = KvaserDriver::new().expect("CANlib not found");
/// let mut channel = driver
///     .channel(0)
///     .unwrap()
///     .classic(500_000)
///     .unwrap()
///     .connect()
///     .unwrap();
/// ```
pub struct KvaserDriver {
    lib: Arc<KvaserLibrary>,
}

impl KvaserDriver {
    /// Load CANlib from the default system location.
    pub fn new() -> Result<Self, KvaserError> {
        Ok(Self {
            lib: KvaserLibrary::load()?,
        })
    }

    /// Load CANlib from a custom path.
    pub fn with_library_path(path: &str) -> Result<Self, KvaserError> {
        Ok(Self {
            lib: KvaserLibrary::load_from(path)?,
        })
    }

    /// Begin configuring the channel at the given 0-based index.
    #[allow(clippy::cast_possible_wrap)] // channel index is small, fits in i32
    pub fn channel(&self, index: u32) -> Result<KvaserChannelBuilder<Initial>, KvaserError> {
        Ok(KvaserChannelBuilder {
            lib: Arc::clone(&self.lib),
            channel_index: index as i32,
            state: Initial,
            _mode: PhantomData,
        })
    }
}

/// Typestate-driven builder for a KVASER channel.
///
/// Start with [`Initial`] (returned by [`KvaserDriver::channel`]) and
/// transition to either [`Classic`] (via [`classic`](Self::classic)) or
/// [`Fd`] (via [`fd`](Self::fd)). Only the methods valid for the current
/// state are available.
pub struct KvaserChannelBuilder<Mode> {
    lib: Arc<KvaserLibrary>,
    channel_index: i32,
    state: Mode,
    _mode: PhantomData<Mode>,
}

impl KvaserChannelBuilder<Initial> {
    /// Configure the channel for classic CAN at the given bitrate (Hz).
    /// Returns `Err` if the bitrate does not evenly divide the 80 MHz
    /// CANlib clock. Sample point defaults to
    /// [`SamplePoint::NOMINAL_DEFAULT`] (70%); override on the returned
    /// builder.
    pub fn classic(self, bitrate_hz: u32) -> Result<KvaserChannelBuilder<Classic>, KvaserError> {
        if bitrate_hz == 0 || KVASER_CLOCK_HZ % bitrate_hz != 0 {
            return Err(KvaserError::NotSupported(format!(
                "classic bitrate {bitrate_hz} Hz does not divide {KVASER_CLOCK_HZ} Hz"
            )));
        }
        Ok(KvaserChannelBuilder {
            lib: self.lib,
            channel_index: self.channel_index,
            state: Classic::new(bitrate_hz, DEFAULT_NOMINAL_SAMPLE_POINT),
            _mode: PhantomData,
        })
    }

    /// Configure the channel for CAN FD with the given nominal and data
    /// bitrates (Hz). Returns `Err` if either bitrate does not evenly divide
    /// the 80 MHz CANlib clock. Sample points default to
    /// [`SamplePoint::NOMINAL_DEFAULT`] (70%) and
    /// [`SamplePoint::DATA_DEFAULT`] (80%); override on the returned builder.
    pub fn fd(
        self,
        nominal_hz: u32,
        data_hz: u32,
    ) -> Result<KvaserChannelBuilder<Fd>, KvaserError> {
        if nominal_hz == 0 || KVASER_CLOCK_HZ % nominal_hz != 0 {
            return Err(KvaserError::NotSupported(format!(
                "nominal bitrate {nominal_hz} Hz does not divide {KVASER_CLOCK_HZ} Hz"
            )));
        }
        if data_hz == 0 || KVASER_CLOCK_HZ % data_hz != 0 {
            return Err(KvaserError::NotSupported(format!(
                "data bitrate {data_hz} Hz does not divide {KVASER_CLOCK_HZ} Hz"
            )));
        }
        Ok(KvaserChannelBuilder {
            lib: self.lib,
            channel_index: self.channel_index,
            state: Fd::new(
                nominal_hz,
                data_hz,
                DEFAULT_NOMINAL_SAMPLE_POINT,
                DEFAULT_DATA_SAMPLE_POINT,
            ),
            _mode: PhantomData,
        })
    }

    /// Configure the channel for classic CAN with explicit nominal timing.
    ///
    /// Bypasses the timing solver. The returned builder has no setters; the
    /// next call must be [`connect`](KvaserChannelBuilder::connect). The
    /// `bitrate_hz` is still required because CANlib's `canSetBusParams`
    /// takes the frequency as input and derives the prescaler from
    /// `KVASER_CLOCK_HZ / (bitrate_hz * (1 + tseg1 + tseg2))`. Returns `Err`
    /// if `bitrate_hz` does not evenly divide the 80 MHz CANlib clock, so
    /// the failure surfaces here instead of at `connect()`.
    pub fn classic_explicit(
        self,
        bitrate_hz: u32,
        params: BusParams,
    ) -> Result<KvaserChannelBuilder<ClassicExplicit>, KvaserError> {
        if bitrate_hz == 0 || KVASER_CLOCK_HZ % bitrate_hz != 0 {
            return Err(KvaserError::NotSupported(format!(
                "classic bitrate {bitrate_hz} Hz does not divide {KVASER_CLOCK_HZ} Hz"
            )));
        }
        Ok(KvaserChannelBuilder {
            lib: self.lib,
            channel_index: self.channel_index,
            state: ClassicExplicit::new(bitrate_hz, params),
            _mode: PhantomData,
        })
    }

    /// Configure the channel for CAN FD with explicit nominal and data
    /// timing.
    ///
    /// Bypasses the timing solver. The returned builder has no setters; the
    /// next call must be [`connect`](KvaserChannelBuilder::connect). Returns
    /// `Err` if either bitrate does not evenly divide the 80 MHz CANlib
    /// clock.
    pub fn fd_explicit(
        self,
        nominal_hz: u32,
        data_hz: u32,
        params: BusParams,
        fd_params: BusParamsFd,
    ) -> Result<KvaserChannelBuilder<FdExplicit>, KvaserError> {
        if nominal_hz == 0 || KVASER_CLOCK_HZ % nominal_hz != 0 {
            return Err(KvaserError::NotSupported(format!(
                "nominal bitrate {nominal_hz} Hz does not divide {KVASER_CLOCK_HZ} Hz"
            )));
        }
        if data_hz == 0 || KVASER_CLOCK_HZ % data_hz != 0 {
            return Err(KvaserError::NotSupported(format!(
                "data bitrate {data_hz} Hz does not divide {KVASER_CLOCK_HZ} Hz"
            )));
        }
        Ok(KvaserChannelBuilder {
            lib: self.lib,
            channel_index: self.channel_index,
            state: FdExplicit::new(nominal_hz, data_hz, params, fd_params),
            _mode: PhantomData,
        })
    }
}

impl KvaserChannelBuilder<Classic> {
    /// Override the nominal sample point. Defaults to
    /// [`SamplePoint::NOMINAL_DEFAULT`] (70%) when not called.
    #[must_use]
    pub fn sample_point(mut self, sample_point: SamplePoint) -> Self {
        self.state.set_sample_point(sample_point);
        self
    }

    /// Finalize configuration and go on-bus in classic-CAN mode.
    pub fn connect(self) -> Result<KvaserChannel<Classic>, KvaserError> {
        let params = solve_nominal(self.state.bitrate_hz(), self.state.sample_point())?;
        open_classic(
            &self.lib,
            self.channel_index,
            self.state.bitrate_hz(),
            &params,
        )
    }
}

impl KvaserChannelBuilder<ClassicExplicit> {
    /// Finalize configuration and go on-bus with the previously supplied
    /// explicit timing.
    pub fn connect(self) -> Result<KvaserChannel<Classic>, KvaserError> {
        open_classic(
            &self.lib,
            self.channel_index,
            self.state.bitrate_hz(),
            self.state.params(),
        )
    }
}

impl KvaserChannelBuilder<Fd> {
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
    pub fn connect(self) -> Result<KvaserChannel<Fd>, KvaserError> {
        let params = solve_nominal(self.state.nominal_hz(), self.state.sample_point())?;
        let fd_params = solve_data(self.state.data_hz(), self.state.data_sample_point())?;
        open_fd(
            &self.lib,
            self.channel_index,
            self.state.nominal_hz(),
            self.state.data_hz(),
            &params,
            &fd_params,
        )
    }
}

impl KvaserChannelBuilder<FdExplicit> {
    /// Finalize configuration and go on-bus with the previously supplied
    /// explicit timing.
    pub fn connect(self) -> Result<KvaserChannel<Fd>, KvaserError> {
        open_fd(
            &self.lib,
            self.channel_index,
            self.state.nominal_hz(),
            self.state.data_hz(),
            self.state.params(),
            self.state.fd_params(),
        )
    }
}

/// Open a classic-CAN channel and go on-bus. Shared between the solver and
/// explicit paths.
#[allow(clippy::cast_lossless, clippy::cast_possible_wrap)]
fn open_classic(
    lib: &Arc<KvaserLibrary>,
    channel_index: i32,
    bitrate_hz: u32,
    params: &BusParams,
) -> Result<KvaserChannel<Classic>, KvaserError> {
    // SAFETY: canOpenChannel was loaded from canlib
    let handle = unsafe { (lib.open_channel)(channel_index, 0) };
    if handle < 0 {
        return Err(KvaserError::Canlib(crate::error::KvaserStatus(handle)));
    }

    let result = (|| {
        // SAFETY: canSetBusParams was loaded from canlib; handle is valid
        check_status(unsafe {
            (lib.set_bus_params)(
                handle,
                bitrate_hz as c_long,
                params.tseg1,
                params.tseg2,
                params.sjw,
                params.no_samp,
                params.sync_mode,
            )
        })?;
        // SAFETY: canBusOn was loaded from canlib; handle is valid
        check_status(unsafe { (lib.bus_on)(handle) })?;
        let event = ReceiveEvent::new(lib, handle)?;
        KvaserChannel::<Classic>::new(lib.clone(), handle, event)
    })();

    if result.is_err() {
        // SAFETY: canClose was loaded from canlib; handle is valid
        unsafe { (lib.close)(handle) };
    }
    result
}

/// Open a CAN FD channel and go on-bus. Shared between the solver and
/// explicit paths.
#[allow(clippy::cast_lossless, clippy::cast_possible_wrap)]
fn open_fd(
    lib: &Arc<KvaserLibrary>,
    channel_index: i32,
    nominal_hz: u32,
    data_hz: u32,
    params: &BusParams,
    fd_params: &BusParamsFd,
) -> Result<KvaserChannel<Fd>, KvaserError> {
    // SAFETY: canOpenChannel was loaded from canlib
    let handle = unsafe { (lib.open_channel)(channel_index, CAN_OPEN_CAN_FD) };
    if handle < 0 {
        return Err(KvaserError::Canlib(crate::error::KvaserStatus(handle)));
    }

    let result = (|| {
        // Try canSetBusParamsFdTq first. It sets both nominal and data-phase
        // timing in one call using explicit time quanta and works identically
        // on Windows and Linux.
        let use_legacy = if let Some(set_fd_tq) = lib.set_bus_params_fd_tq {
            let nominal_tq = to_nominal_tq(nominal_hz, params)?;
            let data_tq = to_data_tq(data_hz, fd_params)?;
            // SAFETY: canSetBusParamsFdTq was loaded from canlib; handle is valid
            let status = unsafe { (set_fd_tq)(handle, nominal_tq, data_tq) };
            if status >= 0 {
                false
            } else if status == CAN_ERR_NOT_SUPPORTED {
                true
            } else {
                check_status(status)?;
                unreachable!()
            }
        } else {
            true
        };

        if use_legacy {
            // SAFETY: canSetBusParams was loaded from canlib; handle is valid
            check_status(unsafe {
                (lib.set_bus_params)(
                    handle,
                    nominal_hz as c_long,
                    params.tseg1,
                    params.tseg2,
                    params.sjw,
                    params.no_samp,
                    params.sync_mode,
                )
            })?;
            // SAFETY: canSetBusParamsFd was loaded from canlib; handle is valid
            check_status(unsafe {
                (lib.set_bus_params_fd)(
                    handle,
                    data_hz as c_long,
                    fd_params.tseg1,
                    fd_params.tseg2,
                    fd_params.sjw,
                )
            })?;
        }

        // SAFETY: canBusOn was loaded from canlib; handle is valid
        check_status(unsafe { (lib.bus_on)(handle) })?;
        let event = ReceiveEvent::new(lib, handle)?;
        KvaserChannel::<Fd>::new(lib.clone(), handle, event)
    })();

    if result.is_err() {
        // SAFETY: canClose was loaded from canlib; handle is valid
        unsafe { (lib.close)(handle) };
    }
    result
}

// ---------------------------------------------------------------------------
// TQ conversion helpers
// ---------------------------------------------------------------------------

/// Convert nominal bus parameters + bitrate to a `KvBusParamsTq`.
#[allow(clippy::cast_possible_wrap)] // u32 timing values fit in i32
fn to_nominal_tq(bitrate_hz: u32, p: &BusParams) -> Result<KvBusParamsTq, KvaserError> {
    let tq = 1 + p.tseg1 + p.tseg2;
    let prescaler = compute_prescaler(bitrate_hz, tq)?;
    Ok(KvBusParamsTq {
        tq: tq as i32,
        phase1: p.tseg1 as i32,
        phase2: p.tseg2 as i32,
        sjw: p.sjw as i32,
        prop: 0,
        prescaler,
    })
}

/// Convert FD data-phase bus parameters + bitrate to a `KvBusParamsTq`.
#[allow(clippy::cast_possible_wrap)] // u32 timing values fit in i32
fn to_data_tq(bitrate_hz: u32, p: &BusParamsFd) -> Result<KvBusParamsTq, KvaserError> {
    let tq = 1 + p.tseg1 + p.tseg2;
    let prescaler = compute_prescaler(bitrate_hz, tq)?;
    Ok(KvBusParamsTq {
        tq: tq as i32,
        phase1: p.tseg1 as i32,
        phase2: p.tseg2 as i32,
        sjw: p.sjw as i32,
        prop: 0,
        prescaler,
    })
}

/// Derive the prescaler from the clock, bitrate, and total time quanta.
#[allow(clippy::cast_possible_truncation)] // prescaler fits in i32 for valid CAN bitrates
fn compute_prescaler(bitrate_hz: u32, tq: u32) -> Result<i32, KvaserError> {
    let bit_time = u64::from(bitrate_hz) * u64::from(tq);
    if bit_time == 0 || u64::from(KVASER_CLOCK_HZ) % bit_time != 0 {
        return Err(KvaserError::NotSupported(format!(
            "cannot achieve {bitrate_hz} Hz with {tq} TQ at {KVASER_CLOCK_HZ} Hz clock \
             (prescaler would be non-integer)"
        )));
    }
    Ok((u64::from(KVASER_CLOCK_HZ) / bit_time) as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solver_nominal_500k_default_sample_point() {
        let (tseg1, tseg2, sjw) =
            solve_phase_timing(500_000, SamplePoint::NOMINAL_DEFAULT, TimingPhase::Nominal)
                .unwrap();
        assert_eq!(tseg1, 13);
        assert_eq!(tseg2, 6);
        assert_eq!(sjw, 4);
    }

    #[test]
    fn solver_data_4m_default_sample_point() {
        let (tseg1, tseg2, sjw) =
            solve_phase_timing(4_000_000, SamplePoint::DATA_DEFAULT, TimingPhase::Data).unwrap();
        assert_eq!(tseg1, 7);
        assert_eq!(tseg2, 2);
        assert_eq!(sjw, 2);
    }

    #[test]
    fn solver_data_5m_uses_data_constraints() {
        let (tseg1, tseg2, _sjw) =
            solve_phase_timing(5_000_000, SamplePoint::DATA_DEFAULT, TimingPhase::Data).unwrap();
        assert_eq!(tseg1, 12);
        assert_eq!(tseg2, 3);
    }

    #[test]
    fn solver_rejects_non_divisible_bitrate() {
        assert!(
            solve_phase_timing(333_000, SamplePoint::NOMINAL_DEFAULT, TimingPhase::Nominal)
                .is_none()
        );
    }

    #[test]
    fn solve_nominal_returns_busparams() {
        let p = solve_nominal(500_000, SamplePoint::NOMINAL_DEFAULT).unwrap();
        assert_eq!(p.tseg1, 13);
        assert_eq!(p.tseg2, 6);
        assert_eq!(p.sjw, 4);
        assert_eq!(p.no_samp, 1);
        assert_eq!(p.sync_mode, 0);
    }

    #[test]
    fn solve_data_returns_busparamsfd() {
        let p = solve_data(4_000_000, SamplePoint::DATA_DEFAULT).unwrap();
        assert_eq!(p.tseg1, 7);
        assert_eq!(p.tseg2, 2);
        assert_eq!(p.sjw, 2);
    }

    #[test]
    fn compute_prescaler_500k_at_20tq() {
        assert_eq!(compute_prescaler(500_000, 20).unwrap(), 8);
    }

    #[test]
    fn compute_prescaler_4m_at_10tq() {
        assert_eq!(compute_prescaler(4_000_000, 10).unwrap(), 2);
    }

    #[test]
    fn compute_prescaler_non_integer_fails() {
        assert!(compute_prescaler(333_000, 20).is_err());
    }
}
