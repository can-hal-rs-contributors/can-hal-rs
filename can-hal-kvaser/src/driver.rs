use std::marker::PhantomData;
use std::os::raw::c_long;
use std::sync::Arc;

use can_hal::SamplePoint;

use crate::channel::KvaserChannel;
use crate::error::{check_status, KvaserError};
use crate::event::ReceiveEvent;
use crate::ffi::{
    KvBusParamsTq, KvClockInfo, CAN_CHANNELDATA_CLOCK_INFO, CAN_ERR_NOT_IMPLEMENTED,
    CAN_ERR_NOT_SUPPORTED, CAN_OK, CAN_OPEN_ACCEPT_LARGE_DLC, CAN_OPEN_ACCEPT_VIRTUAL,
    CAN_OPEN_CAN_FD,
};
use crate::library::KvaserLibrary;
use crate::mode::{Classic, ClassicExplicit, Fd, FdExplicit, Initial};

/// The CAN controller clock assumed when CANlib doesn't report the device's
/// own (`canCHANNELDATA_CLOCK_INFO`).
///
/// 80 MHz is standard on most modern Kvaser adapters. To check explicit
/// [`BusParams`] / [`BusParamsFd`] values against the clock a channel
/// actually uses, call [`KvaserChannelBuilder::clock_hz`].
pub const FALLBACK_CLOCK_HZ: u32 = 80_000_000;

/// The former name of [`FALLBACK_CLOCK_HZ`].
#[deprecated(
    note = "renamed to FALLBACK_CLOCK_HZ; check explicit timing against KvaserChannelBuilder::clock_hz"
)]
pub const KVASER_CLOCK_HZ: u32 = FALLBACK_CLOCK_HZ;

/// The fastest CAN controller clock of Kvaser's adapters.
const FASTEST_CLOCK_HZ: u32 = 80_000_000;

/// The clock classic-CAN timing is solved for first when the device's clock
/// is unknown. 8 MHz divides the 16, 24, 32 and 80 MHz clocks of Kvaser's
/// adapters. A timing that fits 8 MHz with a prescaler of at most
/// [`PORTABLE_MAX_PRESCALER`] fits all of them.
const PORTABLE_CLOCK_HZ: u32 = 8_000_000;

/// The largest prescaler a [`PORTABLE_CLOCK_HZ`] timing can take. On
/// [`FASTEST_CLOCK_HZ`] its prescaler is `FASTEST_CLOCK_HZ / PORTABLE_CLOCK_HZ`
/// times larger, and must still be at most [`MAX_PRESCALER`].
const PORTABLE_MAX_PRESCALER: u32 = MAX_PRESCALER / (FASTEST_CLOCK_HZ / PORTABLE_CLOCK_HZ);

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

/// Minimum phase-segment-2 the solver will emit. Windows CANlib rejects a
/// tseg2 of 1 TQ with canERR_PARAM (Linux drivers accept it), so the solver
/// keeps tseg2 >= 2 to stay portable. That changes no classic default, but
/// some FD data-phase defaults get more TQ (3.2 Mbit/s at 80 MHz: 25 instead
/// of 5) or no timing at all (8 Mbit/s at 32 MHz).
const MIN_TSEG2: u32 = 2;

/// Nominal bus parameters: (tseg1, tseg2, sjw, noSamp, syncMode).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusParams {
    pub tseg1: u32,
    pub tseg2: u32,
    pub sjw: u32,
    pub no_samp: u32,
    pub sync_mode: u32,
}

/// FD data-phase bus parameters: (tseg1, tseg2, sjw).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

/// Solve for phase timing at the given controller clock.
///
/// Returns `None` if `bitrate_hz` is zero, does not evenly divide
/// `clock_hz`, or if no `(prescaler, total_tq)` factoring with a prescaler
/// of at most `max_prescaler` lands inside the segment-length constraints.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn solve_phase_timing(
    bitrate_hz: u32,
    sample_point: SamplePoint,
    phase: TimingPhase,
    clock_hz: u32,
    max_prescaler: u32,
) -> Option<(u32, u32, u32)> {
    if bitrate_hz == 0 {
        return None;
    }
    if clock_hz % bitrate_hz != 0 {
        return None;
    }
    let sample_point_f = sample_point.as_fraction();
    let divisor = clock_hz / bitrate_hz;

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
        if prescaler == 0 || prescaler > max_prescaler {
            total_tq += 1;
            continue;
        }

        let tseg1_plus_one = ((total_tq as f32) * sample_point_f).round() as i64;
        let tseg1 = tseg1_plus_one - 1;
        let tseg2 = i64::from(total_tq) - 1 - tseg1;

        if tseg1 < 1
            || tseg1 > i64::from(max_tseg1)
            || tseg2 < i64::from(MIN_TSEG2)
            || tseg2 > i64::from(max_tseg2)
        {
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

fn no_timing(phase: TimingPhase, bitrate_hz: u32, sample_point: SamplePoint) -> KvaserError {
    KvaserError::UnsupportedTiming(format!(
        "no Kvaser {} timing satisfies bitrate={} Hz, sample_point={}",
        phase.as_str(),
        bitrate_hz,
        sample_point.as_fraction(),
    ))
}

fn solve_nominal(
    bitrate_hz: u32,
    sample_point: SamplePoint,
    clock_hz: u32,
    max_prescaler: u32,
) -> Result<BusParams, KvaserError> {
    let (tseg1, tseg2, sjw) = solve_phase_timing(
        bitrate_hz,
        sample_point,
        TimingPhase::Nominal,
        clock_hz,
        max_prescaler,
    )
    .ok_or_else(|| no_timing(TimingPhase::Nominal, bitrate_hz, sample_point))?;
    Ok(BusParams {
        tseg1,
        tseg2,
        sjw,
        no_samp: 1,
        sync_mode: 0,
    })
}

/// Solve classic-CAN timing for the device's clock, or when it's unknown,
/// for [`PORTABLE_CLOCK_HZ`] and failing that [`FALLBACK_CLOCK_HZ`].
fn solve_classic(
    bitrate_hz: u32,
    sample_point: SamplePoint,
    clock_hz: Option<u32>,
) -> Result<BusParams, KvaserError> {
    clock_hz.map_or_else(
        || {
            solve_nominal(
                bitrate_hz,
                sample_point,
                PORTABLE_CLOCK_HZ,
                PORTABLE_MAX_PRESCALER,
            )
            .or_else(|_| solve_nominal(bitrate_hz, sample_point, FALLBACK_CLOCK_HZ, MAX_PRESCALER))
        },
        |clock_hz| solve_nominal(bitrate_hz, sample_point, clock_hz, MAX_PRESCALER),
    )
}

fn solve_data(
    bitrate_hz: u32,
    sample_point: SamplePoint,
    clock_hz: u32,
) -> Result<BusParamsFd, KvaserError> {
    let (tseg1, tseg2, sjw) = solve_phase_timing(
        bitrate_hz,
        sample_point,
        TimingPhase::Data,
        clock_hz,
        MAX_PRESCALER,
    )
    .ok_or_else(|| no_timing(TimingPhase::Data, bitrate_hz, sample_point))?;
    Ok(BusParamsFd { tseg1, tseg2, sjw })
}

/// The device clock in Hz from CANlib's report, if it's a whole number of
/// Hz that fits a `u32`.
fn clock_from_info(info: &KvClockInfo) -> Option<u32> {
    if info.version < 1 || info.numerator <= 0 || info.denominator <= 0 {
        return None;
    }
    let scale = 10u64.checked_pow(u32::try_from(info.power_of_ten).ok()?)?;
    let scaled = u64::try_from(info.numerator).ok()?.checked_mul(scale)?;
    let denominator = u64::try_from(info.denominator).ok()?;
    if scaled % denominator != 0 {
        return None;
    }
    u32::try_from(scaled / denominator).ok()
}

/// Ask CANlib for a channel's clock (`canCHANNELDATA_CLOCK_INFO`). `None`
/// when CANlib doesn't report it.
fn device_clock_hz(lib: &KvaserLibrary, channel_index: i32) -> Option<u32> {
    let get_channel_data = lib.get_channel_data?;
    let mut info = KvClockInfo::default();
    // SAFETY: canGetChannelData was loaded from canlib; the buffer is a
    // KvClockInfo of the size passed.
    let status = unsafe {
        get_channel_data(
            channel_index,
            CAN_CHANNELDATA_CLOCK_INFO,
            std::ptr::addr_of_mut!(info).cast(),
            std::mem::size_of::<KvClockInfo>(),
        )
    };
    if status == CAN_OK {
        clock_from_info(&info)
    } else {
        None
    }
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
    ///
    /// Infallible: invalid indices (out of range, missing hardware) surface
    /// at `connect()` when CANlib's `canOpenChannel` is called.
    /// Asks CANlib for the channel's clock, which the timing is computed for;
    /// see [`classic`](KvaserChannelBuilder::classic) and
    /// [`fd`](KvaserChannelBuilder::fd) for when it's unknown.
    #[must_use]
    #[allow(clippy::cast_possible_wrap)] // channel index is small, fits in i32
    pub fn channel(&self, index: u32) -> KvaserChannelBuilder<Initial> {
        let channel_index = index as i32;
        KvaserChannelBuilder {
            lib: Arc::clone(&self.lib),
            channel_index,
            accept_virtual: false,
            clock_hz: device_clock_hz(&self.lib, channel_index),
            state: Initial,
            _mode: PhantomData,
        }
    }
}

/// Typestate-driven builder for a KVASER channel.
///
/// Start with [`Initial`] (returned by [`KvaserDriver::channel`]) and
/// transition to [`Classic`] (via [`classic`](Self::classic)), [`Fd`] (via
/// [`fd`](Self::fd)), or the raw-timing [`ClassicExplicit`] / [`FdExplicit`]
/// (via [`classic_explicit`](Self::classic_explicit) /
/// [`fd_explicit`](Self::fd_explicit)). Only the methods valid for the
/// current state are available.
pub struct KvaserChannelBuilder<Mode> {
    lib: Arc<KvaserLibrary>,
    channel_index: i32,
    accept_virtual: bool,
    /// The device's clock, if CANlib reports it.
    clock_hz: Option<u32>,
    state: Mode,
    _mode: PhantomData<Mode>,
}

impl<Mode> KvaserChannelBuilder<Mode> {
    /// The clock bitrates and explicit timing are checked against: the
    /// device's, as CANlib reports it (`canCHANNELDATA_CLOCK_INFO`), or the
    /// 80 MHz [`FALLBACK_CLOCK_HZ`] when it doesn't.
    ///
    /// Values passed to
    /// [`classic_explicit`](KvaserChannelBuilder::classic_explicit) /
    /// [`fd_explicit`](KvaserChannelBuilder::fd_explicit) must satisfy
    /// `clock_hz() % (bitrate_hz * (1 + tseg1 + tseg2)) == 0` for each phase.
    #[must_use]
    pub fn clock_hz(&self) -> u32 {
        self.reported_clock_hz().unwrap_or(FALLBACK_CLOCK_HZ)
    }

    /// The device's clock as CANlib reports it (`canCHANNELDATA_CLOCK_INFO`),
    /// or `None` when it doesn't, in which case [`clock_hz`](Self::clock_hz)
    /// assumes [`FALLBACK_CLOCK_HZ`].
    #[must_use]
    pub const fn reported_clock_hz(&self) -> Option<u32> {
        self.clock_hz
    }

    fn with_state<Next>(self, state: Next) -> KvaserChannelBuilder<Next> {
        KvaserChannelBuilder {
            lib: self.lib,
            channel_index: self.channel_index,
            accept_virtual: self.accept_virtual,
            clock_hz: self.clock_hz,
            state,
            _mode: PhantomData,
        }
    }
}

impl<Mode> KvaserChannelBuilder<Mode> {
    /// Allow opening virtual channels (e.g. the Kvaser Virtual CAN Driver).
    /// Defaults to `false`. Maps to CANlib's `canOPEN_ACCEPT_VIRTUAL` flag,
    /// which has no effect on a physical channel; without it, `connect()`
    /// fails on a virtual one.
    #[must_use]
    pub const fn accept_virtual(mut self, accept: bool) -> Self {
        self.accept_virtual = accept;
        self
    }
}

impl KvaserChannelBuilder<Initial> {
    /// Configure the channel for classic CAN at the given bitrate (Hz).
    /// Returns `Err` if the bitrate does not evenly divide the device's
    /// clock, or the 80 MHz [`FALLBACK_CLOCK_HZ`] when CANlib doesn't report
    /// it. Sample point defaults to [`SamplePoint::NOMINAL_DEFAULT`] (70%);
    /// override on the returned builder.
    ///
    /// When the device's clock is unknown, the timing is solved for an
    /// 8 MHz clock, which divides the 16, 24, 32 and 80 MHz clocks of
    /// Kvaser's adapters, so it fits any of them; failing that, for 80 MHz.
    pub fn classic(self, bitrate_hz: u32) -> Result<KvaserChannelBuilder<Classic>, KvaserError> {
        if bitrate_hz == 0 || self.clock_hz() % bitrate_hz != 0 {
            return Err(KvaserError::UnsupportedBitrate(bitrate_hz));
        }
        Ok(self.with_state(Classic::new(bitrate_hz, DEFAULT_NOMINAL_SAMPLE_POINT)))
    }

    /// Configure the channel for CAN FD with the given nominal and data
    /// bitrates (Hz). Returns `Err` if either bitrate does not evenly divide
    /// the device's clock, or the 80 MHz [`FALLBACK_CLOCK_HZ`] when CANlib
    /// doesn't report it. Sample points default to
    /// [`SamplePoint::NOMINAL_DEFAULT`] (70%) and
    /// [`SamplePoint::DATA_DEFAULT`] (80%); override on the returned builder.
    pub fn fd(
        self,
        nominal_hz: u32,
        data_hz: u32,
    ) -> Result<KvaserChannelBuilder<Fd>, KvaserError> {
        let clock_hz = self.clock_hz();
        if nominal_hz == 0 || clock_hz % nominal_hz != 0 {
            return Err(KvaserError::UnsupportedBitrate(nominal_hz));
        }
        if data_hz == 0 || clock_hz % data_hz != 0 {
            return Err(KvaserError::UnsupportedBitrate(data_hz));
        }
        Ok(self.with_state(Fd::new(
            nominal_hz,
            data_hz,
            DEFAULT_NOMINAL_SAMPLE_POINT,
            DEFAULT_DATA_SAMPLE_POINT,
        )))
    }

    /// Configure the channel for classic CAN with explicit nominal timing.
    ///
    /// Bypasses the timing solver. The returned builder has no setters; the
    /// next call must be [`connect`](KvaserChannelBuilder::connect). The
    /// `bitrate_hz` is still required because CANlib's `canSetBusParams`
    /// takes the frequency as input and derives the prescaler from
    /// `clock / (bitrate_hz * (1 + tseg1 + tseg2))`.
    ///
    /// Returns `Err` if any of the following don't hold for the device's
    /// clock, or the 80 MHz [`FALLBACK_CLOCK_HZ`] when CANlib doesn't report
    /// it:
    ///
    /// - `bitrate_hz` evenly divides the clock,
    /// - `tseg1` is in `[1, 256]` and `tseg2` is in `[2, 128]`,
    /// - `bitrate_hz * (1 + tseg1 + tseg2)` evenly divides the clock, and
    /// - the resulting prescaler is in `[1, 1024]`.
    ///
    /// These surface here rather than at `connect()`, except when the clock
    /// is assumed and the device's own differs: then CANlib can still reject
    /// the timing at `connect()`.
    pub fn classic_explicit(
        self,
        bitrate_hz: u32,
        params: BusParams,
    ) -> Result<KvaserChannelBuilder<ClassicExplicit>, KvaserError> {
        let clock_hz = self.clock_hz();
        if bitrate_hz == 0 || clock_hz % bitrate_hz != 0 {
            return Err(KvaserError::UnsupportedBitrate(bitrate_hz));
        }
        // Eagerly validate that segment values are within the
        // controller's accepted range and that the (bitrate, segments)
        // combination yields an integer prescaler. Without these checks,
        // out-of-range params would otherwise fail at `connect()` with a
        // raw CANlib status code.
        check_segment_bounds(params.tseg1, params.tseg2, TimingPhase::Nominal)?;
        let tq = total_tq(params.tseg1, params.tseg2)?;
        let _ = compute_prescaler(bitrate_hz, tq, clock_hz)?;
        Ok(self.with_state(ClassicExplicit::new(bitrate_hz, params)))
    }

    /// Configure the channel for CAN FD with explicit nominal and data
    /// timing.
    ///
    /// Bypasses the timing solver. The returned builder has no setters; the
    /// next call must be [`connect`](KvaserChannelBuilder::connect).
    ///
    /// Returns `Err` if any of the following don't hold for the device's
    /// clock, or the 80 MHz [`FALLBACK_CLOCK_HZ`] when CANlib doesn't report
    /// it:
    ///
    /// - both `nominal_hz` and `data_hz` evenly divide the clock,
    /// - nominal `tseg1` in `[1, 256]`, `tseg2` in `[2, 128]`,
    /// - data `tseg1` in `[1, 32]`, `tseg2` in `[2, 16]`,
    /// - `bitrate * (1 + tseg1 + tseg2)` evenly divides the clock for each
    ///   phase, and
    /// - each resulting prescaler is in `[1, 1024]`.
    ///
    /// These surface here rather than at `connect()`, except when the clock
    /// is assumed and the device's own differs: then CANlib can still reject
    /// the timing at `connect()`.
    pub fn fd_explicit(
        self,
        nominal_hz: u32,
        data_hz: u32,
        params: BusParams,
        fd_params: BusParamsFd,
    ) -> Result<KvaserChannelBuilder<FdExplicit>, KvaserError> {
        let clock_hz = self.clock_hz();
        if nominal_hz == 0 || clock_hz % nominal_hz != 0 {
            return Err(KvaserError::UnsupportedBitrate(nominal_hz));
        }
        if data_hz == 0 || clock_hz % data_hz != 0 {
            return Err(KvaserError::UnsupportedBitrate(data_hz));
        }
        // Eagerly validate segment-length bounds and that the (bitrate,
        // segments) combinations yield integer prescalers within range.
        check_segment_bounds(params.tseg1, params.tseg2, TimingPhase::Nominal)?;
        check_segment_bounds(fd_params.tseg1, fd_params.tseg2, TimingPhase::Data)?;
        let nominal_tq = total_tq(params.tseg1, params.tseg2)?;
        let _ = compute_prescaler(nominal_hz, nominal_tq, clock_hz)?;
        let data_tq_val = total_tq(fd_params.tseg1, fd_params.tseg2)?;
        let _ = compute_prescaler(data_hz, data_tq_val, clock_hz)?;
        Ok(self.with_state(FdExplicit::new(nominal_hz, data_hz, params, fd_params)))
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
        let params = solve_classic(
            self.state.bitrate_hz(),
            self.state.sample_point(),
            self.clock_hz,
        )?;
        open_classic(
            &self.lib,
            self.channel_index,
            self.accept_virtual,
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
            self.accept_virtual,
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
        let clock_hz = self.clock_hz();
        let params = solve_nominal(
            self.state.nominal_hz(),
            self.state.sample_point(),
            clock_hz,
            MAX_PRESCALER,
        )?;
        let fd_params = solve_data(
            self.state.data_hz(),
            self.state.data_sample_point(),
            clock_hz,
        )?;
        open_fd(
            &self.lib,
            self.channel_index,
            self.accept_virtual,
            clock_hz,
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
            self.accept_virtual,
            self.clock_hz(),
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
    accept_virtual: bool,
    bitrate_hz: u32,
    params: &BusParams,
) -> Result<KvaserChannel<Classic>, KvaserError> {
    // Send and receive DLCs 9-15 as they are, rather than as 8.
    let mut flags = CAN_OPEN_ACCEPT_LARGE_DLC;
    if accept_virtual {
        flags |= CAN_OPEN_ACCEPT_VIRTUAL;
    }

    // SAFETY: canOpenChannel was loaded from canlib
    let handle = unsafe { (lib.open_channel)(channel_index, flags) };
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
        Ok(KvaserChannel::<Classic>::new(lib.clone(), handle, event))
    })();

    if result.is_err() {
        // SAFETY: canClose was loaded from canlib; handle is valid
        unsafe { (lib.close)(handle) };
    }
    result
}

/// Open a CAN FD channel and go on-bus. Shared between the solver and
/// explicit paths.
#[allow(
    clippy::cast_lossless,
    clippy::cast_possible_wrap,
    clippy::too_many_arguments
)]
fn open_fd(
    lib: &Arc<KvaserLibrary>,
    channel_index: i32,
    accept_virtual: bool,
    clock_hz: u32,
    nominal_hz: u32,
    data_hz: u32,
    params: &BusParams,
    fd_params: &BusParamsFd,
) -> Result<KvaserChannel<Fd>, KvaserError> {
    let mut flags = CAN_OPEN_CAN_FD;
    if accept_virtual {
        flags |= CAN_OPEN_ACCEPT_VIRTUAL;
    }

    // SAFETY: canOpenChannel was loaded from canlib
    let handle = unsafe { (lib.open_channel)(channel_index, flags) };
    if handle < 0 {
        return Err(KvaserError::Canlib(crate::error::KvaserStatus(handle)));
    }

    let result = (|| {
        // Try canSetBusParamsFdTq first: one call sets nominal and data
        // timing from explicit time quanta. CANlib may not export it, or may
        // refuse it for a channel (below); then use canSetBusParams +
        // canSetBusParamsFd.
        let use_legacy = if let Some(set_fd_tq) = lib.set_bus_params_fd_tq {
            let nominal_tq = to_nominal_tq(nominal_hz, params, clock_hz)?;
            let data_tq = to_data_tq(data_hz, fd_params, clock_hz)?;
            // SAFETY: canSetBusParamsFdTq was loaded from canlib; handle is valid
            let status = unsafe { (set_fd_tq)(handle, nominal_tq, data_tq) };
            if status == CAN_ERR_NOT_SUPPORTED || status == CAN_ERR_NOT_IMPLEMENTED {
                // CANlib can export canSetBusParamsFdTq and still refuse it
                // for a channel. On Linux it returns canERR_NOT_IMPLEMENTED
                // for a channel without time-quanta support, such as a
                // virtual channel, and some builds return
                // canERR_NOT_SUPPORTED for certain hardware or firmware.
                // Either way, fall through to the legacy calls.
                true
            } else {
                check_status(status)?;
                false
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
        Ok(KvaserChannel::<Fd>::new(lib.clone(), handle, event))
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
///
/// Returns `UnsupportedTiming` if the segment values are large enough to
/// overflow u32 addition, or if any field doesn't fit in `i32` (CANlib's
/// FFI takes signed types).
fn to_nominal_tq(
    bitrate_hz: u32,
    p: &BusParams,
    clock_hz: u32,
) -> Result<KvBusParamsTq, KvaserError> {
    let tq = total_tq(p.tseg1, p.tseg2)?;
    let prescaler = compute_prescaler(bitrate_hz, tq, clock_hz)?;
    Ok(KvBusParamsTq {
        tq: u32_to_i32(tq, "tq")?,
        phase1: u32_to_i32(p.tseg1, "tseg1")?,
        phase2: u32_to_i32(p.tseg2, "tseg2")?,
        sjw: u32_to_i32(p.sjw, "sjw")?,
        prop: 0,
        prescaler,
    })
}

/// Convert FD data-phase bus parameters + bitrate to a `KvBusParamsTq`.
fn to_data_tq(
    bitrate_hz: u32,
    p: &BusParamsFd,
    clock_hz: u32,
) -> Result<KvBusParamsTq, KvaserError> {
    let tq = total_tq(p.tseg1, p.tseg2)?;
    let prescaler = compute_prescaler(bitrate_hz, tq, clock_hz)?;
    Ok(KvBusParamsTq {
        tq: u32_to_i32(tq, "tq")?,
        phase1: u32_to_i32(p.tseg1, "tseg1")?,
        phase2: u32_to_i32(p.tseg2, "tseg2")?,
        sjw: u32_to_i32(p.sjw, "sjw")?,
        prop: 0,
        prescaler,
    })
}

fn total_tq(tseg1: u32, tseg2: u32) -> Result<u32, KvaserError> {
    tseg1
        .checked_add(tseg2)
        .and_then(|sum| sum.checked_add(1))
        .ok_or_else(|| {
            KvaserError::UnsupportedTiming(format!(
                "tseg1={tseg1} + tseg2={tseg2} overflows total TQ"
            ))
        })
}

/// Verify segment values fit the controller's range for the given phase.
fn check_segment_bounds(tseg1: u32, tseg2: u32, phase: TimingPhase) -> Result<(), KvaserError> {
    let (max_tseg1, max_tseg2) = match phase {
        TimingPhase::Nominal => (NOMINAL_MAX_TSEG1, NOMINAL_MAX_TSEG2),
        TimingPhase::Data => (DATA_MAX_TSEG1, DATA_MAX_TSEG2),
    };
    let phase_name = phase.as_str();
    if tseg1 == 0 || tseg1 > max_tseg1 {
        return Err(KvaserError::UnsupportedTiming(format!(
            "{phase_name} tseg1={tseg1} out of range [1, {max_tseg1}]"
        )));
    }
    if tseg2 < MIN_TSEG2 || tseg2 > max_tseg2 {
        return Err(KvaserError::UnsupportedTiming(format!(
            "{phase_name} tseg2={tseg2} out of range [{MIN_TSEG2}, {max_tseg2}]"
        )));
    }
    Ok(())
}

#[allow(clippy::cast_possible_wrap)] // checked above
fn u32_to_i32(value: u32, field: &'static str) -> Result<i32, KvaserError> {
    if value > i32::MAX as u32 {
        Err(KvaserError::UnsupportedTiming(format!(
            "{field}={value} does not fit in i32"
        )))
    } else {
        Ok(value as i32)
    }
}

/// Derive the prescaler from the clock, bitrate, and total time quanta.
fn compute_prescaler(bitrate_hz: u32, tq: u32, clock_hz: u32) -> Result<i32, KvaserError> {
    let bit_time = u64::from(bitrate_hz) * u64::from(tq);
    if bit_time == 0 || u64::from(clock_hz) % bit_time != 0 {
        return Err(KvaserError::UnsupportedTiming(format!(
            "cannot achieve {bitrate_hz} Hz with {tq} TQ at {clock_hz} Hz clock \
             (prescaler would be non-integer)"
        )));
    }
    let prescaler = u64::from(clock_hz) / bit_time;
    if prescaler == 0 || prescaler > u64::from(MAX_PRESCALER) {
        return Err(KvaserError::UnsupportedTiming(format!(
            "prescaler {prescaler} for bitrate={bitrate_hz} Hz, tq={tq} \
             out of range [1, {MAX_PRESCALER}]"
        )));
    }
    #[allow(clippy::cast_possible_truncation)] // bounded above by MAX_PRESCALER
    Ok(prescaler as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solver_nominal_500k_default_sample_point() {
        let (tseg1, tseg2, sjw) = solve_phase_timing(
            500_000,
            SamplePoint::NOMINAL_DEFAULT,
            TimingPhase::Nominal,
            FALLBACK_CLOCK_HZ,
            MAX_PRESCALER,
        )
        .unwrap();
        assert_eq!(tseg1, 13);
        assert_eq!(tseg2, 6);
        assert_eq!(sjw, 4);
    }

    #[test]
    fn solver_data_4m_default_sample_point() {
        let (tseg1, tseg2, sjw) = solve_phase_timing(
            4_000_000,
            SamplePoint::DATA_DEFAULT,
            TimingPhase::Data,
            FALLBACK_CLOCK_HZ,
            MAX_PRESCALER,
        )
        .unwrap();
        assert_eq!(tseg1, 7);
        assert_eq!(tseg2, 2);
        assert_eq!(sjw, 2);
    }

    #[test]
    fn solver_data_5m_uses_data_constraints() {
        let (tseg1, tseg2, _sjw) = solve_phase_timing(
            5_000_000,
            SamplePoint::DATA_DEFAULT,
            TimingPhase::Data,
            FALLBACK_CLOCK_HZ,
            MAX_PRESCALER,
        )
        .unwrap();
        assert_eq!(tseg1, 12);
        assert_eq!(tseg2, 3);
    }

    #[test]
    fn solver_data_4m_75pct_avoids_tseg2_of_one() {
        // A 75% data sample point at 4 Mbit/s previously resolved to a 4 TQ
        // timing with tseg2=1, which Windows CANlib rejects (canERR_PARAM).
        // The MIN_TSEG2 floor forces a portable solution instead.
        let (tseg1, tseg2, sjw) = solve_phase_timing(
            4_000_000,
            SamplePoint::PCT_75,
            TimingPhase::Data,
            FALLBACK_CLOCK_HZ,
            MAX_PRESCALER,
        )
        .unwrap();
        assert!(tseg2 >= MIN_TSEG2, "tseg2 must be >= 2, got {tseg2}");
        assert_eq!((tseg1, tseg2, sjw), (14, 5, 4));
    }

    #[test]
    fn solver_rejects_non_divisible_bitrate() {
        assert!(solve_phase_timing(
            333_000,
            SamplePoint::NOMINAL_DEFAULT,
            TimingPhase::Nominal,
            FALLBACK_CLOCK_HZ,
            MAX_PRESCALER
        )
        .is_none());
    }

    #[test]
    fn solve_nominal_returns_busparams() {
        let p = solve_nominal(
            500_000,
            SamplePoint::NOMINAL_DEFAULT,
            FALLBACK_CLOCK_HZ,
            MAX_PRESCALER,
        )
        .unwrap();
        assert_eq!(p.tseg1, 13);
        assert_eq!(p.tseg2, 6);
        assert_eq!(p.sjw, 4);
        assert_eq!(p.no_samp, 1);
        assert_eq!(p.sync_mode, 0);
    }

    #[test]
    fn solve_data_returns_busparamsfd() {
        let p = solve_data(4_000_000, SamplePoint::DATA_DEFAULT, FALLBACK_CLOCK_HZ).unwrap();
        assert_eq!(p.tseg1, 7);
        assert_eq!(p.tseg2, 2);
        assert_eq!(p.sjw, 2);
    }

    #[test]
    fn compute_prescaler_500k_at_20tq() {
        assert_eq!(
            compute_prescaler(500_000, 20, FALLBACK_CLOCK_HZ).unwrap(),
            8
        );
    }

    #[test]
    fn compute_prescaler_4m_at_10tq() {
        assert_eq!(
            compute_prescaler(4_000_000, 10, FALLBACK_CLOCK_HZ).unwrap(),
            2
        );
    }

    #[test]
    fn compute_prescaler_non_integer_fails() {
        assert!(compute_prescaler(333_000, 20, FALLBACK_CLOCK_HZ).is_err());
    }

    #[test]
    fn u32_to_i32_accepts_i32_max() {
        assert_eq!(u32_to_i32(i32::MAX as u32, "x").unwrap(), i32::MAX);
    }

    #[test]
    fn u32_to_i32_rejects_above_i32_max() {
        assert!(u32_to_i32(i32::MAX as u32 + 1, "x").is_err());
        assert!(u32_to_i32(u32::MAX, "x").is_err());
    }

    #[test]
    fn total_tq_handles_overflow() {
        assert!(total_tq(u32::MAX, 1).is_err());
        assert!(total_tq(u32::MAX, u32::MAX).is_err());
        assert_eq!(total_tq(13, 6).unwrap(), 20);
    }

    #[test]
    fn check_segment_bounds_accepts_in_range() {
        assert!(check_segment_bounds(13, 6, TimingPhase::Nominal).is_ok());
        assert!(check_segment_bounds(7, 2, TimingPhase::Data).is_ok());
    }

    #[test]
    fn check_segment_bounds_rejects_zero() {
        assert!(check_segment_bounds(0, 6, TimingPhase::Nominal).is_err());
        assert!(check_segment_bounds(13, 0, TimingPhase::Nominal).is_err());
    }

    #[test]
    fn check_segment_bounds_rejects_tseg2_below_min() {
        // tseg2=1 is accepted by some drivers but rejected by Windows CANlib;
        // the explicit path enforces the same MIN_TSEG2 floor as the solver.
        assert!(check_segment_bounds(13, 1, TimingPhase::Nominal).is_err());
        assert!(check_segment_bounds(7, 1, TimingPhase::Data).is_err());
    }

    #[test]
    fn check_segment_bounds_rejects_nominal_above_max() {
        // tseg1 max for nominal is 256, tseg2 max is 128.
        assert!(check_segment_bounds(257, 6, TimingPhase::Nominal).is_err());
        assert!(check_segment_bounds(13, 129, TimingPhase::Nominal).is_err());
    }

    #[test]
    fn check_segment_bounds_rejects_data_above_max() {
        // tseg1 max for data is 32, tseg2 max is 16.
        assert!(check_segment_bounds(33, 2, TimingPhase::Data).is_err());
        assert!(check_segment_bounds(7, 17, TimingPhase::Data).is_err());
    }

    #[test]
    fn compute_prescaler_at_clock_boundary() {
        // bitrate=clock, tq=1 -> prescaler=1, the lower bound of the range.
        // Documents that the function is safe at the smallest valid prescaler.
        let r = compute_prescaler(80_000_000, 1, FALLBACK_CLOCK_HZ).unwrap();
        assert_eq!(r, 1);
    }

    #[test]
    fn compute_prescaler_rejects_above_max_prescaler() {
        // 100 Hz nominal with tq=3 yields prescaler 80M / (100 * 3) ≈ 266,667,
        // far above MAX_PRESCALER=1024. (Also non-integer here, so the
        // divisibility check would catch it first - try an exact case:
        // 100 Hz, tq=10 -> 80M / 1000 = 80,000 prescaler. Still above 1024.)
        assert!(compute_prescaler(100, 10, FALLBACK_CLOCK_HZ).is_err());
    }

    /// The clocks of Kvaser's adapters: Leaf Light v2 (16 MHz), and the
    /// 24, 32 and 80 MHz ones.
    const ADAPTER_CLOCKS: [u32; 4] = [16_000_000, 24_000_000, 32_000_000, FASTEST_CLOCK_HZ];

    #[test]
    fn classic_timing_without_a_clock_fits_every_adapter() {
        for bitrate_hz in [125_000, 250_000, 500_000, 1_000_000] {
            let p = solve_classic(bitrate_hz, SamplePoint::NOMINAL_DEFAULT, None).unwrap();
            let tq = total_tq(p.tseg1, p.tseg2).unwrap();
            for clock_hz in ADAPTER_CLOCKS {
                assert!(
                    compute_prescaler(bitrate_hz, tq, clock_hz).is_ok(),
                    "{bitrate_hz} Hz with {tq} TQ doesn't fit a {clock_hz} Hz clock"
                );
            }
        }
    }

    #[test]
    fn classic_timing_without_a_clock_fits_every_adapter_at_low_bitrates() {
        // These divide every adapter clock, but an 8 MHz timing without
        // PORTABLE_MAX_PRESCALER needs a prescaler above MAX_PRESCALER at
        // 80 MHz: 2000 at 2 kbit/s (20 TQ), 1250 at 3.2 (20 TQ) and
        // 6.4 kbit/s (10 TQ).
        for bitrate_hz in [2_000, 3_200, 6_400] {
            let p = solve_classic(bitrate_hz, SamplePoint::NOMINAL_DEFAULT, None).unwrap();
            let tq = total_tq(p.tseg1, p.tseg2).unwrap();
            for clock_hz in ADAPTER_CLOCKS {
                assert!(
                    compute_prescaler(bitrate_hz, tq, clock_hz).is_ok(),
                    "{bitrate_hz} Hz with {tq} TQ doesn't fit a {clock_hz} Hz clock"
                );
            }
        }
    }

    #[test]
    fn classic_timing_without_a_clock_at_250k() {
        // 16 TQ, as a Leaf Light v2 takes at 250 kbit/s; 20 doesn't fit its
        // 16 MHz clock.
        let p = solve_classic(250_000, SamplePoint::NOMINAL_DEFAULT, None).unwrap();
        assert_eq!(total_tq(p.tseg1, p.tseg2).unwrap(), 16);
        assert_eq!((p.tseg1, p.tseg2, p.sjw), (10, 5, 4));
    }

    #[test]
    fn classic_timing_without_a_clock_falls_back_to_80_mhz() {
        // 625 kbit/s doesn't divide 8 MHz, but does 80 MHz.
        assert!(solve_nominal(
            625_000,
            SamplePoint::NOMINAL_DEFAULT,
            PORTABLE_CLOCK_HZ,
            PORTABLE_MAX_PRESCALER
        )
        .is_err());
        let p = solve_classic(625_000, SamplePoint::NOMINAL_DEFAULT, None).unwrap();
        let tq = total_tq(p.tseg1, p.tseg2).unwrap();
        assert!(compute_prescaler(625_000, tq, FALLBACK_CLOCK_HZ).is_ok());
    }

    #[test]
    fn classic_timing_with_an_80_mhz_clock_is_unchanged() {
        let p = solve_classic(500_000, SamplePoint::NOMINAL_DEFAULT, Some(80_000_000)).unwrap();
        assert_eq!((p.tseg1, p.tseg2, p.sjw), (13, 6, 4));
    }

    #[test]
    fn classic_timing_for_a_16_mhz_clock() {
        let p = solve_classic(250_000, SamplePoint::NOMINAL_DEFAULT, Some(16_000_000)).unwrap();
        let tq = total_tq(p.tseg1, p.tseg2).unwrap();
        assert!(compute_prescaler(250_000, tq, 16_000_000).is_ok());
    }

    #[test]
    fn clock_from_info_scales() {
        let info = |numerator, denominator, power_of_ten| KvClockInfo {
            version: 1,
            numerator,
            denominator,
            power_of_ten,
            accuracy_ppm: 0,
        };
        assert_eq!(clock_from_info(&info(80, 1, 6)), Some(80_000_000));
        assert_eq!(clock_from_info(&info(16_000_000, 1, 0)), Some(16_000_000));
        assert_eq!(clock_from_info(&info(48, 2, 6)), Some(24_000_000));
        // Not a whole number of Hz, or not reported.
        assert_eq!(clock_from_info(&info(1, 3, 0)), None);
        assert_eq!(clock_from_info(&info(0, 1, 6)), None);
        assert_eq!(clock_from_info(&info(80, 0, 6)), None);
        assert_eq!(clock_from_info(&info(80, 1, -1)), None);
        assert_eq!(clock_from_info(&KvClockInfo::default()), None);
        // Too big for a u32.
        assert_eq!(clock_from_info(&info(80, 1, 9)), None);
    }
}
