//! Type-state markers for the PCAN channel builder and channel.
//!
//! These types are used as a type parameter on
//! [`PcanChannelBuilder`](crate::PcanChannelBuilder) and
//! [`PcanChannel`](crate::PcanChannel) to encode which configuration path
//! the channel is on. The compiler then restricts which methods are
//! callable in each state. Their internals are private; external code can
//! name them in signatures but cannot construct or inspect them.

use can_hal::SamplePoint;

use crate::driver::PcanFdTiming;

/// Initial state. The builder hasn't picked classic vs FD yet.
pub struct Initial;

/// Classic-CAN state. Reached via
/// [`PcanChannelBuilder::classic`](crate::PcanChannelBuilder::classic).
pub struct Classic {
    bitrate_const: u16,
}

impl Classic {
    pub(crate) const fn new(bitrate_const: u16) -> Self {
        Self { bitrate_const }
    }

    pub(crate) const fn bitrate_const(&self) -> u16 {
        self.bitrate_const
    }
}

/// CAN FD state, solver-driven path. Reached via
/// [`PcanChannelBuilder::fd`](crate::PcanChannelBuilder::fd).
pub struct Fd {
    nominal_hz: u32,
    data_hz: u32,
    sample_point: SamplePoint,
    data_sample_point: SamplePoint,
}

impl Fd {
    pub(crate) const fn new(
        nominal_hz: u32,
        data_hz: u32,
        sample_point: SamplePoint,
        data_sample_point: SamplePoint,
    ) -> Self {
        Self {
            nominal_hz,
            data_hz,
            sample_point,
            data_sample_point,
        }
    }

    pub(crate) const fn nominal_hz(&self) -> u32 {
        self.nominal_hz
    }

    pub(crate) const fn data_hz(&self) -> u32 {
        self.data_hz
    }

    pub(crate) const fn sample_point(&self) -> SamplePoint {
        self.sample_point
    }

    pub(crate) const fn data_sample_point(&self) -> SamplePoint {
        self.data_sample_point
    }

    pub(crate) fn set_sample_point(&mut self, sp: SamplePoint) {
        self.sample_point = sp;
    }

    pub(crate) fn set_data_sample_point(&mut self, sp: SamplePoint) {
        self.data_sample_point = sp;
    }
}

/// CAN FD state, raw-timing path. Reached via
/// [`PcanChannelBuilder::fd_explicit`](crate::PcanChannelBuilder::fd_explicit).
/// No further configuration is possible; only `connect()`. The channel
/// produced is indistinguishable from one configured via [`Fd`].
pub struct FdExplicit {
    timing: PcanFdTiming,
}

impl FdExplicit {
    pub(crate) const fn new(timing: PcanFdTiming) -> Self {
        Self { timing }
    }

    pub(crate) const fn timing(&self) -> PcanFdTiming {
        self.timing
    }
}
