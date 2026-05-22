//! Type-state markers for the KVASER channel builder and channel.
//!
//! These types are used as a type parameter on
//! [`KvaserChannelBuilder`](crate::KvaserChannelBuilder) and
//! [`KvaserChannel`](crate::KvaserChannel) to encode which configuration
//! path the channel is on. The compiler then restricts which methods are
//! callable in each state. Their internals are private; external code can
//! name them in signatures but cannot construct or inspect them.

use can_hal::SamplePoint;

use crate::driver::{BusParams, BusParamsFd};

/// Initial state. The builder hasn't picked classic vs FD yet.
pub struct Initial;

/// Classic-CAN state, solver-driven path.
pub struct Classic {
    bitrate_hz: u32,
    sample_point: SamplePoint,
}

impl Classic {
    pub(crate) const fn new(bitrate_hz: u32, sample_point: SamplePoint) -> Self {
        Self {
            bitrate_hz,
            sample_point,
        }
    }

    pub(crate) const fn bitrate_hz(&self) -> u32 {
        self.bitrate_hz
    }

    pub(crate) const fn sample_point(&self) -> SamplePoint {
        self.sample_point
    }

    pub(crate) fn set_sample_point(&mut self, sp: SamplePoint) {
        self.sample_point = sp;
    }
}

/// Classic-CAN state, raw-timing path. The user has fully specified the
/// nominal bus parameters via
/// [`classic_explicit`](crate::KvaserChannelBuilder::classic_explicit);
/// no further configuration is possible.
pub struct ClassicExplicit {
    bitrate_hz: u32,
    params: BusParams,
}

impl ClassicExplicit {
    pub(crate) const fn new(bitrate_hz: u32, params: BusParams) -> Self {
        Self { bitrate_hz, params }
    }

    pub(crate) const fn bitrate_hz(&self) -> u32 {
        self.bitrate_hz
    }

    pub(crate) const fn params(&self) -> &BusParams {
        &self.params
    }
}

/// CAN FD state, solver-driven path.
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

/// CAN FD state, raw-timing path. The user has fully specified both
/// nominal and data bus parameters via
/// [`fd_explicit`](crate::KvaserChannelBuilder::fd_explicit); no further
/// configuration is possible.
pub struct FdExplicit {
    nominal_hz: u32,
    data_hz: u32,
    params: BusParams,
    fd_params: BusParamsFd,
}

impl FdExplicit {
    pub(crate) const fn new(
        nominal_hz: u32,
        data_hz: u32,
        params: BusParams,
        fd_params: BusParamsFd,
    ) -> Self {
        Self {
            nominal_hz,
            data_hz,
            params,
            fd_params,
        }
    }

    pub(crate) const fn nominal_hz(&self) -> u32 {
        self.nominal_hz
    }

    pub(crate) const fn data_hz(&self) -> u32 {
        self.data_hz
    }

    pub(crate) const fn params(&self) -> &BusParams {
        &self.params
    }

    pub(crate) const fn fd_params(&self) -> &BusParamsFd {
        &self.fd_params
    }
}
