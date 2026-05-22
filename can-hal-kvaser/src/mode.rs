//! Type-state markers for the KVASER channel builder and channel.
//!
//! These zero-sized types are used as a type parameter on
//! [`KvaserChannelBuilder`](crate::KvaserChannelBuilder) and
//! [`KvaserChannel`](crate::KvaserChannel) to encode which configuration
//! path the channel is on. The compiler then restricts which methods are
//! callable in each state.

use crate::driver::{BusParams, BusParamsFd};

/// Initial state — the builder hasn't picked classic vs FD yet.
pub struct Initial;

/// Classic-CAN state. Carries the nominal bitrate, sample point, and any
/// explicit timing override.
pub struct Classic {
    pub(crate) bitrate_hz: u32,
    pub(crate) sample_point: f32,
    pub(crate) custom_params: Option<BusParams>,
}

/// CAN FD state. Carries both nominal and data-phase bitrates, sample
/// points, and any explicit timing overrides.
pub struct Fd {
    pub(crate) nominal_hz: u32,
    pub(crate) data_hz: u32,
    pub(crate) sample_point: f32,
    pub(crate) data_sample_point: f32,
    pub(crate) custom_params: Option<BusParams>,
    pub(crate) custom_fd_params: Option<BusParamsFd>,
}
