//! Type-state markers for the PCAN channel builder and channel.
//!
//! These zero-sized types are used as a type parameter on
//! [`PcanChannelBuilder`](crate::PcanChannelBuilder) and
//! [`PcanChannel`](crate::PcanChannel) to encode which configuration path
//! the channel is on. The compiler then restricts which methods are
//! callable in each state.

/// Initial state — the builder hasn't picked classic vs FD yet.
pub struct Initial;

/// Classic-CAN state — bitrate is set; only `connect()` is available.
pub struct Classic {
    pub(crate) bitrate_const: u16,
}

/// CAN FD state — nominal + data bitrates set; sample points and explicit
/// timing overrides may be applied before `connect()`.
pub struct Fd {
    pub(crate) nominal_hz: u32,
    pub(crate) data_hz: u32,
    pub(crate) sample_point: f32,
    pub(crate) data_sample_point: f32,
    pub(crate) explicit_timing: Option<crate::driver::PcanFdTiming>,
}
