use crate::channel::{Receive, Transmit};
use crate::error::CanError;

/// Factory that opens channels on a CAN interface/device.
pub trait Driver {
    type Channel: Transmit + Receive;
    type Builder: ChannelBuilder<Channel = Self::Channel>;
    type Error: CanError;

    /// Begin configuring a channel on the given interface index.
    fn channel(&self, index: u32) -> Result<Self::Builder, Self::Error>;
}

/// Builder for configuring a CAN channel before going on-bus.
///
/// All configuration methods are fallible, allowing backends to validate
/// parameters early. Usage:
///
/// ```rust,ignore
/// let channel = driver.channel(0)?
///     .bitrate(500_000)?
///     .data_bitrate(2_000_000)?
///     .connect()?;
/// ```
pub trait ChannelBuilder: Sized {
    type Channel;
    type Error: CanError;

    /// Set the nominal (arbitration phase) bitrate in bits/s.
    fn bitrate(self, bitrate: u32) -> Result<Self, Self::Error>;

    /// Set the data phase bitrate for CAN FD in bits/s.
    fn data_bitrate(self, bitrate: u32) -> Result<Self, Self::Error>;

    /// Set the sample point as a fraction (e.g., 0.875).
    fn sample_point(self, sample_point: f32) -> Result<Self, Self::Error>;

    /// Finalize configuration and go on-bus.
    fn connect(self) -> Result<Self::Channel, Self::Error>;
}
