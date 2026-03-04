use can_hal::driver::{ChannelBuilder, Driver};

use crate::channel::SocketCanChannel;
use crate::error::SocketCanError;

/// How the CAN interface is identified.
enum InterfaceSpec {
    Index(u32),
    Name(String),
}

/// SocketCAN driver — factory for opening CAN channels.
///
/// Implements the [`Driver`] trait using kernel interface indices.
/// Also provides [`channel_by_name`](SocketCanDriver::channel_by_name) for
/// opening by interface name (e.g., `"can0"`, `"vcan0"`).
///
/// # Example
///
/// ```rust,no_run
/// use can_hal::{Driver, ChannelBuilder};
/// use can_hal_socketcan::SocketCanDriver;
///
/// let driver = SocketCanDriver::new();
/// let mut channel = driver
///     .channel_by_name("vcan0")
///     .unwrap()
///     .bitrate(500_000)
///     .unwrap()
///     .connect()
///     .unwrap();
/// ```
pub struct SocketCanDriver;

impl SocketCanDriver {
    pub fn new() -> Self {
        SocketCanDriver
    }

    /// Begin configuring a channel on the named interface.
    pub fn channel_by_name(&self, name: &str) -> Result<SocketCanChannelBuilder, SocketCanError> {
        Ok(SocketCanChannelBuilder {
            interface: InterfaceSpec::Name(name.to_string()),
            bitrate: None,
            data_bitrate: None,
            sample_point: None,
        })
    }
}

impl Default for SocketCanDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl Driver for SocketCanDriver {
    type Channel = SocketCanChannel;
    type Builder = SocketCanChannelBuilder;
    type Error = SocketCanError;

    fn channel(&self, index: u32) -> Result<Self::Builder, Self::Error> {
        Ok(SocketCanChannelBuilder {
            interface: InterfaceSpec::Index(index),
            bitrate: None,
            data_bitrate: None,
            sample_point: None,
        })
    }
}

/// Builder for configuring a SocketCAN channel before connecting.
///
/// The `bitrate`, `data_bitrate`, and `sample_point` methods store values but
/// do **not** apply them — SocketCAN bitrate configuration is done at the OS
/// level via `ip link set` or netlink, not through the socket API. Configure
/// your interface before calling [`connect`](ChannelBuilder::connect).
pub struct SocketCanChannelBuilder {
    interface: InterfaceSpec,
    bitrate: Option<u32>,
    data_bitrate: Option<u32>,
    sample_point: Option<f32>,
}

impl ChannelBuilder for SocketCanChannelBuilder {
    type Channel = SocketCanChannel;
    type Error = SocketCanError;

    fn bitrate(mut self, bitrate: u32) -> Result<Self, Self::Error> {
        self.bitrate = Some(bitrate);
        Ok(self)
    }

    fn data_bitrate(mut self, bitrate: u32) -> Result<Self, Self::Error> {
        self.data_bitrate = Some(bitrate);
        Ok(self)
    }

    fn sample_point(mut self, sample_point: f32) -> Result<Self, Self::Error> {
        self.sample_point = Some(sample_point);
        Ok(self)
    }

    fn connect(self) -> Result<Self::Channel, Self::Error> {
        match self.interface {
            InterfaceSpec::Index(idx) => SocketCanChannel::open_iface(idx),
            InterfaceSpec::Name(ref name) => SocketCanChannel::open(name),
        }
    }
}
