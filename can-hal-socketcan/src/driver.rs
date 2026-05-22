use crate::channel::SocketCanChannel;
use crate::error::SocketCanError;

/// How the CAN interface is identified.
enum InterfaceSpec {
    Index(u32),
    Name(String),
}

/// SocketCAN driver - factory for opening CAN channels.
///
/// SocketCAN bitrate, sample point, and FD timing are configured at the OS
/// level via `ip link set` or netlink - not through the socket API - so this
/// builder intentionally has no timing methods. Configure the interface
/// before calling [`connect`](SocketCanChannelBuilder::connect):
///
/// ```bash
/// sudo ip link set can0 type can bitrate 500000
/// sudo ip link set can0 up
/// ```
///
/// # Example
///
/// ```rust,no_run
/// use can_hal_socketcan::SocketCanDriver;
///
/// let driver = SocketCanDriver::new();
/// let mut channel = driver
///     .channel_by_name("vcan0")
///     .connect()
///     .unwrap();
/// ```
pub struct SocketCanDriver;

impl SocketCanDriver {
    #[must_use]
    pub const fn new() -> Self {
        Self
    }

    /// Begin configuring a channel on the named interface.
    #[must_use]
    pub fn channel_by_name(&self, name: &str) -> SocketCanChannelBuilder {
        SocketCanChannelBuilder {
            interface: InterfaceSpec::Name(name.to_string()),
        }
    }

    /// Begin configuring a channel at the given 0-based interface index.
    #[must_use]
    pub const fn channel(&self, index: u32) -> SocketCanChannelBuilder {
        SocketCanChannelBuilder {
            interface: InterfaceSpec::Index(index),
        }
    }
}

impl Default for SocketCanDriver {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for opening a SocketCAN channel.
///
/// SocketCAN bitrate is OS-managed, so this builder only chooses the
/// interface - there are deliberately no methods to set bitrate, sample
/// point, or FD timing.
pub struct SocketCanChannelBuilder {
    interface: InterfaceSpec,
}

impl SocketCanChannelBuilder {
    /// Open the channel and go on-bus.
    pub fn connect(self) -> Result<SocketCanChannel, SocketCanError> {
        match self.interface {
            InterfaceSpec::Index(idx) => SocketCanChannel::open_iface(idx),
            InterfaceSpec::Name(ref name) => SocketCanChannel::open(name),
        }
    }
}
