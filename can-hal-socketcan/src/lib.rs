//! # can-hal-socketcan
//!
//! Linux SocketCAN backend for [`can_hal`] traits.
//!
//! This crate provides [`SocketCanDriver`] and [`SocketCanChannel`] which
//! implement the hardware-agnostic CAN traits defined in `can-hal`, enabling
//! portable CAN application code to run on Linux systems with SocketCAN
//! interfaces.
//!
//! # Example
//!
//! ```rust,no_run
//! use can_hal::{CanId, CanFrame, Transmit, Receive};
//! use can_hal_socketcan::SocketCanDriver;
//!
//! let driver = SocketCanDriver::new();
//! let mut channel = driver
//!     .channel_by_name("vcan0")
//!     .connect()
//!     .unwrap();
//!
//! let id = CanId::new_standard(0x123).unwrap();
//! let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
//! channel.transmit(&frame).unwrap();
//! ```
//!
//! # Bitrate Configuration
//!
//! SocketCAN bitrate is configured at the OS level, not through the socket
//! API — the builder deliberately has no timing methods. Use `ip link set`
//! or netlink before opening a channel:
//!
//! ```bash
//! sudo ip link set can0 type can bitrate 500000
//! sudo ip link set can0 up
//! ```

pub mod channel;
pub mod driver;
pub mod error;

mod convert;

pub use channel::SocketCanChannel;
pub use driver::{SocketCanChannelBuilder, SocketCanDriver};
pub use error::SocketCanError;

// Compile-time assertion: channel must be Send so it can be moved across threads.
const _: fn() = || {
    const fn assert_send<T: Send>() {}
    assert_send::<SocketCanChannel>();
};
