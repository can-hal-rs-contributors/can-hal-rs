//! # can-hal-pcan
//!
//! PCAN-Basic backend for [`can_hal`] traits (Windows and Linux).
//!
//! This crate provides [`PcanDriver`] and [`PcanChannel`] which implement the
//! hardware-agnostic CAN traits defined in `can-hal`, enabling portable CAN
//! application code to run on systems with Peak System PCAN hardware.
//!
//! The PCAN-Basic library (`PCANBasic.dll` on Windows, `libpcanbasic.so` on
//! Linux) is loaded dynamically at runtime — no compile-time link is required.
//!
//! # Example
//!
//! ```rust,ignore
//! use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder};
//! use can_hal_pcan::PcanDriver;
//!
//! let driver = PcanDriver::new()?;
//! let mut channel = driver
//!     .channel(0)?
//!     .bitrate(500_000)?
//!     .connect()?;
//!
//! let id = CanId::new_standard(0x123).unwrap();
//! let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
//! channel.transmit(&frame)?;
//! ```
//!
//! # CAN FD
//!
//! Use [`bitrate()`](can_hal::ChannelBuilder::bitrate) and
//! [`data_bitrate()`](can_hal::ChannelBuilder::data_bitrate) to open an FD
//! channel. Timing parameters are derived automatically for common bitrates
//! (80 MHz clock).
//!
//! ```rust,ignore
//! use can_hal::{ChannelBuilder, TransmitFd, CanId, CanFdFrame};
//! use can_hal_pcan::PcanDriver;
//!
//! let driver = PcanDriver::new()?;
//! let mut channel = driver
//!     .channel(0)?
//!     .bitrate(500_000)?
//!     .data_bitrate(4_000_000)?
//!     .connect()?;
//!
//! let id = CanId::new_extended(0x18DA00F1).unwrap();
//! let frame = CanFdFrame::new(id, &[0x10, 0x03], true, false).unwrap();
//! channel.transmit_fd(&frame)?;
//! ```
//!
//! For custom timing, use the backend-specific
//! [`fd_timing_string()`](PcanChannelBuilder::fd_timing_string) instead.
//!
//! # Prerequisites
//!
//! Install the PCAN-Basic library from
//! [Peak System](https://www.peak-system.com/PCAN-Basic.239.0.html):
//!
//! - **Windows**: `PCANBasic.dll` must be in the system PATH or application
//!   directory.
//! - **Linux**: `libpcanbasic.so` must be installed. Build from the
//!   [PCAN-Basic Linux](https://www.peak-system.com/PCAN-Basic-Linux.433.0.html)
//!   package.

pub mod channel;
pub mod driver;
pub mod error;

mod convert;
mod event;
mod ffi;
mod library;

pub use channel::PcanChannel;
pub use driver::{PcanBusType, PcanChannelBuilder, PcanDriver};
pub use error::PcanError;

// Compile-time assertion: channel must be Send so it can be moved across threads.
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<PcanChannel>();
};
