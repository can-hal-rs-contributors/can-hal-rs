//! # can-hal-kvaser
//!
//! KVASER CANlib backend for [`can-hal-rs`](https://crates.io/crates/can-hal-rs).
//!
//! Loads `libcanlib.so` (Linux) or `canlib32.dll` (Windows) at runtime and implements
//! the full `can-hal` trait set for KVASER USB, PCIe, and LAN adapters.
//!
//! ## Quick start
//!
//! ```rust,no_run
//! use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder, Driver};
//! use can_hal_kvaser::KvaserDriver;
//!
//! let driver = KvaserDriver::new().expect("CANlib not found");
//! let mut channel = driver.channel(0).unwrap().bitrate(500_000).unwrap().connect().unwrap();
//!
//! let id = CanId::new_standard(0x100).unwrap();
//! let frame = CanFrame::new(id, &[0x01, 0x02]).unwrap();
//! channel.transmit(&frame).unwrap();
//! ```

pub mod channel;
pub mod driver;
pub mod error;

mod convert;
mod event;
mod ffi;
mod library;

pub use channel::KvaserChannel;
pub use driver::{BusParams, BusParamsFd, KvaserChannelBuilder, KvaserDriver};
pub use error::{KvaserError, KvaserStatus};

// Compile-time assertion: channel must be Send so it can be moved across threads.
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<KvaserChannel>();
};
