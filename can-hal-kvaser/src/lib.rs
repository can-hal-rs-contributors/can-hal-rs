//! # can-hal-kvaser
//!
//! KVASER CANlib backend for [`can-hal-rs`](https://crates.io/crates/can-hal-rs).
//!
//! Loads `libcanlib.so` (Linux) or `canlib32.dll` (Windows) at runtime and
//! provides [`KvaserDriver`] and [`KvaserChannel<Mode>`]. The channel is
//! parameterized on a type-state marker - [`mode::Classic`] or
//! [`mode::Fd`] - so invalid combinations are compile errors.
//!
//! ## Classic CAN example
//!
//! ```rust,no_run
//! use can_hal::{CanId, CanFrame, Transmit, Receive};
//! use can_hal_kvaser::KvaserDriver;
//!
//! let driver = KvaserDriver::new().expect("CANlib not found");
//! let mut channel = driver
//!     .channel(0)
//!     .unwrap()
//!     .classic(500_000)
//!     .unwrap()
//!     .connect()
//!     .unwrap();
//!
//! let id = CanId::new_standard(0x100).unwrap();
//! let frame = CanFrame::new(id, &[0x01, 0x02]).unwrap();
//! channel.transmit(&frame).unwrap();
//! ```
//!
//! ## CAN FD example
//!
//! ```rust,no_run
//! use can_hal::{CanId, CanFdFrame, TransmitFd};
//! use can_hal_kvaser::KvaserDriver;
//!
//! let driver = KvaserDriver::new().unwrap();
//! let mut channel = driver
//!     .channel(0)
//!     .unwrap()
//!     .fd(500_000, 4_000_000)
//!     .unwrap()
//!     .connect()
//!     .unwrap();
//! ```

pub mod channel;
pub mod driver;
pub mod error;
pub mod mode;

mod convert;
mod event;
mod ffi;
mod library;

pub use channel::KvaserChannel;
pub use driver::{BusParams, BusParamsFd, KvaserChannelBuilder, KvaserDriver, KVASER_CLOCK_HZ};
pub use error::{KvaserError, KvaserStatus};
pub use mode::{Classic, ClassicExplicit, Fd, FdExplicit, Initial};

// Compile-time assertion: both channel modes must be Send.
const _: fn() = || {
    const fn assert_send<T: Send>() {}
    assert_send::<KvaserChannel<Classic>>();
    assert_send::<KvaserChannel<Fd>>();
};
