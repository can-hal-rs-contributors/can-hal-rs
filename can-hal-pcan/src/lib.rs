//! # can-hal-pcan
//!
//! PCAN-Basic backend for [`can_hal`] (Windows and Linux).
//!
//! Provides [`PcanDriver`] and [`PcanChannel<Mode>`] which implement the
//! hardware-agnostic channel traits defined in `can-hal`. The channel is
//! parameterized on a type-state marker - [`mode::Classic`] or
//! [`mode::Fd`] - so invalid combinations (e.g., calling `transmit_fd` on a
//! classic channel) are compile errors, not runtime ones.
//!
//! The PCAN-Basic library (`PCANBasic.dll` on Windows, `libpcanbasic.so` on
//! Linux) is loaded dynamically at runtime - no compile-time link is required.
//!
//! # Classic CAN example
//!
//! ```rust,ignore
//! use can_hal::{CanId, CanFrame, Transmit, Receive};
//! use can_hal_pcan::{PcanDriver, ClassicBitrate};
//!
//! let driver = PcanDriver::new()?;
//! let mut channel = driver
//!     .channel(0)?
//!     .classic(ClassicBitrate::Br500K)
//!     .connect()?;
//!
//! let id = CanId::new_standard(0x123).unwrap();
//! let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
//! channel.transmit(&frame)?;
//! ```
//!
//! # CAN FD example
//!
//! Sample points default to 70% (nominal) and 80% (data). Override with
//! `sample_point()` / `data_sample_point()` on the FD-mode builder; for raw
//! per-segment control use `PcanChannelBuilder::<Initial>::fd_explicit` with
//! a [`PcanFdTiming`] value.
//!
//! ```rust,ignore
//! use can_hal::{TransmitFd, CanId, CanFdFrame};
//! use can_hal_pcan::PcanDriver;
//!
//! let driver = PcanDriver::new()?;
//! let mut channel = driver
//!     .channel(0)?
//!     .fd(500_000, 4_000_000)?
//!     .connect()?;
//!
//! let id = CanId::new_extended(0x18DA00F1).unwrap();
//! let frame = CanFdFrame::new(id, &[0x10, 0x03], true, false).unwrap();
//! channel.transmit_fd(&frame)?;
//! ```
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
pub mod mode;

mod convert;
mod event;
mod ffi;
mod library;

pub use channel::PcanChannel;
pub use driver::{
    ClassicBitrate, PcanBusType, PcanChannelBuilder, PcanDriver, PcanFdTiming, PcanPhaseTiming,
    PCAN_CLOCK_HZ,
};
pub use error::PcanError;
pub use mode::{Classic, Fd, FdExplicit, Initial};

// Compile-time assertion: both channel modes must be Send so they can be moved across threads.
const _: fn() = || {
    const fn assert_send<T: Send>() {}
    assert_send::<PcanChannel<Classic>>();
    assert_send::<PcanChannel<Fd>>();
};
