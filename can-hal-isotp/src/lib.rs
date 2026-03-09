//! # can-hal-isotp
//!
//! ISO-TP (ISO 15765-2) transport layer for the [`can_hal`] ecosystem.
//!
//! Provides segmentation and reassembly of payloads larger than a single CAN frame,
//! using the Single Frame / First Frame / Consecutive Frame / Flow Control protocol.
//!
//! # Quick start
//!
//! ```rust,no_run
//! use can_hal::CanId;
//! use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
//!
//! // Assumes `channel` implements `can_hal::channel::{Transmit, Receive}`.
//! // let channel = SocketCanChannel::open("vcan0").unwrap();
//! // let config = IsoTpConfig::new(
//! //     CanId::new_standard(0x7E0).unwrap(),
//! //     CanId::new_standard(0x7E8).unwrap(),
//! // );
//! // let mut isotp = IsoTpChannel::new(channel, config);
//! // isotp.send(&[0x01, 0x02, 0x03]).unwrap();
//! // let data = isotp.receive().unwrap();
//! ```
//!
//! # Addressing modes
//!
//! - **Normal** (default): PCI bytes immediately follow the CAN ID. Maximum 7 bytes per SF.
//! - **Extended**: A target address byte precedes the PCI. Maximum 6 bytes per SF.
//! - **Functional**: Broadcast using [`IsoTpConfig::functional_id`] via
//!   [`IsoTpChannel::send_functional`]. Restricted to single frames only.
//!
//! # CAN FD
//!
//! Use [`IsoTpFdChannel`] for FD-capable hardware. SF payloads up to 62 bytes,
//! CF payloads up to 63 bytes.
//!
//! # Feature flags
//!
//! - `async`: Enables `AsyncIsoTpChannel` and `AsyncIsoTpFdChannel` backed by Tokio.

pub mod channel;
pub mod config;
pub mod error;
pub mod fd_channel;
pub mod frame;

#[cfg(feature = "async")]
pub mod async_channel;
#[cfg(feature = "async")]
pub mod async_fd_channel;

pub use channel::IsoTpChannel;
pub use config::{AddressingMode, IsoTpConfig};
pub use error::IsoTpError;
pub use fd_channel::IsoTpFdChannel;
pub use frame::{FcFlag, IsoTpFrame};

#[cfg(feature = "async")]
pub use async_channel::AsyncIsoTpChannel;
#[cfg(feature = "async")]
pub use async_fd_channel::AsyncIsoTpFdChannel;

#[cfg(test)]
mod tests;

#[cfg(all(test, feature = "async"))]
mod async_tests;
