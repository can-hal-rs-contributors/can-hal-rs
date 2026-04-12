//! # can-hal
//!
//! Hardware-agnostic traits for CAN bus communication.
//!
//! This crate defines portable types and traits that CAN hardware backends
//! (SocketCAN, PCAN, Kvaser, etc.) implement so that application code can be
//! written once and run on any supported hardware.
//!
//! ## `no_std` support
//!
//! This crate is `no_std`-compatible. The `std` feature (enabled by default)
//! is not required for any of the trait definitions or frame types. Disable
//! default features to use in embedded / `no_std` contexts:
//!
//! ```toml
//! [dependencies]
//! can-hal-rs = { version = "0.3", default-features = false }
//! ```
//!
//! ## Quick start
//!
//! ```rust
//! use can_hal::{CanId, CanFrame};
//!
//! let id = CanId::new_standard(0x123).unwrap();
//! let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
//! assert_eq!(frame.id(), id);
//! assert_eq!(frame.data(), &[0xDE, 0xAD]);
//! ```

#![no_std]

#[cfg(feature = "std")]
extern crate std;

pub mod bus;
pub mod channel;
pub mod driver;
pub mod error;
pub mod filter;
pub mod frame;
pub mod id;

#[cfg(feature = "async")]
pub mod async_channel;

// Re-export core types at crate root for convenience.
pub use bus::{BusState, BusStatus, ErrorCounters};
pub use channel::{Receive, ReceiveFd, Transmit, TransmitFd};
pub use driver::{ChannelBuilder, Driver, DriverFd};
pub use error::CanError;
pub use filter::{Filter, Filterable};
pub use frame::{CanFdFrame, CanFrame, Frame, Timestamped};
pub use id::CanId;

#[cfg(feature = "async")]
pub use async_channel::{AsyncReceive, AsyncReceiveFd, AsyncTransmit, AsyncTransmitFd};

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::fmt;
    use std::string::String;
    use std::time::{Duration, Instant};
    use std::vec::Vec;

    // -- Mock error type --

    #[derive(Debug)]
    struct MockError(String);

    impl fmt::Display for MockError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "MockError: {}", self.0)
        }
    }

    impl std::error::Error for MockError {}

    // -- Mock channel --

    struct MockChannel {
        tx_log: Vec<CanFrame>,
        rx_queue: VecDeque<CanFrame>,
    }

    impl MockChannel {
        fn new() -> Self {
            MockChannel {
                tx_log: Vec::new(),
                rx_queue: VecDeque::new(),
            }
        }

        fn push_rx(&mut self, frame: CanFrame) {
            self.rx_queue.push_back(frame);
        }
    }

    impl Transmit for MockChannel {
        type Error = MockError;

        fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
            self.tx_log.push(frame.clone());
            Ok(())
        }
    }

    impl Receive for MockChannel {
        type Error = MockError;
        type Timestamp = Instant;

        fn receive(&mut self) -> Result<Timestamped<CanFrame, Instant>, Self::Error> {
            self.rx_queue
                .pop_front()
                .map(|f| Timestamped::new(f, Instant::now()))
                .ok_or_else(|| MockError("no frames available".into()))
        }

        fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame, Instant>>, Self::Error> {
            Ok(self
                .rx_queue
                .pop_front()
                .map(|f| Timestamped::new(f, Instant::now())))
        }

        fn receive_timeout(
            &mut self,
            _timeout: Duration,
        ) -> Result<Option<Timestamped<CanFrame, Instant>>, Self::Error> {
            // Mock: just return immediately like try_receive.
            self.try_receive()
        }
    }

    // -- Mock driver & builder --

    struct MockDriver;
    struct MockBuilder {
        _bitrate: Option<u32>,
        _data_bitrate: Option<u32>,
        _sample_point: Option<f32>,
    }

    impl Driver for MockDriver {
        type Channel = MockChannel;
        type Builder = MockBuilder;
        type Error = MockError;

        fn channel(&self, _index: u32) -> Result<Self::Builder, Self::Error> {
            Ok(MockBuilder {
                _bitrate: None,
                _data_bitrate: None,
                _sample_point: None,
            })
        }
    }

    impl ChannelBuilder for MockBuilder {
        type Channel = MockChannel;
        type Error = MockError;

        fn bitrate(mut self, bitrate: u32) -> Result<Self, Self::Error> {
            self._bitrate = Some(bitrate);
            Ok(self)
        }

        fn data_bitrate(mut self, bitrate: u32) -> Result<Self, Self::Error> {
            self._data_bitrate = Some(bitrate);
            Ok(self)
        }

        fn sample_point(mut self, sample_point: f32) -> Result<Self, Self::Error> {
            self._sample_point = Some(sample_point);
            Ok(self)
        }

        fn connect(self) -> Result<Self::Channel, Self::Error> {
            Ok(MockChannel::new())
        }
    }

    // -- Tests --

    #[test]
    fn can_id_standard_valid() {
        let id = CanId::new_standard(0x123).unwrap();
        assert_eq!(id.raw(), 0x123);
        assert!(id.is_standard());
    }

    #[test]
    fn can_id_standard_max() {
        assert!(CanId::new_standard(0x7FF).is_some());
        assert!(CanId::new_standard(0x800).is_none());
    }

    #[test]
    fn can_id_extended_valid() {
        let id = CanId::new_extended(0x1234_5678).unwrap();
        assert_eq!(id.raw(), 0x1234_5678);
        assert!(id.is_extended());
    }

    #[test]
    fn can_id_extended_max() {
        assert!(CanId::new_extended(0x1FFF_FFFF).is_some());
        assert!(CanId::new_extended(0x2000_0000).is_none());
    }

    #[test]
    fn can_frame_valid() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new(id, &[1, 2, 3]).unwrap();
        assert_eq!(frame.id(), id);
        assert_eq!(frame.len(), 3);
        assert_eq!(frame.data(), &[1, 2, 3]);
    }

    #[test]
    fn can_frame_empty() {
        let id = CanId::new_standard(0x000).unwrap();
        let frame = CanFrame::new(id, &[]).unwrap();
        assert_eq!(frame.len(), 0);
        assert_eq!(frame.data(), &[]);
    }

    #[test]
    fn can_frame_too_long() {
        let id = CanId::new_standard(0x100).unwrap();
        assert!(CanFrame::new(id, &[0; 9]).is_none());
    }

    #[test]
    fn can_fd_frame_valid() {
        let id = CanId::new_extended(0x100).unwrap();
        let data = [0xAA; 32];
        let frame = CanFdFrame::new(id, &data, true, false).unwrap();
        assert_eq!(frame.len(), 32);
        assert_eq!(frame.data(), &data);
        assert!(frame.brs());
        assert!(!frame.esi());
    }

    #[test]
    fn can_fd_frame_invalid_dlc() {
        let id = CanId::new_standard(0x100).unwrap();
        // 9 is not a valid FD DLC
        assert!(CanFdFrame::new(id, &[0; 9], false, false).is_none());
    }

    #[test]
    fn frame_enum_accessors() {
        let id = CanId::new_standard(0x200).unwrap();
        let classic = CanFrame::new(id, &[0xFF]).unwrap();
        let frame = Frame::Can(classic);
        assert_eq!(frame.id(), id);
        assert_eq!(frame.len(), 1);
        assert_eq!(frame.data(), &[0xFF]);
    }

    #[test]
    fn timestamped_wrapper() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new(id, &[1, 2]).unwrap();
        let now = Instant::now();
        let ts = Timestamped::new(frame.clone(), now);
        assert_eq!(ts.frame(), &frame);
        assert_eq!(*ts.timestamp(), now);
        assert_eq!(ts.into_frame(), frame);
    }

    #[test]
    fn mock_transmit_receive() {
        let mut ch = MockChannel::new();
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new(id, &[1, 2]).unwrap();

        ch.transmit(&frame).unwrap();
        assert_eq!(ch.tx_log.len(), 1);
        assert_eq!(ch.tx_log[0], frame);

        // try_receive on empty queue
        assert!(ch.try_receive().unwrap().is_none());

        // Push and receive
        ch.push_rx(frame.clone());
        let received = ch.receive().unwrap();
        assert_eq!(received.into_frame(), frame);
    }

    #[test]
    fn mock_receive_timeout() {
        let mut ch = MockChannel::new();
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new(id, &[0xAB]).unwrap();

        // Empty queue returns None
        let result = ch.receive_timeout(Duration::from_millis(100)).unwrap();
        assert!(result.is_none());

        // With frame available
        ch.push_rx(frame.clone());
        let result = ch.receive_timeout(Duration::from_millis(100)).unwrap();
        assert_eq!(result.unwrap().into_frame(), frame);
    }

    #[test]
    fn mock_driver_builder() {
        let drv = MockDriver;
        let mut channel = drv
            .channel(0)
            .unwrap()
            .bitrate(500_000)
            .unwrap()
            .data_bitrate(2_000_000)
            .unwrap()
            .sample_point(0.875)
            .unwrap()
            .connect()
            .unwrap();

        let id = CanId::new_standard(0x7FF).unwrap();
        let frame = CanFrame::new(id, &[0xCA, 0xFE]).unwrap();
        channel.transmit(&frame).unwrap();
        assert_eq!(channel.tx_log.len(), 1);
    }

    // Generic function using trait bounds.
    fn send_and_recv<T: Transmit<Error = E> + Receive<Error = E>, E: CanError>(
        channel: &mut T,
        frame: &CanFrame,
    ) -> Result<Option<CanFrame>, E> {
        channel.transmit(frame)?;
        Ok(channel.try_receive()?.map(|ts| ts.into_frame()))
    }

    #[test]
    fn generic_function_over_traits() {
        let mut ch = MockChannel::new();
        let id = CanId::new_standard(0x42).unwrap();
        let frame = CanFrame::new(id, &[0xBE, 0xEF]).unwrap();

        ch.push_rx(frame.clone());
        let result = send_and_recv(&mut ch, &frame).unwrap();
        assert_eq!(result, Some(frame));
    }

    #[test]
    fn bus_state_enum() {
        let state = BusState::ErrorActive;
        assert_eq!(state, BusState::ErrorActive);
        assert_ne!(state, BusState::BusOff);

        let counters = ErrorCounters {
            transmit: 0,
            receive: 0,
        };
        assert_eq!(counters.transmit, 0);
    }
}
