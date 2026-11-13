//! The FF_DL in a First Frame comes from the peer, so `receive()` must not
//! reserve memory for it up front: one 8-byte long FF announcing `u32::MAX`
//! bytes must not turn into a 4 GiB allocation request.
//!
//! This is its own test binary because it installs a `#[global_allocator]`
//! that records, per thread, the largest single allocation request. Checking
//! `Vec::capacity` can't see the reservation (the Vec is dropped when
//! `receive()` errors), and Linux overcommit lets a 4 GiB request succeed, so
//! the request size is the only reliable signal.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

use can_hal::channel::{Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};
use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig, IsoTpError, IsoTpFdChannel};

// ---------------------------------------------------------------------------
// Allocation tracking
// ---------------------------------------------------------------------------

thread_local! {
    static LARGEST: Cell<usize> = const { Cell::new(0) };
}

fn note(size: usize) {
    // `try_with`, not `with`: never panic inside the allocator, even while
    // the thread is being torn down (there is nothing to record then).
    LARGEST
        .try_with(|l| l.set(l.get().max(size)))
        .unwrap_or_default();
}

struct Tracking;

// SAFETY: every method forwards to `System` unchanged; `note` only touches a
// const-initialized thread-local `Cell<usize>`, which never allocates.
unsafe impl GlobalAlloc for Tracking {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        note(layout.size());
        // SAFETY: forwarded with the caller's guarantees.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        note(layout.size());
        // SAFETY: forwarded with the caller's guarantees.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: forwarded with the caller's guarantees.
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        note(new_size);
        // SAFETY: forwarded with the caller's guarantees.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: Tracking = Tracking;

/// Upper bound for any single allocation `receive()` may request while
/// handling the long FF below. Generous: the point is "not 4 GiB".
const LIMIT: usize = 1 << 20;

fn reset() {
    LARGEST.with(|l| l.set(0));
}

fn largest() -> usize {
    LARGEST.with(Cell::get)
}

// ---------------------------------------------------------------------------
// Mocks
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct MockError;

impl std::fmt::Display for MockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MockError")
    }
}

impl std::error::Error for MockError {}

/// Hands out queued frames and discards everything transmitted.
struct Mock<F> {
    rx_queue: VecDeque<F>,
}

impl Transmit for Mock<CanFrame> {
    type Error = MockError;
    fn transmit(&mut self, _frame: &CanFrame) -> Result<(), MockError> {
        Ok(())
    }
}

impl Receive for Mock<CanFrame> {
    type Error = MockError;
    type Timestamp = Instant;

    fn receive(&mut self) -> Result<Timestamped<CanFrame, Instant>, MockError> {
        self.try_receive()?.ok_or(MockError)
    }

    fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame, Instant>>, MockError> {
        Ok(self
            .rx_queue
            .pop_front()
            .map(|f| Timestamped::new(f, Instant::now())))
    }

    fn receive_timeout(
        &mut self,
        _timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame, Instant>>, MockError> {
        self.try_receive()
    }
}

impl TransmitFd for Mock<Frame> {
    type Error = MockError;
    fn transmit_fd(&mut self, _frame: &CanFdFrame) -> Result<(), MockError> {
        Ok(())
    }
}

impl ReceiveFd for Mock<Frame> {
    type Error = MockError;
    type Timestamp = Instant;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame, Instant>, MockError> {
        self.try_receive_fd()?.ok_or(MockError)
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame, Instant>>, MockError> {
        Ok(self
            .rx_queue
            .pop_front()
            .map(|f| Timestamped::new(f, Instant::now())))
    }

    fn receive_fd_timeout(
        &mut self,
        _timeout: Duration,
    ) -> Result<Option<Timestamped<Frame, Instant>>, MockError> {
        self.try_receive_fd()
    }
}

#[cfg(feature = "async")]
#[allow(clippy::manual_async_fn)] // match the traits' `impl Future + Send` shape
mod async_mock {
    use super::*;
    use can_hal::async_channel::{AsyncReceive, AsyncReceiveFd, AsyncTransmit, AsyncTransmitFd};
    use std::future::Future;

    impl AsyncTransmit for Mock<CanFrame> {
        type Error = MockError;
        fn transmit(
            &mut self,
            _frame: &CanFrame,
        ) -> impl Future<Output = Result<(), MockError>> + Send {
            async { Ok(()) }
        }
    }

    impl AsyncReceive for Mock<CanFrame> {
        type Error = MockError;
        type Timestamp = Instant;
        fn receive(
            &mut self,
        ) -> impl Future<Output = Result<Timestamped<CanFrame, Instant>, MockError>> + Send
        {
            let next = self.rx_queue.pop_front();
            async move {
                next.map(|f| Timestamped::new(f, Instant::now()))
                    .ok_or(MockError)
            }
        }
    }

    impl AsyncTransmitFd for Mock<Frame> {
        type Error = MockError;
        fn transmit_fd(
            &mut self,
            _frame: &CanFdFrame,
        ) -> impl Future<Output = Result<(), MockError>> + Send {
            async { Ok(()) }
        }
    }

    impl AsyncReceiveFd for Mock<Frame> {
        type Error = MockError;
        type Timestamp = Instant;
        fn receive_fd(
            &mut self,
        ) -> impl Future<Output = Result<Timestamped<Frame, Instant>, MockError>> + Send {
            let next = self.rx_queue.pop_front();
            async move {
                next.map(|f| Timestamped::new(f, Instant::now()))
                    .ok_or(MockError)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Frames: a long FF announcing u32::MAX bytes, then a CF with the wrong SN so
// receive() ends at once with SequenceError instead of waiting for 4 GiB.
// ---------------------------------------------------------------------------

fn tx_id() -> CanId {
    CanId::new_standard(0x7E0).unwrap()
}

fn rx_id() -> CanId {
    CanId::new_standard(0x7E8).unwrap()
}

fn config() -> IsoTpConfig {
    IsoTpConfig {
        timeout: Duration::from_millis(50),
        ..IsoTpConfig::new(tx_id(), rx_id())
    }
}

const LONG_FF_PCI: [u8; 6] = [0x10, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];

fn classic_frames() -> VecDeque<CanFrame> {
    let mut ff = [0xAAu8; 8];
    ff[..6].copy_from_slice(&LONG_FF_PCI);
    let bad_cf = [0x22u8, 0, 0, 0, 0, 0, 0, 0];
    VecDeque::from([
        CanFrame::new(rx_id(), &ff).unwrap(),
        CanFrame::new(rx_id(), &bad_cf).unwrap(),
    ])
}

fn fd_frames() -> VecDeque<Frame> {
    let mut ff = [0xAAu8; 64];
    ff[..6].copy_from_slice(&LONG_FF_PCI);
    let mut bad_cf = [0u8; 64];
    bad_cf[0] = 0x22;
    VecDeque::from([
        Frame::Fd(CanFdFrame::new(rx_id(), &ff, true, false).unwrap()),
        Frame::Fd(CanFdFrame::new(rx_id(), &bad_cf, true, false).unwrap()),
    ])
}

fn assert_sequence_error<E: std::fmt::Debug + Send + Sync + 'static>(
    result: &Result<Vec<u8>, IsoTpError<E>>,
) {
    assert!(
        matches!(
            result,
            Err(IsoTpError::SequenceError {
                expected: 1,
                got: 2
            })
        ),
        "unexpected result: {result:?}"
    );
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn classic_long_ff_does_not_reserve_ff_dl() {
    let mut ch = IsoTpChannel::new(
        Mock {
            rx_queue: classic_frames(),
        },
        config(),
    );
    reset();
    let result = ch.receive();
    let largest = largest();
    assert_sequence_error(&result);
    assert!(
        largest < LIMIT,
        "receive() requested {largest} bytes at once"
    );
}

#[test]
fn fd_long_ff_does_not_reserve_ff_dl() {
    let mut ch = IsoTpFdChannel::new(
        Mock {
            rx_queue: fd_frames(),
        },
        config(),
    );
    reset();
    let result = ch.receive();
    let largest = largest();
    assert_sequence_error(&result);
    assert!(
        largest < LIMIT,
        "receive() requested {largest} bytes at once"
    );
}

// The async channels never reserved FF_DL up front; these guard that.

#[cfg(feature = "async")]
#[tokio::test]
async fn async_classic_long_ff_does_not_reserve_ff_dl() {
    let mut ch = can_hal_isotp::AsyncIsoTpChannel::new(
        Mock {
            rx_queue: classic_frames(),
        },
        config(),
    );
    reset();
    let result = ch.receive().await;
    let largest = largest();
    assert_sequence_error(&result);
    assert!(
        largest < LIMIT,
        "receive() requested {largest} bytes at once"
    );
}

#[cfg(feature = "async")]
#[tokio::test]
async fn async_fd_long_ff_does_not_reserve_ff_dl() {
    let mut ch = can_hal_isotp::AsyncIsoTpFdChannel::new(
        Mock {
            rx_queue: fd_frames(),
        },
        config(),
    );
    reset();
    let result = ch.receive().await;
    let largest = largest();
    assert_sequence_error(&result);
    assert!(
        largest < LIMIT,
        "receive() requested {largest} bytes at once"
    );
}
