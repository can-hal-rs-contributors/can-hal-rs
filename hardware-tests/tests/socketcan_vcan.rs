//! SocketCAN software-in-the-loop (SITL) tests over a virtual CAN interface.
//!
//! These tests exercise the `can-hal-socketcan` backend end to end without any
//! physical hardware by using a Linux `vcan` interface. Two sockets opened on
//! the same `vcan0` see each other's frames via SocketCAN local loopback, so a
//! frame written on one channel is received on the other.
//!
//! The whole file is gated to Linux because `can-hal-socketcan` and the `vcan`
//! kernel module only exist there; on other platforms it compiles to an empty
//! test binary.
//!
//! ## Requirements
//!
//! A `vcan0` interface must exist and be up:
//!
//! ```bash
//! sudo modprobe vcan
//! sudo ip link add dev vcan0 type vcan
//! sudo ip link set vcan0 up
//! ```
//!
//! When `vcan0` is absent each test skips cleanly (returns early after printing
//! a notice) rather than failing, so the suite is safe to run on any Linux box.
//! That skip is silent to the test harness, so to stop a broken CI setup step
//! from passing the suite without exercising anything, set
//! `CAN_HAL_REQUIRE_VCAN=1`: a missing or unopenable `vcan0` then becomes a hard
//! failure instead of a skip. CI sets this; local runs leave it unset.
//!
//! This suite is gated behind the off-by-default `vcan` cargo feature so a
//! plain local `cargo test` excludes it entirely. Enable it explicitly (CI
//! does this on the Linux runner, with the interface up and the env var set):
//!
//! ```bash
//! CAN_HAL_REQUIRE_VCAN=1 cargo test -p hardware-tests --features vcan
//! ```
#![cfg(all(target_os = "linux", feature = "vcan"))]

use std::time::{Duration, Instant};

use can_hal::channel::{Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal::filter::{Filter, Filterable};
use can_hal::frame::{CanFdFrame, CanFrame, Frame};
use can_hal::CanId;
use can_hal_socketcan::{SocketCanChannel, SocketCanDriver};

/// Virtual CAN interface the tests transmit and receive on.
const IFACE: &str = "vcan0";

/// Environment variable that, when set to a non-empty value, turns a missing
/// `vcan0` into a hard test failure instead of a silent skip. CI sets it so a
/// broken interface-setup step cannot pass the suite without running anything.
const REQUIRE_ENV: &str = "CAN_HAL_REQUIRE_VCAN";

/// Returns `true` when the suite must fail (rather than skip) if `vcan0` cannot
/// be opened. An empty value counts the same as unset.
fn vcan_required() -> bool {
    std::env::var_os(REQUIRE_ENV).is_some_and(|v| !v.is_empty())
}

/// Open a single channel on the virtual interface, or return `None` so callers
/// can skip when `vcan0` is not configured on this machine. Panics instead of
/// returning `None` when [`vcan_required`] reports the interface is mandatory.
fn open() -> Option<SocketCanChannel> {
    match SocketCanDriver::new().channel_by_name(IFACE).connect() {
        Ok(channel) => Some(channel),
        Err(e) => {
            assert!(
                !vcan_required(),
                "{REQUIRE_ENV} is set but {IFACE} could not be opened: {e}. \
                 Bring the interface up (modprobe vcan; ip link add dev {IFACE} \
                 type vcan; ip link set {IFACE} up) or unset {REQUIRE_ENV}."
            );
            eprintln!("skipping SocketCAN vcan test: cannot open {IFACE}: {e}");
            None
        }
    }
}

/// Open a transmit/receive channel pair, or `None` to skip the test. The `?`
/// short-circuits on the first failure so the skip notice prints once.
fn open_pair() -> Option<(SocketCanChannel, SocketCanChannel)> {
    Some((open()?, open()?))
}

/// Drain classic frames for at most `window`, collecting everything received.
fn drain_classic(channel: &mut SocketCanChannel, window: Duration) -> Vec<CanFrame> {
    let deadline = Instant::now() + window;
    let mut frames = Vec::new();
    while Instant::now() < deadline {
        match channel.receive_timeout(Duration::from_millis(100)) {
            Ok(Some(ts)) => frames.push(ts.into_frame()),
            Ok(None) => {}
            Err(e) => panic!("receive error: {e}"),
        }
    }
    frames
}

// ---------------------------------------------------------------------------
// Classic loopback
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_classic_loopback_standard() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let frame = CanFrame::new(
        CanId::new_standard(0x123).unwrap(),
        &[0xDE, 0xAD, 0xBE, 0xEF],
    )
    .unwrap();
    tx.transmit(&frame).expect("transmit failed");

    let received = rx
        .receive_timeout(Duration::from_secs(1))
        .expect("receive error")
        .expect("receive timeout")
        .into_frame();
    assert_eq!(received.id(), frame.id());
    assert_eq!(received.data(), frame.data());
}

#[test]
fn test_vcan_classic_loopback_extended() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let id = CanId::new_extended(0x18DA_F110).unwrap();
    let frame = CanFrame::new(id, &[0x01, 0x02, 0x03]).unwrap();
    tx.transmit(&frame).expect("transmit failed");

    let received = rx
        .receive_timeout(Duration::from_secs(1))
        .expect("receive error")
        .expect("receive timeout")
        .into_frame();
    assert!(received.id().is_extended(), "expected extended frame");
    assert_eq!(received.id(), id);
    assert_eq!(received.data(), frame.data());
}

// ---------------------------------------------------------------------------
// CAN FD loopback - exercises TransmitFd/ReceiveFd directly, including the
// non-contiguous FD DLC table and the BRS flag.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_fd_loopback_brs_and_dlc() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    // Spans single-byte lengths and the FD-only DLC steps (12, 20, 48, 64).
    let lengths = [0usize, 8, 12, 20, 48, 64];
    for (i, &len) in lengths.iter().enumerate() {
        // Alternate the bit-rate-switch flag so both states round-trip.
        let brs = i % 2 == 0;
        let payload: Vec<u8> = (0..len).map(|b| b as u8).collect();
        let frame = CanFdFrame::new(
            CanId::new_standard(0x200 + i as u16).unwrap(),
            &payload,
            brs,
            false,
        )
        .unwrap();
        tx.transmit_fd(&frame).expect("transmit_fd failed");

        let received = rx
            .receive_fd_timeout(Duration::from_secs(1))
            .expect("receive_fd error")
            .expect("receive_fd timeout")
            .into_frame();
        match received {
            Frame::Fd(fd) => {
                assert_eq!(fd.id(), frame.id());
                assert_eq!(fd.data(), frame.data());
                assert_eq!(fd.brs(), brs, "BRS flag mismatch at len {len}");
            }
            other => panic!("expected FD frame at len {len}, got {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Frame-enum dispatch: ReceiveFd must report Frame::Can for classic frames and
// Frame::Fd for FD frames received on the same FD socket.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_receive_fd_discriminates_classic_and_fd() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let classic = CanFrame::new(CanId::new_standard(0x300).unwrap(), &[0x01, 0x02]).unwrap();
    let fd = CanFdFrame::new(
        CanId::new_standard(0x301).unwrap(),
        &[0xAA; 16],
        true,
        false,
    )
    .unwrap();

    // vcan preserves FIFO order, so the classic frame arrives first.
    tx.transmit(&classic).expect("transmit classic");
    tx.transmit_fd(&fd).expect("transmit fd");

    let first = rx
        .receive_fd_timeout(Duration::from_secs(1))
        .expect("receive_fd error")
        .expect("receive_fd timeout")
        .into_frame();
    let second = rx
        .receive_fd_timeout(Duration::from_secs(1))
        .expect("receive_fd error")
        .expect("receive_fd timeout")
        .into_frame();

    match (&first, &second) {
        (Frame::Can(c), Frame::Fd(f)) => {
            assert_eq!(c.id(), classic.id());
            assert_eq!(c.data(), classic.data());
            assert_eq!(f.id(), fd.id());
            assert_eq!(f.data(), fd.data());
        }
        other => panic!("expected (Frame::Can, Frame::Fd) in order, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// The classic Receive path must transparently skip FD frames on an FD socket
// and return the next classic frame.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_classic_receive_skips_fd_frames() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let fd = CanFdFrame::new(
        CanId::new_standard(0x400).unwrap(),
        &[0xFF; 32],
        true,
        false,
    )
    .unwrap();
    let classic = CanFrame::new(CanId::new_standard(0x401).unwrap(), &[0x11, 0x22]).unwrap();

    tx.transmit_fd(&fd).expect("transmit fd");
    tx.transmit(&classic).expect("transmit classic");

    let received = rx
        .receive_timeout(Duration::from_secs(1))
        .expect("receive error")
        .expect("receive timeout")
        .into_frame();
    assert_eq!(
        received.id(),
        classic.id(),
        "classic Receive should skip the FD frame and return the classic one"
    );
    assert_eq!(received.data(), classic.data());
}

// ---------------------------------------------------------------------------
// Filter behavior. SocketCAN supports multiple independent filters with union
// semantics, unlike PCAN/Kvaser which collapse to a single ID+mask pair. This
// is the distinguishing feature worth covering here.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_filter_union_accepts_multiple_ids() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let id_a = CanId::new_standard(0x100).unwrap();
    let id_b = CanId::new_standard(0x200).unwrap();
    let id_blocked = CanId::new_standard(0x300).unwrap();

    rx.set_filters(&[Filter::new(id_a, 0x7FF), Filter::new(id_b, 0x7FF)])
        .expect("set_filters failed");

    tx.transmit(&CanFrame::new(id_a, &[0xAA]).unwrap()).unwrap();
    tx.transmit(&CanFrame::new(id_b, &[0xBB]).unwrap()).unwrap();
    tx.transmit(&CanFrame::new(id_blocked, &[0xCC]).unwrap())
        .unwrap();

    let frames = drain_classic(&mut rx, Duration::from_millis(800));
    let ids: Vec<u32> = frames.iter().map(|f| f.id().raw()).collect();
    assert!(
        ids.contains(&id_a.raw()),
        "filter union should accept {:#x}, got {ids:?}",
        id_a.raw()
    );
    assert!(
        ids.contains(&id_b.raw()),
        "filter union should accept {:#x}, got {ids:?}",
        id_b.raw()
    );
    assert!(
        !ids.contains(&id_blocked.raw()),
        "filter should block {:#x}, got {ids:?}",
        id_blocked.raw()
    );
}

#[test]
fn test_vcan_clear_filters_accepts_all() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    // Narrow to a single ID, then clear; a different ID must now pass.
    rx.set_filters(&[Filter::new(CanId::new_standard(0x100).unwrap(), 0x7FF)])
        .expect("set_filters failed");
    rx.clear_filters().expect("clear_filters failed");

    let id = CanId::new_standard(0x555).unwrap();
    tx.transmit(&CanFrame::new(id, &[0x42]).unwrap()).unwrap();

    let frames = drain_classic(&mut rx, Duration::from_millis(800));
    assert!(
        frames.iter().any(|f| f.id() == id),
        "clear_filters should accept the previously filtered ID, got {frames:?}"
    );
}

#[test]
fn test_vcan_empty_filter_set_accepts_all() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    // An empty slice is documented as equivalent to clear_filters().
    rx.set_filters(&[Filter::new(CanId::new_standard(0x100).unwrap(), 0x7FF)])
        .expect("set_filters failed");
    rx.set_filters(&[]).expect("empty set_filters failed");

    let id = CanId::new_standard(0x556).unwrap();
    tx.transmit(&CanFrame::new(id, &[0x43]).unwrap()).unwrap();

    let frames = drain_classic(&mut rx, Duration::from_millis(800));
    assert!(
        frames.iter().any(|f| f.id() == id),
        "empty filter set should accept the previously filtered ID, got {frames:?}"
    );
}

// ---------------------------------------------------------------------------
// Blocking vs non-blocking mode switching. try_receive flips the socket to
// non-blocking and receive_timeout flips it back; alternating calls must keep
// working and not strand the socket in the wrong mode.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_blocking_nonblocking_toggle() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    // Idle, non-blocking: nothing queued yet.
    assert!(
        rx.try_receive().expect("try_receive error").is_none(),
        "expected no frame on an idle bus"
    );

    // Blocking receive_timeout flips the socket back to blocking and reads it.
    let id = CanId::new_standard(0x321).unwrap();
    tx.transmit(&CanFrame::new(id, &[0x01]).unwrap()).unwrap();
    let received = rx
        .receive_timeout(Duration::from_secs(1))
        .expect("receive error")
        .expect("receive timeout")
        .into_frame();
    assert_eq!(received.id(), id);

    // Back to non-blocking and idle after draining the single frame.
    assert!(
        rx.try_receive().expect("try_receive error").is_none(),
        "expected no frame after draining"
    );
}

#[test]
fn test_vcan_receive_timeout_returns_none_on_idle() {
    let Some(mut rx) = open() else {
        return;
    };
    let start = Instant::now();
    let result = rx
        .receive_timeout(Duration::from_millis(300))
        .expect("receive_timeout error");
    assert!(result.is_none(), "expected timeout on an idle bus");
    assert!(
        start.elapsed() >= Duration::from_millis(250),
        "receive_timeout returned too early: {:?}",
        start.elapsed()
    );
}
