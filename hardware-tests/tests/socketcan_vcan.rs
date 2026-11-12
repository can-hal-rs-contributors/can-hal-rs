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
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};
use can_hal::CanId;
use can_hal_socketcan::{SocketCanChannel, SocketCanDriver, SocketCanError};

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
// Remote frames - a classic remote frame arrives as one, asking for the same
// length, through both the classic and the FD receive paths.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_remote_frame_loopback() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let remote = CanFrame::new_remote(CanId::new_extended(0x18EA_FFFE).unwrap(), 3).unwrap();
    tx.transmit(&remote).expect("transmit remote");

    let received = rx
        .receive_timeout(Duration::from_secs(1))
        .expect("receive error")
        .expect("receive timeout")
        .into_frame();
    assert!(
        received.is_remote_frame(),
        "expected a remote frame, got {received:?}"
    );
    assert_eq!(received.dlc(), 3);
    assert_eq!(received, remote);
}

#[test]
fn test_vcan_large_dlc_loopback() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let id = CanId::new_standard(0x303).unwrap();
    let frame = CanFrame::new_with_dlc(id, &[1, 2, 3, 4, 5, 6, 7, 8], 13).unwrap();
    tx.transmit(&frame).expect("transmit DLC 13");
    let remote = CanFrame::new_remote(id, 10).unwrap();
    tx.transmit(&remote).expect("transmit remote DLC 10");

    for expected in [frame, remote] {
        let received = rx
            .receive_timeout(Duration::from_secs(1))
            .expect("receive error")
            .expect("receive timeout")
            .into_frame();
        assert_eq!(received, expected);
    }
}

#[test]
fn test_vcan_receive_fd_delivers_remote_frames() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let remote = CanFrame::new_remote(CanId::new_standard(0x302).unwrap(), 8).unwrap();
    tx.transmit(&remote).expect("transmit remote");

    let received = rx
        .receive_fd_timeout(Duration::from_secs(1))
        .expect("receive_fd error")
        .expect("receive_fd timeout")
        .into_frame();
    assert_eq!(received, Frame::Can(remote));
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
// Filters and frame types: a filter matches frames of its own ID type only,
// so a type with no filter in the set is rejected. Remote frames match on
// their ID, like data frames.
// ---------------------------------------------------------------------------

#[test]
fn test_vcan_standard_filter_rejects_extended_frames() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let std_id = CanId::new_standard(0x100).unwrap();
    // Low 11 bits equal to 0x100, and the extended ID 0x100 itself.
    let ext_alias = CanId::new_extended(0x1234_5100).unwrap();
    let ext_same = CanId::new_extended(0x100).unwrap();

    rx.set_filters(&[Filter::new(std_id, 0x7FF)])
        .expect("set_filters failed");
    tx.transmit(&CanFrame::new(ext_alias, &[0x01]).unwrap())
        .unwrap();
    tx.transmit(&CanFrame::new(ext_same, &[0x02]).unwrap())
        .unwrap();
    tx.transmit(&CanFrame::new(std_id, &[0x03]).unwrap())
        .unwrap();

    let ids: Vec<CanId> = drain_classic(&mut rx, Duration::from_millis(800))
        .iter()
        .map(CanFrame::id)
        .collect();
    assert_eq!(ids, vec![std_id], "a standard filter passed {ids:?}");
}

#[test]
fn test_vcan_standard_wildcard_filter_rejects_extended_frames() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let std_id = CanId::new_standard(0x555).unwrap();
    let ext_id = CanId::new_extended(0x18DA_00F1).unwrap();

    // Mask 0: any standard ID.
    rx.set_filters(&[Filter::new(CanId::new_standard(0).unwrap(), 0)])
        .expect("set_filters failed");
    tx.transmit(&CanFrame::new(ext_id, &[0x01]).unwrap())
        .unwrap();
    tx.transmit(&CanFrame::new(std_id, &[0x02]).unwrap())
        .unwrap();

    let ids: Vec<CanId> = drain_classic(&mut rx, Duration::from_millis(800))
        .iter()
        .map(CanFrame::id)
        .collect();
    assert_eq!(
        ids,
        vec![std_id],
        "a standard wildcard filter passed {ids:?}"
    );
}

#[test]
fn test_vcan_extended_filter_rejects_standard_frames() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let ext_id = CanId::new_extended(0x100).unwrap();
    let std_same = CanId::new_standard(0x100).unwrap();

    rx.set_filters(&[Filter::new(ext_id, 0x1FFF_FFFF)])
        .expect("set_filters failed");
    tx.transmit(&CanFrame::new(std_same, &[0x01]).unwrap())
        .unwrap();
    tx.transmit(&CanFrame::new(ext_id, &[0x02]).unwrap())
        .unwrap();

    let ids: Vec<CanId> = drain_classic(&mut rx, Duration::from_millis(800))
        .iter()
        .map(CanFrame::id)
        .collect();
    assert_eq!(ids, vec![ext_id], "an extended filter passed {ids:?}");
}

#[test]
fn test_vcan_standard_filter_passes_matching_remote_frame() {
    let Some((mut tx, mut rx)) = open_pair() else {
        return;
    };
    let id = CanId::new_standard(0x100).unwrap();
    rx.set_filters(&[Filter::new(id, 0x7FF)])
        .expect("set_filters failed");
    let remote = CanFrame::new_remote(id, 4).unwrap();
    tx.transmit(&remote).unwrap();

    let frames = drain_classic(&mut rx, Duration::from_millis(800));
    assert_eq!(
        frames,
        vec![remote],
        "the filter must not drop remote frames"
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

// ---------------------------------------------------------------------------
// FD frames can-hal cannot represent. The kernel takes a CAN FD frame of any
// length up to 64 from a raw socket, and loops it back to the host's other
// sockets as written, so a 9-byte FD frame (which `CanFdFrame::new` refuses)
// reaches a `ReceiveFd` reader. It must be reported, as the classic receive
// path reports a frame it can't convert, not dropped.
// ---------------------------------------------------------------------------

/// Write an FD frame of 9 bytes with ID `id`, then a valid sentinel FD frame
/// with ID `id + 1`, from a raw socket on `IFACE`; returns the sentinel.
fn write_invalid_fd_then_sentinel(id: u16) -> CanFdFrame {
    use socketcan::{EmbeddedFrame as _, Socket as _};

    let raw_tx = socketcan::CanFdSocket::open(IFACE).expect("open raw FD socket");
    let mut raw = socketcan::frame::canfd_frame_default();
    raw.can_id = u32::from(id);
    raw.len = 9;
    raw.data[..9].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    raw_tx
        .write_frame(&socketcan::CanFdFrame::from(raw))
        .expect("write a 9-byte FD frame");

    let data = [0xAA; 12];
    let sc_sentinel =
        socketcan::CanFdFrame::new(socketcan::StandardId::new(id + 1).unwrap(), &data).unwrap();
    raw_tx
        .write_frame(&sc_sentinel)
        .expect("write the sentinel");
    CanFdFrame::new(CanId::new_standard(id + 1).unwrap(), &data, false, false).unwrap()
}

/// Assert a receive call reported the 9-byte frame as `InvalidFrame`.
fn assert_invalid_fd<T: std::fmt::Debug>(result: &Result<T, SocketCanError>) {
    assert!(
        matches!(result, Err(SocketCanError::InvalidFrame(_))),
        "a 9-byte FD frame must be reported as InvalidFrame, got {result:?}"
    );
}

/// Assert the channel still delivers the frame after the reported one.
fn assert_sentinel_next(rx: &mut SocketCanChannel, sentinel: CanFdFrame) {
    let next = rx
        .receive_fd_timeout(Duration::from_secs(1))
        .expect("receive error after the reported frame")
        .map(Timestamped::into_frame);
    assert_eq!(next, Some(Frame::Fd(sentinel)));
}

#[test]
fn test_vcan_receive_fd_reports_invalid_fd_length() {
    let Some(mut rx) = open() else {
        return;
    };
    let sentinel = write_invalid_fd_then_sentinel(0x3A0);
    assert_invalid_fd(&rx.receive_fd());
    assert_sentinel_next(&mut rx, sentinel);
}

#[test]
fn test_vcan_receive_fd_timeout_reports_invalid_fd_length() {
    let Some(mut rx) = open() else {
        return;
    };
    let sentinel = write_invalid_fd_then_sentinel(0x3B0);
    assert_invalid_fd(&rx.receive_fd_timeout(Duration::from_secs(1)));
    assert_sentinel_next(&mut rx, sentinel);
}

#[test]
fn test_vcan_try_receive_fd_reports_invalid_fd_length() {
    let Some(mut rx) = open() else {
        return;
    };
    let sentinel = write_invalid_fd_then_sentinel(0x3C0);
    // Both frames are queued once the writes return (local loopback is
    // synchronous), so the first poll reads the 9-byte one.
    assert_invalid_fd(&rx.try_receive_fd());
    assert_sentinel_next(&mut rx, sentinel);
}
