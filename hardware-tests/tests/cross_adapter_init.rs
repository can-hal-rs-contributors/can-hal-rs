//! Cross-adapter initialization and mode tests (PCAN <-> Kvaser).
//!
//! Fills gaps the existing suites leave: direct CAN FD frame round-trips (the
//! ISO-TP tests only reach FD frames indirectly), bus-status queries in both
//! modes and on both backends, FD-mode filtering and extended IDs, channel
//! reopen after Drop, and the eager init-error validation paths that should
//! fail at the builder call site rather than at connect(). Requires a PCAN and
//! a Kvaser adapter on the same physical bus.
//!
//! Run with `--test-threads=1` to avoid hardware contention.

use std::thread;
use std::time::{Duration, Instant};

use can_hal::filter::{Filter, Filterable};
use can_hal::frame::{CanFdFrame, CanFrame, Frame};
use can_hal::{BusState, BusStatus, CanId, Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal_kvaser::BusParams;
use can_hal_pcan::ClassicBitrate;

/// Drain FD frames for at most `window`, collecting everything received.
fn drain_fd<C, E>(channel: &mut C, window: Duration) -> Vec<Frame>
where
    C: ReceiveFd<Error = E, Timestamp = Instant>,
    E: can_hal::CanError,
{
    let deadline = Instant::now() + window;
    let mut frames = Vec::new();
    while Instant::now() < deadline {
        match channel.receive_fd_timeout(Duration::from_millis(100)) {
            Ok(Some(ts)) => frames.push(ts.into_frame()),
            Ok(None) => {}
            Err(e) => panic!("receive_fd error: {e}"),
        }
    }
    frames
}

// ---------------------------------------------------------------------------
// Direct CAN FD frame round-trip: BRS on and off, across the FD DLC table.
// Exercises the convert layers' FD paths that the ISO-TP tests bypass.
// ---------------------------------------------------------------------------

#[test]
fn test_direct_fd_frame_round_trip_brs_and_dlc() {
    let lengths = [0usize, 1, 8, 12, 20, 48, 64];
    let count = lengths.len();

    let rx = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut out = Vec::new();
        for _ in 0..count {
            let frame = ch
                .receive_fd_timeout(Duration::from_secs(5))
                .expect("Kvaser receive error")
                .expect("Kvaser receive timeout")
                .into_frame();
            out.push(frame);
        }
        out
    });

    thread::sleep(Duration::from_millis(300));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .fd(500_000, 4_000_000)
        .unwrap()
        .connect()
        .unwrap();

    let mut sent = Vec::new();
    for (i, &len) in lengths.iter().enumerate() {
        // Alternate the bit-rate-switch flag so both states round-trip.
        let brs = i % 2 == 0;
        let payload: Vec<u8> = (0..len).map(|b| b as u8).collect();
        let frame = CanFdFrame::new(
            CanId::new_standard(0x1A0 + i as u16).unwrap(),
            &payload,
            brs,
            false,
        )
        .unwrap();
        pcan.transmit_fd(&frame).expect("PCAN transmit_fd failed");
        sent.push(frame);
        thread::sleep(Duration::from_millis(20));
    }

    let received = rx.join().expect("Receiver thread panicked");
    assert_eq!(received.len(), sent.len(), "received frame count mismatch");
    for (exp, got) in sent.iter().zip(received.iter()) {
        match got {
            Frame::Fd(fd) => {
                assert_eq!(fd.id(), exp.id());
                assert_eq!(fd.data(), exp.data());
                assert_eq!(
                    fd.brs(),
                    exp.brs(),
                    "BRS flag mismatch for id {:#x}",
                    exp.id().raw()
                );
            }
            other => panic!("expected FD frame, got {other:?}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Bus status: fills the matrix left open by test_pcan_bus_status (PCAN classic
// only) - Kvaser classic, plus FD on both backends. All read a quiet bus.
// ---------------------------------------------------------------------------

#[test]
fn test_kvaser_bus_status_classic() {
    let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
    let ch = driver
        .channel(0)
        .classic(500_000)
        .unwrap()
        .connect()
        .unwrap();
    assert_eq!(ch.bus_state().expect("bus_state"), BusState::ErrorActive);
    let counters = ch.error_counters().expect("error_counters");
    assert_eq!(counters.transmit, 0);
    assert_eq!(counters.receive, 0);
}

#[test]
fn test_pcan_bus_status_fd() {
    let driver = can_hal_pcan::PcanDriver::new().unwrap();
    let ch = driver
        .channel(0)
        .unwrap()
        .fd(500_000, 4_000_000)
        .unwrap()
        .connect()
        .unwrap();
    assert_eq!(ch.bus_state().expect("bus_state"), BusState::ErrorActive);
    let counters = ch.error_counters().expect("error_counters");
    assert_eq!(counters.transmit, 0);
    assert_eq!(counters.receive, 0);
}

#[test]
fn test_kvaser_bus_status_fd() {
    let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
    let ch = driver
        .channel(0)
        .fd(500_000, 4_000_000)
        .unwrap()
        .connect()
        .unwrap();
    assert_eq!(ch.bus_state().expect("bus_state"), BusState::ErrorActive);
    let counters = ch.error_counters().expect("error_counters");
    assert_eq!(counters.transmit, 0);
    assert_eq!(counters.receive, 0);
}

// ---------------------------------------------------------------------------
// FD-mode hardware filter (the existing filter tests are classic only).
// ---------------------------------------------------------------------------

#[test]
fn test_pcan_fd_hardware_filter_blocks_non_matching() {
    let match_id = CanId::new_standard(0x200).unwrap();
    let blocked_a = CanId::new_standard(0x100).unwrap();
    let blocked_b = CanId::new_standard(0x300).unwrap();

    let rx = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .unwrap()
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        ch.set_filters(&[Filter::new(match_id, 0x7FF)])
            .expect("set_filters failed");
        drain_fd(&mut ch, Duration::from_millis(1500))
    });

    thread::sleep(Duration::from_millis(300));

    let mut kvaser = can_hal_kvaser::KvaserDriver::new()
        .unwrap()
        .channel(0)
        .fd(500_000, 4_000_000)
        .unwrap()
        .connect()
        .unwrap();
    kvaser
        .transmit_fd(&CanFdFrame::new(blocked_a, &[0xAA; 8], true, false).unwrap())
        .expect("tx blocked_a");
    thread::sleep(Duration::from_millis(50));
    kvaser
        .transmit_fd(&CanFdFrame::new(match_id, &[0xBB; 8], true, false).unwrap())
        .expect("tx match");
    thread::sleep(Duration::from_millis(50));
    kvaser
        .transmit_fd(&CanFdFrame::new(blocked_b, &[0xCC; 8], true, false).unwrap())
        .expect("tx blocked_b");

    let received = rx.join().expect("Receiver thread panicked");
    let unmatched: Vec<_> = received.iter().filter(|f| f.id() != match_id).collect();
    let matched: Vec<_> = received.iter().filter(|f| f.id() == match_id).collect();
    assert!(
        unmatched.is_empty(),
        "expected no non-matching frames, got {unmatched:?}"
    );
    assert_eq!(
        matched.len(),
        1,
        "expected one matching frame, got {received:?}"
    );
}

// ---------------------------------------------------------------------------
// Extended (29-bit) IDs in FD mode (the existing extended-ID tests are classic).
// ---------------------------------------------------------------------------

#[test]
fn test_fd_extended_id_round_trip() {
    let id = CanId::new_extended(0x1A2B_3C4D).unwrap();
    let frame = CanFdFrame::new(id, &[0x9A; 20], true, false).unwrap();
    let expected = frame.clone();

    let rx = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        ch.receive_fd_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout")
            .into_frame()
    });

    thread::sleep(Duration::from_millis(300));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .fd(500_000, 4_000_000)
        .unwrap()
        .connect()
        .unwrap();
    pcan.transmit_fd(&frame).expect("PCAN transmit failed");

    match rx.join().expect("Receiver thread panicked") {
        Frame::Fd(fd) => {
            assert!(fd.id().is_extended(), "expected extended FD frame");
            assert_eq!(fd.id(), expected.id());
            assert_eq!(fd.data(), expected.data());
        }
        other => panic!("expected FD frame, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Drop-and-reopen: Kvaser classic (mirrors the existing PCAN test) and PCAN FD.
// Confirms Drop releases the hardware handle so the same index reopens.
// ---------------------------------------------------------------------------

#[test]
fn test_kvaser_drop_and_reopen() {
    let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
    let frame_one = CanFrame::new(CanId::new_standard(0x1B1).unwrap(), &[0xAA]).unwrap();
    let frame_two = CanFrame::new(CanId::new_standard(0x1B2).unwrap(), &[0xBB]).unwrap();

    let f1 = frame_one.clone();
    let f2 = frame_two.clone();
    let rx = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        let r1 = ch
            .receive_timeout(Duration::from_secs(5))
            .expect("receive 1 error")
            .expect("receive 1 timeout")
            .into_frame();
        assert_eq!(r1.id(), f1.id());
        let r2 = ch
            .receive_timeout(Duration::from_secs(5))
            .expect("receive 2 error")
            .expect("receive 2 timeout (Kvaser reopen lost the handle)")
            .into_frame();
        assert_eq!(r2.id(), f2.id());
    });

    thread::sleep(Duration::from_millis(300));

    // Open, send, drop.
    {
        let mut ch = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        ch.transmit(&frame_one).expect("Kvaser tx first");
        // ch drops here, releasing the hardware handle.
    }

    thread::sleep(Duration::from_millis(100));

    // Reopen the same index. If Drop didn't actually canClose, this fails.
    let mut ch = driver
        .channel(0)
        .classic(500_000)
        .unwrap()
        .connect()
        .expect("Kvaser reopen failed - Drop did not release the handle");
    ch.transmit(&frame_two).expect("Kvaser tx second");

    rx.join().expect("PCAN receiver thread panicked");
}

#[test]
fn test_pcan_fd_drop_and_reopen() {
    let driver = can_hal_pcan::PcanDriver::new().unwrap();
    let frame_one = CanFdFrame::new(
        CanId::new_standard(0x1C1).unwrap(),
        &[0xAA; 12],
        true,
        false,
    )
    .unwrap();
    let frame_two = CanFdFrame::new(
        CanId::new_standard(0x1C2).unwrap(),
        &[0xBB; 12],
        true,
        false,
    )
    .unwrap();

    let f1 = frame_one.clone();
    let f2 = frame_two.clone();
    let rx = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        let r1 = ch
            .receive_fd_timeout(Duration::from_secs(5))
            .expect("receive 1 error")
            .expect("receive 1 timeout")
            .into_frame();
        assert_eq!(r1.id(), f1.id());
        let r2 = ch
            .receive_fd_timeout(Duration::from_secs(5))
            .expect("receive 2 error")
            .expect("receive 2 timeout (PCAN FD reopen lost the handle)")
            .into_frame();
        assert_eq!(r2.id(), f2.id());
    });

    thread::sleep(Duration::from_millis(300));

    // Open FD, send, drop.
    {
        let mut ch = driver
            .channel(0)
            .unwrap()
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        ch.transmit_fd(&frame_one).expect("PCAN FD tx first");
        // ch drops here, releasing the hardware handle.
    }

    thread::sleep(Duration::from_millis(100));

    let mut ch = driver
        .channel(0)
        .unwrap()
        .fd(500_000, 4_000_000)
        .unwrap()
        .connect()
        .expect("PCAN FD reopen failed - Drop did not release the handle");
    ch.transmit_fd(&frame_two).expect("PCAN FD tx second");

    rx.join().expect("Kvaser receiver thread panicked");
}

// ---------------------------------------------------------------------------
// Init error paths: invalid configurations must be rejected eagerly at the
// builder call site, not deferred to connect(). These need only the vendor
// library loaded, not a second adapter or bus traffic.
// ---------------------------------------------------------------------------

#[test]
fn test_pcan_invalid_channel_index_rejected() {
    let driver = can_hal_pcan::PcanDriver::new().unwrap();
    // PCAN channel indices are 0..=15; anything larger must be rejected before
    // the u16 cast rather than silently truncating to channel 0.
    assert!(
        driver.channel(16).is_err(),
        "PCAN channel index 16 must be rejected"
    );
    assert!(
        driver.channel(99).is_err(),
        "PCAN channel index 99 must be rejected"
    );
}

#[test]
fn test_pcan_fd_non_divisible_bitrate_rejected() {
    let driver = can_hal_pcan::PcanDriver::new().unwrap();
    // 333_000 does not evenly divide the 80 MHz PCAN clock.
    let builder = driver.channel(0).unwrap();
    assert!(
        builder.fd(333_000, 4_000_000).is_err(),
        "non-divisible nominal bitrate must be rejected at the call site"
    );
}

#[test]
fn test_kvaser_non_divisible_bitrate_rejected() {
    let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
    assert!(
        driver.channel(0).classic(333_000).is_err(),
        "non-divisible classic bitrate must be rejected"
    );
    assert!(
        driver.channel(0).fd(500_000, 333_000).is_err(),
        "non-divisible FD data bitrate must be rejected"
    );
}

#[test]
fn test_kvaser_classic_explicit_out_of_range_rejected() {
    let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
    // Nominal tseg2 max is 128; 200 is out of range and must be caught eagerly
    // rather than at connect().
    let bad = BusParams {
        tseg1: 13,
        tseg2: 200,
        sjw: 4,
        no_samp: 1,
        sync_mode: 0,
    };
    assert!(
        driver.channel(0).classic_explicit(500_000, bad).is_err(),
        "out-of-range explicit segments must be rejected at the call site"
    );
}
