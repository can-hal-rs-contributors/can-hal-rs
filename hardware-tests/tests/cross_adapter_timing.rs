//! Cross-adapter timing tests (PCAN <-> Kvaser).
//!
//! These exercise the parts of each backend's typestate builder that decide
//! bit timing: non-default sample points, the explicit raw-timing escape
//! hatches, alternate bitrates, and the negative case where a deliberate
//! timing mismatch must be observable through the error counters. They require
//! a PCAN and a Kvaser adapter on the same physical bus, like the other
//! `hardware-tests`.
//!
//! Run with `--test-threads=1` to avoid hardware contention.

use std::thread;
use std::time::{Duration, Instant};

use can_hal::frame::{CanFdFrame, CanFrame, Frame};
use can_hal::{BusState, BusStatus, CanId, Receive, ReceiveFd, SamplePoint, Transmit, TransmitFd};
use can_hal_kvaser::{BusParams, BusParamsFd};
use can_hal_pcan::{ClassicBitrate, PcanFdTiming, PcanPhaseTiming};

// ---------------------------------------------------------------------------
// Non-default sample points
// ---------------------------------------------------------------------------

#[test]
fn test_kvaser_custom_sample_point_classic_interop() {
    // Kvaser drives 500K nominal at an 87.5% sample point (vs the 70% default)
    // while PCAN uses its fixed 500K predefined timing. The two must still
    // interoperate: the nominal bit rate matches even though the sample points
    // differ, which is what makes the frame decodable on both ends.
    let frame = CanFrame::new(CanId::new_standard(0x140).unwrap(), &[0x5A, 0xA5]).unwrap();
    let expected = frame.clone();

    let rx = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        ch.receive_timeout(Duration::from_secs(5))
            .expect("PCAN receive error")
            .expect("PCAN receive timeout")
            .into_frame()
    });

    thread::sleep(Duration::from_millis(300));

    let mut kvaser = can_hal_kvaser::KvaserDriver::new()
        .unwrap()
        .channel(0)
        .classic(500_000)
        .unwrap()
        .sample_point(SamplePoint::PCT_87_5)
        .connect()
        .unwrap();
    kvaser.transmit(&frame).expect("Kvaser transmit failed");

    let received = rx.join().expect("Receiver thread panicked");
    assert_eq!(received.id(), expected.id());
    assert_eq!(received.data(), expected.data());
}

#[test]
fn test_custom_sample_point_fd_interop() {
    // Both adapters set non-default sample points (nominal 87.5%, data 75%).
    // The 75% data sample point at 4 Mbit/s is the case that exercises the
    // solver's MIN_TSEG2 floor: without it the solver picks a 4 TQ / tseg2=1
    // data timing that Windows CANlib rejects with canERR_PARAM. With the floor
    // it resolves to a portable timing, so this verifies that fix on hardware.
    let frame = CanFdFrame::new(
        CanId::new_standard(0x141).unwrap(),
        &[0xC3; 16],
        true,
        false,
    )
    .unwrap();
    let expected = frame.clone();

    let rx = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .sample_point(SamplePoint::PCT_87_5)
            .data_sample_point(SamplePoint::PCT_75)
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
        .sample_point(SamplePoint::PCT_87_5)
        .data_sample_point(SamplePoint::PCT_75)
        .connect()
        .unwrap();
    pcan.transmit_fd(&frame).expect("PCAN transmit failed");

    match rx.join().expect("Receiver thread panicked") {
        Frame::Fd(fd) => {
            assert_eq!(fd.id(), expected.id());
            assert_eq!(fd.data(), expected.data());
        }
        other => panic!("expected FD frame, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Explicit raw-timing escape hatches
// ---------------------------------------------------------------------------

#[test]
fn test_pcan_fd_explicit_round_trip() {
    // PCAN's raw-timing escape hatch bypasses the solver. Build PcanFdTiming
    // directly with the workspace 20 TQ / 10 TQ convention (the same segments
    // the solver derives for 500K/4M) and confirm it interoperates with a
    // solver-driven Kvaser FD channel.
    let timing = PcanFdTiming {
        nominal: PcanPhaseTiming {
            brp: 8,
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
        },
        data: PcanPhaseTiming {
            brp: 2,
            tseg1: 7,
            tseg2: 2,
            sjw: 2,
        },
    };

    let to_kvaser = CanFdFrame::new(
        CanId::new_standard(0x150).unwrap(),
        &[0x11; 12],
        true,
        false,
    )
    .unwrap();
    let from_kvaser = CanFdFrame::new(
        CanId::new_standard(0x151).unwrap(),
        &[0x22; 20],
        true,
        false,
    )
    .unwrap();
    let f_tx = to_kvaser.clone();
    let f_echo = from_kvaser.clone();

    let kvaser_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        match ch
            .receive_fd_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout")
            .into_frame()
        {
            Frame::Fd(fd) => {
                assert_eq!(fd.id(), f_tx.id());
                assert_eq!(fd.data(), f_tx.data());
            }
            other => panic!("expected FD frame, got {other:?}"),
        }
        thread::sleep(Duration::from_millis(100));
        ch.transmit_fd(&f_echo).expect("Kvaser echo transmit");
    });

    thread::sleep(Duration::from_millis(300));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .fd_explicit(timing)
        .connect()
        .unwrap();
    pcan.transmit_fd(&to_kvaser).expect("PCAN tx initial");

    let echoed = pcan
        .receive_fd_timeout(Duration::from_secs(5))
        .expect("PCAN receive error")
        .expect("PCAN receive timeout (Kvaser echo missing - fd_explicit timing incompatible?)")
        .into_frame();
    match echoed {
        Frame::Fd(fd) => {
            assert_eq!(fd.id(), from_kvaser.id());
            assert_eq!(fd.data(), from_kvaser.data());
        }
        other => panic!("expected FD frame, got {other:?}"),
    }

    kvaser_handle.join().expect("Kvaser thread panicked");
}

#[test]
fn test_kvaser_fd_explicit_round_trip() {
    // Kvaser's FD raw-timing path: supply explicit nominal and data segments
    // matching the workspace convention and confirm interop with a
    // solver-driven PCAN FD channel.
    let params = BusParams {
        tseg1: 13,
        tseg2: 6,
        sjw: 4,
        no_samp: 1,
        sync_mode: 0,
    };
    let fd_params = BusParamsFd {
        tseg1: 7,
        tseg2: 2,
        sjw: 2,
    };

    let to_kvaser = CanFdFrame::new(
        CanId::new_standard(0x160).unwrap(),
        &[0x33; 24],
        true,
        false,
    )
    .unwrap();
    let from_kvaser =
        CanFdFrame::new(CanId::new_standard(0x161).unwrap(), &[0x44; 8], true, false).unwrap();
    let f_tx = to_kvaser.clone();
    let f_echo = from_kvaser.clone();

    let kvaser_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd_explicit(500_000, 4_000_000, params, fd_params)
            .expect("fd_explicit eager validation")
            .connect()
            .unwrap();
        match ch
            .receive_fd_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout")
            .into_frame()
        {
            Frame::Fd(fd) => {
                assert_eq!(fd.id(), f_tx.id());
                assert_eq!(fd.data(), f_tx.data());
            }
            other => panic!("expected FD frame, got {other:?}"),
        }
        thread::sleep(Duration::from_millis(100));
        ch.transmit_fd(&f_echo).expect("Kvaser echo transmit");
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
    pcan.transmit_fd(&to_kvaser).expect("PCAN tx initial");

    let echoed = pcan
        .receive_fd_timeout(Duration::from_secs(5))
        .expect("PCAN receive error")
        .expect("PCAN receive timeout (Kvaser fd_explicit echo missing)")
        .into_frame();
    match echoed {
        Frame::Fd(fd) => {
            assert_eq!(fd.id(), from_kvaser.id());
            assert_eq!(fd.data(), from_kvaser.data());
        }
        other => panic!("expected FD frame, got {other:?}"),
    }

    kvaser_handle.join().expect("Kvaser thread panicked");
}

// ---------------------------------------------------------------------------
// Alternate bitrate matrix
// ---------------------------------------------------------------------------

/// Round-trip a classic frame PCAN -> Kvaser at the given matched bitrate.
fn classic_round_trip_at(pcan_rate: ClassicBitrate, kvaser_hz: u32, id: u16) {
    let frame = CanFrame::new(CanId::new_standard(id).unwrap(), &[0xA1, 0xB2, 0xC3]).unwrap();
    let expected = frame.clone();

    let rx = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .classic(kvaser_hz)
            .unwrap()
            .connect()
            .unwrap();
        ch.receive_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout")
            .into_frame()
    });

    thread::sleep(Duration::from_millis(300));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .classic(pcan_rate)
        .connect()
        .unwrap();
    pcan.transmit(&frame).expect("PCAN transmit failed");

    let received = rx.join().expect("Receiver thread panicked");
    assert_eq!(received.id(), expected.id());
    assert_eq!(received.data(), expected.data());
}

#[test]
fn test_classic_interop_250k() {
    classic_round_trip_at(ClassicBitrate::Br250K, 250_000, 0x170);
}

#[test]
fn test_classic_interop_500k() {
    classic_round_trip_at(ClassicBitrate::Br500K, 500_000, 0x171);
}

#[test]
fn test_classic_interop_1m() {
    classic_round_trip_at(ClassicBitrate::Br1M, 1_000_000, 0x172);
}

/// Round-trip an FD frame PCAN -> Kvaser at 500K nominal and the given data rate.
fn fd_round_trip_at(data_hz: u32, id: u16) {
    let frame =
        CanFdFrame::new(CanId::new_standard(id).unwrap(), &[0x7E; 16], true, false).unwrap();
    let expected = frame.clone();

    let rx = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .fd(500_000, data_hz)
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
        .fd(500_000, data_hz)
        .unwrap()
        .connect()
        .unwrap();
    pcan.transmit_fd(&frame).expect("PCAN transmit failed");

    match rx.join().expect("Receiver thread panicked") {
        Frame::Fd(fd) => {
            assert_eq!(fd.id(), expected.id());
            assert_eq!(fd.data(), expected.data());
        }
        other => panic!("expected FD frame, got {other:?}"),
    }
}

#[test]
fn test_fd_interop_data_2m() {
    fd_round_trip_at(2_000_000, 0x180);
}

#[test]
fn test_fd_interop_data_4m() {
    fd_round_trip_at(4_000_000, 0x181);
}

// ---------------------------------------------------------------------------
// Negative timing: a deliberate bitrate mismatch must surface through the
// transmitter's error state.
// ---------------------------------------------------------------------------

#[test]
fn test_mismatched_timing_raises_errors() {
    // Deliberately mis-configure the two adapters: PCAN at 250K, Kvaser at
    // 500K. With incompatible nominal bit rates the PCAN transmitter cannot
    // get its frames acknowledged cleanly, so its transmit error counter must
    // climb (or the controller must leave ErrorActive) within a short window.
    // If the timing setters were silently ignored and both ran at the same
    // rate, no errors would accumulate and this test would fail - which is the
    // regression it guards against.
    //
    // Each channel re-initializes its controller on open (clearing any prior
    // bus-off) and both are dropped at the end, so this does not contaminate
    // later tests.
    let victim = thread::spawn(move || {
        // A correctly-formed listener at 500K so the bus has another node.
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut ch = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        // Listen for the duration of the test; content is irrelevant.
        let _ = ch.receive_timeout(Duration::from_secs(3));
    });

    thread::sleep(Duration::from_millis(200));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .classic(ClassicBitrate::Br250K)
        .connect()
        .unwrap();
    let frame = CanFrame::new(CanId::new_standard(0x190).unwrap(), &[0xFF; 8]).unwrap();

    let deadline = Instant::now() + Duration::from_secs(2);
    let mut detected = false;
    while Instant::now() < deadline {
        // transmit() may start failing once the controller degrades; ignore it
        // and keep polling the bus status.
        let _ = pcan.transmit(&frame);
        let state = pcan.bus_state().expect("bus_state");
        let counters = pcan.error_counters().expect("error_counters");
        if state != BusState::ErrorActive || counters.transmit > 0 {
            detected = true;
            break;
        }
        thread::sleep(Duration::from_millis(50));
    }
    assert!(
        detected,
        "expected the timing mismatch to raise TX errors or leave ErrorActive"
    );

    victim.join().expect("Kvaser victim thread panicked");
    // Let the bus settle before the next test opens channels.
    thread::sleep(Duration::from_millis(200));
}
