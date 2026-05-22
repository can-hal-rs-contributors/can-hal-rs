use std::thread;
use std::time::{Duration, Instant};

use can_hal::channel::Receive;
use can_hal::filter::{Filter, Filterable};
use can_hal::frame::CanFrame;
use can_hal::{BusState, BusStatus, CanId, Transmit};
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_kvaser::BusParams;
use can_hal_pcan::ClassicBitrate;

const TX_ID: u16 = 0x7E0;
const RX_ID: u16 = 0x7E8;

// ---------------------------------------------------------------------------
// Raw CAN frame tests
// ---------------------------------------------------------------------------

#[test]
fn test_pcan_bus_status() {
    let driver = can_hal_pcan::PcanDriver::new().expect("PCAN-Basic library not found");
    let channel = driver
        .channel(0)
        .unwrap()
        .classic(ClassicBitrate::Br500K)
        .connect()
        .expect("Failed to open PCAN channel");

    let state = channel.bus_state().expect("Failed to read bus state");
    assert_eq!(
        state,
        BusState::ErrorActive,
        "Expected ErrorActive bus state"
    );

    let counters = channel
        .error_counters()
        .expect("Failed to read error counters");
    assert_eq!(counters.transmit, 0, "Expected 0 TX errors");
    assert_eq!(counters.receive, 0, "Expected 0 RX errors");
}

#[test]
fn test_pcan_to_kvaser_raw_frame() {
    let frame = CanFrame::new(
        CanId::new_standard(0x100).unwrap(),
        &[0xDE, 0xAD, 0xBE, 0xEF],
    )
    .expect("Failed to create frame");

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        let received = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout");
        received.into_frame()
    });

    thread::sleep(Duration::from_millis(200));

    let mut pcan_channel = {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap()
    };
    pcan_channel.transmit(&frame).expect("PCAN transmit failed");

    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received.id(), frame.id(), "CAN ID mismatch");
    assert_eq!(received.data(), frame.data(), "Frame data mismatch");
}

#[test]
fn test_kvaser_to_pcan_raw_frame() {
    let frame = CanFrame::new(
        CanId::new_standard(0x200).unwrap(),
        &[0xCA, 0xFE, 0xBA, 0xBE],
    )
    .expect("Failed to create frame");

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        let received = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("PCAN receive error")
            .expect("PCAN receive timeout");
        received.into_frame()
    });

    thread::sleep(Duration::from_millis(200));

    let mut kvaser_channel = {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap()
    };
    kvaser_channel
        .transmit(&frame)
        .expect("Kvaser transmit failed");

    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received.id(), frame.id(), "CAN ID mismatch");
    assert_eq!(received.data(), frame.data(), "Frame data mismatch");
}

// ---------------------------------------------------------------------------
// ISO-TP tests
// ---------------------------------------------------------------------------

fn isotp_transfer_pcan_to_kvaser(payload: &[u8]) {
    let tx_id = CanId::new_standard(TX_ID).unwrap();
    let rx_id = CanId::new_standard(RX_ID).unwrap();
    let payload_send = payload.to_vec();
    let payload_expected = payload.to_vec();

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(rx_id, tx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpChannel::new(channel, config);
        isotp.receive().expect("Kvaser ISO-TP receive failed")
    });

    thread::sleep(Duration::from_millis(200));

    let tx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(tx_id, rx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpChannel::new(channel, config);
        isotp.send(&payload_send).expect("PCAN ISO-TP send failed");
    });

    tx_handle.join().expect("Sender thread panicked");
    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received, payload_expected, "Payload mismatch");
}

fn isotp_transfer_kvaser_to_pcan(payload: &[u8]) {
    let tx_id = CanId::new_standard(TX_ID).unwrap();
    let rx_id = CanId::new_standard(RX_ID).unwrap();
    let payload_send = payload.to_vec();
    let payload_expected = payload.to_vec();

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(rx_id, tx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpChannel::new(channel, config);
        isotp.receive().expect("PCAN ISO-TP receive failed")
    });

    thread::sleep(Duration::from_millis(200));

    let tx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(tx_id, rx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpChannel::new(channel, config);
        isotp
            .send(&payload_send)
            .expect("Kvaser ISO-TP send failed");
    });

    tx_handle.join().expect("Sender thread panicked");
    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received, payload_expected, "Payload mismatch");
}

// --- Standalone channel open (no threading) ---

#[test]
fn test_kvaser_open_no_thread() {
    let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
    let _channel = driver
        .channel(0)
        .classic(500_000)
        .unwrap()
        .connect()
        .expect("Kvaser channel open failed");
}

// --- PCAN -> Kvaser ---

#[test]
fn test_isotp_single_frame_2_bytes() {
    isotp_transfer_pcan_to_kvaser(&[0x10, 0x01]);
}

#[test]
fn test_isotp_single_frame_7_bytes() {
    isotp_transfer_pcan_to_kvaser(&(0..7).collect::<Vec<u8>>());
}

#[test]
fn test_isotp_multi_frame_20_bytes() {
    isotp_transfer_pcan_to_kvaser(&(0..20).map(|i| i as u8).collect::<Vec<u8>>());
}

#[test]
fn test_isotp_multi_frame_200_bytes() {
    isotp_transfer_pcan_to_kvaser(&(0..200).map(|i| i as u8).collect::<Vec<u8>>());
}

#[test]
fn test_isotp_multi_frame_1000_bytes() {
    isotp_transfer_pcan_to_kvaser(&(0..1000).map(|i| i as u8).collect::<Vec<u8>>());
}

// --- Kvaser -> PCAN ---

#[test]
fn test_isotp_reverse_100_bytes() {
    isotp_transfer_kvaser_to_pcan(&(0..100).map(|i| (i * 3) as u8).collect::<Vec<u8>>());
}

// ---------------------------------------------------------------------------
// Extended (29-bit) CAN IDs - exercises the is_extended() branches in both
// backends' convert layers and the EXT message-type flag handling. The
// existing raw-frame and ISO-TP tests all use standard IDs.
// ---------------------------------------------------------------------------

#[test]
fn test_pcan_to_kvaser_extended_id_frame() {
    let id = CanId::new_extended(0x1ABC_DEF0).expect("valid extended ID");
    let frame = CanFrame::new(id, &[0xC0, 0xFF, 0xEE, 0x42]).expect("Failed to create frame");

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        let received = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout");
        received.into_frame()
    });

    thread::sleep(Duration::from_millis(200));

    let mut pcan_channel = {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap()
    };
    pcan_channel.transmit(&frame).expect("PCAN transmit failed");

    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received.id(), id, "extended CAN ID mismatch");
    assert!(received.id().is_extended(), "expected extended frame");
    assert_eq!(received.data(), frame.data(), "frame data mismatch");
}

#[test]
fn test_kvaser_to_pcan_extended_id_frame() {
    let id = CanId::new_extended(0x18DA_00F1).expect("valid extended ID");
    let frame = CanFrame::new(id, &[0xDE, 0xAD, 0xBE, 0xEF]).expect("Failed to create frame");

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        let received = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("PCAN receive error")
            .expect("PCAN receive timeout");
        received.into_frame()
    });

    thread::sleep(Duration::from_millis(200));

    let mut kvaser_channel = {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap()
    };
    kvaser_channel
        .transmit(&frame)
        .expect("Kvaser transmit failed");

    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received.id(), id, "extended CAN ID mismatch");
    assert!(received.id().is_extended(), "expected extended frame");
    assert_eq!(received.data(), frame.data(), "frame data mismatch");
}

// ---------------------------------------------------------------------------
// Hardware filter behavior. Verifies that set_filters() actually applies a
// filter at the controller level (not just in software). Each backend has
// its own filter representation - PCAN uses range-based filter_messages,
// Kvaser uses the canAccept code/mask pair - so both deserve direct tests.
// ---------------------------------------------------------------------------

/// Drain `receive_timeout` for at most `window`, collecting every frame.
fn drain_for<C, E>(channel: &mut C, window: Duration) -> Vec<CanFrame>
where
    C: Receive<Error = E, Timestamp = Instant>,
    E: can_hal::CanError,
{
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

#[test]
fn test_kvaser_hardware_filter_blocks_non_matching() {
    let match_id = CanId::new_standard(0x200).expect("valid standard ID");
    let blocked_a = CanId::new_standard(0x100).expect("valid standard ID");
    let blocked_b = CanId::new_standard(0x300).expect("valid standard ID");

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        channel
            .set_filters(&[Filter::new(match_id, 0x7FF)])
            .expect("set_filters failed");
        drain_for(&mut channel, Duration::from_millis(1500))
    });

    thread::sleep(Duration::from_millis(300));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .classic(ClassicBitrate::Br500K)
        .connect()
        .unwrap();
    pcan.transmit(&CanFrame::new(blocked_a, &[0xAA]).unwrap())
        .expect("tx blocked_a");
    thread::sleep(Duration::from_millis(50));
    pcan.transmit(&CanFrame::new(match_id, &[0xBB]).unwrap())
        .expect("tx match");
    thread::sleep(Duration::from_millis(50));
    pcan.transmit(&CanFrame::new(blocked_b, &[0xCC]).unwrap())
        .expect("tx blocked_b");

    let received = rx_handle.join().expect("Receiver thread panicked");
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
    assert_eq!(matched[0].data(), &[0xBB]);
}

#[test]
fn test_pcan_hardware_filter_blocks_non_matching() {
    let match_id = CanId::new_standard(0x200).expect("valid standard ID");
    let blocked_a = CanId::new_standard(0x100).expect("valid standard ID");
    let blocked_b = CanId::new_standard(0x300).expect("valid standard ID");

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        channel
            .set_filters(&[Filter::new(match_id, 0x7FF)])
            .expect("set_filters failed");
        drain_for(&mut channel, Duration::from_millis(1500))
    });

    thread::sleep(Duration::from_millis(300));

    let mut kvaser = can_hal_kvaser::KvaserDriver::new()
        .unwrap()
        .channel(0)
        .classic(500_000)
        .unwrap()
        .connect()
        .unwrap();
    kvaser
        .transmit(&CanFrame::new(blocked_a, &[0xAA]).unwrap())
        .expect("tx blocked_a");
    thread::sleep(Duration::from_millis(50));
    kvaser
        .transmit(&CanFrame::new(match_id, &[0xBB]).unwrap())
        .expect("tx match");
    thread::sleep(Duration::from_millis(50));
    kvaser
        .transmit(&CanFrame::new(blocked_b, &[0xCC]).unwrap())
        .expect("tx blocked_b");

    let received = rx_handle.join().expect("Receiver thread panicked");
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
    assert_eq!(matched[0].data(), &[0xBB]);
}

// ---------------------------------------------------------------------------
// classic_explicit smoke - Kvaser's classic_explicit path skips the solver
// and goes directly to canSetBusParams with the user-supplied segments. We
// reproduce the solver's 500K nominal output (tseg1=13, tseg2=6, sjw=4) and
// verify a PCAN <-> Kvaser frame still rounds-trips, confirming the explicit
// code path produces a controller configuration compatible with PCAN.
// ---------------------------------------------------------------------------

#[test]
fn test_kvaser_classic_explicit_round_trip() {
    let frame_to_kvaser = CanFrame::new(
        CanId::new_standard(0x101).unwrap(),
        &[0xAA, 0xBB, 0xCC, 0xDD],
    )
    .unwrap();
    let frame_from_kvaser = CanFrame::new(
        CanId::new_standard(0x102).unwrap(),
        &[0x11, 0x22, 0x33, 0x44],
    )
    .unwrap();

    let nominal_params = BusParams {
        tseg1: 13,
        tseg2: 6,
        sjw: 4,
        no_samp: 1,
        sync_mode: 0,
    };

    let f_tx = frame_to_kvaser.clone();
    let f_rx_back = frame_from_kvaser.clone();
    let kvaser_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .classic_explicit(500_000, nominal_params)
            .expect("classic_explicit eager validation")
            .connect()
            .expect("Kvaser classic_explicit connect");

        // Receive PCAN -> Kvaser
        let received = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("Kvaser receive error")
            .expect("Kvaser receive timeout");
        assert_eq!(received.frame().id(), f_tx.id());
        assert_eq!(received.frame().data(), f_tx.data());

        // Echo a different frame back: Kvaser -> PCAN
        thread::sleep(Duration::from_millis(100));
        channel.transmit(&f_rx_back).expect("Kvaser tx echo");
    });

    thread::sleep(Duration::from_millis(300));

    let mut pcan = can_hal_pcan::PcanDriver::new()
        .unwrap()
        .channel(0)
        .unwrap()
        .classic(ClassicBitrate::Br500K)
        .connect()
        .unwrap();
    pcan.transmit(&frame_to_kvaser).expect("PCAN tx initial");

    let echoed = pcan
        .receive_timeout(Duration::from_secs(5))
        .expect("PCAN receive error")
        .expect("PCAN receive timeout (Kvaser explicit-path echo missing)");
    assert_eq!(echoed.frame().id(), frame_from_kvaser.id());
    assert_eq!(echoed.frame().data(), frame_from_kvaser.data());

    kvaser_handle.join().expect("Kvaser thread panicked");
}

// ---------------------------------------------------------------------------
// Drop-and-reopen - verifies that closing a channel (via Drop) properly
// releases the underlying hardware handle so the same index can be reopened.
// Regression cover for the finalize_channel cleanup added to PCAN.
// ---------------------------------------------------------------------------

#[test]
fn test_pcan_drop_and_reopen() {
    let driver = can_hal_pcan::PcanDriver::new().expect("PCAN-Basic library not found");
    let frame_one = CanFrame::new(CanId::new_standard(0x111).unwrap(), &[0xAA]).unwrap();
    let frame_two = CanFrame::new(CanId::new_standard(0x222).unwrap(), &[0xBB]).unwrap();

    let f1 = frame_one.clone();
    let f2 = frame_two.clone();
    let rx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let mut channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();
        let r1 = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("receive 1 error")
            .expect("receive 1 timeout");
        assert_eq!(r1.frame().id(), f1.id());
        let r2 = channel
            .receive_timeout(Duration::from_secs(5))
            .expect("receive 2 error")
            .expect("receive 2 timeout (PCAN reopen lost the handle)");
        assert_eq!(r2.frame().id(), f2.id());
    });

    thread::sleep(Duration::from_millis(300));

    // Open, send, drop.
    {
        let mut ch = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();
        ch.transmit(&frame_one).expect("PCAN tx first");
        // ch drops here, releasing the hardware handle.
    }

    thread::sleep(Duration::from_millis(100));

    // Reopen the same index. If Drop didn't actually CAN_Uninitialize, this
    // returns PCAN_ERROR_INITIALIZE.
    let mut ch = driver
        .channel(0)
        .unwrap()
        .classic(ClassicBitrate::Br500K)
        .connect()
        .expect("PCAN reopen failed - Drop did not release the handle");
    ch.transmit(&frame_two).expect("PCAN tx second");

    rx_handle.join().expect("Kvaser receiver thread panicked");
}
