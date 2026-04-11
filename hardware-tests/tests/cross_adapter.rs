use std::thread;
use std::time::Duration;

use can_hal::channel::Receive;
use can_hal::frame::CanFrame;
use can_hal::{BusState, BusStatus, CanId, ChannelBuilder, Driver, Transmit};
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};

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
        .bitrate(500_000)
        .unwrap()
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
            .unwrap()
            .bitrate(500_000)
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
            .bitrate(500_000)
            .unwrap()
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
            .bitrate(500_000)
            .unwrap()
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
            .unwrap()
            .bitrate(500_000)
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
            .unwrap()
            .bitrate(500_000)
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
            .bitrate(500_000)
            .unwrap()
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
            .bitrate(500_000)
            .unwrap()
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
            .unwrap()
            .bitrate(500_000)
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
        .unwrap()
        .bitrate(500_000)
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
