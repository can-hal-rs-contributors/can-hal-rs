use std::thread;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{IsoTpConfig, IsoTpFdChannel};

const TX_ID: u16 = 0x7E0;
const RX_ID: u16 = 0x7E8;

fn isotp_fd_transfer_pcan_to_kvaser(payload: &[u8]) {
    let tx_id = CanId::new_standard(TX_ID).unwrap();
    let rx_id = CanId::new_standard(RX_ID).unwrap();
    let payload_send = payload.to_vec();
    let payload_expected = payload.to_vec();

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(rx_id, tx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpFdChannel::new(channel, config);
        isotp.receive().expect("Kvaser ISO-TP FD receive failed")
    });

    thread::sleep(Duration::from_millis(200));

    let tx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .unwrap()
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(tx_id, rx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpFdChannel::new(channel, config);
        isotp
            .send(&payload_send)
            .expect("PCAN ISO-TP FD send failed");
    });

    tx_handle.join().expect("Sender thread panicked");
    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received, payload_expected, "Payload mismatch");
}

fn isotp_fd_transfer_kvaser_to_pcan(payload: &[u8]) {
    let tx_id = CanId::new_standard(TX_ID).unwrap();
    let rx_id = CanId::new_standard(RX_ID).unwrap();
    let payload_send = payload.to_vec();
    let payload_expected = payload.to_vec();

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .unwrap()
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(rx_id, tx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpFdChannel::new(channel, config);
        isotp.receive().expect("PCAN ISO-TP FD receive failed")
    });

    thread::sleep(Duration::from_millis(200));

    let tx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().unwrap();
        let channel = driver
            .channel(0)
            .fd(500_000, 4_000_000)
            .unwrap()
            .connect()
            .unwrap();
        let mut config = IsoTpConfig::new(tx_id, rx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpFdChannel::new(channel, config);
        isotp
            .send(&payload_send)
            .expect("Kvaser ISO-TP FD send failed");
    });

    tx_handle.join().expect("Sender thread panicked");
    let received = rx_handle.join().expect("Receiver thread panicked");
    assert_eq!(received, payload_expected, "Payload mismatch");
}

// --- PCAN -> Kvaser (FD) ---

#[test]
fn test_isotp_fd_single_frame_2_bytes() {
    isotp_fd_transfer_pcan_to_kvaser(&[0x10, 0x01]);
}

#[test]
fn test_isotp_fd_single_frame_62_bytes() {
    isotp_fd_transfer_pcan_to_kvaser(&(0..62).map(|i| i as u8).collect::<Vec<u8>>());
}

#[test]
fn test_isotp_fd_multi_frame_200_bytes() {
    isotp_fd_transfer_pcan_to_kvaser(&(0..200).map(|i| i as u8).collect::<Vec<u8>>());
}

#[test]
fn test_isotp_fd_multi_frame_1000_bytes() {
    isotp_fd_transfer_pcan_to_kvaser(&(0..1000).map(|i| i as u8).collect::<Vec<u8>>());
}

#[test]
fn test_isotp_fd_multi_frame_4000_bytes() {
    isotp_fd_transfer_pcan_to_kvaser(&(0..4000u16).map(|i| (i & 0xFF) as u8).collect::<Vec<u8>>());
}

// --- Kvaser -> PCAN (FD) ---

#[test]
fn test_isotp_fd_reverse_500_bytes() {
    isotp_fd_transfer_kvaser_to_pcan(&(0..500).map(|i| (i * 3) as u8).collect::<Vec<u8>>());
}
