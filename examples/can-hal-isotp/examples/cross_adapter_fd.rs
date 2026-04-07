//! Cross-adapter ISO-TP CAN FD test: PCAN <-> Kvaser.
//!
//! Sends ISO-TP messages over CAN FD (500 kbit/s nominal, 4 Mbit/s data) between
//! a PCAN-USB FD adapter and a Kvaser U100 connected on the same CAN bus.
//!
//! Tests single-frame and multi-frame transfers in both directions using
//! 64-byte FD frames for higher throughput.
//!
//! Hardware requirements:
//!   - PCAN-USB FD and Kvaser U100 (or other FD-capable adapters)
//!   - Both connected on the same CAN bus with proper termination
//!
//! Software requirements:
//!   - Linux: pcan driver + libpcanbasic.so, linuxcan (mhydra) + libcanlib.so
//!
//! Usage:
//!   cargo run --example cross_adapter_fd -p can-hal-isotp-examples

use std::thread;
use std::time::Duration;

use can_hal::CanId;
use can_hal::{ChannelBuilder, Driver};
use can_hal_isotp::{IsoTpConfig, IsoTpFdChannel};

fn main() {
    println!("=== ISO-TP CAN FD Cross-Adapter Test: PCAN <-> Kvaser ===\n");

    let tx_id = CanId::new_standard(0x7E0).unwrap();
    let rx_id = CanId::new_standard(0x7E8).unwrap();

    // Test payloads: FD single frame sizes and multi-frame
    let payloads: Vec<Vec<u8>> = vec![
        vec![0x10, 0x01],                                 // 2 bytes: UDS DiagSessionControl
        (0..7).collect(),                                 // 7 bytes: classic SF limit
        (0..62).map(|i| i as u8).collect(),               // 62 bytes: max FD single frame
        (0..200).map(|i| i as u8).collect(),              // 200 bytes: multi-frame
        (0..1000).map(|i| i as u8).collect(),             // 1000 bytes: large multi-frame
        (0..4000u16).map(|i| (i & 0xFF) as u8).collect(), // 4000 bytes: stress test
    ];

    // --- PCAN -> Kvaser ---
    for (i, payload) in payloads.iter().enumerate() {
        println!(
            "--- Test {}/{}: {} bytes (PCAN -> Kvaser) ---",
            i + 1,
            payloads.len(),
            payload.len()
        );

        let payload_send = payload.clone();
        let payload_expected = payload.clone();

        // Spawn Kvaser receiver thread
        let rx_handle = thread::spawn(move || {
            let driver = can_hal_kvaser::KvaserDriver::new().expect("CANlib not found");
            let channel = driver
                .channel(0)
                .unwrap()
                .bitrate(500_000)
                .unwrap()
                .data_bitrate(4_000_000)
                .unwrap()
                .connect()
                .unwrap();

            let mut config = IsoTpConfig::new(rx_id, tx_id);
            config.timeout = Duration::from_secs(10);
            let mut isotp = IsoTpFdChannel::new(channel, config);

            let received = isotp.receive().expect("ISO-TP FD receive failed");
            assert_eq!(received, payload_expected, "Payload mismatch!");
            println!("  Kvaser RX: {} bytes OK", received.len());
        });

        // Small delay to let receiver set up
        thread::sleep(Duration::from_millis(200));

        // PCAN sender
        let tx_handle = thread::spawn(move || {
            let driver = can_hal_pcan::PcanDriver::new().expect("PCAN not found");
            let channel = driver
                .channel(0)
                .unwrap()
                .bitrate(500_000)
                .unwrap()
                .data_bitrate(4_000_000)
                .unwrap()
                .connect()
                .unwrap();

            let mut config = IsoTpConfig::new(tx_id, rx_id);
            config.timeout = Duration::from_secs(10);
            let mut isotp = IsoTpFdChannel::new(channel, config);

            isotp.send(&payload_send).expect("ISO-TP FD send failed");
            println!("  PCAN  TX: {} bytes OK", payload_send.len());
        });

        tx_handle.join().unwrap();
        rx_handle.join().unwrap();
        println!("  PASS\n");

        thread::sleep(Duration::from_millis(100));
    }

    // --- Kvaser -> PCAN ---
    println!(
        "--- Test {}: 500 bytes (Kvaser -> PCAN) ---",
        payloads.len() + 1
    );
    let payload: Vec<u8> = (0..500).map(|i| (i * 3) as u8).collect();
    let payload_send = payload.clone();
    let payload_expected = payload.clone();

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().expect("PCAN not found");
        let channel = driver
            .channel(0)
            .unwrap()
            .bitrate(500_000)
            .unwrap()
            .data_bitrate(4_000_000)
            .unwrap()
            .connect()
            .unwrap();

        let mut config = IsoTpConfig::new(rx_id, tx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpFdChannel::new(channel, config);

        let received = isotp.receive().expect("ISO-TP FD receive failed");
        assert_eq!(received, payload_expected, "Payload mismatch!");
        println!("  PCAN  RX: {} bytes OK", received.len());
    });

    thread::sleep(Duration::from_millis(200));

    let tx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().expect("CANlib not found");
        let channel = driver
            .channel(0)
            .unwrap()
            .bitrate(500_000)
            .unwrap()
            .data_bitrate(4_000_000)
            .unwrap()
            .connect()
            .unwrap();

        let mut config = IsoTpConfig::new(tx_id, rx_id);
        config.timeout = Duration::from_secs(10);
        let mut isotp = IsoTpFdChannel::new(channel, config);

        isotp.send(&payload_send).expect("ISO-TP FD send failed");
        println!("  Kvaser TX: {} bytes OK", payload_send.len());
    });

    tx_handle.join().unwrap();
    rx_handle.join().unwrap();
    println!("  PASS\n");

    println!("=== All ISO-TP CAN FD cross-adapter tests PASSED ===");
}
