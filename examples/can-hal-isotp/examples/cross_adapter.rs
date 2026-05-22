//! Cross-adapter ISO-TP test: PCAN <-> Kvaser.
//!
//! Sends ISO-TP messages of various sizes between a PCAN adapter and a Kvaser
//! adapter connected on the same CAN bus. Tests single-frame and multi-frame
//! transfers in both directions.

use std::thread;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_pcan::ClassicBitrate;

fn main() {
    println!("=== ISO-TP Cross-Adapter Test: PCAN <-> Kvaser ===\n");

    let tx_id = CanId::new_standard(0x7E0).unwrap();
    let rx_id = CanId::new_standard(0x7E8).unwrap();

    // Test payloads: single frame (7 bytes), multi-frame (20 bytes), large (200 bytes)
    let payloads: Vec<Vec<u8>> = vec![
        vec![0x10, 0x01],                     // 2 bytes: UDS DiagSessionControl
        (0..7).collect(),                     // 7 bytes: max single frame
        (0..20).map(|i| i as u8).collect(),   // 20 bytes: multi-frame
        (0..200).map(|i| i as u8).collect(),  // 200 bytes: larger multi-frame
        (0..1000).map(|i| i as u8).collect(), // 1000 bytes: large multi-frame
    ];

    for (i, payload) in payloads.iter().enumerate() {
        println!(
            "--- Test {}: {} bytes (PCAN -> Kvaser) ---",
            i + 1,
            payload.len()
        );

        let payload_send = payload.clone();
        let payload_expected = payload.clone();

        // Spawn Kvaser receiver thread
        let rx_handle = thread::spawn(move || {
            let driver = can_hal_kvaser::KvaserDriver::new().expect("CANlib not found");
            let channel = driver
                .channel(0)
                .classic(500_000)
                .unwrap()
                .connect()
                .unwrap();

            let mut config = IsoTpConfig::new(rx_id, tx_id);
            config.timeout = Duration::from_secs(10);
            // Uses sensible defaults: st_min=5ms, padding=0xCC
            let mut isotp = IsoTpChannel::new(channel, config);

            let received = isotp.receive().expect("ISO-TP receive failed");
            assert_eq!(received, payload_expected, "Payload mismatch!");
            println!("  Kvaser RX: {} bytes OK", received.len());
            received
        });

        // Small delay to let receiver set up
        thread::sleep(Duration::from_millis(200));

        // PCAN sender
        let tx_handle = thread::spawn(move || {
            let driver = can_hal_pcan::PcanDriver::new().expect("PCAN not found");
            let channel = driver
                .channel(0)
                .unwrap()
                .classic(ClassicBitrate::Br500K)
                .connect()
                .unwrap();

            let mut config = IsoTpConfig::new(tx_id, rx_id);
            config.timeout = Duration::from_secs(10);
            // Uses sensible defaults: st_min=5ms, padding=0xCC
            let mut isotp = IsoTpChannel::new(channel, config);

            isotp.send(&payload_send).expect("ISO-TP send failed");
            println!("  PCAN  TX: {} bytes OK", payload_send.len());
        });

        tx_handle.join().unwrap();
        rx_handle.join().unwrap();
        println!("  PASS\n");

        // Small pause between tests
        thread::sleep(Duration::from_millis(100));
    }

    // Now test the reverse direction: Kvaser -> PCAN
    println!("--- Test 5: 100 bytes (Kvaser -> PCAN) ---");
    let payload: Vec<u8> = (0..100).map(|i| (i * 3) as u8).collect();
    let payload_send = payload.clone();
    let payload_expected = payload.clone();

    let rx_handle = thread::spawn(move || {
        let driver = can_hal_pcan::PcanDriver::new().expect("PCAN not found");
        let channel = driver
            .channel(0)
            .unwrap()
            .classic(ClassicBitrate::Br500K)
            .connect()
            .unwrap();

        let mut config = IsoTpConfig::new(rx_id, tx_id);
        config.timeout = Duration::from_secs(10);
        // Uses sensible defaults: st_min=5ms, padding=0xCC
        let mut isotp = IsoTpChannel::new(channel, config);

        let received = isotp.receive().expect("ISO-TP receive failed");
        assert_eq!(received, payload_expected, "Payload mismatch!");
        println!("  PCAN  RX: {} bytes OK", received.len());
    });

    thread::sleep(Duration::from_millis(200));

    let tx_handle = thread::spawn(move || {
        let driver = can_hal_kvaser::KvaserDriver::new().expect("CANlib not found");
        let channel = driver
            .channel(0)
            .classic(500_000)
            .unwrap()
            .connect()
            .unwrap();

        let mut config = IsoTpConfig::new(tx_id, rx_id);
        config.timeout = Duration::from_secs(10);
        // Uses sensible defaults: st_min=5ms, padding=0xCC
        let mut isotp = IsoTpChannel::new(channel, config);

        isotp.send(&payload_send).expect("ISO-TP send failed");
        println!("  Kvaser TX: {} bytes OK", payload_send.len());
    });

    tx_handle.join().unwrap();
    rx_handle.join().unwrap();
    println!("  PASS\n");

    println!("=== All ISO-TP cross-adapter tests PASSED ===");
}
