// Hardware requirements:
//   - KVASER USB adapter (e.g. Leaf Light, Leaf Pro FD, USBcan)
//   - A second CAN adapter connected to the KVASER adapter to provide bus ACK
//   - Both adapters must be configured for the same bitrate (default: 500 kbit/s)
//
// Software requirements:
//   - Linux: KVASER Linux drivers and libcanlib.so installed
//     Install from: https://www.kvaser.com/downloads-kvaser/
//   - Windows: CANlib SDK installed (canlib32.dll in system PATH)
//     Install from: https://www.kvaser.com/downloads-kvaser/
//
// Usage:
//   cargo run --example kvaser_send
//   cargo run --example kvaser_send -- <channel_index>
//
//   channel_index: 0-based channel index (default: 0)

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::{CanFrame, CanId, Transmit};
use can_hal_kvaser::KvaserDriver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening KVASER channel {channel_index}...");

    let driver = KvaserDriver::new()?;
    let mut channel = driver.channel(channel_index).classic(500_000)?.connect()?;

    println!("Channel opened at 500 kbit/s. Sending frames...");
    println!("Press Ctrl+C to stop.\n");

    let id = CanId::new_standard(0x100).expect("valid standard ID");
    let mut counter: u8 = 0;

    loop {
        let data = [counter, !counter, 0xCA, 0xFE, 0x00, 0x00, 0x00, counter];
        let frame = CanFrame::new(id, &data).expect("valid frame");

        match channel.transmit(&frame) {
            Ok(()) => {
                println!(
                    "TX: ID=0x{:03X} DLC={} data=[{}]",
                    frame.id().raw(),
                    frame.len(),
                    frame
                        .data()
                        .iter()
                        .map(|b| format!("{b:02X}"))
                        .collect::<Vec<_>>()
                        .join(" "),
                );
            }
            Err(e) => {
                eprintln!("TX error: {e}");
            }
        }

        counter = counter.wrapping_add(1);
        thread::sleep(Duration::from_secs(1));
    }
}
