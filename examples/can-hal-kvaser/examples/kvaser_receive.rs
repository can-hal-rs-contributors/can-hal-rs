// Hardware requirements:
//   - KVASER USB adapter (e.g. Leaf Light, Leaf Pro FD, USBcan)
//   - A second CAN adapter connected and sending frames
//   - Both adapters must be configured for the same bitrate (default: 500 kbit/s)
//
// Software requirements:
//   - Linux: KVASER Linux drivers and libcanlib.so installed
//     Install from: https://www.kvaser.com/downloads-kvaser/
//   - Windows: CANlib SDK installed (canlib32.dll in system PATH)
//     Install from: https://www.kvaser.com/downloads-kvaser/
//
// Usage:
//   cargo run --example receive
//   cargo run --example receive -- <channel_index>
//
//   channel_index: 0-based channel index (default: 0)

use std::env;
use std::time::Duration;

use can_hal::Receive;
use can_hal_kvaser::KvaserDriver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening KVASER channel {channel_index}...");

    let driver = KvaserDriver::new()?;
    let mut channel = driver.channel(channel_index)?.classic(500_000)?.connect()?;

    println!("Channel opened at 500 kbit/s. Waiting for frames...");
    println!("Press Ctrl+C to stop.\n");

    loop {
        match channel.receive_timeout(Duration::from_secs(1))? {
            Some(timestamped) => {
                let frame = timestamped.frame();
                let elapsed = timestamped.timestamp().elapsed();

                println!(
                    "RX: ID=0x{:03X} DLC={} data=[{}] ({elapsed:.1?} ago)",
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
            None => {
                // Timeout, no frame received - keep waiting
            }
        }
    }
}
