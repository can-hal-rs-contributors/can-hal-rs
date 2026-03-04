// Hardware requirements:
//   - PCAN USB adapter (e.g. PCAN-USB, PCAN-USB FD, PCAN-USB Pro)
//   - A second CAN adapter connected and sending frames
//   - Both adapters must be configured for the same bitrate (default: 500 kbit/s)
//
// Software requirements:
//   - Linux: PCAN driver (peak-linux-driver) and libpcanbasic.so installed
//     Install from: https://www.peak-system.com/fileadmin/media/linux/
//   - Windows: PCAN-Basic API (PCANBasic.dll) installed
//     Install from: https://www.peak-system.com/PCAN-Basic.239.0.html
//
// Usage:
//   cargo run --example receive
//   cargo run --example receive -- <channel_index>
//
//   channel_index: 0-based USB channel index (default: 0 = PCAN_USBBUS1)

use std::env;
use std::time::Duration;

use can_hal::{ChannelBuilder, Driver, Receive};
use can_hal_pcan::PcanDriver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening PCAN USB channel {channel_index}...");

    let driver = PcanDriver::new()?;
    let mut channel = driver.channel(channel_index)?.bitrate(500_000)?.connect()?;

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
                    frame.dlc(),
                    frame
                        .data()
                        .iter()
                        .map(|b| format!("{b:02X}"))
                        .collect::<Vec<_>>()
                        .join(" "),
                );
            }
            None => {
                // Timeout, no frame received — keep waiting
            }
        }
    }
}
