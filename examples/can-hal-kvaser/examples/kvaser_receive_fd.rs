// Hardware requirements:
//   - KVASER CAN FD adapter (e.g. Kvaser U100, Leaf Pro HS v2)
//   - A second CAN FD adapter connected and sending frames
//   - Both adapters must be configured for the same bitrates (500 kbit/s nominal, 4 Mbit/s data)
//
// Software requirements:
//   - Linux: KVASER linuxcan drivers and libcanlib.so installed
//     Install from: https://www.kvaser.com/downloads-kvaser/
//   - Windows: CANlib SDK installed (canlib32.dll in system PATH)
//     Install from: https://www.kvaser.com/downloads-kvaser/
//
// Usage:
//   cargo run --example kvaser_receive_fd
//   cargo run --example kvaser_receive_fd -- <channel_index>
//
//   channel_index: 0-based channel index (default: 0)

use std::env;
use std::time::Duration;

use can_hal::{ChannelBuilder, Driver, Frame, ReceiveFd};
use can_hal_kvaser::KvaserDriver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening KVASER channel {channel_index} in CAN FD mode...");

    let driver = KvaserDriver::new()?;
    let mut channel = driver
        .channel(channel_index)?
        .bitrate(500_000)?
        .data_bitrate(4_000_000)?
        .connect()?;

    println!("Channel opened at 500 kbit/s nominal, 4 Mbit/s data. Waiting for frames...");
    println!("Press Ctrl+C to stop.\n");

    loop {
        match channel.receive_fd_timeout(Duration::from_secs(1))? {
            Some(timestamped) => {
                let elapsed = timestamped.timestamp().elapsed();

                match timestamped.frame() {
                    Frame::Fd(fd) => {
                        println!(
                            "RX FD: ID=0x{:03X} DLC={} BRS={} ESI={} data=[{}] ({elapsed:.1?} ago)",
                            fd.id().raw(),
                            fd.len(),
                            fd.brs(),
                            fd.esi(),
                            fd.data()
                                .iter()
                                .map(|b| format!("{b:02X}"))
                                .collect::<Vec<_>>()
                                .join(" "),
                        );
                    }
                    Frame::Can(classic) => {
                        println!(
                            "RX classic: ID=0x{:03X} DLC={} data=[{}] ({elapsed:.1?} ago)",
                            classic.id().raw(),
                            classic.len(),
                            classic
                                .data()
                                .iter()
                                .map(|b| format!("{b:02X}"))
                                .collect::<Vec<_>>()
                                .join(" "),
                        );
                    }
                }
            }
            None => {
                // Timeout, no frame received — keep waiting
            }
        }
    }
}
