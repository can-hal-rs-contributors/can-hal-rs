// Hardware requirements:
//   - PCAN-USB FD adapter (must support CAN FD)
//   - A second CAN FD adapter connected and sending frames
//   - Both adapters must be configured for the same bitrates (500 kbit/s nominal, 4 Mbit/s data)
//
// Software requirements:
//   - Linux: PCAN driver (peak-linux-driver) and libpcanbasic.so installed
//     Install from: https://www.peak-system.com/fileadmin/media/linux/
//   - Windows: PCAN-Basic API (PCANBasic.dll) installed
//     Install from: https://www.peak-system.com/PCAN-Basic.239.0.html
//
// Usage:
//   cargo run --example pcan_receive_fd
//   cargo run --example pcan_receive_fd -- <channel_index>
//
//   channel_index: 0-based USB channel index (default: 0 = PCAN_USBBUS1)

use std::env;
use std::time::Duration;

use can_hal::{ChannelBuilder, Driver, Frame, ReceiveFd};
use can_hal_pcan::PcanDriver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening PCAN USB channel {channel_index} in CAN FD mode...");

    let driver = PcanDriver::new()?;
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
