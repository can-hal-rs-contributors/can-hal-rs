// Hardware requirements:
//   - KVASER CAN FD adapter (e.g. Kvaser U100, Leaf Pro HS v2)
//   - A second CAN FD adapter connected to the KVASER adapter to provide bus ACK
//   - Both adapters must be configured for the same bitrates (500 kbit/s nominal, 4 Mbit/s data)
//
// Software requirements:
//   - Linux: KVASER linuxcan drivers and libcanlib.so installed
//     Install from: https://www.kvaser.com/downloads-kvaser/
//   - Windows: CANlib SDK installed (canlib32.dll in system PATH)
//     Install from: https://www.kvaser.com/downloads-kvaser/
//
// Usage:
//   cargo run --example kvaser_send_fd
//   cargo run --example kvaser_send_fd -- <channel_index>
//
//   channel_index: 0-based channel index (default: 0)

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::{CanFdFrame, CanId, ChannelBuilder, Driver, TransmitFd};
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

    println!("Channel opened at 500 kbit/s nominal, 4 Mbit/s data. Sending FD frames...");
    println!("Press Ctrl+C to stop.\n");

    let id = CanId::new_standard(0x200).expect("valid standard ID");
    let mut counter: u8 = 0;

    loop {
        // Build a 64-byte CAN FD payload with BRS (bit rate switch) enabled.
        let mut data = [0u8; 64];
        data[0] = counter;
        data[1] = !counter;
        for i in 2..64 {
            data[i] = (counter.wrapping_add(i as u8)) ^ 0xAA;
        }

        let frame = CanFdFrame::new(id, &data, true, false).expect("valid FD frame");

        match channel.transmit_fd(&frame) {
            Ok(()) => {
                println!(
                    "TX: ID=0x{:03X} DLC={} BRS={} data=[{} ...]",
                    frame.id().raw(),
                    frame.len(),
                    frame.brs(),
                    frame.data()[..8]
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
