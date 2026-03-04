// Hardware requirements:
//   - PCAN USB adapter (e.g. PCAN-USB, PCAN-USB FD, PCAN-USB Pro)
//   - A second CAN adapter connected to the PCAN adapter to provide bus ACK
//   - Both adapters must be configured for the same bitrate (default: 500 kbit/s)
//
// Software requirements:
//   - Linux: PCAN driver (peak-linux-driver) and libpcanbasic.so installed
//     Install from: https://www.peak-system.com/fileadmin/media/linux/
//   - Windows: PCAN-Basic API (PCANBasic.dll) installed
//     Install from: https://www.peak-system.com/PCAN-Basic.239.0.html
//
// Usage:
//   cargo run --example send
//   cargo run --example send -- <channel_index>
//
//   channel_index: 0-based USB channel index (default: 0 = PCAN_USBBUS1)

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::{CanFrame, CanId, ChannelBuilder, Driver, Transmit};
use can_hal_pcan::PcanDriver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening PCAN USB channel {channel_index}...");

    let driver = PcanDriver::new()?;
    let mut channel = driver.channel(channel_index)?.bitrate(500_000)?.connect()?;

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
                    frame.dlc(),
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
