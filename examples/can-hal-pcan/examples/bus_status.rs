// Hardware requirements:
//   - PCAN USB adapter (e.g. PCAN-USB, PCAN-USB FD, PCAN-USB Pro)
//   - Optionally, a second CAN adapter connected for a loaded bus
//   - Note: this example only reads the bus state and error counters; it never
//     transmits. To see the state change, have another node send at a different
//     bitrate, which makes this controller count receive errors.
//
// Software requirements:
//   - Linux: PCAN driver (peak-linux-driver) and libpcanbasic.so installed
//     Install from: https://www.peak-system.com/fileadmin/media/linux/
//   - Windows: PCAN-Basic API (PCANBasic.dll) installed
//     Install from: https://www.peak-system.com/PCAN-Basic.239.0.html
//
// Usage:
//   cargo run --example bus_status
//   cargo run --example bus_status -- <channel_index>
//
//   channel_index: 0-based USB channel index (default: 0 = PCAN_USBBUS1)

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::{BusState, BusStatus};
use can_hal_pcan::{ClassicBitrate, PcanDriver};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let channel_index: u32 = env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(0);

    println!("Opening PCAN USB channel {channel_index}...");

    let driver = PcanDriver::new()?;
    let channel = driver
        .channel(channel_index)?
        .classic(ClassicBitrate::Br500K)
        .connect()?;

    println!("Channel opened at 500 kbit/s. Polling bus status...");
    println!("Press Ctrl+C to stop.\n");

    loop {
        let state = channel.bus_state()?;
        // error_counters() is unsupported on Windows.
        let counters = match channel.error_counters() {
            Ok(c) => format!("TX errors: {}  |  RX errors: {}", c.transmit, c.receive),
            Err(e) => format!("error counters: {e}"),
        };

        let state_str = match state {
            BusState::ErrorActive => "Error Active (normal)",
            BusState::ErrorPassive => "Error Passive",
            BusState::BusOff => "Bus Off",
        };

        println!("Bus: {state_str}  |  {counters}");

        thread::sleep(Duration::from_secs(1));
    }
}
