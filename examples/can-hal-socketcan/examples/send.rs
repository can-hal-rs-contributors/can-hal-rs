// Hardware requirements:
//   - A SocketCAN-compatible adapter (e.g. KVASER, Lawicel, USBtin, MCP2515-based)
//   - A second CAN adapter connected to provide bus ACK
//   - Both adapters must be configured for the same bitrate
//
// Software requirements:
//   - Linux only (SocketCAN is a Linux kernel subsystem)
//   - Adapter kernel module loaded (e.g. kvaser_usb, gs_usb, peak_usb, etc.)
//
// Interface setup (run as root or with sudo):
//   sudo ip link set can0 type can bitrate 500000
//   sudo ip link set can0 up
//
// Usage:
//   cargo run --example send
//   cargo run --example send -- <interface>
//
//   interface: SocketCAN interface name (default: "can0")

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::{CanFrame, CanId, Transmit};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "can0".into());

    println!("Opening SocketCAN interface '{ifname}'...");

    let mut channel = SocketCanChannel::open(&ifname)?;

    println!("Channel opened. Sending frames...");
    println!("Press Ctrl+C to stop.\n");

    let id = CanId::new_standard(0x200).expect("valid standard ID");
    let mut counter: u8 = 0;

    loop {
        let data = [counter, !counter, 0xBE, 0xEF, 0x00, 0x00, 0x00, counter];
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
