// Hardware requirements:
//   - A SocketCAN-compatible adapter (e.g. KVASER, Lawicel, USBtin, MCP2515-based)
//   - A second CAN adapter connected and sending frames
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
//   cargo run --example receive
//   cargo run --example receive -- <interface>
//
//   interface: SocketCAN interface name (default: "can0")

use std::env;
use std::time::Duration;

use can_hal::Receive;
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "can0".into());

    println!("Opening SocketCAN interface '{ifname}'...");

    let mut channel = SocketCanChannel::open(&ifname)?;

    println!("Channel opened. Waiting for frames...");
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
