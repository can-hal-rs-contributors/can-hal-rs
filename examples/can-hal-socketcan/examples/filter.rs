// Hardware requirements:
//   - A SocketCAN-compatible adapter (e.g. KVASER, Lawicel, USBtin, MCP2515-based)
//   - A second CAN adapter connected and sending frames on various IDs
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
// This example sets up a filter to only accept frames with standard IDs in the
// range 0x100-0x1FF (mask 0x700 matches the upper 3 bits). Frames outside this
// range are discarded by the kernel before reaching userspace.
//
// Usage:
//   cargo run --example filter
//   cargo run --example filter -- <interface>
//
//   interface: SocketCAN interface name (default: "can0")

use std::env;
use std::time::Duration;

use can_hal::{CanId, Filter, Filterable, Receive};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "can0".into());

    println!("Opening SocketCAN interface '{ifname}'...");

    let mut channel = SocketCanChannel::open(&ifname)?;

    // Accept only standard IDs where the upper 3 bits are 0b001 (0x100-0x1FF).
    // The mask 0x700 checks bits 8-10 of the 11-bit standard ID.
    let filter = Filter {
        id: CanId::new_standard(0x100).expect("valid standard ID"),
        mask: 0x700,
    };

    channel.set_filters(&[filter])?;

    println!("Filter set: accepting standard IDs 0x100-0x1FF only.");
    println!("Waiting for frames...");
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
