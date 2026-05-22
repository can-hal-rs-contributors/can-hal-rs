// Receive a single ISO-TP message over SocketCAN and print it.
//
// Software requirements:
//   - Linux only (SocketCAN is a Linux kernel subsystem)
//   - A SocketCAN interface (virtual or physical)
//
// Virtual interface setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set vcan0 up
//
// Usage:
//   cargo run --example isotp_receive -p can-hal-isotp-examples -- [interface]
//
// Default interface: vcan0
//
// The receiver uses tx_id=0x7E8, rx_id=0x7E0 (the inverse of "send").
// Run the "send" example in another terminal to send a message.

use std::env;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "vcan0".into());

    println!("Opening SocketCAN interface '{ifname}'...");
    let channel = SocketCanChannel::open(&ifname)?;

    // Configure ISO-TP: transmit FC on 0x7E8, receive data on 0x7E0.
    let config = IsoTpConfig {
        timeout: Duration::from_secs(10),
        ..IsoTpConfig::new(
            CanId::new_standard(0x7E8).expect("valid CAN ID"),
            CanId::new_standard(0x7E0).expect("valid CAN ID"),
        )
    };

    let mut isotp = IsoTpChannel::new(channel, config);

    println!("Waiting for ISO-TP message (timeout: 10s)...");

    let data = isotp.receive()?;

    println!(
        "Received {} byte(s): {}",
        data.len(),
        data.iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" "),
    );

    Ok(())
}
