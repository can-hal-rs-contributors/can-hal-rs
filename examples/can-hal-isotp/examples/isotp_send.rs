// Send a single ISO-TP message over SocketCAN.
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
//   cargo run --example send -p can-hal-isotp-examples -- [interface] [hex_payload]
//
// Defaults: interface = "vcan0", payload = "DEADBEEF01020304"
//
// The sender uses tx_id=0x7E0, rx_id=0x7E8.
// Run the "receive" example in another terminal to see the message.

use std::env;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "vcan0".into());
    let hex_payload = env::args()
        .nth(2)
        .unwrap_or_else(|| "DEADBEEF01020304".into());

    // Parse the hex payload into bytes.
    let payload = hex_to_bytes(&hex_payload)?;

    println!("Opening SocketCAN interface '{ifname}'...");
    let channel = SocketCanChannel::open(&ifname)?;

    // Configure ISO-TP: transmit on 0x7E0, receive flow-control on 0x7E8.
    let config = IsoTpConfig {
        timeout: Duration::from_secs(2),
        ..IsoTpConfig::new(
            CanId::new_standard(0x7E0).expect("valid CAN ID"),
            CanId::new_standard(0x7E8).expect("valid CAN ID"),
        )
    };

    let mut isotp = IsoTpChannel::new(channel, config);

    println!(
        "Sending {} byte(s): {}",
        payload.len(),
        bytes_to_hex(&payload),
    );

    isotp.send(&payload)?;

    println!("ISO-TP message sent successfully.");
    Ok(())
}

fn hex_to_bytes(hex: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    if !hex.len().is_multiple_of(2) {
        return Err("hex string must have an even number of characters".into());
    }
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).map_err(Into::into))
        .collect()
}

fn bytes_to_hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}
