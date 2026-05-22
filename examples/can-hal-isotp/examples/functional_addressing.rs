// Functional addressing ISO-TP example on vcan0.
//
// Demonstrates ISO-TP functional addressing, where a broadcast CAN ID (typically
// 0x7DF for UDS) is used to send single-frame requests to all ECUs on the bus.
//
// In a real system, all ECUs listen on 0x7DF and respond on their own physical
// address (e.g., ECU1 responds on 0x7E8, ECU2 on 0x7E9, etc.). Functional
// addressing is restricted to single frames only -- multi-frame transfers
// require physical (point-to-point) addressing.
//
// This example:
//   1. Opens a sender channel configured with functional_id = 0x7DF
//   2. Opens a listener channel to verify the frame appears on the bus
//   3. Sends a UDS DiagnosticSessionControl request via send_functional()
//   4. The listener confirms the frame was transmitted on CAN ID 0x7DF
//
// Software requirements:
//   - Linux only (SocketCAN)
//   - A virtual CAN interface (vcan0)
//
// Virtual interface setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set vcan0 up
//
// Usage:
//   cargo run --example functional_addressing -p can-hal-isotp-examples -- [interface]
//
// Default interface: vcan0

use std::env;
use std::thread;
use std::time::Duration;

use can_hal::channel::Receive;
use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanChannel;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "vcan0".into());

    println!("Functional addressing ISO-TP example on '{ifname}'");
    println!("--------------------------------------------------");

    let functional_id = CanId::new_standard(0x7DF).expect("valid CAN ID");

    let ifname_listener = ifname.clone();
    let ifname_sender = ifname.clone();

    // Spawn a listener thread that watches for any frame on CAN ID 0x7DF.
    let listener_handle = thread::spawn(move || -> Result<(), String> {
        let mut channel = SocketCanChannel::open(&ifname_listener)
            .map_err(|e| format!("Listener open error: {e}"))?;

        let ts_frame = channel
            .receive_timeout(Duration::from_secs(5))
            .map_err(|e| format!("Listener receive error: {e}"))?
            .ok_or_else(|| "Listener timed out waiting for frame".to_string())?;

        let frame = ts_frame.into_frame();
        if frame.id() == CanId::new_standard(0x7DF).expect("valid CAN ID") {
            println!(
                "Listener: received frame on 0x7DF, data: {:02X?}",
                frame.data()
            );
            // Expected data: [02 10 01] (SF_DL=2, then the 2 UDS bytes)
            println!(
                "  SF_DL = {}, payload = {:02X?}",
                frame.data()[0],
                &frame.data()[1..]
            );
            Ok(())
        } else {
            Err(format!(
                "Listener: unexpected CAN ID {:?}, expected 0x7DF",
                frame.id()
            ))
        }
    });

    // Small delay to let the listener bind its socket.
    thread::sleep(Duration::from_millis(50));

    // Sender channel with functional_id configured.
    let sender_channel =
        SocketCanChannel::open(&ifname_sender).map_err(|e| format!("Sender open error: {e}"))?;

    let config = IsoTpConfig {
        functional_id: Some(functional_id),
        ..IsoTpConfig::new(
            CanId::new_standard(0x7E0).expect("valid CAN ID"),
            CanId::new_standard(0x7E8).expect("valid CAN ID"),
        )
    };

    let mut isotp = IsoTpChannel::new(sender_channel, config);

    // UDS DiagnosticSessionControl (0x10) - Default Session (0x01)
    // Service ID: 0x10, Sub-function: 0x01
    // Wrapped in ISO-TP: SF_DL=2, then [10 01]
    let uds_request = [0x10, 0x01];
    println!("Sending UDS DiagnosticSessionControl (functional) on 0x7DF...");
    isotp
        .send_functional(&uds_request)
        .map_err(|e| format!("send_functional error: {e}"))?;
    println!("Sent successfully.");

    // Wait for the listener to confirm.
    match listener_handle.join().expect("Listener thread panicked") {
        Ok(()) => println!("Listener confirmed frame on 0x7DF."),
        Err(e) => eprintln!("Listener error: {e}"),
    }

    println!("\nFunctional addressing example complete.");
    Ok(())
}
