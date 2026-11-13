// Kernel CAN_ISOTP interoperability test.
//
// Validates our `IsoTpChannel` against the Linux kernel's built-in CAN_ISOTP
// socket implementation (the reference ISO-TP). If both sides agree on the
// reassembled payload, our protocol implementation is correct.
//
// Requirements:
//   - Linux kernel >= 5.10 with CONFIG_CAN_ISOTP=y (or can-isotp module loaded)
//   - A SocketCAN interface (virtual or physical)
//
// Virtual interface setup:
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan
//   sudo ip link set vcan0 up
//
// Usage:
//   cargo run --example kernel_interop -p can-hal-isotp-examples -- [interface]
//
// Default interface: vcan0

use std::env;
use std::ffi::CString;
use std::thread;
use std::time::Duration;

use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanChannel;

// ---- Kernel CAN_ISOTP socket constants and types ----------------------------

const AF_CAN: i32 = 29;
const PF_CAN: i32 = 29;
const CAN_ISOTP: i32 = 6;

#[repr(C)]
struct CanAddrTp {
    rx_id: u32,
    tx_id: u32,
}

#[repr(C)]
struct SockaddrCan {
    can_family: u16,
    can_ifindex: i32,
    can_addr: CanAddrTp,
}

// ---- Helpers ----------------------------------------------------------------

/// Open and bind a kernel CAN_ISOTP socket. Returns the file descriptor.
///
/// `rx_id` / `tx_id` are from the kernel socket's perspective.
fn open_kernel_isotp_socket(ifname: &str, rx_id: u32, tx_id: u32) -> Result<i32, String> {
    let sock = unsafe { libc::socket(PF_CAN, libc::SOCK_DGRAM, CAN_ISOTP) };
    if sock < 0 {
        return Err(format!(
            "socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP) failed: {}",
            std::io::Error::last_os_error()
        ));
    }

    let c_ifname = CString::new(ifname).map_err(|e| format!("invalid interface name: {e}"))?;
    let ifindex = unsafe { libc::if_nametoindex(c_ifname.as_ptr()) };
    if ifindex == 0 {
        unsafe { libc::close(sock) };
        return Err(format!(
            "if_nametoindex({ifname}) failed: {}",
            std::io::Error::last_os_error()
        ));
    }

    let addr = SockaddrCan {
        can_family: AF_CAN as u16,
        can_ifindex: ifindex as i32,
        can_addr: CanAddrTp { rx_id, tx_id },
    };

    let ret = unsafe {
        libc::bind(
            sock,
            &addr as *const SockaddrCan as *const libc::sockaddr,
            std::mem::size_of::<SockaddrCan>() as libc::socklen_t,
        )
    };
    if ret < 0 {
        let err = std::io::Error::last_os_error();
        unsafe { libc::close(sock) };
        return Err(format!("bind() failed: {err}"));
    }

    // Set a 5-second receive timeout so reads don't hang forever.
    let tv = libc::timeval {
        tv_sec: 5,
        tv_usec: 0,
    };
    let ret = unsafe {
        libc::setsockopt(
            sock,
            libc::SOL_SOCKET,
            libc::SO_RCVTIMEO,
            &tv as *const libc::timeval as *const libc::c_void,
            std::mem::size_of::<libc::timeval>() as libc::socklen_t,
        )
    };
    if ret < 0 {
        let err = std::io::Error::last_os_error();
        unsafe { libc::close(sock) };
        return Err(format!("setsockopt(SO_RCVTIMEO) failed: {err}"));
    }

    Ok(sock)
}

fn kernel_read(sock: i32, buf: &mut [u8]) -> Result<usize, String> {
    let n = unsafe { libc::read(sock, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
    if n < 0 {
        Err(format!(
            "read() failed: {}",
            std::io::Error::last_os_error()
        ))
    } else {
        Ok(n as usize)
    }
}

fn kernel_write(sock: i32, data: &[u8]) -> Result<usize, String> {
    let n = unsafe { libc::write(sock, data.as_ptr() as *const libc::c_void, data.len()) };
    if n < 0 {
        Err(format!(
            "write() failed: {}",
            std::io::Error::last_os_error()
        ))
    } else {
        Ok(n as usize)
    }
}

// ---- Tests ------------------------------------------------------------------

/// Tests A and C: our IsoTpChannel sends, kernel socket receives.
fn test_our_send_kernel_recv(test: char, ifname: &str, payload: &[u8]) -> bool {
    let label = format!(
        "Test {test} ({} bytes): our crate sends, kernel receives",
        payload.len()
    );
    println!("\n{label}");

    let ifname_k = ifname.to_string();
    let expected = payload.to_vec();

    // Spawn kernel reader thread first.
    let rx_handle = thread::spawn(move || -> Result<Vec<u8>, String> {
        // Kernel socket: rx_id=0x7E0 (data from our crate), tx_id=0x7E8 (FC back)
        let sock = open_kernel_isotp_socket(&ifname_k, 0x7E0, 0x7E8)?;
        let mut buf = vec![0u8; 8192];
        let n = kernel_read(sock, &mut buf)?;
        unsafe { libc::close(sock) };
        buf.truncate(n);
        Ok(buf)
    });

    // Small delay so the kernel socket is bound before we start sending.
    thread::sleep(Duration::from_millis(50));

    let ifname_s = ifname.to_string();
    let tx_data = payload.to_vec();

    // Spawn our crate's sender.
    let tx_handle = thread::spawn(move || -> Result<(), String> {
        let channel =
            SocketCanChannel::open(&ifname_s).map_err(|e| format!("TX open error: {e}"))?;
        let config = IsoTpConfig {
            timeout: Duration::from_secs(5),
            ..IsoTpConfig::new(
                CanId::new_standard(0x7E0).expect("valid CAN ID"),
                CanId::new_standard(0x7E8).expect("valid CAN ID"),
            )
        };
        let mut isotp = IsoTpChannel::new(channel, config);
        isotp.send(&tx_data).map_err(|e| format!("TX error: {e}"))
    });

    let tx_result = tx_handle.join().expect("TX thread panicked");
    let rx_result = rx_handle.join().expect("RX thread panicked");

    if let Err(e) = &tx_result {
        eprintln!("  TX failed: {e}");
        return false;
    }

    match rx_result {
        Ok(received) => {
            if received == expected {
                println!("  PASS");
                true
            } else {
                eprintln!(
                    "  FAIL: expected {} bytes, got {} bytes",
                    expected.len(),
                    received.len()
                );
                false
            }
        }
        Err(e) => {
            eprintln!("  FAIL (RX): {e}");
            false
        }
    }
}

/// Test B: kernel socket sends, our IsoTpChannel receives.
fn test_kernel_send_our_recv(ifname: &str, payload: &[u8]) -> bool {
    let label = format!(
        "Test B ({} bytes): kernel sends, our crate receives",
        payload.len()
    );
    println!("\n{label}");

    let ifname_r = ifname.to_string();

    // Spawn our crate's receiver first.
    let rx_handle = thread::spawn(move || -> Result<Vec<u8>, String> {
        let channel =
            SocketCanChannel::open(&ifname_r).map_err(|e| format!("RX open error: {e}"))?;
        // Our receiver: tx_id=0x7E8 (FC), rx_id=0x7E0 (data from kernel)
        let config = IsoTpConfig {
            timeout: Duration::from_secs(5),
            ..IsoTpConfig::new(
                CanId::new_standard(0x7E8).expect("valid CAN ID"),
                CanId::new_standard(0x7E0).expect("valid CAN ID"),
            )
        };
        let mut isotp = IsoTpChannel::new(channel, config);
        isotp.receive().map_err(|e| format!("RX error: {e}"))
    });

    thread::sleep(Duration::from_millis(50));

    let ifname_k = ifname.to_string();
    let tx_data = payload.to_vec();

    // Spawn kernel sender.
    let tx_handle = thread::spawn(move || -> Result<(), String> {
        // Kernel socket: tx_id=0x7E0 (sends data), rx_id=0x7E8 (receives FC)
        let sock = open_kernel_isotp_socket(&ifname_k, 0x7E8, 0x7E0)?;
        kernel_write(sock, &tx_data)?;
        unsafe { libc::close(sock) };
        Ok(())
    });

    let tx_result = tx_handle.join().expect("TX thread panicked");
    let rx_result = rx_handle.join().expect("RX thread panicked");

    if let Err(e) = &tx_result {
        eprintln!("  TX failed: {e}");
        return false;
    }

    match rx_result {
        Ok(received) => {
            if received == payload {
                println!("  PASS");
                true
            } else {
                eprintln!(
                    "  FAIL: expected {} bytes, got {} bytes",
                    payload.len(),
                    received.len()
                );
                false
            }
        }
        Err(e) => {
            eprintln!("  FAIL (RX): {e}");
            false
        }
    }
}

// ---- Main -------------------------------------------------------------------

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ifname = env::args().nth(1).unwrap_or_else(|| "vcan0".into());

    println!("Kernel CAN_ISOTP interoperability test on '{ifname}'");
    println!("----------------------------------------------------");
    println!("Requires: Linux kernel >= 5.10, CONFIG_CAN_ISOTP=y, {ifname} up");

    let small_payload: Vec<u8> = (0..20).map(|i| (i * 3) as u8).collect();
    let large_payload: Vec<u8> = (0..4000u16).map(|i| (i & 0xFF) as u8).collect();

    let mut passed = 0u32;
    let mut failed = 0u32;

    // Test A: our crate sends 20 bytes, kernel receives
    if test_our_send_kernel_recv('A', &ifname, &small_payload) {
        passed += 1;
    } else {
        failed += 1;
    }

    // Test B: kernel sends 20 bytes, our crate receives
    if test_kernel_send_our_recv(&ifname, &small_payload) {
        passed += 1;
    } else {
        failed += 1;
    }

    // Test C: our crate sends 4000 bytes, kernel receives
    if test_our_send_kernel_recv('C', &ifname, &large_payload) {
        passed += 1;
    } else {
        failed += 1;
    }

    println!("\n----------------------------------------------------");
    println!("Results: {passed} passed, {failed} failed");

    if failed > 0 {
        std::process::exit(1);
    }

    Ok(())
}
