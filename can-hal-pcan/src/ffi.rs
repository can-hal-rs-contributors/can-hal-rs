//! Raw PCAN-Basic FFI type definitions and constants.
//!
//! These mirror the C API types from PCANBasic.h. Function pointers are loaded
//! at runtime via `libloading` in the `library` module.

#![allow(dead_code)]

use std::os::raw::c_char;

// ---------------------------------------------------------------------------
// Handle constants
// ---------------------------------------------------------------------------

/// First USB channel handle.
pub const PCAN_USBBUS1: u16 = 0x51;

/// First PCI channel handle.
pub const PCAN_PCIBUS1: u16 = 0x41;

/// First LAN channel handle.
pub const PCAN_LANBUS1: u16 = 0x801;

/// Map a (bus_type, 0-based index) pair to a PCAN handle value.
///
/// `bus_type`: 0 = USB, 1 = PCI, 2 = LAN.
/// Returns `None` if the index is out of range (must be 0..=15).
pub fn pcan_handle(bus_type: u8, index: u16) -> Option<u16> {
    if index > 15 {
        return None;
    }
    match bus_type {
        0 => Some(PCAN_USBBUS1 + index),
        1 => Some(PCAN_PCIBUS1 + index),
        2 => Some(PCAN_LANBUS1 + index),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Bitrate constants (TPCANBaudrate)
// ---------------------------------------------------------------------------

pub const PCAN_BAUD_1M: u16 = 0x0014;
pub const PCAN_BAUD_800K: u16 = 0x0016;
pub const PCAN_BAUD_500K: u16 = 0x001C;
pub const PCAN_BAUD_250K: u16 = 0x011C;
pub const PCAN_BAUD_125K: u16 = 0x031C;
pub const PCAN_BAUD_100K: u16 = 0x432F;
pub const PCAN_BAUD_50K: u16 = 0x472F;
pub const PCAN_BAUD_20K: u16 = 0x532F;
pub const PCAN_BAUD_10K: u16 = 0x672F;
pub const PCAN_BAUD_5K: u16 = 0x7F7F;

/// Map a standard bitrate in Hz to a `PCAN_BAUD_*` constant.
/// Returns `None` for unsupported bitrates.
pub fn bitrate_to_pcan(hz: u32) -> Option<u16> {
    match hz {
        1_000_000 => Some(PCAN_BAUD_1M),
        800_000 => Some(PCAN_BAUD_800K),
        500_000 => Some(PCAN_BAUD_500K),
        250_000 => Some(PCAN_BAUD_250K),
        125_000 => Some(PCAN_BAUD_125K),
        100_000 => Some(PCAN_BAUD_100K),
        50_000 => Some(PCAN_BAUD_50K),
        20_000 => Some(PCAN_BAUD_20K),
        10_000 => Some(PCAN_BAUD_10K),
        5_000 => Some(PCAN_BAUD_5K),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Message type flags
// ---------------------------------------------------------------------------

pub const PCAN_MESSAGE_STANDARD: u8 = 0x00;
pub const PCAN_MESSAGE_RTR: u8 = 0x01;
pub const PCAN_MESSAGE_EXTENDED: u8 = 0x02;
pub const PCAN_MESSAGE_FD: u8 = 0x04;
pub const PCAN_MESSAGE_BRS: u8 = 0x08;
pub const PCAN_MESSAGE_ESI: u8 = 0x10;
pub const PCAN_MESSAGE_STATUS: u8 = 0x80;

// ---------------------------------------------------------------------------
// Status / error codes (TPCANStatus)
// ---------------------------------------------------------------------------

pub const PCAN_ERROR_OK: u32 = 0x00000;
pub const PCAN_ERROR_XMTFULL: u32 = 0x00001;
pub const PCAN_ERROR_OVERRUN: u32 = 0x00002;
pub const PCAN_ERROR_BUSLIGHT: u32 = 0x00004;
pub const PCAN_ERROR_BUSHEAVY: u32 = 0x00008;
pub const PCAN_ERROR_BUSOFF: u32 = 0x00010;
pub const PCAN_ERROR_QRCVEMPTY: u32 = 0x00020;
pub const PCAN_ERROR_QOVERRUN: u32 = 0x00040;
pub const PCAN_ERROR_QXMTFULL: u32 = 0x00080;
pub const PCAN_ERROR_REGTEST: u32 = 0x00100;
pub const PCAN_ERROR_NODRIVER: u32 = 0x00200;
pub const PCAN_ERROR_HWINUSE: u32 = 0x00400;
pub const PCAN_ERROR_NETINUSE: u32 = 0x00800;
pub const PCAN_ERROR_ILLHW: u32 = 0x01400;
pub const PCAN_ERROR_ILLNET: u32 = 0x01800;
pub const PCAN_ERROR_ILLCLIENT: u32 = 0x01C00;
pub const PCAN_ERROR_RESOURCE: u32 = 0x02000;
pub const PCAN_ERROR_ILLPARAMTYPE: u32 = 0x04000;
pub const PCAN_ERROR_ILLPARAMVAL: u32 = 0x08000;
pub const PCAN_ERROR_UNKNOWN: u32 = 0x10000;
pub const PCAN_ERROR_ILLDATA: u32 = 0x20000;
pub const PCAN_ERROR_BUSPASSIVE: u32 = 0x40000;
pub const PCAN_ERROR_CAUTION: u32 = 0x2000000;
pub const PCAN_ERROR_INITIALIZE: u32 = 0x4000000;
pub const PCAN_ERROR_ILLOPERATION: u32 = 0x8000000;

// ---------------------------------------------------------------------------
// Filter modes
// ---------------------------------------------------------------------------

pub const PCAN_MODE_STANDARD: u8 = 0x01;
pub const PCAN_MODE_EXTENDED: u8 = 0x02;

// ---------------------------------------------------------------------------
// Parameter IDs (for CAN_GetValue / CAN_SetValue)
// ---------------------------------------------------------------------------

pub const PCAN_RECEIVE_EVENT: u8 = 0x03;
pub const PCAN_RECEIVE_STATUS: u8 = 0x04;
pub const PCAN_BUSOFF_AUTORESET: u8 = 0x07;
pub const PCAN_LISTEN_ONLY: u8 = 0x08;
pub const PCAN_CONTROLLER_NUMBER: u8 = 0x1A;
pub const PCAN_CHANNEL_CONDITION: u8 = 0x09;

// ---------------------------------------------------------------------------
// Message structs
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TPCANMsg {
    pub id: u32,
    pub msg_type: u8,
    pub len: u8,
    pub data: [u8; 8],
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TPCANMsgFD {
    pub id: u32,
    pub msg_type: u8,
    pub dlc: u8,
    pub data: [u8; 64],
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TPCANTimestamp {
    pub millis: u32,
    pub millis_overflow: u16,
    pub micros: u16,
}

// ---------------------------------------------------------------------------
// Function pointer type aliases
// ---------------------------------------------------------------------------

pub type FnInitialize = unsafe extern "C" fn(
    handle: u16,
    bitrate: u16,
    hw_type: u8,
    io_port: u32,
    interrupt: u16,
) -> u32;

pub type FnInitializeFD = unsafe extern "C" fn(handle: u16, bitrate_fd: *const c_char) -> u32;

pub type FnUninitialize = unsafe extern "C" fn(handle: u16) -> u32;

pub type FnRead =
    unsafe extern "C" fn(handle: u16, msg: *mut TPCANMsg, timestamp: *mut TPCANTimestamp) -> u32;

pub type FnReadFD =
    unsafe extern "C" fn(handle: u16, msg: *mut TPCANMsgFD, timestamp: *mut u64) -> u32;

pub type FnWrite = unsafe extern "C" fn(handle: u16, msg: *mut TPCANMsg) -> u32;

pub type FnWriteFD = unsafe extern "C" fn(handle: u16, msg: *mut TPCANMsgFD) -> u32;

pub type FnFilterMessages =
    unsafe extern "C" fn(handle: u16, from_id: u32, to_id: u32, mode: u8) -> u32;

pub type FnGetStatus = unsafe extern "C" fn(handle: u16) -> u32;

pub type FnGetValue =
    unsafe extern "C" fn(handle: u16, param: u8, buf: *mut std::ffi::c_void, buf_len: u32) -> u32;

pub type FnSetValue =
    unsafe extern "C" fn(handle: u16, param: u8, buf: *mut std::ffi::c_void, buf_len: u32) -> u32;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn usb_handle_mapping() {
        assert_eq!(pcan_handle(0, 0), Some(PCAN_USBBUS1));
        assert_eq!(pcan_handle(0, 1), Some(PCAN_USBBUS1 + 1));
        assert_eq!(pcan_handle(0, 15), Some(PCAN_USBBUS1 + 15));
        assert_eq!(pcan_handle(0, 16), None);
    }

    #[test]
    fn pci_handle_mapping() {
        assert_eq!(pcan_handle(1, 0), Some(PCAN_PCIBUS1));
        assert_eq!(pcan_handle(1, 15), Some(PCAN_PCIBUS1 + 15));
    }

    #[test]
    fn lan_handle_mapping() {
        assert_eq!(pcan_handle(2, 0), Some(PCAN_LANBUS1));
        assert_eq!(pcan_handle(2, 15), Some(PCAN_LANBUS1 + 15));
    }

    #[test]
    fn invalid_bus_type() {
        assert_eq!(pcan_handle(3, 0), None);
    }

    #[test]
    fn bitrate_mapping() {
        assert_eq!(bitrate_to_pcan(500_000), Some(PCAN_BAUD_500K));
        assert_eq!(bitrate_to_pcan(1_000_000), Some(PCAN_BAUD_1M));
        assert_eq!(bitrate_to_pcan(250_000), Some(PCAN_BAUD_250K));
        assert_eq!(bitrate_to_pcan(123_456), None);
    }
}
