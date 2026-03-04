use std::fmt;

use crate::ffi;

/// Status code returned by the PCAN-Basic API.
///
/// PCAN status codes are bitmask flags that may be ORed together.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcanStatus(pub u32);

impl PcanStatus {
    pub const OK: Self = Self(ffi::PCAN_ERROR_OK);
    pub const XMTFULL: Self = Self(ffi::PCAN_ERROR_XMTFULL);
    pub const OVERRUN: Self = Self(ffi::PCAN_ERROR_OVERRUN);
    pub const BUSLIGHT: Self = Self(ffi::PCAN_ERROR_BUSLIGHT);
    pub const BUSHEAVY: Self = Self(ffi::PCAN_ERROR_BUSHEAVY);
    pub const BUSOFF: Self = Self(ffi::PCAN_ERROR_BUSOFF);
    pub const QRCVEMPTY: Self = Self(ffi::PCAN_ERROR_QRCVEMPTY);
    pub const QOVERRUN: Self = Self(ffi::PCAN_ERROR_QOVERRUN);
    pub const QXMTFULL: Self = Self(ffi::PCAN_ERROR_QXMTFULL);
    pub const BUSPASSIVE: Self = Self(ffi::PCAN_ERROR_BUSPASSIVE);
    pub const INITIALIZE: Self = Self(ffi::PCAN_ERROR_INITIALIZE);
    pub const ILLOPERATION: Self = Self(ffi::PCAN_ERROR_ILLOPERATION);
    pub const NODRIVER: Self = Self(ffi::PCAN_ERROR_NODRIVER);
}

impl fmt::Display for PcanStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let desc = match self.0 {
            ffi::PCAN_ERROR_OK => "no error",
            ffi::PCAN_ERROR_XMTFULL => "transmit buffer full",
            ffi::PCAN_ERROR_OVERRUN => "receive buffer overrun",
            ffi::PCAN_ERROR_BUSLIGHT => "bus light error",
            ffi::PCAN_ERROR_BUSHEAVY => "bus heavy error",
            ffi::PCAN_ERROR_BUSOFF => "bus off",
            ffi::PCAN_ERROR_BUSPASSIVE => "bus passive",
            ffi::PCAN_ERROR_QRCVEMPTY => "receive queue empty",
            ffi::PCAN_ERROR_QOVERRUN => "receive queue overrun",
            ffi::PCAN_ERROR_QXMTFULL => "transmit queue full",
            ffi::PCAN_ERROR_INITIALIZE => "channel not initialized",
            ffi::PCAN_ERROR_ILLOPERATION => "illegal operation",
            ffi::PCAN_ERROR_NODRIVER => "driver not found",
            ffi::PCAN_ERROR_HWINUSE => "hardware in use",
            ffi::PCAN_ERROR_ILLPARAMTYPE => "invalid parameter type",
            ffi::PCAN_ERROR_ILLPARAMVAL => "invalid parameter value",
            _ => "",
        };
        if desc.is_empty() {
            write!(f, "PCAN error 0x{:05X}", self.0)
        } else {
            write!(f, "PCAN error 0x{:05X}: {desc}", self.0)
        }
    }
}

/// Errors from the PCAN-Basic backend.
#[derive(Debug)]
pub enum PcanError {
    /// The PCAN-Basic library could not be loaded at runtime.
    LibraryLoad(libloading::Error),
    /// A PCAN-Basic API call returned a non-OK status.
    Pcan(PcanStatus),
    /// Failed to construct a frame (invalid ID, data length, etc.).
    InvalidFrame(String),
    /// The requested channel index does not map to a known PCAN handle.
    InvalidChannel(u32),
    /// The requested bitrate is not a standard PCAN-Basic bitrate.
    UnsupportedBitrate(u32),
    /// A platform-specific error (e.g., event creation failed).
    Platform(String),
}

impl fmt::Display for PcanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PcanError::LibraryLoad(e) => write!(f, "failed to load PCAN-Basic library: {e}"),
            PcanError::Pcan(status) => write!(f, "{status}"),
            PcanError::InvalidFrame(msg) => write!(f, "invalid frame: {msg}"),
            PcanError::InvalidChannel(idx) => write!(f, "invalid PCAN channel index: {idx}"),
            PcanError::UnsupportedBitrate(br) => write!(f, "unsupported bitrate: {br} bps"),
            PcanError::Platform(msg) => write!(f, "platform error: {msg}"),
        }
    }
}

impl std::error::Error for PcanError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            PcanError::LibraryLoad(e) => Some(e),
            _ => None,
        }
    }
}

impl From<libloading::Error> for PcanError {
    fn from(e: libloading::Error) -> Self {
        PcanError::LibraryLoad(e)
    }
}

/// Check a PCAN status code and convert non-OK to `PcanError::Pcan`.
pub(crate) fn check_status(status: u32) -> Result<(), PcanError> {
    if status == ffi::PCAN_ERROR_OK {
        Ok(())
    } else {
        Err(PcanError::Pcan(PcanStatus(status)))
    }
}
