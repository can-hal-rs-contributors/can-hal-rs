use std::fmt;

use crate::ffi;

/// A raw CANlib status code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KvaserStatus(pub i32);

impl fmt::Display for KvaserStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let desc = match self.0 {
            -1 => "error in parameter",
            -2 => "no messages available",
            -3 => "specified hardware not found",
            -4 => "out of memory",
            -5 => "no channels available",
            -6 => "interrupted by signal",
            -7 => "timeout",
            -8 => "not properly initialized",
            -9 => "out of handles",
            -10 => "handle is invalid",
            -11 => "error in ini-file",
            -12 => "CAN driver type not supported",
            -13 => "transmit buffer overflow",
            -15 => "hardware error detected",
            -16 => "cannot load driver DLL",
            -17 => "wrong DLL version",
            -18 => "error initializing DLL",
            -19 => "function not supported",
            _ => "unknown error",
        };
        write!(f, "CANlib error {} ({desc})", self.0)
    }
}

/// Errors returned by the KVASER CANlib backend.
#[derive(Debug)]
#[non_exhaustive]
pub enum KvaserError {
    /// The CANlib shared library could not be loaded.
    LibraryLoad(libloading::Error),
    /// A CANlib API call returned a non-OK status code.
    Canlib(KvaserStatus),
    /// A frame could not be constructed from the received data.
    InvalidFrame(String),
    /// A bitrate (in Hz) does not evenly divide the 80 MHz CANlib clock,
    /// so no exact prescaler exists.
    UnsupportedBitrate(u32),
    /// No timing parameters satisfy the requested bitrate + sample point
    /// at the 80 MHz CANlib clock within the segment-length constraints.
    UnsupportedTiming(String),
    /// A platform-specific error occurred.
    Platform(String),
}

impl fmt::Display for KvaserError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::LibraryLoad(e) => write!(f, "failed to load CANlib: {e}"),
            // KvaserStatus's Display already includes "CANlib error N (desc)".
            Self::Canlib(s) => write!(f, "{s}"),
            Self::InvalidFrame(msg) => write!(f, "invalid frame: {msg}"),
            Self::UnsupportedBitrate(hz) => {
                write!(f, "unsupported bitrate {hz} bps")
            }
            Self::UnsupportedTiming(msg) => write!(f, "unsupported timing: {msg}"),
            Self::Platform(msg) => write!(f, "platform error: {msg}"),
        }
    }
}

impl std::error::Error for KvaserError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::LibraryLoad(e) => Some(e),
            _ => None,
        }
    }
}

impl From<libloading::Error> for KvaserError {
    fn from(e: libloading::Error) -> Self {
        Self::LibraryLoad(e)
    }
}

/// Convert a raw CANlib return value to a `Result`.
///
/// Per the CANlib documentation, any return value less than zero indicates
/// failure. Positive non-zero values may indicate success with informational
/// codes on some API calls.
pub const fn check_status(status: i32) -> Result<(), KvaserError> {
    if status >= ffi::CAN_OK {
        Ok(())
    } else {
        Err(KvaserError::Canlib(KvaserStatus(status)))
    }
}
