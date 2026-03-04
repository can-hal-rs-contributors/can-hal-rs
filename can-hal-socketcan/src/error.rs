use std::fmt;
use std::io;

/// Errors from the SocketCAN backend.
#[derive(Debug)]
pub enum SocketCanError {
    /// An I/O error from the underlying socket.
    Io(io::Error),
    /// Failed to construct a frame (invalid ID, data length, etc.).
    InvalidFrame(String),
    /// The interface index or name is invalid.
    InvalidInterface(String),
}

impl fmt::Display for SocketCanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SocketCanError::Io(e) => write!(f, "SocketCAN I/O error: {e}"),
            SocketCanError::InvalidFrame(msg) => write!(f, "invalid frame: {msg}"),
            SocketCanError::InvalidInterface(msg) => write!(f, "invalid interface: {msg}"),
        }
    }
}

impl std::error::Error for SocketCanError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            SocketCanError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for SocketCanError {
    fn from(e: io::Error) -> Self {
        SocketCanError::Io(e)
    }
}
