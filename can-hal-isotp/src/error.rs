use std::fmt;

/// Errors that can occur during ISO-TP communication.
///
/// The `Send + Sync + 'static` bounds on `E` match the [`CanError`](can_hal::error::CanError)
/// trait bound, ensuring `IsoTpError` is safe to use across thread boundaries
/// and with error-handling crates like `anyhow`.
#[derive(Debug)]
pub enum IsoTpError<E: Send + Sync + 'static> {
    /// Error from the underlying CAN channel.
    CanError(E),
    /// A timeout expired (N_Bs or N_Cr).
    Timeout,
    /// The receiver sent a Flow Control frame with the Overflow flag.
    BufferOverflow,
    /// A received frame could not be parsed as a valid ISO-TP PDU.
    InvalidFrame,
    /// Consecutive Frame sequence number mismatch.
    SequenceError { expected: u8, got: u8 },
    /// Payload exceeds the maximum allowed size. Both classic and FD use a
    /// 32-bit length field via the long-FF escape, so the per-message ceiling
    /// is `u32::MAX` bytes.
    PayloadTooLarge,
    /// Too many consecutive FC(Wait) frames received (exceeds `max_fc_wait`).
    WaitLimitExceeded,
}

impl<E: fmt::Display + Send + Sync + 'static> fmt::Display for IsoTpError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::CanError(e) => write!(f, "CAN error: {e}"),
            Self::Timeout => write!(f, "ISO-TP timeout"),
            Self::BufferOverflow => write!(f, "ISO-TP receiver buffer overflow"),
            Self::InvalidFrame => write!(f, "invalid ISO-TP frame"),
            Self::SequenceError { expected, got } => {
                write!(f, "ISO-TP sequence error: expected {expected}, got {got}")
            }
            Self::PayloadTooLarge => write!(f, "ISO-TP payload too large"),
            Self::WaitLimitExceeded => write!(f, "ISO-TP FC Wait limit exceeded"),
        }
    }
}

impl<E: std::error::Error + Send + Sync + 'static> std::error::Error for IsoTpError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::CanError(e) => Some(e),
            _ => None,
        }
    }
}
