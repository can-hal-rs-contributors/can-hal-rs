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
    /// A wait timed out: no SF or FF arrived in time in `receive`, or N_Bs (an
    /// FC in `send`) or N_Cr (a CF in `receive`) expired.
    Timeout,
    /// The receiver sent a Flow Control frame with the Overflow flag.
    BufferOverflow,
    /// A frame isn't a valid ISO-TP PDU: [`IsoTpFrame::parse`] rejected it,
    /// or `send` got a malformed Flow Control frame. (`receive` ignores
    /// invalid frames.) `send_functional` also returns this when
    /// [`IsoTpConfig::functional_id`] is `None`.
    ///
    /// [`IsoTpFrame::parse`]: crate::IsoTpFrame::parse
    /// [`IsoTpConfig::functional_id`]: crate::IsoTpConfig::functional_id
    InvalidFrame,
    /// Consecutive Frame sequence number mismatch.
    SequenceError { expected: u8, got: u8 },
    /// The payload is too large to send: over `u32::MAX` bytes for `send`
    /// (classic and FD both reach that with the 32-bit FF_DL form), or over
    /// the single-frame limit for `send_functional` (7 bytes, 6 with extended
    /// addressing).
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
