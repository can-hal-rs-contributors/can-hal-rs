use crate::error::CanError;

/// The error state of a CAN bus controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusState {
    /// Normal operation. Error counters are below 128.
    ErrorActive,
    /// Error counters have reached the warning threshold (128–255).
    /// The controller can still communicate but may be experiencing issues.
    ErrorPassive,
    /// The controller has gone off-bus due to excessive errors (counter > 255).
    /// No frames can be sent or received until recovery.
    BusOff,
}

/// Transmit and receive error counters from the CAN controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ErrorCounters {
    pub transmit: u8,
    pub receive: u8,
}

/// Query the health and error state of a CAN bus controller.
///
/// Not all backends support this — it is an optional trait.
pub trait BusStatus {
    type Error: CanError;

    /// Returns the current bus state of the controller.
    fn bus_state(&self) -> Result<BusState, Self::Error>;

    /// Returns the current transmit and receive error counters.
    fn error_counters(&self) -> Result<ErrorCounters, Self::Error>;
}
