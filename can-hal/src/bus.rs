use crate::error::CanError;

/// The error state of a CAN bus controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusState {
    /// Normal operation; both error counters are at most 127. Some backends
    /// (Kvaser, and PCAN where PCAN-Basic reports the warning state as
    /// BUSHEAVY) already return `ErrorPassive` once a counter reaches the
    /// warning limit (96).
    ErrorActive,
    /// An error counter has reached 128 (or the warning limit on the
    /// backends above). The node still communicates, using passive error
    /// flags.
    ErrorPassive,
    /// The transmit error counter exceeded 255. No frames are sent or
    /// received until recovery.
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
/// Not all backends support this - it is an optional trait.
pub trait BusStatus {
    type Error: CanError;

    /// Returns the current bus state of the controller.
    fn bus_state(&self) -> Result<BusState, Self::Error>;

    /// Returns the current transmit and receive error counters.
    fn error_counters(&self) -> Result<ErrorCounters, Self::Error>;
}
