use std::time::Instant;

use crate::id::CanId;

const CAN_MAX_DLC: u8 = 8;
const CAN_FD_MAX_DLC: u8 = 64;

/// Valid CAN FD data lengths (bytes).
const FD_DLC_VALUES: &[u8] = &[0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

/// A classic CAN 2.0 frame (up to 8 data bytes).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    id: CanId,
    dlc: u8,
    data: [u8; 8],
}

impl CanFrame {
    /// Create a new classic CAN frame.
    ///
    /// Returns `None` if `data` is longer than 8 bytes.
    pub fn new(id: CanId, data: &[u8]) -> Option<Self> {
        if data.len() > CAN_MAX_DLC as usize {
            return None;
        }
        let dlc = data.len() as u8;
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        Some(CanFrame { id, dlc, data: buf })
    }

    /// Returns the frame's CAN identifier.
    pub fn id(&self) -> CanId {
        self.id
    }

    /// Returns the data length code.
    pub fn dlc(&self) -> u8 {
        self.dlc
    }

    /// Returns the data payload (slice of length `dlc`).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

/// A CAN FD frame (up to 64 data bytes).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFdFrame {
    id: CanId,
    dlc: u8,
    data: [u8; 64],
    brs: bool,
    esi: bool,
}

impl CanFdFrame {
    /// Create a new CAN FD frame.
    ///
    /// Returns `None` if `data.len()` is not a valid FD DLC value
    /// (0, 1, ..., 8, 12, 16, 20, 24, 32, 48, or 64).
    pub fn new(id: CanId, data: &[u8], brs: bool, esi: bool) -> Option<Self> {
        let len = data.len() as u8;
        if len > CAN_FD_MAX_DLC || !FD_DLC_VALUES.contains(&len) {
            return None;
        }
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        Some(CanFdFrame {
            id,
            dlc: len,
            data: buf,
            brs,
            esi,
        })
    }

    /// Returns the frame's CAN identifier.
    pub fn id(&self) -> CanId {
        self.id
    }

    /// Returns the data length code.
    pub fn dlc(&self) -> u8 {
        self.dlc
    }

    /// Returns the data payload (slice of length `dlc`).
    pub fn data(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }

    /// Returns `true` if the Bit Rate Switch flag is set.
    pub fn brs(&self) -> bool {
        self.brs
    }

    /// Returns `true` if the Error State Indicator flag is set.
    pub fn esi(&self) -> bool {
        self.esi
    }
}

/// A frame of either type — used when receiving on an FD-capable bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    Can(CanFrame),
    Fd(CanFdFrame),
}

impl Frame {
    /// Returns the frame's CAN identifier regardless of frame type.
    pub fn id(&self) -> CanId {
        match self {
            Frame::Can(f) => f.id(),
            Frame::Fd(f) => f.id(),
        }
    }

    /// Returns the data payload regardless of frame type.
    pub fn data(&self) -> &[u8] {
        match self {
            Frame::Can(f) => f.data(),
            Frame::Fd(f) => f.data(),
        }
    }

    /// Returns the data length code regardless of frame type.
    pub fn dlc(&self) -> u8 {
        match self {
            Frame::Can(f) => f.dlc(),
            Frame::Fd(f) => f.dlc(),
        }
    }
}

/// A received frame paired with its timestamp.
#[derive(Debug, Clone)]
pub struct Timestamped<F> {
    frame: F,
    timestamp: Instant,
}

impl<F> Timestamped<F> {
    /// Create a new timestamped frame.
    pub fn new(frame: F, timestamp: Instant) -> Self {
        Timestamped { frame, timestamp }
    }

    /// Returns a reference to the inner frame.
    pub fn frame(&self) -> &F {
        &self.frame
    }

    /// Returns the timestamp of when the frame was received.
    pub fn timestamp(&self) -> Instant {
        self.timestamp
    }

    /// Consumes self and returns the inner frame.
    pub fn into_frame(self) -> F {
        self.frame
    }
}
