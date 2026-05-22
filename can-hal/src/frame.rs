use crate::id::CanId;

const CAN_MAX_LEN: usize = 8;

/// A classic CAN 2.0 frame (up to 8 data bytes).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    id: CanId,
    len: u8,
    data: [u8; 8],
}

impl CanFrame {
    /// Create a new classic CAN frame.
    ///
    /// Returns `None` if `data` is longer than 8 bytes.
    #[must_use]
    pub fn new(id: CanId, data: &[u8]) -> Option<Self> {
        if data.len() > CAN_MAX_LEN {
            return None;
        }
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        #[allow(clippy::cast_possible_truncation)] // validated above: data.len() <= 8
        Some(Self {
            id,
            len: data.len() as u8,
            data: buf,
        })
    }

    /// Returns the frame's CAN identifier.
    #[must_use]
    pub const fn id(&self) -> CanId {
        self.id
    }

    /// Returns the data length in bytes (0--8).
    ///
    /// For classic CAN 2.0, the data length and the on-wire DLC field are
    /// identical in the range 0--8.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns `true` if the frame carries zero data bytes.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the data payload.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

/// A CAN FD frame (up to 64 data bytes).
///
/// The data length is stored as a byte count (0--64), **not** the 4-bit
/// on-wire DLC code. For DLC values 9--15 the CAN FD specification maps
/// them to 12, 16, 20, 24, 32, 48, and 64 bytes respectively; this struct
/// stores the decoded byte count directly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFdFrame {
    id: CanId,
    len: u8,
    data: [u8; 64],
    brs: bool,
    esi: bool,
}

impl CanFdFrame {
    /// Create a new CAN FD frame.
    ///
    /// Returns `None` if `data.len()` is not a valid FD data length
    /// (0, 1, ..., 8, 12, 16, 20, 24, 32, 48, or 64).
    #[must_use]
    pub fn new(id: CanId, data: &[u8], brs: bool, esi: bool) -> Option<Self> {
        if !matches!(data.len(), 0..=8 | 12 | 16 | 20 | 24 | 32 | 48 | 64) {
            return None;
        }
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        #[allow(clippy::cast_possible_truncation)] // validated above: data.len() <= 64
        Some(Self {
            id,
            len: data.len() as u8,
            data: buf,
            brs,
            esi,
        })
    }

    /// Returns the frame's CAN identifier.
    #[must_use]
    pub const fn id(&self) -> CanId {
        self.id
    }

    /// Returns the data length in bytes (not the 4-bit DLC code).
    ///
    /// The returned value is always one of the valid CAN FD data lengths:
    /// 0, 1, ..., 8, 12, 16, 20, 24, 32, 48, or 64.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns `true` if the frame carries zero data bytes.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the data payload.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// Returns `true` if the Bit Rate Switch flag is set.
    #[must_use]
    pub const fn brs(&self) -> bool {
        self.brs
    }

    /// Returns `true` if the Error State Indicator flag is set.
    #[must_use]
    pub const fn esi(&self) -> bool {
        self.esi
    }
}

/// A frame of either type - used when receiving on an FD-capable bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    Can(CanFrame),
    Fd(CanFdFrame),
}

impl Frame {
    /// Returns the frame's CAN identifier regardless of frame type.
    #[must_use]
    pub const fn id(&self) -> CanId {
        match self {
            Self::Can(f) => f.id(),
            Self::Fd(f) => f.id(),
        }
    }

    /// Returns the data payload regardless of frame type.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        match self {
            Self::Can(f) => f.data(),
            Self::Fd(f) => f.data(),
        }
    }

    /// Returns the data length in bytes regardless of frame type.
    #[must_use]
    pub const fn len(&self) -> usize {
        match self {
            Self::Can(f) => f.len(),
            Self::Fd(f) => f.len(),
        }
    }

    /// Returns `true` if the frame carries zero data bytes.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        match self {
            Self::Can(f) => f.is_empty(),
            Self::Fd(f) => f.is_empty(),
        }
    }
}

/// A received frame paired with its timestamp.
///
/// The timestamp type `T` is chosen by the backend implementation.
/// On `std` platforms this is typically `std::time::Instant`; on `no_std`
/// targets it can be a hardware timer tick count or any `Clone` type.
#[derive(Debug, Clone)]
pub struct Timestamped<F, T> {
    frame: F,
    timestamp: T,
}

impl<F, T> Timestamped<F, T> {
    /// Create a new timestamped frame.
    #[must_use]
    pub const fn new(frame: F, timestamp: T) -> Self {
        Self { frame, timestamp }
    }

    /// Returns a reference to the inner frame.
    #[must_use]
    pub const fn frame(&self) -> &F {
        &self.frame
    }

    /// Returns a reference to the timestamp.
    #[must_use]
    pub const fn timestamp(&self) -> &T {
        &self.timestamp
    }

    /// Consumes self and returns the inner frame.
    #[must_use]
    pub fn into_frame(self) -> F {
        self.frame
    }
}
