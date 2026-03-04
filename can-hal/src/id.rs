/// Maximum value for an 11-bit standard CAN identifier.
const STANDARD_MAX: u16 = 0x7FF;

/// Maximum value for a 29-bit extended CAN identifier.
const EXTENDED_MAX: u32 = 0x1FFF_FFFF;

/// A CAN bus identifier, either standard (11-bit) or extended (29-bit).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanId {
    /// Standard 11-bit identifier (0x000–0x7FF).
    Standard(u16),
    /// Extended 29-bit identifier (0x00000000–0x1FFFFFFF).
    Extended(u32),
}

impl CanId {
    /// Create a standard (11-bit) CAN ID, returning `None` if the value exceeds 0x7FF.
    pub fn new_standard(id: u16) -> Option<Self> {
        if id <= STANDARD_MAX {
            Some(CanId::Standard(id))
        } else {
            None
        }
    }

    /// Create an extended (29-bit) CAN ID, returning `None` if the value exceeds 0x1FFFFFFF.
    pub fn new_extended(id: u32) -> Option<Self> {
        if id <= EXTENDED_MAX {
            Some(CanId::Extended(id))
        } else {
            None
        }
    }

    /// Returns the raw numeric identifier value.
    pub fn raw(&self) -> u32 {
        match *self {
            CanId::Standard(id) => id as u32,
            CanId::Extended(id) => id,
        }
    }

    /// Returns `true` if this is a standard (11-bit) identifier.
    pub fn is_standard(&self) -> bool {
        matches!(self, CanId::Standard(_))
    }

    /// Returns `true` if this is an extended (29-bit) identifier.
    pub fn is_extended(&self) -> bool {
        matches!(self, CanId::Extended(_))
    }
}
