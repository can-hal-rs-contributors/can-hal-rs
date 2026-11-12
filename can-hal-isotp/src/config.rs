use std::time::Duration;

use can_hal::CanId;

/// ISO-TP addressing mode.
#[derive(Debug, Clone, Copy)]
pub enum AddressingMode {
    /// Normal addressing: only PCI bytes in the payload.
    Normal,
    /// Extended addressing: a target address byte precedes the PCI bytes.
    ///
    /// Per ISO 15765-2, the target address (N_TA) in transmitted frames identifies
    /// the remote ECU, while the target address in received frames identifies the
    /// local ECU. These are typically different values:
    ///
    /// - `tx_target_address`: written into byte 0 of every transmitted CAN frame
    /// - `rx_target_address`: expected in byte 0 of every received CAN frame
    Extended {
        tx_target_address: u8,
        rx_target_address: u8,
    },
}

/// Configuration for an ISO-TP channel.
#[derive(Debug, Clone)]
pub struct IsoTpConfig {
    /// CAN ID used for transmitted frames.
    pub tx_id: CanId,
    /// CAN ID expected for received frames.
    pub rx_id: CanId,
    /// Addressing mode (Normal or Extended).
    pub addressing: AddressingMode,
    /// Block Size advertised in Flow Control frames (0 = no limit).
    pub block_size: u8,
    /// Raw STmin byte this channel advertises in its Flow Control frames: the
    /// gap it asks the peer to leave between Consecutive Frames. `send` doesn't
    /// use it; it waits the STmin from the peer's FC instead.
    ///
    /// Encoding (ISO 15765-2):
    /// - `0x00`--`0x7F`: 0--127 milliseconds
    /// - `0xF1`--`0xF9`: 100--900 microseconds
    /// - All other values: reserved, and not to be advertised. A sender that
    ///   receives one waits 127 ms (`0x7F`), as `send` does.
    ///
    /// `send` waits out a peer's `0xF1`--`0xF9` as 1 ms; see
    /// [`st_min_duration`](Self::st_min_duration).
    pub st_min: u8,
    /// Timeout for each wait: for the first SF or FF in `receive`, for each FC
    /// in `send` (N_Bs, which an FC(Wait) does not restart), and for each CF in
    /// `receive` (N_Cr). A frame that is ignored doesn't restart a wait.
    pub timeout: Duration,
    /// Padding for transmitted frames.
    ///
    /// - Classic channels: `Some(b)` pads every frame to 8 bytes with `b`;
    ///   `None` sends only the bytes in use.
    /// - FD channels: a frame of 8 bytes or fewer is never padded; a longer one
    ///   is always padded up to the next valid CAN FD data length (12, 16, 20,
    ///   24, 32, 48 or 64 bytes) with `b`, or with 0x00 if `None`.
    pub padding: Option<u8>,
    /// Maximum number of consecutive FC(Wait) frames to accept before returning
    /// `IsoTpError::WaitLimitExceeded`. For example, a value of `3` permits up to
    /// 2 Wait frames and aborts on the 3rd. `0` means unlimited (rely solely on
    /// the overall `timeout`).
    pub max_fc_wait: u8,
    /// CAN ID used for functionally addressed (broadcast) requests.
    /// Functional requests MUST fit in a single frame - no multi-frame allowed.
    /// Typically 0x7DF (11-bit) for UDS. `None` means functional addressing is not used.
    pub functional_id: Option<CanId>,
}

impl IsoTpConfig {
    /// Create a new `IsoTpConfig` with sensible defaults.
    ///
    /// Defaults: Normal addressing, block_size=0, st_min=5 (5 ms),
    /// timeout=1s, padding=Some(0xCC), max_fc_wait=10.
    ///
    /// The default `st_min` of 5 ms provides reliable inter-frame pacing
    /// across different CAN adapters. Set to 0 only if both endpoints are
    /// known to handle back-to-back frames at full bus speed.
    #[must_use]
    pub const fn new(tx_id: CanId, rx_id: CanId) -> Self {
        Self {
            tx_id,
            rx_id,
            addressing: AddressingMode::Normal,
            block_size: 0,
            st_min: 5,
            timeout: Duration::from_secs(1),
            padding: Some(0xCC),
            max_fc_wait: 10,
            functional_id: None,
        }
    }

    /// Interpret the raw `st_min` byte as a `Duration`, the way `send` treats
    /// the STmin in a peer's FC: `0x00`--`0x7F` are milliseconds, `0xF1`--`0xF9`
    /// (100--900 µs) are rounded up to 1 ms, and reserved values give 127 ms.
    #[must_use]
    pub fn st_min_duration(&self) -> Duration {
        crate::frame::interpret_st_min(self.st_min)
    }

    /// Returns the number of overhead bytes for the current addressing mode.
    /// Normal = 0 extra bytes, Extended = 1 byte (target address).
    #[must_use]
    pub const fn overhead(&self) -> usize {
        match self.addressing {
            AddressingMode::Normal => 0,
            AddressingMode::Extended { .. } => 1,
        }
    }

    /// Returns the TX target address for Extended addressing, if configured.
    #[must_use]
    pub const fn tx_target_address(&self) -> Option<u8> {
        match self.addressing {
            AddressingMode::Normal => None,
            AddressingMode::Extended {
                tx_target_address, ..
            } => Some(tx_target_address),
        }
    }

    /// Returns the RX target address for Extended addressing, if configured.
    #[must_use]
    pub const fn rx_target_address(&self) -> Option<u8> {
        match self.addressing {
            AddressingMode::Normal => None,
            AddressingMode::Extended {
                rx_target_address, ..
            } => Some(rx_target_address),
        }
    }

    /// When a wait on `timeout` that starts now must end: `now + timeout`, or
    /// [`FAR_FUTURE`] from now if that overflows, so that a `timeout` of
    /// `Duration::MAX` waits indefinitely instead of panicking.
    #[cfg(feature = "async")]
    pub(crate) fn tokio_deadline(&self) -> tokio::time::Instant {
        let now = tokio::time::Instant::now();
        now.checked_add(self.timeout)
            .unwrap_or_else(|| now + FAR_FUTURE)
    }
}

/// How far ahead a deadline goes when `now + timeout` overflows: 30 years,
/// the cap `tokio::time::timeout` uses.
#[cfg(feature = "async")]
const FAR_FUTURE: Duration = Duration::from_secs(86_400 * 365 * 30);
