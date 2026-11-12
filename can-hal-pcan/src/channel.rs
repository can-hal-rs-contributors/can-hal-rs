//! PCAN channel implementation.
//!
//! [`PcanChannel`] wraps a PCAN-Basic channel handle and implements the
//! `can-hal` traits for CAN communication. The channel is parameterized on
//! a type-state marker - [`crate::mode::Classic`] or [`crate::mode::Fd`] -
//! and only the trait impls valid for that mode are available, so calling
//! `transmit_fd` on a classic channel is a compile error rather than a
//! runtime `InvalidFrame`.

use std::marker::PhantomData;
use std::sync::Arc;
use std::time::{Duration, Instant};

use can_hal::bus::{BusState, BusStatus, ErrorCounters};
use can_hal::channel::{Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal::filter::{Filter, Filterable};
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};
use can_hal::id::CanId;

use crate::convert;
use crate::error::{check_status, PcanError, PcanStatus};
use crate::event::ReceiveEvent;
use crate::ffi;
use crate::library::PcanLibrary;
use crate::mode::{Classic, Fd};

/// A CAN channel backed by a PCAN-Basic hardware interface.
///
/// Parameterized on a type-state marker:
/// - [`PcanChannel<Classic>`] implements [`Transmit`] and [`Receive`].
/// - [`PcanChannel<Fd>`] implements [`TransmitFd`] and [`ReceiveFd`].
///
/// Both modes implement [`Filterable`] and [`BusStatus`].
///
/// Created via [`PcanDriver`](crate::PcanDriver) and
/// [`PcanChannelBuilder`](crate::PcanChannelBuilder).
pub struct PcanChannel<Mode> {
    lib: Arc<PcanLibrary>,
    handle: u16,
    event: ReceiveEvent,
    /// The ID ranges `set_filters` opened, or `None` while the channel
    /// accepts every frame.
    filter: Option<IdRanges>,
    _mode: PhantomData<Mode>,
}

/// The ID range a filter set opened for each frame type, `None` for a type
/// with no filter.
#[derive(Debug, Clone, Copy)]
struct IdRanges {
    standard: Option<(u32, u32)>,
    extended: Option<(u32, u32)>,
}

impl IdRanges {
    fn accepts(&self, id: CanId) -> bool {
        let (range, raw) = match id {
            CanId::Standard(v) => (self.standard, u32::from(v)),
            CanId::Extended(v) => (self.extended, v),
        };
        range.is_some_and(|(from, to)| (from..=to).contains(&raw))
    }
}

impl<Mode> PcanChannel<Mode> {
    /// Infallible constructor. Called by the builder after both
    /// `CAN_Initialize{,FD}` and `ReceiveEvent::new` have succeeded.
    pub(crate) const fn new(lib: Arc<PcanLibrary>, handle: u16, event: ReceiveEvent) -> Self {
        Self {
            lib,
            handle,
            event,
            filter: None,
            _mode: PhantomData,
        }
    }

    /// Whether the filter set accepts a frame with this ID.
    fn accepts(&self, id: CanId) -> bool {
        self.filter.map_or(true, |ranges| ranges.accepts(id))
    }
}

impl PcanChannel<Classic> {
    /// Try to read a classic CAN frame from the receive queue. Returns
    /// `Ok(None)` if the queue is empty. Status messages, error frames, and
    /// frames the filter set rejects, are skipped.
    fn read_classic(&self) -> Result<Option<CanFrame>, PcanError> {
        loop {
            let mut msg = ffi::TPCANMsg {
                id: 0,
                msg_type: 0,
                len: 0,
                data: [0; 8],
            };
            let mut ts = ffi::TPCANTimestamp {
                millis: 0,
                millis_overflow: 0,
                micros: 0,
            };
            // SAFETY: read() was loaded from PCANBasic and self.handle is valid from a successful CAN_Initialize.
            // msg and ts point to valid stack-allocated TPCANMsg and TPCANTimestamp.
            let status = unsafe { (self.lib.read)(self.handle, &mut msg, &mut ts) };
            if status == ffi::PCAN_ERROR_QRCVEMPTY {
                return Ok(None);
            }
            check_status(status)?;
            if let Some(frame) = convert::from_pcan_msg(&msg)?.filter(|f| self.accepts(f.id())) {
                return Ok(Some(frame));
            }
        }
    }
}

impl PcanChannel<Fd> {
    /// Try to read any frame (classic or FD) via `CAN_ReadFD`. Returns
    /// `Ok(None)` if the queue is empty. Status messages, error frames, and
    /// frames the filter set rejects, are skipped.
    fn read_fd(&self) -> Result<Option<Frame>, PcanError> {
        loop {
            let mut msg = ffi::TPCANMsgFD {
                id: 0,
                msg_type: 0,
                dlc: 0,
                data: [0; 64],
            };
            let mut ts: u64 = 0;
            // SAFETY: read_fd() was loaded from PCANBasic and self.handle is valid from a successful CAN_InitializeFD.
            // msg and ts point to valid stack-allocated TPCANMsgFD and u64.
            let status = unsafe { (self.lib.read_fd)(self.handle, &mut msg, &mut ts) };
            if status == ffi::PCAN_ERROR_QRCVEMPTY {
                return Ok(None);
            }
            check_status(status)?;
            if let Some(frame) = convert::from_pcan_msg_fd(&msg)?.filter(|f| self.accepts(f.id())) {
                return Ok(Some(frame));
            }
        }
    }
}

impl<Mode> Drop for PcanChannel<Mode> {
    fn drop(&mut self) {
        // SAFETY: uninitialize() was loaded from PCANBasic and self.handle is valid.
        // Errors during cleanup are deliberately ignored.
        #[allow(clippy::let_underscore_must_use)]
        unsafe {
            let _ = (self.lib.uninitialize)(self.handle);
        }
    }
}

// ---------------------------------------------------------------------------
// Classic-mode trait impls
// ---------------------------------------------------------------------------

impl Transmit for PcanChannel<Classic> {
    type Error = PcanError;

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        let mut msg = convert::to_pcan_msg(frame);
        // SAFETY: write() was loaded from PCANBasic and self.handle is valid.
        // msg points to a valid stack-allocated TPCANMsg.
        let status = unsafe { (self.lib.write)(self.handle, &mut msg) };
        check_status(status)
    }
}

impl Receive for PcanChannel<Classic> {
    type Error = PcanError;
    type Timestamp = Instant;

    fn receive(&mut self) -> Result<Timestamped<CanFrame, Instant>, Self::Error> {
        loop {
            if let Some(frame) = self.read_classic()? {
                return Ok(Timestamped::new(frame, Instant::now()));
            }
            self.event.wait(None)?;
        }
    }

    fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame, Instant>>, Self::Error> {
        Ok(self
            .read_classic()?
            .map(|frame| Timestamped::new(frame, Instant::now())))
    }

    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame, Instant>>, Self::Error> {
        // Cap poll interval to avoid missing frames when the event fd is
        // edge-triggered: the signal from an earlier frame (e.g. a TX echo)
        // can mask the arrival of a later frame.
        const MAX_POLL: Duration = Duration::from_millis(50);

        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.read_classic()? {
                return Ok(Some(Timestamped::new(frame, Instant::now())));
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            let wait_dur = (deadline - now).min(MAX_POLL);
            let _ = self.event.wait(Some(wait_dur))?;
        }
    }
}

// ---------------------------------------------------------------------------
// FD-mode trait impls
// ---------------------------------------------------------------------------

impl TransmitFd for PcanChannel<Fd> {
    type Error = PcanError;

    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), Self::Error> {
        let mut msg = convert::to_pcan_msg_fd(frame);
        // SAFETY: write_fd() was loaded from PCANBasic and self.handle is valid.
        // msg points to a valid stack-allocated TPCANMsgFD.
        let status = unsafe { (self.lib.write_fd)(self.handle, &mut msg) };
        check_status(status)
    }
}

impl ReceiveFd for PcanChannel<Fd> {
    type Error = PcanError;
    type Timestamp = Instant;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame, Instant>, Self::Error> {
        loop {
            if let Some(frame) = self.read_fd()? {
                return Ok(Timestamped::new(frame, Instant::now()));
            }
            self.event.wait(None)?;
        }
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame, Instant>>, Self::Error> {
        Ok(self
            .read_fd()?
            .map(|frame| Timestamped::new(frame, Instant::now())))
    }

    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame, Instant>>, Self::Error> {
        // Cap poll interval to avoid missing frames when the event fd is
        // edge-triggered: the signal from an earlier frame (e.g. a TX echo)
        // can mask the arrival of a later frame.
        const MAX_POLL: Duration = Duration::from_millis(50);

        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.read_fd()? {
                return Ok(Some(Timestamped::new(frame, Instant::now())));
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            let wait_dur = (deadline - now).min(MAX_POLL);
            let _ = self.event.wait(Some(wait_dur))?;
        }
    }
}

// ---------------------------------------------------------------------------
// Filterable (both modes)
// ---------------------------------------------------------------------------

impl<Mode> Filterable for PcanChannel<Mode> {
    type Error = PcanError;

    /// Apply acceptance filters.
    ///
    /// PCAN uses range-based filtering (`from_id..=to_id`), not mask-based.
    /// Each mask-based [`Filter`] is converted to the smallest contiguous ID
    /// range that covers all matching IDs. Multiple filters of the same type
    /// (standard or extended) are merged into a single encompassing range.
    ///
    /// This may accept additional IDs beyond the intended set when masks have
    /// non-contiguous zero bits.
    ///
    /// PCAN-Basic's `CAN_FilterMessages` only widens the active filter, so to
    /// actually narrow we first switch the channel's `PCAN_MESSAGE_FILTER`
    /// state to `CLOSE` (reject all), then call `CAN_FilterMessages` to open
    /// the requested ranges.
    ///
    /// The hardware filter can pass frames of the other type: an extended
    /// range passes the standard IDs inside it, and on Windows the two types
    /// share one filter. So the receive methods also drop frames outside the
    /// range opened for their type.
    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), Self::Error> {
        if filters.is_empty() {
            return self.clear_filters();
        }

        let mut std_min: Option<u32> = None;
        let mut std_max: Option<u32> = None;
        let mut ext_min: Option<u32> = None;
        let mut ext_max: Option<u32> = None;

        for filter in filters {
            let (from, to, is_extended) = mask_to_range(filter);
            if is_extended {
                ext_min = Some(ext_min.map_or(from, |cur: u32| cur.min(from)));
                ext_max = Some(ext_max.map_or(to, |cur: u32| cur.max(to)));
            } else {
                std_min = Some(std_min.map_or(from, |cur: u32| cur.min(from)));
                std_max = Some(std_max.map_or(to, |cur: u32| cur.max(to)));
            }
        }

        // Close the filter (reject all), then open the requested ranges via
        // CAN_FilterMessages.
        self.set_filter_state(ffi::PCAN_FILTER_CLOSE)?;

        let apply = (|| {
            if let (Some(from), Some(to)) = (std_min, std_max) {
                // SAFETY: filter_messages() was loaded from PCANBasic and self.handle is valid.
                let status = unsafe {
                    (self.lib.filter_messages)(self.handle, from, to, ffi::PCAN_MODE_STANDARD)
                };
                check_status(status)?;
            }
            if let (Some(from), Some(to)) = (ext_min, ext_max) {
                // SAFETY: filter_messages() was loaded from PCANBasic and self.handle is valid.
                let status = unsafe {
                    (self.lib.filter_messages)(self.handle, from, to, ffi::PCAN_MODE_EXTENDED)
                };
                check_status(status)?;
            }
            Ok(())
        })();

        if apply.is_ok() {
            self.filter = Some(IdRanges {
                standard: std_min.zip(std_max),
                extended: ext_min.zip(ext_max),
            });
        } else {
            // Best-effort: leave filters in accept-all rather than a
            // half-merged state. Ignore secondary cleanup failure.
            #[allow(clippy::let_underscore_must_use)]
            let _ = self.clear_filters();
        }
        apply
    }

    fn clear_filters(&mut self) -> Result<(), Self::Error> {
        self.filter = None;
        // Switch PCAN_MESSAGE_FILTER to OPEN to accept all frames regardless
        // of frame type. Using SetValue avoids the expand-only semantics of
        // CAN_FilterMessages.
        self.set_filter_state(ffi::PCAN_FILTER_OPEN)
    }
}

impl<Mode> PcanChannel<Mode> {
    /// Set the channel's `PCAN_MESSAGE_FILTER` parameter.
    fn set_filter_state(&self, state: u32) -> Result<(), PcanError> {
        let mut value = state;
        #[allow(clippy::cast_possible_truncation)] // size_of::<u32>() == 4, fits in u32
        // SAFETY: set_value() was loaded from PCANBasic and self.handle is valid.
        // value is a stack-allocated u32 with the matching buffer length.
        let status = unsafe {
            (self.lib.set_value)(
                self.handle,
                ffi::PCAN_MESSAGE_FILTER,
                std::ptr::from_mut(&mut value).cast::<std::ffi::c_void>(),
                std::mem::size_of::<u32>() as u32,
            )
        };
        check_status(status)
    }
}

/// Convert a mask-based filter to a range `(from_id, to_id, is_extended)`.
///
/// For a filter with `id=X` and `mask=M`, the matching set is all IDs where
/// `(id & mask) == (X & mask)`. The minimum matching ID is `(X & M)` and
/// the maximum is `(X & M) | (!M & max_id)`.
const fn mask_to_range(filter: &Filter) -> (u32, u32, bool) {
    let is_extended = filter.id.is_extended();
    let max_id = if is_extended { 0x1FFF_FFFF } else { 0x7FF };
    let raw_id = filter.id.raw();
    let mask = filter.mask & max_id;

    let from = raw_id & mask;
    let to = from | (!mask & max_id);
    (from, to, is_extended)
}

// ---------------------------------------------------------------------------
// BusStatus (both modes)
// ---------------------------------------------------------------------------

impl<Mode> BusStatus for PcanChannel<Mode> {
    type Error = PcanError;

    fn bus_state(&self) -> Result<BusState, Self::Error> {
        // SAFETY: get_status() was loaded from PCANBasic and self.handle is valid.
        let status = unsafe { (self.lib.get_status)(self.handle) };
        bus_state_from_status(status)
    }

    /// Read the transmit and receive error counters.
    ///
    /// PCAN-Basic has no parameter for them. On Linux they're read from the
    /// `tx_errors` / `rx_errors` attributes that PEAK's driver keeps in sysfs
    /// for the channel's device; a device that doesn't track them returns
    /// [`PcanError::Unsupported`]. The driver updates its copy when the
    /// device reports a bus error, so after the bus recovers it can read
    /// higher than the controller's own counters. On Windows this always
    /// returns [`PcanError::Unsupported`].
    fn error_counters(&self) -> Result<ErrorCounters, Self::Error> {
        #[cfg(target_os = "linux")]
        {
            sysfs_error_counters(self.event.fd())
        }
        #[cfg(not(target_os = "linux"))]
        {
            Err(PcanError::Unsupported(
                "PCAN-Basic has no parameter for the error counters",
            ))
        }
    }
}

/// Read the error counters of the PEAK driver's device behind `fd` from
/// `/sys/dev/char/<major>:<minor>/{tx,rx}_errors`.
#[cfg(target_os = "linux")]
fn sysfs_error_counters(fd: i32) -> Result<ErrorCounters, PcanError> {
    const UNSUPPORTED: PcanError =
        PcanError::Unsupported("the PCAN driver doesn't report error counters for this device");

    // SAFETY: libc::stat is a plain C struct, for which all zero bytes is a
    // valid value.
    let mut st: libc::stat = unsafe { std::mem::zeroed() };
    // SAFETY: fstat() is given a descriptor the channel owns and a pointer
    // to a stat struct for it to fill.
    if unsafe { libc::fstat(fd, &mut st) } != 0 {
        return Err(UNSUPPORTED);
    }
    let dir = format!(
        "/sys/dev/char/{}:{}",
        libc::major(st.st_rdev),
        libc::minor(st.st_rdev)
    );
    let read = |name: &str| -> Option<u8> {
        let text = std::fs::read_to_string(format!("{dir}/{name}")).ok()?;
        let value: u32 = text.trim().parse().ok()?;
        // A receive counter can pass 255; the trait's counters are u8.
        #[allow(clippy::cast_possible_truncation)] // clamped to 255 first
        Some(value.min(255) as u8)
    };
    match (read("tx_errors"), read("rx_errors")) {
        (Some(transmit), Some(receive)) => Ok(ErrorCounters { transmit, receive }),
        _ => Err(UNSUPPORTED),
    }
}

/// Map a `CAN_GetStatus` result to a [`BusState`].
///
/// The bus flags give the state. The transmit and receive queue flags say
/// nothing about it, so they're ignored. Any other status (e.g.
/// `PCAN_ERROR_ILLHW` for an unplugged adapter, or `PCAN_ERROR_INITIALIZE`)
/// means there's no bus state to read, and is returned as an error.
const fn bus_state_from_status(status: u32) -> Result<BusState, PcanError> {
    const BUS_FLAGS: u32 = ffi::PCAN_ERROR_BUSLIGHT
        | ffi::PCAN_ERROR_BUSHEAVY
        | ffi::PCAN_ERROR_BUSPASSIVE
        | ffi::PCAN_ERROR_BUSOFF;
    const QUEUE_FLAGS: u32 = ffi::PCAN_ERROR_XMTFULL
        | ffi::PCAN_ERROR_OVERRUN
        | ffi::PCAN_ERROR_QRCVEMPTY
        | ffi::PCAN_ERROR_QOVERRUN
        | ffi::PCAN_ERROR_QXMTFULL;

    if status & !(BUS_FLAGS | QUEUE_FLAGS) != 0 {
        Err(PcanError::Pcan(PcanStatus(status)))
    } else if status & ffi::PCAN_ERROR_BUSOFF != 0 {
        Ok(BusState::BusOff)
    } else if status & ffi::PCAN_ERROR_BUSPASSIVE != 0 || status & ffi::PCAN_ERROR_BUSHEAVY != 0 {
        Ok(BusState::ErrorPassive)
    } else {
        // PCAN_ERROR_OK or PCAN_ERROR_BUSLIGHT → ErrorActive
        Ok(BusState::ErrorActive)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bus_state_from_bus_flags() {
        assert_eq!(
            bus_state_from_status(ffi::PCAN_ERROR_OK).unwrap(),
            BusState::ErrorActive
        );
        assert_eq!(
            bus_state_from_status(ffi::PCAN_ERROR_BUSLIGHT).unwrap(),
            BusState::ErrorActive
        );
        assert_eq!(
            bus_state_from_status(ffi::PCAN_ERROR_BUSHEAVY).unwrap(),
            BusState::ErrorPassive
        );
        assert_eq!(
            bus_state_from_status(ffi::PCAN_ERROR_BUSPASSIVE).unwrap(),
            BusState::ErrorPassive
        );
        assert_eq!(
            bus_state_from_status(ffi::PCAN_ERROR_BUSOFF).unwrap(),
            BusState::BusOff
        );
        // Queue flags don't change the state.
        assert_eq!(
            bus_state_from_status(ffi::PCAN_ERROR_BUSOFF | ffi::PCAN_ERROR_QOVERRUN).unwrap(),
            BusState::BusOff
        );
    }

    // CAN_GetStatus returns these instead of a bus state when it can't read
    // one: the adapter was unplugged, the handle is invalid, or the channel
    // isn't initialized.
    #[test]
    fn bus_state_from_failed_status_is_an_error() {
        for status in [
            ffi::PCAN_ERROR_ILLHW,
            ffi::PCAN_ERROR_ILLHANDLE,
            ffi::PCAN_ERROR_INITIALIZE,
            ffi::PCAN_ERROR_NODRIVER,
        ] {
            assert!(
                matches!(
                    bus_state_from_status(status),
                    Err(PcanError::Pcan(PcanStatus(s))) if s == status
                ),
                "status 0x{status:X} must be an error, got {:?}",
                bus_state_from_status(status)
            );
        }
    }

    #[test]
    fn id_ranges_keep_each_type_to_its_own_range() {
        let ranges = IdRanges {
            standard: None,
            extended: Some((0x100, 0x1FF)),
        };
        assert!(ranges.accepts(CanId::new_extended(0x100).unwrap()));
        assert!(ranges.accepts(CanId::new_extended(0x1FF).unwrap()));
        assert!(!ranges.accepts(CanId::new_extended(0x200).unwrap()));
        assert!(!ranges.accepts(CanId::new_standard(0x100).unwrap()));

        let ranges = IdRanges {
            standard: Some((0x200, 0x200)),
            extended: None,
        };
        assert!(ranges.accepts(CanId::new_standard(0x200).unwrap()));
        assert!(!ranges.accepts(CanId::new_standard(0x201).unwrap()));
        assert!(!ranges.accepts(CanId::new_extended(0x200).unwrap()));
    }

    #[test]
    fn mask_to_range_exact_standard() {
        let filter = Filter {
            id: CanId::new_standard(0x100).unwrap(),
            mask: 0x7FF,
        };
        let (from, to, ext) = mask_to_range(&filter);
        assert!(!ext);
        assert_eq!(from, 0x100);
        assert_eq!(to, 0x100);
    }

    #[test]
    fn mask_to_range_prefix_standard() {
        // Match 0x100–0x1FF (mask = 0x700)
        let filter = Filter {
            id: CanId::new_standard(0x100).unwrap(),
            mask: 0x700,
        };
        let (from, to, ext) = mask_to_range(&filter);
        assert!(!ext);
        assert_eq!(from, 0x100);
        assert_eq!(to, 0x1FF);
    }

    #[test]
    fn mask_to_range_all_standard() {
        let filter = Filter {
            id: CanId::new_standard(0x000).unwrap(),
            mask: 0x000,
        };
        let (from, to, ext) = mask_to_range(&filter);
        assert!(!ext);
        assert_eq!(from, 0x000);
        assert_eq!(to, 0x7FF);
    }

    #[test]
    fn mask_to_range_exact_extended() {
        let filter = Filter {
            id: CanId::new_extended(0x18DA00F1).unwrap(),
            mask: 0x1FFF_FFFF,
        };
        let (from, to, ext) = mask_to_range(&filter);
        assert!(ext);
        assert_eq!(from, 0x18DA00F1);
        assert_eq!(to, 0x18DA00F1);
    }
}
