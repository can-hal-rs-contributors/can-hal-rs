//! PCAN channel implementation.
//!
//! [`PcanChannel`] wraps a PCAN-Basic channel handle and implements the
//! `can-hal` traits for CAN communication.

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_hal::bus::{BusState, BusStatus, ErrorCounters};
use can_hal::channel::{Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal::filter::{Filter, Filterable};
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};

use crate::convert;
use crate::error::{check_status, PcanError};
use crate::event::ReceiveEvent;
use crate::ffi;
use crate::library::PcanLibrary;

/// A CAN channel backed by a PCAN-Basic hardware interface.
///
/// Implements [`Transmit`], [`Receive`], [`TransmitFd`], [`ReceiveFd`],
/// [`Filterable`], and [`BusStatus`].
///
/// Created via [`PcanDriver`](crate::PcanDriver) and
/// [`PcanChannelBuilder`](crate::PcanChannelBuilder).
pub struct PcanChannel {
    lib: Arc<PcanLibrary>,
    handle: u16,
    event: ReceiveEvent,
    fd_mode: bool,
}

impl PcanChannel {
    /// Create a new channel. Called by the builder after `CAN_Initialize`
    /// succeeds.
    pub(crate) fn new(
        lib: Arc<PcanLibrary>,
        handle: u16,
        fd_mode: bool,
    ) -> Result<Self, PcanError> {
        let event = ReceiveEvent::new(lib.clone(), handle)?;
        Ok(PcanChannel {
            lib,
            handle,
            event,
            fd_mode,
        })
    }

    /// Try to read a classic CAN frame from the receive queue.
    /// Returns `Ok(None)` if the queue is empty or the frame was skipped
    /// (RTR, status).
    fn read_classic(&self) -> Result<Option<CanFrame>, PcanError> {
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
        let status = unsafe { (self.lib.read)(self.handle, &mut msg, &mut ts) };
        if status == ffi::PCAN_ERROR_QRCVEMPTY {
            return Ok(None);
        }
        check_status(status)?;
        convert::from_pcan_msg(&msg)
    }

    /// Try to read any frame (classic or FD) via `CAN_ReadFD`.
    /// Returns `Ok(None)` if the queue is empty or the frame was skipped.
    fn read_fd(&self) -> Result<Option<Frame>, PcanError> {
        let mut msg = ffi::TPCANMsgFD {
            id: 0,
            msg_type: 0,
            dlc: 0,
            data: [0; 64],
        };
        let mut ts: u64 = 0;
        let status = unsafe { (self.lib.read_fd)(self.handle, &mut msg, &mut ts) };
        if status == ffi::PCAN_ERROR_QRCVEMPTY {
            return Ok(None);
        }
        check_status(status)?;
        convert::from_pcan_msg_fd(&msg)
    }
}

impl Drop for PcanChannel {
    fn drop(&mut self) {
        unsafe {
            let _ = (self.lib.uninitialize)(self.handle);
        }
    }
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------

impl Transmit for PcanChannel {
    type Error = PcanError;

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), Self::Error> {
        let mut msg = convert::to_pcan_msg(frame);
        let status = unsafe { (self.lib.write)(self.handle, &mut msg) };
        check_status(status)
    }
}

// ---------------------------------------------------------------------------
// Receive (classic CAN)
// ---------------------------------------------------------------------------

impl Receive for PcanChannel {
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
        match self.read_classic()? {
            Some(frame) => Ok(Some(Timestamped::new(frame, Instant::now()))),
            None => Ok(None),
        }
    }

    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame, Instant>>, Self::Error> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.read_classic()? {
                return Ok(Some(Timestamped::new(frame, Instant::now())));
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            let signaled = self.event.wait(Some(deadline - now))?;
            if !signaled {
                // Timed out — try one last read in case a message arrived
                // between the read and the wait.
                return Ok(self
                    .read_classic()?
                    .map(|f| Timestamped::new(f, Instant::now())));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// TransmitFd
// ---------------------------------------------------------------------------

impl TransmitFd for PcanChannel {
    type Error = PcanError;

    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), Self::Error> {
        if !self.fd_mode {
            return Err(PcanError::InvalidFrame(
                "channel was not initialized in FD mode; use data_bitrate() or fd_timing_string() before connect()"
                    .into(),
            ));
        }
        let mut msg = convert::to_pcan_msg_fd(frame);
        let status = unsafe { (self.lib.write_fd)(self.handle, &mut msg) };
        check_status(status)
    }
}

// ---------------------------------------------------------------------------
// ReceiveFd
// ---------------------------------------------------------------------------

impl ReceiveFd for PcanChannel {
    type Error = PcanError;
    type Timestamp = Instant;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame, Instant>, Self::Error> {
        if !self.fd_mode {
            return Err(PcanError::InvalidFrame(
                "channel was not initialized in FD mode; use data_bitrate() or fd_timing_string() before connect()"
                    .into(),
            ));
        }
        loop {
            if let Some(frame) = self.read_fd()? {
                return Ok(Timestamped::new(frame, Instant::now()));
            }
            self.event.wait(None)?;
        }
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame, Instant>>, Self::Error> {
        if !self.fd_mode {
            return Err(PcanError::InvalidFrame(
                "channel was not initialized in FD mode; use data_bitrate() or fd_timing_string() before connect()"
                    .into(),
            ));
        }
        match self.read_fd()? {
            Some(frame) => Ok(Some(Timestamped::new(frame, Instant::now()))),
            None => Ok(None),
        }
    }

    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame, Instant>>, Self::Error> {
        if !self.fd_mode {
            return Err(PcanError::InvalidFrame(
                "channel was not initialized in FD mode; use data_bitrate() or fd_timing_string() before connect()"
                    .into(),
            ));
        }
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.read_fd()? {
                return Ok(Some(Timestamped::new(frame, Instant::now())));
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            let signaled = self.event.wait(Some(deadline - now))?;
            if !signaled {
                return Ok(self.read_fd()?.map(|f| Timestamped::new(f, Instant::now())));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Filterable
// ---------------------------------------------------------------------------

impl Filterable for PcanChannel {
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

        if let (Some(from), Some(to)) = (std_min, std_max) {
            let status = unsafe {
                (self.lib.filter_messages)(self.handle, from, to, ffi::PCAN_MODE_STANDARD)
            };
            check_status(status)?;
        }

        if let (Some(from), Some(to)) = (ext_min, ext_max) {
            let status = unsafe {
                (self.lib.filter_messages)(self.handle, from, to, ffi::PCAN_MODE_EXTENDED)
            };
            check_status(status)?;
        }

        Ok(())
    }

    fn clear_filters(&mut self) -> Result<(), Self::Error> {
        let status = unsafe {
            (self.lib.filter_messages)(self.handle, 0x000, 0x7FF, ffi::PCAN_MODE_STANDARD)
        };
        check_status(status)?;

        let status = unsafe {
            (self.lib.filter_messages)(
                self.handle,
                0x0000_0000,
                0x1FFF_FFFF,
                ffi::PCAN_MODE_EXTENDED,
            )
        };
        check_status(status)?;

        Ok(())
    }
}

/// Convert a mask-based filter to a range `(from_id, to_id, is_extended)`.
///
/// For a filter with `id=X` and `mask=M`, the matching set is all IDs where
/// `(id & mask) == (X & mask)`. The minimum matching ID is `(X & M)` and
/// the maximum is `(X & M) | (!M & max_id)`.
fn mask_to_range(filter: &Filter) -> (u32, u32, bool) {
    let is_extended = filter.id.is_extended();
    let max_id = if is_extended { 0x1FFF_FFFF } else { 0x7FF };
    let raw_id = filter.id.raw();
    let mask = filter.mask & max_id;

    let from = raw_id & mask;
    let to = from | (!mask & max_id);
    (from, to, is_extended)
}

// ---------------------------------------------------------------------------
// BusStatus
// ---------------------------------------------------------------------------

impl BusStatus for PcanChannel {
    type Error = PcanError;

    fn bus_state(&self) -> Result<BusState, Self::Error> {
        let status = unsafe { (self.lib.get_status)(self.handle) };

        if status & ffi::PCAN_ERROR_BUSOFF != 0 {
            Ok(BusState::BusOff)
        } else if status & ffi::PCAN_ERROR_BUSPASSIVE != 0 || status & ffi::PCAN_ERROR_BUSHEAVY != 0
        {
            Ok(BusState::ErrorPassive)
        } else {
            // PCAN_ERROR_OK or PCAN_ERROR_BUSLIGHT → ErrorActive
            Ok(BusState::ErrorActive)
        }
    }

    fn error_counters(&self) -> Result<ErrorCounters, Self::Error> {
        // PCAN-Basic's support for individual TX/RX error counters varies by
        // hardware. Attempt to read them; fall back to 0 if unsupported.
        let mut rx_errors: u32 = 0;
        let status_rx = unsafe {
            (self.lib.get_value)(
                self.handle,
                ffi::PCAN_BUSERROR_READ,
                &mut rx_errors as *mut _ as *mut std::ffi::c_void,
                std::mem::size_of::<u32>() as u32,
            )
        };

        let mut tx_errors: u32 = 0;
        let status_tx = unsafe {
            (self.lib.get_value)(
                self.handle,
                ffi::PCAN_BUSERROR_WRITE,
                &mut tx_errors as *mut _ as *mut std::ffi::c_void,
                std::mem::size_of::<u32>() as u32,
            )
        };

        Ok(ErrorCounters {
            receive: if status_rx == ffi::PCAN_ERROR_OK {
                rx_errors.min(255) as u8
            } else {
                0
            },
            transmit: if status_tx == ffi::PCAN_ERROR_OK {
                tx_errors.min(255) as u8
            } else {
                0
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_hal::id::CanId;

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
