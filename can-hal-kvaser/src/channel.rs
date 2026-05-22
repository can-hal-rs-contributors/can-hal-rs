use std::marker::PhantomData;
use std::os::raw::{c_long, c_ulong};
use std::sync::Arc;
use std::time::{Duration, Instant};

use can_hal::{
    BusState, BusStatus, CanFdFrame, CanFrame, ErrorCounters, Filter, Filterable, Frame, Receive,
    ReceiveFd, Timestamped, Transmit, TransmitFd,
};

/// Maximum poll interval for the event fd.
///
/// The mhydra (linuxcan) driver uses edge-triggered event semantics: the event fd
/// becomes readable when new frames arrive, but may not re-signal if frames arrive
/// between `read_frame()` returning `None` and `poll()` starting. By capping the
/// poll timeout we guarantee periodic queue drains regardless of event fd state.
const MAX_POLL_INTERVAL: Duration = Duration::from_millis(50);

use crate::convert::{from_canlib_frame, to_canlib_id};
use crate::error::{check_status, KvaserError};
use crate::event::ReceiveEvent;
use crate::ffi::{
    CAN_ERR_NOMSG, CAN_FILTER_SET_CODE_EXT, CAN_FILTER_SET_CODE_STD, CAN_FILTER_SET_MASK_EXT,
    CAN_FILTER_SET_MASK_STD, CAN_MSG_BRS, CAN_MSG_ESI, CAN_MSG_FDF, CAN_STAT_BUS_OFF,
    CAN_STAT_ERROR_PASSIVE, CAN_STAT_ERROR_WARNING,
};
use crate::library::KvaserLibrary;
use crate::mode::{Classic, Fd};

/// An open, on-bus KVASER CAN channel.
///
/// Parameterized on a type-state marker:
/// - [`KvaserChannel<Classic>`] implements [`Transmit`] and [`Receive`].
/// - [`KvaserChannel<Fd>`] implements [`TransmitFd`] and [`ReceiveFd`].
///
/// Both modes implement [`Filterable`] and [`BusStatus`].
pub struct KvaserChannel<Mode> {
    pub(crate) lib: Arc<KvaserLibrary>,
    pub(crate) handle: i32,
    pub(crate) event: ReceiveEvent,
    _mode: PhantomData<Mode>,
}

impl<Mode> KvaserChannel<Mode> {
    pub(crate) const fn new(
        lib: Arc<KvaserLibrary>,
        handle: i32,
        event: ReceiveEvent,
    ) -> Result<Self, KvaserError> {
        Ok(Self {
            lib,
            handle,
            event,
            _mode: PhantomData,
        })
    }

    /// Non-blocking read. Returns `Ok(None)` if the queue is empty.
    fn read_frame(&mut self) -> Result<Option<Frame>, KvaserError> {
        let mut raw_id: c_long = 0;
        // 64 bytes covers both classic CAN (<=8 bytes used) and CAN FD (<=64 bytes).
        let mut data = [0u8; 64];
        let mut dlc: u32 = 0;
        let mut flags: u32 = 0;
        let mut timestamp: c_ulong = 0;

        // SAFETY: canRead was loaded from canlib; handle is valid; all pointers are stack-allocated
        let status = unsafe {
            (self.lib.read)(
                self.handle,
                &mut raw_id,
                data.as_mut_ptr().cast(),
                &mut dlc,
                &mut flags,
                &mut timestamp,
            )
        };

        if status == CAN_ERR_NOMSG {
            return Ok(None);
        }
        check_status(status)?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // CAN ID is at most 29 bits; raw_id from canRead is always non-negative
        from_canlib_frame(raw_id as u32, &data, dlc, flags)
    }
}

impl<Mode> Drop for KvaserChannel<Mode> {
    fn drop(&mut self) {
        // SAFETY: bus_off and close were loaded from canlib; handle is valid from canOpenChannel
        #[allow(clippy::multiple_unsafe_ops_per_block, clippy::let_underscore_must_use)]
        unsafe {
            let _ = (self.lib.bus_off)(self.handle);
            let _ = (self.lib.close)(self.handle);
        }
    }
}

// ---------------------------------------------------------------------------
// Classic-mode trait impls
// ---------------------------------------------------------------------------

impl Transmit for KvaserChannel<Classic> {
    type Error = KvaserError;

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_possible_wrap,
        clippy::cast_sign_loss,
        clippy::cast_lossless
    )]
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), KvaserError> {
        let (id, flags) = to_canlib_id(frame.id());
        // SAFETY: canWrite was loaded from canlib; handle is valid; data pointer is valid
        check_status(unsafe {
            (self.lib.write)(
                self.handle,
                id as c_long,
                frame.data().as_ptr().cast(),
                frame.len() as u32,
                flags,
            )
        })?;
        // SAFETY: canWriteSync was loaded from canlib; handle is valid
        check_status(unsafe { (self.lib.write_sync)(self.handle, 100) })
    }
}

impl Receive for KvaserChannel<Classic> {
    type Error = KvaserError;
    type Timestamp = Instant;

    fn receive(&mut self) -> Result<Timestamped<CanFrame, Instant>, KvaserError> {
        loop {
            match self.read_frame()? {
                Some(Frame::Can(f)) => return Ok(Timestamped::new(f, Instant::now())),
                Some(Frame::Fd(_)) => {} // FD frame on classic receive — skip and retry
                None => {
                    let _ = self.event.wait(Some(MAX_POLL_INTERVAL))?;
                }
            }
        }
    }

    fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame, Instant>>, KvaserError> {
        loop {
            match self.read_frame()? {
                Some(Frame::Can(f)) => return Ok(Some(Timestamped::new(f, Instant::now()))),
                Some(Frame::Fd(_)) => {}
                None => return Ok(None),
            }
        }
    }

    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame, Instant>>, KvaserError> {
        let deadline = Instant::now() + timeout;
        loop {
            loop {
                match self.read_frame()? {
                    Some(Frame::Can(f)) => return Ok(Some(Timestamped::new(f, Instant::now()))),
                    Some(Frame::Fd(_)) => {}
                    None => break,
                }
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            let poll_timeout = remaining.min(MAX_POLL_INTERVAL);
            let _ = self.event.wait(Some(poll_timeout))?;
        }
    }
}

// ---------------------------------------------------------------------------
// FD-mode trait impls
// ---------------------------------------------------------------------------

impl TransmitFd for KvaserChannel<Fd> {
    type Error = KvaserError;

    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_possible_wrap,
        clippy::cast_sign_loss,
        clippy::cast_lossless
    )]
    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), KvaserError> {
        let (id, mut flags) = to_canlib_id(frame.id());
        flags |= CAN_MSG_FDF;
        if frame.brs() {
            flags |= CAN_MSG_BRS;
        }
        if frame.esi() {
            flags |= CAN_MSG_ESI;
        }
        // SAFETY: canWrite was loaded from canlib; handle is valid; data pointer is valid
        check_status(unsafe {
            (self.lib.write)(
                self.handle,
                id as c_long,
                frame.data().as_ptr().cast(),
                frame.len() as u32,
                flags,
            )
        })?;
        // SAFETY: canWriteSync was loaded from canlib; handle is valid
        check_status(unsafe { (self.lib.write_sync)(self.handle, 100) })
    }
}

impl ReceiveFd for KvaserChannel<Fd> {
    type Error = KvaserError;
    type Timestamp = Instant;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame, Instant>, KvaserError> {
        loop {
            match self.read_frame()? {
                Some(frame) => return Ok(Timestamped::new(frame, Instant::now())),
                None => {
                    let _ = self.event.wait(Some(MAX_POLL_INTERVAL))?;
                }
            }
        }
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame, Instant>>, KvaserError> {
        Ok(self
            .read_frame()?
            .map(|frame| Timestamped::new(frame, Instant::now())))
    }

    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame, Instant>>, KvaserError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(frame) = self.read_frame()? {
                return Ok(Some(Timestamped::new(frame, Instant::now())));
            }
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(None);
            }
            let poll_timeout = remaining.min(MAX_POLL_INTERVAL);
            let _ = self.event.wait(Some(poll_timeout))?;
        }
    }
}

// ---------------------------------------------------------------------------
// Filterable (both modes)
// ---------------------------------------------------------------------------

impl<Mode> Filterable for KvaserChannel<Mode> {
    type Error = KvaserError;

    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), KvaserError> {
        apply_merged_filter(
            &self.lib,
            self.handle,
            filters,
            |f| f.id.is_standard(),
            CAN_FILTER_SET_CODE_STD,
            CAN_FILTER_SET_MASK_STD,
        )?;
        apply_merged_filter(
            &self.lib,
            self.handle,
            filters,
            |f| f.id.is_extended(),
            CAN_FILTER_SET_CODE_EXT,
            CAN_FILTER_SET_MASK_EXT,
        )?;
        Ok(())
    }

    fn clear_filters(&mut self) -> Result<(), KvaserError> {
        for &flag in &[
            CAN_FILTER_SET_CODE_STD,
            CAN_FILTER_SET_MASK_STD,
            CAN_FILTER_SET_CODE_EXT,
            CAN_FILTER_SET_MASK_EXT,
        ] {
            // SAFETY: canAccept was loaded from canlib; handle is valid
            check_status(unsafe { (self.lib.accept)(self.handle, 0, flag) })?;
        }
        Ok(())
    }
}

/// Merge all filters matching `predicate` into a single (code, mask) pair and
/// apply it via `canAccept`. No heap allocation.
#[allow(clippy::cast_possible_wrap, clippy::cast_lossless)] // c_long is i32 on Windows, i64 on Linux
fn apply_merged_filter(
    lib: &KvaserLibrary,
    handle: i32,
    filters: &[Filter],
    predicate: impl Fn(&Filter) -> bool,
    code_flag: u32,
    mask_flag: u32,
) -> Result<(), KvaserError> {
    let mut merged: Option<(u32, u32)> = None;

    for f in filters.iter().filter(|f| predicate(f)) {
        let code = f.id.raw() & f.mask;
        merged = Some(match merged {
            None => (code, f.mask),
            Some((c, m)) => (c & code, m | f.mask),
        });
    }

    if let Some((code, mask)) = merged {
        // SAFETY: canAccept was loaded from canlib; handle is valid
        check_status(unsafe { (lib.accept)(handle, code as c_long, code_flag) })?;
        // SAFETY: canAccept was loaded from canlib; handle is valid
        check_status(unsafe { (lib.accept)(handle, mask as c_long, mask_flag) })?;
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// BusStatus (both modes)
// ---------------------------------------------------------------------------

impl<Mode> BusStatus for KvaserChannel<Mode> {
    type Error = KvaserError;

    fn bus_state(&self) -> Result<BusState, KvaserError> {
        let mut flags: c_ulong = 0;
        // SAFETY: canReadStatus was loaded from canlib; handle is valid
        check_status(unsafe { (self.lib.read_status)(self.handle, &mut flags) })?;

        if flags & CAN_STAT_BUS_OFF != 0 {
            Ok(BusState::BusOff)
        } else if flags & (CAN_STAT_ERROR_PASSIVE | CAN_STAT_ERROR_WARNING) != 0 {
            Ok(BusState::ErrorPassive)
        } else {
            Ok(BusState::ErrorActive)
        }
    }

    fn error_counters(&self) -> Result<ErrorCounters, KvaserError> {
        let mut tx_err: u32 = 0;
        let mut rx_err: u32 = 0;
        let mut overrun: u32 = 0;
        // SAFETY: canReadErrorCounters was loaded from canlib; handle is valid
        check_status(unsafe {
            (self.lib.read_error_counters)(self.handle, &mut tx_err, &mut rx_err, &mut overrun)
        })?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(ErrorCounters {
            transmit: tx_err.min(255) as u8,
            receive: rx_err.min(255) as u8,
        })
    }
}
