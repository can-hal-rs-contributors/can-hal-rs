//! Platform-specific receive event handling via `canIoCtl`.
//!
//! On Linux:   `canIoCtl(canIOCTL_GET_EVENTHANDLE)` returns a file descriptor → `poll()`.
//! On Windows: `canIoCtl(canIOCTL_GET_EVENTHANDLE)` returns a `HANDLE`         → `WaitForSingleObject`.
//!
//! The event handle / fd is owned by the CANlib driver and must NOT be closed.

use std::ffi::c_void;
use std::time::Duration;

use crate::error::KvaserError;
use crate::ffi::CAN_IOCTL_GET_EVENTHANDLE;
use crate::library::KvaserLibrary;

/// An opaque handle to a platform-specific receive-ready event.
///
/// Created once per `KvaserChannel` during `connect()`. Used to efficiently
/// block until a CAN message is available, avoiding busy-wait polling.
pub(crate) struct ReceiveEvent {
    #[cfg(not(target_os = "windows"))]
    fd: i32,
    #[cfg(target_os = "windows")]
    event_handle: *mut c_void,
}

impl ReceiveEvent {
    /// Obtain the receive event for the given open channel handle.
    pub(crate) fn new(lib: &KvaserLibrary, handle: i32) -> Result<Self, KvaserError> {
        #[cfg(not(target_os = "windows"))]
        {
            Self::new_unix(lib, handle)
        }
        #[cfg(target_os = "windows")]
        {
            Self::new_windows(lib, handle)
        }
    }

    /// Block until a frame may be available, or until `timeout` elapses.
    ///
    /// Returns `true` if the event was signalled, `false` on timeout.
    /// `None` timeout waits indefinitely.
    pub(crate) fn wait(&self, timeout: Option<Duration>) -> Result<bool, KvaserError> {
        #[cfg(not(target_os = "windows"))]
        {
            self.wait_unix(timeout)
        }
        #[cfg(target_os = "windows")]
        {
            self.wait_windows(timeout)
        }
    }

    // -----------------------------------------------------------------------
    // Linux / Unix implementation
    // -----------------------------------------------------------------------

    #[cfg(not(target_os = "windows"))]
    fn new_unix(lib: &KvaserLibrary, handle: i32) -> Result<Self, KvaserError> {
        let mut fd: i32 = 0;
        let status = unsafe {
            (lib.io_ctl)(
                handle,
                CAN_IOCTL_GET_EVENTHANDLE,
                &mut fd as *mut i32 as *mut c_void,
                std::mem::size_of::<i32>() as u32,
            )
        };
        crate::error::check_status(status)?;
        Ok(ReceiveEvent { fd })
    }

    #[cfg(not(target_os = "windows"))]
    fn wait_unix(&self, timeout: Option<Duration>) -> Result<bool, KvaserError> {
        let timeout_ms = match timeout {
            Some(d) => d.as_millis().min(i32::MAX as u128) as i32,
            None => -1, // infinite
        };

        let mut pfd = libc::pollfd {
            fd: self.fd,
            events: libc::POLLIN,
            revents: 0,
        };

        let ret = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        if ret < 0 {
            Err(KvaserError::Platform(format!(
                "poll() failed: {}",
                std::io::Error::last_os_error()
            )))
        } else {
            Ok(ret > 0)
        }
    }

    // -----------------------------------------------------------------------
    // Windows implementation
    // -----------------------------------------------------------------------

    #[cfg(target_os = "windows")]
    fn new_windows(lib: &KvaserLibrary, handle: i32) -> Result<Self, KvaserError> {
        let mut event_handle: *mut c_void = std::ptr::null_mut();
        let status = unsafe {
            (lib.io_ctl)(
                handle,
                CAN_IOCTL_GET_EVENTHANDLE,
                &mut event_handle as *mut _ as *mut c_void,
                std::mem::size_of::<*mut c_void>() as u32,
            )
        };
        crate::error::check_status(status)?;
        Ok(ReceiveEvent { event_handle })
    }

    #[cfg(target_os = "windows")]
    fn wait_windows(&self, timeout: Option<Duration>) -> Result<bool, KvaserError> {
        use windows_sys::Win32::Foundation::{WAIT_OBJECT_0, WAIT_TIMEOUT};
        use windows_sys::Win32::System::Threading::WaitForSingleObject;

        let ms = match timeout {
            Some(d) => d.as_millis().min(u32::MAX as u128) as u32,
            None => 0xFFFF_FFFF, // INFINITE
        };

        let result = unsafe { WaitForSingleObject(self.event_handle, ms) };
        match result {
            WAIT_OBJECT_0 => Ok(true),
            WAIT_TIMEOUT => Ok(false),
            _ => Err(KvaserError::Platform(format!(
                "WaitForSingleObject returned 0x{result:08X}"
            ))),
        }
    }
}

// SAFETY: On Windows the HANDLE is an opaque kernel object that is safe to send
// across threads. On Unix the fd is a plain integer — also Send-safe.
unsafe impl Send for ReceiveEvent {}

// The event handle / fd is owned by the CANlib driver — no cleanup needed.
