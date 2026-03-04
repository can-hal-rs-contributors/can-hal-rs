//! Platform-specific receive event handling.
//!
//! On Windows: `CreateEventW` → `CAN_SetValue(PCAN_RECEIVE_EVENT)` →
//! `WaitForSingleObject`.
//!
//! On Linux: `CAN_GetValue(PCAN_RECEIVE_EVENT)` to get a file descriptor →
//! `poll()`.

use std::sync::Arc;
use std::time::Duration;

#[cfg(target_os = "windows")]
use crate::error::PcanStatus;
use crate::error::{check_status, PcanError};
use crate::ffi;
use crate::library::PcanLibrary;

/// An opaque handle to a platform-specific receive event.
///
/// Created by [`PcanChannel`](crate::channel::PcanChannel) during
/// initialization. The event is used to efficiently block until a CAN message
/// is available, avoiding busy-wait polling.
pub(crate) struct ReceiveEvent {
    lib: Arc<PcanLibrary>,
    handle: u16,
    #[cfg(target_os = "windows")]
    event_handle: isize,
    #[cfg(not(target_os = "windows"))]
    fd: i32,
}

impl ReceiveEvent {
    /// Create and register a receive event for the given PCAN channel.
    pub(crate) fn new(lib: Arc<PcanLibrary>, handle: u16) -> Result<Self, PcanError> {
        #[cfg(target_os = "windows")]
        {
            Self::new_windows(lib, handle)
        }
        #[cfg(not(target_os = "windows"))]
        {
            Self::new_unix(lib, handle)
        }
    }

    /// Wait for a receive event.
    ///
    /// Returns `true` if the event was signaled (a message may be available),
    /// `false` if the wait timed out. `None` timeout means wait indefinitely.
    pub(crate) fn wait(&self, timeout: Option<Duration>) -> Result<bool, PcanError> {
        #[cfg(target_os = "windows")]
        {
            self.wait_windows(timeout)
        }
        #[cfg(not(target_os = "windows"))]
        {
            self.wait_unix(timeout)
        }
    }

    // -----------------------------------------------------------------------
    // Windows implementation
    // -----------------------------------------------------------------------

    #[cfg(target_os = "windows")]
    fn new_windows(lib: Arc<PcanLibrary>, handle: u16) -> Result<Self, PcanError> {
        use std::ptr;
        use windows_sys::Win32::Foundation::INVALID_HANDLE_VALUE;
        use windows_sys::Win32::System::Threading::CreateEventW;

        // Create an auto-reset, initially non-signaled event.
        let event_handle = unsafe { CreateEventW(ptr::null(), 0, 0, ptr::null()) };
        if event_handle == INVALID_HANDLE_VALUE || event_handle == 0 {
            return Err(PcanError::Platform("CreateEventW failed".into()));
        }

        // Register the event with PCAN.
        let mut ev = event_handle;
        let status = unsafe {
            (lib.set_value)(
                handle,
                ffi::PCAN_RECEIVE_EVENT,
                &mut ev as *mut _ as *mut std::ffi::c_void,
                std::mem::size_of::<isize>() as u32,
            )
        };
        if status != ffi::PCAN_ERROR_OK {
            unsafe {
                windows_sys::Win32::Foundation::CloseHandle(event_handle);
            }
            return Err(PcanError::Pcan(PcanStatus(status)));
        }

        Ok(ReceiveEvent {
            lib,
            handle,
            event_handle,
        })
    }

    #[cfg(target_os = "windows")]
    fn wait_windows(&self, timeout: Option<Duration>) -> Result<bool, PcanError> {
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
            _ => Err(PcanError::Platform(format!(
                "WaitForSingleObject returned 0x{result:08X}"
            ))),
        }
    }

    // -----------------------------------------------------------------------
    // Unix / Linux implementation
    // -----------------------------------------------------------------------

    #[cfg(not(target_os = "windows"))]
    fn new_unix(lib: Arc<PcanLibrary>, handle: u16) -> Result<Self, PcanError> {
        let mut fd: i32 = 0;
        let status = unsafe {
            (lib.get_value)(
                handle,
                ffi::PCAN_RECEIVE_EVENT,
                &mut fd as *mut _ as *mut std::ffi::c_void,
                std::mem::size_of::<i32>() as u32,
            )
        };
        check_status(status)?;

        Ok(ReceiveEvent { lib, handle, fd })
    }

    #[cfg(not(target_os = "windows"))]
    fn wait_unix(&self, timeout: Option<Duration>) -> Result<bool, PcanError> {
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
            Err(PcanError::Platform(format!(
                "poll() failed: {}",
                std::io::Error::last_os_error()
            )))
        } else if ret == 0 {
            Ok(false)
        } else {
            Ok(true)
        }
    }
}

impl Drop for ReceiveEvent {
    fn drop(&mut self) {
        #[cfg(target_os = "windows")]
        {
            // Deregister the event from PCAN, then close the handle.
            let mut zero: isize = 0;
            unsafe {
                let _ = (self.lib.set_value)(
                    self.handle,
                    ffi::PCAN_RECEIVE_EVENT,
                    &mut zero as *mut _ as *mut std::ffi::c_void,
                    std::mem::size_of::<isize>() as u32,
                );
                windows_sys::Win32::Foundation::CloseHandle(self.event_handle);
            }
        }
        // On Linux, the fd is owned by the PCAN library -- do not close it.
        #[cfg(not(target_os = "windows"))]
        {
            let _ = &self.lib;
            let _ = self.handle;
            let _ = self.fd;
        }
    }
}
