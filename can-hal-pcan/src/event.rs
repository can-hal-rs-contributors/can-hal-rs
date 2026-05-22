//! Platform-specific receive event handling.
//!
//! On Windows: `CreateEventW` → `CAN_SetValue(PCAN_RECEIVE_EVENT)` →
//! `WaitForSingleObject`.
//!
//! On Linux: `CAN_GetValue(PCAN_RECEIVE_EVENT)` to get a file descriptor →
//! `poll()`.

use std::sync::Arc;
use std::time::Duration;

#[cfg(not(target_os = "windows"))]
use crate::error::check_status;
use crate::error::PcanError;
#[cfg(target_os = "windows")]
use crate::error::PcanStatus;
use crate::ffi;
use crate::library::PcanLibrary;

/// An opaque handle to a platform-specific receive event.
///
/// Created by [`PcanChannel`](crate::channel::PcanChannel) during
/// initialization. The event is used to efficiently block until a CAN message
/// is available, avoiding busy-wait polling.
pub struct ReceiveEvent {
    lib: Arc<PcanLibrary>,
    handle: u16,
    #[cfg(target_os = "windows")]
    event_handle: *mut std::ffi::c_void,
    #[cfg(not(target_os = "windows"))]
    fd: i32,
}

impl ReceiveEvent {
    /// Create and register a receive event for the given PCAN channel.
    pub fn new(lib: Arc<PcanLibrary>, handle: u16) -> Result<Self, PcanError> {
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
    pub fn wait(&self, timeout: Option<Duration>) -> Result<bool, PcanError> {
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

        // SAFETY: CreateEventW is a safe Windows API call with valid null pointers
        let event_handle = unsafe { CreateEventW(ptr::null(), 0, 0, ptr::null()) };
        if event_handle == INVALID_HANDLE_VALUE || event_handle.is_null() {
            return Err(PcanError::Platform("CreateEventW failed".into()));
        }

        // Register the event with PCAN.
        let mut ev = event_handle;
        #[allow(clippy::cast_possible_truncation)] // size_of::<isize>() fits in u32
        // SAFETY: set_value was loaded from PCANBasic; handle is valid; ev is stack-allocated
        let status = unsafe {
            (lib.set_value)(
                handle,
                ffi::PCAN_RECEIVE_EVENT,
                std::ptr::from_mut(&mut ev).cast::<std::ffi::c_void>(),
                std::mem::size_of::<isize>() as u32,
            )
        };
        if status != ffi::PCAN_ERROR_OK {
            // SAFETY: event_handle was successfully created above
            unsafe {
                windows_sys::Win32::Foundation::CloseHandle(event_handle);
            }
            return Err(PcanError::Pcan(PcanStatus(status)));
        }

        Ok(Self {
            lib,
            handle,
            event_handle,
        })
    }

    #[cfg(target_os = "windows")]
    fn wait_windows(&self, timeout: Option<Duration>) -> Result<bool, PcanError> {
        use windows_sys::Win32::Foundation::{WAIT_OBJECT_0, WAIT_TIMEOUT};
        use windows_sys::Win32::System::Threading::WaitForSingleObject;

        #[allow(clippy::cast_possible_truncation)] // clamped to u32::MAX
        let ms = timeout.map_or(0xFFFF_FFFF, |d| {
            d.as_millis().min(u128::from(u32::MAX)) as u32
        });

        // SAFETY: event_handle was obtained from CreateEventW
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
        #[allow(clippy::cast_possible_truncation)] // size_of::<i32>() == 4, fits in u32
        // SAFETY: get_value() was loaded from PCANBasic and handle is valid.
        // fd points to a valid stack-allocated i32 with correct buffer length.
        let status = unsafe {
            (lib.get_value)(
                handle,
                ffi::PCAN_RECEIVE_EVENT,
                std::ptr::from_mut(&mut fd).cast::<std::ffi::c_void>(),
                std::mem::size_of::<i32>() as u32,
            )
        };
        check_status(status)?;

        Ok(Self { lib, handle, fd })
    }

    #[cfg(not(target_os = "windows"))]
    fn wait_unix(&self, timeout: Option<Duration>) -> Result<bool, PcanError> {
        // i32::MAX as u128 is always lossless; the final `as i32` is bounded by .min().
        #[allow(clippy::cast_possible_truncation)]
        let timeout_ms = timeout.map_or(-1, |d| d.as_millis().min(i32::MAX as u128) as i32);

        let mut pfd = libc::pollfd {
            fd: self.fd,
            events: libc::POLLIN,
            revents: 0,
        };

        // SAFETY: poll() is called with a valid pointer to a stack-allocated pollfd
        // and nfds=1, which matches the single-element array.
        let ret = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        match ret {
            _ if ret < 0 => Err(PcanError::Platform(format!(
                "poll() failed: {}",
                std::io::Error::last_os_error()
            ))),
            0 => Ok(false),
            _ => Ok(true),
        }
    }
}

// SAFETY: On Windows the HANDLE is an opaque kernel object that is safe to send
// across threads. On Unix the fd is a plain integer - also Send-safe.
unsafe impl Send for ReceiveEvent {}

impl Drop for ReceiveEvent {
    fn drop(&mut self) {
        #[cfg(target_os = "windows")]
        {
            // Deregister the event from PCAN, then close the handle.
            let mut zero: *mut std::ffi::c_void = std::ptr::null_mut();
            #[allow(
                clippy::multiple_unsafe_ops_per_block,
                clippy::let_underscore_must_use,
                clippy::cast_possible_truncation
            )]
            // SAFETY: set_value and CloseHandle cleanup; errors deliberately ignored in Drop
            unsafe {
                let _ = (self.lib.set_value)(
                    self.handle,
                    ffi::PCAN_RECEIVE_EVENT,
                    std::ptr::from_mut(&mut zero).cast::<std::ffi::c_void>(),
                    std::mem::size_of::<isize>() as u32,
                );
                windows_sys::Win32::Foundation::CloseHandle(self.event_handle);
            }
        }
        // On Linux the fd is owned by the PCAN library - do not close it.
        // Fields `lib`, `handle`, and `fd` are dropped implicitly; no cleanup needed.
        #[cfg(not(target_os = "windows"))]
        {
            let _ = (&self.lib, self.handle, self.fd);
        }
    }
}
