//! Platform-specific receive event handling via `canIoCtl`.
//!
//! On Linux:   `canIoCtl(canIOCTL_GET_EVENTHANDLE)` returns a file descriptor -> `poll()`.
//! On Windows: `canIoCtl(canIOCTL_GET_EVENTHANDLE)` returns a `HANDLE`         -> `WaitForSingleObject`.
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
pub struct ReceiveEvent {
    #[cfg(not(target_os = "windows"))]
    fd: i32,
    #[cfg(target_os = "windows")]
    event_handle: *mut c_void,
}

impl ReceiveEvent {
    /// Obtain the receive event for the given open channel handle.
    pub fn new(lib: &KvaserLibrary, handle: i32) -> Result<Self, KvaserError> {
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
    pub fn wait(&self, timeout: Option<Duration>) -> Result<bool, KvaserError> {
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
        // SAFETY: io_ctl was loaded from canlib; handle is valid; fd is a valid stack-allocated i32
        let status = unsafe {
            (lib.io_ctl)(
                handle,
                CAN_IOCTL_GET_EVENTHANDLE,
                std::ptr::from_mut(&mut fd).cast::<c_void>(),
                #[allow(clippy::cast_possible_truncation)]
                {
                    std::mem::size_of::<i32>() as u32
                },
            )
        };
        crate::error::check_status(status)?;
        Ok(Self { fd })
    }

    #[cfg(not(target_os = "windows"))]
    #[allow(clippy::cast_possible_truncation)] // timeout_ms clamped to i32::MAX
    fn wait_unix(&self, timeout: Option<Duration>) -> Result<bool, KvaserError> {
        let timeout_ms = timeout.map_or(-1, |d| d.as_millis().min(i32::MAX as u128) as i32);

        let mut pfd = libc::pollfd {
            fd: self.fd,
            events: libc::POLLIN,
            revents: 0,
        };

        // SAFETY: pfd is a valid stack-allocated pollfd; nfds=1
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
        // SAFETY: io_ctl was loaded from canlib; handle is valid; event_handle is stack-allocated
        let status = unsafe {
            (lib.io_ctl)(
                handle,
                CAN_IOCTL_GET_EVENTHANDLE,
                std::ptr::from_mut(&mut event_handle).cast::<c_void>(),
                #[allow(clippy::cast_possible_truncation)]
                {
                    std::mem::size_of::<*mut c_void>() as u32
                },
            )
        };
        crate::error::check_status(status)?;
        Ok(Self { event_handle })
    }

    #[cfg(target_os = "windows")]
    fn wait_windows(&self, timeout: Option<Duration>) -> Result<bool, KvaserError> {
        use windows_sys::Win32::Foundation::{WAIT_OBJECT_0, WAIT_TIMEOUT};
        use windows_sys::Win32::System::Threading::WaitForSingleObject;

        #[allow(clippy::cast_possible_truncation)] // clamped to u32::MAX
        let ms = timeout.map_or(0xFFFF_FFFF, |d| {
            d.as_millis().min(u128::from(u32::MAX)) as u32
        });

        // SAFETY: event_handle was obtained from canIoCtl
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
// across threads. On Unix the fd is a plain integer - also Send-safe.
unsafe impl Send for ReceiveEvent {}

// The event handle / fd is owned by the CANlib driver - no cleanup needed.
