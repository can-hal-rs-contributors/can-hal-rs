//! Runtime loading of the PCAN-Basic shared library.

use std::sync::Arc;

use libloading::Library;

use crate::error::PcanError;
use crate::ffi;

/// Holds a loaded PCAN-Basic library and its resolved function pointers.
///
/// Wrapped in [`Arc`] so that channels can share the loaded library with the
/// driver that created them. The library stays alive as long as any channel
/// or driver references it.
#[allow(dead_code)]
pub struct PcanLibrary {
    _lib: Library,
    pub(crate) initialize: ffi::FnInitialize,
    pub(crate) initialize_fd: ffi::FnInitializeFD,
    pub(crate) uninitialize: ffi::FnUninitialize,
    pub(crate) read: ffi::FnRead,
    pub(crate) read_fd: ffi::FnReadFD,
    pub(crate) write: ffi::FnWrite,
    pub(crate) write_fd: ffi::FnWriteFD,
    pub(crate) filter_messages: ffi::FnFilterMessages,
    pub(crate) get_status: ffi::FnGetStatus,
    pub(crate) get_value: ffi::FnGetValue,
    pub(crate) set_value: ffi::FnSetValue,
}

impl PcanLibrary {
    /// Load the PCAN-Basic library from the default system path.
    ///
    /// - Windows: `PCANBasic.dll`
    /// - Linux: `libpcanbasic.so`
    pub fn load() -> Result<Arc<Self>, PcanError> {
        Self::load_from(default_library_name())
    }

    /// Load the PCAN-Basic library from a specific path or filename.
    pub fn load_from(path: &str) -> Result<Arc<Self>, PcanError> {
        // SAFETY: Loading a shared library can execute arbitrary code (DllMain
        // on Windows, constructor functions on Linux). This is inherent to
        // dynamic loading and is the caller's responsibility.
        unsafe {
            let lib = Library::new(path)?;

            let initialize = *lib.get::<ffi::FnInitialize>(b"CAN_Initialize\0")?;
            let initialize_fd = *lib.get::<ffi::FnInitializeFD>(b"CAN_InitializeFD\0")?;
            let uninitialize = *lib.get::<ffi::FnUninitialize>(b"CAN_Uninitialize\0")?;
            let read = *lib.get::<ffi::FnRead>(b"CAN_Read\0")?;
            let read_fd = *lib.get::<ffi::FnReadFD>(b"CAN_ReadFD\0")?;
            let write = *lib.get::<ffi::FnWrite>(b"CAN_Write\0")?;
            let write_fd = *lib.get::<ffi::FnWriteFD>(b"CAN_WriteFD\0")?;
            let filter_messages = *lib.get::<ffi::FnFilterMessages>(b"CAN_FilterMessages\0")?;
            let get_status = *lib.get::<ffi::FnGetStatus>(b"CAN_GetStatus\0")?;
            let get_value = *lib.get::<ffi::FnGetValue>(b"CAN_GetValue\0")?;
            let set_value = *lib.get::<ffi::FnSetValue>(b"CAN_SetValue\0")?;

            Ok(Arc::new(PcanLibrary {
                _lib: lib,
                initialize,
                initialize_fd,
                uninitialize,
                read,
                read_fd,
                write,
                write_fd,
                filter_messages,
                get_status,
                get_value,
                set_value,
            }))
        }
    }
}

fn default_library_name() -> &'static str {
    #[cfg(target_os = "windows")]
    {
        "PCANBasic.dll"
    }

    #[cfg(not(target_os = "windows"))]
    {
        "libpcanbasic.so"
    }
}
