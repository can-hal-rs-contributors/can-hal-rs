use std::sync::Arc;

use libloading::Library;

use crate::error::KvaserError;
use crate::ffi::*;

/// Holds the dynamically loaded CANlib library and all resolved function pointers.
///
/// Shared via `Arc` so that `KvaserChannel` instances can keep the library alive.
pub struct KvaserLibrary {
    // The library handle must be kept alive as long as any function pointer is in use.
    _lib: Library,
    pub(crate) initialize_library: FnInitializeLibrary,
    pub(crate) open_channel: FnOpenChannel,
    pub(crate) close: FnClose,
    pub(crate) set_bus_params: FnSetBusParams,
    pub(crate) set_bus_params_fd: FnSetBusParamsFd,
    pub(crate) set_bus_params_fd_tq: Option<FnSetBusParamsFdTq>,
    pub(crate) get_channel_data: Option<FnGetChannelData>,
    pub(crate) bus_on: FnBusOn,
    pub(crate) bus_off: FnBusOff,
    pub(crate) write: FnWrite,
    pub(crate) write_sync: FnWriteSync,
    pub(crate) read: FnRead,
    pub(crate) accept: FnAccept,
    pub(crate) read_status: FnReadStatus,
    pub(crate) read_error_counters: FnReadErrorCounters,
    pub(crate) io_ctl: FnIoCtl,
}

#[cfg(unix)]
const fn default_library_name() -> &'static str {
    // Use the versioned SONAME to avoid relying on the development symlink.
    "libcanlib.so.1"
}

#[cfg(windows)]
const fn default_library_name() -> &'static str {
    "canlib32.dll"
}

impl KvaserLibrary {
    /// Load CANlib from the default system location.
    pub fn load() -> Result<Arc<Self>, KvaserError> {
        Self::load_from(default_library_name())
    }

    /// Load CANlib from a custom path.
    #[allow(clippy::multiple_unsafe_ops_per_block)]
    pub fn load_from(path: &str) -> Result<Arc<Self>, KvaserError> {
        // SAFETY: All operations are dlsym lookups from a valid library handle.
        // Each symbol name matches the CANlib C API signature.
        unsafe {
            let lib = Library::new(path)?;

            let initialize_library = *lib.get::<FnInitializeLibrary>(b"canInitializeLibrary\0")?;
            let open_channel = *lib.get::<FnOpenChannel>(b"canOpenChannel\0")?;
            let close = *lib.get::<FnClose>(b"canClose\0")?;
            let set_bus_params = *lib.get::<FnSetBusParams>(b"canSetBusParams\0")?;
            let set_bus_params_fd = *lib.get::<FnSetBusParamsFd>(b"canSetBusParamsFd\0")?;
            let bus_on = *lib.get::<FnBusOn>(b"canBusOn\0")?;
            let bus_off = *lib.get::<FnBusOff>(b"canBusOff\0")?;
            let write = *lib.get::<FnWrite>(b"canWrite\0")?;
            let write_sync = *lib.get::<FnWriteSync>(b"canWriteSync\0")?;
            let read = *lib.get::<FnRead>(b"canRead\0")?;
            let accept = *lib.get::<FnAccept>(b"canAccept\0")?;
            let read_status = *lib.get::<FnReadStatus>(b"canReadStatus\0")?;
            let read_error_counters = *lib.get::<FnReadErrorCounters>(b"canReadErrorCounters\0")?;
            let io_ctl = *lib.get::<FnIoCtl>(b"canIoCtl\0")?;

            // Optional: older CANlib builds don't export it. open_fd falls
            // back to canSetBusParams + canSetBusParamsFd when it's absent
            // or refused for a channel.
            let set_bus_params_fd_tq = lib
                .get::<FnSetBusParamsFdTq>(b"canSetBusParamsFdTq\0")
                .ok()
                .map(|sym| *sym);

            // Optional: used only to ask for the device's clock. Without
            // it the clock is unknown, and timing is computed for
            // FALLBACK_CLOCK_HZ (classic tries an 8 MHz timing first).
            let get_channel_data = lib
                .get::<FnGetChannelData>(b"canGetChannelData\0")
                .ok()
                .map(|sym| *sym);

            let kvaser_lib = Arc::new(Self {
                _lib: lib,
                initialize_library,
                open_channel,
                close,
                set_bus_params,
                set_bus_params_fd,
                set_bus_params_fd_tq,
                get_channel_data,
                bus_on,
                bus_off,
                write,
                write_sync,
                read,
                accept,
                read_status,
                read_error_counters,
                io_ctl,
            });

            // CANlib requires this to be called once before any other API call.
            (kvaser_lib.initialize_library)();

            Ok(kvaser_lib)
        }
    }
}
