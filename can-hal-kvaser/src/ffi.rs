use std::ffi::c_void;
use std::os::raw::{c_long, c_ulong};

pub type CanHandle = i32;
pub type CanStatus = i32;

// Return codes
pub const CAN_OK: i32 = 0;
pub const CAN_ERR_NOMSG: i32 = -2;

// canOpenChannel flags
pub const CAN_OPEN_CAN_FD: i32 = 0x0400;

// Message flags
pub const CAN_MSG_STD: u32 = 0x0002;
pub const CAN_MSG_EXT: u32 = 0x0004;
pub const CAN_MSG_RTR: u32 = 0x0001;
pub const CAN_MSG_FDF: u32 = 0x010000;
pub const CAN_MSG_BRS: u32 = 0x020000;
pub const CAN_MSG_ESI: u32 = 0x040000;
pub const CAN_MSG_ERROR_FRAME: u32 = 0x0020;

// canReadStatus flags (from canstat.h)
pub const CAN_STAT_ERROR_PASSIVE: c_ulong = 0x0000_0001;
pub const CAN_STAT_BUS_OFF: c_ulong = 0x0000_0002;
pub const CAN_STAT_ERROR_WARNING: c_ulong = 0x0000_0004;

// canAccept flag constants (unsigned int in CANlib API)
pub const CAN_FILTER_SET_CODE_STD: u32 = 3;
pub const CAN_FILTER_SET_MASK_STD: u32 = 4;
pub const CAN_FILTER_SET_CODE_EXT: u32 = 5;
pub const CAN_FILTER_SET_MASK_EXT: u32 = 6;

// canIoCtl function codes
pub const CAN_IOCTL_GET_EVENTHANDLE: u32 = 6;

// Function pointer types for dynamically loaded CANlib symbols.
//
// Parameter types follow the CANlib C API exactly:
//   - `long` / `unsigned long` → c_long / c_ulong  (64-bit on Linux, 32-bit on Windows)
//   - `int`  / `unsigned int`  → i32 / u32

pub type FnInitializeLibrary = unsafe extern "C" fn();

pub type FnOpenChannel = unsafe extern "C" fn(channel: CanHandle, flags: i32) -> CanHandle;

pub type FnClose = unsafe extern "C" fn(hnd: CanHandle) -> CanStatus;

pub type FnSetBusParams = unsafe extern "C" fn(
    hnd: CanHandle,
    freq: c_long,
    tseg1: u32,
    tseg2: u32,
    sjw: u32,
    no_samp: u32,
    sync_mode: u32,
) -> CanStatus;

pub type FnSetBusParamsFd = unsafe extern "C" fn(
    hnd: CanHandle,
    freq_brs: c_long,
    tseg1: u32,
    tseg2: u32,
    sjw: u32,
) -> CanStatus;

pub type FnBusOn = unsafe extern "C" fn(hnd: CanHandle) -> CanStatus;

pub type FnBusOff = unsafe extern "C" fn(hnd: CanHandle) -> CanStatus;

/// `canWrite(CanHandle, long id, void *msg, unsigned int dlc, unsigned int flag)`
pub type FnWrite = unsafe extern "C" fn(
    hnd: CanHandle,
    id: c_long,
    msg: *const c_void,
    dlc: u32,
    flag: u32,
) -> CanStatus;

pub type FnWriteSync = unsafe extern "C" fn(hnd: CanHandle, timeout: c_ulong) -> CanStatus;

/// `canRead(CanHandle, long *id, void *msg, unsigned int *dlc, unsigned int *flag, unsigned long *time)`
pub type FnRead = unsafe extern "C" fn(
    hnd: CanHandle,
    id: *mut c_long,
    msg: *mut c_void,
    dlc: *mut u32,
    flag: *mut u32,
    time: *mut c_ulong,
) -> CanStatus;

/// `canAccept(CanHandle, long envelope, unsigned int flag)`
pub type FnAccept = unsafe extern "C" fn(hnd: CanHandle, envelope: c_long, flag: u32) -> CanStatus;

/// `canReadStatus(CanHandle, unsigned long *flags)`
pub type FnReadStatus = unsafe extern "C" fn(hnd: CanHandle, flags: *mut c_ulong) -> CanStatus;

/// `canReadErrorCounters(CanHandle, unsigned int *txErr, unsigned int *rxErr, unsigned int *ovErr)`
pub type FnReadErrorCounters = unsafe extern "C" fn(
    hnd: CanHandle,
    tx_err: *mut u32,
    rx_err: *mut u32,
    overrun_err: *mut u32,
) -> CanStatus;

/// `canIoCtl(CanHandle, unsigned int func, void *buf, unsigned int buflen)`
pub type FnIoCtl =
    unsafe extern "C" fn(hnd: CanHandle, func: u32, buf: *mut c_void, buflen: u32) -> CanStatus;
