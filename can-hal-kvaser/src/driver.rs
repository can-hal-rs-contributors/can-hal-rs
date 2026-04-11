use std::os::raw::c_long;
use std::sync::Arc;

use can_hal::{ChannelBuilder, Driver, DriverFd};

use crate::channel::KvaserChannel;
use crate::error::{check_status, KvaserError};
use crate::event::ReceiveEvent;
use crate::ffi::{KvBusParamsTq, CAN_ERR_NOT_SUPPORTED, CAN_OPEN_CAN_FD};
use crate::library::KvaserLibrary;

/// Assumed CAN controller clock frequency.
///
/// 80 MHz is standard on Kvaser U100, Leaf Pro HS v2, and most modern Kvaser
/// adapters. Used to compute the prescaler for the `canSetBusParamsFdTq` API.
const CLOCK_HZ: u32 = 80_000_000;

/// Nominal bus parameters: (tseg1, tseg2, sjw, noSamp, syncMode).
#[derive(Debug, Clone, Copy)]
pub struct BusParams {
    pub tseg1: u32,
    pub tseg2: u32,
    pub sjw: u32,
    pub no_samp: u32,
    pub sync_mode: u32,
}

/// FD data-phase bus parameters: (tseg1, tseg2, sjw).
#[derive(Debug, Clone, Copy)]
pub struct BusParamsFd {
    pub tseg1: u32,
    pub tseg2: u32,
    pub sjw: u32,
}

/// Default nominal timing parameters for common bitrates.
///
/// These assume an 80 MHz CAN clock (Kvaser U100 and most modern Kvaser
/// adapters). The hardware derives the prescaler from `freq / (1 + tseg1 + tseg2)`.
fn default_nominal_params(bitrate_hz: u32) -> BusParams {
    // 20 TQ (1 + tseg1 + tseg2), 70% sample point, SJW=4 for good
    // resynchronization tolerance. These values are verified on Kvaser U100
    // and provide broad compatibility with other CAN FD adapters.
    match bitrate_hz {
        1_000_000 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        500_000 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        250_000 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        125_000 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        100_000 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        83_333 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        50_000 => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
        // Fallback: 20 TQ, 70% sample point.
        _ => BusParams {
            tseg1: 13,
            tseg2: 6,
            sjw: 4,
            no_samp: 1,
            sync_mode: 0,
        }, // 70.0%
    }
}

/// Default FD data-phase timing parameters for common data bitrates.
fn default_fd_params(data_bitrate_hz: u32) -> BusParamsFd {
    match data_bitrate_hz {
        5_000_000 => BusParamsFd {
            tseg1: 5,
            tseg2: 2,
            sjw: 2,
        }, // 75.0%
        4_000_000 => BusParamsFd {
            tseg1: 7,
            tseg2: 2,
            sjw: 2,
        }, // 80.0%
        2_000_000 => BusParamsFd {
            tseg1: 7,
            tseg2: 2,
            sjw: 2,
        }, // 80.0%
        1_000_000 => BusParamsFd {
            tseg1: 7,
            tseg2: 2,
            sjw: 2,
        }, // 80.0%
        // Fallback: 10 TQ, 80% sample point
        _ => BusParamsFd {
            tseg1: 7,
            tseg2: 2,
            sjw: 2,
        }, // 80.0%
    }
}

/// Driver for KVASER CAN adapters using the CANlib API.
///
/// Loads `libcanlib.so.1` (Linux) or `canlib32.dll` (Windows) at construction time.
///
/// # Example
///
/// ```rust,no_run
/// use can_hal::{ChannelBuilder, Driver};
/// use can_hal_kvaser::KvaserDriver;
///
/// let driver = KvaserDriver::new().expect("CANlib not found");
/// let mut channel = driver.channel(0).unwrap().bitrate(500_000).unwrap().connect().unwrap();
/// ```
pub struct KvaserDriver {
    lib: Arc<KvaserLibrary>,
}

impl KvaserDriver {
    /// Load CANlib from the default system location.
    pub fn new() -> Result<Self, KvaserError> {
        Ok(KvaserDriver {
            lib: KvaserLibrary::load()?,
        })
    }

    /// Load CANlib from a custom path.
    pub fn with_library_path(path: &str) -> Result<Self, KvaserError> {
        Ok(KvaserDriver {
            lib: KvaserLibrary::load_from(path)?,
        })
    }
}

impl Driver for KvaserDriver {
    type Channel = KvaserChannel;
    type Builder = KvaserChannelBuilder;
    type Error = KvaserError;

    /// Begin configuring the channel at the given 0-based index.
    fn channel(&self, index: u32) -> Result<KvaserChannelBuilder, KvaserError> {
        Ok(KvaserChannelBuilder {
            lib: Arc::clone(&self.lib),
            channel_index: index as i32,
            bitrate_hz: None,
            fd_bitrate_hz: None,
            custom_params: None,
            custom_fd_params: None,
        })
    }
}

impl DriverFd for KvaserDriver {}

/// Builder for configuring a KVASER channel before going on-bus.
pub struct KvaserChannelBuilder {
    pub(crate) lib: Arc<KvaserLibrary>,
    pub(crate) channel_index: i32,
    pub(crate) bitrate_hz: Option<u32>,
    pub(crate) fd_bitrate_hz: Option<u32>,
    custom_params: Option<BusParams>,
    custom_fd_params: Option<BusParamsFd>,
}

impl KvaserChannelBuilder {
    /// Set explicit nominal bus timing parameters.
    ///
    /// Overrides the defaults chosen by [`bitrate()`](ChannelBuilder::bitrate).
    /// Call this after `bitrate()` to keep the frequency but use custom timing.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use can_hal_kvaser::{BusParams, KvaserDriver};
    ///
    /// let channel = driver.channel(0)?
    ///     .bitrate(500_000)?
    ///     .bus_params(BusParams { tseg1: 13, tseg2: 2, sjw: 2, no_samp: 1, sync_mode: 0 })
    ///     .connect()?;
    /// ```
    pub fn bus_params(mut self, params: BusParams) -> Self {
        self.custom_params = Some(params);
        self
    }

    /// Set explicit FD data-phase bus timing parameters.
    ///
    /// Overrides the defaults chosen by [`data_bitrate()`](ChannelBuilder::data_bitrate).
    pub fn bus_params_fd(mut self, params: BusParamsFd) -> Self {
        self.custom_fd_params = Some(params);
        self
    }
}

impl ChannelBuilder for KvaserChannelBuilder {
    type Channel = KvaserChannel;
    type Error = KvaserError;

    fn bitrate(mut self, hz: u32) -> Result<Self, KvaserError> {
        self.bitrate_hz = Some(hz);
        Ok(self)
    }

    fn data_bitrate(mut self, hz: u32) -> Result<Self, KvaserError> {
        self.fd_bitrate_hz = Some(hz);
        Ok(self)
    }

    fn sample_point(self, _sample_point: f32) -> Result<Self, KvaserError> {
        Err(KvaserError::NotSupported(
            "sample_point() is not supported; use bus_params() for custom timing".into(),
        ))
    }

    fn connect(self) -> Result<KvaserChannel, KvaserError> {
        let bitrate_hz = self.bitrate_hz.ok_or_else(|| {
            KvaserError::NotSupported("bitrate() must be called before connect()".into())
        })?;

        let mut flags = 0i32;
        if self.fd_bitrate_hz.is_some() {
            flags |= CAN_OPEN_CAN_FD;
        }

        let handle = unsafe { (self.lib.open_channel)(self.channel_index, flags) };
        if handle < 0 {
            return Err(KvaserError::Canlib(crate::error::KvaserStatus(handle)));
        }

        // Close the handle on any subsequent failure to avoid a resource leak.
        let result = (|| {
            let params = self
                .custom_params
                .unwrap_or_else(|| default_nominal_params(bitrate_hz));

            if let Some(fd_hz) = self.fd_bitrate_hz {
                let fd_params = self
                    .custom_fd_params
                    .unwrap_or_else(|| default_fd_params(fd_hz));

                // Try canSetBusParamsFdTq first — it sets both nominal and
                // data-phase timing in one call using explicit time quanta and
                // works identically on Windows and Linux.
                let use_legacy = if let Some(set_fd_tq) = self.lib.set_bus_params_fd_tq {
                    let nominal_tq = to_nominal_tq(bitrate_hz, &params)?;
                    let data_tq = to_data_tq(fd_hz, &fd_params)?;
                    let status = unsafe { (set_fd_tq)(handle, nominal_tq, data_tq) };
                    if status >= 0 {
                        false // TQ succeeded
                    } else if status == CAN_ERR_NOT_SUPPORTED {
                        // Some linuxcan versions export the symbol but return
                        // canERR_NOT_SUPPORTED for certain hardware/firmware.
                        true
                    } else {
                        check_status(status)?;
                        unreachable!()
                    }
                } else {
                    true // symbol not present
                };

                if use_legacy {
                    // Fallback: canSetBusParams + canSetBusParamsFd.
                    check_status(unsafe {
                        (self.lib.set_bus_params)(
                            handle,
                            bitrate_hz as c_long,
                            params.tseg1,
                            params.tseg2,
                            params.sjw,
                            params.no_samp,
                            params.sync_mode,
                        )
                    })?;
                    check_status(unsafe {
                        (self.lib.set_bus_params_fd)(
                            handle,
                            fd_hz as c_long,
                            fd_params.tseg1,
                            fd_params.tseg2,
                            fd_params.sjw,
                        )
                    })?;
                }
            } else {
                // Classic CAN — canSetBusParams works on all platforms.
                check_status(unsafe {
                    (self.lib.set_bus_params)(
                        handle,
                        bitrate_hz as c_long,
                        params.tseg1,
                        params.tseg2,
                        params.sjw,
                        params.no_samp,
                        params.sync_mode,
                    )
                })?;
            }

            check_status(unsafe { (self.lib.bus_on)(handle) })?;

            let event = ReceiveEvent::new(&self.lib, handle)?;

            Ok(KvaserChannel {
                lib: self.lib.clone(),
                handle,
                fd_mode: self.fd_bitrate_hz.is_some(),
                event,
            })
        })();

        if result.is_err() {
            unsafe { (self.lib.close)(handle) };
        }

        result
    }
}

// ---------------------------------------------------------------------------
// TQ conversion helpers
// ---------------------------------------------------------------------------

/// Convert nominal bus parameters + bitrate to a `KvBusParamsTq`.
///
/// For the nominal (arbitration) phase we set `prop = 0` and `phase1 = tseg1`.
/// This is valid for short-cable USB setups; the propagation segment only
/// matters on long buses where signal delay is significant.
fn to_nominal_tq(bitrate_hz: u32, p: &BusParams) -> Result<KvBusParamsTq, KvaserError> {
    let tq = 1 + p.tseg1 + p.tseg2;
    let prescaler = compute_prescaler(bitrate_hz, tq)?;
    Ok(KvBusParamsTq {
        tq: tq as i32,
        phase1: p.tseg1 as i32,
        phase2: p.tseg2 as i32,
        sjw: p.sjw as i32,
        prop: 0,
        prescaler,
    })
}

/// Convert FD data-phase bus parameters + bitrate to a `KvBusParamsTq`.
///
/// The data phase requires `prop = 0`.
fn to_data_tq(bitrate_hz: u32, p: &BusParamsFd) -> Result<KvBusParamsTq, KvaserError> {
    let tq = 1 + p.tseg1 + p.tseg2;
    let prescaler = compute_prescaler(bitrate_hz, tq)?;
    Ok(KvBusParamsTq {
        tq: tq as i32,
        phase1: p.tseg1 as i32,
        phase2: p.tseg2 as i32,
        sjw: p.sjw as i32,
        prop: 0,
        prescaler,
    })
}

/// Derive the prescaler from the clock, bitrate, and total time quanta.
///
/// Returns an error if the bitrate cannot be achieved exactly with the given
/// TQ count at an 80 MHz clock.
fn compute_prescaler(bitrate_hz: u32, tq: u32) -> Result<i32, KvaserError> {
    let bit_time = (bitrate_hz as u64) * (tq as u64);
    if bit_time == 0 || !(CLOCK_HZ as u64).is_multiple_of(bit_time) {
        return Err(KvaserError::NotSupported(format!(
            "cannot achieve {bitrate_hz} Hz with {tq} TQ at {CLOCK_HZ} Hz clock \
             (prescaler would be non-integer)"
        )));
    }
    Ok((CLOCK_HZ as u64 / bit_time) as i32)
}
