//! PCAN driver and channel builder.

use std::ffi::CString;
use std::sync::Arc;

use can_hal::driver::{ChannelBuilder, Driver, DriverFd};

use crate::channel::PcanChannel;
use crate::error::{check_status, PcanError};
use crate::ffi;
use crate::library::PcanLibrary;

/// Bus type for selecting a PCAN hardware family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PcanBusType {
    Usb,
    Pci,
    Lan,
}

/// PCAN-Basic driver — factory for opening CAN channels.
///
/// The default [`Driver::channel`] method maps to USB channels. For PCI or
/// LAN channels, use [`channel_on_bus`](PcanDriver::channel_on_bus).
///
/// # Example
///
/// ```rust,ignore
/// use can_hal::{Driver, ChannelBuilder};
/// use can_hal_pcan::PcanDriver;
///
/// let driver = PcanDriver::new()?;
/// let mut channel = driver
///     .channel(0)?          // first USB device
///     .bitrate(500_000)?
///     .connect()?;
/// ```
pub struct PcanDriver {
    lib: Arc<PcanLibrary>,
}

impl PcanDriver {
    /// Create a new PCAN driver by loading the PCAN-Basic library from the
    /// default system path (`PCANBasic.dll` on Windows, `libpcanbasic.so` on
    /// Linux).
    pub fn new() -> Result<Self, PcanError> {
        let lib = PcanLibrary::load()?;
        Ok(PcanDriver { lib })
    }

    /// Create a new PCAN driver by loading the library from a custom path.
    pub fn with_library_path(path: &str) -> Result<Self, PcanError> {
        let lib = PcanLibrary::load_from(path)?;
        Ok(PcanDriver { lib })
    }

    /// Begin configuring a channel on a specific bus type.
    ///
    /// `index` is 0-based (0 through 15).
    pub fn channel_on_bus(
        &self,
        bus_type: PcanBusType,
        index: u32,
    ) -> Result<PcanChannelBuilder, PcanError> {
        let bus_code = match bus_type {
            PcanBusType::Usb => 0,
            PcanBusType::Pci => 1,
            PcanBusType::Lan => 2,
        };
        let handle =
            ffi::pcan_handle(bus_code, index as u16).ok_or(PcanError::InvalidChannel(index))?;

        Ok(PcanChannelBuilder {
            lib: self.lib.clone(),
            handle,
            bitrate: None,
            data_bitrate: None,
            sample_point: None,
            fd_timing_string: None,
        })
    }
}

impl Driver for PcanDriver {
    type Channel = PcanChannel;
    type Builder = PcanChannelBuilder;
    type Error = PcanError;

    /// Begin configuring a USB channel by 0-based index.
    fn channel(&self, index: u32) -> Result<Self::Builder, Self::Error> {
        self.channel_on_bus(PcanBusType::Usb, index)
    }
}

impl DriverFd for PcanDriver {}

/// Builder for configuring a PCAN channel before going on-bus.
///
/// For classic CAN, use [`bitrate()`](ChannelBuilder::bitrate) with a
/// standard value (500000, 250000, etc.).
///
/// For CAN FD, use [`fd_timing_string()`](PcanChannelBuilder::fd_timing_string)
/// to provide the raw PCAN timing parameter string, since FD initialization
/// requires detailed timing parameters that cannot be derived from a simple
/// bitrate value.
///
/// # CAN 2.0 Example
///
/// ```rust,ignore
/// use can_hal::ChannelBuilder;
/// use can_hal_pcan::PcanDriver;
///
/// let driver = PcanDriver::new()?;
/// let channel = driver.channel(0)?
///     .bitrate(500_000)?
///     .connect()?;
/// ```
///
/// # CAN FD Example
///
/// ```rust,ignore
/// use can_hal::ChannelBuilder;
/// use can_hal_pcan::PcanDriver;
///
/// let driver = PcanDriver::new()?;
/// let channel = driver.channel(0)?
///     .fd_timing_string(
///         "f_clock_mhz=80, nom_brp=1, nom_tseg1=63, nom_tseg2=16, \
///          nom_sjw=16, data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2"
///     )?
///     .connect()?;
/// ```
pub struct PcanChannelBuilder {
    lib: Arc<PcanLibrary>,
    handle: u16,
    bitrate: Option<u16>,
    data_bitrate: Option<u32>,
    sample_point: Option<f32>,
    fd_timing_string: Option<String>,
}

impl PcanChannelBuilder {
    /// Set an FD timing parameter string for `CAN_InitializeFD`.
    ///
    /// This is a backend-specific method not part of the [`ChannelBuilder`]
    /// trait. When set, [`connect()`](ChannelBuilder::connect) will use
    /// `CAN_InitializeFD` instead of `CAN_Initialize`.
    ///
    /// Format: `"f_clock_mhz=80, nom_brp=1, nom_tseg1=63, nom_tseg2=16,
    ///           nom_sjw=16, data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2"`
    pub fn fd_timing_string(mut self, timing: &str) -> Result<Self, PcanError> {
        self.fd_timing_string = Some(timing.to_string());
        Ok(self)
    }
}

impl ChannelBuilder for PcanChannelBuilder {
    type Channel = PcanChannel;
    type Error = PcanError;

    fn bitrate(mut self, bitrate: u32) -> Result<Self, Self::Error> {
        let pcan_baud =
            ffi::bitrate_to_pcan(bitrate).ok_or(PcanError::UnsupportedBitrate(bitrate))?;
        self.bitrate = Some(pcan_baud);
        Ok(self)
    }

    fn data_bitrate(mut self, bitrate: u32) -> Result<Self, Self::Error> {
        self.data_bitrate = Some(bitrate);
        Ok(self)
    }

    fn sample_point(mut self, sample_point: f32) -> Result<Self, Self::Error> {
        self.sample_point = Some(sample_point);
        Ok(self)
    }

    fn connect(self) -> Result<Self::Channel, Self::Error> {
        if let Some(ref timing) = self.fd_timing_string {
            // FD initialization — data_bitrate and sample_point are ignored
            // because the fd_timing_string contains the full timing parameters.
            let _ = self.data_bitrate;
            let _ = self.sample_point;

            let c_timing = CString::new(timing.as_str())
                .map_err(|_| PcanError::InvalidFrame("timing string contains null byte".into()))?;

            let status = unsafe { (self.lib.initialize_fd)(self.handle, c_timing.as_ptr()) };
            check_status(status)?;
            PcanChannel::new(self.lib, self.handle, true)
        } else {
            // Classic CAN initialization — sample_point is not supported
            // by PCAN-Basic's CAN_Initialize (fixed at the hardware default).
            let _ = self.sample_point;
            let baud = self.bitrate.ok_or(PcanError::UnsupportedBitrate(0))?;

            // Plug & Play hardware (USB, PCI, LAN): hw_type=0, io_port=0, interrupt=0
            let status = unsafe { (self.lib.initialize)(self.handle, baud, 0, 0, 0) };
            check_status(status)?;
            PcanChannel::new(self.lib, self.handle, false)
        }
    }
}
