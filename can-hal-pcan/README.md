<div align="center">
  <h1>can-hal-pcan</h1>
  <p>
    <strong>PCAN-Basic backend for <a href="https://crates.io/crates/can-hal-rs">can-hal-rs</a></strong>
  </p>
  <p>

[![crates.io](https://img.shields.io/crates/v/can-hal-pcan?label=latest)](https://crates.io/crates/can-hal-pcan)
[![Documentation](https://docs.rs/can-hal-pcan/badge.svg)](https://docs.rs/can-hal-pcan)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.81+-ab6000.svg)
![License](https://img.shields.io/crates/l/can-hal-pcan.svg)
[![CI](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml)

  </p>
</div>

Implements `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, and `BusStatus` using the PCAN-Basic API from Peak System. The channel builder uses typestate so that classic vs FD paths are tracked at compile time and invalid combinations are compile errors.

Supports USB, PCI, and LAN PCAN interfaces on Windows and Linux.

## Usage

```rust,no_run
use can_hal::{CanId, CanFrame, Transmit, Receive};
use can_hal_pcan::{PcanDriver, ClassicBitrate};

let driver = PcanDriver::new().expect("PCAN-Basic library not found");
let mut channel = driver
    .channel(0)
    .unwrap()
    .classic(ClassicBitrate::Br500K)
    .connect()
    .unwrap();

let id = CanId::new_standard(0x123).unwrap();
let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
channel.transmit(&frame).unwrap();

let response = channel.receive().unwrap();
println!("{:?}", response.frame());
```

Classic bitrate is a `ClassicBitrate` enum so invalid values aren't representable; `.classic(...)` is infallible.

## CAN FD

`.fd(nominal_hz, data_hz)` validates that both bitrates evenly divide the 80 MHz PCAN clock; on success the builder transitions to FD state and exposes sample-point overrides. Defaults are 70% nominal and 80% data; override via `.sample_point()` / `.data_sample_point()`:

```rust,no_run
use can_hal::{TransmitFd, CanId, CanFdFrame};
use can_hal_pcan::PcanDriver;

let driver = PcanDriver::new().unwrap();
let mut channel = driver
    .channel(0)
    .unwrap()
    .fd(500_000, 4_000_000)
    .unwrap()
    .sample_point(0.75)
    .connect()
    .unwrap();
```

For raw control over per-segment timing, use `fd_timing()` with a `PcanFdTiming` value (e.g., for unusually large SJW values).

## Prerequisites

The PCAN-Basic library must be installed:

- **Windows**: Download from [Peak System](https://www.peak-system.com/PCAN-Basic.239.0.html). Ensure `PCANBasic.dll` is in the system PATH.
- **Linux**: Build and install from the [PCAN-Basic Linux](https://www.peak-system.com/PCAN-Basic-Linux.433.0.html) package (`libpcanbasic.so`).

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
