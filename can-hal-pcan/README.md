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

Implements `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `BusStatus`, `Driver`, and `ChannelBuilder` using the PCAN-Basic API from Peak System.

Supports USB, PCI, and LAN PCAN interfaces on Windows and Linux.

## Usage

```rust,no_run
use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder};
use can_hal_pcan::PcanDriver;

let driver = PcanDriver::new().expect("PCAN-Basic library not found");
let mut channel = driver
    .channel(0)
    .unwrap()
    .bitrate(500_000)
    .unwrap()
    .connect()
    .unwrap();

let id = CanId::new_standard(0x123).unwrap();
let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
channel.transmit(&frame).unwrap();

let response = channel.receive().unwrap();
println!("{:?}", response.frame());
```

## CAN FD

CAN FD initialization requires detailed timing parameters. Use the backend-specific `fd_timing_string()` method:

```rust,no_run
use can_hal::{ChannelBuilder, TransmitFd, CanId, CanFdFrame};
use can_hal_pcan::PcanDriver;

let driver = PcanDriver::new().unwrap();
let mut channel = driver
    .channel(0)
    .unwrap()
    .fd_timing_string(
        "f_clock_mhz=80, nom_brp=1, nom_tseg1=63, nom_tseg2=16, \
         nom_sjw=16, data_brp=1, data_tseg1=7, data_tseg2=2, data_sjw=2"
    )
    .unwrap()
    .connect()
    .unwrap();
```

## Prerequisites

The PCAN-Basic library must be installed:

- **Windows**: Download from [Peak System](https://www.peak-system.com/PCAN-Basic.239.0.html). Ensure `PCANBasic.dll` is in the system PATH.
- **Linux**: Build and install from the [PCAN-Basic Linux](https://www.peak-system.com/PCAN-Basic-Linux.433.0.html) package (`libpcanbasic.so`).

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
