# can-hal-kvaser

KVASER CANlib backend for [`can-hal-rs`](https://crates.io/crates/can-hal-rs).

Implements `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `BusStatus`, `Driver`, and `ChannelBuilder` using the CANlib API from KVASER.

Supports USB, PCIe, and LAN KVASER interfaces on Windows and Linux.

## Usage

```rust,no_run
use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder};
use can_hal_kvaser::KvaserDriver;

let driver = KvaserDriver::new().expect("CANlib library not found");
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

```rust,no_run
use can_hal::{ChannelBuilder, TransmitFd, CanId, CanFdFrame};
use can_hal_kvaser::KvaserDriver;

let driver = KvaserDriver::new().unwrap();
let mut channel = driver
    .channel(0)
    .unwrap()
    .bitrate(500_000)
    .unwrap()
    .data_bitrate(2_000_000)
    .unwrap()
    .connect()
    .unwrap();

let id = CanId::new_standard(0x123).unwrap();
let frame = CanFdFrame::new(id, &[0xDE, 0xAD], true, false).unwrap();
channel.transmit_fd(&frame).unwrap();
```

## Prerequisites

The CANlib library must be installed:

- **Windows**: Download from [KVASER](https://www.kvaser.com/download/). Ensure `canlib32.dll` is in the system PATH.
- **Linux**: Install the KVASER Linux drivers and CANlib from [KVASER Linux](https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_inifile=canlib). The library installs as `libcanlib.so.1`.

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
