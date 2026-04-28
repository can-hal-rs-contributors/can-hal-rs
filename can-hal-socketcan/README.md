<div align="center">
  <h1>can-hal-socketcan</h1>
  <p>
    <strong>Linux SocketCAN backend for <a href="https://crates.io/crates/can-hal-rs">can-hal-rs</a></strong>
  </p>
  <p>

[![crates.io](https://img.shields.io/crates/v/can-hal-socketcan?label=latest)](https://crates.io/crates/can-hal-socketcan)
[![Documentation](https://docs.rs/can-hal-socketcan/badge.svg)](https://docs.rs/can-hal-socketcan)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.81+-ab6000.svg)
![License](https://img.shields.io/crates/l/can-hal-socketcan.svg)
[![CI](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml)

  </p>
</div>

Implements `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `Driver`, and `ChannelBuilder` using SocketCAN sockets.

## Usage

```rust,no_run
use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder};
use can_hal_socketcan::SocketCanDriver;

let driver = SocketCanDriver::new();
let mut channel = driver
    .channel_by_name("can0")
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

## Interface setup

SocketCAN bitrate is configured at the OS level, not through the socket API:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

For testing without hardware, use a virtual CAN interface:

```bash
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set vcan0 up
```

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
