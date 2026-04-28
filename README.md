<div align="center">
  <h1>can-hal</h1>
  <p>
    <strong>Hardware-agnostic CAN bus traits for Rust</strong>
  </p>
  <p>

[![CI](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.81+-ab6000.svg)
![License](https://img.shields.io/crates/l/can-hal-rs.svg)

  </p>
</div>

> **Warning:** These crates are unstable and under active development. APIs may change without notice. Not yet recommended for production use.

Backend implementations for Linux SocketCAN, PEAK PCAN, and KVASER adapters. The core `can-hal-rs` traits crate is `no_std`-compatible.

## Crates

| Crate | Description |
|---|---|
| [`can-hal-rs`](can-hal/) | Core traits: `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Driver`, `ChannelBuilder`, `Filterable`, `BusStatus` |
| [`can-hal-socketcan`](can-hal-socketcan/) | Linux SocketCAN backend |
| [`can-hal-pcan`](can-hal-pcan/) | PEAK PCAN-Basic backend (Windows and Linux) |
| [`can-hal-kvaser`](can-hal-kvaser/) | KVASER CANlib backend (Windows and Linux) |
| [`can-hal-isotp`](can-hal-isotp/) | ISO-TP (ISO 15765-2) transport layer |

## Example

```rust
use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder};
use can_hal_socketcan::SocketCanDriver;

let driver = SocketCanDriver::new();
let mut channel = driver.channel_by_name("can0")?.bitrate(500_000)?.connect()?;

let id = CanId::new_standard(0x100)?;
let frame = CanFrame::new(id, &[0x01, 0x02, 0x03])?;
channel.transmit(&frame)?;
```

Switching backends requires only changing the driver:

```rust
use can_hal_pcan::PcanDriver;
let driver = PcanDriver::new()?;
let mut channel = driver.channel(0)?.bitrate(500_000)?.connect()?;
```

```rust
use can_hal_kvaser::KvaserDriver;
let driver = KvaserDriver::new()?;
let mut channel = driver.channel(0)?.bitrate(500_000)?.connect()?;
```

## ISO-TP

Send and receive multi-frame payloads over any `can-hal-rs` backend:

```rust
use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};

let config = IsoTpConfig::new(
    CanId::new_standard(0x7E0)?,
    CanId::new_standard(0x7E8)?,
);
let mut isotp = IsoTpChannel::new(channel, config);
isotp.send(&[0x10, 0x01])?;
let response = isotp.receive()?;
```

Supports normal, extended, and functional addressing. CAN FD via `IsoTpFdChannel`. Async via the `async` feature flag.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
