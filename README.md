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
| [`can-hal-rs`](can-hal/) | Core traits: `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `BusStatus` |
| [`can-hal-socketcan`](can-hal-socketcan/) | Linux SocketCAN backend |
| [`can-hal-pcan`](can-hal-pcan/) | PEAK PCAN-Basic backend (Windows and Linux) |
| [`can-hal-kvaser`](can-hal-kvaser/) | KVASER CANlib backend (Windows and Linux) |
| [`can-hal-isotp`](can-hal-isotp/) | ISO-TP (ISO 15765-2) transport layer |

Each backend exposes a concrete driver with a typestate-driven builder: `.channel(idx)` returns an `Initial` state, then `.classic(...)` or `.fd(nominal, data)` transitions to a `Classic` or `Fd` state where only the methods valid for that mode are callable. Channels are likewise mode-parameterized - `PcanChannel<Classic>` implements `Transmit + Receive`; `PcanChannel<Fd>` implements `TransmitFd + ReceiveFd`. Invalid combinations are compile errors, not runtime errors.

## Example

```rust,no_run
use can_hal::{CanId, CanFrame, Transmit};
use can_hal_socketcan::SocketCanDriver;

let driver = SocketCanDriver::new();
let mut channel = driver.channel_by_name("can0").connect().unwrap();

let id = CanId::new_standard(0x100).unwrap();
let frame = CanFrame::new(id, &[0x01, 0x02, 0x03]).unwrap();
channel.transmit(&frame).unwrap();
```

SocketCAN bitrate is OS-managed (`ip link set ... bitrate ...`). For PCAN, classic bitrate is a checked enum:

```rust,no_run
use can_hal_pcan::{PcanDriver, ClassicBitrate};
let driver = PcanDriver::new().unwrap();
let mut channel = driver.channel(0).unwrap().classic(ClassicBitrate::Br500K).connect().unwrap();
```

For Kvaser, classic bitrate is a `u32` validated against the 80 MHz CANlib clock:

```rust,no_run
use can_hal_kvaser::KvaserDriver;
let driver = KvaserDriver::new().unwrap();
let mut channel = driver.channel(0).classic(500_000).unwrap().connect().unwrap();
```

CAN FD on either hardware-backed crate (here, PCAN):

```rust,no_run
use can_hal_pcan::PcanDriver;
let driver = PcanDriver::new().unwrap();
let mut channel = driver.channel(0).unwrap().fd(500_000, 4_000_000).unwrap().connect().unwrap();
```

## ISO-TP

Send and receive multi-frame payloads over any `can-hal-rs` backend:

```rust,no_run
use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
# fn example<C: can_hal::Transmit<Error = E> + can_hal::Receive<Error = E>, E: can_hal::CanError>(channel: C) {
let config = IsoTpConfig::new(
    CanId::new_standard(0x7E0).unwrap(),
    CanId::new_standard(0x7E8).unwrap(),
);
let mut isotp = IsoTpChannel::new(channel, config);
isotp.send(&[0x10, 0x01]).unwrap();
let response = isotp.receive().unwrap();
# }
```

Supports normal, extended, and functional addressing. CAN FD via `IsoTpFdChannel`. Async via the `async` feature flag.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
