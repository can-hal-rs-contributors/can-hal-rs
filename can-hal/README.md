<div align="center">
  <h1>can-hal-rs</h1>
  <p>
    <strong>Hardware-agnostic traits for CAN bus communication in Rust</strong>
  </p>
  <p>

[![crates.io](https://img.shields.io/crates/v/can-hal-rs?label=latest)](https://crates.io/crates/can-hal-rs)
[![Documentation](https://docs.rs/can-hal-rs/badge.svg)](https://docs.rs/can-hal-rs)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.81+-ab6000.svg)
![License](https://img.shields.io/crates/l/can-hal-rs.svg)
[![CI](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml)

  </p>
</div>

`can-hal-rs` defines the interface. Backend crates implement it. Application code is portable across CAN hardware.

## `no_std` support

This crate is `no_std`-compatible (requires Rust 1.81+). The `std` feature is enabled by default. To use in embedded / `no_std` contexts:

```toml
[dependencies]
can-hal-rs = { version = "0.4", default-features = false }
```

## Traits

| Trait | Purpose |
|---|---|
| `Transmit` / `Receive` | Classic CAN data and remote frames (up to 8 bytes) |
| `TransmitFd` / `ReceiveFd` | CAN FD (up to 64 bytes); `ReceiveFd` also receives classic frames |
| `Filterable` | Hardware acceptance filtering |
| `BusStatus` | Bus state and error counters |

Async variants (`AsyncTransmit`, `AsyncReceive`, `AsyncTransmitFd`, `AsyncReceiveFd`) are behind the dependency-free `async` feature. The SocketCAN, PCAN, and Kvaser backends implement only the blocking traits.

There is no shared `Driver` or `ChannelBuilder` trait - each backend exposes its own concrete driver and builder. The PCAN and Kvaser builders are typestate-driven, so invalid combinations are compile errors. See the backend crates for details.

## Types

| Type | Description |
|---|---|
| `CanId` | Standard (11-bit) or extended (29-bit) ID |
| `CanFrame` | Classic data or remote frame, DLC 0-15 |
| `CanFdFrame` | CAN FD frame with BRS and ESI flags |
| `Frame` | `CanFrame` or `CanFdFrame` |
| `Timestamped<F, T>` | Received frame with backend timestamp |
| `Filter` | ID and mask |
| `SamplePoint` | Sample point in per-mille, 500-950 |

## Filters

Filters match only frames of their own ID type; a frame type with no filter is rejected. SocketCAN applies each filter exactly. PCAN and Kvaser merge filters into one window per ID type, which can pass unlisted IDs. For exact matching on every backend, filter in software.

## Usage

```rust
use can_hal::{Transmit, Receive};

fn echo<T: Transmit<Error = E> + Receive<Error = E>, E: can_hal::CanError>(
    channel: &mut T,
) -> Result<(), E> {
    let msg = channel.receive()?;
    channel.transmit(msg.frame())?;
    Ok(())
}
```

## Backend crates

- [`can-hal-socketcan`](https://crates.io/crates/can-hal-socketcan) - Linux SocketCAN
- [`can-hal-pcan`](https://crates.io/crates/can-hal-pcan) - PEAK PCAN-Basic (Windows and Linux)
- [`can-hal-kvaser`](https://crates.io/crates/can-hal-kvaser) - KVASER CANlib (Windows and Linux)
- [`can-hal-isotp`](https://crates.io/crates/can-hal-isotp) provides ISO-TP (ISO 15765-2) over any backend.

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
