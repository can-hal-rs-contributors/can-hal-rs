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
[![CI](https://github.com/Dolphindalt/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/Dolphindalt/can-hal-rs/actions/workflows/ci.yml)

  </p>
</div>

`can-hal-rs` defines the interface. Backend crates implement it. Application code is portable across CAN hardware.

## `no_std` support

This crate is `no_std`-compatible (requires Rust 1.81+). The `std` feature is enabled by default. To use in embedded / `no_std` contexts:

```toml
[dependencies]
can-hal-rs = { version = "0.3", default-features = false }
```

## Traits

| Trait | Purpose |
|---|---|
| `Transmit` / `Receive` | Classic CAN (up to 8 bytes) |
| `TransmitFd` / `ReceiveFd` | CAN FD (up to 64 bytes) |
| `Driver` / `ChannelBuilder` | Open and configure channels |
| `Filterable` | Hardware acceptance filtering |
| `BusStatus` | Bus state and error counters |

Async variants (`AsyncTransmit`, `AsyncReceive`, etc.) are available behind the `async` feature flag.

## Usage

```rust
use can_hal::{CanId, CanFrame, Transmit, Receive};

fn echo<T: Transmit<Error = E> + Receive<Error = E>, E: can_hal::CanError>(
    channel: &mut T,
) -> Result<(), E> {
    let msg = channel.receive()?;
    channel.transmit(msg.frame())?;
    Ok(())
}
```

## Backend crates

- [`can-hal-socketcan`](https://crates.io/crates/can-hal-socketcan) — Linux SocketCAN
- [`can-hal-pcan`](https://crates.io/crates/can-hal-pcan) — PEAK PCAN-Basic (Windows and Linux)
- [`can-hal-kvaser`](https://crates.io/crates/can-hal-kvaser) — KVASER CANlib (Windows and Linux)

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
