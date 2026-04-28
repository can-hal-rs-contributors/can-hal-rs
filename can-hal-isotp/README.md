<div align="center">
  <h1>can-hal-isotp</h1>
  <p>
    <strong>ISO-TP (ISO 15765-2) transport layer for <a href="https://crates.io/crates/can-hal-rs">can-hal-rs</a></strong>
  </p>
  <p>

[![crates.io](https://img.shields.io/crates/v/can-hal-isotp?label=latest)](https://crates.io/crates/can-hal-isotp)
[![Documentation](https://docs.rs/can-hal-isotp/badge.svg)](https://docs.rs/can-hal-isotp)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.81+-ab6000.svg)
![License](https://img.shields.io/crates/l/can-hal-isotp.svg)
[![CI](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml)

  </p>
</div>

Provides segmentation and reassembly of payloads larger than a single CAN frame,
using the Single Frame / First Frame / Consecutive Frame / Flow Control protocol.

## Usage

```rust,no_run
use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};

// `channel` must implement `can_hal::{Transmit, Receive}`.
// let channel = ...;
// let config = IsoTpConfig::new(
//     CanId::new_standard(0x7E0).unwrap(),
//     CanId::new_standard(0x7E8).unwrap(),
// );
// let mut isotp = IsoTpChannel::new(channel, config);
// isotp.send(&[0x10, 0x01]).unwrap();
// let response = isotp.receive().unwrap();
```

## Addressing modes

- **Normal** (default): PCI bytes immediately follow the CAN ID. Maximum 7 bytes per SF.
- **Extended**: A target address byte precedes the PCI. Maximum 6 bytes per SF.
- **Functional**: Broadcast using `IsoTpConfig::functional_id` via `IsoTpChannel::send_functional`. Single frames only.

## CAN FD

Use `IsoTpFdChannel` for FD-capable hardware. SF payloads up to 62 bytes, CF payloads up to 63 bytes.

## Feature flags

- `async`: Enables `AsyncIsoTpChannel` and `AsyncIsoTpFdChannel` backed by Tokio.

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
