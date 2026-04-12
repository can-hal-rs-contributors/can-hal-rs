# can-hal-isotp

ISO-TP (ISO 15765-2) transport layer for [`can-hal-rs`](https://crates.io/crates/can-hal-rs).

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
