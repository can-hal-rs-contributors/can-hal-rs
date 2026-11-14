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
Works with any `can-hal-rs` backend.

## Usage

```rust,no_run
use can_hal::{CanError, CanId, Receive, Transmit};
use can_hal_isotp::{IsoTpChannel, IsoTpConfig, IsoTpError};

/// Reads the VIN (UDS ReadDataByIdentifier 0xF190).
fn read_vin<C, E>(channel: C) -> Result<Vec<u8>, IsoTpError<E>>
where
    C: Transmit<Error = E> + Receive<Error = E>,
    E: CanError,
{
    let config = IsoTpConfig::new(
        CanId::new_standard(0x7E0).unwrap(),
        CanId::new_standard(0x7E8).unwrap(),
    );
    let mut isotp = IsoTpChannel::new(channel, config);
    isotp.send(&[0x22, 0xF1, 0x90])?;
    isotp.receive()
}
```

Multi-frame payloads up to 4095 bytes use the 12-bit First Frame length; longer
payloads, up to `u32::MAX` bytes, use the 32-bit form. `into_inner()` returns the
CAN channel.

## Configuration

`IsoTpConfig::new(tx_id, rx_id)` sets these defaults; override them through the
public fields.

| Field | Default | Description |
|---|---|---|
| `addressing` | `Normal` | `Normal` or `Extended { tx_target_address, rx_target_address }` |
| `block_size` | `0` | Block Size advertised in Flow Control frames (0 = unlimited) |
| `st_min` | `5` (5 ms) | Raw STmin advertised in Flow Control frames; `send` uses the peer's |
| `timeout` | 1 s | Per-wait timeout: first SF/FF and each CF in `receive`, each FC in `send` |
| `padding` | `Some(0xCC)` | Pad byte. `None` disables padding on classic CAN; FD frames over 8 bytes are always padded (0x00 for `None`) |
| `max_fc_wait` | `10` | Consecutive FC(Wait) frames that abort `send` (0 = unlimited) |
| `functional_id` | `None` | CAN ID for functional requests |

## Addressing modes

- **Normal** (default): PCI bytes immediately follow the CAN ID. Maximum 7 bytes per SF.
- **Extended**: A target address byte precedes the PCI. Maximum 6 bytes per SF.
- **Functional**: Broadcast to `IsoTpConfig::functional_id` via `send_functional` (`IsoTpChannel` and `AsyncIsoTpChannel`). Single frames only.

## CAN FD

Use `IsoTpFdChannel` (requires `TransmitFd + ReceiveFd`) for FD hardware. SF
payloads up to 62 bytes and CF payloads up to 63, one fewer each with extended
addressing.

## Feature flags

- `async`: Enables `AsyncIsoTpChannel` and `AsyncIsoTpFdChannel`, for channels
  that implement the `can-hal-rs` async traits. Requires a Tokio runtime with the
  timer enabled.

## Examples

See the [ISO-TP examples](../examples/can-hal-isotp/examples/): loopback (classic
and FD), send/receive, extended and functional addressing, kernel `CAN_ISOTP`
interop, and PCAN <-> Kvaser.

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
