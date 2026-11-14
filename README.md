<div align="center">
  <h1>can-hal</h1>
  <p>
    <strong>Hardware-agnostic CAN bus traits for Rust</strong>
  </p>
  <p>

[![crates.io](https://img.shields.io/crates/v/can-hal-rs?label=latest)](https://crates.io/crates/can-hal-rs)
[![CI](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/can-hal-rs-contributors/can-hal-rs/actions/workflows/ci.yml)
![Minimum Supported Rust Version](https://img.shields.io/badge/rustc-1.81+-ab6000.svg)
![License](https://img.shields.io/crates/l/can-hal-rs.svg)

  </p>
</div>

> **Warning:** These crates are unstable and under active development. APIs may change without notice. Not yet recommended for production use.

Backend implementations for Linux SocketCAN, PEAK PCAN, and KVASER adapters, plus an ISO-TP transport layer. The core `can-hal-rs` traits crate has no dependencies and is `no_std`-compatible.

## Crates

| Crate | Description |
|---|---|
| [`can-hal-rs`](can-hal/) | Core traits: `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `BusStatus` |
| [`can-hal-socketcan`](can-hal-socketcan/) | Linux SocketCAN backend |
| [`can-hal-pcan`](can-hal-pcan/) | PEAK PCAN-Basic backend (Windows and Linux) |
| [`can-hal-kvaser`](can-hal-kvaser/) | KVASER CANlib backend (Windows and Linux) |
| [`can-hal-isotp`](can-hal-isotp/) | ISO-TP (ISO 15765-2) transport layer |

## Features

- Classic CAN data and remote frames, standard and extended IDs, DLCs 9-15
- CAN FD frames up to 64 bytes, with BRS and ESI
- Acceptance filters and bus status
- Async trait variants (`async` feature); the backends here implement only the blocking traits

## Backends

| | SocketCAN | PCAN | Kvaser |
|---|---|---|---|
| Platforms | Linux | Windows, Linux | Windows, Linux |
| Vendor library | none | PCAN-Basic, loaded at runtime | CANlib, loaded at runtime |
| Timing | OS (`ip link`) | `ClassicBitrate` enum; FD in Hz with sample points | Hz with sample points |
| Filters | exact, per filter | one ID range per ID type | one code/mask per ID type |
| DLCs 9-15 | with `cc-len8-dlc on` | sent, untested | classic channels only |
| `BusStatus` | no | yes; error counters on Linux only | yes |

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

For Kvaser, classic bitrate is a `u32` checked against the device's CAN clock (as CANlib reports it, or 80 MHz when it doesn't):

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

Send and receive multi-frame payloads over any `can-hal-rs` backend (here, SocketCAN):

```rust,no_run
use can_hal::CanId;
use can_hal_isotp::{IsoTpChannel, IsoTpConfig};
use can_hal_socketcan::SocketCanDriver;

let channel = SocketCanDriver::new().channel_by_name("can0").connect().unwrap();
let config = IsoTpConfig::new(
    CanId::new_standard(0x7E0).unwrap(),
    CanId::new_standard(0x7E8).unwrap(),
);
let mut isotp = IsoTpChannel::new(channel, config);
isotp.send(&[0x10, 0x01]).unwrap();
let response = isotp.receive().unwrap();
```

## Examples

Example crates live under [`examples/`](examples/):

```bash
cargo run -p can-hal-socketcan-examples --example socketcan_send -- vcan0
cargo run -p can-hal-pcan-examples --example pcan_send_fd
cargo run -p can-hal-kvaser-examples --example kvaser_receive
cargo run -p can-hal-isotp-examples --example loopback -- vcan0
```

The PCAN and Kvaser examples need the adapter and vendor library. The rest run on `vcan0` ([setup](can-hal-socketcan/README.md#interface-setup)), except the ISO-TP `cross_adapter*` examples, which need PCAN and Kvaser adapters. `kernel_interop` also needs the `can-isotp` kernel module.

## Testing

```bash
# Unit tests
cargo test -p can-hal-rs -p can-hal-isotp --features async
cargo test -p can-hal-socketcan -p can-hal-pcan -p can-hal-kvaser

# Hardware: PCAN and Kvaser adapters on one bus
cargo test -p hardware-tests -- --test-threads=1

# Software-in-the-loop: SocketCAN over vcan0, Kvaser virtual channels
CAN_HAL_REQUIRE_VCAN=1 cargo test -p hardware-tests --features vcan \
  --test socketcan_vcan -- --test-threads=1
CAN_HAL_REQUIRE_KVASER_VIRTUAL=1 cargo test -p hardware-tests --features kvaser-virtual \
  --test kvaser_virtual -- --test-threads=1
```

`--test-threads=1` is required. Without `CAN_HAL_REQUIRE_*`, the software-in-the-loop tests skip when `vcan0` or the Kvaser virtual channels are missing.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
