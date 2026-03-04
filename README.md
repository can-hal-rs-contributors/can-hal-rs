# can-hal

Hardware-agnostic CAN bus traits for Rust, with backend implementations for Linux SocketCAN and PEAK PCAN adapters.

## Crates

| Crate | Description |
|---|---|
| [`can-hal`](can-hal/) | Core traits: `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Driver`, `ChannelBuilder`, `Filterable`, `BusStatus` |
| [`can-hal-socketcan`](can-hal-socketcan/) | Linux SocketCAN backend |
| [`can-hal-pcan`](can-hal-pcan/) | PEAK PCAN-Basic backend (Windows and Linux) |

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

Switching to a PCAN adapter requires only changing the driver:

```rust
use can_hal_pcan::PcanDriver;

let driver = PcanDriver::new()?;
let mut channel = driver.channel(0)?.bitrate(500_000)?.connect()?;
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
