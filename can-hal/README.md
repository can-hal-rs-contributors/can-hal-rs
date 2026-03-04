# can-hal

Hardware-agnostic traits for CAN bus communication in Rust.

`can-hal` defines the interface. Backend crates implement it. Application code is portable across CAN hardware.

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

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
