# can-hal-socketcan

Linux SocketCAN backend for [`can-hal-rs`](https://crates.io/crates/can-hal-rs).

Implements `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `Driver`, and `ChannelBuilder` using SocketCAN sockets.

## Usage

```rust,no_run
use can_hal::{CanId, CanFrame, Transmit, Receive, ChannelBuilder};
use can_hal_socketcan::SocketCanDriver;

let driver = SocketCanDriver::new();
let mut channel = driver
    .channel_by_name("can0")
    .unwrap()
    .bitrate(500_000)
    .unwrap()
    .connect()
    .unwrap();

let id = CanId::new_standard(0x123).unwrap();
let frame = CanFrame::new(id, &[0xDE, 0xAD]).unwrap();
channel.transmit(&frame).unwrap();

let response = channel.receive().unwrap();
println!("{:?}", response.frame());
```

## Interface setup

SocketCAN bitrate is configured at the OS level, not through the socket API:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

For testing without hardware, use a virtual CAN interface:

```bash
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set vcan0 up
```

## License

Licensed under either of [Apache License, Version 2.0](../LICENSE-APACHE) or [MIT License](../LICENSE-MIT) at your option.
