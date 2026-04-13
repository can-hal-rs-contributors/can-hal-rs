# can-hal Agent Instructions

can-hal is a Rust workspace providing hardware-agnostic CAN bus traits with backend
implementations for Linux SocketCAN, PEAK PCAN-Basic, and KVASER CANlib adapters. The core
traits crate (`can-hal-rs`) is `no_std`-compatible with zero dependencies. An ISO-TP
(ISO 15765-2) transport layer crate rounds out the workspace.

Always reference these instructions first and fall back to searching the codebase only when
you encounter something unexpected.

## Shared Ground Rules

- This is a Rust workspace (resolver v2, edition 2021, MSRV 1.81). Use idiomatic Rust.
- The core `can-hal` crate has **zero dependencies** and must stay that way. It is
  `no_std`-compatible by default; the `std` feature is opt-in.
- Backend crates (`can-hal-socketcan`, `can-hal-pcan`, `can-hal-kvaser`) use **dynamic
  library loading** via `libloading` (PCAN and Kvaser) or the system `socketcan` crate.
  Do not introduce compile-time links to vendor libraries.
- Dual license: MIT OR Apache-2.0. Preserve this in any new crate or file.
- Commit in imperative mood, keep changes scoped, and link issues in PRs.
- Run `cargo fmt --check` and `cargo clippy` before submitting.
- Do not add unnecessary dependencies. Evaluate whether a dependency is truly needed before
  adding it to any crate.

## Repository Structure

```
can-hal/
├── Cargo.toml              # Workspace root
├── can-hal/                 # Core traits crate (published as can-hal-rs)
│   └── src/
│       ├── lib.rs           # Re-exports, module declarations, unit tests
│       ├── channel.rs       # Transmit, Receive, TransmitFd, ReceiveFd traits
│       ├── driver.rs        # Driver, DriverFd, ChannelBuilder traits
│       ├── frame.rs         # CanFrame, CanFdFrame, Frame enum, Timestamped
│       ├── id.rs            # CanId (Standard 11-bit / Extended 29-bit)
│       ├── filter.rs        # Filter struct, Filterable trait
│       ├── bus.rs           # BusState, ErrorCounters, BusStatus trait
│       ├── error.rs         # CanError blanket trait
│       └── async_channel.rs # Async variants of channel traits
├── can-hal-socketcan/       # Linux SocketCAN backend
├── can-hal-pcan/            # PEAK PCAN-Basic backend (Windows + Linux)
├── can-hal-kvaser/          # KVASER CANlib backend (Windows + Linux)
├── can-hal-isotp/           # ISO-TP transport layer (generic over any backend)
├── examples/                # Per-backend example crates
│   ├── can-hal-socketcan/
│   ├── can-hal-pcan/
│   ├── can-hal-kvaser/
│   └── can-hal-isotp/
├── hardware-tests/          # Cross-adapter integration tests (physical hardware)
│   └── tests/
│       ├── cross_adapter.rs     # Classic CAN: PCAN <-> Kvaser
│       └── cross_adapter_fd.rs  # CAN FD: PCAN <-> Kvaser
└── .github/workflows/
    ├── ci.yml               # Format, clippy, unit tests, build checks
    └── hardware-test.yml    # Self-hosted HIL tests (Windows VM + Linux)
```

## Core Traits (can-hal)

The trait hierarchy is the heart of the project. Understand these before working on
any backend:

| Trait | Purpose |
|---|---|
| `Transmit` / `Receive` | Send/receive classic CAN frames |
| `TransmitFd` / `ReceiveFd` | Send/receive CAN FD frames (`ReceiveFd` returns `Frame` enum) |
| `Driver` / `DriverFd` | Factory to open channels on hardware interfaces |
| `ChannelBuilder` | Configure bitrate, sample point, etc. before going on-bus |
| `Filterable` | Set/clear hardware acceptance filters |
| `BusStatus` | Query bus state (ErrorActive/Passive/BusOff) and error counters |
| `CanError` | Blanket trait: `Error + Send + Sync + 'static` |
| `Async*` variants | Async versions of Transmit/Receive/TransmitFd/ReceiveFd (behind `async` feature) |

**Filter semantics vary by backend:** SocketCAN supports multiple independent filters
(union semantics), while PCAN and Kvaser support a single ID+mask pair per frame type.
Code that uses `Filterable` must be aware of this.

## Build and Development

### Prerequisites

- Rust toolchain >= 1.81 (install via [rustup](https://rustup.rs))
- **SocketCAN backend**: Linux with kernel CAN headers (available on most distros)
- **PCAN backend**: `libpcanbasic.so` (Linux) or `PCANBasic.dll` (Windows) from PEAK
- **Kvaser backend**: `libcanlib.so.1` (Linux) or `canlib32.dll` (Windows) from KVASER
- **Hardware tests**: Physical PCAN and Kvaser adapters connected to the same CAN bus

### Common Commands

```bash
# Format check
cargo fmt --check

# Lint core + ISO-TP (the crates that build without vendor libraries)
cargo clippy -p can-hal-rs -p can-hal-isotp --all-targets --features async -- -D warnings

# Lint hardware backend crates (requires headers/libs or cross-compile target)
cargo clippy -p can-hal-pcan -p can-hal-kvaser --all-targets -- -D warnings

# Unit tests (core traits + ISO-TP)
cargo test -p can-hal-rs -p can-hal-isotp --features async

# Build check for SocketCAN (Linux only)
cargo check -p can-hal-socketcan

# Windows cross-compile check
cargo clippy -p can-hal-pcan -p can-hal-kvaser \
  --target x86_64-pc-windows-gnu --all-targets -- -D warnings
```

### Running Examples

Each backend has an example crate under `examples/`:

```bash
# SocketCAN (requires vcan0 or physical interface)
cargo run -p can-hal-socketcan-example

# PCAN (requires PCAN hardware + library)
cargo run -p can-hal-pcan-example

# Kvaser (requires Kvaser hardware + library)
cargo run -p can-hal-kvaser-example

# ISO-TP
cargo run -p can-hal-isotp-example
```

### Virtual CAN Setup (for SocketCAN testing without hardware)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Hardware-in-the-Loop (HIL) Testing

Hardware tests live in the `hardware-tests` crate and exercise cross-adapter communication
(e.g., send from PCAN, receive on Kvaser and vice versa).

### Requirements

- At least 2 CAN adapters on the same physical bus (PCAN-USB FD + Kvaser U100)
- Vendor libraries installed (libpcanbasic, libcanlib)

### Running Locally

```bash
cargo test -p hardware-tests -- --test-threads=1
```

`--test-threads=1` is **required** to prevent hardware contention between tests.

### CI Infrastructure

The hardware CI runs on a self-hosted NixOS runner with physical CAN adapters:

1. **Windows tests first**: Starts a Windows VM, attaches USB devices via passthrough,
   runs `cargo test -p hardware-tests` over SSH.
2. **Linux tests second**: After the VM releases USB devices, binds them to host drivers
   and runs the same tests natively.

This sequential ordering exists because USB devices are free (unbound) at boot for VM
passthrough, and must be explicitly returned to the host for Linux testing.

## CAN Bus Timing

Timing parameters must match across all adapters on the same bus, or nodes will go bus-off.

**Nominal CAN (500 Kbit/s, 20 TQ):**
- `freq_hz=500000, tseg1=13, tseg2=6, sjw=4`

**CAN FD Data Phase (4 Mbit/s, 10 TQ):**
- `freq_hz=4000000, tseg1=7, tseg2=2, sjw=2`

### Backend-Specific Timing Notes

- **PCAN**: Uses `fd_timing_string()` on the builder. Clock is 80 MHz.
- **Kvaser**: Requires explicit timing values. The `libcanlib.so.1` on Linux does **not**
  accept predefined bitrate constants (negative frequency values like `-2` for 500K); these
  return `canERR_PARAM`. Always pass explicit `freq_hz` + segment values.
- **SocketCAN**: Bitrate is configured at the OS level (`ip link set can0 type can
  bitrate 500000`), not through the socket API. The `bitrate()` builder method is a no-op
  for SocketCAN.

## Backend Architecture

All three backends follow the same pattern:

| File | Purpose |
|---|---|
| `driver.rs` | `Driver` / `DriverFd` impl — opens channels |
| `channel.rs` | `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable`, `BusStatus` impls |
| `convert.rs` | Conversions between `can-hal` frame types and vendor-specific types |
| `error.rs` | Backend error type implementing `CanError` |
| `ffi.rs` | FFI bindings (PCAN and Kvaser only) |
| `library.rs` | Dynamic library loading via `libloading` (PCAN and Kvaser only) |
| `event.rs` | Status/event callbacks (PCAN and Kvaser only) |

### Key Design Decisions

- **Dynamic linking**: PCAN and Kvaser load vendor libraries at runtime via `libloading`.
  This means the crates compile without the vendor SDK installed; errors surface at runtime
  when `Driver::new()` is called.
- **SocketCAN uses the `socketcan` crate**: Unlike the other backends, SocketCAN wraps an
  existing Rust crate rather than doing raw FFI.
- **Timestamped frames**: The `Timestamped<F, T>` wrapper lets each backend choose its own
  timestamp type (e.g., `std::time::Instant`, hardware tick counts).

## ISO-TP (can-hal-isotp)

The ISO-TP crate implements ISO 15765-2 segmentation and reassembly, generic over any
`Transmit + Receive` channel.

**Key types:**
- `IsoTpChannel<C>` — classic CAN transport
- `IsoTpFdChannel<C>` — CAN FD transport (up to 4095 bytes per message)
- `IsoTpConfig` — TX/RX CAN IDs, addressing mode
- `AddressingMode` — Normal, Extended, Functional

**Async support:** Enable the `async` feature for `AsyncIsoTpChannel` and
`AsyncIsoTpFdChannel` (backed by `tokio`).

## CI Workflow Summary

### `ci.yml` (runs on push to master and all PRs)

| Job | What it does |
|---|---|
| `fmt` | `cargo fmt --check` |
| `clippy` | Lint `can-hal-rs` + `can-hal-isotp` with `--features async` |
| `test` | Unit tests for `can-hal-rs` + `can-hal-isotp` |
| `build-check` | `cargo check` for SocketCAN; clippy for PCAN + Kvaser |
| `build-check-windows` | Cross-compile clippy for PCAN + Kvaser targeting `x86_64-pc-windows-gnu` |

### `hardware-test.yml` (manual dispatch)

| Job | What it does |
|---|---|
| `windows-hardware-tests` | Start Windows VM, attach USB, run hardware tests via SSH |
| `linux-hardware-tests` | After Windows completes, bind USB to host, run hardware tests natively |

## Key Files to Know

| File | Why it matters |
|---|---|
| `can-hal/src/channel.rs` | Core Transmit/Receive traits — start here |
| `can-hal/src/frame.rs` | Frame types used everywhere |
| `can-hal/src/driver.rs` | Driver/ChannelBuilder pattern |
| `can-hal-isotp/src/channel.rs` | ISO-TP segmentation logic |
| `hardware-tests/tests/cross_adapter.rs` | Integration test template |
| `.github/workflows/ci.yml` | What CI checks |
| `Cargo.toml` (root) | Workspace member list |

## Code Style

- Follow `rustfmt` defaults (no custom config).
- All warnings are errors in CI (`-D warnings` via clippy).
- Use `snake_case` for functions/variables, `CamelCase` for types, `UPPER_CASE` for
  constants.
- Keep `unsafe` blocks minimal and well-documented. The FFI layers in PCAN/Kvaser contain
  most of the `unsafe` code.
- Prefer returning `Result<T, E>` over panicking. Backend errors should implement `CanError`.
- When adding a new trait or modifying an existing one in `can-hal`, update **all** backends
  that implement it.

## Validation Checklist

Before submitting changes:

1. **Format**: `cargo fmt --check`
2. **Lint**: `cargo clippy -p can-hal-rs -p can-hal-isotp --all-targets --features async -- -D warnings`
3. **Unit tests**: `cargo test -p can-hal-rs -p can-hal-isotp --features async`
4. **Build check** (if touching backend crates):
   - `cargo clippy -p can-hal-pcan -p can-hal-kvaser --all-targets -- -D warnings`
   - `cargo check -p can-hal-socketcan`
5. **Hardware tests** (if touching backend crates and hardware is available):
   - `cargo test -p hardware-tests -- --test-threads=1`
