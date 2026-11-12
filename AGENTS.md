# can-hal Agent Instructions

can-hal is a Rust workspace providing hardware-agnostic CAN bus traits with backend
implementations for Linux SocketCAN, PEAK PCAN-Basic, and KVASER CANlib adapters. The core
traits crate (`can-hal-rs`) is `no_std`-compatible with zero dependencies. An ISO-TP
(ISO 15765-2) transport layer crate rounds out the workspace.

Always reference these instructions first and fall back to searching the codebase only when
you encounter something unexpected.

## Shared Ground Rules

- This is a Rust workspace (resolver v2, edition 2021, MSRV 1.81). Use idiomatic Rust.
- The MSRV is declared once, as `rust-version` in the root `Cargo.toml`'s
  `[workspace.package]`; every member inherits it with `rust-version.workspace = true`.
  CI's `msrv` job checks it. To raise it, change that line, the MSRV badge in all six
  READMEs, the "1.81" in `can-hal/README.md`, and the two mentions in this file.
  `.cargo/config.toml` makes `cargo update` keep to versions that support it.
- The core `can-hal` crate has **zero dependencies** and must stay that way. It is
  `no_std`-compatible: the `std` feature is on by default, and `default-features = false`
  builds it without std (CI's `msrv` job checks this). `can-hal-isotp` needs std.
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
│       ├── frame.rs         # CanFrame, CanFdFrame, Frame enum, Timestamped
│       ├── id.rs            # CanId (Standard 11-bit / Extended 29-bit)
│       ├── filter.rs        # Filter struct, Filterable trait
│       ├── bus.rs           # BusState, ErrorCounters, BusStatus trait
│       ├── error.rs         # CanError blanket trait
│       ├── timing.rs        # SamplePoint newtype
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
├── hardware-tests/          # Integration tests (physical hardware + software-in-the-loop)
│   └── tests/
│       ├── cross_adapter.rs       # Classic CAN: PCAN <-> Kvaser
│       ├── cross_adapter_fd.rs    # CAN FD: PCAN <-> Kvaser
│       ├── cross_adapter_timing.rs # Timing: sample points, explicit timing, bitrate matrix, mismatch
│       ├── cross_adapter_init.rs  # Init/modes: direct FD frames, bus status, FD filters, reopen, errors
│       ├── kvaser_virtual.rs      # Kvaser CANlib SITL over virtual channels (`kvaser-virtual` feature)
│       └── socketcan_vcan.rs      # SocketCAN SITL over vcan0 (Linux; `vcan` feature)
└── .github/workflows/
    ├── ci.yml               # Format, clippy, unit tests, build checks, SocketCAN SITL, MSRV
    └── hardware-test.yml    # Self-hosted HIL tests (Windows VM + Linux)
```

## Core Traits (can-hal)

The trait hierarchy is the heart of the project. Understand these before working on
any backend:

| Trait | Purpose |
|---|---|
| `Transmit` / `Receive` | Send/receive classic CAN frames |
| `TransmitFd` / `ReceiveFd` | Send/receive CAN FD frames (`ReceiveFd` returns `Frame` enum) |
| `Filterable` | Set/clear hardware acceptance filters |
| `BusStatus` | Query bus state (ErrorActive/Passive/BusOff) and error counters |
| `CanError` | Blanket trait: `Error + Send + Sync + 'static` |
| `Async*` variants | Async versions of Transmit/Receive/TransmitFd/ReceiveFd (behind `async` feature) |

There is no shared `Driver` or `ChannelBuilder` trait. Each backend exposes its own
concrete driver type and a **typestate-driven builder**: an `Initial` state transitions
to `Classic`, `Fd`, or one of the `*Explicit` raw-timing states, and only the methods
valid for the current state are callable. Invalid combinations (e.g., `data_sample_point`
in classic mode, `transmit_fd` on a classic channel, mixing solver-driven sample-point
setters with raw segment values) are compile errors, not runtime ones. Channels are
likewise parameterized: `PcanChannel<Classic>` implements `Transmit + Receive`;
`PcanChannel<Fd>` implements `TransmitFd + ReceiveFd`. Both implement `Filterable`
and `BusStatus`.

Sample points are wrapped in a [`can_hal::SamplePoint`] newtype with a `const fn`
constructor that asserts `[500, 950]` per-mille at compile time when used in a
`const` context. Use `SamplePoint::NOMINAL_DEFAULT` (70%) and `SamplePoint::DATA_DEFAULT`
(80%) for the workspace-wide interop conventions.

**Filter semantics vary by backend:** SocketCAN supports multiple independent filters
(union semantics), while PCAN and Kvaser support a single ID+mask pair per frame type.
Code that uses `Filterable` must be aware of this.

### Builder API per backend

| Backend | Classic | FD | Sample-point setters | Raw escape hatch |
|---|---|---|---|---|
| SocketCAN | `driver.channel_by_name(name).connect()?` (timing OS-managed) | n/a | n/a (no setters) | n/a |
| PCAN | `driver.channel(0)?.classic(ClassicBitrate::Br500K).connect()?` (enum, infallible) | `driver.channel(0)?.fd(500_000, 4_000_000)?.connect()?` | `<Fd>` only (none in classic; use `fd(rate, rate)` for custom classic timing) | `<Initial>::fd_explicit(PcanFdTiming)` -> `<FdExplicit>` |
| Kvaser | `driver.channel(0).classic(500_000)?.connect()?` (`channel()` infallible; all four bitrate-taking transitions - `classic`, `fd`, `classic_explicit`, `fd_explicit` - fail on divisibility of the device's clock, as CANlib reports it, or 80 MHz when it doesn't; the `_explicit` variants additionally check segment bounds, `(bitrate * (1 + tseg1 + tseg2))` divisibility, and prescaler `[1, 1024]`) | `driver.channel(0).fd(500_000, 4_000_000)?.connect()?` | both `<Classic>` and `<Fd>` | `<Initial>::classic_explicit(hz, BusParams)?` / `fd_explicit(hz, hz, BusParams, BusParamsFd)?` |

## Build and Development

### Prerequisites

- Rust toolchain >= 1.81 (install via [rustup](https://rustup.rs))
- **SocketCAN backend**: Linux only (socketcan's build script refuses other targets); no
  system headers or C libraries needed
- **PCAN backend**: `libpcanbasic.so` (Linux) or `PCANBasic.dll` (Windows) from PEAK
- **Kvaser backend**: `libcanlib.so.1` (Linux) or `canlib32.dll` (Windows) from KVASER
- **Hardware tests**: Physical PCAN and Kvaser adapters connected to the same CAN bus

### Common Commands

```bash
# Format check
cargo fmt --check

# Lint core + ISO-TP
cargo clippy -p can-hal-rs -p can-hal-isotp --all-targets --features async -- -D warnings

# Lint the PCAN and Kvaser backends (no vendor SDK needed: they load it at runtime)
cargo clippy -p can-hal-pcan -p can-hal-kvaser --all-targets -- -D warnings

# Unit tests (core traits + ISO-TP)
cargo test -p can-hal-rs -p can-hal-isotp --features async

# Unit tests (backends; SocketCAN builds on Linux only)
cargo test -p can-hal-socketcan -p can-hal-pcan -p can-hal-kvaser

# Build check for SocketCAN (Linux only)
cargo check -p can-hal-socketcan

# Windows cross-compile check
cargo clippy -p can-hal-pcan -p can-hal-kvaser \
  --target x86_64-pc-windows-gnu --all-targets -- -D warnings
```

### Running Examples

Each backend has an example crate under `examples/`, with one example per file in its
`examples/` directory. Run via `cargo run -p <crate> --example <name>`:

```bash
# SocketCAN (the interface defaults to can0; name another, such as vcan0, after --)
cargo run -p can-hal-socketcan-examples --example socketcan_send -- vcan0
cargo run -p can-hal-socketcan-examples --example socketcan_receive -- vcan0
cargo run -p can-hal-socketcan-examples --example filter -- vcan0

# PCAN (requires PCAN hardware + library)
cargo run -p can-hal-pcan-examples --example pcan_send
cargo run -p can-hal-pcan-examples --example pcan_receive
cargo run -p can-hal-pcan-examples --example pcan_send_fd
cargo run -p can-hal-pcan-examples --example pcan_receive_fd
cargo run -p can-hal-pcan-examples --example bus_status

# Kvaser (requires Kvaser hardware + library)
cargo run -p can-hal-kvaser-examples --example kvaser_send
cargo run -p can-hal-kvaser-examples --example kvaser_receive
cargo run -p can-hal-kvaser-examples --example kvaser_send_fd
cargo run -p can-hal-kvaser-examples --example kvaser_receive_fd

# ISO-TP
cargo run -p can-hal-isotp-examples --example isotp_send
cargo run -p can-hal-isotp-examples --example isotp_receive
cargo run -p can-hal-isotp-examples --example loopback
# see examples/can-hal-isotp/examples/ for the full list
```

### Virtual CAN Setup (for SocketCAN testing without hardware)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## Hardware-in-the-Loop (HIL) Testing

Hardware tests live in the `hardware-tests` crate and exercise cross-adapter communication
(e.g., send from PCAN, receive on Kvaser and vice versa). The crate also holds two
software-in-the-loop (SITL) suites that need no physical hardware: SocketCAN over a `vcan`
interface, and Kvaser CANlib over its virtual channels.

### Requirements

- Cross-adapter tests: at least 2 CAN adapters on the same physical bus (PCAN-USB FD +
  Kvaser U100) and vendor libraries installed (libpcanbasic, libcanlib).
- SocketCAN SITL tests: a `vcan0` interface (Linux only); no adapters or vendor libraries.
- Kvaser SITL tests: CANlib and its virtual driver (on Linux, linuxcan's `virtualcan`; on
  Windows, the CANlib driver installs virtual channels by default); no adapters.

### Running Locally

```bash
# Cross-adapter tests (PCAN <-> Kvaser; needs both adapters)
cargo test -p hardware-tests -- --test-threads=1

# SocketCAN SITL tests (needs vcan0, see "Virtual CAN Setup" above)
cargo test -p hardware-tests --features vcan --test socketcan_vcan -- --test-threads=1

# Kvaser SITL tests (needs CANlib's virtual channels)
CAN_HAL_REQUIRE_KVASER_VIRTUAL=1 cargo test -p hardware-tests --features kvaser-virtual \
  --test kvaser_virtual -- --test-threads=1
```

`--test-threads=1` is **required**: it prevents hardware contention for the cross-adapter
tests, and stops the SITL tests from seeing each other's frames on the shared `vcan0` or
virtual bus.

The SITL tests are behind the off-by-default `vcan` and `kvaser-virtual` features, so a
plain `cargo test -p hardware-tests` excludes them (and the SocketCAN dependency) entirely.
When `vcan0` is absent the SocketCAN tests skip cleanly; set `CAN_HAL_REQUIRE_VCAN=1` (as
CI does) to turn a missing interface into a hard failure instead of a silent skip. The
Kvaser tests likewise skip when CANlib or the channels are missing, unless
`CAN_HAL_REQUIRE_KVASER_VIRTUAL=1` is set. They use CANlib channels 0 and 1 unless
`CAN_HAL_KVASER_VIRTUAL_CHANNELS="<tx>,<rx>"` names others, as it must when a physical
Kvaser interface comes first. No CI job runs them; the `msrv` job only compiles them.

### CI Infrastructure

The hardware CI runs on a self-hosted NixOS runner with physical CAN adapters:

1. **Windows tests first**: Starts a Windows VM, power-cycles the adapters' USB hub
   (`uhubctl`), attaches the devices to the VM, runs
   `cargo test -p hardware-tests -- --test-threads=1` over SSH, stops the VM and
   power-cycles the hub again.
2. **Linux tests second**: Once the Windows job succeeds, runs the same tests natively.

This sequential ordering exists because USB devices are free (unbound) at boot for VM
passthrough; the hub power cycle after the VM releases them returns them to the host.

## CAN Bus Timing

Timing parameters must match across all adapters on the same bus, or nodes will go bus-off.

**Nominal CAN (500 Kbit/s, 20 TQ):**
- `freq_hz=500000, tseg1=13, tseg2=6, sjw=4`

**CAN FD Data Phase (4 Mbit/s, 10 TQ):**
- `freq_hz=4000000, tseg1=7, tseg2=2, sjw=2`

### Backend-Specific Timing Notes

- **PCAN**: 80 MHz clock. The builder runs an internal solver that derives
  `(brp, tseg1, tseg2, sjw)` from `.fd(nominal, data)` plus optional
  `.sample_point(SamplePoint)` / `.data_sample_point(SamplePoint)`. Sample
  points default to 70% (nominal) and 80% (data). For raw per-segment
  control, transition straight from `<Initial>` to `<FdExplicit>` via
  `.fd_explicit(PcanFdTiming)`. Classic CAN uses the predefined
  `ClassicBitrate` enum and has no sample-point control (PCAN-Basic API
  limitation).
- **Kvaser**: the device's clock, from CANlib's `canCHANNELDATA_CLOCK_INFO`,
  or 80 MHz when it isn't reported (Linux CANlib doesn't for a Leaf Light v2,
  whose CAN clock is 16 MHz, or for virtual channels). Classic timing with an
  unknown clock is solved for 8 MHz first, which fits 16, 24, 32 and 80 MHz
  clocks (16 TQ at 250 and 500 kbit/s). Same solver shape as PCAN. Classic and FD
  builders both honor `.sample_point()` / `.data_sample_point()`. For raw
  control, use `.classic_explicit(hz, BusParams)?` or
  `.fd_explicit(hz, hz, BusParams, BusParamsFd)?` on `<Initial>`. The
  `libcanlib.so.1` on Linux does **not** accept predefined bitrate constants
  (negative frequency values like `-2` for 500K); these return
  `canERR_PARAM`. Always pass explicit `freq_hz` + segment values.
- **SocketCAN**: Bitrate is configured at the OS level (`ip link set can0 type can
  bitrate 500000`), not through the socket API. The builder has no timing methods.

## Backend Architecture

All three backends follow the same pattern:

| File | Purpose |
|---|---|
| `driver.rs` | Concrete driver type + typestate `ChannelBuilder<Mode>` |
| `mode.rs` | Type-state markers (`Initial`, `Classic`, `Fd`, `*Explicit`) with private state |
| `channel.rs` | `Transmit`, `Receive`, `TransmitFd`, `ReceiveFd`, `Filterable` impls (split by mode); `BusStatus` on PCAN/Kvaser only |
| `convert.rs` | Conversions between `can-hal` frame types and vendor-specific types |
| `error.rs` | Backend error type implementing `CanError` |
| `ffi.rs` | FFI bindings (PCAN and Kvaser only) |
| `library.rs` | Dynamic library loading via `libloading` (PCAN and Kvaser only) |
| `event.rs` | Status/event callbacks (PCAN and Kvaser only) |

### Key Design Decisions

- **Dynamic linking**: PCAN and Kvaser load vendor libraries at runtime via `libloading`.
  This means the crates compile without the vendor SDK installed; errors surface at runtime
  when the driver's `new()` is called.
- **SocketCAN uses the `socketcan` crate**: Unlike the other backends, SocketCAN wraps an
  existing Rust crate rather than doing raw FFI.
- **Timestamped frames**: The `Timestamped<F, T>` wrapper lets each backend choose its own
  timestamp type (e.g., `std::time::Instant`, hardware tick counts).

## ISO-TP (can-hal-isotp)

The ISO-TP crate implements ISO 15765-2 segmentation and reassembly, generic over any
`Transmit + Receive` channel.

**Key types:**
- `IsoTpChannel<C>` - classic CAN transport (up to 4095 bytes via short FF; up to 4 GB via long FF)
- `IsoTpFdChannel<C>` - CAN FD transport (same limits)
- `IsoTpConfig` - TX/RX CAN IDs, addressing mode
- `AddressingMode` - Normal, Extended, Functional

**Async support:** Enable the `async` feature for `AsyncIsoTpChannel` and
`AsyncIsoTpFdChannel` (backed by `tokio`).

## CI Workflow Summary

### `ci.yml` (runs on push to master and all PRs)

| Job | What it does |
|---|---|
| `fmt` | `cargo fmt --check` |
| `clippy` | Lint `can-hal-rs` + `can-hal-isotp` with `--features async` |
| `test` | Unit tests for `can-hal-rs` + `can-hal-isotp` (with `async`), and for `can-hal-socketcan`, `can-hal-pcan` and `can-hal-kvaser` |
| `build-check` | `cargo check` for SocketCAN; clippy for PCAN + Kvaser |
| `build-check-windows` | Cross-compile clippy for PCAN + Kvaser targeting `x86_64-pc-windows-gnu` |
| `socketcan-sitl` | Set up a `vcan0` interface and run the SocketCAN SITL tests (`hardware-tests --features vcan --test socketcan_vcan`). No physical hardware; `CAN_HAL_REQUIRE_VCAN=1` makes a missing `vcan0` a hard failure rather than a silent skip |
| `msrv` | Install the MSRV read from the workspace's `rust-version`; check every crate with it on Linux, and for `x86_64-pc-windows-gnu` except the SocketCAN crates and the ISO-TP examples; check `can-hal-rs` without std for `thumbv7em-none-eabihf` (with and without `async`); run the `can-hal-rs` + `can-hal-isotp` unit tests |

### `hardware-test.yml` (manual dispatch, or a PR labelled `run-hardware-tests`, again on each push)

| Job | What it does |
|---|---|
| `windows-hardware-tests` | Run the hardware tests in the Windows VM via SSH (see CI Infrastructure) |
| `linux-hardware-tests` | Once the Windows job succeeds, run the hardware tests natively |

## Key Files to Know

| File | Why it matters |
|---|---|
| `can-hal/src/channel.rs` | Core Transmit/Receive traits - start here |
| `can-hal/src/frame.rs` | Frame types used everywhere |
| `can-hal-{pcan,kvaser}/src/driver.rs` | Typestate builder per backend |
| `can-hal-{pcan,kvaser}/src/mode.rs` | `Initial`/`Classic`/`Fd` type-state markers |
| `can-hal-isotp/src/channel.rs` | ISO-TP segmentation logic |
| `hardware-tests/tests/cross_adapter.rs` | Integration test template |
| `.github/workflows/ci.yml` | What CI checks |
| `Cargo.toml` (root) | Workspace member list |

## Code Style

- Follow `rustfmt` defaults (no custom config).
- Treat warnings as errors: CI runs clippy with `-D warnings` on `can-hal-rs`,
  `can-hal-isotp`, `can-hal-pcan` and `can-hal-kvaser`.
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
3. **Unit tests**:
   - `cargo test -p can-hal-rs -p can-hal-isotp --features async`
   - `cargo test -p can-hal-socketcan -p can-hal-pcan -p can-hal-kvaser`
4. **Build check** (if touching backend crates):
   - `cargo clippy -p can-hal-pcan -p can-hal-kvaser --all-targets -- -D warnings`
   - `cargo check -p can-hal-socketcan`
5. **Hardware tests** (if touching backend crates and hardware is available):
   - `cargo test -p hardware-tests -- --test-threads=1`
