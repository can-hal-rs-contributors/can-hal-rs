use std::collections::VecDeque;
use std::time::{Duration, Instant};

use can_hal::channel::{Receive, ReceiveFd, Transmit, TransmitFd};
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};
use can_hal::id::CanId;

use crate::channel::IsoTpChannel;
use crate::config::{AddressingMode, IsoTpConfig};
use crate::error::IsoTpError;
use crate::fd_channel::IsoTpFdChannel;
use crate::frame::IsoTpFrame;
use crate::frame::{
    build_cf, build_cf_fd, build_ff, build_ff_fd, build_sf, build_sf_fd, interpret_st_min,
    next_fd_dlc,
};

// ---------------------------------------------------------------------------
// Mock
// ---------------------------------------------------------------------------

/// Minimal error type for the mock channel.
///
/// `can_hal::error::CanError` has a blanket impl for all
/// `std::error::Error + Send + Sync + 'static` types, so no explicit impl is needed.
#[derive(Debug)]
pub struct MockError;

impl std::fmt::Display for MockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MockError")
    }
}

impl std::error::Error for MockError {}

pub struct MockChannel {
    pub tx_frames: Vec<CanFrame>,
    pub rx_queue: VecDeque<CanFrame>,
    /// How long each frame takes to arrive in `receive_timeout`, counted from
    /// the call. Zero hands frames out at once.
    pub rx_delay: Duration,
}

impl MockChannel {
    pub fn new(rx: impl IntoIterator<Item = CanFrame>) -> Self {
        Self {
            tx_frames: Vec::new(),
            rx_queue: rx.into_iter().collect(),
            rx_delay: Duration::ZERO,
        }
    }
}

/// Sleep as `receive_timeout(timeout)` would for a frame that arrives `delay`
/// after the call, and say whether it arrived in time.
fn frame_arrives(delay: Duration, timeout: Duration) -> bool {
    if !delay.is_zero() {
        std::thread::sleep(delay.min(timeout));
    }
    delay <= timeout
}

impl Transmit for MockChannel {
    type Error = MockError;
    fn transmit(&mut self, frame: &CanFrame) -> Result<(), MockError> {
        self.tx_frames.push(frame.clone());
        Ok(())
    }
}

impl Receive for MockChannel {
    type Error = MockError;
    type Timestamp = Instant;

    fn receive(&mut self) -> Result<Timestamped<CanFrame, Instant>, MockError> {
        self.rx_queue
            .pop_front()
            .map_or(Err(MockError), |f| Ok(Timestamped::new(f, Instant::now())))
    }

    fn try_receive(&mut self) -> Result<Option<Timestamped<CanFrame, Instant>>, MockError> {
        Ok(self
            .rx_queue
            .pop_front()
            .map(|f| Timestamped::new(f, Instant::now())))
    }

    fn receive_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<CanFrame, Instant>>, MockError> {
        if !frame_arrives(self.rx_delay, timeout) {
            return Ok(None);
        }
        Ok(self
            .rx_queue
            .pop_front()
            .map(|f| Timestamped::new(f, Instant::now())))
    }
}

/// CAN FD counterpart of [`MockChannel`].
pub struct FdMockChannel {
    pub tx_frames: Vec<CanFdFrame>,
    pub rx_queue: VecDeque<Frame>,
    /// As [`MockChannel::rx_delay`], for `receive_fd_timeout`.
    pub rx_delay: Duration,
}

impl FdMockChannel {
    pub fn new(rx: impl IntoIterator<Item = Frame>) -> Self {
        Self {
            tx_frames: Vec::new(),
            rx_queue: rx.into_iter().collect(),
            rx_delay: Duration::ZERO,
        }
    }
}

impl TransmitFd for FdMockChannel {
    type Error = MockError;
    fn transmit_fd(&mut self, frame: &CanFdFrame) -> Result<(), MockError> {
        self.tx_frames.push(frame.clone());
        Ok(())
    }
}

impl ReceiveFd for FdMockChannel {
    type Error = MockError;
    type Timestamp = Instant;

    fn receive_fd(&mut self) -> Result<Timestamped<Frame, Instant>, MockError> {
        self.try_receive_fd()?.ok_or(MockError)
    }

    fn try_receive_fd(&mut self) -> Result<Option<Timestamped<Frame, Instant>>, MockError> {
        Ok(self
            .rx_queue
            .pop_front()
            .map(|f| Timestamped::new(f, Instant::now())))
    }

    fn receive_fd_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Timestamped<Frame, Instant>>, MockError> {
        if !frame_arrives(self.rx_delay, timeout) {
            return Ok(None);
        }
        self.try_receive_fd()
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn tx_id() -> CanId {
    CanId::new_standard(0x7E0).unwrap()
}

fn rx_id() -> CanId {
    CanId::new_standard(0x7E8).unwrap()
}

fn default_config() -> IsoTpConfig {
    IsoTpConfig {
        timeout: Duration::from_millis(500),
        max_fc_wait: 3,
        ..IsoTpConfig::new(tx_id(), rx_id())
    }
}

fn make_frame(id: CanId, data: &[u8]) -> CanFrame {
    CanFrame::new(id, data).unwrap()
}

/// Build a Flow Control frame on the rx_id.
fn fc_frame(flag: u8, bs: u8, stmin: u8) -> CanFrame {
    make_frame(rx_id(), &[0x30 | flag, bs, stmin])
}

fn fc_cts() -> CanFrame {
    fc_frame(0, 0, 0)
}

fn fc_wait() -> CanFrame {
    fc_frame(1, 0, 0)
}

fn fc_overflow() -> CanFrame {
    fc_frame(2, 0, 0)
}

/// A classic First Frame on rx_id for a `total`-byte message starting with `payload`.
pub fn classic_ff(total: usize, payload: &[u8]) -> CanFrame {
    let mut buf = [0u8; 8];
    build_ff(&mut buf, payload, total, 0);
    make_frame(rx_id(), &buf)
}

/// A classic Consecutive Frame on rx_id.
pub fn classic_cf(sn: u8, chunk: &[u8]) -> CanFrame {
    let mut buf = [0u8; 8];
    let len = build_cf(&mut buf, chunk, sn, 0);
    make_frame(rx_id(), &buf[..len])
}

/// A CAN FD frame on rx_id, rounded up to a valid FD length with 0xCC.
pub fn fd_frame(data: &[u8]) -> Frame {
    let mut buf = [0xCCu8; 64];
    buf[..data.len()].copy_from_slice(data);
    let len = next_fd_dlc(data.len());
    Frame::Fd(CanFdFrame::new(rx_id(), &buf[..len], true, false).unwrap())
}

/// A 64-byte FD First Frame on rx_id for a `total`-byte message starting with `payload`.
pub fn fd_ff(total: usize, payload: &[u8]) -> Frame {
    let mut buf = [0u8; 64];
    build_ff_fd(&mut buf, payload, total, 0);
    fd_frame(&buf)
}

/// An FD Consecutive Frame on rx_id.
pub fn fd_cf(sn: u8, chunk: &[u8]) -> Frame {
    let mut buf = [0u8; 64];
    let len = build_cf_fd(&mut buf, chunk, sn, 0);
    fd_frame(&buf[..len])
}

/// An FD Single Frame on rx_id.
pub fn fd_sf(payload: &[u8]) -> Frame {
    let mut buf = [0u8; 64];
    let len = build_sf_fd(&mut buf, payload, 0);
    fd_frame(&buf[..len])
}

// ---------------------------------------------------------------------------
// Send tests
// ---------------------------------------------------------------------------

#[test]
fn test_send_single_frame() {
    let mock = MockChannel::new(vec![]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let payload = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
    ch.send(&payload).unwrap();

    let inner = ch.into_inner();
    assert_eq!(inner.tx_frames.len(), 1);
    let data = inner.tx_frames[0].data();
    // PCI byte: SF with DL=7
    assert_eq!(data[0], 0x07);
    assert_eq!(&data[1..], &payload);
}

#[test]
fn test_send_single_frame_padding() {
    let mut cfg = default_config();
    cfg.padding = Some(0xCC);

    let mock = MockChannel::new(vec![]);
    let mut ch = IsoTpChannel::new(mock, cfg);

    let payload = [0xAA, 0xBB, 0xDD];
    ch.send(&payload).unwrap();

    let inner = ch.into_inner();
    assert_eq!(inner.tx_frames.len(), 1);
    let data = inner.tx_frames[0].data();
    // Padded to 8 bytes
    assert_eq!(data.len(), 8);
    assert_eq!(data[0], 0x03); // PCI: SF DL=3
    assert_eq!(&data[1..4], &payload);
    // Remaining bytes are padding
    for &b in &data[4..8] {
        assert_eq!(b, 0xCC, "padding byte should be 0xCC");
    }
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn test_send_multi_frame() {
    // 20 bytes: FF(20) + CFs, need FC(CTS) in rx_queue
    let payload: Vec<u8> = (0..20).collect();

    let mock = MockChannel::new(vec![fc_cts()]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    ch.send(&payload).unwrap();

    let inner = ch.into_inner();

    // First frame: FF with 12-bit length
    let ff_data = inner.tx_frames[0].data();
    assert_eq!(ff_data[0] & 0xF0, 0x10, "first frame should have FF PCI");
    let total_len = (((ff_data[0] & 0x0F) as usize) << 8) | (ff_data[1] as usize);
    assert_eq!(total_len, 20);
    // FF carries 6 data bytes (8 - 2 header bytes)
    assert_eq!(&ff_data[2..8], &payload[0..6]);

    // Consecutive frames
    let cf_count = inner.tx_frames.len() - 1;
    assert!(cf_count >= 1, "should have at least 1 CF");

    let mut offset = 6;
    for (i, cf) in inner.tx_frames[1..].iter().enumerate() {
        let d = cf.data();
        let expected_sn = ((i + 1) & 0x0F) as u8;
        assert_eq!(d[0] & 0xF0, 0x20, "CF should have CF PCI");
        assert_eq!(d[0] & 0x0F, expected_sn, "sequence number mismatch");

        let remaining = payload.len() - offset;
        let chunk_len = remaining.min(7);
        assert_eq!(&d[1..=chunk_len], &payload[offset..offset + chunk_len]);
        offset += chunk_len;
    }
    assert_eq!(offset, 20, "all bytes should be sent");
}

#[test]
fn test_send_fc_wait_then_cts() {
    let payload: Vec<u8> = (0..20).collect();

    // Two FC(Wait) then FC(CTS)
    let mock = MockChannel::new(vec![fc_wait(), fc_wait(), fc_cts()]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    ch.send(&payload).unwrap();

    let inner = ch.into_inner();
    // Should succeed: FF + CFs
    assert!(
        inner.tx_frames.len() >= 2,
        "should have FF + at least one CF"
    );
    assert_eq!(inner.tx_frames[0].data()[0] & 0xF0, 0x10);
}

#[test]
fn test_send_fc_wait_limit_exceeded() {
    let payload: Vec<u8> = (0..20).collect();

    // 4 FC(Wait) frames, but max_fc_wait=3 so it should fail on the 3rd
    let mock = MockChannel::new(vec![fc_wait(), fc_wait(), fc_wait(), fc_wait()]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let result = ch.send(&payload);
    assert!(result.is_err());
    match result.unwrap_err() {
        IsoTpError::WaitLimitExceeded => {}
        e => panic!("expected WaitLimitExceeded, got {e:?}"),
    }
}

#[test]
fn test_send_fc_overflow() {
    let payload: Vec<u8> = (0..20).collect();

    let mock = MockChannel::new(vec![fc_overflow()]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let result = ch.send(&payload);
    assert!(result.is_err());
    match result.unwrap_err() {
        IsoTpError::BufferOverflow => {}
        e => panic!("expected BufferOverflow, got {e:?}"),
    }
}

#[test]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn test_send_long_ff_path() {
    // Verifies the 32-bit length encoding path (long FF) for payloads > 4095 bytes.
    // send()'s PayloadTooLarge (data.len() > u32::MAX) isn't tested: it needs a
    // payload of over 4 GiB.
    let payload: Vec<u8> = (0..4096).map(|i| (i & 0xFF) as u8).collect();

    // Need FC(CTS) for the wait_for_fc call
    let mock = MockChannel::new(vec![fc_cts()]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    ch.send(&payload).unwrap();

    let inner = ch.into_inner();
    let ff_data = inner.tx_frames[0].data();
    // 4096 > 0xFFF so this should be a long FF: [0x10, 0x00, 0x00, 0x00, 0x10, 0x00]
    assert_eq!(ff_data[0], 0x10);
    assert_eq!(ff_data[1], 0x00);
    // 4096 = 0x00001000
    assert_eq!(ff_data[2], 0x00);
    assert_eq!(ff_data[3], 0x00);
    assert_eq!(ff_data[4], 0x10);
    assert_eq!(ff_data[5], 0x00);
}

// ---------------------------------------------------------------------------
// Receive tests
// ---------------------------------------------------------------------------

#[test]
fn test_receive_single_frame() {
    let sf = make_frame(rx_id(), &[0x03, 0xAA, 0xBB, 0xCC]);
    let mock = MockChannel::new(vec![sf]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let data = ch.receive().unwrap();
    assert_eq!(data, vec![0xAA, 0xBB, 0xCC]);
}

#[test]
fn test_receive_skips_remote_frame() {
    // A remote frame on rx_id carries no ISO-TP frame: it is passed over, not parsed.
    let remote = CanFrame::new_remote(rx_id(), 8).unwrap();
    let sf = make_frame(rx_id(), &[0x03, 0xAA, 0xBB, 0xCC]);
    let mock = MockChannel::new(vec![remote, sf]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let data = ch.receive().unwrap();
    assert_eq!(data, vec![0xAA, 0xBB, 0xCC]);
}

#[test]
fn test_receive_multi_frame() {
    // 20-byte message
    let payload: Vec<u8> = (0..20).collect();

    // First Frame: 12-bit length = 20, header [0x10, 0x14], then 6 data bytes
    let mut ff_bytes = vec![0x10, 0x14];
    ff_bytes.extend_from_slice(&payload[0..6]);
    let ff = make_frame(rx_id(), &ff_bytes);

    // CF sn=1: 7 bytes (payload[6..13])
    let mut cf1_bytes = vec![0x21];
    cf1_bytes.extend_from_slice(&payload[6..13]);
    let cf1 = make_frame(rx_id(), &cf1_bytes);

    // CF sn=2: 7 bytes (payload[13..20])
    let mut cf2_bytes = vec![0x22];
    cf2_bytes.extend_from_slice(&payload[13..20]);
    let cf2 = make_frame(rx_id(), &cf2_bytes);

    let mock = MockChannel::new(vec![ff, cf1, cf2]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let data = ch.receive().unwrap();
    assert_eq!(data, payload);

    // Verify FC(CTS) was transmitted after FF
    let inner = ch.into_inner();
    assert!(!inner.tx_frames.is_empty(), "should have sent FC");
    let fc_data = inner.tx_frames[0].data();
    assert_eq!(fc_data[0], 0x30, "FC CTS flag byte");
}

#[test]
fn test_receive_sequence_error() {
    let payload: Vec<u8> = (0..20).collect();

    let mut ff_bytes = vec![0x10, 0x14];
    ff_bytes.extend_from_slice(&payload[0..6]);
    let ff = make_frame(rx_id(), &ff_bytes);

    // CF sn=1
    let mut cf1_bytes = vec![0x21];
    cf1_bytes.extend_from_slice(&payload[6..13]);
    let cf1 = make_frame(rx_id(), &cf1_bytes);

    // CF sn=3 (should be 2) -- sequence error
    let mut cf_bad = vec![0x23];
    cf_bad.extend_from_slice(&payload[13..20]);
    let cf_bad = make_frame(rx_id(), &cf_bad);

    let mock = MockChannel::new(vec![ff, cf1, cf_bad]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let result = ch.receive();
    assert!(result.is_err());
    match result.unwrap_err() {
        IsoTpError::SequenceError { expected, got } => {
            assert_eq!(expected, 2);
            assert_eq!(got, 3);
        }
        e => panic!("expected SequenceError, got {e:?}"),
    }
}

#[test]
fn test_receive_timeout() {
    // Empty rx_queue => receive_timeout returns None => Timeout
    let mock = MockChannel::new(vec![]);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let result = ch.receive();
    assert!(result.is_err());
    match result.unwrap_err() {
        IsoTpError::Timeout => {}
        e => panic!("expected Timeout, got {e:?}"),
    }
}

#[test]
fn test_receive_extended_addressing() {
    let mut cfg = default_config();
    cfg.addressing = AddressingMode::Extended {
        tx_target_address: 0xF1,
        rx_target_address: 0xF1,
    };

    // SF with TA byte prefix: [0xF1, 0x03, 0xAA, 0xBB, 0xCC]
    let sf = make_frame(rx_id(), &[0xF1, 0x03, 0xAA, 0xBB, 0xCC]);
    let mock = MockChannel::new(vec![sf]);
    let mut ch = IsoTpChannel::new(mock, cfg);

    let data = ch.receive().unwrap();
    assert_eq!(data, vec![0xAA, 0xBB, 0xCC]);
}

#[test]
#[allow(clippy::cast_possible_truncation)]
fn test_long_ff() {
    // Long FF: total length > 4095, uses 32-bit encoding.
    // We'll use total_len = 5000 = 0x00001388
    let total_len: usize = 5000;

    // Long FF header: [0x10, 0x00, hi3, hi2, hi1, hi0, data0, data1]
    // 5000 = 0x00001388
    let ff_bytes: Vec<u8> = vec![0x10, 0x00, 0x00, 0x00, 0x13, 0x88, 0x00, 0x01];
    let ff = make_frame(rx_id(), &ff_bytes);

    // FF carries 2 data bytes (8 - 6 header bytes): payload[0..2] = [0x00, 0x01]
    // Build enough CFs to fill remaining 4998 bytes (7 bytes per CF)
    let mut payload: Vec<u8> = vec![0x00, 0x01]; // from FF
    let mut frames: Vec<CanFrame> = vec![ff];

    let mut sn: u8 = 1;
    while payload.len() < total_len {
        let remaining = total_len - payload.len();
        let chunk_len = remaining.min(7);
        let chunk: Vec<u8> = (0..chunk_len)
            .map(|j| ((payload.len() + j) & 0xFF) as u8)
            .collect();

        let mut cf_bytes = vec![0x20 | (sn & 0x0F)];
        cf_bytes.extend_from_slice(&chunk);
        frames.push(make_frame(rx_id(), &cf_bytes));

        payload.extend_from_slice(&chunk);
        sn = (sn + 1) & 0x0F;
    }

    // Remove the FF from the beginning to just have CFs in rx_queue after FF
    let ff_frame = frames.remove(0);
    let mut all_frames = vec![ff_frame];
    all_frames.extend(frames);

    let mock = MockChannel::new(all_frames);
    let mut ch = IsoTpChannel::new(mock, default_config());

    let data = ch.receive().unwrap();
    assert_eq!(data.len(), total_len);
    assert_eq!(data, payload);
}

// ---------------------------------------------------------------------------
// An SF or FF during a reception (ISO 15765-2 unexpected N_PDU): the
// reception in progress is dropped and the new frame is processed.
// ---------------------------------------------------------------------------

#[test]
fn test_receive_sf_aborts_reception() {
    let a: Vec<u8> = (0..13).collect();
    let frames = vec![
        classic_ff(13, &a),
        // An SF arrives before the CF that would complete message A.
        make_frame(rx_id(), &[0x03, 0xAA, 0xBB, 0xCC]),
        classic_cf(1, &a[6..13]),
    ];
    let mut ch = IsoTpChannel::new(MockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), vec![0xAA, 0xBB, 0xCC]);
}

#[test]
fn test_receive_ff_replaces_partial_message() {
    // Ignoring B's FF would splice A's first 6 bytes with B's CF.
    let a: Vec<u8> = (0..13).collect();
    let b: Vec<u8> = (100..113).collect();
    let frames = vec![
        classic_ff(13, &a),
        classic_ff(13, &b),
        classic_cf(1, &b[6..13]),
    ];
    let mut ch = IsoTpChannel::new(MockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), b);
}

#[test]
fn test_receive_ff_restarts_reception() {
    // A CF of message A arrives before B's FF, so B must restart the
    // sequence number at 1 and the block count at 0: with block_size 2, a
    // block count left at 1 would send a third FC after B's first CF.
    let a: Vec<u8> = (0..20).collect();
    let b: Vec<u8> = (100..120).collect();
    let frames = vec![
        classic_ff(20, &a),
        classic_cf(1, &a[6..13]),
        classic_ff(20, &b),
        classic_cf(1, &b[6..13]),
        classic_cf(2, &b[13..20]),
    ];
    let cfg = IsoTpConfig {
        block_size: 2,
        ..default_config()
    };
    let mut ch = IsoTpChannel::new(MockChannel::new(frames), cfg);
    assert_eq!(ch.receive().unwrap(), b);
    // One FC(CTS) per FF, none for a full block.
    let inner = ch.into_inner();
    assert_eq!(inner.tx_frames.len(), 2);
    assert!(inner.tx_frames.iter().all(|f| f.data()[0] == 0x30));
}

/// Every frame arrives 100 ms after it is asked for, and the timeout is
/// 250 ms. FF(A) at 100 ms starts N_Cr, which would end at 350 ms; FF(B) at
/// 300 ms must restart it, or B's CF at 500 ms comes too late.
fn ff_restart_config() -> IsoTpConfig {
    IsoTpConfig {
        timeout: Duration::from_millis(250),
        ..default_config()
    }
}

#[test]
fn test_receive_ff_restarts_n_cr() {
    let a: Vec<u8> = (0..20).collect();
    let b: Vec<u8> = (100..113).collect();
    let mut mock = MockChannel::new(vec![
        classic_ff(20, &a),
        fc_cts(),
        classic_ff(13, &b),
        fc_cts(),
        classic_cf(1, &b[6..13]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut ch = IsoTpChannel::new(mock, ff_restart_config());
    assert_eq!(ch.receive().unwrap(), b);
}

#[test]
fn test_fd_receive_ff_restarts_n_cr() {
    let a: Vec<u8> = (0u8..200).collect();
    let b: Vec<u8> = (50u8..175).collect();
    let fc = fd_frame(&[0x30, 0, 0]);
    let mut mock = FdMockChannel::new(vec![
        fd_ff(200, &a),
        fc.clone(),
        fd_ff(125, &b),
        fc,
        fd_cf(1, &b[62..]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut ch = IsoTpFdChannel::new(mock, ff_restart_config());
    assert_eq!(ch.receive().unwrap(), b);
}

#[test]
fn test_fd_receive_sf_aborts_reception() {
    let a: Vec<u8> = (0u8..125).collect();
    let frames = vec![
        fd_ff(125, &a),
        fd_sf(&[0xAA, 0xBB, 0xCC]),
        fd_cf(1, &a[62..]),
    ];
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), vec![0xAA, 0xBB, 0xCC]);
}

#[test]
fn test_fd_receive_ff_replaces_partial_message() {
    let a: Vec<u8> = (0u8..125).collect();
    let b: Vec<u8> = (100u8..225).collect();
    let frames = vec![fd_ff(125, &a), fd_ff(125, &b), fd_cf(1, &b[62..])];
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), b);
}

#[test]
fn test_fd_receive_ff_restarts_reception() {
    // As test_receive_ff_restarts_reception: FF 62 bytes, CFs 63 each.
    let a: Vec<u8> = (0u8..200).collect();
    let b: Vec<u8> = (50u8..238).collect();
    let frames = vec![
        fd_ff(200, &a),
        fd_cf(1, &a[62..125]),
        fd_ff(188, &b),
        fd_cf(1, &b[62..125]),
        fd_cf(2, &b[125..]),
    ];
    let cfg = IsoTpConfig {
        block_size: 2,
        ..default_config()
    };
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(frames), cfg);
    assert_eq!(ch.receive().unwrap(), b);
    let inner = ch.into_inner();
    assert_eq!(inner.tx_frames.len(), 2);
    assert!(inner.tx_frames.iter().all(|f| f.data()[0] == 0x30));
}

// ---------------------------------------------------------------------------
// Reserved STmin values (0x80-0xF0, 0xFA-0xFF): ISO 15765-2 has the sender
// use the longest valid value, 0x7F (127 ms), as the Linux kernel does.
// ---------------------------------------------------------------------------

#[test]
fn test_interpret_st_min_reserved_is_127ms() {
    for raw in (0x80..=0xF0).chain(0xFA..=0xFF) {
        assert_eq!(
            interpret_st_min(raw),
            Duration::from_millis(127),
            "STmin {raw:#04x}"
        );
    }
    // The defined values are unchanged.
    assert_eq!(interpret_st_min(0x00), Duration::ZERO);
    assert_eq!(interpret_st_min(0x05), Duration::from_millis(5));
    assert_eq!(interpret_st_min(0x7F), Duration::from_millis(127));
    assert_eq!(interpret_st_min(0xF1), Duration::from_millis(1));
    assert_eq!(interpret_st_min(0xF9), Duration::from_millis(1));
}

#[test]
fn test_config_st_min_duration_reserved_is_127ms() {
    let cfg = IsoTpConfig {
        st_min: 0xFA,
        ..default_config()
    };
    assert_eq!(cfg.st_min_duration(), Duration::from_millis(127));
}

#[test]
fn test_send_honors_reserved_st_min_as_127ms() {
    // 20 bytes = FF + 2 CFs, so one STmin gap.
    let payload: Vec<u8> = (0..20).collect();
    let mock = MockChannel::new(vec![fc_frame(0, 0, 0xFA)]);
    let mut ch = IsoTpChannel::new(mock, default_config());
    let start = Instant::now();
    ch.send(&payload).unwrap();
    let elapsed = start.elapsed();
    assert!(
        elapsed >= Duration::from_millis(127),
        "send() took {elapsed:?}"
    );
}

#[test]
fn test_fd_send_honors_reserved_st_min_as_127ms() {
    // 126 bytes = FF (62) + CF (63) + CF (1), so one STmin gap.
    let payload: Vec<u8> = (0u8..126).collect();
    let mock = FdMockChannel::new(vec![fd_frame(&[0x30, 0, 0xFA])]);
    let mut ch = IsoTpFdChannel::new(mock, default_config());
    let start = Instant::now();
    ch.send(&payload).unwrap();
    let elapsed = start.elapsed();
    assert!(
        elapsed >= Duration::from_millis(127),
        "send() took {elapsed:?}"
    );
}

// ---------------------------------------------------------------------------
// Frames on rx_id that aren't valid N_PDUs are ignored (ISO 15765-2, and the
// Linux kernel's isotp_rcv), whether or not a reception is in progress.
// ---------------------------------------------------------------------------

/// Payloads that fail to parse as an ISO-TP frame with normal addressing.
pub const INVALID_PDUS: &[&[u8]] = &[
    &[],                          // no PCI byte
    &[0x40, 0x01, 0x02],          // PCI type 0x4 (unknown)
    &[0xF0],                      // PCI type 0xF (unknown)
    &[0x10],                      // FF without its length byte
    &[0x10, 0x00, 0x00, 0x00],    // long FF without its 32-bit length
    &[0x3F, 0x00, 0x00],          // FC with flow status 0xF
    &[0x30, 0x00],                // FC without STmin
    &[0x00, 0x00],                // nibble-form SF with SF_DL 0
    &[0x08, 1, 2, 3, 4, 5, 6, 7], // SF_DL 8 in an 8-byte frame
];

pub fn invalid_frames() -> Vec<CanFrame> {
    INVALID_PDUS
        .iter()
        .map(|d| make_frame(rx_id(), d))
        .collect()
}

pub fn invalid_fd_frames() -> Vec<Frame> {
    INVALID_PDUS.iter().map(|d| fd_frame(d)).collect()
}

/// Check that `receive()` ignores `junk`, both while idle and mid-message: it
/// sends no FC for it and returns the message around it.
fn assert_receive_ignores(junk: &[CanFrame]) {
    let mut frames = junk.to_vec();
    frames.push(make_frame(rx_id(), &[0x02, 0x11, 0x22]));
    let mut ch = IsoTpChannel::new(MockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), vec![0x11, 0x22]);
    assert!(ch.into_inner().tx_frames.is_empty());

    let a: Vec<u8> = (0..13).collect();
    let mut frames = vec![classic_ff(13, &a)];
    frames.extend_from_slice(junk);
    frames.push(classic_cf(1, &a[6..13]));
    let mut ch = IsoTpChannel::new(MockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), a);
    // Only the FF got an FC.
    assert_eq!(ch.into_inner().tx_frames.len(), 1);
}

/// [`assert_receive_ignores`] for the FD channel.
fn assert_fd_receive_ignores(junk: &[Frame]) {
    let mut frames = junk.to_vec();
    frames.push(fd_sf(&[0x11, 0x22]));
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), vec![0x11, 0x22]);
    assert!(ch.into_inner().tx_frames.is_empty());

    let a: Vec<u8> = (0u8..125).collect();
    let mut frames = vec![fd_ff(125, &a)];
    frames.extend_from_slice(junk);
    frames.push(fd_cf(1, &a[62..]));
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(frames), default_config());
    assert_eq!(ch.receive().unwrap(), a);
    // Only the FF got an FC.
    assert_eq!(ch.into_inner().tx_frames.len(), 1);
}

#[test]
fn test_receive_ignores_invalid_frames() {
    assert_receive_ignores(&invalid_frames());
}

#[test]
fn test_receive_times_out_on_invalid_frames_only() {
    let cfg = IsoTpConfig {
        timeout: Duration::from_millis(50),
        ..default_config()
    };
    let mut ch = IsoTpChannel::new(MockChannel::new(invalid_frames()), cfg);
    let result = ch.receive();
    assert!(
        matches!(result, Err(IsoTpError::Timeout)),
        "expected Timeout, got {result:?}"
    );
}

#[test]
fn test_fd_receive_ignores_invalid_frames() {
    assert_fd_receive_ignores(&invalid_fd_frames());
}

#[test]
fn test_fd_receive_times_out_on_invalid_frames_only() {
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(invalid_fd_frames()), default_config());
    let result = ch.receive();
    assert!(
        matches!(result, Err(IsoTpError::Timeout)),
        "expected Timeout, got {result:?}"
    );
}

// ---------------------------------------------------------------------------
// While send() waits for an FC, a frame that fails to parse is ignored unless
// it is an FC: a malformed FC still ends the transfer, as in the Linux kernel
// (isotp_rcv_fc), where other frames never reach the sender.
// ---------------------------------------------------------------------------

pub fn is_fc_pci(pdu: &[u8]) -> bool {
    pdu.first().is_some_and(|b| b >> 4 == 0x3)
}

pub fn invalid_non_fc_frames() -> Vec<CanFrame> {
    INVALID_PDUS
        .iter()
        .filter(|d| !is_fc_pci(d))
        .map(|d| make_frame(rx_id(), d))
        .collect()
}

#[test]
fn test_send_ignores_invalid_non_fc_frames() {
    let mut rx = invalid_non_fc_frames();
    rx.push(fc_cts());
    let mut ch = IsoTpChannel::new(MockChannel::new(rx), default_config());
    let payload: Vec<u8> = (0..20).collect();
    ch.send(&payload).unwrap();
    // FF + 2 CFs.
    assert_eq!(ch.into_inner().tx_frames.len(), 3);
}

#[test]
fn test_send_fails_on_malformed_fc() {
    for pdu in INVALID_PDUS.iter().filter(|d| is_fc_pci(d)) {
        let mock = MockChannel::new(vec![make_frame(rx_id(), pdu)]);
        let mut ch = IsoTpChannel::new(mock, default_config());
        let payload: Vec<u8> = (0..20).collect();
        let result = ch.send(&payload);
        assert!(
            matches!(result, Err(IsoTpError::InvalidFrame)),
            "FC {pdu:02X?}: expected InvalidFrame, got {result:?}"
        );
    }
}

#[test]
fn test_fd_send_ignores_invalid_non_fc_frames() {
    let mut rx: Vec<Frame> = INVALID_PDUS
        .iter()
        .filter(|d| !is_fc_pci(d))
        .map(|d| fd_frame(d))
        .collect();
    rx.push(fd_frame(&[0x30, 0, 0]));
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(rx), default_config());
    let payload: Vec<u8> = (0u8..126).collect();
    ch.send(&payload).unwrap();
    // FF + 2 CFs.
    assert_eq!(ch.into_inner().tx_frames.len(), 3);
}

#[test]
fn test_fd_send_fails_on_malformed_fc() {
    for pdu in INVALID_PDUS.iter().filter(|d| is_fc_pci(d)) {
        let mock = FdMockChannel::new(vec![fd_frame(pdu)]);
        let mut ch = IsoTpFdChannel::new(mock, default_config());
        let payload: Vec<u8> = (0u8..126).collect();
        let result = ch.send(&payload);
        assert!(
            matches!(result, Err(IsoTpError::InvalidFrame)),
            "FC {pdu:02X?}: expected InvalidFrame, got {result:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// A First Frame whose FF_DL is below the payload it carries itself is
// ignored. The rule is the Linux kernel's (isotp_rcv_ff): FF_DL must cover
// the bytes after the FF's PCI, less one in an FD frame longer than 8 bytes.
// ---------------------------------------------------------------------------

/// The FF_DL and data length `parse` gives `raw`, or `None` if it rejects it.
fn parse_ff(raw: &[u8], overhead: usize) -> Option<(usize, usize)> {
    match IsoTpFrame::parse(raw, overhead) {
        Ok(IsoTpFrame::FirstFrame { total_len, data }) => Some((total_len, data.len())),
        Ok(other) => panic!("{raw:02X?} parsed as {other:?}"),
        Err(_) => None,
    }
}

#[test]
fn test_parse_ff_dl_below_ff_payload() {
    let d = [0xD0u8; 8];
    let ff = |pci: &[u8]| {
        let mut f = d;
        f[..pci.len()].copy_from_slice(pci);
        f
    };
    // 12-bit FF_DL: the FF carries 6 bytes.
    assert_eq!(parse_ff(&ff(&[0x10, 0x03]), 0), None);
    assert_eq!(parse_ff(&ff(&[0x10, 0x05]), 0), None);
    assert_eq!(parse_ff(&ff(&[0x10, 0x06]), 0), Some((6, 6)));
    assert_eq!(parse_ff(&ff(&[0x10, 0x14]), 0), Some((20, 6)));
    // 32-bit FF_DL: the FF carries 2 bytes.
    assert_eq!(parse_ff(&ff(&[0x10, 0, 0, 0, 0, 0]), 0), None);
    assert_eq!(parse_ff(&ff(&[0x10, 0, 0, 0, 0, 1]), 0), None);
    assert_eq!(parse_ff(&ff(&[0x10, 0, 0, 0, 0, 2]), 0), Some((2, 2)));
    // Extended addressing: the FF carries 5 bytes.
    assert_eq!(parse_ff(&ff(&[0xF1, 0x10, 0x04]), 1), None);
    assert_eq!(parse_ff(&ff(&[0xF1, 0x10, 0x05]), 1), Some((5, 5)));
}

#[test]
fn test_parse_fd_ff_dl_below_ff_payload() {
    let ff = |len: usize, pci: &[u8]| {
        let mut f = vec![0xD0u8; len];
        f[..pci.len()].copy_from_slice(pci);
        f
    };
    // A 64-byte FF carries 62 bytes; FF_DL may be one less, and the data
    // is then cut to FF_DL.
    assert_eq!(parse_ff(&ff(64, &[0x10, 60]), 0), None);
    assert_eq!(parse_ff(&ff(64, &[0x10, 61]), 0), Some((61, 61)));
    assert_eq!(parse_ff(&ff(64, &[0x10, 62]), 0), Some((62, 62)));
    assert_eq!(parse_ff(&ff(64, &[0x10, 200]), 0), Some((200, 62)));
    // A 12-byte FF carries 10.
    assert_eq!(parse_ff(&ff(12, &[0x10, 8]), 0), None);
    assert_eq!(parse_ff(&ff(12, &[0x10, 9]), 0), Some((9, 9)));
    // 32-bit FF_DL in 64 bytes: 58 bytes carried.
    assert_eq!(parse_ff(&ff(64, &[0x10, 0, 0, 0, 0, 56]), 0), None);
    assert_eq!(
        parse_ff(&ff(64, &[0x10, 0, 0, 0, 0, 57]), 0),
        Some((57, 57))
    );
    // Extended addressing in 64 bytes: 61 bytes carried.
    assert_eq!(parse_ff(&ff(64, &[0xF1, 0x10, 59]), 1), None);
    assert_eq!(parse_ff(&ff(64, &[0xF1, 0x10, 60]), 1), Some((60, 60)));
    // An 8-byte FD frame follows the classic rule.
    assert_eq!(parse_ff(&ff(8, &[0x10, 0x05]), 0), None);
    assert_eq!(parse_ff(&ff(8, &[0x10, 0x06]), 0), Some((6, 6)));
}

#[test]
fn test_receive_ignores_ff_dl_below_ff_payload() {
    // [10 03 01 02 03 04 05 06]: FF_DL 3, six bytes carried.
    assert_receive_ignores(&[classic_ff(3, &[1, 2, 3, 4, 5, 6])]);
}

#[test]
fn test_fd_receive_ignores_ff_dl_below_ff_payload() {
    let carried: Vec<u8> = (0u8..62).collect();
    assert_fd_receive_ignores(&[fd_ff(60, &carried)]);
}

#[test]
fn test_fd_receive_cuts_ff_payload_to_ff_dl() {
    // FF_DL 61 in a 64-byte FF that carries 62 bytes.
    let carried: Vec<u8> = (0u8..62).collect();
    let mut ch = IsoTpFdChannel::new(
        FdMockChannel::new(vec![fd_ff(61, &carried)]),
        default_config(),
    );
    assert_eq!(ch.receive().unwrap(), carried[..61]);
}

// ---------------------------------------------------------------------------
// Single Frame forms, as in the Linux kernel (isotp_rcv): a frame of up to
// 8 bytes carries SF_DL in the PCI's low nibble, and a CAN FD frame longer
// than that uses the escape form (nibble 0, SF_DL in the next byte). An SF
// in the other form is ignored.
// ---------------------------------------------------------------------------

/// The payload `parse` finds in an SF, or `None` if it rejects it.
fn parse_sf(raw: &[u8], overhead: usize) -> Option<Vec<u8>> {
    match IsoTpFrame::parse(raw, overhead) {
        Ok(IsoTpFrame::SingleFrame { data }) => Some(data.to_vec()),
        Ok(other) => panic!("{raw:02X?} parsed as {other:?}"),
        Err(_) => None,
    }
}

/// An SF escape-form PDU (`[00, len, data..]`) in a buffer of `frame_len` bytes.
#[allow(clippy::cast_possible_truncation)]
fn escape_sf(frame_len: usize, data: &[u8]) -> Vec<u8> {
    let mut f = vec![0xCC; frame_len];
    f[0] = 0x00;
    f[1] = data.len() as u8;
    f[2..2 + data.len()].copy_from_slice(data);
    f
}

#[test]
fn test_parse_sf_escape_form_needs_fd_frame_over_8_bytes() {
    let abc = [0xAA, 0xBB, 0xCC];
    // Escape form in frames of 8 bytes or fewer: ignored.
    assert_eq!(parse_sf(&escape_sf(5, &abc), 0), None);
    assert_eq!(parse_sf(&escape_sf(8, &abc), 0), None);
    assert_eq!(parse_sf(&[0xF1, 0x00, 0x03, 0xAA, 0xBB, 0xCC], 1), None);
    // Escape form in longer FD frames.
    let ten: Vec<u8> = (1..=10).collect();
    assert_eq!(parse_sf(&escape_sf(12, &ten), 0), Some(ten));
    let max: Vec<u8> = (0..62).collect();
    assert_eq!(parse_sf(&escape_sf(64, &max), 0), Some(max));
    let mut too_long = escape_sf(64, &[0; 62]);
    too_long[1] = 63;
    assert_eq!(parse_sf(&too_long, 0), None);
    assert_eq!(parse_sf(&escape_sf(12, &[]), 0), None);
    let mut ext = vec![0xF1];
    ext.extend(escape_sf(63, &[7; 61]));
    assert_eq!(parse_sf(&ext, 1), Some(vec![7; 61]));
}

#[test]
fn test_parse_sf_nibble_form_needs_frame_of_8_bytes_or_fewer() {
    assert_eq!(
        parse_sf(&[0x03, 0xAA, 0xBB, 0xCC], 0),
        Some(vec![0xAA, 0xBB, 0xCC])
    );
    let mut padded = [0xCCu8; 8];
    padded[..4].copy_from_slice(&[0x03, 0xAA, 0xBB, 0xCC]);
    assert_eq!(parse_sf(&padded, 0), Some(vec![0xAA, 0xBB, 0xCC]));
    // The same PDU padded to a 12-byte FD frame: ignored.
    let mut fd = [0xCCu8; 12];
    fd[..4].copy_from_slice(&[0x03, 0xAA, 0xBB, 0xCC]);
    assert_eq!(parse_sf(&fd, 0), None);
}

// An escape-form SF needs a second PCI byte for SF_DL; with the addressing
// overhead taking all but one byte of the frame, there is none.
#[test]
fn test_parse_escape_sf_without_its_sf_dl_byte() {
    assert!(matches!(
        IsoTpFrame::parse(&[0u8; 12], 11),
        Err(IsoTpError::InvalidFrame)
    ));
}

#[test]
fn test_parse_accepts_every_sf_we_build() {
    for overhead in 0..=1 {
        for len in 1..=7 - overhead {
            let data: Vec<u8> = (0u8..).take(len).collect();
            let mut buf = [0xCCu8; 8];
            buf[0] = 0xF1;
            let n = build_sf(&mut buf, &data, overhead);
            assert_eq!(parse_sf(&buf[..n], overhead).as_ref(), Some(&data));
            assert_eq!(parse_sf(&buf, overhead).as_ref(), Some(&data));
        }
        for len in 1..=62 - overhead {
            let data: Vec<u8> = (0u8..).take(len).collect();
            let mut buf = [0xCCu8; 64];
            buf[0] = 0xF1;
            let n = build_sf_fd(&mut buf, &data, overhead);
            let frame = &buf[..next_fd_dlc(n)];
            assert_eq!(
                parse_sf(frame, overhead).as_ref(),
                Some(&data),
                "FD SF of {len} bytes, overhead {overhead}"
            );
        }
    }
}

/// A classic escape-form SF carrying [AA BB CC]: invalid in 5 bytes.
pub fn classic_escape_sf() -> CanFrame {
    make_frame(rx_id(), &[0x00, 0x03, 0xAA, 0xBB, 0xCC])
}

#[test]
fn test_receive_ignores_escape_sf_in_classic_frame() {
    assert_receive_ignores(&[classic_escape_sf()]);
}

/// SFs in the wrong form for their FD frame length, each carrying [AA BB CC].
pub fn fd_wrong_form_sfs() -> Vec<Frame> {
    vec![
        // Escape form in a 5-byte FD frame.
        fd_frame(&[0x00, 0x03, 0xAA, 0xBB, 0xCC]),
        // Nibble form in a 12-byte FD frame.
        fd_frame(&[0x03, 0xAA, 0xBB, 0xCC, 0, 0, 0, 0, 0]),
    ]
}

#[test]
fn test_fd_receive_ignores_sf_in_wrong_form() {
    assert_fd_receive_ignores(&fd_wrong_form_sfs());
}

// ---------------------------------------------------------------------------
// A timeout of Duration::MAX must not overflow a deadline: like the async
// channels and tokio::time::timeout, the sync channels cap it 30 years ahead.
// ---------------------------------------------------------------------------

fn duration_max_config() -> IsoTpConfig {
    IsoTpConfig {
        timeout: Duration::MAX,
        ..default_config()
    }
}

#[test]
fn test_receive_with_timeout_duration_max() {
    let a: Vec<u8> = (0..13).collect();
    let frames = vec![classic_ff(13, &a), classic_cf(1, &a[6..13])];
    let mut ch = IsoTpChannel::new(MockChannel::new(frames), duration_max_config());
    assert_eq!(ch.receive().unwrap(), a);
}

#[test]
fn test_send_with_timeout_duration_max() {
    let payload: Vec<u8> = (0..20).collect();
    let mut ch = IsoTpChannel::new(MockChannel::new(vec![fc_cts()]), duration_max_config());
    ch.send(&payload).unwrap();
}

#[test]
fn test_fd_receive_with_timeout_duration_max() {
    let a: Vec<u8> = (0u8..125).collect();
    let frames = vec![fd_ff(125, &a), fd_cf(1, &a[62..])];
    let mut ch = IsoTpFdChannel::new(FdMockChannel::new(frames), duration_max_config());
    assert_eq!(ch.receive().unwrap(), a);
}

#[test]
fn test_fd_send_with_timeout_duration_max() {
    let payload: Vec<u8> = (0u8..126).collect();
    let mock = FdMockChannel::new(vec![fd_frame(&[0x30, 0, 0])]);
    let mut ch = IsoTpFdChannel::new(mock, duration_max_config());
    ch.send(&payload).unwrap();
}

// ---------------------------------------------------------------------------
// A First Frame must fill its CAN frame, as in the Linux kernel's
// isotp_rcv_ff: 8 bytes, or a CAN FD length from 12 to 64.
// ---------------------------------------------------------------------------

/// FF_DL 0 in the 32-bit form, in 6 bytes: nothing carried.
pub const SHORT_FF: [u8; 6] = [0x10, 0x00, 0x00, 0x00, 0x00, 0x00];

#[test]
fn test_parse_ff_must_fill_its_frame() {
    for len in 2..=64 {
        // FF_DL 4095, more than any FF carries.
        let mut raw = vec![0xD0u8; len];
        raw[..2].copy_from_slice(&[0x1F, 0xFF]);
        let fills = matches!(len, 8 | 12 | 16 | 20 | 24 | 32 | 48 | 64);
        assert_eq!(parse_ff(&raw, 0).is_some(), fills, "FF of {len} bytes");
    }
    assert_eq!(parse_ff(&SHORT_FF, 0), None);
    // Extended addressing: the TA byte counts toward the frame's length.
    assert_eq!(parse_ff(&[0xF1, 0x1F, 0xFF, 1, 2, 3, 4], 1), None);
    assert_eq!(
        parse_ff(&[0xF1, 0x1F, 0xFF, 1, 2, 3, 4, 5], 1),
        Some((4095, 5))
    );
}

#[test]
fn test_receive_ignores_short_ff() {
    assert_receive_ignores(&[make_frame(rx_id(), &SHORT_FF)]);
}

#[test]
fn test_fd_receive_ignores_short_ff() {
    assert_fd_receive_ignores(&[fd_frame(&SHORT_FF)]);
}

// ---------------------------------------------------------------------------
// With a block size of 0 (no limit), a message of more than u16::MAX CFs
// must not overflow the block count (a debug build panicked).
// ---------------------------------------------------------------------------

/// CFs in the messages below: one more than a u16 can count.
pub const MANY_CFS: usize = 65_536;

/// A message of `MANY_CFS` CFs after a long FF that carries `ff_bytes`.
#[allow(clippy::cast_possible_truncation)]
pub fn many_cf_payload(ff_bytes: usize, cf_bytes: usize) -> Vec<u8> {
    (0..ff_bytes + MANY_CFS * cf_bytes)
        .map(|i| i as u8)
        .collect()
}

/// The frames of a long classic message: an FF carrying 2 bytes, then CFs.
#[allow(clippy::cast_possible_truncation)]
pub fn classic_message(payload: &[u8]) -> Vec<CanFrame> {
    let mut frames = vec![classic_ff(payload.len(), payload)];
    for (i, chunk) in payload[2..].chunks(7).enumerate() {
        frames.push(classic_cf(((i + 1) & 0x0F) as u8, chunk));
    }
    frames
}

/// The frames of a long FD message: an FF carrying 58 bytes, then CFs.
#[allow(clippy::cast_possible_truncation)]
pub fn fd_message(payload: &[u8]) -> Vec<Frame> {
    let mut frames = vec![fd_ff(payload.len(), payload)];
    for (i, chunk) in payload[58..].chunks(63).enumerate() {
        frames.push(fd_cf(((i + 1) & 0x0F) as u8, chunk));
    }
    frames
}

#[test]
fn test_send_65536_cfs_with_block_size_0() {
    let payload = many_cf_payload(2, 7);
    let mut ch = IsoTpChannel::new(MockChannel::new(vec![fc_cts()]), default_config());
    ch.send(&payload).unwrap();
    assert_eq!(ch.into_inner().tx_frames.len(), 1 + MANY_CFS);
}

#[test]
fn test_receive_65536_cfs_with_block_size_0() {
    let payload = many_cf_payload(2, 7);
    let mock = MockChannel::new(classic_message(&payload));
    let mut ch = IsoTpChannel::new(mock, default_config());
    assert_eq!(ch.receive().unwrap(), payload);
}

#[test]
fn test_fd_send_65536_cfs_with_block_size_0() {
    let payload = many_cf_payload(58, 63);
    let mock = FdMockChannel::new(vec![fd_frame(&[0x30, 0, 0])]);
    let mut ch = IsoTpFdChannel::new(mock, default_config());
    ch.send(&payload).unwrap();
    assert_eq!(ch.into_inner().tx_frames.len(), 1 + MANY_CFS);
}

#[test]
fn test_fd_receive_65536_cfs_with_block_size_0() {
    let payload = many_cf_payload(58, 63);
    let mock = FdMockChannel::new(fd_message(&payload));
    let mut ch = IsoTpFdChannel::new(mock, default_config());
    assert_eq!(ch.receive().unwrap(), payload);
}
