use std::collections::VecDeque;
use std::time::{Duration, Instant};

use can_hal::async_channel::{AsyncReceive, AsyncReceiveFd, AsyncTransmit, AsyncTransmitFd};
use can_hal::frame::{CanFdFrame, CanFrame, Frame, Timestamped};
use can_hal::CanId;

use crate::async_channel::AsyncIsoTpChannel;
use crate::async_fd_channel::AsyncIsoTpFdChannel;
use crate::config::IsoTpConfig;
use crate::error::IsoTpError;
use crate::frame::FcFlag;
use crate::tests::*;

// ---------------------------------------------------------------------------
// Mock error type
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct MockError;

impl std::fmt::Display for MockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "mock CAN error")
    }
}

impl std::error::Error for MockError {}

// ---------------------------------------------------------------------------
// Async mock CAN channel
// ---------------------------------------------------------------------------

struct AsyncMockChannel {
    tx_frames: Vec<CanFrame>,
    rx_queue: VecDeque<CanFrame>,
    /// How long each `receive()` takes to hand out its frame.
    rx_delay: Duration,
}

impl AsyncMockChannel {
    const fn new() -> Self {
        Self {
            tx_frames: Vec::new(),
            rx_queue: VecDeque::new(),
            rx_delay: Duration::ZERO,
        }
    }
}

impl AsyncTransmit for AsyncMockChannel {
    type Error = MockError;

    fn transmit(
        &mut self,
        frame: &CanFrame,
    ) -> impl std::future::Future<Output = Result<(), MockError>> + Send {
        self.tx_frames.push(frame.clone());
        async { Ok(()) }
    }
}

impl AsyncReceive for AsyncMockChannel {
    type Error = MockError;
    type Timestamp = Instant;

    fn receive(
        &mut self,
    ) -> impl std::future::Future<Output = Result<Timestamped<CanFrame, Instant>, MockError>> + Send
    {
        let frame = self
            .rx_queue
            .pop_front()
            .expect("rx_queue unexpectedly empty");
        let delay = self.rx_delay;
        async move {
            tokio::time::sleep(delay).await;
            Ok(Timestamped::new(frame, Instant::now()))
        }
    }
}

/// CAN FD counterpart of [`AsyncMockChannel`].
struct AsyncFdMockChannel {
    tx_frames: Vec<CanFdFrame>,
    rx_queue: VecDeque<Frame>,
    /// How long each `receive_fd()` takes to hand out its frame.
    rx_delay: Duration,
}

impl AsyncFdMockChannel {
    fn new(rx: impl IntoIterator<Item = Frame>) -> Self {
        Self {
            tx_frames: Vec::new(),
            rx_queue: rx.into_iter().collect(),
            rx_delay: Duration::ZERO,
        }
    }
}

impl AsyncTransmitFd for AsyncFdMockChannel {
    type Error = MockError;

    fn transmit_fd(
        &mut self,
        frame: &CanFdFrame,
    ) -> impl std::future::Future<Output = Result<(), MockError>> + Send {
        self.tx_frames.push(frame.clone());
        async { Ok(()) }
    }
}

impl AsyncReceiveFd for AsyncFdMockChannel {
    type Error = MockError;
    type Timestamp = Instant;

    fn receive_fd(
        &mut self,
    ) -> impl std::future::Future<Output = Result<Timestamped<Frame, Instant>, MockError>> + Send
    {
        let frame = self
            .rx_queue
            .pop_front()
            .expect("rx_queue unexpectedly empty");
        let delay = self.rx_delay;
        async move {
            tokio::time::sleep(delay).await;
            Ok(Timestamped::new(frame, Instant::now()))
        }
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
    IsoTpConfig::new(tx_id(), rx_id())
}

/// Build a CAN frame with the given id and raw bytes.
fn can_frame(id: CanId, data: &[u8]) -> CanFrame {
    CanFrame::new(id, data).unwrap()
}

/// An [`AsyncMockChannel`] that will hand out `frames` in order.
fn mock_with(frames: impl IntoIterator<Item = CanFrame>) -> AsyncMockChannel {
    let mut mock = AsyncMockChannel::new();
    mock.rx_queue.extend(frames);
    mock
}

/// Build a Flow Control frame (CTS, bs=0, st_min=0) from the receiver side.
fn fc_cts_frame() -> CanFrame {
    let mut buf = [0u8; 8];
    crate::frame::build_fc(&mut buf, FcFlag::ContinueToSend, 0, 0, 0);
    can_frame(rx_id(), &buf[..3])
}

/// Build a Flow Control frame with Wait flag.
fn fc_wait_frame() -> CanFrame {
    let mut buf = [0u8; 8];
    crate::frame::build_fc(&mut buf, FcFlag::Wait, 0, 0, 0);
    can_frame(rx_id(), &buf[..3])
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_send_single_frame() {
    let mock = AsyncMockChannel::new();
    let config = default_config();
    let mut isotp = AsyncIsoTpChannel::new(mock, config);

    let payload = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
    isotp.send(&payload).await.unwrap();

    let inner = isotp.into_inner();
    assert_eq!(inner.tx_frames.len(), 1);

    let sent = &inner.tx_frames[0];
    assert_eq!(sent.id(), tx_id());

    // PCI byte: 0x07 (SF, DL=7)
    let data = sent.data();
    assert_eq!(data[0] >> 4, 0x0); // Single Frame type nibble
    assert_eq!(data[0] & 0x0F, 7); // SF_DL = 7
    assert_eq!(&data[1..8], &payload);
}

#[tokio::test]
async fn test_async_send_multi_frame() {
    let mut mock = AsyncMockChannel::new();

    // Pre-load an FC(CTS) frame so the sender can proceed after FF.
    mock.rx_queue.push_back(fc_cts_frame());

    let config = default_config();
    let mut isotp = AsyncIsoTpChannel::new(mock, config);

    // 20 bytes: FF carries 6 data bytes, then we need CFs for the remaining 14.
    let payload: Vec<u8> = (0..20).collect();
    isotp.send(&payload).await.unwrap();

    let inner = isotp.into_inner();

    // Expect: 1 FF + ceil(14/7) = 1 FF + 2 CFs = 3 frames total
    assert_eq!(inner.tx_frames.len(), 3, "expected FF + 2 CFs");

    // --- First Frame ---
    let ff = &inner.tx_frames[0];
    assert_eq!(ff.id(), tx_id());
    let ff_data = ff.data();
    assert_eq!(ff_data[0] >> 4, 0x1); // FF type nibble
    let total_len = (((ff_data[0] & 0x0F) as usize) << 8) | (ff_data[1] as usize);
    assert_eq!(total_len, 20);
    // FF payload: bytes 0..6 of original data
    assert_eq!(&ff_data[2..8], &payload[0..6]);

    // --- Consecutive Frame 1 (SN=1) ---
    let cf1 = &inner.tx_frames[1];
    assert_eq!(cf1.id(), tx_id());
    let cf1_data = cf1.data();
    assert_eq!(cf1_data[0] >> 4, 0x2); // CF type nibble
    assert_eq!(cf1_data[0] & 0x0F, 1); // SN = 1
    assert_eq!(&cf1_data[1..8], &payload[6..13]);

    // --- Consecutive Frame 2 (SN=2) ---
    let cf2 = &inner.tx_frames[2];
    assert_eq!(cf2.id(), tx_id());
    let cf2_data = cf2.data();
    assert_eq!(cf2_data[0] >> 4, 0x2); // CF type nibble
    assert_eq!(cf2_data[0] & 0x0F, 2); // SN = 2
    assert_eq!(&cf2_data[1..], &payload[13..20]);
}

#[tokio::test]
async fn test_async_receive_single_frame() {
    // Build a single-frame ISO-TP message with 5 bytes of data.
    let payload = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
    let mut sf_buf = [0u8; 8];
    crate::frame::build_sf(&mut sf_buf, &payload, 0);
    let sf = can_frame(rx_id(), &sf_buf[..=payload.len()]);

    let mut mock = AsyncMockChannel::new();
    mock.rx_queue.push_back(sf);

    let config = default_config();
    let mut isotp = AsyncIsoTpChannel::new(mock, config);

    let received = isotp.receive().await.unwrap();
    assert_eq!(received, payload);
}

#[tokio::test]
async fn test_async_receive_multi_frame() {
    // 20 bytes of data, sent as FF + CFs.
    let payload: Vec<u8> = (0..20).collect();

    // FF: total_len=20, carries first 6 bytes
    let mut ff_buf = [0u8; 8];
    crate::frame::build_ff(&mut ff_buf, &payload, 20, 0);
    let ff = can_frame(rx_id(), &ff_buf);

    // CF1: SN=1, bytes 6..13
    let mut cf1_buf = [0u8; 8];
    crate::frame::build_cf(&mut cf1_buf, &payload[6..13], 1, 0);
    let cf1 = can_frame(rx_id(), &cf1_buf);

    // CF2: SN=2, bytes 13..20
    let mut cf2_buf = [0u8; 8];
    crate::frame::build_cf(&mut cf2_buf, &payload[13..20], 2, 0);
    let cf2 = can_frame(rx_id(), &cf2_buf);

    let mut mock = AsyncMockChannel::new();
    mock.rx_queue.push_back(ff);
    mock.rx_queue.push_back(cf1);
    mock.rx_queue.push_back(cf2);

    let config = default_config();
    let mut isotp = AsyncIsoTpChannel::new(mock, config);

    let received = isotp.receive().await.unwrap();
    assert_eq!(received, payload);

    // Verify that an FC(CTS) frame was transmitted.
    let inner = isotp.into_inner();
    assert_eq!(inner.tx_frames.len(), 1);
    let fc_data = inner.tx_frames[0].data();
    assert_eq!(fc_data[0] & 0xF0, 0x30); // FC type nibble
    assert_eq!(fc_data[0] & 0x0F, 0x00); // CTS flag
}

#[tokio::test]
async fn test_async_send_fc_wait_limit() {
    let mut mock = AsyncMockChannel::new();

    // Pre-load 4 FC(Wait) frames. With max_fc_wait=3, the 3rd Wait should
    // trigger WaitLimitExceeded *before* reading the 4th.
    for _ in 0..4 {
        mock.rx_queue.push_back(fc_wait_frame());
    }

    let mut config = default_config();
    config.max_fc_wait = 3;

    let mut isotp = AsyncIsoTpChannel::new(mock, config);

    // 20 bytes triggers multi-frame -> will need FC after FF.
    let payload: Vec<u8> = (0..20).collect();
    let result = isotp.send(&payload).await;

    assert!(result.is_err(), "expected WaitLimitExceeded error");
    match result.unwrap_err() {
        IsoTpError::WaitLimitExceeded => {} // expected
        other => panic!("expected WaitLimitExceeded, got: {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// An SF or FF during a reception (ISO 15765-2 unexpected N_PDU): the
// reception in progress is dropped and the new frame is processed.
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_receive_sf_aborts_reception() {
    let a: Vec<u8> = (0..13).collect();
    let mock = mock_with([
        classic_ff(13, &a),
        can_frame(rx_id(), &[0x03, 0xAA, 0xBB, 0xCC]),
        classic_cf(1, &a[6..13]),
    ]);
    let mut isotp = AsyncIsoTpChannel::new(mock, default_config());
    assert_eq!(isotp.receive().await.unwrap(), vec![0xAA, 0xBB, 0xCC]);
}

#[tokio::test]
async fn test_async_receive_ff_replaces_partial_message() {
    let a: Vec<u8> = (0..13).collect();
    let b: Vec<u8> = (100..113).collect();
    let mock = mock_with([
        classic_ff(13, &a),
        classic_ff(13, &b),
        classic_cf(1, &b[6..13]),
    ]);
    let mut isotp = AsyncIsoTpChannel::new(mock, default_config());
    assert_eq!(isotp.receive().await.unwrap(), b);
}

#[tokio::test]
async fn test_async_receive_ff_restarts_reception() {
    // See test_receive_ff_restarts_reception in tests.rs.
    let a: Vec<u8> = (0..20).collect();
    let b: Vec<u8> = (100..120).collect();
    let mock = mock_with([
        classic_ff(20, &a),
        classic_cf(1, &a[6..13]),
        classic_ff(20, &b),
        classic_cf(1, &b[6..13]),
        classic_cf(2, &b[13..20]),
    ]);
    let cfg = IsoTpConfig {
        block_size: 2,
        ..default_config()
    };
    let mut isotp = AsyncIsoTpChannel::new(mock, cfg);
    assert_eq!(isotp.receive().await.unwrap(), b);
    let inner = isotp.into_inner();
    assert_eq!(inner.tx_frames.len(), 2);
    assert!(inner.tx_frames.iter().all(|f| f.data()[0] == 0x30));
}

#[tokio::test]
async fn test_async_fd_receive_sf_aborts_reception() {
    let a: Vec<u8> = (0u8..125).collect();
    let mock = AsyncFdMockChannel::new([
        fd_ff(125, &a),
        fd_sf(&[0xAA, 0xBB, 0xCC]),
        fd_cf(1, &a[62..]),
    ]);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, default_config());
    assert_eq!(isotp.receive().await.unwrap(), vec![0xAA, 0xBB, 0xCC]);
}

#[tokio::test]
async fn test_async_fd_receive_ff_replaces_partial_message() {
    let a: Vec<u8> = (0u8..125).collect();
    let b: Vec<u8> = (100u8..225).collect();
    let mock = AsyncFdMockChannel::new([fd_ff(125, &a), fd_ff(125, &b), fd_cf(1, &b[62..])]);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, default_config());
    assert_eq!(isotp.receive().await.unwrap(), b);
}

#[tokio::test]
async fn test_async_fd_receive_ff_restarts_reception() {
    let a: Vec<u8> = (0u8..200).collect();
    let b: Vec<u8> = (50u8..238).collect();
    let mock = AsyncFdMockChannel::new([
        fd_ff(200, &a),
        fd_cf(1, &a[62..125]),
        fd_ff(188, &b),
        fd_cf(1, &b[62..125]),
        fd_cf(2, &b[125..]),
    ]);
    let cfg = IsoTpConfig {
        block_size: 2,
        ..default_config()
    };
    let mut isotp = AsyncIsoTpFdChannel::new(mock, cfg);
    assert_eq!(isotp.receive().await.unwrap(), b);
    let inner = isotp.into_inner();
    assert_eq!(inner.tx_frames.len(), 2);
    assert!(inner.tx_frames.iter().all(|f| f.data()[0] == 0x30));
}

// ---------------------------------------------------------------------------
// Reserved STmin values: the sender waits 127 ms (see tests.rs).
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_send_honors_reserved_st_min_as_127ms() {
    // 20 bytes = FF + 2 CFs, so one STmin gap.
    let mock = mock_with([can_frame(rx_id(), &[0x30, 0, 0xFA])]);
    let mut isotp = AsyncIsoTpChannel::new(mock, default_config());
    let payload: Vec<u8> = (0..20).collect();
    let start = Instant::now();
    isotp.send(&payload).await.unwrap();
    let elapsed = start.elapsed();
    assert!(
        elapsed >= Duration::from_millis(127),
        "send() took {elapsed:?}"
    );
}

#[tokio::test]
async fn test_async_fd_send_honors_reserved_st_min_as_127ms() {
    // 126 bytes = FF (62) + CF (63) + CF (1), so one STmin gap.
    let mock = AsyncFdMockChannel::new([fd_frame(&[0x30, 0, 0xFA])]);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, default_config());
    let payload: Vec<u8> = (0u8..126).collect();
    let start = Instant::now();
    isotp.send(&payload).await.unwrap();
    let elapsed = start.elapsed();
    assert!(
        elapsed >= Duration::from_millis(127),
        "send() took {elapsed:?}"
    );
}

// ---------------------------------------------------------------------------
// N_Cr: restarted by a CF (or a new FF), not by a frame receive() ignores.
// Every frame below arrives 100 ms after it is asked for, and the timeout is
// 250 ms. In the first two tests N_Cr starts with the FF at 100 ms and ends
// at 350 ms, so the FCs on rx_id sent at 200, 300 and 400 ms must not keep
// it alive for the CF at 500 ms.
// ---------------------------------------------------------------------------

fn n_cr_config() -> IsoTpConfig {
    IsoTpConfig {
        timeout: Duration::from_millis(250),
        ..default_config()
    }
}

#[tokio::test]
async fn test_async_receive_ignored_frames_do_not_extend_n_cr() {
    let a: Vec<u8> = (0..13).collect();
    let mut mock = mock_with([
        classic_ff(13, &a),
        fc_cts_frame(),
        fc_cts_frame(),
        fc_cts_frame(),
        classic_cf(1, &a[6..13]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut isotp = AsyncIsoTpChannel::new(mock, n_cr_config());
    let result = isotp.receive().await;
    assert!(
        matches!(result, Err(IsoTpError::Timeout)),
        "expected Timeout, got {result:?}"
    );
}

#[tokio::test]
async fn test_async_fd_receive_ignored_frames_do_not_extend_n_cr() {
    let a: Vec<u8> = (0u8..125).collect();
    let fc = fd_frame(&[0x30, 0, 0]);
    let mut mock = AsyncFdMockChannel::new([
        fd_ff(125, &a),
        fc.clone(),
        fc.clone(),
        fc,
        fd_cf(1, &a[62..]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, n_cr_config());
    let result = isotp.receive().await;
    assert!(
        matches!(result, Err(IsoTpError::Timeout)),
        "expected Timeout, got {result:?}"
    );
}

#[tokio::test]
async fn test_async_receive_ff_restarts_n_cr() {
    // FF(A) at 100 ms, FC at 200, FF(B) at 300, FC at 400, CF1(B) at 500:
    // B's FF must move the end of N_Cr from 350 ms to 550 ms.
    let a: Vec<u8> = (0..20).collect();
    let b: Vec<u8> = (100..113).collect();
    let mut mock = mock_with([
        classic_ff(20, &a),
        fc_cts_frame(),
        classic_ff(13, &b),
        fc_cts_frame(),
        classic_cf(1, &b[6..13]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut isotp = AsyncIsoTpChannel::new(mock, n_cr_config());
    assert_eq!(isotp.receive().await.unwrap(), b);
}

#[tokio::test]
async fn test_async_fd_receive_ff_restarts_n_cr() {
    let a: Vec<u8> = (0u8..200).collect();
    let b: Vec<u8> = (50u8..175).collect();
    let fc = fd_frame(&[0x30, 0, 0]);
    let mut mock = AsyncFdMockChannel::new([
        fd_ff(200, &a),
        fc.clone(),
        fd_ff(125, &b),
        fc,
        fd_cf(1, &b[62..]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, n_cr_config());
    assert_eq!(isotp.receive().await.unwrap(), b);
}

// A timeout of Duration::MAX must not overflow the N_Cr deadline;
// tokio::time::timeout, used for the other waits, caps it instead.

#[tokio::test]
async fn test_async_receive_with_timeout_duration_max() {
    let a: Vec<u8> = (0..13).collect();
    let mock = mock_with([classic_ff(13, &a), classic_cf(1, &a[6..13])]);
    let cfg = IsoTpConfig {
        timeout: Duration::MAX,
        ..default_config()
    };
    let mut isotp = AsyncIsoTpChannel::new(mock, cfg);
    assert_eq!(isotp.receive().await.unwrap(), a);
}

#[tokio::test]
async fn test_async_fd_receive_with_timeout_duration_max() {
    let a: Vec<u8> = (0u8..125).collect();
    let mock = AsyncFdMockChannel::new([fd_ff(125, &a), fd_cf(1, &a[62..])]);
    let cfg = IsoTpConfig {
        timeout: Duration::MAX,
        ..default_config()
    };
    let mut isotp = AsyncIsoTpFdChannel::new(mock, cfg);
    assert_eq!(isotp.receive().await.unwrap(), a);
}

#[tokio::test]
async fn test_async_receive_each_cf_restarts_n_cr() {
    // FF + 3 CFs at 100 ms intervals: 400 ms in all, but never 250 ms
    // between two frames.
    let a: Vec<u8> = (0..27).collect();
    let mut mock = mock_with([
        classic_ff(27, &a),
        classic_cf(1, &a[6..13]),
        classic_cf(2, &a[13..20]),
        classic_cf(3, &a[20..27]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut isotp = AsyncIsoTpChannel::new(mock, n_cr_config());
    assert_eq!(isotp.receive().await.unwrap(), a);
}

#[tokio::test]
async fn test_async_fd_receive_each_cf_restarts_n_cr() {
    let a: Vec<u8> = (0u8..251).collect();
    let mut mock = AsyncFdMockChannel::new([
        fd_ff(251, &a),
        fd_cf(1, &a[62..125]),
        fd_cf(2, &a[125..188]),
        fd_cf(3, &a[188..]),
    ]);
    mock.rx_delay = Duration::from_millis(100);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, n_cr_config());
    assert_eq!(isotp.receive().await.unwrap(), a);
}

// ---------------------------------------------------------------------------
// Frames on rx_id that aren't valid N_PDUs are ignored (see tests.rs).
// ---------------------------------------------------------------------------

/// Check that `receive()` ignores `junk`, both while idle and mid-message: it
/// sends no FC for it and returns the message around it.
async fn assert_receive_ignores(junk: &[CanFrame]) {
    let mut frames = junk.to_vec();
    frames.push(can_frame(rx_id(), &[0x02, 0x11, 0x22]));
    let mut isotp = AsyncIsoTpChannel::new(mock_with(frames), default_config());
    assert_eq!(isotp.receive().await.unwrap(), vec![0x11, 0x22]);
    assert!(isotp.into_inner().tx_frames.is_empty());

    let a: Vec<u8> = (0..13).collect();
    let mut frames = vec![classic_ff(13, &a)];
    frames.extend_from_slice(junk);
    frames.push(classic_cf(1, &a[6..13]));
    let mut isotp = AsyncIsoTpChannel::new(mock_with(frames), default_config());
    assert_eq!(isotp.receive().await.unwrap(), a);
    // Only the FF got an FC.
    assert_eq!(isotp.into_inner().tx_frames.len(), 1);
}

/// [`assert_receive_ignores`] for the FD channel.
async fn assert_fd_receive_ignores(junk: &[Frame]) {
    let mut frames = junk.to_vec();
    frames.push(fd_sf(&[0x11, 0x22]));
    let mut isotp = AsyncIsoTpFdChannel::new(AsyncFdMockChannel::new(frames), default_config());
    assert_eq!(isotp.receive().await.unwrap(), vec![0x11, 0x22]);
    assert!(isotp.into_inner().tx_frames.is_empty());

    let a: Vec<u8> = (0u8..125).collect();
    let mut frames = vec![fd_ff(125, &a)];
    frames.extend_from_slice(junk);
    frames.push(fd_cf(1, &a[62..]));
    let mut isotp = AsyncIsoTpFdChannel::new(AsyncFdMockChannel::new(frames), default_config());
    assert_eq!(isotp.receive().await.unwrap(), a);
    // Only the FF got an FC.
    assert_eq!(isotp.into_inner().tx_frames.len(), 1);
}

#[tokio::test]
async fn test_async_receive_ignores_invalid_frames() {
    assert_receive_ignores(&invalid_frames()).await;
}

#[tokio::test]
async fn test_async_fd_receive_ignores_invalid_frames() {
    assert_fd_receive_ignores(&invalid_fd_frames()).await;
}

// ---------------------------------------------------------------------------
// While send() waits for an FC, a frame that fails to parse is ignored unless
// it is an FC (see tests.rs).
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_send_ignores_invalid_non_fc_frames() {
    let mut rx = invalid_non_fc_frames();
    rx.push(fc_cts_frame());
    let mut isotp = AsyncIsoTpChannel::new(mock_with(rx), default_config());
    let payload: Vec<u8> = (0..20).collect();
    isotp.send(&payload).await.unwrap();
    // FF + 2 CFs.
    assert_eq!(isotp.into_inner().tx_frames.len(), 3);
}

#[tokio::test]
async fn test_async_send_fails_on_malformed_fc() {
    for pdu in INVALID_PDUS.iter().filter(|d| is_fc_pci(d)) {
        let mock = mock_with([can_frame(rx_id(), pdu)]);
        let mut isotp = AsyncIsoTpChannel::new(mock, default_config());
        let payload: Vec<u8> = (0..20).collect();
        let result = isotp.send(&payload).await;
        assert!(
            matches!(result, Err(IsoTpError::InvalidFrame)),
            "FC {pdu:02X?}: expected InvalidFrame, got {result:?}"
        );
    }
}

#[tokio::test]
async fn test_async_fd_send_ignores_invalid_non_fc_frames() {
    let mut rx: Vec<Frame> = INVALID_PDUS
        .iter()
        .filter(|d| !is_fc_pci(d))
        .map(|d| fd_frame(d))
        .collect();
    rx.push(fd_frame(&[0x30, 0, 0]));
    let mut isotp = AsyncIsoTpFdChannel::new(AsyncFdMockChannel::new(rx), default_config());
    let payload: Vec<u8> = (0u8..126).collect();
    isotp.send(&payload).await.unwrap();
    // FF + 2 CFs.
    assert_eq!(isotp.into_inner().tx_frames.len(), 3);
}

#[tokio::test]
async fn test_async_fd_send_fails_on_malformed_fc() {
    for pdu in INVALID_PDUS.iter().filter(|d| is_fc_pci(d)) {
        let mock = AsyncFdMockChannel::new([fd_frame(pdu)]);
        let mut isotp = AsyncIsoTpFdChannel::new(mock, default_config());
        let payload: Vec<u8> = (0u8..126).collect();
        let result = isotp.send(&payload).await;
        assert!(
            matches!(result, Err(IsoTpError::InvalidFrame)),
            "FC {pdu:02X?}: expected InvalidFrame, got {result:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// A First Frame whose FF_DL is below the payload it carries itself is
// ignored (see tests.rs).
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_receive_ignores_ff_dl_below_ff_payload() {
    assert_receive_ignores(&[classic_ff(3, &[1, 2, 3, 4, 5, 6])]).await;
}

#[tokio::test]
async fn test_async_fd_receive_ignores_ff_dl_below_ff_payload() {
    let carried: Vec<u8> = (0u8..62).collect();
    assert_fd_receive_ignores(&[fd_ff(60, &carried)]).await;
}

#[tokio::test]
async fn test_async_fd_receive_cuts_ff_payload_to_ff_dl() {
    let carried: Vec<u8> = (0u8..62).collect();
    let mock = AsyncFdMockChannel::new([fd_ff(61, &carried)]);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, default_config());
    assert_eq!(isotp.receive().await.unwrap(), carried[..61]);
}

// ---------------------------------------------------------------------------
// Single Frame forms: an SF in the wrong form for its frame length is
// ignored (see tests.rs).
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_receive_ignores_escape_sf_in_classic_frame() {
    assert_receive_ignores(&[classic_escape_sf()]).await;
}

#[tokio::test]
async fn test_async_fd_receive_ignores_sf_in_wrong_form() {
    assert_fd_receive_ignores(&fd_wrong_form_sfs()).await;
}

// ---------------------------------------------------------------------------
// send() with a timeout of Duration::MAX (receive() is tested with the N_Cr
// tests above).
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_send_with_timeout_duration_max() {
    let cfg = IsoTpConfig {
        timeout: Duration::MAX,
        ..default_config()
    };
    let mut isotp = AsyncIsoTpChannel::new(mock_with([fc_cts_frame()]), cfg);
    let payload: Vec<u8> = (0..20).collect();
    isotp.send(&payload).await.unwrap();
}

#[tokio::test]
async fn test_async_fd_send_with_timeout_duration_max() {
    let cfg = IsoTpConfig {
        timeout: Duration::MAX,
        ..default_config()
    };
    let mock = AsyncFdMockChannel::new([fd_frame(&[0x30, 0, 0])]);
    let mut isotp = AsyncIsoTpFdChannel::new(mock, cfg);
    let payload: Vec<u8> = (0u8..126).collect();
    isotp.send(&payload).await.unwrap();
}

// ---------------------------------------------------------------------------
// A First Frame must fill its CAN frame (see tests.rs).
// ---------------------------------------------------------------------------

#[tokio::test]
async fn test_async_receive_ignores_short_ff() {
    assert_receive_ignores(&[can_frame(rx_id(), &SHORT_FF)]).await;
}

#[tokio::test]
async fn test_async_fd_receive_ignores_short_ff() {
    assert_fd_receive_ignores(&[fd_frame(&SHORT_FF)]).await;
}
