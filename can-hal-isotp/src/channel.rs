use std::time::{Duration, Instant};

use can_hal::channel::{Receive, Transmit};
use can_hal::error::CanError;
use can_hal::frame::CanFrame;

use crate::config::{AddressingMode, IsoTpConfig};
use crate::error::IsoTpError;
use crate::frame::{self, has_fc_pci, interpret_st_min, FcFlag, IsoTpFrame, MAX_RX_PREALLOC};

/// Synchronous ISO-TP transport channel wrapping a `can-hal` CAN channel.
pub struct IsoTpChannel<C> {
    channel: C,
    config: IsoTpConfig,
}

impl<C> IsoTpChannel<C> {
    /// Create a new ISO-TP channel around a CAN channel with the given config.
    pub const fn new(channel: C, config: IsoTpConfig) -> Self {
        Self { channel, config }
    }

    /// Consume this ISO-TP channel and return the inner CAN channel.
    pub fn into_inner(self) -> C {
        self.channel
    }
}

impl<C, E> IsoTpChannel<C>
where
    C: Transmit<Error = E> + Receive<Error = E>,
    E: CanError,
{
    /// Send an ISO-TP message. Handles segmentation (SF/FF+CF) and flow control.
    pub fn send(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let max_sf = 7 - overhead;

        // ISO 15765-2 maximum: 4,294,967,295 bytes (32-bit length in long FF).
        if data.len() as u64 > u64::from(u32::MAX) {
            return Err(IsoTpError::PayloadTooLarge);
        }

        if data.len() <= max_sf {
            // Single Frame
            let mut buf = [0u8; 8];
            self.write_ta(&mut buf);
            let len = frame::build_sf(&mut buf, data, overhead);
            self.transmit_padded(&mut buf, len)?;
            return Ok(());
        }

        // First Frame
        let mut buf = [0u8; 8];
        self.write_ta(&mut buf);
        let ff_header_size = if data.len() <= 0xFFF {
            overhead + 2
        } else {
            overhead + 6
        };
        let ff_data_len = 8 - ff_header_size;
        frame::build_ff(&mut buf, data, data.len(), overhead);
        self.transmit_padded(&mut buf, 8)?;

        let mut offset = ff_data_len;
        let mut sn: u8 = 1;

        // Wait for Flow Control
        let (mut fc_bs, mut st_min_dur) = self.wait_for_fc()?;
        let mut block_count: u16 = 0;

        // Send Consecutive Frames
        while offset < data.len() {
            let cf_data_capacity = 7 - overhead;
            let end = (offset + cf_data_capacity).min(data.len());
            let chunk = &data[offset..end];

            let mut cf_buf = [0u8; 8];
            self.write_ta(&mut cf_buf);
            let cf_len = frame::build_cf(&mut cf_buf, chunk, sn, overhead);
            self.transmit_padded(&mut cf_buf, cf_len)?;

            offset = end;
            sn = (sn + 1) & 0x0F;
            // Counted only under a block size: with BS 0 nothing resets
            // the count, so a long message would overflow it.
            if fc_bs > 0 {
                block_count += 1;
            }

            if offset < data.len() {
                // STmin delay between consecutive frames
                if !st_min_dur.is_zero() {
                    std::thread::sleep(st_min_dur);
                }

                // If block_size > 0, wait for FC every BS frames.
                // The receiver may change BS and STmin in subsequent FC frames
                // (ISO 15765-2), so we must honor the latest values.
                if fc_bs > 0 && block_count >= u16::from(fc_bs) {
                    let (new_bs, new_st) = self.wait_for_fc()?;
                    fc_bs = new_bs;
                    st_min_dur = new_st;
                    block_count = 0;
                }
            }
        }

        Ok(())
    }

    /// Send a functionally addressed single-frame request.
    ///
    /// Functional addressing is broadcast-only and restricted to single frames
    /// (≤ 7 bytes for normal addressing, ≤ 6 for extended). Returns
    /// `IsoTpError::PayloadTooLarge` if data exceeds the SF limit, or
    /// `IsoTpError::InvalidFrame` if `config.functional_id` is not set.
    pub fn send_functional(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let max_sf = 7 - overhead;
        if data.len() > max_sf {
            return Err(IsoTpError::PayloadTooLarge);
        }
        let functional_id = self.config.functional_id.ok_or(IsoTpError::InvalidFrame)?;
        let mut buf = [0u8; 8];
        self.write_ta(&mut buf);
        let len = frame::build_sf(&mut buf, data, overhead);
        let send_len = self.config.padding.map_or(len, |pad| {
            buf[len..].fill(pad);
            8
        });
        let frame =
            CanFrame::new(functional_id, &buf[..send_len]).ok_or(IsoTpError::InvalidFrame)?;
        self.channel.transmit(&frame).map_err(IsoTpError::CanError)
    }

    /// Receive an ISO-TP message. Handles reassembly from SF or FF+CF sequences.
    pub fn receive(&mut self) -> Result<Vec<u8>, IsoTpError<E>> {
        let overhead = self.config.overhead();
        let deadline = self.config.deadline();

        // Wait for first frame (SF or FF) from rx_id
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(IsoTpError::Timeout);
            }

            let Some(ts_frame) = self
                .channel
                .receive_timeout(remaining)
                .map_err(IsoTpError::CanError)?
            else {
                continue;
            };

            let can_frame = ts_frame.into_frame();
            if can_frame.id() != self.config.rx_id || can_frame.is_remote_frame() {
                continue;
            }
            if !self.check_ta(can_frame.data()) {
                continue;
            }

            // ISO 15765-2 has a receiver ignore a frame that isn't a valid
            // N_PDU (an unknown PCI type, a bad length or flow status), as the
            // Linux kernel does.
            let Ok(isotp) = IsoTpFrame::parse(can_frame.data(), overhead) else {
                continue;
            };

            match isotp {
                IsoTpFrame::SingleFrame { data } => {
                    return Ok(data.to_vec());
                }
                IsoTpFrame::FirstFrame {
                    mut total_len,
                    data,
                } => {
                    let mut result = Vec::with_capacity(total_len.min(MAX_RX_PREALLOC));
                    result.extend_from_slice(data);

                    // Send FC(CTS)
                    self.send_fc(FcFlag::ContinueToSend)?;

                    let mut expected_sn: u8 = 1;
                    let mut block_count: u16 = 0;

                    // Receive Consecutive Frames
                    let mut cf_deadline = self.config.deadline();
                    while result.len() < total_len {
                        let remaining = cf_deadline.saturating_duration_since(Instant::now());
                        if remaining.is_zero() {
                            return Err(IsoTpError::Timeout);
                        }

                        let Some(ts) = self
                            .channel
                            .receive_timeout(remaining)
                            .map_err(IsoTpError::CanError)?
                        else {
                            continue;
                        };

                        let cf = ts.into_frame();
                        if cf.id() != self.config.rx_id || cf.is_remote_frame() {
                            continue;
                        }
                        if !self.check_ta(cf.data()) {
                            continue;
                        }

                        // Ignored, as above; the CF deadline stands.
                        let Ok(parsed) = IsoTpFrame::parse(cf.data(), overhead) else {
                            continue;
                        };

                        match parsed {
                            IsoTpFrame::ConsecutiveFrame { sn, data } => {
                                if sn != expected_sn {
                                    return Err(IsoTpError::SequenceError {
                                        expected: expected_sn,
                                        got: sn,
                                    });
                                }

                                let bytes_needed = total_len - result.len();
                                let take = data.len().min(bytes_needed);
                                result.extend_from_slice(&data[..take]);

                                expected_sn = (expected_sn + 1) & 0x0F;
                                // Counted only under a block size, as in send().
                                if self.config.block_size > 0 {
                                    block_count += 1;
                                }

                                // Reset CF deadline on each successful CF
                                cf_deadline = self.config.deadline();

                                // Send FC every block_size frames (if block_size > 0)
                                if self.config.block_size > 0
                                    && block_count >= u16::from(self.config.block_size)
                                    && result.len() < total_len
                                {
                                    self.send_fc(FcFlag::ContinueToSend)?;
                                    block_count = 0;
                                }
                            }
                            // ISO 15765-2 (unexpected N_PDU): an SF or FF that
                            // arrives during a reception ends it. The partial
                            // message is dropped, and the new frame is handled
                            // as the start of a new reception.
                            IsoTpFrame::SingleFrame { data } => return Ok(data.to_vec()),
                            IsoTpFrame::FirstFrame {
                                total_len: new_len,
                                data,
                            } => {
                                total_len = new_len;
                                result.clear();
                                result.extend_from_slice(data);
                                self.send_fc(FcFlag::ContinueToSend)?;
                                expected_sn = 1;
                                block_count = 0;
                                cf_deadline = self.config.deadline();
                            }
                            // An FC is meant for a sender; a receiver ignores it.
                            IsoTpFrame::FlowControl { .. } => {}
                        }
                    }

                    return Ok(result);
                }
                // A CF with no reception in progress, or an FC: ignore it.
                _ => {}
            }
        }
    }

    /// Wait for a Flow Control frame. Returns (block_size, st_min_duration).
    fn wait_for_fc(&mut self) -> Result<(u8, Duration), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let deadline = self.config.deadline();
        let mut wait_count: u8 = 0;

        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(IsoTpError::Timeout);
            }

            let Some(ts_frame) = self
                .channel
                .receive_timeout(remaining)
                .map_err(IsoTpError::CanError)?
            else {
                continue;
            };

            let can_frame = ts_frame.into_frame();
            if can_frame.id() != self.config.rx_id || can_frame.is_remote_frame() {
                continue;
            }
            if !self.check_ta(can_frame.data()) {
                continue;
            }

            let isotp = match IsoTpFrame::parse(can_frame.data(), overhead) {
                Ok(isotp) => isotp,
                // A malformed FC ends the transfer, as in the Linux kernel;
                // any other frame that doesn't parse isn't for a sender.
                Err(_) if has_fc_pci(can_frame.data(), overhead) => {
                    return Err(IsoTpError::InvalidFrame);
                }
                Err(_) => continue,
            };

            if let IsoTpFrame::FlowControl {
                flag,
                block_size,
                st_min,
            } = isotp
            {
                match flag {
                    FcFlag::ContinueToSend => {
                        let st_dur = interpret_st_min(st_min);
                        return Ok((block_size, st_dur));
                    }
                    FcFlag::Wait => {
                        if self.config.max_fc_wait > 0 {
                            wait_count += 1;
                            if wait_count >= self.config.max_fc_wait {
                                return Err(IsoTpError::WaitLimitExceeded);
                            }
                        }
                    }
                    FcFlag::Overflow => {
                        return Err(IsoTpError::BufferOverflow);
                    }
                }
            } else {
                // Skip non-FC frames
            }
        }
    }

    /// Send a Flow Control frame.
    fn send_fc(&mut self, flag: FcFlag) -> Result<(), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let mut buf = [0u8; 8];
        self.write_ta(&mut buf);
        let len = frame::build_fc(
            &mut buf,
            flag,
            self.config.block_size,
            self.config.st_min,
            overhead,
        );
        self.transmit_padded(&mut buf, len)
    }

    /// Pad (if configured) and transmit a CAN frame built in an 8-byte buffer.
    fn transmit_padded(&mut self, buf: &mut [u8; 8], len: usize) -> Result<(), IsoTpError<E>> {
        let send_len = self.config.padding.map_or(len, |pad| {
            buf[len..].fill(pad);
            8
        });
        self.transmit_frame(&buf[..send_len])
    }

    /// Transmit a CAN frame with the configured tx_id.
    fn transmit_frame(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let frame = CanFrame::new(self.config.tx_id, data).ok_or(IsoTpError::InvalidFrame)?;
        self.channel.transmit(&frame).map_err(IsoTpError::CanError)
    }

    /// Write the TX target address byte for Extended addressing.
    fn write_ta(&self, buf: &mut [u8]) {
        if let AddressingMode::Extended {
            tx_target_address, ..
        } = self.config.addressing
        {
            buf[0] = tx_target_address;
        }
    }

    /// Check that the RX target address matches for Extended addressing.
    /// For Normal addressing, always returns true.
    fn check_ta(&self, data: &[u8]) -> bool {
        match self.config.addressing {
            AddressingMode::Normal => true,
            AddressingMode::Extended {
                rx_target_address, ..
            } => !data.is_empty() && data[0] == rx_target_address,
        }
    }
}
