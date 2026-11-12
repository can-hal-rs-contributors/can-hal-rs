use can_hal::async_channel::{AsyncReceive, AsyncTransmit};
use can_hal::error::CanError;
use can_hal::frame::CanFrame;

use crate::config::{AddressingMode, IsoTpConfig};
use crate::error::IsoTpError;
use crate::frame::{self, interpret_st_min, FcFlag, IsoTpFrame};

/// Async ISO-TP transport channel wrapping an async `can-hal` CAN channel.
pub struct AsyncIsoTpChannel<C> {
    channel: C,
    config: IsoTpConfig,
}

impl<C> AsyncIsoTpChannel<C> {
    /// Create a new async ISO-TP channel around a CAN channel with the given config.
    pub const fn new(channel: C, config: IsoTpConfig) -> Self {
        Self { channel, config }
    }

    /// Consume this ISO-TP channel and return the inner CAN channel.
    pub fn into_inner(self) -> C {
        self.channel
    }
}

impl<C, E> AsyncIsoTpChannel<C>
where
    C: AsyncTransmit<Error = E> + AsyncReceive<Error = E>,
    E: CanError,
{
    /// Send an ISO-TP message asynchronously.
    pub async fn send(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let max_sf = 7 - overhead;

        if data.len() as u64 > u64::from(u32::MAX) {
            return Err(IsoTpError::PayloadTooLarge);
        }

        if data.len() <= max_sf {
            let mut buf = [0u8; 8];
            self.write_ta(&mut buf);
            let len = frame::build_sf(&mut buf, data, overhead);
            self.transmit_padded(&mut buf, len).await?;
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
        self.transmit_padded(&mut buf, 8).await?;

        let mut offset = ff_data_len;
        let mut sn: u8 = 1;

        let (mut fc_bs, mut st_min_dur) = self.wait_for_fc().await?;
        let mut block_count: u16 = 0;

        while offset < data.len() {
            let cf_data_capacity = 7 - overhead;
            let end = (offset + cf_data_capacity).min(data.len());
            let chunk = &data[offset..end];

            let mut cf_buf = [0u8; 8];
            self.write_ta(&mut cf_buf);
            let cf_len = frame::build_cf(&mut cf_buf, chunk, sn, overhead);
            self.transmit_padded(&mut cf_buf, cf_len).await?;

            offset = end;
            sn = (sn + 1) & 0x0F;
            block_count += 1;

            if offset < data.len() {
                if !st_min_dur.is_zero() {
                    tokio::time::sleep(st_min_dur).await;
                }

                if fc_bs > 0 && block_count >= u16::from(fc_bs) {
                    let (new_bs, new_st) = self.wait_for_fc().await?;
                    fc_bs = new_bs;
                    st_min_dur = new_st;
                    block_count = 0;
                }
            }
        }

        Ok(())
    }

    /// Send a functionally addressed single-frame request asynchronously.
    ///
    /// Functional addressing is broadcast-only and restricted to single frames
    /// (≤ 7 bytes for normal addressing, ≤ 6 for extended). Returns
    /// `IsoTpError::PayloadTooLarge` if data exceeds the SF limit, or
    /// `IsoTpError::InvalidFrame` if `config.functional_id` is not set.
    pub async fn send_functional(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
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
        self.channel
            .transmit(&frame)
            .await
            .map_err(IsoTpError::CanError)
    }

    /// Receive an ISO-TP message asynchronously.
    pub async fn receive(&mut self) -> Result<Vec<u8>, IsoTpError<E>> {
        let overhead = self.config.overhead();

        // Wait for first frame (SF or FF) from rx_id.
        // Returns None for a complete single frame, or Some(total_len) for a first frame.
        let (total_len, mut collected) = tokio::time::timeout(self.config.timeout, async {
            loop {
                let ts_frame = self.channel.receive().await.map_err(IsoTpError::CanError)?;

                let can_frame = ts_frame.into_frame();
                if can_frame.id() != self.config.rx_id || can_frame.is_remote_frame() {
                    continue;
                }
                if !self.check_ta(can_frame.data()) {
                    continue;
                }

                // ISO 15765-2 has a receiver ignore a frame that isn't a valid
                // N_PDU (an unknown PCI type, a bad length or flow status), as
                // the Linux kernel does.
                let Ok(isotp) = IsoTpFrame::parse(can_frame.data(), overhead) else {
                    continue;
                };

                match isotp {
                    IsoTpFrame::SingleFrame { data } => {
                        return Ok::<_, IsoTpError<E>>((None, data.to_vec()));
                    }
                    IsoTpFrame::FirstFrame { total_len, data } => {
                        return Ok((Some(total_len), data.to_vec()));
                    }
                    // A CF with no reception in progress, or an FC: ignore it.
                    _ => {}
                }
            }
        })
        .await
        .map_err(|_| IsoTpError::Timeout)??;

        // Single frame: reassembly is complete.
        let Some(mut total_len) = total_len else {
            return Ok(collected);
        };

        // Multi-frame: send FC and collect CFs
        self.send_fc(FcFlag::ContinueToSend).await?;

        let mut expected_sn: u8 = 1;
        let mut block_count: u16 = 0;

        // N_Cr: restarted by each CF and by a new FF, not by ignored frames.
        let mut cf_deadline = self.config.tokio_deadline();
        while collected.len() < total_len {
            let cf_frame = tokio::time::timeout_at(cf_deadline, async {
                loop {
                    let ts = self.channel.receive().await.map_err(IsoTpError::CanError)?;

                    let cf = ts.into_frame();
                    if cf.id() != self.config.rx_id || cf.is_remote_frame() {
                        continue;
                    }
                    if !self.check_ta(cf.data()) {
                        continue;
                    }

                    return Ok::<_, IsoTpError<E>>(cf);
                }
            })
            .await
            .map_err(|_| IsoTpError::Timeout)??;

            // Ignored, as above; the CF deadline stands.
            let Ok(parsed) = IsoTpFrame::parse(cf_frame.data(), overhead) else {
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

                    let bytes_needed = total_len - collected.len();
                    let take = data.len().min(bytes_needed);
                    collected.extend_from_slice(&data[..take]);

                    expected_sn = (expected_sn + 1) & 0x0F;
                    block_count += 1;

                    cf_deadline = self.config.tokio_deadline();

                    if self.config.block_size > 0
                        && block_count >= u16::from(self.config.block_size)
                        && collected.len() < total_len
                    {
                        self.send_fc(FcFlag::ContinueToSend).await?;
                        block_count = 0;
                    }
                }
                // ISO 15765-2 (unexpected N_PDU): an SF or FF that arrives
                // during a reception ends it. The partial message is dropped,
                // and the new frame is handled as the start of a new reception.
                IsoTpFrame::SingleFrame { data } => return Ok(data.to_vec()),
                IsoTpFrame::FirstFrame {
                    total_len: new_len,
                    data,
                } => {
                    total_len = new_len;
                    collected.clear();
                    collected.extend_from_slice(data);
                    self.send_fc(FcFlag::ContinueToSend).await?;
                    expected_sn = 1;
                    block_count = 0;
                    cf_deadline = self.config.tokio_deadline();
                }
                // An FC is meant for a sender; a receiver ignores it.
                IsoTpFrame::FlowControl { .. } => {}
            }
        }

        Ok(collected)
    }

    /// Wait for a Flow Control frame asynchronously.
    async fn wait_for_fc(&mut self) -> Result<(u8, std::time::Duration), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let max_fc_wait = self.config.max_fc_wait;

        tokio::time::timeout(self.config.timeout, async {
            let mut wait_count: u8 = 0;
            loop {
                let ts_frame = self.channel.receive().await.map_err(IsoTpError::CanError)?;

                let can_frame = ts_frame.into_frame();
                if can_frame.id() != self.config.rx_id || can_frame.is_remote_frame() {
                    continue;
                }
                if !self.check_ta(can_frame.data()) {
                    continue;
                }

                let isotp = IsoTpFrame::parse(can_frame.data(), overhead)
                    .map_err(|_| IsoTpError::InvalidFrame)?;

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
                            if max_fc_wait > 0 {
                                wait_count += 1;
                                if wait_count >= max_fc_wait {
                                    return Err(IsoTpError::WaitLimitExceeded);
                                }
                            }
                        }
                        FcFlag::Overflow => return Err(IsoTpError::BufferOverflow),
                    }
                }
            }
        })
        .await
        .map_err(|_| IsoTpError::Timeout)?
    }

    /// Send a Flow Control frame asynchronously.
    async fn send_fc(&mut self, flag: FcFlag) -> Result<(), IsoTpError<E>> {
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
        self.transmit_padded(&mut buf, len).await
    }

    /// Pad (if configured) and transmit a CAN frame built in an 8-byte buffer.
    async fn transmit_padded(
        &mut self,
        buf: &mut [u8; 8],
        len: usize,
    ) -> Result<(), IsoTpError<E>> {
        let send_len = self.config.padding.map_or(len, |pad| {
            buf[len..].fill(pad);
            8
        });
        self.transmit_frame(&buf[..send_len]).await
    }

    async fn transmit_frame(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let frame = CanFrame::new(self.config.tx_id, data).ok_or(IsoTpError::InvalidFrame)?;
        self.channel
            .transmit(&frame)
            .await
            .map_err(IsoTpError::CanError)
    }

    fn write_ta(&self, buf: &mut [u8]) {
        if let AddressingMode::Extended {
            tx_target_address, ..
        } = self.config.addressing
        {
            buf[0] = tx_target_address;
        }
    }

    fn check_ta(&self, data: &[u8]) -> bool {
        match self.config.addressing {
            AddressingMode::Normal => true,
            AddressingMode::Extended {
                rx_target_address, ..
            } => !data.is_empty() && data[0] == rx_target_address,
        }
    }
}
