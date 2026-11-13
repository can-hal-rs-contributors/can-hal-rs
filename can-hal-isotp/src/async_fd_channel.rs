use can_hal::async_channel::{AsyncReceiveFd, AsyncTransmitFd};
use can_hal::error::CanError;
use can_hal::frame::CanFdFrame;

use crate::config::{AddressingMode, IsoTpConfig};
use crate::error::IsoTpError;
use crate::frame::{self, has_fc_pci, interpret_st_min, next_fd_dlc, FcFlag, IsoTpFrame};

/// Async ISO-TP transport channel for CAN FD.
pub struct AsyncIsoTpFdChannel<C> {
    channel: C,
    config: IsoTpConfig,
}

impl<C> AsyncIsoTpFdChannel<C> {
    /// Create a new async ISO-TP FD channel.
    pub const fn new(channel: C, config: IsoTpConfig) -> Self {
        Self { channel, config }
    }

    /// Consume this channel and return the inner CAN FD channel.
    pub fn into_inner(self) -> C {
        self.channel
    }
}

impl<C, E> AsyncIsoTpFdChannel<C>
where
    C: AsyncTransmitFd<Error = E> + AsyncReceiveFd<Error = E>,
    E: CanError,
{
    /// Send an ISO-TP message asynchronously over CAN FD.
    pub async fn send(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let max_sf_fd = 62 - overhead;

        if data.len() as u64 > u64::from(u32::MAX) {
            return Err(IsoTpError::PayloadTooLarge);
        }

        if data.len() <= max_sf_fd {
            let mut buf = [0u8; 64];
            self.write_ta(&mut buf);
            let len = frame::build_sf_fd(&mut buf, data, overhead);
            self.transmit_padded_fd(&mut buf, len).await?;
            return Ok(());
        }

        // First Frame
        let mut buf = [0u8; 64];
        self.write_ta(&mut buf);
        let ff_header_size = if data.len() <= 0xFFF {
            overhead + 2
        } else {
            overhead + 6
        };
        let ff_data_len = 64 - ff_header_size;
        frame::build_ff_fd(&mut buf, data, data.len(), overhead);
        self.transmit_padded_fd(&mut buf, 64).await?;

        let mut offset = ff_data_len;
        let mut sn: u8 = 1;

        let (mut fc_bs, mut st_min_dur) = self.wait_for_fc().await?;
        let mut block_count: u16 = 0;

        let cf_data_capacity = 63 - overhead;
        while offset < data.len() {
            let end = (offset + cf_data_capacity).min(data.len());
            let chunk = &data[offset..end];

            let mut cf_buf = [0u8; 64];
            self.write_ta(&mut cf_buf);
            let cf_len = frame::build_cf_fd(&mut cf_buf, chunk, sn, overhead);
            self.transmit_padded_fd(&mut cf_buf, cf_len).await?;

            offset = end;
            sn = (sn + 1) & 0x0F;
            // Counted only under a block size: with BS 0 nothing resets
            // the count, so a long message would overflow it.
            if fc_bs > 0 {
                block_count += 1;
            }

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

    /// Receive an ISO-TP message asynchronously over CAN FD.
    pub async fn receive(&mut self) -> Result<Vec<u8>, IsoTpError<E>> {
        let overhead = self.config.overhead();

        let (total_len, mut collected) = tokio::time::timeout(self.config.timeout, async {
            loop {
                let ts_frame = self
                    .channel
                    .receive_fd()
                    .await
                    .map_err(IsoTpError::CanError)?;

                let frame = ts_frame.into_frame();
                if frame.id() != self.config.rx_id || frame.is_remote_frame() {
                    continue;
                }
                if !self.check_ta(frame.data()) {
                    continue;
                }

                // ISO 15765-2 has a receiver ignore a frame that isn't a valid
                // N_PDU (an unknown PCI type, a bad length or flow status), as
                // the Linux kernel does.
                let Ok(isotp) = IsoTpFrame::parse(frame.data(), overhead) else {
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

        let Some(mut total_len) = total_len else {
            return Ok(collected);
        };

        self.send_fc(FcFlag::ContinueToSend).await?;

        let mut expected_sn: u8 = 1;
        let mut block_count: u16 = 0;

        // N_Cr: restarted by each CF and by a new FF, not by ignored frames.
        let mut cf_deadline = self.config.tokio_deadline();
        while collected.len() < total_len {
            let cf_frame = tokio::time::timeout_at(cf_deadline, async {
                loop {
                    let ts = self
                        .channel
                        .receive_fd()
                        .await
                        .map_err(IsoTpError::CanError)?;

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
                    // Counted only under a block size, as in send().
                    if self.config.block_size > 0 {
                        block_count += 1;
                    }

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

    async fn wait_for_fc(&mut self) -> Result<(u8, std::time::Duration), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let max_fc_wait = self.config.max_fc_wait;

        tokio::time::timeout(self.config.timeout, async {
            let mut wait_count: u8 = 0;
            loop {
                let ts_frame = self
                    .channel
                    .receive_fd()
                    .await
                    .map_err(IsoTpError::CanError)?;

                let frame = ts_frame.into_frame();
                if frame.id() != self.config.rx_id || frame.is_remote_frame() {
                    continue;
                }
                if !self.check_ta(frame.data()) {
                    continue;
                }

                let isotp = match IsoTpFrame::parse(frame.data(), overhead) {
                    Ok(isotp) => isotp,
                    // A malformed FC ends the transfer, as in the Linux
                    // kernel; any other frame that doesn't parse isn't for a
                    // sender.
                    Err(_) if has_fc_pci(frame.data(), overhead) => {
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

    async fn send_fc(&mut self, flag: FcFlag) -> Result<(), IsoTpError<E>> {
        let overhead = self.config.overhead();
        let mut buf = [0u8; 64];
        self.write_ta(&mut buf);
        let len = frame::build_fc_fd(
            &mut buf,
            flag,
            self.config.block_size,
            self.config.st_min,
            overhead,
        );
        self.transmit_padded_fd(&mut buf, len).await
    }

    /// CAN FD frames must have a valid data length, so always round up. Fill
    /// the gap with the configured padding byte or 0x00.
    async fn transmit_padded_fd(
        &mut self,
        buf: &mut [u8; 64],
        len: usize,
    ) -> Result<(), IsoTpError<E>> {
        let padded = next_fd_dlc(len);
        let pad_byte = self.config.padding.unwrap_or(0x00);
        buf[len..padded].fill(pad_byte);
        self.transmit_fd_frame(&buf[..padded]).await
    }

    async fn transmit_fd_frame(&mut self, data: &[u8]) -> Result<(), IsoTpError<E>> {
        let frame = CanFdFrame::new(self.config.tx_id, data, true, false)
            .ok_or(IsoTpError::InvalidFrame)?;
        self.channel
            .transmit_fd(&frame)
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
