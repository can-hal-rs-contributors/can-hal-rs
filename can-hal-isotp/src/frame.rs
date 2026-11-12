use std::time::Duration;

use crate::error::IsoTpError;

/// Most bytes `receive()` reserves up front for a message: the largest FF_DL
/// of the 12-bit form. FF_DL comes from the peer and can be up to `u32::MAX`
/// with the 32-bit form, so a longer message grows as its CFs arrive instead.
pub(crate) const MAX_RX_PREALLOC: usize = 0xFFF;

/// Flow Control flag values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FcFlag {
    ContinueToSend,
    Wait,
    Overflow,
}

/// Parsed ISO-TP frame types.
#[derive(Debug)]
pub enum IsoTpFrame<'a> {
    SingleFrame {
        data: &'a [u8],
    },
    FirstFrame {
        total_len: usize,
        data: &'a [u8],
    },
    ConsecutiveFrame {
        sn: u8,
        data: &'a [u8],
    },
    FlowControl {
        flag: FcFlag,
        block_size: u8,
        st_min: u8,
    },
}

impl<'a> IsoTpFrame<'a> {
    /// Parse raw CAN frame data into an ISO-TP frame.
    ///
    /// `overhead` is the number of addressing bytes before the PCI (0 for Normal, 1 for Extended).
    pub fn parse(raw: &'a [u8], overhead: usize) -> Result<Self, IsoTpError<()>> {
        if raw.len() <= overhead {
            return Err(IsoTpError::InvalidFrame);
        }

        let pci = &raw[overhead..];
        let frame_type = pci[0] >> 4;

        match frame_type {
            // Single Frame
            0x0 => {
                let sf_dl = (pci[0] & 0x0F) as usize;
                if sf_dl == 0 {
                    // FD Single Frame: 2-byte PCI [0x00, DL]
                    if pci.len() < 2 || pci[1] == 0 {
                        return Err(IsoTpError::InvalidFrame);
                    }
                    let fd_dl = pci[1] as usize;
                    if overhead + 2 + fd_dl > raw.len() {
                        return Err(IsoTpError::InvalidFrame);
                    }
                    let data = &raw[overhead + 2..overhead + 2 + fd_dl];
                    Ok(IsoTpFrame::SingleFrame { data })
                } else {
                    // Classic SF: 1-byte PCI
                    if sf_dl > 7 || overhead + 1 + sf_dl > raw.len() {
                        return Err(IsoTpError::InvalidFrame);
                    }
                    let data = &raw[overhead + 1..overhead + 1 + sf_dl];
                    Ok(IsoTpFrame::SingleFrame { data })
                }
            }
            // First Frame
            0x1 => {
                if pci.len() < 2 {
                    return Err(IsoTpError::InvalidFrame);
                }
                let len12 = (((pci[0] & 0x0F) as usize) << 8) | (pci[1] as usize);
                if len12 == 0 {
                    // Long FF: 32-bit length in bytes 2..6
                    if pci.len() < 6 {
                        return Err(IsoTpError::InvalidFrame);
                    }
                    let total_len = ((pci[2] as usize) << 24)
                        | ((pci[3] as usize) << 16)
                        | ((pci[4] as usize) << 8)
                        | (pci[5] as usize);
                    let data_start = overhead + 6;
                    let data = if data_start < raw.len() {
                        &raw[data_start..]
                    } else {
                        &[]
                    };
                    Ok(IsoTpFrame::FirstFrame { total_len, data })
                } else {
                    let data_start = overhead + 2;
                    let data = if data_start < raw.len() {
                        &raw[data_start..]
                    } else {
                        &[]
                    };
                    Ok(IsoTpFrame::FirstFrame {
                        total_len: len12,
                        data,
                    })
                }
            }
            // Consecutive Frame
            0x2 => {
                let sn = pci[0] & 0x0F;
                let data_start = overhead + 1;
                let data = if data_start < raw.len() {
                    &raw[data_start..]
                } else {
                    &[]
                };
                Ok(IsoTpFrame::ConsecutiveFrame { sn, data })
            }
            // Flow Control
            0x3 => {
                if pci.len() < 3 {
                    return Err(IsoTpError::InvalidFrame);
                }
                let flag_nibble = pci[0] & 0x0F;
                let flag = match flag_nibble {
                    0 => FcFlag::ContinueToSend,
                    1 => FcFlag::Wait,
                    2 => FcFlag::Overflow,
                    _ => return Err(IsoTpError::InvalidFrame),
                };
                Ok(IsoTpFrame::FlowControl {
                    flag,
                    block_size: pci[1],
                    st_min: pci[2],
                })
            }
            _ => Err(IsoTpError::InvalidFrame),
        }
    }
}

/// Whether `raw` carries the Flow Control PCI type (0x3), valid FC or not.
///
/// `overhead` is as for [`IsoTpFrame::parse`].
pub(crate) fn has_fc_pci(raw: &[u8], overhead: usize) -> bool {
    raw.get(overhead).is_some_and(|pci| pci >> 4 == 0x3)
}

/// Build a Single Frame into `buf`. Returns the total frame length.
#[allow(clippy::cast_possible_truncation)] // protocol-bounded: SF data <= 7 bytes
pub fn build_sf(buf: &mut [u8], data: &[u8], overhead: usize) -> usize {
    let mut pos = 0;
    if overhead > 0 {
        // Extended addressing: caller should have set buf[0] = target_address already
        pos = overhead;
    }
    buf[pos] = data.len() as u8; // PCI: 0x0 | SF_DL
    pos += 1;
    buf[pos..pos + data.len()].copy_from_slice(data);
    pos + data.len()
}

/// Build a First Frame into `buf`. Returns the total frame length.
///
/// Copies as many bytes from `data` as fit into a single 8-byte CAN frame.
#[allow(clippy::cast_possible_truncation)] // protocol-bounded: byte-level fields masked to 8 bits
pub fn build_ff(buf: &mut [u8], data: &[u8], total_len: usize, overhead: usize) -> usize {
    let mut pos = overhead;
    if total_len <= 0xFFF {
        // Standard FF: 12-bit length
        buf[pos] = 0x10 | ((total_len >> 8) & 0x0F) as u8;
        buf[pos + 1] = (total_len & 0xFF) as u8;
        pos += 2;
    } else {
        // Long FF: 32-bit length
        buf[pos] = 0x10;
        buf[pos + 1] = 0x00;
        buf[pos + 2] = ((total_len >> 24) & 0xFF) as u8;
        buf[pos + 3] = ((total_len >> 16) & 0xFF) as u8;
        buf[pos + 4] = ((total_len >> 8) & 0xFF) as u8;
        buf[pos + 5] = (total_len & 0xFF) as u8;
        pos += 6;
    }
    let remaining = 8usize.saturating_sub(pos);
    let copy_len = data.len().min(remaining);
    buf[pos..pos + copy_len].copy_from_slice(&data[..copy_len]);
    8 // FF always fills a full 8-byte CAN frame
}

/// Build a Consecutive Frame into `buf`. Returns the total frame length.
pub fn build_cf(buf: &mut [u8], data: &[u8], sn: u8, overhead: usize) -> usize {
    let mut pos = overhead;
    buf[pos] = 0x20 | (sn & 0x0F);
    pos += 1;
    let copy_len = data.len().min(buf.len() - pos);
    buf[pos..pos + copy_len].copy_from_slice(&data[..copy_len]);
    pos + copy_len
}

/// Build a Flow Control frame into `buf`. Returns the total frame length.
pub fn build_fc(
    buf: &mut [u8],
    flag: FcFlag,
    block_size: u8,
    st_min: u8,
    overhead: usize,
) -> usize {
    let pos = overhead;
    let flag_byte = match flag {
        FcFlag::ContinueToSend => 0x30,
        FcFlag::Wait => 0x31,
        FcFlag::Overflow => 0x32,
    };
    buf[pos] = flag_byte;
    buf[pos + 1] = block_size;
    buf[pos + 2] = st_min;
    pos + 3
}

/// Returns the next valid CAN FD DLC value >= `len`. Panics if `len > 64`.
pub(crate) fn next_fd_dlc(len: usize) -> usize {
    match len {
        0..=8 => len,
        9..=12 => 12,
        13..=16 => 16,
        17..=20 => 20,
        21..=24 => 24,
        25..=32 => 32,
        33..=48 => 48,
        49..=64 => 64,
        _ => panic!("next_fd_dlc: len {len} exceeds 64"),
    }
}

/// Build a CAN FD Single Frame into `buf`. Returns the total frame length.
///
/// For `data.len() <= 7 - overhead`: classic 1-byte PCI (0x0N).
/// For `data.len() <= 62 - overhead`: FD 2-byte PCI (0x00, N).
#[allow(clippy::cast_possible_truncation)] // protocol-bounded: SF FD data <= 62 bytes
pub fn build_sf_fd(buf: &mut [u8; 64], data: &[u8], overhead: usize) -> usize {
    let max_classic = 7 - overhead;
    let mut pos = overhead;

    if data.len() <= max_classic {
        // Classic 1-byte PCI even in an FD frame
        buf[pos] = data.len() as u8;
        pos += 1;
    } else {
        // FD 2-byte PCI: [0x00, DL]
        buf[pos] = 0x00;
        buf[pos + 1] = data.len() as u8;
        pos += 2;
    }
    buf[pos..pos + data.len()].copy_from_slice(data);
    pos + data.len()
}

/// Build a CAN FD First Frame into `buf`. Always fills 64 bytes.
#[allow(clippy::cast_possible_truncation)] // protocol-bounded: byte-level fields masked to 8 bits
pub fn build_ff_fd(buf: &mut [u8; 64], data: &[u8], total_len: usize, overhead: usize) -> usize {
    let mut pos = overhead;
    if total_len <= 0xFFF {
        buf[pos] = 0x10 | ((total_len >> 8) & 0x0F) as u8;
        buf[pos + 1] = (total_len & 0xFF) as u8;
        pos += 2;
    } else {
        buf[pos] = 0x10;
        buf[pos + 1] = 0x00;
        buf[pos + 2] = ((total_len >> 24) & 0xFF) as u8;
        buf[pos + 3] = ((total_len >> 16) & 0xFF) as u8;
        buf[pos + 4] = ((total_len >> 8) & 0xFF) as u8;
        buf[pos + 5] = (total_len & 0xFF) as u8;
        pos += 6;
    }
    let remaining = 64usize.saturating_sub(pos);
    let copy_len = data.len().min(remaining);
    buf[pos..pos + copy_len].copy_from_slice(&data[..copy_len]);
    64 // FF always fills a full 64-byte FD frame
}

/// Build a CAN FD Consecutive Frame into `buf`. Returns the total frame length.
pub fn build_cf_fd(buf: &mut [u8; 64], data: &[u8], sn: u8, overhead: usize) -> usize {
    let mut pos = overhead;
    buf[pos] = 0x20 | (sn & 0x0F);
    pos += 1;
    let copy_len = data.len().min(64 - pos);
    buf[pos..pos + copy_len].copy_from_slice(&data[..copy_len]);
    pos + copy_len
}

/// Build a CAN FD Flow Control frame into `buf`. Returns the total frame length.
pub fn build_fc_fd(
    buf: &mut [u8; 64],
    flag: FcFlag,
    block_size: u8,
    st_min: u8,
    overhead: usize,
) -> usize {
    let pos = overhead;
    let flag_byte = match flag {
        FcFlag::ContinueToSend => 0x30,
        FcFlag::Wait => 0x31,
        FcFlag::Overflow => 0x32,
    };
    buf[pos] = flag_byte;
    buf[pos + 1] = block_size;
    buf[pos + 2] = st_min;
    pos + 3
}

/// Interpret a raw STmin byte (from an FC frame) as a `Duration`.
///
/// - 0x00–0x7F: value in milliseconds (0–127 ms)
/// - 0xF1–0xF9: 100–900 microseconds, rounded up to 1 ms (OS sleep granularity)
/// - All other values are reserved: treated as 0x7F (127 ms), the longest
///   valid STmin, as ISO 15765-2 has a sender do when it receives one
pub(crate) fn interpret_st_min(st_min: u8) -> Duration {
    match st_min {
        0x00..=0x7F => Duration::from_millis(u64::from(st_min)),
        0xF1..=0xF9 => {
            let us = u64::from(st_min - 0xF0) * 100;
            Duration::from_micros(us).max(Duration::from_millis(1))
        }
        _ => Duration::from_millis(127),
    }
}
