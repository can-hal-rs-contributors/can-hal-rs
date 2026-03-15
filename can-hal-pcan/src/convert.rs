//! Bidirectional conversions between can-hal types and PCAN-Basic FFI types.

use can_hal::frame::{CanFdFrame, CanFrame, Frame};
use can_hal::id::CanId;

use crate::error::PcanError;
use crate::ffi::{
    self, TPCANMsg, TPCANMsgFD, PCAN_MESSAGE_BRS, PCAN_MESSAGE_ESI, PCAN_MESSAGE_EXTENDED,
    PCAN_MESSAGE_FD, PCAN_MESSAGE_STANDARD,
};

// ---------------------------------------------------------------------------
// ID conversions
// ---------------------------------------------------------------------------

/// Encode a `CanId` into (raw_id, msg_type_flags).
pub(crate) fn to_pcan_id(id: CanId) -> (u32, u8) {
    match id {
        CanId::Standard(v) => (v as u32, PCAN_MESSAGE_STANDARD),
        CanId::Extended(v) => (v, PCAN_MESSAGE_EXTENDED),
    }
}

/// Decode a PCAN (raw_id, msg_type) into a `CanId`.
pub(crate) fn from_pcan_id(raw_id: u32, msg_type: u8) -> Result<CanId, PcanError> {
    if msg_type & PCAN_MESSAGE_EXTENDED != 0 {
        CanId::new_extended(raw_id)
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid extended ID: 0x{raw_id:08X}")))
    } else {
        if raw_id > 0x7FF {
            return Err(PcanError::InvalidFrame(format!(
                "standard ID out of range: 0x{raw_id:08X}"
            )));
        }
        CanId::new_standard(raw_id as u16)
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid standard ID: 0x{raw_id:04X}")))
    }
}

// ---------------------------------------------------------------------------
// Classic CAN frame conversions
// ---------------------------------------------------------------------------

/// Convert a `CanFrame` to a `TPCANMsg`.
pub(crate) fn to_pcan_msg(frame: &CanFrame) -> TPCANMsg {
    let (id, msg_type) = to_pcan_id(frame.id());
    let mut data = [0u8; 8];
    data[..frame.len()].copy_from_slice(frame.data());
    TPCANMsg {
        id,
        msg_type,
        len: frame.len() as u8,
        data,
    }
}

/// Convert a `TPCANMsg` to a `CanFrame`.
/// Returns `Ok(None)` for RTR or status messages.
pub(crate) fn from_pcan_msg(msg: &TPCANMsg) -> Result<Option<CanFrame>, PcanError> {
    if msg.msg_type & ffi::PCAN_MESSAGE_RTR != 0 || msg.msg_type & ffi::PCAN_MESSAGE_STATUS != 0 {
        return Ok(None);
    }
    let id = from_pcan_id(msg.id, msg.msg_type)?;
    let len = (msg.len as usize).min(msg.data.len());
    let frame = CanFrame::new(id, &msg.data[..len])
        .ok_or_else(|| PcanError::InvalidFrame(format!("invalid DLC: {}", msg.len)))?;
    Ok(Some(frame))
}

// ---------------------------------------------------------------------------
// CAN FD frame conversions
// ---------------------------------------------------------------------------

/// Convert a `CanFdFrame` to a `TPCANMsgFD`.
pub(crate) fn to_pcan_msg_fd(frame: &CanFdFrame) -> TPCANMsgFD {
    let (id, mut msg_type) = to_pcan_id(frame.id());
    msg_type |= PCAN_MESSAGE_FD;
    if frame.brs() {
        msg_type |= PCAN_MESSAGE_BRS;
    }
    if frame.esi() {
        msg_type |= PCAN_MESSAGE_ESI;
    }
    let mut data = [0u8; 64];
    data[..frame.len()].copy_from_slice(frame.data());
    TPCANMsgFD {
        id,
        msg_type,
        dlc: dlc_bytes_to_code(frame.len() as u8),
        data,
    }
}

/// Convert a `TPCANMsgFD` to a `Frame`.
/// Returns `Ok(None)` for status messages.
pub(crate) fn from_pcan_msg_fd(msg: &TPCANMsgFD) -> Result<Option<Frame>, PcanError> {
    if msg.msg_type & ffi::PCAN_MESSAGE_STATUS != 0 {
        return Ok(None);
    }

    let id = from_pcan_id(msg.id, msg.msg_type)?;

    if msg.msg_type & PCAN_MESSAGE_FD != 0 {
        let len = dlc_code_to_bytes(msg.dlc) as usize;
        let brs = msg.msg_type & PCAN_MESSAGE_BRS != 0;
        let esi = msg.msg_type & PCAN_MESSAGE_ESI != 0;
        let frame = CanFdFrame::new(id, &msg.data[..len], brs, esi)
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid FD DLC: {}", msg.dlc)))?;
        Ok(Some(Frame::Fd(frame)))
    } else {
        // Classic frame received on an FD channel.
        let len = std::cmp::min(msg.dlc, 8) as usize;
        let frame = CanFrame::new(id, &msg.data[..len])
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid classic DLC: {}", msg.dlc)))?;
        Ok(Some(Frame::Can(frame)))
    }
}

// ---------------------------------------------------------------------------
// DLC encoding / decoding
// ---------------------------------------------------------------------------

/// Convert a byte count to a CAN FD DLC code (0–15).
///
/// Non-standard byte counts are rounded up to the next valid DLC.
fn dlc_bytes_to_code(bytes: u8) -> u8 {
    match bytes {
        0..=8 => bytes,
        9..=12 => 9,
        13..=16 => 10,
        17..=20 => 11,
        21..=24 => 12,
        25..=32 => 13,
        33..=48 => 14,
        49..=64 => 15,
        _ => 15,
    }
}

/// Convert a CAN FD DLC code (0–15) to a byte count.
///
/// Out-of-range codes (> 15) are clamped to 64 to prevent panics when
/// slicing into a 64-byte buffer with data from an FFI source.
fn dlc_code_to_bytes(dlc: u8) -> u8 {
    match dlc {
        0..=8 => dlc,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        15 => 64,
        _ => 64,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_standard_id() {
        let id = CanId::new_standard(0x123).unwrap();
        let (raw, msg_type) = to_pcan_id(id);
        let back = from_pcan_id(raw, msg_type).unwrap();
        assert_eq!(id, back);
    }

    #[test]
    fn roundtrip_extended_id() {
        let id = CanId::new_extended(0x1234_5678).unwrap();
        let (raw, msg_type) = to_pcan_id(id);
        let back = from_pcan_id(raw, msg_type).unwrap();
        assert_eq!(id, back);
    }

    #[test]
    fn roundtrip_standard_id_zero() {
        let id = CanId::new_standard(0x000).unwrap();
        let (raw, msg_type) = to_pcan_id(id);
        let back = from_pcan_id(raw, msg_type).unwrap();
        assert_eq!(id, back);
    }

    #[test]
    fn roundtrip_standard_id_max() {
        let id = CanId::new_standard(0x7FF).unwrap();
        let (raw, msg_type) = to_pcan_id(id);
        let back = from_pcan_id(raw, msg_type).unwrap();
        assert_eq!(id, back);
    }

    #[test]
    fn roundtrip_extended_id_max() {
        let id = CanId::new_extended(0x1FFF_FFFF).unwrap();
        let (raw, msg_type) = to_pcan_id(id);
        let back = from_pcan_id(raw, msg_type).unwrap();
        assert_eq!(id, back);
    }

    #[test]
    fn roundtrip_can_frame() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new(id, &[1, 2, 3]).unwrap();
        let msg = to_pcan_msg(&frame);
        let back = from_pcan_msg(&msg).unwrap().unwrap();
        assert_eq!(frame.id(), back.id());
        assert_eq!(frame.data(), back.data());
    }

    #[test]
    fn roundtrip_can_frame_empty() {
        let id = CanId::new_standard(0x7FF).unwrap();
        let frame = CanFrame::new(id, &[]).unwrap();
        let msg = to_pcan_msg(&frame);
        let back = from_pcan_msg(&msg).unwrap().unwrap();
        assert_eq!(frame.id(), back.id());
        assert_eq!(frame.len(), back.len());
        assert_eq!(0, back.len());
    }

    #[test]
    fn roundtrip_can_frame_full() {
        let id = CanId::new_extended(0x1000).unwrap();
        let frame = CanFrame::new(id, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        let msg = to_pcan_msg(&frame);
        assert_eq!(msg.msg_type & PCAN_MESSAGE_EXTENDED, PCAN_MESSAGE_EXTENDED);
        let back = from_pcan_msg(&msg).unwrap().unwrap();
        assert_eq!(frame.data(), back.data());
    }

    #[test]
    fn roundtrip_fd_frame() {
        let id = CanId::new_extended(0x200).unwrap();
        let data = [0xAA; 24];
        let frame = CanFdFrame::new(id, &data, true, false).unwrap();
        let msg = to_pcan_msg_fd(&frame);
        assert!(msg.msg_type & PCAN_MESSAGE_FD != 0);
        assert!(msg.msg_type & PCAN_MESSAGE_BRS != 0);
        assert!(msg.msg_type & PCAN_MESSAGE_ESI == 0);
        let back = from_pcan_msg_fd(&msg).unwrap().unwrap();
        match back {
            Frame::Fd(fd) => {
                assert_eq!(fd.id(), frame.id());
                assert_eq!(fd.data(), frame.data());
                assert!(fd.brs());
                assert!(!fd.esi());
            }
            Frame::Can(_) => panic!("expected FD frame"),
        }
    }

    #[test]
    fn fd_frame_with_esi() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFdFrame::new(id, &[1, 2, 3, 4, 5, 6, 7, 8], false, true).unwrap();
        let msg = to_pcan_msg_fd(&frame);
        assert!(msg.msg_type & PCAN_MESSAGE_ESI != 0);
        assert!(msg.msg_type & PCAN_MESSAGE_BRS == 0);
    }

    #[test]
    fn classic_frame_on_fd_channel() {
        // A classic CAN frame read via CAN_ReadFD appears without the FD flag.
        let msg = TPCANMsgFD {
            id: 0x100,
            msg_type: PCAN_MESSAGE_STANDARD,
            dlc: 3,
            data: {
                let mut d = [0u8; 64];
                d[0] = 0xAA;
                d[1] = 0xBB;
                d[2] = 0xCC;
                d
            },
        };
        let frame = from_pcan_msg_fd(&msg).unwrap().unwrap();
        match frame {
            Frame::Can(cf) => {
                assert_eq!(cf.len(), 3);
                assert_eq!(cf.data(), &[0xAA, 0xBB, 0xCC]);
            }
            Frame::Fd(_) => panic!("expected classic frame"),
        }
    }

    #[test]
    fn skip_rtr_message() {
        let msg = TPCANMsg {
            id: 0x100,
            msg_type: ffi::PCAN_MESSAGE_RTR,
            len: 0,
            data: [0; 8],
        };
        assert!(from_pcan_msg(&msg).unwrap().is_none());
    }

    #[test]
    fn skip_status_message() {
        let msg = TPCANMsg {
            id: 0,
            msg_type: ffi::PCAN_MESSAGE_STATUS,
            len: 0,
            data: [0; 8],
        };
        assert!(from_pcan_msg(&msg).unwrap().is_none());
    }

    #[test]
    fn skip_status_message_fd() {
        let msg = TPCANMsgFD {
            id: 0,
            msg_type: ffi::PCAN_MESSAGE_STATUS,
            dlc: 0,
            data: [0; 64],
        };
        assert!(from_pcan_msg_fd(&msg).unwrap().is_none());
    }

    #[test]
    fn dlc_encoding_roundtrip() {
        for &bytes in &[0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64] {
            let code = dlc_bytes_to_code(bytes);
            let back = dlc_code_to_bytes(code);
            assert_eq!(bytes, back, "DLC roundtrip failed for {bytes}");
        }
    }
}
