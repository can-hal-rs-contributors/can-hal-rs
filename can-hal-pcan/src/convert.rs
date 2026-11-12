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
#[allow(clippy::cast_lossless)] // u16 as u32 is lossless but From is not const-stable
pub const fn to_pcan_id(id: CanId) -> (u32, u8) {
    match id {
        CanId::Standard(v) => (v as u32, PCAN_MESSAGE_STANDARD),
        CanId::Extended(v) => (v, PCAN_MESSAGE_EXTENDED),
    }
}

/// Decode a PCAN (raw_id, msg_type) into a `CanId`.
pub fn from_pcan_id(raw_id: u32, msg_type: u8) -> Result<CanId, PcanError> {
    if msg_type & PCAN_MESSAGE_EXTENDED != 0 {
        CanId::new_extended(raw_id)
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid extended ID: 0x{raw_id:08X}")))
    } else {
        if raw_id > 0x7FF {
            return Err(PcanError::InvalidFrame(format!(
                "standard ID out of range: 0x{raw_id:08X}"
            )));
        }
        // raw_id is verified <= 0x7FF above, so u32 -> u16 truncation cannot happen.
        #[allow(clippy::cast_possible_truncation)]
        CanId::new_standard(raw_id as u16)
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid standard ID: 0x{raw_id:04X}")))
    }
}

// ---------------------------------------------------------------------------
// Classic CAN frame conversions
// ---------------------------------------------------------------------------

/// Convert a `CanFrame` to a `TPCANMsg`: a remote frame with the RTR flag,
/// and the length it asks for.
pub fn to_pcan_msg(frame: &CanFrame) -> TPCANMsg {
    let (id, mut msg_type) = to_pcan_id(frame.id());
    if frame.is_remote_frame() {
        msg_type |= ffi::PCAN_MESSAGE_RTR;
    }
    let mut data = [0u8; 8];
    data[..frame.len()].copy_from_slice(frame.data());
    TPCANMsg {
        id,
        msg_type,
        // A classic frame's DLC is at most 15, fits in u8. PCAN-Basic
        // documents LEN as 0-8; a DLC of 9-15 goes as it is, untested.
        #[allow(clippy::cast_possible_truncation)]
        len: frame.dlc() as u8,
        data,
    }
}

/// Whether a received message is a status message or an error frame rather
/// than a frame from the bus.
const fn is_status_or_error(msg_type: u8) -> bool {
    msg_type & (ffi::PCAN_MESSAGE_STATUS | ffi::PCAN_MESSAGE_ERRFRAME) != 0
}

/// Convert a `TPCANMsg` to a `CanFrame`, a remote frame for an RTR message.
/// Returns `Ok(None)` for status messages and error frames.
pub fn from_pcan_msg(msg: &TPCANMsg) -> Result<Option<CanFrame>, PcanError> {
    if is_status_or_error(msg.msg_type) {
        return Ok(None);
    }
    let id = from_pcan_id(msg.id, msg.msg_type)?;
    if msg.msg_type & ffi::PCAN_MESSAGE_RTR != 0 {
        let frame = CanFrame::new_remote(id, usize::from(msg.len.min(15)))
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid DLC: {}", msg.len)))?;
        return Ok(Some(frame));
    }
    Ok(Some(classic_data_frame(id, msg.len, &msg.data)?))
}

/// A classic data frame from a DLC and the data it came with, keeping a DLC
/// of 9-15 (8 bytes) as it is.
fn classic_data_frame(id: CanId, dlc: u8, data: &[u8]) -> Result<CanFrame, PcanError> {
    let dlc = usize::from(dlc.min(15));
    CanFrame::new_with_dlc(id, &data[..dlc.min(8)], dlc)
        .ok_or_else(|| PcanError::InvalidFrame(format!("invalid classic DLC: {dlc}")))
}

// ---------------------------------------------------------------------------
// CAN FD frame conversions
// ---------------------------------------------------------------------------

/// Convert a `CanFdFrame` to a `TPCANMsgFD`.
pub fn to_pcan_msg_fd(frame: &CanFdFrame) -> TPCANMsgFD {
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
        // A DLC is at most 15, fits in u8.
        #[allow(clippy::cast_possible_truncation)]
        dlc: frame.dlc() as u8,
        data,
    }
}

/// Convert a `TPCANMsgFD` to a `Frame`.
/// Returns `Ok(None)` for status messages and error frames.
pub fn from_pcan_msg_fd(msg: &TPCANMsgFD) -> Result<Option<Frame>, PcanError> {
    if is_status_or_error(msg.msg_type) {
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
    } else if msg.msg_type & ffi::PCAN_MESSAGE_RTR != 0 {
        // Classic remote frame received on an FD channel.
        let frame = CanFrame::new_remote(id, usize::from(msg.dlc.min(15)))
            .ok_or_else(|| PcanError::InvalidFrame(format!("invalid classic DLC: {}", msg.dlc)))?;
        Ok(Some(Frame::Can(frame)))
    } else {
        // Classic frame received on an FD channel.
        let frame = classic_data_frame(id, msg.dlc, &msg.data)?;
        Ok(Some(Frame::Can(frame)))
    }
}

// ---------------------------------------------------------------------------
// DLC decoding
// ---------------------------------------------------------------------------

/// Convert a CAN FD DLC code (0–15) to a byte count.
///
/// Out-of-range codes (> 15) are clamped to 64 to prevent panics when
/// slicing into a 64-byte buffer with data from an FFI source.
const fn dlc_code_to_bytes(dlc: u8) -> u8 {
    match dlc {
        0..=8 => dlc,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
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
    fn large_dlc_roundtrip() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new_with_dlc(id, &[1, 2, 3, 4, 5, 6, 7, 8], 12).unwrap();
        let msg = to_pcan_msg(&frame);
        assert_eq!(msg.len, 12);
        assert_eq!(from_pcan_msg(&msg).unwrap().unwrap(), frame);
    }

    #[test]
    fn large_dlc_classic_frame_on_fd_channel() {
        let msg = TPCANMsgFD {
            id: 0x100,
            msg_type: PCAN_MESSAGE_STANDARD,
            dlc: 15,
            data: [0x55; 64],
        };
        let Some(Frame::Can(frame)) = from_pcan_msg_fd(&msg).unwrap() else {
            panic!("expected classic frame");
        };
        assert_eq!(frame.dlc(), 15);
        assert_eq!(frame.data(), &[0x55; 8]);
    }

    #[test]
    fn rtr_message_is_a_remote_frame() {
        let msg = TPCANMsg {
            id: 0x100,
            msg_type: ffi::PCAN_MESSAGE_RTR,
            len: 4,
            data: [0; 8],
        };
        let frame = from_pcan_msg(&msg).unwrap().unwrap();
        assert!(frame.is_remote_frame());
        assert_eq!(frame.id(), CanId::new_standard(0x100).unwrap());
        assert_eq!(frame.dlc(), 4);
        assert_eq!(frame.len(), 0);
    }

    #[test]
    fn remote_frame_roundtrip() {
        let id = CanId::new_extended(0x18EA_FFFE).unwrap();
        let frame = CanFrame::new_remote(id, 8).unwrap();
        let msg = to_pcan_msg(&frame);
        assert_ne!(msg.msg_type & ffi::PCAN_MESSAGE_RTR, 0);
        assert_eq!(msg.len, 8);
        assert_eq!(from_pcan_msg(&msg).unwrap(), Some(frame));
    }

    #[test]
    fn classic_remote_frame_on_fd_channel() {
        let msg = TPCANMsgFD {
            id: 0x100,
            msg_type: ffi::PCAN_MESSAGE_RTR,
            dlc: 2,
            data: [0; 64],
        };
        let frame = from_pcan_msg_fd(&msg).unwrap().unwrap();
        assert!(frame.is_remote_frame());
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

    // An error frame (with PCAN_ALLOW_ERROR_FRAMES on) carries the error type
    // as its ID (1 = bit, 2 = form, 4 = stuff, 8 = other) and the counters in
    // its data, so it isn't a frame from the bus.
    #[test]
    fn skip_error_frame() {
        let msg = TPCANMsg {
            id: 0x1,
            msg_type: ffi::PCAN_MESSAGE_ERRFRAME,
            len: 4,
            data: [0, 0, 8, 128, 0, 0, 0, 0],
        };
        assert!(from_pcan_msg(&msg).unwrap().is_none());
    }

    #[test]
    fn skip_error_frame_fd() {
        let mut data = [0; 64];
        data[2] = 8;
        data[3] = 128;
        let msg = TPCANMsgFD {
            id: 0x1,
            msg_type: ffi::PCAN_MESSAGE_ERRFRAME,
            dlc: 4,
            data,
        };
        assert!(from_pcan_msg_fd(&msg).unwrap().is_none());
    }

    #[test]
    fn dlc_encoding_roundtrip() {
        let id = CanId::new_standard(0x100).unwrap();
        for &bytes in &[0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64] {
            let frame = CanFdFrame::new(id, &[0; 64][..usize::from(bytes)], false, false).unwrap();
            #[allow(clippy::cast_possible_truncation)] // a DLC is at most 15
            let back = dlc_code_to_bytes(frame.dlc() as u8);
            assert_eq!(bytes, back, "DLC roundtrip failed for {bytes}");
        }
    }
}
