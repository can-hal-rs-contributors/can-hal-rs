use can_hal::filter::Filter as HalFilter;
use can_hal::frame::{CanFdFrame as HalFdFrame, CanFrame, Frame as HalFrame};
use can_hal::id::CanId;
use socketcan::{
    frame::{CanAnyFrame, CanDataFrame, CanFdFrame as ScFdFrame, CanRemoteFrame},
    CanFilter, CanFrame as ScFrame, EmbeddedFrame, ExtendedId, Id, StandardId,
};

const CAN_EFF_FLAG: u32 = 0x8000_0000;

use crate::error::SocketCanError;

/// Convert a can-hal CanId to a socketcan/embedded_can Id.
pub fn to_socketcan_id(id: CanId) -> Id {
    match id {
        // Unwraps are safe: can-hal already validates the range.
        CanId::Standard(v) => Id::Standard(StandardId::new(v).unwrap()),
        CanId::Extended(v) => Id::Extended(ExtendedId::new(v).unwrap()),
    }
}

/// Convert a socketcan/embedded_can Id to a can-hal CanId.
pub fn from_socketcan_id(id: Id) -> CanId {
    match id {
        Id::Standard(sid) => CanId::Standard(sid.as_raw()),
        Id::Extended(eid) => CanId::Extended(eid.as_raw()),
    }
}

/// Convert a can-hal CanFrame to a socketcan classic frame: a data frame, or a
/// remote frame.
pub fn to_socketcan_frame(frame: &CanFrame) -> Result<ScFrame, SocketCanError> {
    if frame.is_remote_frame() {
        to_socketcan_remote_frame(frame).map(ScFrame::Remote)
    } else {
        to_socketcan_data_frame(frame).map(ScFrame::Data)
    }
}

/// Convert a can-hal data CanFrame to a socketcan CanDataFrame. A remote frame
/// is refused: see [`to_socketcan_frame`].
pub fn to_socketcan_data_frame(frame: &CanFrame) -> Result<CanDataFrame, SocketCanError> {
    if frame.is_remote_frame() {
        return Err(SocketCanError::InvalidFrame(
            "a remote frame is not a data frame".into(),
        ));
    }
    let id = to_socketcan_id(frame.id());
    let data_frame = CanDataFrame::new(id, frame.data())
        .ok_or_else(|| SocketCanError::InvalidFrame("failed to construct CanDataFrame".into()))?;
    if frame.dlc() <= 8 {
        return Ok(data_frame);
    }
    // Linux keeps the DLC (9-15) of an 8-byte classic frame in `len8_dlc`. A
    // CAN interface sends it, and reports it on receive, only with
    // `cc-len8-dlc` on; a virtual one passes it through.
    #[allow(clippy::cast_possible_truncation)] // a classic DLC is at most 15
    let dlc = frame.dlc() as u8;
    let mut raw = *data_frame.as_ref();
    raw.len8_dlc = dlc;
    CanDataFrame::try_from(raw)
        .map_err(|_| SocketCanError::InvalidFrame("failed to construct CanDataFrame".into()))
}

/// The DLC of a raw classic frame: `len8_dlc` for an 8-byte frame that has
/// one (9-15), or the length.
fn raw_dlc(len: usize, len8_dlc: u8) -> usize {
    if len == 8 && (9..=15).contains(&len8_dlc) {
        usize::from(len8_dlc)
    } else {
        len
    }
}

/// Convert a can-hal remote CanFrame to a socketcan CanRemoteFrame. A data
/// frame is refused: see [`to_socketcan_frame`].
pub fn to_socketcan_remote_frame(frame: &CanFrame) -> Result<CanRemoteFrame, SocketCanError> {
    if !frame.is_remote_frame() {
        return Err(SocketCanError::InvalidFrame(
            "a data frame is not a remote frame".into(),
        ));
    }
    let id = to_socketcan_id(frame.id());
    let remote_frame = CanRemoteFrame::new_remote(id, frame.dlc().min(8))
        .ok_or_else(|| SocketCanError::InvalidFrame("failed to construct CanRemoteFrame".into()))?;
    if frame.dlc() <= 8 {
        return Ok(remote_frame);
    }
    // As for a data frame: see `to_socketcan_data_frame`.
    #[allow(clippy::cast_possible_truncation)] // a classic DLC is at most 15
    let dlc = frame.dlc() as u8;
    let mut raw = *remote_frame.as_ref();
    raw.len8_dlc = dlc;
    CanRemoteFrame::try_from(raw)
        .map_err(|_| SocketCanError::InvalidFrame("failed to construct CanRemoteFrame".into()))
}

/// Convert a socketcan CanDataFrame to a can-hal CanFrame.
pub fn from_socketcan_data_frame(frame: &CanDataFrame) -> Result<CanFrame, SocketCanError> {
    let id = from_socketcan_id(EmbeddedFrame::id(frame));
    let dlc = raw_dlc(frame.data().len(), frame.as_ref().len8_dlc);
    CanFrame::new_with_dlc(id, frame.data(), dlc)
        .ok_or_else(|| SocketCanError::InvalidFrame("failed to construct can-hal CanFrame".into()))
}

/// Convert a socketcan CanRemoteFrame to a can-hal remote CanFrame.
pub fn from_socketcan_remote_frame(frame: &CanRemoteFrame) -> Result<CanFrame, SocketCanError> {
    let id = from_socketcan_id(EmbeddedFrame::id(frame));
    let dlc = raw_dlc(frame.dlc(), frame.as_ref().len8_dlc);
    CanFrame::new_remote(id, dlc).ok_or_else(|| {
        SocketCanError::InvalidFrame("failed to construct can-hal remote CanFrame".into())
    })
}

/// Convert a socketcan frame to a can-hal CanFrame if it is a classic data or
/// remote frame; `None` for an FD or error frame.
pub fn from_socketcan_classic_frame(
    frame: &CanAnyFrame,
) -> Option<Result<CanFrame, SocketCanError>> {
    match frame {
        CanAnyFrame::Normal(data_frame) => Some(from_socketcan_data_frame(data_frame)),
        CanAnyFrame::Remote(remote_frame) => Some(from_socketcan_remote_frame(remote_frame)),
        CanAnyFrame::Fd(_) | CanAnyFrame::Error(_) => None,
    }
}

/// Convert a can-hal CanFdFrame to a socketcan CanFdFrame.
pub fn to_socketcan_fd_frame(frame: &HalFdFrame) -> Result<ScFdFrame, SocketCanError> {
    let id = to_socketcan_id(frame.id());
    let mut sc_frame = ScFdFrame::new(id, frame.data()).ok_or_else(|| {
        SocketCanError::InvalidFrame("failed to construct socketcan CanFdFrame".into())
    })?;
    sc_frame.set_brs(frame.brs());
    sc_frame.set_esi(frame.esi());
    Ok(sc_frame)
}

/// Convert a socketcan CanFdFrame to a can-hal CanFdFrame.
pub fn from_socketcan_fd_frame(frame: &ScFdFrame) -> Result<HalFdFrame, SocketCanError> {
    let id = from_socketcan_id(EmbeddedFrame::id(frame));
    HalFdFrame::new(id, frame.data(), frame.is_brs(), frame.is_esi()).ok_or_else(|| {
        SocketCanError::InvalidFrame("failed to construct can-hal CanFdFrame".into())
    })
}

/// Convert a socketcan frame to a can-hal Frame; `None` for an error frame.
pub fn from_socketcan_any_frame(frame: &CanAnyFrame) -> Option<Result<HalFrame, SocketCanError>> {
    match frame {
        CanAnyFrame::Normal(data_frame) => {
            Some(from_socketcan_data_frame(data_frame).map(HalFrame::Can))
        }
        CanAnyFrame::Remote(remote_frame) => {
            Some(from_socketcan_remote_frame(remote_frame).map(HalFrame::Can))
        }
        CanAnyFrame::Fd(fd_frame) => Some(from_socketcan_fd_frame(fd_frame).map(HalFrame::Fd)),
        CanAnyFrame::Error(_) => None,
    }
}

/// Convert a can-hal Filter to a socketcan CanFilter.
///
/// Re-clamps the mask to the ID width to defend against struct-literal
/// `Filter` construction that bypasses `Filter::new`. Both masks include
/// `CAN_EFF_FLAG`, so the kernel compares the frame type too: a standard
/// filter rejects extended frames and an extended filter standard ones.
/// `CAN_RTR_FLAG` stays out of the mask, so remote frames match like data
/// frames.
pub fn to_socketcan_filter(filter: &HalFilter) -> CanFilter {
    let (raw_id, mask) = match filter.id {
        CanId::Standard(v) => (u32::from(v), (filter.mask & 0x7FF) | CAN_EFF_FLAG),
        CanId::Extended(v) => (
            (v & 0x1FFF_FFFF) | CAN_EFF_FLAG,
            (filter.mask & 0x1FFF_FFFF) | CAN_EFF_FLAG,
        ),
    };
    CanFilter::new(raw_id, mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_standard_id() {
        let hal_id = CanId::new_standard(0x123).unwrap();
        let sc_id = to_socketcan_id(hal_id);
        let back = from_socketcan_id(sc_id);
        assert_eq!(hal_id, back);
    }

    #[test]
    fn roundtrip_extended_id() {
        let hal_id = CanId::new_extended(0x1234_5678).unwrap();
        let sc_id = to_socketcan_id(hal_id);
        let back = from_socketcan_id(sc_id);
        assert_eq!(hal_id, back);
    }

    #[test]
    fn roundtrip_can_frame() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new(id, &[1, 2, 3]).unwrap();
        let sc = to_socketcan_data_frame(&frame).unwrap();
        let back = from_socketcan_data_frame(&sc).unwrap();
        assert_eq!(frame, back);
    }

    #[test]
    fn roundtrip_fd_frame() {
        let id = CanId::new_extended(0x200).unwrap();
        let data = [0xAA; 24];
        let frame = HalFdFrame::new(id, &data, true, false).unwrap();
        let sc = to_socketcan_fd_frame(&frame).unwrap();
        let back = from_socketcan_fd_frame(&sc).unwrap();
        assert_eq!(frame, back);
    }

    #[test]
    fn any_frame_classic() {
        let id = CanId::new_standard(0x300).unwrap();
        let frame = CanFrame::new(id, &[0xFF]).unwrap();
        let sc = to_socketcan_data_frame(&frame).unwrap();
        let any = CanAnyFrame::Normal(sc);
        let back = from_socketcan_any_frame(&any).unwrap().unwrap();
        assert_eq!(back, HalFrame::Can(frame));
    }

    #[test]
    fn any_frame_fd() {
        let id = CanId::new_standard(0x400).unwrap();
        let data = [0xBB; 12];
        let frame = HalFdFrame::new(id, &data, false, true).unwrap();
        let sc = to_socketcan_fd_frame(&frame).unwrap();
        let any = CanAnyFrame::Fd(sc);
        let back = from_socketcan_any_frame(&any).unwrap().unwrap();
        assert_eq!(back, HalFrame::Fd(frame));
    }

    #[test]
    fn roundtrip_remote_frame() {
        let id = CanId::new_extended(0x18EA_FFFE).unwrap();
        let frame = CanFrame::new_remote(id, 8).unwrap();
        let ScFrame::Remote(sc) = to_socketcan_frame(&frame).unwrap() else {
            panic!("a remote frame went as a data frame");
        };
        assert!(sc.is_remote_frame());
        assert_eq!(sc.dlc(), 8);
        let back = from_socketcan_remote_frame(&sc).unwrap();
        assert_eq!(frame, back);
    }

    #[test]
    fn roundtrip_large_dlc() {
        let id = CanId::new_standard(0x100).unwrap();
        let frame = CanFrame::new_with_dlc(id, &[1, 2, 3, 4, 5, 6, 7, 8], 12).unwrap();
        let sc = to_socketcan_data_frame(&frame).unwrap();
        assert_eq!(sc.data().len(), 8);
        assert_eq!(sc.as_ref().len8_dlc, 12);
        assert_eq!(from_socketcan_data_frame(&sc).unwrap(), frame);

        let remote = CanFrame::new_remote(id, 15).unwrap();
        let sc = to_socketcan_remote_frame(&remote).unwrap();
        assert_eq!(sc.dlc(), 8);
        assert_eq!(sc.as_ref().len8_dlc, 15);
        assert_eq!(from_socketcan_remote_frame(&sc).unwrap(), remote);
    }

    #[test]
    fn a_remote_frame_is_no_data_frame() {
        let id = CanId::new_standard(0x100).unwrap();
        let remote = CanFrame::new_remote(id, 2).unwrap();
        assert!(to_socketcan_data_frame(&remote).is_err());
        let data = CanFrame::new(id, &[1, 2]).unwrap();
        assert!(to_socketcan_remote_frame(&data).is_err());
        assert!(matches!(to_socketcan_frame(&data), Ok(ScFrame::Data(_))));
    }

    #[test]
    fn any_frame_remote() {
        let id = CanId::new_standard(0x500).unwrap();
        let frame = CanFrame::new_remote(id, 4).unwrap();
        let sc = to_socketcan_remote_frame(&frame).unwrap();
        let back = from_socketcan_any_frame(&CanAnyFrame::Remote(sc))
            .unwrap()
            .unwrap();
        assert_eq!(back, HalFrame::Can(frame.clone()));
        let classic = from_socketcan_classic_frame(&CanAnyFrame::Remote(sc))
            .unwrap()
            .unwrap();
        assert_eq!(classic, frame);
    }

    #[test]
    fn filter_standard() {
        let filter = HalFilter {
            id: CanId::new_standard(0x100).unwrap(),
            mask: 0x7FF,
        };
        let sc = to_socketcan_filter(&filter);
        // The ID has no CAN_EFF_FLAG, and the mask makes it count.
        assert_eq!(sc, CanFilter::new(0x100, 0x7FF | CAN_EFF_FLAG));
    }

    #[test]
    fn filter_extended() {
        let filter = HalFilter {
            id: CanId::new_extended(0x1234).unwrap(),
            mask: 0x1FFF_FFFF,
        };
        let sc = to_socketcan_filter(&filter);
        assert_eq!(
            sc,
            CanFilter::new(0x1234 | CAN_EFF_FLAG, 0x1FFF_FFFF | CAN_EFF_FLAG)
        );
    }

    #[test]
    fn any_frame_error_is_skipped() {
        let err = socketcan::CanErrorFrame::new_error(0x004, &[]).unwrap();
        assert!(from_socketcan_any_frame(&CanAnyFrame::Error(err)).is_none());
    }

    #[test]
    fn any_frame_fd_invalid_length_is_an_error() {
        let mut raw = socketcan::frame::canfd_frame_default();
        raw.can_id = 0x123;
        raw.len = 9;
        let any = CanAnyFrame::Fd(ScFdFrame::from(raw));
        assert!(matches!(
            from_socketcan_any_frame(&any),
            Some(Err(SocketCanError::InvalidFrame(_)))
        ));
    }
}
