use can_hal::filter::Filter as HalFilter;
use can_hal::frame::{CanFdFrame as HalFdFrame, CanFrame, Frame as HalFrame};
use can_hal::id::CanId;
use socketcan::{
    frame::{CanAnyFrame, CanDataFrame, CanFdFrame as ScFdFrame},
    CanFilter, EmbeddedFrame, ExtendedId, Id, StandardId,
};

const CAN_EFF_FLAG: u32 = 0x8000_0000;

use crate::error::SocketCanError;

/// Convert a can-hal CanId to a socketcan/embedded_can Id.
pub(crate) fn to_socketcan_id(id: CanId) -> Id {
    match id {
        // Unwraps are safe: can-hal already validates the range.
        CanId::Standard(v) => Id::Standard(StandardId::new(v).unwrap()),
        CanId::Extended(v) => Id::Extended(ExtendedId::new(v).unwrap()),
    }
}

/// Convert a socketcan/embedded_can Id to a can-hal CanId.
pub(crate) fn from_socketcan_id(id: Id) -> CanId {
    match id {
        Id::Standard(sid) => CanId::Standard(sid.as_raw()),
        Id::Extended(eid) => CanId::Extended(eid.as_raw()),
    }
}

/// Convert a can-hal CanFrame to a socketcan CanDataFrame.
pub(crate) fn to_socketcan_data_frame(frame: &CanFrame) -> Result<CanDataFrame, SocketCanError> {
    let id = to_socketcan_id(frame.id());
    CanDataFrame::new(id, frame.data())
        .ok_or_else(|| SocketCanError::InvalidFrame("failed to construct CanDataFrame".into()))
}

/// Convert a socketcan CanDataFrame to a can-hal CanFrame.
pub(crate) fn from_socketcan_data_frame(frame: &CanDataFrame) -> Result<CanFrame, SocketCanError> {
    let id = from_socketcan_id(EmbeddedFrame::id(frame));
    CanFrame::new(id, frame.data())
        .ok_or_else(|| SocketCanError::InvalidFrame("failed to construct can-hal CanFrame".into()))
}

/// Convert a can-hal CanFdFrame to a socketcan CanFdFrame.
pub(crate) fn to_socketcan_fd_frame(frame: &HalFdFrame) -> Result<ScFdFrame, SocketCanError> {
    let id = to_socketcan_id(frame.id());
    let mut sc_frame = ScFdFrame::new(id, frame.data()).ok_or_else(|| {
        SocketCanError::InvalidFrame("failed to construct socketcan CanFdFrame".into())
    })?;
    sc_frame.set_brs(frame.brs());
    sc_frame.set_esi(frame.esi());
    Ok(sc_frame)
}

/// Convert a socketcan CanFdFrame to a can-hal CanFdFrame.
pub(crate) fn from_socketcan_fd_frame(frame: &ScFdFrame) -> Result<HalFdFrame, SocketCanError> {
    let id = from_socketcan_id(EmbeddedFrame::id(frame));
    HalFdFrame::new(id, frame.data(), frame.is_brs(), frame.is_esi()).ok_or_else(|| {
        SocketCanError::InvalidFrame("failed to construct can-hal CanFdFrame".into())
    })
}

/// Convert a socketcan CanAnyFrame to a can-hal Frame.
pub(crate) fn from_socketcan_any_frame(frame: CanAnyFrame) -> Result<HalFrame, SocketCanError> {
    match frame {
        CanAnyFrame::Normal(data_frame) => {
            Ok(HalFrame::Can(from_socketcan_data_frame(&data_frame)?))
        }
        CanAnyFrame::Fd(fd_frame) => Ok(HalFrame::Fd(from_socketcan_fd_frame(&fd_frame)?)),
        CanAnyFrame::Remote(_) => Err(SocketCanError::InvalidFrame(
            "remote frames are not supported".into(),
        )),
        CanAnyFrame::Error(_) => Err(SocketCanError::InvalidFrame(
            "error frames are not supported".into(),
        )),
    }
}

/// Convert a can-hal Filter to a socketcan CanFilter.
pub(crate) fn to_socketcan_filter(filter: &HalFilter) -> CanFilter {
    let (raw_id, mask) = match filter.id {
        CanId::Standard(v) => (v as u32, filter.mask),
        CanId::Extended(v) => (v | CAN_EFF_FLAG, filter.mask | CAN_EFF_FLAG),
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
        let back = from_socketcan_any_frame(any).unwrap();
        assert_eq!(back, HalFrame::Can(frame));
    }

    #[test]
    fn any_frame_fd() {
        let id = CanId::new_standard(0x400).unwrap();
        let data = [0xBB; 12];
        let frame = HalFdFrame::new(id, &data, false, true).unwrap();
        let sc = to_socketcan_fd_frame(&frame).unwrap();
        let any = CanAnyFrame::Fd(sc);
        let back = from_socketcan_any_frame(any).unwrap();
        assert_eq!(back, HalFrame::Fd(frame));
    }

    #[test]
    fn filter_standard() {
        let filter = HalFilter {
            id: CanId::new_standard(0x100).unwrap(),
            mask: 0x7FF,
        };
        let sc = to_socketcan_filter(&filter);
        // Standard ID should not have CAN_EFF_FLAG set
        assert_eq!(sc, CanFilter::new(0x100, 0x7FF));
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
}
