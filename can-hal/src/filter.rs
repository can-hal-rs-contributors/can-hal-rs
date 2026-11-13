use crate::error::CanError;
use crate::id::CanId;

/// A hardware acceptance filter defined by an ID and a mask.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Filter {
    pub id: CanId,
    pub mask: u32,
}

impl Filter {
    /// Create a filter, masking off any bits above the ID's valid range.
    ///
    /// Standard IDs: mask is clamped to 11 bits (0x7FF).
    /// Extended IDs: mask is clamped to 29 bits (0x1FFF_FFFF).
    #[must_use]
    pub const fn new(id: CanId, mask: u32) -> Self {
        let max = match id {
            CanId::Standard(_) => 0x7FF,
            CanId::Extended(_) => 0x1FFF_FFFF,
        };
        Self {
            id,
            mask: mask & max,
        }
    }
}

/// Hardware acceptance filtering.
///
/// A filter matches frames of its own ID type only: a standard filter never
/// passes an extended frame, nor an extended filter a standard one. So a frame
/// type with no filter in the set is rejected. Remote frames match on their ID
/// like data frames.
///
/// **Important**: The exact semantics of multiple filters depend on the backend.
/// Some hardware (e.g. SocketCAN) supports multiple independent filters
/// (union - a frame passes if it matches *any* filter). Other hardware
/// (e.g. PCAN, Kvaser) only supports a single filter per frame type
/// (standard / extended), so multiple filters must be merged into one, which
/// may accept a broader range of IDs of that type than intended, but never a
/// frame of the other type.
///
/// For portable code that needs precise multi-ID filtering, consider using a
/// single permissive hardware filter and applying software-level filtering on
/// received frames.
pub trait Filterable {
    type Error: CanError;

    /// Apply the given set of acceptance filters.
    ///
    /// Replaces any previously configured filters. An empty slice is equivalent
    /// to calling [`clear_filters`](Self::clear_filters).
    ///
    /// On `Err`, implementations should make a best-effort attempt to leave
    /// the channel in an accept-all state rather than a half-applied one.
    /// If that secondary cleanup itself fails, the filter state is
    /// unspecified.
    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), Self::Error>;

    /// Remove all acceptance filters (accept everything).
    fn clear_filters(&mut self) -> Result<(), Self::Error>;
}
