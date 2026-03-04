use crate::error::CanError;
use crate::id::CanId;

/// A hardware acceptance filter defined by an ID and a mask.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Filter {
    pub id: CanId,
    pub mask: u32,
}

/// Hardware acceptance filtering. Not all backends support this.
pub trait Filterable {
    type Error: CanError;

    /// Apply the given set of acceptance filters.
    fn set_filters(&mut self, filters: &[Filter]) -> Result<(), Self::Error>;

    /// Remove all acceptance filters (accept everything).
    fn clear_filters(&mut self) -> Result<(), Self::Error>;
}
