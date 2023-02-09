use nonmax::NonMaxU32;

/// A packet ID.
///
/// This packet ID can be used to obtain information about a specific
/// ethernet frame (either sent or received) from the DMA.
///
#[cfg_attr(
    feature = "ptp",
    doc = "
The main use is obtaining timestamps for frames using [`EthernetDMA::get_timestamp_for_id`](crate::EthernetDMA::get_timestamp_for_id)
"
)]
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct PacketId(pub NonMaxU32);

#[cfg(feature = "defmt")]
impl defmt::Format for PacketId {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "PacketId({})", self.0.get())
    }
}

impl PacketId {
    /// The initial value for an [`Option<PacketId>`]
    pub const INIT: Option<Self> = None;
}

impl From<u32> for PacketId {
    fn from(value: u32) -> Self {
        let value = if value == u32::MAX {
            // SAFETY: 0 != u32::MAX
            unsafe { NonMaxU32::new_unchecked(0) }
        } else {
            // SAFETY: value != u32::MAX
            unsafe { NonMaxU32::new_unchecked(value) }
        };
        Self(value)
    }
}
