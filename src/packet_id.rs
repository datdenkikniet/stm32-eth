#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Clone)]
pub struct PacketId(pub(crate) usize);

impl PacketId {
    pub const INIT: Option<Self> = None;
}

/// This trait should not be implemented by other crates
pub trait IntoPacketId {
    fn to_packet_id(&self) -> PacketId;
}

impl IntoPacketId for PacketId {
    fn to_packet_id(&self) -> PacketId {
        Self(self.0)
    }
}

#[cfg(feature = "smoltcp-phy")]
impl IntoPacketId for smoltcp::phy::PacketId {
    fn to_packet_id(&self) -> PacketId {
        PacketId(self.id())
    }
}
