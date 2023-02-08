use core::marker::PhantomData;

use super::EthernetDMA;
use super::MTU;
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, PacketId, RxToken, TxToken};
use smoltcp::time::Instant;

/// Use this Ethernet driver with [smoltcp](https://github.com/smoltcp-rs/smoltcp)
impl<'rx, 'tx, 'b> Device for &'b mut EthernetDMA<'rx, 'tx> {
    type RxToken<'a> = EthRxToken<'a> where Self: 'a;
    type TxToken<'a> = EthTxToken<'a, 'rx, 'tx> where Self: 'a;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = crate::dma::MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::ignored();
        caps
    }

    fn receive(
        &mut self,
        _timestamp: Instant,
        rx_packet_id: PacketId,
        tx_packet_id: PacketId,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let packet = self.recv_next(Some(rx_packet_id.into())).ok()?;

        let mut data = [0u8; MTU + 2];
        data[..packet.len()].copy_from_slice(&packet);
        let len = packet.len();

        drop(packet);

        let rx = EthRxToken {
            data,
            len,
            _lifetime: Default::default(),
        };

        let tx = EthTxToken {
            eth: self,
            packet_id: tx_packet_id,
        };
        Some((rx, tx))
    }

    fn transmit(&mut self, _timestamp: Instant, packet_id: PacketId) -> Option<Self::TxToken<'_>> {
        Some(EthTxToken {
            eth: self,
            packet_id,
        })
    }
}

/// An Ethernet RX token that can be consumed in order to receive
/// an ethernet packet.
pub struct EthRxToken<'a> {
    len: usize,
    data: [u8; MTU + 2],
    _lifetime: PhantomData<&'a u32>,
}

impl<'a> RxToken for EthRxToken<'a> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(&mut self.data[..self.len])
    }
}

/// Just a reference to [`Eth`](../struct.EthernetDMA.html) for sending a
/// packet later with [`consume()`](#method.consume).
pub struct EthTxToken<'dma, 'rx, 'tx> {
    eth: &'dma mut EthernetDMA<'rx, 'tx>,
    packet_id: PacketId,
}

impl<'dma, 'rx, 'tx> TxToken for EthTxToken<'dma, 'rx, 'tx> {
    /// Allocate a [`Buffer`](../struct.Buffer.html), yield with
    /// `f(buffer)`, and send it as an Ethernet packet.
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        self.eth.send(len, Some(self.packet_id.into()), f).unwrap()
    }
}
