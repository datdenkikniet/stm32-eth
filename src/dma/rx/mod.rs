use core::marker::PhantomData;

use super::{
    ring::{Buffer, EntryRing, SettableDescriptorEntry},
    PacketId,
};
use crate::peripherals::ETHERNET_DMA;

#[cfg(feature = "ptp")]
use crate::{dma::TimestampError, ptp::Timestamp};

#[cfg(feature = "f-series")]
mod f_series_desc;
#[cfg(feature = "f-series")]
use f_series_desc as descriptor;

#[cfg(feature = "stm32h7xx-hal")]
mod h_desc;
#[cfg(feature = "stm32h7xx-hal")]
use h_desc as descriptor;

pub use descriptor::RxDescriptor;

impl SettableDescriptorEntry for RxDescriptor {
    fn set_buffer(&mut self, buffer: Buffer) {
        self.set_owned(buffer);
    }
}

/// Errors that can occur during RX
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub enum RxError {
    /// Receiving would block
    WouldBlock,
    /// The received packet was truncated
    Truncated,
    /// An error occured with the DMA
    DmaError,
}

/// An RX descriptor ring.
pub type RxDescriptorRing<'rx> = EntryRing<'rx, RxDescriptor>;

pub struct NotRunning;
pub struct Running;

/// Rx DMA state
pub struct RxRing<'data, STATE> {
    ring: RxDescriptorRing<'data>,
    next_entry: usize,
    state: PhantomData<STATE>,
}

impl<'data, STATE> RxRing<'data, STATE> {
    /// Get current `RunningState`
    pub fn running_state(&self) -> RunningState {
        // SAFETY: we only perform an atomic read of `dmasr`
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        #[cfg(feature = "f-series")]
        let rps = eth_dma.dmasr.read().rps().bits();
        #[cfg(feature = "stm32h7xx-hal")]
        let rps = eth_dma.dmadsr.read().rps0().bits();

        match rps {
            //  Reset or Stop Receive Command issued
            0b000 => RunningState::Stopped,
            //  Fetching receive transfer descriptor
            0b001 => RunningState::Running,
            //  Waiting for receive packet
            0b011 => RunningState::Running,
            //  Receive descriptor unavailable
            0b100 => RunningState::Stopped,
            //  Closing receive descriptor
            0b101 => RunningState::Running,
            //  Transferring the receive packet data from receive buffer to host memory
            0b111 => RunningState::Running,
            #[cfg(feature = "stm32h7xx-hal")]
            // Timestamp write state
            0b110 => RunningState::Running,
            _ => RunningState::Unknown,
        }
    }
}

impl<'data> RxRing<'data, NotRunning> {
    /// Allocate
    pub fn new(ring: RxDescriptorRing<'data>) -> Self {
        RxRing {
            ring,
            next_entry: 0,
            state: Default::default(),
        }
    }

    /// Start the RX ring
    pub fn start(mut self, eth_dma: &ETHERNET_DMA) -> RxRing<'data, Running> {
        // Setup ring
        self.ring.buffers_and_entries(|entry, buffer| {
            if let Some(buffer) = &buffer {
                defmt::trace!("Setting up buffer {}", buffer.idx());
            }
            entry.setup(buffer);
        });

        self.next_entry = 0;
        let ring_ptr = self.ring.entries_start_address();

        #[cfg(feature = "f-series")]
        {
            self.ring.last_entry_mut().set_end_of_ring();
            // Set the RxDma ring start address.
            eth_dma
                .dmardlar
                .write(|w| unsafe { w.srl().bits(ring_ptr as u32) });

            // // Start receive
            eth_dma.dmaomr.modify(|_, w| w.sr().set_bit());
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // TODO: assert that ethernet DMA can access
            // the memory in these rings
            assert!(self.ring.entry_count() >= 4);

            // Assert that the descriptors are properly aligned.
            assert!(ring_ptr as u32 % 4 == 0);
            assert!(self.ring.last_entry_mut() as *const _ as u32 % 4 == 0);

            // Set the start pointer.
            eth_dma
                .dmacrx_dlar
                .write(|w| unsafe { w.bits(ring_ptr as u32) });

            // Set the Receive Descriptor Ring Length
            eth_dma
                .dmacrx_rlr
                .write(|w| w.rdrl().variant((self.ring.entry_count() - 1) as u16));

            // Set the tail pointer
            eth_dma
                .dmacrx_dtpr
                .write(|w| unsafe { w.bits(self.ring.last_entry_mut() as *const _ as u32) });

            // Set receive buffer size
            let receive_buffer_size = self.ring.buffer_size() as u16;
            assert!(receive_buffer_size % 4 == 0);

            eth_dma.dmacrx_cr.modify(|_, w| unsafe {
                w
                    // Start receive
                    .sr()
                    .set_bit()
                    // Set receive buffer size
                    .rbsz()
                    .bits(receive_buffer_size >> 1)
                    // AUtomatically flush on bus error
                    .rpf()
                    .set_bit()
                    // RX DMA programmable burst length.
                    .rxpbl()
                    .variant(32)
            });
        }

        let me = RxRing {
            ring: self.ring,
            next_entry: self.next_entry,
            state: Default::default(),
        };

        me.demand_poll();

        me
    }
}

impl<'data> RxRing<'data, Running> {
    /// Demand that the DMA engine polls the current `RxDescriptor`
    /// (when in `RunningState::Stopped`.)
    pub fn demand_poll(&self) {
        // # SAFETY
        //
        // On F7, we only perform an atomic write to `damrpdr`.
        //
        // On H7, we only perform a Read-Write to `dmacrx_dtpr`,
        // always with the same value. Running `demand_poll` concurrently
        // with the other location in which this register is written ([`RxRing::start`])
        // is impossible, which is guaranteed the state transition from NotRunning to
        // Running.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        #[cfg(feature = "f-series")]
        eth_dma.dmarpdr.write(|w| unsafe { w.rpd().bits(1) });

        // On H7, we poll by re-writing the tail pointer register.
        #[cfg(feature = "stm32h7xx-hal")]
        eth_dma
            .dmacrx_dtpr
            .modify(|r, w| unsafe { w.bits(r.bits()) });
    }

    /// Stop the DMA engine.
    pub fn stop(&mut self, eth_dma: &ETHERNET_DMA) {
        #[cfg(feature = "f-series")]
        let start_reg = &eth_dma.dmaomr;

        #[cfg(feature = "stm32h7xx-hal")]
        let start_reg = &eth_dma.dmacrx_cr;

        start_reg.modify(|_, w| w.sr().clear_bit());

        while self.running_state() != RunningState::Stopped {}
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next<'a>(
        &'a mut self,
        #[allow(unused_variables)] packet_id: Option<PacketId>,
    ) -> Result<RxPacket<'a, 'data>, RxError> {
        if !self.running_state().is_running() {
            self.demand_poll();
        }

        let entry = self.next_entry;
        let entries_len = self.ring.len();

        let (descriptor, buffer) = self.ring.entry_buffer(entry).ok_or(RxError::WouldBlock)?;

        let mut res = descriptor.take_received(packet_id);

        if res.as_mut().err() != Some(&mut RxError::WouldBlock) {
            self.next_entry = (self.next_entry + 1) % entries_len;
        }

        #[cfg(all(feature = "ptp", feature = "stm32h7xx-hal"))]
        let (timestamp, descriptor, buffer) = {
            if res.is_ok() {
                let desc_has_timestamp = descriptor.has_timestamp();

                drop(descriptor);

                // On H7's, the timestamp is stored in the next Context
                // descriptor.
                let ctx_descriptor = self.ring.entry(self.next_entry);

                let timestamp = if desc_has_timestamp {
                    if let Some(timestamp) = ctx_descriptor.read_timestamp() {
                        Some(timestamp)
                    } else {
                        None
                    }
                } else {
                    None
                };

                if let Some((ctx_descriptor, ctx_des_buffer)) =
                    self.ring.entry_and_next_buffer(self.next_entry)
                {
                    if !ctx_descriptor.is_owned() {
                        // Advance over this buffer
                        self.next_entry = (self.next_entry + 1) % entries_len;
                        ctx_descriptor.set_owned(ctx_des_buffer);
                    }
                }

                let descriptor = self.ring.entry_mut(entry);

                descriptor.attach_timestamp(timestamp);

                (timestamp, descriptor, buffer)
            } else {
                let descriptor = self.ring.entry_mut(entry);
                descriptor.attach_timestamp(None);
                (None, descriptor, buffer)
            }
        };

        res.map(move |_| RxPacket {
            entry,
            buffer,
            ring: self,
        })
    }

    pub fn available(&mut self) -> bool {
        !self.ring.entry(self.next_entry).is_owned()
    }
}

#[cfg(feature = "ptp")]
impl<'data, STATE> RxRing<'data, STATE> {
    pub fn get_timestamp_for_id(&self, id: PacketId) -> Result<Timestamp, TimestampError> {
        for descriptor in self.ring.entries() {
            if let (Some(packet_id), Some(timestamp)) =
                (descriptor.packet_id(), descriptor.timestamp())
            {
                if packet_id == &id {
                    return Ok(timestamp.clone());
                }
            }
        }
        return Err(TimestampError::IdNotFound);
    }
}

/// Running state of the `RxRing`
#[derive(PartialEq, Eq, Debug)]
pub enum RunningState {
    Unknown,
    Stopped,
    Running,
}

impl RunningState {
    /// whether self equals to `RunningState::Running`
    pub fn is_running(&self) -> bool {
        *self == RunningState::Running
    }
}

/// A received packet.
///
/// This packet implements [Deref<\[u8\]>](core::ops::Deref) and should be used
/// as a slice.
pub struct RxPacket<'a, 'ring> {
    entry: usize,
    buffer: Buffer,
    ring: &'a mut RxRing<'ring, Running>,
}

impl RxPacket<'_, '_> {
    /// Pass the received packet back to the DMA engine.
    pub fn free(self) {
        drop(self)
    }

    fn frame_length(&self) -> usize {
        self.ring.ring.entry(self.entry).frame_length()
    }

    /// Get the timestamp associated with this packet
    #[cfg(feature = "ptp")]
    pub fn timestamp(&self) -> Option<Timestamp> {
        self.timestamp
    }
}

impl core::ops::Deref for RxPacket<'_, '_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer[0..self.frame_length()]
    }
}

impl core::ops::DerefMut for RxPacket<'_, '_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        let frame_len = self.frame_length();
        &mut self.buffer[0..frame_len]
    }
}

impl Drop for RxPacket<'_, '_> {
    fn drop(&mut self) {
        self.ring.ring.free(self.buffer.idx());
        self.ring.ring.attach_free_buffers(self.entry + 1);
        self.ring.demand_poll();
    }
}
