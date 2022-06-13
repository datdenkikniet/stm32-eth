use crate::{stm32::ETHERNET_DMA, PacketId, Timestamp, TimestampError};

use core::{
    ops::{Deref, DerefMut},
    sync::atomic::{self, Ordering},
};

use crate::{
    desc::Descriptor,
    ring::{RingDescriptor, RingEntry},
};

#[derive(Debug, PartialEq)]
pub enum RxError {
    WouldBlock,
    Truncated,
    DmaError,
}

/// Owned by DMA engine
const RXDESC_0_OWN: u32 = 1 << 31;
/// First descriptor
const RXDESC_0_FS: u32 = 1 << 9;
/// Last descriptor
const RXDESC_0_LS: u32 = 1 << 8;
/// RX timestamp
const RXDESC_0_TIMESTAMP: u32 = 1 << 7;
/// Error summary
const RXDESC_0_ES: u32 = 1 << 15;
/// Frame length
const RXDESC_0_FL_MASK: u32 = 0x3FFF;
const RXDESC_0_FL_SHIFT: usize = 16;

const RXDESC_1_RBS_SHIFT: usize = 0;
const RXDESC_1_RBS_MASK: u32 = 0x0fff << RXDESC_1_RBS_SHIFT;
/// Second address chained
const RXDESC_1_RCH: u32 = 1 << 14;
/// End Of Ring
const RXDESC_1_RER: u32 = 1 << 15;

#[repr(C)]
#[derive(Clone)]
pub struct RxDescriptor {
    desc: Descriptor,
    timestamp_info: Option<(PacketId, Timestamp)>,
    buffer_address: Option<u32>,
    next_descriptor: Option<u32>,
}

impl RxDescriptor {
    /// Creates an zeroed RxDescriptor.
    pub const fn new() -> Self {
        Self {
            desc: Descriptor::new(),
            timestamp_info: None,
            buffer_address: None,
            next_descriptor: None,
        }
    }

    /// Is owned by the DMA engine?
    fn is_owned(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_OWN) == RXDESC_0_OWN
    }

    /// Pass ownership to the DMA engine
    ///
    /// Overrides old timestamp data
    fn set_owned(&mut self) {
        self.write_buffer1();
        self.write_buffer2();

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::Release);
        atomic::compiler_fence(Ordering::Release);

        unsafe {
            self.desc.modify(0, |w| w | RXDESC_0_OWN);
        }

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::SeqCst);
    }

    fn has_error(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_ES) == RXDESC_0_ES
    }

    /// Descriptor contains first buffer of frame
    fn is_first(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_FS) == RXDESC_0_FS
    }

    /// Descriptor contains last buffers of frame
    fn is_last(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_LS) == RXDESC_0_LS
    }

    /// Get PTP timestamps if available
    pub fn timestamp(&self) -> Option<Timestamp> {
        if self.desc.read(0) & RXDESC_0_TIMESTAMP == RXDESC_0_TIMESTAMP && self.is_last() {
            let seconds = self.desc.read(3);
            let subseconds = self.desc.read(2);

            let timestamp = Timestamp::new(seconds, subseconds);

            Some(timestamp)
        } else {
            None
        }
    }

    /// Rewrite buffer1 to the last value we wrote to it
    ///
    /// In our case, the address of the data buffer for this descriptor
    fn write_buffer1(&mut self) {
        let buffer_addr = self
            .buffer_address
            .expect("Writing buffer1 of an RX descriptor, but `buffer_address` is None");

        unsafe {
            self.desc.write(2, buffer_addr as u32);
        }
    }

    fn set_buffer1(&mut self, buffer: *const u8, len: usize) {
        self.buffer_address = Some(buffer as u32);
        unsafe {
            self.write_buffer1();
            self.desc.modify(1, |w| {
                (w & !RXDESC_1_RBS_MASK) | ((len as u32) << RXDESC_1_RBS_SHIFT)
            });
        }
    }

    /// Rewrite buffer2 to the last value we wrote it to
    ///
    /// In our case, the address of the next descriptor (may be zero)
    fn write_buffer2(&mut self) {
        let addr = self
            .next_descriptor
            .expect("Writing buffer2 of an RX descriptor, but `next_descriptor` is None");

        unsafe {
            self.desc.write(3, addr);
        }
    }

    // points to next descriptor (RCH)
    fn set_buffer2(&mut self, buffer: *const u8) {
        self.next_descriptor = Some(buffer as u32);
        self.write_buffer2();
    }

    fn set_end_of_ring(&mut self) {
        unsafe {
            self.desc.modify(1, |w| w | RXDESC_1_RER);
        }
    }

    fn get_frame_len(&self) -> usize {
        ((self.desc.read(0) >> RXDESC_0_FL_SHIFT) & RXDESC_0_FL_MASK) as usize
    }
}

pub type RxRingEntry = RingEntry<RxDescriptor>;

impl RingDescriptor for RxDescriptor {
    fn setup(&mut self, buffer: *const u8, len: usize, next: Option<&Self>) {
        // Defer this initialization to this function, so we can have `RingEntry` on bss.
        unsafe {
            self.desc.write(1, RXDESC_1_RCH);
        }
        self.set_buffer1(buffer, len);
        match next {
            Some(next) => self.set_buffer2(&next.desc as *const Descriptor as *const u8),
            None => {
                #[allow(clippy::zero_ptr)]
                self.set_buffer2(0 as *const u8);
                self.set_end_of_ring();
            }
        };
        self.set_owned();
    }
}

impl RxRingEntry {
    pub const RX_INIT: Self = Self::new();

    fn take_received(&mut self) -> Result<RxPacket, RxError> {
        if self.desc().is_owned() {
            Err(RxError::WouldBlock)
        } else if self.desc().has_error() {
            self.desc_mut().set_owned();
            Err(RxError::DmaError)
        } else if self.desc().is_first() && self.desc().is_last() {
            let frame_len = self.desc().get_frame_len();

            // "Subsequent reads and writes cannot be moved ahead of preceding reads."
            atomic::compiler_fence(Ordering::Acquire);

            // TODO: obtain ethernet frame type (RDESC_1_FT)
            let pkt = RxPacket {
                entry: self,
                length: frame_len,
            };
            Ok(pkt)
        } else {
            self.desc_mut().set_owned();
            Err(RxError::Truncated)
        }
    }
}

pub struct RxPacket<'a> {
    entry: &'a mut RxRingEntry,
    length: usize,
}

impl<'a> Deref for RxPacket<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.entry.as_slice()[0..self.length]
    }
}

impl<'a> DerefMut for RxPacket<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.entry.as_mut_slice()[0..self.length]
    }
}

impl<'a> Drop for RxPacket<'a> {
    fn drop(&mut self) {
        self.entry.desc_mut().set_owned();
    }
}

impl<'a> RxPacket<'a> {
    // Pass back to DMA engine
    pub fn free(self) {
        drop(self)
    }
}

/// Rx DMA state
pub struct RxRing<'a> {
    entries: &'a mut [RxRingEntry],
    next_entry: usize,
}

impl<'a> RxRing<'a> {
    pub fn get_timestamp_for_id(
        &mut self,
        id: PacketId,
    ) -> Result<Timestamp, (TimestampError, PacketId)> {
        for entry in self.entries.iter_mut() {
            if let Some((packet_id, timestamp)) = &mut entry.desc_mut().timestamp_info {
                if packet_id == &id {
                    let ts = *timestamp;
                    entry.desc_mut().timestamp_info.take();
                    return Ok(ts);
                }
            }
        }

        return Err((TimestampError::IdNotFound, id));
    }

    /// Allocate
    pub fn new(entries: &'a mut [RxRingEntry]) -> Self {
        RxRing {
            entries,
            next_entry: 0,
        }
    }

    /// Setup the DMA engine (**required**)
    pub fn start(&mut self, eth_dma: &ETHERNET_DMA) {
        // Setup ring
        {
            let mut previous: Option<&mut RxRingEntry> = None;
            for entry in self.entries.iter_mut() {
                if let Some(prev_entry) = &mut previous {
                    prev_entry.setup(Some(entry));
                }
                previous = Some(entry);
            }
            if let Some(entry) = &mut previous {
                entry.setup(None);
            }
        }
        self.next_entry = 0;
        let ring_ptr = self.entries[0].desc() as *const RxDescriptor;

        // Register RxDescriptor
        eth_dma
            .dmardlar
            .write(|w| unsafe { w.srl().bits(ring_ptr as u32) });

        // We already have fences in `set_owned`, which is called in `setup`

        // Start receive
        eth_dma.dmaomr.modify(|_, w| w.sr().set_bit());

        self.demand_poll(eth_dma);
    }

    /// Demand that the DMA engine polls the current `RxDescriptor`
    /// (when in `RunningState::Stopped`.)
    pub fn demand_poll(&self, eth_dma: &ETHERNET_DMA) {
        eth_dma.dmarpdr.write(|w| unsafe { w.rpd().bits(1) });
    }

    /// Get current `RunningState`
    pub fn running_state(&self, eth_dma: &ETHERNET_DMA) -> RunningState {
        match eth_dma.dmasr.read().rps().bits() {
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
            _ => RunningState::Unknown,
        }
    }

    pub fn clear_rx_timestamps(&mut self) {
        self.entries.iter_mut().for_each(|entry| {
            entry.desc_mut().timestamp_info.take();
        });
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(
        &mut self,
        eth_dma: &ETHERNET_DMA,
        packet_id: Option<PacketId>,
    ) -> Result<RxPacket, RxError> {
        if !self.running_state(eth_dma).is_running() {
            self.demand_poll(eth_dma);
        }

        let entries_len = self.entries.len();
        let mut result = self.entries[self.next_entry].take_received();

        if result.as_mut().err() != Some(&mut RxError::WouldBlock) {
            self.next_entry += 1;
            if self.next_entry >= entries_len {
                self.next_entry = 0;
            }

            if let Ok(entry) = &mut result {
                if let (Some(packet_id), Some(timestamp)) =
                    (packet_id, entry.entry.desc().timestamp())
                {
                    entry.entry.desc_mut().timestamp_info = Some((packet_id, timestamp))
                }
            }
        }

        result
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
