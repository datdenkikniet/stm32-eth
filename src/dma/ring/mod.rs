use nonmax::NonMaxU16;

const MTU: usize = 1522;

mod buffer_ring;
pub(crate) use buffer_ring::{Buffer, BufferIndex, BufferRing};

pub trait DescriptorEntry {
    /// A function that returns [`Some`] if there is a
    /// buffer index associated with this entry that may
    /// be freed.
    fn take_buffer(&mut self) -> Option<BufferIndex>;

    /// Check if this entry has a buffer
    fn has_buffer(&self) -> bool;
}

pub struct EntryRing<'data, T> {
    entries: &'data mut [T],
    buffers: BufferRing<'data>,
}

impl<'data, T> EntryRing<'data, T> {
    pub fn new(descriptors: &'data mut [T], buffers: &'data mut [[u8; MTU + 2]]) -> Self {
        Self {
            entries: descriptors,
            buffers: BufferRing::new(buffers),
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }

    pub(crate) fn buffer_count(&self) -> NonMaxU16 {
        self.buffers.buffer_count()
    }

    pub(crate) fn entry_count(&self) -> u16 {
        self.entries.len() as u16
    }

    /// Get the [`T`] at index `index`
    pub(crate) fn entry(&self, index: usize) -> &T {
        &self.entries[index]
    }

    pub(crate) fn entries_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.entries.iter_mut()
    }

    pub(crate) fn entries(&self) -> impl Iterator<Item = &T> {
        self.entries.iter()
    }

    pub(crate) fn buffers_and_entries<F>(&mut self, mut f: F)
    where
        F: FnMut(&mut T, Option<Buffer>),
    {
        let Self { entries, buffers } = self;

        entries.iter_mut().for_each(|e| {
            let buffer = buffers.next_buffer();
            f(e, buffer)
        });
    }

    pub(crate) fn last_entry_mut(&mut self) -> &mut T {
        &mut self.entries[self.entries.len() - 1]
    }

    pub(crate) fn entries_start_address(&self) -> *const T {
        self.entries.as_ptr()
    }
}

impl<'data, T> EntryRing<'data, T>
where
    T: DescriptorEntry,
{
    /// Get the entry at `index` and a now-reserved buffer.
    ///
    /// Returns `None` if no buffers are available.
    pub fn next_entry_and_next_buffer(&mut self, index: usize) -> Option<(&mut T, Buffer)> {
        let (after, before_inc) = self.entries.split_at_mut(index);
        let mut entries = before_inc.iter_mut().chain(after.iter_mut());

        // Free an entry whose buffer is now available.
        while let Some(entry) = entries.next() {
            if let Some(buffer) = entry.take_buffer() {
                self.buffers.free(buffer);
                break;
            }
        }

        let entry = &mut self.entries[index];
        let buffer = self.buffers.next_buffer()?;

        Some((entry, buffer))
    }

    /// Get the entry at `index`, and the buffer currently associated
    /// with that entry. The buffer must be freed or it can be used again.
    pub fn entry_buffer(&mut self, index: usize) -> Option<(&mut T, Buffer)> {
        let entry = &mut self.entries[index];

        let idx = entry.take_buffer()?;

        let buffer = unsafe { self.buffers.get_buffer(idx) };

        Some((entry, buffer))
    }
}

#[cfg(all(test, not(target_os = "none")))]
use std::collections::VecDeque;
#[cfg(all(test, not(target_os = "none")))]
#[test]
fn buffer_ring() {
    const BUFFERS: usize = 8;

    let mut descriptors = [(); 20];
    let mut buffers = [[0u8; 1524]; BUFFERS];

    let mut buf_ids = VecDeque::new();

    let mut ring = BufferRing::new(&mut buffers);

    assert_eq!(ring.free_buffer_count() as usize, BUFFERS);
    while let Some(buffer) = ring.next_buffer() {
        buf_ids.push_front(buffer);
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }

    assert_eq!(ring.free_buffer_count(), 0);
    assert_eq!(buf_ids.len(), BUFFERS);

    assert_eq!(ring.free_buffer_count() as usize, 0);
    for _ in 0..4 {
        ring.free(buf_ids.pop_back().unwrap().idx());
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }

    assert_eq!(ring.free_buffer_count(), 4);
    assert_eq!(buf_ids.len(), BUFFERS - 4);

    while let Some(buffer) = ring.next_buffer() {
        buf_ids.push_front(buffer);
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }

    assert!(buf_ids.len() == BUFFERS);

    assert_eq!(ring.free_buffer_count() as usize, 0);
    while let Some(buffer) = buf_ids.pop_back() {
        ring.free(buffer.idx());
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }
}
